"""Body pose in the camera image (YOLOv8-Pose on the Hailo-10H).

For gestures a box around the person is not enough -- you need to know where
the shoulders and wrists are. The model ``yolov8s_pose_h10.hef`` delivers
17 keypoints per person (COCO order, see ``KP_*``).

Inference runs on the AI HAT+ 2, not on the CPU: the Pi's CPU stays free,
and CPU load spikes trigger USB resets on the X3 (sensor check
28.10.2026). Decoding the model outputs (boxes, points, NMS), on the other
hand, is little work and runs here in numpy.

The pattern is the same as in ``people.py``: its own thread, computes only
while someone has called ``activate(True)``, and reports via ``status()``
why it is not working right now. If the Hailo fails, it is reopened after a
pause -- the rest of the server only notices that nobody is detected for a
moment.

With ``mock=True`` (``main.py --mock``) a simulated person appears that
shows all gestures one after another. That way the whole chain up to
driving can be tried without a robot.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time

from .. import config
from .camera import CameraService

log = logging.getLogger(__name__)

# COCO keypoints. "left" is the person's left side -- if they face the
# camera, it is on the right in the image.
KP_NOSE = 0
KP_SHOULDER_L, KP_SHOULDER_R = 5, 6
KP_ELBOW_L, KP_ELBOW_R = 7, 8
KP_HAND_L, KP_HAND_R = 9, 10
KP_HIP_L, KP_HIP_R = 11, 12

#: Skeleton lines, for drawing in the app (pairs of point indices).
SKELETON = [
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6),
]

#: Older results no longer count -- the person may be long gone.
_MAX_AGE_S = 1.0

#: Pause before a failed Hailo is reopened.
_REOPEN_S = 2.0

#: If two boxes overlap more than this, it is the same person (NMS).
_NMS_IOU = 0.7


class PoseDetection:
    def __init__(self, camera: CameraService | None, mock: bool = False) -> None:
        self._camera = camera
        self._mock = mock
        self._lock = threading.Lock()
        self._active = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._people: list[dict] = []
        self._time = 0.0
        self._error: str | None = None
        self._frame_buffer = 0
        self._fps = 0.0
        self._reopened = 0
        self._np = None
        self._cv2 = None
        self._available = False

        if not config.POSE_ENABLED:
            self._error = "abgeschaltet (config.POSE_ENABLED)"
            return
        try:
            import numpy as np

            self._np = np
        except ImportError:
            self._error = "numpy fehlt"
            return
        if mock:
            self._available = True
            return
        if camera is None:
            self._error = "keine Kamera"
            return
        try:
            import cv2
            import hailo_platform  # noqa: F401 -- only check that it is there

            self._cv2 = cv2
        except Exception as exc:  # ImportError or broken installation
            self._error = f"nicht verfügbar ({exc.__class__.__name__}: {exc})"
            log.warning("Pose detection unavailable (%s: %s)", exc.__class__.__name__, exc)
            return
        if not os.path.exists(config.POSE_HEF):
            self._error = f"Modell fehlt: {config.POSE_HEF}"
            log.warning("Pose detection: model missing: %s", config.POSE_HEF)
            return
        self._available = True

    # --- Control (like PersonDetection) -------------------------------------------

    @property
    def available(self) -> bool:
        return self._available

    @property
    def simulated(self) -> bool:
        return self._mock

    def start(self) -> None:
        if not self.available or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="pose", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._active.set()  # wake it from waiting
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def activate(self, an: bool) -> None:
        """Only compute while the interaction is running."""
        if an:
            self._active.set()
        else:
            self._active.clear()
            with self._lock:
                self._people = []

    @property
    def active(self) -> bool:
        return self._active.is_set() and self.available

    def people(self) -> list[dict]:
        """Currently seen people, the largest (= closest) first.

        Each person: ``box`` [x1, y1, x2, y2] and ``kp`` (17 × [x, y, confidence])
        relative to the image size (0..1, y pointing down), ``score``, ``winkel``
        (degrees, left positive as with the lidar), ``seitenverhaeltnis`` (width/height
        of the image, so distances in x and y become comparable) and ``zeit``.
        """
        with self._lock:
            if time.monotonic() - self._time > _MAX_AGE_S:
                return []
            return list(self._people)

    def angle_ranges(self) -> list[tuple[float, float]]:
        """Directions (from, to) in degrees -- the same form as in ``PersonDetection``."""
        fov = self.fov_deg
        return [((0.5 - p["box"][2]) * fov, (0.5 - p["box"][0]) * fov) for p in self.people()]

    @property
    def fov_deg(self) -> float:
        """Field of view of the camera being looked through -- the C270 sees
        narrower than the general default (config.CAMERA_FOV_DEG)."""
        return self._camera.fov_deg if self._camera is not None else config.CAMERA_FOV_DEG

    def status(self) -> dict:
        with self._lock:
            return {
                "verfuegbar": self.available,
                "simuliert": self._mock,
                "aktiv": self.active,
                "fehler": self._error,
                "fps": round(self._fps, 1),
                "bilder": self._frame_buffer,
                "neu_geoeffnet": self._reopened,
                # Which camera image the points belong to -- the app
                # overlays the skeleton only there.
                "kamera": self._camera.name if self._camera is not None else None,
                "letzte_erkennung_vor_s": round(time.monotonic() - self._time, 1) if self._time else None,
            }

    # --- Compute loop ---------------------------------------------------------------

    def _loop(self) -> None:
        hailo: _Hailo | None = None
        if not self._mock and config.POSE_KEEP_WARM:
            # Open right at server start, while the robot is standing still.
            # Opening (device + loading the model) is a load spike; when it
            # only came at the start of gesture control, it often reset the
            # USB hub on the X3 -- "Keine Verbindung zum Antrieb" (29.10.2026).
            hailo = self._open_device()
        try:
            while not self._stop.is_set():
                if not self._active.wait(timeout=1.0):
                    # Not needed. Keeping warm is the normal case (see
                    # above); otherwise release it so other programs
                    # (hailo-apps) have the chip to themselves.
                    if hailo is not None and not config.POSE_KEEP_WARM:
                        hailo.close()
                        hailo = None
                    continue
                if self._stop.is_set():
                    # stop() wakes us via _active. Without this check the
                    # thread would still open the Hailo while shutting down --
                    # on 29.10. exactly that briefly cut the USB power.
                    break
                begin = time.monotonic()
                try:
                    if self._mock:
                        people = _simulated_people(begin)
                    else:
                        if hailo is None:
                            hailo = _Hailo(self._np)
                            log.info("Pose detection: Hailo ready (%s)", config.POSE_HEF)
                            if self._stop.is_set():
                                break  # opened during shutdown -- close it right away
                        people = self._detect(hailo)
                    if people is not None:
                        self._remember(people, begin)
                except Exception as exc:
                    with self._lock:
                        self._error = f"{exc.__class__.__name__}: {exc}"
                    if hailo is not None or not self._mock:
                        # Hailo gone (USB/PCIe dropout, another program
                        # holds it) -- close it and reopen after a pause.
                        log.warning("Pose detection: %s -- reopening the Hailo", self._error)
                        if hailo is not None:
                            hailo.close()
                            hailo = None
                        self._reopened += 1
                        self._stop.wait(_REOPEN_S)
                        continue
                # Re-read every time: adjustable on the settings page.
                tick = 1.0 / max(0.5, config.POSE_HZ)
                rest = tick - (time.monotonic() - begin)
                if rest > 0:
                    self._stop.wait(rest)
        finally:
            if hailo is not None:
                hailo.close()

    def _open_device(self) -> "_Hailo | None":
        """Open the Hailo in advance. If that fails, the loop tries again later."""
        if self._stop.is_set():
            return None
        try:
            hailo = _Hailo(self._np)
        except Exception as exc:
            with self._lock:
                self._error = f"{exc.__class__.__name__}: {exc}"
            log.warning("Pose detection: cannot open the Hailo (%s) -- retrying later", self._error)
            return None
        log.info("Pose detection: Hailo opened in advance (%s)", config.POSE_HEF)
        return hailo

    def _remember(self, people: list[dict], begin: float) -> None:
        now = time.monotonic()
        with self._lock:
            if self._time:
                # Smoothed frame rate for the status
                instant = 1.0 / max(1e-3, now - self._time)
                self._fps = 0.8 * self._fps + 0.2 * instant if self._fps else instant
            self._people = people
            self._time = now
            self._frame_buffer += 1
            self._error = None

    def _detect(self, hailo: "_Hailo") -> list[dict] | None:
        if self._camera is None:
            return None
        # With capture time where the camera knows it: a person's "zeit" should
        # say when the image was taken, not when the Hailo finished --
        # otherwise following works with an angle up to ~100 ms too late.
        if hasattr(self._camera, "latest_with_time"):
            frame, captured_at = self._camera.latest_with_time()
        else:
            frame, captured_at = self._camera.latest(), 0.0
        if frame is None:
            return None
        cv2, np = self._cv2, self._np
        image = cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None
        h, w = image.shape[:2]
        size = hailo.size

        # Letterbox as in training: keep the aspect ratio, grey border.
        # Stretching instead would make arms shorter or longer.
        scale = min(size / w, size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        dx, dy = (size - nw) // 2, (size - nh) // 2
        model_input = np.full((size, size, 3), 114, dtype=np.uint8)
        small = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_CUBIC)
        model_input[dy:dy + nh, dx:dx + nw] = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)

        hits = decode_poses(np, hailo.infer(model_input), size, config.POSE_MIN_SCORE)

        people = []
        for score, box, kp in hits:
            # Back into the original image, then to 0..1
            bx = ((box[[0, 2]] - dx) / scale / w).clip(0, 1)
            by = ((box[[1, 3]] - dy) / scale / h).clip(0, 1)
            kx = ((kp[:, 0] - dx) / scale / w).clip(0, 1)
            ky = ((kp[:, 1] - dy) / scale / h).clip(0, 1)
            people.append(_person(
                [bx[0], by[0], bx[1], by[1]],
                [[float(x), float(y), float(c)] for x, y, c in zip(kx, ky, kp[:, 2])],
                float(score), w / h, self.fov_deg, captured_at or None,
            ))
            # Appearance for the target lock (interaction/target.py). Here,
            # because only the pose thread has the decoded image.
            feature = torso_feature(cv2, np, image, people[-1]["kp"])
            if feature is not None:
                people[-1]["merkmal"] = feature
        people.sort(key=lambda p: -(p["box"][3] - p["box"][1]))
        return people


def _person(
    box, kp, score: float, aspect_ratio: float, fov: float | None = None,
    timestamp: float | None = None,
) -> dict:
    center = (box[0] + box[2]) / 2
    return {
        "box": [round(float(v), 3) for v in box],
        "kp": [[round(x, 3), round(y, 3), round(c, 2)] for x, y, c in kp],
        "score": round(score, 2),
        "winkel": round((0.5 - center) * (fov if fov is not None else config.CAMERA_FOV_DEG), 1),
        "seitenverhaeltnis": round(aspect_ratio, 3),
        "zeit": timestamp if timestamp is not None else time.monotonic(),
    }


def torso_feature(cv2, np, image, kp) -> list[float] | None:
    """Color histogram (H-S, 16x8) of the upper body, normalized to sum 1.

    The torso between shoulders and hips is visible from the front as well
    as from behind and usually carries the most distinctive color (the top).
    Faces are deliberately not used. None if shoulders/hips are not detected
    reliably -- a histogram of the background would be worse than none.
    """
    points = [kp[i] for i in (KP_SHOULDER_L, KP_SHOULDER_R, KP_HIP_L, KP_HIP_R)]
    if any(c < 0.3 for _, _, c in points):
        return None
    h, w = image.shape[:2]
    xs = [x * w for x, _, _ in points]
    ys = [y * h for _, y, _ in points]
    # Shrink inward a bit so arms and background at the edge do not count.
    margin_x = (max(xs) - min(xs)) * 0.15
    x1, x2 = int(min(xs) + margin_x), int(max(xs) - margin_x)
    y1, y2 = int(min(ys)), int(max(ys))
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    hsv = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    total = float(hist.sum())
    if total <= 0:
        return None
    return [round(float(v) / total, 4) for v in hist]


# --- Hailo -----------------------------------------------------------------------


class _Hailo:
    """An opened model on the Hailo. Use from one thread only.

    The outputs are requested as FLOAT32: then HailoRT undoes the
    quantization itself, and decoding does not need to know scales and
    zero points per output.
    """

    def __init__(self, np) -> None:
        from hailo_platform import HEF, FormatType, HailoSchedulingAlgorithm, VDevice

        self._np = np
        params = VDevice.create_params()
        # "SHARED" + round robin: other programs (e.g. hailo-apps) may use
        # the chip at the same time instead of one blocking it.
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "SHARED"
        self._vdev = VDevice(params)
        self._ctx = None
        try:
            model = self._vdev.create_infer_model(config.POSE_HEF)
            model.set_batch_size(1)
            names = [i.name for i in HEF(config.POSE_HEF).get_output_vstream_infos()]
            for name in names:
                model.output(name).set_format_type(FormatType.FLOAT32)
            self.size = int(model.input().shape[0])
            self._ctx = model.configure()
            self._conf = self._ctx.__enter__()
            self._names = names
            self._bindings = self._conf.create_bindings(output_buffers={
                name: np.empty(model.output(name).shape, dtype=np.float32) for name in names
            })
        except Exception:
            self.close()
            raise

    def infer(self, model_input) -> list:
        self._bindings.input().set_buffer(self._np.ascontiguousarray(model_input))
        self._conf.run([self._bindings], 2000)
        return [self._bindings.output(name).get_buffer() for name in self._names]

    def close(self) -> None:
        try:
            if self._ctx is not None:
                self._ctx.__exit__(None, None, None)
                self._ctx = None
        except Exception:
            log.debug("Closing the Hailo model failed", exc_info=True)
        try:
            self._vdev.release()
        except Exception:
            log.debug("Releasing the Hailo failed", exc_info=True)


def decode_poses(np, outputs: list, size: int, min_score: float) -> list:
    """Raw YOLOv8-Pose outputs -> [(score, box[4], kp[17, 3])] in input pixels.

    The model has three outputs per grid (80, 40, 20 cells at 640 px),
    recognized by their channel count: 64 (box as a distribution over 16
    distances per side, "DFL"), 1 (person probability) and 51 (17 points × x,
    y, confidence). The order of the outputs in the HEF is not guaranteed,
    so they are sorted by shape instead of by name.
    """
    per_grid: dict[int, dict[int, object]] = {}
    for a in outputs:
        a = np.asarray(a, dtype=np.float32)
        channels = a.shape[-1]
        a = a.reshape(-1, channels)
        grid = int(round(math.sqrt(a.shape[0])))
        per_grid.setdefault(grid, {})[channels] = a

    scores, boxes, points = [], [], []
    for grid, parts in per_grid.items():
        if not {64, 1, 51} <= parts.keys():
            continue
        step = size / grid
        s = parts[1][:, 0]
        if s.min() < 0.0 or s.max() > 1.0:  # logits instead of probabilities
            s = 1.0 / (1.0 + np.exp(-s))
        selection = s >= min_score
        if not selection.any():
            continue
        gy, gx = np.meshgrid(np.arange(grid), np.arange(grid), indexing="ij")
        center = np.column_stack(((gx.ravel() + 0.5) * step, (gy.ravel() + 0.5) * step))[selection]

        # Box: per side a distribution over 16 distances -> expected value
        distribution = parts[64][selection].reshape(-1, 4, 16)
        distribution = np.exp(distribution - distribution.max(axis=2, keepdims=True))
        distribution /= distribution.sum(axis=2, keepdims=True)
        dist = (distribution @ np.arange(16, dtype=np.float32)) * step  # left, top, right, bottom
        box = np.column_stack((center - dist[:, :2], center + dist[:, 2:]))

        kp = parts[51][selection].reshape(-1, 17, 3).copy()
        kp[:, :, :2] = kp[:, :, :2] * 2.0 * step + (center[:, None, :] - 0.5 * step)
        kp[:, :, 2] = 1.0 / (1.0 + np.exp(-kp[:, :, 2]))

        scores.append(s[selection])
        boxes.append(box)
        points.append(kp)

    if not scores:
        return []
    scores = np.concatenate(scores)
    boxes = np.concatenate(boxes)
    points = np.concatenate(points)
    return [(float(scores[i]), boxes[i], points[i]) for i in _nms(np, boxes, scores, _NMS_IOU)]


def _nms(np, boxes, scores, iou_limit: float) -> list[int]:
    """Remove duplicate hits of the same person (the best one stays)."""
    order = list(np.argsort(-scores))
    area = (boxes[:, 2] - boxes[:, 0]).clip(0) * (boxes[:, 3] - boxes[:, 1]).clip(0)
    keep = []
    while order:
        i = order.pop(0)
        keep.append(int(i))
        if not order:
            break
        rest = np.array(order)
        x1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        y1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        x2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        y2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        intersection = (x2 - x1).clip(0) * (y2 - y1).clip(0)
        iou = intersection / np.maximum(1e-6, area[i] + area[rest] - intersection)
        order = [int(j) for j, u in zip(rest, iou) if u < iou_limit]
    return keep


# --- Simulation (--mock) ------------------------------------------------------------

#: Sequence of the simulated person: (duration in s, gesture). One cycle takes
#: 40 s, each gesture occurs once, in between the person stands still.
_SIM_SEQUENCE = [
    (6.0, None), (3.0, "folgen"), (5.0, None), (2.0, "stopp"), (5.0, None),
    (3.0, "herkommen"), (5.0, None), (3.0, "weggehen"), (8.0, None),
]


#: Second, smaller (farther away) person: walks across the image once in
#: this part of the cycle, crossing the first one. That way the target lock
#: can be observed without a robot too.
_SIM_SECOND = (20.0, 34.0)


def _sim_feature(hue: int, saturation: int) -> list[float]:
    """Artificial torso histogram (16x8): one color, slightly smeared."""
    hist = [0.0] * 128
    for dh, ds, share in ((0, 0, 0.6), (1, 0, 0.15), (-1, 0, 0.15), (0, -1, 0.1)):
        h = (hue + dh) % 16
        s_ = max(0, min(7, saturation + ds))
        hist[h * 8 + s_] += share
    return hist


def _simulated_people(t: float) -> list[dict]:
    """One person who slowly walks back and forth and shows gestures.

    At times a second, smaller one walks past (``_SIM_SECOND``). The
    first stays the largest and is therefore always first in the list.
    """
    people = [_sim_main_person(t)]
    rest = t % sum(d for d, _ in _SIM_SEQUENCE)
    from_, until = _SIM_SECOND
    if from_ <= rest < until:
        share = (rest - from_) / (until - from_)
        mx = 0.9 - 0.8 * share
        kp, box = _sim_body(mx, 0.3, 0.45, 0.04)
        second = _person(box, [[x, yy, 0.9] for x, yy in kp], 0.85,
                         config.CAMERA_WIDTH / config.CAMERA_HEIGHT)
        second["merkmal"] = _sim_feature(10, 5)  # blue top
        people.append(second)
    return people


def _sim_body(mx: float, top: float, height: float, e: float):
    """Keypoints (without gesture) and box of a standing person."""

    def y(share: float) -> float:
        return top + share * height

    kp = [[mx, y(0.07)], [mx + e * 0.3, y(0.05)], [mx - e * 0.3, y(0.05)],
          [mx + e * 0.6, y(0.06)], [mx - e * 0.6, y(0.06)],
          [mx + e, y(0.2)], [mx - e, y(0.2)],             # shoulders L, R
          [mx + e * 1.2, y(0.35)], [mx - e * 1.2, y(0.35)],  # elbows
          [mx + e * 1.1, y(0.5)], [mx - e * 1.1, y(0.5)],    # wrists
          [mx + e * 0.7, y(0.52)], [mx - e * 0.7, y(0.52)],  # hips
          [mx + e * 0.7, y(0.74)], [mx - e * 0.7, y(0.74)],
          [mx + e * 0.7, y(0.96)], [mx - e * 0.7, y(0.96)]]
    box = [mx - e * 2.5, top, mx + e * 2.5, top + height]
    return kp, box


def _sim_main_person(t: float) -> dict:
    """The first person: walks slowly back and forth and shows the gestures."""
    duration = sum(d for d, _ in _SIM_SEQUENCE)
    rest = t % duration
    gesture, since = None, 0.0
    for d, g in _SIM_SEQUENCE:
        if rest < d:
            gesture, since = g, rest
            break
        rest -= d

    # Image coordinates 0..1; the test image is 4:3. "e" is half a
    # shoulder width in x, converted accordingly for y.
    aspect_ratio = config.CAMERA_WIDTH / config.CAMERA_HEIGHT
    mx = 0.5 + 0.15 * math.sin(t / 6.0)
    top, height = 0.12, 0.8
    e = 0.06
    ey = e * aspect_ratio

    def y(share: float) -> float:
        return top + share * height

    kp, box = _sim_body(mx, top, height, e)
    shoulder_y = y(0.2)
    if gesture == "folgen":  # right arm stretched above the head
        kp[KP_ELBOW_R] = [mx - e * 1.2, shoulder_y - ey * 1.2]
        kp[KP_HAND_R] = [mx - e * 1.1, y(0.07) - ey * 1.5]
    elif gesture == "stopp":  # both arms up
        kp[KP_ELBOW_L] = [mx + e * 1.4, shoulder_y - ey * 1.0]
        kp[KP_ELBOW_R] = [mx - e * 1.4, shoulder_y - ey * 1.0]
        kp[KP_HAND_L] = [mx + e * 1.4, shoulder_y - ey * 2.2]
        kp[KP_HAND_R] = [mx - e * 1.4, shoulder_y - ey * 2.2]
    elif gesture == "herkommen":  # hand waves up and down in front of the chest
        kp[KP_ELBOW_R] = [mx - e * 1.6, y(0.33)]
        hub = math.sin(since * 2 * math.pi * 1.5) * ey * 0.7
        kp[KP_HAND_R] = [mx - e * 1.3, shoulder_y + ey * 0.3 + hub]
    elif gesture == "weggehen":  # hand keeps sweeping outward
        kp[KP_ELBOW_R] = [mx - e * 1.5, y(0.33)]
        share = (since * 1.2) % 1.0  # sawtooth: out, quickly back
        kp[KP_HAND_R] = [mx - e * (0.5 + 3.0 * share), shoulder_y + ey * 0.8]

    person = _person(box, [[x, yy, 0.9] for x, yy in kp], 0.9, aspect_ratio)
    person["merkmal"] = _sim_feature(1, 6)  # red top
    return person
