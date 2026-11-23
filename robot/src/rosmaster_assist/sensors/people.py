"""Find people in the camera image: Hailo pose boxes, OpenCV HOG as fallback.

The lidar only sees legs -- it cannot reliably tell whether two posts are a
person or a chair. The camera can.

Two ways to find them (``PERSON_DETECT_BACKEND``):

- **Hailo** (default when it works): the person boxes of the pose detection
  (``pose.py``, YOLOv8-Pose on the Hailo AI chip). The network runs on the
  chip; the CPU only decodes the image. When gestures or the healthbot use
  the pose detection anyway, the boxes cost nothing extra. YOLO also finds
  people sitting, lying or seen from the side, which HOG misses.
- **HOG** (fallback): OpenCV's HOG detector needs no model and manages a few
  frames per second on the Pi at 320 px width, but costs ~0.1-0.2 s CPU per
  frame. It runs when there is no Hailo, the Hailo reports an error, or a
  user asks for it explicitly (test runs compare both).

Detection only runs while someone needs it (``activate``), because it costs
CPU -- and CPU load spikes trigger USB resets on the X3 (sensor check
28.10.2026).

If OpenCV is missing, ``status()`` reports it and everything else keeps
running: then only the lidar names the obstacles.
"""

from __future__ import annotations

import logging
import threading
import time

from .. import config
from .camera import CameraService

log = logging.getLogger(__name__)

#: Older hits no longer count -- the person may be long gone.
_MAX_AGE_S = 1.5

#: user name under which this module switches on the pose detection
_POSE_USER = "personen"


class PersonDetection:
    def __init__(self, camera: CameraService | None, pose=None) -> None:
        self._camera = camera
        self._pose = pose
        self._lock = threading.Lock()
        self._active = threading.Event()
        self._users: set[str] = set()
        self._hog_users: set[str] = set()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._boxes: list[dict] = []
        self._time = 0.0
        self._error: str | None = None
        self._hog = None
        self._cv2 = None
        self._np = None
        # The detector is loaded even when detection is currently switched
        # off: it can be switched on at runtime on the settings page
        # (PERSON_DETECTION_ENABLED).
        if camera is None:
            self._error = "keine Kamera"
        else:
            try:
                import cv2
                import numpy as np

                hog = cv2.HOGDescriptor()
                hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
                self._cv2, self._np, self._hog = cv2, np, hog
            except Exception as exc:  # ImportError or broken OpenCV version
                self._error = f"nicht verfügbar (opencv fehlt: {exc.__class__.__name__})"
                log.warning("Person detection unavailable (opencv missing: %s)", exc.__class__.__name__)

    @property
    def available(self) -> bool:
        return (self._hog is not None or self._use_hailo()) and config.PERSON_DETECTION_ENABLED

    def _use_hailo(self) -> bool:
        """Take the boxes from the Hailo pose detection right now?"""
        pose = self._pose
        if pose is None or config.PERSON_DETECT_BACKEND == "hog":
            return False
        # the simulated pose shows a gesture person all the time, the mock explorer would stop for it
        return pose.available and not pose.simulated and pose.healthy

    @property
    def backend(self) -> str | None:
        """"hailo", "hog" or None (nothing available)."""
        if self._use_hailo():
            return "hailo"
        return "hog" if self._hog is not None else None

    def start(self) -> None:
        if self._hog is None or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="personen", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._active.set()  # wake it from waiting
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._pose is not None:
            self._pose.activate(False, _POSE_USER)

    def activate(self, an: bool, who: str = "explorer", hog: bool = False) -> None:
        """Only compute while someone needs the hits.

        ``who`` tells the users apart (explorer, test run). Detection runs as
        long as at least one of them has switched it on; otherwise the end
        of a test run would switch it off for a running explorer. ``hog``:
        this user needs the HOG boxes (``hog_boxes``) even when the Hailo
        delivers the normal ones.
        """
        with self._lock:
            if an:
                self._users.add(who)
                if hog:
                    self._hog_users.add(who)
            else:
                self._users.discard(who)
                self._hog_users.discard(who)
            if self._users:
                self._active.set()
            else:
                self._active.clear()
                self._boxes = []
            wanted = bool(self._users)
        if self._pose is not None and config.PERSON_DETECT_BACKEND != "hog":
            # the pose thread decides itself whether it runs; it also follows a hailo that comes back
            self._pose.activate(wanted, _POSE_USER)

    def boxes(self) -> list[dict]:
        """Current hits: x1,y1,x2,y2 relative to the image size (0..1), score."""
        if not config.PERSON_DETECTION_ENABLED or not self._active.is_set():
            return []
        if self._use_hailo():
            return [
                {"x1": p["box"][0], "y1": p["box"][1], "x2": p["box"][2], "y2": p["box"][3],
                 "score": p["score"]}
                for p in self._pose.people()
            ]
        return self.hog_boxes()

    def hog_boxes(self) -> list[dict]:
        """Hits of the HOG detector only (empty while the Hailo does the work
        and no user asked for HOG)."""
        with self._lock:
            if time.monotonic() - self._time > _MAX_AGE_S:
                return []
            return list(self._boxes)

    def angle_ranges(self) -> list[tuple[float, float]]:
        """Directions (from, to) in degrees in which a person is standing.

        Image center = 0°, left in the image = positive angles (as with the lidar).
        Assumes that the camera faces forward.
        """
        # The field of view belongs to the camera, not to the detection: the
        # C270 sees considerably narrower than the Orbbec.
        fov = self._camera.fov_deg if self._camera is not None else config.CAMERA_FOV_DEG
        return [((0.5 - b["x2"]) * fov, (0.5 - b["x1"]) * fov) for b in self.boxes()]

    def status(self) -> dict:
        backend = self.backend
        last = self._time
        if backend == "hailo":
            pose_age = self._pose.status().get("letzte_erkennung_vor_s")
            last = time.monotonic() - pose_age if pose_age is not None else 0.0
        with self._lock:
            return {
                "verfuegbar": self.available,
                "aktiv": self._active.is_set() and self.available,
                "fehler": (
                    "abgeschaltet (Einstellungen)"
                    if (self._hog is not None or backend == "hailo") and not config.PERSON_DETECTION_ENABLED
                    else (None if backend == "hailo" else self._error)
                ),
                # "hailo" (pose boxes from the ai chip) or "hog" (opencv on the cpu)
                "verfahren": backend,
                # Which image the boxes belong to -- the app draws them only
                # over this camera.
                "kamera": self._camera.name if self._camera is not None else None,
                "letzte_erkennung_vor_s": round(time.monotonic() - last, 1) if last else None,
            }

    def _hog_needed(self) -> bool:
        with self._lock:
            if self._hog_users:
                return True
        return not self._use_hailo()

    def _loop(self) -> None:
        while not self._stop.is_set():
            if not self._active.wait(timeout=1.0):
                continue
            if not config.PERSON_DETECTION_ENABLED or not self._hog_needed():
                # switched off, or the hailo does the work: hog stays idle
                with self._lock:
                    self._boxes = []
                self._stop.wait(1.0 / max(0.1, config.PERSON_DETECT_HZ))
                continue
            # Re-read every time: adjustable at runtime.
            tick = 1.0 / max(0.1, config.PERSON_DETECT_HZ)
            begin = time.monotonic()
            try:
                self._detect()
            except Exception as exc:
                # A broken frame must not end the detection.
                with self._lock:
                    self._error = f"{exc.__class__.__name__}: {exc}"
                log.debug("Person detection: frame skipped", exc_info=True)
            rest = tick - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)

    def _detect(self) -> None:
        frame = self._camera.latest() if self._camera is not None else None
        if frame is None:
            return
        cv2, np = self._cv2, self._np
        image = cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        h, w = image.shape[:2]
        width = config.PERSON_DETECT_WIDTH
        if w > width:
            image = cv2.resize(image, (width, int(h * width / w)))
            h, w = image.shape[:2]
        rects, weights = self._hog.detectMultiScale(
            image, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        boxes = []
        for (x, y, bw, bh), score in zip(rects, np.ravel(weights) if len(weights) else []):
            if float(score) < config.PERSON_MIN_SCORE:
                continue
            boxes.append({
                "x1": round(x / w, 3), "y1": round(y / h, 3),
                "x2": round((x + bw) / w, 3), "y2": round((y + bh) / h, 3),
                "score": round(float(score), 2),
            })
        with self._lock:
            self._boxes = boxes
            self._time = time.monotonic()
            self._error = None
