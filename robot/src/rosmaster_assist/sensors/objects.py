"""Objects in the camera image: YOLO on the Hailo, 80 COCO classes.

The lidar sees one plane ~20 cm above the floor and can only guess from the
shape whether two posts are a chair or a person (``navigation/obstacles.py``).
The camera sees what it is. This module runs a detection network
(``OBJECTS_HEF``, YOLOv8m with the NMS on the chip) on the forward camera and
delivers boxes with a class name.

Inference runs on the Hailo next to the pose detection: both open the chip
in the shared group with round-robin scheduling (see ``pose._Hailo``). Not
via hailo-apps: those are GStreamer pipelines that open the camera
themselves, and the camera already belongs to this server (stream, people,
map photos).

Like ``pose.py`` it only computes while someone needs it (``activate``):
the object view in the app, or the obstacle naming while the person
detection runs (driving). If the Hailo fails it is reopened after a pause.

Uses:

- ``objects()``: boxes with class, German name, score, direction
- ``angle_labels()``: directions with a name, for the obstacle naming
- ``annotated_jpeg()``: the camera image with boxes and names drawn in, for
  ``/objekte/stream``
"""

from __future__ import annotations

import logging
import os
import threading
import time

from .. import config

log = logging.getLogger(__name__)

#: COCO classes in model order with the German name the app shows
COCO_DE = [
    "Mensch", "Fahrrad", "Auto", "Motorrad", "Flugzeug", "Bus", "Zug", "Lkw", "Boot",
    "Ampel", "Hydrant", "Stoppschild", "Parkuhr", "Bank", "Vogel", "Katze", "Hund",
    "Pferd", "Schaf", "Kuh", "Elefant", "Bär", "Zebra", "Giraffe", "Rucksack",
    "Regenschirm", "Handtasche", "Krawatte", "Koffer", "Frisbee", "Ski", "Snowboard",
    "Ball", "Drachen", "Baseballschläger", "Baseballhandschuh", "Skateboard",
    "Surfbrett", "Tennisschläger", "Flasche", "Weinglas", "Tasse", "Gabel", "Messer",
    "Löffel", "Schüssel", "Banane", "Apfel", "Sandwich", "Orange", "Brokkoli",
    "Karotte", "Hotdog", "Pizza", "Donut", "Kuchen", "Stuhl", "Sofa", "Pflanze",
    "Bett", "Tisch", "Toilette", "Fernseher", "Laptop", "Maus", "Fernbedienung",
    "Tastatur", "Handy", "Mikrowelle", "Ofen", "Toaster", "Spüle", "Kühlschrank",
    "Buch", "Uhr", "Vase", "Schere", "Teddy", "Föhn", "Zahnbürste",
]

#: classes that stand on the floor and may name a lidar obstacle in their
#: direction; small things on tables (cup, laptop) would mislabel the legs
FLOOR_CLASSES = {
    "Mensch", "Fahrrad", "Bank", "Katze", "Hund", "Rucksack", "Koffer", "Stuhl",
    "Sofa", "Pflanze", "Bett", "Tisch", "Toilette", "Kühlschrank", "Ball",
}


def decode_nms(np, output, min_score: float) -> list[tuple[int, float, list[float]]]:
    """Hailo NMS-by-class output -> [(class, score, [x1, y1, x2, y2] 0..1 of the input)].

    float32 layout per class: number of boxes, then that many boxes of
    (y_min, x_min, y_max, x_max, score), relative to the model input. some
    hailort versions hand out a list of per-class arrays instead
    """
    hits = []
    if isinstance(output, (list, tuple)):
        for cls, boxes in enumerate(output):
            for b in np.asarray(boxes, dtype=np.float32).reshape(-1, 5):
                if b[4] >= min_score:
                    hits.append((cls, float(b[4]), [float(b[1]), float(b[0]), float(b[3]), float(b[2])]))
        return hits
    flat = np.asarray(output, dtype=np.float32).ravel()
    off = 0
    for cls in range(len(COCO_DE)):
        if off >= len(flat):
            break
        n = int(flat[off])
        off += 1
        if n < 0 or off + 5 * n > len(flat):
            break
        for b in flat[off:off + 5 * n].reshape(-1, 5):
            if b[4] >= min_score:
                hits.append((cls, float(b[4]), [float(b[1]), float(b[0]), float(b[3]), float(b[2])]))
        off += 5 * n
    return hits


class ObjectDetection:
    def __init__(self, camera, follow=None) -> None:
        """``follow``: the person detection; while it runs (driving) the
        obstacle naming wants objects too"""
        self._camera = camera
        self._follow = follow
        self._lock = threading.Lock()
        self._users: set[str] = set()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._objects: list[dict] = []
        self._time = 0.0
        self._image = None          # last decoded camera image (bgr) for the view
        self._image_time = 0.0
        # setup problem (no hailo, no model): permanent; runtime errors of
        # the hailo go to _error and are retried
        self._setup_error: str | None = None
        self._error: str | None = None
        self._ms: float | None = None
        self._last_capture = 0.0
        self._cv2 = None
        self._np = None
        if not config.OBJECTS_ENABLED:
            self._setup_error = "abgeschaltet (OBJECTS_ENABLED)"
            return
        if camera is None:
            self._setup_error = "keine Kamera"
            return
        try:
            import cv2
            import numpy as np
            self._cv2, self._np = cv2, np
        except ImportError:
            self._setup_error = "opencv oder numpy fehlt"
            return
        try:
            import hailo_platform  # noqa: F401 -- only check that it is there
        except ImportError:
            self._setup_error = "kein Hailo (hailo_platform fehlt)"
            return
        if not os.path.exists(config.OBJECTS_HEF):
            self._setup_error = f"Modell fehlt: {config.OBJECTS_HEF}"
            log.warning("Object detection: model missing: %s", config.OBJECTS_HEF)

    @property
    def available(self) -> bool:
        return self._cv2 is not None and self._setup_error is None

    @property
    def camera_name(self) -> str | None:
        return self._camera.name if self._camera is not None else None

    @property
    def fov_deg(self) -> float:
        return self._camera.fov_deg if self._camera is not None else config.CAMERA_FOV_DEG

    def start(self) -> None:
        if not self.available or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="objekte", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def activate(self, on: bool, who: str) -> None:
        """Compute only while at least one user needs it (view, test, ...)."""
        with self._lock:
            if on:
                self._users.add(who)
            else:
                self._users.discard(who)
        self._wake.set()

    def _needed(self) -> bool:
        with self._lock:
            if self._users:
                return True
        follow = self._follow
        return bool(config.OBJECTS_WITH_PEOPLE and follow is not None
                    and getattr(follow, "active", False))

    # --- results ---------------------------------------------------------------

    def objects(self) -> list[dict]:
        """Current objects, biggest first: ``klasse`` (German), ``score``,
        ``box`` [x1, y1, x2, y2] 0..1, ``winkel`` [from, to] degrees (left +)."""
        with self._lock:
            if time.monotonic() - self._time > config.OBJECTS_MAX_AGE_S:
                return []
            return list(self._objects)

    def angle_labels(self) -> list[tuple[float, float, str]]:
        """Directions (from, to, name) of things standing on the floor."""
        return [(o["winkel"][0], o["winkel"][1], o["klasse"]) for o in self.objects()
                if o["klasse"] in FLOOR_CLASSES]

    def status(self) -> dict:
        # outside the lock: _needed() takes it itself
        active = self._needed() and self.available
        with self._lock:
            age = time.monotonic() - self._time if self._time else None
            return {
                "verfuegbar": self.available,
                "aktiv": active,
                "fehler": self._setup_error or self._error,
                "kamera": self.camera_name,
                "modell": os.path.basename(config.OBJECTS_HEF),
                "rechenzeit_ms": round(self._ms, 1) if self._ms is not None else None,
                "letzte_erkennung_vor_s": round(age, 1) if age is not None else None,
                "anzahl": len(self._objects) if age is not None and age <= config.OBJECTS_MAX_AGE_S else 0,
            }

    def annotated_jpeg(self, quality: int = 80) -> tuple[bytes, float] | None:
        """The last computed image with boxes and names, plus its time; None
        before the first one. drawn on demand, costs nothing without a viewer"""
        cv2 = self._cv2
        with self._lock:
            image, taken, objects = self._image, self._image_time, list(self._objects)
            error = self._setup_error or self._error
        if image is None:
            if self._camera is None or cv2 is None:
                return None
            # no detection yet (hailo missing or still opening): plain image with a note
            frame = self._camera.latest()
            if frame is None:
                return None
            image = cv2.imdecode(self._np.frombuffer(frame[0], dtype=self._np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return None
            taken = time.monotonic()
            objects = []
            note = error or "Objekterkennung startet …"
        else:
            image = image.copy()
            note = None
        h, w = image.shape[:2]
        thick = max(2, w // 320)
        scale = max(0.5, w / 1000)
        for o in objects:
            x1, y1, x2, y2 = (int(round(v * s)) for v, s in zip(o["box"], (w, h, w, h)))
            color = _color(o["klasse"])
            cv2.rectangle(image, (x1, y1), (x2, y2), color, thick)
            text = f"{o['klasse']} {o['score'] * 100:.0f}%"
            (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            ty = y1 - 4 if y1 - th - 8 > 0 else y1 + th + 6
            cv2.rectangle(image, (x1, ty - th - 4), (x1 + tw + 6, ty + base - 2), color, -1)
            cv2.putText(image, _ascii(text), (x1 + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, scale,
                        (0, 0, 0), 1, cv2.LINE_AA)
        if note:
            cv2.putText(image, _ascii(note), (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
                        cv2.LINE_AA)
        ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return (data.tobytes(), taken) if ok else None

    # --- thread ----------------------------------------------------------------

    def _loop(self) -> None:
        from .pose import HailoModel

        hailo = None
        while not self._stop.is_set():
            if not self._needed():
                # nobody looks: free the chip for the pose detection
                if hailo is not None and not config.OBJECTS_KEEP_WARM:
                    hailo.close()
                    hailo = None
                with self._lock:
                    # a view opened later must not show an old frame
                    self._objects = []
                    self._image = None
                self._wake.wait(config.OBJECTS_IDLE_POLL_S)
                self._wake.clear()
                continue
            begin = time.monotonic()
            try:
                if hailo is None:
                    hailo = HailoModel(self._np, config.OBJECTS_HEF)
                    log.info("Object detection: Hailo ready (%s)", config.OBJECTS_HEF)
                    with self._lock:
                        self._error = None
                self._detect(hailo)
            except Exception as e:
                log.warning("Object detection failed: %s -- reopening in %.0f s", e, config.OBJECTS_REOPEN_S)
                with self._lock:
                    self._error = f"Hailo: {e}"
                    self._image = None
                if hailo is not None:
                    hailo.close()
                    hailo = None
                self._stop.wait(config.OBJECTS_REOPEN_S)
                continue
            rest = 1.0 / max(0.5, config.OBJECTS_HZ) - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)
        if hailo is not None:
            hailo.close()

    def _detect(self, hailo) -> None:
        frame, captured = self._camera.latest_with_time()
        if frame is None or not captured or captured == self._last_capture:
            return
        self._last_capture = captured
        if time.monotonic() - captured > config.OBJECTS_MAX_FRAME_AGE_S:
            return
        cv2, np = self._cv2, self._np
        image = cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        started = time.perf_counter()
        h, w = image.shape[:2]
        size = hailo.size
        # letterbox as in training: keep the aspect ratio, grey border
        scale = min(size / w, size / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        dx, dy = (size - nw) // 2, (size - nh) // 2
        model_input = np.full((size, size, 3), 114, dtype=np.uint8)
        small = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        model_input[dy:dy + nh, dx:dx + nw] = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        outputs = hailo.infer(model_input)
        fov = self.fov_deg
        found = []
        for cls, score, box in decode_nms(np, outputs[0], config.OBJECTS_MIN_SCORE):
            name = COCO_DE[cls] if cls < len(COCO_DE) else f"Klasse {cls}"
            # back from the letterboxed input into the camera image, 0..1
            x1 = min(1.0, max(0.0, (box[0] * size - dx) / scale / w))
            x2 = min(1.0, max(0.0, (box[2] * size - dx) / scale / w))
            y1 = min(1.0, max(0.0, (box[1] * size - dy) / scale / h))
            y2 = min(1.0, max(0.0, (box[3] * size - dy) / scale / h))
            if x2 - x1 < config.OBJECTS_MIN_BOX or y2 - y1 < config.OBJECTS_MIN_BOX:
                continue
            found.append({
                "klasse": name,
                "score": round(score, 2),
                "box": [round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3)],
                # image centre = 0 deg, left in the image = positive, like the lidar
                "winkel": [round((0.5 - x2) * fov, 1), round((0.5 - x1) * fov, 1)],
            })
        found.sort(key=lambda o: -(o["box"][2] - o["box"][0]) * (o["box"][3] - o["box"][1]))
        with self._lock:
            self._objects = found
            self._time = captured
            self._image = image
            self._image_time = captured
            self._ms = 1000.0 * (time.perf_counter() - started)


def _color(name: str) -> tuple[int, int, int]:
    """Fixed colour per class (bgr), people always green."""
    if name == "Mensch":
        return (80, 220, 80)
    h = sum(ord(c) * (i + 1) for i, c in enumerate(name))
    return (60 + h * 37 % 196, 60 + h * 91 % 196, 60 + h * 53 % 196)


def _ascii(text: str) -> str:
    """opencv's fonts know no umlauts"""
    return (text.replace("ä", "ae").replace("ö", "oe").replace("ü", "ue")
            .replace("Ä", "Ae").replace("Ö", "Oe").replace("Ü", "Ue").replace("ß", "ss")
            .replace("…", "..."))
