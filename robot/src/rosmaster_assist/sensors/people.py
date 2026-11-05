"""Find people in the camera image (OpenCV HOG).

The lidar only sees legs -- it cannot reliably tell whether two posts are a
person or a chair. The camera can. OpenCV's HOG detector needs no model to
download and manages a few frames per second on the Pi at 320 px width;
exploring needs no more than that.

Detection only runs while the robot drives autonomously (``activate``),
because it costs noticeable CPU -- and CPU load spikes trigger USB resets
on the X3 (sensor check 28.10.2026).

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


class PersonDetection:
    def __init__(self, camera: CameraService | None) -> None:
        self._camera = camera
        self._lock = threading.Lock()
        self._active = threading.Event()
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
        return self._hog is not None and config.PERSON_DETECTION_ENABLED

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

    def activate(self, an: bool) -> None:
        """Only compute while exploring."""
        if an:
            self._active.set()
        else:
            self._active.clear()
            with self._lock:
                self._boxes = []

    def boxes(self) -> list[dict]:
        """Current hits: x1,y1,x2,y2 relative to the image size (0..1), score."""
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
        with self._lock:
            return {
                "verfuegbar": self.available,
                "aktiv": self._active.is_set() and self.available,
                "fehler": (
                    "abgeschaltet (Einstellungen)"
                    if self._hog is not None and not config.PERSON_DETECTION_ENABLED
                    else self._error
                ),
                # Which image the boxes belong to -- the app draws them only
                # over this camera.
                "kamera": self._camera.name if self._camera is not None else None,
                "letzte_erkennung_vor_s": round(time.monotonic() - self._time, 1) if self._time else None,
            }

    def _loop(self) -> None:
        while not self._stop.is_set():
            if not self._active.wait(timeout=1.0):
                continue
            if not config.PERSON_DETECTION_ENABLED:
                with self._lock:
                    self._boxes = []
                self._stop.wait(1.0)
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
