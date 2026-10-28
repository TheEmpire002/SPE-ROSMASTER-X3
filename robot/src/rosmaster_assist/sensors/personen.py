"""Personen im Kamerabild finden (OpenCV-HOG).

Der Lidar sieht nur Beine -- ob zwei Stäbe ein Mensch oder ein Stuhl sind,
kann er nicht sicher sagen. Die Kamera kann das. Der HOG-Detektor von
OpenCV braucht kein Modell zum Herunterladen und schafft auf dem Pi bei
320 px Breite einige Bilder pro Sekunde; mehr braucht das Erkunden nicht.

Die Erkennung läuft nur, solange der Roboter autonom fährt (``aktivieren``),
denn sie kostet spürbar CPU -- und CPU-Lastsprünge lösen am X3 USB-Resets
aus (Sensor-Check 28.10.2026).

Fehlt OpenCV, meldet ``status()`` das, und alles andere läuft weiter: Dann
benennt eben nur der Lidar die Hindernisse.
"""

from __future__ import annotations

import logging
import threading
import time

from .. import config
from .camera import CameraService

log = logging.getLogger(__name__)

#: Ältere Treffer zählen nicht mehr -- die Person kann längst weg sein.
_MAX_ALTER_S = 1.5


class Personenerkennung:
    def __init__(self, camera: CameraService | None) -> None:
        self._camera = camera
        self._lock = threading.Lock()
        self._aktiv = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._boxen: list[dict] = []
        self._zeit = 0.0
        self._fehler: str | None = None
        self._hog = None
        self._cv2 = None
        self._np = None
        if not config.PERSON_DETECTION_ENABLED:
            self._fehler = "abgeschaltet (config.PERSON_DETECTION_ENABLED)"
        elif camera is None:
            self._fehler = "keine Kamera"
        else:
            try:
                import cv2
                import numpy as np

                hog = cv2.HOGDescriptor()
                hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
                self._cv2, self._np, self._hog = cv2, np, hog
            except Exception as exc:  # ImportError oder kaputte OpenCV-Version
                self._fehler = f"nicht verfügbar (opencv fehlt: {exc.__class__.__name__})"
                log.warning("Personenerkennung %s", self._fehler)

    @property
    def verfuegbar(self) -> bool:
        return self._hog is not None

    def start(self) -> None:
        if not self.verfuegbar or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="personen", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._aktiv.set()  # aus dem Warten wecken
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def aktivieren(self, an: bool) -> None:
        """Nur während der Erkundung rechnen."""
        if an:
            self._aktiv.set()
        else:
            self._aktiv.clear()
            with self._lock:
                self._boxen = []

    def boxen(self) -> list[dict]:
        """Aktuelle Treffer: x1,y1,x2,y2 relativ zur Bildgröße (0..1), score."""
        with self._lock:
            if time.monotonic() - self._zeit > _MAX_ALTER_S:
                return []
            return list(self._boxen)

    def winkelbereiche(self) -> list[tuple[float, float]]:
        """Richtungen (von, bis) in Grad, in denen eine Person steht.

        Bildmitte = 0°, links im Bild = positive Winkel (wie beim Lidar).
        Setzt voraus, dass die Kamera nach vorn schaut.
        """
        fov = config.CAMERA_FOV_DEG
        return [((0.5 - b["x2"]) * fov, (0.5 - b["x1"]) * fov) for b in self.boxen()]

    def status(self) -> dict:
        with self._lock:
            return {
                "verfuegbar": self.verfuegbar,
                "aktiv": self._aktiv.is_set() and self.verfuegbar,
                "fehler": self._fehler,
                "letzte_erkennung_vor_s": round(time.monotonic() - self._zeit, 1) if self._zeit else None,
            }

    def _loop(self) -> None:
        takt = 1.0 / max(0.1, config.PERSON_DETECT_HZ)
        while not self._stop.is_set():
            if not self._aktiv.wait(timeout=1.0):
                continue
            beginn = time.monotonic()
            try:
                self._erkennen()
            except Exception as exc:
                # Ein kaputtes Bild darf die Erkennung nicht beenden.
                with self._lock:
                    self._fehler = f"{exc.__class__.__name__}: {exc}"
                log.debug("Personenerkennung: Bild übersprungen", exc_info=True)
            rest = takt - (time.monotonic() - beginn)
            if rest > 0:
                self._stop.wait(rest)

    def _erkennen(self) -> None:
        frame = self._camera.latest() if self._camera is not None else None
        if frame is None:
            return
        cv2, np = self._cv2, self._np
        bild = cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if bild is None:
            return
        h, w = bild.shape[:2]
        breite = config.PERSON_DETECT_WIDTH
        if w > breite:
            bild = cv2.resize(bild, (breite, int(h * breite / w)))
            h, w = bild.shape[:2]
        rechtecke, gewichte = self._hog.detectMultiScale(
            bild, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        boxen = []
        for (x, y, bw, bh), score in zip(rechtecke, np.ravel(gewichte) if len(gewichte) else []):
            if float(score) < config.PERSON_MIN_SCORE:
                continue
            boxen.append({
                "x1": round(x / w, 3), "y1": round(y / h, 3),
                "x2": round((x + bw) / w, 3), "y2": round((y + bh) / h, 3),
                "score": round(float(score), 2),
            })
        with self._lock:
            self._boxen = boxen
            self._zeit = time.monotonic()
            self._fehler = None
