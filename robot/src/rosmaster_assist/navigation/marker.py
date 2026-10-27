"""Genaues Andocken am Startpunkt mit einem ArUco-Marker.

Ein ausgedruckter Marker hängt in Sichtweite des Startpunkts, etwa an der
Wand. Beim Setzen des Startpunkts merkt sich der Roboter, wo er den Marker
im Bild sieht -- genauer: wie der Marker relativ zur Kamera liegt. Bei der
Heimfahrt bringt ihn der Lidar in die Nähe, und für das letzte Stück fährt
er so lange nach, bis er den Marker wieder genauso sieht.

**Warum das ohne Kamera-Kalibrierung genau wird:** Referenz und aktuelle
Messung werden mit denselben, nur ungefähr bekannten Kameradaten gerechnet
(Bildwinkel, Markergröße). Ein Fehler darin verzerrt beide gleich. Am Ziel,
wo beide Messungen übereinstimmen, hebt er sich vollständig auf -- er
beeinflusst nur, wie zügig der Roboter dorthin findet.

Die Erkennung läuft auf der CPU: ein 640×480-Bild braucht wenige
Millisekunden. Das KI-Modul bleibt frei für Aufgaben, die es wirklich
brauchen (Personenerkennung).
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .. import config

log = logging.getLogger(__name__)


class MarkerLage:
    """Wo der Marker relativ zur Kamera liegt (OpenCV-Kamerakoordinaten)."""

    __slots__ = ("R", "t", "mitte_px", "groesse_px")

    def __init__(self, R, t, mitte_px: tuple[float, float], groesse_px: float) -> None:
        self.R = R              # 3x3, Marker -> Kamera
        self.t = t              # 3,   Marker-Mitte in Kamerakoordinaten (m)
        self.mitte_px = mitte_px
        self.groesse_px = groesse_px


class MarkerAndocken:
    """Erkennt den Marker und rechnet die Abweichung zur Referenz aus."""

    def __init__(self, camera) -> None:
        self._camera = camera
        self._lock = threading.Lock()
        self.enabled = config.MARKER_ENABLED
        self._referenz: MarkerLage | None = None
        self._detektor = None
        self._fehler: str | None = None
        self._zuletzt_ausgewertet = 0.0

    # --- Zustand -------------------------------------------------------------

    @property
    def verfuegbar(self) -> bool:
        """Kamera da und OpenCV mit ArUco-Unterstützung installiert."""
        return self._camera is not None and self._detektor_holen() is not None

    @property
    def hat_referenz(self) -> bool:
        return self._referenz is not None

    @property
    def aktiv(self) -> bool:
        """Soll und kann bei der Heimfahrt angedockt werden?"""
        return self.enabled and self.hat_referenz and self.verfuegbar

    def einstellen(self, enabled: bool | None) -> None:
        if enabled is not None:
            self.enabled = bool(enabled)
            log.info("Andocken per Marker: %s", "an" if self.enabled else "aus")

    def zustand(self) -> dict:
        return {
            "enabled": self.enabled,
            "verfuegbar": self.verfuegbar,
            "referenz": self.hat_referenz,
            "fehler": self._fehler,
        }

    def status(self) -> dict:
        """Für /marker/status: Zustand plus eine frische Erkennung."""
        lage = self.erkennen(nur_neu=False)
        abweichung = self.abweichung(lage) if lage is not None else None
        return {
            **self.zustand(),
            "marker_id": config.MARKER_ID,
            "marker_groesse_m": config.MARKER_SIZE_M,
            "sichtbar": lage is not None,
            "entfernung_m": round(float(lage.t[2]), 3) if lage is not None else None,
            "abweichung": (
                {
                    "x_m": round(abweichung[0], 3),
                    "y_m": round(abweichung[1], 3),
                    "winkel_grad": round(math.degrees(abweichung[2]), 1),
                }
                if abweichung is not None
                else None
            ),
        }

    # --- Referenz ------------------------------------------------------------

    def referenz_aufnehmen(self) -> bool:
        """Merken, wie der Marker vom Startpunkt aus aussieht."""
        if self._camera is None:
            return False
        lage = self.erkennen(nur_neu=False)
        with self._lock:
            self._referenz = lage
        if lage is None:
            log.info("Kein Marker im Bild -- Andocken per Kamera nicht möglich")
            return False
        log.info(
            "Marker als Referenz gemerkt (%.2f m entfernt, Bildmitte %.0f px)",
            float(lage.t[2]),
            lage.mitte_px[0],
        )
        return True

    def referenz_verwerfen(self) -> None:
        with self._lock:
            self._referenz = None

    # --- Messen --------------------------------------------------------------

    def erkennen(self, nur_neu: bool = True, max_alter_s: float = 0.6) -> MarkerLage | None:
        """Marker im aktuellen Kamerabild suchen.

        ``nur_neu``: dasselbe Bild nicht zweimal auswerten (für Regelungen).
        """
        if self._camera is None:
            return None
        detektor = self._detektor_holen()
        if detektor is None:
            return None

        frame, zeit = self._camera.latest_mit_zeit()
        if frame is None or time.monotonic() - zeit > max_alter_s:
            self._fehler = "kein aktuelles Kamerabild"
            return None
        if nur_neu and zeit <= self._zuletzt_ausgewertet:
            return None
        self._zuletzt_ausgewertet = zeit

        try:
            return self._auswerten(detektor, frame[0])
        except Exception as exc:
            self._fehler = f"{type(exc).__name__}: {exc}"
            log.debug("Marker-Auswertung fehlgeschlagen", exc_info=True)
            return None

    def abweichung(self, lage: MarkerLage | None = None) -> tuple[float, float, float] | None:
        """Wo der Roboter relativ zum Startpunkt steht, laut Marker.

        (x, y, Winkel im Bogenmaß) in Startpunkt-Koordinaten: x in die
        Blickrichtung beim Start, y links davon, Winkel gegen den
        Uhrzeigersinn. (0, 0, 0) heißt: steht genau wie beim Setzen.
        """
        import numpy as np

        ref = self._referenz
        if ref is None:
            return None
        if lage is None:
            lage = self.erkennen()
        if lage is None:
            return None

        # Ein Punkt in den aktuellen Kamerakoordinaten, ausgedrückt in denen
        # der Referenzkamera: über den Marker als gemeinsamen Bezug.
        #   X_ref = R0 · R^T · (X_jetzt - t) + t0
        dreh = ref.R @ lage.R.T
        position = ref.t - dreh @ lage.t        # Ursprung der jetzigen Kamera
        blick = dreh @ np.array([0.0, 0.0, 1.0])  # ihre Blickrichtung

        # OpenCV-Kamera: x rechts, y unten, z nach vorn.
        # Roboter: x vorn, y links -- also x = z und y = -x.
        x = float(position[2])
        y = float(-position[0])
        winkel = math.atan2(float(-blick[0]), float(blick[2]))
        return x, y, winkel

    # --- Druckvorlage --------------------------------------------------------

    @staticmethod
    def druckvorlage_png(pixel: int = 800) -> bytes:
        """Der Marker als PNG mit weißem Rand -- zum Ausdrucken."""
        import cv2
        import numpy as np

        aruco = cv2.aruco
        woerterbuch = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        if hasattr(aruco, "generateImageMarker"):
            bild = aruco.generateImageMarker(woerterbuch, config.MARKER_ID, pixel)
        else:  # OpenCV < 4.7
            bild = aruco.drawMarker(woerterbuch, config.MARKER_ID, pixel)
        # Der weiße Rand ist Pflicht: ohne ihn findet die Erkennung die
        # Außenkante des Markers nicht.
        rand = pixel // 4
        bild = np.pad(bild, rand, constant_values=255)
        ok, png = cv2.imencode(".png", bild)
        if not ok:
            raise RuntimeError("PNG ließ sich nicht erzeugen")
        return png.tobytes()

    # --- intern --------------------------------------------------------------

    def _detektor_holen(self):
        """Erkennungsfunktion, je nach OpenCV-Version. None ohne ArUco."""
        if self._detektor is not None:
            return self._detektor or None
        try:
            import cv2

            aruco = cv2.aruco
            woerterbuch = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            if hasattr(aruco, "ArucoDetector"):  # OpenCV >= 4.7
                det = aruco.ArucoDetector(woerterbuch, aruco.DetectorParameters())
                self._detektor = det.detectMarkers
            else:
                parameter = aruco.DetectorParameters_create()
                self._detektor = lambda grau: aruco.detectMarkers(
                    grau, woerterbuch, parameters=parameter
                )
        except Exception as exc:
            self._fehler = (
                f"OpenCV ohne ArUco ({exc}) -- pip install opencv-contrib-python"
            )
            log.warning("Marker-Erkennung nicht verfügbar: %s", self._fehler)
            self._detektor = False  # nicht bei jedem Aufruf neu versuchen
            return None
        return self._detektor

    def _auswerten(self, detektor, daten: bytes) -> MarkerLage | None:
        import cv2
        import numpy as np

        grau = cv2.imdecode(np.frombuffer(daten, np.uint8), cv2.IMREAD_GRAYSCALE)
        if grau is None:
            self._fehler = "Kamerabild nicht lesbar"
            return None

        ecken, ids, _ = detektor(grau)
        if ids is None:
            self._fehler = "Marker nicht im Bild"
            return None
        treffer = [i for i, mid in enumerate(ids.flatten()) if mid == config.MARKER_ID]
        if not treffer:
            self._fehler = f"anderer Marker im Bild, gesucht wird ID {config.MARKER_ID}"
            return None
        pixel = ecken[treffer[0]].reshape(4, 2).astype(np.float64)

        hoehe, breite = grau.shape[:2]
        f = (breite / 2.0) / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2.0)
        kamera = np.array([[f, 0, breite / 2.0], [0, f, hoehe / 2.0], [0, 0, 1]])

        # Eckpunkte in der Reihenfolge, die ArUco liefert und die das
        # Verfahren für quadratische Marker erwartet: oben links, oben
        # rechts, unten rechts, unten links.
        h = config.MARKER_SIZE_M / 2.0
        modell = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            modell, pixel, kamera, None, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            self._fehler = "Lage des Markers nicht bestimmbar"
            return None

        R, _ = cv2.Rodrigues(rvec)
        mitte = pixel.mean(axis=0)
        groesse = float(np.mean([np.linalg.norm(pixel[i] - pixel[(i + 1) % 4]) for i in range(4)]))
        self._fehler = None
        return MarkerLage(R, tvec.reshape(3), (float(mitte[0]), float(mitte[1])), groesse)
