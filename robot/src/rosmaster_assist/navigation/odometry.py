"""Mitrechnen, wo der Roboter steht.

Verfahren: Koppelnavigation. Es gibt keinen Sensor, der die Position
verrät -- sie wird laufend aus Geschwindigkeit und Blickrichtung
aufsummiert, ausgehend vom Startpunkt.

**Das driftet, und zwar unvermeidlich.** Jeder Messfehler und jedes
durchdrehende Rad geht dauerhaft in die Summe ein und wird nie wieder
korrigiert. Mecanum-Räder rutschen bauartbedingt viel, besonders beim
Drehen und auf glattem Boden. In einem Zimmer sind einige Dezimeter normal.

Für den Home-Button reicht das. Wer Zentimeter braucht, kommt um eine
äußere Referenz nicht herum -- Lidar und SLAM.

Koordinaten: x zeigt in die anfängliche Blickrichtung, y nach links,
Winkel gegen den Uhrzeigersinn. Startpunkt ist (0, 0) mit Winkel 0.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from ..drive.base import DriveBase
from ..protocol import DriveCommand

log = logging.getLogger(__name__)


class Odometry:
    """Schreibt die Position aus den Bewegungsdaten fort."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Bogenmaß

        self._letzte_zeit: float | None = None
        # Rohwert der IMU beim Nullen -- der Winkel wird relativ dazu geführt,
        # weil die IMU ihren eigenen Nullpunkt hat.
        self._imu_referenz: float | None = None
        self._imu_vorher: float | None = None
        self._gemessen = False

    # --- Abfrage -------------------------------------------------------------

    def pose(self) -> tuple[float, float, float]:
        """(x, y, Winkel in Grad)."""
        with self._lock:
            return self.x, self.y, math.degrees(self.theta)

    @property
    def misst_tatsaechlich(self) -> bool:
        """Ob echte Messwerte eingehen oder nur die Befehle gerechnet werden."""
        return self._gemessen

    def distanz_zum_start(self) -> float:
        with self._lock:
            return math.hypot(self.x, self.y)

    # --- Nullen --------------------------------------------------------------

    def reset(self) -> None:
        """Hier ist jetzt der Startpunkt.

        Setzt auch den IMU-Bezug neu, damit der Winkel wieder bei null
        beginnt -- sonst zeigte der Roboter nach dem Nullen in eine
        Richtung, die es nie gab.
        """
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self._imu_referenz = None
            self._imu_vorher = None
            self._letzte_zeit = None
        log.info("Startpunkt neu gesetzt")

    # --- Fortschreiben --------------------------------------------------------

    def update(self, befohlen: DriveCommand) -> None:
        """Einen Zeitschritt rechnen. Regelmäßig aufrufen."""
        jetzt = time.monotonic()
        if self._letzte_zeit is None:
            self._letzte_zeit = jetzt
            return

        dt = jetzt - self._letzte_zeit
        self._letzte_zeit = jetzt
        # Nach einer Pause (Server hing, Rechner im Ruhezustand) wäre der
        # Schritt riesig und würde die Position verreißen.
        if dt <= 0 or dt > 1.0:
            return

        gemessen = self._driver.read_motion()
        self._gemessen = gemessen is not None
        vx, vy, omega = gemessen if gemessen is not None else (
            befohlen.vx,
            befohlen.vy,
            befohlen.omega,
        )

        with self._lock:
            neuer_winkel = self._winkel_bestimmen(omega, dt)

            # In der Mitte des Zeitschritts auswerten: bei gleichzeitigem
            # Fahren und Drehen ist das deutlich genauer, als Anfangs- oder
            # Endwinkel zu nehmen.
            mittel = (self.theta + neuer_winkel) / 2.0
            self.x += (vx * math.cos(mittel) - vy * math.sin(mittel)) * dt
            self.y += (vx * math.sin(mittel) + vy * math.cos(mittel)) * dt
            self.theta = neuer_winkel

    def _winkel_bestimmen(self, omega: float, dt: float) -> float:
        """Blickrichtung aus der IMU, ersatzweise aus der Drehrate."""
        imu = self._driver.read_heading()
        if imu is None:
            # Aufsummieren: der Fehler wächst mit jeder Drehung.
            return _normalisieren(self.theta + omega * dt)

        if self._imu_referenz is None:
            self._imu_referenz = imu
            self._imu_vorher = imu
            return self.theta

        # Der Sprung von +180 auf -180 muss abgefangen werden, sonst dreht
        # sich der Roboter rechnerisch schlagartig um eine halbe Drehung.
        roh = math.radians(imu - self._imu_referenz)
        self._imu_vorher = imu
        return _normalisieren(roh)


def _normalisieren(winkel: float) -> float:
    """Auf -pi..+pi bringen."""
    return math.atan2(math.sin(winkel), math.cos(winkel))
