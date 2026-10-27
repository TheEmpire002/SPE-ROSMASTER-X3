"""Lidar-gestützte Ortung: korrigiert die Koppelnavigation.

Arbeitsteilung zwischen beiden Verfahren:

* **Koppelnavigation** läuft immer und überall. Sie ist sofort verfügbar und
  nie "verloren", driftet aber mit jedem Meter.
* **Lidar-Abgleich** vergleicht die aktuelle Umgebung mit einer Aufnahme vom
  Startpunkt. Er summiert nichts auf, braucht aber Überschneidung: Hinter
  der nächsten Zimmerecke sieht der Roboter nichts Vertrautes mehr.

Deshalb: Die Koppelnavigation trägt grob, der Lidar zieht gerade, sobald er
etwas wiedererkennt. Praktisch heißt das, der Roboter findet mit der
Odometrie *in die Nähe* des Startpunkts und wird dort vom Lidar genau
eingewiesen -- genau das, was ein Home-Button braucht.

Das ist bewusst **kein** SLAM: Es wird keine Karte aufgebaut, nur eine
einzige Referenzaufnahme gehalten. Für "zurück zum Startpunkt" genügt das
und es kostet einen Bruchteil des Aufwands.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .. import config
from ..sensors.lidar import LidarService
from .odometry import Odometry
from .scan_matching import match

log = logging.getLogger(__name__)


class Localizer:
    """Hält die Referenzaufnahme und korrigiert damit die Position."""

    def __init__(self, odometry: Odometry, lidar: LidarService | None) -> None:
        self._odometry = odometry
        self._lidar = lidar
        self._lock = threading.Lock()

        self._referenz = None          # Punktwolke vom Startpunkt
        self._referenz_zeit = 0.0
        self._letztes_residuum: float | None = None
        self._korrekturen = 0
        self._verworfen = 0
        self._letzte_korrektur_m: float | None = None

    @property
    def hat_referenz(self) -> bool:
        return self._referenz is not None

    @property
    def verfuegbar(self) -> bool:
        return self._lidar is not None

    def status(self) -> dict:
        with self._lock:
            return {
                "lidar_vorhanden": self._lidar is not None,
                "referenz_aufgenommen": self._referenz is not None,
                "referenzpunkte": 0 if self._referenz is None else len(self._referenz),
                "korrekturen": self._korrekturen,
                "verworfen": self._verworfen,
                "letztes_residuum_m": (
                    round(self._letztes_residuum, 3)
                    if self._letztes_residuum is not None
                    else None
                ),
                "letzte_korrektur_m": (
                    round(self._letzte_korrektur_m, 3)
                    if self._letzte_korrektur_m is not None
                    else None
                ),
            }

    # --- Referenz ---------------------------------------------------------------

    def referenz_aufnehmen(self) -> bool:
        """Merkt sich, wie der Raum vom aktuellen Standort aussieht.

        Beim Setzen des Startpunkts aufzurufen. Ohne Referenz gibt es keine
        Korrektur -- dann bleibt es bei reiner Koppelnavigation.
        """
        if self._lidar is None:
            return False

        scan = self._lidar.latest()
        if scan is None or len(scan) < 60:
            log.warning(
                "Keine brauchbare Lidar-Aufnahme für den Startpunkt "
                "(%s Punkte) -- Ortung bleibt bei Koppelnavigation",
                0 if scan is None else len(scan),
            )
            return False

        with self._lock:
            self._referenz = scan.punkte()
            self._referenz_zeit = time.monotonic()
            self._korrekturen = 0
            self._verworfen = 0
            self._letztes_residuum = None
            self._letzte_korrektur_m = None

        log.info("Startpunkt eingemessen: %d Lidar-Punkte als Referenz", len(scan))
        return True

    def referenz_verwerfen(self) -> None:
        with self._lock:
            self._referenz = None

    # --- Laufender Abgleich -------------------------------------------------------

    def abgleichen(self) -> bool:
        """Einen Abgleich versuchen. True, wenn korrigiert wurde."""
        if self._lidar is None or self._referenz is None:
            return False

        scan = self._lidar.latest()
        if scan is None:
            return False

        # Zu alte Aufnahmen nicht verwenden: der Roboter ist inzwischen
        # woanders, und eine veraltete Messung zöge die Position zurück.
        if time.monotonic() - scan.zeit > 1.5:
            return False

        x, y, winkel_grad = self._odometry.pose()

        # Weit weg vom Startpunkt überschneidet sich zu wenig. Ein
        # Fehlabgleich wäre dort schlimmer als gar keine Korrektur: er
        # würde die Position verreißen, ohne dass es jemand merkt.
        if math.hypot(x, y) > config.MATCH_RADIUS_M:
            return False

        # Die Aufnahme zeigt, wo der Roboter *beim Messen* stand. Als
        # Startwert deshalb die damalige Position nehmen, nicht die
        # jetzige -- sonst beginnt die Suche schon schief.
        damals = self._odometry.pose_bei(scan.zeit)
        if damals is None:
            damals = (x, y, math.radians(winkel_grad))

        ergebnis = match(scan.punkte(), self._referenz, damals)

        with self._lock:
            if ergebnis is None:
                self._verworfen += 1
                return False

            self._letztes_residuum = ergebnis.residuum
            if ergebnis.residuum > config.MATCH_MAX_RESIDUAL_M:
                self._verworfen += 1
                return False

        # Seit der Aufnahme ist der Roboter weitergefahren. Diese Strecke
        # draufrechnen, sonst zieht die Korrektur ihn in die Vergangenheit
        # zurück -- bei 0,3 m/s und 0,3 s Verzug schon fast zehn Zentimeter.
        jetzt_x, jetzt_y, jetzt_grad = self._odometry.pose()
        jetzt_theta = math.radians(jetzt_grad)
        ziel_x = ergebnis.x + (jetzt_x - damals[0])
        ziel_y = ergebnis.y + (jetzt_y - damals[1])
        ziel_theta = ergebnis.theta + _winkeldifferenz(jetzt_theta, damals[2])

        with self._lock:
            self._korrekturen += 1
            self._letzte_korrektur_m = math.hypot(ziel_x - jetzt_x, ziel_y - jetzt_y)

        self._odometry.korrigieren(ziel_x, ziel_y, ziel_theta, config.MATCH_TRUST)
        return True

    # --- Hindernisse ---------------------------------------------------------------

    def hindernis_voraus(self, richtung_grad: float) -> float | None:
        """Abstand zum nächsten Hindernis in einer Richtung, falls messbar.

        ``richtung_grad`` im Roboterkoordinatensystem: 0 ist vorn. Für die
        Heimfahrt wird die Fahrtrichtung übergeben -- bei Mecanum ist das
        nicht zwangsläufig "vorn".
        """
        if self._lidar is None:
            return None
        scan = self._lidar.latest()
        if scan is None or time.monotonic() - scan.zeit > 2.0:
            return None

        abstand = scan.min_abstand(richtung_grad, config.HOME_CONE_DEG)
        return None if abstand == float("inf") else abstand


def _winkeldifferenz(a: float, b: float) -> float:
    """a - b, auf -pi..+pi gebracht."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
