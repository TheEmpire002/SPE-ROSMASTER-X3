"""Position aus aufeinanderfolgenden Lidar-Aufnahmen (Lidar-Odometrie).

Die Koppelnavigation (``odometry.py``) taugt am X3 nur für Strecken, nicht
für Drehungen: Die IMU der Bibliothek liefert als Kurswinkel immer 0
(Sensor-Check 28.10.2026), und am Boden dreht er nur rund drei Viertel
dessen, was befohlen wird. Für SLAM und für das Bahnenfahren beim
Kartieren ist eine falsche Blickrichtung aber das Schlimmste, was passieren
kann: Jede Drehung, die nicht gemeldet wird, verdreht die Karte.

Deshalb wird hier jede neue Aufnahme per ICP (``scan_matching.py``) auf
eine kurz zuvor gemachte gelegt. Die nötige Drehung und Verschiebung ist
die Bewegung des Roboters dazwischen. Die Räder liefern nur den Startwert
für die Suche, die Wände das Ergebnis.

Verglichen wird nicht mit der jeweils vorigen Aufnahme, sondern mit einer
**Bezugsaufnahme**, die erst nach etwas Bewegung erneuert wird. Steht der
Roboter, summiert sich so kein Rauschen auf: Er vergleicht immer wieder
mit derselben Aufnahme und bleibt, wo er ist.

Wie jede Odometrie driftet auch diese -- aber viel weniger als die Räder,
und vor allem nicht bei jeder Drehung um ein Viertel. Den Rest gleicht
slam_toolbox mit seiner Karte aus.

Läuft nur, solange jemand sie braucht: die Kartierung und, solange eine
Karte geladen ist, die Ortung in der Karte (``kartenortung.py``). Der
Abgleich kostet CPU, und Lastsprünge sind am X3 heikel (docs/hardware.md).
Deshalb meldet sich jeder Nutzer mit Namen an und ab; erst wenn keiner
mehr da ist, endet sie. Ein Neustart beginnt wieder bei (0, 0, 0) -- wer
sich auf den Rahmen verlässt, erkennt das an ``lauf``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import NamedTuple

from .. import config
from .scan_matching import match

log = logging.getLogger(__name__)

#: Mehr Punkte machen den Abgleich kaum genauer, aber deutlich langsamer
#: (er vergleicht jeden mit jedem).
_MAX_PUNKTE = 240
#: Weiter als das darf das Ergebnis nicht von der Schätzung abweichen. Sonst
#: hat ICP wahrscheinlich eine falsche Lage gefunden (lange Flure, Spiegelungen).
_MAX_SPRUNG_M = 0.25
_MAX_SPRUNG_RAD = math.radians(15.0)
#: Höher geschätzte Drehraten sind ein Rechenfehler, keine Bewegung.
_MAX_OMEGA = 2.0


class Messung(NamedTuple):
    """Eine verarbeitete Aufnahme mit der Position zu ihrem Messzeitpunkt."""

    nr: int
    scan: object          # sensors.lidar.Scan
    pose: tuple[float, float, float]  # x, y, Winkel (Bogenmaß)
    zeit: float           # monotonic, wie Scan.zeit


class ScanOdometrie:
    """Hält die Position aus dem Lidar-Abgleich, im Hintergrund nachgeführt."""

    def __init__(self, lidar, odometry) -> None:
        self._lidar = lidar
        self._odometry = odometry
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._nutzer: set[str] = set()
        #: Zählt jeden echten Start. Ändert er sich, begann der Rahmen neu.
        self.lauf = 0
        self._zuruecksetzen()

    def _zuruecksetzen(self) -> None:
        self._pose = (0.0, 0.0, 0.0)
        self._messung: Messung | None = None
        self._bezug_punkte = None
        self._bezug_pose = (0.0, 0.0, 0.0)
        self._relativ = (0.0, 0.0, 0.0)   # letzte Aufnahme relativ zur Bezugsaufnahme
        self._odo_vorher: tuple[float, float, float] | None = None
        self._zeit_vorher: float | None = None
        self._omega = 0.0
        self._abgleiche = 0
        self._verworfen = 0
        self._letztes_residuum: float | None = None

    # --- Steuerung -------------------------------------------------------------

    @property
    def laeuft(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, nutzer: str = "kartierung") -> None:
        """Beginnt bei (0, 0, 0) -- das ist der Ursprung der Karte.

        Läuft sie schon (für einen anderen Nutzer), bleibt der Rahmen, wie
        er ist: Die Ortung soll ihre Lage nicht verlieren, nur weil eine
        neue Kartierung beginnt, und umgekehrt.
        """
        if self._lidar is None:
            return
        self._nutzer.add(nutzer)
        if self.laeuft:
            return
        with self._lock:
            self._zuruecksetzen()
        self.lauf += 1
        self._stop.clear()
        self._thread = threading.Thread(target=self._schleife, name="scan-odometrie", daemon=True)
        self._thread.start()
        log.info("Lidar-Odometrie gestartet (für %s)", nutzer)

    def stop(self, nutzer: str = "kartierung") -> None:
        """Abmelden; beendet sie erst, wenn niemand sie mehr braucht."""
        self._nutzer.discard(nutzer)
        if self._nutzer:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    # --- Abfrage ---------------------------------------------------------------

    def messung(self) -> Messung | None:
        """Die zuletzt verarbeitete Aufnahme samt Position."""
        with self._lock:
            return self._messung

    def pose(self) -> tuple[float, float, float] | None:
        """Position jetzt (x, y, Bogenmaß). None, solange nichts gemessen wurde.

        Zwischen zwei Aufnahmen (0,15-0,2 s) wird mit der Radbewegung und der
        zuletzt gemessenen Drehrate fortgeschrieben, damit Regler mit 20 Hz
        nicht auf einem veralteten Wert sitzen.
        """
        with self._lock:
            messung = self._messung
            odo_vorher = self._odo_vorher
            omega = self._omega
        if messung is None or odo_vorher is None:
            return None
        dt = min(max(time.monotonic() - messung.zeit, 0.0), 0.3)
        dx, dy = self._radweg(odo_vorher, self._odo_jetzt())
        return _verketten(messung.pose, (dx, dy, omega * dt))

    def alter(self) -> float | None:
        """Sekunden seit der letzten verarbeiteten Aufnahme."""
        messung = self.messung()
        return time.monotonic() - messung.zeit if messung else None

    def status(self) -> dict:
        with self._lock:
            pose = self._pose
            return {
                "laeuft": self.laeuft,
                "nutzer": sorted(self._nutzer),
                "pose": [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)],
                "abgleiche": self._abgleiche,
                "verworfen": self._verworfen,
                "letztes_residuum_m": (
                    round(self._letztes_residuum, 3) if self._letztes_residuum is not None else None
                ),
            }

    # --- Verarbeitung ----------------------------------------------------------

    def _schleife(self) -> None:
        letzter = None
        while not self._stop.is_set():
            scan = self._lidar.latest()
            if scan is not None and scan is not letzter:
                letzter = scan
                try:
                    self._verarbeiten(scan)
                except Exception:
                    # Ein einzelner kaputter Abgleich darf die Position nicht
                    # einfrieren -- die nächste Aufnahme versucht es neu.
                    log.exception("Lidar-Odometrie: Aufnahme nicht verarbeitet")
            self._stop.wait(0.02)

    def _verarbeiten(self, scan) -> None:
        punkte = scan.punkte()
        if len(punkte) > _MAX_PUNKTE:
            punkte = punkte[:: math.ceil(len(punkte) / _MAX_PUNKTE)]
        odo = self._odometry.pose_bei(scan.zeit) or self._odo_jetzt()

        if self._bezug_punkte is None:
            # Erste Aufnahme: Sie ist der Bezug, hier ist der Ursprung.
            with self._lock:
                self._bezug_punkte = punkte
                self._odo_vorher, self._zeit_vorher = odo, scan.zeit
                self._messung = Messung(1, scan, self._pose, scan.zeit)
            return

        # Schätzung für die Suche: Strecke von den Rädern, Drehung aus der
        # zuletzt gemessenen Drehrate. Die Räder taugen für Drehungen nicht
        # (siehe oben), für ein paar Zentimeter Strecke schon.
        dt = max(scan.zeit - self._zeit_vorher, 1e-3)
        dx, dy = self._radweg(self._odo_vorher, odo)
        schaetzung = _verketten(self._relativ, (dx, dy, self._omega * dt))

        ergebnis = match(punkte, self._bezug_punkte, schaetzung, iterationen=15)
        gut = (
            ergebnis is not None
            and ergebnis.residuum <= config.MAP_ICP_MAX_RESIDUUM_M
            and math.hypot(ergebnis.x - schaetzung[0], ergebnis.y - schaetzung[1]) < _MAX_SPRUNG_M
            and abs(_normalisieren(ergebnis.theta - schaetzung[2])) < _MAX_SPRUNG_RAD
        )
        relativ = (ergebnis.x, ergebnis.y, ergebnis.theta) if gut else schaetzung
        neu = _verketten(self._bezug_pose, relativ)

        with self._lock:
            omega = _normalisieren(neu[2] - self._pose[2]) / dt
            self._omega = max(-_MAX_OMEGA, min(_MAX_OMEGA, omega))
            self._pose = neu
            if gut:
                self._abgleiche += 1
                self._letztes_residuum = ergebnis.residuum
            else:
                self._verworfen += 1
            # Bezug erneuern, wenn er sich zu weit entfernt hat oder der
            # Abgleich nicht mehr passt -- sonst überlappen die Aufnahmen
            # irgendwann zu wenig.
            if (
                not gut
                or math.hypot(relativ[0], relativ[1]) > config.MAP_BEZUG_M
                or abs(relativ[2]) > math.radians(config.MAP_BEZUG_GRAD)
            ):
                self._bezug_punkte = punkte
                self._bezug_pose = neu
                self._relativ = (0.0, 0.0, 0.0)
            else:
                self._relativ = relativ
            self._odo_vorher, self._zeit_vorher = odo, scan.zeit
            nr = self._messung.nr + 1 if self._messung else 1
            self._messung = Messung(nr, scan, neu, scan.zeit)

    def _odo_jetzt(self) -> tuple[float, float, float]:
        x, y, grad = self._odometry.pose()
        return x, y, math.radians(grad)

    @staticmethod
    def _radweg(vorher, jetzt) -> tuple[float, float]:
        """Strecke laut Rädern zwischen zwei Odometrie-Posen, im Roboterrahmen.

        Mit dem Winkel der Odometrie selbst zurückgerechnet: Ob der stimmt,
        ist dabei egal -- sie hat mit genau diesem Winkel aufsummiert, das
        Zurückrechnen ergibt also wieder die gefahrene Strecke.
        """
        dx, dy, _ = _relativ(vorher, jetzt)
        # Wird der Startpunkt neu gesetzt, springt die Odometrie auf (0, 0).
        # Das ist keine Fahrt und darf nicht in die Position eingehen.
        if math.hypot(dx, dy) > config.MAP_SPRUNG_M:
            return 0.0, 0.0
        return dx, dy


def _verketten(a, b) -> tuple[float, float, float]:
    """Pose ``b`` (relativ zu ``a``) an Pose ``a`` anhängen."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalisieren(a[2] + b[2])


def _relativ(a, b) -> tuple[float, float, float]:
    """Pose ``b`` aus Sicht von Pose ``a``."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, _normalisieren(b[2] - a[2])


def _normalisieren(winkel: float) -> float:
    return math.atan2(math.sin(winkel), math.cos(winkel))
