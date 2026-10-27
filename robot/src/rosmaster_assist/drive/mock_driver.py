"""Antrieb ohne Hardware -- schreibt Fahrbefehle nur ins Log.

Damit lässt sich das komplette Projekt auf einem Laptop entwickeln und
testen, obwohl es nur einen Roboter gibt.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


class MockDriver(DriveBase):
    name = "mock"

    #: Anteil der befohlenen Geschwindigkeit, den die "Encoder" melden.
    #: Echte Mecanum-Räder rutschen; mit 1.0 wäre die Simulation unrealistisch
    #: optimistisch und die Lidar-Korrektur hätte nichts zu korrigieren.
    SCHLUPF = 0.88

    def __init__(self) -> None:
        self._connected = False
        self._last = STOP

        # Wo der Roboter in der Simulation wirklich steht. Davon weiß die
        # Positionsverfolgung nichts -- sie bekommt nur die geschlupften
        # Messwerte. Der simulierte Lidar dagegen tastet von hier aus ab,
        # genau wie es die Hardware täte.
        self._wahr_x = 0.0
        self._wahr_y = 0.0
        self._wahr_theta = 0.0
        self._wahr_zeit: float | None = None
        # Fahrbefehle und der simulierte Lidar greifen aus verschiedenen
        # Threads zu. Ohne Sperre überschreiben sie sich gegenseitig den
        # Zeitstempel, und die "wahre" Position wird selbst falsch.
        self._wahr_lock = threading.Lock()

    def wahre_pose_nullen(self) -> None:
        """Setzt die simulierte Wirklichkeit auf den Startpunkt zurück.

        Nur für die Simulation: Beim Setzen des Startpunkts nullt die
        Positionsverfolgung ihre Schätzung. Täte die Simulation das nicht
        auch, stünde der gedachte Roboter irgendwo weit außerhalb des
        gedachten Raums -- und der simulierte Lidar lieferte Unsinn.
        """
        with self._wahr_lock:
            self._wahr_x = self._wahr_y = self._wahr_theta = 0.0
            self._wahr_zeit = None

    def wahre_pose(self) -> tuple[float, float, float]:
        """(x, y, Winkel im Bogenmaß) -- nur in der Simulation bekannt."""
        with self._wahr_lock:
            self._fortschreiben()
            return self._wahr_x, self._wahr_y, self._wahr_theta

    def _wahre_pose_fortschreiben(self) -> None:
        with self._wahr_lock:
            self._fortschreiben()

    def _fortschreiben(self) -> None:
        """Nur mit gehaltener Sperre aufrufen."""
        jetzt = time.monotonic()
        if self._wahr_zeit is None:
            self._wahr_zeit = jetzt
            return
        dt = jetzt - self._wahr_zeit
        self._wahr_zeit = jetzt
        if dt <= 0 or dt > 1.0:
            return

        c = self._last
        mittel = self._wahr_theta + c.omega * dt / 2.0
        self._wahr_x += (c.vx * math.cos(mittel) - c.vy * math.sin(mittel)) * dt
        self._wahr_y += (c.vx * math.sin(mittel) + c.vy * math.cos(mittel)) * dt
        self._wahr_theta += c.omega * dt

    def connect(self) -> None:
        self._connected = True
        log.info("Mock-Antrieb bereit (keine echte Hardware)")

    def disconnect(self) -> None:
        self.stop()
        self._connected = False
        log.info("Mock-Antrieb beendet")

    def drive(self, cmd: DriveCommand) -> None:
        # Vor dem Übernehmen fortschreiben, damit die zurückgelegte Strecke
        # dem alten Befehl zugerechnet wird und nicht dem neuen.
        self._wahre_pose_fortschreiben()
        # Nur bei Änderung loggen -- sonst ist die Konsole bei 10 Befehlen
        # pro Sekunde unlesbar.
        # "SIM" im Klartext, damit niemand diese Zeilen für echte Fahrt
        # hält und den Fehler woanders sucht, wenn sich nichts bewegt.
        if cmd != self._last:
            log.info("SIM fahren  vx=%+.2f  vy=%+.2f  omega=%+.2f",
                     cmd.vx, cmd.vy, cmd.omega)
            self._last = cmd

    def stop(self) -> None:
        if self._last != STOP:
            log.info("SIM anhalten")
            self._last = STOP

    def beep(self, ms: int = 100) -> None:
        log.info("SIM piep (%d ms)", ms)

    def read_motion(self) -> tuple[float, float, float] | None:
        """Was die "Encoder" melden -- mit Schlupf, also leicht daneben.

        Absichtlich nicht der wahre Wert: nur so entsteht in der Simulation
        derselbe Drift wie in Wirklichkeit, und nur so lässt sich prüfen,
        ob die Lidar-Korrektur ihn tatsächlich ausgleicht.
        """
        return (
            self._last.vx * self.SCHLUPF,
            self._last.vy * self.SCHLUPF,
            self._last.omega * self.SCHLUPF,
        )

    @property
    def connected(self) -> bool:
        return self._connected
