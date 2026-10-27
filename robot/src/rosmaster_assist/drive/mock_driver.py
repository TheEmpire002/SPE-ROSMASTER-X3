"""Antrieb ohne Hardware -- schreibt Fahrbefehle nur ins Log.

Damit lässt sich das komplette Projekt auf einem Laptop entwickeln und
testen, obwohl es nur einen Roboter gibt.
"""

from __future__ import annotations

import logging

from .base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


class MockDriver(DriveBase):
    name = "mock"

    def __init__(self) -> None:
        self._connected = False
        self._last = STOP

    def connect(self) -> None:
        self._connected = True
        log.info("Mock-Antrieb bereit (keine echte Hardware)")

    def disconnect(self) -> None:
        self.stop()
        self._connected = False
        log.info("Mock-Antrieb beendet")

    def drive(self, cmd: DriveCommand) -> None:
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

    @property
    def connected(self) -> bool:
        return self._connected
