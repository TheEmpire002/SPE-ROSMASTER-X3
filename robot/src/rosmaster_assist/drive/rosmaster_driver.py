"""Antrieb über Yahboons `Rosmaster_Lib` (echte Hardware).

Dieses Modul wird nur auf dem Roboter importiert. Auf Entwicklungsrechnern
nimmt `create_driver` stattdessen den Mock.
"""

from __future__ import annotations

import logging

from .base import DriveBase
from .. import config
from ..protocol import DriveCommand

log = logging.getLogger(__name__)


class RosmasterDriver(DriveBase):
    name = "rosmaster"

    def __init__(self, port: str | None = None) -> None:
        self._port = port if port is not None else config.SERIAL_PORT
        self._bot = None

    def connect(self) -> None:
        try:
            from Rosmaster_Lib import Rosmaster
        except ImportError as exc:
            raise RuntimeError(
                "Rosmaster_Lib ist nicht installiert. Auf dem Entwicklungsrechner "
                "stattdessen mit --mock starten. Auf dem Roboter: siehe "
                "docs/hardware.md."
            ) from exc

        # Rosmaster() ohne Argument sucht die Schnittstelle selbst.
        self._bot = Rosmaster(com=self._port) if self._port else Rosmaster()

        # Startet den Hintergrund-Thread, der Rückmeldungen (IMU, Akku,
        # Encoder) vom Board einliest. Ohne diesen Aufruf bleiben alle
        # Lesefunktionen der Bibliothek leer.
        self._bot.create_receive_threading()

        log.info("ROSMASTER verbunden (Port: %s)", self._port or "automatisch")

    def disconnect(self) -> None:
        if self._bot is None:
            return
        self.stop()
        # Rosmaster_Lib hat kein close(); der Serial-Port wird beim Beenden
        # des Prozesses freigegeben. Wichtig ist nur, dass der Roboter steht.
        self._bot = None
        log.info("ROSMASTER getrennt")

    def drive(self, cmd: DriveCommand) -> None:
        if self._bot is None:
            raise RuntimeError("drive() vor connect() aufgerufen")
        # Mecanum: set_car_motion(vx, vy, vz) erwartet m/s, m/s, rad/s.
        self._bot.set_car_motion(cmd.vx, cmd.vy, cmd.omega)

    def stop(self) -> None:
        # Darf nie werfen -- stop() ist der Notausgang und wird auch aus
        # Fehlerpfaden und beim Herunterfahren aufgerufen.
        if self._bot is None:
            return
        try:
            self._bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            log.exception("Anhalten fehlgeschlagen -- Hardware prüfen!")

    def beep(self, ms: int = 100) -> None:
        if self._bot is None:
            return
        try:
            self._bot.set_beep(ms)
        except Exception:
            log.exception("Buzzer fehlgeschlagen")

    @property
    def connected(self) -> bool:
        return self._bot is not None
