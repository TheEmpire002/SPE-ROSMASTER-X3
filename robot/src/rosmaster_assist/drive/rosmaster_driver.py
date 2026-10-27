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
        self._last: DriveCommand | None = None

    def _warnen_bei_mehreren_ports(self) -> None:
        """Meldet, wenn mehr als ein USB-Seriell-Gerät angeschlossen ist.

        Am ROSMASTER hängen typischerweise zwei: das Expansion Board (CH340)
        und der RPLIDAR (CP210x). Sucht sich Rosmaster_Lib die Schnittstelle
        selbst, kann es den Lidar erwischen -- und dann bewegt sich nichts,
        ohne dass irgendwo ein Fehler erscheint.
        """
        import glob

        geraete = sorted(glob.glob("/dev/serial/by-id/*"))
        if len(geraete) <= 1:
            return

        log.warning(
            "%d serielle Geräte gefunden, SERIAL_PORT ist nicht gesetzt -- "
            "Rosmaster_Lib sucht sich eines aus:",
            len(geraete),
        )
        for g in geraete:
            log.warning("    %s", g)
        log.warning("Bei Problemen das richtige in config.py bei SERIAL_PORT eintragen.")

    def connect(self) -> None:
        if self._port is None:
            self._warnen_bei_mehreren_ports()

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
        # Lesefunktionen der Bibliothek leer -- Fahren geht trotzdem.
        if config.TELEMETRY_ENABLED:
            self._bot.create_receive_threading()
        else:
            log.warning(
                "Rückkanal abgeschaltet (TELEMETRY_ENABLED=False) -- "
                "keine Akku- und IMU-Werte"
            )

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

        # Nur bei Änderung loggen, sonst 10 Zeilen pro Sekunde. Dass hier
        # überhaupt geloggt wird, ist Absicht: sonst sieht ein stiller
        # Roboter im echten Betrieb genauso aus wie einer in der
        # Simulation, und man sucht den Fehler an der falschen Stelle.
        if cmd != self._last:
            log.info("FAHRE  vx=%+.2f  vy=%+.2f  omega=%+.2f", cmd.vx, cmd.vy, cmd.omega)
            self._last = cmd

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

    def read_motion(self) -> tuple[float, float, float] | None:
        # Braucht den Empfangs-Thread; ohne ihn liefert die Bibliothek Nullen,
        # was schlimmer wäre als keine Messung (die Position bliebe stehen).
        if self._bot is None or not config.TELEMETRY_ENABLED:
            return None
        try:
            vx, vy, omega = self._bot.get_motion_data()
            return float(vx), float(vy), float(omega)
        except Exception:
            return None

    def read_heading(self) -> float | None:
        if self._bot is None or not config.TELEMETRY_ENABLED:
            return None
        try:
            _roll, _pitch, yaw = self._bot.get_imu_attitude_data()
            return float(yaw)
        except Exception:
            return None

    @property
    def connected(self) -> bool:
        return self._bot is not None
