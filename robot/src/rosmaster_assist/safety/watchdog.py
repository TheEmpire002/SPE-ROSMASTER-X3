"""Sicherheitsschicht zwischen Server und Antrieb.

Zwei Aufgaben:

1. **Begrenzen.** Jeder Fahrbefehl wird auf die Grenzwerte aus `config`
   gekappt, bevor er die Motoren erreicht. Ein Programmierfehler in der App
   kann den Roboter damit nicht schneller machen als erlaubt.

2. **Totmannschaltung.** Bleiben die Fahrbefehle aus -- WLAN weg, Browser-Tab
   geschlossen, App abgestürzt -- hält der Roboter nach `COMMAND_TIMEOUT_S`
   von selbst an.

Punkt 2 ist der Grund, warum diese Schicht existiert. Sie darf nicht etwas
sein, woran die App denken muss: ein fahrender Roboter ohne Verbindung ist
genau die Situation, in der oben nichts mehr funktioniert.
"""

from __future__ import annotations

import asyncio
import logging
import time

from .. import config
from ..drive.base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class SafetyWatchdog:
    """Nimmt Fahrbefehle entgegen und reicht sie begrenzt an den Antrieb weiter."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._last_command = STOP
        self._last_command_at = 0.0
        self._stopped = True
        self._task: asyncio.Task[None] | None = None

    # --- Steuerung ------------------------------------------------------------

    def submit(self, cmd: DriveCommand) -> DriveCommand:
        """Fahrbefehl ausführen. Gibt den tatsächlich gefahrenen Befehl zurück."""
        safe = DriveCommand(
            vx=_clamp(cmd.vx, config.MAX_SPEED_X),
            vy=_clamp(cmd.vy, config.MAX_SPEED_Y),
            omega=_clamp(cmd.omega, config.MAX_SPEED_OMEGA),
        )
        if safe != cmd:
            log.debug("Fahrbefehl begrenzt: %s -> %s", cmd, safe)

        self._last_command = safe
        self._last_command_at = time.monotonic()
        self._stopped = safe.is_stop()

        try:
            self._driver.drive(safe)
        except Exception:
            # Im Fehlerfall gilt: lieber stehen bleiben als weiterfahren.
            log.exception("Antrieb meldet Fehler -- halte an")
            self.stop()
            raise

        return safe

    def stop(self) -> None:
        """Sofort anhalten."""
        self._driver.stop()
        self._last_command = STOP
        self._stopped = True

    @property
    def current_command(self) -> DriveCommand:
        """Was der Roboter gerade tut (nach Begrenzung)."""
        return self._last_command

    # --- Hintergrundüberwachung -----------------------------------------------

    async def run(self) -> None:
        """Überwachungsschleife. Läuft, solange der Server läuft."""
        log.info("Watchdog aktiv (Timeout %.1fs)", config.COMMAND_TIMEOUT_S)
        try:
            while True:
                await asyncio.sleep(config.WATCHDOG_INTERVAL_S)
                self._check_timeout()
        except asyncio.CancelledError:
            # Beim Herunterfahren: sicherstellen, dass der Roboter steht.
            self.stop()
            raise

    def _check_timeout(self) -> None:
        if self._stopped:
            return  # steht bereits
        age = time.monotonic() - self._last_command_at
        if age > config.COMMAND_TIMEOUT_S:
            log.warning("Seit %.2fs kein Fahrbefehl -- Not-Stopp", age)
            self.stop()

    # --- Lebenszyklus ----------------------------------------------------------

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self.run())

    async def shutdown(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self.stop()
