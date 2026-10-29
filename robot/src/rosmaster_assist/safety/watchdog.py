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

3. **Rampe.** Befehle springen nicht direkt auf die Motoren, sondern werden
   mit begrenzter Beschleunigung angefahren (`RAMPE_*` in `config`). Harte
   Sprünge zwischen Fahren und Halten erzeugen Stromspitzen, die am X3 den
   USB-Hub zurücksetzen. `stop()` umgeht die Rampe und steht sofort.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .. import config
from ..drive.base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _rampe(ist: float, soll: float, anfahren: float, bremsen: float, dt: float) -> float:
    """Einen Rampenschritt von `ist` Richtung `soll` gehen.

    Als Bremsen zählt alles, was den Betrag verkleinert -- auch ein
    Richtungswechsel, der erst einmal über null führt. Dort endet der Schritt
    bei null; in die neue Richtung geht es erst im nächsten Schritt mit der
    Anfahrgrenze weiter.
    """
    if ist * soll < 0:
        schritt = bremsen * dt
        if abs(ist) <= schritt:
            return 0.0
        return ist - math.copysign(schritt, ist)
    grenze = bremsen if abs(soll) < abs(ist) else anfahren
    schritt = grenze * dt
    if abs(soll - ist) <= schritt:
        return soll
    return ist + math.copysign(schritt, soll - ist)


class SafetyWatchdog:
    """Nimmt Fahrbefehle entgegen und reicht sie begrenzt an den Antrieb weiter."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._last_command = STOP     # was gerade gefahren wird (nach Rampe)
        self._ziel = STOP             # was zuletzt verlangt wurde
        self._last_command_at = 0.0   # wann zuletzt etwas verlangt wurde
        self._ausgabe_at = 0.0        # wann zuletzt an den Antrieb gegeben
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

        jetzt = time.monotonic()
        self._ziel = safe
        self._last_command_at = jetzt
        return self._ausgeben(jetzt)

    def _ausgeben(self, jetzt: float) -> DriveCommand:
        """Einen Rampenschritt Richtung Ziel an den Antrieb geben."""
        ziel = self._ziel
        if config.RAMPE_AKTIV:
            dt = min(max(jetzt - self._ausgabe_at, 0.0), config.RAMPE_DT_MAX_S)
            ist = self._last_command
            neu = DriveCommand(
                vx=_rampe(ist.vx, ziel.vx, config.RAMPE_ANFAHREN_XY,
                          config.RAMPE_BREMSEN_XY, dt),
                vy=_rampe(ist.vy, ziel.vy, config.RAMPE_ANFAHREN_XY,
                          config.RAMPE_BREMSEN_XY, dt),
                omega=_rampe(ist.omega, ziel.omega, config.RAMPE_ANFAHREN_OMEGA,
                             config.RAMPE_BREMSEN_OMEGA, dt),
            )
        else:
            neu = ziel
        self._ausgabe_at = jetzt
        self._last_command = neu
        # Erst "steht", wenn auch die Rampe unten angekommen ist -- bis dahin
        # muss die Überwachung weiter Schritte nachliefern.
        self._stopped = neu.is_stop() and ziel.is_stop()

        try:
            self._driver.drive(neu)
        except Exception:
            # Im Fehlerfall gilt: lieber stehen bleiben als weiterfahren.
            log.exception("Antrieb meldet Fehler -- halte an")
            self.stop()
            raise

        return neu

    def stop(self) -> None:
        """Sofort anhalten -- ohne Rampe."""
        self._driver.stop()
        self._last_command = STOP
        self._ziel = STOP
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
                self._rampe_nachfuehren()
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

    def _rampe_nachfuehren(self) -> None:
        """Rampe auch zwischen zwei Befehlen weiterführen.

        Die App schickt nur zehn Befehle pro Sekunde, und wer einmal "halt"
        sagt, schickt danach oft gar nichts mehr. Ohne diesen Schritt bliebe
        die Ausgabe dann auf halbem Weg stehen.
        """
        if self._stopped or self._last_command == self._ziel:
            return
        try:
            self._ausgeben(time.monotonic())
        except Exception:
            # _ausgeben hat schon angehalten und geloggt. Die Überwachung
            # selbst darf daran nicht sterben.
            pass

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
