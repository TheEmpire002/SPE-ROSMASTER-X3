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

4. **Autonome Grenze.** Befehle der autonomen Modi (`quelle="autonom"`)
   werden zusätzlich auf `AUTONOM_MAX_*` gekappt, bei schwachem Akku noch
   einmal reduziert. So kann kein Rechenfehler in Folgen, Erkunden oder
   Heimfahrt den Roboter schneller machen, als der Lidar Hindernisse sieht.

5. **Fahrschutz autonom.** Danach prüft `fahrschutz` (`fahrschutz.py`,
   vom Server gesetzt), ob die Grundfläche des Roboters bei diesem Befehl
   etwas berühren würde -- auch im Bogen und beim Drehen, wenn die Ecken
   ausschwenken -- und bremst stufenlos so, dass er mit Reaktionszeit und
   Bremsweg vorher steht. Wegfahren bleibt erlaubt. Ohne frische
   Lidar-Daten fährt kein autonomer Modus.
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
        # Akkuspannung wird nur gelegentlich nachgesehen -- sie ändert sich
        # langsam, und submit() läuft im 20-Hz-Takt.
        self._akku_v: float | None = None
        self._akku_at = -1e9
        # Kollisionsprüfung über die Grundfläche für autonome Befehle
        # (Punkt 5, safety/fahrschutz.py). Bleibt None, wenn kein Lidar
        # konfiguriert ist -- dann gibt es auch keine autonomen Modi, die
        # fahren könnten. letzte_pruefung: Ergebnis des letzten autonomen
        # Befehls, damit die Modi sehen, ob und warum gebremst wurde.
        self.fahrschutz = None
        self.letzte_pruefung = None
        # Fahrtenschreiber (aufzeichnung.py), setzt der Server. Hält jeden
        # Befehl vor und nach den Grenzen fest -- so sieht man später, wo
        # der Fahrschutz eingegriffen hat.
        self.aufzeichnung = None

    # --- Steuerung ------------------------------------------------------------

    def submit(self, cmd: DriveCommand, quelle: str = "hand") -> DriveCommand:
        """Fahrbefehl ausführen. Gibt den tatsächlich gefahrenen Befehl zurück.

        `quelle` ist "hand" (App, ESP32) oder "autonom" (Folgen, Erkunden,
        Kartierfahrt, Heimfahrt, genaue Fahrbefehle). Für autonome Befehle
        gilt die engere Grenze, und der Fahrschutz prüft sie. Die
        Handsteuerung prüft vorher über den ``CollisionGuard`` (abschaltbar
        in der App).
        """
        jetzt = time.monotonic()
        grenze_xy_x, grenze_xy_y = config.MAX_SPEED_X, config.MAX_SPEED_Y
        grenze_w = config.MAX_SPEED_OMEGA
        if quelle == "autonom":
            autonom = config.AUTONOM_MAX_SPEED * self._akku_faktor(jetzt)
            grenze_xy_x = min(grenze_xy_x, autonom)
            grenze_xy_y = min(grenze_xy_y, autonom)
            grenze_w = min(grenze_w, config.AUTONOM_MAX_OMEGA)
        safe = DriveCommand(
            vx=_clamp(cmd.vx, grenze_xy_x),
            vy=_clamp(cmd.vy, grenze_xy_y),
            omega=_clamp(cmd.omega, grenze_w),
        )
        if quelle == "autonom":
            # Ohne Prüfung None, sonst sähen die Modi ein veraltetes Ergebnis.
            self.letzte_pruefung = None
            if self.fahrschutz is not None and config.AUTONOM_SCHUTZ_AKTIV:
                self.letzte_pruefung = self.fahrschutz.pruefen(safe)
                safe = self.letzte_pruefung.befehl
        if safe != cmd:
            log.debug("Fahrbefehl begrenzt (%s): %s -> %s", quelle, cmd, safe)
        if self.aufzeichnung is not None:
            try:
                self.aufzeichnung.befehl(quelle, cmd, safe, jetzt)
            except Exception:
                # Die Aufzeichnung ist Beiwerk -- fahren geht vor.
                log.debug("Aufzeichnung des Fahrbefehls fehlgeschlagen", exc_info=True)

        self._ziel = safe
        self._last_command_at = jetzt
        return self._ausgeben(jetzt)

    def _akku_faktor(self, jetzt: float) -> float:
        """Tempofaktor für autonome Modi je nach Akkuspannung."""
        if jetzt - self._akku_at > 2.0:
            self._akku_at = jetzt
            try:
                self._akku_v = self._driver.read_battery()
            except Exception:
                # Ohne Messwert nicht drosseln -- die Diagnose meldet den
                # Ausfall ohnehin, und ein Treiberfehler darf hier nicht
                # den Fahrbefehl verhindern.
                self._akku_v = None
        if self._akku_v is not None and self._akku_v < config.AKKU_WARN_V:
            return config.AUTONOM_AKKU_FAKTOR
        return 1.0

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
        if self.aufzeichnung is not None:
            try:
                self.aufzeichnung.stop()
            except Exception:
                log.debug("Aufzeichnung des Halts fehlgeschlagen", exc_info=True)

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
