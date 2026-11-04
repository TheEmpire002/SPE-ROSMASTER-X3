"""Automatische Rückfahrt zum Startpunkt.

Nutzt aus, dass der X3 Mecanum-Räder hat: er kann in jede Richtung fahren,
ohne sich vorher zu drehen. Die Heimfahrt ist damit denkbar einfach --
Richtung zum Startpunkt ausrechnen, in diese Richtung fahren, fertig. Das
Drehen ist bei der Koppelnavigation die größte Fehlerquelle, und genau das
entfällt hier.

**Ausweichen (03.11.2026, F3):** Steht etwas auf der Verbindungslinie,
schiebt er schräg daran vorbei, bis der Weg wieder frei ist
(``ausweichen.richtung_waehlen``). Ohne Karte, also um einzelne Hindernisse
herum, nicht durch Labyrinthe. Ist ``HOME_AUSWEICH_WARTEN_S`` lang nirgends
Platz, hält er an und meldet es. Wird er hochgehoben, bricht er ab: Die
Koppelnavigation weiß vom Tragen nichts, die Richtung nach Hause stimmt dann
nicht mehr. Deshalb weiter das gemächliche Tempo und die Zeitbegrenzung.

**Beliebiges Ziel (04.11.2026, Zustandserkennung):** ``start_zu`` fährt
dieselbe Strecke zu einem anderen Punkt im Odometrie-Rahmen, etwa zu einer
gemeldeten Person. Weil dort die Person selbst das Hindernis ist, kann
"blockiert, aber schon nah genug" als angekommen gelten.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from typing import TYPE_CHECKING

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import ausweichen
from .odometry import Odometry

if TYPE_CHECKING:  # nur für die Typangabe -- vermeidet einen Ringimport
    from .localizer import Localizer

log = logging.getLogger(__name__)


class Homing:
    """Steuert den Roboter zurück zum Startpunkt."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        odometry: Odometry,
        localizer: "Localizer | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._odometry = odometry
        self._localizer = localizer
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None
        # Ausweichen (F3): Kurzgedächtnis, Hochheben, zuletzt gewählte Seite
        # (gegen Pendeln vor breiten Hindernissen).
        self._gedaechtnis = ausweichen.Kurzgedaechtnis(odometry)
        self._hoch = ausweichen.HochhebeErkennung()
        self._seite = 0.0
        self._blockiert_seit: float | None = None
        # Ziel im Odometrie-Rahmen; ohne start_zu der Startpunkt (0, 0).
        self._ziel = (0.0, 0.0)
        self._zielname = "Startpunkt"
        self._toleranz: float | None = None
        self._blockiert_reicht_m = 0.0
        self._angekommen = False

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        """Warum die letzte Heimfahrt endete -- für die Anzeige in der App."""
        return self._grund

    @property
    def angekommen(self) -> bool:
        """Ob die letzte Fahrt das Ziel erreicht hat (nicht abgebrochen)."""
        return self._angekommen

    @property
    def ziel(self) -> tuple[float, float]:
        return self._ziel

    def start(self) -> bool:
        """Heimfahrt beginnen. False, wenn schon eine läuft."""
        return self.start_zu(0.0, 0.0)

    def start_zu(
        self,
        x: float,
        y: float,
        toleranz: float | None = None,
        name: str = "Startpunkt",
        blockiert_reicht_m: float = 0.0,
    ) -> bool:
        """Zu einem Punkt im Odometrie-Rahmen fahren. False, wenn schon eine Fahrt läuft.

        ``toleranz`` ersetzt ``HOME_TOLERANCE_M``. Ist ``blockiert_reicht_m``
        größer null, gilt die Fahrt als angekommen, wenn ihr in dieser
        Entfernung zum Ziel etwas im Weg steht -- gedacht für Ziele, die
        selbst ein Hindernis sind (eine Person am Boden).
        """
        if self.aktiv:
            return False
        self._ziel = (float(x), float(y))
        self._zielname = name
        self._toleranz = toleranz
        self._blockiert_reicht_m = max(0.0, float(blockiert_reicht_m))
        self._angekommen = False
        self._grund = None
        self._task = asyncio.create_task(self._fahren())
        return True

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        """Sofort beenden. Auch aufzurufen, wenn der Mensch eingreift."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._grund = grund
            log.info("Heimfahrt %s", grund)
        self._task = None

    async def _fahren(self) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        frist = time.monotonic() + config.HOME_TIMEOUT_S
        self._gedaechtnis.leeren()
        self._hoch.zuruecksetzen()
        self._seite = 0.0
        self._blockiert_seit = None
        ziel_x, ziel_y = self._ziel
        toleranz = config.HOME_TOLERANCE_M if self._toleranz is None else self._toleranz
        name = self._zielname

        start_x, start_y, _ = self._odometry.pose()
        entfernung = math.hypot(start_x - ziel_x, start_y - ziel_y)
        log.info("Fahrt zum %s gestartet -- %.2f m zu fahren", name, entfernung)

        if not self._odometry.misst_tatsaechlich:
            log.warning(
                "Keine Bewegungsmessung verfügbar -- die Position stammt nur "
                "aus den Fahrbefehlen und ist entsprechend ungenau"
            )

        try:
            while True:
                x, y, winkel_grad = self._odometry.pose()
                # Ab hier relativ zum Ziel: _schritt fährt immer nach (0, 0).
                x, y = x - ziel_x, y - ziel_y
                abstand = math.hypot(x, y)

                if abstand <= toleranz:
                    self._watchdog.stop()
                    self._angekommen = True
                    self._grund = f"angekommen ({abstand:.2f} m Restabweichung)"
                    log.info("Heimfahrt beendet -- %s", self._grund)
                    return

                if time.monotonic() > frist:
                    self._watchdog.stop()
                    self._grund = f"Zeit abgelaufen, noch {abstand:.2f} m entfernt"
                    log.warning("Heimfahrt abgebrochen -- %s", self._grund)
                    return

                befehl = self._schritt(x, y, winkel_grad, abstand)
                self._gedaechtnis.fortschreiben(takt)
                scan = self._localizer.scan() if self._localizer is not None else None

                if scan is not None:
                    lage = self._gedaechtnis.lage if self._gedaechtnis.misst else None
                    if self._hoch.pruefen(scan, lage):
                        self._watchdog.stop()
                        self._grund = "hochgehoben -- Position stimmt nicht mehr"
                        log.warning("Heimfahrt abgebrochen -- %s", self._grund)
                        return
                    frei = self._umfahren(befehl, abstand, scan)
                    if frei is None and abstand <= self._blockiert_reicht_m:
                        # Das Ziel selbst steht im Weg (Person am Boden):
                        # näher geht es nicht, also ist er da.
                        self._watchdog.stop()
                        self._angekommen = True
                        self._grund = f"angekommen, Weg versperrt ({abstand:.2f} m vor dem Ziel)"
                        log.info("Fahrt beendet -- %s", self._grund)
                        return
                    if frei is None:
                        # Kurz warten: Ein vorbeigehender Mensch soll die
                        # Heimfahrt nicht beenden.
                        jetzt = time.monotonic()
                        if self._blockiert_seit is None:
                            self._blockiert_seit = jetzt
                        if jetzt - self._blockiert_seit > config.HOME_AUSWEICH_WARTEN_S:
                            self._watchdog.stop()
                            blockiert = self._hindernis(befehl)
                            ort = f"Hindernis in {blockiert:.2f} m" if blockiert is not None else "kein Weg frei"
                            self._grund = f"{ort}, noch {abstand:.2f} m bis zum {name}"
                            log.warning("Heimfahrt angehalten -- %s", self._grund)
                            return
                        self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), quelle="autonom")
                        await asyncio.sleep(takt)
                        continue
                    self._blockiert_seit = None
                    self._watchdog.submit(frei, quelle="autonom")
                    await asyncio.sleep(takt)
                    continue

                # Ohne frische Runde: wie früher nur prüfen und anhalten.
                # Erst kurz vor dem Losfahren, nicht vorher: die
                # Fahrtrichtung ergibt sich aus dem Befehl.
                blockiert = self._hindernis(befehl)
                if blockiert is not None:
                    self._watchdog.stop()
                    if abstand <= self._blockiert_reicht_m:
                        self._angekommen = True
                        self._grund = f"angekommen, Weg versperrt ({abstand:.2f} m vor dem Ziel)"
                        return
                    self._grund = (
                        f"Hindernis in {blockiert:.2f} m, noch {abstand:.2f} m "
                        f"bis zum {name}"
                    )
                    log.warning("Heimfahrt angehalten -- %s", self._grund)
                    return

                self._watchdog.submit(befehl, quelle="autonom")
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            # Immer anhalten, egal warum abgebrochen wurde.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            log.exception("Heimfahrt abgebrochen")

    def _hindernis(self, befehl: DriveCommand) -> float | None:
        """Steht etwas im Weg? Gibt die Entfernung zurück, sonst None.

        Rückfall ohne frische Runde fürs Umfahren -- dann hält er an, statt
        auszuweichen -- und Entfernung für die Meldung, wenn kein Weg frei ist.
        """
        if self._localizer is None:
            return None
        if befehl.is_stop():
            return None

        # Bei Mecanum ist die Fahrtrichtung nicht "vorn": der Roboter kann
        # seitwärts fahren. Also in die Richtung schauen, in die es geht.
        richtung = math.degrees(math.atan2(befehl.vy, befehl.vx))
        abstand = self._localizer.hindernis_voraus(richtung)
        if abstand is None:
            return None
        return abstand if abstand < config.HOME_STOP_DISTANCE_M else None

    def _umfahren(self, befehl: DriveCommand, abstand: float, scan) -> DriveCommand | None:
        """Den Befehl in eine freie Richtung drehen; None, wenn keine frei ist.

        Gebraucht wird Platz bis knapp hinter den Startpunkt (Front plus
        Rand), höchstens ``UMFAHREN_VORAUS_M`` plus Halteabstand: Eine Wand
        kurz hinter dem Startpunkt soll ihn nicht vom Ziel abhalten. Ist
        dafür nirgends Platz, reicht der Halteabstand -- dann fährt er wie
        früher, bis es eng wird.
        """
        if befehl.is_stop():
            return befehl
        tempo = math.hypot(befehl.vx, befehl.vy)
        richtung = math.degrees(math.atan2(befehl.vy, befehl.vx))
        punkte = self._gedaechtnis.punkte(scan)
        weit = min(abstand + config.AUSWEICH_VORN_M + config.AUSWEICH_RAND_M,
                   config.UMFAHREN_VORAUS_M + config.HOME_STOP_DISTANCE_M)
        wahl = None
        for noetig in (weit, min(weit, config.HOME_STOP_DISTANCE_M)):
            wahl = ausweichen.richtung_waehlen(
                punkte, richtung, noetig, config.EXPLORE_HALF_WIDTH_M, self._seite
            )
            if wahl is not None:
                break
        if wahl is None:
            return None
        neu, _frei = wahl
        versatz = ausweichen._winkel_normieren(math.radians(neu - richtung))
        self._seite = 0.0 if abs(versatz) < 1e-6 else math.copysign(1.0, versatz)
        a = math.radians(neu)
        return DriveCommand(vx=tempo * math.cos(a), vy=tempo * math.sin(a), omega=0.0)

    def _schritt(self, x: float, y: float, winkel_grad: float, abstand: float) -> DriveCommand:
        """Fahrbefehl für diesen Moment (x, y relativ zum Ziel)."""
        # Richtung zum Ziel, in Weltkoordinaten.
        dx, dy = -x, -y

        # In Roboterkoordinaten drehen: der Antrieb kennt nur "vorwärts"
        # und "seitwärts", nicht "Norden".
        theta = math.radians(winkel_grad)
        vorwaerts = dx * math.cos(theta) + dy * math.sin(theta)
        seitwaerts = -dx * math.sin(theta) + dy * math.cos(theta)

        # Auf Länge 1 bringen und mit dem Tempo malnehmen. Kurz vor dem Ziel
        # abbremsen, sonst schießt er darüber hinaus und pendelt.
        tempo = config.HOME_SPEED
        if abstand < config.HOME_SLOWDOWN_M:
            anteil = abstand / config.HOME_SLOWDOWN_M
            # Nicht beliebig langsam werden -- sonst bleibt er vor dem Ziel
            # stehen, weil die Motoren bei sehr kleinen Werten nicht anlaufen.
            tempo *= max(0.35, anteil)

        laenge = math.hypot(vorwaerts, seitwaerts) or 1.0
        return DriveCommand(
            vx=vorwaerts / laenge * tempo,
            vy=seitwaerts / laenge * tempo,
            omega=0.0,  # Drehen ist unnötig: Mecanum fährt in jede Richtung.
        )
