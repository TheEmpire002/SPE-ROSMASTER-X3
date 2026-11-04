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

**Mit Karte (04.11.2026, E12):** Ist eine Karte geladen und die Lage darin
sicher (``kartenortung.py``), fährt er nicht mehr geradewegs, sondern einen
geplanten Weg (``pfadplanung.py``) -- durch Türen, um Ecken, in andere
Räume. Zu Hause ist dann der Ausgangspunkt der Karte. Dieselbe Klasse
fährt auch zu jedem anderen Punkt auf der Karte (``start(ziel=...)``).
Sie bleibt eine Klasse, damit jeder bestehende Abbruch (Not-Stopp, Hand,
Verbindung weg, andere Fahrt startet) auch die Fahrt zum Ziel beendet.
Wird der Weg versperrt, gilt die Stelle eine Weile als Wand, und er plant
drumherum neu.
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
from . import pfadfolger as pf
from .odometry import Odometry
from .pfadplanung import HintergrundPlaner

if TYPE_CHECKING:  # nur für die Typangabe -- vermeidet einen Ringimport
    from .kartenortung import Kartenortung
    from .localizer import Localizer

log = logging.getLogger(__name__)


class Homing:
    """Steuert den Roboter zurück zum Startpunkt -- oder mit Karte zu jedem Ziel."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        odometry: Odometry,
        localizer: "Localizer | None" = None,
        ortung: "Kartenortung | None" = None,
        planer: HintergrundPlaner | None = None,
    ) -> None:
        self._watchdog = watchdog
        self._odometry = odometry
        self._localizer = localizer
        self._ortung = ortung
        self._planer = planer if planer is not None else (
            HintergrundPlaner() if ortung is not None else None
        )
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None
        # Fahrt nach Karte: Ziel (None = nach Hause), Weg, Folger.
        self._ziel: tuple[float, float] | None = None
        self._nach_karte = False
        self._folger = pf.Pfadfolger()
        self._weg: list[tuple[float, float]] = []
        self._sperren: list[tuple[float, float, float]] = []  # x, y, gültig bis
        # Ausweichen (F3): Kurzgedächtnis, Hochheben, zuletzt gewählte Seite
        # (gegen Pendeln vor breiten Hindernissen).
        self._gedaechtnis = ausweichen.Kurzgedaechtnis(odometry)
        self._hoch = ausweichen.HochhebeErkennung()
        self._seite = 0.0
        self._blockiert_seit: float | None = None

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        """Warum die letzte Heimfahrt endete -- für die Anzeige in der App."""
        return self._grund

    @property
    def bezeichnung(self) -> str:
        """Für Meldungen: "Heimfahrt" oder "Fahrt zum Ziel"."""
        return "Heimfahrt" if self._ziel is None else "Fahrt zum Ziel"

    @property
    def nach_karte(self) -> bool:
        """Läuft gerade eine Fahrt nach Karte?"""
        return self.aktiv and self._nach_karte

    @property
    def karte_bereit(self) -> bool:
        """Lage in einer Karte bekannt -- dann geht es nach Karte."""
        return self._ortung is not None and self._planer is not None and self._ortung.sicher

    def start(self, ziel: tuple[float, float] | None = None, winkel: float | None = None) -> bool:
        """Heimfahrt bzw. Fahrt zu ``ziel`` (Kartenrahmen, m) beginnen.

        ``winkel`` (Bogenmaß): am Ziel noch so ausrichten. False, wenn schon
        eine Fahrt läuft oder ein Ziel ohne sichere Lage in der Karte
        verlangt wird (Grund dann in ``letzter_grund``).
        """
        if self.aktiv:
            return False
        if ziel is not None and not self.karte_bereit:
            self._grund = "ohne sichere Lage in einer Karte geht keine Fahrt zum Ziel"
            return False
        self._grund = None
        self._ziel = None if ziel is None else (float(ziel[0]), float(ziel[1]))
        self._weg = []
        self._nach_karte = self.karte_bereit
        if self._nach_karte:
            if ziel is None:
                start = self._ortung.start
                ziel, winkel = (start[0], start[1]), start[2]
            self._task = asyncio.create_task(self._fahren_karte((ziel[0], ziel[1]), winkel))
        else:
            self._task = asyncio.create_task(self._fahren())
        return True

    def zustand(self) -> dict:
        """Für state.karte und die Seite /map."""
        aktiv = self.aktiv
        return {
            "aktiv": aktiv,
            "art": "heim" if self._ziel is None else "ziel",
            "nach_karte": self._nach_karte if aktiv else None,
            "ziel": list(self._ziel) if self._ziel is not None else None,
            "weg": [[round(p[0], 2), round(p[1], 2)] for p in self._weg] if aktiv else [],
            "rest_m": round(self._folger.rest_m, 2) if aktiv and self._nach_karte else None,
            "grund": None if aktiv else self._grund,
        }

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

        start_x, start_y, _ = self._odometry.pose()
        entfernung = math.hypot(start_x, start_y)
        log.info("Heimfahrt gestartet -- %.2f m zu fahren", entfernung)

        if not self._odometry.misst_tatsaechlich:
            log.warning(
                "Keine Bewegungsmessung verfügbar -- die Position stammt nur "
                "aus den Fahrbefehlen und ist entsprechend ungenau"
            )

        try:
            while True:
                x, y, winkel_grad = self._odometry.pose()
                abstand = math.hypot(x, y)

                if abstand <= config.HOME_TOLERANCE_M:
                    self._watchdog.stop()
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
                            self._grund = f"{ort}, noch {abstand:.2f} m bis zum Startpunkt"
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
                    self._grund = (
                        f"Hindernis in {blockiert:.2f} m, noch {abstand:.2f} m "
                        "bis zum Startpunkt"
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

    # --- Mit Karte -------------------------------------------------------------------

    async def _fahren_karte(self, ziel: tuple[float, float], winkel: float | None) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        self._gedaechtnis.leeren()
        self._hoch.zuruecksetzen()
        neuplanungen = 0
        unsicher_seit: float | None = None
        blockiert_seit: float | None = None
        name = self.bezeichnung
        try:
            weg = await self._planen(ziel)
            if weg is None:
                self._beenden("kein Weg durch die bekannte Karte", warnung=True)
                return
            log.info("%s nach Karte -- %.1f m Weg, %d Wegpunkte", name, weg.laenge, len(weg.punkte))
            frist = time.monotonic() + max(config.KARTE_FAHRT_MIN_S,
                                           weg.laenge * config.KARTE_FAHRT_S_JE_M)
            self._weg_setzen(weg, winkel)

            while True:
                jetzt = time.monotonic()
                if jetzt > frist:
                    self._beenden(f"Zeit abgelaufen, noch {self._folger.rest_m:.2f} m", warnung=True)
                    return

                # Ohne sichere Lage nicht weiterfahren -- aber kurz abwarten:
                # Ein Mensch, der vor dem Lidar steht, verdeckt nur kurz.
                pose = self._ortung.pose()
                if pose is None or not self._ortung.sicher:
                    unsicher_seit = unsicher_seit or jetzt
                    if jetzt - unsicher_seit > config.KARTE_SICHER_S:
                        self._beenden("Lage in der Karte verloren", warnung=True)
                        return
                    self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), quelle="autonom")
                    await asyncio.sleep(takt)
                    continue
                unsicher_seit = None

                self._gedaechtnis.fortschreiben(takt)
                scan = self._localizer.scan() if self._localizer is not None else None
                if scan is None:
                    # Blind fährt er nicht; der Lidar meldet sich meist gleich wieder.
                    self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), quelle="autonom")
                    await asyncio.sleep(takt)
                    continue
                lage = self._gedaechtnis.lage if self._gedaechtnis.misst else None
                if self._hoch.pruefen(scan, lage):
                    self._beenden("hochgehoben -- Lage wird neu gesucht", warnung=True)
                    return

                punkte = self._gedaechtnis.punkte(scan)
                befehl, zustand = self._folger.schritt(pose, punkte)
                if zustand == pf.ANGEKOMMEN:
                    rest = math.hypot(ziel[0] - pose[0], ziel[1] - pose[1])
                    self._beenden(f"angekommen ({rest:.2f} m Restabweichung)")
                    return

                # Steht er, obwohl er fahren will (Weg versperrt oder der
                # Fahrschutz lässt nichts durch), plant er nach einer Weile
                # drumherum.
                pruefung = self._watchdog.letzte_pruefung
                gesperrt = (
                    zustand == pf.BLOCKIERT
                    or (pruefung is not None and pruefung.anteil == 0.0 and not befehl.is_stop())
                )
                if zustand == pf.ABGEKOMMEN or gesperrt:
                    blockiert_seit = blockiert_seit or jetzt
                    if zustand == pf.ABGEKOMMEN or jetzt - blockiert_seit > config.KARTE_BLOCKIERT_S:
                        neuplanungen += 1
                        if neuplanungen > config.KARTE_MAX_NEUPLANUNG:
                            self._beenden(
                                f"Weg bleibt versperrt, noch {self._folger.rest_m:.2f} m",
                                warnung=True,
                            )
                            return
                        if gesperrt:
                            self._sperre_setzen(pose, punkte)
                        self._watchdog.stop()
                        weg = await self._planen(ziel)
                        if weg is None:
                            self._beenden("kein Weg mehr frei", warnung=True)
                            return
                        self._weg_setzen(weg, winkel)
                        blockiert_seit = None
                        continue
                    if zustand == pf.BLOCKIERT:
                        befehl = DriveCommand(0.0, 0.0, 0.0)
                else:
                    blockiert_seit = None

                self._watchdog.submit(befehl, quelle="autonom")
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            log.exception("%s abgebrochen", name)

    def _beenden(self, grund: str, warnung: bool = False) -> None:
        self._watchdog.stop()
        self._grund = grund
        (log.warning if warnung else log.info)("%s beendet -- %s", self.bezeichnung, grund)

    def _weg_setzen(self, weg, winkel: float | None) -> None:
        self._weg = list(weg.punkte)
        self._folger.setzen(weg.punkte, von=self._ortung.pose(), ziel_winkel=winkel)

    async def _planen(self, ziel):
        pose = self._ortung.pose()
        karte = self._ortung.karte
        if pose is None or karte is None:
            return None
        jetzt = time.monotonic()
        self._sperren = [s for s in self._sperren if s[2] > jetzt]
        zukunft = self._planer.weg(karte, (pose[0], pose[1]), ziel,
                                   [(s[0], s[1]) for s in self._sperren])
        return await asyncio.wrap_future(zukunft)

    def _sperre_setzen(self, pose, punkte) -> None:
        """Die Stelle, die den Weg versperrt, eine Weile als Wand eintragen.

        Das nächste Hindernis in Richtung des Weges, sonst einfach ein
        Stück voraus. In Kartenkoordinaten, damit es beim Neuplanen zählt.
        """
        import numpy as np

        x, y, th = pose
        ziel = self._folger.punkte[-1] if self._folger.punkte else (x, y)
        for p in self._folger.punkte:
            if math.hypot(p[0] - x, p[1] - y) > 0.3:
                ziel = p
                break
        richtung = math.atan2(ziel[1] - y, ziel[0] - x) - th
        stelle = (0.4 * math.cos(richtung), 0.4 * math.sin(richtung))
        pkt = np.asarray(punkte, dtype=float).reshape(-1, 2)
        if len(pkt):
            winkel = np.arctan2(pkt[:, 1], pkt[:, 0]) - richtung
            winkel = np.arctan2(np.sin(winkel), np.cos(winkel))
            r = np.hypot(pkt[:, 0], pkt[:, 1])
            vorn = (np.abs(winkel) < math.radians(40.0)) & (r < 1.0)
            if vorn.any():
                i = int(np.argmin(np.where(vorn, r, np.inf)))
                stelle = (float(pkt[i, 0]), float(pkt[i, 1]))
        c, s = math.cos(th), math.sin(th)
        wx, wy = x + c * stelle[0] - s * stelle[1], y + s * stelle[0] + c * stelle[1]
        self._sperren.append((wx, wy, time.monotonic() + config.KARTE_SPERRE_S))
        log.info("Weg versperrt bei (%.2f, %.2f) -- plane drumherum", wx, wy)

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
        """Fahrbefehl für diesen Moment."""
        # Richtung zum Startpunkt, in Weltkoordinaten.
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
