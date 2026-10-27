"""Automatische Rückfahrt zum Startpunkt.

Die Heimfahrt läuft in Phasen ab:

1. **Rückweg.** Die Strecke der Hinfahrt rückwärts abfahren (siehe
   ``rueckweg.py``). Liegt ein Wegpunkt näher am Start in freier Sicht,
   wird direkt dorthin abgekürzt -- im offenen Raum ist das die Luftlinie,
   durch eine Tür hindurch der Weg, den er gekommen ist.
2. **Anfahrt.** Die letzten Meter direkt auf den Startpunkt zu, mit
   Abbremsen.
3. **Feinausrichtung.** Im Stand ein paar ruhige Lidar-Abgleiche abwarten,
   dann die Restabweichung im Schritttempo wegfahren. Während der Fahrt
   hinkt die Ortung immer etwas hinterher; erst im Stand ist sie genau.
4. **Blickrichtung** vom Startpunkt wiederherstellen.
5. **Marker** (wenn eingeschaltet): das letzte Stück per Kamera, bis der
   Marker wieder genauso im Bild liegt wie beim Setzen des Startpunkts.

Nutzt aus, dass der X3 Mecanum-Räder hat: er fährt in jede Richtung, ohne
sich vorher zu drehen. Das Drehen ist bei der Koppelnavigation die größte
Fehlerquelle, und unterwegs entfällt es dadurch ganz.

**Der Roboter weicht nicht aus.** Steht etwas im Weg, wartet er kurz und
gibt dann auf. Ausweichen ohne Karte endet leicht in einer Ecke.
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
from .odometry import Odometry

if TYPE_CHECKING:  # nur für die Typangabe -- vermeidet einen Ringimport
    from .localizer import Localizer
    from .marker import MarkerAndocken
    from .rueckweg import Rueckweg

log = logging.getLogger(__name__)


class _Abbruch(Exception):
    """Die Heimfahrt endet vorzeitig; der Grund steht in der Nachricht."""


class Homing:
    """Steuert den Roboter zurück zum Startpunkt."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        odometry: Odometry,
        localizer: "Localizer | None" = None,
        rueckweg: "Rueckweg | None" = None,
        marker: "MarkerAndocken | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._odometry = odometry
        self._localizer = localizer
        self._rueckweg = rueckweg
        self._marker = marker
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None
        self._phase: str | None = None
        self._frist = 0.0

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        """Warum die letzte Heimfahrt endete -- für die Anzeige in der App."""
        return self._grund

    @property
    def phase(self) -> str | None:
        """Was die Heimfahrt gerade tut, oder None, wenn sie nicht läuft."""
        return self._phase if self.aktiv else None

    def start(self) -> bool:
        """Heimfahrt beginnen. False, wenn schon eine läuft."""
        if self.aktiv:
            return False
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

    # --- Ablauf ----------------------------------------------------------------

    async def _fahren(self) -> None:
        x, y, _ = self._odometry.pose()
        weg = self._rueckweg.laenge_m() if self._rueckweg is not None else 0.0
        strecke = max(weg, math.hypot(x, y))
        # Die Frist wächst mit der Strecke: ein langer Rückweg ist kein
        # Zeichen, dass etwas schiefläuft.
        self._frist = time.monotonic() + max(
            config.HOME_TIMEOUT_S, strecke / config.HOME_SPEED * 2.5 + 20.0
        )
        log.info(
            "Heimfahrt gestartet -- %.2f m Luftlinie, %.2f m Rückweg", math.hypot(x, y), weg
        )
        if not self._odometry.misst_tatsaechlich:
            log.warning(
                "Keine Bewegungsmessung verfügbar -- die Position stammt nur "
                "aus den Fahrbefehlen und ist entsprechend ungenau"
            )

        try:
            self._phase = "Rückweg"
            await self._rueckweg_abfahren()

            self._phase = "Anfahrt"
            await self._fahre_zu(0.0, 0.0, config.HOME_TOLERANCE_M, config.HOME_SPEED, True)

            lidar = self._localizer is not None and self._localizer.hat_referenz
            if lidar:
                self._phase = "Feinausrichtung"
                await self._feinausrichten(config.HOME_FEIN_RUNDEN)
            if config.HOME_AUSRICHTEN:
                self._phase = "Ausrichten"
                await self._ausrichten()
                if lidar:
                    # Drehen verschiebt den Roboter ein wenig (Schlupf).
                    self._phase = "Feinausrichtung"
                    await self._feinausrichten(2)

            verfahren = "Lidar" if lidar else "Odometrie"
            if self._marker is not None and self._marker.aktiv:
                self._phase = "Marker"
                if await self._marker_andocken():
                    verfahren = "Marker"
                else:
                    verfahren += ", Marker nicht erreicht"

            self._watchdog.stop()
            if self._rueckweg is not None:
                self._rueckweg.zuruecksetzen()  # zu Hause: Weg ist abgefahren
            x, y, grad = self._odometry.pose()
            self._grund = (
                f"angekommen ({math.hypot(x, y):.2f} m, {abs(grad):.1f}° Abweichung, "
                f"{verfahren})"
            )
            log.info("Heimfahrt beendet -- %s", self._grund)

        except _Abbruch as abbruch:
            self._watchdog.stop()
            self._grund = str(abbruch)
            log.warning("Heimfahrt abgebrochen -- %s", self._grund)
        except asyncio.CancelledError:
            # Immer anhalten, egal warum abgebrochen wurde.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            log.exception("Heimfahrt abgebrochen")
        finally:
            self._phase = None

    # --- Phase 1: Rückweg ------------------------------------------------------

    async def _rueckweg_abfahren(self) -> None:
        if self._rueckweg is None:
            return
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        while True:
            punkte = self._rueckweg.punkte()
            if len(punkte) <= 1:
                return
            x, y, grad = self._odometry.pose()
            theta = math.radians(grad)

            ziel = self._ziel_waehlen(punkte, x, y, theta)
            if ziel == 0:
                return  # Startpunkt in Sicht: weiter mit der Anfahrt
            # Was hinter dem Ziel liegt, ist erledigt oder wird abgekürzt.
            self._rueckweg.kuerzen(ziel)

            zx, zy = punkte[ziel]
            abstand = math.hypot(zx - x, zy - y)
            if abstand <= config.RUECKWEG_ERREICHT_M:
                self._rueckweg.kuerzen(ziel - 1)
                continue

            self._frist_pruefen()
            befehl = _befehl_zu(x, y, theta, zx, zy, config.HOME_SPEED)
            await self._frei_oder_warten(befehl, abstand)
            self._watchdog.submit(befehl)
            await asyncio.sleep(takt)

    def _ziel_waehlen(self, punkte, x: float, y: float, theta: float) -> int:
        """Den Wegpunkt am nächsten zum Start, der direkt erreichbar ist.

        Ohne Lidar gibt es keine Sichtprüfung -- dann wird der Weg Punkt für
        Punkt abgefahren.
        """
        letzter = len(punkte) - 1
        if self._localizer is None:
            return letzter
        for i, (px, py) in enumerate(punkte[:letzter]):
            entfernung = math.hypot(px - x, py - y)
            if entfernung > config.RUECKWEG_ABKUERZUNG_M:
                continue
            richtung = _richtung_im_roboter(x, y, theta, px, py)
            if self._localizer.sicht_frei(richtung, entfernung):
                return i
        return letzter

    # --- Phase 2 und Feinausrichtung: auf einen Punkt zufahren -------------------

    async def _fahre_zu(
        self, zx: float, zy: float, toleranz: float, tempo: float, abbremsen: bool
    ) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        while True:
            x, y, grad = self._odometry.pose()
            abstand = math.hypot(zx - x, zy - y)
            if abstand <= toleranz:
                self._watchdog.stop()
                return
            self._frist_pruefen()

            v = tempo
            if abbremsen and abstand < config.HOME_SLOWDOWN_M:
                # Nicht beliebig langsam werden -- sonst bleibt er vor dem
                # Ziel stehen, weil die Motoren bei sehr kleinen Werten
                # nicht anlaufen.
                v *= max(0.35, abstand / config.HOME_SLOWDOWN_M)

            befehl = _befehl_zu(x, y, math.radians(grad), zx, zy, v)
            await self._frei_oder_warten(befehl, abstand)
            self._watchdog.submit(befehl)
            await asyncio.sleep(takt)

    async def _feinausrichten(self, runden: int) -> None:
        """Im Stand messen lassen, dann die Restabweichung wegfahren."""
        for _ in range(runden):
            self._watchdog.stop()
            # Rund vier Abgleiche im Stand: unverzerrte Aufnahmen, kein
            # Verzug -- genauer wird die Position nicht.
            await asyncio.sleep(1.0)
            x, y, _ = self._odometry.pose()
            if math.hypot(x, y) <= config.HOME_FEIN_TOLERANZ_M:
                return
            await self._fahre_zu(
                0.0, 0.0, config.HOME_FEIN_TOLERANZ_M * 0.6, config.HOME_FEIN_TEMPO, False
            )

    # --- Phase 4: Blickrichtung ----------------------------------------------------

    async def _ausrichten(self) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        ende = time.monotonic() + 20.0
        while time.monotonic() < ende:
            _, _, grad = self._odometry.pose()
            fehler = math.radians(-grad)
            if abs(math.degrees(fehler)) <= config.HOME_WINKEL_TOLERANZ_DEG:
                self._watchdog.stop()
                return
            omega = _begrenzen(1.5 * fehler, config.HOME_DREH_TEMPO, 0.15)
            self._watchdog.submit(DriveCommand(0.0, 0.0, omega))
            await asyncio.sleep(takt)
        self._watchdog.stop()
        log.warning("Ausrichten nicht abgeschlossen (20 s) -- weiter ohne")

    # --- Phase 5: Marker -----------------------------------------------------------

    async def _marker_andocken(self) -> bool:
        """Nachfahren, bis der Marker liegt wie beim Setzen. True bei Erfolg."""
        marker = self._marker
        ende = time.monotonic() + config.MARKER_TIMEOUT_S
        zuletzt_gesehen = time.monotonic()
        erster_abstand: float | None = None
        gut = 0

        while time.monotonic() < ende:
            await asyncio.sleep(0.03)
            jetzt = time.monotonic()
            # Rechnet kurz (wenige Millisekunden), aber nicht in der
            # Ereignisschleife, sonst stockt die Steuerung.
            lage = await asyncio.to_thread(marker.abweichung)
            if lage is None:
                if jetzt - zuletzt_gesehen > 0.3:
                    self._watchdog.stop()  # ohne frisches Bild nicht weiterfahren
                if jetzt - zuletzt_gesehen > 2.5:
                    log.warning("Marker nicht im Bild -- bleibe beim Lidar-Ergebnis")
                    return False
                continue
            zuletzt_gesehen = jetzt

            x, y, theta = lage
            abstand = math.hypot(x, y)
            if erster_abstand is None:
                erster_abstand = abstand
            elif abstand > max(0.5, erster_abstand + 0.25):
                # Läuft die Regelung weg, stimmt ein Vorzeichen nicht -- dann
                # lieber sofort stehen bleiben als munter weiterfahren.
                self._watchdog.stop()
                log.error(
                    "Marker-Andocken entfernt sich vom Ziel (%.2f m) -- abgebrochen",
                    abstand,
                )
                return False

            if (
                abstand <= config.MARKER_TOLERANZ_M
                and abs(math.degrees(theta)) <= config.MARKER_WINKEL_TOLERANZ_DEG
            ):
                self._watchdog.stop()
                gut += 1
                if gut >= 3:  # dreimal hintereinander: kein Zufallstreffer
                    # Hier steht er nachweislich am Startpunkt. Die Ortung
                    # darauf setzen, damit die nächste Fahrt ohne
                    # mitgeschleppte Drift beginnt.
                    self._odometry.korrigieren(0.0, 0.0, 0.0, 1.0)
                    return True
                continue
            gut = 0

            tempo = _begrenzen(1.2 * abstand, config.MARKER_TEMPO, 0.03)
            befehl = _befehl_zu(x, y, theta, 0.0, 0.0, tempo)
            omega = _begrenzen(-1.2 * theta, 0.3, 0.0)
            self._watchdog.submit(DriveCommand(befehl.vx, befehl.vy, omega))

        self._watchdog.stop()
        log.warning("Marker-Andocken: Zeit abgelaufen")
        return False

    # --- Sicherheit ------------------------------------------------------------------

    def _frist_pruefen(self) -> None:
        if time.monotonic() > self._frist:
            x, y, _ = self._odometry.pose()
            raise _Abbruch(f"Zeit abgelaufen, noch {math.hypot(x, y):.2f} m entfernt")

    async def _frei_oder_warten(self, befehl: DriveCommand, ziel_abstand: float) -> None:
        """Bei Hindernis anhalten und kurz warten; wirft, wenn es bleibt."""
        blockiert = self._hindernis(befehl, ziel_abstand)
        if blockiert is None:
            return
        self._watchdog.stop()
        log.info("Hindernis in %.2f m -- warte, ob es verschwindet", blockiert)
        ende = time.monotonic() + config.HOME_HINDERNIS_WARTEN_S
        while time.monotonic() < ende:
            await asyncio.sleep(0.2)
            blockiert = self._hindernis(befehl, ziel_abstand)
            if blockiert is None:
                return
        x, y, _ = self._odometry.pose()
        raise _Abbruch(
            f"Hindernis in {blockiert:.2f} m, noch {math.hypot(x, y):.2f} m "
            "bis zum Startpunkt"
        )

    def _hindernis(self, befehl: DriveCommand, ziel_abstand: float) -> float | None:
        """Steht etwas im Weg? Gibt die Entfernung zurück, sonst None.

        Nur was *vor* dem Ziel liegt, zählt: steht der Startpunkt dicht vor
        einer Wand, darf der Roboter trotzdem bis dorthin fahren.
        """
        if self._localizer is None or befehl.is_stop():
            return None
        # Bei Mecanum ist die Fahrtrichtung nicht "vorn": der Roboter kann
        # seitwärts fahren. Also in die Richtung schauen, in die es geht.
        richtung = math.degrees(math.atan2(befehl.vy, befehl.vx))
        abstand = self._localizer.hindernis_voraus(richtung)
        if abstand is None:
            return None
        grenze = min(config.HOME_STOP_DISTANCE_M, ziel_abstand + 0.15)
        return abstand if abstand < grenze else None


# --- Rechenhilfen --------------------------------------------------------------------


def _richtung_im_roboter(x: float, y: float, theta: float, zx: float, zy: float) -> float:
    """In welcher Richtung (Grad, 0 = vorn) das Ziel vom Roboter aus liegt."""
    dx, dy = zx - x, zy - y
    vorwaerts = dx * math.cos(theta) + dy * math.sin(theta)
    seitwaerts = -dx * math.sin(theta) + dy * math.cos(theta)
    return math.degrees(math.atan2(seitwaerts, vorwaerts))


def _befehl_zu(
    x: float, y: float, theta: float, zx: float, zy: float, tempo: float
) -> DriveCommand:
    """Fahrbefehl geradewegs auf ein Ziel zu, ohne zu drehen.

    Die Richtung zum Ziel wird in Roboterkoordinaten gedreht: der Antrieb
    kennt nur "vorwärts" und "seitwärts", nicht "Richtung Startpunkt".
    """
    dx, dy = zx - x, zy - y
    vorwaerts = dx * math.cos(theta) + dy * math.sin(theta)
    seitwaerts = -dx * math.sin(theta) + dy * math.cos(theta)
    laenge = math.hypot(vorwaerts, seitwaerts) or 1.0
    return DriveCommand(
        vx=vorwaerts / laenge * tempo,
        vy=seitwaerts / laenge * tempo,
        omega=0.0,
    )


def _begrenzen(wert: float, hoechstens: float, mindestens: float) -> float:
    """Betrag zwischen ``mindestens`` und ``hoechstens``, Vorzeichen bleibt.

    Das Minimum gibt es, weil die Motoren bei sehr kleinen Werten nicht
    anlaufen -- der Roboter bliebe kurz vor dem Ziel einfach stehen.
    """
    betrag = min(hoechstens, max(mindestens, abs(wert)))
    return math.copysign(betrag, wert)
