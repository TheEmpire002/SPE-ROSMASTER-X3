"""Autonomes Erkunden (Test 1): zufällig umherfahren, allem ausweichen.

Der Roboter fährt geradeaus, bis vorn etwas im Weg ist, dreht sich dann in
die freieste Richtung und fährt weiter. Damit er nicht ewig dieselbe Bahn
abfährt, wechselt er ab und zu ohne Anlass die Richtung.

Zwei Entscheidungen, die das Verhalten prägen:

**Gedreht wird nach Zeit, nicht nach Winkel.** Der Kurswinkel aus der IMU
der Bibliothek ist am X3 unbrauchbar (Sensor-Check 28.10.2026: ``attitude``
immer 0, Gyro mit großem Versatz). Deshalb dreht er für eine aus Winkel und
Drehrate berechnete Zeit und prüft danach mit dem Lidar, ob vorn frei ist.
Ob er dabei 80° oder 100° geschafft hat, ist für das Erkunden egal.

**Es zählt der Fahrschlauch, nicht ein Kegel.** Ein Kegel ist nah am
Roboter schmal (übersieht ein Tischbein neben dem Rad) und weit weg breit
(hält vor Wänden, an denen er vorbeikäme). Geprüft wird stattdessen ein
Streifen von Roboterbreite in Fahrtrichtung.

Jeder Fahrbefehl läuft über den ``SafetyWatchdog`` -- bleibt diese Schleife
hängen, hält der Roboter nach 0,5 s von selbst an.
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import time

from typing import TYPE_CHECKING

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import hindernisse as hd

if TYPE_CHECKING:  # nur für die Typangaben -- vermeidet Ringimporte
    from ..sensors.lidar import LidarService
    from ..sensors.personen import Personenerkennung

log = logging.getLogger(__name__)

#: Zustände, so wie sie in der App erscheinen (state.autonomie.zustand).
FAEHRT = "faehrt"
DREHT = "dreht"
WARTET = "wartet"
MENSCH_WARTEN = "wartet auf Mensch"
KEIN_LIDAR = "wartet auf Lidar"

#: Kandidaten für die neue Fahrtrichtung, in Grad (0 = geradeaus).
_RICHTUNGEN = list(range(-180, 181, 10))

#: Nach so vielen erfolglosen Drehungen in Folge gibt er auf -- dann steckt
#: er fest, und Weiterdrehen bringt nichts außer leerem Akku.
_MAX_FEHLVERSUCHE = 6


def frei_in_richtung(punkte, richtung_grad: float, halbe_breite: float) -> float:
    """Wie weit der Roboter in dieser Richtung fahren könnte (Meter).

    Gemessen ab Lidar-Mitte bis zum ersten Punkt im Fahrschlauch.
    Unendlich, wenn dort nichts ist.
    """
    import numpy as np

    if len(punkte) == 0:
        return float("inf")
    a = math.radians(richtung_grad)
    c, s = math.cos(a), math.sin(a)
    vor = punkte[:, 0] * c + punkte[:, 1] * s
    quer = -punkte[:, 0] * s + punkte[:, 1] * c
    im_weg = (vor > 0.0) & (np.abs(quer) < halbe_breite)
    return float(vor[im_weg].min()) if im_weg.any() else float("inf")


class Erkunder:
    """Fährt autonom umher. Gleiche Bedienung wie ``Homing``."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        lidar: "LidarService | None",
        hinderniserkennung: hd.Hinderniserkennung,
        personen: "Personenerkennung | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._erkennung = hinderniserkennung
        self._personen = personen
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None

        # Für die Anzeige -- wird von der Schleife geschrieben, vom
        # Status-Takt gelesen (beides im selben Event-Loop, kein Lock nötig).
        self._zustand = WARTET
        self._text = ""
        self._frei_vorn: float | None = None
        self._gestartet = 0.0
        self._ausweichen = 0
        self._menschen_gesehen = 0

        # Ablauf der aktuellen Drehung
        self._dreh_richtung = 0.0
        self._dreh_ende = 0.0
        self._fehlversuche = 0
        self._naechster_wechsel = 0.0
        self._warten_bis = 0.0
        self._ohne_lidar_seit: float | None = None

    # --- Bedienung (wie Homing) ------------------------------------------------

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        """Warum die letzte Erkundung endete -- für die Anzeige in der App."""
        return self._grund

    @property
    def verfuegbar(self) -> bool:
        """Ohne Lidar fährt er nicht -- blind umherfahren wäre gefährlich."""
        return self._lidar is not None

    def start(self) -> bool:
        """Erkundung beginnen. False, wenn schon eine läuft oder kein Lidar da ist."""
        if self.aktiv or not self.verfuegbar:
            return False
        self._grund = None
        self._task = asyncio.create_task(self._fahren())
        return True

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        """Sofort beenden. Auch aufzurufen, wenn der Mensch eingreift."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._grund = grund
            log.info("Erkundung %s", grund)
        self._task = None

    def zustand(self) -> dict:
        """Für state.autonomie in der App."""
        aktiv = self.aktiv
        return {
            "aktiv": aktiv,
            "verfuegbar": self.verfuegbar,
            "zustand": self._zustand if aktiv else None,
            "text": self._text if aktiv else None,
            "frei_vorn": (
                round(self._frei_vorn, 2)
                if aktiv and self._frei_vorn is not None and math.isfinite(self._frei_vorn)
                else None
            ),
            "dauer_s": round(time.monotonic() - self._gestartet) if aktiv else None,
            "ausweichen": self._ausweichen,
            "menschen": self._menschen_gesehen,
            "grund": self._grund,
        }

    # --- Fahrschleife ------------------------------------------------------------

    async def _fahren(self) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        jetzt = time.monotonic()
        self._gestartet = jetzt
        self._ausweichen = 0
        self._menschen_gesehen = 0
        self._fehlversuche = 0
        self._ohne_lidar_seit = None
        self._naechster_wechsel = jetzt + self._wanderzeit()
        self._setzen(FAEHRT, "fährt los")
        if self._personen is not None:
            self._personen.aktivieren(True)
        log.info("Erkundung gestartet")

        try:
            while True:
                befehl = self._schritt(time.monotonic())
                if befehl is None:  # fertig, Grund steht in self._grund
                    self._watchdog.stop()
                    log.info("Erkundung beendet -- %s", self._grund)
                    return
                self._watchdog.submit(befehl)
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            # Immer anhalten, egal warum abgebrochen wurde.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            log.exception("Erkundung abgebrochen")
        finally:
            if self._personen is not None:
                self._personen.aktivieren(False)

    def _schritt(self, jetzt: float) -> DriveCommand | None:
        """Ein Takt: Lage ansehen, Fahrbefehl zurückgeben. None = Ende."""
        halt = DriveCommand(0.0, 0.0, 0.0)

        if self._grund is not None:  # im letzten Takt aufgegeben
            return None
        if config.EXPLORE_TIMEOUT_S > 0 and jetzt - self._gestartet > config.EXPLORE_TIMEOUT_S:
            self._grund = f"Zeit abgelaufen ({config.EXPLORE_TIMEOUT_S:.0f} s)"
            return None

        # Ohne frische Lidar-Runde fährt er nicht. Kurze Aussetzer (USB-Reset)
        # werden abgewartet, längere beenden die Fahrt.
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or jetzt - scan.zeit > config.EXPLORE_SCAN_MAX_AGE_S:
            if self._ohne_lidar_seit is None:
                self._ohne_lidar_seit = jetzt
                log.warning("Erkundung: keine frischen Lidar-Daten -- halte an")
            if jetzt - self._ohne_lidar_seit > config.EXPLORE_LIDAR_WAIT_S:
                self._grund = "Lidar liefert keine Daten mehr"
                return None
            self._setzen(KEIN_LIDAR, "keine frischen Lidar-Daten")
            return halt
        self._ohne_lidar_seit = None

        punkte = scan.punkte()
        breite = config.EXPLORE_HALF_WIDTH_M
        vorn = frei_in_richtung(punkte, 0.0, breite)
        self._frei_vorn = vorn

        # Rückt etwas ganz nah heran, nicht bewegen -- auch nicht drehen: die
        # Ecken des Roboters schwenken beim Drehen nach außen. Erst ab drei
        # Punkten: ein einzelner Fehlstrahl soll ihn nicht festhalten.
        nahe = sorted(r for r in scan.entfernungen if r < config.EXPLORE_NAH_M)
        if len(nahe) >= 3:
            nah = nahe[0]
            self._setzen(WARTET, f"etwas sehr nah ({nah:.2f} m) -- warte")
            return halt

        if self._zustand == DREHT:
            return self._drehen(jetzt, punkte, vorn)

        if self._zustand == MENSCH_WARTEN:
            if self._mensch_voraus(scan) is None:
                self._setzen(FAEHRT, "Weg wieder frei")
            elif jetzt >= self._warten_bis:
                # Die Person bleibt stehen -- dann eben drumherum.
                return self._wenden(jetzt, punkte, "Mensch bleibt stehen, weiche aus")
            else:
                return halt

        # --- Geradeaus fahren ---
        mensch = self._mensch_voraus(scan)
        if mensch is not None:
            self._menschen_gesehen += 1
            self._warten_bis = jetzt + config.EXPLORE_PERSON_WAIT_S
            self._setzen(MENSCH_WARTEN, f"Mensch in {mensch:.2f} m -- warte kurz")
            return halt

        if vorn < config.EXPLORE_STOP_M:
            return self._wenden(jetzt, punkte, f"Hindernis in {vorn:.2f} m")

        if jetzt >= self._naechster_wechsel:
            return self._wenden(jetzt, punkte, "zufälliger Richtungswechsel", zufall=True)

        self._fehlversuche = 0
        self._setzen(FAEHRT, "fährt" if vorn >= config.EXPLORE_SLOW_M else f"bremst, frei {vorn:.2f} m")
        return self._geradeaus(punkte, vorn)

    def _geradeaus(self, punkte, vorn: float) -> DriveCommand:
        # Vor Hindernissen abbremsen, aber nicht unter ~1/3: Sehr kleine
        # Werte setzen die Motoren am Boden nicht mehr um.
        spanne = config.EXPLORE_SLOW_M - config.EXPLORE_STOP_M
        anteil = min(1.0, max(0.35, (vorn - config.EXPLORE_STOP_M) / spanne))
        vx = config.EXPLORE_SPEED * anteil

        # Sachte zur offeneren Seite lenken, solange vorn etwas näher kommt
        # -- so fährt er um Hindernisse herum, statt erst davor zu wenden.
        omega = 0.0
        if vorn < config.EXPLORE_SLOW_M:
            links = frei_in_richtung(punkte, 30.0, config.EXPLORE_HALF_WIDTH_M)
            rechts = frei_in_richtung(punkte, -30.0, config.EXPLORE_HALF_WIDTH_M)
            if abs(links - rechts) > 0.15:
                omega = 0.35 * (1.0 - anteil + 0.3) * (1 if links > rechts else -1)

        # Mecanum: seitlich zu nahe Dinge schiebt er weg, statt anzuhalten.
        # Nur der Bereich neben dem Roboter zählt (x zwischen Heck und Bug).
        vy = 0.0
        if len(punkte):
            x, y = punkte[:, 0], punkte[:, 1]
            neben = (x > -0.15) & (x < 0.25)
            links_nah = neben & (y > 0) & (y < config.EXPLORE_SIDE_M)
            rechts_nah = neben & (y < 0) & (y > -config.EXPLORE_SIDE_M)
            if links_nah.any() and not rechts_nah.any():
                vy = -0.08
            elif rechts_nah.any() and not links_nah.any():
                vy = 0.08
        return DriveCommand(vx, vy, omega)

    def _wenden(self, jetzt: float, punkte, anlass: str, zufall: bool = False) -> DriveCommand:
        """Neue Richtung wählen und die Drehung dorthin beginnen."""
        richtung, frei = self._richtung_waehlen(punkte, zufall)
        if not zufall:
            self._ausweichen += 1
        if frei < config.EXPLORE_GO_M:
            self._fehlversuche += 1
            if self._fehlversuche >= _MAX_FEHLVERSUCHE:
                self._grund = "festgefahren -- nirgends genug Platz"
                return DriveCommand(0.0, 0.0, 0.0)  # Ende im nächsten Takt
        self._dreh_richtung = richtung
        dauer = abs(math.radians(richtung)) / config.EXPLORE_TURN_SPEED * config.EXPLORE_TURN_FACTOR
        self._dreh_ende = jetzt + max(0.3, dauer)
        self._setzen(DREHT, f"{anlass} -- drehe {richtung:+.0f}°")
        log.info("Erkundung: %s -- drehe %+.0f° (dort %.2f m frei)", anlass, richtung, frei)
        return self._drehbefehl()

    def _drehen(self, jetzt: float, punkte, vorn: float) -> DriveCommand | None:
        if self._grund is not None:  # in _wenden aufgegeben
            return None
        if jetzt < self._dreh_ende:
            return self._drehbefehl()
        if vorn >= config.EXPLORE_GO_M:
            self._fehlversuche = 0
            self._naechster_wechsel = jetzt + self._wanderzeit()
            self._setzen(FAEHRT, "Weg frei")
            return self._geradeaus(punkte, vorn)
        # Gedreht, aber vorn immer noch zu eng (Drehung zu kurz geraten oder
        # die Welt hat sich bewegt) -- neu wählen.
        self._fehlversuche += 1
        if self._fehlversuche >= _MAX_FEHLVERSUCHE:
            self._grund = "festgefahren -- nirgends genug Platz"
            return None
        return self._wenden(jetzt, punkte, f"vorn nur {vorn:.2f} m frei")

    def _drehbefehl(self) -> DriveCommand:
        richtung = 1.0 if self._dreh_richtung >= 0 else -1.0
        return DriveCommand(0.0, 0.0, richtung * config.EXPLORE_TURN_SPEED)

    def _richtung_waehlen(self, punkte, zufall: bool) -> tuple[float, float]:
        """Freieste Richtung -- unter den guten eine zufällige.

        Zufall statt immer der allerfreiesten, sonst pendelt er in einem
        langen Flur ewig zwischen den beiden Enden.
        """
        breite = config.EXPLORE_HALF_WIDTH_M
        kandidaten = []
        for r in _RICHTUNGEN:
            # Beim Ausweichen ist geradeaus ja gerade versperrt; beim
            # zufälligen Wechsel soll es eine echte Wende sein.
            if abs(r) < (45 if zufall else 20):
                continue
            kandidaten.append((r, min(frei_in_richtung(punkte, r, breite), config.OBSTACLE_RANGE_M)))

        weit = [k for k in kandidaten if k[1] >= config.EXPLORE_GO_M + 0.5]
        if weit:
            return random.choice(weit)
        return max(kandidaten, key=lambda k: k[1])

    def _mensch_voraus(self, scan) -> float | None:
        """Abstand zum Menschen vor dem Roboter, sonst None."""
        personen = self._personen.winkelbereiche() if self._personen is not None else []
        for h in self._erkennung.aktuell(scan, personen):
            if h.art == hd.MENSCH and h.abstand < 1.0 and abs(h.winkel) < 35.0:
                return h.abstand
        return None

    def _setzen(self, zustand: str, text: str) -> None:
        self._zustand = zustand
        self._text = text

    @staticmethod
    def _wanderzeit() -> float:
        return random.uniform(config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S)
