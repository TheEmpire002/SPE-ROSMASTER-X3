"""Autonomes Erkunden (Test 1): zufällig umherfahren, allem ausweichen.

Der Roboter fährt auf einem gewählten Kurs, umfährt Hindernisse in Bögen
und dreht sich erst, wenn kein sicherer Bogen mehr passt, in eine freie
Richtung (eine Tür oder Lücke möglichst mittig). Damit er nicht ewig
dieselbe Bahn abfährt, wechselt er ab und zu ohne Anlass die Richtung. Die
Bausteine dafür (Kurzgedächtnis, Bögen, Lücken, Hochheben) stehen in
``ausweichen.py``.

Entscheidungen, die das Verhalten prägen:

**Gedreht wird nach gemessenem Winkel, die Zeit ist nur Obergrenze.** Der
IMU-Kurswinkel der Bibliothek ist am X3 unbrauchbar (Sensor-Check
28.10.2026: ``attitude`` immer 0). Die Drehrate, die das Board meldet,
stimmt dagegen gut mit dem Lidar überein. Der Erkunder summiert sie
während der Drehung selbst auf, statt ``theta`` der Odometrie zu nehmen:
Lidar-Korrekturen können ``theta`` springen lassen. Fehlt die Messung,
dreht er wie früher nach Zeit (``EXPLORE_TURN_FACTOR``). Danach entscheidet
er erst mit einer Lidar-Runde, die nach dem Drehen begonnen hat; die Runde
von davor zeigt noch die alte Richtung.

**Es zählt der Fahrschlauch, nicht ein Kegel.** Ein Kegel ist nah am
Roboter schmal (übersieht ein Tischbein neben dem Rad) und weit weg breit
(hält vor Wänden, an denen er vorbeikäme). Geprüft wird stattdessen ein
Streifen von Roboterbreite in Fahrtrichtung.

**Befreien statt Dauerstand.** Rückt etwas ganz nah heran, wartet er erst
kurz (vielleicht ein Mensch, der gleich weitergeht) und schiebt sich dann
mit dem Mecanum-Antrieb ohne Drehung in die freieste Richtung: Beim Drehen
schwenken die Ecken nach außen. Dasselbe, wenn er trotz Fahrbefehl nicht
vorankommt (etwas unterhalb der Lidar-Ebene). Erst wenn mehrere
Befreiungen hintereinander nichts bringen, gibt er auf.

**Hochgehoben heißt neu anfangen.** Trägt ihn jemand woandershin, passt
nichts Gelerntes mehr (Gedächtnis, Kurs, Zähler). Er hält an, bis das
Lidar-Bild wieder ruhig ist, und beginnt dann von vorn.

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
from . import ausweichen
from . import hindernisse as hd

if TYPE_CHECKING:  # nur für die Typangaben -- vermeidet Ringimporte
    from ..sensors.lidar import LidarService
    from ..sensors.personen import Personenerkennung
    from .odometry import Odometry

log = logging.getLogger(__name__)

#: Zustände, so wie sie in der App erscheinen (state.autonomie.zustand).
FAEHRT = "faehrt"
DREHT = "dreht"
WARTET = "wartet"
MENSCH_WARTEN = "wartet auf Mensch"
KEIN_LIDAR = "wartet auf Lidar"
BEFREIT = "befreit sich"
HOCHGEHOBEN = "hochgehoben"

#: Kandidaten für die neue Fahrtrichtung, in Grad (0 = geradeaus).
_RICHTUNGEN = list(range(-180, 181, 10))

#: Kandidaten beim Befreien (seitlich/rückwärts schieben), ringsum.
_BEFREI_RICHTUNGEN = list(range(-180, 180, 15))

# Strahlen rundum für die Festfahr-Erkennung per Lidar. Nur nach vorn zu
# schauen genügt nicht: Fährt er schräg an einer Wand entlang, bleibt der
# Abstand im Fahrschlauch gleich, obwohl er gut vorankommt.
_FEST_RICHTUNGEN = list(range(-180, 180, 30))
_FEST_STRAHL_M = 0.05

#: Nach so vielen erfolglosen Drehungen in Folge hilft Weiterdrehen nicht
#: mehr -- dann versucht er, sich freizuschieben.
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
        odometry: "Odometry | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._erkennung = hinderniserkennung
        self._personen = personen
        self._odometry = odometry
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
        self._befreit_gesamt = 0

        self._zuruecksetzen(0.0)

    def _zuruecksetzen(self, jetzt: float) -> None:
        """Ablauf-Zustand für eine neue Fahrt."""
        self._letzter_takt: float | None = None
        self._letzter_befehl = DriveCommand(0.0, 0.0, 0.0)
        # Drehung
        self._dreh_richtung = 0.0
        self._dreh_start = jetzt
        self._dreh_ende = jetzt
        self._dreh_grenze = jetzt
        self._dreh_gemessen = False
        self._gedreht = 0.0  # Grad, aus der Board-Drehrate aufsummiert
        self._dreh_fertig: float | None = None  # wann der Drehbefehl endete
        self._fehlversuche = 0
        self._naechster_wechsel = jetzt
        self._warten_bis = 0.0
        self._ohne_lidar_seit: float | None = None
        # Nah-Halt und Befreien
        self._nah_seit: float | None = None
        self._befrei_richtung = 0.0
        self._befrei_ende = 0.0
        self._befreiungen = 0  # in Folge, ohne dazwischen frei zu fahren
        self._frei_seit: float | None = None
        # Festfahr-Erkennung
        self._fest_start: float | None = None
        self._fest_bild: dict[int, float] = {}
        self._fest_still_s = 0.0
        self._fest_takt = 0.0
        # Ausweichen (F2): Kurzgedächtnis, Hochheben und der Kurs, auf den
        # er nach dem Umfahren zurücklenkt (None: beim nächsten Geradeaus-
        # fahren den aktuellen übernehmen).
        self._gedaechtnis = ausweichen.Kurzgedaechtnis(self._odometry)
        self._hoch = ausweichen.HochhebeErkennung()
        self._ziel_kurs: float | None = None

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
            "befreien": self._befreit_gesamt,
            "grund": self._grund,
        }

    # --- Fahrschleife ------------------------------------------------------------

    def _beginnen(self, jetzt: float) -> None:
        """Zähler und Ablauf für eine neue Fahrt -- auch für die Tests."""
        self._gestartet = jetzt
        self._ausweichen = 0
        self._menschen_gesehen = 0
        self._befreit_gesamt = 0
        self._grund = None
        self._zuruecksetzen(jetzt)
        self._naechster_wechsel = jetzt + self._wanderzeit()
        self._setzen(FAEHRT, "fährt los")

    async def _fahren(self) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        self._beginnen(time.monotonic())
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
                self._watchdog.submit(befehl, quelle="autonom")
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
        dt = 0.0 if self._letzter_takt is None else max(0.0, jetzt - self._letzter_takt)
        self._letzter_takt = jetzt
        befehl = self._entscheiden(jetzt, dt)
        if befehl is not None:
            self._letzter_befehl = befehl
        return befehl

    def _entscheiden(self, jetzt: float, dt: float) -> DriveCommand | None:
        halt = DriveCommand(0.0, 0.0, 0.0)

        if self._grund is not None:  # im letzten Takt aufgegeben
            return None
        if config.EXPLORE_TIMEOUT_S > 0 and jetzt - self._gestartet > config.EXPLORE_TIMEOUT_S:
            self._grund = f"Zeit abgelaufen ({config.EXPLORE_TIMEOUT_S:.0f} s)"
            return None

        # Drehung mitzählen, bevor irgendetwas anderes entscheidet: auch ein
        # Halt-Takt (Lidar-Aussetzer) gehört noch zur laufenden Drehung.
        drehrate = self._drehrate_gemessen()
        if self._zustand == DREHT and drehrate is not None:
            self._gedreht += math.degrees(drehrate) * dt
        # Eigene Lage fürs Kurzgedächtnis -- jeden Takt, auch ohne neue Runde.
        self._gedaechtnis.fortschreiben(dt)

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
            # Eine laufende Drehung oder Befreiung bleibt gemerkt, nur die
            # Anzeige wechselt -- sonst finge er danach von vorn an.
            self._text = "keine frischen Lidar-Daten"
            if self._zustand not in (DREHT, BEFREIT):
                self._zustand = KEIN_LIDAR
            return halt
        self._ohne_lidar_seit = None

        # Hochgehoben oder getragen: anhalten, bis er wieder ruhig steht.
        lage = self._gedaechtnis.lage if self._gedaechtnis.misst else None
        if self._hoch.pruefen(scan, lage):
            if self._zustand != HOCHGEHOBEN:
                self._hochgehoben()
            return halt
        if self._zustand == HOCHGEHOBEN:
            self._naechster_wechsel = jetzt + self._wanderzeit()
            self._setzen(FAEHRT, "wieder abgesetzt")

        # Die rohe Runde für alles, was "jetzt" meint (Festfahren); die Runde
        # samt Gedächtnis fürs Ausweichen -- dort zählt auch, was gerade aus
        # dem Blickfeld gerutscht ist.
        roh = scan.punkte()
        punkte = self._gedaechtnis.punkte(scan)
        breite = config.EXPLORE_HALF_WIDTH_M
        vorn = frei_in_richtung(punkte, 0.0, breite)
        self._frei_vorn = vorn

        if self._zustand == BEFREIT:
            return self._befreien(jetzt, punkte, vorn)

        # Rückt etwas ganz nah heran, erst nicht bewegen -- auch nicht drehen:
        # die Ecken des Roboters schwenken beim Drehen nach außen. Erst ab drei
        # Punkten: ein einzelner Fehlstrahl soll ihn nicht festhalten. Geht
        # es nicht von selbst weg, schiebt er sich frei, statt ewig zu warten.
        nahe = sorted(r for r in scan.entfernungen if r < config.EXPLORE_NAH_M)
        if len(nahe) >= 3:
            if self._nah_seit is None:
                self._nah_seit = jetzt
            if jetzt - self._nah_seit >= config.EXPLORE_NAH_WARTEN_S:
                self._nah_seit = None
                return self._befreien_beginnen(jetzt, punkte, f"etwas sehr nah ({nahe[0]:.2f} m)")
            # Freie Fahrt unterbrochen: die Erholungszeit beginnt danach neu.
            self._frei_seit = None
            self._setzen(WARTET, f"etwas sehr nah ({nahe[0]:.2f} m) -- warte")
            return halt
        self._nah_seit = None

        if self._zustand == WARTET:
            self._setzen(FAEHRT, "Weg wieder frei")

        if self._zustand == DREHT:
            return self._drehen(jetzt, scan, punkte, vorn)

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

        fest = self._festgefahren(jetzt, dt, roh)
        if fest is not None:
            # Nach vorn geht es offenbar nicht -- also seitlich oder zurück.
            return self._befreien_beginnen(jetzt, punkte, fest, vorwaerts=False)

        # Den zufälligen Wechsel nicht mitten in einer Tür oder Engstelle:
        # erst hindurch, sonst dreht er genau dort um, wo er hinwollte.
        eng = min(scan.entfernungen, default=config.OBSTACLE_RANGE_M) < config.AUSWEICH_ENG_M
        if jetzt >= self._naechster_wechsel and not eng:
            return self._wenden(jetzt, punkte, "zufälliger Richtungswechsel", zufall=True)

        # Eine Weile frei gefahren: Fehlversuche und Befreiungen zählen neu.
        # Nicht sofort -- sonst setzte jeder Takt gegen ein unsichtbares
        # Hindernis den Zähler zurück, bevor die Festfahr-Erkennung greift.
        if self._frei_seit is None:
            self._frei_seit = jetzt
        elif jetzt - self._frei_seit >= config.EXPLORE_FREI_ZURUECK_S:
            self._fehlversuche = 0
            self._befreiungen = 0
        return self._geradeaus(jetzt, punkte, vorn)

    def _geradeaus(self, jetzt: float, punkte, vorn: float) -> DriveCommand:
        """Den besten sicheren Bogen fahren; passt keiner, wenden."""
        # Vor Hindernissen abbremsen, aber nicht unter ~1/3: Sehr kleine
        # Werte setzen die Motoren am Boden nicht mehr um.
        spanne = config.EXPLORE_SLOW_M - config.EXPLORE_STOP_M
        anteil = min(1.0, max(0.35, (vorn - config.EXPLORE_STOP_M) / spanne))
        vx_max = config.EXPLORE_SPEED * anteil

        # Den Kurs vom letzten Losfahren halten: Nach dem Umfahren lenkt er
        # zurück, statt im Bogen weiterzudrehen. Ohne gemessene Drehrate
        # gibt es keinen Kurs -- dann zählt nur, wenig zu lenken.
        kurs = self._gedaechtnis.kurs
        if kurs is None:
            self._ziel_kurs = None
        elif self._ziel_kurs is None:
            self._ziel_kurs = kurs
        fehler = 0.0 if kurs is None else ausweichen._winkel_normieren(self._ziel_kurs - kurs)

        wahl = ausweichen.bogen_planen(punkte, vx_max, fehler, self._letzter_befehl.omega)
        if wahl is None:
            return self._wenden(jetzt, punkte, "kein sicherer Bogen")
        (vx, vy, omega), _luft = wahl
        gerade = omega == 0.0 and vy == 0.0
        if vorn >= config.EXPLORE_SLOW_M:
            text = "fährt" if gerade else "fährt Bogen"
        else:
            text = f"bremst, frei {vorn:.2f} m" if gerade else f"umfährt, frei {vorn:.2f} m"
        self._setzen(FAEHRT, text)
        return DriveCommand(vx, vy, omega)

    # --- Drehen ------------------------------------------------------------------

    def _wenden(self, jetzt: float, punkte, anlass: str, zufall: bool = False) -> DriveCommand:
        """Neue Richtung wählen und die Drehung dorthin beginnen."""
        self._frei_seit = None
        richtung, frei = self._richtung_waehlen(punkte, zufall)
        if not zufall:
            self._ausweichen += 1
        if frei < config.EXPLORE_GO_M:
            self._fehlversuche += 1
            if self._fehlversuche >= _MAX_FEHLVERSUCHE:
                # Drehen bringt nichts mehr -- freischieben, dann neu.
                self._fehlversuche = 0
                return self._befreien_beginnen(jetzt, punkte, "nirgends genug Platz zum Drehen")
        self._dreh_richtung = richtung
        dauer = abs(math.radians(richtung)) / config.EXPLORE_TURN_SPEED
        self._dreh_start = jetzt
        self._gedreht = 0.0
        self._dreh_fertig = None
        self._dreh_gemessen = self._drehrate_gemessen() is not None
        if self._dreh_gemessen:
            # Der Winkel entscheidet; die Zeit fängt nur ab, dass er sich
            # nicht dreht (blockiert) oder die Messung ausfällt.
            self._dreh_ende = jetzt + max(0.3, dauer * config.EXPLORE_TURN_FACTOR)
            self._dreh_grenze = jetzt + max(1.0, dauer * config.EXPLORE_DREH_ZEIT_FAKTOR)
        else:
            self._dreh_ende = jetzt + max(0.3, dauer * config.EXPLORE_TURN_FACTOR)
            self._dreh_grenze = self._dreh_ende
        self._setzen(DREHT, f"{anlass} -- drehe {richtung:+.0f}°")
        log.info("Erkundung: %s -- drehe %+.0f° (dort %.2f m frei)", anlass, richtung, frei)
        return self._drehbefehl()

    def _drehen(self, jetzt: float, scan, punkte, vorn: float) -> DriveCommand | None:
        if self._dreh_fertig is None:
            if self._dreh_gemessen and self._drehrate_gemessen() is not None:
                # Kurz vor dem Ziel aufhören: Verzögerung und Bremsrampe
                # drehen ihn noch ein Stück weiter.
                rest = abs(self._dreh_richtung) - abs(self._gedreht)
                fertig = rest <= config.EXPLORE_DREH_VORHALT_GRAD or jetzt >= self._dreh_grenze
            else:
                # Messung fehlt (oder fiel während der Drehung weg): nach Zeit.
                fertig = jetzt >= self._dreh_ende
            if not fertig:
                return self._drehbefehl()
            self._dreh_fertig = jetzt
            if abs(self._gedreht) > 0.0:
                log.info(
                    "Erkundung: Drehung beendet, %+.0f° von %+.0f° gemessen",
                    self._gedreht, self._dreh_richtung,
                )

        # Erst mit einer Runde entscheiden, die nach der Drehung begann --
        # die laufende zeigt teils noch die alte Richtung.
        if scan.zeit - scan.dauer < self._dreh_fertig + config.EXPLORE_NACHLAUF_S:
            self._text = "prüft den Weg"
            return DriveCommand(0.0, 0.0, 0.0)

        if vorn >= config.EXPLORE_GO_M:
            self._fehlversuche = 0
            self._naechster_wechsel = jetzt + self._wanderzeit()
            self._ziel_kurs = None  # neuer Kurs: der nach der Drehung
            self._setzen(FAEHRT, "Weg frei")
            return self._geradeaus(jetzt, punkte, vorn)
        # Gedreht, aber vorn immer noch zu eng (Drehung zu kurz geraten oder
        # die Welt hat sich bewegt) -- neu wählen.
        self._fehlversuche += 1
        if self._fehlversuche >= _MAX_FEHLVERSUCHE:
            self._fehlversuche = 0
            return self._befreien_beginnen(jetzt, punkte, f"nach {_MAX_FEHLVERSUCHE} Drehungen kein Weg")
        return self._wenden(jetzt, punkte, f"vorn nur {vorn:.2f} m frei")

    def _drehbefehl(self) -> DriveCommand:
        richtung = 1.0 if self._dreh_richtung >= 0 else -1.0
        return DriveCommand(0.0, 0.0, richtung * config.EXPLORE_TURN_SPEED)

    def _drehrate_gemessen(self) -> float | None:
        """Vom Board gemessene Drehrate (rad/s), None ohne echte Messung.

        Ohne Messung rechnet die Odometrie mit dem Befehl -- daraus einen
        Winkel zu summieren, hieße wieder nach Zeit drehen, nur verdeckt.
        """
        if self._odometry is None or not self._odometry.misst_tatsaechlich:
            return None
        return self._odometry.geschwindigkeit()[2]

    def _richtung_waehlen(self, punkte, zufall: bool) -> tuple[float, float]:
        """Freieste Richtung -- unter den guten eine zufällige.

        Zufall statt immer der allerfreiesten, sonst pendelt er in einem
        langen Flur ewig zwischen den beiden Enden.
        """
        breite = config.EXPLORE_HALF_WIDTH_M
        # Beim Ausweichen ist geradeaus ja gerade versperrt; beim
        # zufälligen Wechsel soll es eine echte Wende sein.
        grenze = 45 if zufall else 20
        # Gute Richtungen kommen aus den freien Bereichen: die Mitte einer
        # Tür statt knapp am Rahmen vorbei.
        weit = [
            k for k in ausweichen.luecken(punkte, breite, config.EXPLORE_GO_M + 0.5, config.OBSTACLE_RANGE_M)
            if abs(k[0]) >= grenze
        ]
        if weit:
            return random.choice(weit)
        kandidaten = [
            (r, min(frei_in_richtung(punkte, r, breite), config.OBSTACLE_RANGE_M))
            for r in _RICHTUNGEN if abs(r) >= grenze
        ]
        return max(kandidaten, key=lambda k: k[1])

    # --- Befreien ----------------------------------------------------------------

    def _befreien_beginnen(
        self, jetzt: float, punkte, anlass: str, vorwaerts: bool = True
    ) -> DriveCommand | None:
        """Ohne Drehung in die freieste Richtung schieben.

        ``vorwaerts=False`` schließt die vordere Hälfte aus: Steckt er fest,
        obwohl vorn frei aussieht, ist dort etwas, das der Lidar nicht sieht.
        """
        self._frei_seit = None
        self._fest_start = None
        self._fest_still_s = 0.0
        if self._befreiungen >= config.EXPLORE_BEFREIEN_MAX:
            self._grund = f"festgefahren -- {self._befreiungen} Befreiungsversuche ohne Erfolg"
            log.warning("Erkundung: %s (%s)", self._grund, anlass)
            return None
        self._befreiungen += 1
        self._befreit_gesamt += 1

        breite = config.EXPLORE_HALF_WIDTH_M
        kandidaten = [
            (r, frei_in_richtung(punkte, r, breite))
            for r in _BEFREI_RICHTUNGEN
            if vorwaerts or abs(r) >= 90
        ]
        r, frei = max(kandidaten, key=lambda k: (k[1], -abs(k[0])))
        if frei < config.EXPLORE_BEFREIEN_FREI_M:
            # Nirgends Platz: stehen bleiben und es nach der Wartezeit noch
            # einmal versuchen (zählt als Versuch) -- vielleicht geht der
            # Mensch, der ihn umringt, ja weg.
            self._nah_seit = jetzt
            self._setzen(WARTET, f"{anlass} -- kein Platz zum Befreien, warte")
            log.info("Erkundung: %s -- kein Platz zum Befreien (beste %.2f m)", anlass, frei)
            return DriveCommand(0.0, 0.0, 0.0)

        self._befrei_richtung = float(r)
        self._befrei_ende = jetzt + config.EXPLORE_BEFREIEN_S
        self._setzen(BEFREIT, f"{anlass} -- schiebt sich frei ({r:+.0f}°)")
        log.info("Erkundung: %s -- befreie mich Richtung %+.0f° (dort %.2f m frei)", anlass, r, frei)
        return self._befreibefehl()

    def _befreien(self, jetzt: float, punkte, vorn: float) -> DriveCommand | None:
        frei = frei_in_richtung(punkte, self._befrei_richtung, config.EXPLORE_HALF_WIDTH_M)
        if jetzt < self._befrei_ende and frei >= config.EXPLORE_BEFREIEN_STOP_M:
            return self._befreibefehl()
        # Zeit um oder in Schieberichtung wird es eng: neu entscheiden.
        # Die Nah-Prüfung im nächsten Takt fängt ab, wenn es nichts brachte.
        self._nah_seit = None
        if vorn >= config.EXPLORE_GO_M:
            self._naechster_wechsel = jetzt + self._wanderzeit()
            self._ziel_kurs = None
            self._setzen(FAEHRT, "frei, fährt weiter")
            return DriveCommand(0.0, 0.0, 0.0)
        return self._wenden(jetzt, punkte, "nach dem Befreien")

    def _befreibefehl(self) -> DriveCommand:
        a = math.radians(self._befrei_richtung)
        v = config.EXPLORE_BEFREIEN_SPEED
        return DriveCommand(v * math.cos(a), v * math.sin(a), 0.0)

    def _festgefahren(self, jetzt: float, dt: float, punkte) -> str | None:
        """Anlass, wenn er trotz Fahrbefehl nicht vorankommt, sonst None.

        Zwei unabhängige Hinweise, beide über ``EXPLORE_FEST_S``:
        das Board meldet Stillstand, oder das Lidar-Bild rundum bleibt
        gleich. Die Grenze liegt unter den 3 s, nach denen die Odometrie
        eine Stillstand meldende Messung ganz verwirft.
        """
        # Nur in einer zusammenhängenden Geradeausfahrt zählen -- nach einer
        # Drehung oder Wartezeit beginnt die Beobachtung neu.
        if jetzt - self._fest_takt > 0.3:
            self._fest_start = None
            self._fest_still_s = 0.0
        self._fest_takt = jetzt

        b = self._letzter_befehl
        if b.vx < config.EXPLORE_FEST_MIN_VX:
            self._fest_start = None
            self._fest_still_s = 0.0
            return None

        # Hinweis 1: Board-Geschwindigkeit (nur, wenn wirklich gemessen).
        if self._odometry is not None and self._odometry.misst_tatsaechlich:
            vx, vy, _ = self._odometry.geschwindigkeit()
            if math.hypot(vx, vy) < config.EXPLORE_FEST_V_MS:
                self._fest_still_s += dt
            else:
                self._fest_still_s = 0.0
            if self._fest_still_s >= config.EXPLORE_FEST_S:
                return "fährt nicht trotz Fahrbefehl"

        # Hinweis 2: Lidar-Bild rundum. Bewegt er sich, ändert sich der
        # Abstand in mehreren Richtungen; steckt er fest, nur im Rauschen.
        # Bewegt sich etwas um ihn herum, sieht es nach Fahrt aus -- dann
        # meldet er lieber nichts (das Board und die Nah-Prüfung bleiben).
        # Alle Messwerte, nicht nur die nahen: Ein Strahl, der beim Fahren
        # aus der Reichweite wandert oder neu auftaucht, ist auch Bewegung.
        # Mit einer Grenze fielen im großen Raum genau die Strahlen in
        # Fahrtrichtung weg, und die Seitenwände ändern sich nicht.
        bild = {}
        for r in _FEST_RICHTUNGEN:
            d = frei_in_richtung(punkte, r, _FEST_STRAHL_M)
            if math.isfinite(d):
                bild[r] = d
        if len(bild) < 4:
            # Zu wenig zum Vergleichen (offenes Gelände).
            self._fest_start = None
            return None
        if self._fest_start is not None:
            alt = self._fest_bild
            veraendert = sum(
                1 for r in _FEST_RICHTUNGEN
                if (r in bild) != (r in alt)
                or (r in bild and abs(bild[r] - alt[r]) > max(config.EXPLORE_FEST_WEG_M, 0.02 * bild[r]))
            )
            if veraendert < 2:
                if jetzt - self._fest_start >= config.EXPLORE_FEST_S:
                    return "kommt nicht voran (Lidar-Bild unverändert)"
                return None
        self._fest_start = jetzt
        self._fest_bild = bild
        return None

    # --- Hochgehoben -------------------------------------------------------------

    def _hochgehoben(self) -> None:
        """Alles vergessen, was am alten Ort galt -- danach beginnt er neu."""
        self._gedaechtnis.leeren()
        self._ziel_kurs = None
        self._nah_seit = None
        self._frei_seit = None
        self._fest_start = None
        self._fest_still_s = 0.0
        self._fehlversuche = 0
        self._befreiungen = 0
        self._dreh_fertig = None
        self._gedreht = 0.0
        self._setzen(HOCHGEHOBEN, "hochgehoben -- wartet, bis er wieder steht")
        log.info("Erkundung: hochgehoben (%.0f %% des Lidar-Bilds verändert)", 100 * self._hoch.anteil)

    # --- Hilfen ------------------------------------------------------------------

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
