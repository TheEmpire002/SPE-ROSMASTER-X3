"""Kartierfahrt: den Raum abfahren wie ein Saugroboter.

Läuft als eigener Modus im ``Erkunder`` (``start(modus=KARTE)``). Damit
gelten alle Sicherheitswege des Erkunders unverändert: Not-Stopp,
Eingreifen von Hand, Verbindungsverlust, Lidar-Ausfall, "etwas ganz nah".
Dieses Modul entscheidet nur, *wohin* gefahren wird.

Zwei Abschnitte:

1. **Randfahrt.** Zur nächsten Wand, dann mit der Wand rechts einmal
   rundherum, bis er wieder am Anfang ist. Danach sind die Raumgrenzen in
   der Karte, und er steht an einer Wand, parallel zu ihr.

2. **Bahnen.** Parallel zu dieser Wand hin und her, jede Bahn um
   ``MAP_FAHRT_BAHNABSTAND_M`` weiter in den Raum hinein, bis die
   gegenüberliegende Wand erreicht ist.

   Anders als ein Saugroboter **dreht er dabei nicht um**, sondern behält
   die Blickrichtung: vorwärts bis zum Hindernis, seitlich versetzen,
   rückwärts zurück. Das können die Mecanum-Räder, und es hat zwei
   Vorteile: Drehungen sind am X3 der ungenaueste Teil jeder Bewegung, und
   SLAM verliert beim Drehen am ehesten die Orientierung. Der Lidar sieht
   rundum, rückwärts fährt er also nicht blind.

3. **Weiter ins Unbekannte (04.11.2026).** Ist das Zimmer fertig -- oder
   kommt er dort nicht weiter (keine Wand in Sicht, festgefahren, Ecke) --,
   hört er nicht auf, sondern fährt zur nächsten Stelle, an der die Karte
   ins Unbekannte übergeht: offene Tür, Flur, Schatten hinter dem Sofa
   (``pfadplanung.luecke_suchen``). So wächst die Karte Raum für Raum, bis
   keine erreichbare Lücke mehr übrig ist. Dann fährt er zum Ausgangspunkt
   der Karte zurück und meldet "Karte vollständig". Dafür braucht er seine
   Lage in der wachsenden Karte (``kartenortung.py``); ohne sie endet die
   Fahrt wie früher nach dem ersten Zimmer.

Gelenkt wird nur mit dem, was der Lidar gerade sieht (Wand rechts,
Hindernis voraus) und der Lage aus der Lidar-Odometrie
(``scan_odometrie.py``) bzw. in der Karte, nie nach Zeit.
"""

from __future__ import annotations

import logging
import math
import time

from .. import config
from ..protocol import DriveCommand
from . import pfadfolger as pf
from .explorer import frei_in_richtung

log = logging.getLogger(__name__)

# Abschnitte und Schritte -- so erscheinen sie auch in der App.
ANFAHREN = "fährt zur Wand"
WEG_VON_INSEL = "verlässt ein Hindernis mitten im Raum"
AUSRICHTEN = "richtet sich an der Wand aus"
RAND = "fährt an der Wand entlang"
ECKE = "dreht in der Ecke"
UM_DIE_ECKE = "fährt um die Ecke"
BAHN = "fährt Bahnen"
VERSATZ = "versetzt zur nächsten Bahn"
LUECKE = "sucht Platz für den Versatz"
LUECKE_SUCHEN = "sucht unbekannte Stellen"
ZUR_LUECKE = "fährt zur nächsten unbekannten Stelle"
ZURUECK = "fährt zum Ausgangspunkt der Karte"
_KARTENSCHRITTE = (LUECKE_SUCHEN, ZUR_LUECKE, ZURUECK)

#: Fenster rechts neben dem Roboter, in dem die Wand gesucht wird (Meter,
#: Roboterrahmen). Vorn und hinten knapp über die Roboterlänge hinaus.
_FENSTER_X = (-0.25, 0.45)
#: So viele Punkte braucht die Wand im Fenster, sonst gilt sie als verloren.
_MIN_WANDPUNKTE = 8
#: Wie lange er sich auf der Stelle dreht, bevor er es anders versucht.
_MAX_DREHEN_S = 15.0
#: Höchstens so lange um eine Ecke herum suchen (einmal im Kreis und etwas).
_MAX_UM_DIE_ECKE_S = 30.0
#: So weit fährt er auf einer Bahn höchstens zurück, um eine Lücke für den
#: Versatz zu finden (etwa Sofalänge).
_MAX_LUECKE_M = 2.5
#: So lange (s) auf eine sichere Lage in der Karte warten, bevor die
#: Weiterfahrt aufgibt (die erste Karte kommt erst nach einigen Sekunden).
_LAGE_WARTEN_S = 30.0
#: So oft (s) beim Hinfahren prüfen, ob die Lücke inzwischen bekannt ist.
_LUECKE_PRUEFEN_S = 3.0


class Kartierfahrt:
    """Entscheidet Takt für Takt den nächsten Fahrbefehl."""

    def __init__(self, scan_odometrie, ortung=None, planer=None) -> None:
        self._odo = scan_odometrie
        # Für die Weiterfahrt ins Unbekannte: Lage in der Karte und Planer.
        self._ortung = ortung
        self._planer = planer
        self._folger = pf.Pfadfolger(tempo=config.MAP_FAHRT_TEMPO * 1.3,
                                     toleranz=config.KARTE_ZIEL_TOLERANZ_M * 2)
        self._erledigt: list[tuple[float, float]] = []
        self._luecken_angefahren = 0
        self.beginnen(time.monotonic())

    # --- Ablauf ------------------------------------------------------------------

    def beginnen(self, jetzt: float, fortsetzen: bool = False) -> None:
        """Neue Kartierfahrt -- oder, mit ``fortsetzen``, gleich weiter ins
        Unbekannte (nach einer Unterbrechung, etwa einem USB-Reset). Die
        schon angefahrenen Lücken bleiben dann gemerkt."""
        self._schritt_name = ANFAHREN
        self._text = "sucht die nächste Wand"
        self.grund: str | None = None
        self._schritt_seit = jetzt
        # Weiterfahrt ins Unbekannte
        self._auftrag = None
        self._ziel_luecke: tuple[float, float] | None = None
        self._lage_fehlt_seit: float | None = None
        self._blockiert_seit: float | None = None
        self._luecke_geprueft = jetzt
        self._fehler_in_folge = 0
        if not fortsetzen:
            self._erledigt = []
            self._luecken_angefahren = 0
        self._rand_beginn = jetzt
        self._rand_start: tuple[float, float] | None = None
        self._rand_weg = 0.0
        self._rand_weit_weg = False
        self._inseln = 0
        self._insel_geradeaus = False
        self._letzte_pos: tuple[float, float] | None = None
        # Bahnen
        self._bahn_ursprung = (0.0, 0.0)
        self._bahn_winkel = 0.0
        self._bahn_wand: float | None = None
        self._luecke_gesucht = False
        self._luecke_ab = 0.0
        self._bahn_nr = 0
        self._bahn_richtung = 1
        self._bahnen_fertig = 0
        self._leere_bahnen = 0
        self._bahn_start_s = 0.0
        if fortsetzen and self._luecken_moeglich():
            self._wechseln(LUECKE_SUCHEN, "setzt die Kartierung fort", jetzt)

    def anzeige(self) -> str:
        return f"{self._schritt_name}: {self._text}" if self._text else self._schritt_name

    def zustand(self) -> dict:
        return {
            "schritt": self._schritt_name,
            "text": self._text,
            "randweg_m": round(self._rand_weg, 1),
            "bahnen": self._bahnen_fertig,
            "luecken": self._luecken_angefahren,
            "ziel": (
                [round(self._ziel_luecke[0], 2), round(self._ziel_luecke[1], 2)]
                if self._ziel_luecke is not None and self._schritt_name in _KARTENSCHRITTE
                else None
            ),
            "weg": (
                [[round(p[0], 2), round(p[1], 2)] for p in self._folger.punkte]
                if self._schritt_name in (ZUR_LUECKE, ZURUECK) else []
            ),
        }

    def schritt(self, jetzt: float, scan, punkte) -> DriveCommand | None:
        """Ein Takt. ``None`` heißt fertig (Grund in ``self.grund``)."""
        if self.grund is not None:
            return None
        halt = DriveCommand(0.0, 0.0, 0.0)
        pose = self._odo.pose()
        alter = self._odo.alter()
        if pose is None or alter is None or alter > 1.0:
            # Ohne Lage keine Bahnen. Der Erkunder beendet die Fahrt, wenn
            # der Lidar ganz ausfällt; hier nur warten.
            self._text = "wartet auf die Lidar-Odometrie"
            return halt
        self._weg_mitzaehlen(pose)

        name = self._schritt_name
        if name == ANFAHREN:
            return self._anfahren(jetzt, scan, punkte)
        if name == WEG_VON_INSEL:
            return self._weg_von_insel(jetzt, scan, punkte)
        if name == AUSRICHTEN:
            return self._ausrichten(jetzt, scan, pose)
        if name in (RAND, ECKE, UM_DIE_ECKE):
            return self._randfahrt(jetzt, punkte, pose)
        if name in (BAHN, VERSATZ, LUECKE):
            return self._bahnen(jetzt, punkte, pose)
        if name in _KARTENSCHRITTE:
            return self._ins_unbekannte(jetzt, punkte)
        return halt

    def _wechseln(self, name: str, text: str, jetzt: float) -> None:
        if name != self._schritt_name:
            log.info("Kartierfahrt: %s (%s)", name, text)
        self._schritt_name = name
        self._text = text
        self._schritt_seit = jetzt

    def _fertig(self, grund: str) -> None:
        self.grund = grund
        log.info("Kartierfahrt fertig: %s", grund)

    def _raum_fertig(self, jetzt: float, grund: str) -> None:
        """Das Zimmer ist fertig (oder es geht dort nicht weiter): ab ins Unbekannte."""
        if not self._luecken_moeglich():
            self._fertig(grund)
            return
        log.info("Kartierfahrt: Zimmer beendet (%s) -- weiter zur nächsten Lücke", grund)
        self._wechseln(LUECKE_SUCHEN, f"Zimmer fertig ({grund})", jetzt)

    def _luecken_moeglich(self) -> bool:
        return (
            config.MAP_LUECKE_AKTIV
            and self._ortung is not None
            and self._planer is not None
            and self._ortung.karte is not None
        )

    def hindernis_melden(self, grund: str) -> bool:
        """Der Erkunder kommt nicht weiter (Fahrschutz, festgefahren).

        True: Die Kartierfahrt versucht es woanders, der Erkunder soll
        weiterfahren. False: Sie kann nichts tun, die Fahrt endet.
        """
        if not self._luecken_moeglich() or self.grund is not None:
            return False
        jetzt = time.monotonic()
        if self._schritt_name in _KARTENSCHRITTE:
            self._ziel_aufgeben(jetzt, grund)
        else:
            self._raum_fertig(jetzt, f"kommt nicht weiter: {grund}")
        return True

    # --- 3. Weiter ins Unbekannte ------------------------------------------------------

    def _ins_unbekannte(self, jetzt: float, punkte) -> DriveCommand:
        halt = DriveCommand(0.0, 0.0, 0.0)
        ortung = self._ortung
        pose = ortung.pose() if ortung is not None else None
        if pose is None or not ortung.sicher or ortung.karte is None:
            self._lage_fehlt_seit = self._lage_fehlt_seit or jetzt
            if jetzt - self._lage_fehlt_seit > _LAGE_WARTEN_S:
                self._fertig("Lage in der Karte unbekannt, Weiterfahrt nicht möglich")
                return halt
            self._text = "wartet auf die Lage in der Karte"
            return halt
        self._lage_fehlt_seit = None

        # Ein Weg wird gerade geplant: abwarten.
        if self._auftrag is not None:
            if not self._auftrag.done():
                self._text = "plant den Weg"
                return halt
            weg = self._auftrag.result()
            self._auftrag = None
            return self._weg_erhalten(jetzt, pose, weg)

        if self._schritt_name == LUECKE_SUCHEN:
            self._auftrag = self._planer.luecke(ortung.karte, (pose[0], pose[1]), self._erledigt)
            self._text = "sucht die nächste unbekannte Stelle"
            return halt

        # Unterwegs: Ist die Lücke inzwischen bekannt (der Lidar sieht weit),
        # muss er nicht bis hin.
        if self._schritt_name == ZUR_LUECKE and jetzt - self._luecke_geprueft >= _LUECKE_PRUEFEN_S:
            self._luecke_geprueft = jetzt
            if self._ziel_luecke is not None and not _noch_unbekannt(ortung.karte, self._ziel_luecke):
                self._luecke_erledigt(jetzt, "schon von unterwegs gesehen")
                return halt

        befehl, zustand = self._folger.schritt(pose, punkte)
        if zustand == pf.ANGEKOMMEN:
            if self._schritt_name == ZURUECK:
                self._fertig(
                    f"Karte vollständig ({self._luecken_angefahren} Lücken angefahren), "
                    "zurück am Ausgangspunkt"
                )
                return halt
            self._luecke_erledigt(jetzt, "angekommen")
            return halt
        if zustand in (pf.BLOCKIERT, pf.ABGEKOMMEN):
            self._blockiert_seit = self._blockiert_seit or jetzt
            if zustand == pf.ABGEKOMMEN or jetzt - self._blockiert_seit > config.KARTE_BLOCKIERT_S:
                self._ziel_aufgeben(
                    jetzt, "Weg versperrt" if zustand == pf.BLOCKIERT else "vom Weg abgekommen"
                )
            return halt
        self._blockiert_seit = None
        self._text = f"noch {self._folger.rest_m:.1f} m, {self._luecken_angefahren} Lücken angefahren"
        return befehl

    def _weg_erhalten(self, jetzt: float, pose, weg) -> DriveCommand:
        halt = DriveCommand(0.0, 0.0, 0.0)
        if self._schritt_name == ZURUECK:
            if weg is None:
                self._fertig("Karte vollständig, aber kein Weg zurück zum Ausgangspunkt")
                return halt
            self._folger.setzen(weg.punkte, von=pose, ziel_winkel=self._ortung.start[2])
            self._text = f"Rückweg {weg.laenge:.1f} m"
            return halt
        if weg is None:
            # Keine erreichbare Lücke mehr: Die Karte ist fertig.
            start = self._ortung.start
            weit = math.hypot(start[0] - pose[0], start[1] - pose[1]) > config.MAP_LUECKE_NAH_M
            if config.MAP_FAHRT_ZURUECK and weit:
                self._wechseln(ZURUECK, "keine unbekannte Stelle mehr", jetzt)
                self._ziel_luecke = None
                self._folger.setzen([])
                self._auftrag = self._planer.weg(
                    self._ortung.karte, (pose[0], pose[1]), (start[0], start[1])
                )
                return halt
            self._fertig(f"Karte vollständig ({self._luecken_angefahren} Lücken angefahren)")
            return halt
        self._ziel_luecke = weg.luecke
        self._folger.setzen(weg.punkte, von=pose)
        self._blockiert_seit = None
        self._luecke_geprueft = jetzt
        self._wechseln(ZUR_LUECKE, f"Lücke in {weg.laenge:.1f} m Weg", jetzt)
        return halt

    def _luecke_erledigt(self, jetzt: float, wie: str) -> None:
        if self._ziel_luecke is not None:
            self._erledigt.append(self._ziel_luecke)
        self._luecken_angefahren += 1
        self._fehler_in_folge = 0
        self._wechseln(LUECKE_SUCHEN, f"Lücke {self._luecken_angefahren}: {wie}", jetzt)

    def _ziel_aufgeben(self, jetzt: float, grund: str) -> None:
        """Diese Lücke ist nicht erreichbar: merken und die nächste nehmen."""
        self._auftrag = None
        if self._schritt_name == ZURUECK:
            self._fertig(f"Karte vollständig, Rückweg zum Ausgangspunkt versperrt ({grund})")
            return
        if self._ziel_luecke is not None:
            self._erledigt.append(self._ziel_luecke)
        self._fehler_in_folge += 1
        if self._fehler_in_folge > config.MAP_LUECKE_MAX_FEHLER:
            self._fertig(
                f"{self._fehler_in_folge} unbekannte Stellen in Folge nicht erreicht ({grund})"
            )
            return
        log.info("Kartierfahrt: Lücke aufgegeben (%s)", grund)
        self._wechseln(LUECKE_SUCHEN, f"Lücke aufgegeben: {grund}", jetzt)

    # --- 1. Zur Wand ---------------------------------------------------------------

    def _anfahren(self, jetzt: float, scan, punkte) -> DriveCommand:
        richtung, abstand = _naechste_wand(scan)
        if richtung is None:
            self._raum_fertig(jetzt, "keine Wand in Sicht")
            return DriveCommand(0.0, 0.0, 0.0)

        vorn = frei_in_richtung(punkte, 0.0, config.EXPLORE_HALF_WIDTH_M)
        if vorn <= config.MAP_FAHRT_WANDABSTAND_M + 0.05:
            self._wechseln(AUSRICHTEN, "Wand voraus, dreht nach links", jetzt)
            return DriveCommand(0.0, 0.0, 0.0)

        # Erst die Wand nach vorn drehen, dann geradeaus darauf zu. Gedreht
        # wird, bis der Lidar sie vorn sieht -- nicht nach Zeit.
        if abs(richtung) > 10.0:
            if jetzt - self._schritt_seit > _MAX_DREHEN_S:
                self._raum_fertig(jetzt, "konnte sich nicht zur Wand drehen")
                return DriveCommand(0.0, 0.0, 0.0)
            self._text = f"nächste Wand {abstand:.1f} m bei {richtung:+.0f}°"
            return DriveCommand(0.0, 0.0, math.copysign(config.MAP_FAHRT_DREH_TEMPO, richtung))
        self._schritt_seit = jetzt
        self._text = f"Wand in {vorn:.1f} m"
        omega = 0.02 * richtung  # klein nachlenken, die Wand bleibt vorn
        return DriveCommand(self._tempo(vorn, config.MAP_FAHRT_WANDABSTAND_M), 0.0, omega)

    def _weg_von_insel(self, jetzt: float, scan, punkte) -> DriveCommand:
        """Der Insel den Rücken zudrehen und geradeaus zur nächsten Wand.

        Geradeaus und nicht "zur nächsten Wand": Die nächste wäre wieder
        die Insel, an der er gerade steht.
        """
        richtung, _ = _naechste_wand(scan)
        if not self._insel_geradeaus and richtung is not None and abs(richtung) < 150.0:
            if jetzt - self._schritt_seit > _MAX_DREHEN_S:
                self._raum_fertig(jetzt, "kam von einem Hindernis nicht los")
                return DriveCommand(0.0, 0.0, 0.0)
            # Die Insel nach hinten drehen, auf kürzestem Weg.
            return DriveCommand(0.0, 0.0, -math.copysign(config.MAP_FAHRT_DREH_TEMPO, richtung))
        # Einmal abgewandt, nur noch geradeaus -- auch wenn danach etwas
        # anderes das Nächste ist.
        self._insel_geradeaus = True
        self._text = "fährt geradeaus"
        vorn = frei_in_richtung(punkte, 0.0, config.EXPLORE_HALF_WIDTH_M)
        if vorn <= config.MAP_FAHRT_WANDABSTAND_M + 0.05:
            self._wechseln(AUSRICHTEN, "Wand voraus, dreht nach links", jetzt)
            return DriveCommand(0.0, 0.0, 0.0)
        return DriveCommand(self._tempo(vorn, config.MAP_FAHRT_WANDABSTAND_M), 0.0, 0.0)

    def _ausrichten(self, jetzt: float, scan, pose) -> DriveCommand:
        """Nach links drehen, bis die Wand rechts liegt."""
        richtung, _ = _naechste_wand(scan)
        if richtung is not None and -100.0 <= richtung <= -80.0:
            self._wechseln(RAND, "Wand rechts", jetzt)
            self._rand_beginn = jetzt
            self._rand_start = (pose[0], pose[1])
            self._rand_weg = 0.0
            self._rand_weit_weg = False
            return DriveCommand(0.0, 0.0, 0.0)
        if jetzt - self._schritt_seit > _MAX_DREHEN_S:
            self._raum_fertig(jetzt, "konnte sich nicht an der Wand ausrichten")
            return DriveCommand(0.0, 0.0, 0.0)
        return DriveCommand(0.0, 0.0, config.MAP_FAHRT_DREH_TEMPO)

    # --- 2. Randfahrt ----------------------------------------------------------------

    def _randfahrt(self, jetzt: float, punkte, pose) -> DriveCommand:
        if self._rund_herum(pose):
            if self._rand_weg < config.MAP_FAHRT_INSEL_M and self._inseln < 3:
                # Zu kurz für einen Raum: Das war eine Säule, ein Sofa oder
                # ein Tisch mitten im Raum, nicht die Wand. Weg davon und
                # die Wand suchen.
                self._inseln += 1
                self._insel_geradeaus = False
                self._wechseln(
                    WEG_VON_INSEL, f"Runde nur {self._rand_weg:.1f} m, war keine Wand", jetzt
                )
                self._rand_start = None
                return DriveCommand(0.0, 0.0, 0.0)
            return self._bahnen_beginnen(jetzt, pose, punkte, "einmal rundherum")
        if jetzt - self._rand_beginn > config.MAP_FAHRT_RAND_MAX_S:
            return self._bahnen_beginnen(jetzt, pose, punkte, "Randfahrt dauerte zu lange")

        abstand = config.MAP_FAHRT_WANDABSTAND_M
        vorn = frei_in_richtung(punkte, 0.0, config.EXPLORE_HALF_WIDTH_M)
        wand = _wand_rechts(punkte)

        # Innenecke: vorn wird es eng. Auf der Stelle nach links drehen, bis
        # vorn wieder Platz ist -- dann liegt die neue Wand rechts.
        if self._schritt_name == ECKE or vorn < abstand + 0.1:
            if self._schritt_name != ECKE:
                self._wechseln(ECKE, f"vorn {vorn:.2f} m", jetzt)
            if vorn >= abstand + 0.45 and wand is not None and abs(wand[0]) < 20.0:
                self._wechseln(RAND, "weiter an der Wand", jetzt)
            elif jetzt - self._schritt_seit > _MAX_DREHEN_S:
                return self._bahnen_beginnen(jetzt, pose, punkte, "kam aus einer Ecke nicht heraus")
            else:
                return DriveCommand(0.0, 0.0, config.MAP_FAHRT_DREH_TEMPO)

        if wand is None:
            # Außenecke oder Türöffnung: Die Wand rechts endet. Im Bogen
            # nach rechts um ihr Ende herum, bis sie wieder auftaucht.
            if self._schritt_name != UM_DIE_ECKE:
                self._wechseln(UM_DIE_ECKE, "Wand rechts endet", jetzt)
            if jetzt - self._schritt_seit > _MAX_UM_DIE_ECKE_S:
                return self._bahnen_beginnen(jetzt, pose, punkte, "Wand verloren")
            vx = config.MAP_FAHRT_TEMPO * 0.6
            return DriveCommand(vx, 0.0, -vx / abstand)

        if self._schritt_name != RAND:
            self._wechseln(RAND, "Wand wieder rechts", jetzt)
        winkel, entfernung = wand
        # Parallel bleiben: so weit drehen, wie die Wand schräg liegt.
        omega = 0.03 * winkel
        # Abstand halten mit den Mecanum-Rädern, ohne zu drehen: zu weit weg
        # heißt nach rechts (negatives vy).
        vy = max(-0.08, min(0.08, -0.8 * (entfernung - abstand)))
        self._text = f"Wand {entfernung:.2f} m, {self._rand_weg:.1f} m gefahren"
        return DriveCommand(self._tempo(vorn, abstand + 0.1), vy, max(-0.5, min(0.5, omega)))

    def _rund_herum(self, pose) -> bool:
        """Wieder am Anfang der Randfahrt angekommen?

        Erst zählen, wenn er weit genug weg war -- sonst wäre er gleich zu
        Beginn schon "zurück".
        """
        if self._rand_start is None:
            return False
        weg_vom_start = math.hypot(pose[0] - self._rand_start[0], pose[1] - self._rand_start[1])
        if weg_vom_start > 2 * config.MAP_FAHRT_RUNDE_M:
            self._rand_weit_weg = True
        return (
            self._rand_weit_weg
            and self._rand_weg >= config.MAP_FAHRT_RUNDE_MIN_M
            and weg_vom_start < config.MAP_FAHRT_RUNDE_M
        )

    def _weg_mitzaehlen(self, pose) -> None:
        pos = (pose[0], pose[1])
        if self._letzte_pos is not None and self._schritt_name in (RAND, ECKE, UM_DIE_ECKE):
            self._rand_weg += math.hypot(pos[0] - self._letzte_pos[0], pos[1] - self._letzte_pos[1])
        self._letzte_pos = pos

    # --- 3. Bahnen ---------------------------------------------------------------------

    def _bahnen_beginnen(self, jetzt: float, pose, punkte, anlass: str) -> DriveCommand:
        """Bahnen parallel zur Wand, an der er gerade steht."""
        log.info("Kartierfahrt: Randfahrt beendet (%s, %.1f m)", anlass, self._rand_weg)
        self._bahn_ursprung = (pose[0], pose[1])
        self._bahn_winkel = pose[2]
        self._bahn_wand = hauptrichtung(punkte)
        # Bahn 0 ist die Randfahrt selbst. Die Wand liegt rechts, also geht
        # es nach links in den Raum hinein. Beim Versatz kehrt sich die
        # Richtung um: Bahn 1 vorwärts, Bahn 2 rückwärts und so fort.
        self._bahn_nr = 0
        self._bahn_richtung = -1
        self._bahnen_fertig = 0
        self._leere_bahnen = 0
        self._wechseln(VERSATZ, f"Randfahrt fertig ({anlass}), Bahn 1", jetzt)
        return DriveCommand(0.0, 0.0, 0.0)

    def _bahnen(self, jetzt: float, punkte, pose) -> DriveCommand:
        entlang, quer = self._bahn_koordinaten(pose)
        # Blickrichtung festhalten, damit die Bahnen parallel bleiben. Am
        # liebsten an den Wänden: Liegen sie im selben Winkel wie zu Beginn,
        # stimmt die Richtung, egal wie weit die Odometrie inzwischen
        # gedriftet ist. Nur ohne klare Wandrichtung zählt die Odometrie.
        jetzt_wand = hauptrichtung(punkte) if self._bahn_wand is not None else None
        if jetzt_wand is not None:
            fehler = math.radians((jetzt_wand - self._bahn_wand + 45.0) % 90.0 - 45.0)
        else:
            fehler = _normalisieren(self._bahn_winkel - pose[2])
        omega = max(-0.4, min(0.4, 1.5 * fehler))

        ziel = (self._bahn_nr + 1) * config.MAP_FAHRT_BAHNABSTAND_M
        links = frei_in_richtung(punkte, 90.0, config.MAP_FAHRT_HALBE_LAENGE_M)

        if self._schritt_name == LUECKE:
            # Auf der Bahn zurück, bis links genug Platz für den Versatz ist.
            richtung = -self._bahn_richtung
            frei = frei_in_richtung(punkte, 0.0 if richtung > 0 else 180.0,
                                    config.EXPLORE_HALF_WIDTH_M)
            if links >= (ziel - quer) + config.EXPLORE_STOP_M:
                self._wechseln(VERSATZ, f"Lücke gefunden, Bahn {self._bahn_nr + 1}", jetzt)
                return DriveCommand(0.0, 0.0, omega)
            if frei < config.EXPLORE_STOP_M or abs(entlang - self._luecke_ab) > _MAX_LUECKE_M:
                self._raum_fertig(
                    jetzt, f"seitlich kein Platz mehr, {self._bahnen_fertig} Bahnen gefahren"
                )
                return DriveCommand(0.0, 0.0, 0.0)
            soll_quer = self._bahn_nr * config.MAP_FAHRT_BAHNABSTAND_M
            vy = max(-0.08, min(0.08, -1.0 * (quer - soll_quer)))
            self._text = f"links {links:.2f} m frei, sucht weiter"
            return DriveCommand(richtung * self._tempo(frei, config.EXPLORE_STOP_M), vy, omega)

        if self._schritt_name == VERSATZ:
            if links < config.EXPLORE_STOP_M:
                if not self._luecke_gesucht:
                    # Links ist es eng -- vielleicht nur ein Möbel, an dem die
                    # Bahn endete, und nicht die gegenüberliegende Wand. Ein
                    # Stück auf der Bahn zurück und dort versetzen.
                    self._luecke_gesucht = True
                    self._luecke_ab = entlang
                    self._wechseln(LUECKE, "links zu eng, fährt ein Stück zurück", jetzt)
                    return DriveCommand(0.0, 0.0, omega)
                self._raum_fertig(
                    jetzt, f"gegenüberliegende Wand erreicht, {self._bahnen_fertig} Bahnen gefahren"
                )
                return DriveCommand(0.0, 0.0, 0.0)
            dauer_max = 3.0 * config.MAP_FAHRT_BAHNABSTAND_M / config.MAP_FAHRT_SEITE_TEMPO
            if quer >= ziel - 0.03 or jetzt - self._schritt_seit > dauer_max:
                self._bahn_nr += 1
                self._luecke_gesucht = False
                # Nach einer Lückensuche zeigt er schon in die Gegenrichtung
                # der letzten Bahn; die neue Bahn läuft dann in diese weiter.
                self._bahn_richtung = -self._bahn_richtung
                self._bahn_start_s = entlang
                self._wechseln(
                    BAHN, f"Bahn {self._bahn_nr} "
                    + ("vorwärts" if self._bahn_richtung > 0 else "rückwärts"), jetzt,
                )
                return DriveCommand(0.0, 0.0, omega)
            self._text = f"noch {max(0.0, ziel - quer):.2f} m zur Bahn {self._bahn_nr + 1}"
            tempo = config.MAP_FAHRT_SEITE_TEMPO
            if links < config.EXPLORE_SLOW_M:
                tempo *= 0.5
            return DriveCommand(0.0, tempo, omega)

        # Auf der Bahn: geradeaus (vor- oder rückwärts), seitlich auf Linie halten.
        richtung_grad = 0.0 if self._bahn_richtung > 0 else 180.0
        frei = frei_in_richtung(punkte, richtung_grad, config.EXPLORE_HALF_WIDTH_M)
        if frei < config.EXPLORE_STOP_M:
            laenge = abs(entlang - self._bahn_start_s)
            self._bahnen_fertig += 1
            # Lauter Bahnen ohne Länge: Er steckt zwischen Hindernissen fest.
            self._leere_bahnen = self._leere_bahnen + 1 if laenge < 0.2 else 0
            if self._leere_bahnen >= 3:
                self._raum_fertig(jetzt, "kein Platz mehr für Bahnen")
                return DriveCommand(0.0, 0.0, 0.0)
            self._wechseln(VERSATZ, f"Bahn {self._bahn_nr} zu Ende ({laenge:.1f} m)", jetzt)
            return DriveCommand(0.0, 0.0, omega)
        soll_quer = self._bahn_nr * config.MAP_FAHRT_BAHNABSTAND_M
        vy = max(-0.08, min(0.08, -1.0 * (quer - soll_quer)))
        self._text = f"Bahn {self._bahn_nr}, frei {min(frei, 9.9):.1f} m"
        vx = self._bahn_richtung * self._tempo(frei, config.EXPLORE_STOP_M)
        return DriveCommand(vx, vy, omega)

    def _bahn_koordinaten(self, pose) -> tuple[float, float]:
        """(entlang, quer) relativ zum Anfang der Bahnen; quer > 0 = in den Raum."""
        dx = pose[0] - self._bahn_ursprung[0]
        dy = pose[1] - self._bahn_ursprung[1]
        c, s = math.cos(self._bahn_winkel), math.sin(self._bahn_winkel)
        return c * dx + s * dy, -s * dx + c * dy

    # --- Hilfen ------------------------------------------------------------------------

    @staticmethod
    def _tempo(frei: float, halt_bei: float) -> float:
        """Reisetempo, vor Hindernissen gedrosselt (nicht unter 40 %)."""
        spanne = max(config.EXPLORE_SLOW_M - halt_bei, 0.1)
        anteil = min(1.0, max(0.4, (frei - halt_bei) / spanne))
        return config.MAP_FAHRT_TEMPO * anteil


def _wand_rechts(punkte) -> tuple[float, float] | None:
    """Wand rechts neben dem Roboter: (Winkel in Grad, Abstand in Metern).

    Eine Gerade durch die Punkte im Fenster rechts (Hauptachse). Der Winkel
    ist ihre Neigung gegenüber der Fahrtrichtung: positiv heißt, sie kommt
    nach vorn hin näher -- dann muss er nach links drehen, um parallel zu
    bleiben. Ein einzelner Strahl wäre einfacher, verwechselt aber jedes
    Stuhlbein mit einer Wand.
    """
    import numpy as np

    if len(punkte) == 0:
        return None
    x, y = punkte[:, 0], punkte[:, 1]
    weit = config.MAP_FAHRT_WANDABSTAND_M + 0.6
    im_fenster = (x > _FENSTER_X[0]) & (x < _FENSTER_X[1]) & (y < -0.05) & (y > -weit)
    if int(im_fenster.sum()) < _MIN_WANDPUNKTE:
        return None
    p = punkte[im_fenster]
    mitte = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - mitte, full_matrices=False)
    dx, dy = vt[0]
    if dx < 0:
        dx, dy = -dx, -dy
    winkel = math.degrees(math.atan2(dy, dx))
    if abs(winkel) > 60.0:
        # Steht eher quer als längs -- das ist keine Wand neben ihm.
        return None
    # Senkrechter Abstand der Geraden zur Roboter-Mitte.
    abstand = abs(-dy * mitte[0] + dx * mitte[1])
    return winkel, abstand


def _noch_unbekannt(karte, punkt) -> bool:
    """Liegt in der Nähe dieser Lücke noch Unbekanntes?"""
    import numpy as np

    r = config.MAP_LUECKE_NAH_M
    winkel = np.linspace(0.0, 2 * math.pi, 16, endpoint=False)
    xs = np.concatenate(([punkt[0]], punkt[0] + r * np.cos(winkel)))
    ys = np.concatenate(([punkt[1]], punkt[1] + r * np.sin(winkel)))
    return bool(karte.unbekannt_bei(xs, ys).any())


def _normalisieren(winkel: float) -> float:
    return math.atan2(math.sin(winkel), math.cos(winkel))


def hauptrichtung(punkte) -> float | None:
    """Richtung der Wände im Roboterrahmen, in Grad modulo 90 (0..90).

    Zimmerwände stehen fast immer rechtwinklig zueinander. Ihre gemeinsame
    Richtung lässt sich deshalb aus jeder einzelnen Aufnahme ablesen -- ohne
    etwas aufzusummieren, also ohne Drift. Dreht sich der Roboter um +5°,
    liegen die Wände für ihn um 5° anders.

    Verfahren: Richtungen kurzer Stücke zwischen benachbarten Punkten,
    gewichtet nach Länge, in ein Histogramm über 90°; die Spitze ist die
    Hauptrichtung. Stücke über acht Punkte statt zwei, sonst wackelt jede
    Richtung mit dem Messrauschen des A1: Mit simulierten 3 cm Rauschen lag
    das Ergebnis so höchstens 1° daneben, über vier Punkte gab es oft gar
    keins. Längere Stücke schnitten dagegen Ecken ab. None, wenn keine
    klare Richtung zu sehen ist (rundes Zimmer, nur Möbel in Sicht).
    """
    import numpy as np

    if len(punkte) < 30:
        return None
    reihenfolge = np.argsort(np.arctan2(punkte[:, 1], punkte[:, 0]))
    p = punkte[reihenfolge]
    d = p[8:] - p[:-8]
    laenge = np.hypot(d[:, 0], d[:, 1])
    gut = (laenge > 0.1) & (laenge < 1.0)
    if int(gut.sum()) < 15:
        return None
    laenge = laenge[gut]
    winkel = np.degrees(np.arctan2(d[gut, 1], d[gut, 0])) % 90.0
    hist = np.zeros(90)
    np.add.at(hist, winkel.astype(int) % 90, laenge)
    glatt = hist + np.roll(hist, 1) + np.roll(hist, -1)
    spitze = int(np.argmax(glatt))
    if glatt[spitze] < 0.2 * laenge.sum():
        return None
    # Verfeinern: Mittel der Stücke nahe der Spitze, über die 0/90-Grenze hinweg.
    abweichung = (winkel - spitze + 45.0) % 90.0 - 45.0
    nah = np.abs(abweichung) <= 3.0
    return float((spitze + np.average(abweichung[nah], weights=laenge[nah])) % 90.0)


def _naechste_wand(scan) -> tuple[float | None, float]:
    """Richtung (Grad, -180..180) und Abstand des nächsten festen Hindernisses.

    In 5°-Sektoren, jeweils der zweitnächste Punkt: Ein einzelner Fehlstrahl
    soll nicht als "nächste Wand" gelten.
    """
    sektoren: dict[int, list[float]] = {}
    for w, r in zip(scan.winkel, scan.entfernungen):
        sektoren.setdefault(int(w // 5) % 72, []).append(r)
    bester: tuple[float | None, float] = (None, float("inf"))
    for nr, werte in sektoren.items():
        if len(werte) < 2:
            continue
        werte.sort()
        if werte[1] < bester[1]:
            mitte = nr * 5 + 2.5
            bester = (mitte - 360.0 if mitte > 180.0 else mitte, werte[1])
    return bester
