"""Auf Handzeichen reagieren: folgen, herkommen, weggehen, anhalten.

Ist der Interaktionsmodus an, schaut der Roboter nach Menschen und wartet
auf eine Geste (``BEREIT``). Erkannte Gesten schalten das Verhalten um:

``folgen``     im Abstand ``FOLGEN_ABSTAND_M`` hinter der Person bleiben
``herkommen``  bis ``HERKOMMEN_ABSTAND_M`` heranfahren, dann stehen
``weggehen``   ``WEGGEHEN_STRECKE_M`` rückwärts, dann stehen
``stopp``      sofort anhalten, zurück in BEREIT

Woher welche Größe kommt:

* **Richtung** aus der Kamera (Winkel der Person zur Bildmitte). Die Kamera
  weiß, *wer* ein Mensch ist, kann aber keine Entfernung messen.
* **Abstand** aus dem Lidar, genau in dieser Richtung. Der Lidar misst
  zentimetergenau, weiß aber nicht, was er sieht. Zusammen ergibt sich
  "Mensch, 15° links, 1,3 m".

Sicherheit wie beim Erkunder: Jeder Befehl geht über den
``SafetyWatchdog``; vorwärts nur, wenn der Fahrschlauch frei ist, rückwärts
nur, wenn hinten frei ist; rückt etwas ganz nah heran, steht er; ohne
frische Lidar-Daten steht er. Bleibt diese Schleife hängen, hält der
Watchdog nach 0,5 s an.

Sanft fahren (29.10.2026): Sprünge im Fahrbefehl ziehen Stromspitzen, die
am X3 USB-Resets auslösen. Daher wird der Kamerawinkel geglättet, Drehrate
und Geschwindigkeit wachsen stetig mit dem Fehler (keine harten Grenzen),
kurze Aussetzer der Erkennung rollt er aus, statt hart zu bremsen, und der
Watchdog begrenzt zusätzlich die Beschleunigung (``RAMPE_*``).

In BEREIT schickt die Schleife **keine** Befehle -- der Roboter steht, und
nichts hält den Antrieb unnötig wach. Handsteuerung aus der App beendet den
ganzen Interaktionsmodus (wie beim Erkunder): Wer selbst lenkt, will nicht,
dass eine Geste dazwischenfährt.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from typing import TYPE_CHECKING

from .. import config
from ..navigation.explorer import frei_in_richtung
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import gesten as gs

if TYPE_CHECKING:  # nur für die Typangaben -- vermeidet Ringimporte
    from ..sensors.lidar import LidarService
    from ..sensors.pose import Posenerkennung

log = logging.getLogger(__name__)

#: Modi, so wie sie in der App erscheinen (state.interaktion.modus).
BEREIT = "bereit"
FOLGEN = "folgen"
HERKOMMEN = "herkommen"
WEGGEHEN = "weggehen"
SUCHEN = "sucht"

#: Welche Geste welchen Modus auslöst.
_GESTE_ZU_MODUS = {
    gs.FOLGEN: FOLGEN,
    gs.HERKOMMEN: HERKOMMEN,
    gs.WEGGEHEN: WEGGEHEN,
    gs.STOPP: BEREIT,
}

#: Text für Meldungen in der App.
_GESTE_TEXT = {
    gs.FOLGEN: "Folgen",
    gs.STOPP: "Stopp",
    gs.HERKOMMEN: "Herkommen",
    gs.WEGGEHEN: "Weggehen",
}

#: So lange (s) bleibt die letzte Geste in der Anzeige stehen.
_GESTE_ANZEIGE_S = 3.0

#: Steht die Person so genau (°) vor ihm, gilt "herkommen" als geschafft.
_ZIEL_WINKEL = 8.0


class Interaktion:
    """Interaktionsmodus. Bedienung wie ``Erkunder`` (start/abbrechen/zustand)."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        lidar: "LidarService | None",
        pose: "Posenerkennung | None",
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._pose = pose
        self._gesten = gs.Gestenerkennung()
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None
        self._meldungen: list[str] = []

        # Für die Anzeige -- von der Schleife geschrieben, vom Status-Takt
        # gelesen (beides im selben Event-Loop, kein Lock nötig).
        self._modus = BEREIT
        self._text = ""
        self._winkel: float | None = None
        self._abstand: float | None = None
        self._anzahl = 0

        self._modus_seit = 0.0
        self._gesehen = 0.0  # wann die Zielperson zuletzt zu sehen war
        self._letzte_seite = 1.0  # +1 links, -1 rechts -- dorthin wird gesucht
        self._suchen_fuer = FOLGEN  # in diesen Modus nach erfolgreicher Suche
        self._letztes_bild = -1.0
        self._ohne_lidar_seit: float | None = None

        # Sanft fahren
        self._winkel_glatt: float | None = None
        self._glatt_bild = -1.0
        self._letzter_befehl: DriveCommand | None = None
        self._abstand_gut: float | None = None
        self._abstand_gut_zeit = -1e9
        self._nah_zuletzt = -1e9

    # --- Bedienung ---------------------------------------------------------------

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        return self._grund

    @property
    def verfuegbar(self) -> bool:
        """Braucht Posenerkennung (wer, welche Geste) und Lidar (wie weit)."""
        return self._lidar is not None and self._pose is not None and self._pose.verfuegbar

    @property
    def modus(self) -> str | None:
        return self._modus if self.aktiv else None

    def start(self, modus: str = BEREIT) -> bool:
        """Interaktionsmodus einschalten. False, wenn schon an oder nicht möglich."""
        if self.aktiv or not self.verfuegbar:
            return False
        self._grund = None
        self._task = asyncio.create_task(self._laufen(modus))
        return True

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        """Sofort beenden und anhalten."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._grund = grund
            log.info("Interaktion %s", grund)
        self._task = None

    def folgen(self) -> bool:
        """Folgen per App-Knopf statt per Geste. Schaltet den Modus nötigenfalls ein."""
        if not self.aktiv:
            # Den Modus gleich mitgeben -- die Schleife startet erst im
            # nächsten Durchlauf des Event-Loops und würde ihn sonst auf
            # BEREIT zurücksetzen.
            return self.start(FOLGEN)
        self._wechseln(FOLGEN, "folgt (per App)", time.monotonic())
        return True

    def folgen_stop(self) -> None:
        """Verhalten beenden, Interaktionsmodus bleibt an."""
        if self.aktiv and self._modus != BEREIT:
            self._wechseln(BEREIT, "angehalten (per App)", time.monotonic())

    def meldungen_abholen(self) -> list[str]:
        """Ereignisse seit dem letzten Aufruf -- für Hinweise in der App."""
        neu, self._meldungen = self._meldungen, []
        return neu

    def zustand(self) -> dict:
        """Für state.interaktion in der App."""
        aktiv = self.aktiv
        jetzt = time.monotonic()
        geste = None
        if self._gesten.zuletzt_zeit is not None and jetzt - self._gesten.zuletzt_zeit < _GESTE_ANZEIGE_S:
            geste = self._gesten.zuletzt
        person = None
        if aktiv and self._winkel is not None:
            person = {
                "winkel": round(self._winkel, 1),
                "abstand": (
                    round(self._abstand, 2)
                    if self._abstand is not None and math.isfinite(self._abstand)
                    else None
                ),
            }
        return {
            "aktiv": aktiv,
            "verfuegbar": self.verfuegbar,
            "modus": self._modus if aktiv else None,
            "text": self._text if aktiv else None,
            "person": person,
            "personen": self._anzahl if aktiv else 0,
            "geste": geste if aktiv else None,
            "grund": self._grund,
        }

    # --- Schleife ----------------------------------------------------------------

    async def _laufen(self, modus: str) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        self._gesten = gs.Gestenerkennung()
        self._winkel = self._abstand = None
        self._letztes_bild = -1.0
        self._ohne_lidar_seit = None
        self._winkel_glatt, self._glatt_bild = None, -1.0
        self._abstand_gut, self._abstand_gut_zeit = None, -1e9
        self._nah_zuletzt = -1e9
        self._modus = BEREIT
        self._wechseln(
            modus, "folgt (per App)" if modus == FOLGEN else "wartet auf ein Handzeichen", time.monotonic()
        )
        if self._pose is not None:
            self._pose.aktivieren(True)
        log.info("Interaktion gestartet")

        try:
            while True:
                befehl = self._schritt(time.monotonic())
                # None heißt: nichts schicken (BEREIT). Angehalten wurde beim
                # Wechsel nach BEREIT schon.
                if befehl is not None:
                    self._watchdog.submit(befehl)
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler in der Interaktion"
            log.exception("Interaktion abgebrochen")
        finally:
            if self._pose is not None:
                self._pose.aktivieren(False)

    def _schritt(self, jetzt: float) -> DriveCommand | None:
        """Ein Takt: Person und Geste ansehen, Fahrbefehl zurückgeben."""
        halt = DriveCommand(0.0, 0.0, 0.0)

        ziel = self._ziel_waehlen()
        self._geste_pruefen(ziel, jetzt)

        if ziel is not None:
            self._glaetten(ziel, jetzt)
            self._gesehen = jetzt
            self._winkel = ziel["winkel"]
            if abs(self._winkel) > 2.0:
                self._letzte_seite = 1.0 if self._winkel > 0 else -1.0

        if self._modus == BEREIT:
            self._abstand = self._abstand_zu(ziel)
            return None

        if self._modus in (HERKOMMEN, WEGGEHEN) and jetzt - self._modus_seit > config.INTERAKTION_VERHALTEN_MAX_S:
            return self._wechseln(BEREIT, f"{self._modus}: Zeit abgelaufen", jetzt, melden=True)

        # Ohne frische Lidar-Runde wird nicht gefahren. Kurze Aussetzer
        # (USB-Reset) abwarten, bei längeren das Verhalten beenden.
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or jetzt - scan.zeit > config.EXPLORE_SCAN_MAX_AGE_S:
            if self._ohne_lidar_seit is None:
                self._ohne_lidar_seit = jetzt
                log.warning("Interaktion: keine frischen Lidar-Daten -- halte an")
            if jetzt - self._ohne_lidar_seit > config.EXPLORE_LIDAR_WAIT_S:
                return self._wechseln(BEREIT, "Lidar liefert keine Daten", jetzt, melden=True)
            self._text = "keine frischen Lidar-Daten"
            self._letzter_befehl = None
            return halt
        self._ohne_lidar_seit = None

        punkte = scan.punkte()
        self._abstand = self._abstand_zu(ziel, scan)
        if self._abstand is not None and math.isfinite(self._abstand):
            self._abstand_gut, self._abstand_gut_zeit = self._abstand, jetzt
        elif ziel is not None and jetzt - self._abstand_gut_zeit < config.FOLGEN_HALTEN_S:
            # In Richtung der Person kurz kein Lidar-Punkt (Beine zwischen
            # zwei Strahlen) -- den letzten guten Wert nehmen, statt
            # stehenzubleiben und gleich wieder anzufahren.
            self._abstand = self._abstand_gut

        if self._modus == WEGGEHEN:
            return self._weggehen(jetzt, ziel, punkte)

        # Folgen, Herkommen, Suchen: alles, was nach vorn oder im Stand
        # dreht. Ganz nah etwas -> gar nicht bewegen (die Ecken schwenken
        # beim Drehen aus). Erst ab drei Punkten, ein Fehlstrahl zählt nicht.
        # Nach "nah" noch kurz stehen bleiben, damit er an der Grenze nicht
        # zwischen Halt und Fahrt flackert.
        if self._nah(punkte):
            self._nah_zuletzt = jetzt
        if jetzt - self._nah_zuletzt < config.FOLGEN_NAH_HALTEN_S:
            self._text = "etwas sehr nah -- warte"
            self._letzter_befehl = None
            return halt

        if self._modus == SUCHEN:
            return self._suchen(jetzt, ziel)

        if ziel is None:
            if jetzt - self._gesehen > config.FOLGEN_VERLOREN_S:
                self._suchen_fuer = self._modus
                return self._wechseln(SUCHEN, "Person verloren -- suche", jetzt, melden=True)
            auslauf = self._ausrollen(jetzt, punkte)
            if auslauf is not None:
                return auslauf
            self._text = "Person kurz nicht zu sehen -- warte"
            return halt

        soll = config.FOLGEN_ABSTAND_M if self._modus == FOLGEN else config.HERKOMMEN_ABSTAND_M
        if (
            self._modus == HERKOMMEN
            and self._abstand is not None
            and self._abstand <= soll + config.FOLGEN_TOTZONE_M
            and abs(self._winkel) < _ZIEL_WINKEL
        ):
            return self._wechseln(BEREIT, "bin da", jetzt, melden=True)
        befehl = self._annaehern(punkte, soll, rueckwaerts=self._modus == FOLGEN)
        self._letzter_befehl = befehl
        return befehl

    # --- Verhalten ---------------------------------------------------------------

    def _annaehern(self, punkte, soll: float, rueckwaerts: bool) -> DriveCommand:
        """P-Regler: zur Person drehen und den Abstand ``soll`` halten."""
        winkel = self._winkel_glatt if self._winkel_glatt is not None else (self._winkel or 0.0)
        # Kleine Winkel nicht nachregeln: das Bildrauschen der Pose (ein bis
        # zwei Grad) ließe ihn sonst ständig hin- und herzucken. Abgezogen
        # statt abgeschnitten, damit omega an der Grenze bei 0 beginnt.
        rest = max(0.0, abs(winkel) - config.FOLGEN_WINKEL_TOTZONE)
        omega = math.copysign(config.FOLGEN_KP_W * math.radians(rest), winkel)
        omega = max(-config.FOLGEN_MAX_OMEGA, min(config.FOLGEN_MAX_OMEGA, omega))

        # Erst ausrichten, dann fahren: schräg losfahren schiebt ihn an der
        # Person vorbei, und der Fahrschlauch geradeaus sagt nichts über den
        # Weg zur Person. Statt hart bei MAX_WINKEL abzuschalten, nimmt die
        # Geschwindigkeit ab VOLL_WINKEL stetig ab -- ein Ein/Aus an der
        # Grenze war einer der Rucke.
        betrag = abs(winkel)
        if betrag <= config.FOLGEN_VOLL_WINKEL:
            anteil = 1.0
        else:
            spanne = max(1e-6, config.FOLGEN_MAX_WINKEL_FAHREN - config.FOLGEN_VOLL_WINKEL)
            anteil = max(0.0, (config.FOLGEN_MAX_WINKEL_FAHREN - betrag) / spanne)

        vx = 0.0
        abstand = self._abstand
        if abstand is not None and math.isfinite(abstand) and anteil > 0.0:
            fehler = abstand - soll
            tot = config.FOLGEN_TOTZONE_M
            breite = config.EXPLORE_HALF_WIDTH_M
            # Auch hier ab der Totzone gerechnet, damit vx dort bei 0 anfängt.
            if fehler > tot:
                vorn = frei_in_richtung(punkte, 0.0, breite)
                # Nie näher als die Stoppgrenze an irgendetwas im Fahrschlauch.
                platz = vorn - config.EXPLORE_STOP_M
                vx = min(config.FOLGEN_MAX_SPEED, config.FOLGEN_KP_V * (fehler - tot)) * anteil
                vx = min(vx, max(0.0, platz))
            elif fehler < -tot and rueckwaerts:
                hinten = frei_in_richtung(punkte, 180.0, breite)
                if hinten > config.EXPLORE_STOP_M:
                    vx = -min(config.FOLGEN_RUECK_SPEED, config.FOLGEN_KP_V * (-fehler - tot)) * anteil

        if vx > 0:
            self._text = f"fährt zur Person ({abstand:.2f} m)"
        elif vx < 0:
            self._text = f"weicht zurück ({abstand:.2f} m)"
        elif abstand is None or not math.isfinite(abstand):
            self._text = "richtet sich aus (Abstand unbekannt)"
        else:
            self._text = f"hält Abstand ({abstand:.2f} m)"
        return DriveCommand(vx, 0.0, omega)

    def _ausrollen(self, jetzt: float, punkte) -> DriveCommand | None:
        """Person für einen Moment weg: den letzten Befehl auslaufen lassen.

        Die Pose verliert die Person oft für ein, zwei Bilder (Bewegungs-
        unschärfe, Verdeckung). Jedes Mal hart anzuhalten und gleich wieder
        anzufahren war der Hauptgrund für das Ruckeln. Der Fahrschlauch
        wird dabei weiter geprüft.
        """
        if self._letzter_befehl is None:
            return None
        seit = jetzt - self._gesehen
        if seit >= config.FOLGEN_HALTEN_S:
            return None
        anteil = 1.0 - seit / config.FOLGEN_HALTEN_S
        letzter = self._letzter_befehl
        breite = config.EXPLORE_HALF_WIDTH_M
        vx = letzter.vx * anteil
        if vx > 0:
            platz = frei_in_richtung(punkte, 0.0, breite) - config.EXPLORE_STOP_M
            vx = min(vx, max(0.0, platz))
        elif vx < 0 and frei_in_richtung(punkte, 180.0, breite) <= config.EXPLORE_STOP_M:
            vx = 0.0
        self._text = "Person kurz nicht zu sehen -- rollt aus"
        return DriveCommand(vx, 0.0, letzter.omega * anteil)

    def _weggehen(self, jetzt: float, ziel: dict | None, punkte) -> DriveCommand:
        dauer = config.WEGGEHEN_STRECKE_M / max(0.01, config.WEGGEHEN_SPEED)
        if jetzt - self._modus_seit >= dauer:
            self._wechseln(BEREIT, "Abstand genommen", jetzt, melden=True)
            return DriveCommand(0.0, 0.0, 0.0)
        # Rückwärts zählt nur, was hinten ist -- vorn steht ja gerade der
        # Mensch, der ihn weggeschickt hat.
        hinten = frei_in_richtung(punkte, 180.0, config.EXPLORE_HALF_WIDTH_M)
        if hinten <= config.EXPLORE_STOP_M or self._nah(punkte, nur_hinten=True):
            self._wechseln(BEREIT, "hinten kein Platz -- bleibe stehen", jetzt, melden=True)
            return DriveCommand(0.0, 0.0, 0.0)
        # Dabei die Person im Blick behalten (sanft, ohne Ausrichtungszwang).
        omega = 0.0
        if ziel is not None:
            omega = 0.5 * config.FOLGEN_KP_W * math.radians(ziel["winkel"])
            omega = max(-config.FOLGEN_MAX_OMEGA, min(config.FOLGEN_MAX_OMEGA, omega))
        self._text = "fährt zurück"
        return DriveCommand(-config.WEGGEHEN_SPEED, 0.0, omega)

    def _suchen(self, jetzt: float, ziel: dict | None) -> DriveCommand:
        if ziel is not None:
            self._wechseln(self._suchen_fuer, "Person wiedergefunden", jetzt, melden=True)
            return DriveCommand(0.0, 0.0, 0.0)
        if jetzt - self._modus_seit > config.FOLGEN_SUCHEN_S:
            self._wechseln(BEREIT, "Person nicht wiedergefunden", jetzt, melden=True)
            return DriveCommand(0.0, 0.0, 0.0)
        # Zur Seite drehen, auf der die Person zuletzt war -- dort ist sie
        # meist aus dem Bild gelaufen.
        self._text = "sucht die Person"
        return DriveCommand(0.0, 0.0, self._letzte_seite * config.EXPLORE_TURN_SPEED)

    # --- Hilfen ------------------------------------------------------------------

    def _ziel_waehlen(self) -> dict | None:
        """Die Person, um die es geht.

        Beim Folgen dieselbe wie bisher (nächster Winkel), sonst wäre ein
        Passant, der näher an der Kamera vorbeigeht, plötzlich das Ziel. Wer
        neu anfängt (BEREIT, SUCHEN), nimmt die größte -- meist die nächste.
        """
        personen = self._pose.personen() if self._pose is not None else []
        self._anzahl = len(personen)
        if not personen:
            return None
        if self._winkel is not None:
            naechste = min(personen, key=lambda p: abs(p["winkel"] - self._winkel))
            if abs(naechste["winkel"] - self._winkel) <= config.INTERAKTION_SPRUNG_GRAD:
                return naechste
            if self._modus in (FOLGEN, HERKOMMEN, WEGGEHEN):
                return None
        return personen[0]

    def _geste_pruefen(self, ziel: dict | None, jetzt: float) -> None:
        if ziel is None:
            if self._anzahl == 0:
                self._gesten.update(None, jetzt)
            return
        # Nur neue Kamerabilder auswerten -- die Pose kommt mit ~10 Hz, diese
        # Schleife läuft mit 20 Hz.
        if ziel["zeit"] == self._letztes_bild:
            return
        self._letztes_bild = ziel["zeit"]
        geste = self._gesten.update(ziel, ziel["zeit"], ruhig=self._roboter_ruhig())
        if geste is None:
            return
        log.info("Interaktion: Geste %s erkannt", geste)
        self._meldungen.append(f"Geste erkannt: {_GESTE_TEXT[geste]}")
        modus = _GESTE_ZU_MODUS[geste]
        texte = {
            BEREIT: "angehalten -- wartet auf ein Handzeichen",
            FOLGEN: "folgt",
            HERKOMMEN: "kommt her",
            WEGGEHEN: "fährt zurück",
        }
        self._wechseln(modus, texte[modus], jetzt)

    def _glaetten(self, ziel: dict, jetzt: float) -> None:
        """Kamerawinkel tiefpassfiltern -- einmal pro neuem Kamerabild.

        Die Box der Pose springt von Bild zu Bild um ein paar Grad; ungefiltert
        wird daraus ein zappelndes omega. Nach einer Lücke fängt der Filter
        neu an, sonst zöge er einen veralteten Winkel nach.
        """
        if ziel["zeit"] == self._glatt_bild:
            return
        self._glatt_bild = ziel["zeit"]
        roh = ziel["winkel"]
        if self._winkel_glatt is None or jetzt - self._gesehen > config.FOLGEN_HALTEN_S:
            self._winkel_glatt = roh
        else:
            self._winkel_glatt += config.FOLGEN_WINKEL_GLAETTUNG * (roh - self._winkel_glatt)

    def _roboter_ruhig(self) -> bool:
        """Steht der Roboter (fast)? Nur dann zählen bewegte Gesten.

        Fährt er, wackelt die Kamera, und die Person wandert durchs Bild --
        das sah für die Erkennung wie Winken aus.
        """
        befehl = getattr(self._watchdog, "current_command", None)
        if befehl is None:
            return True
        return (
            abs(befehl.vx) <= config.GESTE_ROBOTER_RUHIG_V
            and abs(befehl.vy) <= config.GESTE_ROBOTER_RUHIG_V
            and abs(befehl.omega) <= config.GESTE_ROBOTER_RUHIG_W
        )

    def _abstand_zu(self, ziel: dict | None, scan=None) -> float | None:
        """Lidar-Abstand in Richtung der Person (über die Breite ihrer Box)."""
        if ziel is None:
            return None
        if scan is None:
            scan = self._lidar.latest() if self._lidar is not None else None
            if scan is None:
                return None
        fov = self._pose.fov_deg if self._pose is not None else config.CAMERA_FOV_DEG
        von = (0.5 - ziel["box"][2]) * fov
        bis = (0.5 - ziel["box"][0]) * fov
        # Halbe Boxbreite, aber mindestens 3°: bei weit entfernten Personen
        # ist die Box schmal, und ein Lidar-Strahl trifft sonst daneben.
        mitte = (von + bis) / 2
        halb = max(3.0, (bis - von) / 2)
        return scan.min_abstand(mitte, halb)

    @staticmethod
    def _nah(punkte, nur_hinten: bool = False) -> bool:
        if len(punkte) == 0:
            return False
        r = (punkte[:, 0] ** 2 + punkte[:, 1] ** 2) ** 0.5
        nah = r < config.EXPLORE_NAH_M
        if nur_hinten:
            nah &= punkte[:, 0] < 0.0
        return int(nah.sum()) >= 3

    def _wechseln(self, modus: str, text: str, jetzt: float, melden: bool = False) -> None:
        if modus == BEREIT and self._modus != BEREIT:
            # Beim Wechsel nach BEREIT einmal ausdrücklich anhalten -- danach
            # schickt die Schleife nichts mehr.
            self._watchdog.stop()
        if modus != self._modus:
            log.info("Interaktion: %s -> %s (%s)", self._modus, modus, text)
            # Ausrollen gilt nur innerhalb eines Verhaltens.
            self._letzter_befehl = None
        if modus in (FOLGEN, HERKOMMEN):
            # Frist fürs Verlorengehen ab jetzt -- sonst sucht er nach einem
            # Knopfdruck sofort, wenn die Person gerade nicht im Bild ist.
            self._gesehen = jetzt
        self._modus = modus
        self._modus_seit = jetzt
        self._text = text
        if melden:
            self._meldungen.append(text)
        return None
