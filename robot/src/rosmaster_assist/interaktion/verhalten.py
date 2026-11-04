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

Umfahren (03.11.2026, F3): Steht etwas zwischen ihm und der Person (Stuhl,
Türrahmen), schiebt er schräg daran vorbei und behält die Person dabei im
Blick (``ausweichen.richtung_waehlen``, Mecanum). Was beim Vorbeifahren
seitlich aus dem Blickfeld rutscht, merkt er sich kurz (Kurzgedächtnis).
Wird er hochgehoben, hält er an und geht nach dem Absetzen in BEREIT --
von selbst weiterfahren tut er dann nicht.

Sanft fahren (29.10.2026): Sprünge im Fahrbefehl ziehen Stromspitzen, die
am X3 USB-Resets auslösen. Daher wird der Kamerawinkel geglättet, Drehrate
und Geschwindigkeit wachsen stetig mit dem Fehler (keine harten Grenzen),
kurze Aussetzer der Erkennung rollt er aus, statt hart zu bremsen, und der
Watchdog begrenzt zusätzlich die Beschleunigung (``RAMPE_*``).

In BEREIT schickt die Schleife **keine** Befehle -- der Roboter steht, und
nichts hält den Antrieb unnötig wach. Handsteuerung aus der App beendet den
ganzen Interaktionsmodus (wie beim Erkunder): Wer selbst lenkt, will nicht,
dass eine Geste dazwischenfährt.

Antrieb weg (29.10.2026): Bricht die Verbindung zum Board ab (USB-Reset),
endet der Interaktionsmodus nicht mehr, sondern fällt nach BEREIT zurück
und schaut weiter zu. Fahrende Modi werden erst wieder angenommen, wenn
das Board zurück ist -- und auch dann nur auf eine *neue* Geste oder einen
neuen Befehl hin. Von selbst losfahren nach dem Neuverbinden tut er nie.

Wem er folgt (04.11.2026): ``ziel.py`` verfolgt die Personen über Bilder
(Track-IDs, Eigendrehung herausgerechnet) und hält Gesten zweier Personen
auseinander. Unterschieden werden sie nicht mehr: Er folgt der Person im
Bild, und wer kurz weg war und zurückkommt, ist wieder das Ziel. Die
Ziel-Sperre mit Wiedererkennung am Aussehen (03.11.) ist dafür raus -- sie
hielt zurückkehrende Personen oft für fremd und blieb stehen.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from typing import TYPE_CHECKING, Callable

from .. import config
from ..navigation import ausweichen
from ..navigation import hindernisse as hd
from ..navigation.explorer import frei_in_richtung
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import gesten as gs
from .ziel import IDLE, PersonenTracker, ZielWahl

if TYPE_CHECKING:  # nur für die Typangaben -- vermeidet Ringimporte
    from ..navigation.odometry import Odometry
    from ..sensors.lidar import LidarService
    from ..sensors.pose import Posenerkennung
    from .stimme import Stimme

log = logging.getLogger(__name__)

#: Modi, so wie sie in der App erscheinen (state.interaktion.modus).
BEREIT = "bereit"
FOLGEN = "folgen"
HERKOMMEN = "herkommen"
WEGGEHEN = "weggehen"
SUCHEN = "sucht"

#: Modi, in denen gefahren wird -- die brauchen ein verbundenes Board.
_FAHRMODI = (FOLGEN, HERKOMMEN, WEGGEHEN, SUCHEN)

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

#: Was der Roboter laut sagt, wenn er eine Geste verstanden hat. Kurz und in
#: der Ich-Form -- die Person soll sofort hören, dass er sie meint.
_GESTE_SAGEN = {
    gs.FOLGEN: "Ich folge dir.",
    gs.STOPP: "Okay, ich halte an.",
    gs.HERKOMMEN: "Ich komme.",
    gs.WEGGEHEN: "Ich fahre zurück.",
}

#: Anzeige und Satz, wenn der Modus gleich mit einem Auftrag startet
#: (App-Knopf "Folgen" oder Sprachbefehl "Robi komm her").
_START_TEXT = {
    BEREIT: ("wartet auf ein Handzeichen", "Hallo! Zeig mir ein Handzeichen."),
    FOLGEN: ("folgt (per App)", "Jetzt folge ich dir."),
    HERKOMMEN: ("kommt her", "Ich komme."),
    WEGGEHEN: ("nimmt Abstand", "Ich fahre zurück."),
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
        stimme: "Stimme | None" = None,
        antrieb: Callable[[], bool] | None = None,
        odometry: "Odometry | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._pose = pose
        self._stimme = stimme
        # Gemessene Drehrate für die Vorhersage der Tracks (ohne: Fahrbefehl).
        self._odometry = odometry
        # Fragt, ob das Board gerade verbunden ist. Ohne Angabe (Tests)
        # gilt es als verbunden.
        self._antrieb = antrieb
        self._gesten = gs.Gestenerkennung()
        self._task: asyncio.Task[None] | None = None
        # Während des sanften Starts (siehe _laufen): Wünsche merken statt
        # gleich losfahren.
        self._anlauf = False
        self._start_modus = BEREIT
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
        # Woher der letzte Abstand kam ("Cluster: Mensch", "Minimum") -- für
        # Tests und Fehlersuche. Eigene Erkennung, weil der Server seine mit
        # den Personenwinkeln aufruft: dort hieße auch ein Stuhl vor der
        # Person "Mensch", und genau den wollen wir hier unterscheiden.
        self._abstand_quelle: str | None = None
        self._hinderniserkennung = hd.Hinderniserkennung()
        self._nah_zuletzt = -1e9
        # Ausweichen (F3): dieselben Bausteine wie beim Erkunder.
        self._gedaechtnis = ausweichen.Kurzgedaechtnis(odometry)
        self._hoch = ausweichen.HochhebeErkennung()
        self._hoch_gemeldet = False
        self._umfahr_seite = 0.0  # +1 links, -1 rechts, 0: fährt direkt
        self._letzter_takt: float | None = None

        # Wem er folgt (nur im RAM, wird beim Ausschalten gelöscht)
        self._tracker = PersonenTracker()
        self._zielwahl = ZielWahl()
        self._tracker_bild = None  # Bildzeit, mit der der Tracker zuletzt lief
        self._auswahl_id: int | None = None  # wem er beim nächsten Start folgt (Geste)
        self._gesten_je_track: dict[int, gs.Gestenerkennung] = {}
        self._geste_anzeige: tuple[str | None, float | None] = (None, None)
        # Pause aus der App: Ziel bleibt gemerkt und wird weiter verfolgt,
        # aber er fährt nicht.
        self._pausiert = False

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
            self._sagen("Interaktion beendet.")
        self._task = None

    def folgen(self, ziel_id: int | None = None) -> bool:
        """Folgen per App-Knopf statt per Geste. Schaltet den Modus nötigenfalls ein.

        Er folgt der Person im Bild, bei mehreren der größten. ``ziel_id``
        (Track-ID aus ``/interaktion/pose``) nennt eine bestimmte; die App
        schickt sie seit 04.11. nicht mehr, ältere Clients dürfen weiter.
        Ist die Person nicht mehr im Bild, folgt er einfach der, die da ist.
        """
        if not self.aktiv:
            # Den Modus gleich mitgeben -- die Schleife startet erst im
            # nächsten Durchlauf des Event-Loops und würde ihn sonst auf
            # BEREIT zurücksetzen.
            return self.start(FOLGEN)
        if self._anlauf:
            # Sanfter Start läuft noch: Folgen beginnt nach der Ansage.
            self._start_modus = FOLGEN
            return True
        jetzt = time.monotonic()
        if ziel_id is not None and self._tracker.track(ziel_id) is not None:
            if self._zielwahl.id != ziel_id:
                self._zielwahl.freigeben()
            self._auswahl_id = ziel_id
        self._wechseln(FOLGEN, "folgt (per App)", jetzt, sagen="Ich folge dir.")
        return True

    def pausieren(self) -> bool:
        """Anhalten, Ziel aber behalten. False, wenn gerade nichts fährt."""
        if not self.aktiv or self._modus not in _FAHRMODI or self._pausiert:
            return False
        self._pausiert = True
        self._watchdog.stop()
        self._letzter_befehl = None
        self._text = "pausiert -- Ziel bleibt gemerkt"
        log.info("Interaktion: pausiert (%s)", self._modus)
        return True

    def fortsetzen(self) -> bool:
        """Nach einer Pause weiterfahren. False, wenn nicht pausiert."""
        if not self.aktiv or not self._pausiert:
            return False
        jetzt = time.monotonic()
        self._pausiert = False
        # Die Pause zählt nicht als Verlust- oder Suchzeit, sonst gäbe er
        # gleich nach dem Weiter auf.
        self._gesehen = self._modus_seit = jetzt
        log.info("Interaktion: weiter (%s)", self._modus)
        return True

    @property
    def pausiert(self) -> bool:
        return self.aktiv and self._pausiert

    def personen_mit_id(self, personen: list[dict]) -> list[dict]:
        """Personen für die App: ohne Merkmal, mit Track-``id`` und ``ziel``.

        Die Pose liefert dieselben Dicts, die auch der Tracker bekommen hat;
        daran erkennt man die Zuordnung. Ist das Bild neuer als der letzte
        Tracker-Takt, ist ``id`` kurz ``None``.
        """
        ids = {id(t.person): t.id for t in self._tracker.tracks if t.gemessen}
        folgt = self._zielwahl.id if self._zielwahl.zustand != IDLE else None
        aus = []
        for p in personen:
            tid = ids.get(id(p))
            eintrag = {k: v for k, v in p.items() if k != "merkmal"}
            eintrag["id"] = tid
            eintrag["ziel"] = tid is not None and tid == folgt
            aus.append(eintrag)
        return aus

    def antrieb_verloren(self) -> None:
        """Board weg: anhalten, aber weiter zuschauen (siehe Modulkopf).

        Früher endete hier der ganze Interaktionsmodus. Weil gerade beim
        Einschalten der Gestensteuerung am X3 öfter ein USB-Reset kam,
        war sie dann sofort wieder aus -- und man musste neu starten.
        """
        if self.aktiv and self._modus != BEREIT:
            self._wechseln(BEREIT, "Verbindung zum Antrieb verloren -- angehalten",
                           time.monotonic(), melden=True)

    def wechseln_zu(self, modus: str, text: str, sagen: str | None = None) -> bool:
        """Verhalten von außen setzen (Sprachbefehl). False, wenn nicht an."""
        if not self.aktiv:
            return False
        if self._anlauf:
            # Noch im sanften Start: gilt, sobald er fertig ist.
            self._start_modus = modus
            return True
        self._wechseln(modus, text, time.monotonic(), sagen=sagen)
        return self._modus == modus

    def folgen_stop(self) -> None:
        """Verhalten beenden, Interaktionsmodus bleibt an."""
        if self.aktiv and self._anlauf:
            self._start_modus = BEREIT
        elif self.aktiv and self._modus != BEREIT:
            self._wechseln(BEREIT, "angehalten (per App)", time.monotonic(), sagen="Okay, ich bleibe stehen.")
        if self.aktiv and self._modus == BEREIT:
            # „Loslassen“ löscht auch den Grund der letzten Suche -- danach
            # zeigt die App wieder „kein Ziel“.
            self._zielwahl.freigeben()

    def meldungen_abholen(self) -> list[str]:
        """Ereignisse seit dem letzten Aufruf -- für Hinweise in der App."""
        neu, self._meldungen = self._meldungen, []
        return neu

    def zustand(self) -> dict:
        """Für state.interaktion in der App."""
        aktiv = self.aktiv
        jetzt = time.monotonic()
        geste = None
        # Gesten kommen aus der Erkennung je Track oder (alter Weg) aus der
        # einen Erkennung -- angezeigt wird die jüngere.
        letzte = [
            (zeit, g)
            for g, zeit in ((self._gesten.zuletzt, self._gesten.zuletzt_zeit), self._geste_anzeige)
            if zeit is not None
        ]
        if letzte:
            zeit, g = max(letzte, key=lambda e: e[0])
            if jetzt - zeit < _GESTE_ANZEIGE_S:
                geste = g
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
            "ziel": self._zielwahl.als_dict(),
            "pausiert": self.pausiert,
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
        self._gedaechtnis.leeren()
        self._hoch.zuruecksetzen()
        self._hoch_gemeldet = False
        self._umfahr_seite = 0.0
        self._letzter_takt = None
        self._ziel_vergessen()
        self._modus = BEREIT
        self._start_modus = modus

        try:
            # Sanfter Start: Posenerkennung, Ansage und Motoren nacheinander
            # statt in derselben Sekunde -- zusammen brachen sie am 03.11. die
            # USB-Versorgung (siehe config.INTERAKTION_ANLAUF_S).
            if self._pose is not None:
                self._pose.aktivieren(True)
            log.info("Interaktion gestartet")
            self._anlauf = True
            self._text = "startet …"
            if config.INTERAKTION_ANLAUF_S > 0:
                await asyncio.sleep(config.INTERAKTION_ANLAUF_S)
            # Was in der Zwischenzeit verlangt wurde, gilt (Sprache, App).
            modus = self._start_modus
            text, satz = _START_TEXT.get(modus, _START_TEXT[BEREIT])
            if modus in _FAHRMODI and self._stimme is not None:
                # Erst ausreden, dann losfahren.
                self._sagen(satz)
                await self._ausreden()
                satz = None
                if self._start_modus != modus:  # z.B. "Stopp" während der Ansage
                    modus = self._start_modus
                    text, satz = _START_TEXT.get(modus, _START_TEXT[BEREIT])
            self._anlauf = False
            self._wechseln(modus, text, time.monotonic(), sagen=satz)

            while True:
                befehl = self._schritt(time.monotonic())
                # None heißt: nichts schicken (BEREIT). Angehalten wurde beim
                # Wechsel nach BEREIT schon.
                if befehl is not None:
                    self._watchdog.submit(befehl, quelle="autonom")
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler in der Interaktion"
            log.exception("Interaktion abgebrochen")
        finally:
            self._anlauf = False
            # Gelerntes Aussehen nicht über das Ausschalten hinaus behalten.
            self._ziel_vergessen()
            if self._pose is not None:
                self._pose.aktivieren(False)

    async def _ausreden(self) -> None:
        """Warten, bis die Stimme den Satz gesprochen hat (höchstens STIMME_TIMEOUT_S)."""
        ende = time.monotonic() + config.STIMME_TIMEOUT_S
        # Kurz Zeit geben, bis der Satz in der Schlange angekommen ist.
        await asyncio.sleep(0.1)
        while self._stimme.beschaeftigt and time.monotonic() < ende:
            await asyncio.sleep(0.1)

    def _schritt(self, jetzt: float) -> DriveCommand | None:
        """Ein Takt: Person und Geste ansehen, Fahrbefehl zurückgeben."""
        halt = DriveCommand(0.0, 0.0, 0.0)

        # Eigene Lage fürs Kurzgedächtnis -- jeden Takt, auch ohne neue Runde.
        dt = 0.0 if self._letzter_takt is None else min(0.5, max(0.0, jetzt - self._letzter_takt))
        self._letzter_takt = jetzt
        self._gedaechtnis.fortschreiben(dt)

        ziel = self._ziel_waehlen(jetzt)
        self._geste_pruefen(ziel, jetzt)

        if ziel is not None:
            self._glaetten(ziel, jetzt)
            self._gesehen = jetzt
            self._winkel = ziel["winkel"]
            if abs(self._winkel) > 2.0:
                self._letzte_seite = 1.0 if self._winkel > 0 else -1.0

        if self._modus != BEREIT and not self._antrieb_da():
            # Zweite Absicherung neben antrieb_verloren(): die
            # Verbindungsüberwachung prüft nur alle RECONNECT_CHECK_S.
            return self._wechseln(BEREIT, "Verbindung zum Antrieb verloren -- angehalten",
                                  jetzt, melden=True)

        if self._modus == BEREIT:
            self._abstand = self._abstand_zu(ziel, jetzt=jetzt)
            return None

        if self._pausiert:
            # Tracker und Zielwahl laufen oben weiter, damit das Ziel nach dem
            # Weiter noch dasselbe ist. Gefahren wird nicht.
            self._abstand = self._abstand_zu(ziel, jetzt=jetzt)
            self._text = "pausiert -- Ziel bleibt gemerkt"
            self._letzter_befehl = None
            return halt

        if self._modus in (HERKOMMEN, WEGGEHEN) and jetzt - self._modus_seit > config.INTERAKTION_VERHALTEN_MAX_S:
            return self._wechseln(BEREIT, f"{self._modus}: Zeit abgelaufen", jetzt, melden=True,
                                 sagen="Das dauert zu lange. Ich bleibe stehen.")

        # Ohne frische Lidar-Runde wird nicht gefahren. Kurze Aussetzer
        # (USB-Reset) abwarten, bei längeren das Verhalten beenden.
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or jetzt - scan.zeit > config.EXPLORE_SCAN_MAX_AGE_S:
            if self._ohne_lidar_seit is None:
                self._ohne_lidar_seit = jetzt
                log.warning("Interaktion: keine frischen Lidar-Daten -- halte an")
            if jetzt - self._ohne_lidar_seit > config.EXPLORE_LIDAR_WAIT_S:
                return self._wechseln(BEREIT, "Lidar liefert keine Daten", jetzt, melden=True,
                                     sagen="Ich sehe nichts mehr und halte an.")
            self._text = "keine frischen Lidar-Daten"
            self._letzter_befehl = None
            return halt
        self._ohne_lidar_seit = None

        # Hochgehoben oder getragen: anhalten. Nach dem Absetzen stimmt
        # nichts mehr (wo die Person ist, was im Weg war) -- also nicht von
        # selbst weiterfahren, sondern auf eine neue Geste warten.
        lage = self._gedaechtnis.lage if self._gedaechtnis.misst else None
        if self._hoch.pruefen(scan, lage):
            if not self._hoch_gemeldet:
                self._hoch_gemeldet = True
                self._gedaechtnis.leeren()
                self._umfahr_seite = 0.0
                log.info("Interaktion: hochgehoben (%.0f %% des Lidar-Bilds verändert)",
                         100 * self._hoch.anteil)
            self._text = "hochgehoben -- wartet, bis er wieder steht"
            self._letzter_befehl = None
            return halt
        if self._hoch_gemeldet:
            self._hoch_gemeldet = False
            return self._wechseln(BEREIT, "hochgehoben -- angehalten", jetzt, melden=True,
                                 sagen="Ich wurde hochgehoben. Ich bleibe stehen.")

        # Die rohe Runde für "jetzt ganz nah"; samt Gedächtnis fürs Fahren --
        # dort zählt auch der Türrahmen, der gerade neben ihm aus dem Blick
        # gerutscht ist.
        roh = scan.punkte()
        punkte = self._gedaechtnis.punkte(scan)
        self._abstand = self._abstand_zu(ziel, scan, jetzt)
        if self._abstand is not None and math.isfinite(self._abstand):
            self._abstand_gut, self._abstand_gut_zeit = self._abstand, jetzt
        elif ziel is not None and jetzt - self._abstand_gut_zeit < config.FOLGEN_HALTEN_S:
            # In Richtung der Person kurz kein Lidar-Punkt (Beine zwischen
            # zwei Strahlen) -- den letzten guten Wert nehmen, statt
            # stehenzubleiben und gleich wieder anzufahren.
            self._abstand = self._abstand_gut

        if self._modus == WEGGEHEN:
            return self._weggehen(jetzt, ziel, punkte, roh)

        # Folgen, Herkommen, Suchen: alles, was nach vorn oder im Stand
        # dreht. Ganz nah etwas -> gar nicht bewegen (die Ecken schwenken
        # beim Drehen aus). Erst ab drei Punkten, ein Fehlstrahl zählt nicht.
        # Nach "nah" noch kurz stehen bleiben, damit er an der Grenze nicht
        # zwischen Halt und Fahrt flackert.
        if self._nah(roh):
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
                return self._wechseln(SUCHEN, "Person verloren -- suche", jetzt, melden=True,
                                     sagen="Wo bist du?")
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
            return self._wechseln(BEREIT, "bin da", jetzt, melden=True, sagen="Da bin ich.")
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

        vx = vy = 0.0
        umfahren: str | None = None
        abstand = self._abstand
        if abstand is not None and math.isfinite(abstand) and anteil > 0.0:
            fehler = abstand - soll
            tot = config.FOLGEN_TOTZONE_M
            breite = config.EXPLORE_HALF_WIDTH_M
            # Auch hier ab der Totzone gerechnet, damit vx dort bei 0 anfängt.
            if fehler > tot:
                v = min(config.FOLGEN_MAX_SPEED, config.FOLGEN_KP_V * (fehler - tot)) * anteil
                # Mit demselben Rand wie beim Ausweichen: Sonst schwenkt er
                # nach dem Vorbeischieben zu früh zurück und streift die Ecke.
                vorn = frei_in_richtung(punkte, 0.0, breite + config.AUSWEICH_RAND_M)
                # Platz für das nächste Stück. Es endet vor der Person, sie
                # selbst versperrt sich also nie den Weg.
                noetig = min(fehler - tot, config.UMFAHREN_VORAUS_M) + config.EXPLORE_STOP_M
                wahl = None
                if vorn >= noetig:
                    self._umfahr_seite = 0.0
                else:
                    # Geradeaus versperrt: schräg vorbeischieben, der Blick
                    # bleibt (omega) auf der Person.
                    wahl = ausweichen.richtung_waehlen(punkte, winkel, noetig, breite, self._umfahr_seite)
                if wahl is None:
                    # Nie näher als die Stoppgrenze an irgendetwas im Fahrschlauch.
                    vx = min(v, max(0.0, vorn - config.EXPLORE_STOP_M))
                else:
                    richtung, frei = wahl
                    v = min(v, max(0.0, frei - config.EXPLORE_STOP_M))
                    a = math.radians(richtung)
                    vx, vy = v * math.cos(a), v * math.sin(a)
                    versatz = ausweichen._winkel_normieren(math.radians(richtung - winkel))
                    if abs(versatz) > 1e-6:
                        self._umfahr_seite = math.copysign(1.0, versatz)
                        umfahren = "links" if versatz > 0 else "rechts"
            elif fehler < -tot and rueckwaerts:
                hinten = frei_in_richtung(punkte, 180.0, breite)
                if hinten > config.EXPLORE_STOP_M:
                    vx = -min(config.FOLGEN_RUECK_SPEED, config.FOLGEN_KP_V * (-fehler - tot)) * anteil

        if umfahren is not None and vx > 0:
            self._text = f"umfährt Hindernis ({umfahren}, {abstand:.2f} m)"
        elif vx > 0:
            self._text = f"fährt zur Person ({abstand:.2f} m)"
        elif vx < 0:
            self._text = f"weicht zurück ({abstand:.2f} m)"
        elif abstand is None or not math.isfinite(abstand):
            self._text = "richtet sich aus (Abstand unbekannt)"
        else:
            self._text = f"hält Abstand ({abstand:.2f} m)"
        return DriveCommand(vx, vy, omega)

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
        vx, vy = letzter.vx * anteil, letzter.vy * anteil
        if vx > 0:
            # Beim Umfahren schräg: in Fahrtrichtung prüfen und beide
            # Anteile gemeinsam kürzen, damit die Richtung bleibt.
            richtung = math.degrees(math.atan2(vy, vx))
            platz = max(0.0, frei_in_richtung(punkte, richtung, breite) - config.EXPLORE_STOP_M)
            v = math.hypot(vx, vy)
            if v > platz:
                vx, vy = vx * platz / v, vy * platz / v
        elif vx < 0 and frei_in_richtung(punkte, 180.0, breite) <= config.EXPLORE_STOP_M:
            vx = vy = 0.0
        self._text = "Person kurz nicht zu sehen -- rollt aus"
        return DriveCommand(vx, vy, letzter.omega * anteil)

    def _weggehen(self, jetzt: float, ziel: dict | None, punkte, roh) -> DriveCommand:
        dauer = config.WEGGEHEN_STRECKE_M / max(0.01, config.WEGGEHEN_SPEED)
        if jetzt - self._modus_seit >= dauer:
            self._wechseln(BEREIT, "Abstand genommen", jetzt, melden=True, sagen="So, genug Abstand.")
            return DriveCommand(0.0, 0.0, 0.0)
        # Rückwärts zählt nur, was hinten ist -- vorn steht ja gerade der
        # Mensch, der ihn weggeschickt hat.
        hinten = frei_in_richtung(punkte, 180.0, config.EXPLORE_HALF_WIDTH_M)
        if hinten <= config.EXPLORE_STOP_M or self._nah(roh, nur_hinten=True):
            self._wechseln(BEREIT, "hinten kein Platz -- bleibe stehen", jetzt, melden=True,
                          sagen="Hinter mir ist kein Platz.")
            return DriveCommand(0.0, 0.0, 0.0)
        # Dabei die Person im Blick behalten (sanft, ohne Ausrichtungszwang).
        omega = 0.0
        if ziel is not None:
            omega = 0.5 * config.FOLGEN_KP_W * math.radians(ziel["winkel"])
            omega = max(-config.FOLGEN_MAX_OMEGA, min(config.FOLGEN_MAX_OMEGA, omega))
        self._text = "fährt zurück"
        return DriveCommand(-config.WEGGEHEN_SPEED, 0.0, omega)

    def _suchen(self, jetzt: float, ziel: dict | None) -> DriveCommand:
        # Wer jetzt auftaucht, ist es (ZielWahl in ziel.py) -- auch mit neuer
        # Track-Nummer, weil sie kurz aus dem Bild war.
        if ziel is not None:
            self._wechseln(self._suchen_fuer, "Person wiedergefunden", jetzt, melden=True,
                          sagen="Da bist du ja.")
            return DriveCommand(0.0, 0.0, 0.0)
        sucht_seit = jetzt - self._modus_seit
        if config.FOLGEN_SUCHEN_S > 0 and sucht_seit > config.FOLGEN_SUCHEN_S:
            self._zielwahl.freigeben("Person nicht wiedergefunden")
            self._wechseln(BEREIT, "Person nicht wiedergefunden", jetzt, melden=True,
                          sagen="Ich habe dich verloren.")
            return DriveCommand(0.0, 0.0, 0.0)
        # Drehen und kurz stillhalten im Wechsel: Im Drehen verwischt das
        # Bild, im Stand sieht die Posenerkennung scharf.
        takt = config.FOLGEN_SUCHEN_DREH_S + config.FOLGEN_SUCHEN_PAUSE_S
        if takt > 0 and sucht_seit % takt >= config.FOLGEN_SUCHEN_DREH_S:
            self._text = "sucht die Person -- schaut"
            return DriveCommand(0.0, 0.0, 0.0)
        # Zur Seite drehen, auf der die Person zuletzt war -- dort ist sie
        # meist aus dem Bild gelaufen. Ohne Zeitgrenze immer weiter im Kreis.
        self._text = "sucht die Person"
        return DriveCommand(0.0, 0.0, self._letzte_seite * config.EXPLORE_TURN_SPEED)

    # --- Hilfen ------------------------------------------------------------------

    def _ziel_waehlen(self, jetzt: float) -> dict | None:
        """Die Person, um die es geht -- mit ihrer Track-ID (``"id"``, nur intern).

        In fahrenden Modi die von der ZielWahl: dieselbe wie zuletzt, und
        wenn die weg ist, die nächste, die dort auftaucht. In BEREIT ist es
        nur die Anzeige: die größte Person.
        """
        personen = self._pose.personen() if self._pose is not None else []
        self._anzahl = len(personen)
        # Neues Kamerabild erkennt man an der Bildzeit. Ohne Personen läuft
        # der Tracker mit der Uhrzeit weiter, damit alte Tracks verfallen.
        bild = personen[0].get("zeit", jetzt) if personen else jetzt
        if bild != self._tracker_bild:
            self._tracker_bild = bild
            self._tracker.aktualisieren(personen, bild, self._eigendrehung())

        if self._modus in _FAHRMODI:
            if self._zielwahl.zustand == IDLE:
                # Modus wurde gesetzt, ohne dass _wechseln lief (Tests).
                self._zielwahl.erfassen(self._auswahl_id, jetzt)
                self._auswahl_id = None
            track = self._zielwahl.zuordnen(self._tracker.tracks, self._tracker_bild, jetzt)
        else:
            gemessen = [t for t in self._tracker.tracks if t.gemessen]
            track = max(gemessen, key=lambda t: t.hoehe) if gemessen else None
        return dict(track.person, id=track.id) if track is not None else None

    def _eigendrehung(self) -> float:
        """Drehrate des Roboters (rad/s): gemessen, sonst der Fahrbefehl."""
        if self._odometry is not None:
            return float(self._odometry.geschwindigkeit()[2])
        befehl = getattr(self._watchdog, "current_command", None)
        return float(befehl.omega) if befehl is not None else 0.0

    def _ziel_vergessen(self) -> None:
        """Tracks und Ziel löschen."""
        self._tracker.leeren()
        self._zielwahl.freigeben()
        self._tracker_bild = None
        self._auswahl_id = None
        self._gesten_je_track.clear()
        self._geste_anzeige = (None, None)

    def _geste_pruefen(self, ziel: dict | None, jetzt: float) -> None:
        if ziel is not None and "id" not in ziel:
            # Person ohne Track (direkter Aufruf, z. B. aus test_stimme).
            self._geste_pruefen_einzeln(ziel, jetzt)
            return
        if self._anzahl == 0:
            self._gesten_je_track.clear()
            return
        # Nur neue Kamerabilder auswerten -- die Pose kommt mit ~10 Hz, diese
        # Schleife läuft mit 20 Hz.
        bild = self._tracker_bild
        if bild == self._letztes_bild:
            return
        self._letztes_bild = bild

        # Jede Person hat ihre eigene Erkennung: Sprang früher die Auswahl
        # zwischen zwei Personen, mischten sich ihre Armbewegungen.
        lebend = {t.id for t in self._tracker.tracks}
        for tid in list(self._gesten_je_track):
            if tid not in lebend:
                del self._gesten_je_track[tid]
        ruhig = self._roboter_ruhig()
        treffer = []
        for t in self._tracker.tracks:
            if not t.gemessen:
                continue
            erkennung = self._gesten_je_track.setdefault(t.id, gs.Gestenerkennung())
            geste = erkennung.update(t.person, bild, ruhig=ruhig)
            if geste is not None:
                treffer.append((t.id, geste))
        if not treffer:
            return

        if self._modus in _FAHRMODI:
            # Beim Fahren zählt nur das Ziel -- ein Passant darf ihn nicht
            # wegschicken. Stopp gilt von jedem: anhalten ist immer sicher.
            ziel_id = self._zielwahl.id
            treffer = [(tid, g) for tid, g in treffer if g == gs.STOPP or tid == ziel_id]
            if not treffer:
                return
        # Stopp hat Vorrang vor allem anderen.
        tid, geste = min(treffer, key=lambda e: e[1] != gs.STOPP)
        self._geste_anzeige = (geste, bild)
        if self._modus == BEREIT and geste != gs.STOPP:
            # Wer die Geste macht, wird das Ziel (nicht die größte Person).
            self._auswahl_id = tid
        self._geste_ausfuehren(geste, jetzt)

    def _geste_pruefen_einzeln(self, ziel: dict | None, jetzt: float) -> None:
        """Alter Weg: eine Erkennung für die gewählte Person."""
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
        self._geste_ausfuehren(geste, jetzt)

    def _geste_ausfuehren(self, geste: str, jetzt: float) -> None:
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
        if self._modus == modus:  # sonst abgelehnt (kein Antrieb), das sagt _wechseln
            # Stopp hat Vorrang: was davor in der Schlange stand, stimmt nicht mehr.
            self._sagen(_GESTE_SAGEN[geste], vorrang=geste == gs.STOPP)

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

    def _abstand_zu(self, ziel: dict | None, scan=None, jetzt: float | None = None) -> float | None:
        """Lidar-Abstand zur Person: ihr Cluster im Bildwinkel der Box.

        Früher das Minimum im Winkelbereich der Box -- dann galt ein Stuhl
        vor der Person als ihr Abstand. Jetzt wird unter den Clustern in
        diesem Bereich derjenige gewählt, der am ehesten die Person ist:

        1. zwei Menschenbeine ("Mensch") -- die stärkste Form im Lidar,
        2. passt zum letzten guten Abstand (Sprung < FOLGEN_CLUSTER_SPRUNG_M),
        3. dicke Einzelbeine oder Objekte (Rock, Mantel) vor Möbelbeinen,
        4. Winkel am nächsten an der Boxmitte.

        Findet sich kein Cluster (Person weiter als OBSTACLE_RANGE_M, an
        der Wand, nur ein Strahl), gilt wie früher das Minimum. Sicherheit
        hängt nicht daran: ``_annaehern`` begrenzt die Vorwärtsfahrt immer
        mit dem freien Fahrschlauch, ein Stuhl davor hält ihn also trotzdem.
        """
        if ziel is None:
            self._abstand_quelle = None
            return None
        if scan is None:
            scan = self._lidar.latest() if self._lidar is not None else None
            if scan is None:
                self._abstand_quelle = None
                return None
        fov = self._pose.fov_deg if self._pose is not None else config.CAMERA_FOV_DEG
        von = (0.5 - ziel["box"][2]) * fov
        bis = (0.5 - ziel["box"][0]) * fov
        # Halbe Boxbreite, aber mindestens 3°: bei weit entfernten Personen
        # ist die Box schmal, und ein Lidar-Strahl trifft sonst daneben.
        mitte = (von + bis) / 2
        halb = max(3.0, (bis - von) / 2)

        rand = config.FOLGEN_CLUSTER_RAND_GRAD
        kandidaten = [
            h for h in self._hinderniserkennung.aktuell(scan)
            if h.art != hd.WAND and abs((h.winkel - mitte + 180.0) % 360.0 - 180.0) <= halb + rand
        ]
        if not kandidaten:
            self._abstand_quelle = "Minimum"
            return scan.min_abstand(mitte, halb)

        gut = None
        if jetzt is not None and jetzt - self._abstand_gut_zeit < config.FOLGEN_CLUSTER_GUT_S:
            gut = self._abstand_gut

        def rang(h):
            unplausibel = gut is not None and abs(h.abstand - gut) > config.FOLGEN_CLUSTER_SPRUNG_M
            moebel = h.art in (hd.STUHL, hd.TISCH, hd.BEIN)
            winkel = abs((h.winkel - mitte + 180.0) % 360.0 - 180.0)
            return (h.art != hd.MENSCH, unplausibel, moebel, winkel)

        bester = min(kandidaten, key=rang)
        self._abstand_quelle = f"Cluster: {bester.art}"
        return bester.abstand

    @staticmethod
    def _nah(punkte, nur_hinten: bool = False) -> bool:
        if len(punkte) == 0:
            return False
        r = (punkte[:, 0] ** 2 + punkte[:, 1] ** 2) ** 0.5
        nah = r < config.EXPLORE_NAH_M
        if nur_hinten:
            nah &= punkte[:, 0] < 0.0
        return int(nah.sum()) >= 3

    def _antrieb_da(self) -> bool:
        return self._antrieb is None or bool(self._antrieb())

    def _sagen(self, text: str, vorrang: bool = False) -> None:
        if self._stimme is not None:
            self._stimme.sagen(text, vorrang=vorrang)

    def _wechseln(
        self, modus: str, text: str, jetzt: float, melden: bool = False, sagen: str | None = None
    ) -> None:
        if modus in _FAHRMODI and not self._antrieb_da():
            # Ohne Board würde jeder Befehl ins Leere gehen, und nach dem
            # Neuverbinden führe er plötzlich los. Also gar nicht erst
            # umschalten, sondern stehen bleiben und Bescheid geben.
            log.warning("Interaktion: %s abgelehnt -- keine Verbindung zum Antrieb", modus)
            modus, text, melden = BEREIT, "wartet auf den Antrieb", True
            self._meldungen.append("Keine Verbindung zum Antrieb -- fahre nicht los")
            sagen = "Ich habe keine Verbindung zu meinen Rädern."
        # Jeder Moduswechsel (Geste, Stopp, Verlust) beendet eine Pause.
        self._pausiert = False
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
        # Zielwahl mitführen. Ein neues Ziel gibt es nur bei einem Start aus
        # dem Stand, nie beim Wechsel zwischen fahrenden Modi.
        if modus == SUCHEN:
            self._zielwahl.suchen(jetzt)
        elif modus == BEREIT:
            if self._zielwahl.zustand != IDLE:
                self._zielwahl.freigeben()
        elif modus in _FAHRMODI and self._zielwahl.zustand == IDLE:
            self._zielwahl.erfassen(self._auswahl_id, jetzt)
            self._auswahl_id = None
        self._modus = modus
        self._modus_seit = jetzt
        self._text = text
        if melden:
            self._meldungen.append(text)
        if sagen:
            self._sagen(sagen)
        return None
