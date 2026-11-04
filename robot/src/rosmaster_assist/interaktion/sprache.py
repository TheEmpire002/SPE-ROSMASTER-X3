"""Sprachbefehle über das USB-Mikrofon -- Weckwort "Robi".

"Robi, folge mir", "Robi komm her", "Stopp": Das Mikrofon läuft in einem
eigenen Thread, Vosk erkennt offline auf der CPU (kleines deutsches Modell,
siehe docs/hardware.md). Erkannt wird nur ein fester Wortschatz (Grammatik):
Das ist auf dem Pi schnell und verwechselt viel seltener als freie
Erkennung, die aus jedem Geräusch irgendein Wort macht.

Ablauf:

* Ein Befehl gilt nur zusammen mit dem Weckwort -- davor oder danach
  ("Robi folgen", "folgen Robi") oder bis ``SPRACHE_WECK_FENSTER_S``
  nach einem einzelnen "Robi". Sonst führe der Roboter los, sobald im Raum
  jemand "komm mit" sagt.
* Ausnahme "La-da-da" (``SPRACHE_LADADA_OHNE_WECKWORT``): spielt das Lied
  aus ``SPRACHE_LADADA_URL`` -- ein Fehlalarm ist nur Musik.
* Ausnahme Stopp (``SPRACHE_STOPP_OHNE_WECKWORT``): Anhalten darf nie an
  einem überhörten Namen scheitern.
* Während die eigene Stimme spricht, wird nicht zugehört, sonst versteht
  der Roboter sich selbst ("Ich folge dir" -> "folge").
* Die Grammatik hat eine Kehrseite: Vosk presst jedes Geräusch in einen
  erlaubten Satz. Musik und Gespräche im Raum wurden so zu "robbie hallo"
  und schalteten die Interaktion ein. Deshalb lässt unsicher Gehörtes
  (``SPRACHE_MIN_KONFIDENZ``, ``SPRACHE_MAX_UNK_ANTEIL``) und Wortsalat
  aus mehreren Sätzen (``SPRACHE_MAX_ZUSATZWOERTER``) nur Stopp durch.
  Bei laufender Musik braucht jeder Befehl außer Stopp den Namen und
  ``SPRACHE_MIN_KONFIDENZ_MUSIK``.
* "Robi" allein beantwortet der Server mit ``SPRACHE_WECK_ANTWORT``
  ("Ja?"), damit man hört, dass er zuhört.

Das Modul fährt nicht selbst. Es legt erkannte Befehle in eine Schlange;
der Server holt sie ab und behandelt sie wie Knöpfe der App (über den
Watchdog, mit allen Sicherheitsregeln).

Fehlen Vosk, sounddevice, Modell oder Mikrofon, ist die Erkennung einfach
aus -- der Server läuft weiter. Meldet sich das Mikrofon nach einem
USB-Aussetzer ab, verbindet sich der Thread selbst neu.

``hoeren(text)`` nimmt Text, als wäre er gehört worden: für ``--mock``,
die Tests und den Befehl ``speech`` aus der App.
"""

from __future__ import annotations

import collections
import json
import logging
import os
import queue
import re
import threading
import time

from .. import config

log = logging.getLogger(__name__)

# Befehle (Werte in state.sprache.befehl und im Log)
FOLGEN = "folgen"
STOPP = "stopp"
HERKOMMEN = "herkommen"
WEGGEHEN = "weggehen"
START = "start"
AUS = "aus"
NACH_HAUSE = "nach_hause"
ERKUNDEN = "erkunden"
MUSIK_AUS = "musik_aus"
LADADA = "ladada"
BEATBOX = "beatbox"
#: Genauer Fahrbefehl mit Zahl: "fahr zwei Meter vor", "dreh dich um neunzig
#: Grad nach rechts". Steht nicht in BEFEHLE, weil er eine Zahl trägt; siehe
#: manoever_erkennen(). Ausgeführt von navigation/manoever.py.
MANOEVER = "manoever"
#: Kein Befehl, nur "Robi" allein: Der Server antwortet ("Ja?"), damit man
#: hört, dass er zuhört. Steht nur in der Schlange, nie in state.sprache.
WECKEN = "wecken"

#: Befehl -> Wendungen. Die Reihenfolge zählt: Kommen in einem Satz
#: mehrere vor, gewinnt der weiter oben -- Stopp also vor allem, was fährt.
#: Nur "Musik stopp" steht davor, sonst hielte er an statt die Musik.
BEFEHLE: dict[str, tuple[str, ...]] = {
    MUSIK_AUS: ("musik aus", "musik stopp", "ruhe", "sei leise"),
    STOPP: ("stopp", "stop", "halt", "anhalten", "halt an", "bleib stehen", "steh"),
    FOLGEN: ("folgen", "folge", "folge mir", "folg mir", "komm mit"),
    HERKOMMEN: ("komm her", "komm zu mir", "hierher", "komm"),
    WEGGEHEN: ("geh weg", "weg", "zurück", "fahr zurück", "abstand"),
    NACH_HAUSE: ("nach hause", "heim", "fahr nach hause"),
    ERKUNDEN: ("erkunde", "erkunden", "schau dich um"),
    AUS: ("schlafen", "schlaf", "gute nacht", "feierabend"),
    START: ("hallo", "wach auf", "aufwachen", "guten morgen"),
    # Vosk kennt "ladada" nicht als Wort, hört aber "la da da"; "ladada"
    # selbst bleibt für Getipptes (Befehl speech, --mock).
    LADADA: ("la da da", "ladada", "la la la", "lalala"),
    # "Beatbox" steht vermutlich nicht im Vosk-Modell; die Varianten fängt
    # die Wortschatz-Prüfung auf -- was das Modell nicht kennt, fällt weg.
    BEATBOX: ("beatbox", "beat box", "bit box", "biet box", "beet box"),
}

# Zustände der Erkennung (status()["zustand"])
AUS_GESCHALTET = "aus"
LAEDT = "laedt"
HOERT = "hoert"
FEHLER = "fehler"
SIMULIERT = "simuliert"


def normalisieren(text: str) -> list[str]:
    """Kleinbuchstaben, ohne Satzzeichen, in Wörter zerlegt.

    Ziffern bleiben (getippt: "fahr 1,5 m vor"); Vosk liefert Zahlwörter.
    """
    text = str(text).lower().replace("ß", "ss").replace("zurueck", "zurück")
    return re.findall(r"\d+(?:[.,]\d+)?|[a-zäöü]+", text)


# --- Genaue Fahrbefehle mit Zahl ----------------------------------------------------
# Werte wie in navigation/manoever.py (VOR, ZURUECK, ... DREHEN).

_ZAHLWOERTER = {
    "ein": 1, "eins": 1, "einen": 1, "einem": 1, "zwei": 2, "drei": 3, "vier": 4,
    "fünf": 5, "sechs": 6, "sieben": 7, "acht": 8, "neun": 9, "zehn": 10, "elf": 11,
    "zwölf": 12, "fünfzehn": 15, "zwanzig": 20, "dreissig": 30, "vierzig": 40,
    "fünfundvierzig": 45, "fünfzig": 50, "sechzig": 60, "siebzig": 70, "achtzig": 80,
    "neunzig": 90, "hundert": 100, "einhundert": 100, "hundertzwanzig": 120,
    "hundertfünfunddreissig": 135, "hundertachtzig": 180, "zweihundert": 200,
    "zweihundertsiebzig": 270, "dreihundert": 300, "dreihundertsechzig": 360,
}
_BRUECHE = {"halben": 0.5, "halbe": 0.5, "anderthalb": 1.5, "eineinhalb": 1.5}

#: Gesagte Fahrtrichtung -> Richtung in navigation/manoever.py.
_FAHRTRICHTUNGEN = {
    ("vor",): "vor", ("vorwärts",): "vor", ("nach", "vorne"): "vor", ("nach", "vorn"): "vor",
    ("geradeaus",): "vor",
    ("zurück",): "zurueck", ("rückwärts",): "zurueck", ("nach", "hinten"): "zurueck",
    ("nach", "links"): "links", ("links",): "links",
    ("nach", "rechts"): "rechts", ("rechts",): "rechts",
}


def _zahl(wort: str) -> float | None:
    if wort[0].isdigit():
        return float(wort.replace(",", "."))
    wert = _ZAHLWOERTER.get(wort)
    return None if wert is None else float(wert)


def _zahlen_zusammenfassen(woerter: list[str]) -> list:
    """Zahlwörter zu Zahlen: "hundert achtzig" -> 180.0, "einen halben" -> 0.5.

    Vosk schreibt zusammengesetzte Zahlen mal zusammen, mal getrennt.
    """
    aus: list = []
    i = 0
    while i < len(woerter):
        w = woerter[i]
        if w in _BRUECHE:
            if _BRUECHE[w] == 0.5 and aus and aus[-1] == 1.0:
                aus[-1] = 0.5  # "einen halben"
            else:
                aus.append(_BRUECHE[w])
            i += 1
            continue
        wert = _zahl(w)
        if wert is None:
            aus.append(w)
            i += 1
            continue
        i += 1
        while i < len(woerter):
            weiter = _zahl(woerter[i])
            if weiter is None:
                break
            if weiter == 100 and wert < 10:
                wert *= 100  # "drei hundert"
            elif weiter < wert:
                wert += weiter  # "hundert achtzig"
            else:
                break
            i += 1
        aus.append(wert)
    return aus


def manoever_erkennen(woerter: list[str]) -> tuple[str, float] | None:
    """(Richtung, Wert) für einen genauen Fahrbefehl, sonst None.

    ``woerter`` ohne Weckwort. Der Satz muss ganz passen -- ein Wort zu viel,
    und es ist keiner (so prüft das gleich mit, ob es Wortsalat ist):

    * "[fahr|fahre] <Zahl> [meter|zentimeter] <vor|zurück|nach links|...>"
    * "dreh dich um <Zahl> grad [nach links|nach rechts]" (ohne Seite: links)
    * "dreh dich um" (180°), "dreh dich nach links|rechts" (90°)
    """
    t = _zahlen_zusammenfassen(woerter)
    if t[:1] in (["fahr"], ["fahre"]):
        t = t[1:]
    if len(t) >= 2 and isinstance(t[0], float):
        zahl, rest, einheit = t[0], t[1:], 1.0
        if rest[:1] in (["meter"], ["metern"], ["m"]):
            rest = rest[1:]
        elif rest[:1] in (["zentimeter"], ["cm"]):
            rest, einheit = rest[1:], 0.01
        richtung = _FAHRTRICHTUNGEN.get(tuple(rest)) if all(isinstance(w, str) for w in rest) else None
        if richtung is not None and zahl > 0:
            return richtung, round(zahl * einheit, 3)
    if t[:1] in (["dreh"], ["drehe"]):
        rest = t[1:]
        if rest[:1] == ["dich"]:
            rest = rest[1:]
        if rest == ["um"]:
            return "drehen", 180.0
        if rest in (["nach", "links"], ["links"]):
            return "drehen", 90.0
        if rest in (["nach", "rechts"], ["rechts"]):
            return "drehen", -90.0
        if rest[:1] == ["um"]:
            rest = rest[1:]
        if len(rest) >= 2 and isinstance(rest[0], float) and rest[1] == "grad" and rest[0] > 0:
            seite = tuple(rest[2:])
            if seite in ((), ("nach", "links"), ("links",)):
                return "drehen", rest[0]
            if seite in (("nach", "rechts"), ("rechts",)):
                return "drehen", -rest[0]
    return None


def _manoever_saetze() -> list[str]:
    """Die genauen Fahrbefehle für die Vosk-Grammatik.

    Vosk erkennt nur Sätze, die es kennt -- also die gängigen Zahlen
    ausschreiben. Getippt (App, --mock) geht jede Zahl.
    """
    strecken = ["einen meter", "zwei meter", "drei meter", "vier meter", "fünf meter",
                "einen halben meter", "anderthalb meter", "zwanzig zentimeter",
                "fünfzig zentimeter"]
    richtungen = ["vor", "vorwärts", "nach vorne", "zurück", "rückwärts", "nach hinten",
                  "nach links", "nach rechts"]
    saetze = [f"{verb}{s} {r}" for verb in ("fahr ", "fahre ", "") for s in strecken for r in richtungen]
    # Wie im Modell geschrieben (ß); erkannt wird über normalisieren() mit ss.
    winkel = ["dreißig", "fünfundvierzig", "sechzig", "neunzig", "hundertachtzig",
              "hundert achtzig", "dreihundertsechzig", "dreihundert sechzig"]
    for dreh in ("dreh dich", "drehe dich"):
        saetze += [f"{dreh} um", f"{dreh} nach links", f"{dreh} nach rechts"]
        saetze += [f"{dreh} um {w} grad{s}" for w in winkel for s in ("", " nach links", " nach rechts")]
    return saetze


def _weckwoerter() -> set[str]:
    return {w.lower() for w in config.SPRACHE_WECKWORT_VARIANTEN} | {config.SPRACHE_WECKWORT.lower()}


def _enthaelt(woerter: list[str], wendung: list[str]) -> bool:
    n = len(wendung)
    return any(woerter[i:i + n] == wendung for i in range(len(woerter) - n + 1))


def erkennen(text: str, geweckt: bool = False) -> tuple[str | None, bool]:
    """Text -> (Befehl oder None, kam das Weckwort vor?).

    ``geweckt``: Das Weckwort fiel kurz vorher allein -- dann reicht der
    Befehl ohne Namen.
    """
    woerter = normalisieren(text)
    weck = _weckwoerter()
    weckwort = any(w in weck for w in woerter)
    rest = [w for w in woerter if w not in weck]
    # Vor den festen Befehlen: "fahr zwei meter zurück" ist kein Weggehen.
    # Ein genauer Fahrbefehl passt nur als ganzer Satz, Stopp steckt nie darin.
    if rest and manoever_erkennen(rest) is not None:
        return (MANOEVER if weckwort or geweckt else None), weckwort
    for befehl, wendungen in BEFEHLE.items():
        if not any(_enthaelt(rest, w.replace("ß", "ss").split()) for w in wendungen):
            continue
        if weckwort or geweckt or _ohne_weckwort(befehl):
            return befehl, weckwort
        return None, weckwort
    return None, weckwort


def wortsalat(text: str, befehl: str) -> str | None:
    """Warum ``text`` kein sauberer Befehl ist, oder None.

    Ein gesprochener Befehl kommt von Vosk als genau ein Grammatik-Satz
    ("robbie komm her"). Hängt Vosk aus Geräuschen mehrere Sätze
    aneinander, entsteht Salat wie "robin hallo hause zu weg ruby": das
    Weckwort doppelt oder Wörter, die zu keiner Wendung des Befehls gehören.
    """
    woerter = normalisieren(text)
    weck = _weckwoerter()
    rest = [w for w in woerter if w not in weck]
    if len(woerter) - len(rest) > 1:
        return "Weckwort mehrfach"
    if befehl == MANOEVER:
        # Passt nur als ganzer Satz, also schon geprüft.
        return None if manoever_erkennen(rest) is not None else "kein ganzer Fahrbefehl"
    wendungen = [w.replace("ß", "ss").split() for w in BEFEHLE.get(befehl, ())]
    laengste = max((len(w) for w in wendungen if _enthaelt(rest, w)), default=0)
    zusatz = len(rest) - laengste
    if zusatz > config.SPRACHE_MAX_ZUSATZWOERTER:
        return f"{zusatz} Wörter zu viel"
    return None


def _ohne_weckwort(befehl: str) -> bool:
    """Befehle, die auch ohne "Robi" gelten (siehe config)."""
    if befehl == STOPP:
        return config.SPRACHE_STOPP_OHNE_WECKWORT
    if befehl == LADADA:
        return config.SPRACHE_LADADA_OHNE_WECKWORT
    return False


class _MikrofonWechsel(Exception):
    """Ein besseres Mikrofon ist (wieder) da -- Stream schließen und neu suchen."""


def _mikrofon_namen() -> list[str]:
    """Namensteile der Mikrofone in der Reihenfolge der Vorliebe."""
    wunsch = config.SPRACHE_MIKROFON
    if isinstance(wunsch, int):
        return []
    return [wunsch] if wunsch else list(config.SPRACHE_MIKROFON_NAMEN)


def _rang(name: str) -> int | None:
    """Platz des Mikrofons in der Vorliebe (0 = bestes), None = unbekannt."""
    for i, teil in enumerate(_mikrofon_namen()):
        if teil.lower() in name.lower():
            return i
    return None


def _angemeldete_karten() -> str:
    """Soundkarten laut Kernel. Billig und ohne PortAudio, das sich seine
    Geräteliste beim Start merkt und neue Geräte erst nach einem Neustart sieht."""
    try:
        with open("/proc/asound/cards", encoding="utf-8", errors="replace") as f:
            return f.read().lower()
    except OSError:
        return ""


def _ohne_warten(aufgabe, was: str) -> None:
    """Aufräumen, das an einem toten USB-Gerät hängen bleiben kann.

    PortAudio wartet beim Schließen auf ALSA; ist das Gerät mitten im Lauf
    verschwunden, kommt das unter Umständen nie zurück. Dann bleibt der Rest
    liegen (Daemon-Thread), statt dass die Spracherkennung mit hängt.
    """
    t = threading.Thread(target=aufgabe, name="sprache-aufraeumen", daemon=True)
    t.start()
    t.join(config.SPRACHE_SCHLIESSEN_S)
    if t.is_alive():
        log.warning("Spracherkennung: %s hängt -- lasse es liegen", was)


def _pegel(roh: bytes) -> float:
    """Lautstärke eines Blocks (16 bit) als 0..1: -60 dBFS und leiser = 0, Vollaussteuerung = 1."""
    import numpy as np

    werte = np.frombuffer(roh, dtype="<i2").astype(np.float32)
    if werte.size == 0:
        return 0.0
    rms = float(np.sqrt(np.mean(werte * werte))) / 32768.0
    db = 20.0 * np.log10(max(rms, 1e-6))
    return max(0.0, min(1.0, (db + 60.0) / 60.0))


def befehl_aus_text(text: str) -> str | None:
    """Nur der Befehl, ohne vorher gefallenes Weckwort."""
    return erkennen(text)[0]


class Spracherkennung:
    def __init__(self, mock: bool = False, stimme=None, mukke=None) -> None:
        self._mock = mock
        # Nur ``spricht`` wird gebraucht: solange schweigt das Ohr.
        self._stimme = stimme
        # Nur ``laeuft`` wird gebraucht: Das Mikrofon hört die Musik mit.
        self._mukke = mukke
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._befehle: collections.deque[dict] = collections.deque(maxlen=5)

        self._zustand = AUS_GESCHALTET
        self._mikrofon: str | None = None
        self._weck_bis = 0.0  # monotonic; > jetzt heißt "Weckwort gehört"
        self._gehoert: str | None = None
        self._befehl: str | None = None
        self._befehl_zeit = 0.0
        self._erkannt = 0
        self._konfidenz: float | None = None
        self._verworfen = 0
        self._zuletzt_verworfen: str | None = None
        self._fehler: str | None = None

        # Live-Feed für die App (siehe live() und ton_abonnieren()). Pegel und
        # Zwischentext werden nur ausgerechnet, solange jemand mitliest.
        self._live_bis = 0.0  # monotonic
        self._pegel = 0.0  # 0..1
        self._teil = ""  # was Vosk im laufenden Satz bisher versteht
        self._rate: int | None = None
        self._ereignisse: collections.deque[dict] = collections.deque(maxlen=config.SPRACHE_LIVE_EREIGNISSE)
        self._ton_abos: list[queue.Queue] = []

        # Wache über den Hör-Thread (siehe _wachen()): Bleibt er in PortAudio
        # stecken, übernimmt ein neuer. Der alte merkt an _generation, dass er
        # abgelöst ist, falls er je wieder aufwacht.
        self._generation = 0
        self._lebt = 0.0  # monotonic, 0 = Modell lädt noch
        self._modell = None  # (Modell, Grammatik), damit ein Nachfolger nicht neu lädt
        self._wache: threading.Thread | None = None

    # --- Bedienung -------------------------------------------------------------

    @property
    def verfuegbar(self) -> bool:
        return self._zustand in (HOERT, SIMULIERT)

    @property
    def geweckt(self) -> bool:
        return time.monotonic() < self._weck_bis

    def start(self) -> None:
        if not config.SPRACHE_ENABLED:
            log.info("Spracherkennung abgeschaltet (config.SPRACHE_ENABLED)")
            return
        if self._mock:
            # Kein Mikrofon in der Simulation; Befehle kommen über hoeren().
            self._zustand = SIMULIERT
            return
        self._stop.clear()
        self._zustand = LAEDT
        self._lebt = 0.0
        self._hoer_thread_starten()
        self._wache = threading.Thread(target=self._wachen, name="sprache-wache", daemon=True)
        self._wache.start()

    def _hoer_thread_starten(self) -> None:
        self._generation += 1
        self._thread = threading.Thread(
            target=self._loop, args=(self._generation,), name="sprache", daemon=True
        )
        self._thread.start()

    def _wachen(self) -> None:
        """Startet den Hör-Thread neu, wenn er feststeckt.

        Die Fristen in _hoeren_bis_fehler() helfen nur, solange der Thread
        selbst noch läuft. Nach einem USB-Aussetzer kann aber schon das
        Öffnen des Mikrofons in PortAudio hängen bleiben; dann käme ohne
        Wache nie wieder ein Befehl an, bis jemand den Server neu startet.
        Einen Thread abbrechen geht in Python nicht -- er bleibt liegen.
        """
        neu = 0
        while not self._stop.wait(1.0):
            if not self._lebt:
                continue  # Modell lädt noch, das darf dauern
            if self._zustand == HOERT:
                neu = 0
            if time.monotonic() - self._lebt < config.SPRACHE_THREAD_HAENGT_S:
                continue
            if neu >= config.SPRACHE_THREAD_MAX_NEU:
                aufgegeben = "Mikrofon hängt in PortAudio fest -- Server neu starten"
                if self._fehler != aufgegeben:  # nur einmal laut
                    self._fehler = aufgegeben
                    log.error("Spracherkennung: %s", aufgegeben)
                self._zustand = FEHLER
                continue
            neu += 1
            log.warning("Spracherkennung: Hör-Thread hängt seit %.0f s -- starte einen neuen (%d/%d)",
                        time.monotonic() - self._lebt, neu, config.SPRACHE_THREAD_MAX_NEU)
            self._zustand = FEHLER
            self._lebt = time.monotonic()
            self._hoer_thread_starten()

    def stop(self) -> None:
        self._stop.set()
        if self._wache is not None:
            self._wache.join(timeout=2.0)
            self._wache = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._zustand != FEHLER:
            self._zustand = AUS_GESCHALTET

    def hoeren(
        self, text: str, erlaubt: tuple[str, ...] | None = None, grund: str | None = None
    ) -> str | None:
        """Einen gehörten (oder getippten) Satz auswerten. Gibt den Befehl zurück.

        ``erlaubt``: Nur diese Befehle gelten (unsicher Gehörtes, Musik);
        ``grund`` steht dann im Log. Ein so gehörtes Weckwort weckt nicht,
        sonst reichte danach ein einzelnes "komm" aus dem Lied.
        """
        text = " ".join(str(text).split())
        if not text:
            return None
        war_geweckt = self.geweckt
        befehl, weckwort = erkennen(text, war_geweckt)
        verworfen = None
        if erlaubt is not None:
            weckwort = False
            if befehl is not None and befehl not in erlaubt:
                verworfen, befehl = befehl, None
        # Nur ein "Robi" ohne weitere Wörter weckt: Steckt der Name in
        # Wortsalat ("robin la weg"), hat ihn eher ein Geräusch gesagt.
        weckwort = weckwort and all(w in _weckwoerter() for w in normalisieren(text))
        jetzt = time.monotonic()
        with self._lock:
            self._gehoert = text
            if verworfen is not None:
                self._verworfen += 1
                self._zuletzt_verworfen = f"{verworfen}: {text!r} ({grund})"
            if befehl is not None:
                self._weck_bis = 0.0  # verbraucht; der nächste Befehl braucht wieder "Robi"
                self._befehl, self._befehl_zeit = befehl, jetzt
                self._erkannt += 1
                eintrag = {"befehl": befehl, "text": text}
                if befehl == MANOEVER:
                    art, wert = manoever_erkennen(
                        [w for w in normalisieren(text) if w not in _weckwoerter()]
                    )
                    eintrag["wie"] = {"art": art, "wert": wert}
                self._befehle.append(eintrag)
            elif weckwort:
                self._weck_bis = jetzt + config.SPRACHE_WECK_FENSTER_S
                if not war_geweckt:
                    self._befehle.append({"befehl": WECKEN, "text": text})
            if befehl is not None:
                ereignis = {"ergebnis": "befehl", "befehl": befehl}
            elif verworfen is not None:
                ereignis = {"ergebnis": "verworfen", "befehl": verworfen, "grund": grund}
            elif weckwort:
                ereignis = {"ergebnis": "weckwort"}
            else:
                ereignis = {"ergebnis": "ohne"}
            self._ereignisse.append({"zeit": time.time(), "text": text, **ereignis})
            self._teil = ""
        if befehl is not None:
            log.info("Sprachbefehl %s (gehört: %r)", befehl, text)
        elif verworfen is not None:
            log.info("Sprachbefehl %s verworfen, %s (gehört: %r)", verworfen, grund, text)
        else:
            log.debug("Gehört ohne Befehl: %r", text)
        return befehl

    def gehoert_seit(self, zeit: float) -> list[str]:
        """Alles, was seit ``zeit`` (Unix-Zeit) gehört wurde, ältestes zuerst.

        Für Rückfragen wie die des Healthbots ("Geht es Ihnen gut?"): Die
        Antwort ist kein Befehl und steht deshalb nicht in der Schlange,
        sondern nur in den Ereignissen.
        """
        with self._lock:
            return [e["text"] for e in self._ereignisse if e["zeit"] >= zeit]

    def live(self) -> dict:
        """Für das Mitlesen in der App (GET /sprache/live), etwa 5x pro Sekunde.

        Jeder Abruf hält die Live-Auswertung SPRACHE_LIVE_HALTEN_S lang an;
        fragt niemand mehr, rechnet der Hör-Thread Pegel und Zwischentext
        nicht mehr aus.
        """
        self._live_bis = time.monotonic() + config.SPRACHE_LIVE_HALTEN_S
        with self._lock:
            ereignisse = list(self._ereignisse)
        return {
            "verfuegbar": self.verfuegbar,
            "zustand": self._zustand,
            "mikrofon": self._mikrofon,
            "rate": self._rate,
            "pegel": round(self._pegel, 3),
            "teil": self._teil,
            # Während der Roboter spricht, wertet das Ohr nichts aus.
            "taub": bool(self._stimme is not None and self._stimme.spricht),
            "geweckt": self.geweckt,
            "ereignisse": ereignisse,
        }

    @property
    def ton_rate(self) -> int | None:
        """Abtastrate des Mikrofons (Hz), None solange keins offen ist."""
        return self._rate if self._zustand == HOERT else None

    def ton_abonnieren(self) -> queue.Queue:
        """Rohton vom Mikrofon (16 bit, mono, ``live()["rate"]`` Hz) in eine Schlange.

        Für das Mithören in der App (WebSocket /sprache/ton). Kommt der
        Empfänger nicht hinterher, fallen Blöcke weg, statt sich zu stauen.
        """
        abo: queue.Queue = queue.Queue(maxsize=config.SPRACHE_TON_PUFFER)
        with self._lock:
            self._ton_abos.append(abo)
        return abo

    def ton_abmelden(self, abo: queue.Queue) -> None:
        with self._lock:
            if abo in self._ton_abos:
                self._ton_abos.remove(abo)

    def befehle_abholen(self) -> list[dict]:
        with self._lock:
            befehle = list(self._befehle)
            self._befehle.clear()
        return befehle

    def zustand(self) -> dict:
        """Für state.sprache in der App."""
        return {
            "verfuegbar": self.verfuegbar,
            "geweckt": self.geweckt,
            "gehoert": self._gehoert,
            # Nur kurz anzeigen; ein Befehl von vor einer Minute verwirrt.
            "befehl": self._befehl if time.monotonic() - self._befehl_zeit < 5.0 else None,
        }

    def status(self) -> dict:
        with self._lock:
            return {
                "aktiviert": config.SPRACHE_ENABLED,
                "verfuegbar": self.verfuegbar,
                "simuliert": self._mock,
                "zustand": self._zustand,
                "mikrofon": self._mikrofon,
                "modell": os.path.expanduser(config.SPRACHE_MODELL),
                "weckwort": config.SPRACHE_WECKWORT,
                "geweckt": self.geweckt,
                "zuletzt_gehoert": self._gehoert,
                "letzter_befehl": self._befehl,
                "erkannt": self._erkannt,
                # Kleinste Wort-Konfidenz des letzten Satzes vom Mikrofon
                "konfidenz": self._konfidenz,
                "verworfen": self._verworfen,
                "zuletzt_verworfen": self._zuletzt_verworfen,
                "befehle": {b: list(w) for b, w in BEFEHLE.items()},
                "fehler": self._fehler,
            }

    # --- Hör-Thread -------------------------------------------------------------------

    def _loop(self, generation: int) -> None:
        if self._modell is None:
            self._modell = self._modell_laden()
            if self._modell is None:
                return
        sd, vosk, modell, grammatik = self._modell
        abgeloest = lambda: generation != self._generation  # noqa: E731

        while not self._stop.is_set() and not abgeloest():
            self._lebt = time.monotonic()
            warten = config.SPRACHE_NEUVERSUCH_S
            try:
                self._hoeren_bis_fehler(sd, vosk, modell, grammatik, abgeloest)
            except _MikrofonWechsel as exc:
                log.info("Spracherkennung: %s -- wechsle", exc)
                warten = 0.5  # kein Fehler, nur umstecken
            except Exception as exc:  # Mikrofon weg, ALSA-Fehler, ...
                if abgeloest():
                    return  # ein Nachfolger hört schon
                meldung = f"{type(exc).__name__}: {exc}"
                if meldung != self._fehler:
                    # Nur einmal laut, nicht alle paar Sekunden.
                    log.warning("Spracherkennung: %s -- versuche es neu", meldung)
                self._fehler = meldung
                self._zustand = FEHLER
            self._lebt = time.monotonic()
            self._stop.wait(warten)
            # PortAudio merkt sich die Geräteliste vom Start; ein neu
            # angemeldetes Mikrofon sieht es erst nach einem Neustart.
            def neu_starten() -> None:
                try:
                    sd._terminate()
                    sd._initialize()
                except Exception:
                    pass

            _ohne_warten(neu_starten, "PortAudio neu starten")

    def _modell_laden(self):
        """Vosk und Modell laden; None (und Zustand aus), wenn etwas fehlt."""
        try:
            import sounddevice as sd
            import vosk
        except ImportError as exc:
            self._aus(f"{exc.name} fehlt (pip install vosk sounddevice)")
            return
        except OSError as exc:
            # sounddevice ist installiert, aber die C-Bibliothek fehlt:
            # "PortAudio library not found" kommt als OSError, nicht als
            # ImportError -- ohne diesen Zweig stürbe der Hör-Thread still.
            self._aus(f"{exc} (sudo apt install libportaudio2)")
            return
        modell_pfad = os.path.expanduser(config.SPRACHE_MODELL)
        if not os.path.isdir(modell_pfad):
            self._aus(f"Vosk-Modell fehlt: {modell_pfad} (siehe docs/hardware.md)")
            return
        try:
            vosk.SetLogLevel(-1)
            modell = vosk.Model(modell_pfad)
        except Exception as exc:  # kaputtes oder halbes Modell
            self._aus(f"Vosk-Modell lässt sich nicht laden: {exc}")
            return
        grammatik = json.dumps(self._grammatik(modell_pfad, modell), ensure_ascii=False)
        return sd, vosk, modell, grammatik

    def _hoeren_bis_fehler(self, sd, vosk, modell, grammatik: str, abgeloest=lambda: False) -> None:
        geraet, name = self._mikrofon_finden(sd)
        rate = self._abtastrate(sd, geraet)
        # Vosk rechnet intern auf 16 kHz um; das Mikrofon darf bei seiner
        # eigenen Rate bleiben, wenn es 16 kHz nicht kann.
        erkenner = vosk.KaldiRecognizer(modell, rate, grammatik)
        # Einzelne Wörter mit Konfidenz, siehe _auswerten().
        erkenner.SetWords(True)
        block = int(rate * 0.25)
        # Callback statt strom.read(): Ein blockierendes read() kam nach einem
        # USB-Aussetzer manchmal nie wieder zurück, ohne Fehler -- kein
        # Neuversuch griff. Aus der Warteschlange lässt sich mit Frist lesen.
        bloecke: queue.Queue[bytes] = queue.Queue(maxsize=config.SPRACHE_BLOCK_PUFFER)

        def rein(daten, _frames, _zeit, _status) -> None:
            try:
                bloecke.put_nowait(bytes(daten))
            except queue.Full:
                pass  # Auswertung hängt kurz hinterher: lieber Lücke als Stau

        strom = sd.RawInputStream(
            samplerate=rate, blocksize=block, device=geraet, dtype="int16", channels=1,
            callback=rein,
        )
        try:
            strom.start()
            self._mikrofon = name
            self._rate = rate
            self._zustand = HOERT
            if self._fehler:
                log.info("Spracherkennung: Mikrofon wieder da (%s)", name)
            else:
                log.info("Spracherkennung hört auf %s (%d Hz), Weckwort %r",
                         name, rate, config.SPRACHE_WECKWORT)
            self._fehler = None
            taub = False
            # Failsafe nach USB-Aussetzern: toter Stream (nur Stille) und
            # Ersatz-Mikrofon, obwohl das bessere wieder da ist.
            rang = _rang(name)
            geoeffnet = time.monotonic()
            karten_vorher = _angemeldete_karten()
            naechste_pruefung = geoeffnet + config.SPRACHE_MIKROFON_PRUEFEN_S
            stumm_seit: float | None = None
            while not self._stop.is_set() and not abgeloest():
                try:
                    roh = bloecke.get(timeout=config.SPRACHE_HAENGT_S)
                except queue.Empty:
                    raise RuntimeError(
                        f"{name} liefert seit {config.SPRACHE_HAENGT_S:.0f} s keinen Ton (Stream hängt)"
                    ) from None
                jetzt = self._lebt = time.monotonic()
                # Ein echtes Mikrofon rauscht immer ein wenig; lauter exakte
                # Nullen heißen: Der Stream lebt noch, das Gerät nicht mehr.
                if roh.strip(b"\x00"):
                    stumm_seit = None
                elif stumm_seit is None:
                    stumm_seit = jetzt
                elif jetzt - stumm_seit > config.SPRACHE_STUMM_S:
                    raise RuntimeError(f"{name} liefert seit {config.SPRACHE_STUMM_S:.0f} s nur Stille")
                if rang and jetzt >= naechste_pruefung:
                    naechste_pruefung = jetzt + config.SPRACHE_MIKROFON_PRUEFEN_S
                    self._besseres_pruefen(rang, karten_vorher, jetzt - geoeffnet)
                # Mithören und Pegel auch, während er spricht: Das Mikrofon
                # hört ja trotzdem, nur ausgewertet wird es nicht.
                self._ton_verteilen(roh)
                live = time.monotonic() < self._live_bis
                if live:
                    self._pegel = _pegel(roh)
                if self._stimme is not None and self._stimme.spricht:
                    # Weiterlesen, damit kein alter Ton im Puffer bleibt,
                    # aber nichts davon auswerten.
                    taub = True
                    self._teil = ""
                    if self.geweckt:
                        # Das Fenster nach "Robi" läuft erst ab, wenn das
                        # "Ja?" vorbei ist -- solange kann man nichts sagen.
                        self._weck_bis = time.monotonic() + config.SPRACHE_WECK_FENSTER_S
                    continue
                if taub:
                    erkenner.Reset()
                    taub = False
                if erkenner.AcceptWaveform(roh):
                    self._auswerten(json.loads(erkenner.Result()))
                elif live:
                    teil = json.loads(erkenner.PartialResult()).get("partial", "")
                    self._teil = " ".join(teil.replace("[unk]", " ").split())
        finally:
            _ohne_warten(strom.close, "Mikrofon-Stream schließen")

    @staticmethod
    def _besseres_pruefen(rang: int, karten_vorher: str, offen_s: float) -> None:
        """Läuft die Erkennung auf einem Ersatz-Mikrofon, zurück zum besseren.

        Nach einem USB-Aussetzer melden sich die Geräte unterschiedlich schnell
        zurück. Der Neuversuch fand dann nur das schlechtere und blieb dort,
        auch wenn das bessere Mikrofon Sekunden später wieder da war. (Damals
        war das die C270; die ist inzwischen ganz ausgeschlossen.)

        Gewechselt wird, wenn ein besseres neu angemeldet ist. War es schon beim
        Öffnen da (PortAudio fand es trotzdem nicht), nur alle
        SPRACHE_MIKROFON_ERNEUT_S -- sonst stünde der Hör-Thread im Dauerwechsel.
        """
        karten = _angemeldete_karten()
        for teil in _mikrofon_namen()[:rang]:
            teil = teil.lower()
            if teil not in karten:
                continue
            if teil not in karten_vorher or offen_s > config.SPRACHE_MIKROFON_ERNEUT_S:
                raise _MikrofonWechsel(f"besseres Mikrofon ({teil!r}) wieder da")

    def _ton_verteilen(self, roh: bytes) -> None:
        if not self._ton_abos:
            return
        with self._lock:
            abos = list(self._ton_abos)
        for abo in abos:
            try:
                abo.put_nowait(roh)
            except queue.Full:
                pass  # Empfänger zu langsam (WLAN): lieber Lücke als Verzögerung

    def _auswerten(self, ergebnis: dict) -> None:
        """Ein Ergebnis von Vosk prüfen, bevor es als Befehl gelten darf."""
        text = ergebnis.get("text", "").replace("[unk]", " ")
        if not text.strip():
            return
        woerter = ergebnis.get("result") or []
        bekannt = [w for w in woerter if w.get("word") != "[unk]"]
        konfidenz = min((float(w.get("conf", 1.0)) for w in bekannt), default=1.0)
        self._konfidenz = round(konfidenz, 2)
        unbekannt = len(woerter) - len(bekannt)
        befehl, weckwort = erkennen(text, self.geweckt)
        salat = wortsalat(text, befehl) if befehl is not None else None
        # Bei Musik strenger statt taub: Ganz zu sperren hieß, dass nach
        # "Robi, Beatbox" bis zum Liedende kein "Robi" mehr ankam.
        musik = self._mukke is not None and self._mukke.laeuft
        mindestens = config.SPRACHE_MIN_KONFIDENZ_MUSIK if musik else config.SPRACHE_MIN_KONFIDENZ
        grund = None
        if salat is not None:
            grund = f"Wortsalat, {salat}"
        elif konfidenz < mindestens:
            grund = f"unsicher ({konfidenz:.2f})"
        elif woerter and unbekannt / len(woerter) > config.SPRACHE_MAX_UNK_ANTEIL:
            grund = f"{unbekannt} von {len(woerter)} Wörtern unbekannt"
        elif musik and befehl not in (None, STOPP) and not (weckwort or self.geweckt):
            # "La da da" gilt sonst ohne Namen -- das Lied selbst darf
            # kein anderes starten.
            grund = "ohne Weckwort"
        if grund is None:
            self.hoeren(text)
        elif musik:
            self.hoeren(text, erlaubt=(STOPP, MUSIK_AUS), grund=f"{grund}, Musik läuft")
        else:
            self.hoeren(text, erlaubt=(STOPP,), grund=grund)

    def _aus(self, grund: str) -> None:
        self._fehler = grund
        self._zustand = FEHLER
        log.warning("Spracherkennung aus: %s", grund)

    @staticmethod
    def _mikrofon_finden(sd) -> tuple[int, str]:
        geraete = [
            (i, g["name"]) for i, g in enumerate(sd.query_devices())
            if g.get("max_input_channels", 0) > 0
        ]
        wunsch = config.SPRACHE_MIKROFON
        if isinstance(wunsch, int):
            for i, name in geraete:
                if i == wunsch:
                    return i, name
        namen = _mikrofon_namen()
        # Kamera-Mikrofone nie von selbst nehmen: Das der C270 sprang nach
        # USB-Aussetzern ein und verstand kaum etwas. Ein fest eingetragenes
        # Mikrofon hat der Mensch bewusst gewählt, das gilt trotzdem.
        kandidaten = geraete if wunsch else [
            (i, name) for i, name in geraete
            if not any(nie.lower() in name.lower() for nie in config.SPRACHE_MIKROFON_NIE)
        ]
        # Nach Vorliebe suchen, nicht nach Geräteliste: Das bevorzugte
        # Mikrofon soll gewinnen, egal unter welcher Nummer es hängt.
        for teil in namen:
            for i, name in kandidaten:
                if teil.lower() in name.lower():
                    return i, name
        raise RuntimeError(
            "kein Mikrofon gefunden (vorhanden: "
            + (", ".join(n for _, n in geraete) or "keins") + ")"
        )

    @staticmethod
    def _abtastrate(sd, geraet: int) -> int:
        try:
            sd.check_input_settings(
                device=geraet, samplerate=config.SPRACHE_ABTASTRATE, channels=1, dtype="int16"
            )
            return config.SPRACHE_ABTASTRATE
        except Exception:
            return int(sd.query_devices(geraet)["default_samplerate"])

    @staticmethod
    def _grammatik(modell_pfad: str, modell=None) -> list[str]:
        """Alle Sätze, die Vosk erkennen darf.

        Wörter, die das Modell nicht kennt, kann es nicht erkennen -- "robi"
        steht z.B. nicht im kleinen deutschen Modell, "robbie" oder "ruby"
        schon. Deshalb die Varianten gegen den Wortschatz prüfen, statt
        Vosk bei jedem Start mit Warnungen zu fluten.
        """
        bekannt: set[str] | None = None
        woerter_txt = os.path.join(modell_pfad, "graph", "words.txt")
        try:
            with open(woerter_txt, encoding="utf-8") as f:
                bekannt = {zeile.split()[0] for zeile in f if zeile.strip()}
        except OSError:
            pass  # die kleinen Modelle haben keine words.txt, siehe unten

        def kennt(wort: str) -> bool:
            if bekannt is not None:
                return wort in bekannt
            # Ohne words.txt (z.B. vosk-model-small-de-0.15) steckt der
            # Wortschatz nur in graph/Gr.fst; Vosk kann ihn selbst abfragen
            # (-1 = unbekannt). Am echten Modell am 29.10. geprüft.
            finden = getattr(modell, "vosk_model_find_word", None)
            if finden is None:
                return True  # nicht prüfbar: alles versuchen
            try:
                return finden(wort) >= 0
            except Exception:
                return True

        def geht(satz: str) -> bool:
            return all(kennt(w) for w in satz.split())

        weck = [w for w in dict.fromkeys(
            [config.SPRACHE_WECKWORT.lower(), *config.SPRACHE_WECKWORT_VARIANTEN]
        ) if geht(w)]
        if not weck:
            log.warning("Spracherkennung: keine Weckwort-Variante im Modell -- nur Stopp geht")
        else:
            log.info("Spracherkennung: Weckwort als %s", ", ".join(weck))
        wendungen = [w for ws in BEFEHLE.values() for w in ws if geht(w)]
        fahrbefehle = [s for s in _manoever_saetze() if geht(s)]
        log.info("Spracherkennung: %d genaue Fahrbefehle in der Grammatik", len(fahrbefehle))
        wendungen += fahrbefehle
        # Antworten auf Rückfragen (Healthbot), ohne Weckwort und ohne
        # Befehl: Sie landen nur in den Ereignissen (gehoert_seit()).
        antworten = [
            w for w in dict.fromkeys(
                [*config.HEALTH_WOERTER_HILFE, *config.HEALTH_WOERTER_NEIN]
            ) if geht(w)
        ]
        saetze = weck + wendungen + antworten
        # Name vor und nach dem Befehl als ganze Sätze: so passt Vosk das
        # Gesprochene an einen der erlaubten Sätze an, statt Wortsalat zu bauen.
        for w in weck:
            for b in wendungen:
                saetze += [f"{w} {b}", f"{b} {w}"]
        return saetze + ["[unk]"]
