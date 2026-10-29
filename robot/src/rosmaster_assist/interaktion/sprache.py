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
}

# Zustände der Erkennung (status()["zustand"])
AUS_GESCHALTET = "aus"
LAEDT = "laedt"
HOERT = "hoert"
FEHLER = "fehler"
SIMULIERT = "simuliert"


def normalisieren(text: str) -> list[str]:
    """Kleinbuchstaben, ohne Satzzeichen, in Wörter zerlegt."""
    text = str(text).lower().replace("ß", "ss").replace("zurueck", "zurück")
    return re.findall(r"[a-zäöü]+", text)


def _enthaelt(woerter: list[str], wendung: list[str]) -> bool:
    n = len(wendung)
    return any(woerter[i:i + n] == wendung for i in range(len(woerter) - n + 1))


def erkennen(text: str, geweckt: bool = False) -> tuple[str | None, bool]:
    """Text -> (Befehl oder None, kam das Weckwort vor?).

    ``geweckt``: Das Weckwort fiel kurz vorher allein -- dann reicht der
    Befehl ohne Namen.
    """
    woerter = normalisieren(text)
    weck = {w.lower() for w in config.SPRACHE_WECKWORT_VARIANTEN} | {config.SPRACHE_WECKWORT.lower()}
    weckwort = any(w in weck for w in woerter)
    rest = [w for w in woerter if w not in weck]
    for befehl, wendungen in BEFEHLE.items():
        if not any(_enthaelt(rest, w.replace("ß", "ss").split()) for w in wendungen):
            continue
        if weckwort or geweckt or _ohne_weckwort(befehl):
            return befehl, weckwort
        return None, weckwort
    return None, weckwort


def _ohne_weckwort(befehl: str) -> bool:
    """Befehle, die auch ohne "Robi" gelten (siehe config)."""
    if befehl == STOPP:
        return config.SPRACHE_STOPP_OHNE_WECKWORT
    if befehl == LADADA:
        return config.SPRACHE_LADADA_OHNE_WECKWORT
    return False


def befehl_aus_text(text: str) -> str | None:
    """Nur der Befehl, ohne vorher gefallenes Weckwort."""
    return erkennen(text)[0]


class Spracherkennung:
    def __init__(self, mock: bool = False, stimme=None) -> None:
        self._mock = mock
        # Nur ``spricht`` wird gebraucht: solange schweigt das Ohr.
        self._stimme = stimme
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
        self._fehler: str | None = None

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
        self._thread = threading.Thread(target=self._loop, name="sprache", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._zustand != FEHLER:
            self._zustand = AUS_GESCHALTET

    def hoeren(self, text: str) -> str | None:
        """Einen gehörten (oder getippten) Satz auswerten. Gibt den Befehl zurück."""
        text = " ".join(str(text).split())
        if not text:
            return None
        befehl, weckwort = erkennen(text, self.geweckt)
        jetzt = time.monotonic()
        with self._lock:
            self._gehoert = text
            if befehl is not None:
                self._weck_bis = 0.0  # verbraucht; der nächste Befehl braucht wieder "Robi"
                self._befehl, self._befehl_zeit = befehl, jetzt
                self._erkannt += 1
                self._befehle.append({"befehl": befehl, "text": text})
            elif weckwort:
                self._weck_bis = jetzt + config.SPRACHE_WECK_FENSTER_S
        if befehl is not None:
            log.info("Sprachbefehl %s (gehört: %r)", befehl, text)
        else:
            log.debug("Gehört ohne Befehl: %r", text)
        return befehl

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
                "befehle": {b: list(w) for b, w in BEFEHLE.items()},
                "fehler": self._fehler,
            }

    # --- Hör-Thread -------------------------------------------------------------------

    def _loop(self) -> None:
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

        while not self._stop.is_set():
            try:
                self._hoeren_bis_fehler(sd, vosk, modell, grammatik)
            except Exception as exc:  # Mikrofon weg, ALSA-Fehler, ...
                meldung = f"{type(exc).__name__}: {exc}"
                if meldung != self._fehler:
                    # Nur einmal laut, nicht alle paar Sekunden.
                    log.warning("Spracherkennung: %s -- versuche es neu", meldung)
                self._fehler = meldung
                self._zustand = FEHLER
            self._stop.wait(config.SPRACHE_NEUVERSUCH_S)
            # PortAudio merkt sich die Geräteliste vom Start; ein neu
            # angemeldetes Mikrofon sieht es erst nach einem Neustart.
            try:
                sd._terminate()
                sd._initialize()
            except Exception:
                pass

    def _hoeren_bis_fehler(self, sd, vosk, modell, grammatik: str) -> None:
        geraet, name = self._mikrofon_finden(sd)
        rate = self._abtastrate(sd, geraet)
        # Vosk rechnet intern auf 16 kHz um; das Mikrofon darf bei seiner
        # eigenen Rate bleiben, wenn es 16 kHz nicht kann.
        erkenner = vosk.KaldiRecognizer(modell, rate, grammatik)
        block = int(rate * 0.25)
        with sd.RawInputStream(
            samplerate=rate, blocksize=block, device=geraet, dtype="int16", channels=1
        ) as strom:
            self._mikrofon = name
            self._zustand = HOERT
            if self._fehler:
                log.info("Spracherkennung: Mikrofon wieder da (%s)", name)
            else:
                log.info("Spracherkennung hört auf %s (%d Hz), Weckwort %r",
                         name, rate, config.SPRACHE_WECKWORT)
            self._fehler = None
            taub = False
            while not self._stop.is_set():
                daten, _ = strom.read(block)
                if self._stimme is not None and self._stimme.spricht:
                    # Weiterlesen, damit kein alter Ton im Puffer bleibt,
                    # aber nichts davon auswerten.
                    taub = True
                    continue
                if taub:
                    erkenner.Reset()
                    taub = False
                if erkenner.AcceptWaveform(bytes(daten)):
                    text = json.loads(erkenner.Result()).get("text", "")
                    text = text.replace("[unk]", " ")
                    if text.strip():
                        self.hoeren(text)

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
            namen: list[str] = []
            for i, name in geraete:
                if i == wunsch:
                    return i, name
        else:
            namen = [wunsch] if wunsch else list(config.SPRACHE_MIKROFON_NAMEN)
        # Nach Vorliebe suchen, nicht nach Geräteliste: Das USB-Mikrofon
        # soll vor den Mikrofonen in den Kameras gewinnen.
        for teil in namen:
            for i, name in geraete:
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
        saetze = weck + wendungen
        # Name vor und nach dem Befehl als ganze Sätze: so passt Vosk das
        # Gesprochene an einen der erlaubten Sätze an, statt Wortsalat zu bauen.
        for w in weck:
            for b in wendungen:
                saetze += [f"{w} {b}", f"{b} {w}"]
        return saetze + ["[unk]"]
