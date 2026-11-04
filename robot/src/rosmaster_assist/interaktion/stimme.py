"""Sprachausgabe über den USB-Lautsprecher.

Der Roboter sagt kurz, was er tut ("Ich folge dir.", "Bin da.") und liest
Texte aus der App vor. Gesprochen wird in einem eigenen Thread mit kleiner
Warteschlange: Ein Satz dauert ein, zwei Sekunden, und so lange darf weder
der Event-Loop noch die Fahrschleife warten.

Ausgabe, je nachdem, was am Pi installiert ist (siehe docs/hardware.md):

``piper``      Piper (offline, natürliche deutsche Stimme) -> aplay
``espeak``     espeak-ng -> aplay, klingt blechern, ist aber fast überall da
``text``       nur ins Log -- der Server läuft trotzdem
``simuliert``  ``--mock``: nur ins Log, Sprechdauer wird nachgeahmt

Der Lautsprecher wird über seinen ALSA-Kartennamen angesprochen, nicht über
die Nummer. Meldet er sich nach einem Spannungseinbruch neu an, findet aplay
ihn beim nächsten Satz unter demselben Namen wieder; ein Satz, der in die
Lücke fällt, geht verloren, mehr nicht.

``spricht`` ist wahr, solange ein Satz läuft (plus kurzem Nachlauf). Die
Spracherkennung soll in dieser Zeit nicht zuhören, sonst versteht der
Roboter sich selbst.
"""

from __future__ import annotations

import collections
import json
import logging
import os
import queue
import re
import shutil
import subprocess
import threading
import time

from .. import config

log = logging.getLogger(__name__)

PIPER = "piper"
ESPEAK = "espeak"
TEXT = "text"
SIMULIERT = "simuliert"


def _leiser(quelle, ziel, faktor: float) -> None:
    """Rohton von Piper (16 bit, mono) um ``faktor`` leiser an aplay weiterreichen."""
    import numpy as np

    rest = b""
    try:
        while True:
            daten = quelle.read1(4096)
            if not daten:
                break
            daten = rest + daten
            # Ein Wert hat zwei Bytes; ein halber wartet auf den nächsten Block.
            gerade = len(daten) - len(daten) % 2
            daten, rest = daten[:gerade], daten[gerade:]
            werte = np.frombuffer(daten, dtype="<i2").astype(np.float32) * faktor
            ziel.write(werte.astype("<i2").tobytes())
    except (OSError, ValueError):
        pass  # aplay weg oder Satz abgebrochen -- dann ist er ohnehin vorbei
    finally:
        for datei in (quelle, ziel):
            try:
                datei.close()
            except OSError:
                pass


class Durchsage:
    """Live-Ton aus der App (16 bit, mono), der über den Lautsprecher geht.

    Steht in derselben Warteschlange wie die Sätze. Dadurch gilt alles, was
    für die Stimme gilt, auch hier: MUKKE pausiert, die Spracherkennung hört
    weg (sonst verstünde sie die Durchsage als Befehl), und Sätze und
    Durchsage reden sich nicht gegenseitig ins Gerät.
    """

    def __init__(self, rate: int) -> None:
        self.rate = rate
        self._bloecke: queue.Queue[bytes | None] = queue.Queue()
        self.vorbei = False
        # aplay ging nicht (Lautsprecher belegt oder weg): Die App soll dann
        # nicht mit jedem Tonpaket gleich die nächste Durchsage beginnen.
        self.fehlgeschlagen = False

    def rein(self, daten: bytes) -> None:
        self._bloecke.put(daten)

    def ende(self) -> None:
        self._bloecke.put(None)

    def __str__(self) -> str:  # für status()["warteschlange"]
        return "(Durchsage)"


class Stimme:
    def __init__(self, mock: bool = False, vor_sprechen=None, nach_sprechen=None) -> None:
        self._mock = mock
        # Rückrufe vor dem ersten und nach dem letzten Satz einer Folge: Der
        # Server räumt damit den Lautsprecher (MUKKE pausiert), denn das
        # ALSA-Gerät lässt sich nur einmal öffnen.
        self._vor_sprechen = vor_sprechen
        self._nach_sprechen = nach_sprechen
        self._lock = threading.Lock()
        self._neu = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._warteschlange: collections.deque[str] = collections.deque()
        self._prozesse: list[subprocess.Popen] = []
        self._laufende: Durchsage | None = None  # die gerade spielt

        self._ausgabe = TEXT
        self._piper: str | None = None  # gefundener Pfad, siehe _ausgabe_finden()
        self._abtastrate = 22050
        self._spricht_bis = 0.0  # monotonic; > jetzt heißt "spricht"
        self._sprechend = False
        self._zuletzt: str | None = None
        self._zuletzt_zeit = 0.0
        self._gesagt = 0
        self._verworfen = 0
        self._fehler: str | None = None

    # --- Bedienung -------------------------------------------------------------

    @property
    def verfuegbar(self) -> bool:
        """Kommt tatsächlich Ton heraus (oder wird er simuliert)?"""
        return config.STIMME_ENABLED and self._ausgabe != TEXT

    @property
    def spricht(self) -> bool:
        return self._sprechend or time.monotonic() < self._spricht_bis

    @property
    def beschaeftigt(self) -> bool:
        """Spricht gerade oder hat noch Sätze in der Schlange."""
        with self._lock:
            wartet = bool(self._warteschlange)
        return wartet or self.spricht

    def start(self) -> None:
        if not config.STIMME_ENABLED:
            log.info("Stimme abgeschaltet (config.STIMME_ENABLED)")
            return
        self._ausgabe = self._ausgabe_finden()
        log.info("Stimme: Ausgabe %s auf %s", self._ausgabe, config.STIMME_GERAET)
        if self._ausgabe in (PIPER, ESPEAK):
            self._lautstaerke_setzen()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="stimme", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.verstummen()
        self._neu.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def sagen(self, text: str, vorrang: bool = False) -> bool:
        """Satz in die Warteschlange. False, wenn er verworfen wurde.

        ``vorrang`` bricht den laufenden Satz ab und leert die Schlange --
        für Dinge, die jetzt wichtiger sind als alles davor (Stopp).
        """
        text = " ".join(str(text).split())[: config.STIMME_MAX_ZEICHEN]
        if not text or not config.STIMME_ENABLED:
            return False
        jetzt = time.monotonic()
        with self._lock:
            # Derselbe Satz kurz hintereinander nervt nur.
            if (
                not vorrang
                and text == self._zuletzt
                and jetzt - self._zuletzt_zeit < config.STIMME_WIEDERHOLUNG_S
            ):
                return False
            if text in self._warteschlange:
                return False
            self._zuletzt = text
            self._zuletzt_zeit = jetzt
            if vorrang:
                self._verworfen += len(self._warteschlange)
                self._leeren()
            while len(self._warteschlange) >= config.STIMME_MAX_WARTESCHLANGE:
                # Das Älteste fällt weg: Es beschreibt, was längst vorbei ist.
                alt = self._warteschlange.popleft()
                if isinstance(alt, Durchsage):
                    alt.vorbei = True
                self._verworfen += 1
            self._warteschlange.append(text)
        if vorrang:
            self._prozesse_beenden()
        self._neu.set()
        return True

    def durchsage(self, rate: int) -> Durchsage:
        """Eine Durchsage beginnen; der Ton kommt danach mit ``rein()``.

        Läuft gerade ein Satz, kommt sie direkt danach. Ein Satz mit Vorrang
        (Stopp) bricht sie ab, dann ist ``vorbei`` wahr.
        """
        d = Durchsage(rate)
        if not config.STIMME_ENABLED:
            d.vorbei = True
            return d
        with self._lock:
            self._warteschlange.append(d)
        self._neu.set()
        return d

    def verstummen(self) -> None:
        """Schlange leeren und den laufenden Satz abbrechen (Not-Stopp)."""
        with self._lock:
            self._verworfen += len(self._warteschlange)
            self._leeren()
        self._prozesse_beenden()

    def _leeren(self) -> None:
        """Schlange leeren (unter self._lock). Wartende Durchsagen erfahren,
        dass sie vorbei sind -- sonst schickte die App ins Leere."""
        for eintrag in self._warteschlange:
            if isinstance(eintrag, Durchsage):
                eintrag.vorbei = True
        self._warteschlange.clear()

    def zustand(self) -> dict:
        """Für state.stimme in der App."""
        return {
            "verfuegbar": self.verfuegbar,
            "spricht": self.spricht,
            "text": self._zuletzt,
        }

    def status(self) -> dict:
        with self._lock:
            return {
                "aktiviert": config.STIMME_ENABLED,
                "verfuegbar": self.verfuegbar,
                "simuliert": self._mock,
                "ausgabe": self._ausgabe,
                "geraet": config.STIMME_GERAET,
                "spricht": self.spricht,
                "warteschlange": [str(t) for t in self._warteschlange],
                "zuletzt": self._zuletzt,
                "gesagt": self._gesagt,
                "verworfen": self._verworfen,
                "fehler": self._fehler,
            }

    # --- Sprech-Thread ------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._neu.wait(timeout=1.0)
            self._neu.clear()
            # Lautsprecher einmal pro Folge von Sätzen räumen, nicht vor
            # jedem Satz: Musik zwischen zwei Sätzen kurz anzuspielen klänge
            # nach Störung.
            geraeumt = False
            while not self._stop.is_set():
                with self._lock:
                    if not self._warteschlange:
                        break
                    text = self._warteschlange.popleft()
                    # Nur-Text ist nicht hörbar -- dann muss auch niemand
                    # weghören. Gleich unter der Sperre setzen, sonst wäre
                    # ``beschaeftigt`` bis zum ersten Ton kurz falsch.
                    hoerbar = self._ausgabe != TEXT
                    self._sprechend = hoerbar
                if hoerbar and not geraeumt:
                    geraeumt = True
                    self._rufen(self._vor_sprechen)
                    # Nach einem USB-Aussetzer meldet sich der Lautsprecher
                    # mit seinem Standard-Mixer neu an -- also jedes Mal setzen.
                    if self._ausgabe in (PIPER, ESPEAK):
                        self._lautstaerke_setzen(melden=False)
                self._sprechend = hoerbar
                try:
                    self._sprechen(text)
                except Exception as exc:  # nie den Thread verlieren
                    self._fehler = f"{type(exc).__name__}: {exc}"
                    log.exception("Stimme: Satz fehlgeschlagen")
                finally:
                    if hoerbar:
                        self._spricht_bis = time.monotonic() + config.STIMME_NACHLAUF_S
                    self._sprechend = False
            if geraeumt:
                self._rufen(self._nach_sprechen)

    @staticmethod
    def _rufen(rueckruf) -> None:
        if rueckruf is None:
            return
        try:
            rueckruf()
        except Exception:  # Musik darf die Stimme nicht abschalten
            log.exception("Stimme: Rückruf fehlgeschlagen")

    def _durchsage_abspielen(self, d: Durchsage) -> None:
        if d.vorbei:
            return  # in der Schlange schon abgebrochen
        log.info("Stimme: Durchsage aus der App (%d Hz)", d.rate)
        faktor = max(0.0, min(100.0, float(config.LAUTSTAERKE))) / 100.0
        ziel = None
        if self._ausgabe not in (SIMULIERT, TEXT):
            ziel = subprocess.Popen(
                ["aplay", "-q", "-D", config.STIMME_GERAET, "-r", str(d.rate), "-f", "S16_LE",
                 "-c", "1", "-t", "raw", f"--buffer-time={config.DURCHSAGE_APLAY_PUFFER_US}"],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        with self._lock:
            self._prozesse = [ziel] if ziel is not None else []
            self._laufende = d
        # Was sich angestaut hat, während noch ein Satz lief, ist überholt.
        max_bytes = int(d.rate * 2 * config.DURCHSAGE_MAX_PUFFER_S)
        import numpy as np

        try:
            while not self._stop.is_set():
                try:
                    daten = d._bloecke.get(timeout=config.DURCHSAGE_PAUSE_S)
                except queue.Empty:
                    break  # App still oder weg: Lautsprecher freigeben
                if daten is None:
                    break
                stau = d._bloecke.qsize() * len(daten)
                while stau > max_bytes:
                    try:
                        weg = d._bloecke.get_nowait()
                    except queue.Empty:
                        break
                    if weg is None:
                        return
                    stau -= len(weg)
                if ziel is None:
                    continue  # Simulation: Ton verwerfen, aber "spricht" bleibt wahr
                if ziel.poll() is not None:
                    break  # abgebrochen (Vorrang) oder Lautsprecher weg
                daten = daten[: len(daten) - len(daten) % 2]
                if faktor < 0.999:
                    werte = np.frombuffer(daten, dtype="<i2").astype(np.float32) * faktor
                    daten = werte.astype("<i2").tobytes()
                ziel.stdin.write(daten)
                ziel.stdin.flush()
        except (OSError, ValueError):
            pass  # aplay weg (Vorrang, USB) -- die Durchsage ist dann vorbei
        finally:
            d.vorbei = True
            if ziel is not None:
                try:
                    ziel.stdin.close()
                    ziel.wait(timeout=2.0)
                except (OSError, ValueError, subprocess.TimeoutExpired):
                    pass
                self._prozesse_beenden()
                # Positiver Code: aplay selbst ging nicht (belegt, weg).
                # Negativ hieße abgebrochen (Vorrang) -- das ist kein Fehler.
                if (ziel.returncode or 0) > 0:
                    d.fehlgeschlagen = True
                    meldung = f"Durchsage: Lautsprecher geht nicht (aplay Code {ziel.returncode})"
                    if meldung != self._fehler:
                        log.warning("Stimme: %s", meldung)
                    self._fehler = meldung
            log.info("Stimme: Durchsage beendet")

    def _sprechen(self, text: str | Durchsage) -> None:
        if isinstance(text, Durchsage):
            self._durchsage_abspielen(text)
            return
        log.info("Stimme sagt: %s", text)
        if self._ausgabe == SIMULIERT:
            # Ungefähr so lange wie echtes Sprechen, damit ``spricht`` sich
            # wie am Roboter verhält.
            ende = time.monotonic() + min(5.0, 0.3 + 0.06 * len(text))
            while time.monotonic() < ende and not self._stop.is_set():
                time.sleep(0.05)
            self._gesagt += 1
            return
        if self._ausgabe == TEXT:
            return

        # Der eine Lautstärkeregler (config.LAUTSTAERKE); der Mixer steht fest.
        faktor = max(0.0, min(100.0, float(config.LAUTSTAERKE))) / 100.0
        aplay = ["aplay", "-q", "-D", config.STIMME_GERAET]
        if self._ausgabe == PIPER:
            erzeuger = [
                self._piper,
                "--model", os.path.expanduser(config.STIMME_MODELL),
                "--length_scale", str(config.STIMME_TEMPO),
                "--output-raw",
            ]
            if config.STIMME_KERNE and shutil.which("taskset"):
                erzeuger = ["taskset", "-c", str(config.STIMME_KERNE), *erzeuger]
            aplay += ["-r", str(self._abtastrate), "-f", "S16_LE", "-c", "1", "-t", "raw"]
        else:
            # espeak regelt selbst (Amplitude, 100 = normal). Seine Ausgabe
            # ist WAV mit Kopf, die ließe sich nicht einfach skalieren.
            erzeuger = [config.STIMME_ESPEAK, "-v", "de", "-s", "150",
                        "-a", str(round(100 * faktor)), "--stdout"]
        # Piper liefert rohe 16-bit-Werte: die rechnen wir selbst leiser.
        regeln = self._ausgabe == PIPER and faktor < 0.999

        quelle = subprocess.Popen(
            erzeuger, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        ziel = subprocess.Popen(
            aplay, stdin=subprocess.PIPE if regeln else quelle.stdout,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        )
        if regeln:
            threading.Thread(
                target=_leiser, args=(quelle.stdout, ziel.stdin, faktor), name="stimme-leiser", daemon=True
            ).start()
        else:
            quelle.stdout.close()  # gehört jetzt aplay -- sonst endet aplay nie
        with self._lock:
            self._prozesse = [quelle, ziel]
        try:
            quelle.stdin.write((text + "\n").encode("utf-8"))
            quelle.stdin.close()
            # wait statt communicate: communicate schlösse aplays Eingang,
            # in den _leiser() noch schreibt. aplay meldet nur wenige Zeilen.
            ziel.wait(timeout=config.STIMME_TIMEOUT_S)
            fehler = ziel.stderr.read()
            quelle.wait(timeout=2.0)
        except BrokenPipeError:
            fehler = b"Ausgabeprogramm vorzeitig beendet"
        except subprocess.TimeoutExpired:
            fehler = b"Zeitlimit beim Sprechen"
        finally:
            self._prozesse_beenden()

        if ziel.returncode == 0 and (quelle.returncode or 0) > 0:
            # aplay lief, bekam aber nichts: Piper/espeak ist abgestürzt
            # (z.B. kaputtes Modell).
            meldung = f"{erzeuger[0]} Code {quelle.returncode}"
            if meldung != self._fehler:
                log.warning("Stimme: Sprachsynthese fehlgeschlagen: %s", meldung)
            self._fehler = meldung
        elif ziel.returncode == 0:
            if self._fehler:
                log.info("Stimme: Lautsprecher wieder da")
            self._fehler = None
            self._gesagt += 1
        elif ziel.returncode is not None and ziel.returncode < 0:
            pass  # absichtlich abgebrochen (verstummen / Vorrang)
        else:
            meldung = (fehler or b"").decode("utf-8", "replace").strip() or f"aplay Code {ziel.returncode}"
            if meldung != self._fehler:
                # Nur einmal laut melden, nicht bei jedem Satz, solange der
                # Lautsprecher fehlt.
                log.warning("Stimme: Ausgabe fehlgeschlagen: %s", meldung)
            self._fehler = meldung

    def _prozesse_beenden(self) -> None:
        with self._lock:
            prozesse, self._prozesse = self._prozesse, []
            laufende, self._laufende = self._laufende, None
        if laufende is not None:
            # Auch ohne aplay (Simulation) sofort beenden: ende() weckt den
            # wartenden Sprech-Thread, vorbei lässt die App neu beginnen.
            laufende.vorbei = True
            laufende.ende()
        for p in prozesse:
            if p.poll() is None:
                try:
                    p.kill()
                except OSError:
                    pass

    # --- Einrichtung -----------------------------------------------------------------

    def _ausgabe_finden(self) -> str:
        if self._mock:
            return SIMULIERT
        if shutil.which("aplay") is None:
            self._fehler = "aplay fehlt (Paket alsa-utils)"
            log.warning("Stimme: %s -- nur Text", self._fehler)
            return TEXT
        modell = os.path.expanduser(config.STIMME_MODELL)
        # Das Piper-Release bringt seine Bibliotheken im eigenen Ordner mit;
        # deshalb dort suchen, nicht nur im PATH.
        self._piper = next(
            (p for p in (shutil.which(os.path.expanduser(n)) for n in config.STIMME_PIPER) if p),
            None,
        )
        if self._piper and os.path.isfile(modell):
            self._abtastrate = self._abtastrate_lesen(modell)
            return PIPER
        if shutil.which(config.STIMME_ESPEAK):
            log.warning("Stimme: Piper oder Modell %s fehlt -- weiche auf espeak-ng aus", modell)
            return ESPEAK
        self._fehler = "weder Piper noch espeak-ng installiert"
        log.warning("Stimme: %s -- nur Text", self._fehler)
        return TEXT

    @staticmethod
    def _abtastrate_lesen(modell: str) -> int:
        """Piper legt die Abtastrate in ``<modell>.json`` ab; aplay muss sie kennen."""
        try:
            with open(modell + ".json", encoding="utf-8") as f:
                return int(json.load(f)["audio"]["sample_rate"])
        except (OSError, ValueError, KeyError, TypeError):
            return 22050

    @staticmethod
    def _lautstaerke_setzen(melden: bool = True) -> None:
        """Mixer per amixer fest auf TON_MIXER. Der Regler heißt je nach Gerät anders.

        Leiser wird in Software (LAUTSTAERKE), damit es nur einen Regler gibt.
        """
        treffer = re.search(r"CARD=([^,]+)", config.STIMME_GERAET)
        if treffer is None or shutil.which("amixer") is None:
            return
        karte = treffer.group(1)
        for regler in ("PCM", "Speaker", "Master", "Headphone"):
            try:
                ergebnis = subprocess.run(
                    ["amixer", "-q", "-c", karte, "sset", regler, f"{config.TON_MIXER}%"],
                    capture_output=True,
                    timeout=3.0,
                )
            except (OSError, subprocess.TimeoutExpired):
                return
            if ergebnis.returncode == 0:
                if melden:
                    log.info("Stimme: Mixer %d %% (%s), Lautstärke %d %% in Software",
                             config.TON_MIXER, regler, config.LAUTSTAERKE)
                return
