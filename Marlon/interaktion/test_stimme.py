"""Prüft die Sprachausgabe -- ohne Roboter und ohne Lautsprecher.

    python Marlon/interaktion/test_stimme.py

Getestet wird:

* Warteschlange: Wiederholungen werden verschluckt, die Länge ist begrenzt,
  ``vorrang`` und ``verstummen`` leeren sie;
* ``spricht`` ist während eines Satzes wahr und danach wieder falsch;
* fehlen Piper und espeak-ng, bleibt nur Text -- ohne Absturz;
* das Verhalten sagt bei Gesten und Moduswechseln die passenden Sätze.

Am Roboter selbst hört man den Lautsprecher am einfachsten so:

    curl -s localhost:8000/stimme/status
    (WebSocket) {"type": "say", "text": "Hallo, ich bin der Rosmaster."}
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaktion import gesten as gs  # noqa: E402
from rosmaster_assist.interaktion import stimme as st  # noqa: E402
from rosmaster_assist.interaktion import verhalten as vh  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


# --- Warteschlange (ohne laufenden Thread) ----------------------------------------
s = st.Stimme(mock=True)
pruefe("Satz angenommen", s.sagen("Ich folge dir."))
pruefe("Wiederholung verschluckt", not s.sagen("Ich folge dir."))
pruefe("Leertext verworfen", not s.sagen("   "))
for i in range(config.STIMME_MAX_WARTESCHLANGE + 2):
    s.sagen(f"Satz {i}")
schlange = s.status()["warteschlange"]
pruefe("Schlange begrenzt", len(schlange) == config.STIMME_MAX_WARTESCHLANGE, schlange)
pruefe("Ältestes fällt weg", schlange[-1] == f"Satz {config.STIMME_MAX_WARTESCHLANGE + 1}", schlange)
s.sagen("Halt.", vorrang=True)
pruefe("Vorrang leert die Schlange", s.status()["warteschlange"] == ["Halt."], s.status()["warteschlange"])
s.verstummen()
pruefe("Verstummen leert die Schlange", s.status()["warteschlange"] == [])
lang = s.sagen("x" * 1000) and s.status()["warteschlange"][0]
pruefe("Lange Texte gekürzt", len(lang) == config.STIMME_MAX_ZEICHEN, len(lang))

# --- Sprech-Thread im Mock ----------------------------------------------------------
s = st.Stimme(mock=True)
s.start()
pruefe("Mock ist verfügbar", s.verfuegbar and s.status()["ausgabe"] == st.SIMULIERT, s.status())
s.sagen("Hallo")
time.sleep(0.2)
pruefe("spricht während des Satzes", s.spricht)
time.sleep(0.6 + config.STIMME_NACHLAUF_S)
pruefe("danach still", not s.spricht and s.status()["gesagt"] == 1, s.status())
s.stop()

# --- Ohne Sprachprogramme: nur Text --------------------------------------------------
alt = (config.STIMME_PIPER, config.STIMME_ESPEAK)
config.STIMME_PIPER = config.STIMME_ESPEAK = "gibt-es-nicht-xyz"
s = st.Stimme(mock=False)
s.start()
s.sagen("Niemand hört mich.")
time.sleep(0.2)
pruefe("ohne Synthese nur Text", s.status()["ausgabe"] == st.TEXT and not s.verfuegbar, s.status())
pruefe("Thread lebt weiter", s._thread is not None and s._thread.is_alive())
s.stop()
config.STIMME_PIPER, config.STIMME_ESPEAK = alt


# --- Verhalten spricht -----------------------------------------------------------------
class Mitschrift:
    def __init__(self):
        self.saetze = []

    def sagen(self, text, vorrang=False):
        self.saetze.append((text, vorrang))
        return True


class Watchdog:
    def submit(self, cmd):
        pass

    def stop(self):
        pass


mit = Mitschrift()
ia = vh.Interaktion(Watchdog(), None, None, mit)
ia._wechseln(vh.FOLGEN, "folgt", 0.0, sagen="Ich folge dir.")
pruefe("Moduswechsel wird gesagt", mit.saetze[-1] == ("Ich folge dir.", False), mit.saetze)


class Gesten:
    """Liefert genau eine Geste, wie die echte Erkennung nach einer Bewegung."""

    zuletzt = zuletzt_zeit = None

    def __init__(self, geste):
        self.geste = geste

    def update(self, ziel, zeit, ruhig=True):
        return self.geste


for geste, satz, vorrang in [
    (gs.STOPP, "Okay, ich halte an.", True),
    (gs.HERKOMMEN, "Ich komme.", False),
]:
    ia._gesten = Gesten(geste)
    ia._letztes_bild = -1.0
    ia._geste_pruefen({"zeit": time.monotonic()}, time.monotonic())
    pruefe(f"Geste {geste} wird beantwortet", mit.saetze[-1] == (satz, vorrang), mit.saetze[-1])

ia = vh.Interaktion(Watchdog(), None, None)  # ohne Stimme darf nichts passieren
ia._wechseln(vh.FOLGEN, "folgt", 0.0, sagen="Ich folge dir.")
pruefe("ohne Stimme kein Fehler", True)

print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
