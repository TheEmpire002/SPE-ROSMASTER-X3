"""Prüft die Sprachbefehle -- ohne Roboter, ohne Mikrofon, ohne Vosk.

    python Marlon/interaktion/test_sprache.py

Getestet wird:

* Befehle gelten nur mit Weckwort "Robi" (davor, danach oder kurz vorher
  allein gesagt), Stopp auch ohne;
* das Weckfenster wird von einem Befehl verbraucht;
* die Grammatik lässt Wörter weg, die das Vosk-Modell nicht kennt;
* ohne Vosk ist die Erkennung aus -- ohne Absturz.

Am Roboter selbst:

    curl -s localhost:8000/sprache/status
    (WebSocket) {"type": "speech", "text": "robi folgen"}
"""

import os
import pathlib
import sys
import tempfile
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaktion import sprache as sp  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


# --- Erkennen (reine Rechnung) -------------------------------------------------------
for text, erwartet in [
    ("robi folgen", sp.FOLGEN),
    ("Robi, folge mir!", sp.FOLGEN),
    ("folgen robi", sp.FOLGEN),
    ("Robbie komm her", sp.HERKOMMEN),
    ("robi komm mit", sp.FOLGEN),  # "komm mit" ist Folgen, nicht Herkommen
    ("ruby geh weg", sp.WEGGEHEN),
    ("robi fahr nach hause", sp.NACH_HAUSE),
    ("robi erkunde", sp.ERKUNDEN),
    ("robi gute nacht", sp.AUS),
    ("hallo robi", sp.START),
    ("robi musik aus", sp.MUSIK_AUS),
    ("robi musik stopp", sp.MUSIK_AUS),  # nicht Anhalten
    ("la da da", sp.LADADA),  # ohne Weckwort
    ("La-da-da!", sp.LADADA),
    ("ladada", sp.LADADA),
    ("la la la", sp.LADADA),
    ("da", None),
    ("Robi, Straße", None),
    ("stopp", sp.STOPP),  # ohne Weckwort
    ("halt", sp.STOPP),
    ("bleib stehen", sp.STOPP),
    ("robi folgen stopp", sp.STOPP),  # Stopp gewinnt gegen alles, was fährt
    ("komm mit", None),  # ohne Weckwort fährt er nicht los
    ("folgen", None),
    ("das wetter ist schön", None),
]:
    befehl = sp.befehl_aus_text(text)
    pruefe(f"{text!r} -> {erwartet}", befehl == erwartet, befehl)

alt = config.SPRACHE_STOPP_OHNE_WECKWORT
config.SPRACHE_STOPP_OHNE_WECKWORT = False
pruefe("Stopp ohne Weckwort abschaltbar", sp.befehl_aus_text("stopp") is None)
config.SPRACHE_STOPP_OHNE_WECKWORT = alt

# --- Weckfenster ------------------------------------------------------------------------
s = sp.Spracherkennung(mock=True)
s.start()
pruefe("Mock ist verfügbar", s.verfuegbar and s.status()["zustand"] == sp.SIMULIERT, s.status())
pruefe("nur Weckwort: kein Befehl", s.hoeren("robi") is None)
pruefe("danach geweckt", s.geweckt and s.zustand()["geweckt"])
pruefe("Befehl im Weckfenster", s.hoeren("folge mir") == sp.FOLGEN)
pruefe("Fenster verbraucht", not s.geweckt)
pruefe("nächster Befehl braucht wieder Robi", s.hoeren("komm her") is None)

alt = config.SPRACHE_WECK_FENSTER_S
config.SPRACHE_WECK_FENSTER_S = 0.1
s.hoeren("robi")
time.sleep(0.2)
pruefe("Fenster läuft ab", s.hoeren("komm her") is None)
config.SPRACHE_WECK_FENSTER_S = alt

abgeholt = s.befehle_abholen()
pruefe("Befehl in der Schlange", abgeholt == [{"befehl": sp.FOLGEN, "text": "folge mir"}], abgeholt)
pruefe("Schlange danach leer", s.befehle_abholen() == [])
pruefe("Zustand zeigt den Befehl", s.zustand()["befehl"] == sp.FOLGEN, s.zustand())
pruefe("leerer Text wird ignoriert", s.hoeren("   ") is None)
for _ in range(10):
    s.hoeren("stopp")
pruefe("Schlange begrenzt", len(s.befehle_abholen()) == 5)
st = s.status()
pruefe("Status vollständig", st["erkannt"] == 11 and st["weckwort"] == config.SPRACHE_WECKWORT, st)
s.stop()

# --- Grammatik gegen den Wortschatz des Modells ---------------------------------------
with tempfile.TemporaryDirectory() as ordner:
    os.makedirs(os.path.join(ordner, "graph"))
    # "robi" kennt das kleine Modell nicht, "robbie" schon.
    wortschatz = {"robbie", "folgen", "folge", "mir", "stopp", "komm", "her"}
    with open(os.path.join(ordner, "graph", "words.txt"), "w", encoding="utf-8") as f:
        for i, w in enumerate(sorted(wortschatz)):
            f.write(f"{w} {i}\n")
    g = sp.Spracherkennung._grammatik(ordner)
    pruefe("unbekanntes Weckwort fehlt", "robi" not in g and "robbie" in g, g[:5])
    pruefe("Sätze mit Weckwort", "robbie folgen" in g and "folge mir robbie" in g)
    pruefe("unbekannte Wendungen fehlen", "geh weg" not in g and "komm her" in g)
    pruefe("alle Wörter bekannt", all(w in wortschatz or w == "[unk]" for s in g for w in s.split()))
    pruefe("Rest als [unk]", g[-1] == "[unk]")

# --- Ohne Vosk: aus, aber kein Absturz -------------------------------------------------
sys.modules["vosk"] = None  # Import schlägt dann sicher fehl
s = sp.Spracherkennung(mock=False)
s.start()
for _ in range(50):
    if s.status()["zustand"] != sp.LAEDT:
        break
    time.sleep(0.05)
st = s.status()
pruefe("ohne Vosk: Fehler statt Absturz", st["zustand"] == sp.FEHLER and not s.verfuegbar, st)
pruefe("Fehler nennt die Lösung", "pip install" in (st["fehler"] or ""), st["fehler"])
s.stop()

print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
