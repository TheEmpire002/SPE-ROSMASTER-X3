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
    ("robi beatbox", sp.BEATBOX),
    ("robbie beat box", sp.BEATBOX),
    ("beatbox", None),  # ohne Weckwort nicht
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
pruefe("Robi allein meldet sich zum Antworten", abgeholt[0] == {"befehl": sp.WECKEN, "text": "robi"}, abgeholt)
befehle = [e for e in abgeholt if e["befehl"] != sp.WECKEN]
pruefe("Befehl in der Schlange", befehle == [{"befehl": sp.FOLGEN, "text": "folge mir"}], abgeholt)
s.hoeren("robin la")
pruefe("Robi im Wortsalat weckt nicht", not s.geweckt and s.befehle_abholen() == [])
pruefe("Schlange danach leer", s.befehle_abholen() == [])
pruefe("Zustand zeigt den Befehl", s.zustand()["befehl"] == sp.FOLGEN, s.zustand())
pruefe("leerer Text wird ignoriert", s.hoeren("   ") is None)
for _ in range(10):
    s.hoeren("stopp")
pruefe("Schlange begrenzt", len(s.befehle_abholen()) == 5)
st = s.status()
pruefe("Status vollständig", st["erkannt"] == 11 and st["weckwort"] == config.SPRACHE_WECKWORT, st)
s.stop()

# --- Genaue Fahrbefehle mit Zahl -----------------------------------------------------
for text, erwartet in [
    ("fahr zwei meter vor", ("vor", 2.0)),
    ("fahre einen meter zurück", ("zurueck", 1.0)),
    ("einen halben meter nach links", ("links", 0.5)),
    ("fahr anderthalb meter nach rechts", ("rechts", 1.5)),
    ("fahr zwanzig zentimeter vorwärts", ("vor", 0.2)),
    ("fahr 1,5 m vor", ("vor", 1.5)),
    ("dreh dich um neunzig grad", ("drehen", 90.0)),
    ("dreh dich um neunzig grad nach rechts", ("drehen", -90.0)),
    ("drehe dich um hundert achtzig grad", ("drehen", 180.0)),
    ("dreh dich um dreihundertsechzig grad nach links", ("drehen", 360.0)),
    ("dreh dich um", ("drehen", 180.0)),
    ("dreh dich nach rechts", ("drehen", -90.0)),
    ("fahr zwei meter vor bitte", None),       # ein Wort zu viel
    ("zwei meter", None),                      # keine Richtung
    ("dreh dich um neunzig", None),            # kein "Grad"
]:
    ergebnis = sp.manoever_erkennen(sp.normalisieren(text))
    pruefe(f"Fahrbefehl {text!r} -> {erwartet}", ergebnis == erwartet, ergebnis)

pruefe("mit Robi: Fahrbefehl", sp.erkennen("robi fahr zwei meter zurück") == (sp.MANOEVER, True))
pruefe("'zwei meter zurück' ist kein Weggehen",
       sp.erkennen("robbie zwei meter zurück")[0] == sp.MANOEVER)
pruefe("ohne Robi: nichts", sp.erkennen("fahr zwei meter vor") == (None, False))
pruefe("'fahr zurück' bleibt Weggehen", sp.erkennen("robi fahr zurück")[0] == sp.WEGGEHEN)
pruefe("Fahrbefehl ist kein Wortsalat", sp.wortsalat("robi fahr zwei meter vor", sp.MANOEVER) is None)
pruefe("Fahrbefehl mit doppeltem Robi ist Salat",
       sp.wortsalat("robi fahr zwei meter vor robi", sp.MANOEVER) is not None)
s = sp.Spracherkennung(mock=True)
s.start()
pruefe("Fahrbefehl gehört", s.hoeren("Robi, fahr zwei Meter vor") == sp.MANOEVER)
abgeholt = s.befehle_abholen()
pruefe("Fahrbefehl mit Richtung und Wert in der Schlange",
       abgeholt and abgeholt[0].get("wie") == {"art": "vor", "wert": 2.0}, abgeholt)
s.stop()
saetze = sp._manoever_saetze()
pruefe("Grammatik enthält Fahrbefehle", "fahr zwei meter vor" in saetze
       and "dreh dich um neunzig grad nach rechts" in saetze, len(saetze))
pruefe("jeder Grammatik-Satz wird verstanden",
       all(sp.manoever_erkennen(sp.normalisieren(x)) is not None for x in saetze),
       [x for x in saetze if sp.manoever_erkennen(sp.normalisieren(x)) is None][:3])

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

# --- Hängender Stream: Fehler statt ewig warten ----------------------------------------
# Nach einem USB-Aussetzer kam vom Mikrofon nichts mehr, ohne Fehlermeldung.
class _TotesSd:
    class RawInputStream:
        geschlossen = False

        def __init__(self, **kw):
            self.callback = kw["callback"]  # wird nie aufgerufen

        def start(self):
            pass

        def close(self):
            type(self).geschlossen = True


class _Vosk:
    class KaldiRecognizer:
        def __init__(self, *a):
            pass

        def SetWords(self, _):
            pass


s = sp.Spracherkennung(mock=False)
s._mikrofon_finden = lambda sd: (0, "USB Microphone")
s._abtastrate = lambda sd, g: 16000
t0 = time.monotonic()
try:
    s._hoeren_bis_fehler(_TotesSd, _Vosk, None, "[]")
    meldung = ""
except RuntimeError as exc:
    meldung = str(exc)
dauer = time.monotonic() - t0
pruefe("hängender Stream wird erkannt", "hängt" in meldung, meldung)
pruefe("nach der Frist, nicht ewig", dauer < config.SPRACHE_HAENGT_S + 2, dauer)
pruefe("toter Stream wird geschlossen", _TotesSd.RawInputStream.geschlossen)


# --- Hör-Thread steckt fest: Wache startet neu, dann Fehler ----------------------------
class _HaengendesSd:
    _terminate = _initialize = staticmethod(lambda: None)

    class RawInputStream:
        def __init__(self, **kw):
            threading.Event().wait()  # Öffnen hängt in PortAudio, für immer


import threading  # noqa: E402

alt = config.SPRACHE_THREAD_HAENGT_S, config.SPRACHE_ENABLED
config.SPRACHE_THREAD_HAENGT_S, config.SPRACHE_ENABLED = 0.5, True
s = sp.Spracherkennung(mock=False)
s._modell = (_HaengendesSd, _Vosk, None, "[]")  # Modell "schon geladen"
s._mikrofon_finden = lambda sd: (0, "USB Microphone")
s._abtastrate = lambda sd, g: 16000
s.start()
time.sleep(config.SPRACHE_THREAD_MAX_NEU + 3)
st = s.status()
pruefe("Wache startet hängenden Hör-Thread neu",
       s._generation == 1 + config.SPRACHE_THREAD_MAX_NEU, s._generation)
pruefe("danach Fehler mit Hinweis", st["zustand"] == sp.FEHLER and "neu starten" in (st["fehler"] or ""), st)
s.stop()
config.SPRACHE_THREAD_HAENGT_S, config.SPRACHE_ENABLED = alt

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
