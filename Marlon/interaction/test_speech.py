"""Checks the speech commands -- without robot, microphone or Vosk.

    python Marlon/interaction/test_speech.py

Tested:

* commands only count with the wake word "Robi" (before, after, or said
  alone shortly before), stop also without it;
* the wake window is consumed by one command;
* the grammar leaves out words the Vosk model does not know;
* without Vosk recognition is off -- without crashing.

On the robot itself:

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
from rosmaster_assist.interaction import speech as sp  # noqa: E402

error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, result)
    error += not condition


# --- Recognition (pure computation) -------------------------------------------------
for text, expected in [
    ("robi folgen", sp.FOLLOW),
    ("Robi, folge mir!", sp.FOLLOW),
    ("folgen robi", sp.FOLLOW),
    ("Robbie komm her", sp.COME_HERE),
    ("robi komm mit", sp.FOLLOW),  # "komm mit" is follow, not come here
    ("ruby geh weg", sp.GO_AWAY),
    ("robi fahr nach hause", sp.GO_HOME),
    ("robi erkunde", sp.EXPLORE),
    ("robi gute nacht", sp.OFF),
    ("hallo robi", sp.START),
    ("robi musik aus", sp.MUSIC_OFF),
    ("robi musik stopp", sp.MUSIC_OFF),  # not stop
    ("la da da", sp.LADADA),  # without wake word
    ("La-da-da!", sp.LADADA),
    ("ladada", sp.LADADA),
    ("la la la", sp.LADADA),
    ("da", None),
    ("robi beatbox", sp.BEATBOX),
    ("robbie beat box", sp.BEATBOX),
    ("beatbox", None),  # not without wake word
    ("Robi, Straße", None),
    ("stopp", sp.STOP),  # without wake word
    ("halt", sp.STOP),
    ("bleib stehen", sp.STOP),
    ("robi folgen stopp", sp.STOP),  # stop beats everything that drives
    ("komm mit", None),  # without wake word it does not start driving
    ("folgen", None),
    ("das wetter ist schön", None),
]:
    command = sp.command_from_text(text)
    check(f"{text!r} -> {expected}", command == expected, command)

old = config.SPEECH_STOP_WITHOUT_WAKE_WORD
config.SPEECH_STOP_WITHOUT_WAKE_WORD = False
check("stop without wake word can be disabled", sp.command_from_text("stopp") is None)
config.SPEECH_STOP_WITHOUT_WAKE_WORD = old

# --- Wake window ------------------------------------------------------------------------
s = sp.SpeechRecognition(mock=True)
s.start()
check("mock is available", s.available and s.status()["zustand"] == sp.SIMULATED, s.status())
check("wake word only: no command", s.hear("robi") is None)
check("awake afterwards", s.awake and s.state()["geweckt"])
check("command in wake window", s.hear("folge mir") == sp.FOLLOW)
check("window consumed", not s.awake)
check("next command needs Robi again", s.hear("komm her") is None)

old = config.SPEECH_WAKE_WINDOW_S
config.SPEECH_WAKE_WINDOW_S = 0.1
s.hear("robi")
time.sleep(0.2)
check("window expires", s.hear("komm her") is None)
config.SPEECH_WAKE_WINDOW_S = old

fetched = s.fetch_commands()
check("Robi alone is reported so the robot can answer", fetched[0] == {"befehl": sp.WAKE, "text": "robi"}, fetched)
commands = [e for e in fetched if e["befehl"] != sp.WAKE]
check("command in the queue", commands == [{"befehl": sp.FOLLOW, "text": "folge mir"}], fetched)
s.hear("robin la")
check("Robi in word salad does not wake", not s.awake and s.fetch_commands() == [])
check("queue empty afterwards", s.fetch_commands() == [])
check("state shows the command", s.state()["befehl"] == sp.FOLLOW, s.state())
check("empty text is ignored", s.hear("   ") is None)
for _ in range(10):
    s.hear("stopp")
check("queue is bounded", len(s.fetch_commands()) == 5)
st = s.status()
check("status complete", st["erkannt"] == 11 and st["weckwort"] == config.SPEECH_WAKE_WORD, st)
s.stop()

# --- Exact drive commands with a number --------------------------------------------
for text, expected in [
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
    ("fahr zwei meter vor bitte", None),       # one word too many
    ("zwei meter", None),                      # no direction
    ("dreh dich um neunzig", None),            # no "Grad"
]:
    result = sp.detect_maneuver(sp.normalize(text))
    check(f"drive command {text!r} -> {expected}", result == expected, result)

check("with Robi: drive command", sp.recognize("robi fahr zwei meter zurück") == (sp.MANEUVER, True))
check("'zwei meter zurück' is not go away",
       sp.recognize("robbie zwei meter zurück")[0] == sp.MANEUVER)
check("without Robi: nothing", sp.recognize("fahr zwei meter vor") == (None, False))
check("'fahr zurück' stays go away", sp.recognize("robi fahr zurück")[0] == sp.GO_AWAY)
check("drive command is not word salad", sp.word_salad("robi fahr zwei meter vor", sp.MANEUVER) is None)
check("drive command with double Robi is salad",
       sp.word_salad("robi fahr zwei meter vor robi", sp.MANEUVER) is not None)
s = sp.SpeechRecognition(mock=True)
s.start()
check("drive command heard", s.hear("Robi, fahr zwei Meter vor") == sp.MANEUVER)
fetched = s.fetch_commands()
check("drive command with direction and value in the queue",
       fetched and fetched[0].get("wie") == {"art": "vor", "wert": 2.0}, fetched)
s.stop()
sentences = sp._maneuver_phrases()
check("grammar contains drive commands", "fahr zwei meter vor" in sentences
       and "dreh dich um neunzig grad nach rechts" in sentences, len(sentences))
check("every grammar sentence is understood",
       all(sp.detect_maneuver(sp.normalize(x)) is not None for x in sentences),
       [x for x in sentences if sp.detect_maneuver(sp.normalize(x)) is None][:3])

# --- Grammar against the model vocabulary -------------------------------------------
with tempfile.TemporaryDirectory() as folder:
    os.makedirs(os.path.join(folder, "graph"))
    # The small model does not know "robi", but it knows "robbie".
    vocabulary = {"robbie", "folgen", "folge", "mir", "stopp", "komm", "her"}
    with open(os.path.join(folder, "graph", "words.txt"), "w", encoding="utf-8") as f:
        for i, w in enumerate(sorted(vocabulary)):
            f.write(f"{w} {i}\n")
    g = sp.SpeechRecognition._grammar(folder)
    check("unknown wake word is missing", "robi" not in g and "robbie" in g, g[:5])
    check("sentences with wake word", "robbie folgen" in g and "folge mir robbie" in g)
    check("unknown phrases are missing", "geh weg" not in g and "komm her" in g)
    check("all words known", all(w in vocabulary or w == "[unk]" for s in g for w in s.split()))
    check("rest as [unk]", g[-1] == "[unk]")

# --- Hanging stream: error instead of waiting forever ---------------------------------
# After a USB dropout nothing came from the microphone any more, without an error message.
class _DeadSd:
    class RawInputStream:
        closed = False

        def __init__(self, **kw):
            self.callback = kw["callback"]  # never called

        def start(self):
            pass

        def close(self):
            type(self).closed = True


class _Vosk:
    class KaldiRecognizer:
        def __init__(self, *a):
            pass

        def SetWords(self, _):
            pass


s = sp.SpeechRecognition(mock=False)
s._find_microphone = lambda sd: (0, "USB Microphone")
s._sample_rate = lambda sd, g: 16000
t0 = time.monotonic()
try:
    s._listen_until_error(_DeadSd, _Vosk, None, "[]")
    message = ""
except RuntimeError as exc:
    message = str(exc)
duration = time.monotonic() - t0
check("hanging stream is detected", "hängt" in message, message)
check("after the deadline, not forever", duration < config.SPEECH_HANG_S + 2, duration)
check("dead stream is closed", _DeadSd.RawInputStream.closed)


# --- Listener thread stuck: watchdog restarts it, then error ---------------------------
class _HangingSd:
    _terminate = _initialize = staticmethod(lambda: None)

    class RawInputStream:
        def __init__(self, **kw):
            threading.Event().wait()  # opening hangs in PortAudio, forever


import threading  # noqa: E402

old = config.SPEECH_THREAD_HANG_S, config.SPEECH_ENABLED
config.SPEECH_THREAD_HANG_S, config.SPEECH_ENABLED = 0.5, True
s = sp.SpeechRecognition(mock=False)
s._model = (_HangingSd, _Vosk, None, "[]")  # model "already loaded"
s._find_microphone = lambda sd: (0, "USB Microphone")
s._sample_rate = lambda sd, g: 16000
s.start()
time.sleep(config.SPEECH_THREAD_MAX_RESTARTS + 3)
st = s.status()
check("watchdog restarts hanging listener thread",
       s._generation == 1 + config.SPEECH_THREAD_MAX_RESTARTS, s._generation)
check("then error with hint", st["zustand"] == sp.ERROR and "neu starten" in (st["fehler"] or ""), st)
s.stop()
config.SPEECH_THREAD_HANG_S, config.SPEECH_ENABLED = old

# --- Without Vosk: off, but no crash ---------------------------------------------------
sys.modules["vosk"] = None  # import is then sure to fail
s = sp.SpeechRecognition(mock=False)
s.start()
for _ in range(50):
    if s.status()["zustand"] != sp.LOADING:
        break
    time.sleep(0.05)
st = s.status()
check("without Vosk: error instead of crash", st["zustand"] == sp.ERROR and not s.available, st)
check("error names the fix", "pip install" in (st["fehler"] or ""), st["fehler"])
s.stop()

print("\nall passed" if not error else f"\n{error} failed")
sys.exit(1 if error else 0)
