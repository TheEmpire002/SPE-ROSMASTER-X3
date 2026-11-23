"""Checks the voice output -- without a robot and without a speaker.

    python Marlon/interaction/test_voice.py

Tested:

* queue: repetitions are swallowed, the length is limited,
  ``priority`` and ``mute`` clear it;
* ``speaking`` is true during a sentence and false again afterwards;
* if Piper and espeak-ng are missing, only text remains -- without a crash;
* the behavior says the matching sentences on gestures and mode changes.

On the robot itself the easiest way to hear the speaker is:

    curl -s localhost:8000/stimme/status
    (WebSocket) {"type": "say", "text": "Hallo, ich bin der Rosmaster."}
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaction import gestures as gs  # noqa: E402
from rosmaster_assist.interaction import voice as st  # noqa: E402
from rosmaster_assist.interaction import behavior as vh  # noqa: E402

error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "ERROR "), name, result)
    error += not condition


# --- Queue (without a running thread) ----------------------------------------------
s = st.Voice(mock=True)
check("Sentence accepted", s.say("Ich folge dir."))
check("Repetition swallowed", not s.say("Ich folge dir."))
check("Empty text discarded", not s.say("   "))
for i in range(config.VOICE_MAX_QUEUE + 2):
    s.say(f"Satz {i}")
queued = s.status()["warteschlange"]
check("Queue limited", len(queued) == config.VOICE_MAX_QUEUE, queued)
check("Oldest is dropped", queued[-1] == f"Satz {config.VOICE_MAX_QUEUE + 1}", queued)
s.say("Halt.", priority=True)
check("Priority clears the queue", s.status()["warteschlange"] == ["Halt."], s.status()["warteschlange"])
s.mute()
check("Mute clears the queue", s.status()["warteschlange"] == [])
long_text = s.say("x" * 1000) and s.status()["warteschlange"][0]
check("Long texts shortened", len(long_text) == config.VOICE_MAX_CHARS, len(long_text))

# --- Speech thread in mock -----------------------------------------------------------
s = st.Voice(mock=True)
s.start()
check("Mock is available", s.available and s.status()["ausgabe"] == st.SIMULATED, s.status())
s.say("Hallo")
time.sleep(0.2)
check("speaking during the sentence", s.speaking)
time.sleep(0.6 + config.VOICE_TAIL_S)
check("silent afterwards", not s.speaking and s.status()["gesagt"] == 1, s.status())

# Announcement from the app: keeps "speaking" (speech recognition stops listening), ends with end()
d = s.announcement(16000)
check("Announcement readable in the status queue", all(isinstance(t, str) for t in s.status()["warteschlange"]))
for _ in range(4):
    d.feed(b"\x00\x01" * 4000)
    time.sleep(0.1)
check("Announcement: speaking", s.speaking and not d.finished)
d.end()
time.sleep(0.3 + config.VOICE_TAIL_S)
check("Announcement: over and silent after end()", d.finished and not s.speaking, (d.finished, s.speaking))

# Stop (priority) aborts a running announcement
d = s.announcement(16000)
d.feed(b"\x00\x01" * 4000)
time.sleep(0.2)
s.say("Halt.", priority=True)
time.sleep(0.3)
check("Announcement: priority aborts", d.finished, d.finished)

# If nothing more arrives (app gone), it ends by itself after ANNOUNCE_PAUSE_S
old_pause = config.ANNOUNCE_PAUSE_S
config.ANNOUNCE_PAUSE_S = 0.3
time.sleep(1.5)  # let "Halt." finish
d = s.announcement(16000)
d.feed(b"\x00\x01" * 4000)
time.sleep(1.0)
check("Announcement: ends by itself after the pause", d.finished, d.finished)
config.ANNOUNCE_PAUSE_S = old_pause
s.stop()

# --- Without speech programs: text only ---------------------------------------------
old = (config.VOICE_PIPER, config.VOICE_ESPEAK)
config.VOICE_PIPER = ("gibt-es-nicht-xyz",)
config.VOICE_ESPEAK = "gibt-es-nicht-xyz"
s = st.Voice(mock=False)
s.start()
s.say("Niemand hört mich.")
time.sleep(0.2)
check("without synthesis text only", s.status()["ausgabe"] == st.TEXT and not s.available, s.status())
check("Thread keeps running", s._thread is not None and s._thread.is_alive())
s.stop()
config.VOICE_PIPER, config.VOICE_ESPEAK = old


# --- Behavior speaks -------------------------------------------------------------------
class Transcript:
    def __init__(self):
        self.sentences = []

    def say(self, text, priority=False):
        self.sentences.append((text, priority))
        return True


class Watchdog:
    def submit(self, cmd, source="hand"):
        pass

    def stop(self):
        pass


transcript = Transcript()
ia = vh.Interaction(Watchdog(), None, None, transcript)
ia._switch(vh.FOLLOW, "folgt", 0.0, say="Ich folge dir.")
check("Mode change is spoken", transcript.sentences[-1] == ("Ich folge dir.", False), transcript.sentences)


class Gestures:
    """Returns exactly one gesture, like the real recognition after a movement."""

    last = last_time = None

    def __init__(self, gesture):
        self.gesture = gesture

    def update(self, target, timestamp, still=True):
        return self.gesture


for gesture, sentence, priority in [
    (gs.STOP, "Okay, ich halte an.", True),
    (gs.COME_HERE, "Ich komme.", False),
]:
    ia._gestures = Gestures(gesture)
    ia._last_frame = -1.0
    ia._check_gesture({"zeit": time.monotonic()}, time.monotonic())
    check(f"Gesture {gesture} is answered", transcript.sentences[-1] == (sentence, priority), transcript.sentences[-1])

ia = vh.Interaction(Watchdog(), None, None)  # without a voice nothing must happen
ia._switch(vh.FOLLOW, "folgt", 0.0, say="Ich folge dir.")
check("without a voice no error", True)

print("\nall passed" if not error else f"\n{error} errors")
sys.exit(1 if error else 0)
