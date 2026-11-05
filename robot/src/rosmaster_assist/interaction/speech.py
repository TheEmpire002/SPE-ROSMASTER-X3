"""Speech commands via the USB microphone -- wake word "Robi".

"Robi, folge mir", "Robi komm her", "Stopp": the microphone runs in its
own thread, Vosk recognizes offline on the CPU (small German model, see
docs/hardware.md). Only a fixed vocabulary (grammar) is recognized: that is
fast on the Pi and confuses words far less often than free recognition,
which turns every noise into some word.

Flow:

* A command only counts together with the wake word -- before or after it
  ("Robi folgen", "folgen Robi") or up to ``SPEECH_WAKE_WINDOW_S``
  after a lone "Robi". Otherwise the robot would drive off as soon as
  someone in the room says "komm mit".
* Exception "La-da-da" (``SPEECH_LADADA_WITHOUT_WAKE_WORD``): plays the song
  from ``SPEECH_LADADA_URL`` -- a false alarm is just music.
* Exception stop (``SPEECH_STOP_WITHOUT_WAKE_WORD``): stopping must never
  fail because the name was not heard.
* While the robot's own voice is speaking, it does not listen, otherwise
  the robot understands itself ("Ich folge dir" -> "folge").
* The grammar has a downside: Vosk squeezes every noise into an allowed
  sentence. Music and conversations in the room thus became "robbie hallo"
  and switched the interaction on. That is why uncertain input
  (``SPEECH_MIN_CONFIDENCE``, ``SPEECH_MAX_UNK_SHARE``) and word salad
  from several sentences (``SPEECH_MAX_EXTRA_WORDS``) only let stop through.
  While music is playing, every command except stop needs the name and
  ``SPEECH_MIN_CONFIDENCE_MUSIC``.
* The server answers a lone "Robi" with ``SPEECH_WAKE_REPLY``
  ("Ja?"), so you can hear that it is listening.

The module does not drive by itself. It puts recognized commands into a
queue; the server picks them up and treats them like buttons of the app
(via the watchdog, with all safety rules).

If Vosk, sounddevice, the model or the microphone is missing, recognition
is simply off -- the server keeps running. If the microphone disappears
after a USB dropout, the thread reconnects by itself.

``hear(text)`` takes text as if it had been heard: for ``--mock``,
the tests and the ``speech`` command from the app.
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

# Commands (values in state.sprache.befehl and in the log)
FOLLOW = "folgen"
STOP = "stopp"
COME_HERE = "herkommen"
GO_AWAY = "weggehen"
START = "start"
OFF = "aus"
GO_HOME = "nach_hause"
EXPLORE = "erkunden"
MUSIC_OFF = "musik_aus"
LADADA = "ladada"
BEATBOX = "beatbox"
#: Precise drive command with a number: "fahr zwei Meter vor", "dreh dich um
#: neunzig Grad nach rechts". Not in COMMANDS because it carries a number; see
#: detect_maneuver(). Executed by navigation/maneuver.py.
MANEUVER = "manoever"
#: Not a command, just a lone "Robi": the server replies ("Ja?") so you can
#: hear that it is listening. Only ever in the queue, never in state.sprache.
WAKE = "wecken"

#: Command -> phrases. The order matters: if several occur in one sentence,
#: the one higher up wins -- so stop comes before everything that drives.
#: Only "Musik stopp" comes before it, otherwise it would stop instead of the music.
COMMANDS: dict[str, tuple[str, ...]] = {
    MUSIC_OFF: ("musik aus", "musik stopp", "ruhe", "sei leise"),
    STOP: ("stopp", "stop", "halt", "anhalten", "halt an", "bleib stehen", "steh"),
    FOLLOW: ("folgen", "folge", "folge mir", "folg mir", "komm mit"),
    COME_HERE: ("komm her", "komm zu mir", "hierher", "komm"),
    GO_AWAY: ("geh weg", "weg", "zurück", "fahr zurück", "abstand"),
    GO_HOME: ("nach hause", "heim", "fahr nach hause"),
    EXPLORE: ("erkunde", "erkunden", "schau dich um"),
    OFF: ("schlafen", "schlaf", "gute nacht", "feierabend"),
    START: ("hallo", "wach auf", "aufwachen", "guten morgen"),
    # Vosk does not know "ladada" as a word but hears "la da da"; "ladada"
    # itself stays for typed input (speech command, --mock).
    LADADA: ("la da da", "ladada", "la la la", "lalala"),
    # "Beatbox" is probably not in the Vosk model; the vocabulary check
    # catches the variants -- whatever the model does not know is dropped.
    BEATBOX: ("beatbox", "beat box", "bit box", "biet box", "beet box"),
}

# Recognition states (status()["zustand"])
SWITCHED_OFF = "aus"
LOADING = "laedt"
LISTENING = "hoert"
ERROR = "fehler"
SIMULATED = "simuliert"
PAUSED = "pausiert"  # microphone deliberately closed, e.g. while mapping (set_pause())


def normalize(text: str) -> list[str]:
    """Lower case, without punctuation, split into words.

    Digits stay (typed: "fahr 1,5 m vor"); Vosk delivers number words.
    """
    text = str(text).lower().replace("ß", "ss").replace("zurueck", "zurück")
    return re.findall(r"\d+(?:[.,]\d+)?|[a-zäöü]+", text)


# --- Precise drive commands with a number ------------------------------------------
# Values as in navigation/maneuver.py (FORWARD, BACK, ... TURN).

_NUMBER_WORDS = {
    "ein": 1, "eins": 1, "einen": 1, "einem": 1, "zwei": 2, "drei": 3, "vier": 4,
    "fünf": 5, "sechs": 6, "sieben": 7, "acht": 8, "neun": 9, "zehn": 10, "elf": 11,
    "zwölf": 12, "fünfzehn": 15, "zwanzig": 20, "dreissig": 30, "vierzig": 40,
    "fünfundvierzig": 45, "fünfzig": 50, "sechzig": 60, "siebzig": 70, "achtzig": 80,
    "neunzig": 90, "hundert": 100, "einhundert": 100, "hundertzwanzig": 120,
    "hundertfünfunddreissig": 135, "hundertachtzig": 180, "zweihundert": 200,
    "zweihundertsiebzig": 270, "dreihundert": 300, "dreihundertsechzig": 360,
}
_FRACTIONS = {"halben": 0.5, "halbe": 0.5, "anderthalb": 1.5, "eineinhalb": 1.5}

#: Spoken driving direction -> direction in navigation/maneuver.py.
_DRIVE_DIRECTIONS = {
    ("vor",): "vor", ("vorwärts",): "vor", ("nach", "vorne"): "vor", ("nach", "vorn"): "vor",
    ("geradeaus",): "vor",
    ("zurück",): "zurueck", ("rückwärts",): "zurueck", ("nach", "hinten"): "zurueck",
    ("nach", "links"): "links", ("links",): "links",
    ("nach", "rechts"): "rechts", ("rechts",): "rechts",
}


def _number(word: str) -> float | None:
    if word[0].isdigit():
        return float(word.replace(",", "."))
    value = _NUMBER_WORDS.get(word)
    return None if value is None else float(value)


def _merge_numbers(words: list[str]) -> list:
    """Number words to numbers: "hundert achtzig" -> 180.0, "einen halben" -> 0.5.

    Vosk writes compound numbers sometimes as one word, sometimes separated.
    """
    out: list = []
    i = 0
    while i < len(words):
        w = words[i]
        if w in _FRACTIONS:
            if _FRACTIONS[w] == 0.5 and out and out[-1] == 1.0:
                out[-1] = 0.5  # "einen halben"
            else:
                out.append(_FRACTIONS[w])
            i += 1
            continue
        value = _number(w)
        if value is None:
            out.append(w)
            i += 1
            continue
        i += 1
        while i < len(words):
            resumed = _number(words[i])
            if resumed is None:
                break
            if resumed == 100 and value < 10:
                value *= 100  # "drei hundert"
            elif resumed < value:
                value += resumed  # "hundert achtzig"
            else:
                break
            i += 1
        out.append(value)
    return out


def detect_maneuver(words: list[str]) -> tuple[str, float] | None:
    """(direction, value) for a precise drive command, otherwise None.

    ``words`` without the wake word. The whole sentence must match -- one word
    too many and it is none (this also checks whether it is word salad):

    * "[fahr|fahre] <number> [meter|zentimeter] <vor|zurück|nach links|...>"
    * "dreh dich um <number> grad [nach links|nach rechts]" (no side: left)
    * "dreh dich um" (180°), "dreh dich nach links|rechts" (90°)
    """
    t = _merge_numbers(words)
    if t[:1] in (["fahr"], ["fahre"]):
        t = t[1:]
    if len(t) >= 2 and isinstance(t[0], float):
        number, rest, unit = t[0], t[1:], 1.0
        if rest[:1] in (["meter"], ["metern"], ["m"]):
            rest = rest[1:]
        elif rest[:1] in (["zentimeter"], ["cm"]):
            rest, unit = rest[1:], 0.01
        direction = _DRIVE_DIRECTIONS.get(tuple(rest)) if all(isinstance(w, str) for w in rest) else None
        if direction is not None and number > 0:
            return direction, round(number * unit, 3)
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
            side = tuple(rest[2:])
            if side in ((), ("nach", "links"), ("links",)):
                return "drehen", rest[0]
            if side in (("nach", "rechts"), ("rechts",)):
                return "drehen", -rest[0]
    return None


def _maneuver_phrases() -> list[str]:
    """The precise drive commands for the Vosk grammar.

    Vosk only recognizes sentences it knows -- so spell out the common
    numbers. Typed (app, --mock), any number works.
    """
    segments = ["einen meter", "zwei meter", "drei meter", "vier meter", "fünf meter",
                "einen halben meter", "anderthalb meter", "zwanzig zentimeter",
                "fünfzig zentimeter"]
    directions = ["vor", "vorwärts", "nach vorne", "zurück", "rückwärts", "nach hinten",
                  "nach links", "nach rechts"]
    sentences = [f"{verb}{s} {r}" for verb in ("fahr ", "fahre ", "") for s in segments for r in directions]
    # Spelled as in the model (ß); matching goes through normalize() with ss.
    angle = ["dreißig", "fünfundvierzig", "sechzig", "neunzig", "hundertachtzig",
              "hundert achtzig", "dreihundertsechzig", "dreihundert sechzig"]
    for turn in ("dreh dich", "drehe dich"):
        sentences += [f"{turn} um", f"{turn} nach links", f"{turn} nach rechts"]
        sentences += [f"{turn} um {w} grad{s}" for w in angle for s in ("", " nach links", " nach rechts")]
    return sentences


def _wake_words() -> set[str]:
    return {w.lower() for w in config.SPEECH_WAKE_WORD_VARIANTS} | {config.SPEECH_WAKE_WORD.lower()}


def _contains(words: list[str], phrase: list[str]) -> bool:
    n = len(phrase)
    return any(words[i:i + n] == phrase for i in range(len(words) - n + 1))


def recognize(text: str, awake: bool = False) -> tuple[str | None, bool]:
    """Text -> (command or None, did the wake word occur?).

    ``awake``: the wake word was said alone shortly before -- then the
    command without the name is enough.
    """
    words = normalize(text)
    wake = _wake_words()
    wake_word = any(w in wake for w in words)
    rest = [w for w in words if w not in wake]
    # Before the fixed commands: "fahr zwei meter zurück" is not "go away".
    # A precise drive command only matches as a whole sentence; stop is never in it.
    if rest and detect_maneuver(rest) is not None:
        return (MANEUVER if wake_word or awake else None), wake_word
    for command, phrases in COMMANDS.items():
        if not any(_contains(rest, w.replace("ß", "ss").split()) for w in phrases):
            continue
        if wake_word or awake or _no_wake_word(command):
            return command, wake_word
        return None, wake_word
    return None, wake_word


def word_salad(text: str, command: str) -> str | None:
    """Why ``text`` is not a clean command, or None.

    A spoken command comes from Vosk as exactly one grammar sentence
    ("robbie komm her"). If Vosk strings several sentences together from
    noise, you get salad like "robin hallo hause zu weg ruby": the wake
    word twice, or words that belong to no phrase of the command.
    """
    words = normalize(text)
    wake = _wake_words()
    rest = [w for w in words if w not in wake]
    if len(words) - len(rest) > 1:
        return "Weckwort mehrfach"
    if command == MANEUVER:
        # Only matches as a whole sentence, so already checked.
        return None if detect_maneuver(rest) is not None else "kein ganzer Fahrbefehl"
    phrases = [w.replace("ß", "ss").split() for w in COMMANDS.get(command, ())]
    longest = max((len(w) for w in phrases if _contains(rest, w)), default=0)
    extra = len(rest) - longest
    if extra > config.SPEECH_MAX_EXTRA_WORDS:
        return f"{extra} Wörter zu viel"
    return None


def _no_wake_word(command: str) -> bool:
    """Commands that also count without "Robi" (see config)."""
    if command == STOP:
        return config.SPEECH_STOP_WITHOUT_WAKE_WORD
    if command == LADADA:
        return config.SPEECH_LADADA_WITHOUT_WAKE_WORD
    return False


class _MicrophoneChange(Exception):
    """A better microphone is (back) there -- close the stream and search again."""


def _microphone_names() -> list[str]:
    """Name fragments of the microphones in order of preference."""
    preferred = config.SPEECH_MICROPHONE
    if isinstance(preferred, int):
        return []
    return [preferred] if preferred else list(config.SPEECH_MICROPHONE_NAMES)


def _rank(name: str) -> int | None:
    """Rank of the microphone in the preference (0 = best), None = unknown."""
    for i, part in enumerate(_microphone_names()):
        if part.lower() in name.lower():
            return i
    return None


def _registered_sound_cards() -> str:
    """Sound cards according to the kernel. Cheap and without PortAudio, which
    remembers its device list at startup and only sees new devices after a restart."""
    try:
        with open("/proc/asound/cards", encoding="utf-8", errors="replace") as f:
            return f.read().lower()
    except OSError:
        return ""


def _no_wait(task, what: str) -> None:
    """Cleanup that can get stuck on a dead USB device.

    PortAudio waits for ALSA when closing; if the device vanished mid-run,
    that call may never return. Then the rest is left behind (daemon
    thread) instead of the speech recognition hanging along with it.
    """
    t = threading.Thread(target=task, name="sprache-aufraeumen", daemon=True)
    t.start()
    t.join(config.SPEECH_CLOSE_S)
    if t.is_alive():
        log.warning("Speech recognition: %s is stuck -- leaving it behind", what)


def _level(raw: bytes) -> float:
    """Loudness of a block (16 bit) as 0..1: -60 dBFS and quieter = 0, full scale = 1."""
    import numpy as np

    values = np.frombuffer(raw, dtype="<i2").astype(np.float32)
    if values.size == 0:
        return 0.0
    rms = float(np.sqrt(np.mean(values * values))) / 32768.0
    db = 20.0 * np.log10(max(rms, 1e-6))
    return max(0.0, min(1.0, (db + 60.0) / 60.0))


def command_from_text(text: str) -> str | None:
    """Only the command, without a wake word said earlier."""
    return recognize(text)[0]


class SpeechRecognition:
    def __init__(self, mock: bool = False, voice=None, mukke=None) -> None:
        self._mock = mock
        # Only ``speaking`` is needed: while it is true, the ear stays silent.
        self._voice = voice
        # Only ``running`` is needed: the microphone also hears the music.
        self._mukke = mukke
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._commands: collections.deque[dict] = collections.deque(maxlen=5)

        self._state = SWITCHED_OFF
        self._microphone: str | None = None
        self._awake_until = 0.0  # monotonic; > now means "wake word heard"
        self._heard: str | None = None
        self._command: str | None = None
        self._command_time = 0.0
        self._recognized = 0
        self._confidence: float | None = None
        self._discarded = 0
        self._last_discarded: str | None = None
        self._error: str | None = None

        # Live feed for the app (see live() and sound_subscribe()). Level and
        # partial text are only computed while someone is following along.
        self._live_until = 0.0  # monotonic
        self._level = 0.0  # 0..1
        self._partial = ""  # what Vosk understands of the current sentence so far
        self._rate: int | None = None
        self._events: collections.deque[dict] = collections.deque(maxlen=config.SPEECH_LIVE_EVENTS)
        self._audio_subs: list[queue.Queue] = []

        # Watchdog over the listening thread (see _monitor_loop()): if it gets
        # stuck in PortAudio, a new one takes over. The old one notices via
        # _generation that it has been superseded, should it ever wake up.
        self._generation = 0
        self._alive = 0.0  # monotonic, 0 = model still loading
        self._model = None  # (model, grammar), so a successor does not reload it
        self._monitor: threading.Thread | None = None

        # Polled continuously by the listening thread; returns a reason while
        # the microphone should stay closed (see set_pause()).
        self._pause_reason = lambda: None
        self._pause: str | None = None  # last reported reason, only for the log

    # --- Control ---------------------------------------------------------------

    @property
    def available(self) -> bool:
        return self._state in (LISTENING, SIMULATED) and self._pause_reason() is None

    def set_pause(self, reason) -> None:
        """``reason()`` returns a text while the microphone should be closed.

        Polled instead of registered and unregistered: that way the pause ends
        by itself, no matter how the cause ended (page, app, crash, shutdown).
        """
        self._pause_reason = reason

    @property
    def awake(self) -> bool:
        return time.monotonic() < self._awake_until

    def start(self) -> None:
        if not config.SPEECH_ENABLED:
            log.info("Speech recognition disabled (config.SPEECH_ENABLED)")
            return
        if self._mock:
            # No microphone in the simulation; commands come via hear().
            self._state = SIMULATED
            return
        self._stop.clear()
        self._state = LOADING
        self._alive = 0.0
        self._start_listen_thread()
        self._monitor = threading.Thread(target=self._monitor_loop, name="sprache-wache", daemon=True)
        self._monitor.start()

    def _start_listen_thread(self) -> None:
        self._generation += 1
        self._thread = threading.Thread(
            target=self._loop, args=(self._generation,), name="sprache", daemon=True
        )
        self._thread.start()

    def _monitor_loop(self) -> None:
        """Restarts the listening thread when it is stuck.

        The timeouts in _listen_until_error() only help while the thread
        itself is still running. After a USB dropout, however, even opening
        the microphone can hang in PortAudio; without this watchdog no
        command would ever arrive again until someone restarts the server.
        A thread cannot be killed in Python -- it is left behind.
        """
        new = 0
        while not self._stop.wait(1.0):
            if not self._alive:
                continue  # model still loading, that may take a while
            if self._state == LISTENING:
                new = 0
            if time.monotonic() - self._alive < config.SPEECH_THREAD_HANG_S:
                continue
            if new >= config.SPEECH_THREAD_MAX_RESTARTS:
                given_up = "Mikrofon hängt in PortAudio fest -- Server neu starten"
                if self._error != given_up:  # log loudly only once
                    self._error = given_up
                    log.error("Speech recognition: %s", given_up)
                self._state = ERROR
                continue
            new += 1
            log.warning("Speech recognition: listening thread stuck for %.0f s -- starting a new one (%d/%d)",
                        time.monotonic() - self._alive, new, config.SPEECH_THREAD_MAX_RESTARTS)
            self._state = ERROR
            self._alive = time.monotonic()
            self._start_listen_thread()

    def stop(self) -> None:
        self._stop.set()
        if self._monitor is not None:
            self._monitor.join(timeout=2.0)
            self._monitor = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._state != ERROR:
            self._state = SWITCHED_OFF

    def hear(
        self,
        text: str,
        allowed: tuple[str, ...] | None = None,
        reason: str | None = None,
        details: dict | None = None,
    ) -> str | None:
        """Evaluate a heard (or typed) sentence. Returns the command.

        ``allowed``: only these commands count (uncertain input, music);
        ``reason`` then goes into the log. A wake word heard this way does not
        wake, otherwise a single "komm" from the song would be enough afterwards.
        ``details`` (confidence, words from Vosk) only go into the event log.
        """
        text = " ".join(str(text).split())
        if not text:
            return None
        was_awake = self.awake
        command, wake_word = recognize(text, was_awake)
        rejected = None
        if allowed is not None:
            wake_word = False
            if command is not None and command not in allowed:
                rejected, command = command, None
        # Only a "Robi" without further words wakes: if the name is buried in
        # word salad ("robin la weg"), a noise more likely said it.
        wake_word = wake_word and all(w in _wake_words() for w in normalize(text))
        now = time.monotonic()
        with self._lock:
            self._heard = text
            if rejected is not None:
                self._discarded += 1
                self._last_discarded = f"{rejected}: {text!r} ({reason})"
            if command is not None:
                self._awake_until = 0.0  # used up; the next command needs "Robi" again
                self._command, self._command_time = command, now
                self._recognized += 1
                entry = {"befehl": command, "text": text}
                if command == MANEUVER:
                    category, value = detect_maneuver(
                        [w for w in normalize(text) if w not in _wake_words()]
                    )
                    entry["wie"] = {"art": category, "wert": value}
                self._commands.append(entry)
            elif wake_word:
                self._awake_until = now + config.SPEECH_WAKE_WINDOW_S
                if not was_awake:
                    self._commands.append({"befehl": WAKE, "text": text})
            if command is not None:
                event = {"ergebnis": "befehl", "befehl": command}
            elif rejected is not None:
                event = {"ergebnis": "verworfen", "befehl": rejected, "grund": reason}
            elif wake_word:
                event = {"ergebnis": "weckwort"}
            else:
                event = {"ergebnis": "ohne"}
            self._events.append({"zeit": time.time(), "text": text, **event, **(details or {})})
            self._partial = ""
        if command is not None:
            log.info("Speech command %s (heard: %r)", command, text)
        elif rejected is not None:
            log.info("Speech command %s rejected, %s (heard: %r)", rejected, reason, text)
        else:
            log.debug("Heard without command: %r", text)
        return command

    def heard_since(self, t: float) -> list[str]:
        """Texts heard since wall-clock time ``t``, e.g. answers in the health check."""
        with self._lock:
            return [e["text"] for e in self._events if e["zeit"] >= t]

    def live(self) -> dict:
        """For following along in the app (GET /sprache/live), about 5x per second.

        Each request keeps the live evaluation running for SPEECH_LIVE_HOLD_S;
        once nobody asks any more, the listening thread stops computing level
        and partial text.
        """
        self._live_until = time.monotonic() + config.SPEECH_LIVE_HOLD_S
        with self._lock:
            events = list(self._events)
        return {
            "verfuegbar": self.available,
            "zustand": self._state,
            "mikrofon": self._microphone,
            "rate": self._rate,
            "pegel": round(self._level, 3),
            "teil": self._partial,
            # While the robot is speaking, the ear evaluates nothing.
            "taub": bool(self._voice is not None and self._voice.speaking),
            "geweckt": self.awake,
            "ereignisse": events,
        }

    @property
    def sound_rate(self) -> int | None:
        """Sample rate of the microphone (Hz), None while none is open."""
        return self._rate if self._state == LISTENING else None

    def sound_subscribe(self) -> queue.Queue:
        """Raw audio from the microphone (16 bit, mono, ``live()["rate"]`` Hz) into a queue.

        For listening in from the app (WebSocket /sprache/ton). If the
        receiver cannot keep up, blocks are dropped instead of piling up.
        """
        subscription: queue.Queue = queue.Queue(maxsize=config.SPEECH_SOUND_BUFFER)
        with self._lock:
            self._audio_subs.append(subscription)
        return subscription

    def sound_unsubscribe(self, subscription: queue.Queue) -> None:
        with self._lock:
            if subscription in self._audio_subs:
                self._audio_subs.remove(subscription)

    def fetch_commands(self) -> list[dict]:
        with self._lock:
            commands = list(self._commands)
            self._commands.clear()
        return commands

    def state(self) -> dict:
        """For state.sprache in the app."""
        return {
            "verfuegbar": self.available,
            "geweckt": self.awake,
            "gehoert": self._heard,
            # Show only briefly; a command from a minute ago is confusing.
            "befehl": self._command if time.monotonic() - self._command_time < 5.0 else None,
            # Why the microphone is deliberately off right now (otherwise None).
            "pause": self._pause_reason(),
        }

    def status(self) -> dict:
        with self._lock:
            return {
                "aktiviert": config.SPEECH_ENABLED,
                "verfuegbar": self.available,
                "simuliert": self._mock,
                "zustand": self._state,
                "mikrofon": self._microphone,
                "modell": os.path.expanduser(config.SPEECH_MODEL),
                "weckwort": config.SPEECH_WAKE_WORD,
                "geweckt": self.awake,
                "zuletzt_gehoert": self._heard,
                "letzter_befehl": self._command,
                "erkannt": self._recognized,
                # Lowest word confidence of the last sentence from the microphone
                "konfidenz": self._confidence,
                "verworfen": self._discarded,
                "zuletzt_verworfen": self._last_discarded,
                "befehle": {b: list(w) for b, w in COMMANDS.items()},
                "fehler": self._error,
                "pause": self._pause_reason(),
            }

    # --- Listening thread -------------------------------------------------------------

    def _loop(self, generation: int) -> None:
        if self._model is None:
            self._model = self._load_model()
            if self._model is None:
                return
        sd, vosk, model, grammar = self._model
        superseded = lambda: generation != self._generation  # noqa: E731

        while not self._stop.is_set() and not superseded():
            self._alive = time.monotonic()
            if self._check_pause():
                self._stop.wait(config.SPEECH_PAUSE_CHECK_S)
                continue
            wait_s = config.SPEECH_RETRY_S
            try:
                self._listen_until_error(sd, vosk, model, grammar, superseded)
            except _MicrophoneChange as exc:
                log.info("Speech recognition: %s -- switching", exc)
                wait_s = 0.5  # not an error, just a device change
            except Exception as exc:  # microphone gone, ALSA error, ...
                if superseded():
                    return  # a successor is already listening
                message = f"{type(exc).__name__}: {exc}"
                if message != self._error:
                    # Log loudly only once, not every few seconds.
                    log.warning("Speech recognition: %s -- retrying", message)
                self._error = message
                self._state = ERROR
            self._alive = time.monotonic()
            if self._check_pause():
                continue  # closed deliberately, not an error: do not restart
            self._stop.wait(wait_s)
            # PortAudio remembers the device list from startup; it only sees
            # a newly registered microphone after a restart.
            def restart() -> None:
                try:
                    sd._terminate()
                    sd._initialize()
                except Exception:
                    pass

            _no_wait(restart, "restarting PortAudio")

    def _load_model(self):
        """Load Vosk and the model; None (and state off) if something is missing."""
        try:
            import sounddevice as sd
            import vosk
        except ImportError as exc:
            self._disable(f"{exc.name} fehlt (pip install vosk sounddevice)")
            return
        except OSError as exc:
            # sounddevice is installed but the C library is missing:
            # "PortAudio library not found" comes as OSError, not as
            # ImportError -- without this branch the listening thread would die silently.
            self._disable(f"{exc} (sudo apt install libportaudio2)")
            return
        model_path = os.path.expanduser(config.SPEECH_MODEL)
        if not os.path.isdir(model_path):
            self._disable(f"Vosk-Modell fehlt: {model_path} (siehe docs/hardware.md)")
            return
        try:
            vosk.SetLogLevel(-1)
            model = vosk.Model(model_path)
        except Exception as exc:  # broken or incomplete model
            self._disable(f"Vosk-Modell lässt sich nicht laden: {exc}")
            return
        grammar = json.dumps(self._grammar(model_path, model), ensure_ascii=False)
        return sd, vosk, model, grammar

    def _listen_until_error(self, sd, vosk, model, grammar: str, superseded=lambda: False) -> None:
        device, name = self._find_microphone(sd)
        rate = self._sample_rate(sd, device)
        # Vosk resamples to 16 kHz internally; the microphone may stay at its
        # own rate if it cannot do 16 kHz.
        recognizer = vosk.KaldiRecognizer(model, rate, grammar)
        # Individual words with confidence, see _interpret().
        recognizer.SetWords(True)
        block = int(rate * 0.25)
        # Callback instead of stream.read(): after a USB dropout a blocking
        # read() sometimes never returned, without an error -- no retry
        # kicked in. The queue can be read with a timeout.
        blocks: queue.Queue[bytes] = queue.Queue(maxsize=config.SPEECH_BLOCK_BUFFER)

        def feed(data, _frames, _time, _status) -> None:
            try:
                blocks.put_nowait(bytes(data))
            except queue.Full:
                pass  # evaluation briefly lagging: better a gap than a backlog

        stream = sd.RawInputStream(
            samplerate=rate, blocksize=block, device=device, dtype="int16", channels=1,
            callback=feed,
        )
        try:
            stream.start()
            self._microphone = name
            self._rate = rate
            self._state = LISTENING
            if self._error:
                log.info("Speech recognition: microphone back (%s)", name)
            else:
                log.info("Speech recognition listening on %s (%d Hz), wake word %r",
                         name, rate, config.SPEECH_WAKE_WORD)
            self._error = None
            deaf = False
            # Failsafe after USB dropouts: dead stream (only silence) and
            # fallback microphone although the better one is back.
            rank = _rank(name)
            opened = time.monotonic()
            cards_before = _registered_sound_cards()
            next_check = opened + config.SPEECH_MICROPHONE_CHECK_S
            silent_since: float | None = None
            while not self._stop.is_set() and not superseded():
                try:
                    raw = blocks.get(timeout=config.SPEECH_HANG_S)
                except queue.Empty:
                    raise RuntimeError(
                        f"{name} liefert seit {config.SPEECH_HANG_S:.0f} s keinen Ton (Stream hängt)"
                    ) from None
                now = self._alive = time.monotonic()
                if self._check_pause():
                    return  # closes the stream (finally)
                # A real microphone always has a little noise; nothing but exact
                # zeros means: the stream is still alive, the device is not.
                if raw.strip(b"\x00"):
                    silent_since = None
                elif silent_since is None:
                    silent_since = now
                elif now - silent_since > config.SPEECH_SILENT_S:
                    raise RuntimeError(f"{name} liefert seit {config.SPEECH_SILENT_S:.0f} s nur Stille")
                if rank and now >= next_check:
                    next_check = now + config.SPEECH_MICROPHONE_CHECK_S
                    self._check_better(rank, cards_before, now - opened)
                # Listen-in and level also while it speaks: the microphone
                # still hears, it is just not evaluated.
                self._distribute_audio(raw)
                live = time.monotonic() < self._live_until
                if live:
                    self._level = _level(raw)
                if self._voice is not None and self._voice.speaking:
                    # Keep reading so no old audio stays in the buffer,
                    # but evaluate none of it.
                    deaf = True
                    self._partial = ""
                    if self.awake:
                        # The window after "Robi" only starts running once
                        # the "Ja?" is over -- until then nobody can speak.
                        self._awake_until = time.monotonic() + config.SPEECH_WAKE_WINDOW_S
                    continue
                if deaf:
                    recognizer.Reset()
                    deaf = False
                if recognizer.AcceptWaveform(raw):
                    self._interpret(json.loads(recognizer.Result()))
                elif live:
                    part = json.loads(recognizer.PartialResult()).get("partial", "")
                    self._partial = " ".join(part.replace("[unk]", " ").split())
        finally:
            _no_wait(stream.close, "closing the microphone stream")

    @staticmethod
    def _check_better(rank: int, cards_before: str, open_s: float) -> None:
        """If recognition runs on a fallback microphone, go back to the better one.

        After a USB dropout the devices come back at different speeds. The
        retry then found only the worse one and stayed there, even when the
        better microphone was back seconds later. (Back then that was the
        C270; it is now excluded entirely.)

        It switches when a better one has newly registered. If it was already
        there when opening (and PortAudio still did not find it), only every
        SPEECH_MICROPHONE_RETRY_S -- otherwise the listening thread would keep switching.
        """
        map_control = _registered_sound_cards()
        for part in _microphone_names()[:rank]:
            part = part.lower()
            if part not in map_control:
                continue
            if part not in cards_before or open_s > config.SPEECH_MICROPHONE_RETRY_S:
                raise _MicrophoneChange(f"better microphone ({part!r}) is back")

    def _distribute_audio(self, raw: bytes) -> None:
        if not self._audio_subs:
            return
        with self._lock:
            subscriptions = list(self._audio_subs)
        for subscription in subscriptions:
            try:
                subscription.put_nowait(raw)
            except queue.Full:
                pass  # receiver too slow (Wi-Fi): better a gap than a delay

    def _interpret(self, result: dict) -> None:
        """Check a result from Vosk before it may count as a command."""
        text = result.get("text", "").replace("[unk]", " ")
        if not text.strip():
            return
        words = result.get("result") or []
        known = [w for w in words if w.get("word") != "[unk]"]
        confidence = min((float(w.get("conf", 1.0)) for w in known), default=1.0)
        self._confidence = round(confidence, 2)
        unknown = len(words) - len(known)
        command, wake_word = recognize(text, self.awake)
        salad = word_salad(text, command) if command is not None else None
        # Stricter rather than deaf during music: blocking entirely meant that
        # after "Robi, Beatbox" no "Robi" got through until the song ended.
        music = self._mukke is not None and self._mukke.running
        min_confidence = config.SPEECH_MIN_CONFIDENCE_MUSIC if music else config.SPEECH_MIN_CONFIDENCE
        reason = None
        if salad is not None:
            reason = f"Wortsalat, {salad}"
        elif confidence < min_confidence:
            reason = f"unsicher ({confidence:.2f})"
        elif words and unknown / len(words) > config.SPEECH_MAX_UNK_SHARE:
            reason = f"{unknown} von {len(words)} Wörtern unbekannt"
        elif music and command not in (None, STOP) and not (wake_word or self.awake):
            # "La da da" otherwise counts without the name -- the song itself
            # must not start another one.
            reason = "ohne Weckwort"
        details = {
            "konfidenz": self._confidence,
            "woerter": [
                {"wort": w.get("word"), "konfidenz": round(float(w.get("conf", 1.0)), 2)}
                for w in words
            ],
        }
        if reason is None:
            self.hear(text, details=details)
        elif music:
            self.hear(text, allowed=(STOP, MUSIC_OFF), reason=f"{reason}, Musik läuft", details=details)
        else:
            self.hear(text, allowed=(STOP,), reason=reason, details=details)

    def _check_pause(self) -> bool:
        """True while the microphone should stay closed; then sets the state.

        Closed rather than just deaf: an open stream keeps the USB bus busy and
        Vosk the CPU, and mapping needs both (lidar on the same USB, SLAM in
        the container).
        """
        reason = self._pause_reason()
        if reason != self._pause:
            if reason:
                log.info("Speech recognition paused: %s", reason)
            else:
                log.info("Speech recognition: pause over, reopening the microphone")
            self._pause = reason
        if not reason:
            return False
        self._state = PAUSED
        self._awake_until = 0.0  # a "Robi" from before no longer counts
        self._partial = ""
        self._level = 0.0
        return True

    def _disable(self, reason: str) -> None:
        self._error = reason
        self._state = ERROR
        log.warning("Speech recognition off: %s", reason)

    @staticmethod
    def _find_microphone(sd) -> tuple[int, str]:
        devices = [
            (i, g["name"]) for i, g in enumerate(sd.query_devices())
            if g.get("max_input_channels", 0) > 0
        ]
        preferred = config.SPEECH_MICROPHONE
        if isinstance(preferred, int):
            for i, name in devices:
                if i == preferred:
                    return i, name
        names = _microphone_names()
        # Never pick camera microphones on our own: the C270's one jumped in
        # after USB dropouts and understood hardly anything. A microphone set
        # explicitly was chosen deliberately by a human, so it still counts.
        candidates = devices if preferred else [
            (i, name) for i, name in devices
            if not any(excluded.lower() in name.lower() for excluded in config.SPEECH_MICROPHONE_NEVER)
        ]
        # Search by preference, not by device list: the preferred microphone
        # should win no matter which index it has.
        for part in names:
            for i, name in candidates:
                if part.lower() in name.lower():
                    return i, name
        raise RuntimeError(
            "kein Mikrofon gefunden (vorhanden: "
            + (", ".join(n for _, n in devices) or "keins") + ")"
        )

    @staticmethod
    def _sample_rate(sd, device: int) -> int:
        try:
            sd.check_input_settings(
                device=device, samplerate=config.SPEECH_SAMPLE_RATE, channels=1, dtype="int16"
            )
            return config.SPEECH_SAMPLE_RATE
        except Exception:
            return int(sd.query_devices(device)["default_samplerate"])

    @staticmethod
    def _grammar(model_path: str, model=None) -> list[str]:
        """All sentences Vosk may recognize.

        The model cannot recognize words it does not know -- "robi" is not
        in the small German model, for example, but "robbie" or "ruby" are.
        That is why the variants are checked against the vocabulary instead
        of flooding Vosk with warnings at every start.
        """
        known: set[str] | None = None
        words_txt = os.path.join(model_path, "graph", "words.txt")
        try:
            with open(words_txt, encoding="utf-8") as f:
                known = {line.split()[0] for line in f if line.strip()}
        except OSError:
            pass  # the small models have no words.txt, see below

        def knows(word: str) -> bool:
            if known is not None:
                return word in known
            # Without words.txt (e.g. vosk-model-small-de-0.15) the vocabulary
            # only lives in graph/Gr.fst; Vosk can query it itself
            # (-1 = unknown). Verified on the real model on 29.10.
            find = getattr(model, "vosk_model_find_word", None)
            if find is None:
                return True  # cannot check: try everything
            try:
                return find(word) >= 0
            except Exception:
                return True

        def possible(sentence: str) -> bool:
            return all(knows(w) for w in sentence.split())

        wake = [w for w in dict.fromkeys(
            [config.SPEECH_WAKE_WORD.lower(), *config.SPEECH_WAKE_WORD_VARIANTS]
        ) if possible(w)]
        if not wake:
            log.warning("Speech recognition: no wake word variant in the model -- only stop works")
        else:
            log.info("Speech recognition: wake word as %s", ", ".join(wake))
        phrases = [w for ws in COMMANDS.values() for w in ws if possible(w)]
        drive_commands = [s for s in _maneuver_phrases() if possible(s)]
        log.info("Speech recognition: %d precise drive commands in the grammar", len(drive_commands))
        phrases += drive_commands
        # health check answers are no commands: they only land in the events (heard_since())
        answers = [w for w in dict.fromkeys([*config.HEALTH_WORDS_HELP, *config.HEALTH_WORDS_NO])
                   if possible(w)]
        sentences = wake + phrases + answers
        # Name before and after the command as whole sentences: that way Vosk
        # fits what was spoken to one of the allowed sentences instead of building word salad.
        for w in wake:
            for b in phrases:
                sentences += [f"{w} {b}", f"{b} {w}"]
        return sentences + ["[unk]"]
