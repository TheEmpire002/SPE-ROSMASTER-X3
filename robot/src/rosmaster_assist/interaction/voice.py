"""Speech output via the USB speaker.

The robot briefly says what it is doing ("Ich folge dir.", "Bin da.") and
reads out texts from the app. Speaking happens in a separate thread with a
small queue: a sentence takes a second or two, and neither the event loop
nor the drive loop may wait that long.

Output, depending on what is installed on the Pi (see docs/hardware.md):

``piper``      Piper (offline, natural German voice) -> aplay
``espeak``     espeak-ng -> aplay, sounds tinny but is available almost everywhere
``text``       log only -- the server still runs
``simuliert``  ``--mock``: log only, speaking duration is imitated

The speaker is addressed by its ALSA card name, not by its number. If it
re-registers after a voltage drop, aplay finds it again under the same name
on the next sentence; a sentence that falls into the gap is lost, nothing
more.

``speaking`` is true while a sentence is playing (plus a short tail). Speech
recognition must not listen during that time, otherwise the robot would
understand itself.
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
SIMULATED = "simuliert"


def _attenuate(source, target, factor: float) -> None:
    """Pass raw audio from Piper (16 bit, mono) to aplay, attenuated by ``factor``."""
    import numpy as np

    rest = b""
    try:
        while True:
            data = source.read1(4096)
            if not data:
                break
            data = rest + data
            # A sample has two bytes; a half one waits for the next block.
            straight = len(data) - len(data) % 2
            data, rest = data[:straight], data[straight:]
            values = np.frombuffer(data, dtype="<i2").astype(np.float32) * factor
            target.write(values.astype("<i2").tobytes())
    except (OSError, ValueError):
        pass  # aplay gone or sentence cancelled -- then it is over anyway
    finally:
        for file in (source, target):
            try:
                file.close()
            except OSError:
                pass


class Announcement:
    """Live audio from the app (16 bit, mono) that goes out over the speaker.

    It sits in the same queue as the sentences. So everything that applies to
    the voice applies here too: MUKKE pauses, speech recognition stops
    listening (otherwise it would take the announcement as a command), and
    sentences and announcement do not talk over each other on the device.
    """

    def __init__(self, rate: int) -> None:
        self.rate = rate
        self._chunks: queue.Queue[bytes | None] = queue.Queue()
        self.finished = False
        # aplay did not work (speaker busy or gone): then the app should not
        # start the next announcement right away with every audio packet.
        self.failed = False

    def feed(self, data: bytes) -> None:
        self._chunks.put(data)

    def end(self) -> None:
        self._chunks.put(None)

    def __str__(self) -> str:  # for status()["warteschlange"]
        return "(Durchsage)"


class Voice:
    def __init__(self, mock: bool = False, before_speaking=None, after_speech=None) -> None:
        self._mock = mock
        # Callbacks before the first and after the last sentence of a
        # sequence: the server uses them to free the speaker (MUKKE pauses),
        # because the ALSA device can only be opened once.
        self._before_speaking = before_speaking
        self._after_speaking = after_speech
        self._lock = threading.Lock()
        self._new_signal = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._queue: collections.deque[str] = collections.deque()
        self._processes: list[subprocess.Popen] = []
        self._current: Announcement | None = None  # the one currently playing

        self._output = TEXT
        self._piper: str | None = None  # found path, see _find_output()
        self._sample_rate = 22050
        self._speaking_until = 0.0  # monotonic; > now means "speaking"
        self._speaking = False
        self._last: str | None = None
        self._last_time = 0.0
        self._said = 0
        self._discarded = 0
        self._error: str | None = None

    # --- Operation ------------------------------------------------------------

    @property
    def available(self) -> bool:
        """Does sound actually come out (or is it simulated)?"""
        return config.VOICE_ENABLED and self._output != TEXT

    @property
    def speaking(self) -> bool:
        return self._speaking or time.monotonic() < self._speaking_until

    @property
    def busy(self) -> bool:
        """Speaking right now or still has sentences in the queue."""
        with self._lock:
            waiting = bool(self._queue)
        return waiting or self.speaking

    def start(self) -> None:
        if not config.VOICE_ENABLED:
            log.info("Voice disabled (config.VOICE_ENABLED)")
            return
        self._output = self._find_output()
        log.info("Voice: output %s on %s", self._output, config.VOICE_DEVICE)
        if self._output in (PIPER, ESPEAK):
            self._set_volume()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="stimme", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.mute()
        self._new_signal.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def say(self, text: str, priority: bool = False) -> bool:
        """Put a sentence into the queue. False if it was discarded.

        ``priority`` cancels the current sentence and empties the queue --
        for things that are now more important than everything before (stop).
        """
        text = " ".join(str(text).split())[: config.VOICE_MAX_CHARS]
        if not text or not config.VOICE_ENABLED:
            return False
        now = time.monotonic()
        with self._lock:
            # The same sentence twice in quick succession is just annoying.
            if (
                not priority
                and text == self._last
                and now - self._last_time < config.VOICE_REPEAT_S
            ):
                return False
            if text in self._queue:
                return False
            self._last = text
            self._last_time = now
            if priority:
                self._discarded += len(self._queue)
                self._drain()
            while len(self._queue) >= config.VOICE_MAX_QUEUE:
                # The oldest one is dropped: it describes what is long over.
                old = self._queue.popleft()
                if isinstance(old, Announcement):
                    old.finished = True
                self._discarded += 1
            self._queue.append(text)
        if priority:
            self._stop_processes()
        self._new_signal.set()
        return True

    def announcement(self, rate: int) -> Announcement:
        """Begin an announcement; the audio follows via ``feed()``.

        If a sentence is playing, it comes right after it. A sentence with
        priority (stop) cancels it, then ``finished`` is true.
        """
        d = Announcement(rate)
        if not config.VOICE_ENABLED:
            d.finished = True
            return d
        with self._lock:
            self._queue.append(d)
        self._new_signal.set()
        return d

    def mute(self) -> None:
        """Empty the queue and cancel the current sentence (emergency stop)."""
        with self._lock:
            self._discarded += len(self._queue)
            self._drain()
        self._stop_processes()

    def _drain(self) -> None:
        """Empty the queue (under self._lock). Waiting announcements learn
        that they are over -- otherwise the app would send into the void."""
        for entry in self._queue:
            if isinstance(entry, Announcement):
                entry.finished = True
        self._queue.clear()

    def state(self) -> dict:
        """For state.stimme in the app."""
        return {
            "verfuegbar": self.available,
            "spricht": self.speaking,
            "text": self._last,
        }

    def status(self) -> dict:
        with self._lock:
            return {
                "aktiviert": config.VOICE_ENABLED,
                "verfuegbar": self.available,
                "simuliert": self._mock,
                "ausgabe": self._output,
                "geraet": config.VOICE_DEVICE,
                "spricht": self.speaking,
                "warteschlange": [str(t) for t in self._queue],
                "zuletzt": self._last,
                "gesagt": self._said,
                "verworfen": self._discarded,
                "fehler": self._error,
            }

    # --- Speech thread ------------------------------------------------------------

    def _loop(self) -> None:
        while not self._stop.is_set():
            self._new_signal.wait(timeout=1.0)
            self._new_signal.clear()
            # Free the speaker once per sequence of sentences, not before
            # every sentence: briefly playing music between two sentences
            # would sound like a glitch.
            cleared = False
            while not self._stop.is_set():
                with self._lock:
                    if not self._queue:
                        break
                    text = self._queue.popleft()
                    # Text-only is not audible -- then nobody needs to stop
                    # listening. Set it right under the lock, otherwise
                    # ``busy`` would briefly be wrong until the first sound.
                    audible = self._output != TEXT
                    self._speaking = audible
                if audible and not cleared:
                    cleared = True
                    self._call(self._before_speaking)
                    # After a USB dropout the speaker re-registers with its
                    # default mixer -- so set it every time.
                    if self._output in (PIPER, ESPEAK):
                        self._set_volume(report=False)
                self._speaking = audible
                try:
                    self._speak(text)
                except Exception as exc:  # never lose the thread
                    self._error = f"{type(exc).__name__}: {exc}"
                    log.exception("Voice: sentence failed")
                finally:
                    if audible:
                        self._speaking_until = time.monotonic() + config.VOICE_TAIL_S
                    self._speaking = False
            if cleared:
                self._call(self._after_speaking)

    @staticmethod
    def _call(callback) -> None:
        if callback is None:
            return
        try:
            callback()
        except Exception:  # music must not switch off the voice
            log.exception("Voice: callback failed")

    def _play_announcement(self, d: Announcement) -> None:
        if d.finished:
            return  # already cancelled in the queue
        log.info("Voice: announcement from the app (%d Hz)", d.rate)
        factor = max(0.0, min(100.0, float(config.VOLUME))) / 100.0
        target = None
        if self._output not in (SIMULATED, TEXT):
            target = subprocess.Popen(
                ["aplay", "-q", "-D", config.VOICE_DEVICE, "-r", str(d.rate), "-f", "S16_LE",
                 "-c", "1", "-t", "raw", f"--buffer-time={config.ANNOUNCE_APLAY_BUFFER_US}"],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        with self._lock:
            self._processes = [target] if target is not None else []
            self._current = d
        # Whatever piled up while a sentence was still playing is outdated.
        max_bytes = int(d.rate * 2 * config.ANNOUNCE_MAX_BUFFER_S)
        import numpy as np

        try:
            while not self._stop.is_set():
                try:
                    data = d._chunks.get(timeout=config.ANNOUNCE_PAUSE_S)
                except queue.Empty:
                    break  # app silent or gone: free the speaker
                if data is None:
                    break
                backlog = d._chunks.qsize() * len(data)
                while backlog > max_bytes:
                    try:
                        path = d._chunks.get_nowait()
                    except queue.Empty:
                        break
                    if path is None:
                        return
                    backlog -= len(path)
                if target is None:
                    continue  # simulation: discard audio, but "speaking" stays true
                if target.poll() is not None:
                    break  # cancelled (priority) or speaker gone
                data = data[: len(data) - len(data) % 2]
                if factor < 0.999:
                    values = np.frombuffer(data, dtype="<i2").astype(np.float32) * factor
                    data = values.astype("<i2").tobytes()
                target.stdin.write(data)
                target.stdin.flush()
        except (OSError, ValueError):
            pass  # aplay gone (priority, USB) -- the announcement is over then
        finally:
            d.finished = True
            if target is not None:
                try:
                    target.stdin.close()
                    target.wait(timeout=2.0)
                except (OSError, ValueError, subprocess.TimeoutExpired):
                    pass
                self._stop_processes()
                # Positive code: aplay itself failed (busy, gone).
                # Negative would mean cancelled (priority) -- that is not an error.
                if (target.returncode or 0) > 0:
                    d.failed = True
                    message = f"Durchsage: Lautsprecher geht nicht (aplay Code {target.returncode})"
                    if message != self._error:
                        log.warning("Voice: %s", message)
                    self._error = message
            log.info("Voice: announcement finished")

    def _speak(self, text: str | Announcement) -> None:
        if isinstance(text, Announcement):
            self._play_announcement(text)
            return
        log.info("Voice says: %s", text)
        if self._output == SIMULATED:
            # Roughly as long as real speaking, so that ``speaking`` behaves
            # as on the robot.
            end = time.monotonic() + min(5.0, 0.3 + 0.06 * len(text))
            while time.monotonic() < end and not self._stop.is_set():
                time.sleep(0.05)
            self._said += 1
            return
        if self._output == TEXT:
            return

        # The single volume control (config.VOLUME); the mixer is fixed.
        factor = max(0.0, min(100.0, float(config.VOLUME))) / 100.0
        aplay = ["aplay", "-q", "-D", config.VOICE_DEVICE]
        if self._output == PIPER:
            producer = [
                self._piper,
                "--model", os.path.expanduser(config.VOICE_MODEL),
                "--length_scale", str(config.VOICE_SPEED),
                "--output-raw",
            ]
            if config.VOICE_CORES and shutil.which("taskset"):
                producer = ["taskset", "-c", str(config.VOICE_CORES), *producer]
            aplay += ["-r", str(self._sample_rate), "-f", "S16_LE", "-c", "1", "-t", "raw"]
        else:
            # espeak controls it itself (amplitude, 100 = normal). Its output
            # is WAV with a header, which could not simply be scaled.
            producer = [config.VOICE_ESPEAK, "-v", "de", "-s", "150",
                        "-a", str(round(100 * factor)), "--stdout"]
        # Piper delivers raw 16-bit samples: we attenuate them ourselves.
        regulate = self._output == PIPER and factor < 0.999

        source = subprocess.Popen(
            producer, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        target = subprocess.Popen(
            aplay, stdin=subprocess.PIPE if regulate else source.stdout,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        )
        if regulate:
            threading.Thread(
                target=_attenuate, args=(source.stdout, target.stdin, factor), name="stimme-leiser", daemon=True
            ).start()
        else:
            source.stdout.close()  # now belongs to aplay -- otherwise aplay never ends
        with self._lock:
            self._processes = [source, target]
        try:
            source.stdin.write((text + "\n").encode("utf-8"))
            source.stdin.close()
            # wait instead of communicate: communicate would close aplay's
            # input, which _attenuate() is still writing to. aplay only
            # reports a few lines.
            target.wait(timeout=config.VOICE_TIMEOUT_S)
            error = target.stderr.read()
            source.wait(timeout=2.0)
        except BrokenPipeError:
            error = b"Ausgabeprogramm vorzeitig beendet"
        except subprocess.TimeoutExpired:
            error = b"Zeitlimit beim Sprechen"
        finally:
            self._stop_processes()

        if target.returncode == 0 and (source.returncode or 0) > 0:
            # aplay ran but got nothing: Piper/espeak crashed
            # (e.g. broken model).
            message = f"{producer[0]} Code {source.returncode}"
            if message != self._error:
                log.warning("Voice: speech synthesis failed: %s", message)
            self._error = message
        elif target.returncode == 0:
            if self._error:
                log.info("Voice: speaker is back")
            self._error = None
            self._said += 1
        elif target.returncode is not None and target.returncode < 0:
            pass  # deliberately cancelled (mute / priority)
        else:
            message = (error or b"").decode("utf-8", "replace").strip() or f"aplay Code {target.returncode}"
            if message != self._error:
                # Report it loudly only once, not on every sentence while the
                # speaker is missing.
                log.warning("Voice: output failed: %s", message)
            self._error = message

    def _stop_processes(self) -> None:
        with self._lock:
            processes, self._processes = self._processes, []
            current, self._current = self._current, None
        if current is not None:
            # End immediately even without aplay (simulation): end() wakes the
            # waiting speech thread, finished lets the app start again.
            current.finished = True
            current.end()
        for p in processes:
            if p.poll() is None:
                try:
                    p.kill()
                except OSError:
                    pass

    # --- Setup ----------------------------------------------------------------------

    def _find_output(self) -> str:
        if self._mock:
            return SIMULATED
        if shutil.which("aplay") is None:
            self._error = "aplay fehlt (Paket alsa-utils)"
            log.warning("Voice: %s -- text only", self._error)
            return TEXT
        model = os.path.expanduser(config.VOICE_MODEL)
        # The Piper release ships its libraries in its own folder; so search
        # there, not only in PATH.
        self._piper = next(
            (p for p in (shutil.which(os.path.expanduser(n)) for n in config.VOICE_PIPER) if p),
            None,
        )
        if self._piper and os.path.isfile(model):
            self._sample_rate = self._read_sample_rate(model)
            return PIPER
        if shutil.which(config.VOICE_ESPEAK):
            log.warning("Voice: Piper or model %s missing -- falling back to espeak-ng", model)
            return ESPEAK
        self._error = "weder Piper noch espeak-ng installiert"
        log.warning("Voice: %s -- text only", self._error)
        return TEXT

    @staticmethod
    def _read_sample_rate(model: str) -> int:
        """Piper stores the sample rate in ``<model>.json``; aplay needs to know it."""
        try:
            with open(model + ".json", encoding="utf-8") as f:
                return int(json.load(f)["audio"]["sample_rate"])
        except (OSError, ValueError, KeyError, TypeError):
            return 22050

    @staticmethod
    def _set_volume(report: bool = True) -> None:
        """Set the mixer to SOUND_MIXER via amixer. The control has a different name per device.

        Attenuation happens in software (VOLUME), so that there is only one control.
        """
        hits = re.search(r"CARD=([^,]+)", config.VOICE_DEVICE)
        if hits is None or shutil.which("amixer") is None:
            return
        occ_map = hits.group(1)
        for control in ("PCM", "Speaker", "Master", "Headphone"):
            try:
                result = subprocess.run(
                    ["amixer", "-q", "-c", occ_map, "sset", control, f"{config.SOUND_MIXER}%"],
                    capture_output=True,
                    timeout=3.0,
                )
            except (OSError, subprocess.TimeoutExpired):
                return
            if result.returncode == 0:
                if report:
                    log.info("Voice: mixer %d %% (%s), volume %d %% in software",
                             config.SOUND_MIXER, control, config.VOLUME)
                return
