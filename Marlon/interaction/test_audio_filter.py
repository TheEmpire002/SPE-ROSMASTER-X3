"""Checks the speech filter in front of Vosk -- synthetic signals, no microphone.

    python Marlon/interaction/test_audio_filter.py

Tested:

* output blocks are as long as the input blocks (Vosk gets the same stream),
* 50 Hz hum is removed, a 1 kHz tone in the speech band stays,
* steady noise is pushed down, a "voice" burst on top of it stands out,
* the automatic gain lifts a quiet voice but never beyond the maximum,
* the limiter never clips, even for a full-scale input,
* the speech recognition falls back to raw audio when the filter is off.
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaction import speech as sp  # noqa: E402
from rosmaster_assist.interaction.audio_filter import SpeechFilter  # noqa: E402

RATE = 16000
BLOCK = RATE // 4
error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, result)
    error += not condition


def to_bytes(x):
    return (np.clip(x, -1, 1) * 32767).astype("<i2").tobytes()


def run(signal, flt=None):
    """Feeds the signal block by block, returns the output as floats."""
    flt = flt or SpeechFilter(RATE)
    out = []
    flt.max_snr = -99.0
    for i in range(0, len(signal), BLOCK):
        raw = to_bytes(signal[i:i + BLOCK])
        res = flt.process(raw)
        assert len(res) == len(raw), (len(res), len(raw))
        flt.max_snr = max(flt.max_snr, flt.snr_db)
        out.append(np.frombuffer(res, dtype="<i2") / 32768.0)
    return np.concatenate(out), flt


def dbfs(x):
    return 20 * np.log10(max(np.sqrt(np.mean(x ** 2)), 1e-9))


t = np.arange(RATE * 4) / RATE
rng = np.random.default_rng(1)

# --- Length -------------------------------------------------------------------------
flt = SpeechFilter(RATE)
lengths = {len(flt.process(to_bytes(rng.normal(0, 0.01, n)))) for n in (4000, 1, 777, 4000)}
check("output as long as input, also odd sizes", lengths == {8000, 2, 1554}, lengths)

# --- Hum vs. tone ---------------------------------------------------------------------
hum, _ = run(0.1 * np.sin(2 * np.pi * 50 * t))
check("50 Hz hum removed", dbfs(hum[RATE:]) < dbfs(0.1 * np.sin(2 * np.pi * 50 * t)) - 30,
      f"{dbfs(hum[RATE:]):.1f} dBFS")

# --- Steady noise and a voice burst ------------------------------------------------
noise = rng.normal(0, 0.02, len(t))
burst = np.zeros_like(t)
on = (t > 3.0) & (t < 3.5)
burst[on] = 0.1 * np.sin(2 * np.pi * 1000 * t[on])
out, flt = run(noise + burst)
quiet = out[int(2.0 * RATE):int(2.9 * RATE)]
loud = out[int(3.1 * RATE):int(3.4 * RATE)]
check("steady noise pushed down", dbfs(quiet) < dbfs(noise) - 5,
      f"noise {dbfs(noise):.1f} -> {dbfs(quiet):.1f} dBFS")
check("1 kHz burst stands out of the noise", dbfs(loud) - dbfs(quiet) > 20,
      f"{dbfs(loud) - dbfs(quiet):.1f} dB")
check("burst counted as voice", flt.max_snr > config.SPEECH_FILTER_VOICE_SNR_DB, f"{flt.max_snr:.1f} dB")

# --- Gain -----------------------------------------------------------------------------
quiet_voice = np.zeros_like(t)
for start in (0.5, 1.5, 2.5, 3.5):
    on = (t > start) & (t < start + 0.4)
    quiet_voice[on] = 0.003 * np.sin(2 * np.pi * 800 * t[on])
_, flt = run(quiet_voice + rng.normal(0, 0.0003, len(t)))
check("quiet voice lifted", flt.gain_db > 6, f"{flt.gain_db:.1f} dB")
check("gain bounded by the maximum", flt.gain_db <= config.SPEECH_FILTER_MAX_GAIN_DB + 1e-9,
      f"{flt.gain_db:.1f} dB")

# --- Limiter ----------------------------------------------------------------------------
square = np.sign(np.sin(2 * np.pi * 700 * t)) * 0.999
out, flt = run(square)
check("limiter keeps below full scale", np.max(np.abs(out)) < 0.999, f"{np.max(np.abs(out)):.3f}")
check("clipped input reported", flt.clipped_share > 0.5, f"{flt.clipped_share:.2f}")

# --- Speech recognition switch -------------------------------------------------------
s = sp.SpeechRecognition(mock=True)
check("filter built when enabled", s._make_filter(RATE) is not None)
old = config.SPEECH_FILTER_ENABLED
config.SPEECH_FILTER_ENABLED = False
check("no filter when disabled", s._make_filter(RATE) is None and s._filter_stats is None)
config.SPEECH_FILTER_ENABLED = old

print("\nall passed" if not error else f"\n{error} failed")
sys.exit(1 if error else 0)
