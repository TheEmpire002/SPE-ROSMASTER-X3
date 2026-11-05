"""Cleans the microphone signal before speech recognition.

The microphone is only used for speech commands. Recordings from the robot
(mic tests 05.11.) showed:

* a strong hum below 100 Hz (46/52 Hz and harmonics) at about -20 dBFS,
  louder than the voice itself (speech band -28 to -35 dBFS),
* motor noise mostly below 50 Hz while driving, plus broadband noise,
* a quiet voice, 25-30 dB below full scale.

``SpeechFilter`` therefore works in the frequency domain (STFT, 50 % overlap,
square-root Hann window, so the unprocessed signal is reconstructed exactly):

1. band limit: everything outside ``SPEECH_FILTER_LOW_HZ`` ..
   ``SPEECH_FILTER_HIGH_HZ`` is removed (hum, DC, motor rumble, hiss),
2. noise suppression: a noise spectrum is estimated per frequency from the
   minimum over a sliding window (follows a new noise like the motor within
   ``SPEECH_FILTER_NOISE_WINDOW_S``), then a Wiener gain with the
   decision-directed SNR (little "musical noise"),
3. automatic gain: frames with speech bring the voice to
   ``SPEECH_FILTER_TARGET_DBFS``, in between the gain is held, so the
   remaining noise is not amplified,
4. a soft limiter keeps loud words from clipping.

Pure numpy, no state outside the object: one filter per microphone stream.
"""

from __future__ import annotations

import numpy as np

from .. import config

_EPS = 1e-12


def _db(power: float) -> float:
    return 10.0 * np.log10(max(power, _EPS))


class SpeechFilter:
    """Stateful filter: int16 mono blocks in, equally long int16 blocks out."""

    def __init__(self, rate: int) -> None:
        self.rate = rate
        # about 32 ms frames: short enough for speech, fine enough for 50 Hz hum
        self.frame = 1 << int(np.ceil(np.log2(rate * 0.032)))
        self.hop = self.frame // 2
        n = np.arange(self.frame)
        self._window = np.sqrt(0.5 - 0.5 * np.cos(2 * np.pi * n / self.frame))
        freqs = np.fft.rfftfreq(self.frame, 1.0 / rate)
        self._band = self._band_mask(freqs)
        self._speech_bins = (freqs >= 300) & (freqs <= 3400)
        bins = len(freqs)
        hop_s = self.hop / rate

        # noise: minimum of the smoothed power over sub-windows (minimum statistics)
        self._sub_frames = max(1, int(round(config.SPEECH_FILTER_NOISE_WINDOW_S / 8 / hop_s)))
        self._subs = np.full((8, bins), np.inf)
        self._sub_min = np.full(bins, np.inf)
        self._sub_count = 0
        self._smooth = None
        self.noise = None
        self._prev_clean = np.zeros(bins)

        self._gain_db = 0.0
        self._level_db = config.SPEECH_FILTER_TARGET_DBFS
        # release is slow so the gain does not chase every syllable
        self._release_db = config.SPEECH_FILTER_RELEASE_DB_S * hop_s

        self._input = np.zeros(0)
        self._overlap = np.zeros(self.frame - self.hop)
        # the first frame needs a full window, hence a fixed latency of one frame
        self._output = np.zeros(self.frame)

        # values of the last block, for the level display and the voice check
        self.voiced = False
        self.voiced_share = 0.0
        self.snr_db = 0.0
        self.clipped_share = 0.0
        self.gain_db = 0.0

    def _band_mask(self, freqs: np.ndarray) -> np.ndarray:
        low, high = config.SPEECH_FILTER_LOW_HZ, config.SPEECH_FILTER_HIGH_HZ
        edge = config.SPEECH_FILTER_EDGE_HZ
        mask = np.ones_like(freqs)
        # half-cosine edges instead of a hard cut: no ringing at the band limits
        rise = np.clip((freqs - (low - edge)) / edge, 0.0, 1.0)
        fall = np.clip(((high + edge) - freqs) / edge, 0.0, 1.0)
        mask *= 0.5 - 0.5 * np.cos(np.pi * rise)
        mask *= 0.5 - 0.5 * np.cos(np.pi * fall)
        return mask

    def process(self, raw: bytes) -> bytes:
        samples = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
        self.clipped_share = float(np.mean(np.abs(samples) > 0.99)) if len(samples) else 0.0
        self._input = np.concatenate([self._input, samples])
        pieces = [self._output]
        voiced = frames = 0
        snr = []
        while len(self._input) >= self.frame:
            out, is_voiced, frame_snr = self._frame(self._input[: self.frame])
            self._input = self._input[self.hop:]
            pieces.append(out)
            voiced += is_voiced
            frames += 1
            snr.append(frame_snr)
        output = np.concatenate(pieces)
        result, self._output = output[: len(samples)], output[len(samples):]
        if frames:
            self.voiced_share = voiced / frames
            self.voiced = self.voiced_share >= config.SPEECH_FILTER_VOICED_SHARE
            self.snr_db = float(max(snr))
        self.gain_db = self._gain_db
        return (self._limit(result) * 32767.0).astype("<i2").tobytes()

    def _frame(self, chunk: np.ndarray) -> tuple[np.ndarray, bool, float]:
        spectrum = np.fft.rfft(chunk * self._window) * self._band
        power = np.abs(spectrum) ** 2
        noise = self._track_noise(power)

        # Wiener gain from the decision-directed a-priori snr
        posterior = power / noise
        prior = 0.98 * self._prev_clean / noise + 0.02 * np.maximum(posterior - 1.0, 0.0)
        gain = np.maximum(prior / (1.0 + prior), config.SPEECH_FILTER_FLOOR)
        clean = spectrum * gain
        self._prev_clean = np.abs(clean) ** 2

        band = self._speech_bins
        snr_db = _db(power[band].sum()) - _db(noise[band].sum())
        is_voiced = snr_db >= config.SPEECH_FILTER_VOICE_SNR_DB
        self._update_gain(self._prev_clean[band].sum(), is_voiced)

        out = np.fft.irfft(clean * 10.0 ** (self._gain_db / 20.0), self.frame) * self._window
        out[: len(self._overlap)] += self._overlap
        self._overlap = out[self.hop:].copy()
        return out[: self.hop], is_voiced, snr_db

    def _track_noise(self, power: np.ndarray) -> np.ndarray:
        if self._smooth is None:
            self._smooth = power.copy()
            self.noise = np.maximum(power, _EPS)
        self._smooth = 0.8 * self._smooth + 0.2 * power
        self._sub_min = np.minimum(self._sub_min, self._smooth)
        self._sub_count += 1
        if self._sub_count >= self._sub_frames:
            # roll the window: the oldest sub-window falls out, so a louder
            # noise (motor starts) is fully learned after the window length
            self._subs = np.roll(self._subs, 1, axis=0)
            self._subs[0] = self._sub_min
            self._sub_min = np.full_like(self._sub_min, np.inf)
            self._sub_count = 0
        floor = np.minimum(self._subs.min(axis=0), self._sub_min)
        # the minimum lies below the mean noise power, hence the factor
        self.noise = np.maximum(floor * config.SPEECH_FILTER_NOISE_FACTOR, _EPS)
        return self.noise

    def _update_gain(self, speech_power: float, is_voiced: bool) -> None:
        if not is_voiced:
            return  # hold: between words the gain must not lift the rest noise
        # frame rms from the band power of a sqrt-Hann frame (Parseval)
        rms_db = _db(speech_power * 2.0 / (self.frame * np.sum(self._window ** 2)))
        self._level_db += 0.2 * (rms_db - self._level_db)
        wanted = config.SPEECH_FILTER_TARGET_DBFS - self._level_db
        wanted = min(max(wanted, 0.0), config.SPEECH_FILTER_MAX_GAIN_DB)
        if wanted < self._gain_db:
            self._gain_db = wanted  # too loud: lower at once
        else:
            self._gain_db = min(wanted, self._gain_db + self._release_db)

    @staticmethod
    def _limit(x: np.ndarray) -> np.ndarray:
        # soft knee above the threshold, never beyond full scale
        t = config.SPEECH_FILTER_LIMIT
        over = np.abs(x) > t
        if np.any(over):
            x = x.copy()
            excess = (np.abs(x[over]) - t) / (1.0 - t)
            x[over] = np.sign(x[over]) * (t + (1.0 - t) * np.tanh(excess))
        return x
