"""Music from YouTube on the USB speaker ("MUKKE" in the app).

First ``yt-dlp`` (as a Python library) asks YouTube for the direct address
of the audio track, then ``mpv`` plays it as a separate process, audio
only -- we neither have to download nor decode. mpv is controlled via its
IPC interface (a Unix socket, one JSON per line): pause, volume, query the
title.

A separate process instead of a Python library, because:
- a hanging or crashing player does not take the server down with it,
- mpv already handles network dropouts, buffering and format changes,
- stopping is simply "terminate the process".

When the voice speaks (interaction/voice.py), MUKKE frees the speaker and
afterwards continues at the same position -- both at once is not possible,
the device can only be opened once.

If mpv or yt-dlp is missing, ``status()`` reports it and everything else
keeps running.
"""

from __future__ import annotations

import collections
import json
import logging
import os
import re
import shutil
import socket
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlparse

from .. import config

log = logging.getLogger(__name__)

#: Only these addresses are played. Without this check, the field in the
#: app could pass any file on the Pi or any arbitrary source to mpv.
ALLOWED_HOSTS = {
    "youtube.com", "www.youtube.com", "m.youtube.com", "music.youtube.com", "youtu.be",
}


class MukkeError(ValueError):
    """The text goes to the app as is."""


def _yt_dlp_available() -> bool:
    try:
        import yt_dlp  # noqa: F401
    except ImportError:
        return False
    return True


def _find_audio_track(url: str) -> tuple[str, str | None, dict[str, str]]:
    """Asks yt-dlp for the direct address of the audio track.

    We ask ourselves instead of letting mpv do it via its ytdl script: that
    script often does not find yt-dlp in the venv, and if it fails, mpv only
    says "Failed to recognize file format". This way the real reason shows
    up in the app.
    """
    import yt_dlp

    options = {
        "format": "bestaudio/best",
        "quiet": True,
        "no_warnings": True,
        "noplaylist": True,  # link from a playlist: only this song
        "logger": _QuietLogger(),
    }
    try:
        with yt_dlp.YoutubeDL(options) as ydl:
            info = ydl.extract_info(url, download=False)
    except Exception as exc:  # DownloadError, but also network errors
        text = re.sub(r"\x1b\[[0-9;]*m", "", str(exc)).replace("ERROR: ", "").strip()
        raise MukkeError(f"YouTube: {text[:300]}") from None

    # With playlists there is sometimes an entry below despite noplaylist.
    if info.get("entries"):
        info = next((e for e in info["entries"] if e), info)
    source = info.get("url")
    if not source:
        raise MukkeError("YouTube: keine abspielbare Tonspur gefunden")
    return source, info.get("title"), dict(info.get("http_headers") or {})


class _QuietLogger:
    """yt-dlp must not write to the server log -- errors come as exceptions."""

    def debug(self, msg: str) -> None:
        pass

    def info(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        log.debug("yt-dlp: %s", msg)

    def error(self, msg: str) -> None:
        log.debug("yt-dlp: %s", msg)


def check_link(url: str) -> str:
    url = (url or "").strip()
    parts = urlparse(url)
    if parts.scheme not in ("http", "https") or (parts.hostname or "").lower() not in ALLOWED_HOSTS:
        raise MukkeError("Bitte einen YouTube-Link einfügen (youtube.com oder youtu.be)")
    return url


class Mukke:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._url: str | None = None
        self._title: str | None = None
        self._error: str | None = None
        self._volume = int(config.MUKKE_VOLUME)
        # Last output lines of mpv: so that "does not play" has a cause.
        self._output: collections.deque[str] = collections.deque(maxlen=20)
        # What mpv was last started with, and -- while the voice is
        # speaking -- where it should continue (see interrupt()).
        self._started: dict | None = None
        self._interrupted: dict | None = None
        self._socket = os.path.join(tempfile.gettempdir(), f"rosmaster-mukke-{os.getpid()}.sock")

    # --- State ----------------------------------------------------------------------

    def missing(self) -> str | None:
        """What is missing for playback, or None."""
        if not config.MUKKE_ENABLED:
            return "abgeschaltet (config.MUKKE_ENABLED)"
        if shutil.which("mpv") is None:
            return "mpv fehlt -- sudo apt install mpv"
        if not _yt_dlp_available():
            return "yt-dlp fehlt -- robot/.venv/bin/pip install -U yt-dlp"
        if not hasattr(socket, "AF_UNIX"):
            return "nur auf Linux"
        return None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def status(self) -> dict:
        with self._lock:
            running = self.running
            # If mpv exited by itself (song finished, broken link), get the
            # reason from its output.
            if self._proc is not None and not running and self._error is None:
                code = self._proc.returncode
                if code not in (0, None, -15):
                    self._error = self._last_message() or f"mpv beendet mit Code {code}"
            paused = bool(self._ipc(["get_property", "pause"])) if running else False
            if self._interrupted is not None:
                # The voice is speaking right now -- for the app the song keeps playing.
                running, paused = True, self._interrupted["pausiert"]
            if running and not self._title:
                title = self._ipc(["get_property", "media-title"])
                # Until yt-dlp is done, mpv only reports the address.
                if isinstance(title, str) and not title.startswith("http") and "watch?" not in title:
                    self._title = title
            return {
                "verfuegbar": self.missing() is None,
                "hinweis": self.missing(),
                "laeuft": running,
                "pausiert": paused,
                "url": self._url,
                "titel": self._title,
                "lautstaerke": self._volume,
                "fehler": self._error,
            }

    # --- Control --------------------------------------------------------------------

    def play(self, url: str) -> None:
        url = check_link(url)
        missing = self.missing()
        if missing:
            raise MukkeError(f"Abspielen geht nicht: {missing}")

        # Outside the lock: asking YouTube takes seconds, and /mukke/status
        # must not hang that long.
        source, title, headers = _find_audio_track(url)

        with self._lock:
            self._shutdown()
            self._interrupted = None
            self._url, self._title, self._error = url, title, None
            self._start(source, title, headers)
        log.info("MUKKE: playing %s", url)

    def interrupt(self) -> None:
        """Free the speaker while the voice is speaking.

        Pausing alone is not enough: mpv keeps the ALSA device open even when
        paused, and aplay would get "Device or resource busy". So terminate
        mpv and remember the position; ``resume()`` restarts there.
        """
        with self._lock:
            if not self.running or self._started is None:
                return
            position = self._ipc(["get_property", "time-pos"])
            self._interrupted = {
                **self._started,
                "start": float(position) if isinstance(position, (int, float)) else None,
                "pausiert": bool(self._ipc(["get_property", "pause"])),
            }
            self._shutdown(quiet=True)

    def resume(self) -> None:
        """After speaking, continue playing at the remembered position."""
        with self._lock:
            resumed, self._interrupted = self._interrupted, None
            if resumed is None or self.running:
                return
            self._start(**resumed)

    def _start(
        self,
        source: str,
        title: str | None,
        headers: dict[str, str],
        start: float | None = None,
        paused: bool = False,
    ) -> None:
        """Start mpv with the audio track. Only call while holding the lock."""
        self._output.clear()
        self._started = {"quelle": source, "titel": title, "kopfzeilen": headers}
        command = [
            "mpv",
            "--no-video",
            "--input-terminal=no",
            "--msg-level=all=warn",
            # We already have the audio track -- mpv must not ask YouTube
            # again by itself.
            "--ytdl=no",
            f"--audio-device={config.MUKKE_AUDIO_DEVICE}",
            f"--volume={self._effective_volume()}",
            f"--input-ipc-server={self._socket}",
        ]
        if title:
            command.append(f"--force-media-title={title}")
        # YouTube only delivers the audio track with the same User-Agent
        # that yt-dlp asked with. Passed individually instead of as
        # --http-header-fields: there mpv splits at commas, and those
        # occur in other headers (Accept).
        if headers.get("User-Agent"):
            command.append(f"--user-agent={headers['User-Agent']}")
        if headers.get("Referer"):
            command.append(f"--referrer={headers['Referer']}")
        if start:
            command.append(f"--start={start:.1f}")
        if paused:
            command.append("--pause")
        # "--" in front: an address starting with "-" would otherwise be an
        # mpv option.
        command += ["--", source]
        self._proc = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
        )
        threading.Thread(
            target=self._tail_output, args=(self._proc,), name="mukke-ausgabe", daemon=True
        ).start()

    def stop(self) -> None:
        with self._lock:
            self._shutdown()
            self._interrupted = self._started = None
            self._url = self._title = None

    def toggle_pause(self) -> None:
        with self._lock:
            if self.running:
                self._ipc(["cycle", "pause"])
            elif self._interrupted is not None:
                # While the voice is speaking: takes effect on resume.
                self._interrupted["pausiert"] = not self._interrupted["pausiert"]

    def set_volume(self, value: float) -> None:
        try:
            value = int(round(float(value)))
        except (TypeError, ValueError):
            raise MukkeError("Lautstärke als Zahl 0..100 erwartet") from None
        with self._lock:
            self._volume = max(0, min(100, value))
            if self.running:
                self._ipc(["set_property", "volume", self._effective_volume()])

    def apply_volume(self) -> None:
        """After a change of the overall volume (config.VOLUME)."""
        with self._lock:
            if self.running:
                self._ipc(["set_property", "volume", self._effective_volume()])

    # --- internal -------------------------------------------------------------------

    def _effective_volume(self) -> int:
        """What mpv gets: the music slider as a share of the overall volume."""
        overall = max(0.0, min(100.0, float(config.VOLUME)))
        return round(self._volume * overall / 100.0)

    def _shutdown(self, quiet: bool = False) -> None:
        proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        if not quiet:
            log.info("MUKKE: stopped")

    def _tail_output(self, proc: subprocess.Popen) -> None:
        # Without reading along, the output buffer would fill up and mpv would stall.
        for line in proc.stdout:
            line = line.strip()
            if line:
                self._output.append(line)
                log.debug("mpv: %s", line)

    def _last_message(self) -> str | None:
        for line in reversed(self._output):
            if "error" in line.lower() or "fehler" in line.lower() or "failed" in line.lower():
                return line[-200:]
        return self._output[-1][-200:] if self._output else None

    def _ipc(self, command: list):
        """One command to mpv, response ("data") or None.

        Short deadline: mpv answers immediately as long as it is running. If
        it hangs, the app must not hang with it.
        """
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect(self._socket)
                s.sendall((json.dumps({"command": command, "request_id": 1}) + "\n").encode())
                buffer = b""
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buffer += chunk
                    # mpv also sends unsolicited events -- the response
                    # is the line with our request_id.
                    for line in buffer.split(b"\n"):
                        try:
                            response = json.loads(line)
                        except ValueError:
                            continue
                        if response.get("request_id") == 1:
                            return response.get("data")
        except (OSError, ValueError):
            return None
        return None
