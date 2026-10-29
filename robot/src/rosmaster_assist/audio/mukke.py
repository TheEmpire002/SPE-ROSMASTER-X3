"""Musik von YouTube auf dem USB-Lautsprecher ("MUKKE" in der App).

Erst fragt ``yt-dlp`` (als Python-Bibliothek) YouTube nach der direkten
Adresse der Tonspur, dann spielt ``mpv`` als eigener Prozess sie ab, nur
den Ton -- wir müssen weder herunterladen noch dekodieren. Gesteuert wird
mpv über seine IPC-Schnittstelle (ein Unix-Socket, JSON pro Zeile): Pause,
Lautstärke, Titel abfragen.

Ein eigener Prozess statt einer Python-Bibliothek, weil:
- ein hängender oder abstürzender Player den Server nicht mitreißt,
- mpv Netzwerkaussetzer, Puffern und Formatwechsel schon alles kann,
- Stoppen einfach "Prozess beenden" ist.

Fehlt mpv oder yt-dlp, meldet ``status()`` das, und alles andere läuft
weiter.
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

#: Nur diese Adressen werden abgespielt. Ohne Prüfung ließe sich über das
#: Feld in der App jede Datei auf dem Pi oder jede beliebige Quelle an mpv
#: übergeben.
ERLAUBTE_HOSTS = {
    "youtube.com", "www.youtube.com", "m.youtube.com", "music.youtube.com", "youtu.be",
}


class MukkeFehler(ValueError):
    """Text geht so an die App."""


def _yt_dlp_da() -> bool:
    try:
        import yt_dlp  # noqa: F401
    except ImportError:
        return False
    return True


def _tonspur_finden(url: str) -> tuple[str, str | None, dict[str, str]]:
    """Fragt yt-dlp nach der direkten Adresse der Tonspur.

    Selbst gefragt statt mpv das über sein ytdl-Skript machen zu lassen:
    das findet yt-dlp in der venv oft nicht, und scheitert es, sagt mpv nur
    "Failed to recognize file format". So steht der echte Grund in der App.
    """
    import yt_dlp

    optionen = {
        "format": "bestaudio/best",
        "quiet": True,
        "no_warnings": True,
        "noplaylist": True,  # Link aus einer Playlist: nur dieses Lied
        "logger": _StillerLogger(),
    }
    try:
        with yt_dlp.YoutubeDL(optionen) as ydl:
            info = ydl.extract_info(url, download=False)
    except Exception as exc:  # DownloadError, aber auch Netzwerkfehler
        text = re.sub(r"\x1b\[[0-9;]*m", "", str(exc)).replace("ERROR: ", "").strip()
        raise MukkeFehler(f"YouTube: {text[:300]}") from None

    # Bei Playlists trotz noplaylist manchmal ein Eintrag darunter.
    if info.get("entries"):
        info = next((e for e in info["entries"] if e), info)
    quelle = info.get("url")
    if not quelle:
        raise MukkeFehler("YouTube: keine abspielbare Tonspur gefunden")
    return quelle, info.get("title"), dict(info.get("http_headers") or {})


class _StillerLogger:
    """yt-dlp soll nicht ins Server-Log schreiben -- Fehler kommen als Ausnahme."""

    def debug(self, msg: str) -> None:
        pass

    def info(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        log.debug("yt-dlp: %s", msg)

    def error(self, msg: str) -> None:
        log.debug("yt-dlp: %s", msg)


def link_pruefen(url: str) -> str:
    url = (url or "").strip()
    teile = urlparse(url)
    if teile.scheme not in ("http", "https") or (teile.hostname or "").lower() not in ERLAUBTE_HOSTS:
        raise MukkeFehler("Bitte einen YouTube-Link einfügen (youtube.com oder youtu.be)")
    return url


class Mukke:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._url: str | None = None
        self._titel: str | None = None
        self._fehler: str | None = None
        self._lautstaerke = int(config.MUKKE_LAUTSTAERKE)
        # Letzte Ausgabezeilen von mpv: damit "spielt nicht" eine Ursache hat.
        self._ausgabe: collections.deque[str] = collections.deque(maxlen=20)
        self._socket = os.path.join(tempfile.gettempdir(), f"rosmaster-mukke-{os.getpid()}.sock")

    # --- Zustand --------------------------------------------------------------------

    def fehlt(self) -> str | None:
        """Was zum Abspielen fehlt, oder None."""
        if not config.MUKKE_ENABLED:
            return "abgeschaltet (config.MUKKE_ENABLED)"
        if shutil.which("mpv") is None:
            return "mpv fehlt -- sudo apt install mpv"
        if not _yt_dlp_da():
            return "yt-dlp fehlt -- robot/.venv/bin/pip install -U yt-dlp"
        if not hasattr(socket, "AF_UNIX"):
            return "nur auf Linux"
        return None

    @property
    def laeuft(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def status(self) -> dict:
        with self._lock:
            laeuft = self.laeuft
            # Ist mpv von selbst beendet (Lied zu Ende, Link kaputt), den
            # Grund aus seiner Ausgabe holen.
            if self._proc is not None and not laeuft and self._fehler is None:
                code = self._proc.returncode
                if code not in (0, None, -15):
                    self._fehler = self._letzte_meldung() or f"mpv beendet mit Code {code}"
            pausiert = bool(self._ipc(["get_property", "pause"])) if laeuft else False
            if laeuft and not self._titel:
                titel = self._ipc(["get_property", "media-title"])
                # Bis yt-dlp fertig ist, nennt mpv nur die Adresse.
                if isinstance(titel, str) and not titel.startswith("http") and "watch?" not in titel:
                    self._titel = titel
            return {
                "verfuegbar": self.fehlt() is None,
                "hinweis": self.fehlt(),
                "laeuft": laeuft,
                "pausiert": pausiert,
                "url": self._url,
                "titel": self._titel,
                "lautstaerke": self._lautstaerke,
                "fehler": self._fehler,
            }

    # --- Steuerung ------------------------------------------------------------------

    def abspielen(self, url: str) -> None:
        url = link_pruefen(url)
        fehlt = self.fehlt()
        if fehlt:
            raise MukkeFehler(f"Abspielen geht nicht: {fehlt}")

        # Außerhalb der Sperre: das Nachfragen bei YouTube dauert Sekunden,
        # und so lange soll /mukke/status nicht hängen.
        quelle, titel, kopfzeilen = _tonspur_finden(url)

        with self._lock:
            self._beenden()
            self._ausgabe.clear()
            self._url, self._titel, self._fehler = url, titel, None
            befehl = [
                "mpv",
                "--no-video",
                "--input-terminal=no",
                "--msg-level=all=warn",
                # Die Tonspur haben wir schon -- mpv soll nicht selbst noch
                # einmal bei YouTube nachfragen.
                "--ytdl=no",
                f"--audio-device={config.MUKKE_AUDIO_DEVICE}",
                f"--volume={self._lautstaerke}",
                f"--input-ipc-server={self._socket}",
            ]
            if titel:
                befehl.append(f"--force-media-title={titel}")
            # YouTube liefert die Tonspur nur mit demselben User-Agent, mit
            # dem yt-dlp gefragt hat. Einzeln übergeben statt als
            # --http-header-fields: dort trennt mpv an Kommas, und die
            # stecken in anderen Kopfzeilen (Accept) drin.
            if kopfzeilen.get("User-Agent"):
                befehl.append(f"--user-agent={kopfzeilen['User-Agent']}")
            if kopfzeilen.get("Referer"):
                befehl.append(f"--referrer={kopfzeilen['Referer']}")
            # "--" davor: eine Adresse, die mit "-" beginnt, wäre sonst eine
            # mpv-Option.
            befehl += ["--", quelle]
            self._proc = subprocess.Popen(
                befehl,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
            )
            threading.Thread(
                target=self._mitlesen, args=(self._proc,), name="mukke-ausgabe", daemon=True
            ).start()
        log.info("MUKKE: spiele %s", url)

    def stoppen(self) -> None:
        with self._lock:
            self._beenden()
            self._url = self._titel = None

    def pause_umschalten(self) -> None:
        with self._lock:
            if self.laeuft:
                self._ipc(["cycle", "pause"])

    def lautstaerke_setzen(self, wert: float) -> None:
        try:
            wert = int(round(float(wert)))
        except (TypeError, ValueError):
            raise MukkeFehler("Lautstärke als Zahl 0..100 erwartet") from None
        with self._lock:
            self._lautstaerke = max(0, min(100, wert))
            if self.laeuft:
                self._ipc(["set_property", "volume", self._lautstaerke])

    # --- intern ---------------------------------------------------------------------

    def _beenden(self) -> None:
        proc, self._proc = self._proc, None
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.info("MUKKE: gestoppt")

    def _mitlesen(self, proc: subprocess.Popen) -> None:
        # Ohne Mitlesen liefe der Ausgabepuffer voll, und mpv bliebe stehen.
        for zeile in proc.stdout:
            zeile = zeile.strip()
            if zeile:
                self._ausgabe.append(zeile)
                log.debug("mpv: %s", zeile)

    def _letzte_meldung(self) -> str | None:
        for zeile in reversed(self._ausgabe):
            if "error" in zeile.lower() or "fehler" in zeile.lower() or "failed" in zeile.lower():
                return zeile[-200:]
        return self._ausgabe[-1][-200:] if self._ausgabe else None

    def _ipc(self, befehl: list):
        """Ein Befehl an mpv, Antwort ("data") oder None.

        Kurze Frist: mpv antwortet sofort, solange es läuft. Hängt es, soll
        die App nicht mit hängen.
        """
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect(self._socket)
                s.sendall((json.dumps({"command": befehl, "request_id": 1}) + "\n").encode())
                puffer = b""
                frist = time.monotonic() + 0.5
                while time.monotonic() < frist:
                    stueck = s.recv(4096)
                    if not stueck:
                        break
                    puffer += stueck
                    # mpv schickt auch unaufgefordert Ereignisse -- die
                    # Antwort ist die Zeile mit unserer request_id.
                    for zeile in puffer.split(b"\n"):
                        try:
                            antwort = json.loads(zeile)
                        except ValueError:
                            continue
                        if antwort.get("request_id") == 1:
                            return antwort.get("data")
        except (OSError, ValueError):
            return None
        return None
