"""Crash capture -- records what survives a crash.

On 29.10. the report was "the robot crashes randomly", also on the bench power
supply and with the other board. Remotely there was no way to tell *what*
crashes:

- the server (Python error, memory full, segfault in the Hailo driver,
  SSH window closed),
- the whole Pi (power gone, reset, kernel hangs),
- or just a hang where the server is still running but not responding.

This module therefore records into ``config.DIAG_FOLDER``:

- ``laeufe.json``: every server start with boot ID, last sign of life and
  how it ended. If the end is missing, the run crashed; the boot ID
  reveals whether the Pi kept running.
- ``server.log``: the server log as a file, so it is still there after a
  crash (the terminal output is gone by then).
- ``absturz.txt``: stack dump from ``faulthandler`` on a segfault.
- ``blockiert.txt``: what the event loop was stuck on when it stalled.

Everything here is "best effort": if something is missing (Windows, no
journalctl, folder not writable), the rest still works.
"""

from __future__ import annotations

import json
import logging
import logging.handlers
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from . import config

log = logging.getLogger(__name__)


def boot_id() -> str | None:
    """ID of the current Pi boot. Changes on every reboot."""
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip() or None
    except OSError:
        return None


def journal_persistent() -> bool:
    """Whether the journal survives reboots (otherwise the kernel log from before is gone).

    It is not enough that /var/log/journal exists: on the Pi it only held a
    folder of an old machine-id, the current journal ran volatile in RAM.
    """
    try:
        mid = Path("/etc/machine-id").read_text().strip()
    except OSError:
        return False
    return bool(mid) and (Path("/var/log/journal") / mid).is_dir()


def last_lines(file: Path, n: int) -> list[str]:
    try:
        with open(file, encoding="utf-8", errors="replace") as f:
            return [z.rstrip("\n") for z in f.readlines()[-n:]]
    except OSError:
        return []


def journal(*args: str, n: int = 40) -> list[str]:
    """Kernel log via journalctl. Empty list if that is not possible."""
    if shutil.which("journalctl") is None:
        return []
    try:
        out = subprocess.run(
            ["journalctl", "-k", "--no-pager", "-o", "short-iso", "-n", str(n), *args],
            capture_output=True, text=True, timeout=5,
        )
    except (subprocess.SubprocessError, OSError):
        return []
    return [z for z in out.stdout.splitlines() if z and not z.startswith("-- ")]


def memory() -> dict[str, Any]:
    """Memory, threads and open files of the server (Linux only)."""
    values: dict[str, Any] = {}
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                values["server_mb"] = round(int(line.split()[1]) / 1024)
            elif line.startswith("Threads:"):
                values["threads"] = int(line.split()[1])
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                values["frei_mb"] = round(int(line.split()[1]) / 1024)
            elif line.startswith("MemTotal:"):
                values["gesamt_mb"] = round(int(line.split()[1]) / 1024)
            elif line.startswith("SwapFree:"):
                values["swap_frei_mb"] = round(int(line.split()[1]) / 1024)
        values["dateien"] = len(os.listdir("/proc/self/fd"))
    except (OSError, ValueError, IndexError):
        pass
    return values


def setup_logfile(file: Path) -> None:
    """Attaches a rotating log file to the root logger -- only once.

    On the root logger so that uvicorn and libraries end up in it too.
    """
    root = logging.getLogger()
    if any(getattr(h, "_rosmaster_file", False) for h in root.handlers):
        return
    h = logging.handlers.RotatingFileHandler(
        file, maxBytes=config.DIAG_LOGFILE_BYTES, backupCount=2, encoding="utf-8",
    )
    h._rosmaster_file = True  # type: ignore[attr-defined]
    h.setLevel(logging.INFO)
    h.setFormatter(logging.Formatter(
        "%(asctime)s  %(levelname)-7s %(name)-28s %(message)s", "%Y-%m-%d %H:%M:%S",
    ))
    root.addHandler(h)
    # Without basicConfig (e.g. in tests) the root logger would be at WARNING.
    if root.level > logging.INFO:
        root.setLevel(logging.INFO)


class RunLog:
    """The most recent server runs in a small JSON file.

    Each entry: pid, boot, start, zuletzt (last sign of life), ende
    (reason) -- ``ende`` stays empty if the process did not end cleanly.
    """

    def __init__(self, file: Path) -> None:
        self.file = file
        self._runs: list[dict] = []
        self._own_run: dict | None = None

    def _load(self) -> list[dict]:
        try:
            data = json.loads(self.file.read_text(encoding="utf-8"))
            return [d for d in data if isinstance(d, dict)] if isinstance(data, list) else []
        except (OSError, ValueError):
            return []

    def _write(self) -> None:
        # First into a side file, then rename: if power fails in the middle of
        # writing, the old file stays intact.
        tmp = self.file.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(self._runs, indent=1), encoding="utf-8")
            os.replace(tmp, self.file)
        except OSError:
            log.debug("Run log not writable", exc_info=True)

    def begin(self) -> dict | None:
        """Records this run. Returns the previous run (or None)."""
        self._runs = self._load()
        previous = self._runs[-1] if self._runs else None
        now = time.time()
        self._own_run = {
            "pid": os.getpid(), "boot": boot_id(), "start": now,
            "zuletzt": now, "ende": None,
        }
        self._runs = (self._runs + [self._own_run])[-config.DIAG_RUNS:]
        self._write()
        return dict(previous) if previous else None

    def heartbeat(self) -> None:
        if self._own_run is not None and self._own_run["ende"] is None:
            self._own_run["zuletzt"] = time.time()
            self._write()

    def shutdown(self, reason: str) -> None:
        # The first reason counts: SIGHUP is followed by the normal shutdown,
        # which must not overwrite "SIGHUP" with "beendet".
        if self._own_run is not None and self._own_run["ende"] is None:
            self._own_run["ende"] = reason
            self._own_run["zuletzt"] = time.time()
            self._write()

    def entries(self) -> list[dict]:
        return [dict(d) for d in self._runs][::-1]


def categorize(previous: dict, boot_now: str | None, kernel: list[str],
              segfault: bool) -> dict[str, Any]:
    """Turns the previous, uncleanly ended run into an explanation."""
    timestamp = time.strftime("%d.%m. %H:%M:%S", time.localtime(previous.get("zuletzt") or 0))
    kernel_text = "\n".join(kernel).lower()
    if previous.get("boot") and boot_now and previous["boot"] != boot_now:
        return {
            "art": "pi_neustart",
            "titel": "Der ganze Pi war aus oder ist neu gestartet",
            "was": f"Der letzte Serverlauf hörte um {timestamp} mitten im Betrieb auf, und "
                   "seitdem ist der Pi neu gestartet. Der Server selbst ist also nicht "
                   "abgestürzt -- der Pi hatte keinen Strom, wurde zurückgesetzt oder "
                   "der Kernel hing. Das ist ein Hardware- bzw. Stromproblem, kein "
                   "Fehler im Python-Code.",
            "tun": [
                "Wurde er absichtlich aus- oder abgesteckt? Dann ist alles gut.",
                "Netzteil: Reicht der Strom (Strombegrenzung/CC-Anzeige am Labornetzteil)? "
                "Beim Anlaufen von Motoren, Lidar und Hailo gibt es Spitzen.",
                "Kabel und Stecker der 5-V-Versorgung zum Pi prüfen (Wackelkontakt).",
                "Kernel-Log vom Lauf davor unten ansehen (nur bei dauerhaftem Journal).",
                "`journalctl --list-boots` zeigt die letzten Starts (nur bei dauerhaftem Journal).",
            ],
        }
    if segfault or "segfault" in kernel_text:
        return {
            "art": "segfault",
            "titel": "Der Server ist hart abgestürzt (Speicherzugriffsfehler)",
            "was": f"Der Server endete um {timestamp} durch einen Segfault -- ein Fehler in "
                   "einer C-Bibliothek (Hailo, OpenCV, Kamera-/Lidar-Treiber), nicht im "
                   "Python-Code. Der Stack-Dump unten zeigt, in welchem Teil es passierte.",
            "tun": ["Stack-Dump unten ansehen: Welche Datei steht ganz oben?",
                    "Bericht kopieren und ans Team geben."],
        }
    if "out of memory" in kernel_text or "killed process" in kernel_text or "oom" in kernel_text:
        return {
            "art": "speicher",
            "titel": "Der Kernel hat den Server wegen Speichermangel beendet",
            "was": f"Um {timestamp} war der Arbeitsspeicher voll, und der Kernel hat den "
                   "größten Prozess beendet -- den Server.",
            "tun": ["Speicherverlauf unter „Abstürze & Hänger“ beobachten: wächst der Server?",
                    "Andere große Programme auf dem Pi beenden."],
        }
    return {
        "art": "prozess_weg",
        "titel": "Der Server wurde ohne Herunterfahren beendet",
        "was": f"Der letzte Serverlauf hörte um {timestamp} auf, der Pi lief dabei weiter. "
               "Kein Python-Fehler (der stünde im Log), kein Segfault, kein "
               "Speichermangel im Kernel-Log. Bleibt: hart beendet (`kill -9`, "
               "`pkill`, ein zweiter Start), oder das Terminal/SSH ging zu, als "
               "es diese Erfassung noch nicht gab.",
        "tun": ["Server in `tmux` starten, dann überlebt er ein geschlossenes SSH-Fenster.",
                "Letzte Log-Zeilen unten ansehen: Was passierte direkt davor?"],
    }
