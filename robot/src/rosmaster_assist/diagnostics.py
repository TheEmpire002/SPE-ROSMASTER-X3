"""Diagnostics -- collects everything that can go wrong on the X3 in one place.

Until now, every dropout meant looking via SSH into the kernel log,
``vcgencmd`` and the server output and piecing the timestamps together by
hand. This module does that continuously in the background and turns it
into *problems* in everyday language: what happened, what it means, what
to do. The ``/diagnose`` page shows them.

Two kinds of problems:

- **States** persist as long as the condition holds (battery low, lidar
  gone, Pi too hot).
- **Incidents** are one-off events (USB overcurrent, disconnect). They
  stay visible for ``config.PROBLEM_WINDOW_S``.

Whoever has seen a problem acknowledges it. If it occurs again afterwards,
it reappears -- so nothing gets lost without known issues glowing red
all the time.

Everything here is "best effort": if a source is missing (Windows, no
``vcgencmd``, kernel log not readable, simulation), it simply says
"unknown" -- diagnostics must never crash, since it runs precisely when
something else is broken.
"""

from __future__ import annotations

import faulthandler
import glob
import logging
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import crash, config

log = logging.getLogger(__name__)

#: Ranking for sorting and the overall state.
_RANK = {"kritisch": 0, "warnung": 1, "hinweis": 2}

#: Processes that create heavy load on the Pi. On 28.10. a ``dpkg``/``dkms``
#: build hung, heated the Pi to 83 °C and triggered one USB overcurrent after
#: another. /proc/<pid>/comm is truncated to 15 characters, hence
#: "unattended-upgr".
_BUILD_PROCESSES = {"cc1", "cc1plus", "dkms", "dpkg", "apt", "apt-get", "make", "unattended-upgr"}

#: Bits of ``vcgencmd get_throttled`` (0-3 now, 16-19 since boot).
_THROTTLE_BITS = {
    0: "Unterspannung",
    1: "Takt begrenzt",
    2: "gedrosselt",
    3: "Temperaturgrenze",
}

_USB_TEXTS = [
    "Stromversorgung prüfen: Akku laden (unter etwa 11,5 V passiert das deutlich "
    "häufiger) bzw. Netzteil und Kabel kontrollieren.",
    "Orbbec-Kamera abziehen, wenn sie nicht gebraucht wird; sie zieht am meisten Strom.",
    "Dauerhafte Lösung: USB-Hub mit eigener Stromversorgung bzw. eigener 5-V-Wandler "
    "für den Pi (siehe docs/hardware.md).",
]


# --- Log buffer ------------------------------------------------------------------


class LogBuffer(logging.Handler):
    """Keeps the server's latest log lines for the diagnostics page.

    A normal handler on the "rosmaster_assist" logger: the console still
    gets everything, the page additionally. Every line has a sequential
    number so the browser only fetches what is new.
    """

    def __init__(self, size: int) -> None:
        super().__init__(level=logging.INFO)
        self._lines: deque[dict] = deque(maxlen=size)
        self._next_no = 1
        self._error = 0
        self._latest_error: dict | None = None
        self._buffer_lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = record.getMessage()
            if record.exc_info and record.exc_info[1] is not None:
                exc = record.exc_info[1]
                text += f" -- {type(exc).__name__}: {exc}"
            with self._buffer_lock:
                line = {
                    "id": self._next_no,
                    "zeit": record.created,
                    "stufe": record.levelname,
                    "quelle": record.name.removeprefix("rosmaster_assist."),
                    "text": text,
                }
                self._next_no += 1
                self._lines.append(line)
                if record.levelno >= logging.ERROR:
                    self._error += 1
                    self._latest_error = line
        except Exception:
            # An error while logging must never hit the caller.
            self.handleError(record)

    def lines(self, after: int = 0) -> tuple[list[dict], int]:
        """Lines after number ``after`` and the number of the last line.

        The last number always comes from the buffer, never from the caller:
        if it is smaller than what the browser already has, the server was
        restarted and the browser starts over.
        """
        with self._buffer_lock:
            return [z for z in self._lines if z["id"] > after], self._next_no - 1

    def error(self) -> tuple[int, dict | None]:
        with self._buffer_lock:
            return self._error, self._latest_error


# --- Problems --------------------------------------------------------------------


@dataclass
class Problem:
    id: str
    severity: str
    title: str
    what: str
    actions: list[str]
    since: float
    last: float
    count: int = 1
    #: State that currently still exists (never for incidents).
    exists: bool = False
    #: True for conditions (battery, lidar ...), False for one-off incidents.
    state: bool = False
    #: Time of the last *new* occurrence -- basis for acknowledging.
    incident: float = 0.0
    acknowledged: float | None = None
    extra: dict = field(default_factory=dict)

    def as_dict(self, now: float) -> dict:
        return {
            "id": self.id,
            "schwere": self.severity,
            "titel": self.title,
            "was": self.what,
            "tun": self.actions,
            "seit": self.since,
            "zuletzt": self.last,
            "anzahl": self.count,
            "besteht": self.exists,
            "zustand": self.state,
            "quittiert": self.is_acknowledged(),
            "vor_s": round(now - self.last, 1),
        }

    def is_acknowledged(self) -> bool:
        return self.acknowledged is not None and self.incident <= self.acknowledged


# --- System values (all return None if not available) -----------------------------


def _read_file(path: str) -> str | None:
    try:
        with open(path, encoding="ascii", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return None


def usb_overcurrent_count() -> int | None:
    """Sum of the overcurrent counters of all USB ports since boot.

    Reads without root rights. On the Pi 5 all four sockets hang on one
    shared 1.6 A switch -- every overcurrent resets everything.
    """
    files = glob.glob("/sys/bus/usb/devices/*-0:1.0/usb*-port*/over_current_count")
    if not files:
        return None
    total = 0
    for path in files:
        value = _read_file(path)
        if value and value.isdigit():
            total += int(value)
    return total


def usb_overcurrent_ports() -> dict[str, int]:
    """Overcurrent counter per Pi socket, e.g. {"usb1-port1": 187}.

    If *all* sockets rise at the same time, it is not one device but the
    shared power switch (finding from 29.10.).
    """
    values = {}
    for path in glob.glob("/sys/bus/usb/devices/*-0:1.0/usb*-port*/over_current_count"):
        value = _read_file(path)
        if value and value.isdigit():
            values[os.path.basename(os.path.dirname(path))] = int(value)
    return values


def cpu_temperature() -> float | None:
    value = _read_file("/sys/class/thermal/thermal_zone0/temp")
    try:
        return round(int(value) / 1000.0, 1) if value else None
    except ValueError:
        return None


def cpu_max_mhz() -> int | None:
    value = _read_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq")
    try:
        return int(value) // 1000 if value else None
    except ValueError:
        return None


def cpu_actual_mhz() -> int | None:
    value = _read_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    try:
        return int(value) // 1000 if value else None
    except ValueError:
        return None


def pi_uptime_s() -> float | None:
    value = _read_file("/proc/uptime")
    try:
        return float(value.split()[0]) if value else None
    except (ValueError, IndexError):
        return None


def throttling() -> int | None:
    """Raw value of ``vcgencmd get_throttled`` or None."""
    if shutil.which("vcgencmd") is None:
        return None
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2
        )
        return int(out.stdout.strip().split("=", 1)[-1], 16)
    except (subprocess.SubprocessError, OSError, ValueError):
        return None


def throttle_text(raw: int | None) -> dict | None:
    if raw is None:
        return None
    return {
        "roh": hex(raw),
        "jetzt": [t for b, t in _THROTTLE_BITS.items() if raw & (1 << b)],
        "seit_start": [t for b, t in _THROTTLE_BITS.items() if raw & (1 << (b + 16))],
    }


def build_processes() -> list[str]:
    """Names of running build/installation processes (empty if none)."""
    found = set()
    for path in glob.glob("/proc/[0-9]*/comm"):
        name = _read_file(path)
        if name in _BUILD_PROCESSES:
            found.add(name)
    return sorted(found)


def battery_percent(volt: float | None) -> int | None:
    """Rough charge estimate for the 3S battery (10.5 V empty, 12.6 V full).

    Only a ballpark figure: the voltage sags under load and recovers at
    rest -- that is why the page always shows the volts as well.
    """
    if volt is None:
        return None
    return int(max(0.0, min(1.0, (volt - 10.5) / (12.6 - 10.5))) * 100)


# --- Diagnostics -----------------------------------------------------------------


class Diagnostics:
    """Background thread that maintains readings, events and problems."""

    def __init__(
        self,
        driver,
        watchdog,
        lidar=None,
        cameras: dict | None = None,
        pose=None,
        log_buffer: LogBuffer | None = None,
        mock: bool = False,
    ) -> None:
        self._driver = driver
        self._watchdog = watchdog
        self._lidar = lidar
        self._cameras = cameras or {}
        self._pose = pose
        self._log = log_buffer
        self._mock = mock

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._started = time.time()

        self._problems: dict[str, Problem] = {}
        self._events: deque[dict] = deque(maxlen=config.DIAG_EVENTS)
        self._event_no = 1
        self._battery: deque[tuple[float, float]] = deque(maxlen=config.DIAG_BATTERY_POINTS)
        self._measurements: dict[str, Any] = {}

        # Reference values from the last pass, to detect changes.
        self._oc_prev: int | None = None
        self._oc_start: int | None = None
        self._was_connected: bool | None = None
        self._lidar_new_prev: int | None = None
        self._camera_new_prev: dict[str, int] = {}
        self._errors_prev = 0
        self._throttle_prev = 0
        self._last_driven = 0.0
        self._last_turned = 0.0
        self._last_voltage_drop = 0.0
        self._kernel_since: float | None = None
        self._kernel_source = "unbekannt"
        self._last_slow = 0.0
        self._start_mono = time.monotonic()

        # USB storm: (time, new overcurrents) of the last minute and the
        # counters per socket, to detect "all at the same time".
        self._oc_history: deque[tuple[float, int]] = deque(maxlen=120)
        self._oc_ports_prev: dict[str, int] = {}
        self._oc_ports_new: dict[str, int] = {}
        self._last_enum_warning: dict[str, float] = {}

        # Crash capture (see crash.py). Files are only created by
        # setup_crash_handling() -- tests that only build Diagnostics() write nothing.
        self._folder: Path | None = None
        self._runs: crash.RunLog | None = None
        self._boot = crash.boot_id()
        self._journal_persistent = crash.journal_persistent()
        self._previous_info: dict | None = None
        self._log_prev: list[str] = []
        self._dump_file = None
        self._finished = False
        self._memory: dict[str, Any] = {}
        self._memory_history: deque[tuple[float, int]] = deque(maxlen=360)
        self._memory_base: int | None = None
        # Pulse of the event loop (pulse() from the server).
        self._pulse_mono: float | None = None
        self._pulse_thread: int | None = None
        self._pulse_max = 0.0
        self._hangs = 0
        self._blocked_reported = False
        self._blocked: dict | None = None

    # --- Lifecycle -----------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._start_mono = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="diagnose", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # The slow pass waits at most for journalctl (timeout 3 s).
            self._thread.join(timeout=4.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._fast_check()
                if time.monotonic() - self._last_slow >= config.DIAG_SLOW_S:
                    self._last_slow = time.monotonic()
                    self._slow()
            except Exception:
                # Only a warning, not ERROR: otherwise diagnostics would report its
                # own crash as "error in the log" -- on every pass.
                log.warning("Diagnostics pass failed", exc_info=True)
            self._stop.wait(config.DIAG_INTERVAL_S)

    # --- Crash capture --------------------------------------------------------------

    def setup_crash_handling(self) -> None:
        """Defines what survives a crash and classifies the previous run.

        Call once at server start from the main thread (signals).
        """
        folder = config.DIAG_FOLDER
        if self._mock:
            # Own folder: a simulation run on the Pi should not mess up the
            # log of the real runs.
            folder = folder.with_name(folder.name + "_mock")
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            log.warning("Diagnostics folder %s cannot be created -- no crash capture", folder)
            return
        self._folder = folder

        # Remember the end of the old log before the new log file appends.
        self._log_prev = crash.last_lines(folder / "server.log", 40)
        try:
            crash.setup_logfile(folder / "server.log")
        except OSError:
            log.warning("Server log cannot be written as a file", exc_info=True)

        # A non-empty dump comes from a segfault in the last run.
        dump = folder / "absturz.txt"
        segfault = False
        try:
            if dump.exists() and dump.stat().st_size > 0:
                segfault = True
                os.replace(dump, folder / "absturz_vorher.txt")
        except OSError:
            pass
        try:
            if self._dump_file is None:
                self._dump_file = open(dump, "w", encoding="utf-8")
            faulthandler.enable(self._dump_file, all_threads=True)
        except (OSError, ValueError, RuntimeError):
            log.debug("faulthandler cannot be enabled", exc_info=True)

        self._runs = crash.RunLog(folder / "laeufe.json")
        previous = self._runs.begin()
        if previous is not None and not previous.get("ende"):
            self._classify_previous(previous, segfault)
        self._setup_sighup()
        threading.excepthook = self._thread_error

    def _classify_previous(self, previous: dict, segfault: bool) -> None:
        until = float(previous.get("zuletzt") or 0)
        kernel: list[str] = []
        if previous.get("boot") and previous.get("boot") != self._boot:
            # Different boot: the kernel log from back then only exists if persistent.
            if self._journal_persistent:
                kernel = crash.journal("-b", "-1")
        else:
            kernel = crash.journal(f"--since=@{until - 120:.0f}", f"--until=@{until + 120:.0f}")
        info = crash.categorize(previous, self._boot, kernel, segfault)
        info["zeit"] = until
        info["kernel"] = kernel[-40:]
        info["log"] = self._log_prev
        info["dump"] = (crash.last_lines(self._folder / "absturz_vorher.txt", 60)
                        if segfault and self._folder else [])
        self._previous_info = info

        actions = list(info["tun"])
        if not self._journal_persistent:
            actions.append("Journal dauerhaft machen, damit das Kernel-Log einen Neustart überlebt: "
                       "`sudo mkdir -p /var/log/journal/$(cat /etc/machine-id) && "
                       "sudo systemd-tmpfiles --create --prefix /var/log/journal && "
                       "sudo systemctl restart systemd-journald`")
        severity = "warnung" if info["art"] == "prozess_weg" else "kritisch"
        self._event("absturz", severity, f"Voriger Lauf: {info['titel']}", until or None)
        self._incident("absturz_vorher", severity, info["titel"], info["was"], actions)

    def _setup_sighup(self) -> None:
        """Closed SSH window: shut down cleanly instead of dying immediately.

        Without a handler SIGHUP kills the process hard -- without stopping
        the motors and without a trace. This turns it into a normal shutdown
        (SIGTERM, which uvicorn handles cleanly), and the run log names the reason.
        """
        if not hasattr(signal, "SIGHUP") or threading.current_thread() is not threading.main_thread():
            return
        try:
            if signal.getsignal(signal.SIGHUP) == signal.SIG_IGN:
                return  # nohup/tmux: SIGHUP should stay ignored
            signal.signal(signal.SIGHUP, self._on_sighup)
        except (ValueError, OSError):
            pass

    def _on_sighup(self, signum, frame) -> None:
        log.warning("SIGHUP: terminal/SSH window closed -- server shutting down. "
                    "Tip: start it in tmux.")
        if self._runs is not None:
            self._runs.shutdown("SIGHUP (Terminal/SSH geschlossen)")
        os.kill(os.getpid(), signal.SIGTERM)

    def _thread_error(self, args) -> None:
        if args.exc_type is SystemExit:
            return
        name = args.thread.name if args.thread else "?"
        log.error("Unhandled error in thread %s", name,
                  exc_info=(args.exc_type, args.exc_value, args.exc_traceback))
        self._event("absturz", "kritisch", f"Thread „{name}“ abgestürzt: {args.exc_value!r}"[:200])

    def watch_task(self, task, name: str, endless: bool = True) -> None:
        """Reports when a background task of the server ends unexpectedly.

        Previously e.g. the status loop died silently -- the app then showed
        frozen values, and it looked like a crash.
        """
        def done(t) -> None:
            if t.cancelled() or self._finished:
                return
            exc = t.exception()
            if exc is None and not endless:
                return  # may finish (e.g. calibration at startup)
            if exc is not None:
                log.error("Background task %s crashed", name,
                          exc_info=(type(exc), exc, exc.__traceback__))
                text = f"{type(exc).__name__}: {exc}"[:200]
            else:
                log.error("Background task %s ended unexpectedly", name)
                text = "ohne Fehler beendet"
            self._event("absturz", "kritisch", f"Aufgabe „{name}“ beendet ({text})")
            self._incident(
                f"task_{name}", "kritisch", f"Server-Teil „{name}“ läuft nicht mehr",
                f"Die Hintergrundaufgabe „{name}“ ist beendet ({text}). Das ist ein "
                "Fehler im Code; der Rest des Servers läuft weiter.",
                ["Server neu starten.", "Fehlertext aus dem Log ans Team geben."],
            )
        task.add_done_callback(done)

    def loop_error(self, loop, context: dict) -> None:
        """Exception handler of the event loop: into the log and the timeline."""
        loop.default_exception_handler(context)
        exc = context.get("exception")
        text = context.get("message", "")
        if exc is not None:
            text += f": {type(exc).__name__}: {exc}"
        self._event("absturz", "warnung", f"Fehler in der Ereignisschleife: {text}"[:200])

    def pulse(self, delay: float) -> None:
        """Sign of life of the event loop, called every DIAG_PULSE_S."""
        self._pulse_mono = time.monotonic()
        self._pulse_thread = threading.get_ident()
        self._pulse_max = max(self._pulse_max, delay)
        if self._blocked_reported:
            self._blocked_reported = False
            self._event("haenger", "warnung", "Ereignisschleife läuft wieder")
        if delay > config.DIAG_HANG_S:
            self._hangs += 1
            self._event("haenger", "warnung",
                           f"Server hing {delay:.1f} s (reagierte in der Zeit nicht)")
            self._incident(
                "haenger", "warnung", "Server reagierte zeitweise nicht",
                f"Die Ereignisschleife kam {delay:.1f} s zu spät. Solange reagiert "
                "der Server auf nichts -- auch nicht auf den Not-Stopp (der Watchdog "
                "hält die Motoren trotzdem an).",
                ["Auf hohe CPU-Last oder Installation achten (Messwerte).",
                 "Häufen sich die Hänger, Log ans Team geben."],
            )

    def pulse_off(self) -> None:
        self._pulse_mono = None

    def mark_clean_exit(self, reason: str = "beendet") -> None:
        self._finished = True
        self.pulse_off()
        if self._runs is not None:
            self._runs.shutdown(reason)

    def _check_blockage(self, mono: float) -> None:
        """If the event loop is stuck, write down where it hangs."""
        if self._pulse_mono is None or self._blocked_reported:
            return
        duration = mono - self._pulse_mono
        if duration < config.DIAG_BLOCKED_S:
            return
        self._blocked_reported = True
        frame = sys._current_frames().get(self._pulse_thread or -1)
        stack = traceback.format_stack(frame) if frame is not None else ["(nicht lesbar)"]
        self._blocked = {"zeit": time.time(), "dauer_s": round(duration, 1),
                           "stapel": "".join(stack[-12:])}
        log.error("Event loop stuck for %.1f s, it hangs here:\n%s",
                  duration, self._blocked["stapel"])
        if self._folder is not None:
            try:
                (self._folder / "blockiert.txt").write_text(
                    time.strftime("%Y-%m-%d %H:%M:%S") + f"  steht seit {duration:.1f} s\n"
                    + "".join(stack), encoding="utf-8")
            except OSError:
                pass
        self._event("haenger", "kritisch", f"Server steht seit {duration:.0f} s")
        self._incident(
            "blockiert", "kritisch", "Server steht (Ereignisschleife blockiert)",
            f"Seit {duration:.0f} s kommt die Ereignisschleife nicht mehr dran. Von außen "
            "sieht das aus wie ein Absturz: Die App bekommt keine Werte mehr. Wo sie "
            "hängt, steht unter „Abstürze & Hänger“ (Stapel) und in blockiert.txt.",
            ["Stapel ansehen: Welche Datei steht ganz unten?",
             "Bericht kopieren und ans Team geben."],
        )

    def _check_memory(self) -> None:
        values = crash.memory()
        self._memory = values
        mb = values.get("server_mb")
        if mb is None:
            return
        self._memory_history.append((round(time.time(), 1), mb))
        if self._in_startup():
            return
        if self._memory_base is None:
            self._memory_base = mb
        growth = mb - self._memory_base
        self._state(
            "speicher_wachstum", growth > config.DIAG_MEMORY_WARN_MB, "warnung",
            f"Server-Speicher wächst (+{growth} MB)",
            "Der Server belegt deutlich mehr Arbeitsspeicher als nach dem Start. "
            "Wächst er weiter, beendet ihn irgendwann der Kernel.",
            ["Verlauf unter „Abstürze & Hänger“ beobachten.", "Log und Werte ans Team geben."],
        )
        free = values.get("frei_mb")
        self._state(
            "speicher_knapp", free is not None and free < config.DIAG_MEMORY_WARN_MB,
            "kritisch", f"Arbeitsspeicher fast voll ({free} MB frei)",
            "Ist der Speicher voll, beendet der Kernel den größten Prozess -- meist den Server.",
            ["Andere große Programme auf dem Pi beenden (`top`)."],
        )
        files = values.get("dateien") or 0
        self._state(
            "dateien", files > config.DIAG_FILES_WARN, "warnung",
            f"Sehr viele offene Dateien ({files})",
            "Der Server hält ungewöhnlich viele Dateien/Geräte offen -- vermutlich ein "
            "Leck beim Neuverbinden. Irgendwann scheitert dann jedes Öffnen.",
            ["`ls -l /proc/$(pgrep -f rosmaster_assist)/fd` ansehen.", "Server neu starten."],
        )

    def _crash_status(self) -> dict:
        now = time.monotonic()
        return {
            "aktiv": self._folder is not None,
            "ordner": str(self._folder) if self._folder else None,
            "boot": self._boot,
            "journal_dauerhaft": self._journal_persistent,
            "vorher": self._previous_info,
            "laeufe": self._runs.entries() if self._runs else [],
            "haenger": self._hangs,
            "puls_max_s": round(self._pulse_max, 2),
            "puls_alter_s": round(now - self._pulse_mono, 1) if self._pulse_mono else None,
            "blockiert": self._blocked,
            "speicher": self._memory,
            "speicher_verlauf": [list(p) for p in list(self._memory_history)[::6]],
            "usb_ports": dict(self._oc_ports_prev),
            "usb_ports_neu": dict(self._oc_ports_new),
            "usb_pro_min": self._measurements.get("usb_ueberlast_pro_min"),
        }

    # --- Maintaining problems -------------------------------------------------------

    def _incident(
        self, pid: str, severity: str, title: str, what: str, actions: list[str],
        timestamp: float | None = None, count: int = 1,
    ) -> None:
        """One-off event -- counts up and makes the problem visible again."""
        timestamp = timestamp or time.time()
        with self._lock:
            p = self._problems.get(pid)
            if p is None or timestamp - p.last > config.PROBLEM_WINDOW_S:
                self._problems[pid] = Problem(
                    pid, severity, title, what, actions, since=timestamp, last=timestamp,
                    count=count, incident=timestamp, acknowledged=p.acknowledged if p else None,
                )
                return
            p.severity, p.title, p.what, p.actions = severity, title, what, actions
            p.last = max(p.last, timestamp)
            p.incident = max(p.incident, timestamp)
            p.count += count

    def _state(
        self, pid: str, active: bool, severity: str = "warnung", title: str = "",
        what: str = "", actions: list[str] | None = None,
    ) -> None:
        """Condition that may or may not exist. Only the start counts as new."""
        now = time.time()
        with self._lock:
            p = self._problems.get(pid)
            if not active:
                if p is not None:
                    p.exists = False
                return
            if p is None or (not p.exists and now - p.last > config.PROBLEM_WINDOW_S):
                self._problems[pid] = Problem(
                    pid, severity, title, what, actions or [], since=now, last=now,
                    exists=True, state=True, incident=now,
                    acknowledged=p.acknowledged if p else None,
                )
                return
            if not p.exists:
                p.count += 1
                p.incident = now
            p.exists = True
            p.last = now
            p.severity, p.title, p.what = severity, title, what
            p.actions = actions or []

    def _event(self, category: str, severity: str, text: str, timestamp: float | None = None) -> None:
        with self._lock:
            self._events.append({
                "id": self._event_no,
                "zeit": timestamp or time.time(),
                "art": category,
                "schwere": severity,
                "text": text,
            })
            self._event_no += 1
        # Into the log too: whoever only sees the console should notice it as well.
        if severity != "info":
            log.warning("Diagnostics: %s", text)

    def acknowledge(self, pid: str | None = None) -> int:
        """Marks a problem (or all with ``None``) as seen."""
        now = time.time()
        n = 0
        with self._lock:
            for p in self._problems.values():
                if pid is None or p.id == pid:
                    p.acknowledged = now
                    n += 1
        return n

    # --- Fast measurements (every second) --------------------------------------------

    def _fast_check(self) -> None:
        now = time.time()
        mono = time.monotonic()

        cmd = self._watchdog.current_command
        if abs(cmd.vx) > 0.01 or abs(cmd.vy) > 0.01 or abs(cmd.omega) > 0.01:
            self._last_driven = mono
        if abs(cmd.omega) > 0.05:
            self._last_turned = mono

        self._check_blockage(mono)
        self._check_battery(now)
        self._check_usb(mono)
        self._check_drive()
        self._check_lidar()
        self._check_cameras()
        self._check_pose()
        self._check_log()

    def _check_battery(self, now: float) -> None:
        try:
            volt = self._driver.read_battery()
        except Exception:
            volt = None
        self._measurements["akku_v"] = volt
        self._measurements["akku_prozent"] = battery_percent(volt)
        if volt is None:
            self._state("akku_kritisch", False)
            self._state("akku_schwach", False)
            return

        with self._lock:
            self._battery.append((now, volt))
            last_minute = [v for t, v in self._battery if now - t <= 60.0]
        high = max(last_minute)

        critical = volt < config.BATTERY_CRITICAL_V
        self._state(
            "akku_kritisch", critical, "kritisch", f"Akku fast leer ({volt:.1f} V)",
            "Die Spannung liegt unter der kritischen Schwelle. Das Board und die "
            "USB-Geräte melden sich jetzt ständig ab, und der Akku nimmt beim "
            "Weiterentladen Schaden.",
            ["Roboter anhalten und ausschalten.", "Akku laden (voll = 12,6 V).",
             "Zum Testen am Schreibtisch das Netzteil statt des Akkus verwenden."],
        )
        self._state(
            "akku_schwach", not critical and volt < config.BATTERY_WARN_V, "warnung",
            f"Akku schwach ({volt:.1f} V)",
            "Mit schwachem Akku bricht beim Anfahren der Motoren die Spannung ein. "
            "Dann setzen sich Board, Lidar und Kamera zurück -- der Roboter bleibt "
            "stehen oder fährt ruckelig.",
            ["Bald laden.", "Bis dahin langsam fahren und abrupte Drehungen vermeiden."],
        )

        # Drop compared with the maximum of the last minute. Report at most
        # every 10 s: a drop often spans several measurements.
        if high - volt >= config.VOLTAGE_DROP_V and now - self._last_voltage_drop > 10.0:
            self._last_voltage_drop = now
            text = f"Spannungseinbruch: {high:.1f} V → {volt:.1f} V"
            if time.monotonic() - self._last_driven < 3.0:
                text += " (beim Fahren)"
            self._event("spannung", "warnung", text)
            self._incident(
                "spannungseinbruch", "warnung", "Spannungseinbruch am Akku",
                f"Die Akkuspannung ist kurz um {high - volt:.1f} V gefallen. Meist "
                "laufen dabei die Motoren an; bei schwachem oder kaltem Akku fällt "
                "die Spannung stärker ab.",
                ["Akku laden bzw. wechseln.",
                 "Prüfen, ob gleichzeitig USB-Geräte ausgefallen sind (Zeitleiste)."],
            )

    def _check_usb(self, mono: float) -> None:
        oc = usb_overcurrent_count()
        self._measurements["usb_ueberlast_gesamt"] = oc
        if oc is None:
            return
        if self._oc_start is None:
            self._oc_start = oc
            self._oc_prev = oc
            if oc > 0:
                # Already happened before the server start, e.g. during boot.
                self._event("usb", "hinweis", f"Seit dem Booten bereits {oc} USB-Überlast(en)")
        self._measurements["usb_ueberlast_seit_serverstart"] = oc - self._oc_start
        new = oc - (self._oc_prev or 0)
        self._oc_prev = oc

        ports = usb_overcurrent_ports()
        if self._oc_ports_prev:
            for port, value in ports.items():
                increase = value - self._oc_ports_prev.get(port, value)
                if increase > 0:
                    self._oc_ports_new[port] = self._oc_ports_new.get(port, 0) + increase
        self._oc_ports_prev = ports
        self._oc_history.append((mono, max(new, 0)))
        per_min = sum(n for t, n in self._oc_history if mono - t <= 60.0)
        self._measurements["usb_ueberlast_pro_min"] = per_min
        if new <= 0:
            return

        turned = mono - self._last_turned < 5.0
        driven = mono - self._last_driven < 5.0
        if turned:
            cause = " kurz nach einer Drehung"
        elif driven:
            cause = " beim Fahren"
        else:
            cause = " im Stand"
        self._event("usb", "warnung", f"USB-Überlast (+{new}){cause}")

        what = (
            "Der Pi 5 versorgt alle vier USB-Buchsen über einen gemeinsamen "
            "1,6-A-Schalter. Zieht ein Gerät zu viel Strom oder bricht die "
            "5-V-Versorgung ein, schaltet er kurz ab -- Board, Lidar und Kamera "
            "melden sich dann ab und wieder an."
        )
        actions = list(_USB_TEXTS)
        if turned or driven:
            what += (
                " Diesmal kurz nach einer Fahrbewegung: typisch für den Anlaufstrom "
                "der Motoren, der die Versorgung kurz in die Knie zwingt."
            )
            actions.insert(1, "Sanfter anfahren (geringeres Höchsttempo in den Einstellungen).")
        else:
            actions.insert(0, "Prüfen, ob im Hintergrund etwas Rechenintensives läuft "
                          "(Installation, Build) -- siehe Messwerte.")
        self._incident("usb_ueberlast", "warnung", "USB-Überlast: Buchsen kurz abgeschaltet",
                      what, actions, count=new)

        if per_min >= config.USB_STORM_PER_MIN and not (turned or driven):
            affected = sorted(self._oc_ports_new)
            all_ports = len(affected) >= 3
            self._incident(
                "usb_sturm", "kritisch",
                f"USB-Sturm: {per_min} Überlasten pro Minute im Stand",
                f"Die USB-Buchsen des Pi schalten im Sekundentakt ab, obwohl der Roboter "
                f"steht. Betroffen: {', '.join(affected) or '?'}"
                + (" -- also alle Buchsen gleichzeitig, auch Geräte direkt am Pi. "
                   if all_ports else ". ")
                + "Das ist kein Fehler im Code und kein Anlaufstrom: Der gemeinsame "
                "Stromschalter der Pi-Buchsen löst dauernd aus. Typische Ursachen: "
                "Rückspeisung (Backfeed) vom selbstversorgten USB-Hub des Expansion "
                "Boards in die 5 V des Pi, oder ein Kurzschluss bzw. defektes Gerät/"
                "Kabel (am 29.10. meldete sich zugleich der Lidar nicht an). Board, "
                "Lidar, Kamera, Mikrofon fallen dabei immer wieder aus -- das wirkt "
                "wie „random Abstürze“.",
                ["USB-Kabel vom Board zum Pi abziehen: Hört der Sturm auf (Zähler unter "
                 "„Abstürze & Hänger“)? Dann liegt es am Board/Hub.",
                 "Geräte einzeln wieder anstecken, den Lidar zuletzt -- wann kommt der Sturm zurück?",
                 "Lidar-Kabel und -Adapter tauschen.",
                 "Hub ohne Rückspeisung verwenden bzw. die 5-V-Ader im Kabel Board→Pi trennen.",
                 "Pi und Board an gemeinsame Masse und saubere Versorgung."],
                count=new,
            )

    def _in_startup(self) -> bool:
        """Shortly after start the sensors deliver nothing yet -- no false alarm."""
        return time.monotonic() - self._start_mono < config.DIAG_STARTUP_S

    def _check_drive(self) -> None:
        connected = bool(self._driver.connected)
        if self._was_connected is not None and connected != self._was_connected:
            if connected:
                self._event("antrieb", "info", "Antrieb wieder verbunden")
            else:
                self._event("antrieb", "kritisch", "Verbindung zum Antrieb verloren")
        self._was_connected = connected
        self._state(
            "antrieb", not connected, "kritisch", "Keine Verbindung zum Antrieb",
            "Das Expansion Board (Motoren, Akku-Messung, IMU) antwortet nicht. "
            "Der Server versucht laufend, sich neu zu verbinden; solange fährt "
            "der Roboter nicht.",
            ["Ist der Roboter eingeschaltet (Schalter am Akku)?",
             "Stromversorgung prüfen (Akku geladen bzw. Netzteil an) -- häufigste Ursache.",
             "Viele USB-Überlasten in der Zeitleiste? Dann fällt das Board mit "
             "aus -- siehe „USB-Sturm“.",
             "USB-Kabel zwischen Pi und Board kontrollieren.",
             "Läuft noch ein anderes Programm am Board (ROS2-Knoten)? "
             "Siehe AGENTS.md „Am Roboter“."],
        )

    def _check_lidar(self) -> None:
        if self._lidar is None:
            return
        try:
            st = self._lidar.status()
        except Exception:
            return
        old = st.get("letzte_runde_vor_s")
        delivering = bool(st.get("geoeffnet")) and old is not None and old < 3.0
        self._measurements["lidar"] = st
        self._state(
            "lidar", not delivering and not self._in_startup(), "warnung", "Lidar liefert keine Daten",
            "Ohne Lidar gibt es keinen Hindernisstopp, kein autonomes Fahren und "
            "keine Ortung. Der Server versucht, ihn neu zu öffnen."
            + (f" Letzter Fehler: {st['letzter_fehler']}" if st.get("letzter_fehler") else ""),
            ["Dreht sich der Lidar-Kopf? Wenn nicht: Stromversorgung/USB prüfen.",
             "Läuft noch ein sllidar_node? `sudo pkill -f sllidar_node`.",
             "Kabel am Lidar-Adapter kontrollieren."],
        )
        new = int(st.get("neu_verbunden") or 0)
        if self._lidar_new_prev is not None and new > self._lidar_new_prev:
            self._event("lidar", "warnung", "Lidar hat sich neu verbunden")
            self._incident(
                "lidar_neu", "hinweis", "Lidar musste neu verbunden werden",
                "Der Lidar war kurz weg und wurde automatisch neu geöffnet. Das "
                "passiert fast immer zusammen mit einer USB-Überlast.",
                ["Zeitleiste ansehen: kam gleichzeitig eine USB-Überlast?",
                 "Wenn ja: siehe Problem „USB-Überlast“."],
                count=new - self._lidar_new_prev,
            )
        self._lidar_new_prev = new

    def _check_cameras(self) -> None:
        cameras = {}
        for name, camera in self._cameras.items():
            try:
                st = camera.status()
            except Exception:
                continue
            cameras[name] = st
            old = st.get("letztes_bild_vor_s")
            delivering = bool(st.get("geoeffnet")) and old is not None and old < 5.0
            self._state(
                f"kamera_{name}", not delivering and not self._in_startup(), "warnung",
                f"Kamera „{st.get('kamera') or name}“ liefert kein Bild",
                "Die Kamera ist nicht geöffnet oder schickt keine Bilder. Der Server "
                "versucht es weiter."
                + (f" Letzter Fehler: {st['letzter_fehler']}" if st.get("letzter_fehler") else ""),
                ["USB-Stecker der Kamera prüfen.",
                 "Wurde die Kamera absichtlich abgezogen (z. B. Orbbec zum Stromsparen)? "
                 "Dann kann diese Meldung quittiert werden.",
                 "Gerätepfad in den Einstellungen bzw. config.py (CAMERA_ROLES) prüfen."],
            )
            new = int(st.get("neu_geoeffnet") or 0)
            previous = self._camera_new_prev.get(name)
            if previous is not None and new > previous:
                self._event("kamera", "warnung", f"Kamera „{name}“ neu geöffnet")
                self._incident(
                    f"kamera_neu_{name}", "hinweis", f"Kamera „{name}“ musste neu geöffnet werden",
                    "Die Kamera war kurz weg (meist USB-Rücksetzer) und läuft wieder.",
                    ["Zeitleiste ansehen: kam gleichzeitig eine USB-Überlast?"],
                    count=new - previous,
                )
            self._camera_new_prev[name] = new
        self._measurements["kameras"] = cameras

    def _check_pose(self) -> None:
        if self._pose is None:
            return
        try:
            st = self._pose.status()
        except Exception:
            return
        self._measurements["pose"] = st
        error = st.get("fehler")
        self._state(
            "pose", bool(error) and bool(st.get("aktiv")), "hinweis",
            "Körperposen-Erkennung (Hailo) hat ein Problem",
            f"Die Gestenerkennung läuft nicht: {error}",
            ["Läuft ein anderes Programm auf dem Hailo (hailo-apps)? Beenden.",
             "`ls /dev/hailo0` -- fehlt das Gerät, den Pi neu starten."],
        )

    def _check_log(self) -> None:
        if self._log is None:
            return
        count, last = self._log.error()
        new = count - self._errors_prev
        self._errors_prev = count
        if new > 0 and last is not None:
            self._incident(
                "log_fehler", "hinweis", "Fehler im Server-Log",
                f"Der Server hat einen Fehler protokolliert ({last['quelle']}): "
                f"{last['text']}",
                ["Im Server-Log unten nach Zeilen der Stufe ERROR filtern.",
                 "Wiederholt sich der Fehler: Bericht kopieren und ans Team geben."],
                timestamp=last["zeit"], count=new,
            )

    # --- Slow measurements (every few seconds) --------------------------------------

    def _slow(self) -> None:
        temp = cpu_temperature()
        mhz = cpu_max_mhz()
        raw = throttling()
        build = build_processes()
        try:
            last = round(os.getloadavg()[0], 2)
        except (AttributeError, OSError):
            last = None
        self._measurements.update({
            "temp_c": temp,
            "cpu_max_mhz": mhz,
            "cpu_ist_mhz": cpu_actual_mhz(),
            "last_1min": last,
            "kerne": os.cpu_count(),
            "drosselung": throttle_text(raw),
            "bau_prozesse": build,
            "pi_laufzeit_s": pi_uptime_s(),
        })

        if temp is not None:
            self._state(
                "temperatur", temp >= config.TEMP_WARN_C, "warnung",
                f"Pi wird heiß ({temp:.0f} °C)",
                "Hohe Temperatur kommt fast immer von Dauerlast. Ab 85 °C drosselt "
                "der Pi; davor steigt schon der Strombedarf -- am X3 ein bekannter "
                "Auslöser für USB-Überlast.",
                ["Nachsehen, was rechnet (Messwerte: Installation/Build?).",
                 "Lüfter des Pi prüfen.", "Nicht benötigte Programme beenden."],
            )
        if mhz is not None:
            self._state(
                "takt", mhz > config.CPU_MAX_MHZ, "warnung",
                f"CPU-Takt {mhz} MHz (empfohlen ≤ {config.CPU_MAX_MHZ})",
                "Bei 2,4 GHz lösten Lastsprünge am X3 USB-Rücksetzer aus. In "
                "/boot/firmware/config.txt sollte `arm_freq=2000` stehen; das wirkt "
                "erst nach einem Neustart.",
                ["`grep arm_freq /boot/firmware/config.txt` prüfen.",
                 "Sofort (bis zum Neustart): `echo 2000000 | sudo tee "
                 "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq`.",
                 "Details: docs/hardware.md."],
            )
        self._state(
            "bauen", bool(build), "warnung",
            "Im Hintergrund läuft eine Installation",
            f"Gerade laufen: {', '.join(build)}. Solche Builds lasten die CPU "
            "minutenlang aus. Am 28.10. hing ein Treiber-Build endlos, heizte den "
            "Pi auf 83 °C und löste eine USB-Überlast nach der anderen aus.",
            ["Nicht fahren, bis die Installation fertig ist.",
             "Hängt sie (läuft > 10 min)? `ps aux | grep -E 'dpkg|apt|dkms'` und ggf. abbrechen."],
        )
        self._check_throttle(raw)
        self._read_kernel_log()
        self._check_memory()
        if self._runs is not None:
            self._runs.heartbeat()

    def _check_throttle(self, raw: int | None) -> None:
        if raw is None:
            return
        self._state(
            "unterspannung", bool(raw & 0x1), "kritisch", "Pi meldet Unterspannung",
            "Die 5-V-Versorgung des Pi ist gerade zu niedrig. Dann drosselt er und "
            "schaltet USB-Geräte ab -- alles, was dranhängt, wird unzuverlässig.",
            ["Akku laden bzw. Netzteil prüfen.",
             "5-V-Wandler (Board → Pi) und Kabel kontrollieren.",
             "Stromhungrige USB-Geräte (Orbbec) abziehen."],
        )
        self._state(
            "temperaturgrenze", bool(raw & 0x8), "warnung",
            "Pi drosselt wegen Temperatur",
            "Der Pi hat seine Temperaturgrenze erreicht und rechnet langsamer.",
            ["Last reduzieren, Lüfter prüfen."],
        )
        new = raw & ~self._throttle_prev & 0xF0000
        self._throttle_prev |= raw
        if new & 0x10000:
            self._event("pi", "warnung", "Der Pi hatte seit dem Start Unterspannung")
            self._incident(
                "unterspannung_frueher", "hinweis", "Seit dem Start gab es Unterspannung",
                "Irgendwann seit dem Einschalten war die 5-V-Versorgung zu niedrig "
                "(z. B. beim Anfahren mit schwachem Akku). Wann genau, verrät der "
                "Pi nicht -- die Zeitleiste hilft beim Eingrenzen.",
                ["Akku laden.", "Tritt es erneut auf, siehe „Pi meldet Unterspannung“."],
            )
        if new & 0x80000:
            self._event("pi", "warnung", "Der Pi war seit dem Start an der Temperaturgrenze")

    def _read_kernel_log(self) -> None:
        """USB disconnects and undervoltage from the kernel log.

        journalctl instead of dmesg: it delivers real clock times and can be
        queried from a point in time. For that the user must be in the group
        ``systemd-journal`` or ``adm`` -- otherwise they only see their own
        messages and the hint below appears.
        """
        if self._mock or shutil.which("journalctl") is None:
            self._kernel_source = "nicht vorhanden"
            return
        since = self._kernel_since or (time.time() - config.DIAG_KERNEL_LOOKBACK_S)
        try:
            out = subprocess.run(
                ["journalctl", "-k", "-o", "short-unix", "--no-pager",
                 f"--since=@{since:.6f}"],
                capture_output=True, text=True, timeout=3,
            )
        except (subprocess.SubprocessError, OSError):
            self._kernel_source = "Fehler beim Lesen"
            return

        message = out.stderr + out.stdout[:400]
        if "not seeing messages" in message or "No journal files" in message \
                or "insufficient permissions" in message.lower():
            if self._kernel_source != "keine Berechtigung":
                self._event("kernel", "hinweis", "Kernel-Log nicht lesbar (Berechtigung)")
            self._kernel_source = "keine Berechtigung"
            self._state(
                "kernel_log", True, "hinweis", "Kernel-Log nicht lesbar",
                "Ohne Kernel-Log sieht die Diagnose keine USB-Abmeldungen, nur die "
                "Überlast-Zähler. Dem Benutzer fehlt die Leseberechtigung.",
                ["`sudo usermod -aG systemd-journal pi`, danach neu anmelden und "
                 "den Server neu starten."],
            )
            return
        self._kernel_source = "ok"
        self._state("kernel_log", False)

        for line in out.stdout.splitlines():
            parts = line.split(None, 3)
            if len(parts) < 4:
                continue
            try:
                timestamp = float(parts[0])
            except ValueError:
                continue
            if self._kernel_since is not None and timestamp <= self._kernel_since:
                continue  # --since is inclusive
            self._kernel_since = max(self._kernel_since or 0.0, timestamp)
            text = parts[3].removeprefix("kernel: ")
            self._kernel_line(timestamp, text)
        if self._kernel_since is None:
            self._kernel_since = time.time() - 1.0

    def _kernel_line(self, timestamp: float, text: str) -> None:
        if "USB disconnect" in text:
            # "usb 3-1.2: USB disconnect, device number 7" -> "usb 3-1.2"
            port = text.split(":", 1)[0]
            self._event("usb", "warnung", f"USB-Gerät abgemeldet ({port})", timestamp)
            self._incident(
                "usb_abmeldung", "warnung", "USB-Geräte haben sich abgemeldet",
                "Laut Kernel ist mindestens ein USB-Gerät kurz verschwunden. Die "
                "Software verbindet sich selbst neu; fuhr der Roboter gerade, hat "
                "er angehalten.",
                ["Zeitleiste ansehen: gleichzeitig eine USB-Überlast? Dann gilt deren Lösung.",
                 "Kabel und Stecker am USB-Hub kontrollieren (Wackelkontakt).",
                 *_USB_TEXTS[:1]],
                timestamp=timestamp,
            )
        elif "ndervoltage" in text or "under-voltage" in text:
            self._event("pi", "kritisch", "Kernel meldet Unterspannung", timestamp)
        elif "over-current" in text and "change" not in text:
            # The counter already records the overcurrent -- this is only for the
            # timeline record, so the port number is visible.
            self._event("usb", "info", f"Kernel: {text[:120]}", timestamp)
        else:
            self._kernel_crash(timestamp, text)

    def _kernel_crash(self, timestamp: float, text: str) -> None:
        """Kernel messages that explain a crash or hang."""
        t = text.lower()
        if "out of memory" in t or "oom-kill" in t or "killed process" in t:
            self._event("absturz", "kritisch", f"Kernel: Speicher voll ({text[:100]})", timestamp)
            self._incident("kernel_oom", "kritisch", "Kernel hat wegen Speichermangel Prozesse beendet",
                          "Der Arbeitsspeicher war voll; der Kernel beendet dann den größten Prozess.",
                          ["Speicherwerte unter „Abstürze & Hänger“ ansehen."], timestamp=timestamp)
        elif "segfault" in t and ("python" in t or "uvicorn" in t):
            self._event("absturz", "kritisch", f"Kernel: Segfault ({text[:100]})", timestamp)
        elif "soft lockup" in t or "hard lockup" in t or "blocked for more than" in t \
                or ("rcu" in t and "stall" in t):
            self._event("absturz", "kritisch", f"Kernel hängt: {text[:100]}", timestamp)
            self._incident("kernel_haenger", "kritisch", "Der Kernel des Pi hing",
                          "Der Kernel meldet, dass ein Teil des Systems länger blockiert war. "
                          "Das kommt von Treibern bzw. Hardware, nicht vom Python-Code.",
                          ["Kernel-Log unten ansehen.", "Ans Team geben."], timestamp=timestamp)
        elif "hc died" in t or ("xhci" in t and "not responding" in t):
            self._event("usb", "kritisch", f"USB-Controller ausgefallen: {text[:100]}", timestamp)
            self._incident("usb_controller", "kritisch", "USB-Controller des Pi ausgefallen",
                          "Der Kernel hat einen USB-Controller aufgegeben. Bis zum Neustart "
                          "bleiben alle Geräte daran weg.",
                          ["Pi neu starten.", "Siehe „USB-Sturm“: Stromversorgung der Buchsen prüfen."],
                          timestamp=timestamp)
        elif "cannot reset" in t or "device descriptor read" in t or "unable to enumerate" in t:
            port = text.split(":", 1)[0]
            # Often comes dozens of times -- in the timeline only every 30 s per port.
            if timestamp - self._last_enum_warning.get(port, 0.0) > 30.0:
                self._last_enum_warning[port] = timestamp
                self._event("usb", "warnung", f"USB-Gerät lässt sich nicht anmelden ({port})", timestamp)
            self._incident("usb_anmeldung", "warnung", "Ein USB-Gerät lässt sich nicht anmelden",
                          f"Der Kernel bekommt von {port} keine gültige Antwort. Am 29.10. war "
                          "das der Lidar-Anschluss am Board-Hub (/dev/rplidar fehlte).",
                          ["Gerät und Kabel an diesem Port tauschen.",
                           "Tritt es mit vielen Überlasten auf, siehe „USB-Sturm“."], timestamp=timestamp)
        elif "hailo" in t and ("error" in t or "fail" in t or "timeout" in t):
            self._event("pi", "warnung", f"Hailo: {text[:110]}", timestamp)
        elif "pcie" in t and ("error" in t or "link down" in t):
            self._event("pi", "warnung", f"PCIe: {text[:110]}", timestamp)

    # --- Queries --------------------------------------------------------------------

    def status(self) -> dict:
        now = time.time()
        with self._lock:
            # Forget finished incidents once the window has passed.
            for pid in [
                pid for pid, p in self._problems.items()
                if not p.exists and now - p.last > config.PROBLEM_WINDOW_S
            ]:
                del self._problems[pid]
            problems = sorted(
                (p.as_dict(now) for p in self._problems.values()),
                key=lambda p: (p["quittiert"], _RANK[p["schwere"]], -p["zuletzt"]),
            )
            events = list(self._events)[-150:][::-1]
            battery = list(self._battery)
            meas = dict(self._measurements)

        open_set = [p for p in problems if not p["quittiert"]]
        if any(p["schwere"] == "kritisch" for p in open_set):
            overall = ("kritisch", "Es gibt ein ernstes Problem -- bitte zuerst ansehen.")
        elif any(p["schwere"] == "warnung" for p in open_set):
            overall = ("warnung", "Läuft, aber etwas stimmt nicht.")
        else:
            overall = ("ok", "Alles in Ordnung." if not open_set else "Alles in Ordnung, nur Hinweise.")

        # ~300 points are enough for the curve -- sending 1800 points to the
        # browser every 3 s would just be unnecessary Wi-Fi traffic.
        step = max(1, len(battery) // 300)
        history = [[round(t, 1), v] for t, v in battery[::step]]

        return {
            "zeit": now,
            "simulation": self._mock,
            "gesamt": {"stufe": overall[0], "text": overall[1]},
            "probleme": problems,
            "messwerte": {
                **{k: v for k, v in meas.items() if k not in ("lidar", "kameras", "pose")},
                "server_laufzeit_s": round(now - self._started),
            },
            "geraete": self._devices(meas),
            "absturz": self._crash_status(),
            "ereignisse": events,
            "akku_verlauf": history,
            "schwellen": {
                "akku_warn_v": config.BATTERY_WARN_V,
                "akku_kritisch_v": config.BATTERY_CRITICAL_V,
                "temp_warn_c": config.TEMP_WARN_C,
                "cpu_max_mhz": config.CPU_MAX_MHZ,
            },
            "quellen": {
                "kernel_log": self._kernel_source,
                "usb_zaehler": meas.get("usb_ueberlast_gesamt") is not None,
                "vcgencmd": meas.get("drosselung") is not None,
                "akku": meas.get("akku_v") is not None,
            },
        }

    def _devices(self, meas: dict) -> list[dict]:
        """One entry per device with a traffic light -- for humans, not for programs."""
        devices = []
        connected = bool(self._driver.connected)
        devices.append({
            "name": "Antrieb (Expansion Board)",
            "ok": connected,
            "text": ("Simulation" if self._driver.name == "mock" else "verbunden")
            if connected else "nicht verbunden",
            "neu": None,
        })
        lidar = meas.get("lidar")
        if lidar is not None:
            old = lidar.get("letzte_runde_vor_s")
            ok = bool(lidar.get("geoeffnet")) and old is not None and old < 3.0
            devices.append({
                "name": f"Lidar ({lidar.get('lidar')})",
                "ok": ok,
                "text": f"{lidar.get('umdrehungen', 0)} Umdrehungen" if ok
                else (lidar.get("letzter_fehler") or "keine Daten"),
                "neu": lidar.get("neu_verbunden"),
            })
        for name, st in (meas.get("kameras") or {}).items():
            old = st.get("letztes_bild_vor_s")
            ok = bool(st.get("geoeffnet")) and old is not None and old < 5.0
            devices.append({
                "name": f"Kamera „{name}“ ({st.get('kamera')})",
                "ok": ok,
                "text": f"{st.get('fps_ist', 0)} fps" if ok
                else (st.get("letzter_fehler") or "kein Bild"),
                "neu": st.get("neu_geoeffnet"),
            })
        pose = meas.get("pose")
        if pose is not None:
            if pose.get("fehler"):
                ok, text = False, str(pose["fehler"])
            elif pose.get("aktiv"):
                ok, text = True, f"{pose.get('fps', 0)} Bilder/s" + (" (simuliert)" if pose.get("simuliert") else "")
            else:
                # Idle while interaction is off -- not a problem.
                ok, text = None, "ruht (Interaktion aus)"
            devices.append({"name": "Hailo-Posenerkennung", "ok": ok, "text": text,
                            "neu": pose.get("neu_geoeffnet")})
        return devices
