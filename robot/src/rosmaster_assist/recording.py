"""Drive recorder -- records real drives so that they can be replayed.

On the robot it is hard to understand why it got stuck at a door or brushed
a chair: by the time you look, the situation is over, and it can rarely be
reproduced exactly. That is why this recorder continuously keeps the last
``RECORDING_BUFFER_S`` seconds in memory:

- every lidar scan (angles and distances, rounded),
- every drive command at the watchdog: requested (``soll``) and after the
  speed limit and braking distance check (``safe``), with source,
- every emergency stop (``stop()``),
- at the odometry tick the board velocity, pose and the executed command
  (after the ramp),
- state changes of explorer, interaction and homing.

Only pressing "Fahrt speichern" (save drive) in the app writes the buffer
to the SD card (gzip, one JSON line per event). Recording all the time would
wear the SD card, and the only interesting part anyway is the seconds
before the moment you think: "That was wrong."

Analysis happens on your own computer with ``scripts/replay.py``: it shows
what happened, draws the drive as SVG and can feed the recorded scans to a
new explorer -- this way a changed avoidance logic can be checked against
real drives without moving the robot.

The recorder must never disturb driving: the watchdog calls it inside a
``try``, and all methods are short and thread-safe.
"""

from __future__ import annotations

import gzip
import io
import json
import logging
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable

from . import config
from .sensors.lidar import Scan

log = logging.getLogger(__name__)

#: File format. Increase on incompatible changes so that playback recognizes
#: old files instead of misinterpreting them.
VERSION = 1

#: Only files named like this are served -- the name comes from the URL,
#: and ".." or a path must not lead to other files.
_FILENAME = re.compile(r"^fahrt_\d{8}_\d{6}(_\d{1,3})?\.jsonl\.gz$")

#: These settings end up in the file header: whoever replays a drive later
#: needs to know which limits the robot was running with.
_HEADER_SETTINGS = (
    "AUTONOMOUS_", "EXPLORE_", "LIDAR_", "RAMP_", "ODOMETRY_", "OBSTACLE_",
    "FOLLOW_", "MAX_SPEED_",
)


def valid_name(name: str) -> bool:
    """Whether ``name`` denotes a recording file (and nothing else)."""
    return bool(_FILENAME.match(name))


def _r(value: float, digits: int = 4) -> float:
    return round(float(value), digits)


def _command_list(cmd) -> list[float]:
    return [_r(cmd.vx), _r(cmd.vy), _r(cmd.omega)]


def round_scan(scan: Scan) -> tuple[list[int], list[int]]:
    """Angles in tenths of a degree, distances in millimeters.

    The RPLIDAR A1 does not measure more precisely anyway, and integers take
    only a fraction of the space in memory and in the file. Tests that
    compare recording and playback round their scans the same way.
    """
    return (
        [int(round(w * 10.0)) for w in scan.angle],
        [int(round(r * 1000.0)) for r in scan.ranges],
    )


def scan_from(event: dict) -> Scan:
    """Turn a scan event back into a ``Scan``."""
    scan = Scan([w / 10.0 for w in event["w"]], [r / 1000.0 for r in event["r"]])
    scan.timestamp = event["zeit"]
    scan.duration = event.get("dauer", Scan.DEFAULT_DURATION_S)
    return scan


class Recorder:
    """Ring buffer of the last seconds of a drive, saved on request."""

    def __init__(
        self,
        buffer_s: float | None = None,
        folder: Path | None = None,
        max_files: int | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._buffer_s = config.RECORDING_BUFFER_S if buffer_s is None else buffer_s
        self._folder = Path(config.RECORDING_FOLDER if folder is None else folder)
        self._max_files = (
            config.RECORDING_MAX_FILES if max_files is None else max_files
        )
        self._clock = clock
        self._lock = threading.Lock()
        self._buffer: deque[dict] = deque()
        self._last_scan: float | None = None
        self._last_state: dict | None = None
        self._saved = 0
        self._last_file: str | None = None
        #: Goes into the file header (driver, simulation, ...); set by the server.
        self.header: dict[str, Any] = {}

    # --- Recording ----------------------------------------------------------

    def _append(self, event: dict) -> None:
        with self._lock:
            self._buffer.append(event)
            # Trim by age, not by count: scans, commands and odometry
            # arrive at different rates, and what is wanted is "the last
            # minute", not "the last 3000 lines".
            limit = event["t"] - self._buffer_s
            while self._buffer and self._buffer[0]["t"] < limit:
                self._buffer.popleft()

    def command(self, source: str, requested, safe, now: float | None = None) -> None:
        """Drive command at the watchdog: requested and after the limits."""
        self._append({
            "t": self._clock() if now is None else now,
            "art": "befehl",
            "quelle": source,
            "soll": _command_list(requested),
            "safe": _command_list(safe),
        })

    def stop(self, now: float | None = None) -> None:
        """Hard stop at the watchdog (without ramp)."""
        self._append({"t": self._clock() if now is None else now, "art": "stop"})

    def scan(self, scan: Scan | None, now: float | None = None) -> bool:
        """Record a lidar scan if it is new. True if recorded.

        The loop asks more often than the lidar delivers (20 Hz vs.
        7.5 Hz); storing the same scan twice would distort the scan rate
        during playback.
        """
        if scan is None:
            return False
        with self._lock:
            if self._last_scan == scan.timestamp:
                return False
            self._last_scan = scan.timestamp
        w, r = round_scan(scan)
        self._append({
            "t": self._clock() if now is None else now,
            "art": "scan",
            "zeit": scan.timestamp,
            "dauer": _r(scan.duration),
            "w": w,
            "r": r,
        })
        return True

    def odo(
        self,
        v: Iterable[float],
        pose: Iterable[float],
        actual,
        measuring: bool,
        now: float | None = None,
    ) -> None:
        """Ego-motion: board velocity, pose, executed command."""
        self._append({
            "t": self._clock() if now is None else now,
            "art": "odo",
            "v": [_r(x) for x in v],
            "pose": [_r(x) for x in pose],
            "ist": _command_list(actual),
            "misst": bool(measuring),
        })

    def state(self, now: float | None = None, **fields: Any) -> bool:
        """States of the modules; only a change is recorded."""
        with self._lock:
            if fields == self._last_state:
                return False
            self._last_state = dict(fields)
        self._append({"t": self._clock() if now is None else now, "art": "zustand", **fields})
        return True

    # --- Queries ------------------------------------------------------------

    def events(self) -> list[dict]:
        """Copy of the buffer (oldest first)."""
        with self._lock:
            return list(self._buffer)

    def status(self) -> dict:
        with self._lock:
            count = len(self._buffer)
            scans = sum(1 for e in self._buffer if e["art"] == "scan")
            duration = self._buffer[-1]["t"] - self._buffer[0]["t"] if count > 1 else 0.0
        return {
            "aktiv": True,
            "puffer_s": self._buffer_s,
            "ereignisse": count,
            "scans": scans,
            "dauer_s": round(duration, 1),
            "ordner": str(self._folder),
            "gespeichert": self._saved,
            "letzte_datei": self._last_file,
        }

    # --- Saving -------------------------------------------------------------

    def save(self, note: str = "") -> Path:
        """Write the buffer as ``fahrt_<date>_<time>.jsonl.gz``.

        Runs in the server via ``asyncio.to_thread``: gzip and writing to
        the SD card take a few hundred milliseconds, and the drive loop must
        not wait that long.
        """
        events = self.events()
        t0 = events[0]["t"] if events else self._clock()
        now = datetime.now()
        header = {
            "art": "kopf",
            "version": VERSION,
            "erstellt": now.isoformat(timespec="seconds"),
            "notiz": str(note)[:200],
            "dauer_s": round(events[-1]["t"] - t0, 2) if events else 0.0,
            **self.header,
            "config": _config_snapshot(),
        }
        self._folder.mkdir(parents=True, exist_ok=True)
        path = self._unused_name(now)
        # Times relative to the first event: the Pi's monotonic clock counts
        # from power-on and means nothing in the file.
        buffer = io.StringIO()
        buffer.write(json.dumps(header, ensure_ascii=False) + "\n")
        for e in events:
            e = dict(e, t=round(e["t"] - t0, 4))
            if "zeit" in e:
                e["zeit"] = round(e["zeit"] - t0, 4)
            buffer.write(json.dumps(e, ensure_ascii=False, separators=(",", ":")) + "\n")
        # Write under a different name first, then rename: if power fails
        # while writing, no half-written file with a valid name is left
        # behind.
        tmp = path.with_name(path.name + ".tmp")
        with gzip.open(tmp, "wt", encoding="utf-8") as f:
            f.write(buffer.getvalue())
        tmp.replace(path)
        self._cleanup()
        with self._lock:
            self._saved += 1
            self._last_file = path.name
        log.info("Drive saved: %s (%d events)", path.name, len(events))
        return path

    def _unused_name(self, now: datetime) -> Path:
        base = f"fahrt_{now:%Y%m%d_%H%M%S}"
        path = self._folder / f"{base}.jsonl.gz"
        n = 1
        while path.exists():  # saved twice within the same second
            path = self._folder / f"{base}_{n}.jsonl.gz"
            n += 1
        return path

    def _files(self) -> list[Path]:
        if not self._folder.is_dir():
            return []
        return sorted(
            (p for p in self._folder.iterdir() if valid_name(p.name)),
            key=lambda p: p.stat().st_mtime,
        )

    def _cleanup(self) -> None:
        """Keep only the newest ``max_files`` -- the SD card is small."""
        files = self._files()
        for old in files[: max(0, len(files) - self._max_files)]:
            try:
                old.unlink()
            except OSError:
                log.warning("Old recording %s cannot be deleted", old.name)

    def entries(self) -> list[dict]:
        """Saved drives, newest first."""
        return [
            {"name": p.name, "bytes": p.stat().st_size,
             "zeit": datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec="seconds")}
            for p in reversed(self._files())
        ]

    def file(self, name: str) -> Path | None:
        """Path of a saved drive, None for an invalid name."""
        if not valid_name(name):
            return None
        path = self._folder / name
        return path if path.is_file() else None


def _config_snapshot() -> dict[str, Any]:
    values = {}
    for name in dir(config):
        if name.startswith(_HEADER_SETTINGS):
            value = getattr(config, name)
            if isinstance(value, (bool, int, float, str)):
                values[name] = value
    return values


# --- Reading and playback ------------------------------------------------------


def load(path: str | Path) -> tuple[dict, list[dict]]:
    """Read a file: (header, events)."""
    header: dict = {}
    events: list[dict] = []
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            e = json.loads(line)
            if e.get("art") == "kopf":
                header = e
            else:
                events.append(e)
    if header.get("version", VERSION) > VERSION:
        raise ValueError(f"Recording has version {header['version']}, supported up to {VERSION}")
    return header, events


class Playback:
    """Plays back recorded sensor data with matching timing.

    Provides the same methods that the explorer and co. expect from lidar
    and odometry (``latest()``, ``velocity()``, ``actually_measuring``).
    ``until(t)`` sets the clock: then the last lidar scan completed by
    ``t`` and the last odometry value up to ``t`` apply. This way the
    replayed explorer sees exactly what the real one could see at that
    moment.
    """

    def __init__(self, events: list[dict]) -> None:
        self._scans = [e for e in events if e["art"] == "scan"]
        self._scans.sort(key=lambda e: e["zeit"])
        self._odo = [e for e in events if e["art"] == "odo"]
        self._i_scan = -1
        self._i_odo = -1
        self._scan: Scan | None = None
        self._t = float("-inf")

    def until(self, t: float) -> None:
        """Advance the clock (forward only)."""
        if t < self._t:
            raise ValueError("Playback only runs forward")
        self._t = t
        old = self._i_scan
        while self._i_scan + 1 < len(self._scans) and self._scans[self._i_scan + 1]["zeit"] <= t:
            self._i_scan += 1
        if self._i_scan != old:
            self._scan = scan_from(self._scans[self._i_scan])
        while self._i_odo + 1 < len(self._odo) and self._odo[self._i_odo + 1]["t"] <= t:
            self._i_odo += 1

    # like LidarService
    def latest(self) -> Scan | None:
        return self._scan

    # like Odometry
    @property
    def actually_measuring(self) -> bool:
        return self._i_odo >= 0 and self._odo[self._i_odo]["misst"]

    def velocity(self) -> tuple[float, float, float]:
        if self._i_odo < 0:
            return 0.0, 0.0, 0.0
        vx, vy, w = self._odo[self._i_odo]["v"]
        return vx, vy, w

    def pose(self) -> tuple[float, float, float]:
        if self._i_odo < 0:
            return 0.0, 0.0, 0.0
        x, y, deg = self._odo[self._i_odo]["pose"]
        return x, y, deg


def segments(events: list[dict], field_: str = "erkunder") -> list[tuple[float, float]]:
    """Time spans (from, to) in which ``field_`` was set in the state.

    So for ``"erkunder"``: when the explorer was running. A segment that is
    still open at the end of the recording ends with the last event.
    """
    result = []
    from_ = None
    for e in events:
        if e["art"] != "zustand":
            continue
        an = e.get(field_) is not None
        if an and from_ is None:
            from_ = e["t"]
        elif not an and from_ is not None:
            result.append((from_, e["t"]))
            from_ = None
    if from_ is not None and events:
        result.append((from_, events[-1]["t"]))
    return result


def replay_explorer(
    events: list[dict],
    build: Callable[[Playback], Any],
) -> list[dict]:
    """Feed recorded explorer drives to a (new) explorer.

    ``build(playback)`` returns an explorer that uses ``playback`` as lidar
    and odometry. For each segment in which the explorer was running, a
    fresh drive begins; for every recorded autonomous drive command it
    computes one tick at the same clock time. Result per tick:
    ``{"t", "aufgezeichnet", "nachgespielt"}`` (each ``[vx, vy, omega]``,
    ``nachgespielt`` None if it gives up).

    Deliberately calls ``_begin``/``_step`` (like the tests): this way no
    drive loop runs, and the recording alone determines the clock.
    Identical commands are only to be expected if the explorer is unchanged
    and ``random`` was seeded the same -- on the robot it picks directions
    and wander times at random.
    """
    result = []
    for from_, until_t in segments(events, "erkunder"):
        playback = Playback(events)
        explorer = build(playback)
        explorer._begin(from_)
        for e in events:
            if e["art"] != "befehl" or e.get("quelle") != "autonom":
                continue
            if not from_ <= e["t"] <= until_t:
                continue
            playback.until(e["t"])
            command = explorer._step(e["t"])
            result.append({
                "t": e["t"],
                "aufgezeichnet": e["soll"],
                "nachgespielt": None if command is None else _command_list(command),
            })
            if command is None:
                break
    return result
