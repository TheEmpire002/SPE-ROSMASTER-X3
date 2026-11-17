"""Samples Pi voltages, throttling, temperature and CPU load, never raising."""

import os
import re
import subprocess
import time
from dataclasses import dataclass, field

from . import config

_PMIC_RE = re.compile(r"^\s*(\S+) (?:volt|current)\(\d+\)=([\d.]+)[VA]$")
_THROTTLE_KEYS = ("under_voltage", "freq_capped", "throttled", "soft_temp")


def parse_pmic(text: str) -> dict[str, float]:
    rails: dict[str, float] = {}
    for line in text.splitlines():
        m = _PMIC_RE.match(line.rstrip())
        if m:
            rails[m.group(1)] = float(m.group(2))
    return rails


def parse_throttled(text: str) -> dict[str, bool]:
    m = re.search(r"throttled=(0x[0-9a-fA-F]+)", text)
    if not m:
        return {}
    bits = int(m.group(1), 16)
    out = {}
    for i, key in enumerate(_THROTTLE_KEYS):
        out[f"{key}_now"] = bool(bits >> i & 1)
    for i, key in enumerate(_THROTTLE_KEYS):
        out[f"{key}_boot"] = bool(bits >> (16 + i) & 1)
    return out


def read_cpu_times(text: str) -> tuple[int, int]:
    """(busy, total) jiffies from the aggregate cpu line of /proc/stat."""
    first = text.splitlines()[0].split()
    vals = [int(v) for v in first[1:9]]  # guest time is already inside user
    total = sum(vals)
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return total - idle, total


def cpu_percent(prev: tuple[int, int], cur: tuple[int, int]) -> float:
    dtotal = cur[1] - prev[1]
    if dtotal <= 0:
        return 0.0
    return 100.0 * (cur[0] - prev[0]) / dtotal


@dataclass
class Sample:
    mono: float
    wall: float
    ext5v: float | None = None
    rails: dict = field(default_factory=dict)
    throttled: dict = field(default_factory=dict)
    temp_c: float | None = None
    freq_mhz: float | None = None
    cpu_pct: float | None = None
    load1: float | None = None


def _run(cmd: list[str]) -> str | None:
    try:
        res = subprocess.run(cmd, capture_output=True, text=True,
                             timeout=config.VCGENCMD_TIMEOUT_S, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return res.stdout


def _read(path: str) -> str | None:
    try:
        with open(path, encoding="ascii", errors="replace") as fh:
            return fh.read()
    except OSError:
        return None


class Sampler:
    def __init__(self):
        self._prev_cpu: tuple[int, int] | None = None

    def sample(self) -> Sample:
        s = Sample(mono=time.monotonic(), wall=time.time())
        out = _run(["vcgencmd", "pmic_read_adc"])
        if out:
            s.rails = parse_pmic(out)
            s.ext5v = s.rails.get("EXT5V_V")
        out = _run(["vcgencmd", "get_throttled"])
        if out:
            s.throttled = parse_throttled(out)
        try:
            s.temp_c = int(_read(config.THERMAL_FILE)) / 1000
        except (TypeError, ValueError):
            pass
        try:
            s.freq_mhz = int(_read(config.CPUFREQ_FILE)) / 1000
        except (TypeError, ValueError):
            pass
        try:
            cur = read_cpu_times(_read("/proc/stat"))
            if self._prev_cpu is not None:
                s.cpu_pct = cpu_percent(self._prev_cpu, cur)
            self._prev_cpu = cur
        except (AttributeError, ValueError, IndexError):
            pass
        try:
            s.load1 = float(_read("/proc/loadavg").split()[0])
        except (AttributeError, ValueError, IndexError):
            pass
        return s


def scan_processes(proc: str = "/proc") -> dict:
    """Is the robot server running, and which package or build jobs are busy."""
    server = False
    busy: set[str] = set()
    me = str(os.getpid())
    try:
        names = os.listdir(proc)
    except OSError:
        return {"server_running": False, "busy": []}
    for pid in names:
        if not pid.isdigit() or pid == me:
            continue
        comm = (_read(f"{proc}/{pid}/comm") or "").strip()
        if comm in config.BUSY_PROCESSES:
            busy.add(comm)
        cmd = _read(f"{proc}/{pid}/cmdline") or ""
        if config.SERVER_MARKER in cmd and "usbdiag" not in cmd:
            server = True
    return {"server_running": server, "busy": sorted(busy)}
