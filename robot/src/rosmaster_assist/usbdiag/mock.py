"""Fake sysfs for tests and mock sources for the daemon.

Links are plain files holding the target path, and interface dirs use "!" instead
of ":" (windows forbids it in names). MockSources changes that fake sysfs over time,
writes a kernel log with bursts and plays back the recorded usbmon lines.
"""

import heapq
import json
import pathlib
import random
import re
import shutil
import struct
import tempfile
import time
import weakref
from dataclasses import replace

from . import config, topology
from .kmsg import Event, parse_record
from .sampler import Sample, parse_throttled
from .usbmon import DEVICE_ID_COMMAND, Urb, parse_line

_PORT_RE = re.compile(r"(\S+)-port(\d+)")
_ROOT_RE = re.compile(r"usb(\d+)")


def _write(path: pathlib.Path, text) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text), encoding="utf-8")


def write_fake_sysfs(tree: dict, root: pathlib.Path) -> None:
    root = pathlib.Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for path, attrs in tree.get("devices", {}).items():
        (root / path).mkdir(parents=True, exist_ok=True)
        for key, value in attrs.items():
            if key != "interfaces":
                _write(root / path / key, value)
        for iface, info in attrs.get("interfaces", {}).items():
            idir = root / f"{path}!{iface}"
            idir.mkdir(parents=True, exist_ok=True)
            if info.get("driver"):
                _write(idir / "driver", f"../../../bus/usb/drivers/{info['driver']}")
            if info.get("tty"):
                (idir / info["tty"]).mkdir(exist_ok=True)
    for name, info in tree.get("ports", {}).items():
        owner = _PORT_RE.fullmatch(name).group(1)
        root_m = _ROOT_RE.fullmatch(owner)
        if root_m:
            pdir = root / owner / f"{root_m.group(1)}-0!1.0" / name
        else:
            pdir = root / f"{owner}!1.0" / name
        pdir.mkdir(parents=True, exist_ok=True)
        _write(pdir / "state", info.get("state", ""))
        _write(pdir / "over_current_count", info.get("over_current_count", 0))
        if info.get("peer"):
            _write(pdir / "peer", f"../../../{info['peer']}")
        if info.get("device"):
            _write(pdir / "device", "../../../child")


# mock sources

FIXTURES = pathlib.Path(__file__).parent / "fixtures"

BOARD = "1-1.4"
LIDAR = "1-1.2"
DEPTH = "3-1.1"
BOARD_ROOT_PORT = "usb1-port1"
DEPTH_ROOT_PORT = "usb3-port1"
# device path -> (hub port it hangs on, kernel driver name, tty) for the burst messages
_PLUGGED = {
    LIDAR: ("1-1-port2", "cp210x", "ttyUSB0"),
    BOARD: ("1-1-port4", "ch341-uart", "ttyUSB1"),
    DEPTH: ("3-1-port1", None, None),
}


def _kmsg_records(text: str) -> list[str]:
    # a record starts with its priority digit; "#" lines and continuation lines are skipped
    return [line for line in text.splitlines() if line[:1].isdigit()]


def _motion_frame(vx: float) -> bytes:
    body = bytes([0xFF, 0xFC, 10, 0x12, 1]) + struct.pack("<hhh", round(vx * 1000), 0, 0)
    return body + bytes([(sum(body) + 257 - DEVICE_ID_COMMAND) & 0xFF])


class MockSources:
    """Daemon sources without hardware; speed scales the mock clock, 0 means backlog only."""

    def __init__(self, speed: float = 1.0, seed: int | None = None):
        self.speed = speed
        self._rate = speed if speed > 0 else 1.0
        self._t0 = time.monotonic()
        self._boot_wall = time.time() - config.MOCK_START_MONO
        self._rng = random.Random(seed)
        self.tree = json.loads((FIXTURES / "topology.json").read_text(encoding="utf-8"))
        self._dir = pathlib.Path(tempfile.mkdtemp(prefix="usbdiag-mock-"))
        weakref.finalize(self, shutil.rmtree, str(self._dir), True)
        self._root = self._dir / "usb"
        write_fake_sysfs(self.tree, self._root)
        self._unplugged: dict[str, dict] = {}
        self._backlog = _kmsg_records((FIXTURES / "kmsg_burst.txt").read_text(encoding="utf-8"))
        self._backlog_done = False
        self._seq = 6000  # above the backlog sequence numbers
        self._actions: list = []  # heap of (mono, order, fn)
        self._next_burst = 0
        lines = (FIXTURES / "usbmon_board.txt").read_text(encoding="ascii").splitlines()
        self._urbs = [u for u in map(parse_line, lines) if u is not None]
        self._urb_first = self._urbs[0].ts
        self._urb_period = self._urbs[-1].ts - self._urb_first + config.MOCK_URB_GAP_S
        self._urb_cursor = config.MOCK_START_MONO
        self._pending_urbs: list[Urb] = []
        self.capture_bus: int | None = None

    # time

    def now(self) -> float:
        return config.MOCK_START_MONO + (time.monotonic() - self._t0) * self._rate

    def boot_wall(self) -> float:
        return self._boot_wall

    # kernel log

    def kmsg_fd(self) -> int | None:
        return None

    def read_kmsg(self) -> list[Event]:
        if not self._backlog_done:
            self._backlog_done = True
            return [ev for ev in (parse_record(r, self._boot_wall) for r in self._backlog) if ev]
        if self.speed <= 0:
            return []
        now = self.now()
        self._plan_bursts(now)
        out: list[Event] = []
        while self._actions and self._actions[0][0] <= now:
            t, _, fn = heapq.heappop(self._actions)
            for msg in fn(t):
                self._seq += 1
                ev = parse_record(f"6,{self._seq},{int(t * 1e6)},-;{msg}", self._boot_wall)
                if ev is not None:
                    out.append(ev)
        return sorted(out, key=lambda e: e.mono)

    def _at(self, t: float, fn) -> None:
        self._seq += 1
        heapq.heappush(self._actions, (t, self._seq, fn))

    def _burst_time(self, k: int) -> float:
        return config.MOCK_START_MONO + config.MOCK_FIRST_BURST_S + k * config.MOCK_BURST_PERIOD_S

    def _plan_bursts(self, now: float) -> None:
        # plan each burst a little ahead so its drive command can come first
        while self._burst_time(self._next_burst) - 2 * config.MOCK_MOTION_LEAD_S <= now:
            k = self._next_burst
            self._next_burst += 1
            t = self._burst_time(k)
            if k % config.MOCK_MOTOR_BURST_EVERY == 0:
                self._at(t - config.MOCK_MOTION_LEAD_S, lambda at: self._queue_motion(at, config.MOCK_MOTION_VX))
                self._at(t + config.MOCK_MOTION_HOLD_S, lambda at: self._queue_motion(at, 0.0))
            self._at(t, self._burst)

    def _queue_motion(self, at: float, vx: float) -> list[str]:
        attrs = self.tree["devices"].get(BOARD) or self._unplugged[BOARD]
        frame = _motion_frame(vx)
        self._pending_urbs.append(Urb(at, "S", "Bo", 1, int(attrs["devnum"]), 2, -115, len(frame), frame))
        return []

    def _overcurrent(self, port: str) -> str:
        info = self.tree["ports"][port]
        info["over_current_count"] = int(info["over_current_count"]) + 1
        return f"usb {port}: over-current change #{info['over_current_count']}"

    def _unplug(self, path: str) -> list[str]:
        attrs = self.tree["devices"].pop(path, None)
        if attrs is None:
            return []
        self._unplugged[path] = attrs
        port, driver, tty = _PLUGGED[path]
        self.tree["ports"][port].update(state="not attached", device=False)
        # write_fake_sysfs never deletes, so the device dirs go here
        for sub in [path] + [f"{path}!{i}" for i in attrs.get("interfaces", {})]:
            shutil.rmtree(self._root / sub, ignore_errors=True)
        msgs = [f"usb {path}: USB disconnect, device number {attrs['devnum']}"]
        if tty:
            msgs.append(f"{driver} {tty}: {driver} converter now disconnected from {tty}")
        return msgs

    def _plug(self, path: str) -> list[str]:
        attrs = self._unplugged.pop(path, None)
        if attrs is None:
            return []
        self.tree["devices"][path] = attrs
        port, driver, tty = _PLUGGED[path]
        self.tree["ports"][port].update(state="configured", device=True)
        speed = "high" if attrs["speed"] == "480" else "full"
        msgs = [f"usb {path}: new {speed}-speed USB device number {attrs['devnum']} using xhci-hcd",
                f"usb {path}: New USB device found, idVendor={attrs['idVendor']}, "
                f"idProduct={attrs['idProduct']}, bcdDevice= 1.00"]
        if tty:
            msgs.append(f"usb {path}: {driver} converter now attached to {tty}")
        return msgs

    def _burst(self, t: float) -> list[str]:
        msgs = [self._overcurrent(BOARD_ROOT_PORT)]
        for path in (LIDAR, BOARD, DEPTH):
            msgs += self._unplug(path)
        write_fake_sysfs(self.tree, self._root)
        self._at(t + self._rng.uniform(*config.MOCK_RESTORE_S), self._restore)
        return msgs

    def _restore(self, t: float) -> list[str]:
        msgs = []
        for path in (BOARD, LIDAR, DEPTH):
            msgs += self._plug(path)
        write_fake_sysfs(self.tree, self._root)
        self._at(t + config.MOCK_LOOP_DELAY_S, self._depth_loop)
        return msgs

    def _depth_loop(self, t: float) -> list[str]:
        # the depth camera trips its port again right after coming back
        msgs = [self._overcurrent(DEPTH_ROOT_PORT)] + self._unplug(DEPTH)
        msgs += [f"usb {_PLUGGED[DEPTH][0]}: cannot reset (err = -71)"] * config.LOOP_RESET_FAILS
        write_fake_sysfs(self.tree, self._root)
        self._at(t + config.MOCK_LOOP_RESTORE_S, self._depth_back)
        return msgs

    def _depth_back(self, t: float) -> list[str]:
        msgs = self._plug(DEPTH)
        write_fake_sysfs(self.tree, self._root)
        return msgs

    # sysfs and /proc

    def scan_topology(self) -> topology.Topology:
        return topology.scan(str(self._root))

    def sample(self) -> Sample:
        r = self._rng
        v = round(r.uniform(*config.MOCK_EXT5V_V), 3)
        return Sample(self.now(), time.time(), ext5v=v, rails={"EXT5V_V": v},
                      throttled=parse_throttled("throttled=0x0"),
                      temp_c=round(r.uniform(*config.MOCK_TEMP_C), 1), freq_mhz=config.MOCK_FREQ_MHZ,
                      cpu_pct=round(r.uniform(*config.MOCK_CPU_PCT), 1), load1=0.5)

    def processes(self) -> dict:
        return {"server_running": True, "busy": []}

    # usbmon

    def ensure_capture(self) -> bool:
        return True

    def open_capture(self, bus: int) -> None:
        self.capture_bus = bus
        self._urb_cursor = self.now()

    def close_capture(self) -> None:
        self.capture_bus = None

    def capture_fd(self) -> int | None:
        return None

    def read_capture(self) -> list[Urb]:
        now = self.now()
        start = max(self._urb_cursor, now - config.MOCK_CATCH_UP_S)
        self._urb_cursor = now
        if self.speed <= 0 or self.capture_bus != 1 or BOARD not in self.tree["devices"]:
            return []
        out = []
        origin = config.MOCK_START_MONO
        for p in range(int((start - origin) // self._urb_period), int((now - origin) // self._urb_period) + 1):
            shift = origin + p * self._urb_period - self._urb_first
            out += [replace(u, ts=u.ts + shift) for u in self._urbs if start < u.ts + shift <= now]
        due = [u for u in self._pending_urbs if u.ts <= now]
        self._pending_urbs = [u for u in self._pending_urbs if u.ts > now]
        return sorted(out + due, key=lambda u: u.ts)
