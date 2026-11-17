"""Main loop of the USB diagnosis: reads kernel, sysfs and usbmon, never touches a device.

The daemon only reads /dev/kmsg, usbmon, sysfs and /proc. It never opens a serial
port, never writes to the board and never loads a kernel module.
"""

import os
import select
import threading
import time
from typing import Protocol

from . import config, topology, web
from .kmsg import Event, KmsgReader
from .sampler import Sample, Sampler, scan_processes
from .state import DiagState
from .topology import Topology
from .usbmon import Urb, UsbmonTap

_FAILED = object()  # sentinel for a source call that raised or is waiting for its retry


class Sources(Protocol):
    def kmsg_fd(self) -> int | None: ...
    def read_kmsg(self) -> list[Event]: ...
    def scan_topology(self) -> Topology: ...
    def sample(self) -> Sample: ...
    def processes(self) -> dict: ...
    def ensure_capture(self) -> bool: ...
    def open_capture(self, bus: int) -> None: ...
    def close_capture(self) -> None: ...
    def capture_fd(self) -> int | None: ...
    def read_capture(self) -> list[Urb]: ...
    def now(self) -> float: ...
    def boot_wall(self) -> float: ...


class RealSources:
    def __init__(self, kmsg_path: str = "/dev/kmsg", usbmon_root: str = config.USBMON_ROOT,
                 sysfs_root: str = config.SYSFS_USB_ROOT):
        self.kmsg_path = kmsg_path
        self.sysfs_root = sysfs_root
        self._kmsg: KmsgReader | None = None
        self._seen_mono: float | None = None
        self._skip_until: float | None = None
        self._tap = UsbmonTap(usbmon_root)
        self._tap_open = False
        self._sampler = Sampler()
        self._boot_wall: float | None = None

    # kernel log

    def kmsg_fd(self) -> int | None:
        return self._kmsg.fileno() if self._kmsg is not None else None

    def _close_kmsg(self) -> None:
        if self._kmsg is not None:
            try:
                os.close(self._kmsg.fileno())
            except OSError:
                pass
        self._kmsg = None

    def read_kmsg(self) -> list[Event]:
        if self._kmsg is None:
            # a reopen starts at the ring beginning again, so skip what was already handed out
            self._skip_until = self._seen_mono
            self._kmsg = KmsgReader(self.kmsg_path, self.boot_wall())
        try:
            events = self._kmsg.read_available()
        except OSError:
            self._close_kmsg()
            raise
        out = []
        for ev in events:
            if self._skip_until is not None:
                if ev.mono <= self._skip_until:
                    continue
                self._skip_until = None
            out.append(ev)
        if out:
            self._seen_mono = max(self._seen_mono or 0.0, out[-1].mono)
        return out

    # sysfs and /proc

    def scan_topology(self) -> Topology:
        return topology.scan(self.sysfs_root)

    def sample(self) -> Sample:
        return self._sampler.sample()

    def processes(self) -> dict:
        return scan_processes()

    # usbmon

    def ensure_capture(self) -> bool:
        # never modprobe: usbmon is either there already or capture stays off
        return os.path.exists(self._tap.root)

    def open_capture(self, bus: int) -> None:
        self._tap_open = False
        self._tap.open(bus)
        self._tap_open = True

    def close_capture(self) -> None:
        self._tap.close()
        self._tap_open = False

    def capture_fd(self) -> int | None:
        return self._tap.fileno()

    def read_capture(self) -> list[Urb]:
        if not self._tap_open:
            return []
        urbs = self._tap.read_available()
        if self._tap.fileno() is None:
            # the tap closes itself when its bus file vanished
            self._tap_open = False
            raise OSError("usbmon file closed")
        return urbs

    # time

    def now(self) -> float:
        return time.monotonic()

    def boot_wall(self) -> float:
        if self._boot_wall is None:
            with open("/proc/uptime", encoding="ascii") as fh:
                self._boot_wall = time.time() - float(fh.read().split()[0])
        return self._boot_wall


class _Guard:
    """Runs source calls so that one failing source never stops the loop.

    Uses the real clock (not the mock one) so retry and log limits are in real seconds.
    """

    def __init__(self, logger):
        self.logger = logger
        self.retry_at: dict[str, float] = {}
        self.logged: dict[str, float] = {}

    def waiting(self, name: str) -> bool:
        return time.monotonic() < self.retry_at.get(name, 0.0)

    def call(self, name: str, fn, *args):
        if self.waiting(name):
            return _FAILED
        try:
            return fn(*args)
        except Exception as exc:  # noqa: BLE001
            self.failed(name, exc)
            return _FAILED

    def failed(self, name: str, exc: BaseException) -> None:
        self.retry_at[name] = time.monotonic() + config.SOURCE_RETRY_S
        key = f"{name}: {type(exc).__name__}: {exc}"
        if self._should_log(key, config.SOURCE_ERROR_REPEAT_S) and self.logger is not None:
            self.logger.warning("source failed, retry in %g s: %s", config.SOURCE_RETRY_S, key)

    def state_failed(self, method: str, exc: BaseException) -> None:
        # no backoff: a skipped state call would lose the kernel events already read
        if self._should_log(f"state.{method}", config.STATE_ERROR_REPEAT_S) and self.logger is not None:
            self.logger.warning("state update failed, continuing: state.%s: %s: %s",
                                method, type(exc).__name__, exc)

    def _should_log(self, key: str, repeat_s: float) -> bool:
        now = time.monotonic()
        if now - self.logged.get(key, -repeat_s) < repeat_s:
            return False
        if len(self.logged) >= config.SOURCE_ERROR_KEYS_MAX:
            self.logged = {k: t for k, t in self.logged.items() if now - t < config.SOURCE_ERROR_REPEAT_S}
        self.logged[key] = now
        return True


class _Loop:
    def __init__(self, sources, store, stop: threading.Event):
        self.src = sources
        self.store = store
        self.stop = stop
        self.logger = getattr(store, "logger", None)
        self.guard = _Guard(self.logger)
        self.state: DiagState | None = None
        self.capture_bus: int | None = None
        self.capture_missing_logged = False

    def _apply(self, fn, *args):
        try:
            with self.state.lock:
                return fn(*args)
        except Exception as exc:  # noqa: BLE001
            self.guard.state_failed(getattr(fn, "__name__", "call"), exc)
            return _FAILED

    def start(self) -> DiagState:
        src = self.src
        self.state = DiagState(self.store, src.boot_wall(), src.now())
        self.scan_topology(src.now())
        events = self.guard.call("kmsg", src.read_kmsg)
        if events is not _FAILED:
            for ev in events:
                self._apply(self.state.apply_event, ev, True)
        # close a backlog incident that is long over, or the first live event would join it
        self._apply(self.state.tick, src.now())
        return self.state

    def scan_topology(self, now: float) -> None:
        topo = self.guard.call("topology", self.src.scan_topology)
        if topo is not _FAILED:
            self._apply(self.state.apply_topology, topo, now)
        self.update_capture()

    def _close_capture(self, status: str | None) -> None:
        self.guard.call("capture", self.src.close_capture)
        self.capture_bus = None
        if status is not None:
            self._apply(self.state.set_capture, status)

    def update_capture(self) -> None:
        state = self.state
        target = self._apply(state.capture_target)
        if target is _FAILED:
            return
        if target is None:
            if self.capture_bus is not None:
                # capture_target already set camera_on_bus when that is the reason
                self._close_capture("unavailable" if state.capture == "on" else None)
            return
        bus = target[0]
        if bus == self.capture_bus:
            return
        if self.capture_bus is not None:
            self._close_capture(None)
        if self.guard.waiting("capture"):
            return
        if not self.guard.call("usbmon", self.src.ensure_capture):
            if not self.capture_missing_logged and self.logger is not None:
                self.logger.info("usbmon is not available, board traffic is not captured")
            self.capture_missing_logged = True
            self._apply(state.set_capture, "unavailable")
            return
        if self.guard.call("capture", self.src.open_capture, bus) is _FAILED:
            self._apply(state.set_capture, "unavailable")
            return
        self.capture_bus = bus
        self._apply(state.set_capture, "on")

    def wait(self) -> None:
        fds = [fd for fd in (self.src.kmsg_fd(), self.src.capture_fd()) if fd is not None]
        if not fds:
            self.stop.wait(config.LOOP_SELECT_S)
            return
        try:
            select.select(fds, [], [], config.LOOP_SELECT_S)
        except (OSError, ValueError) as exc:
            self.guard.failed("select", exc)
            self.stop.wait(config.LOOP_SELECT_S)

    def run(self) -> None:
        src = self.src
        now = src.now()
        next_topo = now + config.TOPOLOGY_POLL_S
        next_sample = now
        next_proc = now
        next_tick = now
        rescan_at: float | None = None
        while not self.stop.is_set():
            self.wait()
            now = src.now()

            events = self.guard.call("kmsg", src.read_kmsg)
            if events is not _FAILED and events:
                for ev in events:
                    self._apply(self.state.apply_event, ev, False)
                if any(ev.port or ev.kind.startswith("driver_") for ev in events) and rescan_at is None:
                    rescan_at = now + config.TOPOLOGY_DEBOUNCE_S

            if self.capture_bus is not None:
                urbs = self.guard.call("capture", src.read_capture)
                if urbs is _FAILED:
                    self._close_capture("unavailable")
                elif urbs:
                    self._apply(self.state.apply_urbs, urbs, now)

            if now >= next_topo or (rescan_at is not None and now >= rescan_at):
                self.scan_topology(now)
                next_topo = now + config.TOPOLOGY_POLL_S
                rescan_at = None  # a failed tap open is retried on this scan too

            if now >= next_sample:
                s = self.guard.call("sampler", src.sample)
                if s is not _FAILED:
                    self._apply(self.state.apply_sample, s)
                next_sample = now + config.SAMPLE_INTERVAL_S

            if now >= next_proc:
                p = self.guard.call("processes", src.processes)
                if p is not _FAILED:
                    self._apply(self.state.apply_processes, p)
                next_proc = now + config.PROCESS_INTERVAL_S

            if now >= next_tick:
                self._apply(self.state.tick, now)
                next_tick = now + config.TICK_INTERVAL_S

    def shutdown(self) -> None:
        # same path for ctrl-c and sigterm: an open incident is written before the process ends
        self._apply(self.state.shutdown, self.src.now())


def run(sources, store, host: str, port: int, stop: threading.Event | None = None) -> None:
    """Serve the page and run the loop until stop is set (forever without one)."""
    stop = stop or threading.Event()
    loop = _Loop(sources, store, stop)
    state = loop.start()  # the backlog is in before the first page is served
    server = web.make_server(state, host, port, clock=sources.now, logger=loop.logger)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": config.HTTP_POLL_S},
                              name="usbdiag-http", daemon=True)
    thread.start()
    if loop.logger is not None:
        loop.logger.info("usb diagnosis started on port %d", port)
    try:
        loop.run()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        loop.guard.call("capture", sources.close_capture)
        loop.shutdown()
