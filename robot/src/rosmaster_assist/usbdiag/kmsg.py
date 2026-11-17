"""Parser and reader for /dev/kmsg records."""

import os
import re
from dataclasses import dataclass, field


@dataclass
class Event:
    mono: float  # s since boot
    wall: float  # epoch s
    kind: str
    port: str | None  # device-path form, e.g. "1-1.4"
    details: dict = field(default_factory=dict)
    raw: str = ""


def normalize_port(name: str) -> str:
    m = re.fullmatch(r"usb(\d+)-port(\d+)", name)
    if m:
        return f"{m.group(1)}-{m.group(2)}"
    m = re.fullmatch(r"(\S+)-port(\d+)", name)
    if m:
        return f"{m.group(1)}.{m.group(2)}"
    return name


# (kind, regex, port group or None, details builder)
_PATTERNS = [
    ("disconnect", re.compile(r"usb (\S+): USB disconnect, device number (\d+)"), 1,
     lambda m: {}),
    ("new_device", re.compile(r"usb (\S+): new .*USB device number (\d+) using"), 1,
     lambda m: {}),
    ("connect", re.compile(r"usb (\S+): New USB device found, idVendor=(\w+), idProduct=(\w+)"), 1,
     lambda m: {"vid": m.group(2), "pid": m.group(3)}),
    # root hub ports read usb1-port1, ports of an external hub 1-1-port2
    ("overcurrent", re.compile(r"usb (\S+-port\d+): over-current change #(\d+)"), 1,
     lambda m: {}),
    ("reset_failed", re.compile(r"usb (\S+-port\d+): cannot reset \(err = (-\d+)\)"), 1,
     lambda m: {"error": int(m.group(2))}),
    ("enum_failed", re.compile(r"usb (\S+): device descriptor read/\S+, error (-\d+)"), 1,
     lambda m: {"error": int(m.group(2))}),
    ("protocol_error", re.compile(r"usb (\S+): clear tt \d+ \(\w+\) error (-\d+)"), 1,
     lambda m: {"error": int(m.group(2))}),
    ("driver_attached", re.compile(r"usb (\S+): \S+ converter now attached to (ttyUSB\d+)"), 1,
     lambda m: {"tty": m.group(2)}),
    ("driver_detached",
     re.compile(r"\S+ (ttyUSB\d+): \S+ converter now disconnected from ttyUSB\d+"), None,
     lambda m: {"tty": m.group(1)}),
    ("undervoltage", re.compile(r"Undervoltage detected"), None, lambda m: {}),
]


def parse_record(record: str, boot_wall: float) -> Event | None:
    # only the first line counts; continuation lines start with a space
    first = record.split("\n", 1)[0]
    head, sep, msg = first.partition(";")
    if not sep:
        return None
    fields = head.split(",")
    if len(fields) < 3:
        return None
    try:
        mono = int(fields[2]) / 1e6
    except ValueError:
        return None
    for kind, rx, port_group, build in _PATTERNS:
        m = rx.search(msg)
        if m:
            port = normalize_port(m.group(port_group)) if port_group else None
            return Event(mono, boot_wall + mono, kind, port, build(m), first)
    return None


class KmsgReader:
    def __init__(self, path: str = "/dev/kmsg", boot_wall: float = 0.0):
        self.boot_wall = boot_wall
        # starts at the ring beginning so boot-time events are included
        self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)

    def fileno(self) -> int:
        return self._fd

    def read_available(self) -> list[Event]:
        events: list[Event] = []
        while True:
            try:
                data = os.read(self._fd, 8192)
            except BlockingIOError:
                break
            except BrokenPipeError:
                continue  # records were overwritten, keep reading
            if not data:
                break
            text = data.decode("utf-8", errors="replace").rstrip("\n")
            ev = parse_record(text, self.boot_wall)
            if ev is not None:
                events.append(ev)
        return events
