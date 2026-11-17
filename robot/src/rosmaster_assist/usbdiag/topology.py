"""Scan of the USB tree in sysfs; every read tolerates files vanishing."""

import os
import re
from dataclasses import dataclass, field

from . import config
from .kmsg import normalize_port

_DEVICE_RE = re.compile(r"\d+-\d+(\.\d+)*")
_ROOT_RE = re.compile(r"usb(\d+)")
_PORT_RE = re.compile(r"(\S+)-port(\d+)")
# interface dirs are "1-1:1.0"; the test tree uses "!" because windows forbids ":"
_IFACE_SEP_RE = re.compile(r"[:!]")


@dataclass
class DeviceInfo:
    path: str
    vid: str
    pid: str
    product: str
    speed_mbps: int
    max_power_ma: int
    devnum: int
    busnum: int
    driver: str | None
    tty: str | None


@dataclass
class PortInfo:
    path: str  # path a device on this port gets, e.g. "1-1.4"
    hub: str  # "usb1" for root ports, else the hub path
    number: int
    state: str
    over_current: int
    peer: str | None
    device: DeviceInfo | None


@dataclass
class Topology:
    ports: dict[str, PortInfo] = field(default_factory=dict)
    devices: dict[str, DeviceInfo] = field(default_factory=dict)


def _read(path: str, default: str = "") -> str:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return fh.read().strip()
    except (OSError, ValueError):
        return default


def _read_int(path: str, default: int = 0) -> int:
    m = re.match(r"\d+", _read(path))
    return int(m.group()) if m else default


def _listdir(path: str) -> list[str]:
    try:
        return sorted(os.listdir(path))
    except OSError:
        return []


def _link_name(path: str) -> str | None:
    # real sysfs has symlinks, the test tree has files holding the target path
    try:
        if os.path.islink(path):
            target = os.readlink(path)
        else:
            with open(path, encoding="utf-8", errors="replace") as fh:
                target = fh.read().strip()
    except (OSError, ValueError):
        return None
    return target.replace("\\", "/").rstrip("/").rsplit("/", 1)[-1] or None


def _scan_device(root: str, path: str, interfaces: list[str]) -> DeviceInfo:
    base = os.path.join(root, path)
    driver = tty = None
    for iface in interfaces:
        idir = os.path.join(root, iface)
        if driver is None:
            driver = _link_name(os.path.join(idir, "driver"))
        if tty is None:
            tty = next((n for n in _listdir(idir) if n.startswith("ttyUSB")), None)
    return DeviceInfo(
        path=path,
        vid=_read(os.path.join(base, "idVendor")),
        pid=_read(os.path.join(base, "idProduct")),
        product=_read(os.path.join(base, "product")),
        speed_mbps=_read_int(os.path.join(base, "speed")),
        max_power_ma=_read_int(os.path.join(base, "bMaxPower")),
        devnum=_read_int(os.path.join(base, "devnum")),
        busnum=_read_int(os.path.join(base, "busnum")),
        driver=driver,
        tty=tty,
    )


def _scan_port(port_dir: str, path: str, hub: str, number: int) -> PortInfo:
    peer = _link_name(os.path.join(port_dir, "peer"))
    return PortInfo(
        path=path,
        hub=hub,
        number=number,
        state=_read(os.path.join(port_dir, "state")),
        over_current=_read_int(os.path.join(port_dir, "over_current_count")),
        peer=normalize_port(peer) if peer else None,
        device=None,
    )


def _scan_hub_ports(topo: Topology, dir_: str, owner: str, prefix: str, sep: str) -> None:
    for child in _listdir(dir_):
        pm = _PORT_RE.fullmatch(child)
        if pm and pm.group(1) == owner:
            num = int(pm.group(2))
            path = f"{prefix}{sep}{num}"
            topo.ports[path] = _scan_port(os.path.join(dir_, child), path, owner, num)


def scan(root: str = config.SYSFS_USB_ROOT) -> Topology:
    topo = Topology()
    entries = _listdir(root)
    interfaces: dict[str, list[str]] = {}
    for name in entries:
        parts = _IFACE_SEP_RE.split(name, maxsplit=1)
        if len(parts) == 2:
            owner = parts[0]
            interfaces.setdefault(owner, []).append(name)

    for name in entries:
        m = _ROOT_RE.fullmatch(name)
        if m:
            bus = m.group(1)
            bdir = os.path.join(root, name)
            for sub in _listdir(bdir):
                if sub.startswith(f"{bus}-0") and _IFACE_SEP_RE.search(sub):
                    _scan_hub_ports(topo, os.path.join(bdir, sub), name, bus, "-")
        elif _DEVICE_RE.fullmatch(name):
            topo.devices[name] = _scan_device(root, name, interfaces.get(name, []))
            for iface in interfaces.get(name, []):
                _scan_hub_ports(topo, os.path.join(root, iface), name, name, ".")

    for path, port in topo.ports.items():
        port.device = topo.devices.get(path)
    return topo


def device_role(dev: DeviceInfo | None) -> str:
    if dev is None:
        return "other"
    return config.KNOWN_DEVICES.get((dev.vid, dev.pid), ("", "other"))[1]


def device_name(dev: DeviceInfo | None) -> str:
    if dev is None:
        return config.NO_DEVICE_NAME
    known = config.KNOWN_DEVICES.get((dev.vid, dev.pid))
    if known:
        return known[0]
    product = dev.product.strip()
    if product:
        return f"„{product}“"
    return config.UNKNOWN_DEVICE_NAME
