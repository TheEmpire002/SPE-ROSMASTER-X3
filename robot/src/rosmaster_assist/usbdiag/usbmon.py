"""Passive usbmon reader and decoder for the board serial frames."""

import errno
import os
import struct
import subprocess
from dataclasses import dataclass, field

from . import config

FUNC_NAMES = {
    0x01: "AUTO_REPORT", 0x02: "BEEP", 0x03: "PWM_SERVO", 0x04: "PWM_SERVO_ALL",
    0x05: "RGB", 0x06: "RGB_EFFECT", 0x0F: "RESET_STATE", 0x10: "MOTOR",
    0x11: "CAR_RUN", 0x12: "MOTION", 0x13: "SET_MOTOR_PID", 0x14: "SET_YAW_PID",
    0x15: "SET_CAR_TYPE", 0x20: "UART_SERVO", 0x23: "ARM_CTRL", 0x50: "REQUEST_DATA",
    0x51: "VERSION", 0xA0: "RESET_FLASH",
}

HEAD_REPORT = b"\xff\xfb"
HEAD_COMMAND = b"\xff\xfc"
DEVICE_ID_COMMAND = 0xFC


@dataclass
class Urb:
    ts: float
    event: str
    xfer: str
    bus: int
    dev: int
    ep: int
    status: int | None
    length: int
    data: bytes = b""


def parse_line(line: str) -> Urb | None:
    """Parse one usbmon text line, None for anything that is not a URB line."""
    tok = line.split()
    if len(tok) < 6 or tok[2] not in ("S", "C", "E"):
        return None
    try:
        xfer, bus, dev, ep = tok[3].split(":")
        ts = int(tok[1]) / 1e6
        if tok[4] == "s":
            # setup packet: s, five setup fields, then the length
            status, length, rest = None, int(tok[10]), tok[11:]
        else:
            status, length, rest = int(tok[4]), int(tok[5]), tok[6:]
        data = b""
        if rest and rest[0] == "=":
            data = bytes.fromhex("".join(rest[1:]))
        return Urb(ts, tok[2], xfer, int(bus), int(dev), int(ep), status, length, data)
    except (ValueError, IndexError):
        return None


@dataclass
class Frame:
    type: int
    payload: bytes  # without checksum


class FrameDecoder:
    """Board to host frames: FF FB ext_len ext_type payload(ext_len-2) checksum."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        out: list[Frame] = []
        buf = self._buf
        while True:
            i = buf.find(HEAD_REPORT)
            if i < 0:
                # keep a trailing ff, it may start the next head
                del buf[:len(buf) - 1 if buf.endswith(b"\xff") else len(buf)]
                break
            del buf[:i]
            if len(buf) < 4:
                break
            ext_len = buf[2]
            if ext_len < 2 or ext_len > config.FRAME_MAX_EXT_LEN:
                del buf[:1]
                continue
            total = ext_len + 3
            if len(buf) < total:
                break
            ext_type = buf[3]
            payload = bytes(buf[4:total - 1])
            if (ext_len + ext_type + sum(payload)) % 256 == buf[total - 1]:
                out.append(Frame(ext_type, payload))
                del buf[:total]
            else:
                # wrong checksum: a truncated urb may hide the real head inside
                del buf[:1]
        if len(buf) > config.FRAME_BUFFER_MAX:
            del buf[:len(buf) - config.FRAME_BUFFER_MAX]
        return out


def decode_report(frame: Frame) -> dict:
    p = frame.payload
    try:
        if frame.type == 0x0A:
            vx, vy, vz, batt = struct.unpack_from("<hhhB", p)
            return {"vx": vx / 1000, "vy": vy / 1000, "vz": vz / 1000, "battery_v": batt / 10}
        if frame.type == 0x0C:
            roll, pitch, yaw = struct.unpack_from("<hhh", p)
            return {"roll": roll / 10000, "pitch": pitch / 10000, "yaw": yaw / 10000}
        if frame.type == 0x0D:
            return {"encoders": list(struct.unpack_from("<iiii", p))}
    except struct.error:
        return {}
    return {}


@dataclass
class Command:
    func: int
    name: str
    params: dict = field(default_factory=dict)


def decode_commands(data: bytes) -> list[Command]:
    """Host to board frames: FF FC len func params checksum, bad ones skipped."""
    out: list[Command] = []
    i = 0
    while True:
        i = data.find(HEAD_COMMAND, i)
        if i < 0 or i + 4 > len(data):
            break
        total = data[i + 2] + 2
        frame = data[i:i + total]
        if total < 5 or len(frame) < total:
            i += 1
            continue
        body = frame[:-1]
        if (sum(body) + 257 - DEVICE_ID_COMMAND) & 0xFF != frame[-1]:
            i += 1
            continue
        func = body[3]
        params = body[4:]
        parsed: dict = {}
        if func == 0x12 and len(params) >= 7:
            vx, vy, vz = struct.unpack_from("<hhh", params, 1)
            parsed = {"car_type": params[0], "vx": vx / 1000, "vy": vy / 1000, "vz": vz / 1000}
        elif func == 0x10 and len(params) >= 4:
            parsed = {"speeds": list(struct.unpack_from("<4b", params))}
        out.append(Command(func, FUNC_NAMES.get(func, f"0x{func:02X}"), parsed))
        i += total
    return out


class UsbmonTap:
    def __init__(self, root: str = config.USBMON_ROOT):
        self.root = root
        self.bus: int | None = None
        self._fd: int | None = None
        self._partial = b""

    def ensure_loaded(self) -> bool:
        if os.path.exists(self.root):
            return True
        try:
            subprocess.run(["modprobe", "usbmon"], timeout=config.MODPROBE_TIMEOUT_S,
                           capture_output=True, check=False)
        except (OSError, subprocess.SubprocessError):
            return False
        return os.path.exists(self.root)

    def open(self, bus: int) -> None:
        self.close()
        self._fd = os.open(f"{self.root}/{bus}u", os.O_RDONLY | os.O_NONBLOCK)
        self.bus = bus
        self._partial = b""

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
        self._fd = None
        self.bus = None
        self._partial = b""

    def fileno(self) -> int | None:
        return self._fd

    def read_available(self) -> list[Urb]:
        if self._fd is None:
            return []
        try:
            chunk = os.read(self._fd, config.USBMON_READ_BYTES)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EINTR):
                return []
            self.close()  # bus or file vanished
            return []
        lines = (self._partial + chunk).split(b"\n")
        self._partial = lines.pop()
        urbs = []
        for raw in lines:
            urb = parse_line(raw.decode("ascii", "replace"))
            if urb is not None:
                urbs.append(urb)
        return urbs
