"""Camera images for the web app and image processing.

All cameras that are found run, each with its own background thread.
Three variants of the same interface:

1. ``Picamera2``  -- camera on the Pi's CSI connector
2. ``OpenCV``     -- USB camera
3. ``Mock``       -- generated test image, so the app can be developed
                     without a camera

Which USB camera serves which purpose (person detection, orientation) is
set in ``config.CAMERA_ROLES``. Assignment is by USB ID, not by
``/dev/videoN``: the numbers get swapped after a reboot or re-plugging.

Delivery is as MJPEG: an endless sequence of single images in one HTTP
response. Every browser can show that directly in an ``<img>`` without
JavaScript or an extra library -- much less effort than WebRTC, and
perfectly sufficient for a camera image on the local Wi-Fi.
"""

from __future__ import annotations

import io
import logging
import math
import os
import struct
import threading
import time
import zlib
from abc import ABC, abstractmethod
from dataclasses import dataclass

from .. import config

log = logging.getLogger(__name__)

#: A single image: the raw data and its MIME type.
Frame = tuple[bytes, str]


class CameraBase(ABC):
    """A camera that can deliver single images.

    ``open()`` must be callable repeatedly without claiming the camera a
    second time: with most drivers a device opened twice can no longer be
    opened, and the error then looks as if there were no camera at all.
    """

    name: str = "base"
    #: Human-readable name for log messages.
    label: str = "Kamera"
    #: Horizontal field of view in degrees -- maps image positions to the lidar.
    fov_deg: float = config.CAMERA_FOV_DEG

    def __init__(
        self,
        name: str | None = None,
        label: str | None = None,
        fov_deg: float | None = None,
    ) -> None:
        self._opened = False
        # Overridable per instance: with several cameras each needs its own
        # name, otherwise endpoints and log messages cannot be told apart.
        if name is not None:
            self.name = name
        if label is not None:
            self.label = label
        if fov_deg is not None:
            self.fov_deg = fov_deg

    @property
    def device(self) -> str | None:
        """Which device is currently in use -- for /camera/status."""
        return None

    @property
    def opened(self) -> bool:
        return self._opened

    def open(self) -> None:
        if self._opened:
            return
        try:
            self._open()
        except Exception:
            # A failure can leave a half-opened device behind -- e.g. when
            # picamera2 has already claimed the camera and only the encoder
            # fails. Without cleanup it would stay blocked for the next
            # attempt.
            try:
                self._close()
            except Exception:
                log.debug("Cleanup after failure did not succeed", exc_info=True)
            raise
        self._opened = True

    def close(self) -> None:
        if not self._opened:
            return
        try:
            self._close()
        finally:
            # Even if closing fails (device already gone): otherwise open()
            # would consider the camera open and do nothing on the retry.
            self._opened = False

    @abstractmethod
    def _open(self) -> None: ...

    @abstractmethod
    def _close(self) -> None: ...

    @abstractmethod
    def read(self) -> Frame | None:
        """Fetch an image, or None if none is available right now."""


# --- Camera on the CSI connector ------------------------------------------------------


class _JpegSink(io.BufferedIOBase):
    """Receives the encoded images from picamera2.

    The encoder calls ``write()`` for every image. Only the newest one is
    kept -- whoever fetches too slowly skips images instead of a backlog
    building up.
    """

    def __init__(self) -> None:
        self.frame: bytes | None = None
        self._lock = threading.Lock()

    def write(self, buf) -> int:  # type: ignore[override]
        data = bytes(buf)
        with self._lock:
            self.frame = data
        return len(data)

    def latest(self) -> bytes | None:
        with self._lock:
            return self.frame


class Picamera2Camera(CameraBase):
    name = "picamera2"
    #: Appears in log messages and the app -- German, as a user-facing label.
    label = "CSI-Kamera"

    def __init__(self, index: int = 0, **kwargs) -> None:
        super().__init__(**kwargs)
        self._number = index
        self._cam = None
        self._sink: _JpegSink | None = None

    @property
    def device(self) -> str | None:
        return f"CSI {self._number}"

    def _open(self) -> None:
        # All of these exist only on the Pi.
        from picamera2 import Picamera2
        from picamera2.encoders import JpegEncoder
        from picamera2.outputs import FileOutput

        self._cam = Picamera2(self._number)
        self._cam.configure(
            self._cam.create_video_configuration(
                main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)}
            )
        )

        # Images are encoded continuously and handed in, instead of us
        # requesting each one. Single captures (capture_file) are too slow
        # for a continuous stream and tend to stall.
        self._sink = _JpegSink()
        self._cam.start_recording(
            JpegEncoder(q=config.CAMERA_QUALITY), FileOutput(self._sink)
        )

        # Wait for the first image: the sensor needs a moment for exposure
        # and white balance. Without this the first image would be black
        # -- and a failure here would only be noticed later.
        deadline = time.monotonic() + 5.0
        while self._sink.latest() is None:
            if time.monotonic() > deadline:
                raise RuntimeError("Kamera liefert kein Bild (5 s gewartet)")
            time.sleep(0.1)

        log.info("Camera '%s' (CSI %d) ready", self.name, self._number)

    def _close(self) -> None:
        if self._cam is None:
            return
        try:
            self._cam.stop_recording()
            # stop_recording() only ends the image stream -- the camera stays
            # assigned to the process. Without close() every later open
            # fails with "Device or resource busy".
            self._cam.close()
        except Exception:
            log.exception("Camera could not be closed cleanly")
        finally:
            self._cam = None
            self._sink = None

    def read(self) -> Frame | None:
        if self._sink is None:
            return None
        data = self._sink.latest()
        return (data, "image/jpeg") if data else None


# --- USB camera -------------------------------------------------------------------


#: Devices that show up under /dev/video* but are not cameras.
#: On the Pi 5 these are the image processing and codec units --
#: they can be opened and then block on read.
NOT_A_CAMERA = ("pispbe", "hevc", "codec", "isp", "cfe", "rpivid")

#: "flip" in config.CAMERA_ROLES -> code for cv2.flip.
FLIP_CODES = {"horizontal": 1, "vertikal": 0, "180": -1}


def _read_text(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def _device_name(idx: int) -> str:
    """What the kernel says about /dev/videoN, e.g. 'USB 2.0 Camera'."""
    return _read_text(f"/sys/class/video4linux/video{idx}/name")


@dataclass(frozen=True)
class UsbDevice:
    """A connected USB camera, as the kernel currently sees it."""

    index: int       # /dev/videoN -- changes after re-plugging
    usb_id: str      # "046d:0825", as in lsusb
    usb_port: str   # USB port, e.g. "1-1.1.2" -- fixed as long as it stays plugged there
    name: str        # e.g. "Logi C270 HD WebCam"


def usb_cameras() -> list[UsbDevice]:
    """All USB cameras that can currently deliver images.

    A Pi 5 comes with about twenty ``/dev/video*``, most of which are codec
    and image processing units -- those are filtered out, as is everything
    without a USB ID.
    """
    import glob
    import re

    found: list[UsbDevice] = []
    for path in glob.glob("/sys/class/video4linux/video*"):
        hits = re.search(r"video(\d+)$", path)
        if not hits:
            continue
        idx = int(hits.group(1))
        name = _device_name(idx)
        if any(word in name.lower() for word in NOT_A_CAMERA):
            continue
        # UVC creates two nodes per camera: index 0 delivers images, index 1
        # only metadata. The second can be opened but never delivers an
        # image -- and would otherwise look like a second, broken camera.
        if _read_text(f"{path}/index") not in ("", "0"):
            continue
        # "device" points to the USB interface (e.g. 1-1.1.2:1.0);
        # vendor and product are one level up, at the device itself.
        usb = os.path.dirname(os.path.realpath(f"{path}/device"))
        vendor = _read_text(f"{usb}/idVendor")
        if not vendor:
            continue
        product = _read_text(f"{usb}/idProduct")
        found.append(
            UsbDevice(idx, f"{vendor}:{product}".lower(), os.path.basename(usb), name)
        )
    return sorted(found, key=lambda g: g.index)


class OpenCVCamera(CameraBase):
    """USB camera, found by USB ID and/or port.

    Without either, it takes the first USB camera that can be opened.
    """

    name = "usb"
    label = "USB-Kamera"

    def __init__(
        self, usb_id: str | None = None, usb_port: str | None = None, **kwargs
    ) -> None:
        super().__init__(**kwargs)
        self._usb_id = usb_id.lower() if usb_id else None
        self._usb_port = usb_port
        self._index: int | None = None
        self._cap = None

    @property
    def device(self) -> str | None:
        return f"/dev/video{self._index}" if self._index is not None else None

    def matches(self, device: UsbDevice) -> bool:
        if self._usb_id and not device.usb_id.startswith(self._usb_id):
            return False
        if self._usb_port and device.usb_port != self._usb_port:
            return False
        return True

    def _wanted(self) -> str:
        parts = []
        if self._usb_id:
            parts.append(f"USB {self._usb_id}")
        if self._usb_port:
            parts.append(f"Anschluss {self._usb_port}")
        return ", ".join(parts) or "beliebige USB-Kamera"

    def _open(self) -> None:
        import cv2

        # Search again on every open: after a USB dropout the camera often
        # gets a different /dev/videoN number.
        candidates = [g for g in usb_cameras() if self.matches(g)]
        if not candidates:
            raise RuntimeError(f"nicht angeschlossen ({self._wanted()})")

        tried: list[str] = []
        for device in candidates:
            idx = device.index
            cap, api = self._open_device(cv2, idx)
            if cap is None:
                tried.append(f"video{idx} ({device.name}): öffnet nicht")
                continue

            chosen = self._find_setting(cv2, cap, idx, tried)
            if chosen is None:
                cap.release()
                continue

            self._cap = cap
            self._index = idx
            log.info(
                "Camera '%s' (USB %s, /dev/video%d, %s) ready -- %s, %s",
                self.name,
                device.usb_id,
                idx,
                device.name,
                api,
                chosen,
            )
            return

        raise RuntimeError("keine brauchbare Kamera unter " + "; ".join(tried))

    @staticmethod
    def _open_device(cv2, idx: int):
        """Opens the device, preferably via V4L2.

        Without an explicit choice, OpenCV on the Pi tends to pick a
        GStreamer path that fails on a plain USB camera -- silently, with
        ``isOpened() == False``.
        """
        for name, api in (("V4L2", cv2.CAP_V4L2), ("Standard", cv2.CAP_ANY)):
            cap = cv2.VideoCapture(idx, api)
            if cap.isOpened():
                return cap, name
            cap.release()
        return None, ""

    def _find_setting(self, cv2, cap, idx: int, tried: list[str]) -> str | None:
        """Tries format and resolution combinations until an image arrives.

        There is no setting all cameras like: MJPG saves some (otherwise the
        USB bandwidth is not enough) and breaks others that do not support
        it at all. Hence trying in order from the best to the least
        demanding variant.
        """
        w, h = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
        variants = [
            ("MJPG", w, h),
            (None, w, h),       # leave the format untouched
            ("YUYV", w, h),
            (None, 640, 480),
            (None, 320, 240),
        ]

        for fourcc, width, height in variants:
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # The camera always runs at full rate; throttling happens when
            # fetching (CAMERA_FPS). That way a changed frame rate takes
            # effect immediately without reopening the camera.
            cap.set(cv2.CAP_PROP_FPS, config.CAMERA_MAX_FPS)
            # Buffer only one image: otherwise, with throttled fetching,
            # read() returns the oldest of several and the image lags behind.
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            frame = self._warm_up(cap)
            if frame is not None:
                actual_h, actual_w = frame.shape[:2]
                return f"{fourcc or 'Standardformat'} {actual_w}x{actual_h}"

            tried.append(f"video{idx} {fourcc or 'Standardformat'} {width}x{height}: kein Bild")

        return None

    @staticmethod
    def _warm_up(cap, seconds: float = 1.5):
        """Reads repeatedly until an image arrives or the time runs out.

        A single read attempt is not enough: many USB cameras deliver
        nothing right after opening and need a few hundred milliseconds.
        Giving up immediately then looks as if the camera were broken.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ok, frame = cap.read()
            if ok and frame is not None and getattr(frame, "size", 0) > 0:
                return frame
            time.sleep(0.05)
        return None

    def _close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def read(self) -> Frame | None:
        if self._cap is None:
            return None
        import cv2

        ok, frame = self._cap.read()
        if not ok:
            return None
        # Flip here already, not only in the app: person and pose detection
        # get the same image, otherwise left and right would be swapped
        # when matching against the lidar.
        code = FLIP_CODES.get(config.CAMERA_ROLES.get(self.name, {}).get("flip"))
        if code is not None:
            frame = cv2.flip(frame, code)
        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), config.CAMERA_QUALITY]
        )
        if not ok:
            return None
        return buf.tobytes(), "image/jpeg"


# --- Test image without a camera -----------------------------------------------------------


def _png(width: int, height: int, rows: list[bytearray]) -> bytes:
    """Builds a PNG from RGB rows.

    Hand-built because no library is needed for it: zlib is enough, and
    the mock should run on any computer -- even without OpenCV or Pillow.
    """

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8 bit, RGB
    # Each row gets a leading filter byte (0 = no filter).
    raw = b"".join(b"\x00" + bytes(row) for row in rows)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


class MockCamera(CameraBase):
    """Generates a moving test image -- not a still.

    The motion is intentional: with a still image you cannot tell whether
    the stream is still running or froze long ago.
    """

    name = "mock"
    label = "Testbild"

    WIDTH = 320
    HEIGHT = 240

    def __init__(self, color: tuple[int, int, int] = (0x4D, 0xA3, 0xFF), **kwargs) -> None:
        super().__init__(**kwargs)
        # With several test images, one color per camera -- otherwise you
        # cannot tell in the app whether switching does anything at all.
        self._color = color
        self._frame = 0

    def _open(self) -> None:
        log.info("Camera '%s': test image (no real camera)", self.name)

    def _close(self) -> None:
        pass

    def read(self) -> Frame | None:
        if not self.opened:
            return None

        self._frame += 1
        # A bright bar moves horizontally across the image.
        pos = int((math.sin(self._frame / 12.0) * 0.5 + 0.5) * (self.WIDTH - 1))

        rows: list[bytearray] = []
        for y in range(self.HEIGHT):
            row = bytearray(self.WIDTH * 3)
            for x in range(self.WIDTH):
                i = x * 3
                if abs(x - pos) < 14:
                    row[i], row[i + 1], row[i + 2] = self._color
                else:
                    gray = 0x22 + (y * 24 // self.HEIGHT)
                    row[i] = row[i + 1] = row[i + 2] = gray
            rows.append(row)

        return _png(self.WIDTH, self.HEIGHT, rows), "image/png"


# --- Selection and background operation ---------------------------------------------------


#: Bar colors of the test images, one per camera.
_TEST_COLORS = [(0x4D, 0xA3, 0xFF), (0x3F, 0xD0, 0x7A), (0xF5, 0xA5, 0x24)]


def _csi_numbers() -> list[int]:
    """Numbers of the cameras on the CSI connector, empty without picamera2.

    Only real CSI cameras: on the Pi libcamera also lists USB cameras. If
    picamera2 claimed one of them, OpenCV could no longer get it
    -- "Device or resource busy", and the role would have no image.
    """
    try:
        from picamera2 import Picamera2

        infos = Picamera2.global_camera_info()
    except Exception:
        return []
    return [
        info.get("Num", n)
        for n, info in enumerate(infos)
        if "usb" not in str(info.get("Id", "")).lower()
    ]


def create_cameras(use_mock: bool = False) -> dict[str, CameraBase]:
    """All cameras, by name. They are only opened in ``CameraService``.

    The roles from ``config.CAMERA_ROLES`` are always included, even if
    their camera is currently missing: it is then searched for in the
    background. A USB dropout at startup should not disable person
    detection until the next restart.
    """
    roles = config.CAMERA_ROLES

    if use_mock:
        # One test image per role, so the app can be developed with
        # several cameras even without the robot.
        names = list(roles) or ["kamera"]
        return {
            name: MockCamera(
                color=_TEST_COLORS[i % len(_TEST_COLORS)],
                name=name,
                label=f"Testbild ({roles[name]['label']})" if name in roles else "Testbild",
                fov_deg=roles.get(name, {}).get("fov_deg"),
            )
            for i, name in enumerate(names)
        }

    cameras: dict[str, CameraBase] = {}
    found = usb_cameras()
    taken: set[str] = set()

    for name, role in roles.items():
        camera = OpenCVCamera(
            usb_id=role["usb_id"],
            name=name,
            label=role.get("label"),
            fov_deg=role.get("fov_deg"),
        )
        cameras[name] = camera
        hits = next(
            (g for g in found if camera.matches(g) and g.usb_port not in taken), None
        )
        if hits is None:
            log.warning(
                "Camera '%s' (%s, USB %s) not connected -- will keep searching",
                name, camera.label, role["usb_id"],
            )
        else:
            taken.add(hits.usb_port)

    # Everything without a role runs too -- pinned to its USB port so that
    # after a dropout it does not snatch a role's camera.
    for nr, device in enumerate((g for g in found if g.usb_port not in taken), 1):
        cameras[f"usb{nr}"] = OpenCVCamera(
            usb_port=device.usb_port, name=f"usb{nr}", label=device.name
        )

    for nr in _csi_numbers():
        name = "csi" if nr == 0 else f"csi{nr}"
        cameras[name] = Picamera2Camera(nr, name=name)

    if not cameras:
        log.warning("No camera found -- showing a test image")
        log.warning("Find the cause: python scripts/check_camera.py")
        cameras["kamera"] = MockCamera(name="kamera")

    log.info("Cameras: %s", ", ".join(f"{n} ({k.label})" for n, k in cameras.items()))
    return cameras


class CameraService:
    """Keeps a current image ready that several clients share.

    Without this layer every browser tab would open the camera again --
    which simply fails on most devices. Instead a background thread fetches
    images, and all streams access the latest one. As a side effect a
    second viewer costs almost nothing.
    """

    def __init__(self, camera: CameraBase) -> None:
        self._camera = camera
        self._latest: Frame | None = None
        # Capture time of the current image (time.monotonic). Pose and
        # following need to know *when* an image was taken in order to
        # subtract the robot's own motion since then.
        self._latest_time = 0.0
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        # For /camera/status -- so that "no image is coming" can be
        # answered without searching the log.
        self._frame_buffer = 0
        self._last_frame = 0.0
        self._latest_error: str | None = None
        self._reopened = 0
        # Actually achieved frame rate, smoothed. Shows what camera and Pi
        # really manage -- CAMERA_FPS is only the wish.
        self._fps_actual = 0.0
        self._reopen_requested = threading.Event()

    def reopen(self) -> None:
        """Reopen the camera on the next pass, e.g. for a new resolution.

        Only request it instead of opening directly: opening takes time and
        belongs in the camera thread, not in the caller (usually the
        server's event loop).
        """
        self._reopen_requested.set()

    def status(self) -> dict:
        """Short report on why an image is arriving or not."""
        with self._lock:
            frame = self._latest
            images = self._frame_buffer
            last = self._last_frame
            error = self._latest_error

        running = self._thread is not None and self._thread.is_alive()
        return {
            "kamera": self._camera.label,
            "geoeffnet": self._camera.opened,
            "thread_laeuft": running,
            "bilder_gesamt": images,
            "letztes_bild_vor_s": (
                round(time.monotonic() - last, 1) if last else None
            ),
            "bildgroesse_bytes": len(frame[0]) if frame else None,
            "format": frame[1] if frame else None,
            "letzter_fehler": error,
            "neu_geoeffnet": self._reopened,
            "name": self._camera.name,
            "geraet": self._camera.device,
            "fov_deg": self._camera.fov_deg,
            "fps_ist": round(self._fps_actual, 1) if running and self._camera.opened else 0.0,
            "fps_soll": config.CAMERA_FPS,
        }

    @property
    def name(self) -> str:
        return self._camera.name

    @property
    def label(self) -> str:
        return self._camera.label

    @property
    def fov_deg(self) -> float:
        return self._camera.fov_deg

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            # open() is deliberately repeatable and does nothing if the
            # camera is already open.
            self._camera.open()
        except Exception as exc:
            log.warning("Camera '%s' cannot be opened -- %s", self.name, exc)
            with self._lock:
                self._latest_error = f"Öffnen fehlgeschlagen: {exc}"
            # The thread starts anyway: it reopens the camera at growing
            # intervals. That way a camera that was missing or busy at
            # startup joins without restarting the server.

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name=f"kamera-{self.name}", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._camera.close()

    def latest(self) -> Frame | None:
        with self._lock:
            return self._latest

    def latest_with_time(self) -> tuple[Frame | None, float]:
        """Current image plus capture time (time.monotonic, 0.0 = none yet).

        A separate method instead of changing latest(): the streams rely on
        latest() returning only the image.
        """
        with self._lock:
            return self._latest, self._latest_time

    def _reopen(self, reason: str = "delivers nothing") -> None:
        log.warning("Camera '%s' %s -- reopening it", self.name, reason)
        try:
            self._camera.close()
        except Exception:
            log.debug("Closing the camera failed", exc_info=True)
        try:
            self._camera.open()
        except Exception as exc:
            with self._lock:
                self._latest_error = f"Neu öffnen: {type(exc).__name__}: {exc}"
            log.warning("Camera '%s' cannot be opened yet -- %s", self.name, exc)
            return
        self._reopened += 1
        log.warning("Camera '%s' opened", self.name)

    def _loop(self) -> None:
        error = 0
        last_change = time.monotonic()
        frozen_reported = False
        reopen_wait = config.CAMERA_REOPEN_S

        while not self._stop.is_set():
            begin = time.monotonic()
            try:
                frame = self._camera.read()
                if frame is None:
                    with self._lock:
                        # For a closed camera the reason (e.g. "nicht
                        # angeschlossen") is already there -- do not
                        # overwrite it with a meaningless "None".
                        if self._frame_buffer == 0 and self._camera.opened:
                            self._latest_error = "Kamera liefert None (kein Bild)"
                else:
                    with self._lock:
                        new = frame != self._latest
                        # Only replace on a real change: the stream uses
                        # the object to tell whether it has already sent
                        # an image. Assigning blindly would send every
                        # still image over Wi-Fi again.
                        if new:
                            self._latest = frame
                            self._latest_time = begin
                            self._frame_buffer += 1
                            self._last_frame = begin
                            self._latest_error = None
                    error = 0
                    if new:
                        distance = begin - last_change
                        if 0 < distance < 2.0:
                            self._fps_actual += 0.1 * (1.0 / distance - self._fps_actual)
                        last_change = begin
                        reopen_wait = config.CAMERA_REOPEN_S
                        if frozen_reported:
                            log.info("Camera '%s' is delivering images again", self.name)
                            frozen_reported = False
            except Exception as exc:
                error += 1
                with self._lock:
                    self._latest_error = f"{type(exc).__name__}: {exc}"
                # Single dropouts are normal. Report the first few anyway
                # -- otherwise a frozen image means a long search.
                if error <= 3 or error % 100 == 0:
                    log.exception(
                        "Camera '%s' delivers no image (attempt %d)", self.name, error
                    )

            # A frozen image looks like a running stream in the browser.
            # Without this message nobody notices the standstill.
            # A camera that was never opened has its own message.
            if (
                not frozen_reported
                and self._camera.opened
                and begin - last_change > 5.0
            ):
                log.warning(
                    "Camera '%s' without a new image for %.0f s -- stream frozen",
                    self.name,
                    begin - last_change,
                )
                frozen_reported = True

            # If the stream stays frozen longer, the device is usually gone
            # -- e.g. after a dropout on the USB hub. Then only reopening
            # helps; reading from the old device never delivers anything again.
            if begin - last_change > reopen_wait:
                self._reopen()
                last_change = time.monotonic()
                reopen_wait = min(reopen_wait * 2, config.CAMERA_REOPEN_MAX_S)
            elif self._reopen_requested.is_set():
                self._reopen_requested.clear()
                self._reopen("has new settings")
                last_change = time.monotonic()

            # Read it every time: the frame rate can be changed at runtime.
            interval = 1.0 / max(1, config.CAMERA_FPS)
            rest = interval - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)
