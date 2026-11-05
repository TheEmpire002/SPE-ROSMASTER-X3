"""Depth camera (Orbbec Astra): obstacles below and above the lidar plane.

The lidar only measures in its own plane, about 17 cm above the floor.
Anything lower -- chair legs, runners, cable ducts, bed frames -- it does
not see; the robot drives onto it and gets stuck. The depth camera looks
forward and measures the distance for every pixel. From that, this module
derives:

1. a point cloud in the robot frame (mounting height and tilt from config),
2. the **floor**: a plane through the points near the expected floor height.
   It compensates for mounting errors and the robot pitching. Without it,
   one degree of wrong tilt would already create 2.6 cm of "obstacle" at
   1.5 m.
3. **obstacle points**: everything between ``DEPTH_MIN_HEIGHT_M`` and
   ``DEPTH_MAX_HEIGHT_M`` above the floor, put onto a grid. A cell only
   counts from ``DEPTH_MIN_POINTS_PER_CELL`` points on -- outliers in the
   depth image should not stop the robot.
4. **free cells**: where the camera sees floor with nothing on it. The
   obstacle memory uses them to clear points again.

The result is 2D points like those from the lidar (x forward, y left). So
the drive guard, explorer and mapping drive can use them without special
cases.

The camera is accessed through Orbbec's OpenNI2 build via ctypes -- without
ROS and without an extra package. The PyPI package ``openni`` no longer
runs under Python 3.13 (ctypes enums), and the few calls are quickly
written by hand.

The camera only runs while someone needs it (``activate``): the projector
draws power, and on the X3 power on the USB hub means dropouts
(docs/hardware.md). If it fails, the service reconnects by itself.
"""

from __future__ import annotations

import ctypes as C
import logging
import math
import os
import threading
import time
from typing import NamedTuple

from .. import config

log = logging.getLogger(__name__)

# --- OpenNI2 (C interface, OniCAPI.h) -------------------------------------------

_ONI_API_VERSION = 2002          # 2.2
_SENSOR_DEPTH = 3
_FORMAT_DEPTH_1MM = 100
_PROP_HFOV = 1
_PROP_VFOV = 2
_PROP_VIDEO_MODE = 3
_PROP_MIRRORING = 7


class _Mode(C.Structure):
    _fields_ = [("format", C.c_int), ("breite", C.c_int), ("hoehe", C.c_int), ("fps", C.c_int)]


class _Frame(C.Structure):
    _fields_ = [
        ("groesse", C.c_int), ("daten", C.c_void_p), ("sensor", C.c_int),
        ("zeitstempel", C.c_uint64), ("nummer", C.c_int),
        ("breite", C.c_int), ("hoehe", C.c_int), ("modus", _Mode),
        ("zuschnitt", C.c_int), ("zuschnitt_x", C.c_int), ("zuschnitt_y", C.c_int),
        ("zeilenbreite", C.c_int),
    ]


class _SensorInfo(C.Structure):
    _fields_ = [("sensor", C.c_int), ("anzahl", C.c_int), ("modi", C.POINTER(_Mode))]


class _OpenNI:
    """The bare minimum of OpenNI2: one device, one depth stream."""

    _lib = None  # load and initialize once per process

    def __init__(self, path: str) -> None:
        if _OpenNI._lib is None:
            lib = C.CDLL(os.path.join(path, "libOpenNI2.so"))
            lib.oniGetExtendedError.restype = C.c_char_p
            lib.oniStreamGetSensorInfo.restype = C.POINTER(_SensorInfo)
            self._check(lib, lib.oniInitialize(_ONI_API_VERSION), "initialisieren")
            _OpenNI._lib = lib
        self._l = _OpenNI._lib
        self._device = C.c_void_p()
        self._stream = C.c_void_p()
        self.width = self.height = 0
        self.hfov = self.vfov = 0.0

    @staticmethod
    def _check(lib, status: int, what: str) -> None:
        if status != 0:
            error = lib.oniGetExtendedError() or b""
            raise OSError(f"OpenNI2 {what}: Status {status} {error.decode(errors='replace').strip()}")

    def open_device(self, width: int, height: int) -> None:
        l = self._l
        status = l.oniDeviceOpen(None, C.byref(self._device))
        if status != 0:
            # Even a failed open writes a pointer into it. Closing it later
            # with oniDeviceClose crashed OpenNI2 (segfault, server gone,
            # 04.11.2026) -- so forget it.
            self._device = C.c_void_p()
        self._check(l, status, "Gerät öffnen")
        self._check(l, l.oniDeviceCreateStream(self._device, _SENSOR_DEPTH, C.byref(self._stream)),
                      "Tiefenstrom anlegen")
        info = l.oniStreamGetSensorInfo(self._stream).contents
        modes = [info.modes[i] for i in range(info.count)]
        matching = [m for m in modes if m.format == _FORMAT_DEPTH_1MM
                   and m.width == width and m.height == height]
        if matching:
            mode = _Mode(matching[0].format, matching[0].width, matching[0].height, matching[0].fps)
            self._check(l, l.oniStreamSetProperty(self._stream, _PROP_VIDEO_MODE, C.byref(mode),
                                                    C.sizeof(mode)), "Videomodus setzen")
        # Do not mirror: left in the image should be left of the robot.
        out = C.c_int(0)
        l.oniStreamSetProperty(self._stream, _PROP_MIRRORING, C.byref(out), C.sizeof(out))
        value, size = C.c_float(), C.c_int(C.sizeof(C.c_float))
        self._check(l, l.oniStreamGetProperty(self._stream, _PROP_HFOV, C.byref(value), C.byref(size)),
                      "Bildwinkel lesen")
        self.hfov = float(value.value)
        self._check(l, l.oniStreamGetProperty(self._stream, _PROP_VFOV, C.byref(value), C.byref(size)),
                      "Bildwinkel lesen")
        self.vfov = float(value.value)
        self._check(l, l.oniStreamStart(self._stream), "Tiefenstrom starten")

    def read(self, deadline_ms: int = 500):
        """Next depth image as a uint16 array (mm, 0 = invalid), None on timeout."""
        import numpy as np

        l = self._l
        index = C.c_int()
        streams = (C.c_void_p * 1)(self._stream)
        if l.oniWaitForAnyStream(streams, 1, C.byref(index), deadline_ms) != 0:
            return None
        pointer = C.POINTER(_Frame)()
        self._check(l, l.oniStreamReadFrame(self._stream, C.byref(pointer)), "Bild lesen")
        try:
            b = pointer.contents
            data = np.ctypeslib.as_array(C.cast(b.data, C.POINTER(C.c_uint16)),
                                          shape=(b.height, b.stride // 2))
            image = data[:, :b.width].copy()
        finally:
            l.oniFrameRelease(pointer)
        self.width, self.height = image.shape[1], image.shape[0]
        return image

    def close(self) -> None:
        l = self._l
        if self._stream:
            l.oniStreamStop(self._stream)
            l.oniStreamDestroy(self._stream)
            self._stream = C.c_void_p()
        if self._device:
            l.oniDeviceClose(self._device)
            self._device = C.c_void_p()


# --- Evaluation ----------------------------------------------------------------------


class DepthFrame(NamedTuple):
    """An evaluated depth image, everything in the robot frame (x forward, y left)."""

    timestamp: float
    obstacles: object          # Nx2: centers of the grid cells with an obstacle
    free: object                 # Mx2: centers of the cells containing only floor
    floor: tuple | None          # (a, b, c) of the plane z = a*x + b*y + c, None = estimated
    # For the display (/tiefe/stream): raw image in mm and which pixels
    # counted as obstacle (every second pixel, i.e. half resolution).
    raw: object = None
    mask: object = None


def evaluate(depth_mm, hfov: float, vfov: float, prev_floor=None):
    """Depth image -> (obstacle points, free cells, floor plane, newly detected?,
    obstacle mask).

    Pure computation, testable without a camera. ``prev_floor`` is the most
    recently detected plane; if none is found this time, it stays valid
    (then the fourth value is False). The mask has half the image resolution
    and marks every pixel that counted as obstacle -- for the display only.
    """
    import numpy as np

    step = 2  # every second pixel is enough for 5 cm cells
    t = depth_mm[::step, ::step].astype(np.float32) / 1000.0
    h_px, w_px = depth_mm.shape
    v, u = np.mgrid[0:h_px:step, 0:w_px:step]
    valid = t > 0.0
    z_c = t[valid]
    fx = (w_px / 2.0) / math.tan(hfov / 2.0)
    fy = (h_px / 2.0) / math.tan(vfov / 2.0)
    x_c = (u[valid] - w_px / 2.0) * z_c / fx   # right
    y_c = (v[valid] - h_px / 2.0) * z_c / fy   # down

    # Camera -> robot frame: forward = viewing direction, left = -right, up = -down,
    # then rotate down by the tilt and shift to the mounting position.
    p = math.radians(config.DEPTH_CAMERA_TILT_DEG)
    xr, zr = z_c, -y_c
    x = xr * math.cos(p) + zr * math.sin(p) + config.DEPTH_CAMERA_FORWARD_M
    y = -x_c
    z = -xr * math.sin(p) + zr * math.cos(p) + config.DEPTH_CAMERA_HEIGHT_M

    in_range = (x >= config.DEPTH_MIN_M) & (x <= config.DEPTH_MAX_M + 0.5)
    x, y, z = x[in_range], y[in_range], z[in_range]
    # Where in the (thinned) image each remaining point lies -- for the mask.
    position = np.flatnonzero(valid)[in_range]

    # Floor: plane through the points near the expected floor height, twice,
    # with fewer outliers each time. Only accept it if it is plausible.
    floor = None
    candidate = np.abs(z) < 0.08
    for _ in range(2):
        if candidate.sum() < 200:
            break
        A = np.column_stack((x[candidate], y[candidate], np.ones(int(candidate.sum()))))
        solution, *_ = np.linalg.lstsq(A, z[candidate], rcond=None)
        rest = z - (solution[0] * x + solution[1] * y + solution[2])
        candidate = candidate & (np.abs(rest) < 0.025)
        floor = solution
    if floor is not None and (abs(floor[0]) > 0.15 or abs(floor[1]) > 0.15 or abs(floor[2]) > 0.08
                              or candidate.sum() < 200):
        floor = None
    new = floor is not None
    if floor is None:
        floor = prev_floor
    a, b, c = floor if floor is not None else (0.0, 0.0, 0.0)
    height = z - (a * x + b * y + c)

    cell = config.DEPTH_CELL_M
    min_height, max_range = config.DEPTH_MIN_HEIGHT_M, config.DEPTH_MAX_M
    if floor is None:
        # Floor never seen (dark, glossy floor; robot raised up): then only
        # the mounting values count, and their error grows with distance.
        # Better to miss a flat obstacle than to turn the floor itself into
        # an obstacle.
        min_height += config.DEPTH_NO_FLOOR_MARGIN_M
        max_range = min(max_range, config.DEPTH_NO_FLOOR_MAX_M)
    close = x <= max_range

    def cells(mask):
        """Centers of the cells with enough points from ``mask``."""
        if not mask.any():
            return np.zeros((0, 2)), set()
        ix = np.floor(x[mask] / cell).astype(np.int32)
        iy = np.floor(y[mask] / cell).astype(np.int32)
        key, count = np.unique(np.column_stack((ix, iy)), axis=0, return_counts=True)
        full = key[count >= config.DEPTH_MIN_POINTS_PER_CELL]
        return (full + 0.5) * cell, {(int(i), int(j)) for i, j in full}

    obstacle = close & (height >= min_height) & (height <= config.DEPTH_MAX_HEIGHT_M)
    obstacles, occupied = cells(obstacle)
    mask = np.zeros(t.shape, dtype=bool)
    mask.flat[position[obstacle]] = True
    free, free_cells = cells(close & (np.abs(height) < 0.02))
    if len(free) and occupied:
        ix = np.floor(free[:, 0] / cell).astype(int)
        iy = np.floor(free[:, 1] / cell).astype(int)
        not_occupied = np.array([(i, j) not in occupied for i, j in zip(ix, iy)])
        free = free[not_occupied]
    floor = tuple(float(w) for w in floor) if floor is not None else None
    return obstacles, free, floor, new, mask


# --- Service --------------------------------------------------------------------------


class DepthCamera:
    """Reads the camera in the background while someone needs it."""

    def __init__(self) -> None:
        self._path = os.path.expanduser(config.DEPTH_OPENNI_PATH)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._requesters: set[str] = set()
        self._depth_frame: DepthFrame | None = None
        self._floor = None
        self._floor_time = 0.0
        self._connected = False
        self._error: str | None = None
        self._frame_buffer = 0
        self._fps = 0.0
        self._hfov = self._vfov = 0.0

    @property
    def available(self) -> bool:
        """Whether the library is present -- without it there is no depth."""
        return config.DEPTH_ACTIVE and os.path.exists(os.path.join(self._path, "libOpenNI2.so"))

    @property
    def hfov(self) -> float:
        """Horizontal field of view (radians), 0 as long as never connected."""
        return self._hfov

    def start(self) -> None:
        if self._thread is not None or not self.available:
            return
        self._thread = threading.Thread(target=self._loop, name="tiefe", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def activate(self, who: str, an: bool) -> None:
        """Register or unregister the camera for ``who``. It runs while anyone wants it."""
        with self._lock:
            if an:
                self._requesters.add(who)
            else:
                self._requesters.discard(who)
                if not self._requesters:
                    self._depth_frame = None
        self._wake.set()

    def image(self) -> DepthFrame | None:
        """Latest evaluated image, None if too old or none."""
        with self._lock:
            b = self._depth_frame
        if b is None or time.monotonic() - b.timestamp > config.DEPTH_MAX_AGE_S:
            return None
        return b

    def status(self) -> dict:
        b = self.image()
        with self._lock:
            who = sorted(self._requesters)
        floor = b.floor if b is not None else None
        # Nearest obstacle point (x forward, y left) -- to check that left and
        # right are correct: hand in front of the camera on the left, then y
        # must be positive.
        nearest = None
        if b is not None and len(b.obstacles):
            import numpy as np

            i = int(np.argmin(np.hypot(b.obstacles[:, 0], b.obstacles[:, 1])))
            nearest = [round(float(b.obstacles[i, 0]), 2), round(float(b.obstacles[i, 1]), 2)]
        return {
            "verfuegbar": self.available,
            "verbunden": self._connected,
            "aktiv_fuer": who,
            "bilder_pro_s": round(self._fps, 1),
            "hindernisse": int(len(b.obstacles)) if b is not None else None,
            "frei_zellen": int(len(b.free)) if b is not None else None,
            "naechstes": nearest,
            # Detected mounting pose: camera height above the floor and tilt.
            "boden_erkannt": floor is not None,
            "kamera_hoehe_m": round(config.DEPTH_CAMERA_HEIGHT_M - floor[2], 3) if floor else None,
            "neigung_grad": round(config.DEPTH_CAMERA_TILT_DEG + math.degrees(math.atan(floor[0])), 1)
            if floor else None,
            "bildwinkel_grad": round(math.degrees(self._hfov), 1) if self._hfov else None,
            "fehler": self._error,
        }

    def _needed(self) -> bool:
        with self._lock:
            return bool(self._requesters)

    def _loop(self) -> None:
        camera: _OpenNI | None = None
        last_eval = 0.0
        counter_start, counter = time.monotonic(), 0
        while not self._stop.is_set():
            if not self._needed():
                if camera is not None:
                    # Switch off, not just stop reading: the projector draws power.
                    self._close(camera)
                    camera = None
                    log.info("Depth camera off (nobody needs it)")
                self._wake.wait(1.0)
                self._wake.clear()
                continue
            try:
                if camera is None:
                    camera = _OpenNI(self._path)
                    camera.open_device(config.DEPTH_WIDTH, config.DEPTH_HEIGHT_PX)
                    self._hfov, self._vfov = camera.hfov, camera.vfov
                    self._connected = True
                    self._error = None
                    log.info("Depth camera on (field of view %.0f° x %.0f°)",
                             math.degrees(camera.hfov), math.degrees(camera.vfov))
                depth = camera.read()
                if depth is None:
                    raise OSError("liefert keine Bilder")
                now = time.monotonic()
                # The camera delivers 30 frames/s; evaluation runs less often.
                # Every frame must still be read, otherwise they pile up.
                if now - last_eval < 1.0 / max(0.5, config.DEPTH_HZ):
                    continue
                last_eval = now
                prev_floor = self._floor if now - self._floor_time < 10.0 else None
                obstacles, free, floor, new, mask = evaluate(
                    depth, camera.hfov, camera.vfov, prev_floor)
                if new:
                    self._floor, self._floor_time = floor, now
                with self._lock:
                    if self._requesters:
                        self._depth_frame = DepthFrame(now, obstacles, free, floor, depth, mask)
                counter += 1
                if now - counter_start >= 2.0:
                    self._fps = counter / (now - counter_start)
                    counter_start, counter = now, 0
            except Exception as exc:
                # USB dropout, camera unplugged, library missing: close, wait,
                # retry. A failure must never disturb the server.
                if self._error != str(exc):
                    log.warning("Depth camera: %s -- retrying in %.0f s", exc,
                                config.DEPTH_RETRY_S)
                self._error = str(exc)
                self._connected = False
                with self._lock:
                    self._depth_frame = None
                if camera is not None:
                    self._close(camera)
                    camera = None
                self._stop.wait(config.DEPTH_RETRY_S)
        if camera is not None:
            self._close(camera)

    def _close(self, camera: _OpenNI) -> None:
        try:
            camera.close()
        except Exception:
            log.debug("Closing the depth camera failed", exc_info=True)
        self._connected = False


def create_depth(use_mock: bool = False) -> DepthCamera | None:
    """On the robot the Astra, in the simulation no depth.

    Deliberately no made-up depth in the simulation: it would show obstacles
    that do not exist. Everything that uses depth works without it.
    """
    if use_mock or not config.DEPTH_ACTIVE:
        return None
    camera = DepthCamera()
    if not camera.available:
        log.warning("Depth camera: no OpenNI2 library in %s -- depth off "
                    "(scripts/setup_orbbec.sh)", camera._path)
        return None
    return camera
