"""3D map: what the depth camera saw, as blocks in the map frame.

While the pose in a map is confident, every depth image (Orbbec Astra,
``sensors/depth.py``) is turned into points in the robot frame, moved to the
robot's map pose at the moment the image was taken, and counted into blocks
of ``MAP3D_VOXEL_M``. A block is shown from ``MAP3D_MIN_HITS`` images on, so
single noisy pixels do not show up.

The pose is only 2D (x, y, heading): the robot drives on a flat floor, and
the floor plane the depth evaluation finds takes care of the camera tipping
while driving. The pose comes from the localization at tick time and is
moved back to the image's capture time with the wheel odometry, otherwise
turning would smear the walls.

The depth camera only runs while someone needs it (its projector draws a lot
of current on the usb hub, docs/hardware.md). "3D aufnehmen" on /map switches
it on for this module; with ``MAP3D_WHEN_DEPTH_ON`` the map also grows
whenever another user (autonomous driving, mapping drive) has it on anyway.

Colour (new 24.11.): the colour image of the same Orbbec (``MAP3D_COLOR_CAMERA``)
taken at about the same moment gives every point a colour, with the same
geometry as the overlay view /tiefe/ueberlagert (``depth_view.overlay_box``):
what lines up there lines up here, and ``DEPTH_OVERLAY_SHIFT_PX``/``ZOOM``
correct both. Per block a running average over the last
``MAP3D_COLOR_AVG_N`` images, so changing light and auto exposure even out.

Surface direction (test branch feat/3d-surfaces): every point gets a normal
from its neighbours in the depth image (cross product of the steps to the
right and down neighbour; none across depth jumps, ``MAP3D_NORMAL_MAX_JUMP_M``).
Per block a running average like the colour. The viewer draws each block as
a round disc facing that direction, which looks like a surface instead of
cubes.

Per map one file ``<name>_3d.npz`` beside the map files: packed block keys,
hit counts, colours, normals, block size and the map's frame id. Like the
photos it is set aside as ``<name>_3d.<stamp>.alt.npz`` when the main map is
replaced, the old blocks would sit in the wrong places.

Raw recording (test branch feat/3d-surfaces): for a real surface model the
blocks are too coarse. "Rohaufnahme" stores every image that goes into the
blocks as it is, in ``<name>_roh/<stamp>/``: the depth image (16-bit png,
mm), the colour image used (jpeg), and one line per image in
``frames.jsonl`` with the map pose at capture time and the floor plane;
``meta.json`` holds fields of view, mounting and the overlay box.
``scripts/build_mesh.py`` on a pc fuses them into a coloured mesh (open3d
tsdf). A writer thread does the files, the sd card must not hold up tick.

Live stream (test branch feat/3d-surfaces): the same images also go to every
pc connected to the websocket /api/mapping/raum3d/live, which builds the mesh
while the robot drives (``scripts/build_mesh.py --live``). One small queue per
listener; a slow pc loses images, the robot never waits for it.

``tick`` runs in a worker thread (map_control's tick loop), the endpoints in
the event loop; everything shared is behind ``_lock``. File access (load,
save, delete) is serialized by ``_io_lock``, so a delete cannot be undone by
a save that already copied the old blocks. Saving runs in its own thread,
compressing 400k blocks must not hold up the localization.
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import shutil
import tempfile
import threading
import time
import zipfile
from pathlib import Path
from typing import Any

from .. import config
from ..navigation.map_localization import compose, invert
from .photos import safe_name

log = logging.getLogger(__name__)

# block indices packed into one int64: 21 bits per axis, offset to be positive
_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1
_MAX_HITS = 65535
# colour stored as 0x1RRGGBB: bit 24 marks "has a colour", 0 = none yet
_HAS_COLOR = 1 << 24
# normal stored the same way: three bytes (component * 127 + 128), bit 24 = has one
_HAS_NORMAL = 1 << 24


def pack(ix, iy, iz):
    """Block indices (numpy int64 arrays) -> one key per block."""
    return ((ix + _OFF) << (2 * _BITS)) | ((iy + _OFF) << _BITS) | (iz + _OFF)


def unpack(keys):
    """Keys -> (ix, iy, iz), numpy int64 arrays."""
    return ((keys >> (2 * _BITS)) & _MASK) - _OFF, ((keys >> _BITS) & _MASK) - _OFF, (keys & _MASK) - _OFF


def _blend(old: int, new: int, n: int) -> int:
    """Running average of two 0x1RRGGBB colours, the new one with weight 1/n"""
    if not old:
        return new
    out = _HAS_COLOR
    for shift in (16, 8, 0):
        a, b = (old >> shift) & 255, (new >> shift) & 255
        out |= int(round(a + (b - a) / n)) << shift
    return out


def _pack_normal(n) -> int:
    return _HAS_NORMAL | sum(int(round(max(-1.0, min(1.0, v)) * 127.0)) + 128 << s
                             for v, s in zip(n, (16, 8, 0)))


def _unpack_normal(v: int) -> tuple[float, float, float]:
    return tuple(((v >> s) & 255) - 128 for s in (16, 8, 0))  # type: ignore[return-value]


def _blend_normal(old: int, new: int, n: int) -> int:
    """Running average of two packed normals, renormalized; new one with weight 1/n"""
    if not old:
        return new
    a, b = _unpack_normal(old), _unpack_normal(new)
    m = [x + (y - x) / n for x, y in zip(a, b)]
    length = math.sqrt(sum(v * v for v in m))
    if length < 1e-6:
        return new
    return _pack_normal([v / length for v in m])


class Room3D:
    def __init__(self, storage_dir: Path, localization, depth, odometry=None,
                 color_camera=None) -> None:
        self._dir = Path(storage_dir)
        self._localization = localization
        self._depth = depth
        self._odometry = odometry
        self._color_camera = color_camera
        self._lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._saver: threading.Thread | None = None
        self._map: tuple[str, str | None] | None = None
        self._hits: dict[int, int] = {}
        self._colors: dict[int, int] = {}
        self._normals: dict[int, int] = {}
        self._shown = 0
        self._colored = 0           # images of the last minute that had a colour image
        self._last_frame = 0.0
        self._last_time = 0.0
        self._dirty = False
        self._save_time = time.monotonic()
        self.recording = False
        # bumped when the shown blocks change; the viewer reloads when it moves
        self.stand = 0
        self.text = "noch nichts aufgenommen"
        self.last_error: str | None = None
        self._full = False
        # raw recording: session folder, frames written, and the writer
        self._raw: dict[str, Any] | None = None
        self._raw_queue: queue.Queue = queue.Queue(maxsize=config.MAP3D_RAW_QUEUE)
        self._raw_thread: threading.Thread | None = None
        # colour image the last integrated depth image used (jpeg, box, size)
        self._last_color: tuple[bytes, tuple, tuple] | None = None
        # live listeners (websocket clients), one queue each
        self._live: list[queue.Queue] = []
        self._live_lock = threading.Lock()
        self._live_meta: tuple[Any, dict] | None = None    # (map, meta) sent with the images
        if config.MAP3D_RECORD:
            self.set_recording(True)

    @property
    def available(self) -> bool:
        return bool(config.MAP3D_ENABLED) and self._depth is not None and self._depth.available

    # --- switching ---

    def set_recording(self, on: bool) -> None:
        """Keep the depth camera on for the 3d map (or release it)."""
        self.recording = bool(on) and self.available
        if self._depth is not None:
            self._depth.activate("raum3d", self.recording)

    def close(self) -> None:
        """Server shutdown: release the camera, write what is still open."""
        try:
            self.stop_raw()
            # let the writer finish what is queued (a few images)
            deadline = time.monotonic() + config.MAP3D_RAW_DRAIN_S
            while self._raw_queue.unfinished_tasks and time.monotonic() < deadline:
                time.sleep(0.05)
            if self._depth is not None:
                self._depth.activate("raum3d", False)
            saver = self._saver
            if saver is not None:
                saver.join(timeout=5.0)
            self._save()
        except Exception:
            log.warning("3D map: closing failed", exc_info=True)

    def clear(self) -> None:
        """Delete the 3d map of the current map."""
        with self._io_lock:
            with self._lock:
                self._hits = {}
                self._colors = {}
                self._normals = {}
                self._shown = 0
                self._full = False
                self._dirty = False
                self.stand += 1
                current = self._map
            if current is not None:
                try:
                    self._file(current[0]).unlink()
                except FileNotFoundError:
                    pass
                except OSError as exc:
                    log.warning("3D map: deleting failed: %s", exc)

    # --- which map (as for the photos) ---

    def _current_map(self) -> tuple[str, str | None] | None:
        loc = self._localization
        occ_map = loc.occ_map
        if occ_map is None or loc.source not in ("dauer", "geladen"):
            return None
        name = getattr(occ_map, "name", None)
        if not name or not safe_name(str(name)):
            return None
        return str(name), getattr(occ_map, "frame_id", None)

    def _file(self, name: str) -> Path:
        return self._dir / f"{name}_3d.npz"

    def _switch(self, current: tuple[str, str | None]) -> None:
        if current == self._map:
            return
        self._save()  # the old map's blocks first
        with self._io_lock:
            self._load(current)

    def _load(self, current: tuple[str, str | None]) -> None:
        import numpy as np

        name, frame_id = current
        path = self._file(name)
        hits: dict[int, int] = {}
        colors: dict[int, int] = {}
        normals: dict[int, int] = {}
        try:
            with np.load(path, allow_pickle=False) as data:
                saved_frame = str(data["frame_id"]) if "frame_id" in data.files else ""
                voxel = float(data["voxel_m"])
                keys, counts = data["keys"], data["hits"]
                # files from before the colour have none
                rgb = data["rgb"] if "rgb" in data.files else None
                nor = data["normal"] if "normal" in data.files else None
            wrong_frame = bool(frame_id and saved_frame and saved_frame != frame_id)
            if wrong_frame or abs(voxel - config.MAP3D_VOXEL_M) > 1e-6:
                aside = path.with_name(f"{name}_3d.{time.strftime('%Y%m%d-%H%M%S')}.alt.npz")
                os.replace(path, aside)
                log.warning("3D map of '%s' belongs to an older map or block size -- moved to %s",
                            name, aside.name)
            else:
                hits = dict(zip(keys.tolist(), counts.tolist()))
                if rgb is not None:
                    colors = {k: c for k, c in zip(keys.tolist(), rgb.tolist()) if c}
                if nor is not None:
                    normals = {k: c for k, c in zip(keys.tolist(), nor.tolist()) if c}
        except FileNotFoundError:
            pass
        except Exception:
            log.warning("3D map of '%s' unreadable -- starting a new one", name, exc_info=True)
        with self._lock:
            self._map = current
            self._hits = hits
            self._colors = colors
            self._normals = normals
            self._shown = sum(1 for h in hits.values() if h >= config.MAP3D_MIN_HITS)
            self._full = len(hits) >= config.MAP3D_MAX_VOXELS
            self._dirty = False
            self.stand += 1

    # --- recording ---

    def tick(self) -> None:
        """Add the newest depth image if the pose is good enough (worker thread)."""
        if not self.available:
            return
        current = self._current_map()
        if current is None:
            self.text = "keine Karte, in der er seine Lage kennt"
            return
        self._switch(current)
        now = time.monotonic()
        if self._dirty and now - self._save_time >= config.MAP3D_SAVE_S and (
                self._saver is None or not self._saver.is_alive()):
            self._save_time = now
            self._saver = threading.Thread(target=self._save, name="raum3d-speichern", daemon=True)
            self._saver.start()
        if not self.recording and not config.MAP3D_WHEN_DEPTH_ON:
            self.text = "Aufnahme aus"
            return
        frame = self._depth.image()
        if frame is None or frame.raw is None:
            self.text = "wartet auf die Tiefenkamera" if self.recording else "Aufnahme aus"
            return
        if frame.timestamp == self._last_frame or now - self._last_time < 1.0 / config.MAP3D_HZ:
            return
        self._last_frame = frame.timestamp
        if now - frame.timestamp > config.MAP3D_MAX_FRAME_AGE_S:
            return
        if not self._localization.confident:
            self.text = "Lage in der Karte unsicher -- nimmt nichts auf"
            return
        if self._odometry is not None and abs(self._odometry.velocity()[2]) > config.MAP3D_MAX_OMEGA:
            self.text = "dreht zu schnell -- nimmt nichts auf"
            return
        pose = self._pose_at(frame.timestamp)
        if pose is None:
            return
        self._last_time = now
        try:
            added = self._integrate(frame, pose)
        except Exception as exc:
            self.last_error = f"Auswertung: {exc}"
            log.warning("3D map: evaluation failed", exc_info=True)
            return
        self.last_error = None
        added, colored = added
        self._raw_frame(frame, pose)
        self._live_frame(frame, pose)
        self.text = ("Speicher voll -- ergänzt nur noch" if self._full
                     else f"nimmt auf ({added} Punkte im letzten Bild"
                          + ("" if colored or self._color_camera is None else ", ohne Farbbild") + ")")

    def _pose_at(self, instant: float) -> tuple[float, float, float] | None:
        """Map pose at the image's capture time: the pose now, moved back by
        the wheel odometry's motion since then"""
        pose = self._localization.pose()
        if pose is None or self._odometry is None:
            return pose
        then = self._odometry.pose_at(instant)
        if then is None:
            return pose
        x, y, deg = self._odometry.pose()
        delta = compose(invert((x, y, math.radians(deg))), then)
        return compose(pose, delta)

    def _point_colors(self, frame, rows, cols, step):
        """Colour (n x 3 rgb uint8) and has-colour mask for depth pixels at
        (rows, cols) of the thinned grid, or None without a matching colour image"""
        import numpy as np

        camera = self._color_camera
        if camera is None:
            return None
        data, taken = camera.latest_with_time()
        # the robot moves: a colour image from another moment paints the wrong things
        if data is None or not taken or abs(taken - frame.timestamp) > config.MAP3D_COLOR_MAX_DT_S:
            return None
        import cv2

        from ..sensors.depth_view import overlay_box

        image = cv2.imdecode(np.frombuffer(data[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None
        H, W = image.shape[:2]
        h, w = frame.raw.shape
        x0, y0, bw, bh = overlay_box(w, h, W, H, self._depth.hfov, getattr(camera, "fov_deg", None))
        self._last_color = (data[0], (x0, y0, bw, bh), (W, H))
        px = (x0 + (cols * step + 0.5) * bw / w).astype(np.int64)
        py = (y0 + (rows * step + 0.5) * bh / h).astype(np.int64)
        inside = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        rgb = np.zeros((len(px), 3), dtype=np.uint8)
        rgb[inside] = image[py[inside], px[inside], ::-1]
        return rgb, inside

    def _integrate(self, frame, pose) -> tuple[int, bool]:
        """Count one depth image into the blocks; (points used, had colour)"""
        import numpy as np

        from ..sensors.depth import robot_points

        hfov, vfov = self._depth.hfov, self._depth.vfov
        if not hfov or not vfov:
            return 0, False
        step = config.MAP3D_PIXEL_STEP
        self._last_color = None
        x, y, z, valid = robot_points(frame.raw, hfov, vfov, step=step)
        # same order as the boolean indexing in robot_points: row by row
        rows, cols = np.nonzero(valid)
        normal = _normals(np, x, y, z, valid, step)
        try:
            color = self._point_colors(frame, rows, cols, step)
        except Exception:
            log.debug("3D map: colour image unusable", exc_info=True)
            color = None
        keep = (x >= config.MAP3D_MIN_M) & (np.hypot(x, y) <= config.MAP3D_MAX_M)
        # height above the floor the camera sees, not above the mounting
        # guess: the robot tips a little while driving
        if frame.floor is not None:
            a, b, c = frame.floor
            z = z - (a * x + b * y + c)
        keep &= (z >= config.MAP3D_MIN_HEIGHT_M) & (z <= config.MAP3D_MAX_HEIGHT_M)
        x, y, z, normal = x[keep], y[keep], z[keep], normal[keep]
        if not len(x):
            return 0, color is not None
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = c * x - s * y + pose[0]
        wy = s * x + c * y + pose[1]
        # normals into the map frame: only the heading turns them
        nx = c * normal[:, 0] - s * normal[:, 1]
        ny = s * normal[:, 0] + c * normal[:, 1]
        normal = np.column_stack((nx, ny, normal[:, 2]))
        v = config.MAP3D_VOXEL_M
        keys, inverse = np.unique(
            pack(np.floor(wx / v).astype(np.int64), np.floor(wy / v).astype(np.int64),
                 np.floor(z / v).astype(np.int64)),
            return_inverse=True)
        inverse = inverse.ravel()
        # mean colour of the points per block in this image
        new_color = None
        if color is not None:
            rgb, has = color[0][keep], color[1][keep]
            n = np.bincount(inverse[has], minlength=len(keys))
            if n.any():
                sums = [np.bincount(inverse[has], weights=rgb[has, ch], minlength=len(keys)) for ch in range(3)]
                mean = np.stack(sums, axis=1) / np.maximum(n, 1)[:, None]
                new_color = np.where(n > 0, _HAS_COLOR | (mean[:, 0].astype(np.int64) << 16)
                                     | (mean[:, 1].astype(np.int64) << 8) | mean[:, 2].astype(np.int64), 0)
        # mean normal per block in this image (sum of unit vectors, renormalized)
        has_n = ~np.isnan(normal[:, 0])
        normal_list = None
        if has_n.any():
            sums = np.stack([np.bincount(inverse[has_n], weights=normal[has_n, ch], minlength=len(keys))
                             for ch in range(3)], axis=1)
            length = np.linalg.norm(sums, axis=1)
            good = length > 1e-6
            unit = sums / np.maximum(length, 1e-6)[:, None]
            packed = (_HAS_NORMAL | ((np.round(unit[:, 0] * 127) + 128).astype(np.int64) << 16)
                      | ((np.round(unit[:, 1] * 127) + 128).astype(np.int64) << 8)
                      | (np.round(unit[:, 2] * 127) + 128).astype(np.int64))
            normal_list = np.where(good, packed, 0).tolist()
        threshold = config.MAP3D_MIN_HITS
        average_n = config.MAP3D_COLOR_AVG_N
        new_list = new_color.tolist() if new_color is not None else None
        with self._lock:
            hits, colors, normals = self._hits, self._colors, self._normals
            for i, k in enumerate(keys.tolist()):
                h = hits.get(k)
                if h is None:
                    # full: only strengthen what is there, no new blocks
                    if len(hits) >= config.MAP3D_MAX_VOXELS:
                        self._full = True
                        continue
                    h = 0
                if h < _MAX_HITS:
                    hits[k] = h + 1
                    if h + 1 == threshold:
                        self._shown += 1
                if new_list is not None and new_list[i]:
                    colors[k] = _blend(colors.get(k, 0), new_list[i], min(h + 1, average_n))
                if normal_list is not None and normal_list[i]:
                    normals[k] = _blend_normal(normals.get(k, 0), normal_list[i], min(h + 1, average_n))
            self._dirty = True
            self.stand += 1
        return int(len(x)), color is not None

    # --- raw recording (for the surface model on a pc) ---

    def _raw_dir(self, name: str) -> Path:
        return self._dir / f"{name}_roh"

    def start_raw(self) -> str | None:
        """Start a raw recording in the current map; error text (german) or None"""
        if not self.available:
            return "Keine Tiefenkamera"
        current = self._current_map()
        if current is None:
            return "Keine Karte, in der er seine Lage kennt"
        if self._raw is not None:
            return None
        folder = self._raw_dir(current[0]) / time.strftime("%Y%m%d-%H%M%S")
        try:
            folder.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            return f"Ordner geht nicht: {exc}"
        self._raw = {"dir": folder, "map": current, "frames": 0, "meta": None}
        if self._raw_thread is None or not self._raw_thread.is_alive():
            self._raw_thread = threading.Thread(target=self._raw_writer, name="raum3d-roh", daemon=True)
            self._raw_thread.start()
        self.set_recording(True)
        log.info("3D map: raw recording in %s", folder)
        return None

    def stop_raw(self) -> None:
        if self._raw is None:
            return
        log.info("3D map: raw recording stopped after %d images", self._raw["frames"])
        self._raw = None
        # the writer finishes what is queued; nothing new comes in

    def _raw_frame(self, frame, pose) -> None:
        """Queue one integrated image for the raw recording (tick thread)"""
        raw = self._raw
        if raw is None:
            return
        if raw["map"] != self._map or raw["frames"] >= config.MAP3D_RAW_MAX_FRAMES:
            # other map or full: positions would not fit / sd card
            self.stop_raw()
            return
        if self._raw_thread is None or not self._raw_thread.is_alive():
            self._raw_thread = threading.Thread(target=self._raw_writer, name="raum3d-roh", daemon=True)
            self._raw_thread.start()
        index = raw["frames"]
        if not raw["meta"]:
            raw["meta"] = self._meta(frame, raw["map"])
        item = {
            "dir": raw["dir"], "index": index, "depth": frame.raw.copy(),
            "color": self._last_color, "pose": pose, "floor": frame.floor,
            "time": frame.timestamp, "meta": raw["meta"],
        }
        try:
            self._raw_queue.put_nowait(item)
        except queue.Full:
            return  # sd card too slow: skip this image rather than stall
        raw["frames"] = index + 1

    def _meta(self, frame, current) -> dict[str, Any]:
        """What a pc needs to place the images: sizes, fields of view, mounting,
        and the 2d map (the pc fills the floor from its free cells; the depth
        camera often does not see a shiny floor)"""
        h, w = frame.raw.shape
        return {
            "karte": current[0], "frame_id": current[1],
            "tiefe_breite": w, "tiefe_hoehe": h,
            "hfov": self._depth.hfov, "vfov": self._depth.vfov,
            "kamera_hoehe_m": config.DEPTH_CAMERA_HEIGHT_M,
            "kamera_vorn_m": config.DEPTH_CAMERA_FORWARD_M,
            "kamera_neigung_grad": config.DEPTH_CAMERA_TILT_DEG,
            "min_m": config.MAP3D_MIN_M, "max_m": config.MAP3D_MAX_M,
            "karte2d": self._map2d(),
        }

    def _map2d(self) -> dict[str, Any] | None:
        """The 2d occupancy grid as in /api/mapping/map: int8 per cell (-1
        unknown, 0..100), row 0 = lowest y"""
        import base64

        from .occ_map import FREE_MAX, OCCUPIED_MIN

        occ = self._localization.occ_map
        if occ is None:
            return None
        return {
            "aufloesung": occ.resolution, "ursprung": list(occ.origin),
            "breite": occ.width, "hoehe": occ.height,
            "frei_max": FREE_MAX, "belegt_min": OCCUPIED_MIN,
            "data_b64": base64.b64encode(occ.occupancy.tobytes()).decode("ascii"),
        }

    # --- live stream to a pc ---

    def add_live(self) -> queue.Queue:
        """Register a listener; the depth camera stays on while anyone listens"""
        q: queue.Queue = queue.Queue(maxsize=config.MAP3D_LIVE_QUEUE)
        # camera switch under the lock: a leaving client must not switch it off
        # right after a new one switched it on
        with self._live_lock:
            self._live.append(q)
            if self._depth is not None:
                self._depth.activate("raum3d-live", True)
        return q

    def remove_live(self, q: queue.Queue) -> None:
        with self._live_lock:
            if q in self._live:
                self._live.remove(q)
            if not self._live and self._depth is not None:
                self._depth.activate("raum3d-live", False)

    @property
    def live_count(self) -> int:
        with self._live_lock:
            return len(self._live)

    def _live_frame(self, frame, pose) -> None:
        """Hand one integrated image to every listener (tick thread)"""
        with self._live_lock:
            listeners = list(self._live)
        if not listeners or self._map is None:
            return
        if self._live_meta is None or self._live_meta[0] != self._map:
            self._live_meta = (self._map, self._meta(frame, self._map))
        item = {"depth": frame.raw.copy(), "color": self._last_color, "pose": pose,
                "floor": frame.floor, "time": frame.timestamp, "meta": self._live_meta[1]}
        for q in listeners:
            try:
                q.put_nowait(item)
            except queue.Full:
                pass  # this pc is behind: it misses the image, the robot goes on

    def _raw_writer(self) -> None:
        while True:
            item = self._raw_queue.get()
            try:
                import cv2

                folder: Path = item["dir"]
                if not folder.is_dir():
                    continue  # session deleted meanwhile
                # until it is there: build_mesh needs it, one failed write must not lose it
                meta_file = folder / "meta.json"
                if not meta_file.exists():
                    meta_file.write_text(json.dumps(item["meta"], indent=1), encoding="utf-8")
                stem = f"{item['index']:05d}"
                if not cv2.imwrite(str(folder / f"{stem}_tiefe.png"), item["depth"]):
                    raise OSError("depth png not written (sd card full?)")
                line = {"i": item["index"], "t": round(item["time"], 3),
                        "pose": [round(v, 4) for v in item["pose"]],
                        "boden": [round(v, 5) for v in item["floor"]] if item["floor"] else None,
                        "farbe": None}
                if item["color"] is not None:
                    jpeg, box, size = item["color"]
                    (folder / f"{stem}_farbe.jpg").write_bytes(jpeg)
                    line["farbe"] = {"box": list(box), "groesse": list(size)}
                with open(folder / "frames.jsonl", "a", encoding="utf-8") as f:
                    f.write(json.dumps(line) + "\n")
            except Exception as exc:
                self.last_error = f"Rohaufnahme: {exc}"
                log.warning("3D map: raw image not written: %s", exc)
            finally:
                self._raw_queue.task_done()

    def raw_sessions(self) -> dict[str, Any]:
        """Raw recordings of the current map: name, images, size"""
        current = self._current_map()
        out: list[dict[str, Any]] = []
        if current is not None:
            base = self._raw_dir(current[0])
            for folder in sorted(base.glob("*"), reverse=True) if base.is_dir() else []:
                if not folder.is_dir():
                    continue
                files = list(folder.iterdir())
                out.append({
                    "name": folder.name,
                    "bilder": sum(1 for f in files if f.name.endswith("_tiefe.png")),
                    "mb": round(sum(f.stat().st_size for f in files) / 1e6, 1),
                })
        raw = self._raw
        return {"karte": current[0] if current else None, "sitzungen": out,
                "laeuft": raw["dir"].name if raw is not None else None}

    def _raw_session(self, session: str) -> Path | None:
        current = self._current_map()
        if current is None or not safe_name(session) or session.startswith("."):
            return None
        base = self._raw_dir(current[0])
        folder = base / session
        # only a session folder directly inside <name>_roh, never the folder itself
        if folder.resolve().parent != base.resolve() or not folder.is_dir():
            return None
        return folder

    def raw_zip(self, session: str) -> Path | None:
        """The session as a zip in a temp file (caller deletes it); None if unknown"""
        folder = self._raw_session(session)
        if folder is None:
            return None
        fd, name = tempfile.mkstemp(suffix=".zip")
        os.close(fd)
        try:
            # png and jpeg are compressed already: store, do not deflate again
            with zipfile.ZipFile(name, "w", zipfile.ZIP_STORED) as z:
                for f in sorted(folder.iterdir()):
                    z.write(f, f"{session}/{f.name}")
        except BaseException:
            os.unlink(name)
            raise
        return Path(name)

    def raw_running(self, session: str) -> bool:
        raw = self._raw
        return raw is not None and raw["dir"].name == session

    def delete_raw(self, session: str) -> bool:
        folder = self._raw_session(session)
        if folder is None:
            return False
        raw = self._raw
        if raw is not None and raw["dir"] == folder:
            self.stop_raw()
        shutil.rmtree(folder, ignore_errors=True)
        return True

    # --- storing ---

    def _save(self) -> None:
        with self._io_lock:
            with self._lock:
                if not self._dirty or self._map is None:
                    return
                name, frame_id = self._map
                # dict copy is fast c code; the arrays are built outside the lock
                snapshot = dict(self._hits)
                color_snapshot = dict(self._colors)
                normal_snapshot = dict(self._normals)
                self._dirty = False
                self._save_time = time.monotonic()
            import numpy as np

            path = self._file(name)
            tmp = path.with_name(path.name + ".tmp")
            try:
                keys = np.fromiter(snapshot.keys(), dtype=np.int64, count=len(snapshot))
                hits = np.fromiter(snapshot.values(), dtype=np.uint16, count=len(snapshot))
                rgb = np.fromiter((color_snapshot.get(k, 0) for k in snapshot), dtype=np.uint32,
                                  count=len(snapshot))
                nor = np.fromiter((normal_snapshot.get(k, 0) for k in snapshot), dtype=np.uint32,
                                  count=len(snapshot))
                with open(tmp, "wb") as f:
                    np.savez_compressed(f, keys=keys, hits=hits, rgb=rgb, normal=nor,
                                        voxel_m=np.float64(config.MAP3D_VOXEL_M),
                                        frame_id=np.str_(frame_id or ""))
                os.replace(tmp, path)
            except Exception as exc:
                self.last_error = f"Speichern: {exc}"
                log.warning("3D map: saving failed: %s", exc)
                with self._lock:
                    self._dirty = True

    # --- output ---

    def blocks(self) -> bytes:
        """Shown blocks, 16 bytes each, little-endian: int16 ix, iy, iz, 0, then
        uint8 r, g, b, a (a = 255 with colour, 0 without), then int8 nx, ny, nz
        (normal * 127) and int8 n (127 with normal, 0 without). index times
        MAP3D_VOXEL_M = lower corner in the map frame (z = height above the floor)"""
        import numpy as np

        with self._lock:
            snapshot = dict(self._hits)
            color_snapshot = dict(self._colors)
            normal_snapshot = dict(self._normals)
        keys = np.fromiter(snapshot.keys(), dtype=np.int64, count=len(snapshot))
        hits = np.fromiter(snapshot.values(), dtype=np.int64, count=len(snapshot))
        keys = keys[hits >= config.MAP3D_MIN_HITS]
        rgb = np.fromiter((color_snapshot.get(k, 0) for k in keys.tolist()), dtype=np.int64, count=len(keys))
        nor = np.fromiter((normal_snapshot.get(k, 0) for k in keys.tolist()), dtype=np.int64,
                          count=len(keys))
        ix, iy, iz = unpack(keys)
        out = np.zeros(len(keys), dtype=[("ix", "<i2"), ("iy", "<i2"), ("iz", "<i2"), ("pad", "<i2"),
                                         ("r", "u1"), ("g", "u1"), ("b", "u1"), ("a", "u1"),
                                         ("nx", "i1"), ("ny", "i1"), ("nz", "i1"), ("n", "i1")])
        out["ix"], out["iy"], out["iz"] = (a.clip(-32768, 32767) for a in (ix, iy, iz))
        out["r"], out["g"], out["b"] = (rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255
        out["a"] = np.where(rgb & _HAS_COLOR, 255, 0)
        out["nx"] = ((nor >> 16) & 255) - 128
        out["ny"] = ((nor >> 8) & 255) - 128
        out["nz"] = (nor & 255) - 128
        out["n"] = np.where(nor & _HAS_NORMAL, 127, 0)
        return out.tobytes()

    def brief(self) -> dict[str, Any]:
        with self._lock:
            return {
                "verfuegbar": self.available,
                "aufnahme": self.recording,
                "karte": self._map[0] if self._map else None,
                "anzahl": self._shown,
                "bloecke_gesamt": len(self._hits),
                "mit_farbe": len(self._colors),
                "farbkamera": getattr(self._color_camera, "name", None),
                "voll": self._full,
                "voxel_m": config.MAP3D_VOXEL_M,
                "stand": self.stand,
                "text": self.text,
                "fehler": self.last_error,
                "live": self.live_count,
                "roh": {"aktiv": self._raw is not None,
                        "bilder": self._raw["frames"] if self._raw is not None else 0},
            }


def _normals(np, x, y, z, valid, step):
    """Unit normal per valid depth point (n x 3, nan where none), facing the
    camera: cross product of the steps to the right and the lower neighbour
    in the thinned image. none across depth jumps (edges, gaps)"""
    shape = valid.shape
    grid = np.full(shape + (3,), np.nan, dtype=np.float32)
    grid[valid] = np.column_stack((x, y, z))
    right = grid[:-1, 1:] - grid[:-1, :-1]
    down = grid[1:, :-1] - grid[:-1, :-1]
    n = np.cross(right, down)
    jump = config.MAP3D_NORMAL_MAX_JUMP_M * step
    bad = (np.linalg.norm(right, axis=2) > jump) | (np.linalg.norm(down, axis=2) > jump)
    length = np.linalg.norm(n, axis=2)
    bad |= ~(length > 1e-9)
    n = n / np.where(bad, 1.0, length)[..., None]
    n[bad] = np.nan
    out = np.full(shape + (3,), np.nan, dtype=np.float32)
    out[:-1, :-1] = n
    out = out[valid]
    # facing the camera: the side the measurement came from
    cam = np.array([config.DEPTH_CAMERA_FORWARD_M, 0.0, config.DEPTH_CAMERA_HEIGHT_M], dtype=np.float32)
    toward = cam - np.column_stack((x, y, z))
    flip = np.einsum("ij,ij->i", out, toward) < 0
    out[flip] *= -1.0
    return out


def encode_live(item: dict, index: int) -> bytes:
    """One live image as a binary websocket message: uint32 (little-endian)
    length of a json header, the header (i, t, pose, boden, farbe as in
    frames.jsonl, plus tiefe_bytes and farbe_bytes), the depth png (16-bit,
    mm), then the colour jpeg if any"""
    import cv2

    ok, png = cv2.imencode(".png", item["depth"])
    if not ok:
        raise OSError("depth png not encoded")
    png = png.tobytes()
    jpeg = b""
    farbe = None
    if item["color"] is not None:
        jpeg, box, size = item["color"]
        farbe = {"box": list(box), "groesse": list(size)}
    header = json.dumps({
        "i": index, "t": round(item["time"], 3),
        "pose": [round(v, 4) for v in item["pose"]],
        "boden": [round(v, 5) for v in item["floor"]] if item["floor"] else None,
        "farbe": farbe, "tiefe_bytes": len(png), "farbe_bytes": len(jpeg),
    }).encode("utf-8")
    return len(header).to_bytes(4, "little") + header + png + jpeg
