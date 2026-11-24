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

Per map one file ``<name>_3d.npz`` beside the map files: packed block keys,
hit counts, block size and the map's frame id. Like the photos it is set
aside as ``<name>_3d.<stamp>.alt.npz`` when the main map is replaced, the old
blocks would sit in the wrong places.

``tick`` runs in a worker thread (map_control's tick loop), the endpoints in
the event loop; everything shared is behind ``_lock``. File access (load,
save, delete) is serialized by ``_io_lock``, so a delete cannot be undone by
a save that already copied the old blocks. Saving runs in its own thread,
compressing 400k blocks must not hold up the localization.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
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


def pack(ix, iy, iz):
    """Block indices (numpy int64 arrays) -> one key per block."""
    return ((ix + _OFF) << (2 * _BITS)) | ((iy + _OFF) << _BITS) | (iz + _OFF)


def unpack(keys):
    """Keys -> (ix, iy, iz), numpy int64 arrays."""
    return ((keys >> (2 * _BITS)) & _MASK) - _OFF, ((keys >> _BITS) & _MASK) - _OFF, (keys & _MASK) - _OFF


class Room3D:
    def __init__(self, storage_dir: Path, localization, depth, odometry=None) -> None:
        self._dir = Path(storage_dir)
        self._localization = localization
        self._depth = depth
        self._odometry = odometry
        self._lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._saver: threading.Thread | None = None
        self._map: tuple[str, str | None] | None = None
        self._hits: dict[int, int] = {}
        self._shown = 0
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
        try:
            with np.load(path, allow_pickle=False) as data:
                saved_frame = str(data["frame_id"]) if "frame_id" in data.files else ""
                voxel = float(data["voxel_m"])
                keys, counts = data["keys"], data["hits"]
            wrong_frame = bool(frame_id and saved_frame and saved_frame != frame_id)
            if wrong_frame or abs(voxel - config.MAP3D_VOXEL_M) > 1e-6:
                aside = path.with_name(f"{name}_3d.{time.strftime('%Y%m%d-%H%M%S')}.alt.npz")
                os.replace(path, aside)
                log.warning("3D map of '%s' belongs to an older map or block size -- moved to %s",
                            name, aside.name)
            else:
                hits = dict(zip(keys.tolist(), counts.tolist()))
        except FileNotFoundError:
            pass
        except Exception:
            log.warning("3D map of '%s' unreadable -- starting a new one", name, exc_info=True)
        with self._lock:
            self._map = current
            self._hits = hits
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
        self.text = ("Speicher voll -- ergänzt nur noch" if self._full
                     else f"nimmt auf ({added} Punkte im letzten Bild)")

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

    def _integrate(self, frame, pose) -> int:
        import numpy as np

        from ..sensors.depth import robot_points

        hfov, vfov = self._depth.hfov, self._depth.vfov
        if not hfov or not vfov:
            return 0
        x, y, z, _ = robot_points(frame.raw, hfov, vfov, step=config.MAP3D_PIXEL_STEP)
        keep = (x >= config.MAP3D_MIN_M) & (np.hypot(x, y) <= config.MAP3D_MAX_M)
        x, y, z = x[keep], y[keep], z[keep]
        # height above the floor the camera sees, not above the mounting
        # guess: the robot tips a little while driving
        if frame.floor is not None:
            a, b, c = frame.floor
            z = z - (a * x + b * y + c)
        keep = (z >= config.MAP3D_MIN_HEIGHT_M) & (z <= config.MAP3D_MAX_HEIGHT_M)
        x, y, z = x[keep], y[keep], z[keep]
        if not len(x):
            return 0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = c * x - s * y + pose[0]
        wy = s * x + c * y + pose[1]
        v = config.MAP3D_VOXEL_M
        keys = np.unique(pack(np.floor(wx / v).astype(np.int64), np.floor(wy / v).astype(np.int64),
                              np.floor(z / v).astype(np.int64)))
        threshold = config.MAP3D_MIN_HITS
        with self._lock:
            hits = self._hits
            for k in keys.tolist():
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
            self._dirty = True
            self.stand += 1
        return int(len(x))

    # --- storing ---

    def _save(self) -> None:
        with self._io_lock:
            with self._lock:
                if not self._dirty or self._map is None:
                    return
                name, frame_id = self._map
                # dict copy is fast c code; the arrays are built outside the lock
                snapshot = dict(self._hits)
                self._dirty = False
                self._save_time = time.monotonic()
            import numpy as np

            path = self._file(name)
            tmp = path.with_name(path.name + ".tmp")
            try:
                keys = np.fromiter(snapshot.keys(), dtype=np.int64, count=len(snapshot))
                hits = np.fromiter(snapshot.values(), dtype=np.uint16, count=len(snapshot))
                with open(tmp, "wb") as f:
                    np.savez_compressed(f, keys=keys, hits=hits, voxel_m=np.float64(config.MAP3D_VOXEL_M),
                                        frame_id=np.str_(frame_id or ""))
                os.replace(tmp, path)
            except Exception as exc:
                self.last_error = f"Speichern: {exc}"
                log.warning("3D map: saving failed: %s", exc)
                with self._lock:
                    self._dirty = True

    # --- output ---

    def blocks(self) -> bytes:
        """Shown blocks as little-endian int16 triplets (ix, iy, iz), times
        MAP3D_VOXEL_M = lower corner in the map frame (z = height above the floor)"""
        import numpy as np

        with self._lock:
            snapshot = dict(self._hits)
        keys = np.fromiter(snapshot.keys(), dtype=np.int64, count=len(snapshot))
        hits = np.fromiter(snapshot.values(), dtype=np.int64, count=len(snapshot))
        ix, iy, iz = unpack(keys[hits >= config.MAP3D_MIN_HITS])
        out = np.column_stack((ix, iy, iz)).clip(-32768, 32767).astype("<i2")
        return out.tobytes()

    def brief(self) -> dict[str, Any]:
        with self._lock:
            return {
                "verfuegbar": self.available,
                "aufnahme": self.recording,
                "karte": self._map[0] if self._map else None,
                "anzahl": self._shown,
                "bloecke_gesamt": len(self._hits),
                "voll": self._full,
                "voxel_m": config.MAP3D_VOXEL_M,
                "stand": self.stand,
                "text": self.text,
                "fehler": self.last_error,
            }
