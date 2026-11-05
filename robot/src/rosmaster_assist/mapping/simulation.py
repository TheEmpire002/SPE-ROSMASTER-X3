"""Mapping in the simulation (--mock): the map is built in Python.

At the desk there is neither Docker nor slam_toolbox. So that the /map
page, the mapping drive and everything that uses the map (localization,
goal drive) still work, this class builds a simple occupancy map from the
simulated lidar and the lidar odometry: every ray makes the cells up to the
hit more free and the cell at the hit more occupied (log-odds).

Cells are entered with the **true** pose, which only the simulation knows
(``Scan.true_pose``), converted into the frame in which mapping began. That
way the map behaves like slam_toolbox's with loop closure: it is correct
while the lidar odometry slowly drifts -- and exactly this drift is what
localization in the map then has to compensate. Entered with the lidar
odometry pose, the map smeared by about 30° after 7 minutes in the first
attempt (04.11.).

Deliberately only for the simulation (E8). The map comes in the same format
as from the ROS bridge, so the rest of the code notices no difference.
"""

from __future__ import annotations

import base64
import logging
import math
import threading
import time

import numpy as np

from .. import config
from ..navigation.map_localization import invert, compose

log = logging.getLogger(__name__)

#: Size of the grid (m) around the origin; when sent, the map is cropped to
#: the known part.
_SIZE_M = 30.0
_FREE = -0.4
_OCCUPIED = 0.9
_LIMIT = 4.0


class SimMapping:
    """Occupancy map from scans and lidar odometry, in its own thread."""

    def __init__(self, scan_odometry) -> None:
        self._odo = scan_odometry
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._map: dict | None = None
        self._scans = 0
        self._started_at: float | None = None
        self._reset()

    def _reset(self) -> None:
        self.resolution = config.MAP_SIM_RESOLUTION_M
        n = int(_SIZE_M / self.resolution)
        self._n = n
        self._origin = -_SIZE_M / 2.0
        self._lo = np.zeros((n, n), dtype=np.float32)
        self._seen = np.zeros((n, n), dtype=bool)
        self._map = None
        self._scans = 0
        # True pose -> map frame, fixed at the first scan.
        self._true_to_map: tuple[float, float, float] | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        with self._lock:
            self._reset()
        self._stop.clear()
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._loop, name="sim-kartierung", daemon=True)
        self._thread.start()
        log.info("Mapping started in the simulation (without Docker/slam_toolbox)")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def get_latest_map(self) -> dict | None:
        with self._lock:
            return dict(self._map) if self._map is not None else None

    def status(self) -> dict:
        with self._lock:
            return {
                "running": self.running,
                "map_available": self._map is not None,
                "last_map_at": self._map["received_at"] if self._map else None,
                "scans": self._scans,
            }

    def _loop(self) -> None:
        last_nr = None
        tick = 1.0 / max(config.MAP_SIM_HZ, 0.1)
        next_map = 0.0
        while not self._stop.is_set():
            measurement = self._odo.measurement()
            if measurement is not None and measurement.nr != last_nr:
                last_nr = measurement.nr
                try:
                    self._integrate(measurement.scan.points(), self._pose(measurement))
                    if time.monotonic() >= next_map:
                        next_map = time.monotonic() + tick
                        self._publish()
                except Exception:
                    log.exception("Simulated mapping: scan not processed")
            self._stop.wait(0.05)

    def _pose(self, measurement) -> tuple[float, float, float]:
        """Pose for entering the scan: the true one, in the mapping frame.

        The map frame is -- as with slam_toolbox -- that of the lidar
        odometry at the start. At the first scan both still agree; that
        gives the fixed conversion from true to map. Without a true pose
        (no mock lidar) only the odometry is left.
        """
        truth = getattr(measurement.scan, "true_pose", None)
        if truth is None:
            return measurement.pose
        if self._true_to_map is None:
            self._true_to_map = compose(measurement.pose, invert(truth))
        return compose(self._true_to_map, truth)

    def _integrate(self, points, pose) -> None:
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return
        pts = pts[:: max(1, len(pts) // 240)]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = c * pts[:, 0] - s * pts[:, 1] + pose[0]
        wy = s * pts[:, 0] + c * pts[:, 1] + pose[1]
        r = np.hypot(wx - pose[0], wy - pose[1])
        steps = int(np.ceil(r.max() / self.resolution))
        t = np.arange(steps) * self.resolution
        # Free cells along each ray up to just before the hit.
        share = t[None, :] / np.maximum(r[:, None], 1e-6)
        valid = t[None, :] < (r[:, None] - self.resolution)
        fx = pose[0] + share * (wx - pose[0])[:, None]
        fy = pose[1] + share * (wy - pose[1])[:, None]
        free = self._indices(fx[valid], fy[valid])
        occupied = self._indices(wx, wy)
        with self._lock:
            flat = self._lo.ravel()
            seen = self._seen.ravel()
            free = np.setdiff1d(free, occupied)
            flat[free] += _FREE
            flat[occupied] += _OCCUPIED
            np.clip(flat, -_LIMIT, _LIMIT, out=flat)
            seen[free] = True
            seen[occupied] = True
            self._scans += 1

    def _indices(self, x, y) -> np.ndarray:
        ix = np.floor((x - self._origin) / self.resolution).astype(int)
        iy = np.floor((y - self._origin) / self.resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < self._n) & (iy < self._n)
        return np.unique(iy[inside] * self._n + ix[inside])

    def _publish(self) -> None:
        with self._lock:
            lo = self._lo.copy()
            seen = self._seen.copy()
        ys, xs = np.nonzero(seen)
        if len(xs) == 0:
            return
        margin = 10
        x0, x1 = max(0, xs.min() - margin), min(self._n, xs.max() + margin + 1)
        y0, y1 = max(0, ys.min() - margin), min(self._n, ys.max() + margin + 1)
        lo, seen = lo[y0:y1, x0:x1], seen[y0:y1, x0:x1]
        occupancy = np.full(lo.shape, -1, dtype=np.int8)
        occupancy[seen & (lo < 0.0)] = 0
        occupancy[seen & (lo > 0.6)] = 100
        occ_map = {
            "topic": "/map",
            "frame_id": "map",
            "info": {
                "resolution": self.resolution,
                "width": int(occupancy.shape[1]),
                "height": int(occupancy.shape[0]),
                "origin": {
                    "position": {"x": self._origin + x0 * self.resolution,
                                 "y": self._origin + y0 * self.resolution, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data_b64": base64.b64encode(occupancy.tobytes()).decode("ascii"),
            "received_at": time.time(),
        }
        with self._lock:
            self._map = occ_map
