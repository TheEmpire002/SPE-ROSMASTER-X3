"""Shared helpers for the always-on mapping tests: the imagined apartment.

Not a test itself (no ``test_`` prefix, ``scripts/run_tests.py`` skips it).
The apartment is the one of the simulation (``MockLidar``,
``MOCK_APARTMENT``): two rooms, door at x = 1.0 m, y = 1.4 to 2.3 m, a
pillar, a cupboard and a sofa. Import after setting
``config.MOCK_APARTMENT = True``.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.mapping.occ_map import OccupancyMap  # noqa: E402
from rosmaster_assist.sensors.lidar import MockLidar, Scan  # noqa: E402

LIDAR = MockLidar()
X_MIN, X_MAX, Y_MIN, Y_MAX = MockLidar.ROOM
DOOR = (1.4, 2.3)


def _segment_distance(x, y, segment):
    x1, y1, x2, y2 = segment
    ex, ey = x2 - x1, y2 - y1
    t = np.clip(((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
    return np.hypot(x - x1 - t * ex, y - y1 - t * ey)


def world_distance(x, y):
    """Distance of a point to the nearest wall or piece of furniture."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    d = np.minimum.reduce([np.abs(x - X_MIN), np.abs(X_MAX - x), np.abs(y - Y_MIN), np.abs(Y_MAX - y)])
    for s in MockLidar.APARTMENT_WALLS:
        d = np.minimum(d, _segment_distance(x, y, s))
    cx, cy, r = MockLidar.PILLAR
    return np.minimum(d, np.abs(np.hypot(x - cx, y - cy) - r))


def apartment(resolution=0.05):
    """A perfect map of the whole apartment."""
    origin = (X_MIN - 0.5, Y_MIN - 0.5)
    width = int((X_MAX - X_MIN + 1.0) / resolution)
    height = int((Y_MAX - Y_MIN + 1.0) / resolution)
    iy, ix = np.mgrid[0:height, 0:width]
    x = origin[0] + (ix + 0.5) * resolution
    y = origin[1] + (iy + 0.5) * resolution
    inside = (x > X_MIN) & (x < X_MAX) & (y > Y_MIN) & (y < Y_MAX)
    d = world_distance(x, y)
    occupancy = np.full((height, width), -1, dtype=np.int8)
    occupancy[inside & (d > resolution)] = 0
    occupancy[(d <= resolution) & (x > X_MIN - resolution) & (x < X_MAX + resolution)
              & (y > Y_MIN - resolution) & (y < Y_MAX + resolution)] = 100
    return OccupancyMap(occupancy, resolution, origin)


def corridor(length=12.0, width=1.2, resolution=0.05):
    """A long empty corridor along x: walls only at y = ±width/2."""
    origin = (-1.0, -width / 2 - 0.5)
    w = int((length + 2.0) / resolution)
    h = int((width + 1.0) / resolution)
    iy, ix = np.mgrid[0:h, 0:w]
    x = origin[0] + (ix + 0.5) * resolution
    y = origin[1] + (iy + 0.5) * resolution
    occupancy = np.full((h, w), -1, dtype=np.int8)
    inside = np.abs(y) < width / 2
    occupancy[inside] = 0
    occupancy[np.abs(np.abs(y) - width / 2) <= resolution / 2 + 1e-9] = 100
    occupancy[(x < 0) | (x > length)] = -1
    return OccupancyMap(occupancy, resolution, origin)


def corridor_scan(x, width=1.2, max_range=6.0, step_deg=1.0):
    """Scan in the corridor (heading along x): only the side walls echo."""
    pts = []
    for i in range(int(360 / step_deg)):
        a = math.radians(i * step_deg)
        s = math.sin(a)
        if abs(s) < 1e-6:
            continue
        r = (width / 2) / abs(s)
        if config.LIDAR_MIN_M <= r <= max_range:
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.asarray(pts)


def scan_points(x, y, theta, step_deg=1.0, noise=0.0, rng=None, drop=0.0):
    """What the simulated lidar would see at this pose, in the robot frame."""
    pts = []
    for i in range(int(360 / step_deg)):
        if drop and rng is not None and rng.random() < drop:
            continue
        a = math.radians(i * step_deg)
        r = LIDAR._ray(x, y, a + theta)
        if noise and rng is not None:
            r += rng.normal(0.0, noise)
        if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.asarray(pts)


def make_scan(points, timestamp, true_pose=None):
    """A real ``Scan`` from robot-frame points (angles and ranges)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    angle = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])).tolist()
    ranges = np.hypot(pts[:, 0], pts[:, 1]).tolist()
    scan = Scan(angle, ranges)
    scan.timestamp = timestamp
    scan.true_pose = true_pose
    return scan
