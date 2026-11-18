"""The map as a Python object: occupancy, distance to the nearest wall, loading.

slam_toolbox delivers the map as a ROS OccupancyGrid (via
``ros_bridge.py``), and it is saved as PGM + YAML (``map_service.py``).
Localization and path planning, however, need it as a numeric array, and
they need it often: the localization asks several times per second "how far
is this point from the nearest wall?". That is why the distance field is
precomputed once per map.

Coordinates as in ROS: cell ``[iy, ix]`` lies at
``origin + (ix + 0.5, iy + 0.5) * resolution`` in the map frame. The map
frame is the one from slam_toolbox (``map``); it starts where mapping
started, x forward, y to the left (like the lidar odometry at startup).
"""

from __future__ import annotations

import base64
import json
import math
import os
import re
import threading
import time
from pathlib import Path

import numpy as np

from .. import config

#: Occupancy values as in nav_msgs/OccupancyGrid.
UNKNOWN = -1
#: From here on a cell counts as wall, up to here as free (thresholds as in
#: map_server: occupied_thresh 0.65, free_thresh 0.196).
OCCUPIED_MIN = 65
FREE_MAX = 19


class OccupancyMap:
    """Immutable snapshot of an occupancy map."""

    def __init__(
        self,
        occupancy: np.ndarray,
        resolution: float,
        origin: tuple[float, float],
        name: str | None = None,
        timestamp: float | None = None,
    ) -> None:
        self.occupancy = np.asarray(occupancy, dtype=np.int8)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.name = name
        self.timestamp = time.time() if timestamp is None else timestamp
        # always-on mapping: same frame_id = newer state of the same map
        self.frame_id: str | None = None
        self.height, self.width = self.occupancy.shape
        b = self.occupancy.astype(np.int16)
        self.occupied = b >= OCCUPIED_MIN
        self.free = (b >= 0) & (b <= FREE_MAX)
        self.unknown = b < 0
        self._distance: np.ndarray | None = None
        self._gradient: tuple[np.ndarray, np.ndarray] | None = None

    # --- Creation ------------------------------------------------------------------

    @classmethod
    def from_payload(cls, payload: dict) -> "OccupancyMap":
        """From the message of the ROS bridge (or the simulation)."""
        info = payload["info"]
        width, height = int(info["width"]), int(info["height"])
        raw = base64.b64decode(payload["data_b64"])
        if len(raw) != width * height:
            raise ValueError(f"Karte unvollständig: {len(raw)} statt {width * height} Zellen")
        occupancy = np.frombuffer(raw, dtype=np.int8).reshape(height, width)
        pos = info["origin"]["position"]
        return cls(
            occupancy, float(info["resolution"]), (float(pos["x"]), float(pos["y"])),
            timestamp=payload.get("received_at"),
        )

    @classmethod
    def load(cls, yaml_path: str | Path) -> "OccupancyMap":
        """A map saved with ``MapService.save_map`` (PGM + YAML).

        Deliberately without a YAML package: the file only has the few lines
        that ``map_service.py`` writes itself.
        """
        yaml_path = Path(yaml_path)
        values: dict[str, str] = {}
        for line in yaml_path.read_text(encoding="utf-8").splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                values[k.strip()] = v.strip()
        resolution = float(values["resolution"])
        origin = [float(z) for z in re.findall(r"[-+0-9.eE]+", values["origin"])]
        image = yaml_path.parent / values.get("image", yaml_path.with_suffix(".pgm").name)
        gray = _read_pgm(image)
        # PGM starts at the top, the map at the bottom (ROS); gray values as in
        # map_server "trinary" mode: dark = wall, light = free, in between unknown.
        gray = gray[::-1]
        occupancy = np.full(gray.shape, UNKNOWN, dtype=np.int8)
        share = (255.0 - gray) / 255.0
        occupancy[share >= 0.65] = 100
        occupancy[share <= 0.196] = 0
        return cls(occupancy, resolution, (origin[0], origin[1]), name=yaml_path.stem,
                   timestamp=yaml_path.stat().st_mtime)

    def as_payload(self) -> dict:
        """In the format of the ROS bridge -- for the /map page."""
        return {
            "topic": "/map",
            "frame_id": "map",
            "info": {
                "resolution": self.resolution,
                "width": self.width,
                "height": self.height,
                "origin": {
                    "position": {"x": self.origin[0], "y": self.origin[1], "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data_b64": base64.b64encode(self.occupancy.tobytes()).decode("ascii"),
            "received_at": self.timestamp,
            "name": self.name,
        }

    @classmethod
    def derived(cls, occupancy, resolution, origin, previous: "OccupancyMap | None",
                dirty: tuple[int, int, int, int] | None, name: str | None = None) -> "OccupancyMap":
        """A newer state of ``previous`` where only the cells in ``dirty`` changed.

        ``dirty`` = (ix0, iy0, ix1, iy1), end exclusive. The distance field is
        clamped at NAV_DISTANCE_MAX_M, so a change can only reach that far:
        only that neighbourhood is recomputed, the rest is copied. Exact, and
        for a growing map a fraction of the full computation. Same shape,
        origin and resolution required -- otherwise everything is rebuilt.
        """
        new = cls(occupancy, resolution, origin, name=name)
        if (
            previous is None or dirty is None
            or previous.occupancy.shape != new.occupancy.shape
            or previous.origin != new.origin or previous.resolution != new.resolution
        ):
            _ = new.gradient
            return new
        ix0, iy0, ix1, iy1 = dirty
        h, w = new.height, new.width
        if (ix1 - ix0) * (iy1 - iy0) > 0.5 * h * w:
            _ = new.gradient
            return new
        m = int(math.ceil(config.NAV_DISTANCE_MAX_M / resolution)) + 1
        # inner: cells whose distance may have changed; window: everything
        # that can be their nearest wall
        ax0, ay0 = max(0, ix0 - m), max(0, iy0 - m)
        ax1, ay1 = min(w, ix1 + m), min(h, iy1 + m)
        wx0, wy0 = max(0, ix0 - 2 * m), max(0, iy0 - 2 * m)
        wx1, wy1 = min(w, ix1 + 2 * m), min(h, iy1 + 2 * m)
        local = distance_field(new.occupied[wy0:wy1, wx0:wx1], resolution, config.NAV_DISTANCE_MAX_M)
        d = previous.distance.copy()
        d[ay0:ay1, ax0:ax1] = local[ay0 - wy0:ay1 - wy0, ax0 - wx0:ax1 - wx0]
        new._distance = d
        gx, gy = previous.gradient
        gx, gy = gx.copy(), gy.copy()
        # one cell more for the central differences at the edge of inner
        bx0, by0 = max(0, ax0 - 1), max(0, ay0 - 1)
        bx1, by1 = min(w, ax1 + 1), min(h, ay1 + 1)
        if by1 - by0 >= 2 and bx1 - bx0 >= 2:
            ly, lx = np.gradient(d[by0:by1, bx0:bx1], resolution)
            gx[ay0:ay1, ax0:ax1] = lx[ay0 - by0:ay1 - by0, ax0 - bx0:ax1 - bx0]
            gy[ay0:ay1, ax0:ax1] = ly[ay0 - by0:ay1 - by0, ax0 - bx0:ax1 - bx0]
        else:
            gy_all, gx_all = np.gradient(d, resolution)
            gx, gy = gx_all, gy_all
        new._gradient = (gx, gy)
        return new

    def nearest_free(self, x: float, y: float, reach_m: float) -> tuple[float, float] | None:
        """Nearest known free cell centre within ``reach_m`` (m), None if there is none."""
        ix, iy = self.cell(x, y)
        r = int(math.ceil(reach_m / self.resolution))
        x0, y0 = max(0, int(ix) - r), max(0, int(iy) - r)
        x1, y1 = min(self.width, int(ix) + r + 1), min(self.height, int(iy) + r + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        ys, xs = np.nonzero(self.free[y0:y1, x0:x1])
        if len(xs) == 0:
            return None
        cx, cy = self.world(xs + x0, ys + y0)
        dist = np.hypot(cx - x, cy - y)
        i = int(np.argmin(dist))
        if dist[i] > reach_m:
            return None
        return float(cx[i]), float(cy[i])

    # --- Conversion ----------------------------------------------------------------

    def cell(self, x, y):
        """World coordinates (m) -> cell indices (ix, iy), also for arrays."""
        ix = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return ix, iy

    def world(self, ix, iy):
        """Cell center (m) for cell indices."""
        return (
            self.origin[0] + (np.asarray(ix) + 0.5) * self.resolution,
            self.origin[1] + (np.asarray(iy) + 0.5) * self.resolution,
        )

    def inside(self, ix, iy):
        return (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)

    def is_free(self, x: float, y: float) -> bool:
        ix, iy = self.cell(x, y)
        return bool(self.inside(ix, iy) and self.free[iy, ix])

    # --- Distance to the nearest wall ------------------------------------------------

    @property
    def distance(self) -> np.ndarray:
        """Distance of each cell to the nearest wall (m), capped at NAV_DISTANCE_MAX_M."""
        if self._distance is None:
            self._distance = distance_field(self.occupied, self.resolution, config.NAV_DISTANCE_MAX_M)
        return self._distance

    @property
    def gradient(self) -> tuple[np.ndarray, np.ndarray]:
        """Gradient of the distance field per meter (d/dx, d/dy) -- for localization."""
        if self._gradient is None:
            gy, gx = np.gradient(self.distance, self.resolution)
            self._gradient = (gx, gy)
        return self._gradient

    def distance_at(self, x, y):
        """Distance to the wall at world points (bilinear), the maximum outside the map."""
        d, _, _ = self.sample(np.asarray(x, dtype=float), np.asarray(y, dtype=float),
                                 with_gradient=False)
        return d

    def sample(self, x: np.ndarray, y: np.ndarray, with_gradient: bool = True):
        """Distance (and gradient) at arbitrary points, interpolated between cell centers."""
        fx = (x - self.origin[0]) / self.resolution - 0.5
        fy = (y - self.origin[1]) / self.resolution - 0.5
        x0 = np.floor(fx).astype(int)
        y0 = np.floor(fy).astype(int)
        ax, ay = fx - x0, fy - y0
        inside = (x0 >= 0) & (y0 >= 0) & (x0 < self.width - 1) & (y0 < self.height - 1)
        x0c = np.clip(x0, 0, self.width - 2)
        y0c = np.clip(y0, 0, self.height - 2)

        def mix(field_):
            return (
                field_[y0c, x0c] * (1 - ax) * (1 - ay)
                + field_[y0c, x0c + 1] * ax * (1 - ay)
                + field_[y0c + 1, x0c] * (1 - ax) * ay
                + field_[y0c + 1, x0c + 1] * ax * ay
            )

        d = np.where(inside, mix(self.distance), config.NAV_DISTANCE_MAX_M)
        if not with_gradient:
            return d, None, inside
        gx, gy = self.gradient
        return d, (np.where(inside, mix(gx), 0.0), np.where(inside, mix(gy), 0.0)), inside

    def unknown_at(self, x, y):
        """True for points in unknown cells or outside the map."""
        ix, iy = self.cell(x, y)
        inside = self.inside(ix, iy)
        ixc = np.clip(ix, 0, self.width - 1)
        iyc = np.clip(iy, 0, self.height - 1)
        return ~inside | self.unknown[iyc, ixc]


def distance_field(occupied: np.ndarray, resolution: float, maximum: float) -> np.ndarray:
    """Distance to the nearest occupied cell (m), capped at ``maximum``.

    Chamfer distance with 8 neighbors, only as many passes as ``maximum``
    requires -- each pass is a handful of whole-array operations in numpy.
    Accurate to about 8 %, more than enough for localization and path costs,
    and without scipy.
    """
    d = np.where(occupied, 0.0, np.inf)
    passes = int(math.ceil(maximum / resolution)) + 1
    diag = resolution * math.sqrt(2.0)
    for _ in range(passes):
        new = d.copy()
        new[1:, :] = np.minimum(new[1:, :], d[:-1, :] + resolution)
        new[:-1, :] = np.minimum(new[:-1, :], d[1:, :] + resolution)
        new[:, 1:] = np.minimum(new[:, 1:], d[:, :-1] + resolution)
        new[:, :-1] = np.minimum(new[:, :-1], d[:, 1:] + resolution)
        new[1:, 1:] = np.minimum(new[1:, 1:], d[:-1, :-1] + diag)
        new[:-1, :-1] = np.minimum(new[:-1, :-1], d[1:, 1:] + diag)
        new[1:, :-1] = np.minimum(new[1:, :-1], d[:-1, 1:] + diag)
        new[:-1, 1:] = np.minimum(new[:-1, 1:], d[1:, :-1] + diag)
        if np.array_equal(new, d):
            break
        d = new
    return np.minimum(d, maximum)


def _read_pgm(path: Path) -> np.ndarray:
    """Binary PGM (P5, 8 bit) as a gray value array, row 0 at the top."""
    data = path.read_bytes()
    parts: list[bytes] = []
    pos = 0
    # Header: "P5", width, height, max value -- skip comments (#).
    while len(parts) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        parts.append(data[pos:end])
        pos = end
    if parts[0] != b"P5":
        raise ValueError(f"{path.name}: kein binäres PGM")
    width, height = int(parts[1]), int(parts[2])
    pos += 1  # exactly one whitespace character after the max value
    pixel = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos)
    return pixel.reshape(height, width).astype(float)


# --- Extra data for the saved map --------------------------------------------------


def extra_path(yaml_path: str | Path) -> Path:
    return Path(yaml_path).with_suffix(".json")


def read_extra(yaml_path: str | Path) -> dict:
    """Starting point and named places of a saved map (if present)."""
    path = extra_path(yaml_path)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def write_extra(yaml_path: str | Path, data: dict) -> None:
    path = extra_path(yaml_path)
    # own tmp name per writer: two threads must not replace each other's file
    tmp = path.with_suffix(f".json.{os.getpid()}.{threading.get_ident()}.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
