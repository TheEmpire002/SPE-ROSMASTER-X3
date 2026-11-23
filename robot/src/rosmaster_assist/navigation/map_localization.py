"""Where is the robot in the map?

The lidar odometry (``scan_odometry.py``) knows how the robot has moved
since it started -- in its own frame, with slow drift. The map has a
different frame (that of slam_toolbox or of the saved map). This class
holds the **correction** between the two:

    pose in the map = correction ∘ pose according to lidar odometry

Several times per second the current scan is laid onto the map
(``refine``): every lidar point should lie on a wall of the map. How far it
is from one is stored in the map's precomputed distance field -- that makes
the matching fast enough for the Pi, without searching point pairs as ICP
does. If it succeeds, the correction moves a step towards the result. So
the pose stays attached to the map even on long drives instead of drifting
away with the odometry.

When mapping, map and lidar odometry start at the same place, the
correction is initially zero. When a saved map is loaded, nobody knows
where the robot is: then it searches around a hint (``search``) -- first at
the map's starting point, because that is where it usually stands, or where
someone tapped on the map.

Deliberately not slam_toolbox in localization mode: that would need a
second bridge back out of the container, and driving with a map would
depend on Docker and ROS2 Foxy. This way numpy alone is enough (E12).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import NamedTuple

import numpy as np

from .. import config

log = logging.getLogger(__name__)

#: This many points per scan are enough for the matching; more only cost time.
_MAX_POINTS = 180
_SEARCH_POINTS = 90
#: Points that land in the unknown part of the map do not count towards the
#: share -- when mapping, that is the space it is only just seeing.
#: If fewer than this many remain, the match is worthless.
_MIN_POINTS = 30
#: Repeat a search at most this often (s) when it finds nothing.
_SEARCH_PAUSE_S = 2.0


class Match(NamedTuple):
    x: float
    y: float
    theta: float
    share: float      # points that hit / points in known area
    residuum: float    # mean distance of the points that hit (m)
    points: int        # points in known area


def _normalize(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def compose(a, b) -> tuple[float, float, float]:
    """Append pose ``b`` (relative to ``a``) to pose ``a``."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalize(a[2] + b[2])


def invert(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def score(occ_map, points, pose) -> Match:
    """How well does the scan fit the map at this pose?"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    qx = c * points[:, 0] - s * points[:, 1] + pose[0]
    qy = s * points[:, 0] + c * points[:, 1] + pose[1]
    known = ~occ_map.unknown_at(qx, qy)
    n = int(known.sum())
    if n == 0:
        return Match(pose[0], pose[1], pose[2], 0.0, float("inf"), 0)
    d = occ_map.distance_at(qx[known], qy[known])
    hits = d < config.NAV_HIT_M
    residuum = float(d[hits].mean()) if hits.any() else float("inf")
    return Match(pose[0], pose[1], pose[2], float(hits.mean()), residuum, n)


def refine(occ_map, points, start, iterations: int = 12) -> Match:
    """Shift the pose near ``start`` so that the points lie on walls.

    Gauss-Newton on the distance field: every point pulls towards the
    nearest wall (gradient of the field), and the less the further away it
    is -- points on people or moved chairs should not bend the pose. Points
    further than ``NAV_DISTANCE_MAX_M`` from any wall do not pull at all
    (the field is flat there).
    """
    x, y, th = start
    px, py = points[:, 0], points[:, 1]
    for k in range(iterations):
        c, s = math.cos(th), math.sin(th)
        qx = c * px - s * py + x
        qy = s * px + c * py + y
        d, (gx, gy), inside = occ_map.sample(qx, qy)
        # Coarse first (wide capture range), then only nearby points.
        gate = 0.5 if k < iterations // 2 else 0.25
        usable = inside & (d < gate)
        if int(usable.sum()) < _MIN_POINTS:
            break
        w = np.where(d < 0.05, 1.0, 0.05 / np.maximum(d, 1e-6))[usable]
        dth = gx * (-s * px - c * py) + gy * (c * px - s * py)
        J = np.stack((gx[usable], gy[usable], dth[usable]), axis=1)
        H = (J * w[:, None]).T @ J + np.diag((1e-3, 1e-3, 1e-2))
        b = (J * w[:, None]).T @ d[usable]
        try:
            step = -np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            break
        # Big jumps in small steps: the field is only meaningful near the
        # walls.
        length = math.hypot(step[0], step[1])
        if length > 0.1:
            step[:2] *= 0.1 / length
        step[2] = max(-0.08, min(0.08, step[2]))
        x += float(step[0])
        y += float(step[1])
        th = _normalize(th + float(step[2]))
        if length < 1e-4 and abs(step[2]) < 1e-4:
            break
    return score(occ_map, points, (x, y, th))


def search(occ_map, points, center: tuple[float, float] | None, radius: float,
           angle: float | None = None, min_share: float | None = None) -> Match | None:
    """Search for the pose around ``center`` (without ``center``: in the whole map).

    All free spots on a grid, all headings (or ±30° around ``angle``),
    scored with the distance field; the best ones are refined. Only a clear
    hit counts -- better "don't know" than a wrong pose it then drives off
    with: it must hit ``min_share`` of the points, and no clearly different
    pose may fit almost as well (a rectangular room also fits rotated by
    180°).
    """
    if min_share is None:
        min_share = config.NAV_SEARCH_MIN_SHARE
    if len(points) < _MIN_POINTS:
        return None
    pts = points[:: max(1, math.ceil(len(points) / _SEARCH_POINTS))]
    grid = config.NAV_SEARCH_GRID_M if center is not None else 2 * config.NAV_SEARCH_GRID_M
    step = max(1, int(round(grid / occ_map.resolution)))
    iy, ix = np.nonzero(occ_map.free[::step, ::step])
    iy, ix = iy * step, ix * step
    kx, ky = occ_map.world(ix, iy)
    good = occ_map.distance[iy, ix] >= 0.1
    if center is not None:
        good &= np.hypot(kx - center[0], ky - center[1]) <= radius
    kx, ky = kx[good], ky[good]
    if len(kx) == 0:
        return None
    if angle is None:
        angles = np.radians(np.arange(0.0, 360.0, config.NAV_SEARCH_DEG))
    else:
        angles = angle + np.radians(np.arange(-30.0, 30.1, config.NAV_SEARCH_DEG))

    cap = 0.3
    distance = np.minimum(occ_map.distance, cap)
    # Unknown cells cost as much as "far away from any wall".
    distance = np.where(occ_map.unknown, cap, distance)
    top: list[tuple[float, float, float, float]] = []
    for th in angles:
        c, s = math.cos(th), math.sin(th)
        rx = c * pts[:, 0] - s * pts[:, 1]
        ry = s * pts[:, 0] + c * pts[:, 1]
        # In chunks, so the array (candidates x points) stays small.
        for a in range(0, len(kx), 4000):
            qx = kx[a:a + 4000, None] + rx[None, :]
            qy = ky[a:a + 4000, None] + ry[None, :]
            jx, jy = occ_map.cell(qx, qy)
            inside = occ_map.inside(jx, jy)
            d = np.where(
                inside,
                distance[np.clip(jy, 0, occ_map.height - 1), np.clip(jx, 0, occ_map.width - 1)],
                cap,
            )
            value = d.mean(axis=1)
            for i in np.argsort(value)[:4]:
                top.append((float(value[i]), float(kx[a + i]), float(ky[a + i]), float(th)))
    top.sort()
    candidates: list[Match] = []
    for _, x, y, th in top[:12]:
        a = refine(occ_map, points, (x, y, th), iterations=16)
        if a.points >= _MIN_POINTS:
            candidates.append(a)
    if not candidates:
        return None
    candidates.sort(key=lambda a: (a.share, -a.residuum), reverse=True)
    result = candidates[0]
    if result.share < min_share:
        return None
    for a in candidates[1:]:
        different = (
            math.hypot(a.x - result.x, a.y - result.y) > config.NAV_AMBIGUOUS_M
            or abs(_normalize(a.theta - result.theta))
            > math.radians(config.NAV_AMBIGUOUS_DEG)
        )
        if different and a.share >= result.share - config.NAV_AMBIGUOUS_SHARE:
            log.info(
                "Search ambiguous: (%.2f, %.2f, %.0f°) and (%.2f, %.2f, %.0f°) fit equally well",
                result.x, result.y, math.degrees(result.theta),
                a.x, a.y, math.degrees(a.theta),
            )
            return None
    return result


class MapLocalization:
    """The robot's pose in the current map, continuously tracked."""

    def __init__(self, scan_odometry) -> None:
        self._odo = scan_odometry
        self._lock = threading.Lock()
        self._map = None
        self._live = False
        self._start = (0.0, 0.0, 0.0)
        self._correction = (0.0, 0.0, 0.0)
        self._run_id: int | None = None        # odometry run the correction belongs to
        self._last_no: int | None = None
        self._success_time: float | None = None
        self._last_pose: tuple[float, float, float] | None = None
        self._share: float | None = None
        self._residual: float | None = None
        self._search: tuple | None = None     # (center, radius, angle, min_share) or None
        self._last_search = 0.0
        self._search_text: str | None = None
        self._trail: deque = deque(maxlen=config.NAV_TRAIL_MAX)
        self._matches = 0
        self._discarded = 0

    # --- Map ---------------------------------------------------------------------

    @property
    def occ_map(self):
        return self._map

    @property
    def start(self) -> tuple[float, float, float]:
        """Starting point of the map (where the mapping began) -- "home"."""
        return self._start

    def set_map(self, occ_map, start=None, live: bool = False) -> None:
        """Take over a new map.

        ``live``: the map is currently growing (mapping), its frame is that
        of the lidar odometry, which restarted with the mapping (that is why
        the MappingManager calls ``unload`` before starting). The first map
        of a mapping run sets the correction to zero, every further one
        leaves it as the tracking found it. Otherwise it is a loaded map,
        and the pose has to be searched for first.
        """
        self._odo.start("ortung")
        with self._lock:
            new = self._map is None or live != self._live
            self._map = occ_map
            self._live = live
            if start is not None:
                self._start = tuple(start)
            if live and new:
                # Mapping begins: map and odometry share the origin.
                self._start = (0.0, 0.0, 0.0)
                self._correction = (0.0, 0.0, 0.0)
                self._run_id = self._odo.run
                self._success_time = time.monotonic()
                self._search = None
                self._trail.clear()
            elif not live:
                self._run_id = None
                self._success_time = None
                self._trail.clear()
                # First with the heading from the end of the mapping (it usually
                # still faces that way); if that finds nothing: all headings.
                self._search = ((self._start[0], self._start[1]), config.NAV_SEARCH_RADIUS_M,
                               self._start[2], config.NAV_SEARCH_MIN_SHARE)
                self._last_search = 0.0
                self._search_text = "sucht die eigene Lage am Ausgangspunkt der Karte"

    @property
    def live(self) -> bool:
        """The map comes from the running (or just finished) mapping."""
        return self._map is not None and self._live

    def unload(self) -> None:
        with self._lock:
            self._map = None
            self._live = False
            self._run_id = None
            self._search = None
            self._trail.clear()
        self._odo.stop("ortung")

    def set_start(self, start) -> None:
        """New starting point ("home") in this map."""
        with self._lock:
            self._start = (float(start[0]), float(start[1]), float(start[2]))

    def set_pose(self, x: float, y: float, angle: float | None = None) -> None:
        """Someone shows on the map where the robot is: search there."""
        with self._lock:
            radius = 0.6 if angle is not None else 1.0
            self._search = ((float(x), float(y)), radius, angle, config.NAV_HINT_MIN_SHARE)
            self._last_search = 0.0
            self._search_text = "sucht die eigene Lage an der gezeigten Stelle"
            self._success_time = None

    # --- Queries -----------------------------------------------------------------

    def pose(self) -> tuple[float, float, float] | None:
        """Pose in the map (x, y, radians) or None while unknown."""
        with self._lock:
            if self._map is None or self._run_id is None or self._run_id != self._odo.run:
                return None
            correction = self._correction
        odo = self._odo.pose()
        if odo is None:
            return None
        return compose(correction, odo)

    @property
    def confident(self) -> bool:
        """Pose known and recently confirmed -- only then may it drive by the map."""
        with self._lock:
            if self._map is None or self._run_id is None or self._run_id != self._odo.run:
                return False
            if self._search is not None:
                return False
            return (
                self._success_time is not None
                and time.monotonic() - self._success_time <= config.NAV_CONFIDENT_S
            )

    def trail(self) -> list[list[float]]:
        with self._lock:
            return [list(p) for p in self._trail]

    def state(self) -> dict:
        pose = self.pose()
        confident = self.confident
        with self._lock:
            occ_map = self._map
            return {
                "karte": occ_map is not None,
                "name": getattr(occ_map, "name", None),
                "live": occ_map is not None and self._live,
                "pose": (
                    {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "winkel": round(math.degrees(pose[2]), 1)}
                    if pose is not None else None
                ),
                "sicher": confident,
                "sucht": self._search is not None,
                "text": self._text(pose, confident),
                "anteil": round(self._share, 2) if self._share is not None else None,
                "residuum_m": (
                    round(self._residual, 3)
                    if self._residual is not None and math.isfinite(self._residual) else None
                ),
                "start": [round(self._start[0], 3), round(self._start[1], 3),
                          round(math.degrees(self._start[2]), 1)],
                "abgleiche": self._matches,
                "verworfen": self._discarded,
            }

    def _text(self, pose, confident: bool) -> str:
        if self._map is None:
            return "keine Karte geladen"
        if self._search is not None:
            return self._search_text or "sucht die eigene Lage"
        if pose is None:
            return "Lage unbekannt -- auf der Karte zeigen, wo er steht"
        if not confident:
            return "Lage unsicher -- die Aufnahme passt gerade nicht zur Karte"
        return "Lage in der Karte bekannt"

    # --- Tracking (call in its own thread, takes a few ms) -----------------------

    def match_scan(self) -> None:
        """Lay the latest scan onto the map."""
        occ_map = self._map
        if occ_map is None or not self._odo.running:
            return
        measurement = self._odo.measurement()
        if measurement is None or measurement.nr == self._last_no:
            return
        self._last_no = measurement.nr
        points = measurement.scan.points()
        if len(points) < _MIN_POINTS:
            return
        points = np.asarray(points, dtype=float)
        points = points[np.hypot(points[:, 0], points[:, 1]) <= config.LIDAR_MAX_M]
        if len(points) > _MAX_POINTS:
            points = points[:: math.ceil(len(points) / _MAX_POINTS)]
        odo_pose = measurement.pose

        with self._lock:
            pending_search = self._search
            run_matches = self._run_id is not None and self._run_id == self._odo.run
            correction = self._correction
            success = self._success_time
            last = self._last_pose

        if pending_search is None and not run_matches:
            # The odometry started over (restart): the robot is still where
            # it last was -- search there.
            center = (last[0], last[1]) if last is not None else self._start[:2]
            pending_search = (center, config.NAV_SEARCH_RADIUS_M, None, config.NAV_SEARCH_MIN_SHARE)
            with self._lock:
                self._search = pending_search
                self._search_text = "sucht die eigene Lage nach einem Neustart der Odometrie"
        elif (
            pending_search is None and success is not None
            and time.monotonic() - success > 2 * config.NAV_CONFIDENT_S
            and last is not None
        ):
            # Nothing has matched for a long time (lifted, pushed): search
            # again around the last good pose.
            pending_search = ((last[0], last[1]), config.NAV_SEARCH_RADIUS_M, None,
                     config.NAV_SEARCH_MIN_SHARE)
            with self._lock:
                self._search = pending_search
                self._search_text = "hat die Lage verloren und sucht sie wieder"

        if pending_search is not None:
            self._search_step(occ_map, points, odo_pose, pending_search)
            return

        prediction = compose(correction, odo_pose)
        a = refine(occ_map, points, prediction)
        jump = math.hypot(a.x - prediction[0], a.y - prediction[1])
        turn = abs(_normalize(a.theta - prediction[2]))
        good = (
            a.points >= _MIN_POINTS
            and a.share >= config.NAV_MIN_SHARE
            and jump <= config.NAV_MAX_JUMP_M
            and turn <= math.radians(config.NAV_MAX_JUMP_DEG)
        )
        with self._lock:
            if self._map is not occ_map or self._search is not None:
                return  # a new map or a hint arrived during the calculation
            self._share, self._residual = a.share, a.residuum
            if not good:
                self._discarded += 1
                return
            self._matches += 1
            w = config.NAV_TRUST
            target = (
                prediction[0] + w * (a.x - prediction[0]),
                prediction[1] + w * (a.y - prediction[1]),
                _normalize(prediction[2] + w * _normalize(a.theta - prediction[2])),
            )
            self._correction = compose(target, invert(odo_pose))
            self._success_time = time.monotonic()
            self._last_pose = target
            self._extend_trail(target)

    def _search_step(self, occ_map, points, odo_pose, pending_search) -> None:
        if time.monotonic() - self._last_search < _SEARCH_PAUSE_S:
            return
        self._last_search = time.monotonic()
        center, radius, angle, min_share = pending_search
        a = search(occ_map, points, center, radius, angle, min_share)
        with self._lock:
            if self._map is not occ_map or self._search != pending_search:
                return
            if a is None:
                if angle is not None:
                    # Nothing with the given heading: try all headings.
                    self._search = (center, radius, None, min_share)
                    self._last_search = 0.0
                    return
                self._search_text = (
                    "findet sich nicht -- auf der Karte zeigen, wo er steht"
                )
                return
            self._correction = compose((a.x, a.y, a.theta), invert(odo_pose))
            self._run_id = self._odo.run
            self._success_time = time.monotonic()
            self._last_pose = (a.x, a.y, a.theta)
            self._share, self._residual = a.share, a.residuum
            self._search = None
            self._search_text = None
            self._extend_trail((a.x, a.y, a.theta))
        log.info(
            "Pose in the map found: x=%.2f y=%.2f %.0f° (%.0f %% hits)",
            a.x, a.y, math.degrees(a.theta), 100 * a.share,
        )

    def _extend_trail(self, pose) -> None:
        """Only call while holding the lock."""
        if not self._trail or math.hypot(
            pose[0] - self._trail[-1][0], pose[1] - self._trail[-1][1]
        ) >= config.NAV_TRAIL_M:
            self._trail.append((round(pose[0], 2), round(pose[1], 2)))
