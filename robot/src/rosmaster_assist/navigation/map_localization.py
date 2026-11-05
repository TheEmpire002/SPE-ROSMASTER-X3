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

**Sure or not (E14).** Always-on mapping writes into the map behind this
pose, so "sure" must not just mean "matched recently". It also means: not
driven further than ``NAV_BLIND_MAX_M`` since the last good match, and after
a search a few good matches confirmed it. Every match is handed out as a
``TrackResult`` with its quality (share, residual, overlap, how well the
walls pin the pose down) -- the integrate gate decides from that whether
the scan may change the map. If the pose is lost, it searches in stages:
around where the odometry says it should be (near the driven trail first),
then everywhere.

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
from typing import Callable, NamedTuple

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


class Stage(NamedTuple):
    """One step of a pose search."""

    center: tuple[float, float] | None   # None: the whole map
    radius: float
    angle: float | None                  # ±30° around it, None: all headings
    min_share: float
    near: object = None                  # Nx2 trail points: only candidates close to them
    text: str = "sucht die eigene Lage"


class TrackResult(NamedTuple):
    """What one tracking step saw -- input for the integrate gate."""

    nr: int
    timestamp: float | None
    run: int
    scan: object                  # sensors.lidar.Scan
    points: object                # the (subsampled) points used, robot frame
    odo_pose: tuple
    prediction: tuple | None
    match: Match | None           # raw refine result
    accepted: bool                # passed tracking, the correction moved
    kind: str                     # "track" | "search_ok" | "search" | "skip"
    jump_m: float
    turn_rad: float
    known_share: float            # points in known cells / all points
    constraint: float             # how well the walls pin x/y down (0..0.5)
    map_gen: int                  # changes when the map is replaced (not updated)
    gap: bool                     # lidar odometry bridged a dropout
    odo_good: bool
    odo_streak: int


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
    d_all = occ_map.distance_at(qx, qy)
    # a point right next to a known wall hits known map even if its own cell
    # is unknown: a growing map has walls one cell thin with unknown cells
    # behind and just in front of them, and half the wall points land there
    known = ~occ_map.unknown_at(qx, qy) | (d_all < config.NAV_HIT_M)
    n = int(known.sum())
    if n == 0:
        return Match(pose[0], pose[1], pose[2], 0.0, float("inf"), 0)
    d = d_all[known]
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


def constraint(occ_map, points, pose) -> float:
    """How firmly the walls pin the position down, 0 (not at all) .. 0.5 (ideal).

    smallest over largest share of the wall normals: in a room with corners
    the normals point every way, in a long corridor all of them point across
    it -- sliding along the corridor then changes nothing in the match, and
    the result there is a guess

    the wall directions come from the scan itself (a line through each point
    and its neighbours), not from the map: in a growing map walls end where
    the lidar's reach ended, and the distance field bends around those ends
    as if there were cross walls. Only points on known walls count.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < _MIN_POINTS:
        return 0.0
    c, s = math.cos(pose[2]), math.sin(pose[2])
    qx = c * pts[:, 0] - s * pts[:, 1] + pose[0]
    qy = s * pts[:, 0] + c * pts[:, 1] + pose[1]
    # near an occupied cell is on a known wall, whatever its own cell says
    # (as in score: growing maps have walls one cell thin)
    on_wall = occ_map.distance_at(qx, qy) < config.NAV_HIT_M
    order = np.argsort(np.arctan2(pts[:, 1], pts[:, 0]))
    p, on_wall = pts[order], on_wall[order]
    # neighbours by angle, only those close enough to be the same wall
    near = [np.roll(p, k, axis=0) for k in (-2, -1, 0, 1, 2)]
    close = np.stack([np.hypot(*(q - p).T) <= config.NAV_NORMAL_REACH_M for q in near])
    count = close.sum(axis=0)
    stack = np.stack(near)                                  # 5 x n x 2
    w = close[..., None]
    mean = (stack * w).sum(axis=0) / np.maximum(count, 1)[:, None]
    dev = (stack - mean) * w
    sxx = (dev[..., 0] ** 2).sum(axis=0)
    syy = (dev[..., 1] ** 2).sum(axis=0)
    sxy = (dev[..., 0] * dev[..., 1]).sum(axis=0)
    # smallest and largest spread: a wall is a thin line, a corner is not
    spread = np.sqrt(((sxx - syy) / 2) ** 2 + sxy ** 2)
    small, large = (sxx + syy) / 2 - spread, (sxx + syy) / 2 + spread
    line = (count >= 4) & (large > 1e-6) & (small <= 0.05 * large)
    usable = on_wall & line
    if int(usable.sum()) < _MIN_POINTS:
        return 0.0
    # normal = across the line
    along = 0.5 * np.arctan2(2 * sxy[usable], (sxx - syy)[usable])
    gx, gy = -np.sin(along), np.cos(along)
    h = np.array([[np.sum(gx * gx), np.sum(gx * gy)], [np.sum(gx * gy), np.sum(gy * gy)]])
    trace = float(np.trace(h))
    if trace <= 0.0:
        return 0.0
    return float(np.linalg.eigvalsh(h)[0]) / trace


def search(occ_map, points, center: tuple[float, float] | None, radius: float,
           angle: float | None = None, min_share: float | None = None,
           near=None) -> Match | None:
    """Search for the pose around ``center`` (without ``center``: in the whole map).

    All free spots on a grid, all headings (or ±30° around ``angle``),
    scored with the distance field; the best ones are refined. Only a clear
    hit counts -- better "don't know" than a wrong pose it then drives off
    with: it must hit ``min_share`` of the points, and no clearly different
    pose may fit almost as well (a rectangular room also fits rotated by
    180°). ``near``: only spots close to these points (the driven trail).
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
    if near is not None and len(near):
        trail = np.asarray(near, dtype=float).reshape(-1, 2)
        close = np.zeros(len(kx), dtype=bool)
        # in chunks: candidates x trail points stays small
        for a in range(0, len(trail), 500):
            t = trail[a:a + 500]
            dist = np.hypot(kx[:, None] - t[None, :, 0], ky[:, None] - t[None, :, 1])
            close |= (dist <= config.NAV_CRUMB_SEARCH_M).any(axis=1)
        good &= close
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
        self._map_gen = 0                     # bumped when the map is replaced, not updated
        self._live = False
        self._source: str | None = None       # "dauer" | "slam" | "geladen"
        self._start = (0.0, 0.0, 0.0)
        self._correction = (0.0, 0.0, 0.0)
        self._run_id: int | None = None        # odometry run the correction belongs to
        self._last_no: int | None = None
        self._success_time: float | None = None
        self._success_odo: tuple[float, float, float] | None = None
        self._confirm_left = 0                 # good matches still needed after a search
        self._last_pose: tuple[float, float, float] | None = None
        self._share: float | None = None
        self._residual: float | None = None
        self._search: Stage | None = None      # running search stage or None
        self._stages: deque[Stage] = deque()   # stages after the running one
        self._failed_stages = 0
        self._last_search = 0.0
        self._search_text: str | None = None
        self._trail: deque = deque(maxlen=config.NAV_TRAIL_MAX)
        self._matches = 0
        self._discarded = 0
        #: Latest tracking step, for the integrate gate.
        self.last_track: TrackResult | None = None
        #: Returns the driven trail (Nx2) for searching near it; set by the server.
        self.near_source: Callable[[], object] | None = None

    # --- Map ---------------------------------------------------------------------

    @property
    def occ_map(self):
        return self._map

    @property
    def map_gen(self) -> int:
        return self._map_gen

    @property
    def start(self) -> tuple[float, float, float]:
        """Starting point of the map (where the mapping began) -- "home"."""
        return self._start

    def set_map(self, occ_map, start=None, live: bool = False, source: str | None = None,
                stages: list[Stage] | None = None, correction=None) -> None:
        """Take over a new map.

        ``live``: the map is currently growing (slam mapping), its frame is
        that of the lidar odometry. The first map of a mapping run sets the
        correction to zero, every further one leaves it as the tracking
        found it. Otherwise it is a loaded map, and the pose has to be
        searched for first -- at the map's starting point, or along
        ``stages`` (after a restart: the last saved pose first).
        ``correction``: the pose is known (a map founded right here, or the
        main map back after slam) -- no search, odometry frame -> map frame.
        """
        self._odo.start("ortung")
        with self._lock:
            new = self._map is None or live != self._live
            if new or not live:
                self._map_gen += 1
            self._map = occ_map
            self._live = live
            self._source = source or ("slam" if live else "geladen")
            if start is not None:
                self._start = tuple(start)
            if live and new:
                # mapping begins: slam maps in the odometry frame, so the
                # correction is zero; home is where the robot stands now
                # (with permanent odometry that is no longer (0, 0, 0))
                self._start = self._odo.pose() or (0.0, 0.0, 0.0)
                self._correction = (0.0, 0.0, 0.0)
                self._run_id = self._odo.run
                self._success_time = time.monotonic()
                self._success_odo = self._odo.pose()
                self._confirm_left = 0
                self._clear_search()
                self._trail.clear()
            elif not live and correction is not None:
                self._correction = tuple(float(v) for v in correction)
                self._run_id = self._odo.run
                self._success_time = time.monotonic()
                self._success_odo = self._odo.pose()
                self._confirm_left = 0
                self._clear_search()
                self._trail.clear()
            elif not live:
                self._run_id = None
                self._success_time = None
                self._trail.clear()
                # First with the heading from the end of the mapping (it usually
                # still faces that way); if that finds nothing: all headings.
                self._begin_search(stages or [Stage(
                    (self._start[0], self._start[1]), config.NAV_SEARCH_RADIUS_M,
                    self._start[2], config.NAV_SEARCH_MIN_SHARE,
                    text="sucht die eigene Lage am Ausgangspunkt der Karte",
                )])

    def update_map(self, occ_map) -> None:
        """A newer state of the same map (always-on mapping): keep pose and search."""
        with self._lock:
            if self._map is None:
                return
            if getattr(self._map, "frame_id", None) != getattr(occ_map, "frame_id", None):
                return   # a late snapshot of a map that is no longer loaded
            self._map = occ_map

    @property
    def live(self) -> bool:
        """The map comes from the running (or just finished) slam mapping."""
        return self._map is not None and self._live

    @property
    def source(self) -> str | None:
        return self._source if self._map is not None else None

    def unload(self) -> None:
        with self._lock:
            self._map = None
            self._map_gen += 1
            self._live = False
            self._source = None
            self._run_id = None
            self._clear_search()
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
            self._success_time = None
            self._begin_search([Stage(
                (float(x), float(y)), radius, angle, config.NAV_HINT_MIN_SHARE,
                text="sucht die eigene Lage an der gezeigten Stelle",
            )])

    def relocalize(self) -> None:
        """Search right away instead of waiting until the pose counts as lost."""
        if self._map is None or self._search is not None:
            return
        # outside the lock: _lost_stages asks blind_m(), which takes it
        stages = self._lost_stages(self._prediction())
        with self._lock:
            if self._map is None or self._search is not None:
                return
            self._begin_search(stages)

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

    def blind_m(self) -> float | None:
        """Lidar odometry distance since the last good match (m)."""
        with self._lock:
            success_odo = self._success_odo
        odo = self._odo.pose()
        if success_odo is None or odo is None:
            return None
        return math.hypot(odo[0] - success_odo[0], odo[1] - success_odo[1])

    @property
    def confident(self) -> bool:
        """Pose known and recently confirmed -- only then may it drive by the map."""
        blind = self.blind_m()
        with self._lock:
            if self._map is None or self._run_id is None or self._run_id != self._odo.run:
                return False
            if self._search is not None or self._confirm_left > 0:
                return False
            if blind is not None and blind > config.NAV_BLIND_MAX_M:
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
        blind = self.blind_m()
        with self._lock:
            occ_map = self._map
            return {
                "karte": occ_map is not None,
                "name": getattr(occ_map, "name", None),
                "live": occ_map is not None and self._live,
                "quelle": self._source if occ_map is not None else None,
                "pose": (
                    {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "winkel": round(math.degrees(pose[2]), 1)}
                    if pose is not None else None
                ),
                "sicher": confident,
                "sucht": self._search is not None,
                "text": self._text(pose, confident),
                "grund": None if confident else self._reason(pose, blind),
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

    def _reason(self, pose, blind) -> str | None:
        """Why the pose is not sure, in a few words (lock held)."""
        if self._map is None:
            return "keine Karte"
        if self._search is not None:
            return self._search_text or "sucht die eigene Lage"
        if pose is None:
            return "Lage unbekannt"
        if self._confirm_left > 0:
            done = config.NAV_CONFIRM_AFTER_SEARCH - self._confirm_left
            return f"bestätigt die gefundene Lage ({done}/{config.NAV_CONFIRM_AFTER_SEARCH})"
        if blind is not None and blind > config.NAV_BLIND_MAX_M:
            return f"seit {blind:.1f} m ohne passenden Abgleich"
        if self._share is not None and self._share < config.NAV_MIN_SHARE:
            return f"Aufnahme passt nicht zur Karte ({100 * self._share:.0f} % Treffer)"
        return "kein passender Abgleich in letzter Zeit"

    # --- Search stages (lock held, except _lost_stages and _prediction) -----------

    def _clear_search(self) -> None:
        self._search = None
        self._stages.clear()
        self._failed_stages = 0
        self._search_text = None

    def _begin_search(self, stages: list[Stage]) -> None:
        self._stages = deque(stages[1:])
        self._failed_stages = 0
        self._set_stage(stages[0])

    def _set_stage(self, stage: Stage) -> None:
        self._search = stage
        self._search_text = stage.text
        self._last_search = 0.0

    def _lost_stages(self, prediction) -> list[Stage]:
        """Where to look when lost: around the prediction, near the trail first."""
        blind = self.blind_m() or 0.0
        radius = min(max(config.NAV_LOST_RADIUS_BASE_M + config.NAV_LOST_RADIUS_PER_M * blind,
                         config.NAV_LOST_RADIUS_BASE_M), config.NAV_SEARCH_RADIUS_M)
        if prediction is None:
            last = self._last_pose
            if last is None:
                return [Stage(None, 0.0, None, config.NAV_SEARCH_MIN_SHARE,
                              text="sucht die eigene Lage in der ganzen Karte")]
            prediction, radius = last, config.NAV_SEARCH_RADIUS_M
        center = (prediction[0], prediction[1])
        text = "hat die Lage verloren und sucht sie in der Nähe"
        stages = []
        near = self.near_source() if self.near_source is not None else None
        if near is not None and len(near):
            stages.append(Stage(center, radius, prediction[2], config.NAV_SEARCH_MIN_SHARE,
                                near=near, text=text + " der gefahrenen Spur"))
        stages.append(Stage(center, radius, prediction[2], config.NAV_SEARCH_MIN_SHARE, text=text))
        return stages

    def _prediction(self):
        with self._lock:
            run_matches = self._run_id is not None and self._run_id == self._odo.run
            correction = self._correction
        odo = self._odo.pose()
        if not run_matches or odo is None:
            return None
        return compose(correction, odo)

    # --- Tracking (call in its own thread, takes a few ms) -----------------------

    def match_scan(self) -> TrackResult | None:
        """Lay the latest scan onto the map; the result also lands in ``last_track``."""
        occ_map = self._map
        gen = self._map_gen
        if occ_map is None or not self._odo.running:
            return None
        measurement = self._odo.measurement()
        if measurement is None or measurement.nr == self._last_no:
            return None
        self._last_no = measurement.nr
        points = measurement.scan.points()
        if len(points) < _MIN_POINTS:
            return self._publish(measurement, None, None, "skip", gen)
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

        blind = self.blind_m()
        if pending_search is None and not run_matches:
            # The odometry started over (restart): the robot is still where
            # it last was -- search there.
            center = (last[0], last[1]) if last is not None else self._start[:2]
            with self._lock:
                self._begin_search([Stage(
                    center, config.NAV_SEARCH_RADIUS_M, None, config.NAV_SEARCH_MIN_SHARE,
                    text="sucht die eigene Lage nach einem Neustart der Odometrie",
                )])
                pending_search = self._search
        elif pending_search is None and success is not None and (
            time.monotonic() - success > 2 * config.NAV_CONFIDENT_S
            or (blind is not None and blind > 2 * config.NAV_BLIND_MAX_M)
        ):
            # nothing has matched for a long time or for a long way (lifted,
            # pushed, a room that changed): search where it should be
            stages = self._lost_stages(compose(correction, odo_pose))
            with self._lock:
                self._begin_search(stages)
                pending_search = self._search
            log.info("Pose in the map lost -- searching around the prediction")

        if pending_search is not None:
            found = self._search_step(occ_map, points, odo_pose, pending_search, gen)
            if found is None:
                return self._publish(measurement, points, None, "search", gen)
            return self._publish(measurement, points, found, "search_ok", gen, accepted=True,
                                 occ_map=occ_map)

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
            if self._map_gen != gen or self._search is not None:
                return None  # a new map or a hint arrived during the calculation
            self._share, self._residual = a.share, a.residuum
            if not good:
                self._discarded += 1
            else:
                self._matches += 1
                w = config.NAV_TRUST
                target = (
                    prediction[0] + w * (a.x - prediction[0]),
                    prediction[1] + w * (a.y - prediction[1]),
                    _normalize(prediction[2] + w * _normalize(a.theta - prediction[2])),
                )
                self._correction = compose(target, invert(odo_pose))
                self._success_time = time.monotonic()
                self._success_odo = odo_pose
                self._confirm_left = max(0, self._confirm_left - 1)
                self._last_pose = target
                self._extend_trail(target)
        return self._publish(measurement, points, a, "track", gen, accepted=good,
                             prediction=prediction, jump=jump, turn=turn, occ_map=occ_map)

    def _publish(self, measurement, points, a, kind, gen, accepted=False, prediction=None,
                 jump=0.0, turn=0.0, occ_map=None) -> TrackResult:
        n = len(points) if points is not None else 0
        known_share = (a.points / n) if (a is not None and n) else 0.0
        pinned = (
            constraint(occ_map, points, (a.x, a.y, a.theta))
            if a is not None and occ_map is not None and a.points >= _MIN_POINTS else 0.0
        )
        track = TrackResult(
            nr=measurement.nr,
            timestamp=getattr(measurement, "timestamp", None),
            run=self._odo.run,
            scan=measurement.scan,
            points=points,
            odo_pose=measurement.pose,
            prediction=prediction,
            match=a,
            accepted=accepted,
            kind=kind,
            jump_m=jump,
            turn_rad=turn,
            known_share=known_share,
            constraint=pinned,
            map_gen=gen,
            gap=bool(getattr(measurement, "gap", False)),
            odo_good=bool(getattr(measurement, "good", True)),
            odo_streak=int(getattr(measurement, "streak", 0)),
        )
        self.last_track = track
        return track

    def _search_step(self, occ_map, points, odo_pose, stage: Stage, gen) -> Match | None:
        pause = config.NAV_GLOBAL_SEARCH_S if stage.center is None else _SEARCH_PAUSE_S
        if time.monotonic() - self._last_search < pause:
            return None
        self._last_search = time.monotonic()
        a = search(occ_map, points, stage.center, stage.radius, stage.angle, stage.min_share,
                   near=stage.near)
        with self._lock:
            if self._map_gen != gen or self._search is not stage:
                return None
            if a is None:
                self._next_stage(stage)
                return None
            self._correction = compose((a.x, a.y, a.theta), invert(odo_pose))
            self._run_id = self._odo.run
            self._success_time = time.monotonic()
            self._success_odo = odo_pose
            self._confirm_left = config.NAV_CONFIRM_AFTER_SEARCH
            self._last_pose = (a.x, a.y, a.theta)
            self._share, self._residual = a.share, a.residuum
            self._clear_search()
            self._extend_trail((a.x, a.y, a.theta))
        log.info(
            "Pose in the map found: x=%.2f y=%.2f %.0f° (%.0f %% hits)",
            a.x, a.y, math.degrees(a.theta), 100 * a.share,
        )
        return a

    def _next_stage(self, stage: Stage) -> None:
        """This stage found nothing: widen the search (lock held)."""
        if stage.angle is not None:
            # nothing with the given heading: try all headings
            self._set_stage(stage._replace(angle=None))
            return
        if self._stages:
            self._failed_stages += 1
            self._set_stage(self._stages.popleft())
            return
        self._failed_stages += 1
        if stage.center is not None and self._failed_stages >= config.NAV_LOST_GLOBAL_AFTER:
            self._search = Stage(None, 0.0, None, config.NAV_SEARCH_MIN_SHARE,
                                 text="sucht die eigene Lage in der ganzen Karte")
            self._search_text = self._search.text
            return
        # keep trying the same stage, but say so
        self._search_text = "findet sich nicht -- auf der Karte zeigen, wo er steht"

    def _extend_trail(self, pose) -> None:
        """Only call while holding the lock."""
        if not self._trail or math.hypot(
            pose[0] - self._trail[-1][0], pose[1] - self._trail[-1][1]
        ) >= config.NAV_TRAIL_M:
            self._trail.append((round(pose[0], 2), round(pose[1], 2)))
