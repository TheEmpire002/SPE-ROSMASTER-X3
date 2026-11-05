"""The map that grows on every drive -- but only where the pose is sure (E14).

Until E14 a map only came from an explicit mapping run with slam_toolbox.
Now there is one persistent **main map**, and every drive extends it: every
lidar ray makes the cells up to its hit a little more free and the hit cell
a little more occupied (log-odds, as in ``simulation.py``).

What protects the map is the order of things:

1. the localization lays the scan onto the map *before* the scan is
   written (``map_localization.match_scan``) -- a scan never matches against
   itself;
2. only established cells count for that match: one hit (0.85) stays below
   the wall threshold (1.6), so a wall needs hits from at least two poses;
3. the integrate gate (``navigation/integrate_gate.py``) must pass the scan,
   and the ``LIVE_GATE_LAG_N`` scans after it as well -- when something goes
   wrong, the scans just before it never reach the map;
4. it is written at the raw match pose, and only after the robot moved;
5. cells that were sure at the start of the session change only slowly and
   at most by ``LIVE_MAP_SESSION_CHANGE_MAX``: one bad session can weaken a
   wall to "unknown" (not drivable), never to "free".

Without any map (first start, "Neue Karte") the robot founds one: a few good
scans at standstill, at the lidar odometry pose. From then on the map frame
is fixed; it is saved with ``live_store.py``.

No loop closure: drift in big loops is not corrected. When the robot comes
back into known space with a pose that no longer fits, the gate freezes the
map instead of drawing a second wall; for big homes an initial map from
slam_toolbox (adopted with ``import_occupancy``) is the better start.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid

import numpy as np

from .. import config
from ..navigation import integrate_gate as ig
from ..navigation.avoidance import LiftDetector
from .occ_map import OccupancyMap

log = logging.getLogger(__name__)


def _normalize(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def _relative(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, _normalize(b[2] - a[2])


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalize(a[2] + b[2])


class LiveMapper:
    """Log-odds grid that grows behind the integrate gate."""

    def __init__(self, wheel_pose_at=None, name: str | None = None) -> None:
        #: wheel odometry pose at a time (x, y, rad) or None -- for lift detection
        self._wheel_pose_at = wheel_pose_at
        self.name = name or config.LIVE_MAP_NAME
        self.resolution = config.LIVE_MAP_RESOLUTION_M
        # lock order: _step_lock / _snap_lock before _lock
        self._lock = threading.Lock()          # grid arrays
        self._step_lock = threading.RLock()    # gate, waiting scans, founding
        self._snap_lock = threading.Lock()     # one snapshot at a time
        self._gen = 0                          # +1 when the grid is replaced
        self._lo: np.ndarray | None = None       # float32 [h, w]
        self._base: np.ndarray | None = None     # log-odds at session start
        self._origin = (0.0, 0.0)                # world position of cell [0, 0] corner
        self._dirty: list[int] | None = None     # ix0, iy0, ix1, iy1 since the last snapshot
        self._grew = False
        self._previous: OccupancyMap | None = None
        self.frame_id: str | None = None
        self.gate = ig.IntegrateGate()
        self._pending: list[list] = []           # [points, pose, later passes]
        self._last_candidate: tuple | None = None
        self._prev_track = None
        self._lift = LiftDetector()
        self._hybrid: tuple | None = None        # wheel travel + lidar rotation
        self._hybrid_prev: tuple | None = None
        self._bootstrap: list = []
        self.changed = False                     # since the last save
        self.too_big = False
        self.scans_written = 0
        self._ms = {"eintragen": 0.0, "schnappschuss": 0.0}
        if not config.LIVE_MAP_ENABLED:
            self.gate.state = ig.OFF
            self.gate.reason = "abgeschaltet (LIVE_MAP_ENABLED)"

    # --- map content -------------------------------------------------------------

    @property
    def has_map(self) -> bool:
        return self._lo is not None

    def start_empty(self) -> None:
        """No map yet: found one at the next standstill."""
        with self._step_lock:
            with self._lock:
                self._lo = None
                self._base = None
                self._previous = None
                self._dirty = None
                self.frame_id = None
                self._gen += 1
            self._reset_step_state()
            if self.gate.state != ig.OFF:
                self.gate.state = ig.START
                self.gate.reason = "wartet im Stillstand, um die Karte zu beginnen"

    def load_array(self, lo, origin, resolution, frame_id: str | None) -> None:
        """Take over a saved grid; its sure cells are established for this session."""
        lo = np.asarray(lo, dtype=np.float32)
        with self._step_lock:
            with self._lock:
                self._lo = lo.copy()
                self._base = lo.copy()
                self._origin = (float(origin[0]), float(origin[1]))
                self.resolution = float(resolution)
                self.frame_id = frame_id or uuid.uuid4().hex
                self._previous = None
                self._dirty = None
                self._grew = True
                self._gen += 1
                self.changed = False
            self._reset_step_state()

    def _reset_step_state(self) -> None:
        """Forget everything tied to the old grid's frame (step lock held)."""
        self._pending.clear()
        self._bootstrap.clear()
        self._last_candidate = None
        self._prev_track = None
        self._hybrid = None
        self._hybrid_prev = None

    def import_occupancy(self, occ_map: OccupancyMap, frame_id: str | None = None) -> None:
        """Seed from a trinary map (slam_toolbox, a pgm): walls and free space become sure."""
        lo = np.zeros(occ_map.occupancy.shape, dtype=np.float32)
        lo[occ_map.occupied] = config.LIVE_MAP_SEED_OCC_LO
        lo[occ_map.free] = config.LIVE_MAP_SEED_FREE_LO
        self.load_array(lo, occ_map.origin, occ_map.resolution, frame_id or uuid.uuid4().hex)
        self.changed = True

    def arrays(self):
        """(lo, origin, resolution) copy for saving, or None."""
        with self._lock:
            if self._lo is None:
                return None
            return self._lo.copy(), self._origin, self.resolution

    def size_m(self) -> tuple[float, float] | None:
        with self._lock:
            if self._lo is None:
                return None
            h, w = self._lo.shape
            return w * self.resolution, h * self.resolution

    # --- writing -----------------------------------------------------------------

    def integrate(self, points, pose) -> None:
        """Write one scan (robot-frame points) at ``pose`` (map frame)."""
        started = time.perf_counter()
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return
        r = np.hypot(pts[:, 0], pts[:, 1])
        keep = r >= config.LIDAR_MIN_M
        pts, r = pts[keep], r[keep]
        if len(pts) == 0:
            return
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = c * pts[:, 0] - s * pts[:, 1] + pose[0]
        wy = s * pts[:, 0] + c * pts[:, 1] + pose[1]
        # every hit is written (one cell each, cheap): with only every n-th
        # ray the walls get gaps, and points of the next scan land in unknown
        # cells -- the gate would then never see enough known map
        hit = r <= config.LIVE_MAP_HIT_RANGE_M
        hx, hy = wx[hit], wy[hit]
        # free rays cost a cell per step: only every n-th
        k = max(1, int(math.ceil(len(pts) / config.LIVE_MAP_MAX_RAYS)))
        wx, wy, r = wx[::k], wy[::k], r[::k]
        # free space along each ray: up to just before the hit, at most FREE_RANGE
        reach = np.minimum(r - config.LIVE_MAP_FREE_MARGIN_M, config.LIVE_MAP_FREE_RANGE_M)
        steps = int(math.ceil(max(float(reach.max()), 0.0) / self.resolution))
        t = np.arange(max(steps, 1)) * self.resolution
        share = t[None, :] / np.maximum(r[:, None], 1e-6)
        valid = t[None, :] < reach[:, None]
        fx = pose[0] + share * (wx - pose[0])[:, None]
        fy = pose[1] + share * (wy - pose[1])[:, None]
        fx, fy = fx[valid], fy[valid]
        with self._lock:
            if self._lo is None:
                self._new_grid(pose)
            xs = np.concatenate((fx, hx, [pose[0]]))
            ys = np.concatenate((fy, hy, [pose[1]]))
            if not self._ensure_inside(float(xs.min()), float(xs.max()),
                                       float(ys.min()), float(ys.max())):
                self.too_big = True
            free = self._indices(fx, fy)
            occupied = self._indices(hx, hy)
            free = np.setdiff1d(free, occupied)
            delta = np.zeros(len(free) + len(occupied), dtype=np.float32)
            delta[:len(free)] = config.LIVE_MAP_MISS_LO
            delta[len(free):] = config.LIVE_MAP_HIT_LO
            index = np.concatenate((free, occupied))
            if len(index):
                self._apply(index, delta)
            self.scans_written += 1
            self.changed = True
        self._ms["eintragen"] = 1000.0 * (time.perf_counter() - started)

    def _apply(self, index, delta) -> None:
        """Add log-odds, established cells slower and capped per session (lock held)."""
        flat = self._lo.ravel()
        base = self._base.ravel()
        old = flat[index]
        b = base[index]
        established = np.abs(b) >= config.LIVE_MAP_ESTABLISHED_LO
        delta = np.where(established, delta * config.LIVE_MAP_ESTABLISHED_WEIGHT, delta)
        new = np.clip(old + delta, config.LIVE_MAP_LO_MIN, config.LIVE_MAP_LO_MAX)
        cap = config.LIVE_MAP_SESSION_CHANGE_MAX
        new = np.where(established, np.clip(new, b - cap, b + cap), new)
        flat[index] = new
        h, w = self._lo.shape
        iy, ix = index // w, index % w
        self._mark_dirty(int(ix.min()), int(iy.min()), int(ix.max()) + 1, int(iy.max()) + 1)

    def _new_grid(self, pose) -> None:
        half = config.LIVE_MAP_GROW_M
        n = int(math.ceil(2 * half / self.resolution))
        # cell corners on a fixed lattice, so saved and loaded maps line up
        ox = math.floor((pose[0] - half) / self.resolution) * self.resolution
        oy = math.floor((pose[1] - half) / self.resolution) * self.resolution
        self._origin = (ox, oy)
        self._lo = np.zeros((n, n), dtype=np.float32)
        self._base = np.zeros((n, n), dtype=np.float32)
        self._grew = True
        if self.frame_id is None:
            self.frame_id = uuid.uuid4().hex

    def _ensure_inside(self, xmin, xmax, ymin, ymax) -> bool:
        """Grow the grid so the box fits; False if that would exceed the maximum size."""
        res = self.resolution
        h, w = self._lo.shape
        ix0 = math.floor((xmin - self._origin[0]) / res)
        iy0 = math.floor((ymin - self._origin[1]) / res)
        ix1 = math.floor((xmax - self._origin[0]) / res) + 1
        iy1 = math.floor((ymax - self._origin[1]) / res) + 1
        if ix0 >= 0 and iy0 >= 0 and ix1 <= w and iy1 <= h:
            return True
        step = int(math.ceil(config.LIVE_MAP_GROW_M / res))
        left = step * math.ceil(-ix0 / step) if ix0 < 0 else 0
        bottom = step * math.ceil(-iy0 / step) if iy0 < 0 else 0
        right = step * math.ceil((ix1 - w) / step) if ix1 > w else 0
        top = step * math.ceil((iy1 - h) / step) if iy1 > h else 0
        limit = int(config.LIVE_MAP_MAX_SIZE_M / res)
        if w + left + right > limit or h + bottom + top > limit:
            return False
        self._lo = np.pad(self._lo, ((bottom, top), (left, right)))
        self._base = np.pad(self._base, ((bottom, top), (left, right)))
        self._origin = (self._origin[0] - left * res, self._origin[1] - bottom * res)
        if self._dirty is not None:
            self._dirty = [self._dirty[0] + left, self._dirty[1] + bottom,
                           self._dirty[2] + left, self._dirty[3] + bottom]
        self._grew = True
        return True

    def _indices(self, x, y) -> np.ndarray:
        h, w = self._lo.shape
        ix = np.floor((np.asarray(x) - self._origin[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(y) - self._origin[1]) / self.resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < w) & (iy < h)
        return np.unique(iy[inside] * w + ix[inside])

    def _mark_dirty(self, ix0, iy0, ix1, iy1) -> None:
        if self._dirty is None:
            self._dirty = [ix0, iy0, ix1, iy1]
        else:
            d = self._dirty
            self._dirty = [min(d[0], ix0), min(d[1], iy0), max(d[2], ix1), max(d[3], iy1)]

    # --- the step behind the gate ----------------------------------------------------

    def step(self, track, now: float | None = None) -> bool:
        """Feed one tracking result; True if a scan was written.

        ``now`` on the clock of the scan timestamps (``time.monotonic``)
        """
        with self._step_lock:
            return self._step(track, now)

    def _step(self, track, now: float | None) -> bool:
        if track is None or self.gate.state == ig.OFF:
            return False
        now = time.monotonic() if now is None else now
        omega, speed, wheel_speed = self._motion(track)
        lifted = self._lifted(track)
        a = track.match
        inp = ig.GateInputs(
            now=now,
            kind=track.kind,
            accepted=track.accepted,
            share=a.share if a is not None else 0.0,
            residual=a.residuum if a is not None else math.inf,
            known=a.points if a is not None else 0,
            known_share=track.known_share,
            constraint=track.constraint,
            jump_m=track.jump_m,
            turn_deg=math.degrees(track.turn_rad),
            omega=omega,
            speed=speed,
            wheel_speed=wheel_speed,
            odo_good=track.odo_good,
            odo_streak=track.odo_streak,
            gap=track.gap,
            age_s=(now - track.timestamp) if track.timestamp is not None else None,
            lifted=lifted,
            map_gen=track.map_gen,
            match_pose=(a.x, a.y, a.theta) if a is not None else None,
            odo_pose=track.odo_pose,
        )
        passed = self.gate.update(inp)
        if self.gate.state != ig.INTEGRATING:
            self._pending.clear()
            return False
        if not passed or a is None or track.scan is None:
            return False
        wrote = False
        for p in self._pending:
            p[2] += 1
        ready = [p for p in self._pending if p[2] >= config.LIVE_GATE_LAG_N]
        self._pending = [p for p in self._pending if p[2] < config.LIVE_GATE_LAG_N]
        for points, pose, _ in ready:
            self.integrate(points, pose)
            wrote = True
        pose = (a.x, a.y, a.theta)
        if self._moved(pose):
            self._pending.append([track.scan.points(), pose, 0])
            self._last_candidate = pose
        return wrote

    def _moved(self, pose) -> bool:
        last = self._last_candidate
        if last is None:
            return True
        return (math.hypot(pose[0] - last[0], pose[1] - last[1]) >= config.LIVE_MAP_MIN_MOVE_M
                or abs(math.degrees(_normalize(pose[2] - last[2]))) >= config.LIVE_MAP_MIN_MOVE_DEG)

    def _motion(self, track) -> tuple[float, float, float | None]:
        """Turn rate and speed from the lidar odometry, speed of the wheels (None: unknown)."""
        prev = self._prev_track
        self._prev_track = track
        if prev is None or prev.timestamp is None or track.timestamp is None:
            return 0.0, 0.0, None
        dt = track.timestamp - prev.timestamp
        if dt <= 1e-3:
            return 0.0, 0.0, None
        rel = _relative(prev.odo_pose, track.odo_pose)
        wheel_speed = None
        if self._wheel_pose_at is not None:
            w0, w1 = self._wheel_pose_at(prev.timestamp), self._wheel_pose_at(track.timestamp)
            if w0 is not None and w1 is not None:
                wheel_speed = math.hypot(w1[0] - w0[0], w1[1] - w0[1]) / dt
        return rel[2] / dt, math.hypot(rel[0], rel[1]) / dt, wheel_speed

    def _lifted(self, track) -> bool:
        """Lift detection with wheel travel and lidar rotation.

        the wheels say nothing about being carried, the lidar sees the room
        shift -- but the wheels' own heading is a quarter off on every turn,
        so the rotation comes from the lidar odometry
        """
        if self._wheel_pose_at is None or track.timestamp is None or track.scan is None:
            return False
        wheel = self._wheel_pose_at(track.timestamp)
        if wheel is None:
            self._hybrid = None
            return False
        if self._hybrid is None or self._hybrid_prev is None:
            self._hybrid = (0.0, 0.0, 0.0)
        else:
            w0, o0 = self._hybrid_prev
            dx, dy, _ = _relative(w0, wheel)
            dth = _normalize(track.odo_pose[2] - o0[2])
            self._hybrid = _compose(self._hybrid, (dx, dy, dth))
        self._hybrid_prev = (wheel, track.odo_pose)
        try:
            return bool(self._lift.check(track.scan, self._hybrid))
        except Exception:
            log.debug("lift detection failed", exc_info=True)
            return False

    # --- founding an empty map -----------------------------------------------------

    def bootstrap(self, measurement) -> bool:
        """Found a map from still scans of the lidar odometry; True once founded."""
        with self._step_lock:
            return self._bootstrap_step(measurement)

    def _bootstrap_step(self, measurement) -> bool:
        if self.gate.state != ig.START or measurement is None:
            return False
        if not getattr(measurement, "good", True) or getattr(measurement, "gap", False):
            self._bootstrap.clear()
            return False
        if self._bootstrap:
            last = self._bootstrap[-1]
            if last.nr == measurement.nr:
                return False
            rel = _relative(last.pose, measurement.pose)
            if (math.hypot(rel[0], rel[1]) > config.LIVE_BOOTSTRAP_MAX_MOVE_M
                    or abs(math.degrees(rel[2])) > config.LIVE_BOOTSTRAP_MAX_TURN_DEG):
                # it moved: a map must start from one sure spot
                self._bootstrap.clear()
        self._bootstrap.append(measurement)
        if len(self._bootstrap) < config.LIVE_BOOTSTRAP_SCANS:
            self.gate.reason = (f"beginnt die Karte ({len(self._bootstrap)}/"
                                f"{config.LIVE_BOOTSTRAP_SCANS}), bitte stillhalten")
            return False
        for m in self._bootstrap:
            self.integrate(m.scan.points(), m.pose)
        self._bootstrap.clear()
        log.info("Always-on mapping: new map founded at (%.2f, %.2f)",
                 measurement.pose[0], measurement.pose[1])
        return True

    # --- snapshot for localization and planner ----------------------------------------

    def snapshot(self) -> OccupancyMap | None:
        """Current state as an immutable map; only the changed part is recomputed."""
        # one at a time: each consumes the dirty box the next one builds on
        with self._snap_lock:
            return self._snapshot()

    def _snapshot(self) -> OccupancyMap | None:
        started = time.perf_counter()
        with self._lock:
            gen = self._gen
            if self._lo is None:
                return None
            lo = self._lo.copy()
            origin = self._origin
            dirty = None if self._grew else (tuple(self._dirty) if self._dirty else None)
            unchanged = not self._grew and self._dirty is None and self._previous is not None
            self._dirty = None
            self._grew = False
            previous = self._previous
        if unchanged:
            return previous
        occupancy = np.full(lo.shape, -1, dtype=np.int8)
        occupancy[lo <= config.LIVE_MAP_FREE_LO] = 0
        occupancy[lo >= config.LIVE_MAP_OCC_LO] = 100
        occ_map = OccupancyMap.derived(occupancy, self.resolution, origin,
                                       previous if dirty is not None else None, dirty,
                                       name=self.name)
        with self._lock:
            if gen != self._gen:
                # the grid was replaced meanwhile: this state belongs to the old one
                return None
            occ_map.frame_id = self.frame_id
            self._previous = occ_map
        self._ms["schnappschuss"] = 1000.0 * (time.perf_counter() - started)
        return occ_map

    # --- state ----------------------------------------------------------------------

    def pause(self, by: str) -> None:
        """Write nothing while ``by`` holds (slam run, the user); idempotent."""
        with self._step_lock:
            if self.gate.state != ig.OFF and by not in self.gate.paused_by:
                self.gate.pause(by)
                self._pending.clear()

    def resume(self, by: str) -> None:
        with self._step_lock:
            if by in self.gate.paused_by:
                self.gate.resume(by)

    @property
    def integrating(self) -> bool:
        return self.gate.state == ig.INTEGRATING

    def status(self) -> dict:
        size = self.size_m()
        g = self.gate.status()
        return {
            "aktiv": self.gate.state not in (ig.OFF, ig.PAUSED),
            "zustand": g["zustand"],
            "grund": g["grund"],
            "bestaetigt": g["bestaetigt"],
            "von": g["von"],
            "name": self.name,
            "groesse_m": [round(size[0], 1), round(size[1], 1)] if size else None,
            "aufnahmen": self.scans_written,
            "eingefroren_mal": g["eingefroren_mal"],
            "pausiert_von": g["pausiert_von"],
            "zu_gross": self.too_big,
            "ms": {k: round(v, 1) for k, v in self._ms.items()},
        }
