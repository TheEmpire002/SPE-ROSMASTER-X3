"""Local avoidance -- without a map. For the explorer (step F2), for
follow/come here and homing (step F3).

Four building blocks, all pure computation on lidar scans and the velocity
measured by the board, so they can be checked without a robot and replayed
on recorded drives:

**Short-term memory.** The lidar does not see everything in every scan: thin
chair legs only show up sometimes, black or shiny surfaces swallow single
beams, and below ``LIDAR_MIN_M`` (dead zone right at the robot) it does not
measure at all. That is why the memory keeps the points of the last few
seconds and integrates its own pose from the board velocity. Only briefly and
only near points: the integrated pose drifts, and the lidar sees things
farther away again anyway. Without a real measurement (board delivers
nothing, the odometry computes from the command) it stays empty -- otherwise
a blocked robot would believe it is driving and push ghost points ahead of
itself.

**Gaps.** Approach doors and passages in the middle instead of narrowly past
the frame: connected areas are formed from the free directions, and their
middle or a direction with distance to the edge is chosen.

**Arc planning.** Instead of stopping in front of an obstacle and turning on
the spot, the explorer computes a few arcs (turn rate, sideways sliding) a
good second ahead and checks with the robot's outline whether there is
clearance everywhere. Among the safe arcs, the one that keeps the chosen
heading, leaves clearance and leads far wins. No randomness: same pose, same
command -- that makes replaying real drives meaningful.

**Lift.** If the robot is picked up or carried, the lidar image changes all
around at once without the robot driving itself. It then stops and forgets
what it has learned (memory, heading) until the image is calm again.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .. import config


def _normalize_angle(a: float) -> float:
    """To -pi..pi."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _relative_pose(from_, after) -> tuple[float, float, float]:
    """Pose ``after`` in the frame of ``from_`` (dx, dy, dth)."""
    dx, dy = after[0] - from_[0], after[1] - from_[1]
    c, s = math.cos(from_[2]), math.sin(from_[2])
    return dx * c + dy * s, -dx * s + dy * c, _normalize_angle(after[2] - from_[2])


def _frame_to_parent(points, pose) -> np.ndarray:
    """Points from the frame of a pose (x, y, th) into the frame above it."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((
        x + points[:, 0] * c - points[:, 1] * s,
        y + points[:, 0] * s + points[:, 1] * c,
    ))


def _parent_to_frame(points, pose) -> np.ndarray:
    """Inverse of ``_frame_to_parent``: points into the frame of the pose."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


# --- Short-term memory ----------------------------------------------------------


class ShortTermMemory:
    """Lidar points of the last few seconds, transformed into the current frame.

    ``advance(dt)`` every tick, ``points(scan)`` returns the points of the
    scan plus the remembered ones. It computes once per new scan; in between
    the same result comes back -- just like ``scan.points()``, which the
    explorer used before.
    """

    def __init__(self, odometry=None) -> None:
        self._odometry = odometry
        # Own pose (x, y, th) in an arbitrarily chosen frame. Keeps running
        # when cleared: the chosen heading refers to it.
        self.pose = (0.0, 0.0, 0.0)
        self.measuring = False
        self._recent_scans: list[tuple[float, np.ndarray]] = []  # (time, points in the pose frame)
        self._last_time: float | None = None
        self._result = np.zeros((0, 2))
        self.remembered = 0  # remembered points in the last result

    @property
    def heading(self) -> float | None:
        """Current heading in its own frame, None without measurement."""
        return self.pose[2] if self.measuring else None

    def clear(self) -> None:
        self._recent_scans.clear()
        self._last_time = None
        self.remembered = 0

    def advance(self, dt: float) -> None:
        """Advance the own pose by one tick."""
        odo = self._odometry
        self.measuring = odo is not None and bool(getattr(odo, "actually_measuring", False))
        if not self.measuring:
            self.clear()
            return
        vx, vy, om = odo.velocity()
        x, y, th = self.pose
        # Using the angle at the middle of the tick -- more accurate in curves.
        center = th + om * dt / 2.0
        c, s = math.cos(center), math.sin(center)
        self.pose = (x + (vx * c - vy * s) * dt, y + (vx * s + vy * c) * dt,
                     _normalize_angle(th + om * dt))

    def points(self, scan) -> np.ndarray:
        """Points of the scan and remembered points in the robot frame (Nx2)."""
        if not self.measuring:
            self.remembered = 0
            self._result = scan.points()
            return self._result
        if scan.timestamp == self._last_time:
            return self._result
        self._last_time = scan.timestamp
        current = scan.points()

        # Drop old scans, remember the current one -- only what is close
        # enough to ever matter again.
        limit = scan.timestamp - config.AVOID_MEMORY_S
        self._recent_scans = [(z, p) for z, p in self._recent_scans if z >= limit]
        old = (np.concatenate([p for _, p in self._recent_scans])
               if self._recent_scans else np.zeros((0, 2)))
        if len(current):
            close = np.hypot(current[:, 0], current[:, 1]) < 2.0 * config.AVOID_MEMORY_M
            self._recent_scans.append((scan.timestamp, _frame_to_parent(current[close], self.pose)))

        if len(old) == 0:
            self.remembered = 0
            self._result = current
            return current
        old = _parent_to_frame(old, self.pose)
        distance = np.hypot(old[:, 0], old[:, 1])
        keep = distance < config.AVOID_MEMORY_M

        # Clearing: if the lidar now sees clearly farther in this direction,
        # the thing is gone (person left, door open, measurement error).
        # No beam in that direction or point in the dead zone: keep it --
        # that is exactly what the memory is for.
        view = np.full(360, np.inf)
        if len(current):
            slot = np.round(np.degrees(np.arctan2(current[:, 1], current[:, 0]))).astype(int) % 360
            np.minimum.at(view, slot, np.hypot(current[:, 0], current[:, 1]))
        slot_old = np.round(np.degrees(np.arctan2(old[:, 1], old[:, 0]))).astype(int) % 360
        s = view[slot_old]
        see_through = np.isfinite(s) & (s > distance + config.AVOID_CLEAR_M)
        see_through &= distance >= config.LIDAR_MIN_M
        keep &= ~see_through

        self.remembered = int(keep.sum())
        self._result = np.concatenate((current, old[keep])) if self.remembered else current
        return self._result


# --- Gaps ------------------------------------------------------------------------


def free_per_direction(points, directions_deg, half_width: float) -> np.ndarray:
    """``free_in_direction`` for many directions at once (infinity = free)."""
    a = np.radians(np.asarray(directions_deg, dtype=float))[:, None]
    if len(points) == 0:
        return np.full(a.shape[0], np.inf)
    c, s = np.cos(a), np.sin(a)
    ahead = points[None, :, 0] * c + points[None, :, 1] * s
    lateral = -points[None, :, 0] * s + points[None, :, 1] * c
    ahead = np.where((ahead > 0.0) & (np.abs(lateral) < half_width), ahead, np.inf)
    return ahead.min(axis=1)


def gaps(points, half_width: float, threshold: float, max_range: float) -> list[tuple[float, float]]:
    """Good directions (degrees, free in m) from the free areas all around.

    An area is a connected run of directions in which at least ``threshold``
    is free. From each, the middle and every 10° direction at
    ``AVOID_GAP_EDGE_DEG`` distance from the edges are candidates. That way a
    door is approached in the middle, and a large free area still leaves a
    choice for randomness.
    """
    grid = config.AVOID_GAP_GRID_DEG
    margin = config.AVOID_GAP_EDGE_DEG
    directions = np.arange(-180, 180, grid)
    free = np.minimum(free_per_direction(points, directions, half_width), max_range)
    good = free >= threshold
    n = len(directions)

    def normalized(r: float) -> float:
        return (r + 180.0) % 360.0 - 180.0

    def free_at(r: float) -> float:
        return float(free[int(round((normalized(r) + 180) / grid)) % n])

    if good.all():
        return [(float(r), free_at(r)) for r in range(-180, 180, 10)]
    if not good.any():
        return []

    # Search areas all around, starting after a bad direction, so that no
    # area is split at -180°/180°.
    start = int(np.argmin(good))
    spans = []  # (first index, length)
    length = 0
    for k in range(1, n + 1):
        i = (start + k) % n
        if good[i]:
            length += 1
        elif length:
            spans.append(((i - length) % n, length))
            length = 0

    result: dict[float, float] = {}
    for first, length in spans:
        from_ = float(directions[first])
        until = from_ + (length - 1) * grid  # may go beyond 180
        center = normalized(round((from_ + until) / 2.0 / grid) * grid)
        result[center] = free_at(center)
        r = math.ceil((from_ + margin) / 10.0) * 10
        while r <= until - margin:
            result[normalized(r)] = free_at(r)
            r += 10
    return sorted(result.items())


def choose_direction(points, goal_deg: float, needed: float, half_width: float,
                     last_side: float = 0.0) -> tuple[float, float] | None:
    """Free driving direction as close as possible to ``goal_deg``: (degrees, free in m).

    For follow and homing: there is a goal there (person, start), and the
    mecanum drive can slide diagonally past an obstacle without looking away.
    Free means: ``needed`` metres in the driving corridor, with
    ``AVOID_MARGIN_M`` in addition to the half width.

    - If the goal direction itself is free, it is returned unchanged.
    - Otherwise the nearest free direction up to ``DETOUR_MAX_DEG`` beside it.
      The last chosen side gets a head start of ``DETOUR_SIDE_DEG``,
      otherwise it oscillates between left and right in front of a wide
      obstacle.
    - Distance to the edge of the free area: at least ``DETOUR_EDGE_DEG``;
      if the area is narrower (door), its middle.

    None if nothing is free -- then stop, do not guess.
    """
    grid = config.DETOUR_GRID_DEG
    n = int(config.DETOUR_MAX_DEG // grid)
    shift = np.arange(-n, n + 1) * grid
    free = free_per_direction(points, goal_deg + shift, half_width + config.AVOID_MARGIN_M)
    good = free >= needed
    if good[n]:
        return float(goal_deg), float(free[n])
    if not good.any():
        return None

    cost = np.abs(shift).astype(float)
    if last_side:
        cost += np.where(np.sign(shift) == -np.sign(last_side), config.DETOUR_SIDE_DEG, 0.0)
    i = int(np.argmin(np.where(good, cost, np.inf)))

    # Find the connected free area around i and move away from its edge.
    a = i
    while a > 0 and good[a - 1]:
        a -= 1
    b = i
    while b < len(good) - 1 and good[b + 1]:
        b += 1
    k = int(math.ceil(config.DETOUR_EDGE_DEG / grid))
    if b - a < 2 * k:
        i = (a + b) // 2
    else:
        i = min(max(i, a + k), b - k)
    return float(_normalize_angle(math.radians(goal_deg + shift[i])) * 180.0 / math.pi), float(free[i])


# --- Arc planning ---------------------------------------------------------------


def _outline_distance(lx, ly):
    """Distance of points (in the robot frame) to its outline."""
    ex = np.maximum(np.maximum(-config.AVOID_REAR_M - lx, lx - config.AVOID_FRONT_M), 0.0)
    ey = np.maximum(np.abs(ly) - config.AVOID_HALF_WIDTH_M, 0.0)
    return np.hypot(ex, ey)


def plan_arc(points, vx_max: float, heading_error: float, last_omega: float):
    """Choose the best safe arc: ``((vx, vy, omega), clearance in m)`` or None.

    ``heading_error``: by how much (rad, left positive) the chosen heading
    deviates from the current heading; 0 if none is known -- then simply
    steering as little as possible counts. None means: no arc is safe, the
    explorer should turn around.
    """
    vx_min = 0.35 * config.EXPLORE_SPEED
    levels = sorted({max(vx_min, vx_max), max(vx_min, 0.7 * vx_max)}, reverse=True)
    cand = np.array([
        (v, vy, om)
        for v in levels for vy in config.AVOID_VYS for om in config.AVOID_OMEGAS
    ])
    vx, vy, om = cand[:, 0:1], cand[:, 1:2], cand[:, 2:3]

    # Poses along the arcs (exact for constant velocities).
    t = np.arange(0.0, config.AVOID_HORIZON_S + 1e-9, config.AVOID_STEP_S)[None, :]
    th = om * t
    straight = np.abs(om) < 1e-6
    om_s = np.where(straight, 1.0, om)
    sin_t, cos_t = np.sin(th), np.cos(th)
    px = np.where(straight, vx * t, (vx * sin_t - vy * (1.0 - cos_t)) / om_s)
    py = np.where(straight, vy * t, (vx * (1.0 - cos_t) + vy * sin_t) / om_s)

    # Only points that can ever come closer than AVOID_CLEARANCE_GOOD_M -- the
    # rest does not change the (capped) score.
    vy_max = max(map(abs, config.AVOID_VYS)) or 1.0
    max_travel = (levels[0] + vy_max) * config.AVOID_HORIZON_S
    footprint = math.hypot(max(config.AVOID_FRONT_M, config.AVOID_REAR_M),
                        config.AVOID_HALF_WIDTH_M)
    d = np.hypot(points[:, 0], points[:, 1]) if len(points) else np.zeros(0)
    close = points[d < max_travel + footprint + config.AVOID_CLEARANCE_GOOD_M] if len(points) else np.zeros((0, 2))

    if len(close):
        # shape: candidate x step x point
        dx = close[None, None, :, 0] - px[:, :, None]
        dy = close[None, None, :, 1] - py[:, :, None]
        c, s = cos_t[:, :, None], sin_t[:, :, None]
        distance = _outline_distance(dx * c + dy * s, -dx * s + dy * c).min(axis=2)
        clearance = distance.min(axis=1)
        now = float(distance[0, 0])  # step 0 is the same for all arcs
    else:
        clearance = np.full(len(cand), np.inf)
        now = math.inf
    # Safe if enough margin remains everywhere -- or, if something is already
    # closer now (door frame, passing sideways), if it at least gets no closer.
    # Otherwise it would stand next to every frame and turn around.
    confident = (clearance >= config.AVOID_MARGIN_M) | (
        (now < config.AVOID_MARGIN_M) & (clearance >= now - 0.005))
    if not confident.any():
        return None

    # Range: how far it is free from the end of the arc in its direction.
    end_x, end_y, end_th = px[:, -1], py[:, -1], th[:, -1]
    reach_good = config.AVOID_RANGE_GOOD_M
    far = (points[d < max_travel + reach_good + config.EXPLORE_HALF_WIDTH_M]
            if len(points) else np.zeros((0, 2)))
    if len(far):
        dx = far[None, :, 0] - end_x[:, None]
        dy = far[None, :, 1] - end_y[:, None]
        c, s = np.cos(end_th)[:, None], np.sin(end_th)[:, None]
        ahead = dx * c + dy * s
        lateral = -dx * s + dy * c
        ahead = np.where((ahead > 0.0) & (np.abs(lateral) < config.EXPLORE_HALF_WIDTH_M), ahead, np.inf)
        reach = np.minimum(ahead.min(axis=1), reach_good)
    else:
        reach = np.full(len(cand), reach_good)

    rest = np.abs((heading_error - end_th + np.pi) % (2.0 * np.pi) - np.pi)
    om_max = max(map(abs, config.AVOID_OMEGAS)) or 1.0
    value = (
        -config.AVOID_WEIGHT_HEADING * rest / np.pi
        + config.AVOID_WEIGHT_CLEARANCE * np.minimum(clearance, config.AVOID_CLEARANCE_GOOD_M) / config.AVOID_CLEARANCE_GOOD_M
        + config.AVOID_WEIGHT_SPEED * cand[:, 0] / config.EXPLORE_SPEED
        + config.AVOID_WEIGHT_RANGE * reach / reach_good
        - config.AVOID_WEIGHT_JERK * np.abs(cand[:, 2] - last_omega) / (2.0 * om_max)
        - config.AVOID_WEIGHT_SIDEWAYS * np.abs(cand[:, 1]) / vy_max
    )
    value = np.where(confident, value, -np.inf)
    i = int(np.argmax(value))  # on a tie the first one: no randomness
    return (float(cand[i, 0]), float(cand[i, 1]), float(cand[i, 2])), float(clearance[i])


# --- Lift -----------------------------------------------------------------------

_SECTORS = 36


def _sectors(points) -> np.ndarray:
    """Median distance per 10° sector, NaN without measurement."""
    values = np.full(_SECTORS, np.nan)
    if len(points) == 0:
        return values
    w = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360.0
    slot = (w // (360.0 / _SECTORS)).astype(int) % _SECTORS
    r = np.hypot(points[:, 0], points[:, 1])
    for k in range(_SECTORS):
        selection = r[slot == k]
        if len(selection):
            values[k] = float(np.median(selection))
    return values


#: At most this many points per scan for the rigid-fit check (the matcher
#: compares every point with every other).
_FIT_MAX_POINTS = 150
#: Start guesses for the fit, around the wheel estimate: the wheels can be
#: off by tens of degrees when turning on a slippery floor.
_FIT_START_ANGLES_DEG = (0.0, -12.0, 12.0, -25.0, 25.0, -40.0, 40.0)


def _thin(points, limit: int) -> np.ndarray:
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.hypot(points[:, 0], points[:, 1]) < 6.0]
    if len(points) > limit:
        points = points[:: math.ceil(len(points) / limit)]
    return points


def _fit_share(now, old, pose) -> float:
    """Share of ``now`` points within LIFT_FIT_INLIER_M of ``old`` at this pose."""
    moved = _frame_to_parent(now, pose)
    diff = moved[:, None, :] - old[None, :, :]
    nearest = np.sqrt(np.einsum("ijk,ijk->ij", diff, diff).min(axis=1))
    return float((nearest < config.LIFT_FIT_INLIER_M).mean())


def _rigid_motion_explains(now_points, old_points, wheel_rel) -> bool:
    """Can a motion of the robot on the floor explain the change between scans?

    ``old_points`` in the old robot frame, ``wheel_rel`` the motion the wheels
    reported (pose of the new frame in the old one). The new scan is aligned
    onto the old one, starting from the wheel estimate turned by a few
    angles and from "did not move at all". Explained means: most points fit
    (no tilt, no lidar seeing floor or ceiling) and the robot was not
    displaced further than the wheels said -- unless it hardly moved at all,
    which is wheels spinning in place, not being carried.
    """
    from .scan_matching import match

    now = _thin(now_points, _FIT_MAX_POINTS)
    old = _thin(old_points, _FIT_MAX_POINTS)
    if len(now) < 40 or len(old) < 40:
        return False
    starts = [
        (wheel_rel[0], wheel_rel[1], wheel_rel[2] + math.radians(d))
        for d in _FIT_START_ANGLES_DEG
    ] + [(0.0, 0.0, 0.0)]
    best_share, best_pose = 0.0, None
    for start in starts:
        fit = match(now, old, start, iterations=20)
        if fit is None:
            continue
        pose = (fit.x, fit.y, fit.theta)
        share = _fit_share(now, old, pose)
        if share > best_share:
            best_share, best_pose = share, pose
    if best_pose is None or best_share < config.LIFT_FIT_MIN_SHARE:
        return False
    unreported = _relative_pose(wheel_rel, best_pose)
    if math.hypot(unreported[0], unreported[1]) <= config.LIFT_CARRY_M:
        return True
    # Displaced against the wheels: carried -- unless the lidar says it did
    # not really move (then the wheels were spinning).
    return math.hypot(best_pose[0], best_pose[1]) < config.LIFT_CARRY_M


class LiftDetector:
    """Notices when the robot is lifted or carried.

    Compares each new scan with an earlier one, with its own motion in
    between subtracted. A person walking past changes few sectors; if the
    robot is tilted or displaced, the image jumps almost everywhere.

    With a measured pose it compares with the scan from ``LIFT_WINDOW_S``
    ago: someone carrying it slowly shifts the image only a little from scan
    to scan, but clearly over a second -- and the wheels report none of it.
    Without measurement only the immediately preceding scan remains,
    otherwise its own driving would already look like being carried.

    Limits: if it is lifted straight up while level, the walls stay equally
    far away -- it does not see that. If the wheels then spin in the air,
    the stuck detection kicks in (board measures motion, lidar image stands
    still), and finally giving up.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.lifted = False
        self.share = 0.0
        self._history: deque = deque()  # (time, points, pose or None)
        self._last_time: float | None = None
        self._restless = 0
        self._still_since: float | None = None

    def check(self, scan, pose) -> bool:
        """Check with the current scan. ``pose`` only if measured."""
        if scan.timestamp == self._last_time:
            return self.lifted
        self._last_time = scan.timestamp
        points = scan.points()
        prior = self._comparison_scan(scan.timestamp, pose)
        self._history.append((scan.timestamp, points, pose))
        if prior is None:
            return self.lifted

        _time, old_points, old_pose = prior
        old_raw = old_points
        wheel_rel = (0.0, 0.0, 0.0)
        if pose is not None and old_pose is not None and len(old_points):
            # Previous scan into the current frame: its own driving is no jump.
            wheel_rel = _relative_pose(old_pose, pose)
            old_points = _parent_to_frame(old_points, wheel_rel)
        now, old = _sectors(points), _sectors(old_points)
        j_ok, a_ok = ~np.isnan(now), ~np.isnan(old)
        both = j_ok & a_ok
        # A sector seen in only one of the two scans counts only if what was
        # seen is near: far sectors often have no echo at all (window, open
        # door) and would flicker in and out.
        near = config.LIFT_NEAR_M
        appeared = j_ok & ~a_ok & (np.nan_to_num(now, nan=np.inf) < near)
        vanished = a_ok & ~j_ok & (np.nan_to_num(old, nan=np.inf) < near)
        compared = int((both | appeared | vanished).sum())
        if compared < 6:
            # Too little to see (open field, lidar covered) -- no verdict.
            self.share = 0.0
        else:
            jump = appeared | vanished
            jump[both] = np.abs(now[both] - old[both]) > config.LIFT_JUMP_M
            self.share = float(jump.sum()) / compared
        if self.share >= config.LIFT_SHARE and _rigid_motion_explains(
            points, old_raw, wheel_rel
        ):
            # The robot moved on the floor, the wheel odometry just got it
            # wrong (rotation error, wheels spinning while stuck): no lift.
            self.share = 0.0

        if not self.lifted:
            self._restless = self._restless + 1 if self.share >= config.LIFT_SHARE else 0
            needed = config.LIFT_SCANS if pose is not None else config.LIFT_SCANS_UNMEASURED
            if self._restless >= needed:
                self.lifted = True
                self._still_since = None
        elif self.share < config.LIFT_STILL_SHARE:
            if self._still_since is None:
                self._still_since = scan.timestamp
            if scan.timestamp - self._still_since >= config.LIFT_REST_S:
                self.lifted = False
                self._restless = 0
        else:
            self._still_since = None
        return self.lifted

    def _comparison_scan(self, timestamp: float, pose):
        """The scan the new one is compared with (see class docstring)."""
        while self._history and timestamp - self._history[0][0] > config.LIFT_WINDOW_S:
            self._history.popleft()
        if not self._history:
            return None
        if pose is None:
            return self._history[-1]
        # The oldest scan in the window with a pose; if there is none (the
        # measurement has only just come back), the previous one.
        for entry in self._history:
            if entry[2] is not None:
                return entry
        return self._history[-1]
