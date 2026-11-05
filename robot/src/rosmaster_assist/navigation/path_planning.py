"""Paths through the map: to a goal and to the nearest unknown spot.

Planning happens on a coarser grid than the map
(``NAV_PLAN_RESOLUTION_M``), with A* over eight neighbours. A cell is
drivable if it is free and at least ``NAV_SAFETY_M`` away from the nearest
wall -- then the robot fits through there whatever its orientation.
Unknown is never drivable: what the map does not know could be a wall.
Proximity to walls costs extra (up to ``NAV_PREFERRED_DISTANCE_M``), so
paths lead through the middle of doors.

With a breadcrumb trail (``corridor``): the cells along it cost only
``BREADCRUMB_COST_FACTOR`` and are drivable from ``BREADCRUMB_MIN_CLEARANCE_M``
on -- the robot was physically there. Occupied stays blocked in any case.

If no path leads through known free space, ``plan_path`` guesses
(``NAV_GUESS_PATHS``): unknown cells and spots down to
``NAV_GUESS_CLEARANCE_M`` from a wall become drivable, unknown ones at
``NAV_GUESS_UNKNOWN_COST`` times the price. The result is marked
``guessed`` -- the robot drives it and looks on site whether it is free.

Frontiers are free spots that border on the unknown: open doors,
corridors, shadows behind furniture. When the mapping drive heads for the
nearest one, the map grows there -- until no reachable one is left. The
search is Dijkstra from the robot; the first frontier reached is the one
with the shortest path.

The computation takes tenths of a second on the Pi. Drive loops running at
20 Hz therefore call it via ``BackgroundPlanner`` and stop meanwhile.
"""

from __future__ import annotations

import heapq
import logging
import math
from concurrent.futures import Future, ThreadPoolExecutor
from typing import NamedTuple

import numpy as np

from .. import config

log = logging.getLogger(__name__)

_SQRT2 = math.sqrt(2.0)


class PlannedPath(NamedTuple):
    points: list[tuple[float, float]]   # waypoints in the map frame, without the start pose
    length: float                       # metres
    target: tuple[float, float]           # actual goal (possibly shifted)
    gap: tuple[float, float] | None = None  # centre of the frontier being approached
    guessed: bool = False               # leads through unknown or narrow spots


class Grid:
    """Planning grid built from a map, with drivable cells and costs."""

    def __init__(self, occ_map, blocked=(), corridor=None, guess: bool = False) -> None:
        f = max(1, int(round(config.NAV_PLAN_RESOLUTION_M / occ_map.resolution)))
        self.resolution = occ_map.resolution * f
        self.origin = occ_map.origin
        h, w = occ_map.height, occ_map.width
        self.height, self.width = -(-h // f), -(-w // f)
        ph, pw = self.height * f - h, self.width * f - w

        def blocks(field_, margin):
            padded = np.pad(field_, ((0, ph), (0, pw)), constant_values=margin)
            return padded.reshape(self.height, f, self.width, f)

        self.occupied = blocks(occ_map.occupied, False).any(axis=(1, 3))
        self.unknown = blocks(occ_map.unknown, True).all(axis=(1, 3))
        self.free = blocks(occ_map.free, False).any(axis=(1, 3)) & ~self.occupied
        distance = blocks(occ_map.distance, 0.0).min(axis=(1, 3))

        # Temporarily blocked spots (on the last attempt something stood there
        # that the map does not know): treated like a wall.
        if len(blocked):
            gy, gx = np.mgrid[0:self.height, 0:self.width]
            mx = self.origin[0] + (gx + 0.5) * self.resolution
            my = self.origin[1] + (gy + 0.5) * self.resolution
            for sx, sy in blocked:
                d = np.hypot(mx - sx, my - sy) - config.NAV_BLOCK_RADIUS_M
                distance = np.minimum(distance, np.maximum(d, 0.0))
        self.distance = distance

        self.drivable = self.free & (distance >= config.NAV_SAFETY_M)
        span = max(config.NAV_PREFERRED_DISTANCE_M - config.NAV_SAFETY_M, 1e-3)
        proximity = np.clip((config.NAV_PREFERRED_DISTANCE_M - distance) / span, 0.0, 1.0)
        self.cost = 1.0 + config.NAV_WALL_COST * proximity
        if guess:
            # what the map does not know may well be free: allowed, but dear
            maybe = (self.free | self.unknown) & ~self.occupied & (
                distance >= config.NAV_GUESS_CLEARANCE_M)
            self.drivable |= maybe
            self.cost = np.where(self.unknown, self.cost * config.NAV_GUESS_UNKNOWN_COST,
                                 self.cost)
        if corridor is not None and len(corridor):
            trail = self._along(np.asarray(corridor, dtype=float).reshape(-1, 2))
            self.drivable |= (trail & self.free & ~self.occupied
                              & (distance >= config.BREADCRUMB_MIN_CLEARANCE_M))
            self.cost = np.where(trail, self.cost * config.BREADCRUMB_COST_FACTOR, self.cost)

    def _along(self, points: np.ndarray) -> np.ndarray:
        """Cells within ``BREADCRUMB_CORRIDOR_M`` of the trail points."""
        near = np.zeros((self.height, self.width), dtype=bool)
        ix = np.floor((points[:, 0] - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((points[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < self.width) & (iy < self.height)
        near[iy[inside], ix[inside]] = True
        r = int(math.ceil(config.BREADCRUMB_CORRIDOR_M / self.resolution))
        grown = near.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                shifted = np.zeros_like(near)
                ys = slice(max(0, dy), self.height + min(0, dy))
                yd = slice(max(0, -dy), self.height + min(0, -dy))
                xs = slice(max(0, dx), self.width + min(0, dx))
                xd = slice(max(0, -dx), self.width + min(0, -dx))
                shifted[ys, xs] = near[yd, xd]
                grown |= shifted
        return grown

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def world(self, ix: int, iy: int) -> tuple[float, float]:
        return (self.origin[0] + (ix + 0.5) * self.resolution,
                self.origin[1] + (iy + 0.5) * self.resolution)

    def inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    # --- Search --------------------------------------------------------------------

    def nearest_drivable(self, ix: int, iy: int, reach_m: float) -> tuple[int, int] | None:
        """Nearest drivable cell (breadth-first search through everything that is not a wall)."""
        if self.inside(ix, iy) and self.drivable[iy, ix]:
            return ix, iy
        limit = int(math.ceil(reach_m / self.resolution))
        seen = {(ix, iy)}
        margin = [(ix, iy)]
        for _ in range(limit):
            new = []
            for cx, cy in margin:
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if (nx, ny) in seen or not self.inside(nx, ny) or self.occupied[ny, nx]:
                        continue
                    if self.drivable[ny, nx]:
                        return nx, ny
                    seen.add((nx, ny))
                    new.append((nx, ny))
            margin = new
        return None

    def search(self, start: tuple[int, int], target: tuple[int, int] | None = None,
               goals: np.ndarray | None = None) -> list[tuple[int, int]] | None:
        """A* to ``target``, or Dijkstra to the nearest cell with ``goals[iy, ix]``."""
        w = self.width
        drivable = self.drivable.ravel().tolist()
        cost = self.cost.ravel().tolist()
        goal_set = goals.ravel().tolist() if goals is not None else None
        s = start[1] * w + start[0]
        z = target[1] * w + target[0] if target is not None else -1
        zx, zy = target if target is not None else (0, 0)

        def h(i: int) -> float:
            if target is None:
                return 0.0
            dx, dy = abs(i % w - zx), abs(i // w - zy)
            return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)

        g = {s: 0.0}
        previous = {s: -1}
        open_set = [(h(s), s)]
        done = set()
        neighbors = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                    (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2))
        n = self.height * w
        while open_set:
            _, i = heapq.heappop(open_set)
            if i in done:
                continue
            done.add(i)
            if i == z or (goal_set is not None and goal_set[i]):
                path = []
                while i != -1:
                    path.append((i % w, i // w))
                    i = previous[i]
                return path[::-1]
            ix, gi = i % w, g[i]
            for dx, dy, step in neighbors:
                nx = ix + dx
                if nx < 0 or nx >= w:
                    continue
                j = i + dy * w + dx
                if j < 0 or j >= n or not drivable[j] or j in done:
                    continue
                if dx and dy and not (drivable[i + dx] and drivable[i + dy * w]):
                    continue  # do not cut diagonally past a corner
                new = gi + step * 0.5 * (cost[i] + cost[j])
                if new < g.get(j, float("inf")):
                    g[j] = new
                    previous[j] = i
                    heapq.heappush(open_set, (new + h(j), j))
        return None

    def smooth(self, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """Drop superfluous intermediate points.

        A shortcut is only taken where the straight line keeps at least as
        much distance to the wall as the planned path at its narrowest point
        in between -- otherwise the door centring produced by the costs would
        be lost.
        """
        if len(cells) <= 2:
            return cells
        distance = [float(self.distance[y, x]) for x, y in cells]
        result = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1:
                needed = min(distance[i:j + 1]) - 1e-6
                if self._line_free(cells[i], cells[j], needed):
                    break
                j -= 1
            result.append(cells[j])
            i = j
        return result

    def _line_free(self, a, b, needed: float) -> bool:
        steps = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2
        for k in range(steps + 1):
            t = k / max(steps, 1)
            x = int(round(a[0] + (b[0] - a[0]) * t))
            y = int(round(a[1] + (b[1] - a[1]) * t))
            if not self.drivable[y, x] or self.distance[y, x] < needed:
                return False
        return True

    def gaps(self) -> list[tuple[list[tuple[int, int]], tuple[float, float]]]:
        """Connected free -> unknown transitions, large enough for a passage.

        Returns the cells and the centre (world) of each frontier.
        """
        unk = self.unknown
        limit = np.zeros_like(unk)
        limit[1:, :] |= unk[:-1, :]
        limit[:-1, :] |= unk[1:, :]
        limit[:, 1:] |= unk[:, :-1]
        limit[:, :-1] |= unk[:, 1:]
        margin = self.free & limit
        ys, xs = np.nonzero(margin)
        open_set = set(zip(xs.tolist(), ys.tolist()))
        min_cells = max(1, int(math.ceil(config.FRONTIER_MIN_M / self.resolution)))
        result = []
        while open_set:
            first = open_set.pop()
            group = [first]
            stack = [first]
            while stack:
                cx, cy = stack.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (cx + dx, cy + dy)
                        if nb in open_set:
                            open_set.discard(nb)
                            group.append(nb)
                            stack.append(nb)
            if len(group) >= min_cells:
                mx = sum(p[0] for p in group) / len(group)
                my = sum(p[1] for p in group) / len(group)
                result.append((group, self.world(mx, my)))
        return result


def _path_from(grid: Grid, start_xy, cells, end=None) -> PlannedPath:
    """Waypoints from cells; ``end``: end exactly there instead of at the cell centre."""
    cells = grid.smooth(cells)
    points = [grid.world(x, y) for x, y in cells[1:]] or [grid.world(*cells[0])]
    if end is not None:
        points[-1] = (float(end[0]), float(end[1]))
    length = 0.0
    previous = start_xy
    for p in points:
        length += math.hypot(p[0] - previous[0], p[1] - previous[1])
        previous = p
    return PlannedPath(points, length, points[-1])


def plan_path(occ_map, start_xy, goal_xy, blocked=(), corridor=None) -> PlannedPath | None:
    """Path from the robot to the goal: a sure one, else a guessed one; None if neither."""
    path = _plan(Grid(occ_map, blocked, corridor), start_xy, goal_xy)
    if path is None and config.NAV_GUESS_PATHS:
        path = _plan(Grid(occ_map, blocked, corridor, guess=True), start_xy, goal_xy)
        if path is not None:
            path = path._replace(guessed=True)
    return path


def _plan(grid: Grid, start_xy, goal_xy) -> PlannedPath | None:
    start = grid.nearest_drivable(*grid.cell(*start_xy), config.NAV_GOAL_SNAP_M)
    target = grid.nearest_drivable(*grid.cell(*goal_xy), config.NAV_GOAL_SNAP_M)
    if start is None or target is None:
        return None
    cells = grid.search(start, target=target)
    if cells is None:
        return None
    # If the goal cell itself is drivable (not shifted), go exactly to the
    # requested point -- the cell centre could be up to 7 cm off.
    precise = goal_xy if target == grid.cell(*goal_xy) else None
    return _path_from(grid, start_xy, cells, end=precise)


def find_frontier(occ_map, start_xy, handled=(), blocked=()) -> PlannedPath | None:
    """Path to the nearest reachable frontier; None if the map is complete."""
    grid = Grid(occ_map, blocked)
    start = grid.nearest_drivable(*grid.cell(*start_xy), config.NAV_GOAL_SNAP_M)
    if start is None:
        return None
    close = max(1, int(round(config.FRONTIER_NEAR_M / grid.resolution)))
    goals = np.zeros((grid.height, grid.width), dtype=bool)
    centers: dict[tuple[int, int], tuple[float, float]] = {}
    for cells, center in grid.gaps():
        if any(math.hypot(center[0] - e[0], center[1] - e[1]) < config.FRONTIER_DONE_M
               for e in handled):
            continue
        for x, y in cells:
            goals[y, x] = True
            centers[(x, y)] = center
    if not goals.any():
        return None
    # The goal is not the frontier itself (it lies at the edge of the known
    # area and often too close to a wall), but any drivable spot near it.
    nearby = goals.copy()
    for _ in range(close):
        n = nearby.copy()
        n[1:, :] |= nearby[:-1, :]
        n[:-1, :] |= nearby[1:, :]
        n[:, 1:] |= nearby[:, :-1]
        n[:, :-1] |= nearby[:, 1:]
        nearby = n
    cells = grid.search(start, goals=nearby & grid.drivable)
    if cells is None:
        return None
    path = _path_from(grid, start_xy, cells)
    # Which frontier is being approached? The one with the cell nearest to the goal.
    zx, zy = cells[-1]
    nearest = min(centers, key=lambda p: (p[0] - zx) ** 2 + (p[1] - zy) ** 2)
    return path._replace(gap=centers[nearest])


def frontiers_for_display(occ_map) -> list[list[float]]:
    """Centres of all frontiers, for the /map page."""
    return [[round(m[0], 2), round(m[1], 2)] for _, m in Grid(occ_map).gaps()]


class BackgroundPlanner:
    """Computes paths in a separate thread -- the drive loop only polls."""

    def __init__(self) -> None:
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pfadplanung")

    def path(self, occ_map, start_xy, goal_xy, blocked=(), corridor=None) -> Future:
        return self._pool.submit(_safe_call, plan_path, occ_map, start_xy, goal_xy, tuple(blocked),
                                 corridor)

    def gap(self, occ_map, start_xy, handled=(), blocked=()) -> Future:
        return self._pool.submit(
            _safe_call, find_frontier, occ_map, start_xy, tuple(handled), tuple(blocked)
        )


def _safe_call(fn, *args):
    """An error while planning means "no path", not "drive loop dead"."""
    try:
        return fn(*args)
    except Exception:
        log.exception("Path planning failed")
        return None
