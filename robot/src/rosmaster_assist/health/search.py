"""Where the healthbot looks: viewpoints around a spot, search points on a map.

Pure functions without hardware, so the tests can check them directly. All
positions are in the frame the healthbot drives in (map or odometry), angles
of poses in radians, bearings in degrees (0 = front, 90 = left).
"""

from __future__ import annotations

import math

import numpy as np

from .. import config


def position(pose: tuple[float, float, float], bearing_deg: float, distance_m: float) -> tuple[float, float]:
    """The point ``distance_m`` away in direction ``bearing_deg`` seen from ``pose``."""
    a = pose[2] + math.radians(bearing_deg)
    return pose[0] + distance_m * math.cos(a), pose[1] + distance_m * math.sin(a)


def _room(occ_map, x: float, y: float, clearance: float) -> bool:
    if occ_map is None:
        return True
    if not occ_map.is_free(x, y):
        return False
    return float(occ_map.distance_at(x, y)) >= clearance


def viewpoints(target, robot_xy, radius: float, count: int, step_deg: float,
               occ_map=None, clearance: float = 0.3) -> list[tuple[float, float]]:
    """Points on a circle around ``target`` to look at it from other sides.

    Starts next to where the robot already is and goes on around the circle
    in one direction, so it drives around the table instead of across it.
    With a map only points with room to the walls count.
    """
    tx, ty = float(target[0]), float(target[1])
    start = math.atan2(robot_xy[1] - ty, robot_xy[0] - tx)
    result = []
    k = 1
    # at most one round, also when points are dropped for lack of room
    while len(result) < count and k * step_deg < 360.0:
        a = start + math.radians(k * step_deg)
        p = (tx + radius * math.cos(a), ty + radius * math.sin(a))
        if _room(occ_map, p[0], p[1], clearance):
            result.append(p)
        k += 1
    return result


def search_points(occ_map, start, spacing: float, clearance: float,
                  max_count: int, avoid=None, avoid_m: float = 0.0) -> list[tuple[float, float]]:
    """Free points on a grid over the map, nearest first, then nearest neighbour.

    The order keeps the drives between the points short. Unreachable points
    are not filtered here: homing gives up on them and the search goes on.
    Points closer than ``avoid_m`` to ``avoid`` (the person on the floor) are
    left out.
    """
    if occ_map is None:
        return []
    x0, y0 = float(occ_map.origin[0]), float(occ_map.origin[1])
    w = occ_map.width * occ_map.resolution
    h = occ_map.height * occ_map.resolution
    candidates = []
    for x in np.arange(x0 + spacing / 2, x0 + w, spacing):
        for y in np.arange(y0 + spacing / 2, y0 + h, spacing):
            if avoid is not None and math.dist((float(x), float(y)), avoid) < avoid_m:
                continue
            if _room(occ_map, float(x), float(y), clearance):
                candidates.append((float(x), float(y)))
    order = []
    here = (float(start[0]), float(start[1]))
    while candidates and len(order) < max_count:
        i = min(range(len(candidates)), key=lambda j: math.dist(candidates[j], here))
        here = candidates.pop(i)
        order.append(here)
    return order


def sight_near(pose: tuple[float, float, float], bearing_deg: float, point, radius: float) -> bool:
    """Is ``bearing_deg`` (seen from ``pose``) in the direction of ``point``?

    The direction counts within the angle a circle of ``radius`` around the
    point covers, at least 60° to each side when the robot is right next to it.
    """
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return True
    off = math.atan2(dy, dx) - (pose[2] + math.radians(bearing_deg))
    off = abs(math.atan2(math.sin(off), math.cos(off)))
    return off <= max(math.atan2(radius, d), math.radians(60.0) if d <= radius else 0.0)


def someone_behind(points, max_m: float, half_deg: float, min_points: int = 3) -> bool:
    """Something within ``max_m`` behind the robot (lidar, robot frame x forward)."""
    if points is None or len(points) == 0:
        return False
    p = np.asarray(points, dtype=float)
    angle = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    diff = np.abs((angle - 180.0 + 180.0) % 360.0 - 180.0)
    r = np.hypot(p[:, 0], p[:, 1])
    near = (diff <= half_deg) & (r > config.ROBOT_REAR_M + 0.05) & (r <= max_m)
    return int(np.count_nonzero(near)) >= min_points
