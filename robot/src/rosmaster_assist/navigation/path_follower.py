"""Follow a planned path -- one drive command per tick.

Shared by the drive to the goal (``homing.py``) and the drive to the next
frontier while mapping (``mapping_drive.py``). Decides only the direction;
the speed limit and braking distance are checked, as always, by the
DriveGuard in the watchdog (``source="autonom"``).

- It steers towards the next waypoint. Once that is reached
  (``NAV_WAYPOINT_M``), the next one applies.
- If something the map does not know (person, chair) is in the way, it
  slides past it diagonally like the homing drive (``avoidance.choose_direction``).
  If that fails too, it reports BLOCKED -- replanning is up to the caller,
  who knows the map.
- Meanwhile it slowly turns into the driving direction. Mecanum could also
  drive sideways, but this way the camera looks where it is going, and long
  sideways drives slip more.
"""

from __future__ import annotations

import math

from .. import config
from ..protocol import DriveCommand
from . import avoidance

DRIVING = "faehrt"
ARRIVED = "angekommen"
BLOCKED = "blockiert"
OFF_PATH = "abgekommen"


def _normalize(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def _distance_to_segment(p, a, b) -> float:
    ax, ay = b[0] - a[0], b[1] - a[1]
    length2 = ax * ax + ay * ay
    if length2 < 1e-9:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / length2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


class PathFollower:
    """Follows a list of waypoints in the map frame."""

    def __init__(self, speed: float | None = None, tolerance: float | None = None) -> None:
        self.speed = config.NAV_DRIVE_SPEED if speed is None else speed
        self.tolerance = config.NAV_GOAL_TOLERANCE_M if tolerance is None else tolerance
        self.points: list[tuple[float, float]] = []
        self.goal_angle: float | None = None
        self._i = 0
        self._from: tuple[float, float] | None = None
        self._page = 0.0
        self.rest_m = 0.0

    def apply(self, points, from_=None, goal_angle: float | None = None) -> None:
        """New path. ``from_``: where it starts (for the deviation from the first segment)."""
        self.points = [tuple(p) for p in points]
        self.goal_angle = goal_angle
        self._i = 0
        self._from = tuple(from_[:2]) if from_ is not None else None
        self._page = 0.0
        self.rest_m = 0.0

    def _remainder(self, pose) -> float:
        p = self.points
        rest = math.hypot(p[self._i][0] - pose[0], p[self._i][1] - pose[1])
        for a, b in zip(p[self._i:], p[self._i + 1:]):
            rest += math.hypot(b[0] - a[0], b[1] - a[1])
        return rest

    def step(self, pose, points) -> tuple[DriveCommand, str]:
        """One tick. ``points``: obstacles in the robot frame (lidar plus memory)."""
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)
        if not self.points:
            return stop_cmd, ARRIVED
        x, y, th = pose
        p = self.points
        while self._i < len(p) - 1 and math.hypot(p[self._i][0] - x, p[self._i][1] - y) < config.NAV_WAYPOINT_M:
            self._from = p[self._i]
            self._i += 1
        self.rest_m = self._remainder(pose)

        if self._from is not None and _distance_to_segment(
            (x, y), self._from, p[self._i]
        ) > config.NAV_DEVIATION_M:
            return stop_cmd, OFF_PATH

        target = p[-1]
        at_goal = self._i == len(p) - 1 and math.hypot(target[0] - x, target[1] - y) <= self.tolerance
        if at_goal:
            if self.goal_angle is None:
                return stop_cmd, ARRIVED
            error = _normalize(self.goal_angle - th)
            if abs(error) <= math.radians(config.NAV_GOAL_DEG):
                return stop_cmd, ARRIVED
            return DriveCommand(0.0, 0.0, math.copysign(config.MAPPING_DRIVE_TURN_SPEED, error)), DRIVING

        tx, ty = p[self._i]
        direction = _normalize(math.atan2(ty - y, tx - x) - th)
        speed = self.speed
        if self.rest_m < config.HOME_SLOWDOWN_M:
            # Not arbitrarily slow: the motors no longer turn very small
            # values into motion on the floor.
            speed *= max(0.35, self.rest_m / config.HOME_SLOWDOWN_M)

        to_point = math.hypot(tx - x, ty - y)
        far = min(to_point + config.AVOID_FRONT_M + config.AVOID_MARGIN_M,
                   config.DETOUR_LOOKAHEAD_M + config.HOME_STOP_DISTANCE_M)
        choice = None
        for needed in (far, min(far, config.HOME_STOP_DISTANCE_M)):
            choice = avoidance.choose_direction(
                points, math.degrees(direction), needed, config.EXPLORE_HALF_WIDTH_M, self._page
            )
            if choice is not None:
                break
        if choice is None:
            return stop_cmd, BLOCKED
        new = math.radians(choice[0])
        shift = _normalize(new - direction)
        self._page = 0.0 if abs(shift) < 1e-6 else math.copysign(1.0, shift)

        omega = 0.0
        if self.rest_m > 0.5:
            omega = max(-config.NAV_TURN_MAX, min(config.NAV_TURN_MAX, 1.0 * direction))
            if abs(omega) < 0.1:
                omega = 0.0  # no jitter around driving straight
        return DriveCommand(speed * math.cos(new), speed * math.sin(new), omega), DRIVING
