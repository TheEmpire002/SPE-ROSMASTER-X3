"""Drive guard: collision check over the robot's footprint.

The last check before the drive, the same for every motion: forward,
sideways, diagonal, in an arc or on the spot. For the command we precompute
everywhere the robot's footprint will be in the near future. If a lidar
point reaches into it, we brake hard enough that the robot still stops in
front of it, including reaction time and braking distance.

Why not a cone in the driving direction as before:

* A cone starting at the lidar centre is narrow close to the robot -- the
  table leg next to the front wheel lies outside it -- and wide far away, so
  it brakes in front of walls the robot would pass.
* A cone knows nothing about rotation. When turning, the corners swing out;
  next to a wall that alone can make contact.
* A hard limit ("stop below 35 cm") lets it approach at full speed. With
  0.22 s command latency (sensor check) and braking distance it then rolls
  a good bit further. Here it brakes continuously instead, the closer the
  slower.

Driving away is always allowed: a point that already lies inside the margin
only blocks motions that come *closer* to it. Otherwise the robot would be
stuck next to every obstacle and would have to be carried.

"Blocked" is not a stop that bypasses the ramp either: the watchdog applies
the result with its braking ramp. Hard stops from full speed cause the
current spikes that reset the USB hub on the X3 (and with it the lidar).

Since 03.11. this is the only protection layer for autonomous driving (in
the watchdog with ``source="autonom"``). The former ``AutonomousGuard``
only checked a strip in the driving direction; taken over from it are the
age of the oldest scan point and preparing the points once per scan.
"""

from __future__ import annotations

import logging
import math
import time
from typing import NamedTuple

from .. import config
from ..protocol import STOP, DriveCommand

log = logging.getLogger(__name__)

#: Points farther away do not matter for the lookahead.
_RANGE_M = 2.5
#: Compute at most this far into the future.
_MAX_LOOKAHEAD_S = 4.0
#: Granularity of the lookahead: at most this much distance or rotation per step.
_STEP_M = 0.02
_STEP_RAD = math.radians(2.0)

#: Reason when there is no fresh scan. Callers use it to tell "blind"
#: from "obstacle".
NO_LIDAR = "keine frischen Lidar-Daten"


class Check(NamedTuple):
    """Result of a check."""

    command: DriveCommand   # what may be driven
    share: float          # 1.0 = unchanged, 0.0 = blocked
    free_m: float | None   # distance until contact (margin included), None = free
    reason: str | None      # why it braked, None = not at all


class DriveGuard:
    """Limits drive commands so that the footprint touches nothing."""

    #: Log interventions at most this often (s).
    REPORT_INTERVAL_S = 3.0

    def __init__(self, lidar, extra=None) -> None:
        self._lidar = lidar
        # Further obstacle points in the robot frame: depth camera including
        # its memory, and the spots where it got stuck
        # (navigation/obstacle_memory.py). The lidar only sees its own plane.
        self._extra = extra
        self.interventions = 0  # during autonomous driving, see report_intervention()
        self._last_reported = -1e9
        self._scan = None
        self._scan_version = None
        self._scan_points = None

    def check(self, cmd: DriveCommand, margin_m: float | None = None) -> Check:
        """Compute the allowed command.

        ``margin_m`` is the minimum distance the robot's edge should keep
        (default ``GUARD_MARGIN_M``).
        """
        if cmd.is_stop():
            return Check(cmd, 1.0, None, None)
        margin = config.GUARD_MARGIN_M if margin_m is None else margin_m

        points, age = self._points()
        if points is None:
            # No driving blind: an old scan shows a world in which the robot
            # has long since moved elsewhere.
            return Check(STOP, 0.0, None, NO_LIDAR)
        if len(points) == 0:
            return Check(cmd, 1.0, None, None)

        free_s = self._free_time(cmd, points, margin, age)
        if free_s is None:
            return Check(cmd, 1.0, None, None)

        # How hard to brake so that, after reaction time and braking distance,
        # it stops within the free stretch. Distance and rotation separately,
        # the stricter one counts. All components scaled equally: that way the
        # trajectory (e.g. an arc) stays the same, just slower.
        reaction = config.GUARD_REACTION_S + age
        v = math.hypot(cmd.vx, cmd.vy)
        w = abs(cmd.omega)
        share = 1.0
        if v > 1e-6:
            share = min(share, _share(v, v * free_s, reaction, config.GUARD_BRAKE))
        if w > 1e-6:
            share = min(share, _share(w, w * free_s, reaction, config.GUARD_BRAKE_TURN))
        free_m = v * free_s if v > 1e-6 else None

        if share >= 0.999:
            return Check(cmd, 1.0, free_m, None)
        if share < config.GUARD_MIN_SHARE:
            # The motors no longer execute very small values; better to
            # honestly stand still then.
            return Check(STOP, 0.0, free_m, "Hindernis im Weg")
        return Check(
            DriveCommand(cmd.vx * share, cmd.vy * share, cmd.omega * share),
            share, free_m, "bremst vor Hindernis",
        )

    def turn_free(self, angle_rad: float, margin_m: float | None = None) -> str | None:
        """Whether a turn on the spot by ``angle_rad`` can be completed.

        For turns with a fixed target ("90 degrees to the left"): check
        beforehand instead of stopping halfway. The footprint is checked over
        the whole angle -- a circle around the lidar would be far too large
        at the front (the robot only sticks out 15 cm there) and would block
        even near furniture that no corner brushes against.

        Returns the reason why it is not possible, otherwise None.
        """
        import numpy as np

        margin = config.GUARD_MARGIN_M if margin_m is None else margin_m
        points, _ = self._points()
        if points is None:
            return NO_LIDAR
        if len(points) == 0 or abs(angle_rad) < 1e-6:
            return None
        # After a full revolution everything repeats.
        arc = min(abs(angle_rad), 2.0 * math.pi)
        steps = max(2, math.ceil(arc / _STEP_RAD))
        theta = math.copysign(1.0, angle_rad) * np.linspace(0.0, arc, steps + 1)
        cs, sn = np.cos(theta)[:, None], np.sin(theta)[:, None]
        px = cs * points[None, :, 0] + sn * points[None, :, 1]
        py = -sn * points[None, :, 0] + cs * points[None, :, 1]
        distance = _distance_to_area(px, py).min(axis=1)
        # Same rule as when driving: only approaching below the margin counts.
        critical = (distance[1:] < margin) & (distance[1:] < distance[:-1] - 1e-4)
        if not critical.any():
            return None
        after = math.degrees(abs(theta[int(np.argmax(critical)) + 1]))
        return f"eine Ecke stößt nach {after:.0f}° an"

    def report_intervention(self, reason: str, now: float) -> None:
        """Count an intervention during autonomous driving and log it debounced.

        Called by the watchdog. The check runs at a 20 Hz tick; in front of a
        stationary obstacle the message would otherwise come twenty times a
        second.
        """
        self.interventions += 1
        if now - self._last_reported >= self.REPORT_INTERVAL_S:
            self._last_reported = now
            log.warning("Autonomous drive command braked: %s", reason)

    def state(self) -> dict:
        return {"aktiv": bool(config.AUTONOMOUS_GUARD_ACTIVE), "eingriffe": self.interventions}

    def _points(self):
        """Lidar points at close range and age of the scan; (None, None) without a fresh scan."""
        scan = self._lidar.latest() if self._lidar is not None else None
        now = time.monotonic()
        if scan is None or now - scan.timestamp > config.GUARD_SCAN_MAX_S:
            return None, None
        # The oldest point of the scan is a full revolution older than its
        # timestamp -- use that, not the average. That is how long the robot
        # has already been driving on blind.
        age = now - (scan.timestamp - getattr(scan, "duration", 0.0))

        extra = self._extra if config.GUARD_WITH_DEPTH else None
        version = None
        if extra is not None:
            extra.update(now)
            version = extra.version

        # Prepare the points only once per scan: the lidar delivers 7.5
        # scans per second, we are asked 20 times.
        if scan is self._scan and version == self._scan_version:
            return self._scan_points, age

        import numpy as np

        points = scan.points()
        if extra is not None:
            more = extra.points()
            if len(more):
                points = np.concatenate((points, more)) if len(points) else more
        if len(points):
            points = points[np.hypot(points[:, 0], points[:, 1]) < _RANGE_M]
        # Points inside the footprint itself cannot exist -- they are parts of
        # the robot or false readings. If counted, they would set the distance
        # to zero permanently and thereby switch off the guard (closer than
        # zero is impossible).
        if len(points):
            points = points[_distance_to_area(points[:, 0], points[:, 1]) > 0.01]
        self._scan, self._scan_version, self._scan_points = scan, version, points
        return points, age

    @staticmethod
    def _free_time(cmd: DriveCommand, points, margin: float, age: float) -> float | None:
        """How long the command can be driven until the margin is touched.

        None if nothing is in the way within the lookahead.
        """
        import numpy as np

        v = math.hypot(cmd.vx, cmd.vy)
        w = abs(cmd.omega)
        reaction = config.GUARD_REACTION_S + age
        # Look ahead over the stopping distance at the full command, with reserve.
        duration = 0.0
        if v > 1e-6:
            duration = max(duration, reaction + v / config.GUARD_BRAKE)
        if w > 1e-6:
            duration = max(duration, reaction + w / config.GUARD_BRAKE_TURN)
        duration = min(duration * 1.5 + 0.2, _MAX_LOOKAHEAD_S)
        steps = int(min(200, max(4, math.ceil(max(v * duration / _STEP_M,
                                                      w * duration / _STEP_RAD)))))
        t = np.linspace(0.0, duration, steps + 1)

        # Pose of the robot at each point in time with a constant command
        # (arc motion, in the robot frame of now).
        theta = cmd.omega * t
        if abs(cmd.omega) < 1e-6:
            x, y = cmd.vx * t, cmd.vy * t
        else:
            s, c = np.sin(theta), np.cos(theta)
            x = (cmd.vx * s + cmd.vy * (c - 1.0)) / cmd.omega
            y = (cmd.vx * (1.0 - c) + cmd.vy * s) / cmd.omega

        # Transform all points into the frame of each future pose and measure
        # the distance to the footprint (rows: points in time, columns: points).
        dx = points[None, :, 0] - x[:, None]
        dy = points[None, :, 1] - y[:, None]
        cs, sn = np.cos(theta)[:, None], np.sin(theta)[:, None]
        px = cs * dx + sn * dy
        py = -sn * dx + cs * dy
        distance = _distance_to_area(px, py).min(axis=1)

        # Contact means the distance drops below the margin AND it gets closer.
        # If the distance stays the same or grows, it is driving parallel or
        # away -- that is allowed, even very close by.
        critical = (distance[1:] < margin) & (distance[1:] < distance[:-1] - 1e-4)
        if not critical.any():
            return None
        return float(t[int(np.argmax(critical))])


def _distance_to_area(x, y):
    """Distance of points (robot frame) to the rectangular footprint, 0 inside."""
    import numpy as np

    outside_x = np.maximum(np.maximum(x - config.ROBOT_FRONT_M, -config.ROBOT_REAR_M - x), 0.0)
    outside_y = np.maximum(np.abs(y) - config.ROBOT_HALF_WIDTH_M, 0.0)
    return np.hypot(outside_x, outside_y)


def _share(speed: float, free: float, reaction: float, brake: float) -> float:
    """Largest share k of the speed at which the stopping distance fits into the free space.

    Stopping distance at speed k*v: k*v*reaction_time + (k*v)^2 / (2*deceleration).
    """
    if free <= 0.0:
        return 0.0
    a = speed * speed / (2.0 * brake)
    b = speed * reaction
    k = (-b + math.sqrt(b * b + 4.0 * a * free)) / (2.0 * a)
    return max(0.0, min(1.0, k))
