"""Progress watch: are the wheels turning while the robot stays put?

If the robot is caught on a chair leg or an edge that neither the lidar nor
the camera sees, everything looks fine to the drive logic: front is clear,
the command goes out. It just does not move. While mapping it then stood
there until someone came -- and the map stopped growing.

The watch compares two things over a window of ~2 s:

* **commanded:** what the watchdog really sent to the motors (after ramp
  and drive guard). If the drive guard slows down, the expectation is
  small too -- standing in front of a wall is not being stuck.
* **measured:** how far the lidar scan from back then can be shifted onto
  the current one (ICP, ``scan_matching.py``). It looks at the walls;
  spinning wheels do not fool it.

If, according to the lidar, less than ``PROGRESS_SHARE`` of the commanded
motion arrived, the robot is stuck. A verdict is only given when the match
is unambiguous: it is computed twice, once starting from "not moved", once
from "as commanded". If the two end up in different places, the
surroundings are ambiguous (long corridor, bare wall) -- better no verdict
than a wrong one.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import NamedTuple

from .. import config
from .scan_matching import match

log = logging.getLogger(__name__)

#: Points per scan for matching (all against all, time grows quadratically).
_MAX_POINTS = 200
#: How far apart the two matches may be for a verdict to be given.
_AGREE_M = 0.05
_AGREE_RAD = math.radians(5.0)


class Stuck(NamedTuple):
    cause: str
    direction_deg: float | None  # where it wanted to go (robot frame), None when turning


def _compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def _invert(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), -at)


def _wrap_angle(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def _thin_out(points, at_most: int):
    if len(points) <= at_most:
        return points
    import numpy as np

    return points[(np.arange(at_most) * (len(points) / at_most)).astype(int)]


class ProgressWatch:
    """Call ``update`` every tick, then ``check``."""

    def __init__(self, watchdog) -> None:
        self._watchdog = watchdog
        self.verdicts = 0
        self.reset()

    def reset(self) -> None:
        """Start observing afresh -- after a verdict, unsticking, lifting."""
        self._commanded = (0.0, 0.0, 0.0)
        self._last_time: float | None = None
        self._history: deque = deque()  # (time, scan time, points, commanded pose)
        self._last_entry = -1e9

    def update(self, now: float) -> None:
        """Accumulate the motion just output."""
        dt = 0.0 if self._last_time is None else min(max(now - self._last_time, 0.0), 0.3)
        self._last_time = now
        c = self._watchdog.current_command
        self._commanded = _compose(self._commanded, (c.vx * dt, c.vy * dt, c.omega * dt))

    def check(self, now: float, scan) -> Stuck | None:
        """A verdict if the lidar says it is not making progress; otherwise None."""
        if not config.PROGRESS_ACTIVE or scan is None:
            return None
        if now - self._last_entry < config.PROGRESS_TICK_S:
            return None
        if self._history and scan.timestamp == self._history[-1][1]:
            return None
        self._last_entry = now
        points = _thin_out(scan.points(), _MAX_POINTS)
        self._history.append((now, scan.timestamp, points, self._commanded))
        # The front entry is the newest scan that is at least one window old.
        window = config.PROGRESS_WINDOW_S
        while len(self._history) > 2 and self._history[1][0] <= now - window:
            self._history.popleft()
        t0, _, reference, commanded0 = self._history[0]
        if now - t0 < 0.9 * window:
            return None

        expected = _compose(_invert(commanded0), self._commanded)
        path = math.hypot(expected[0], expected[1])
        turn = abs(expected[2])
        min_turn = math.radians(config.PROGRESS_MIN_TURN_DEG)
        if path < config.PROGRESS_MIN_DIST_M and turn < min_turn:
            return None

        match_zero = match(points, reference, (0.0, 0.0, 0.0))
        match_command = match(points, reference, expected)
        if match_zero is None or match_command is None or match_zero.residuum > config.MATCH_MAX_RESIDUAL_M:
            return None
        if (math.hypot(match_zero.x - match_command.x, match_zero.y - match_command.y) > _AGREE_M
                or abs(_wrap_angle(match_zero.theta - match_command.theta)) > _AGREE_RAD):
            return None  # ambiguous, see above

        measured_distance = math.hypot(match_zero.x, match_zero.y)
        measured_turn = abs(match_zero.theta)
        share = config.PROGRESS_SHARE
        if path >= config.PROGRESS_MIN_DIST_M and measured_distance < share * path:
            verdict = Stuck(
                f"kommt laut Lidar nicht voran ({measured_distance * 100:.0f} von {path * 100:.0f} cm)",
                math.degrees(math.atan2(expected[1], expected[0])),
            )
        elif turn >= min_turn and measured_turn < share * turn:
            verdict = Stuck(
                f"dreht sich laut Lidar nicht ({math.degrees(measured_turn):.0f}° von "
                f"{math.degrees(turn):.0f}°)",
                None,
            )
        else:
            return None
        self.verdicts += 1
        log.warning("Progress watch: %s", verdict.cause)
        self.reset()
        return verdict
