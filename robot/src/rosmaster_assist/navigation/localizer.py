"""Lidar-based localization: corrects dead reckoning.

Division of labor between the two methods:

* **Dead reckoning** runs always and everywhere. It is available instantly
  and never "lost", but drifts with every meter.
* **Lidar matching** compares the current surroundings with a scan taken at
  the starting point. It accumulates nothing, but needs overlap: behind the
  next corner of the room the robot no longer sees anything familiar.

Hence: dead reckoning carries the robot roughly, the lidar pulls it
straight as soon as it recognizes something. In practice the robot uses
odometry to get *near* the starting point and is guided precisely by the
lidar there -- exactly what a home button needs.

This is deliberately **not** SLAM: no map is built, only a single reference
scan is kept. For "back to the starting point" that is enough, at a
fraction of the effort.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .. import config
from ..sensors.lidar import LidarService
from .odometry import Odometry
from .scan_matching import match

log = logging.getLogger(__name__)


class Localizer:
    """Holds the reference scan and uses it to correct the position."""

    def __init__(self, odometry: Odometry, lidar: LidarService | None) -> None:
        self._odometry = odometry
        self._lidar = lidar
        self._lock = threading.Lock()

        self._reference = None          # point cloud from the starting point
        self._reference_time = 0.0
        self._last_residual: float | None = None
        self._corrections = 0
        self._discarded = 0
        self._last_correction_m: float | None = None

    @property
    def has_reference(self) -> bool:
        return self._reference is not None

    @property
    def available(self) -> bool:
        return self._lidar is not None

    def status(self) -> dict:
        with self._lock:
            return {
                "lidar_vorhanden": self._lidar is not None,
                "referenz_aufgenommen": self._reference is not None,
                "referenzpunkte": 0 if self._reference is None else len(self._reference),
                "korrekturen": self._corrections,
                "verworfen": self._discarded,
                "letztes_residuum_m": (
                    round(self._last_residual, 3)
                    if self._last_residual is not None
                    else None
                ),
                "letzte_korrektur_m": (
                    round(self._last_correction_m, 3)
                    if self._last_correction_m is not None
                    else None
                ),
            }

    # --- Reference ---------------------------------------------------------------

    def capture_reference(self) -> bool:
        """Remembers what the room looks like from the current location.

        Call when setting the starting point. Without a reference there is no
        correction -- then it stays with pure dead reckoning.
        """
        if self._lidar is None:
            return False

        scan = self._lidar.latest()
        if scan is None or len(scan) < 60:
            log.warning(
                "No usable lidar scan for the starting point "
                "(%s points) -- localization stays with dead reckoning",
                0 if scan is None else len(scan),
            )
            return False

        with self._lock:
            self._reference = scan.points()
            self._reference_time = time.monotonic()
            self._corrections = 0
            self._discarded = 0
            self._last_residual = None
            self._last_correction_m = None

        log.info("Starting point captured: %d lidar points as reference", len(scan))
        return True

    def discard_reference(self) -> None:
        with self._lock:
            self._reference = None

    # --- Continuous matching -------------------------------------------------------

    def match_scan(self) -> bool:
        """Try one match. True if a correction was applied."""
        if self._lidar is None or self._reference is None:
            return False

        scan = self._lidar.latest()
        if scan is None:
            return False

        # Do not use scans that are too old: the robot is elsewhere by now,
        # and a stale measurement would pull the position back.
        if time.monotonic() - scan.timestamp > 1.5:
            return False

        x, y, angle_deg = self._odometry.pose()

        # Far from the starting point there is too little overlap. A
        # mismatch there would be worse than no correction at all: it
        # would throw off the position without anyone noticing.
        if math.hypot(x, y) > config.MATCH_RADIUS_M:
            return False

        # The scan shows where the robot stood *when measuring*. So use
        # the position at that time as the initial guess, not the current
        # one -- otherwise the search starts off skewed.
        past_pose = self._odometry.pose_at(scan.timestamp)
        if past_pose is None:
            past_pose = (x, y, math.radians(angle_deg))

        result = match(scan.points(), self._reference, past_pose)

        with self._lock:
            if result is None:
                self._discarded += 1
                return False

            self._last_residual = result.residuum
            if result.residuum > config.MATCH_MAX_RESIDUAL_M:
                self._discarded += 1
                return False

        # The robot has kept driving since the scan. Add that distance,
        # otherwise the correction pulls it back into the past -- at
        # 0.3 m/s and 0.3 s delay that is almost ten centimeters.
        now_x, now_y, now_deg = self._odometry.pose()
        now_theta = math.radians(now_deg)
        goal_x = result.x + (now_x - past_pose[0])
        goal_y = result.y + (now_y - past_pose[1])
        goal_theta = result.theta + _angle_diff(now_theta, past_pose[2])

        with self._lock:
            self._corrections += 1
            self._last_correction_m = math.hypot(goal_x - now_x, goal_y - now_y)

        self._odometry.correct(goal_x, goal_y, goal_theta, config.MATCH_TRUST)
        return True

    # --- Obstacles ---------------------------------------------------------------

    def scan(self):
        """The latest lidar revolution if it is fresh (otherwise None).

        For detouring on the way home: same freshness limit as
        ``obstacle_ahead``.
        """
        if self._lidar is None:
            return None
        scan = self._lidar.latest()
        if scan is None or time.monotonic() - scan.timestamp > 2.0:
            return None
        return scan

    def obstacle_ahead(
        self, direction_deg: float, aperture_deg: float | None = None
    ) -> float | None:
        """Distance to the nearest obstacle in one direction, if measurable.

        ``direction_deg`` in the robot frame: 0 is the front. On the way
        home the direction of travel is passed in -- with mecanum wheels
        that is not necessarily "forward".
        """
        if self._lidar is None:
            return None
        scan = self._lidar.latest()
        if scan is None or time.monotonic() - scan.timestamp > 2.0:
            return None

        if aperture_deg is None:
            aperture_deg = config.HOME_CONE_DEG
        distance = scan.min_distance(direction_deg, aperture_deg)
        return None if distance == float("inf") else distance


def _angle_diff(a: float, b: float) -> float:
    """a - b, wrapped to -pi..+pi."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
