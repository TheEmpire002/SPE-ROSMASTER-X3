"""Keeping track of where the robot is.

Method: dead reckoning. There is no sensor that reveals the position --
it is continuously summed up from velocity and heading, starting from the
starting point.

**This drifts, unavoidably.** Every measurement error and every spinning
wheel enters the sum permanently and is never corrected again. Mecanum
wheels slip a lot by design, especially when turning and on smooth floors.
Within a room, a few decimeters are normal.

That is enough for the home button. Anyone who needs centimeters cannot get
around an external reference -- lidar and SLAM.

Coordinates: x points in the initial heading, y to the left, angles
counterclockwise. The starting point is (0, 0) with angle 0.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque

from .. import config
from ..drive.base import DriveBase
from ..protocol import DriveCommand

log = logging.getLogger(__name__)


class Odometry:
    """Updates the position from the motion data."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        self._last_time: float | None = None
        # Raw IMU value at zeroing -- the angle is tracked relative to it,
        # because the IMU has its own zero point.
        self._imu_reference: float | None = None
        self._imu_offset = 0.0
        self._measured = False
        # Short look-back for the lidar alignment: (time, x, y, angle).
        # Two seconds are enough -- one lidar revolution takes 0.2 s.
        self._history: deque = deque(maxlen=120)

        # If the library's receive thread dies (e.g. due to a port
        # conflict), it keeps delivering values -- namely zeros. That would
        # be worse than no measurement at all: the position would freeze,
        # and the homing drive would lead nowhere without any warning.
        self._stall_despite_command = 0.0
        self._measurement_rejected = False

        # Plausibility check of the IMU: if its angle stands still although
        # the board reports a clear turn rate, it is unusable (on the X3 it
        # constantly delivers 0). Then only sum up the turn rate until the
        # next restart.
        self._imu_frozen_s = 0.0
        self._imu_last: float | None = None
        self._imu_rejected = False
        # Last used turn rate (rad/s), for the person tracker and stuck
        # detection: what counts there is the self-rotation, not the angle.
        self._omega = 0.0
        self._v = (0.0, 0.0)

    # --- Queries -------------------------------------------------------------

    def pose(self) -> tuple[float, float, float]:
        """(x, y, angle in degrees)."""
        with self._lock:
            return self.x, self.y, math.degrees(self.theta)

    @property
    def actually_measuring(self) -> bool:
        """Whether real measurements come in or only the commands are integrated."""
        return self._measured

    @property
    def imu_rejected(self) -> bool:
        """Whether the IMU was detected as unusable (for /diagnose)."""
        return self._imu_rejected

    def velocity(self) -> tuple[float, float, float]:
        """Last used ego-motion (vx, vy, omega) in the robot frame."""
        with self._lock:
            return self._v[0], self._v[1], self._omega

    def distance_to_start(self) -> float:
        with self._lock:
            return math.hypot(self.x, self.y)

    def pose_at(self, instant: float) -> tuple[float, float, float] | None:
        """Where the robot was at an earlier point in time (x, y, radians).

        Needed for the lidar alignment: a scan is already a few tenths of a
        second old when it is evaluated. Without this look-back the
        correction would pull the robot to a spot it left long ago -- at
        0.3 m/s that is already several centimeters.
        """
        with self._lock:
            if not self._history:
                return None
            # Search backwards: the requested time is almost always at the
            # end of the history.
            best = self._history[0]
            for entry in reversed(self._history):
                if entry[0] <= instant:
                    best = entry
                    break
            return best[1], best[2], best[3]

    def correct(self, x: float, y: float, theta: float, weight: float) -> None:
        """Pull the position towards a measured value.

        Comes from the lidar alignment. Deliberately not a hard overwrite: a
        single mismatch would otherwise throw the position off and send the
        homing drive astray. Over several passes the estimate converges to
        the measured value.
        """
        weight = max(0.0, min(1.0, weight))
        with self._lock:
            dx = (x - self.x) * weight
            dy = (y - self.y) * weight
            # Do not simply average angles: between 179° and -179° there is
            # one degree, not 358. Go via the difference.
            dtheta = _normalize(theta - self.theta) * weight

            self.x += dx
            self.y += dy
            self.theta = _normalize(self.theta + dtheta)

            # Shift the history along. Otherwise it would hold positions from
            # before the correction, and the latency compensation in the lidar
            # alignment would mistake the correction itself for distance
            # travelled -- the position then builds up further with every
            # pass instead of settling.
            self._history = deque(
                ((t, vx + dx, vy + dy, _normalize(vt + dtheta))
                 for t, vx, vy, vt in self._history),
                maxlen=self._history.maxlen,
            )

            # After a correction the IMU reference is no longer consistent --
            # it is reset on the next step.
            if self._imu_reference is not None:
                self._imu_reference = None

    # --- Zeroing -------------------------------------------------------------

    def reset(self) -> None:
        """This is now the starting point.

        Also resets the IMU reference so the angle starts at zero again --
        otherwise, after zeroing, the robot would point in a direction that
        never existed.
        """
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self._imu_reference = None
            self._imu_offset = 0.0
            self._last_time = None
            # Give the measurement a new chance -- maybe the port conflict
            # has been resolved in the meantime.
            self._stall_despite_command = 0.0
            self._measurement_rejected = False
            self._history.clear()
        log.info("Starting point reset")

    # --- Updating ------------------------------------------------------------

    def update(self, commanded: DriveCommand) -> None:
        """Compute one time step. Call regularly."""
        now = time.monotonic()
        if self._last_time is None:
            self._last_time = now
            return

        dt = now - self._last_time
        self._last_time = now
        # After a pause (server hung, computer asleep) the step would be
        # huge and throw the position off.
        if dt <= 0 or dt > 1.0:
            return

        measured = self._check(self._driver.read_motion(), commanded, dt)
        self._measured = measured is not None
        vx, vy, omega = measured if measured is not None else (
            commanded.vx,
            commanded.vy,
            commanded.omega,
        )

        with self._lock:
            self._omega = omega
            self._v = (vx, vy)
            new_angle = self._determine_angle(omega, dt)

            # Evaluate at the middle of the time step: when driving and
            # turning at the same time, this is much more accurate than taking
            # the start or end angle.
            avg = (self.theta + new_angle) / 2.0
            self.x += (vx * math.cos(avg) - vy * math.sin(avg)) * dt
            self.y += (vx * math.sin(avg) + vy * math.cos(avg)) * dt
            self.theta = new_angle
            self._history.append((now, self.x, self.y, self.theta))

    #: How long reported standstill despite a drive command may last before
    #: the measurement counts as unusable. Generous, because motors need to
    #: spin up and the robot can be blocked briefly.
    STANDSTILL_LIMIT_S = 3.0

    def _check(
        self, measured: tuple[float, float, float] | None, commanded: DriveCommand, dt: float
    ) -> tuple[float, float, float] | None:
        """Discards measurements that obviously cannot be right.

        If the library persistently reports standstill although the robot is
        supposed to drive, its receive thread has probably died. Then it is
        better to integrate the commands: inaccurate, but not wrong.
        """
        if measured is None:
            return None

        if self._measurement_rejected:
            return None

        is_moving = any(abs(v) > 1e-6 for v in measured)
        should_drive = not commanded.is_stop()

        if should_drive and not is_moving:
            self._stall_despite_command += dt
            if self._stall_despite_command > self.STANDSTILL_LIMIT_S:
                self._measurement_rejected = True
                log.warning(
                    "Motion measurement has reported standstill for %.0f s despite "
                    "a drive command -- the receive thread is probably dead. "
                    "From now on the drive commands are integrated instead "
                    "(less accurate). Cause: see docs/hardware.md, port conflict.",
                    self._stall_despite_command,
                )
                return None
        else:
            self._stall_despite_command = 0.0

        return measured

    def _determine_angle(self, omega: float, dt: float) -> float:
        """Heading from the turn rate, optionally from the IMU.

        The default is summing up the measured turn rate: the IMU attitude of
        the X3 constantly delivers 0 (sensor check 28.10.), and an angle that
        stands still while turning is worse than one that drifts slowly. The
        lidar alignment catches the drift again.
        """
        summed = _normalize(self.theta + omega * dt)
        if not config.ODOM_IMU_HEADING or self._imu_rejected:
            return summed

        imu = self._driver.read_heading()
        if imu is None:
            return summed

        if not self._imu_plausible(imu, omega, dt):
            return summed

        if self._imu_reference is None:
            # New reference point. The current angle is remembered as an
            # offset, otherwise a correction from the lidar would be lost
            # again on the next step -- the IMU would impose its own zero
            # point.
            self._imu_reference = imu
            self._imu_offset = self.theta
            return self.theta

        # Take the difference instead of comparing raw values: otherwise
        # the angle jumps by half a revolution at the transition from +180°
        # to -180°.
        since_ref = _normalize(math.radians(imu - self._imu_reference))
        return _normalize(self._imu_offset + since_ref)

    def _imu_plausible(self, imu: float, omega: float, dt: float) -> bool:
        """Rejects the IMU if its angle stands still while turning."""
        last, self._imu_last = self._imu_last, imu
        if last is None or abs(omega) < config.ODOM_IMU_CHECK_OMEGA:
            self._imu_frozen_s = 0.0
            return True
        if abs(imu - last) > 1e-3:
            self._imu_frozen_s = 0.0
            return True
        self._imu_frozen_s += dt
        if self._imu_frozen_s > config.ODOM_IMU_CHECK_S:
            self._imu_rejected = True
            self._imu_reference = None
            log.warning(
                "IMU angle has been frozen for %.1f s although the robot is "
                "turning (%.2f rad/s) -- ignoring the IMU, the angle comes from "
                "the turn rate from now on.", self._imu_frozen_s, omega,
            )
            return False
        return True


def _normalize(angle: float) -> float:
    """Wrap to -pi..+pi."""
    return math.atan2(math.sin(angle), math.cos(angle))
