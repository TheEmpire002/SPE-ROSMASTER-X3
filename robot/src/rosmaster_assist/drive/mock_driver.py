"""Drive without hardware -- only writes drive commands to the log.

This way the whole project can be developed and tested on a laptop, even
though there is only one robot.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


class MockDriver(DriveBase):
    name = "mock"

    #: Share of the commanded speed that the "encoders" report.
    #: Real mecanum wheels slip; with 1.0 the simulation would be unrealistically
    #: optimistic and the lidar correction would have nothing to correct.
    SLIP = 0.88

    def __init__(self) -> None:
        self._connected = False
        self._last = STOP

        # Where the robot really is in the simulation. The pose tracking
        # knows nothing about this -- it only gets the slipped measurements.
        # The simulated lidar, on the other hand, scans from here, exactly
        # as the hardware would.
        self._true_x = 0.0
        self._true_y = 0.0
        self._true_theta = 0.0
        self._true_time: float | None = None
        # Drive commands and the simulated lidar access this from different
        # threads. Without a lock they overwrite each other's timestamp, and
        # the "true" position itself becomes wrong.
        self._true_lock = threading.Lock()

    def reset_true_pose(self) -> None:
        """Resets the simulated reality to the start.

        Simulation only: when the start is set, the pose tracking zeroes its
        estimate. If the simulation did not do the same, the imaginary robot
        would stand somewhere far outside the imaginary room -- and the
        simulated lidar would deliver nonsense.
        """
        with self._true_lock:
            self._true_x = self._true_y = self._true_theta = 0.0
            self._true_time = None

    def true_pose(self) -> tuple[float, float, float]:
        """(x, y, angle in radians) -- only known in the simulation."""
        with self._true_lock:
            self._advance()
            return self._true_x, self._true_y, self._true_theta

    def _advance_true_pose(self) -> None:
        with self._true_lock:
            self._advance()

    def _advance(self) -> None:
        """Only call with the lock held."""
        now = time.monotonic()
        if self._true_time is None:
            self._true_time = now
            return
        dt = now - self._true_time
        self._true_time = now
        if dt <= 0 or dt > 1.0:
            return

        c = self._last
        avg = self._true_theta + c.omega * dt / 2.0
        self._true_x += (c.vx * math.cos(avg) - c.vy * math.sin(avg)) * dt
        self._true_y += (c.vx * math.sin(avg) + c.vy * math.cos(avg)) * dt
        self._true_theta += c.omega * dt

    def connect(self) -> None:
        self._connected = True
        log.info("Mock drive ready (no real hardware)")

    def disconnect(self) -> None:
        self.stop()
        self._connected = False
        log.info("Mock drive stopped")

    def drive(self, cmd: DriveCommand) -> None:
        # Advance before taking over, so the distance covered is credited to
        # the old command and not to the new one.
        self._advance_true_pose()
        # Only log on change -- otherwise the console is unreadable at 10
        # commands per second.
        # "SIM" in plain text, so nobody mistakes these lines for a real drive
        # and looks for the fault elsewhere when nothing moves.
        if cmd != self._last:
            log.info("SIM drive  vx=%+.2f  vy=%+.2f  omega=%+.2f",
                     cmd.vx, cmd.vy, cmd.omega)
            self._last = cmd

    def stop(self) -> None:
        if self._last != STOP:
            log.info("SIM stop")
            self._last = STOP

    def beep(self, ms: int = 100) -> None:
        log.info("SIM beep (%d ms)", ms)

    def read_motion(self) -> tuple[float, float, float] | None:
        """What the "encoders" report -- with slip, so slightly off.

        Deliberately not the true value: only this way does the simulation
        get the same drift as reality, and only this way can we check whether
        the lidar correction actually compensates for it.
        """
        return (
            self._last.vx * self.SLIP,
            self._last.vy * self.SLIP,
            self._last.omega * self.SLIP,
        )

    def read_battery(self) -> float | None:
        """Imaginary 3S battery: nearly full, sags a little while driving.

        This way the diagnostics page shows a lively curve even in the
        simulation, and you can see what voltage drops look like there.
        """
        if not self._connected:
            return None
        last = abs(self._last.vx) + abs(self._last.vy) + 0.3 * abs(self._last.omega)
        wobble = 0.02 * math.sin(time.monotonic() * 0.7)
        return round(12.2 - 0.6 * min(last, 1.0) + wobble, 2)

    @property
    def connected(self) -> bool:
        return self._connected
