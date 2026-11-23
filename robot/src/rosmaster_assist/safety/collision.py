"""Collision avoidance for manual control.

The autonomous modes are checked by the drive guard directly in the watchdog
(``source="autonom"``, always on). Until 03.11. this was done here by
``AutonomousGuard`` with a strip in the driving direction; it knew no rotation
and no arc and has been merged into the drive guard.

Before every drive command it checks via the drive guard (``drive_guard.py``)
whether the robot's footprint would touch something, and brakes in time --
smoothly, not only at a hard limit. This used to be a cone from the lidar
centre with a hard stop; it missed table legs next to the wheels, let the
robot roll up at full speed and knew no rotation.

Design decisions that shape the behavior:

**Block only what leads closer.** Driving away and turning without any
corner bumping into something stay allowed; as soon as the human steers in
another direction, the robot drives again. A blanket stop would pin it in
front of every obstacle -- you would have to carry it to free it.

**Server side, not in the app.** The lidar data is here, and a block that
can be bypassed through the browser console would be no block at all.

**Off** by default: whoever steers by hand usually sees the robot and does
not want to be patronised. It can be switched on in the app.
"""

from __future__ import annotations

import logging
import time

from .. import config
from ..protocol import STOP, DriveCommand
from .drive_guard import NO_LIDAR

log = logging.getLogger(__name__)


class CollisionGuard:
    """Filters drive commands based on the lidar data."""

    #: Report no more often than every few seconds -- otherwise, with a key
    #: held down, the message floods the log in the app.
    REPORT_INTERVAL_S = 2.0

    def __init__(self, localizer=None, drive_guard=None) -> None:
        self._localizer = localizer
        self._drive_guard = drive_guard
        self.enabled = config.COLLISION_ENABLED
        self.distance = config.COLLISION_DISTANCE_M
        self._blocked_since: float | None = None
        self._last_reported = 0.0

    @property
    def available(self) -> bool:
        """Whether there is any sensor at all that could see obstacles."""
        return self._drive_guard is not None and (
            self._localizer is None or self._localizer.available
        )

    @property
    def blocked(self) -> bool:
        return self._blocked_since is not None

    def configure(self, enabled: bool | None, distance: float | None) -> None:
        if enabled is not None:
            self.enabled = bool(enabled)
            if not self.enabled:
                self._blocked_since = None
        if distance is not None:
            # Clamp so nobody accidentally sets a value at which the robot
            # no longer moves at all.
            self.distance = max(
                config.COLLISION_MIN_M, min(config.COLLISION_MAX_M, float(distance))
            )
        log.info(
            "Collision avoidance: %s, minimum distance %.2f m",
            "on" if self.enabled else "off",
            self.distance,
        )

    def state(self) -> dict:
        return {
            "enabled": self.enabled,
            "distance": round(self.distance, 2),
            "verfuegbar": self.available,
            "blockiert": self.blocked,
        }

    def check(self, cmd: DriveCommand) -> tuple[DriveCommand, float | None]:
        """Returns the allowed command, together with the obstacle distance.

        The second value is None if nothing was in the way.
        """
        if not self.enabled or self._drive_guard is None:
            self._blocked_since = None
            return cmd, None

        # The slider in the app means the distance forward from the lidar centre.
        # As clearance at the robot's edge it is the same margin all around.
        margin = max(0.02, self.distance - config.ROBOT_FRONT_M)
        verdict = self._drive_guard.check(cmd, margin)

        if verdict.reason == NO_LIDAR:
            # Without a lidar the guard cannot help. Whoever steers by hand
            # sees the robot -- so do not cripple manual control as well.
            # Autonomous drives, on the other hand, stop.
            self._blocked_since = None
            return cmd, None
        if verdict.share > 0.0:
            # Free or only braked: no reason for a message.
            self._blocked_since = None
            return verdict.command, None

        # The drive is blocked. The rotation alone may remain if no corner
        # bumps into anything -- with a tight margin, so you can still turn
        # out even right in front of a wall.
        rest = STOP
        if abs(cmd.omega) > 1e-6 and (abs(cmd.vx) > 1e-6 or abs(cmd.vy) > 1e-6):
            turn_check = self._drive_guard.check(DriveCommand(0.0, 0.0, cmd.omega), min(margin, 0.03))
            if turn_check.share > 0.0:
                rest = turn_check.command
        if self._blocked_since is None:
            self._blocked_since = time.monotonic()
        return rest, (verdict.free_m or 0.0) + margin

    def report(self) -> bool:
        """Whether a message to the app makes sense now (debounced)."""
        now = time.monotonic()
        if now - self._last_reported < self.REPORT_INTERVAL_S:
            return False
        self._last_reported = now
        return True
