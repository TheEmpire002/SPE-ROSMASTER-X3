"""Safety layer between the server and the drive.

Two tasks:

1. **Limit.** Every drive command is clamped to the limits from `config`
   before it reaches the motors. A programming error in the app therefore
   cannot make the robot faster than allowed.

2. **Dead man's switch.** If the drive commands stop arriving -- Wi-Fi gone,
   browser tab closed, app crashed -- the robot stops by itself after
   `COMMAND_TIMEOUT_S`.

Point 2 is the reason this layer exists. It must not be something the app
has to remember: a driving robot without a connection is exactly the
situation in which nothing up there works any more.

3. **Ramp.** Commands do not jump straight to the motors but are approached
   with limited acceleration (`RAMP_*` in `config`). Hard jumps between
   driving and stopping cause current spikes that reset the USB hub on the
   X3. `stop()` bypasses the ramp and stops immediately.

4. **Autonomous limit.** Commands from the autonomous modes (`source="autonom"`)
   are additionally clamped to `AUTONOMOUS_MAX_*`, and reduced once more when
   the battery is low. So no calculation error in follow, explore or homing
   can make the robot faster than the lidar sees obstacles.

5. **Autonomous DriveGuard.** Then `drive_guard` (`drive_guard.py`, set by
   the server) checks whether the robot's footprint would touch something
   with this command -- also in an arc and when turning, when the corners
   swing out -- and brakes smoothly so that it stops beforehand, allowing for
   reaction time and braking distance. Driving away stays allowed. Without
   fresh lidar data no autonomous mode drives.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .. import config
from ..drive.base import DriveBase
from ..protocol import DriveCommand, STOP

log = logging.getLogger(__name__)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _ramp(actual: float, requested: float, accel: float, brake: float, dt: float) -> float:
    """Take one ramp step from `actual` towards `requested`.

    Everything that reduces the magnitude counts as braking -- including a
    change of direction, which first passes through zero. There the step ends
    at zero; it continues in the new direction only in the next step, with
    the acceleration limit.
    """
    if actual * requested < 0:
        step = brake * dt
        if abs(actual) <= step:
            return 0.0
        return actual - math.copysign(step, actual)
    limit = brake if abs(requested) < abs(actual) else accel
    step = limit * dt
    if abs(requested - actual) <= step:
        return requested
    return actual + math.copysign(step, requested - actual)


class SafetyWatchdog:
    """Accepts drive commands and passes them on, limited, to the drive."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._last_command = STOP     # what is being driven right now (after the ramp)
        self._target = STOP             # what was requested last
        self._last_command_at = 0.0   # when something was last requested
        self._output_at = 0.0        # when last passed to the drive
        self._stopped = True
        self._task: asyncio.Task[None] | None = None
        # Battery voltage is only checked occasionally -- it changes
        # slowly, and submit() runs at a 20 Hz tick.
        self._battery_v: float | None = None
        self._battery_at = -1e9
        # Collision check via the footprint for autonomous commands
        # (point 5, safety/drive_guard.py). Stays None if no lidar is
        # configured -- then there are no autonomous modes that could
        # drive either. last_check: result of the last autonomous command,
        # so the modes can see whether and why it braked.
        self.drive_guard = None
        self.last_check = None
        # Recorder (recording.py), set by the server. Records every
        # command before and after the limits -- so you can see later where
        # the DriveGuard intervened.
        self.recording = None

    # --- Control --------------------------------------------------------------

    def submit(
        self, cmd: DriveCommand, source: str = "hand", margin_m: float | None = None
    ) -> DriveCommand:
        """Execute a drive command. Returns the command actually driven.

        `source` is "hand" (app, ESP32) or "autonom" (follow, explore,
        mapping drive, homing, exact drive commands). The tighter limit
        applies to autonomous commands, and the DriveGuard checks them.
        Manual control is checked beforehand by the ``CollisionGuard``
        (can be switched off in the app).

        ``margin_m`` changes the distance the DriveGuard keeps for autonomous
        commands (default ``GUARD_MARGIN_M``). The health check's nudge needs
        it because it deliberately drives up to contact; the speed and
        battery limits still apply.
        """
        now = time.monotonic()
        limit_xy_x, limit_xy_y = config.MAX_SPEED_X, config.MAX_SPEED_Y
        limit_w = config.MAX_SPEED_OMEGA
        if source == "autonom":
            autonomous = config.AUTONOMOUS_MAX_SPEED * self._battery_factor(now)
            limit_xy_x = min(limit_xy_x, autonomous)
            limit_xy_y = min(limit_xy_y, autonomous)
            limit_w = min(limit_w, config.AUTONOMOUS_MAX_OMEGA)
        safe = DriveCommand(
            vx=_clamp(cmd.vx, limit_xy_x),
            vy=_clamp(cmd.vy, limit_xy_y),
            omega=_clamp(cmd.omega, limit_w),
        )
        if source == "autonom":
            # None without a check, otherwise the modes would see a stale result.
            self.last_check = None
            if self.drive_guard is not None and config.AUTONOMOUS_GUARD_ACTIVE:
                self.last_check = self.drive_guard.check(safe, margin_m=margin_m)
                safe = self.last_check.command
                if self.last_check.reason is not None:
                    self.drive_guard.report_intervention(self.last_check.reason, now)
        if safe != cmd:
            log.debug("Drive command limited (%s): %s -> %s", source, cmd, safe)
        if self.recording is not None:
            try:
                self.recording.command(source, cmd, safe, now)
            except Exception:
                # The recording is an extra -- driving comes first.
                log.debug("Recording the drive command failed", exc_info=True)

        self._target = safe
        self._last_command_at = now
        return self._emit(now)

    def _battery_factor(self, now: float) -> float:
        """Speed factor for autonomous modes depending on battery voltage."""
        if now - self._battery_at > 2.0:
            self._battery_at = now
            try:
                self._battery_v = self._driver.read_battery()
            except Exception:
                # Do not throttle without a reading -- diagnostics reports the
                # failure anyway, and a driver error must not prevent the
                # drive command here.
                self._battery_v = None
        if self._battery_v is not None and self._battery_v < config.BATTERY_WARN_V:
            return config.AUTONOMOUS_BATTERY_FACTOR
        return 1.0

    def _emit(self, now: float) -> DriveCommand:
        """Pass one ramp step towards the target to the drive."""
        target = self._target
        if config.RAMP_ACTIVE:
            dt = min(max(now - self._output_at, 0.0), config.RAMP_DT_MAX_S)
            actual = self._last_command
            new = DriveCommand(
                vx=_ramp(actual.vx, target.vx, config.RAMP_ACCEL_XY,
                          config.RAMP_BRAKE_XY, dt),
                vy=_ramp(actual.vy, target.vy, config.RAMP_ACCEL_XY,
                          config.RAMP_BRAKE_XY, dt),
                omega=_ramp(actual.omega, target.omega, config.RAMP_ACCEL_OMEGA,
                             config.RAMP_BRAKE_OMEGA, dt),
            )
        else:
            new = target
        self._output_at = now
        self._last_command = new
        # Only "stopped" once the ramp has also reached zero -- until then
        # the monitoring has to keep delivering steps.
        self._stopped = new.is_stop() and target.is_stop()

        try:
            self._driver.drive(new)
        except Exception:
            # On error: better to stand still than to keep driving.
            log.exception("Drive reports an error -- stopping")
            self.stop()
            raise

        return new

    def stop(self) -> None:
        """Stop immediately -- without the ramp."""
        self._driver.stop()
        self._last_command = STOP
        self._target = STOP
        self._stopped = True
        if self.recording is not None:
            try:
                self.recording.stop()
            except Exception:
                log.debug("Recording the stop failed", exc_info=True)

    @property
    def current_command(self) -> DriveCommand:
        """What the robot is doing right now (after limiting)."""
        return self._last_command

    # --- Background monitoring ------------------------------------------------

    async def run(self) -> None:
        """Monitoring loop. Runs as long as the server runs."""
        log.info("Watchdog active (timeout %.1fs)", config.COMMAND_TIMEOUT_S)
        try:
            while True:
                await asyncio.sleep(config.WATCHDOG_INTERVAL_S)
                self._check_timeout()
                self._update_ramp()
        except asyncio.CancelledError:
            # On shutdown: make sure the robot is stopped.
            self.stop()
            raise

    def _check_timeout(self) -> None:
        if self._stopped:
            return  # already stopped
        age = time.monotonic() - self._last_command_at
        if age > config.COMMAND_TIMEOUT_S:
            log.warning("No drive command for %.2fs -- emergency stop", age)
            self.stop()

    def _update_ramp(self) -> None:
        """Continue the ramp between two commands as well.

        The app sends only ten commands per second, and whoever says "stop"
        once often sends nothing at all afterwards. Without this step the
        output would then get stuck halfway.
        """
        if self._stopped or self._last_command == self._target:
            return
        try:
            self._emit(time.monotonic())
        except Exception:
            # _emit has already stopped and logged. The monitoring itself
            # must not die from it.
            pass

    # --- Lifecycle -------------------------------------------------------------

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self.run())

    async def shutdown(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self.stop()
