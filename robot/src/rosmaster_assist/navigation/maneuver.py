"""Precise drive commands: "x metres forward/back/left/right" and "turn x degrees".

The robot should really drive straight and really turn by the angle.
Driving by time (speed × duration) is not good enough for that: the mecanum
wheels slip, especially when turning, and the drive command alone
overestimates the motion by 10-30 % (sensor check 28.10.). The gyro has a
large bias and the wrong sign.

So the lidar measures. At the start a scan is taken, and every further one
is laid onto it (ICP, see scan_matching.py). From that follows how far the
robot has really moved and turned relative to the start -- the lidar does
not care about slipping, it looks at the walls. Wheels and drive command
only provide the estimate for the time between two scans (7.7 per second)
and the initial guess for the matching.

When turning, the reference scan changes every ~25°, otherwise there would
be too little overlap after half a turn. The angles are accumulated, not
wrapped to ±180° -- so "360 degrees" works too.

Driving straight: the controller keeps not only the remaining distance but
also the lateral deviation and the heading at zero. The X3 can drive
sideways, so a deviation is corrected directly instead of by steering.

At the end it measures again while standing still: during the drive a lidar
revolution is distorted (the robot moves meanwhile). Only a scan at
standstill says exactly where it is; if it is off, it corrects.

The camera is not needed for this: it only looks forward and does not
measure distances. The lidar sees all around and measures to the centimetre.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections import deque

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from ..safety.drive_guard import NO_LIDAR
from .explorer import free_in_direction
from .scan_matching import match

log = logging.getLogger(__name__)

FORWARD = "vor"
BACK = "zurueck"
LEFT = "links"
RIGHT = "rechts"
TURN = "drehen"

#: Driving direction in the robot frame (0° = front, 90° = left).
DIRECTIONS = {FORWARD: 0.0, BACK: 180.0, LEFT: 90.0, RIGHT: -90.0}

_DIRECTION_TEXT = {FORWARD: "vor", BACK: "zurück", LEFT: "nach links", RIGHT: "nach rechts"}


def describe(category: str, value: float) -> str:
    """'1,5 m vor', '90° nach links' -- for messages and announcements."""
    if category == TURN:
        side = "nach links" if value >= 0 else "nach rechts"
        return f"{abs(value):g}° {side}".replace(".", ",")
    return f"{value:g} m {_DIRECTION_TEXT[category]}".replace(".", ",")


# --- Pose in the plane: (x, y, angle), angle NOT wrapped ------------------------------

def _compose(a, b):
    """First pose a, then b from there (a ∘ b)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def _invert(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), -at)


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _thin_out(points, at_most: int):
    """Evenly reduce to at most this many points -- the matching works on
    all pairs, so the time grows with the square."""
    if len(points) <= at_most:
        return points
    import numpy as np

    selection = (np.arange(at_most) * (len(points) / at_most)).astype(int)
    return points[selection]


class Maneuver:
    """Executes a precise drive command. Used like ``Homing``."""

    def __init__(self, watchdog: SafetyWatchdog, driver, lidar) -> None:
        self._watchdog = watchdog
        self._driver = driver
        self._lidar = lidar
        self._task: asyncio.Task[None] | None = None
        self._reason: str | None = None
        self._kind: str | None = None
        self._value = 0.0
        self._text = ""
        self.succeeded = False
        self.announcement: str | None = None  # what it says when it did not work
        self._turning_since: float | None = None  # since when it has been turning (from standstill)
        self.finished = 0  # counts every self-ended command (for the server's message)

        # Lidar localization during the command, relative to the pose at the start.
        self._pose = (0.0, 0.0, 0.0)
        self._history: deque = deque(maxlen=200)  # (time, pose) for the lookback
        self._last_time = 0.0
        self._reference = None                  # points of the reference scan
        self._reference_pose = (0.0, 0.0, 0.0)  # where it was taken
        self._last_scan = 0.0
        self._revolution = 0.13                 # duration of one lidar revolution (s), measured
        self._matches = 0
        self._failures = 0

    # --- Control -----------------------------------------------------------------

    @property
    def available(self) -> bool:
        return self._lidar is not None

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def last_reason(self) -> str | None:
        return self._reason

    def start(self, category: str, value: float) -> str | None:
        """Start a command. Returns an error text if it is not possible."""
        if not self.available:
            return "Genaue Fahrbefehle gehen nur mit Lidar"
        try:
            value = float(value)
        except (TypeError, ValueError):
            return "Zahl erwartet"
        if not math.isfinite(value):
            return "Zahl erwartet"
        if category == TURN:
            if not 0 < abs(value) <= config.MANEUVER_MAX_DEG:
                return f"Drehen: 1 bis {config.MANEUVER_MAX_DEG:g} Grad"
        elif category in DIRECTIONS:
            if not 0 < value <= config.MANEUVER_MAX_M:
                return f"Fahren: bis {config.MANEUVER_MAX_M:g} m"
        else:
            return f"unbekannte Richtung {category!r}"
        self.cancel("neuer Befehl")
        self._kind, self._value = category, value
        self._reason = None
        self.succeeded = False
        self.announcement = None
        self._text = f"{describe(category, value)}: startet"
        self._task = asyncio.create_task(self._drive(category, value))
        return None

    def cancel(self, reason: str = "abgebrochen") -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._reason = reason
            log.info("Drive command %s", reason)
        self._task = None

    def state(self) -> dict:
        x, y, th = self._pose
        return {
            "aktiv": self.active,
            "art": self._kind,
            "wert": self._value,
            "text": self._text,
            "grund": self._reason,
            "erfolgreich": self.succeeded,
            # Achieved so far, relative to the start (lidar).
            "gefahren_m": round(math.hypot(x, y), 3),
            "gedreht_grad": round(math.degrees(th), 1),
            "abgleiche": self._matches,
            "fehlschlaege": self._failures,
        }

    # --- Sequence ----------------------------------------------------------------

    async def _drive(self, category: str, value: float) -> None:
        tick = 1.0 / max(1, config.ODOMETRY_HZ)
        if category == TURN:
            target = (0.0, 0.0, math.radians(value))
            duration = abs(math.radians(value)) / config.MANEUVER_TURN
        else:
            r = math.radians(DIRECTIONS[category])
            target = (value * math.cos(r), value * math.sin(r), 0.0)
            duration = value / config.MANEUVER_SPEED
        # Generous: starting up, corrections, brief waits for scans.
        deadline = time.monotonic() + 3.0 * duration + 10.0
        description = describe(category, value)

        try:
            # Whatever was running before stops now; then a scan at standstill.
            self._watchdog.stop()
            scan = await self._scan_after(time.monotonic() + config.MANEUVER_SETTLE_S)
            if scan is None:
                return self._end("keine frische Lidar-Aufnahme",
                                  "Mein Lidar sieht gerade nichts, ich fahre lieber nicht.")
            if category == TURN:
                narrow = self._turn_cramped(scan, math.radians(value))
                if narrow is not None:
                    return self._end(f"zum Drehen zu wenig Platz ({narrow})",
                                      "Zum Drehen ist es mir hier zu eng.")
            self._pose = (0.0, 0.0, 0.0)
            self._reference = _thin_out(scan.points(), config.MANEUVER_MAX_POINTS)
            self._reference_pose = self._pose
            self._last_scan = scan.timestamp
            self._last_time = time.monotonic()
            self._history.clear()
            self._history.append((scan.timestamp, self._pose))
            self._matches = self._failures = 0
            detected = time.monotonic()
            corrections = 0
            blocked_since: float | None = None
            log.info("Drive command %s started", description)

            while True:
                self._predict()
                scan = self._lidar.latest()
                if scan is not None and scan.timestamp > self._last_scan:
                    if await asyncio.to_thread(self._match, scan):
                        detected = time.monotonic()
                if time.monotonic() - detected > config.MANEUVER_BLIND_S:
                    return self._end("Lidar erkennt die Umgebung nicht wieder -- angehalten",
                                      "Ich habe die Orientierung verloren und angehalten.")

                ex, ey, eth = self._error(target)
                if self._reached(category, ex, ey, eth, config.MANEUVER_GOAL_SHARE):
                    self._watchdog.stop()
                    scan = await self._scan_after(time.monotonic() + config.MANEUVER_SETTLE_S)
                    if scan is not None and await asyncio.to_thread(self._match, scan):
                        ex, ey, eth = self._error(target)
                    # Measured at standstill and strictly at the goal: done.
                    # Otherwise correct; only once the corrections are used
                    # up does the normal tolerance suffice.
                    precise = self._reached(category, ex, ey, eth, config.MANEUVER_GOAL_SHARE)
                    exhausted = corrections >= config.MANEUVER_MAX_CORRECTIONS
                    if precise or (exhausted and self._reached(category, ex, ey, eth)):
                        self.succeeded = True
                        return self._end(f"{description} erledigt ({self._remainder(category, ex, ey, eth)})")
                    corrections += 1
                    if corrections > config.MANEUVER_MAX_CORRECTIONS:
                        return self._end(f"{description} fast erledigt ({self._remainder(category, ex, ey, eth)})",
                                          self._off_target(category, ex, ey, eth))
                    log.info("Drive command: measured at standstill, %s left -- correcting",
                             self._remainder(category, ex, ey, eth))
                    detected = time.monotonic()
                    continue

                if time.monotonic() > deadline:
                    return self._end(f"Zeit abgelaufen, noch {self._remainder(category, ex, ey, eth)}",
                                      "Das hat zu lange gedauert, ich habe aufgehört.")

                command = self._command(category, ex, ey, eth)
                announcement = ("Mir ist beim Drehen etwas zu nah, ich habe angehalten."
                          if category == TURN else "Da ist etwas im Weg, ich habe angehalten.")
                obstacle = self._obstacle(category, command)
                if obstacle is not None:
                    return self._end(obstacle, announcement)
                self._text = f"{description}: noch {self._remainder(category, ex, ey, eth)}"
                # Autonomous driving like following and exploring: tighter
                # speed limit, and the drive guard in the watchdog checks the
                # whole footprint including swinging corners. The check above
                # only knows a strip in the driving direction.
                self._watchdog.submit(command, source="autonom")
                verdict = self._watchdog.last_check
                if (verdict is not None and verdict.share == 0.0
                        and verdict.reason != NO_LIDAR and not command.is_stop()):
                    # Wait briefly (someone walking past), then give up instead
                    # of standing in front of the obstacle until the deadline.
                    # Without lidar MANEUVER_BLIND_S applies.
                    now = time.monotonic()
                    blocked_since = blocked_since or now
                    if now - blocked_since > config.MANEUVER_BLOCKED_S:
                        return self._end(f"Fahrschutz: {verdict.reason} -- angehalten", announcement)
                    self._text += f" (Fahrschutz: {verdict.reason})"
                else:
                    blocked_since = None
                await asyncio.sleep(tick)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler während der Fahrt"
            self.finished += 1
            log.exception("Drive command aborted")

    def _end(self, reason: str, announcement: str | None = None) -> None:
        """End the command. ``announcement``: what it says aloud when it did
        not work -- when started by voice nobody looks at the app, and a bare
        "did not work" leaves people guessing whether to clear something away."""
        self._watchdog.stop()
        self._reason = reason
        self._text = reason
        self.announcement = announcement
        self.finished += 1
        (log.info if self.succeeded else log.warning)("Drive command: %s", reason)

    @staticmethod
    def _off_target(category: str, ex: float, ey: float, eth: float) -> str:
        if category == TURN:
            return f"Fast geschafft, ich liege {abs(math.degrees(eth)):.0f} Grad daneben."
        return f"Fast geschafft, ich liege {math.hypot(ex, ey) * 100:.0f} Zentimeter daneben."

    async def _scan_after(self, instant: float, deadline_s: float = 2.0):
        """First lidar scan taken entirely after ``instant``."""
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            self._predict()
            scan = self._lidar.latest()
            # A whole revolution after the instant: scans begun earlier still
            # contain motion.
            if scan is not None and scan.timestamp - self._revolution >= instant:
                return scan
            await asyncio.sleep(0.02)
        return None

    # --- Localization ------------------------------------------------------------

    def _velocity(self) -> tuple[float, float, float]:
        """How it is moving right now (robot frame).

        Forward/sideways from the wheels (accurate), rotation from the drive
        command -- the gyro is useless, and so is the wheel turn rate because
        of slipping. The lidar corrects both with every scan.
        """
        commanded = self._watchdog.current_command
        measured = None
        try:
            measured = self._driver.read_motion()
        except Exception:
            pass
        if measured is None:
            return commanded.vx, commanded.vy, commanded.omega
        return measured[0], measured[1], commanded.omega

    def _predict(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last_time, 0.0), 0.2)
        self._last_time = now
        if dt <= 0:
            return
        vx, vy, w = self._velocity()
        # From standstill the motors only really turn after a while (see
        # MANEUVER_STARTUP_S); do not count it until then. Underestimating is
        # harmless -- the next lidar scan corrects it --, overestimating made
        # corrections miss.
        if abs(w) < 1e-3:
            self._turning_since = None
        else:
            if self._turning_since is None:
                self._turning_since = now
            if now - self._turning_since < config.MANEUVER_STARTUP_S:
                w = 0.0
        # Rotate in the middle of the step: more accurate when driving while turning.
        half = (0.0, 0.0, w * dt / 2.0)
        self._pose = _compose(_compose(_compose(self._pose, half),
                                               (vx * dt, vy * dt, 0.0)), half)
        self._history.append((now, self._pose))

    def _pose_at(self, instant: float):
        for t, pose in reversed(self._history):
            if t <= instant:
                return pose
        return self._history[0][1] if self._history else self._pose

    def _match(self, scan) -> bool:
        """Lay the scan onto the reference scan and correct the pose."""
        distance = scan.timestamp - self._last_scan
        if 0.05 < distance < 0.4:
            self._revolution = 0.8 * self._revolution + 0.2 * distance
        self._last_scan = scan.timestamp
        points = _thin_out(scan.points(), config.MANEUVER_MAX_POINTS)
        # A scan takes one revolution; on average it shows the pose half a
        # revolution before its timestamp.
        past_pose = self._pose_at(scan.timestamp - self._revolution / 2.0)
        estimate = _compose(_invert(self._reference_pose), past_pose)
        result = match(points, self._reference, estimate)
        if result is None or result.residuum > config.MATCH_MAX_RESIDUAL_M:
            self._failures += 1
            return False
        self._matches += 1
        # ICP wraps the angle to ±180°; place it next to the estimate so the
        # accumulated rotation is preserved.
        angle = estimate[2] + _wrap_angle(result.theta - estimate[2])
        relative = (result.x, result.y, angle)
        measured = _compose(self._reference_pose, relative)
        # Correction K with K ∘ then = measured; also applies to everything after.
        k = _compose(measured, _invert(past_pose))
        self._pose = _compose(k, self._pose)
        self._history = deque(((t, _compose(k, p)) for t, p in self._history),
                              maxlen=self._history.maxlen)
        # Only change the reference when the matching degrades (see config).
        if (result.residuum > config.MANEUVER_NEW_REF_RESIDUAL_M
                or result.pairs < config.MANEUVER_NEW_REF_PAIRS * len(points)):
            log.debug("Drive command: new reference scan (residual %.3f m, %d pairs)",
                      result.residuum, result.pairs)
            self._reference, self._reference_pose = points, measured
        return True

    # --- Control loop ------------------------------------------------------------

    def _error(self, target) -> tuple[float, float, float]:
        """Remaining distance (forward, left) in the robot frame and remaining angle."""
        x, y, th = self._pose
        dx, dy = target[0] - x, target[1] - y
        c, s = math.cos(th), math.sin(th)
        return c * dx + s * dy, -s * dx + c * dy, target[2] - th

    @staticmethod
    def _reached(category: str, ex: float, ey: float, eth: float, share: float = 1.0) -> bool:
        """At the goal? ``share`` < 1: stricter (while driving, see MANEUVER_GOAL_SHARE)."""
        angle_ok = abs(math.degrees(eth)) < config.MANEUVER_TOLERANCE_DEG * share
        if category == TURN:
            # When turning on the spot only the angle counts; the few
            # centimetres mecanum wanders meanwhile do not matter.
            return angle_ok
        return angle_ok and math.hypot(ex, ey) < config.MANEUVER_TOLERANCE_M * share

    @staticmethod
    def _remainder(category: str, ex: float, ey: float, eth: float) -> str:
        deg = f"{math.degrees(eth):+.1f}°"
        if category == TURN:
            return deg
        return f"{math.hypot(ex, ey) * 100:.0f} cm, {deg}"

    @staticmethod
    def _command(category: str, ex: float, ey: float, eth: float) -> DriveCommand:
        # Drive into the strict zone, see _reached().
        share = config.MANEUVER_GOAL_SHARE
        tolerance_rad = math.radians(config.MANEUVER_TOLERANCE_DEG) * share
        omega = max(-config.MANEUVER_TURN, min(config.MANEUVER_TURN, config.MANEUVER_KP_W * eth))
        if category == TURN:
            if abs(eth) >= tolerance_rad and abs(omega) < config.MANEUVER_MIN_TURN:
                omega = math.copysign(config.MANEUVER_MIN_TURN, eth)
            return DriveCommand(0.0, 0.0, omega)

        # Driving: towards the goal, correcting the lateral deviation at the same time (mecanum).
        rest = math.hypot(ex, ey)
        speed = min(config.MANEUVER_SPEED, config.MANEUVER_KP_V * rest)
        if rest >= config.MANEUVER_TOLERANCE_M * share:
            speed = max(speed, config.MANEUVER_MIN_SPEED)
            vx, vy = ex / rest * speed, ey / rest * speed
        else:
            vx = vy = 0.0
            # Only the heading is still wrong: turn at standstill.
            if abs(eth) >= tolerance_rad and abs(omega) < config.MANEUVER_MIN_TURN:
                omega = math.copysign(config.MANEUVER_MIN_TURN, eth)
        return DriveCommand(vx, vy, omega)

    def _turn_cramped(self, scan, angle_rad: float) -> str | None:
        """Before turning: reason why there is not enough room, otherwise None.

        With the drive guard the footprint is checked over the whole angle.
        The earlier 30 cm circle around the lidar often blocked for no reason
        in the floor test (legs, furniture in front) -- at the front the robot
        only protrudes 15 cm.
        """
        drive_guard = self._watchdog.drive_guard
        if drive_guard is not None:
            return drive_guard.turn_free(angle_rad)
        close = scan.min_distance(0.0, 180.0)
        if close < config.MANEUVER_TURN_FREE_M:
            return f"etwas in {close:.2f} m"
        return None

    def _obstacle(self, category: str, command: DriveCommand) -> str | None:
        """Reason to stop, or None."""
        scan = self._lidar.latest()
        if scan is None:
            return None  # MANEUVER_BLIND_S applies
        if category == TURN:
            if self._watchdog.drive_guard is not None:
                return None  # the drive guard checks this over the footprint
            close = scan.min_distance(0.0, 180.0)
            if close < config.MANEUVER_TURN_FREE_M:
                return f"beim Drehen etwas in {close:.2f} m -- angehalten"
            return None
        if abs(command.vx) < 1e-6 and abs(command.vy) < 1e-6:
            return None
        direction = math.degrees(math.atan2(command.vy, command.vx))
        free = free_in_direction(scan.points(), direction, config.EXPLORE_HALF_WIDTH_M)
        if free - config.EXPLORE_STOP_M < 0.02:
            return f"Hindernis in {free:.2f} m -- angehalten"
        return None
