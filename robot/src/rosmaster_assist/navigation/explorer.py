"""Autonomous exploration (test 1): drive around randomly, avoid everything.

The robot drives on a chosen heading, goes around obstacles in arcs and
only turns towards a free direction (a door or gap, as centered as
possible) when no safe arc fits any more. So that it does not drive the
same path forever, it changes direction now and then without a reason. The
building blocks for this (short-term memory, arcs, gaps, lift detection)
are in ``avoidance.py``.

Decisions that shape the behavior:

**Turning goes by measured angle, time is only an upper limit.** The
library's IMU heading is unusable on the X3 (sensor check 28.10.2026:
``attitude`` always 0). The turn rate reported by the board, on the other
hand, agrees well with the lidar. The explorer integrates it itself during
the turn instead of taking the odometry's ``theta``: lidar corrections can
make ``theta`` jump. If the measurement is missing, it turns by time as
before (``EXPLORE_TURN_FACTOR``). Afterwards it only decides with a lidar
revolution that started after the turn; the revolution before still shows
the old direction.

**What counts is the driving corridor, not a cone.** A cone is narrow close
to the robot (misses a table leg next to the wheel) and wide far away
(stops in front of walls it would pass). Instead, a strip of robot width in
the driving direction is checked.

**Unstick instead of standing forever.** If something comes very close, it
first waits briefly (maybe a person who is about to move on) and then
pushes itself, using the mecanum drive and without turning, in the freest
direction: when turning, the corners swing outwards. The same if it makes
no progress despite a drive command (something below the lidar plane).
Only when several unstick attempts in a row achieve nothing does it give
up.

**Lifted means starting over.** If someone carries it somewhere else,
nothing learned fits any more (memory, heading, counters). It stops until
the lidar image is calm again and then starts from scratch.

Every drive command goes through the ``SafetyWatchdog`` -- if this loop
hangs, the robot stops by itself after 0.5 s.
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import time

from typing import TYPE_CHECKING

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import avoidance
from . import obstacles as hd
from .progress import Stuck, ProgressWatch

if TYPE_CHECKING:  # only for the type hints -- avoids circular imports
    from ..sensors.lidar import LidarService
    from ..sensors.people import PersonDetection
    from ..sensors.depth import DepthCamera
    from .obstacle_memory import ObstacleMemory
    from .mapping_drive import MappingDrive
    from .odometry import Odometry

log = logging.getLogger(__name__)

#: States as they appear in the app (state.autonomie.zustand).
DRIVING = "faehrt"
TURNING = "dreht"
WAITING = "wartet"
WAITING_FOR_HUMAN = "wartet auf Mensch"
NO_LIDAR = "wartet auf Lidar"
UNSTICKING = "befreit sich"
LIFTED = "hochgehoben"
MAPPED = "kartiert"

#: Modes (state.autonomie.modus): drive around randomly or cover the room
#: systematically for the map (mapping_drive.py).
RANDOM = "zufall"
MAP = "karte"

#: Candidates for the new driving direction, in degrees (0 = straight ahead).
_DIRECTIONS = list(range(-180, 181, 10))

#: Candidates when unsticking (push sideways/backwards), all around.
_UNSTICK_DIRECTIONS = list(range(-180, 180, 15))

# Rays all around for stuck detection via lidar. Looking only forward is not
# enough: if it drives diagonally along a wall, the distance in the driving
# corridor stays the same although it makes good progress.
_STUCK_DIRECTIONS = list(range(-180, 180, 30))
_STUCK_RAY_M = 0.05

#: After this many unsuccessful turns in a row, turning further no longer
#: helps -- then it tries to push itself free.
_MAX_FAILED_ATTEMPTS = 6


def free_in_direction(points, direction_deg: float, half_width: float) -> float:
    """How far the robot could drive in this direction (meters).

    Measured from the lidar center to the first point in the driving
    corridor. Infinite if there is nothing there.
    """
    import numpy as np

    if len(points) == 0:
        return float("inf")
    a = math.radians(direction_deg)
    c, s = math.cos(a), math.sin(a)
    ahead = points[:, 0] * c + points[:, 1] * s
    lateral = -points[:, 0] * s + points[:, 1] * c
    in_path = (ahead > 0.0) & (np.abs(lateral) < half_width)
    return float(ahead[in_path].min()) if in_path.any() else float("inf")


class Explorer:
    """Drives around autonomously. Same controls as ``Homing``.

    Second mode ``MAP``: the room is covered systematically for the map.
    The decisions are then made by ``MappingDrive``; everything else --
    lidar check, near zone, cancellation by the human -- stays here so that
    it is guaranteed to apply equally to both modes.
    """

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        lidar: "LidarService | None",
        obstacle_detection: hd.ObstacleDetection,
        people: "PersonDetection | None" = None,
        odometry: "Odometry | None" = None,
        mapping_drive: "MappingDrive | None" = None,
        obstacles: "ObstacleMemory | None" = None,
        depth: "DepthCamera | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._detection = obstacle_detection
        self._people = people
        self._odometry = odometry
        self._mapping_drive = mapping_drive
        # What the lidar does not see: depth camera with memory, and spots
        # where it got stuck. Both may be missing (simulation, no Astra).
        self._obstacles = obstacles
        self._depth = depth
        self._progress = ProgressWatch(watchdog)
        self._mode = RANDOM
        self._resume = False
        self._task: asyncio.Task[None] | None = None
        self._reason: str | None = None

        # For the display -- written by the loop, read by the status tick
        # (both in the same event loop, no lock needed).
        self._state = WAITING
        self._text = ""
        self._free_ahead: float | None = None
        self._started = 0.0
        self._avoidances = 0
        self._people_seen = 0
        self._unstuck_total = 0

        self._reset(0.0)

    def _reset(self, now: float) -> None:
        """Run state for a new drive."""
        self._last_tick: float | None = None
        self._prev_command = DriveCommand(0.0, 0.0, 0.0)
        # Turning
        self._turn_direction = 0.0
        self._turn_start = now
        self._turn_end = now
        self._turn_limit = now
        self._turn_measured = False
        self._turned = 0.0  # degrees, integrated from the board's turn rate
        self._turn_done: float | None = None  # when the turn command ended
        self._failed_attempts = 0
        self._next_switch = now
        self._wait_until = 0.0
        self._no_lidar_since: float | None = None
        # Near stop and unsticking
        self._near_since: float | None = None
        self._unstick_direction = 0.0
        self._unstick_end = 0.0
        self._unsticks = 0  # in a row, without driving freely in between
        self._free_since: float | None = None
        self._last_unstick = now  # when an unstick attempt last began
        self._progress.reset()
        # Stuck detection
        self._stuck_start: float | None = None
        self._stuck_snapshot: dict[int, float] = {}
        self._stuck_still_s = 0.0
        self._stuck_tick = 0.0
        # Avoidance (F2): short-term memory, lift detection and the heading
        # it steers back to after going around (None: take the current one
        # the next time it drives straight).
        self._memory = avoidance.ShortTermMemory(self._odometry)
        self._lift = avoidance.LiftDetector()
        self._goal_heading: float | None = None

    # --- Controls (like Homing) ------------------------------------------------

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def last_reason(self) -> str | None:
        """Why the last exploration ended -- for display in the app."""
        return self._reason

    @property
    def available(self) -> bool:
        """Without lidar it does not drive -- driving around blind would be dangerous."""
        return self._lidar is not None

    @property
    def mode(self) -> str | None:
        """RANDOM or MAP while a drive is running, otherwise None."""
        return self._mode if self.active else None

    @property
    def last_mode(self) -> str:
        """Mode of the running or most recent drive."""
        return self._mode

    def start(self, mode: str = RANDOM, resume: bool = False) -> bool:
        """Start exploring. False if one is already running or there is no lidar.

        ``resume`` (MAP only): after an interruption, continue straight into
        the unknown instead of covering the room again.
        """
        if self.active or not self.available:
            return False
        if mode == MAP and self._mapping_drive is None:
            return False
        self._mode = mode
        self._resume = resume and mode == MAP
        self._reason = None
        # Right here, not only in the task: otherwise the first status shows
        # the duration since server start.
        self._started = time.monotonic()
        self._task = asyncio.create_task(self._drive())
        return True

    def cancel(self, reason: str = "abgebrochen") -> None:
        """Stop immediately. Also to be called when the human intervenes."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._reason = reason
            log.info("Exploration %s", reason)
        self._task = None

    def state(self) -> dict:
        """For state.autonomie in the app."""
        active = self.active
        return {
            "aktiv": active,
            "modus": self.mode,
            "kartierfahrt": (
                self._mapping_drive.state()
                if active and self._mode == MAP and self._mapping_drive is not None
                else None
            ),
            "verfuegbar": self.available,
            "zustand": self._state if active else None,
            "text": self._text if active else None,
            "frei_vorn": (
                round(self._free_ahead, 2)
                if active and self._free_ahead is not None and math.isfinite(self._free_ahead)
                else None
            ),
            "dauer_s": round(time.monotonic() - self._started) if active else None,
            "ausweichen": self._avoidances,
            "menschen": self._people_seen,
            "befreien": self._unstuck_total,
            # Points only the depth camera saw, and spots where it made no
            # progress despite a drive command (None without depth camera).
            "tiefe": self._obstacles.count if self._obstacles is not None else None,
            "festgefahren": self._progress.verdicts,
            "grund": self._reason,
        }

    # --- Drive loop ----------------------------------------------------------------

    def _begin(self, now: float) -> None:
        """Counters and run state for a new drive -- also for the tests."""
        self._started = now
        self._avoidances = 0
        self._people_seen = 0
        self._unstuck_total = 0
        self._progress.verdicts = 0
        self._reason = None
        self._reset(now)
        self._next_switch = now + self._wander_time()
        self._set(DRIVING, "fährt los")
        if self._mode == MAP:
            self._mapping_drive.begin(now, resume=self._resume)

    async def _drive(self) -> None:
        tick = 1.0 / max(1, config.ODOMETRY_HZ)
        self._begin(time.monotonic())
        occ_map = self._mode == MAP
        # Only random driving needs the person camera (waiting in front of
        # people); the mapping drive relies on the near zone.
        if self._people is not None and not occ_map:
            self._people.activate(True)
        # The depth camera only while driving: its projector draws power,
        # and on the X3 a USB device then drops out more easily.
        if self._depth is not None:
            self._depth.activate("erkunder", True)
        log.info("Exploration started (%s)", self._mode)

        blocked_since: float | None = None
        try:
            while True:
                now = time.monotonic()
                command = self._step(now)
                if command is None:  # done, reason is in self._reason
                    self._watchdog.stop()
                    log.info("Exploration ended -- %s", self._reason)
                    return
                # Last resort: the drive guard in the watchdog checks the whole
                # footprint including rotation and brakes what the logic above missed.
                self._watchdog.submit(command, source="autonom")
                verdict = self._watchdog.last_check
                if verdict is not None and verdict.reason and not command.is_stop():
                    # Not every tick sets the text anew (turning continues).
                    if "Fahrschutz" not in self._text:
                        self._text = f"{self._text} (Fahrschutz: {verdict.reason})"
                    if verdict.share == 0.0:
                        blocked_since = blocked_since or now
                        if now - blocked_since > config.GUARD_STUCK_S:
                            self._watchdog.stop()
                            # Do not give up while mapping: the mapping drive
                            # picks another spot.
                            if occ_map and self._mapping_drive.report_obstacle(
                                f"Fahrschutz: {verdict.reason}"
                            ):
                                blocked_since = None
                                await asyncio.sleep(tick)
                                continue
                            self._reason = f"kommt nicht weiter -- {verdict.reason}"
                            log.info("Exploration ended -- %s", self._reason)
                            return
                    else:
                        blocked_since = None
                else:
                    blocked_since = None
                await asyncio.sleep(tick)

        except asyncio.CancelledError:
            # Always stop, no matter why it was cancelled.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler während der Fahrt"
            log.exception("Exploration aborted")
        finally:
            if self._people is not None and self._mode != MAP:
                self._people.activate(False)
            if self._depth is not None:
                self._depth.activate("erkunder", False)

    def _step(self, now: float) -> DriveCommand | None:
        """One tick: look at the situation, return a drive command. None = end."""
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now
        command = self._decide(now, dt)
        if command is not None:
            self._prev_command = command
        return command

    def _decide(self, now: float, dt: float) -> DriveCommand | None:
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)

        if self._reason is not None:  # gave up in the last tick
            return None
        occ_map = self._mode == MAP
        at_most = config.MAPPING_DRIVE_MAX_S if occ_map else config.EXPLORE_TIMEOUT_S
        if at_most > 0 and now - self._started > at_most:
            self._reason = f"Zeit abgelaufen ({at_most:.0f} s)"
            return None

        # Count the turn before anything else decides: a stop tick (lidar
        # dropout) still belongs to the ongoing turn.
        turn_rate = self._measured_turn_rate()
        if self._state == TURNING and turn_rate is not None:
            self._turned += math.degrees(turn_rate) * dt
        # Own pose for the short-term memory -- every tick, even without a new revolution.
        self._memory.advance(dt)
        # What actually went to the motors last, for the progress watch.
        self._progress.update(now)

        # Without a fresh lidar revolution it does not drive. Short dropouts
        # (USB reset) are waited out, longer ones end the drive.
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or now - scan.timestamp > config.EXPLORE_SCAN_MAX_AGE_S:
            if self._no_lidar_since is None:
                self._no_lidar_since = now
                log.warning("Exploration: no fresh lidar data -- stopping")
            # Longer while mapping: the A1 disconnects and reconnects on voltage
            # dips, a long mapping drive should not end because of that.
            wait_s = config.MAPPING_DRIVE_LIDAR_WAIT_S if occ_map else config.EXPLORE_LIDAR_WAIT_S
            if now - self._no_lidar_since > wait_s:
                self._reason = "Lidar liefert keine Daten mehr"
                return None
            # An ongoing turn or unstick stays remembered, only the display
            # changes -- otherwise it would start over afterwards.
            self._text = "keine frischen Lidar-Daten"
            if self._state not in (TURNING, UNSTICKING):
                self._state = NO_LIDAR
            return stop_cmd
        self._no_lidar_since = None

        # Lifted or carried: stop until it stands still again.
        pose = self._memory.pose if self._memory.measuring else None
        if self._lift.check(scan, pose):
            if self._state != LIFTED:
                self._lifted()
            # Being carried is not being stuck -- observe anew afterwards.
            self._progress.reset()
            return stop_cmd
        if self._state == LIFTED:
            self._next_switch = now + self._wander_time()
            self._set(DRIVING, "wieder abgesetzt")

        # The raw revolution for everything that means "now" (getting stuck);
        # the revolution plus memory for avoidance -- there, what just slipped
        # out of the field of view counts too.
        raw = scan.points()
        points = self._memory.points(scan)
        # Plus what only the depth camera saw (chair legs, table edges below
        # the lidar plane), and spots where it already got stuck. That way
        # avoidance and the mapping drive plan around them right away.
        if self._obstacles is not None:
            self._obstacles.update(now)
            extra = self._obstacles.points()
            if len(extra):
                import numpy as np

                points = np.concatenate((np.asarray(points, dtype=float).reshape(-1, 2), extra))
        width = config.EXPLORE_HALF_WIDTH_M
        front = free_in_direction(points, 0.0, width)
        self._free_ahead = front

        if self._state == UNSTICKING:
            return self._unstick(now, points, front)

        # If something comes very close, do not move at first -- not even
        # turn: the robot's corners swing outwards when turning. Only from
        # three points on: a single stray ray should not hold it. If it does
        # not go away by itself, it pushes itself free instead of waiting forever.
        nearby = sorted(r for r in scan.ranges if r < config.EXPLORE_NEAR_M)
        if len(nearby) >= 3:
            if self._near_since is None:
                self._near_since = now
            if now - self._near_since >= config.EXPLORE_NEAR_WAIT_S:
                self._near_since = None
                return self._begin_unstick(now, points, f"etwas sehr nah ({nearby[0]:.2f} m)")
            # Free driving interrupted: the recovery time starts over afterwards.
            self._free_since = None
            self._set(WAITING, f"etwas sehr nah ({nearby[0]:.2f} m) -- warte")
            return stop_cmd
        self._near_since = None

        if self._state == WAITING:
            self._set(DRIVING, "Weg wieder frei")

        # Wheels turning, but according to the lidar no progress: something
        # nobody sees is there. Remember the spot so that it does not drive
        # into it again right away, and push free away from it -- in every mode.
        stuck = self._progress.check(now, scan)
        if stuck is not None:
            self._set_phantom(stuck, now)
            return self._begin_unstick(now, points, stuck.cause, away_from=stuck.direction_deg)

        if occ_map:
            # While mapping there is no "straight ahead" with recovery time; so
            # the unstick count restarts when none was needed for a while.
            if self._unsticks and now - self._last_unstick >= config.PROGRESS_RECOVERED_S:
                self._unsticks = 0
            command = self._mapping_drive.step(now, scan, points)
            self._set(MAPPED, self._mapping_drive.display())
            if command is None:
                self._reason = f"Kartierfahrt fertig -- {self._mapping_drive.reason}"
            return command

        if self._state == TURNING:
            return self._turn(now, scan, points, front)

        if self._state == WAITING_FOR_HUMAN:
            if self._person_ahead(scan) is None:
                self._set(DRIVING, "Weg wieder frei")
            elif now >= self._wait_until:
                # The person stays put -- then go around.
                return self._turn_around(now, points, "Mensch bleibt stehen, weiche aus")
            else:
                return stop_cmd

        # --- Drive straight ---
        human = self._person_ahead(scan)
        if human is not None:
            self._people_seen += 1
            self._wait_until = now + config.EXPLORE_PERSON_WAIT_S
            self._set(WAITING_FOR_HUMAN, f"Mensch in {human:.2f} m -- warte kurz")
            return stop_cmd

        if front < config.EXPLORE_STOP_M:
            return self._turn_around(now, points, f"Hindernis in {front:.2f} m")

        stuck = self._is_stuck(now, dt, raw)
        if stuck is not None:
            # Forward apparently does not work -- so sideways or back.
            return self._begin_unstick(now, points, stuck, forward=False)

        # Not the random switch in the middle of a door or narrow passage:
        # get through first, otherwise it turns back exactly where it wanted to go.
        narrow = min(scan.ranges, default=config.OBSTACLE_RANGE_M) < config.AVOID_NARROW_M
        if now >= self._next_switch and not narrow:
            return self._turn_around(now, points, "zufälliger Richtungswechsel", randomly=True)

        # Driven freely for a while: failed attempts and unsticks start over.
        # Not immediately -- otherwise every tick against an invisible
        # obstacle would reset the counter before stuck detection kicks in.
        if self._free_since is None:
            self._free_since = now
        elif now - self._free_since >= config.EXPLORE_FREE_RESET_S:
            self._failed_attempts = 0
            self._unsticks = 0
        return self._straight(now, points, front)

    def _straight(self, now: float, points, front: float) -> DriveCommand:
        """Drive the best safe arc; if none fits, turn around."""
        # Slow down in front of obstacles, but not below ~1/3: the motors no
        # longer translate very small values into motion on the floor.
        span = config.EXPLORE_SLOW_M - config.EXPLORE_STOP_M
        share = min(1.0, max(0.35, (front - config.EXPLORE_STOP_M) / span))
        vx_max = config.EXPLORE_SPEED * share

        # Hold the heading from the last start: after going around it steers
        # back instead of continuing to turn in an arc. Without a measured
        # turn rate there is no heading -- then only steering little counts.
        heading = self._memory.heading
        if heading is None:
            self._goal_heading = None
        elif self._goal_heading is None:
            self._goal_heading = heading
        error = 0.0 if heading is None else avoidance._normalize_angle(self._goal_heading - heading)

        choice = avoidance.plan_arc(points, vx_max, error, self._prev_command.omega)
        if choice is None:
            return self._turn_around(now, points, "kein sicherer Bogen")
        (vx, vy, omega), _clearance = choice
        straight = omega == 0.0 and vy == 0.0
        if front >= config.EXPLORE_SLOW_M:
            text = "fährt" if straight else "fährt Bogen"
        else:
            text = f"bremst, frei {front:.2f} m" if straight else f"umfährt, frei {front:.2f} m"
        self._set(DRIVING, text)
        return DriveCommand(vx, vy, omega)

    # --- Turning -----------------------------------------------------------------

    def _turn_around(self, now: float, points, cause: str, randomly: bool = False) -> DriveCommand:
        """Choose a new direction and start the turn towards it."""
        self._free_since = None
        direction, free = self._choose_direction(points, randomly)
        if not randomly:
            self._avoidances += 1
        if free < config.EXPLORE_GO_M:
            self._failed_attempts += 1
            if self._failed_attempts >= _MAX_FAILED_ATTEMPTS:
                # Turning achieves nothing any more -- push free, then start anew.
                self._failed_attempts = 0
                return self._begin_unstick(now, points, "nirgends genug Platz zum Drehen")
        self._turn_direction = direction
        duration = abs(math.radians(direction)) / config.EXPLORE_TURN_SPEED
        self._turn_start = now
        self._turned = 0.0
        self._turn_done = None
        self._turn_measured = self._measured_turn_rate() is not None
        if self._turn_measured:
            # The angle decides; the time only catches the case that it does
            # not turn (blocked) or the measurement fails.
            self._turn_end = now + max(0.3, duration * config.EXPLORE_TURN_FACTOR)
            self._turn_limit = now + max(1.0, duration * config.EXPLORE_TURN_TIME_FACTOR)
        else:
            self._turn_end = now + max(0.3, duration * config.EXPLORE_TURN_FACTOR)
            self._turn_limit = self._turn_end
        self._set(TURNING, f"{cause} -- drehe {direction:+.0f}°")
        log.info("Exploration: %s -- turning %+.0f° (%.2f m free there)", cause, direction, free)
        return self._turn_command()

    def _turn(self, now: float, scan, points, front: float) -> DriveCommand | None:
        if self._turn_done is None:
            if self._turn_measured and self._measured_turn_rate() is not None:
                # Stop shortly before the target: latency and the braking ramp
                # turn it a bit further.
                rest = abs(self._turn_direction) - abs(self._turned)
                done = rest <= config.EXPLORE_TURN_LEAD_DEG or now >= self._turn_limit
            else:
                # Measurement missing (or dropped out during the turn): by time.
                done = now >= self._turn_end
            if not done:
                return self._turn_command()
            self._turn_done = now
            if abs(self._turned) > 0.0:
                log.info(
                    "Exploration: turn finished, %+.0f° of %+.0f° measured",
                    self._turned, self._turn_direction,
                )

        # Only decide with a revolution that began after the turn -- the
        # current one partly still shows the old direction.
        if scan.timestamp - scan.duration < self._turn_done + config.EXPLORE_COAST_S:
            self._text = "prüft den Weg"
            return DriveCommand(0.0, 0.0, 0.0)

        if front >= config.EXPLORE_GO_M:
            self._failed_attempts = 0
            self._next_switch = now + self._wander_time()
            self._goal_heading = None  # new heading: the one after the turn
            self._set(DRIVING, "Weg frei")
            return self._straight(now, points, front)
        # Turned, but still too tight in front (turn came out too short or the
        # world has moved) -- choose again.
        self._failed_attempts += 1
        if self._failed_attempts >= _MAX_FAILED_ATTEMPTS:
            self._failed_attempts = 0
            return self._begin_unstick(now, points, f"nach {_MAX_FAILED_ATTEMPTS} Drehungen kein Weg")
        return self._turn_around(now, points, f"vorn nur {front:.2f} m frei")

    def _turn_command(self) -> DriveCommand:
        direction = 1.0 if self._turn_direction >= 0 else -1.0
        return DriveCommand(0.0, 0.0, direction * config.EXPLORE_TURN_SPEED)

    def _measured_turn_rate(self) -> float | None:
        """Turn rate measured by the board (rad/s), None without a real measurement.

        Without a measurement the odometry computes with the command --
        integrating an angle from that would mean turning by time again,
        just hidden.
        """
        if self._odometry is None or not self._odometry.actually_measuring:
            return None
        return self._odometry.velocity()[2]

    def _choose_direction(self, points, randomly: bool) -> tuple[float, float]:
        """Freest direction -- a random one among the good ones.

        Random instead of always the very freest, otherwise in a long
        corridor it oscillates forever between the two ends.
        """
        width = config.EXPLORE_HALF_WIDTH_M
        # When avoiding, straight ahead is blocked anyway; for the random
        # switch it should be a real turnaround.
        limit = 45 if randomly else 20
        # Good directions come from the free areas: the middle of a door
        # instead of narrowly past the frame.
        far = [
            k for k in avoidance.gaps(points, width, config.EXPLORE_GO_M + 0.5, config.OBSTACLE_RANGE_M)
            if abs(k[0]) >= limit
        ]
        if far:
            return random.choice(far)
        candidates = [
            (r, min(free_in_direction(points, r, width), config.OBSTACLE_RANGE_M))
            for r in _DIRECTIONS if abs(r) >= limit
        ]
        return max(candidates, key=lambda k: k[1])

    # --- Unsticking --------------------------------------------------------------

    def _begin_unstick(
        self, now: float, points, cause: str, forward: bool = True,
        away_from: float | None = None,
    ) -> DriveCommand | None:
        """Push in the freest direction without turning.

        ``forward=False`` excludes the front half: if it is stuck although
        the front looks free, there is something the lidar does not see.
        ``away_from`` (degrees) does the same for any direction -- the one in
        which, according to the progress watch, it made no progress.
        """
        self._free_since = None
        self._stuck_start = None
        self._stuck_still_s = 0.0
        self._last_unstick = now
        self._progress.reset()
        if self._unsticks >= config.EXPLORE_UNSTICK_MAX:
            if self._mode == MAP and self._mapping_drive.report_obstacle(
                f"festgefahren ({cause})"
            ):
                # The mapping drive picks another goal; the attempts count
                # afresh for the new one.
                self._unsticks = 0
                return DriveCommand(0.0, 0.0, 0.0)
            self._reason = f"festgefahren -- {self._unsticks} Befreiungsversuche ohne Erfolg"
            log.warning("Exploration: %s (%s)", self._reason, cause)
            return None
        self._unsticks += 1
        self._unstuck_total += 1

        width = config.EXPLORE_HALF_WIDTH_M
        candidates = [
            (r, free_in_direction(points, r, width))
            for r in _UNSTICK_DIRECTIONS
            if (forward or abs(r) >= 90)
            and (away_from is None or abs((r - away_from + 180.0) % 360.0 - 180.0) >= 90)
        ]
        r, free = max(candidates, key=lambda k: (k[1], -abs(k[0])))
        if free < config.EXPLORE_UNSTICK_FREE_M:
            # No room anywhere: stay put and try again after the wait time
            # (counts as an attempt) -- maybe the person surrounding it will
            # go away.
            self._near_since = now
            self._set(WAITING, f"{cause} -- kein Platz zum Befreien, warte")
            log.info("Exploration: %s -- no room to unstick (best %.2f m)", cause, free)
            return DriveCommand(0.0, 0.0, 0.0)

        self._unstick_direction = float(r)
        self._unstick_end = now + config.EXPLORE_UNSTICK_S
        self._set(UNSTICKING, f"{cause} -- schiebt sich frei ({r:+.0f}°)")
        log.info("Exploration: %s -- unsticking towards %+.0f° (%.2f m free there)", cause, r, free)
        return self._unstick_command()

    def _unstick(self, now: float, points, front: float) -> DriveCommand | None:
        free = free_in_direction(points, self._unstick_direction, config.EXPLORE_HALF_WIDTH_M)
        if now < self._unstick_end and free >= config.EXPLORE_UNSTICK_STOP_M:
            return self._unstick_command()
        # Time is up or it gets tight in the push direction: decide again.
        # The near check in the next tick catches it if it achieved nothing.
        self._near_since = None
        if front >= config.EXPLORE_GO_M:
            self._next_switch = now + self._wander_time()
            self._goal_heading = None
            self._set(DRIVING, "frei, fährt weiter")
            return DriveCommand(0.0, 0.0, 0.0)
        return self._turn_around(now, points, "nach dem Befreien")

    def _unstick_command(self) -> DriveCommand:
        a = math.radians(self._unstick_direction)
        v = config.EXPLORE_UNSTICK_SPEED
        return DriveCommand(v * math.cos(a), v * math.sin(a), 0.0)

    def _is_stuck(self, now: float, dt: float, points) -> str | None:
        """Cause if it makes no progress despite a drive command, otherwise None.

        Two independent indications, both over ``EXPLORE_STUCK_S``: the
        board reports standstill, or the lidar image all around stays the
        same. The limit is below the 3 s after which the odometry discards a
        measurement reporting standstill entirely.
        """
        # Only count within one continuous straight drive -- after a turn or
        # wait the observation starts anew.
        if now - self._stuck_tick > 0.3:
            self._stuck_start = None
            self._stuck_still_s = 0.0
        self._stuck_tick = now

        b = self._prev_command
        if b.vx < config.EXPLORE_STUCK_MIN_VX:
            self._stuck_start = None
            self._stuck_still_s = 0.0
            return None

        # Indication 1: board velocity (only if really measured).
        if self._odometry is not None and self._odometry.actually_measuring:
            vx, vy, _ = self._odometry.velocity()
            if math.hypot(vx, vy) < config.EXPLORE_STUCK_V_MS:
                self._stuck_still_s += dt
            else:
                self._stuck_still_s = 0.0
            if self._stuck_still_s >= config.EXPLORE_STUCK_S:
                return "fährt nicht trotz Fahrbefehl"

        # Indication 2: lidar image all around. If it moves, the distance
        # changes in several directions; if it is stuck, only within noise.
        # If something moves around it, it looks like driving -- then it
        # rather reports nothing (the board and the near check remain).
        # All measurements, not only the near ones: a ray that moves out of
        # range or newly appears while driving is movement too. With a limit,
        # in a large room exactly the rays in the driving direction would
        # drop out, and the side walls do not change.
        image = {}
        for r in _STUCK_DIRECTIONS:
            d = free_in_direction(points, r, _STUCK_RAY_M)
            if math.isfinite(d):
                image[r] = d
        if len(image) < 4:
            # Too little to compare (open terrain).
            self._stuck_start = None
            return None
        if self._stuck_start is not None:
            old = self._stuck_snapshot
            changed = sum(
                1 for r in _STUCK_DIRECTIONS
                if (r in image) != (r in old)
                or (r in image and abs(image[r] - old[r]) > max(config.EXPLORE_STUCK_DIST_M, 0.02 * image[r]))
            )
            if changed < 2:
                if now - self._stuck_start >= config.EXPLORE_STUCK_S:
                    return "kommt nicht voran (Lidar-Bild unverändert)"
                return None
        self._stuck_start = now
        self._stuck_snapshot = image
        return None

    def _set_phantom(self, stuck: Stuck, now: float) -> None:
        """Imaginary obstacle just in front of the footprint, where it could not get to.

        A row of points across the driving direction, as wide as the robot:
        avoidance, the mapping drive and the drive guard then keep their
        distance until it expires. When turning there is no direction --
        then only unstick.
        """
        if self._obstacles is None or stuck.direction_deg is None:
            return
        import numpy as np

        a = math.radians(stuck.direction_deg)
        c, s = math.cos(a), math.sin(a)
        # Distance to the edge of the footprint in this direction.
        lengthwise = config.ROBOT_FRONT_M if c >= 0 else config.ROBOT_REAR_M
        lateral = config.ROBOT_HALF_WIDTH_M
        margin = min(lengthwise / abs(c) if abs(c) > 1e-6 else math.inf,
                   lateral / abs(s) if abs(s) > 1e-6 else math.inf)
        d = margin + 0.03
        shift = np.arange(-lateral, lateral + 1e-6, 0.03)
        points = np.column_stack((d * c - shift * s, d * s + shift * c))
        self._obstacles.phantom(points, now)
        log.info("Exploration: phantom obstacle remembered at %+.0f°", stuck.direction_deg)

    # --- Lifted ------------------------------------------------------------------

    def _lifted(self) -> None:
        """Forget everything that applied at the old place -- then it starts anew."""
        self._memory.clear()
        if self._obstacles is not None:
            self._obstacles.clear()
        self._goal_heading = None
        self._near_since = None
        self._free_since = None
        self._stuck_start = None
        self._stuck_still_s = 0.0
        self._failed_attempts = 0
        self._unsticks = 0
        self._turn_done = None
        self._turned = 0.0
        self._set(LIFTED, "hochgehoben -- wartet, bis er wieder steht")
        log.info("Exploration: lifted (%.0f %% of the lidar image changed)", 100 * self._lift.share)

    # --- Helpers -----------------------------------------------------------------

    def _person_ahead(self, scan) -> float | None:
        """Distance to the person in front of the robot, otherwise None."""
        people = self._people.angle_ranges() if self._people is not None else []
        for h in self._detection.current(scan, people):
            if h.category == hd.HUMAN and h.distance < 1.0 and abs(h.angle) < 35.0:
                return h.distance
        return None

    def _set(self, state: str, text: str) -> None:
        self._state = state
        self._text = text

    @staticmethod
    def _wander_time() -> float:
        return random.uniform(config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S)
