"""Automatic return to the start point.

Takes advantage of the X3's mecanum wheels: it can drive in any direction
without turning first. That makes homing about as simple as it gets --
compute the direction to the start point, drive in that direction, done.
Turning is the biggest source of error in dead reckoning, and that is
exactly what is avoided here.

**Avoidance (03.11.2026, F3):** If something is on the straight line, it
slides past it diagonally until the path is free again
(``avoidance.choose_direction``). Without a map, so around single obstacles,
not through mazes. If there is no room anywhere for ``HOME_DETOUR_WAIT_S``,
it stops and reports it. If it is lifted, it aborts: dead reckoning knows
nothing about being carried, so the direction home is no longer right.
Hence still the leisurely speed and the time limit.

**With a map (04.11.2026, E12):** If a map is loaded and the pose in it is
confident (``map_localization.py``), it no longer drives straight but along
a planned path (``path_planning.py``) -- through doors, around corners,
into other rooms. Home is then the origin of the map. The same class also
drives to any other point on the map (``start(target=...)``). It stays one
class so that every existing abort (emergency stop, manual control,
connection lost, another drive starts) also ends the goal drive. If the
path gets blocked, the spot counts as a wall for a while and it replans
around it.

**Odometry goal (05.11.2026, health check):** ``start_to`` drives the same
straight route to another point in the odometry frame, e.g. to a reported
person -- without a map. Because the person there is the obstacle itself,
"blocked, but already close enough" may count as arrived.

**Way back (E14):** ``kind="zurueck"`` drives to an anchor (mission start,
home) along the breadcrumb trail (``corridor``); if no path is found, it
drives the trail backwards (``fallback_points``). When the pose gets
unsure on a map drive it no longer gives up after a few seconds: it stops,
searches near the trail, turns a little after a while so the lidar sees
more, and drives on after a few good matches -- after
``NAV_RELOCALIZE_WAIT_S`` it gives up and asks to be shown where it is.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from typing import TYPE_CHECKING

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import avoidance
from . import path_follower as pf
from .odometry import Odometry
from .path_planning import BackgroundPlanner, PlannedPath

if TYPE_CHECKING:  # only for the type hint -- avoids a circular import
    from .map_localization import MapLocalization
    from .localizer import Localizer

log = logging.getLogger(__name__)


class Homing:
    """Steers the robot back to the start point -- or, with a map, to any goal."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        odometry: Odometry,
        localizer: "Localizer | None" = None,
        localization: "MapLocalization | None" = None,
        planner: BackgroundPlanner | None = None,
    ) -> None:
        self._watchdog = watchdog
        self._odometry = odometry
        self._localizer = localizer
        self._localization = localization
        self._planner = planner if planner is not None else (
            BackgroundPlanner() if localization is not None else None
        )
        self._task: asyncio.Task[None] | None = None
        self._reason: str | None = None
        # Drive by map: goal (None = home), path, follower.
        self._target: tuple[float, float] | None = None
        self._kind: str | None = None             # "zurueck" or None (heim/ziel)
        self._corridor = None                     # breadcrumb trail for the planner
        self._fallback: list[tuple[float, float]] = []
        self._waiting: str | None = None
        self._guessed = False                     # path through unknown, looked at on site
        self._by_map = False
        self._follower = pf.PathFollower()
        # speed of the running drive, None: the configured default
        self._speed: float | None = None
        self._path: list[tuple[float, float]] = []
        self._blocks: list[tuple[float, float, float]] = []  # x, y, valid until
        # Avoidance (F3): short-term memory, lift detection, last chosen side
        # (against oscillating in front of wide obstacles).
        self._memory = avoidance.ShortTermMemory(odometry)
        self._lift = avoidance.LiftDetector()
        self._page = 0.0
        self._blocked_since: float | None = None
        # straight drive goal in the odometry frame; (0, 0) is home
        self._goal = (0.0, 0.0)
        self._goal_name = "Startpunkt"
        self._tolerance: float | None = None
        self._blocked_enough_m = 0.0
        self._arrived = False

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def last_reason(self) -> str | None:
        """Why the last homing drive ended -- for the display in the app."""
        return self._reason

    @property
    def arrived(self) -> bool:
        """Whether the last drive reached its goal (not aborted)."""
        return self._arrived

    @property
    def _home(self) -> bool:
        return self._target is None and self._goal == (0.0, 0.0)

    @property
    def label(self) -> str:
        """For messages: "Heimfahrt", "Rückfahrt" or "Fahrt zum Ziel"."""
        if self._kind == "zurueck":
            return "Rückfahrt"
        return "Heimfahrt" if self._home else "Fahrt zum Ziel"

    @property
    def by_map(self) -> bool:
        """Is a drive by map running right now?"""
        return self.active and self._by_map

    @property
    def map_ready(self) -> bool:
        """Pose in a map known -- then it drives by map."""
        return self._localization is not None and self._planner is not None and self._localization.confident

    @property
    def occ_map(self):
        """The map it localises in while the pose is known, otherwise None."""
        return getattr(self._localization, "occ_map", None) if self.map_ready else None

    def map_pose(self) -> tuple[float, float, float] | None:
        """Pose in the map (x, y, radians) while it is known, otherwise None."""
        return self._localization.pose() if self.map_ready else None

    def start(self, target: tuple[float, float] | None = None, angle: float | None = None,
              kind: str | None = None, corridor=None, fallback_points=None,
              blocked_enough_m: float = 0.0, speed: float | None = None) -> bool:
        """Begin homing or the drive to ``target`` (map frame, m).

        ``angle`` (radians): align like this at the goal. ``kind="zurueck"``:
        the way back along ``corridor`` (breadcrumbs), ``fallback_points``
        if no path is found. False if a drive is already running or a goal
        is requested without a confident pose in the map (the reason is then
        in ``last_reason``). ``blocked_enough_m`` as in ``start_to``: blocked
        this close to the goal counts as arrived (person on the floor).
        ``speed`` (m/s) replaces the normal drive speed for this drive.
        """
        if self.active:
            return False
        if target is not None and not self.map_ready:
            self._reason = "ohne sichere Lage in einer Karte geht keine Fahrt zum Ziel"
            return False
        self._reason = None
        self._arrived = False
        self._set_goal(0.0, 0.0, None, "Startpunkt", blocked_enough_m)
        self._target = None if target is None else (float(target[0]), float(target[1]))
        self._kind = kind
        self._corridor = corridor
        self._fallback = list(fallback_points or [])
        self._waiting = None
        self._path = []
        self._set_speed(speed)
        self._by_map = self.map_ready
        if self._by_map:
            if target is None:
                start = self._localization.start
                target, angle = (start[0], start[1]), start[2]
            self._task = asyncio.create_task(self._drive_by_map((target[0], target[1]), angle))
        else:
            self._task = asyncio.create_task(self._drive())
        return True

    def start_to(
        self,
        x: float,
        y: float,
        tolerance: float | None = None,
        name: str = "Startpunkt",
        blocked_enough_m: float = 0.0,
        speed: float | None = None,
    ) -> bool:
        """Drive straight to a point in the odometry frame. False if a drive is running.

        No map needed: dead reckoning like the homing drive without a map.
        ``tolerance`` replaces ``HOME_TOLERANCE_M``. If ``blocked_enough_m``
        is greater than zero, the drive counts as arrived when something is
        in the way within that distance of the goal -- meant for goals that
        are an obstacle themselves (a person on the floor). ``speed`` (m/s)
        replaces ``HOME_SPEED`` for this drive.
        """
        if self.active:
            return False
        self._reason = None
        self._arrived = False
        self._target = None
        self._kind = None
        self._corridor = None
        self._fallback = []
        self._waiting = None
        self._path = []
        self._set_speed(speed)
        self._by_map = False
        self._set_goal(x, y, tolerance, name, blocked_enough_m)
        self._task = asyncio.create_task(self._drive())
        return True

    def _set_speed(self, speed: float | None) -> None:
        self._speed = None if speed is None else float(speed)
        self._follower.speed = config.NAV_DRIVE_SPEED if speed is None else float(speed)

    def _set_goal(self, x: float, y: float, tolerance: float | None, name: str,
                  blocked_enough_m: float) -> None:
        self._goal = (float(x), float(y))
        self._goal_name = name
        self._tolerance = tolerance
        self._blocked_enough_m = max(0.0, float(blocked_enough_m))

    def state(self) -> dict:
        """For state.karte and the /map page."""
        active = self.active
        return {
            "aktiv": active,
            "art": self._kind or ("heim" if self._home else "ziel"),
            "nach_karte": self._by_map if active else None,
            "ziel": list(self._target) if self._target is not None else None,
            "weg": [[round(p[0], 2), round(p[1], 2)] for p in self._path] if active else [],
            "rest_m": round(self._follower.rest_m, 2) if active and self._by_map else None,
            "grund": None if active else self._reason,
            "wartet": self._waiting if active else None,
            "vermutet": self._guessed if active else False,
        }

    def cancel(self, reason: str = "abgebrochen") -> None:
        """End immediately. Also to be called when the human intervenes."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._reason = reason
            log.info("Homing %s", reason)
        self._task = None

    async def _drive(self) -> None:
        tick = 1.0 / max(1, config.ODOMETRY_HZ)
        deadline = time.monotonic() + config.HOME_TIMEOUT_S
        self._memory.clear()
        self._lift.reset()
        self._page = 0.0
        self._blocked_since = None
        goal_x, goal_y = self._goal
        tolerance = config.HOME_TOLERANCE_M if self._tolerance is None else self._tolerance
        name = self._goal_name

        start_x, start_y, _ = self._odometry.pose()
        dist = math.hypot(start_x - goal_x, start_y - goal_y)
        log.info("%s started -- %.2f m to drive", self.label, dist)

        if not self._odometry.actually_measuring:
            log.warning(
                "No motion measurement available -- the position comes only "
                "from the drive commands and is correspondingly inaccurate"
            )

        try:
            while True:
                x, y, angle_deg = self._odometry.pose()
                # relative to the goal from here on, _step always drives to (0, 0)
                x, y = x - goal_x, y - goal_y
                distance = math.hypot(x, y)

                if distance <= tolerance:
                    self._watchdog.stop()
                    self._arrived = True
                    self._reason = f"angekommen ({distance:.2f} m Restabweichung)"
                    log.info("Homing finished -- %s", self._reason)
                    return

                if time.monotonic() > deadline:
                    self._watchdog.stop()
                    self._reason = f"Zeit abgelaufen, noch {distance:.2f} m entfernt"
                    log.warning("Homing aborted -- %s", self._reason)
                    return

                command = self._step(x, y, angle_deg, distance)
                self._memory.advance(tick)
                scan = self._localizer.scan() if self._localizer is not None else None

                if scan is not None:
                    pose = self._memory.pose if self._memory.measuring else None
                    if self._lift.check(scan, pose):
                        self._watchdog.stop()
                        self._reason = "hochgehoben -- Position stimmt nicht mehr"
                        log.warning("Homing aborted -- %s", self._reason)
                        return
                    free = self._detour(command, distance, scan)
                    if free is None and distance <= self._blocked_enough_m:
                        # the goal itself is in the way (person on the floor), closer is impossible
                        self._finish_blocked(distance)
                        return
                    if free is None:
                        # Wait briefly: a person walking past should not end
                        # the homing drive.
                        now = time.monotonic()
                        if self._blocked_since is None:
                            self._blocked_since = now
                        if now - self._blocked_since > config.HOME_DETOUR_WAIT_S:
                            self._watchdog.stop()
                            blocked = self._obstacle(command)
                            where = f"Hindernis in {blocked:.2f} m" if blocked is not None else "kein Weg frei"
                            self._reason = f"{where}, noch {distance:.2f} m bis zum {name}"
                            log.warning("Homing stopped -- %s", self._reason)
                            return
                        self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), source="autonom")
                        await asyncio.sleep(tick)
                        continue
                    self._blocked_since = None
                    self._watchdog.submit(free, source="autonom")
                    await asyncio.sleep(tick)
                    continue

                # Without a fresh scan: as before, only check and stop.
                # Only just before driving off, not earlier: the
                # driving direction follows from the command.
                blocked = self._obstacle(command)
                if blocked is not None:
                    self._watchdog.stop()
                    if distance <= self._blocked_enough_m:
                        self._finish_blocked(distance)
                        return
                    self._reason = (
                        f"Hindernis in {blocked:.2f} m, noch {distance:.2f} m "
                        f"bis zum {name}"
                    )
                    log.warning("Homing stopped -- %s", self._reason)
                    return

                self._watchdog.submit(command, source="autonom")
                await asyncio.sleep(tick)

        except asyncio.CancelledError:
            # Always stop, no matter why it was cancelled.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler während der Fahrt"
            log.exception("Homing aborted")

    # --- With a map ------------------------------------------------------------------

    async def _drive_by_map(self, target: tuple[float, float], angle: float | None) -> None:
        tick = 1.0 / max(1, config.ODOMETRY_HZ)
        self._memory.clear()
        self._lift.reset()
        replans = 0
        uncertain_since: float | None = None
        resume_left = 0
        last_nr = None
        blocked_since: float | None = None
        # look-ahead: something on the route the map does not know (a closed
        # door), seen at the same spot on two checks in a row
        check_time = 0.0
        candidate: tuple[float, float] | None = None
        name = self.label
        try:
            path = await self._plan(target) or self._fallback_path()
            if path is None:
                self._shutdown("kein Weg durch die bekannte Karte", warning=True)
                return
            log.info("%s by map -- %.1f m path, %d waypoints%s", name, path.length,
                     len(path.points), " (guessed, looking on site)" if path.guessed else "")
            deadline = time.monotonic() + max(config.NAV_DRIVE_MIN_S,
                                           path.length * config.NAV_DRIVE_S_PER_M)
            self._set_path(path, angle)
            # a guessed path is replanned while driving: the map grows on site
            update = None
            update_time = time.monotonic()

            while True:
                now = time.monotonic()
                if now > deadline:
                    self._shutdown(f"Zeit abgelaufen, noch {self._follower.rest_m:.2f} m", warning=True)
                    return

                # never on without a confident pose: stop, search, turn a
                # little, and only trust it again after a few good matches
                pose = self._localization.pose()
                if pose is None or not self._localization.confident:
                    if uncertain_since is None:
                        uncertain_since = now
                        self._localization.relocalize()
                        log.info("%s waits for the pose in the map", name)
                    waited = now - uncertain_since
                    if waited > config.NAV_RELOCALIZE_WAIT_S:
                        self._shutdown("Lage in der Karte verloren -- auf der Karte mit "
                                       "„Hier stehe ich“ zeigen, wo er steht", warning=True)
                        return
                    self._waiting = "wartet auf Ortung"
                    resume_left = config.NAV_RESUME_CONFIRM_N
                    deadline += tick   # waiting does not count as driving time
                    self._watchdog.submit(self._search_turn(waited), source="autonom")
                    await asyncio.sleep(tick)
                    continue
                if uncertain_since is not None:
                    track = self._localization.last_track
                    if track is not None and track.nr != last_nr:
                        last_nr = track.nr
                        good = track.accepted and track.kind == "track"
                        resume_left = resume_left - 1 if good else config.NAV_RESUME_CONFIRM_N
                    if resume_left > 0:
                        deadline += tick
                        self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), source="autonom")
                        await asyncio.sleep(tick)
                        continue
                    uncertain_since = None
                    self._waiting = None
                    log.info("%s has the pose again -- planning anew", name)
                    self._watchdog.stop()
                    path = await self._plan(target) or self._fallback_path()
                    if path is None:
                        self._shutdown("kein Weg durch die bekannte Karte", warning=True)
                        return
                    self._set_path(path, angle)
                    continue

                if self._guessed:
                    if update is not None and update.done():
                        fresh, update = update.result(), None
                        if fresh is None:
                            self._shutdown("kein Weg -- vor Ort ist dort kein Durchgang",
                                           warning=True)
                            return
                        if not fresh.guessed:
                            log.info("%s: the map now shows a sure path (%.1f m)", name,
                                     fresh.length)
                        self._set_path(fresh, angle)
                        deadline = max(deadline, now + fresh.length * config.NAV_DRIVE_S_PER_M)
                    elif update is None and now - update_time >= config.NAV_GUESS_REPLAN_S:
                        update_time = now
                        update = self._plan_future(target)

                self._memory.advance(tick)
                scan = self._localizer.scan() if self._localizer is not None else None
                if scan is None:
                    # It does not drive blind; the lidar usually comes back right away.
                    self._watchdog.submit(DriveCommand(0.0, 0.0, 0.0), source="autonom")
                    await asyncio.sleep(tick)
                    continue
                memory_pose = self._memory.pose if self._memory.measuring else None
                if self._lift.check(scan, memory_pose):
                    self._shutdown("hochgehoben -- Lage wird neu gesucht", warning=True)
                    return

                points = self._memory.points(scan)
                if now - check_time >= config.NAV_SCAN_CHECK_S:
                    check_time = now
                    seen = self._route_obstacle(pose, scan)
                    if seen is None:
                        candidate = None
                    elif candidate is None or math.hypot(seen[0] - candidate[0],
                                                         seen[1] - candidate[1]) > config.NAV_BLOCK_RADIUS_M:
                        candidate = seen
                    else:
                        # stays there: not a person walking by but a closed door or
                        # furniture; the map only learns it later, plan around now
                        candidate = None
                        replans += 1
                        if replans > config.NAV_MAX_REPLANS:
                            self._shutdown(
                                f"Weg bleibt versperrt, noch {self._follower.rest_m:.2f} m",
                                warning=True,
                            )
                            return
                        self._blocks.append((seen[0], seen[1], now + config.NAV_BLOCK_S))
                        log.info("%s: route blocked ahead at (%.2f, %.2f) -- planning around it",
                                 name, seen[0], seen[1])
                        self._watchdog.stop()
                        path = await self._plan(target)
                        if path is None:
                            self._shutdown("kein Weg mehr frei", warning=True)
                            return
                        log.info("%s: new path %.1f m", name, path.length)
                        self._set_path(path, angle)
                        blocked_since = None
                        continue
                command, state = self._follower.step(pose, points)
                if state == pf.ARRIVED:
                    rest = math.hypot(target[0] - pose[0], target[1] - pose[1])
                    self._arrived = True
                    self._shutdown(f"angekommen ({rest:.2f} m Restabweichung)")
                    return

                # If it stands although it wants to drive (path blocked or the
                # DriveGuard lets nothing through), it plans around it after
                # a while.
                verdict = self._watchdog.last_check
                blocked = (
                    state == pf.BLOCKED
                    or (verdict is not None and verdict.share == 0.0 and not command.is_stop())
                )
                if state == pf.OFF_PATH or blocked:
                    rest = math.hypot(target[0] - pose[0], target[1] - pose[1])
                    if blocked and rest <= self._blocked_enough_m:
                        # the goal itself is in the way (person on the floor)
                        self._finish_blocked(rest)
                        return
                    blocked_since = blocked_since or now
                    if state == pf.OFF_PATH or now - blocked_since > config.NAV_BLOCKED_S:
                        replans += 1
                        if replans > config.NAV_MAX_REPLANS:
                            self._shutdown(
                                f"Weg bleibt versperrt, noch {self._follower.rest_m:.2f} m",
                                warning=True,
                            )
                            return
                        if blocked:
                            self._set_block(pose, points)
                        self._watchdog.stop()
                        path = await self._plan(target) or (None if blocked else self._fallback_path())
                        if path is None:
                            self._shutdown("kein Weg mehr frei", warning=True)
                            return
                        self._set_path(path, angle)
                        blocked_since = None
                        continue
                    if state == pf.BLOCKED:
                        command = DriveCommand(0.0, 0.0, 0.0)
                else:
                    blocked_since = None

                self._watchdog.submit(command, source="autonom")
                await asyncio.sleep(tick)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler während der Fahrt"
            log.exception("%s aborted", name)

    def _finish_blocked(self, distance: float) -> None:
        self._watchdog.stop()
        self._arrived = True
        self._reason = f"angekommen, Weg versperrt ({distance:.2f} m vor dem Ziel)"
        log.info("%s finished -- %s", self.label, self._reason)

    def _search_turn(self, waited: float) -> DriveCommand:
        """While waiting for the pose: after a while turn a little so the lidar sees more."""
        omega = config.NAV_RELOCALIZE_OMEGA
        if (not config.NAV_RELOCALIZE_TURN or omega <= 0.0
                or waited < config.NAV_RELOCALIZE_TURN_AFTER_S):
            return DriveCommand(0.0, 0.0, 0.0)
        phase = math.radians(config.NAV_RELOCALIZE_TURN_DEG) / omega
        t = waited - config.NAV_RELOCALIZE_TURN_AFTER_S
        # left, twice as far right, back to the middle, then hold still
        for sign, length in ((1.0, phase), (-1.0, 2 * phase), (1.0, phase)):
            if t < length:
                return DriveCommand(0.0, 0.0, sign * omega)
            t -= length
        return DriveCommand(0.0, 0.0, 0.0)

    def _fallback_path(self) -> PlannedPath | None:
        """The breadcrumbs backwards, from the crumb nearest to the robot on."""
        if not self._fallback:
            return None
        pose = self._localization.pose()
        points = self._fallback
        if pose is not None:
            nearest = min(range(len(points)),
                          key=lambda i: (points[i][0] - pose[0]) ** 2 + (points[i][1] - pose[1]) ** 2)
            points = points[nearest:]
        length = 0.0
        previous = (pose[0], pose[1]) if pose is not None else points[0]
        for p in points:
            length += math.hypot(p[0] - previous[0], p[1] - previous[1])
            previous = p
        log.info("No path through the map -- driving the trail back (%.1f m)", length)
        return PlannedPath(list(points), length, points[-1])

    def _shutdown(self, reason: str, warning: bool = False) -> None:
        self._waiting = None
        self._guessed = False
        self._watchdog.stop()
        self._reason = reason
        (log.warning if warning else log.info)("%s finished -- %s", self.label, reason)

    def _set_path(self, path, angle: float | None) -> None:
        self._path = list(path.points)
        self._guessed = bool(getattr(path, "guessed", False))
        self._follower.apply(path.points, from_=self._localization.pose(), goal_angle=angle)

    async def _plan(self, target):
        future = self._plan_future(target)
        return None if future is None else await asyncio.wrap_future(future)

    def _plan_future(self, target):
        """Start planning in the background; None without pose or map."""
        pose = self._localization.pose()
        occ_map = self._localization.occ_map
        if pose is None or occ_map is None:
            return None
        now = time.monotonic()
        self._blocks = [s for s in self._blocks if s[2] > now]
        return self._planner.path(occ_map, (pose[0], pose[1]), target,
                                  [(s[0], s[1]) for s in self._blocks], self._corridor)

    def _route_obstacle(self, pose, scan) -> tuple[float, float] | None:
        """Nearest live lidar point on the route ahead (map frame), or None.

        the route keeps NAV_SAFETY_M from every wall of the map, so a point
        closer to it than the robot's half width plus margin is something the
        map does not know. only the fresh scan, not the obstacle memory: a
        remembered point would block the way back
        """
        import numpy as np

        route = [(pose[0], pose[1])] + list(self._follower.remaining)
        if len(route) < 2:
            return None
        pts = np.asarray(scan.points(), dtype=float).reshape(-1, 2)
        if not len(pts):
            return None
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(r >= config.LIDAR_MIN_M) & (r <= config.NAV_SCAN_CHECK_M + 0.5)]
        if not len(pts):
            return None
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        wx = x + c * pts[:, 0] - s * pts[:, 1]
        wy = y + s * pts[:, 0] + c * pts[:, 1]
        width = config.ROBOT_HALF_WIDTH_M + config.NAV_SCAN_CHECK_MARGIN_M
        best = None
        along = 0.0
        for (ax, ay), (bx, by) in zip(route, route[1:]):
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-6:
                continue
            ux, uy = (bx - ax) / seg, (by - ay) / seg
            t = (wx - ax) * ux + (wy - ay) * uy
            d = np.abs(-(wx - ax) * uy + (wy - ay) * ux)
            # skip the robot's own body at the start of the route
            on = (t >= 0) & (t <= seg) & (d < width) & (along + t > config.ROBOT_FRONT_M)
            on &= along + t <= config.NAV_SCAN_CHECK_M
            if on.any():
                i = int(np.argmin(np.where(on, t, np.inf)))
                best = (float(wx[i]), float(wy[i]))
                break
            along += seg
            if along > config.NAV_SCAN_CHECK_M:
                break
        if best is None:
            return None
        # need a few points there, one stray reflection is not a door
        near = np.hypot(wx - best[0], wy - best[1]) < config.NAV_BLOCK_RADIUS_M
        return best if int(near.sum()) >= config.NAV_SCAN_CHECK_MIN_POINTS else None

    def _set_block(self, pose, points) -> None:
        """Enter the spot blocking the path as a wall for a while.

        The nearest obstacle in the direction of the path, otherwise simply a
        bit ahead. In map coordinates so that it counts when replanning.
        """
        import numpy as np

        x, y, th = pose
        target = self._follower.points[-1] if self._follower.points else (x, y)
        for p in self._follower.points:
            if math.hypot(p[0] - x, p[1] - y) > 0.3:
                target = p
                break
        direction = math.atan2(target[1] - y, target[0] - x) - th
        position = (0.4 * math.cos(direction), 0.4 * math.sin(direction))
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(pts):
            angle = np.arctan2(pts[:, 1], pts[:, 0]) - direction
            angle = np.arctan2(np.sin(angle), np.cos(angle))
            r = np.hypot(pts[:, 0], pts[:, 1])
            front = (np.abs(angle) < math.radians(40.0)) & (r < 1.0)
            if front.any():
                i = int(np.argmin(np.where(front, r, np.inf)))
                position = (float(pts[i, 0]), float(pts[i, 1]))
        c, s = math.cos(th), math.sin(th)
        wx, wy = x + c * position[0] - s * position[1], y + s * position[0] + c * position[1]
        self._blocks.append((wx, wy, time.monotonic() + config.NAV_BLOCK_S))
        log.info("Path blocked at (%.2f, %.2f) -- planning around it", wx, wy)

    def _obstacle(self, command: DriveCommand) -> float | None:
        """Is something in the way? Returns the distance, otherwise None.

        Fallback without a fresh scan for the detour -- then it stops instead
        of avoiding -- and the distance for the message when no path is free.
        """
        if self._localizer is None:
            return None
        if command.is_stop():
            return None

        # With mecanum the driving direction is not "ahead": the robot can
        # drive sideways. So look in the direction it is going.
        direction = math.degrees(math.atan2(command.vy, command.vx))
        distance = self._localizer.obstacle_ahead(direction)
        if distance is None:
            return None
        return distance if distance < config.HOME_STOP_DISTANCE_M else None

    def _detour(self, command: DriveCommand, distance: float, scan) -> DriveCommand | None:
        """Rotate the command into a free direction; None if none is free.

        It needs room up to just behind the start point (front plus margin),
        at most ``DETOUR_LOOKAHEAD_M`` plus the stop distance: a wall just
        behind the start point should not keep it from the goal. If there is
        no room for that anywhere, the stop distance is enough -- then it
        drives as before until it gets tight.
        """
        if command.is_stop():
            return command
        speed = math.hypot(command.vx, command.vy)
        direction = math.degrees(math.atan2(command.vy, command.vx))
        points = self._memory.points(scan)
        far = min(distance + config.AVOID_FRONT_M + config.AVOID_MARGIN_M,
                   config.DETOUR_LOOKAHEAD_M + config.HOME_STOP_DISTANCE_M)
        choice = None
        for needed in (far, min(far, config.HOME_STOP_DISTANCE_M)):
            choice = avoidance.choose_direction(
                points, direction, needed, config.EXPLORE_HALF_WIDTH_M, self._page
            )
            if choice is not None:
                break
        if choice is None:
            return None
        new, _free = choice
        shift = avoidance._normalize_angle(math.radians(new - direction))
        self._page = 0.0 if abs(shift) < 1e-6 else math.copysign(1.0, shift)
        a = math.radians(new)
        return DriveCommand(vx=speed * math.cos(a), vy=speed * math.sin(a), omega=0.0)

    def _step(self, x: float, y: float, angle_deg: float, distance: float) -> DriveCommand:
        """Drive command for this moment (x, y relative to the goal)."""
        # direction to the goal, in world coordinates
        dx, dy = -x, -y

        # Rotate into robot coordinates: the drive only knows "forward"
        # and "sideways", not "north".
        theta = math.radians(angle_deg)
        forward = dx * math.cos(theta) + dy * math.sin(theta)
        sideways = -dx * math.sin(theta) + dy * math.cos(theta)

        # Normalise to length 1 and multiply by the speed. Slow down shortly
        # before the goal, otherwise it overshoots and oscillates.
        speed = config.HOME_SPEED if self._speed is None else self._speed
        if distance < config.HOME_SLOWDOWN_M:
            share = distance / config.HOME_SLOWDOWN_M
            # Do not get arbitrarily slow -- otherwise it stops short of the
            # goal, because the motors do not start at very small values.
            speed *= max(0.35, share)

        length = math.hypot(forward, sideways) or 1.0
        return DriveCommand(
            vx=forward / length * speed,
            vy=sideways / length * speed,
            omega=0.0,  # Turning is unnecessary: mecanum drives in any direction.
        )
