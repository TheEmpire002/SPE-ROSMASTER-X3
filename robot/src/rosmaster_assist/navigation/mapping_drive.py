"""Mapping drive: cover the room like a robot vacuum.

Runs as a separate mode in the ``Explorer`` (``start(mode=MAP)``). So all
of the explorer's safety paths apply unchanged: emergency stop, manual
intervention, connection loss, lidar failure, "something very close".
This module only decides *where* to drive.

Phases:

1. **Wall follow.** To the nearest wall, then once around with the wall on
   the right until it is back at the beginning. After that the room
   boundaries are in the map, and it stands at a wall, parallel to it.

2. **Lanes.** Back and forth parallel to this wall, each lane
   ``MAPPING_DRIVE_LANE_SPACING_M`` further into the room, until the
   opposite wall is reached.

   Unlike a robot vacuum it **does not turn around**, but keeps its
   heading: forward up to the obstacle, shift sideways, backward again.
   The mecanum wheels can do that, and it has two advantages: rotations are
   the least accurate part of any movement on the X3, and SLAM is most
   likely to lose its orientation while turning. The lidar sees all
   around, so it does not drive backward blind.

3. **On into the unknown (04.11.2026).** When the room is done -- or it
   cannot get any further there (no wall in sight, stuck, corner) -- it
   does not stop but drives to the nearest place where the map borders on
   the unknown: open door, hallway, shadow behind the sofa
   (``path_planning.find_frontier``). This way the map grows room by room
   until no reachable frontier is left. Then it drives back to the map's
   origin and reports "Karte vollständig" (map complete). For this it needs
   its pose in the growing map (``map_localization.py``); without it the
   drive ends after the first room as before.

Steering uses only what the lidar currently sees (wall on the right,
obstacle ahead) and the pose from lidar odometry (``scan_odometry.py``) or
in the map, never time.
"""

from __future__ import annotations

import logging
import math
import time

from .. import config
from ..protocol import DriveCommand
from . import path_follower as pf
from .explorer import free_in_direction

log = logging.getLogger(__name__)

# Phases and steps -- they appear like this in the app as well.
APPROACH = "fährt zur Wand"
LEAVE_ISLAND = "verlässt ein Hindernis mitten im Raum"
ALIGN = "richtet sich an der Wand aus"
WALL_FOLLOW = "fährt an der Wand entlang"
CORNER = "dreht in der Ecke"
AROUND_CORNER = "fährt um die Ecke"
LANE = "fährt Bahnen"
SHIFT = "versetzt zur nächsten Bahn"
SHIFT_GAP = "sucht Platz für den Versatz"
FRONTIER_SEARCH = "sucht unbekannte Stellen"
TO_FRONTIER = "fährt zur nächsten unbekannten Stelle"
BACK = "fährt zum Ausgangspunkt der Karte"
_MAP_STEPS = (FRONTIER_SEARCH, TO_FRONTIER, BACK)

#: Window to the right of the robot in which the wall is searched (meters,
#: robot frame). Front and back slightly beyond the robot's length.
_WINDOW_X = (-0.25, 0.45)
#: The wall needs this many points in the window, otherwise it counts as lost.
_MIN_WALL_POINTS = 8
#: How long it turns on the spot before trying something else.
_MAX_TURN_S = 15.0
#: Search around a corner for at most this long (one full circle and a bit).
_MAX_AROUND_CORNER_S = 30.0
#: The farthest it drives back along a lane to find a gap for the shift
#: (about the length of a sofa).
_MAX_FRONTIER_M = 2.5
#: How long (s) to wait for a confident pose in the map before giving up on
#: driving on (the first map only arrives after a few seconds).
_POSE_WAIT_S = 30.0
#: How often (s) to check on the way whether the frontier has become known.
_FRONTIER_CHECK_S = 3.0


class MappingDrive:
    """Decides the next drive command tick by tick."""

    def __init__(self, scan_odometry, localization=None, planner=None) -> None:
        self._odo = scan_odometry
        # For driving on into the unknown: pose in the map and planner.
        self._localization = localization
        self._planner = planner
        self._follower = pf.PathFollower(speed=config.MAPPING_DRIVE_SPEED * 1.3,
                                     tolerance=config.NAV_GOAL_TOLERANCE_M * 2)
        self._done: list[tuple[float, float]] = []
        self._frontiers_visited = 0
        self.begin(time.monotonic())

    # --- Sequence ----------------------------------------------------------------

    def begin(self, now: float, resume: bool = False) -> None:
        """New mapping drive -- or, with ``resume``, straight on into the
        unknown (after an interruption, e.g. a USB reset). The frontiers
        already visited are then remembered."""
        self._step_name = APPROACH
        self._text = "sucht die nächste Wand"
        self.reason: str | None = None
        self._step_since = now
        # Driving on into the unknown
        self._job = None
        self._goal_frontier: tuple[float, float] | None = None
        self._pose_missing_since: float | None = None
        self._blocked_since: float | None = None
        self._frontier_checked = now
        self._consecutive_errors = 0
        if not resume:
            self._done = []
            self._frontiers_visited = 0
        self._edge_begin = now
        self._edge_start: tuple[float, float] | None = None
        self._edge_travel = 0.0
        self._edge_far_away = False
        self._islands = 0
        self._island_straight = False
        self._last_pos: tuple[float, float] | None = None
        # Lanes
        self._lane_origin = (0.0, 0.0)
        self._lane_angle = 0.0
        self._lane_wall: float | None = None
        self._frontier_searched = False
        self._frontier_since = 0.0
        self._lane_no = 0
        self._lane_direction = 1
        self._lanes_done = 0
        self._empty_lanes = 0
        self._lane_start_s = 0.0
        if resume and self._frontiers_possible():
            self._switch(FRONTIER_SEARCH, "setzt die Kartierung fort", now)

    def display(self) -> str:
        return f"{self._step_name}: {self._text}" if self._text else self._step_name

    def state(self) -> dict:
        return {
            "schritt": self._step_name,
            "text": self._text,
            "randweg_m": round(self._edge_travel, 1),
            "bahnen": self._lanes_done,
            "luecken": self._frontiers_visited,
            "ziel": (
                [round(self._goal_frontier[0], 2), round(self._goal_frontier[1], 2)]
                if self._goal_frontier is not None and self._step_name in _MAP_STEPS
                else None
            ),
            "weg": (
                [[round(p[0], 2), round(p[1], 2)] for p in self._follower.points]
                if self._step_name in (TO_FRONTIER, BACK) else []
            ),
        }

    def step(self, now: float, scan, points) -> DriveCommand | None:
        """One tick. ``None`` means done (reason in ``self.reason``)."""
        if self.reason is not None:
            return None
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)
        pose = self._odo.pose()
        age = self._odo.age()
        if pose is None or age is None or age > 1.0:
            # No pose, no lanes. The explorer ends the drive if the lidar
            # fails completely; here we only wait.
            self._text = "wartet auf die Lidar-Odometrie"
            return stop_cmd
        self._count_travel(pose)

        name = self._step_name
        if name == APPROACH:
            return self._approach(now, scan, points)
        if name == LEAVE_ISLAND:
            return self._away_from_island(now, scan, points)
        if name == ALIGN:
            return self._align(now, scan, pose)
        if name in (WALL_FOLLOW, CORNER, AROUND_CORNER):
            return self._wall_follow(now, points, pose)
        if name in (LANE, SHIFT, SHIFT_GAP):
            return self._lanes(now, points, pose)
        if name in _MAP_STEPS:
            return self._into_unknown(now, points)
        return stop_cmd

    def _switch(self, name: str, text: str, now: float) -> None:
        if name != self._step_name:
            log.info("Mapping drive: %s (%s)", name, text)
        self._step_name = name
        self._text = text
        self._step_since = now

    def _finish(self, reason: str) -> None:
        self.reason = reason
        log.info("Mapping drive finished: %s", reason)

    def _room_done(self, now: float, reason: str) -> None:
        """The room is done (or it cannot go on there): off into the unknown."""
        if not self._frontiers_possible():
            self._finish(reason)
            return
        log.info("Mapping drive: room finished (%s) -- on to the next frontier", reason)
        self._switch(FRONTIER_SEARCH, f"Zimmer fertig ({reason})", now)

    def _frontiers_possible(self) -> bool:
        return (
            config.FRONTIER_ACTIVE
            and self._localization is not None
            and self._planner is not None
            and self._localization.occ_map is not None
        )

    def report_obstacle(self, reason: str) -> bool:
        """The explorer cannot get any further (drive guard, stuck).

        True: the mapping drive tries elsewhere, the explorer should drive
        on. False: it can do nothing, the drive ends.
        """
        if not self._frontiers_possible() or self.reason is not None:
            return False
        now = time.monotonic()
        if self._step_name in _MAP_STEPS:
            self._abandon_goal(now, reason)
        else:
            self._room_done(now, f"kommt nicht weiter: {reason}")
        return True

    # --- 3. On into the unknown --------------------------------------------------------

    def _into_unknown(self, now: float, points) -> DriveCommand:
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)
        localization = self._localization
        pose = localization.pose() if localization is not None else None
        if pose is None or not localization.confident or localization.occ_map is None:
            self._pose_missing_since = self._pose_missing_since or now
            if now - self._pose_missing_since > _POSE_WAIT_S:
                self._finish("Lage in der Karte unbekannt, Weiterfahrt nicht möglich")
                return stop_cmd
            self._text = "wartet auf die Lage in der Karte"
            return stop_cmd
        self._pose_missing_since = None

        # A path is being planned right now: wait.
        if self._job is not None:
            if not self._job.done():
                self._text = "plant den Weg"
                return stop_cmd
            path = self._job.result()
            self._job = None
            return self._path_received(now, pose, path)

        if self._step_name == FRONTIER_SEARCH:
            self._job = self._planner.gap(localization.occ_map, (pose[0], pose[1]), self._done)
            self._text = "sucht die nächste unbekannte Stelle"
            return stop_cmd

        # On the way: if the frontier has become known in the meantime (the
        # lidar sees far), it does not need to go all the way.
        if self._step_name == TO_FRONTIER and now - self._frontier_checked >= _FRONTIER_CHECK_S:
            self._frontier_checked = now
            if self._goal_frontier is not None and not _still_unknown(localization.occ_map, self._goal_frontier):
                self._frontier_done(now, "schon von unterwegs gesehen")
                return stop_cmd

        command, state = self._follower.step(pose, points)
        if state == pf.ARRIVED:
            if self._step_name == BACK:
                self._finish(
                    f"Karte vollständig ({self._frontiers_visited} Lücken angefahren), "
                    "zurück am Ausgangspunkt"
                )
                return stop_cmd
            self._frontier_done(now, "angekommen")
            return stop_cmd
        if state in (pf.BLOCKED, pf.OFF_PATH):
            self._blocked_since = self._blocked_since or now
            if state == pf.OFF_PATH or now - self._blocked_since > config.NAV_BLOCKED_S:
                self._abandon_goal(
                    now, "Weg versperrt" if state == pf.BLOCKED else "vom Weg abgekommen"
                )
            return stop_cmd
        self._blocked_since = None
        self._text = f"noch {self._follower.rest_m:.1f} m, {self._frontiers_visited} Lücken angefahren"
        return command

    def _path_received(self, now: float, pose, path) -> DriveCommand:
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)
        if self._step_name == BACK:
            if path is None:
                self._finish("Karte vollständig, aber kein Weg zurück zum Ausgangspunkt")
                return stop_cmd
            self._follower.apply(path.points, from_=pose, goal_angle=self._localization.start[2])
            self._text = f"Rückweg {path.length:.1f} m"
            return stop_cmd
        if path is None:
            # No reachable frontier left: the map is done.
            start = self._localization.start
            far = math.hypot(start[0] - pose[0], start[1] - pose[1]) > config.FRONTIER_NEAR_M
            if config.MAPPING_DRIVE_RETURN and far:
                self._switch(BACK, "keine unbekannte Stelle mehr", now)
                self._goal_frontier = None
                self._follower.apply([])
                self._job = self._planner.path(
                    self._localization.occ_map, (pose[0], pose[1]), (start[0], start[1])
                )
                return stop_cmd
            self._finish(f"Karte vollständig ({self._frontiers_visited} Lücken angefahren)")
            return stop_cmd
        self._goal_frontier = path.gap
        self._follower.apply(path.points, from_=pose)
        self._blocked_since = None
        self._frontier_checked = now
        self._switch(TO_FRONTIER, f"Lücke in {path.length:.1f} m Weg", now)
        return stop_cmd

    def _frontier_done(self, now: float, details: str) -> None:
        if self._goal_frontier is not None:
            self._done.append(self._goal_frontier)
        self._frontiers_visited += 1
        self._consecutive_errors = 0
        self._switch(FRONTIER_SEARCH, f"Lücke {self._frontiers_visited}: {details}", now)

    def _abandon_goal(self, now: float, reason: str) -> None:
        """This frontier is not reachable: remember it and take the next one."""
        self._job = None
        if self._step_name == BACK:
            self._finish(f"Karte vollständig, Rückweg zum Ausgangspunkt versperrt ({reason})")
            return
        if self._goal_frontier is not None:
            self._done.append(self._goal_frontier)
        self._consecutive_errors += 1
        if self._consecutive_errors > config.FRONTIER_MAX_ERRORS:
            self._finish(
                f"{self._consecutive_errors} unbekannte Stellen in Folge nicht erreicht ({reason})"
            )
            return
        log.info("Mapping drive: frontier abandoned (%s)", reason)
        self._switch(FRONTIER_SEARCH, f"Lücke aufgegeben: {reason}", now)

    # --- 1. To the wall ------------------------------------------------------------

    def _approach(self, now: float, scan, points) -> DriveCommand:
        direction, distance = _nearest_wall(scan)
        if direction is None:
            self._room_done(now, "keine Wand in Sicht")
            return DriveCommand(0.0, 0.0, 0.0)

        front = free_in_direction(points, 0.0, config.EXPLORE_HALF_WIDTH_M)
        if front <= config.MAPPING_DRIVE_WALL_DISTANCE_M + 0.05:
            self._switch(ALIGN, "Wand voraus, dreht nach links", now)
            return DriveCommand(0.0, 0.0, 0.0)

        # First turn the wall to the front, then drive straight towards it.
        # It turns until the lidar sees it in front -- not by time.
        if abs(direction) > 10.0:
            if now - self._step_since > _MAX_TURN_S:
                self._room_done(now, "konnte sich nicht zur Wand drehen")
                return DriveCommand(0.0, 0.0, 0.0)
            self._text = f"nächste Wand {distance:.1f} m bei {direction:+.0f}°"
            return DriveCommand(0.0, 0.0, math.copysign(config.MAPPING_DRIVE_TURN_SPEED, direction))
        self._step_since = now
        self._text = f"Wand in {front:.1f} m"
        omega = 0.02 * direction  # small steering correction, the wall stays in front
        return DriveCommand(self._speed(front, config.MAPPING_DRIVE_WALL_DISTANCE_M), 0.0, omega)

    def _away_from_island(self, now: float, scan, points) -> DriveCommand:
        """Turn its back to the island and drive straight to the next wall.

        Straight ahead and not "to the nearest wall": the nearest one would
        again be the island it is standing at.
        """
        direction, _ = _nearest_wall(scan)
        if not self._island_straight and direction is not None and abs(direction) < 150.0:
            if now - self._step_since > _MAX_TURN_S:
                self._room_done(now, "kam von einem Hindernis nicht los")
                return DriveCommand(0.0, 0.0, 0.0)
            # Turn the island to the back, the shortest way.
            return DriveCommand(0.0, 0.0, -math.copysign(config.MAPPING_DRIVE_TURN_SPEED, direction))
        # Once turned away, only straight ahead -- even if something else
        # becomes the nearest afterwards.
        self._island_straight = True
        self._text = "fährt geradeaus"
        front = free_in_direction(points, 0.0, config.EXPLORE_HALF_WIDTH_M)
        if front <= config.MAPPING_DRIVE_WALL_DISTANCE_M + 0.05:
            self._switch(ALIGN, "Wand voraus, dreht nach links", now)
            return DriveCommand(0.0, 0.0, 0.0)
        return DriveCommand(self._speed(front, config.MAPPING_DRIVE_WALL_DISTANCE_M), 0.0, 0.0)

    def _align(self, now: float, scan, pose) -> DriveCommand:
        """Turn left until the wall is on the right."""
        direction, _ = _nearest_wall(scan)
        if direction is not None and -100.0 <= direction <= -80.0:
            self._switch(WALL_FOLLOW, "Wand rechts", now)
            self._edge_begin = now
            self._edge_start = (pose[0], pose[1])
            self._edge_travel = 0.0
            self._edge_far_away = False
            return DriveCommand(0.0, 0.0, 0.0)
        if now - self._step_since > _MAX_TURN_S:
            self._room_done(now, "konnte sich nicht an der Wand ausrichten")
            return DriveCommand(0.0, 0.0, 0.0)
        return DriveCommand(0.0, 0.0, config.MAPPING_DRIVE_TURN_SPEED)

    # --- 2. Wall follow --------------------------------------------------------------

    def _wall_follow(self, now: float, points, pose) -> DriveCommand:
        if self._went_around(pose):
            if self._edge_travel < config.MAPPING_DRIVE_ISLAND_M and self._islands < 3:
                # Too short for a room: that was a pillar, a sofa or a table
                # in the middle of the room, not the wall. Get away from it
                # and search for the wall.
                self._islands += 1
                self._island_straight = False
                self._switch(
                    LEAVE_ISLAND, f"Runde nur {self._edge_travel:.1f} m, war keine Wand", now
                )
                self._edge_start = None
                return DriveCommand(0.0, 0.0, 0.0)
            return self._begin_lanes(now, pose, points, "einmal rundherum")
        if now - self._edge_begin > config.MAPPING_DRIVE_WALL_FOLLOW_MAX_S:
            return self._begin_lanes(now, pose, points, "Randfahrt dauerte zu lange")

        distance = config.MAPPING_DRIVE_WALL_DISTANCE_M
        front = free_in_direction(points, 0.0, config.EXPLORE_HALF_WIDTH_M)
        wall = _wall_right(points)

        # Inner corner: it gets tight in front. Turn left on the spot until
        # there is room in front again -- then the new wall is on the right.
        if self._step_name == CORNER or front < distance + 0.1:
            if self._step_name != CORNER:
                self._switch(CORNER, f"vorn {front:.2f} m", now)
            if front >= distance + 0.45 and wall is not None and abs(wall[0]) < 20.0:
                self._switch(WALL_FOLLOW, "weiter an der Wand", now)
            elif now - self._step_since > _MAX_TURN_S:
                return self._begin_lanes(now, pose, points, "kam aus einer Ecke nicht heraus")
            else:
                return DriveCommand(0.0, 0.0, config.MAPPING_DRIVE_TURN_SPEED)

        if wall is None:
            # Outer corner or doorway: the wall on the right ends. Arc to the
            # right around its end until it shows up again.
            if self._step_name != AROUND_CORNER:
                self._switch(AROUND_CORNER, "Wand rechts endet", now)
            if now - self._step_since > _MAX_AROUND_CORNER_S:
                return self._begin_lanes(now, pose, points, "Wand verloren")
            vx = config.MAPPING_DRIVE_SPEED * 0.6
            return DriveCommand(vx, 0.0, -vx / distance)

        if self._step_name != WALL_FOLLOW:
            self._switch(WALL_FOLLOW, "Wand wieder rechts", now)
        angle, dist = wall
        # Stay parallel: turn as much as the wall is slanted.
        omega = 0.03 * angle
        # Keep the distance with the mecanum wheels, without turning: too far
        # away means move right (negative vy).
        vy = max(-0.08, min(0.08, -0.8 * (dist - distance)))
        self._text = f"Wand {dist:.2f} m, {self._edge_travel:.1f} m gefahren"
        return DriveCommand(self._speed(front, distance + 0.1), vy, max(-0.5, min(0.5, omega)))

    def _went_around(self, pose) -> bool:
        """Back at the beginning of the wall follow?

        Only counts once it has been far enough away -- otherwise it would
        already be "back" right at the start.
        """
        if self._edge_start is None:
            return False
        dist_from_start = math.hypot(pose[0] - self._edge_start[0], pose[1] - self._edge_start[1])
        if dist_from_start > 2 * config.MAPPING_DRIVE_LAP_M:
            self._edge_far_away = True
        return (
            self._edge_far_away
            and self._edge_travel >= config.MAPPING_DRIVE_LAP_MIN_M
            and dist_from_start < config.MAPPING_DRIVE_LAP_M
        )

    def _count_travel(self, pose) -> None:
        pos = (pose[0], pose[1])
        if self._last_pos is not None and self._step_name in (WALL_FOLLOW, CORNER, AROUND_CORNER):
            self._edge_travel += math.hypot(pos[0] - self._last_pos[0], pos[1] - self._last_pos[1])
        self._last_pos = pos

    # --- 3. Lanes ----------------------------------------------------------------------

    def _begin_lanes(self, now: float, pose, points, cause: str) -> DriveCommand:
        """Lanes parallel to the wall it is currently standing at."""
        log.info("Mapping drive: wall follow finished (%s, %.1f m)", cause, self._edge_travel)
        self._lane_origin = (pose[0], pose[1])
        self._lane_angle = pose[2]
        self._lane_wall = main_direction(points)
        # Lane 0 is the wall follow itself. The wall is on the right, so it
        # goes left into the room. The direction reverses with each shift:
        # lane 1 forward, lane 2 backward and so on.
        self._lane_no = 0
        self._lane_direction = -1
        self._lanes_done = 0
        self._empty_lanes = 0
        self._switch(SHIFT, f"Randfahrt fertig ({cause}), Bahn 1", now)
        return DriveCommand(0.0, 0.0, 0.0)

    def _lanes(self, now: float, points, pose) -> DriveCommand:
        along, lateral = self._lane_coordinates(pose)
        # Hold the heading so that the lanes stay parallel. Preferably using
        # the walls: if they lie at the same angle as at the beginning, the
        # direction is right, no matter how far odometry has drifted since.
        # Odometry only counts when there is no clear wall direction.
        now_wall = main_direction(points) if self._lane_wall is not None else None
        if now_wall is not None:
            error = math.radians((now_wall - self._lane_wall + 45.0) % 90.0 - 45.0)
        else:
            error = _normalize(self._lane_angle - pose[2])
        omega = max(-0.4, min(0.4, 1.5 * error))

        target = (self._lane_no + 1) * config.MAPPING_DRIVE_LANE_SPACING_M
        left = free_in_direction(points, 90.0, config.MAPPING_DRIVE_HALF_LENGTH_M)

        if self._step_name == SHIFT_GAP:
            # Back along the lane until there is enough room on the left for the shift.
            direction = -self._lane_direction
            free = free_in_direction(points, 0.0 if direction > 0 else 180.0,
                                    config.EXPLORE_HALF_WIDTH_M)
            if left >= (target - lateral) + config.EXPLORE_STOP_M:
                self._switch(SHIFT, f"Lücke gefunden, Bahn {self._lane_no + 1}", now)
                return DriveCommand(0.0, 0.0, omega)
            if free < config.EXPLORE_STOP_M or abs(along - self._frontier_since) > _MAX_FRONTIER_M:
                self._room_done(
                    now, f"seitlich kein Platz mehr, {self._lanes_done} Bahnen gefahren"
                )
                return DriveCommand(0.0, 0.0, 0.0)
            target_lateral = self._lane_no * config.MAPPING_DRIVE_LANE_SPACING_M
            vy = max(-0.08, min(0.08, -1.0 * (lateral - target_lateral)))
            self._text = f"links {left:.2f} m frei, sucht weiter"
            return DriveCommand(direction * self._speed(free, config.EXPLORE_STOP_M), vy, omega)

        if self._step_name == SHIFT:
            if left < config.EXPLORE_STOP_M:
                if not self._frontier_searched:
                    # It is tight on the left -- maybe just a piece of
                    # furniture where the lane ended, and not the opposite
                    # wall. Go back a bit along the lane and shift there.
                    self._frontier_searched = True
                    self._frontier_since = along
                    self._switch(SHIFT_GAP, "links zu eng, fährt ein Stück zurück", now)
                    return DriveCommand(0.0, 0.0, omega)
                self._room_done(
                    now, f"gegenüberliegende Wand erreicht, {self._lanes_done} Bahnen gefahren"
                )
                return DriveCommand(0.0, 0.0, 0.0)
            max_duration = 3.0 * config.MAPPING_DRIVE_LANE_SPACING_M / config.MAPPING_DRIVE_SIDE_SPEED
            if lateral >= target - 0.03 or now - self._step_since > max_duration:
                self._lane_no += 1
                self._frontier_searched = False
                # After a gap search it already points in the opposite
                # direction of the last lane; the new lane then continues
                # that way.
                self._lane_direction = -self._lane_direction
                self._lane_start_s = along
                self._switch(
                    LANE, f"Bahn {self._lane_no} "
                    + ("vorwärts" if self._lane_direction > 0 else "rückwärts"), now,
                )
                return DriveCommand(0.0, 0.0, omega)
            self._text = f"noch {max(0.0, target - lateral):.2f} m zur Bahn {self._lane_no + 1}"
            speed = config.MAPPING_DRIVE_SIDE_SPEED
            if left < config.EXPLORE_SLOW_M:
                speed *= 0.5
            return DriveCommand(0.0, speed, omega)

        # On the lane: straight ahead (forward or backward), hold the line sideways.
        direction_deg = 0.0 if self._lane_direction > 0 else 180.0
        free = free_in_direction(points, direction_deg, config.EXPLORE_HALF_WIDTH_M)
        if free < config.EXPLORE_STOP_M:
            length = abs(along - self._lane_start_s)
            self._lanes_done += 1
            # Nothing but lanes without length: it is stuck between obstacles.
            self._empty_lanes = self._empty_lanes + 1 if length < 0.2 else 0
            if self._empty_lanes >= 3:
                self._room_done(now, "kein Platz mehr für Bahnen")
                return DriveCommand(0.0, 0.0, 0.0)
            self._switch(SHIFT, f"Bahn {self._lane_no} zu Ende ({length:.1f} m)", now)
            return DriveCommand(0.0, 0.0, omega)
        target_lateral = self._lane_no * config.MAPPING_DRIVE_LANE_SPACING_M
        vy = max(-0.08, min(0.08, -1.0 * (lateral - target_lateral)))
        self._text = f"Bahn {self._lane_no}, frei {min(free, 9.9):.1f} m"
        vx = self._lane_direction * self._speed(free, config.EXPLORE_STOP_M)
        return DriveCommand(vx, vy, omega)

    def _lane_coordinates(self, pose) -> tuple[float, float]:
        """(along, lateral) relative to the start of the lanes; lateral > 0 = into the room."""
        dx = pose[0] - self._lane_origin[0]
        dy = pose[1] - self._lane_origin[1]
        c, s = math.cos(self._lane_angle), math.sin(self._lane_angle)
        return c * dx + s * dy, -s * dx + c * dy

    # --- Helpers -----------------------------------------------------------------------

    @staticmethod
    def _speed(free: float, stop_at: float) -> float:
        """Cruising speed, reduced in front of obstacles (not below 40 %)."""
        span = max(config.EXPLORE_SLOW_M - stop_at, 0.1)
        share = min(1.0, max(0.4, (free - stop_at) / span))
        return config.MAPPING_DRIVE_SPEED * share


def _wall_right(points) -> tuple[float, float] | None:
    """Wall to the right of the robot: (angle in degrees, distance in meters).

    A line through the points in the window on the right (principal axis).
    The angle is its slope relative to the direction of travel: positive
    means it comes closer towards the front -- then it has to turn left to
    stay parallel. A single beam would be simpler, but mistakes every chair
    leg for a wall.
    """
    import numpy as np

    if len(points) == 0:
        return None
    x, y = points[:, 0], points[:, 1]
    far = config.MAPPING_DRIVE_WALL_DISTANCE_M + 0.6
    in_window = (x > _WINDOW_X[0]) & (x < _WINDOW_X[1]) & (y < -0.05) & (y > -far)
    if int(in_window.sum()) < _MIN_WALL_POINTS:
        return None
    p = points[in_window]
    center = p.mean(axis=0)
    _, _, vt = np.linalg.svd(p - center, full_matrices=False)
    dx, dy = vt[0]
    if dx < 0:
        dx, dy = -dx, -dy
    angle = math.degrees(math.atan2(dy, dx))
    if abs(angle) > 60.0:
        # Lies more across than along -- that is not a wall beside it.
        return None
    # Perpendicular distance of the line to the robot center.
    distance = abs(-dy * center[0] + dx * center[1])
    return angle, distance


def _still_unknown(occ_map, point) -> bool:
    """Is there still unknown space near this frontier?"""
    import numpy as np

    r = config.FRONTIER_NEAR_M
    angle = np.linspace(0.0, 2 * math.pi, 16, endpoint=False)
    xs = np.concatenate(([point[0]], point[0] + r * np.cos(angle)))
    ys = np.concatenate(([point[1]], point[1] + r * np.sin(angle)))
    return bool(occ_map.unknown_at(xs, ys).any())


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main_direction(points) -> float | None:
    """Direction of the walls in the robot frame, in degrees modulo 90 (0..90).

    Room walls are almost always at right angles to each other. Their common
    direction can therefore be read from every single scan -- without
    summing anything up, so without drift. If the robot turns by +5°, the
    walls lie 5° differently for it.

    Method: directions of short segments between neighboring points,
    weighted by length, into a histogram over 90°; the peak is the main
    direction. Segments over eight points instead of two, otherwise every
    direction wobbles with the A1's measurement noise: with simulated 3 cm
    noise the result was off by at most 1° this way, over four points there
    often was none at all. Longer segments, on the other hand, cut corners.
    None if no clear direction can be seen (round room, only furniture in
    sight).
    """
    import numpy as np

    if len(points) < 30:
        return None
    order = np.argsort(np.arctan2(points[:, 1], points[:, 0]))
    p = points[order]
    d = p[8:] - p[:-8]
    length = np.hypot(d[:, 0], d[:, 1])
    good = (length > 0.1) & (length < 1.0)
    if int(good.sum()) < 15:
        return None
    length = length[good]
    angle = np.degrees(np.arctan2(d[good, 1], d[good, 0])) % 90.0
    hist = np.zeros(90)
    np.add.at(hist, angle.astype(int) % 90, length)
    smoothed = hist + np.roll(hist, 1) + np.roll(hist, -1)
    peak = int(np.argmax(smoothed))
    if smoothed[peak] < 0.2 * length.sum():
        return None
    # Refine: mean of the segments near the peak, across the 0/90 boundary.
    deviation = (angle - peak + 45.0) % 90.0 - 45.0
    close = np.abs(deviation) <= 3.0
    return float((peak + np.average(deviation[close], weights=length[close])) % 90.0)


def _nearest_wall(scan) -> tuple[float | None, float]:
    """Direction (degrees, -180..180) and distance of the nearest solid obstacle.

    In 5° sectors, using the second-nearest point each: a single stray beam
    must not count as the "nearest wall".
    """
    sectors: dict[int, list[float]] = {}
    for w, r in zip(scan.angle, scan.ranges):
        sectors.setdefault(int(w // 5) % 72, []).append(r)
    best: tuple[float | None, float] = (None, float("inf"))
    for nr, values in sectors.items():
        if len(values) < 2:
            continue
        values.sort()
        if values[1] < best[1]:
            center = nr * 5 + 2.5
            best = (center - 360.0 if center > 180.0 else center, values[1])
    return best
