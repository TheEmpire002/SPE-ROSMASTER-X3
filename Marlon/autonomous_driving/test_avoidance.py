"""Checks the explorer's local avoidance (step F2) -- without a robot.

    python Marlon/autonomous_driving/test_avoidance.py

First the building blocks from ``navigation/avoidance.py`` one by one (gaps,
arcs, short-term memory, lift), then the whole explorer in a simulated world
with walls, doors and round obstacles: it should find its way through doors
and corridors, drive around obstacles in an arc instead of turning in front of
them, detect being lifted, and raise no false alarm while doing so.

Like ``test_explorer.py``, the world ray-casts lidar beams (7.5 Hz) and
integrates the pose from the drive commands; wall segments are added.
"""

import math
import pathlib
import random
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.navigation import avoidance as aw  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import obstacles as hd  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0
TICK = 1.0 / config.ODOMETRY_HZ
SCAN_S = 1.0 / 7.5
#: Approximate robot radius around the lidar centre (body ~0.30 m wide).
ROBOT_R = 0.15
INFINITY = float("inf")


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


# --- Simulated world ---------------------------------------------------------------


class World:
    """Rectangular room with wall segments, round obstacles and a robot."""

    def __init__(self, room=(-3.0, 3.0, -3.0, 3.0), walls=(), circles=(), x=0.0, y=0.0, theta=0.0):
        self.room = room
        self.walls = list(walls)  # ((x1, y1), (x2, y2))
        self.circles = list(circles)  # (x, y, r)
        self.x, self.y, self.theta = x, y, theta
        self.v_actual = (0.0, 0.0, 0.0)  # in the robot frame, as the board reports it
        self.tilted = False  # lifted: the lidar plane hits floor and ceiling
        self.carried = None  # (vx, vy, omega) in world coordinates, without wheel rotation

    def ray(self, direction):
        dx, dy = math.cos(direction), math.sin(direction)
        x0, x1, y0, y1 = self.room
        best = INFINITY
        for limit, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9:
                t = (limit - start) / d
                if t > 0:
                    best = min(best, t)
        for (ax, ay), (bx, by) in self.walls:
            ex_, ey = bx - ax, by - ay
            denom = dx * ey - dy * ex_
            if abs(denom) < 1e-12:
                continue
            t = ((ax - self.x) * ey - (ay - self.y) * ex_) / denom
            u = ((ax - self.x) * dy - (ay - self.y) * dx) / denom
            if t > 0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        for cx, cy, r in self.circles:
            mx, my = self.x - cx, self.y - cy
            b = mx * dx + my * dy
            c = mx * mx + my * my - r * r
            disk = b * b - c
            if disk >= 0:
                t = -b - math.sqrt(disk)
                if t > 0:
                    best = min(best, t)
        return best

    def scan(self, timestamp):
        angle, dist = [], []
        # Whoever holds it wobbles: the tilt, and with it the distance at which
        # the lidar plane hits the floor, changes from scan to scan.
        floor = random.uniform(0.25, 1.5) if self.tilted else None
        for i in range(360):
            r = self.ray(self.theta + math.radians(i))
            if floor is not None and (i < 90 or i > 270):
                # Held at an angle: the front half hits the floor.
                r = min(r, floor)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                angle.append(float(i if i <= 180 else i - 360))
                dist.append(r)
        s = Scan(angle, dist)
        s.timestamp = timestamp
        return s

    def move(self, command, dt):
        if self.carried is not None:
            # Carried: it moves, but the wheels report none of it.
            vx, vy, om = self.carried
            self.x += vx * dt
            self.y += vy * dt
            self.theta += om * dt
            self.v_actual = (0.0, 0.0, 0.0)
            return
        if command is None:
            self.v_actual = (0.0, 0.0, 0.0)
            return
        vx, vy, om = command.vx, command.vy, command.omega
        self.v_actual = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def distance(self):
        """Smallest distance from the lidar centre to a wall, wall segment or obstacle."""
        x0, x1, y0, y1 = self.room
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
        for (ax, ay), (bx, by) in self.walls:
            ex_, ey = bx - ax, by - ay
            u = max(0.0, min(1.0, ((self.x - ax) * ex_ + (self.y - ay) * ey) / (ex_ * ex_ + ey * ey)))
            d = min(d, math.hypot(self.x - ax - u * ex_, self.y - ay - u * ey))
        for cx, cy, r in self.circles:
            d = min(d, math.hypot(self.x - cx, self.y - cy) - r)
        return d


class FakeLidar:
    def __init__(self):
        self.last = None

    def latest(self):
        return self.last


class FakeOdometry:
    def __init__(self, world, measuring=True):
        self.world = world
        self.measuring = measuring

    @property
    def actually_measuring(self):
        return self.measuring

    def velocity(self):
        return self.world.v_actual


class WatchdogStub:
    # The ProgressWatch reads the last accepted command.
    current_command = DriveCommand()

    def submit(self, command, source=""):
        self.current_command = command

    def stop(self):
        pass


def build(world, measuring=True):
    lidar = FakeLidar()
    e = ex.Explorer(WatchdogStub(), lidar, hd.ObstacleDetection(), None,
                    odometry=FakeOdometry(world, measuring))
    return e, lidar


def run_sim(world, e, lidar, duration, t0=0.0, observer=None, no_switch=False):
    """Simulates ``duration`` seconds. Returns (states, smallest distance, end)."""
    now = t0
    next_scan = t0
    states = []
    smallest = world.distance()
    if t0 == 0.0:
        lidar.last = world.scan(0.0)
        e._begin(0.0)
    while now < t0 + duration:
        if no_switch:
            e._next_switch = INFINITY
        if now >= next_scan:
            lidar.last = world.scan(now)
            next_scan += SCAN_S
        command = e._step(now)
        states.append((now, e._state, command))
        if observer:
            observer(now, e, command)
        if command is None:
            return states, smallest, now
        world.move(command, TICK)
        if world.carried is None:
            smallest = min(smallest, world.distance())
        now += TICK
    return states, smallest, None


def wall_points(a, b, step=0.02):
    (ax, ay), (bx, by) = a, b
    n = max(2, int(math.hypot(bx - ax, by - ay) / step))
    return np.column_stack([np.linspace(ax, bx, n), np.linspace(ay, by, n)])


def circle_points(cx, cy, r, n=60):
    w = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return np.column_stack([cx + r * np.cos(w), cy + r * np.sin(w)])


def scan_from(angle_dist, timestamp):
    s = Scan([w for w, _ in angle_dist], [r for _, r in angle_dist])
    s.timestamp = timestamp
    return s


# --- 1. Gaps: the middle of a door, not the frame --------------------------------
# Box around the robot, 1.0 m ahead a 0.8 m wide door at y = 0.5;
# the walls are closer than the threshold, it is free only through the door.
box = np.vstack([
    wall_points((1.0, -1.0), (1.0, 0.1)),
    wall_points((1.0, 0.9), (1.0, 1.0)),
    wall_points((-1.0, -1.0), (-1.0, 1.0)),
    wall_points((-1.0, 1.0), (1.0, 1.0)),
    wall_points((-1.0, -1.0), (1.0, -1.0)),
])
width = config.EXPLORE_HALF_WIDTH_M
threshold = config.EXPLORE_GO_M + 0.5
good_gaps = aw.gaps(box, width, threshold, config.OBSTACLE_RANGE_M)
door = math.degrees(math.atan2(0.5, 1.0))
check("gaps: door found", any(abs(r - door) <= 4 for r, _ in good_gaps),
       f"door at {door:.0f}°, found {[r for r, _ in good_gaps]}")
# Corners are farther away than the walls and may be free; no direction may
# point straight at a wall or narrowly past the door frame.
check("gaps: not towards the wall",
       not any(min(abs(r - w) for w in (0, 90, -90, 180, -180)) < 15 for r, _ in good_gaps), str(good_gaps))
# Through the door: both frame posts lie, across the direction, outside the
# driving corridor (which already includes a margin around the robot) -- the middle
# of the free area need not be exactly the geometric door centre, but must lead safely through.
def lateral_distance(r, point):
    w = math.radians(r)
    return abs(-math.sin(w) * point[0] + math.cos(w) * point[1])
door_directions = [r for r, _ in good_gaps if abs(r - door) < 20]
check("gaps: not close to the door frame",
       door_directions and all(min(lateral_distance(r, (1.0, 0.1)), lateral_distance(r, (1.0, 0.9))) >= width
                               for r in door_directions), str(good_gaps))
# Cross-check: every chosen direction is really free in the driving corridor.
free = aw.free_per_direction(box, [r for r, _ in good_gaps], width)
check("gaps: chosen directions free", bool(np.all(free >= threshold)), str(np.round(free, 2)))
check("gaps: all free -> directions all around",
       len(aw.gaps(np.zeros((0, 2)), width, threshold, config.OBSTACLE_RANGE_M)) >= 30)
narrow = np.vstack([circle_points(0.0, 0.0, 0.5, 200)])
check("gaps: closed all around -> none", aw.gaps(narrow, width, threshold, config.OBSTACLE_RANGE_M) == [])

# --- 2. Arcs --------------------------------------------------------------------
empty = np.zeros((0, 2))
choice = aw.plan_arc(empty, config.EXPLORE_SPEED, 0.0, 0.0)
check("arc: free -> straight ahead at full speed",
       choice is not None and choice[0][1] == 0.0 and choice[0][2] == 0.0
       and abs(choice[0][0] - config.EXPLORE_SPEED) < 1e-9, str(choice))
choice = aw.plan_arc(empty, config.EXPLORE_SPEED, 0.6, 0.0)
check("arc: heading is to the left -> steers left", choice is not None and choice[0][2] > 0, str(choice))
choice = aw.plan_arc(circle_points(0.7, 0.05, 0.15), config.EXPLORE_SPEED * 0.5, 0.0, 0.0)
check("arc: obstacle ahead -> drives past instead of into it",
       choice is not None and choice[0][0] > 0 and (choice[0][2] != 0.0 or choice[0][1] != 0.0), str(choice))
check("arc: avoids towards the free side (obstacle slightly left -> right)",
       choice is not None and (choice[0][2] < 0 or choice[0][1] < 0), str(choice))
cell = np.vstack([wall_points((0.22, -0.3), (0.22, 0.3)),
                   wall_points((-0.3, 0.2), (0.22, 0.2)),
                   wall_points((-0.3, -0.2), (0.22, -0.2))])
check("arc: dead end directly ahead -> none (turn around)",
       aw.plan_arc(cell, config.EXPLORE_SPEED * 0.35, 0.0, 0.0) is None)

# --- 3. Short-term memory ------------------------------------------------------


class FixedSpeedOdometry:
    """Odometry that reports a fixed velocity."""

    def __init__(self, v=(0.0, 0.0, 0.0), measuring=True):
        self.v = v
        self.actually_measuring = measuring

    def velocity(self):
        return self.v


drive = FixedSpeedOdometry((0.2, 0.0, 0.0))
g = aw.ShortTermMemory(drive)
g.advance(0.05)
# Box 0.40 m ahead, then it drives 0.30 m closer: at 0.10 m the box is below
# LIDAR_MIN_M and missing from the scan -- the memory still has it.
g.points(scan_from([(0.0, 0.40), (90.0, 2.0), (-90.0, 2.0)], 0.0))
for _ in range(30):
    g.advance(0.05)
p = g.points(scan_from([(90.0, 2.0), (-90.0, 2.0)], 1.5))
close = p[np.hypot(p[:, 0], p[:, 1]) < config.LIDAR_MIN_M]
check("memory: box in the blind spot remains", len(close) == 1 and abs(close[0, 0] - 0.10) < 0.02, str(p))
check("memory: heading measured", g.heading is not None and abs(g.heading) < 1e-9, str(g.heading))

g = aw.ShortTermMemory(FixedSpeedOdometry((0.0, 0.0, 0.0)))
g.advance(0.05)
g.points(scan_from([(0.0, 0.60), (90.0, 2.0)], 0.0))
g.advance(0.05)
p = g.points(scan_from([(0.0, 2.50), (90.0, 2.0)], 0.2))
check("memory: thing that went away is cleared",
       not np.any(np.hypot(p[:, 0] - 0.6, p[:, 1]) < 0.05), str(p))

g = aw.ShortTermMemory(FixedSpeedOdometry((0.0, 0.0, 0.0)))
g.advance(0.05)
g.points(scan_from([(0.0, 0.60)], 0.0))
for _ in range(int(config.AVOID_MEMORY_S / 0.05) + 5):
    g.advance(0.05)
p = g.points(scan_from([(90.0, 2.0)], config.AVOID_MEMORY_S + 0.5))
check("memory: forgets after AVOID_MEMORY_S", len(p) == 1, str(p))

g = aw.ShortTermMemory(FixedSpeedOdometry(measuring=False))
g.advance(0.05)
g.points(scan_from([(0.0, 0.60)], 0.0))
p = g.points(scan_from([(90.0, 2.0)], 0.2))
check("memory: without measurement only the current scan", len(p) == 1 and g.heading is None, str(p))

# --- 4. Lift (building blocks) --------------------------------------------------
random.seed(3)
world = World(room=(-2.0, 3.0, -1.5, 2.0), circles=[(1.0, 0.8, 0.3)])
h = aw.LiftDetector()
alarm = False
for i in range(60):
    world.theta = 0.08 * i  # turns on the spot, pose measured
    alarm |= h.check(world.scan(i * SCAN_S), (0.0, 0.0, world.theta))
check("lift: turning with measurement no alarm", not alarm, f"last share {h.share:.2f}")
h = aw.LiftDetector()
alarm = False
for i in range(60):
    world.theta = 0.08 * i
    alarm |= h.check(world.scan(i * SCAN_S), None)
check("lift: turning without measurement no alarm", not alarm, f"last share {h.share:.2f}")
h = aw.LiftDetector()
h.check(world.scan(9.9), None)  # one calm scan before
world.tilted = True
detected = [h.check(world.scan(10 + i * SCAN_S), None) for i in range(4)]
check("lift: tilt detected", detected[-1], str(detected))
world.tilted = False
still = [h.check(world.scan(11 + i * SCAN_S), None) for i in range(int(config.LIFT_REST_S / SCAN_S) + 3)]
check("lift: calm again -> over", still[0] and not still[-1], f"{sum(still)} scans still lifted")

# --- 5. Explorer: drive around an obstacle in an arc without turning back --------
for measuring in (True, False):
    name = "with turn rate" if measuring else "without turn rate"
    random.seed(5)
    world = World(room=(-1.0, 6.0, -2.0, 2.0), circles=[(1.4, 0.05, 0.20)])
    e, lidar = build(world, measuring)
    finished = [None]

    def observer(now, e, command):
        if finished[0] is None and world.x > 2.2:
            finished[0] = (now, world.theta)

    z, smallest, end = run_sim(world, e, lidar, 20.0, observer=observer, no_switch=True)
    states_seen = {s for t, s, _ in z if finished[0] is None or t <= finished[0][0]}
    check(f"detour ({name}): gets past", finished[0] is not None, f"x={world.x:.2f}")
    check(f"detour ({name}): without turning back", ex.TURNING not in states_seen and ex.UNSTICKING not in states_seen, str(states_seen))
    check(f"detour ({name}): no contact", smallest > ROBOT_R, f"smallest distance {smallest:.2f} m")
    if measuring:
        # With a heading it steers back to the old direction after the obstacle.
        check("detour (with turn rate): back on course afterwards",
               abs(math.degrees(world.theta)) < 15, f"facing {math.degrees(world.theta):.0f}° at x={world.x:.2f}")

# --- 6. Explorer: corridor ------------------------------------------------------
random.seed(6)
world = World(room=(-1.0, 7.0, -0.38, 0.38), x=0.0, y=0.08, theta=math.radians(8))
e, lidar = build(world)
z, smallest, end = run_sim(world, e, lidar, 40.0, no_switch=True,
                            observer=lambda j, e, b: None)
states_seen = {s for _, s, _ in z}
far = max(0.0, world.x)
check("corridor: drives through", end is None and world.x > 5.0 or ex.TURNING in states_seen, f"x={world.x:.2f}, {states_seen}")
check("corridor: no contact", smallest > ROBOT_R, f"smallest distance {smallest:.2f} m")

# --- 7. Explorer: drives through a door in the middle -----------------------------
# Two rooms with a wall at x = 0 between them and a 0.7 m wide door. It stands
# in front of it at an angle and offset; it must find its own way through.
for nr, (y0, th0) in enumerate(((0.25, -0.15), (-0.2, 0.2), (0.0, 0.0))):
    random.seed(70 + nr)
    world = World(room=(-2.5, 2.5, -1.5, 1.5),
                walls=[((0.0, -1.5), (0.0, -0.35)), ((0.0, 0.35), (0.0, 1.5))],
                x=-1.2, y=y0, theta=th0)
    e, lidar = build(world)
    in_doorway = []

    def observer(now, e, command):
        if abs(world.x) < 0.05:
            in_doorway.append(world.y)

    z, smallest, end = run_sim(world, e, lidar, 15.0, observer=observer, no_switch=True)
    states_seen = {s for _, s, _ in z}
    check(f"door {nr}: drives through", world.x > 0.5, f"x={world.x:.2f}, {states_seen}")
    check(f"door {nr}: no contact", smallest > ROBOT_R, f"smallest distance {smallest:.2f} m")
    if in_doorway:
        check(f"door {nr}: centred", max(abs(y) for y in in_doorway) < 0.35 - ROBOT_R,
               f"offset in the door up to {max(abs(y) for y in in_doorway):.2f} m")

# No preset, from the corner: it should find its way through eventually,
# without bumping into anything and without giving up (random; repeatable with a fixed seed).
passed = 0
for seed in range(4):
    random.seed(170 + seed)
    world = World(room=(-2.5, 2.5, -1.5, 1.5),
                walls=[((0.0, -1.5), (0.0, -0.35)), ((0.0, 0.35), (0.0, 1.5))],
                x=-1.8, y=random.uniform(-0.8, 0.8), theta=random.uniform(-3.0, 3.0))
    e, lidar = build(world)
    across = [False]

    def observer(now, e, command):
        across[0] |= world.x > 0.4

    z, smallest, end = run_sim(world, e, lidar, 120.0, observer=observer)
    passed += across[0]
    check(f"door free (run {seed}): no contact, stays active",
           smallest > ROBOT_R and end is None, f"smallest distance {smallest:.2f} m, {e.last_reason}")
check("door free: finds its way through at least once", passed >= 1, f"{passed} of 4")

# --- 8. Explorer: lifted and set down somewhere else ------------------------------
random.seed(8)
world = World(room=(-2.0, 2.0, -2.0, 2.0), circles=[(1.0, 0.8, 0.3)])
e, lidar = build(world)
log = []


def carry(now, e, command):
    log.append((now, e._state, command))
    if abs(now - 5.0) < TICK / 2:
        world.tilted = True
        world.carried = (0.3, 0.2, 0.8)
    if abs(now - 8.0) < TICK / 2:
        # Set down in the neighbouring room.
        world.tilted = False
        world.carried = None
        world.room = (-1.0, 4.0, -1.0, 1.2)
        world.circles = [(2.5, -0.3, 0.25)]
        world.x, world.y, world.theta = 1.0, 0.0, 2.0


z, smallest, end = run_sim(world, e, lidar, 16.0, observer=carry)
high = [t for t, s, _ in log if s == ex.LIFTED]
check("lift: detected", bool(high) and high[0] < 5.6, f"from {high[0]:.2f} s" if high else "never")
check("lift: stands still while lifted",
       all(b is not None and b.vx == 0 and b.vy == 0 and b.omega == 0 for t, s, b in log if s == ex.LIFTED))
later = [(t, s) for t, s, _ in log if t > 8.0]
resumed = next((t for t, s in later if s != ex.LIFTED), None)
check("lift: drives again after being set down",
       resumed is not None and resumed < 8.0 + config.LIFT_REST_S + 1.0 and end is None,
       f"resumed at {resumed}" if resumed else e.last_reason)
check("lift: no unstick/stuck caused by carrying",
       ex.UNSTICKING not in {s for t, s in later[:40]} and end is None)

# --- 9. No false alarm in the room ------------------------------------------------
for measuring in (True, False):
    name = "with turn rate" if measuring else "without turn rate"
    random.seed(9)
    world = World(room=(-2.0, 2.0, -1.5, 1.5),
                circles=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
                x=-0.2)
    e, lidar = build(world, measuring)
    z, smallest, end = run_sim(world, e, lidar, 90.0)
    states_seen = {s for _, s, _ in z}
    check(f"room ({name}): never lifted", ex.LIFTED not in states_seen, str(states_seen))
    check(f"room ({name}): no contact", smallest > ROBOT_R, f"smallest distance {smallest:.2f} m")
    check(f"room ({name}): drives the whole time", end is None, e.last_reason)

print()
print("all cases pass" if error == 0 else f"{error} error(s)")
sys.exit(1 if error else 0)
