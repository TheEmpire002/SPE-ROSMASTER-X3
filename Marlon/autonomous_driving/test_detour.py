"""Checks the detour while following and while homing (F3) -- without a robot.

    python Marlon/autonomous_driving/test_detour.py

Cases:

* ``choose_direction``: free path, avoiding to the side, door centre, nothing
  free, keeping the side;
* following with a chair between robot and person: slides past, touches
  nothing, reaches the following distance;
* following through a door that is not centred in front of it;
* lift while following: stops, READY afterwards;
* homing around an obstacle, lift during homing, and homing without a free
  path (message after the waiting time).

The simulated world comes from ``test_avoidance.py`` (rebuilt here, because
importing that file would run its tests).
"""

import asyncio
import math
import pathlib
import random
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaction import behavior as vh  # noqa: E402
from rosmaster_assist.navigation import avoidance as aw  # noqa: E402
from rosmaster_assist.navigation import homing as hm  # noqa: E402
from rosmaster_assist.navigation.explorer import free_in_direction  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

random.seed(3)
error = 0
INFINITY = float("inf")
# Lidar centre to outline: below this it would have touched something.
ROBOT_R = 0.15


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


# --- Simulated world (as in test_avoidance.py) -------------------------------------


class World:
    def __init__(self, room=(-3.0, 5.0, -3.0, 3.0), walls=(), circles=(), x=0.0, y=0.0, theta=0.0):
        self.room = room
        self.walls = list(walls)
        self.circles = list(circles)
        self.x, self.y, self.theta = x, y, theta
        self.v_actual = (0.0, 0.0, 0.0)
        self.tilted = False

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
        floor = random.uniform(0.25, 1.5) if self.tilted else None
        for i in range(360):
            r = self.ray(self.theta + math.radians(i))
            if floor is not None and (i < 90 or i > 270):
                r = min(r, floor)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                angle.append(float(i if i <= 180 else i - 360))
                dist.append(r)
        s = Scan(angle, dist)
        s.timestamp = timestamp
        return s

    def move(self, command, dt):
        if command is None or self.tilted:
            self.v_actual = (0.0, 0.0, 0.0)
            return
        vx, vy, om = command.vx, command.vy, command.omega
        self.v_actual = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def distance(self, without_circles=()):
        x0, x1, y0, y1 = self.room
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
        for (ax, ay), (bx, by) in self.walls:
            ex_, ey = bx - ax, by - ay
            u = max(0.0, min(1.0, ((self.x - ax) * ex_ + (self.y - ay) * ey) / (ex_ * ex_ + ey * ey)))
            d = min(d, math.hypot(self.x - ax - u * ex_, self.y - ay - u * ey))
        for i, (cx, cy, r) in enumerate(self.circles):
            if i not in without_circles:
                d = min(d, math.hypot(self.x - cx, self.y - cy) - r)
        return d


def box(x0, x1, y0, y1):
    """Four wall segments around a rectangle (chair, box)."""
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class FakeOdometry:
    def __init__(self, world):
        self.world = world
        actually_measuring = True

    actually_measuring = True

    def velocity(self):
        return self.world.v_actual


# --- choose_direction ----------------------------------------------------------


def wall_points(a, b, step=0.02):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return np.array([(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)])


empty = np.zeros((0, 2))
width = config.EXPLORE_HALF_WIDTH_M
w = aw.choose_direction(empty, 0.0, 1.0, width)
check("direction: free path stays straight ahead", w is not None and w[0] == 0.0, w)

chair = wall_points((0.8, -0.15), (0.8, 0.25))
w = aw.choose_direction(chair, 0.0, 1.2, width)
check("direction: chair ahead -> to the side", w is not None and abs(w[0]) >= 15, w)
check("direction: avoids towards the freer side (right)", w is not None and w[0] < 0, w)
if w is not None:
    check("direction: chosen path is really free",
           free_in_direction(chair, w[0], width + config.AVOID_MARGIN_M) >= 1.2, w)

# Door at 1 m, gap y -0.1..0.6: centre at ~14°. The posts must stay outside
# the driving corridor.
door = np.vstack([wall_points((1.0, -2.0), (1.0, -0.1)), wall_points((1.0, 0.6), (1.0, 2.0))])
w = aw.choose_direction(door, 0.0, 2.0, width)
check("direction: door found", w is not None and 5 <= w[0] <= 22, w)
if w is not None:
    a = math.radians(w[0])
    lateral = [abs(-px * math.sin(a) + py * math.cos(a)) for px, py in ((1.0, -0.1), (1.0, 0.6))]
    check("direction: both posts outside the driving corridor", min(lateral) > width, [round(q, 2) for q in lateral])

# Diagonally past a finite wall 1 m may well be free -- so require 2 m.
closed = np.vstack([wall_points((0.5, -2.0), (0.5, 2.0))])
check("direction: wall across in front -> None", aw.choose_direction(closed, 0.0, 2.0, width) is None)

# Centred in front of a symmetric obstacle: the last chosen side wins.
center = wall_points((0.8, -0.2), (0.8, 0.2))
left = aw.choose_direction(center, 0.0, 1.2, width, last_side=1.0)
right = aw.choose_direction(center, 0.0, 1.2, width, last_side=-1.0)
check("direction: side is kept (no oscillation)",
       left is not None and right is not None and left[0] > 0 > right[0], (left, right))


# --- Following in the simulated world ---------------------------------------------

FOV = config.CAMERA_FOV_DEG
ASPECT = 4 / 3
RED = [0.2 / 127] * 128
RED[3] = 0.8


def person(angle, timestamp, distance):
    """Person at camera angle ``angle``; box width matching the distance."""
    mx = 0.5 - angle / FOV
    sx = math.degrees(math.atan2(0.22, distance)) / FOV
    sy = 0.3
    e = sx * ASPECT

    def image(u, v):
        return [mx + u * sx, sy + v * e, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = image(0.0, -0.7)
    kp[5], kp[6] = image(0.5, 0.0), image(-0.5, 0.0)
    kp[9], kp[10] = image(0.6, 1.4), image(-0.6, 1.4)
    kp[11], kp[12] = image(0.35, 1.8), image(-0.35, 1.8)
    return {"box": [mx - sx, sy - 0.1, mx + sx, sy + 0.6], "kp": kp, "score": 0.9, "winkel": angle,
            "seitenverhaeltnis": ASPECT, "merkmal": RED, "zeit": timestamp}


class Watchdog:
    def submit(self, cmd, source="hand"):
        pass

    def stop(self):
        pass


class Lidar:
    def __init__(self):
        self.last = None

    def latest(self):
        return self.last


class Pose:
    available = True
    fov_deg = FOV

    def __init__(self):
        self.entries = []

    def people(self):
        return self.entries

    def activate(self, an):
        pass


def legs(px, py):
    """A person for the lidar: two legs."""
    return [(px, py + 0.1, 0.06), (px, py - 0.1, 0.06)]


def follow(world, px, py, duration, hz=10.0, event=None):
    """Simulate following. Returns (interaction, history, smallest distance).

    The person (legs at px/py) is the last two circles of the world --
    they do not count as an obstacle for the minimum distance.
    """
    world.circles += legs(px, py)
    people_idx = (len(world.circles) - 2, len(world.circles) - 1)
    po, li = Pose(), Lidar()
    ia = vh.Interaction(Watchdog(), li, po, odometry=FakeOdometry(world))
    ia._mode, ia._mode_since, ia._seen = vh.READY, 0.0, 0.0
    history = []
    smallest = world.distance(people_idx)
    started = False
    for i in range(int(duration * hz)):
        t = i / hz
        if event:
            event(t, world)
        dx, dy = px - world.x, py - world.y
        d = math.hypot(dx, dy)
        angle = math.degrees(aw._normalize_angle(math.atan2(dy, dx) - world.theta))
        po.entries = [person(angle, t, d)] if abs(angle) < FOV / 2 else []
        li.last = world.scan(t)
        if not started:
            ia._switch(vh.FOLLOW, "Test", t)
            started = True
        command = ia._step(t)
        history.append((t, ia._mode, command, ia._text, d))
        world.move(command, 1.0 / hz)
        smallest = min(smallest, world.distance(people_idx))
    return ia, history, smallest


# Chair (0.3 x 0.3 m) half in the way, person 2.6 m straight ahead behind it.
world = World(walls=box(0.9, 1.2, -0.35, -0.05))
ia, history, smallest = follow(world, 2.6, 0.0, 30.0)
diagonal = [v for v in history if v[2] is not None and abs(v[2].vy) > 0.02]
texts = {v[3].split(" (")[0] for v in history}
check("follow/chair: slides past diagonally (vy)", len(diagonal) > 5, len(diagonal))
check("follow/chair: reports 'umfährt Hindernis'", "umfährt Hindernis" in texts, sorted(texts))
check("follow/chair: touches nothing", smallest > ROBOT_R, f"{smallest:.2f} m")
check("follow/chair: reaches following distance (~1 m)", abs(history[-1][4] - config.FOLLOW_DISTANCE_M) < 0.3,
       f"{history[-1][4]:.2f} m, x={world.x:.2f}")
check("follow/chair: still following", ia._mode == vh.FOLLOW, ia._mode)

# Door not centred: wall at x = 1.5, gap y -0.1..0.6; person behind it.
world = World(walls=[((1.5, -3.0), (1.5, -0.1)), ((1.5, 0.6), (1.5, 3.0))])
ia, history, smallest = follow(world, 3.2, 0.1, 30.0)
check("follow/door: is through", world.x > 1.6, f"x={world.x:.2f}, y={world.y:.2f}")
check("follow/door: touches no post", smallest > ROBOT_R, f"{smallest:.2f} m")
check("follow/door: reaches following distance (~1 m)", abs(history[-1][4] - config.FOLLOW_DISTANCE_M) < 0.3,
       f"{history[-1][4]:.2f} m")


# Lift: after 6 s tilted for 2 s, then set down.
def tilt(t, world):
    world.tilted = 6.0 <= t < 8.0


world = World()
ia, history, _ = follow(world, 3.0, 0.0, 14.0, event=tilt)
tilted = [v for v in history if 6.3 <= v[0] < 8.0]
check("follow/lift: drives before", any(v[2] is not None and v[2].vx > 0 for v in history if v[0] < 6.0))
check("follow/lift: stands still while being held",
       all(v[2] is None or (v[2].vx == 0 and v[2].vy == 0 and v[2].omega == 0) for v in tilted),
       [v[3] for v in tilted][:3])
check("follow/lift: text 'hochgehoben'", any(v[3].startswith("hochgehoben") for v in tilted))
check("follow/lift: READY afterwards (does not drive on by itself)", ia._mode == vh.READY,
       f"{ia._mode}, {ia._text}")


# --- Homing in the simulated world ----------------------------------------------


class HomeOdometry(FakeOdometry):
    """Pose relative to the start (world origin), angle in degrees."""

    def pose(self):
        return self.world.x, self.world.y, math.degrees(self.world.theta)


class HomeLocalizer:
    def __init__(self, world, clock):
        self.world, self.clock = world, clock

    def scan(self):
        return self.world.scan(self.clock[0])

    def obstacle_ahead(self, direction):
        return free_in_direction(self.scan().points(), direction, config.EXPLORE_HALF_WIDTH_M)


class HomeWatchdog:
    def __init__(self):
        self.command = None
        self.commands = []

    def submit(self, cmd, source="hand"):
        self.command = cmd
        self.commands.append(cmd)

    def stop(self):
        self.command = None


def homing_drive(world, event=None):
    """``Homing._drive`` with a simulated clock. Returns (reason, commands, smallest distance)."""
    clock = [0.0]
    wd = HomeWatchdog()
    smallest = [world.distance()]

    async def fake_sleep(dt):
        world.move(wd.command, dt)
        clock[0] += dt
        smallest[0] = min(smallest[0], world.distance())
        if event:
            event(clock[0], world)

    old_asyncio, old_time = hm.asyncio, hm.time
    hm.asyncio = types.SimpleNamespace(sleep=fake_sleep, CancelledError=asyncio.CancelledError,
                                       Task=asyncio.Task, create_task=asyncio.create_task)
    hm.time = types.SimpleNamespace(monotonic=lambda: clock[0])
    try:
        h = hm.Homing(wd, HomeOdometry(world), HomeLocalizer(world, clock))
        asyncio.run(h._drive())
    finally:
        hm.asyncio, hm.time = old_asyncio, old_time
    return h.last_reason, wd.commands, smallest[0]


# Box on the line between robot (3, 0) and start.
world = World(walls=box(1.3, 1.7, -0.25, 0.15), x=3.0, y=0.0, theta=0.0)
reason, commands, smallest = homing_drive(world)
check("homing/box: arrived", reason is not None and reason.startswith("angekommen"), reason)
check("homing/box: avoids sideways", any(abs(b.vy) > 0.05 for b in commands))
check("homing/box: touches nothing", smallest > ROBOT_R, f"{smallest:.2f} m")


def lift(t, world):
    world.tilted = t >= 2.0


world = World(x=3.0, y=0.0)
reason, commands, _ = homing_drive(world, event=lift)
check("homing/lift: aborts", reason is not None and reason.startswith("hochgehoben"), reason)

# Dead end: walls ahead and at the sides, free only to the rear.
world = World(walls=[((2.6, -0.6), (2.6, 0.6)), ((2.6, 0.45), (3.8, 0.45)), ((2.6, -0.45), (3.8, -0.45))],
            x=3.0, y=0.0)
reason, commands, smallest = homing_drive(world)
check("homing/dead end: stops with a message", reason is not None and "noch" in reason
       and ("Hindernis" in reason or "kein Weg" in reason), reason)
check("homing/dead end: touches nothing", smallest > ROBOT_R, f"{smallest:.2f} m")

print()
print("all cases pass" if not error else f"{error} error(s)")
sys.exit(1 if error else 0)
