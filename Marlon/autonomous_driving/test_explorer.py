"""Checks the Explorer in a small simulation -- without the robot.

    python Marlon/autonomous_driving/test_explorer.py

The test calls ``Explorer._step`` directly with a simulated clock. A world
of walls and round obstacles delivers lidar scans (rays computed, 7.5 Hz),
and the pose is integrated from the drive commands. This lets us recreate
cases that are hard to repeat on the floor: something comes very close, the
wheels spin, it is stuck.

Background: before 03.11. it stood still forever at near points, turned
only by time and gave up after six unsuccessful turns.
"""

import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import obstacles as hd  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0
TICK = 1.0 / config.ODOMETRY_HZ
SCAN_S = 1.0 / 7.5
#: Approximate radius of the robot around the lidar centre (body ~0.30 m wide).
ROBOT_R = 0.15


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


class World:
    """Rectangular room with round obstacles and a robot inside."""

    def __init__(self, room=(-3.0, 3.0, -3.0, 3.0), circles=(), x=0.0, y=0.0, theta=0.0):
        self.room = room
        self.circles = list(circles)  # (x, y, r)
        self.x, self.y, self.theta = x, y, theta
        self.blocked = False  # wheels spin / it is stuck
        self.turn_share = 1.0  # how much of the commanded rotation arrives
        self.v_actual = (0.0, 0.0, 0.0)  # in the robot frame

    def ray(self, direction):
        dx, dy = math.cos(direction), math.sin(direction)
        x0, x1, y0, y1 = self.room
        best = float("inf")
        for limit, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9:
                t = (limit - start) / d
                if t > 0:
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
        for i in range(360):
            r = self.ray(self.theta + math.radians(i))
            # Like the real lidar: nothing arrives below LIDAR_MIN_M.
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                angle.append(float(i if i <= 180 else i - 360))
                dist.append(r)
        s = Scan(angle, dist)
        s.timestamp = timestamp
        return s

    def move(self, command, dt):
        if command is None or self.blocked:
            self.v_actual = (0.0, 0.0, 0.0)
            return
        vx, vy, om = command.vx, command.vy, command.omega * self.turn_share
        self.v_actual = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def distance(self):
        """Smallest distance from the lidar centre to a wall or obstacle."""
        x0, x1, y0, y1 = self.room
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
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


def build(world, measuring=True, with_odometry=True):
    lidar = FakeLidar()
    odo = FakeOdometry(world, measuring) if with_odometry else None
    e = ex.Explorer(WatchdogStub(), lidar, hd.ObstacleDetection(), None, odometry=odo)
    return e, lidar


def run_sim(world, e, lidar, duration, t0=0.0, observer=None):
    """Simulates ``duration`` seconds. Returns (states, smallest distance, end)."""
    now = t0
    next_scan = t0
    states = []
    smallest = world.distance()
    if t0 == 0.0:
        lidar.last = world.scan(0.0)
        e._begin(0.0)
    while now < t0 + duration:
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
        smallest = min(smallest, world.distance())
        now += TICK
    return states, smallest, None


def names(states):
    return [z for _, z, _ in states]


# --- 1. Something comes very close: wait first, then push free ----------------
random.seed(1)
# Obstacle to the right of the robot, 0.18 m from the lidar centre.
world = World(circles=[(0.0, -0.48, 0.30)])
e, lidar = build(world)
z, smallest, end = run_sim(world, e, lidar, 8.0)
n = names(z)
first_unstuck = next((t for t, s, _ in z if s == ex.UNSTICKING), None)
check("Near: waits first", n[1] == ex.WAITING, n[:3])
check(
    "Near: pushes itself free after the wait time",
    first_unstuck is not None and abs(first_unstuck - config.EXPLORE_NEAR_WAIT_S) < 0.2,
    first_unstuck,
)
unstick_commands = [b for _, s, b in z if s == ex.UNSTICKING and b is not None]
check("Near: pushes without rotation", all(b.omega == 0.0 for b in unstick_commands))
check("Near: pushes away from the obstacle (to the left)", unstick_commands and unstick_commands[0].vy > 0, unstick_commands[:1])
check("Near: no longer stands still permanently", n[-1] != ex.WAITING and end is None, n[-1])
check("Near: afterwards at a distance", world.distance() > config.EXPLORE_NEAR_M, f"{world.distance():.2f} m")

# A person who only steps close briefly does not trigger unsticking.
random.seed(1)
world = World(circles=[(0.0, -0.48, 0.30)])
e, lidar = build(world)
z, _, _ = run_sim(world, e, lidar, 1.5)
world.circles.clear()
z2, _, _ = run_sim(world, e, lidar, 2.0, t0=1.5)
check("Near briefly: no unsticking", ex.UNSTICKING not in names(z + z2))
check("Near briefly: drives on afterwards", z2[-1][1] == ex.DRIVING, z2[-1][1])

# --- 2. Stuck: the board reports standstill -------------------------------------
random.seed(2)
world = World(room=(-3.0, 2.0, -3.0, 3.0))  # wall 2 m ahead
world.blocked = True
e, lidar = build(world)
z, _, end = run_sim(world, e, lidar, 3.0)
first_unstuck = next(((t, b) for t, s, b in z if s == ex.UNSTICKING), None)
check(
    "Stuck (board): detected after EXPLORE_STUCK_S",
    first_unstuck is not None and first_unstuck[0] < config.EXPLORE_STUCK_S + 0.5,
    first_unstuck and round(first_unstuck[0], 2),
)
check(
    "Stuck (board): does not unstick forwards",
    first_unstuck is not None and abs(e._unstick_direction) >= 90,
    e._unstick_direction,
)
check("Stuck (board): detection before odometry is discarded (3 s)", config.EXPLORE_STUCK_S < 3.0)

# If it stays blocked, it gives up after EXPLORE_UNSTICK_MAX attempts.
z, _, end = run_sim(world, e, lidar, 40.0, t0=3.0)
check("Stuck (board): gives up eventually", end is not None, e.last_reason)
check("Stuck (board): reason mentions festgefahren", "festgefahren" in (e.last_reason or ""), e.last_reason)
check("Stuck (board): number of unstick attempts", e._unstuck_total == config.EXPLORE_UNSTICK_MAX, e._unstuck_total)

# --- 3. Stuck without board measurement: only the lidar notices ----------------
random.seed(3)
world = World(room=(-3.0, 2.0, -3.0, 3.0))
world.blocked = True
e, lidar = build(world, with_odometry=False)
z, _, _ = run_sim(world, e, lidar, 3.0)
first_unstuck = next((t for t, s, _ in z if s == ex.UNSTICKING), None)
check(
    "Stuck (lidar): distance does not shrink -> unstick",
    first_unstuck is not None and first_unstuck < config.EXPLORE_STUCK_S + 0.5,
    first_unstuck,
)

# If it really drives, the detection does not trigger (even without the board).
random.seed(3)
world = World(room=(-3.0, 2.0, -3.0, 3.0), x=-2.5)
e, lidar = build(world, with_odometry=False)
z, _, _ = run_sim(world, e, lidar, 6.0)
check("Stuck (lidar): no false alarm when driving freely", ex.UNSTICKING not in names(z), names(z)[-1])

# Room without random direction changes: it often drives diagonally towards
# walls and steers along them. The distance ahead stays almost the same --
# the detection used to take that for being stuck (4-5 false alarms in 2 minutes).
old_min, old_max = config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S
config.EXPLORE_WANDER_MIN_S = config.EXPLORE_WANDER_MAX_S = 1e9
for measuring in (True, False):
    random.seed(3)
    world = World(
        room=(-2.5, 2.5, -2.0, 2.0),
        circles=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
        x=-0.2,
    )
    world.turn_share = 0.8
    e, lidar = build(world, measuring=measuring)
    run_sim(world, e, lidar, 120.0)
    check(
        f"Stuck: no false alarm at walls ({'with' if measuring else 'without'} board measurement)",
        e._unstuck_total == 0,
        f"{e._unstuck_total}x unstuck, {e._avoidances}x avoided",
    )
config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S = old_min, old_max

# Large room as in the MockLidar, start in the middle: the walls in the
# driving direction are farther away than OBSTACLE_RANGE_M, the side walls do
# not change. If only rays within range counted, it reported being stuck
# after 1.5 s (noticed like that in the mock).
random.seed(3)
world = World(room=(-3.0, 5.0, -3.0, 3.0), circles=[(2.0, 1.2, 0.35)])
e, lidar = build(world, with_odometry=False)
run_sim(world, e, lidar, 4.0)
check("Stuck: no false alarm in the large room", e._unstuck_total == 0, f"{e._unstuck_total}x unstuck")

# --- 4. Turning by measured angle ---------------------------------------------
def measure_rotation(measuring, with_odometry=True, turn_share=0.6, seed=4):
    """Wall just ahead -> it turns. Returns (target, actually turned)."""
    random.seed(seed)
    world = World(room=(-3.0, 0.35, -3.0, 3.0))
    world.turn_share = turn_share
    e, lidar = build(world, measuring=measuring, with_odometry=with_odometry)
    result = {}

    def observer(now, e, command):
        if e._state == ex.TURNING and "ziel" not in result:
            result["ziel"] = e._turn_direction
            result["start"] = world.theta
        if "ziel" in result and e._turn_done is not None and "ende" not in result:
            result["ende"] = world.theta
            result["fertig"] = e._turn_done
        if "fertig" in result and command is not None and command.vx > 0 and "los" not in result:
            result["los"] = now
            result["scan_beginn"] = lidar.last.timestamp - lidar.last.duration

    run_sim(world, e, lidar, 12.0, observer=observer)
    turned = math.degrees(result.get("ende", 0.0) - result.get("start", 0.0))
    return result, turned


res, turned = measure_rotation(measuring=True)
error_deg = abs(turned - res["ziel"])
check(
    "Angle: on target despite 60 % rotation effect",
    error_deg <= config.EXPLORE_TURN_LEAD_DEG + 3,
    f"target {res['ziel']:+.0f}°, turned {turned:+.0f}°",
)
res_time, turned_time = measure_rotation(measuring=False)
check(
    "Angle: without measurement by time (fallback, inaccurate)",
    abs(turned_time) < abs(res_time["ziel"]) * 0.8,
    f"target {res_time['ziel']:+.0f}°, turned {turned_time:+.0f}°",
)
res_without, turned_without = measure_rotation(measuring=True, with_odometry=False)
check("Angle: turns even without odometry", abs(turned_without) > 20, f"{turned_without:+.0f}°")

# If it does not turn at all (wedged in), the rotation ends via the time limit.
random.seed(4)
world = World(room=(-3.0, 0.35, -3.0, 3.0))
world.turn_share = 0.0
e, lidar = build(world)
first_end = []
run_sim(world, e, lidar, 15.0, observer=lambda t, e, b: (
    first_end.append((t, e._turn_direction)) if e._turn_done is not None and not first_end else None))
limit = (
    abs(math.radians(first_end[0][1])) / config.EXPLORE_TURN_SPEED * config.EXPLORE_TURN_TIME_FACTOR
    if first_end else None
)
check(
    "Angle: time limit when it does not turn",
    bool(first_end) and abs(first_end[0][0] - limit) < 0.1,
    first_end and f"end after {first_end[0][0]:.2f} s, limit {limit:.2f} s",
)

# --- 5. After the rotation, a fresh lidar scan first ---------------------------
check(
    "Fresh scan: only drives with a scan taken after the rotation",
    "los" in res and res["scan_beginn"] >= res["fertig"] + config.EXPLORE_COAST_S - 1e-9,
    {k: round(v, 2) for k, v in res.items() if isinstance(v, float)},
)

# --- 6. Closed in all around: no room to unstick -> gives up after N attempts --
random.seed(6)
world = World(room=(-0.20, 0.20, -0.20, 0.20))
e, lidar = build(world)
z, _, end = run_sim(world, e, lidar, 30.0)
check("Wedged in: does not move", all(b is None or (b.vx, b.vy, b.omega) == (0, 0, 0) for _, _, b in z))
check("Wedged in: ends with a reason", end is not None and "festgefahren" in (e.last_reason or ""), e.last_reason)

# --- 7. Free exploring in a room: no collision, no giving up -------------------
for measuring, name in ((True, "with turn rate"), (False, "by time")):
    random.seed(7)
    world = World(
        room=(-2.5, 2.5, -2.0, 2.0),
        circles=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
        x=-0.2, y=0.0,
    )
    world.turn_share = 0.8
    e, lidar = build(world, measuring=measuring)
    segment = [0.0]
    last = [world.x, world.y]

    def update(now, e, command):
        segment[0] += math.hypot(world.x - last[0], world.y - last[1])
        last[:] = [world.x, world.y]

    z, smallest, end = run_sim(world, e, lidar, 120.0, observer=update)
    check(f"Room ({name}): drives the whole time", end is None, e.last_reason)
    check(f"Room ({name}): no contact", smallest > ROBOT_R, f"smallest distance {smallest:.2f} m")
    check(f"Room ({name}): gets around", segment[0] > 8.0, f"{segment[0]:.1f} m, {e._avoidances}x avoided")

print()
print("all cases pass" if error == 0 else f"{error} errors")
sys.exit(1 if error else 0)
