"""Checks driving with a map -- without robot and without ROS.

    python Marlon/autonomous_driving/test_map.py

The imagined apartment is the one from the simulation with a partition wall
(``MockLidar``, ``MOCK_APARTMENT``): two rooms, door at x = 1.0 m,
y = 1.4 to 2.3 m, a pillar in the right room.

Cases:

* Map: distance field, saving as PGM + YAML and loading again;
* Localization: a shifted pose is pulled back, the search finds the pose in
  the surrounding area, a wrong area is not "found";
* Planning: the path into the neighboring room leads through the door and
  keeps its distance; a closed door means no path; a target in the wall is moved;
* Frontiers: if the neighboring room is unknown, the path leads to the door;
  if everything is known, there is no frontier left;
* Path follower: drives into the neighboring room in the imagined world,
  touches nothing;
* Mapping drive: when the room is done it does not end but drives to the
  frontier; blocked frontiers are skipped; at the end "Karte vollständig".
"""

import math
import pathlib
import sys
import tempfile
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402

config.MOCK_APARTMENT = True

from rosmaster_assist.mapping.occ_map import OccupancyMap  # noqa: E402
from rosmaster_assist.mapping.map_service import MapService  # noqa: E402
from rosmaster_assist.navigation import map_localization as ko  # noqa: E402
from rosmaster_assist.navigation import mapping_drive as kf  # noqa: E402
from rosmaster_assist.navigation import path_follower as pf  # noqa: E402
from rosmaster_assist.navigation import path_planning as pp  # noqa: E402
from rosmaster_assist.sensors.lidar import MockLidar  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FAIL  "), name, info)
    error += not condition


# --- Imagined apartment ------------------------------------------------------------

LIDAR = MockLidar()
X_MIN, X_MAX, Y_MIN, Y_MAX = MockLidar.ROOM
DOOR = (1.4, 2.3)


def _segment_distance(x, y, segment):
    x1, y1, x2, y2 = segment
    ex, ey = x2 - x1, y2 - y1
    t = np.clip(((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
    return np.hypot(x - x1 - t * ex, y - y1 - t * ey)


def world_distance(x, y, door_open=True):
    """Distance of a point to the nearest wall (or nearest piece of furniture) of the apartment.

    The same segments as in the simulated lidar (``MockLidar.APARTMENT_WALLS``).
    """
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    d = np.minimum.reduce([x - X_MIN, X_MAX - x, y - Y_MIN, Y_MAX - y])
    segments = list(MockLidar.APARTMENT_WALLS)
    if not door_open:
        segments.append((1.0, DOOR[0], 1.0, DOOR[1]))
    for s in segments:
        d = np.minimum(d, _segment_distance(x, y, s))
    cx, cy, r = MockLidar.PILLAR
    return np.minimum(d, np.hypot(x - cx, y - cy) - r)


def apartment(resolution=0.05, door_open=True, left_only=False):
    """Map of the apartment; ``left_only``: the right room is unknown."""
    origin = (X_MIN - 0.5, Y_MIN - 0.5)
    width = int((X_MAX - X_MIN + 1.0) / resolution)
    height = int((Y_MAX - Y_MIN + 1.0) / resolution)
    iy, ix = np.mgrid[0:height, 0:width]
    x = origin[0] + (ix + 0.5) * resolution
    y = origin[1] + (iy + 0.5) * resolution
    inside = (x > X_MIN) & (x < X_MAX) & (y > Y_MIN) & (y < Y_MAX)
    d = world_distance(x, y, door_open)
    occupancy = np.full((height, width), -1, dtype=np.int8)
    occupancy[inside & (d > resolution)] = 0
    occupancy[(np.abs(d) <= resolution) & (x > X_MIN - resolution) & (x < X_MAX + resolution)
             & (y > Y_MIN - resolution) & (y < Y_MAX + resolution)] = 100
    if left_only:
        # Right room unknown, only a bit behind the door has been seen.
        right = x > 1.0 + resolution
        behind_door = (x < 1.6) & (y > DOOR[0]) & (y < DOOR[1])
        occupancy[right & ~behind_door] = -1
    return OccupancyMap(occupancy, resolution, origin)


def scan_at(x, y, theta, step_deg=1.0):
    """What the simulated lidar would see at this pose, in the robot frame."""
    pts = []
    for i in range(int(360 / step_deg)):
        a = math.radians(i * step_deg)
        r = LIDAR._ray(x, y, a + theta)
        if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
            pts.append((r * math.cos(a), r * math.sin(a)))
    return np.asarray(pts)


# --- Map -----------------------------------------------------------------------------

occ_map = apartment()
d_center = occ_map.distance_at(np.array([-1.0]), np.array([0.0]))[0]
check("distance field: room center far away (capped)", d_center >= config.NAV_DISTANCE_MAX_M - 0.01,
       f"{d_center:.2f} m")
d_wall = occ_map.distance_at(np.array([-2.7]), np.array([0.0]))[0]
check("distance field: 0.3 m in front of the wall", abs(d_wall - 0.3) < 0.08, f"{d_wall:.2f} m")

with tempfile.TemporaryDirectory() as folder:
    service = MapService(folder)
    info = service.save_map(occ_map.as_payload(), "test")
    loaded = OccupancyMap.load(info["yaml"])
    same = (
        np.array_equal(loaded.occupied, occ_map.occupied)
        and np.array_equal(loaded.free, occ_map.free)
        and np.array_equal(loaded.unknown, occ_map.unknown)
    )
    check("save and load: same map", same)
    check("save and load: origin and resolution",
           np.allclose(loaded.origin, occ_map.origin, atol=1e-6)
           and abs(loaded.resolution - occ_map.resolution) < 1e-9)

# --- Localization ----------------------------------------------------------------------

truth = (-1.2, -0.8, math.radians(30.0))
points = scan_at(*truth)
start = (truth[0] + 0.2, truth[1] - 0.15, truth[2] + math.radians(8.0))
a = ko.refine(occ_map, points, start)
check("localization: shifted pose is pulled back",
       math.hypot(a.x - truth[0], a.y - truth[1]) < 0.05
       and abs(math.degrees(ko._normalize(a.theta - truth[2]))) < 2.0,
       f"({a.x:.2f}, {a.y:.2f}, {math.degrees(a.theta):.1f}°), {100 * a.share:.0f} % hits")

a = ko.search(occ_map, points, (truth[0] + 0.8, truth[1] + 0.6), 1.5)
check("localization: search in the surrounding area finds the pose",
       a is not None and math.hypot(a.x - truth[0], a.y - truth[1]) < 0.1
       and abs(math.degrees(ko._normalize(a.theta - truth[2]))) < 3.0,
       "nothing" if a is None else f"({a.x:.2f}, {a.y:.2f}, {math.degrees(a.theta):.1f}°)")

truth_r = (3.2, -1.5, math.radians(-100.0))
a = ko.search(occ_map, scan_at(*truth_r), None, 0.0)
check("localization: search without hint across the whole map",
       a is not None and math.hypot(a.x - truth_r[0], a.y - truth_r[1]) < 0.15,
       "nothing" if a is None else f"({a.x:.2f}, {a.y:.2f})")

# Scan from the right room (at the pillar), searched in the left one: must
# not match. (Without the pillar both rooms would be identical -- no
# localization can tell two identical rectangles apart, only a hint helps then.)
a = ko.search(occ_map, scan_at(2.6, 1.2, 1.0), (-1.0, 0.5), 1.5)
check("localization: wrong area is not 'found'", a is None,
       "" if a is None else f"supposedly ({a.x:.2f}, {a.y:.2f}), {100 * a.share:.0f} %")

# In the middle of the (almost) symmetric left room the opposite heading fits too:
# without a heading hint better ask back, with a hint it is unambiguous.
room_center = (-1.0, 0.0, 0.0)
a = ko.search(occ_map, scan_at(*room_center), (-1.0, 0.0), 1.5)
check("localization: ambiguous (room center, no heading) -- no wrong pose",
       a is None or abs(math.degrees(ko._normalize(a.theta))) < 5.0,
       "ask back" if a is None else f"{math.degrees(a.theta):.0f}°")
a = ko.search(occ_map, scan_at(*room_center), (-1.0, 0.0), 1.5, angle=math.radians(10.0))
check("localization: unambiguous with heading hint",
       a is not None and abs(math.degrees(ko._normalize(a.theta))) < 3.0,
       "nothing" if a is None else f"{math.degrees(a.theta):.1f}°")


class FakeOdo:
    """Lidar odometry with drift: reports the true pose, slightly skewed."""

    def __init__(self):
        self.run = 1
        self.running = True
        self.nr = 0
        self.truth = (-1.0, 0.0, 0.0)
        self.drift = (0.0, 0.0, 0.0)

    def start(self, user="x"):
        pass

    def stop(self, user="x"):
        pass

    def _odo(self):
        return ko.compose(ko.invert(self.drift), self.truth)

    def pose(self):
        return self._odo()

    def measurement(self):
        scan = types.SimpleNamespace(points=lambda: scan_at(*self.truth))
        return types.SimpleNamespace(nr=self.nr, scan=scan, pose=self._odo())


odo = FakeOdo()
localization = ko.MapLocalization(odo)
localization.set_map(occ_map, start=(-1.0, 0.0, 0.0), live=False)
for _ in range(3):
    odo.nr += 1
    localization._last_search = 0.0
    localization.match_scan()
p = localization.pose()
check("map localization: loaded map, pose found at the starting point",
       localization.confident and p is not None and math.hypot(p[0] + 1.0, p[1]) < 0.08,
       localization.state()["text"])

# Let it drift: the odometry slowly skews, the localization counteracts.
for k in range(40):
    odo.truth = (-1.0 + 0.04 * k, 0.03 * k, math.radians(2.0 * k))
    odo.drift = (0.004 * k, -0.003 * k, math.radians(0.3 * k))
    odo.nr += 1
    localization.match_scan()
p = localization.pose()
check("map localization: odometry drift is compensated",
       p is not None and math.hypot(p[0] - odo.truth[0], p[1] - odo.truth[1]) < 0.08
       and abs(math.degrees(ko._normalize(p[2] - odo.truth[2]))) < 2.5,
       f"{math.hypot(p[0] - odo.truth[0], p[1] - odo.truth[1]):.3f} m off" if p else "no pose")

# --- Planning --------------------------------------------------------------------------


def through_door(path):
    """Does the path cross x = 1.0 within the door opening?"""
    pts = [(-1.5, -1.5)] + list(path.points)
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        if (x1 - 1.0) * (x2 - 1.0) < 0:
            t = (1.0 - x1) / (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not DOOR[0] + 0.1 < y < DOOR[1] - 0.1:
                return False
    return True


def smallest_distance(path, start):
    pts = [start] + list(path.points)
    xs, ys = [], []
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02))
        xs += list(np.linspace(x1, x2, n))
        ys += list(np.linspace(y1, y2, n))
    return float(world_distance(np.array(xs), np.array(ys)).min())


path = pp.plan_path(occ_map, (-1.5, -1.5), (3.5, -1.5))
check("planning: path into the neighboring room found", path is not None,
       "" if path is None else f"{path.length:.1f} m, {len(path.points)} waypoints")
if path is not None:
    check("planning: path leads through the door", through_door(path))
    distance = smallest_distance(path, (-1.5, -1.5))
    check("planning: path keeps distance to walls", distance >= config.NAV_SAFETY_M - 0.06,
           f"narrowest point {distance:.2f} m")

closed = apartment(door_open=False)
check("planning: closed door -- no path", pp.plan_path(closed, (-1.5, -1.5), (3.5, -1.5)) is None)

path = pp.plan_path(occ_map, (-1.5, -1.5), (-2.98, 0.0))
check("planning: target in the wall is moved to a drivable spot",
       path is not None and world_distance(np.array([path.target[0]]), np.array([path.target[1]]))[0]
       >= config.NAV_SAFETY_M - 0.06,
       "" if path is None else f"target ({path.target[0]:.2f}, {path.target[1]:.2f})")

# --- Frontiers -------------------------------------------------------------------------

half = apartment(left_only=True)
path = pp.find_frontier(half, (-1.5, -1.5))
check("frontiers: right room unknown -- path to the door",
       path is not None and path.gap is not None and abs(path.gap[0] - 1.4) < 0.6
       and DOOR[0] - 0.3 < path.gap[1] < DOOR[1] + 0.3,
       "none" if path is None else f"frontier at ({path.gap[0]:.2f}, {path.gap[1]:.2f})")
if path is not None:
    path2 = pp.find_frontier(half, (-1.5, -1.5), handled=[path.gap])
    check("frontiers: visited frontier no longer counts", path2 is None or
           math.hypot(path2.gap[0] - path.gap[0], path2.gap[1] - path.gap[1])
           >= config.FRONTIER_DONE_M)
check("frontiers: whole apartment known -- no frontier", pp.find_frontier(occ_map, (-1.5, -1.5)) is None)

# --- Path follower in the imagined world ----------------------------------------------


def simulate_drive(follower, pose, steps=4000, dt=0.05):
    """Simple kinematics without slip; returns (state, pose, narrowest distance)."""
    x, y, th = pose
    narrowest = float("inf")
    state = pf.DRIVING
    for _ in range(steps):
        command, state = follower.step((x, y, th), scan_at(x, y, th, 2.0))
        if state != pf.DRIVING:
            break
        center = th + command.omega * dt / 2
        x += (command.vx * math.cos(center) - command.vy * math.sin(center)) * dt
        y += (command.vx * math.sin(center) + command.vy * math.cos(center)) * dt
        th += command.omega * dt
        narrowest = min(narrowest, float(world_distance(np.array([x]), np.array([y]))[0]))
    return state, (x, y, th), narrowest


path = pp.plan_path(occ_map, (-1.5, -1.5), (3.5, -1.5))
follower = pf.PathFollower()
follower.apply(path.points, from_=(-1.5, -1.5), goal_angle=math.radians(90.0))
state, pose, narrowest = simulate_drive(follower, (-1.5, -1.5, 0.0))
check("path follower: arrives in the neighboring room", state == pf.ARRIVED
       and math.hypot(pose[0] - 3.5, pose[1] + 1.5) <= config.NAV_GOAL_TOLERANCE_M + 0.02,
       f"{state}, ({pose[0]:.2f}, {pose[1]:.2f})")
check("path follower: aligns itself at the goal",
       abs(math.degrees(ko._normalize(pose[2] - math.radians(90.0)))) <= config.NAV_GOAL_DEG + 1,
       f"{math.degrees(pose[2]):.0f}°")
check("path follower: touches nothing", narrowest > config.ROBOT_HALF_WIDTH_M + 0.03,
       f"narrowest point {narrowest:.2f} m")

# --- Mapping drive: onward into the unknown ---------------------------------------------


class InstantPlanner:
    """Planner without a thread: the result is ready right away (like a finished Future)."""

    def __init__(self, block_frontier=False):
        self.block_frontier = block_frontier
        self.jobs = []

    def _finish(self, value):
        return types.SimpleNamespace(done=lambda: True, result=lambda: value)

    def gap(self, occ_map, start, handled=(), blocked=()):
        self.jobs.append("luecke")
        return self._finish(pp.find_frontier(occ_map, start, handled))

    def path(self, occ_map, start, target, blocked=()):
        self.jobs.append("weg")
        return self._finish(pp.plan_path(occ_map, start, target))


class FakeLocalization:
    def __init__(self, occ_map, pose):
        self.occ_map = occ_map
        self.current_pose = pose
        self.start = (-1.5, -1.5, 0.0)
        self.confident = True

    def pose(self):
        return self.current_pose


class FakeScanOdo:
    def pose(self):
        return (0.0, 0.0, 0.0)

    def age(self):
        return 0.1


localization_a = FakeLocalization(half, (-2.0, -2.0, 0.0))
drive = kf.MappingDrive(FakeScanOdo(), localization=localization_a, planner=InstantPlanner())
drive.begin(0.0)
drive._room_done(0.0, "gegenüberliegende Wand erreicht")
check("mapping drive: room done -- it does not end", drive.reason is None
       and drive._step_name == kf.FRONTIER_SEARCH, drive.display())

scan = types.SimpleNamespace()
x, y, th = localization_a.current_pose
for t in range(3000):
    localization_a.current_pose = (x, y, th)
    command = drive.step(t * 0.05, scan, scan_at(x, y, th, 2.0))
    if command is None:
        break
    x += (command.vx * math.cos(th) - command.vy * math.sin(th)) * 0.05
    y += (command.vx * math.sin(th) + command.vy * math.cos(th)) * 0.05
    th += command.omega * 0.05
    if drive._frontiers_visited >= 1:
        break
check("mapping drive: drives to the frontier at the door", drive._frontiers_visited >= 1
       and x > 0.0 and DOOR[0] - 0.5 < y < DOOR[1] + 0.5,
       f"{drive.display()} at ({x:.2f}, {y:.2f})")

# If everything is known afterwards, "Karte vollständig" follows -- after the return drive.
localization_a.occ_map = occ_map
for t in range(3000):
    localization_a.current_pose = (x, y, th)
    command = drive.step(100 + t * 0.05, scan, scan_at(x, y, th, 2.0))
    if command is None:
        break
    x += (command.vx * math.cos(th) - command.vy * math.sin(th)) * 0.05
    y += (command.vx * math.sin(th) + command.vy * math.cos(th)) * 0.05
    th += command.omega * 0.05
check("mapping drive: at the end 'Karte vollständig' at the starting point",
       drive.reason is not None and "vollständig" in drive.reason
       and math.hypot(x - localization_a.start[0], y - localization_a.start[1]) < 0.4,
       f"{drive.reason} at ({x:.2f}, {y:.2f})")

# Obstacle while driving to the frontier: give up the frontier, not the drive.
localization_b = FakeLocalization(half, (-2.0, -2.0, 0.0))
drive = kf.MappingDrive(FakeScanOdo(), localization=localization_b, planner=InstantPlanner())
drive.begin(0.0)
drive._room_done(0.0, "Test")
drive.step(0.0, scan, scan_at(-2.0, -2.0, 0.0))   # searching
drive.step(0.05, scan, scan_at(-2.0, -2.0, 0.0))  # path received
resumed = drive.report_obstacle("Fahrschutz: Test")
check("mapping drive: stuck -- give up frontier, keep driving",
       resumed and drive.reason is None and drive._step_name == kf.FRONTIER_SEARCH
       and len(drive._done) == 1, drive.display())
for _ in range(config.FRONTIER_MAX_ERRORS + 1):
    drive._goal_frontier = (0.0, 0.0)
    drive._step_name = kf.TO_FRONTIER
    drive.report_obstacle("Test")
check("mapping drive: ends after too many failures", drive.reason is not None,
       str(drive.reason))

# Without a map (no SLAM) as before: end after the room.
drive = kf.MappingDrive(FakeScanOdo())
drive.begin(0.0)
drive._room_done(0.0, "gegenüberliegende Wand erreicht")
check("mapping drive without map: ends after the room as before",
       drive.reason == "gegenüberliegende Wand erreicht")

print()
print("All good." if error == 0 else f"{error} failed.")
sys.exit(1 if error else 0)
