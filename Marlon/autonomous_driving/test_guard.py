"""Checks the DriveGuard for autonomous driving and the follow distance -- without the robot.

    python Marlon/autonomous_driving/test_guard.py

Two parts:

* ``DriveGuard`` (safety/drive_guard.py): braking before the wall, stale
  scan, obstacle ahead/to the side/next to the wheel, turning next to the
  wall, integration in the watchdog only for ``source="autonom"``. Until
  03.11. this tested ``AutonomousGuard``, which has been merged into the DriveGuard;
* ``Interaction._distance_to`` (interaction/behavior.py): distance to the person
  from their lidar cluster -- a furniture leg in front does not count as the person.

Scans are generated like those of the RPLIDAR A1 (rays hit the nearest
object); rays without an echo are missing, as in the lidar service.
"""

import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.base import DriveBase  # noqa: E402
from rosmaster_assist.interaction import behavior as vh  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety import DriveGuard, SafetyWatchdog  # noqa: E402
from rosmaster_assist.safety.drive_guard import NO_LIDAR  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


# --- Synthetic scans -----------------------------------------------------------


def ray_circle(a, cx, cy, r):
    dx, dy = math.cos(a), math.sin(a)
    b = dx * cx + dy * cy
    d = b * b - (cx * cx + cy * cy - r * r)
    if d < 0:
        return None
    t = b - math.sqrt(d)
    return t if t > 0 else None


def ray_segment(a, x1, y1, x2, y2):
    dx, dy = math.cos(a), math.sin(a)
    ex, ey = x2 - x1, y2 - y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    t = (x1 * ey - y1 * ex) / denom
    u = (x1 * dy - y1 * dx) / denom
    return t if t > 0 and 0 <= u <= 1 else None


def scan(circles=(), segments=(), step=1.0, age=0.0):
    angle, dist = [], []
    w = 0.0
    while w < 360.0:
        a = math.radians(w)
        hits = [ray_circle(a, *k) for k in circles] + [ray_segment(a, *s) for s in segments]
        hits = [t for t in hits if t is not None]
        if hits:
            angle.append(w)
            dist.append(min(hits))
        w += step
    s = Scan(angle, dist)
    s.timestamp = time.monotonic() - age
    return s


class Lidar:
    def __init__(self, s=None):
        self.s = s

    def latest(self):
        return self.s


def wall_ahead(x):
    return scan(segments=[(x, -2.0, x, 2.0)])


# --- Part 1: DriveGuard check() ------------------------------------------------
# Footprint: ahead ROBOT_FRONT_M (0.15), to the side ROBOT_HALF_WIDTH_M
# (0.125) from the lidar centre; the edge keeps GUARD_MARGIN_M (0.04) clearance.

li = Lidar()
guard = DriveGuard(li)
STOP = DriveCommand(0.0, 0.0, 0.0)

li.s = None
p = guard.check(DriveCommand(0.2, 0.0, 0.5))
check("no scan: stands", p.command == STOP and p.reason == NO_LIDAR, p)

li.s = wall_ahead(3.0)
li.s.timestamp -= config.GUARD_SCAN_MAX_S + 0.2
p = guard.check(DriveCommand(0.2, 0.1, -0.3))
check("stale scan: stands", p.command == STOP and p.reason == NO_LIDAR, p)

li.s = scan(segments=[(5.0, -5.0, 5.0, 5.0)])
c = guard.check(DriveCommand(0.3, 0.0, 0.2)).command
check("free far away: unchanged", c == DriveCommand(0.3, 0.0, 0.2), c)

li.s = wall_ahead(0.8)
c = guard.check(DriveCommand(0.3, 0.0, 0.0)).command
check("wall 0.8 m ahead: 0.3 m/s still allowed", c == DriveCommand(0.3, 0.0, 0.0), c)

li.s = wall_ahead(0.3)
p = guard.check(DriveCommand(0.3, 0.0, 0.2))
c = p.command
check("wall 0.3 m ahead: braked, not stopped", 0.0 < c.vx < 0.3, p)
check("wall 0.3 m ahead: arc stays the same (everything scaled equally)",
       abs(c.omega / c.vx - 0.2 / 0.3) < 1e-9, c)

# Short board 2 cm in front of the edge (inside the margin), the sides are free.
li.s = scan(segments=[(0.17, -0.3, 0.17, 0.3)])
c = guard.check(DriveCommand(0.2, 0.0, 0.0)).command
check("board in the front margin: stands", c == STOP, c)
c = guard.check(DriveCommand(-0.1, 0.0, 0.0)).command
check("board in the front margin: backing away allowed", c == DriveCommand(-0.1, 0.0, 0.0), c)
c = guard.check(DriveCommand(0.0, 0.15, 0.0)).command
check("board in the front margin: sideways along it allowed", c == DriveCommand(0.0, 0.15, 0.0), c)

# Wall on the left at 0.25 m: 8.5 cm room to the margin, less than the
# stopping distance at 0.25 m/s (about 13 cm). At 0.30 m it is enough without
# braking since the 4 cm margin (previously 6 cm), so there is nothing to check there.
li.s = scan(segments=[(-2.0, 0.25, 2.0, 0.25)])
c = guard.check(DriveCommand(0.0, 0.25, 0.0)).command
check("wall left: sideways to the left braked", 0.0 <= c.vy < 0.25, c)
c = guard.check(DriveCommand(0.25, 0.0, 0.0)).command
check("wall left: forwards along it free", c == DriveCommand(0.25, 0.0, 0.0), c)

li.s = wall_ahead(0.3)
c = guard.check(DriveCommand(0.3, 0.1, 0.0)).command
check("diagonal: direction stays", c.vx < 0.3 and abs(c.vy / c.vx - 1 / 3) < 1e-9, c)

# Turning: AutonomousGuard could not do this -- it let every rotation through.
li.s = scan(segments=[(-2.0, 0.16, 2.0, 0.16)])  # wall 3.5 cm next to the left side
c = guard.check(DriveCommand(0.0, 0.0, 0.5)).command
check("wall close on the left: turning blocked (corner swings out)", c == STOP, c)
check("wall close on the left: turn_free says why", guard.turn_free(math.pi / 2) is not None,
       guard.turn_free(math.pi / 2))
li.s = scan(segments=[(1.0, -2.0, 1.0, 2.0), (-1.0, -2.0, -1.0, 2.0)])
c = guard.check(DriveCommand(0.0, 0.0, 0.5)).command
check("plenty of room: turning unchanged", c == DriveCommand(0.0, 0.0, 0.5), c)
check("plenty of room: turn_free", guard.turn_free(2 * math.pi) is None)

# Table leg front right next to the wheel, 5 cm in front of the edge.
li.s = scan(circles=[(0.22, -0.11, 0.02)], step=0.25)
c = guard.check(DriveCommand(0.3, 0.0, 0.0)).command
check("table leg next to the front wheel: braked", c.vx < 0.3, c)


# --- Part 1: integration in the watchdog ---------------------------------------


class FakeDriver(DriveBase):
    def connect(self):
        pass

    def disconnect(self):
        pass

    def drive(self, cmd):
        pass

    def stop(self):
        pass

    @property
    def connected(self):
        return True

    def read_battery(self):
        return 12.0


wd = SafetyWatchdog(FakeDriver())
li = Lidar(scan(segments=[(0.17, -0.3, 0.17, 0.3)]))
wd.submit(DriveCommand(0.2, 0.0, 0.0), source="autonom")
check("watchdog without guard: autonomous as before", wd._target.vx == 0.2, wd._target)

wd.drive_guard = DriveGuard(li)
wd.submit(DriveCommand(0.2, 0.0, 0.0), source="autonom")
check("watchdog: autonomous stopped in front of the board", wd._target == STOP, wd._target)
check("watchdog: result for the modes", wd.last_check is not None
       and wd.last_check.share == 0.0, wd.last_check)
check("watchdog: intervention counted", wd.drive_guard.state()["eingriffe"] == 1, wd.drive_guard.state())
wd.submit(DriveCommand(0.2, 0.0, 0.0), source="hand")
check("watchdog: manual control untouched", wd._target.vx == 0.2, wd._target)

config.AUTONOMOUS_GUARD_ACTIVE = False
wd.submit(DriveCommand(0.2, 0.0, 0.0), source="autonom")
check("watchdog: switched off -> autonomous untouched", wd._target.vx == 0.2, wd._target)
check("watchdog: switched off -> no old result", wd.last_check is None)
config.AUTONOMOUS_GUARD_ACTIVE = True

check("DriveGuard margin smaller than planner margin", config.GUARD_MARGIN_M < config.AVOID_MARGIN_M)


# --- Part 2: follow distance from the lidar cluster ----------------------------


class Pose:
    available = True
    fov_deg = config.CAMERA_FOV_DEG

    def people(self):
        return []

    def activate(self, an):
        pass


class Watchdog:
    def submit(self, cmd, source="hand"):
        pass

    def stop(self):
        pass


def interaction():
    return vh.Interaction(Watchdog(), Lidar(), Pose())


def box(center_deg, width_deg):
    """Normalised box (x0, y0, x1, y1) around an angle (left positive)."""
    fov = config.CAMERA_FOV_DEG
    x0 = 0.5 - (center_deg + width_deg / 2) / fov
    x1 = 0.5 - (center_deg - width_deg / 2) / fov
    return {"box": [x0, 0.1, x1, 0.9]}


def legs(x, y):
    """Two human legs (12 cm thick, 25 cm apart)."""
    return [(x, y + 0.125, 0.06), (x, y - 0.125, 0.06)]


# Person 1.5 m straight ahead, in front of them a thin chair leg at 0.8 m.
s = scan(circles=legs(1.5, 0.0) + [(0.8, 0.0, 0.0125)], step=0.5)
ia = interaction()
d = ia._distance_to(box(0.0, 12.0), s, 10.0)
check("chair leg in front of the person: distance to the person", d is not None and abs(d - 1.44) < 0.05,
       (round(d, 2), ia._distance_source))
check("chair leg in front of the person: source human", ia._distance_source == "Cluster: Mensch", ia._distance_source)
check("previously: the minimum would be the chair leg", s.min_distance(0.0, 6.0) < 0.85, round(s.min_distance(0.0, 6.0), 2))

# Two person-like objects (e.g. coat, armchair) in the image area: one
# at 1 m left of the box edge, one at 2 m near the box centre.
s = scan(circles=[(1.0, 0.25, 0.15), (2.0, -0.1, 0.15)], step=0.5)
b = box(0.0, 27.0)
ia = interaction()
d = ia._distance_to(b, s, 10.0)
check("two objects, no good distance: the one near the box centre", abs(d - 1.85) < 0.05,
       (round(d, 2), ia._distance_source))
ia._distance_good, ia._distance_good_time = 1.0, 9.5
d = ia._distance_to(b, s, 10.0)
check("two objects, last 1.0 m: the plausible one", abs(d - 0.85) < 0.05, round(d, 2))
ia._distance_good, ia._distance_good_time = 2.0, 9.5
d = ia._distance_to(b, s, 10.0)
check("two objects, last 2.0 m: the plausible one", abs(d - 1.85) < 0.05, round(d, 2))
ia._distance_good, ia._distance_good_time = 1.0, 10.0 - config.FOLLOW_CLUSTER_GOOD_S - 0.1
d = ia._distance_to(b, s, 10.0)
check("two objects, good distance stale: by angle again", abs(d - 1.85) < 0.05, round(d, 2))

# Person in front of the wall: only the wall in the image area -> the minimum, as before.
s = wall_ahead(1.2)
ia = interaction()
d = ia._distance_to(box(0.0, 12.0), s, 10.0)
check("only wall: fallback to minimum", abs(d - 1.2) < 0.01 and ia._distance_source == "Minimum",
       (round(d, 2), ia._distance_source))

# No distance without a scan or without a target.
ia = interaction()
check("no scan: no distance", ia._distance_to(box(0.0, 12.0), None, 10.0) is None)
check("no target: no distance", ia._distance_to(None, wall_ahead(1.0), 10.0) is None)

print()
print("all passed" if error == 0 else f"{error} errors")
sys.exit(1 if error else 0)
