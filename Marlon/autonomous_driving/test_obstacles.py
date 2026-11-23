"""Tests the obstacle labelling (robot/src/rosmaster_assist/navigation/obstacles.py)
on an imaginary room -- no robot, no dependencies.

    python Marlon/autonomous_driving/test_obstacles.py

The scan is generated the way the RPLIDAR A1 produces it: 1° resolution, rays
hit the nearest object. Circles are legs, line segments are walls.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.navigation import obstacles as h  # noqa: E402


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


def scan(circles=(), segments=(), step=1.0):
    angle, dist = [], []
    w = 0.0
    while w < 360.0:
        a = math.radians(w)
        hits = [ray_circle(a, *k) for k in circles] + [ray_segment(a, *s) for s in segments]
        hits = [t for t in hits if t is not None]
        angle.append(w)
        dist.append(min(hits) if hits else 0.0)  # 0 = no echo
        w += step
    return angle, dist


def categories(result):
    return sorted(x.category for x in result)


error = 0


def check(name, condition, result):
    global error
    print(("OK    " if condition else "ERROR "), name, [(x.category, round(x.distance, 2), round(x.angle)) for x in result])
    error += not condition


# Wall across the front of the robot
e = h.classify(*scan(segments=[(1.0, -1.5, 1.0, 1.5)]))
check("wall in front", categories(e) == [h.WALL], e)

# Chair: four legs (2.5 cm radius) in a 40 cm square, 1 m ahead on the left
chair = [(1.0 + dx, 0.6 + dy, 0.0125) for dx in (0, 0.4) for dy in (0, 0.4)]
e = h.classify(*scan(circles=chair, step=0.5))
check("chair", h.CHAIR in categories(e), e)

# Table: two legs 1.2 m apart, ahead on the right
table = [(1.2, -0.4, 0.02), (1.2, -1.6, 0.02)]
e = h.classify(*scan(circles=table, step=0.5))
check("table", categories(e) == [h.TABLE], e)

# Human: two legs (12 cm diameter) 25 cm apart, 1.2 m ahead
human = [(1.2, 0.125, 0.06), (1.2, -0.125, 0.06)]
e = h.classify(*scan(circles=human))
check("human (lidar)", categories(e) == [h.HUMAN], e)

# Camera beats the shape: an object the camera sees as a person
crate = [(1.0, 0.0, 0.2)]
e = h.classify(*scan(circles=crate), people_angles=[(-10.0, 10.0)])
check("human (camera)", categories(e) == [h.HUMAN] and e[0].source == "Kamera+Lidar", e)

# An object straight ahead lies half at the start (0°) and half at the
# end (359°) of the scan -- it must still be detected as one.
e = h.classify(*scan(circles=[(1.0, 0.0, 0.15)]))
check("object across the 0° boundary is one", len(e) == 1, e)

# Room with everything: nothing gets lost, nearest first
e = h.classify(*scan(
    circles=chair + table + human,
    segments=[(2.5, -3, 2.5, 3), (-2.5, -3, -2.5, 3)],
    step=0.5,
))
check("room: sorted", [x.distance for x in e] == sorted(x.distance for x in e), e)
check("room: everything present", {h.WALL, h.CHAIR, h.HUMAN} <= set(categories(e)), e)

print("\nall passed" if not error else f"\n{error} errors")
sys.exit(1 if error else 0)
