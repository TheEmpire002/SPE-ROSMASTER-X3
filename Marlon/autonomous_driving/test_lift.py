"""Checks the lift detection (LiftDetector) without a robot.

    python Marlon/autonomous_driving/test_lift.py

The robot must report "lifted" only when it really is lifted, tilted or
carried -- not when its own wheel odometry is wrong. Cases:

* false alarms (must NOT trigger):
  - turning on the spot while the wheels report a much smaller rotation,
  - wheels spinning at full speed while the robot is stuck,
  - sectors far away flickering (window, open door: no echo every time),
  - a short tilt for two scans (driving over a door sill);
* real events (MUST trigger):
  - carried sideways while the wheels report standstill,
  - tilted (the scan plane hits the floor on one side), held for a second,
  - and it clears again once it stands still.

The world is the simulated room of MockLidar (8 x 6 m with a pillar).
"""

import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.navigation import avoidance as aw  # noqa: E402
from rosmaster_assist.sensors.lidar import MockLidar, Scan  # noqa: E402

random.seed(7)
failures = 0
LIDAR = MockLidar()
SCAN_PERIOD_S = 0.18


def check(name, condition, info=""):
    global failures
    print(("OK    " if condition else "FAIL  "), name, info)
    failures += not condition


def make_scan(x, y, theta, t, tilt=0.0, flicker_far=0.0):
    """Simulated scan at the true pose.

    ``tilt``: fraction of directions (one side) whose beam hits the floor,
    as if the robot were tipped. ``flicker_far``: probability that a far
    point (> 3 m) has no echo.
    """
    angles, ranges = [], []
    for i in range(360):
        r = LIDAR._ray(x, y, math.radians(i) + theta)
        if tilt and i < 360 * tilt:
            r = min(r, 0.6 + 0.4 * math.sin(math.radians(i)) ** 2)  # floor close by
        if r > 3.0 and random.random() < flicker_far:
            continue
        if 0.15 <= r <= 8.0:
            angles.append(float(i))
            ranges.append(r)
    scan = Scan(angles, ranges)
    scan.timestamp = t
    return scan


def run(true_motion, wheel_motion, seconds=6.0, tilt_from=None, tilt_until=None,
        flicker_far=0.0, start=(0.5, -0.5, 0.0)):
    """Feed scans; ``*_motion(t) -> (vx, vy, omega)`` in the robot frame.

    Returns (ever lifted, lifted at the end, highest changed fraction).
    """
    detector = aw.LiftDetector()
    true_pose = list(start)
    wheel_pose = list(start)
    t = 0.0
    ever = False
    highest = 0.0
    while t < seconds:
        t += SCAN_PERIOD_S
        for pose, motion in ((true_pose, true_motion), (wheel_pose, wheel_motion)):
            vx, vy, om = motion(t)
            th = pose[2] + om * SCAN_PERIOD_S / 2
            pose[0] += (vx * math.cos(th) - vy * math.sin(th)) * SCAN_PERIOD_S
            pose[1] += (vx * math.sin(th) + vy * math.cos(th)) * SCAN_PERIOD_S
            pose[2] += om * SCAN_PERIOD_S
        tilted = tilt_from is not None and tilt_from <= t < tilt_until
        scan = make_scan(*true_pose, t, tilt=0.5 if tilted else 0.0, flicker_far=flicker_far)
        lifted = detector.check(scan, tuple(wheel_pose))
        ever |= lifted
        highest = max(highest, detector.share)
    return ever, lifted, round(highest, 2)


def still(_t):
    return 0.0, 0.0, 0.0


# --- False alarms ------------------------------------------------------------------

ever, _, highest = run(lambda t: (0.0, 0.0, 0.8), lambda t: (0.0, 0.0, 0.4))
check("turning, wheels report half the rotation: no alarm", not ever, f"max {highest}")

ever, _, highest = run(still, lambda t: (0.4, 0.0, 0.0))
check("stuck, wheels spinning at 0.4 m/s: no alarm", not ever, f"max {highest}")

ever, _, highest = run(lambda t: (0.0, 0.0, 0.5), lambda t: (0.0, 0.0, 0.5), flicker_far=0.6)
check("far sectors flickering (no echo): no alarm", not ever, f"max {highest}")

ever, _, highest = run(lambda t: (0.2, 0.0, 0.0), lambda t: (0.2, 0.0, 0.0),
                       tilt_from=2.0, tilt_until=2.0 + 2 * SCAN_PERIOD_S)
check("tilted for two scans (door sill): no alarm", not ever, f"max {highest}")

# --- Real events --------------------------------------------------------------------

ever, _, highest = run(lambda t: (0.0, 0.6, 0.0) if 1.0 < t < 3.0 else (0.0, 0.0, 0.0), still)
check("carried sideways, wheels report standstill: alarm", ever, f"max {highest}")

ever, end, highest = run(still, still, seconds=8.0, tilt_from=1.0, tilt_until=3.0)
check("tilted for two seconds: alarm", ever, f"max {highest}")
check("set down again: alarm clears", not end)

print()
print("All checks passed." if failures == 0 else f"{failures} check(s) failed.")
sys.exit(1 if failures else 0)
