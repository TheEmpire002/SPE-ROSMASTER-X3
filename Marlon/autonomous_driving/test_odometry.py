"""Checks the odometry heading and the watchdog speed limits -- without the robot.

    python Marlon/autonomous_driving/test_odometry.py

Background: the X3's IMU attitude always reports 0 (sensor check 28.10.).
The odometry used to take this value as its heading, so theta stood still
while turning. Now theta has to follow the measured turn rate.
"""

import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.base import DriveBase  # noqa: E402
from rosmaster_assist.navigation.odometry import Odometry  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety.watchdog import SafetyWatchdog  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


class FakeDriver(DriveBase):
    """Driver like on the X3: board velocity is correct, IMU angle always 0."""

    def __init__(self):
        self.motion = (0.0, 0.0, 0.0)
        self.battery = 12.0
        self.driven = []

    def connect(self):
        pass

    def disconnect(self):
        pass

    def drive(self, cmd):
        self.driven.append(cmd)

    def stop(self):
        self.driven.append(DriveCommand())

    def read_motion(self):
        return self.motion

    def read_heading(self):
        return 0.0

    def read_battery(self):
        return self.battery

    @property
    def connected(self):
        return True


def simulate_drive(odo, command, seconds, dt=0.05):
    """Simulate time steps without waiting: wind the last timestamp back."""
    odo.update(command)  # first call only sets the time
    for _ in range(int(round(seconds / dt))):
        odo._last_time = time.monotonic() - dt
        odo.update(command)


# 1) Default: the IMU is not queried at all, theta follows the turn rate
driver = FakeDriver()
odo = Odometry(driver)
driver.motion = (0.0, 0.0, 0.5)
simulate_drive(odo, DriveCommand(omega=0.5), math.pi / 2 / 0.5)
_, _, theta = odo.pose()
check("theta follows turn rate (90° turn)", abs(theta - 90) < 3, f"{theta:.1f}°")  # pose() returns degrees
check("turn rate can be queried", abs(odo.velocity()[2] - 0.5) < 1e-9, odo.velocity())

# 2) Then straight ahead: the drive goes to the left (theta = 90°)
driver.motion = (0.2, 0.0, 0.0)
simulate_drive(odo, DriveCommand(vx=0.2), 2.0)
x, y, _ = odo.pose()
check("straight after turn goes in +y", abs(x) < 0.05 and abs(y - 0.4) < 0.05, f"x={x:.2f} y={y:.2f}")

# 3) With IMU heading enabled: a frozen IMU is detected and rejected
config.ODOM_IMU_HEADING = True
try:
    driver = FakeDriver()
    odo = Odometry(driver)
    driver.motion = (0.0, 0.0, 0.8)
    simulate_drive(odo, DriveCommand(omega=0.8), 3.0)
    _, _, theta = odo.pose()
    check("frozen IMU rejected", odo.imu_rejected)
    # Until the rejection (ODOM_IMU_CHECK_S) theta stood still, afterwards it integrates
    expected = 0.8 * (3.0 - config.ODOM_IMU_CHECK_S)
    check(
        "theta keeps turning after the rejection",
        abs(theta) > math.degrees(expected) * 0.7,
        f"{theta:.1f}° (at least ~{math.degrees(expected) * 0.7:.0f}°)",
    )
finally:
    config.ODOM_IMU_HEADING = False

# 4) Watchdog: autonomous commands are limited more tightly than manual control
driver = FakeDriver()
wd = SafetyWatchdog(driver)
wd.submit(DriveCommand(vx=2.0, omega=5.0), source="hand")
check("manual: MAX_SPEED_X applies", abs(wd._target.vx - config.MAX_SPEED_X) < 1e-9, wd._target)
wd.submit(DriveCommand(vx=2.0, vy=-2.0, omega=5.0), source="autonom")
check(
    "autonomous: AUTONOMOUS_MAX_* applies",
    abs(wd._target.vx - config.AUTONOMOUS_MAX_SPEED) < 1e-9
    and abs(wd._target.vy + config.AUTONOMOUS_MAX_SPEED) < 1e-9
    and abs(wd._target.omega - config.AUTONOMOUS_MAX_OMEGA) < 1e-9,
    wd._target,
)

# 5) Low battery: autonomous speed is throttled
driver = FakeDriver()
driver.battery = config.BATTERY_WARN_V - 0.5
wd = SafetyWatchdog(driver)
wd.submit(DriveCommand(vx=2.0), source="autonom")
requested = config.AUTONOMOUS_MAX_SPEED * config.AUTONOMOUS_BATTERY_FACTOR
check("battery low: speed throttled", abs(wd._target.vx - requested) < 1e-9, wd._target)

print("\nall passed" if not error else f"\n{error} failed")
sys.exit(1 if error else 0)
