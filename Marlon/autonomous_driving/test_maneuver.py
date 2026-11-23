"""Checks the precise drive commands ("1 m forward", "turn 90°") in the simulation.

    python Marlon/autonomous_driving/test_maneuver.py

Simulated room and simulated lidar as with the server in --mock mode, plus a
drive that slips: it travels 5 % less far and turns 20 % less than commanded,
and the "wheels" report another 12 % less (MockDriver). Anyone counting by
drive command or wheels would therefore be well off -- the test checks whether
the lidar still gets the robot precisely to the target.

Takes a good minute (driving happens in real time).
"""

import asyncio
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.drive.mock_driver import MockDriver  # noqa: E402
from rosmaster_assist.navigation import maneuver as mv  # noqa: E402
from rosmaster_assist.safety import DriveGuard, SafetyWatchdog  # noqa: E402
from rosmaster_assist.sensors.lidar import LidarService, MockLidar  # noqa: E402

error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "FAIL  "), name, result)
    error += not condition


class SlippingDriver(MockDriver):
    """The "true" motion falls short of the command (mecanum wheels slip)."""

    FORWARD, TURN_FACTOR = 0.95, 0.80

    def _advance(self) -> None:
        now = time.monotonic()
        if self._true_time is None:
            self._true_time = now
            return
        dt = now - self._true_time
        self._true_time = now
        if dt <= 0 or dt > 1.0:
            return
        c = self._last
        w = c.omega * self.TURN_FACTOR
        avg = self._true_theta + w * dt / 2.0
        vx, vy = c.vx * self.FORWARD, c.vy * self.FORWARD
        self._true_x += (vx * math.cos(avg) - vy * math.sin(avg)) * dt
        self._true_y += (vx * math.sin(avg) + vy * math.cos(avg)) * dt
        self._true_theta += w * dt


def relative(start, end):
    """Where ``end`` lies as seen from ``start`` (x forward, y left, angle)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, end[2] - start[2]


async def execute(m, drive_connected, category, value):
    previous = drive_connected.true_pose()
    t0 = time.monotonic()
    check(f"{category} {value}: starts", m.start(category, value) is None)
    while m.active:
        await asyncio.sleep(0.1)
    return relative(previous, drive_connected.true_pose()), time.monotonic() - t0


async def main():
    drive_connected = SlippingDriver()
    drive_connected.connect()
    lidar = LidarService(MockLidar(drive_connected.true_pose))
    lidar.start()
    watchdog = SafetyWatchdog(drive_connected)
    watchdog.drive_guard = DriveGuard(lidar)  # as in the server
    watchdog.start()
    m = mv.Maneuver(watchdog, drive_connected, lidar)
    for _ in range(50):
        if lidar.latest() is not None:
            break
        await asyncio.sleep(0.1)

    # Invalid input is rejected without driving.
    check("0 m rejected", m.start(mv.FORWARD, 0) is not None)
    check("1000° rejected", m.start(mv.TURN, 1000) is not None)
    check("unknown direction rejected", m.start("hoch", 1) is not None)

    for category, value, requested in [
        (mv.FORWARD, 1.0, (1.0, 0.0, 0.0)),
        (mv.TURN, 90, (0.0, 0.0, 90.0)),
        (mv.TURN, -180, (0.0, 0.0, -180.0)),
        (mv.RIGHT, 0.5, (0.0, -0.5, 0.0)),
        (mv.BACK, 0.3, (-0.3, 0.0, 0.0)),
        (mv.TURN, 360, (0.0, 0.0, 360.0)),
    ]:
        (x, y, th), duration = await execute(m, drive_connected, category, value)
        deg = math.degrees(th)
        deviation_m = math.hypot(x - requested[0], y - requested[1])
        deviation_deg = deg - requested[2]
        text = (f"actual x={x:+.3f} y={y:+.3f} {deg:+.1f}° in {duration:.1f} s "
                f"-- {m.last_reason}")
        check(f"{mv.describe(category, value)}: precise (angle ±3°)", abs(deviation_deg) < 3.0, text)
        if category != mv.TURN:
            check(f"{mv.describe(category, value)}: precise (distance ±4 cm, straight)", deviation_m < 0.04,
                   f"{deviation_m * 100:.1f} cm off")
        check(f"{mv.describe(category, value)}: reports success", m.succeeded, m.last_reason)

    # 5 m in any direction never fits completely in the 8 x 6 m room:
    # it has to stop in front of the wall and must not report success.
    (x, y, th), _ = await execute(m, drive_connected, mv.RIGHT, 5.0)
    check("stopped in front of the wall, not successful",
           not m.succeeded and "Hindernis" in (m.last_reason or ""), m.last_reason)
    wx, wy, _ = drive_connected.true_pose()
    x_min, x_max, y_min, y_max = MockLidar.ROOM
    margin = min(wx - x_min, x_max - wx, wy - y_min, y_max - wy)
    check("distance to the wall kept (> 0.3 m)", margin > 0.3, f"{margin:.2f} m at ({wx:.2f}, {wy:.2f})")

    # Cancelling stops immediately.
    m.start(mv.FORWARD, 2.0)
    await asyncio.sleep(1.5)
    m.cancel("Test")
    await asyncio.sleep(0.2)
    check("cancel stops", not m.active and watchdog.current_command.is_stop(),
           watchdog.current_command)

    await watchdog.shutdown()
    lidar.stop()


asyncio.run(main())
print("\nall passed" if not error else f"\n{error} failed")
sys.exit(1 if error else 0)
