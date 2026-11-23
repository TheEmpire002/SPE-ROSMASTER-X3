"""Position from consecutive lidar scans (lidar odometry).

Dead reckoning (``odometry.py``) on the X3 is only good for distances, not
for rotations: the library's IMU always reports a heading of 0 (sensor
check 28.10.2026), and on the floor the robot turns only about three
quarters of what is commanded. For SLAM and for driving lanes while
mapping, a wrong heading is the worst thing that can happen: every rotation
that is not reported twists the map.

That is why every new scan is aligned here via ICP (``scan_matching.py``)
onto one taken shortly before. The required rotation and translation is
the robot's motion in between. The wheels only provide the initial guess
for the search, the walls provide the result.

The comparison is not against the respective previous scan but against a
**reference scan** that is only renewed after some motion. While the robot
stands still, no noise accumulates this way: it keeps comparing against the
same scan and stays where it is.

Like any odometry this one drifts too -- but much less than the wheels,
and above all not by a quarter on every rotation. slam_toolbox corrects
the rest with its map.

Only runs while someone needs it: mapping and, while a map is loaded,
localization in the map (``map_localization.py``). The matching costs CPU,
and load spikes are delicate on the X3 (docs/hardware.md). That is why
every user registers and unregisters by name; it only ends when none is
left. A restart begins at (0, 0, 0) again -- anyone relying on the frame
can detect that via ``run``.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import NamedTuple

from .. import config
from .scan_matching import match

log = logging.getLogger(__name__)

#: More points make the matching hardly more accurate, but much slower
#: (it compares every point with every other).
_MAX_POINTS = 240
#: The result must not deviate from the estimate by more than this. Otherwise
#: ICP probably found a wrong alignment (long corridors, reflections).
_MAX_JUMP_M = 0.25
_MAX_JUMP_RAD = math.radians(15.0)
#: Higher estimated turn rates are a calculation error, not motion.
_MAX_OMEGA = 2.0


class Measurement(NamedTuple):
    """A processed scan with the position at the time it was measured."""

    nr: int
    scan: object          # sensors.lidar.Scan
    pose: tuple[float, float, float]  # x, y, angle (radians)
    timestamp: float           # monotonic, like Scan.timestamp


class ScanOdometry:
    """Holds the position from lidar matching, updated in the background."""

    def __init__(self, lidar, odometry) -> None:
        self._lidar = lidar
        self._odometry = odometry
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._users: set[str] = set()
        #: Counts every real start. If it changes, the frame started over.
        self.run = 0
        self._reset()

    def _reset(self) -> None:
        self._pose = (0.0, 0.0, 0.0)
        self._measurement: Measurement | None = None
        self._reference_points = None
        self._reference_pose = (0.0, 0.0, 0.0)
        self._relative = (0.0, 0.0, 0.0)   # last scan relative to the reference scan
        self._odo_prev: tuple[float, float, float] | None = None
        self._prev_time: float | None = None
        self._omega = 0.0
        self._matches = 0
        self._discarded = 0
        self._last_residual: float | None = None

    # --- Control ---------------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, user: str = "kartierung") -> None:
        """Starts at (0, 0, 0) -- that is the origin of the map.

        If it is already running (for another user), the frame stays as it
        is: localization should not lose its pose just because a new
        mapping run begins, and vice versa.
        """
        if self._lidar is None:
            return
        self._users.add(user)
        if self.running:
            return
        with self._lock:
            self._reset()
        self.run += 1
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="scan-odometrie", daemon=True)
        self._thread.start()
        log.info("Lidar odometry started (for %s)", user)

    def stop(self, user: str = "kartierung") -> None:
        """Unregisters; only stops it once nobody needs it any more."""
        self._users.discard(user)
        if self._users:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    # --- Queries ---------------------------------------------------------------

    def measurement(self) -> Measurement | None:
        """The most recently processed scan together with its position."""
        with self._lock:
            return self._measurement

    def pose(self) -> tuple[float, float, float] | None:
        """Current position (x, y, radians). None as long as nothing was measured.

        Between two scans (0.15-0.2 s) it is extrapolated with the wheel
        motion and the most recently measured turn rate, so that 20 Hz
        controllers do not sit on a stale value.
        """
        with self._lock:
            measurement = self._measurement
            odo_prev = self._odo_prev
            omega = self._omega
        if measurement is None or odo_prev is None:
            return None
        dt = min(max(time.monotonic() - measurement.timestamp, 0.0), 0.3)
        dx, dy = self._wheel_travel(odo_prev, self._odo_now())
        return _compose(measurement.pose, (dx, dy, omega * dt))

    def age(self) -> float | None:
        """Seconds since the last processed scan."""
        measurement = self.measurement()
        return time.monotonic() - measurement.timestamp if measurement else None

    def status(self) -> dict:
        with self._lock:
            pose = self._pose
            return {
                "laeuft": self.running,
                "nutzer": sorted(self._users),
                "pose": [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)],
                "abgleiche": self._matches,
                "verworfen": self._discarded,
                "letztes_residuum_m": (
                    round(self._last_residual, 3) if self._last_residual is not None else None
                ),
            }

    # --- Processing ------------------------------------------------------------

    def _loop(self) -> None:
        last = None
        while not self._stop.is_set():
            scan = self._lidar.latest()
            if scan is not None and scan is not last:
                last = scan
                try:
                    self._process(scan)
                except Exception:
                    # A single broken match must not freeze the position --
                    # the next scan tries again.
                    log.exception("Lidar odometry: scan not processed")
            self._stop.wait(0.02)

    def _process(self, scan) -> None:
        points = scan.points()
        if len(points) > _MAX_POINTS:
            points = points[:: math.ceil(len(points) / _MAX_POINTS)]
        odo = self._odometry.pose_at(scan.timestamp) or self._odo_now()

        if self._reference_points is None:
            # First scan: it is the reference, this is the origin.
            with self._lock:
                self._reference_points = points
                self._odo_prev, self._prev_time = odo, scan.timestamp
                self._measurement = Measurement(1, scan, self._pose, scan.timestamp)
            return

        # Estimate for the search: distance from the wheels, rotation from the
        # most recently measured turn rate. The wheels are no good for
        # rotations (see above), but fine for a few centimetres of distance.
        dt = max(scan.timestamp - self._prev_time, 1e-3)
        dx, dy = self._wheel_travel(self._odo_prev, odo)
        estimate = _compose(self._relative, (dx, dy, self._omega * dt))

        result = match(points, self._reference_points, estimate, iterations=15)
        good = (
            result is not None
            and result.residuum <= config.MAP_ICP_MAX_RESIDUAL_M
            and math.hypot(result.x - estimate[0], result.y - estimate[1]) < _MAX_JUMP_M
            and abs(_normalize(result.theta - estimate[2])) < _MAX_JUMP_RAD
        )
        relative = (result.x, result.y, result.theta) if good else estimate
        new = _compose(self._reference_pose, relative)

        with self._lock:
            omega = _normalize(new[2] - self._pose[2]) / dt
            self._omega = max(-_MAX_OMEGA, min(_MAX_OMEGA, omega))
            self._pose = new
            if good:
                self._matches += 1
                self._last_residual = result.residuum
            else:
                self._discarded += 1
            # Renew the reference when it is too far away or the match no
            # longer fits -- otherwise the scans eventually overlap too
            # little.
            if (
                not good
                or math.hypot(relative[0], relative[1]) > config.MAP_REF_M
                or abs(relative[2]) > math.radians(config.MAP_REF_DEG)
            ):
                self._reference_points = points
                self._reference_pose = new
                self._relative = (0.0, 0.0, 0.0)
            else:
                self._relative = relative
            self._odo_prev, self._prev_time = odo, scan.timestamp
            nr = self._measurement.nr + 1 if self._measurement else 1
            self._measurement = Measurement(nr, scan, new, scan.timestamp)

    def _odo_now(self) -> tuple[float, float, float]:
        x, y, deg = self._odometry.pose()
        return x, y, math.radians(deg)

    @staticmethod
    def _wheel_travel(previous, now) -> tuple[float, float]:
        """Distance according to the wheels between two odometry poses, in the robot frame.

        Transformed back with the odometry's own angle: whether that angle is
        right does not matter -- the odometry accumulated with exactly this
        angle, so transforming back yields the distance driven again.
        """
        dx, dy, _ = _relative(previous, now)
        # When the start point is reset, the odometry jumps to (0, 0).
        # That is not a drive and must not go into the position.
        if math.hypot(dx, dy) > config.MAP_JUMP_M:
            return 0.0, 0.0
        return dx, dy


def _compose(a, b) -> tuple[float, float, float]:
    """Append pose ``b`` (relative to ``a``) to pose ``a``."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalize(a[2] + b[2])


def _relative(a, b) -> tuple[float, float, float]:
    """Pose ``b`` as seen from pose ``a``."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, _normalize(b[2] - a[2])


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
