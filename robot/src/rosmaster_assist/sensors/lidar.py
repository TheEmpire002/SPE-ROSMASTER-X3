"""Reading the RPLIDAR A1.

Delivers all-round distance measurements: a few hundred points of angle and
distance per revolution. They feed the localization (see
``navigation/scan_matching.py``) and the obstacle check during homing.

As with the drive and the camera there is a simulated twin: ``MockLidar``
scans an imaginary room, so localization and homing can be developed and
tested without hardware.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from abc import ABC, abstractmethod

from .. import config

log = logging.getLogger(__name__)


class Scan:
    """One revolution: angles in degrees, distances in metres.

    The points are in the robot frame: 0° is the front, the angle grows
    counter-clockwise -- the same convention as the drive, so there is no
    constant converting back and forth.
    """

    __slots__ = ("angle", "ranges", "timestamp", "duration", "true_pose")

    #: Assumed duration of one revolution until a measured one is available
    #: (RPLIDAR A1 on the X3: 7.5 revolutions/s measured).
    DEFAULT_DURATION_S = 1.0 / 7.5

    def __init__(self, angle: list[float], ranges: list[float]) -> None:
        self.angle = angle
        self.ranges = ranges
        # Time at which the revolution was complete -- the first point is
        # already a whole revolution old by then.
        self.timestamp = time.monotonic()
        self.duration = self.DEFAULT_DURATION_S
        # Simulation only: where the robot really stood during this
        # revolution. This lets the simulated mapping build a map without
        # drift, as slam_toolbox does with loop closure on the robot.
        self.true_pose: tuple[float, float, float] | None = None

    @property
    def mid_time(self) -> float:
        """Mean capture time of the revolution.

        More accurate than `timestamp` for calculations involving the robot's
        own motion (odometry lookback): on average a point is half a
        revolution older than the end of the revolution, about 2 cm at 0.3 m/s.
        """
        return self.timestamp - self.duration / 2.0

    def __len__(self) -> int:
        return len(self.angle)

    def points(self):
        """As an Nx2 array of Cartesian points (x forward, y left)."""
        import numpy as np

        a = np.radians(np.asarray(self.angle, dtype=float))
        r = np.asarray(self.ranges, dtype=float)
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    def min_distance(self, direction_deg: float, aperture_deg: float) -> float:
        """Smallest distance within a circular sector.

        For the obstacle check: "what is closest in the driving direction".
        """
        smallest = float("inf")
        for w, r in zip(self.angle, self.ranges):
            diff = abs((w - direction_deg + 180.0) % 360.0 - 180.0)
            if diff <= aperture_deg and r < smallest:
                smallest = r
        return smallest


class LidarBase(ABC):
    name = "base"
    label = "Lidar"

    def __init__(self) -> None:
        self._opened = False

    @property
    def opened(self) -> bool:
        return self._opened

    def open(self) -> None:
        if self._opened:
            return
        try:
            self._open()
        except Exception:
            try:
                self._close()
            except Exception:
                log.debug("Cleanup after failure failed", exc_info=True)
            raise
        self._opened = True

    def close(self) -> None:
        if not self._opened:
            return
        try:
            self._close()
        finally:
            # Even if closing fails (device already gone): otherwise open()
            # would consider the lidar open and do nothing on the next
            # attempt.
            self._opened = False

    @abstractmethod
    def _open(self) -> None: ...

    @abstractmethod
    def _close(self) -> None: ...

    @abstractmethod
    def read(self) -> Scan | None:
        """Next complete revolution. May block."""


# --- Real hardware ----------------------------------------------------------------


class RPLidar(LidarBase):
    name = "rplidar"
    label = "RPLIDAR A1"

    def __init__(self, port: str | None = None) -> None:
        super().__init__()
        self._port = port or config.LIDAR_PORT
        self._lidar = None
        self._scans = None

    def _open(self) -> None:
        try:
            from rplidar import RPLidar as Driver
        except ImportError as exc:
            raise RuntimeError(
                "Paket fehlt -- robot/.venv/bin/pip install rplidar-roboticia"
            ) from exc

        import os

        if not os.path.exists(self._port):
            raise RuntimeError(f"{self._port} existiert nicht -- Lidar eingesteckt?")

        from ..serial_ports import who_holds

        others = who_holds(self._port)
        if others:
            # Two readers on the same lidar both get only fragments --
            # which then looks like a broken device.
            log.error("Another program has the lidar %s open:", self._port)
            for line in others:
                log.error("    %s", line)
            log.error("Stop it with: kill <PID>  (ROS2: sudo pkill -f sllidar_node)")

        self._lidar = Driver(self._port, baudrate=config.LIDAR_BAUDRATE, timeout=3)

        # If the lidar is still running from the previous program (crashed,
        # ROS2, an earlier run), it keeps sending measurements unasked. Then
        # get_info() reads in the middle of that stream and fails with
        # "Incorrect descriptor starting bytes". So stop it first and clear
        # the buffer -- and allow a few attempts.
        last_error: Exception | None = None
        for _ in range(3):
            try:
                self._lidar.stop()
                time.sleep(0.2)
                # Named differently depending on the package version.
                clear = getattr(self._lidar, "clean_input", None) or getattr(
                    self._lidar, "clear_input", None
                )
                if clear is not None:
                    clear()
                info = self._lidar.get_info()
                state = self._lidar.get_health()
                break
            except Exception as exc:
                last_error = exc
                time.sleep(0.5)
        else:
            raise RuntimeError(f"Lidar antwortet nicht sauber: {last_error}")

        log.info("RPLIDAR connected: %s, health %s", info, state[0])
        if state[0] == "Error":
            raise RuntimeError(f"Lidar meldet Fehler: {state}")

        # iter_scans delivers complete revolutions instead of single points.
        # Keep min_len low: at 5.5 revolutions/s ~180 points per revolution
        # are normal, and too high a value means it never gets going.
        self._scans = self._lidar.iter_scans(min_len=60)

    def _close(self) -> None:
        if self._lidar is None:
            return
        try:
            self._lidar.stop()
            self._lidar.stop_motor()
            self._lidar.disconnect()
        except Exception:
            log.exception("Lidar could not be closed cleanly")
        finally:
            self._lidar = None
            self._scans = None

    def read(self) -> Scan | None:
        if self._scans is None:
            return None

        raw = next(self._scans)  # list of (quality, angle in degrees, distance in mm)
        angle, ranges = [], []
        for _quality, deg, mm in raw:
            meter = mm / 1000.0
            if not (config.LIDAR_MIN_M <= meter <= config.LIDAR_MAX_M):
                continue
            # The driver counts clockwise, our drive counter-clockwise.
            # Without this flip the whole map would be mirrored. Plus the
            # mounting offset, so that 0° really is the front.
            angle.append((-deg + config.LIDAR_ANGLE_OFFSET_DEG) % 360.0)
            ranges.append(meter)

        return Scan(angle, ranges) if angle else None


# --- Simulation ---------------------------------------------------------------------


class MockLidar(LidarBase):
    """Scans an imaginary rectangular room.

    Asks the drive where the robot *really* is and computes what a lidar
    would see from there. This lets the localization be tested at the desk
    -- including whether it actually corrects an odometry error.
    """

    name = "mock"
    label = "Simulierter Lidar"

    #: Room in metres, the robot's starting point is at (0, 0).
    ROOM = (-3.0, 5.0, -3.0, 3.0)  # x_min, x_max, y_min, y_max
    #: A piece of furniture so the room is not symmetric -- otherwise the
    #: orientation would be ambiguous and the matching could find any rotation.
    PILLAR = (2.0, 1.2, 0.35)  # x, y, radius
    #: Only with MOCK_APARTMENT: a partition wall with a door (y 1.4 to 2.3 m),
    #: i.e. two rooms instead of one, for continuing the mapping drive --
    #: plus a cupboard on the left and a sofa on the right. Without furniture
    #: two empty rectangles would look the same to any localization (rotated
    #: by 180° or shifted by 4 m); real rooms never are.
    APARTMENT_WALLS = (
        (1.0, -3.0, 1.0, 1.4), (1.0, 2.3, 1.0, 3.0),
        # cupboard in the top-left corner
        (-3.0, 2.0, -2.4, 2.0), (-2.4, 2.0, -2.4, 3.0),
        # sofa against the wall at the bottom right
        (3.0, -3.0, 3.0, -2.2), (3.0, -2.2, 4.6, -2.2), (4.6, -2.2, 4.6, -3.0),
    )
    STEP_DEG = 1.0

    def __init__(self, true_pose_source=None) -> None:
        super().__init__()
        self._pose_source = true_pose_source
        # Extra round obstacles (x, y, radius), e.g. a person lying on the
        # floor for the health check. Always replaced as a whole, never
        # mutated, so the reader thread needs no lock.
        self._extra: tuple[tuple[float, float, float], ...] = ()

    def set_obstacles(self, circles) -> None:
        """Extra obstacles as circles (x, y, radius) in world coordinates."""
        self._extra = tuple((float(x), float(y), float(r)) for x, y, r in circles)

    def _open(self) -> None:
        log.info("Lidar: simulation (no real hardware)")

    def _close(self) -> None:
        pass

    def read(self) -> Scan | None:
        if not self.opened:
            return None
        time.sleep(0.18)  # ~5.5 revolutions/s like the real A1

        x, y, theta = (0.0, 0.0, 0.0)
        if self._pose_source is not None:
            x, y, theta = self._pose_source()

        angle, ranges = [], []
        steps = int(360 / self.STEP_DEG)
        for i in range(steps):
            a_robot = i * self.STEP_DEG
            a_world = math.radians(a_robot) + theta
            r = self._ray(x, y, a_world)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                angle.append(a_robot)
                ranges.append(r)
        scan = Scan(angle, ranges)
        scan.true_pose = (x, y, theta)
        return scan

    def _ray(self, x: float, y: float, direction: float) -> float:
        """Distance to the nearest wall or to the pillar."""
        dx, dy = math.cos(direction), math.sin(direction)
        x_min, x_max, y_min, y_max = self.ROOM

        hits = float("inf")
        for limit, d, start in ((x_min, dx, x), (x_max, dx, x), (y_min, dy, y), (y_max, dy, y)):
            if abs(d) < 1e-9:
                continue
            t = (limit - start) / d
            if t <= 0:
                continue
            px, py = x + dx * t, y + dy * t
            # Only count it if the intersection lies on the wall itself and
            # not on its imaginary extension.
            if x_min - 1e-6 <= px <= x_max + 1e-6 and y_min - 1e-6 <= py <= y_max + 1e-6:
                hits = min(hits, t)

        for circle in (self.PILLAR, *self._extra):
            t_circle = self._hit_pillar(x, y, dx, dy, circle)
            if t_circle is not None:
                hits = min(hits, t_circle)
        if config.MOCK_APARTMENT:
            for wall in self.APARTMENT_WALLS:
                t_wall = _hit_segment(x, y, dx, dy, wall)
                if t_wall is not None:
                    hits = min(hits, t_wall)
        return hits

    def _hit_pillar(self, x, y, dx, dy, circle=None) -> float | None:
        cx, cy, radius = circle if circle is not None else self.PILLAR
        mx, my = x - cx, y - cy
        b = 2 * (mx * dx + my * dy)
        c = mx * mx + my * my - radius * radius
        discriminant = b * b - 4 * c
        if discriminant < 0:
            return None
        root = math.sqrt(discriminant)
        for t in sorted(((-b - root) / 2, (-b + root) / 2)):
            if t > 0:
                return t
        return None


def _hit_segment(x, y, dx, dy, segment) -> float | None:
    """Distance along the ray to the segment (x1, y1, x2, y2), otherwise None."""
    x1, y1, x2, y2 = segment
    ex, ey = x2 - x1, y2 - y1
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    t = ((x1 - x) * ey - (y1 - y) * ex) / denom
    u = ((x1 - x) * dy - (y1 - y) * dx) / denom
    return t if t > 0 and 0.0 <= u <= 1.0 else None


# --- Selection and background thread ----------------------------------------------------


def create_lidar(use_mock: bool = False, true_pose_source=None) -> LidarBase:
    """On the robot always the real lidar, in mock mode the simulation.

    Deliberately no fallback to the simulation when the real lidar does not
    open at startup: an invented room would be worse than none at all. The
    app would show walls that do not exist, and the localization would match
    against them. Instead ``LidarService`` keeps trying in the background
    until the real lidar answers.
    """
    if use_mock:
        fallback = MockLidar(true_pose_source)
        fallback.open()
        return fallback

    lidar = RPLidar()
    try:
        lidar.open()
    except Exception as exc:
        log.error("Lidar: %s not usable -- %s", lidar.label, exc)
        log.error("Retrying in the background. Status: /lidar/status")
        try:
            lidar.close()
        except Exception:
            pass
    return lidar


class LidarService:
    """Reads continuously in the background and keeps the latest revolution ready."""

    def __init__(self, lidar: LidarBase) -> None:
        self._lidar = lidar
        self._scan: Scan | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._recent_scans = 0
        self._latest_error: str | None = None
        self._reconnects = 0
        # For the grace period when reconnecting (see _reopen).
        self._connected_since: float | None = None
        self._last_wait = config.RECONNECT_MIN_S

    @property
    def label(self) -> str:
        return self._lidar.label

    @property
    def is_simulation(self) -> bool:
        return self._lidar.name == "mock"

    def latest(self) -> Scan | None:
        with self._lock:
            return self._scan

    def status(self) -> dict:
        with self._lock:
            scan = self._scan
            passes = self._recent_scans
            error = self._latest_error
        return {
            "lidar": self._lidar.label,
            "geoeffnet": self._lidar.opened,
            "thread_laeuft": self._thread is not None and self._thread.is_alive(),
            "umdrehungen": passes,
            "punkte_letzte_runde": len(scan) if scan else 0,
            "letzte_runde_vor_s": round(time.monotonic() - scan.timestamp, 1) if scan else None,
            "letzter_fehler": error,
            "neu_verbunden": self._reconnects,
        }

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            self._lidar.open()
        except Exception as exc:
            # No abort: the reader thread starts anyway and keeps trying.
            # Otherwise the lidar would stay dead until the server restarts,
            # just because it refused for a moment during startup.
            log.error("Lidar cannot be opened yet -- %s", exc)
            self._latest_error = f"{type(exc).__name__}: {exc}"
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # Wait generously: read() blocks until the next revolution.
            self._thread.join(timeout=3.0)
            self._thread = None
        self._lidar.close()

    def _loop(self) -> None:
        error = 0
        while not self._stop.is_set():
            if not self._lidar.opened:
                self._reopen()
                continue
            try:
                scan = self._lidar.read()
                if scan is not None:
                    with self._lock:
                        # Revolution duration from the gap to the previous one,
                        # only if plausible (after dropouts it is huge).
                        if self._scan is not None:
                            duration = scan.timestamp - self._scan.timestamp
                            if 0.05 < duration < 0.5:
                                scan.duration = duration
                        self._scan = scan
                        self._recent_scans += 1
                        self._latest_error = None
                    error = 0
            except Exception as exc:
                error += 1
                with self._lock:
                    self._latest_error = f"{type(exc).__name__}: {exc}"
                if error <= 3:
                    log.exception("Lidar delivers nothing (attempt %d)", error)
                # Wait briefly, otherwise the loop spins at full load on a
                # persistent error.
                self._stop.wait(0.5)
                # Reading on from the same device is pointless after that: the
                # driver's data stream is exhausted after the first exception,
                # and after a USB dropout the opened device no longer exists
                # at all.
                if error >= 3:
                    self._reopen()
                    error = 0

    def _reopen(self) -> None:
        """Close and reopen until it works or the service is stopped."""
        log.warning("Lidar no longer responding -- reopening it")
        try:
            self._lidar.close()
        except Exception:
            log.debug("Closing the lidar failed", exc_info=True)

        # If the lidar drops out again shortly after the last reconnect, its
        # own motor start-up was probably the trigger: it draws current, the
        # USB supply dips, everything disconnects. On 28.10. this kept an
        # endless loop going because the wait time restarted at 1 s after
        # every success. So keep doubling in that case.
        now = time.monotonic()
        if (self._connected_since is not None
                and now - self._connected_since < config.LIDAR_STABLE_S):
            wait_time = min(self._last_wait * 2, config.RECONNECT_MAX_S)
            log.warning(
                "Lidar dropped out again %.0f s after reconnecting -- "
                "waiting %.0f s so the motor start-up does not cause an endless loop",
                now - self._connected_since, wait_time,
            )
        else:
            wait_time = config.RECONNECT_MIN_S
        self._connected_since = None

        while not self._stop.wait(wait_time):
            try:
                self._lidar.open()
            except Exception as exc:
                with self._lock:
                    self._latest_error = f"Neu öffnen: {type(exc).__name__}: {exc}"
                log.warning(
                    "Lidar not reachable yet (%s) -- next attempt in %.0f s",
                    exc,
                    min(wait_time * 2, config.RECONNECT_MAX_S),
                )
                wait_time = min(wait_time * 2, config.RECONNECT_MAX_S)
                continue
            self._reconnects += 1
            self._connected_since = time.monotonic()
            self._last_wait = wait_time
            log.warning("Lidar reconnected")
            return
