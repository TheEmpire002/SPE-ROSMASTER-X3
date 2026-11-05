"""Feed lidar and pose into the ROS container.

slam_toolbox builds the map, but needs two things from ROS for that: the
lidar scans on ``/scan`` and the robot's pose as TF
(``odom -> base_footprint``). In the Yahboom stack they would come from
``sllidar_node`` and ``yahboomcar_bringup``. Those must not run here,
though, because our server already has the lidar and board open -- two
readers on the same device both get only fragments (docs/hardware.md).

So it works the other way round: our server reads as always and passes the
data on. For this a small ROS node runs in the container, started via
``docker exec -i``. It receives JSON lines over stdin and publishes them as
ROS messages. stdin as the channel because it needs no network settings
between Pi and container -- and when our server ends, the node gets EOF and
exits by itself instead of continuing with stale data.

Lines to the node (times as Unix time, the container shares the clock
with the Pi)::

    {"typ": "odom", "t": .., "x": .., "y": .., "theta": ..}
    {"typ": "scan", "t": .., "dauer": .., "r": [..]}     (0 = no echo)
"""

from __future__ import annotations

import json
import logging
import shlex
import subprocess
import threading
import time
from collections import deque
from typing import Any

from .. import config

log = logging.getLogger(__name__)

# Frame names as in the default configuration of slam_toolbox (Foxy:
# base_frame = base_footprint). base_link is published additionally so that
# a configuration with base_link as the base works too.
_ODOM_FRAME = "odom"
_BASE_FRAME = "base_footprint"
_LASER_FRAME = "laser"

# Part of the node name and thus of the process command line. This is how a
# leftover node is recognized before it is stopped.
_TAG = "rosmaster_scan_einspeisung"
_PID_FILE = "/tmp/rosmaster-scan-einspeisung.pid"
_READY = "ROSFEED_BEREIT"

# How long the start waits for the node. Bringing up ROS2 takes one or two
# seconds on the Pi; if it fails (missing package), the error should be on
# the page right at start and not only at some later point.
_READY_WAIT_S = 10.0


_NODE_SCRIPT = r'''import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

cfg = json.loads(os.environ['ROSFEED_CONFIG'])


def stempel(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def transform(t, eltern, kind, x=0.0, y=0.0, theta=0.0):
    tf = TransformStamped()
    tf.header.stamp = stempel(t)
    tf.header.frame_id = eltern
    tf.child_frame_id = kind
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.rotation.z = math.sin(theta / 2.0)
    tf.transform.rotation.w = math.cos(theta / 2.0)
    return tf


rclpy.init()
node = Node('rosmaster_scan_einspeisung')
scan_pub = node.create_publisher(LaserScan, cfg['scan_topic'], 10)
odom_pub = node.create_publisher(Odometry, '/odom', 10)
tf_pub = node.create_publisher(TFMessage, '/tf', 100)
static_pub = node.create_publisher(TFMessage, '/tf_static', QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
))
# For us the lidar sits in the center and points forward: mounting position
# and direction of rotation are already compensated in our server.
jetzt = time.time()
static_pub.publish(TFMessage(transforms=[
    transform(jetzt, cfg['basis_frame'], 'base_link'),
    transform(jetzt, 'base_link', cfg['laser_frame']),
]))
print('ROSFEED_BEREIT', flush=True)

try:
    for zeile in sys.stdin:
        try:
            m = json.loads(zeile)
        except ValueError:
            continue
        if m.get('typ') == 'odom':
            tf = transform(m['t'], cfg['odom_frame'], cfg['basis_frame'],
                           m['x'], m['y'], m['theta'])
            tf_pub.publish(TFMessage(transforms=[tf]))
            odom = Odometry()
            odom.header = tf.header
            odom.child_frame_id = cfg['basis_frame']
            odom.pose.pose.position.x = float(m['x'])
            odom.pose.pose.position.y = float(m['y'])
            odom.pose.pose.orientation = tf.transform.rotation
            odom_pub.publish(odom)
        elif m.get('typ') == 'scan':
            n = len(m['r'])
            s = LaserScan()
            s.header.stamp = stempel(m['t'])
            s.header.frame_id = cfg['laser_frame']
            s.angle_min = -math.pi
            s.angle_increment = 2.0 * math.pi / n
            s.angle_max = s.angle_min + (n - 1) * s.angle_increment
            s.time_increment = 0.0
            s.scan_time = float(m.get('dauer', 0.0))
            s.range_min = float(cfg['range_min'])
            s.range_max = float(cfg['range_max'])
            s.ranges = [float(r) if r > 0 else math.inf for r in m['r']]
            scan_pub.publish(s)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''


class ScanFeed:
    """Continuously passes lidar scans and pose on to ROS in the container."""

    def __init__(
        self, lidar, scan_odometry, container: str, setup_file: str, docker_bin: str
    ) -> None:
        self._lidar = lidar
        # Provides the pose including heading for every scan from the lidar
        # matching (navigation/scan_odometry.py). The wheels alone were not
        # good enough: on the X3 they give no usable rotation.
        self._scan_odo = scan_odometry
        self._container = container
        self._setup_file = setup_file
        self._docker_bin = docker_bin

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._process: subprocess.Popen[str] | None = None
        self._ready = threading.Event()
        self._output: deque[str] = deque(maxlen=20)

        self._scans = 0
        self._last_scan: float | None = None  # monotonic
        self._error: str | None = None
        self._restarts = 0

    # --- Control ---------------------------------------------------------------

    def start(self) -> None:
        """Start the node in the container. Raises if it does not come up."""
        if self._lidar is None:
            raise RuntimeError(
                "Ohne Lidar keine Karte -- LIDAR_ENABLED ist in config.py ausgeschaltet."
            )
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop.clear()
        self._scans = 0
        self._last_scan = None
        self._error = None
        self._restarts = 0

        # Otherwise a node from a crashed earlier run would keep publishing
        # its own frozen pose on /tf -- two contradicting poses tear the map
        # apart.
        self._stop_old()
        self._start_process()
        if not self._ready.wait(_READY_WAIT_S):
            process = self._process
            if process is None or process.poll() is not None:
                output = self._output_tail()
                self._stop_process()
                raise RuntimeError(
                    "Lidar-Einspeisung in ROS ist nicht angelaufen. "
                    f"Ausgabe im Container: {output or '(keine)'}"
                )
            # Still running, just slow: carry on, the status shows it.
            log.warning("Lidar feed not reporting ready yet after %.0f s",
                        _READY_WAIT_S)

        self._thread = threading.Thread(
            target=self._loop, name="ros-einspeisung", daemon=True
        )
        self._thread.start()
        log.info("Lidar feed into ROS started (container %s)", self._container)

    def stop(self) -> None:
        self._stop.set()
        self._stop_process()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=3.0)
        self._thread = None
        try:
            self._stop_old()
        except (OSError, subprocess.SubprocessError):
            # Docker already gone (e.g. during shutdown): then so is the
            # node.
            pass

    def status(self) -> dict[str, Any]:
        process = self._process
        scan = self._lidar.latest() if self._lidar is not None else None
        now = time.monotonic()
        return {
            "laeuft": (
                self._thread is not None
                and self._thread.is_alive()
                and process is not None
                and process.poll() is None
            ),
            "bereit": self._ready.is_set(),
            "scans": self._scans,
            "letzter_scan_vor_s": (
                round(now - self._last_scan, 1) if self._last_scan else None
            ),
            # Separate from the above: this shows whether the lidar itself
            # delivers nothing or only the forwarding is stuck.
            "lidar_scan_vor_s": round(now - scan.timestamp, 1) if scan else None,
            "neustarts": self._restarts,
            "fehler": self._error,
        }

    # --- Process in the container ----------------------------------------------

    def _start_process(self) -> None:
        settings = {
            "scan_topic": config.MAP_SCAN_TOPIC,
            "odom_frame": _ODOM_FRAME,
            "basis_frame": _BASE_FRAME,
            "laser_frame": _LASER_FRAME,
            "range_min": config.LIDAR_MIN_M,
            "range_max": config.LIDAR_MAX_M,
        }
        inside = (
            f"source {shlex.quote(self._setup_file)} && "
            f"export ROSFEED_CONFIG={shlex.quote(json.dumps(settings))} && "
            f"echo $$ > {shlex.quote(_PID_FILE)} && "
            f"exec python3 -u -c {shlex.quote(_NODE_SCRIPT)}"
        )
        self._ready.clear()
        self._output.clear()
        try:
            process = subprocess.Popen(
                [self._docker_bin, "exec", "-i", self._container, "bash", "-lc", inside],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
            )
        except OSError as exc:
            raise RuntimeError(f"Lidar-Einspeisung konnte nicht starten: {exc}") from exc
        with self._lock:
            self._process = process
        threading.Thread(
            target=self._read_output, args=(process,), name="ros-einspeisung-ausgabe",
            daemon=True,
        ).start()

    def _read_output(self, process: subprocess.Popen[str]) -> None:
        """Read along the node's output: ready message and error texts."""
        if process.stdout is None:
            return
        for line in process.stdout:
            line = line.rstrip()
            if line == _READY:
                self._ready.set()
            elif line:
                self._output.append(line)

    def _output_tail(self) -> str:
        return " | ".join(list(self._output)[-5:])[-1500:]

    def _process_running(self) -> bool:
        process = self._process
        return process is not None and process.poll() is None

    def _stop_process(self) -> None:
        with self._lock:
            process, self._process = self._process, None
        if process is None:
            return
        # Closing stdin is normally enough: the node reads EOF and exits.
        # Only if it hangs, push harder.
        try:
            if process.stdin is not None:
                process.stdin.close()
        except OSError:
            pass
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def _stop_old(self) -> None:
        """Stop a leftover node in the container."""
        pid_file = shlex.quote(_PID_FILE)
        command = (
            f"if test -f {pid_file}; then pid=\"$(cat {pid_file})\"; "
            "if test -r /proc/$pid/cmdline && "
            f"tr \"\\0\" \" \" < /proc/$pid/cmdline | grep -Fq {_TAG}; then "
            "kill -INT \"$pid\" 2>/dev/null || true; sleep 0.3; "
            "kill -KILL \"$pid\" 2>/dev/null || true; fi; "
            f"rm -f {pid_file}; fi"
        )
        subprocess.run(
            [self._docker_bin, "exec", self._container, "bash", "-lc", command],
            capture_output=True, text=True, timeout=6.0, check=False,
        )

    # --- Forwarding ------------------------------------------------------------

    def _loop(self) -> None:
        tick = 1.0 / config.MAP_FEED_HZ
        wait_time = config.RECONNECT_MIN_S
        last_nr: int | None = None
        last_scan_wall: float | None = None

        while not self._stop.is_set():
            if not self._process_running():
                # Node is gone (container restarted, ROS crashed): restart it,
                # with a growing pause so that a permanently broken container
                # is not poked every second.
                output = self._output_tail()
                self._error = (
                    "Lidar-Einspeisung im Container beendet"
                    + (f": {output}" if output else "")
                )
                log.warning("%s -- next attempt in %.0f s", self._error, wait_time)
                self._stop_process()
                if self._stop.wait(wait_time):
                    break
                wait_time = min(wait_time * 2, config.RECONNECT_MAX_S)
                try:
                    self._stop_old()
                    self._start_process()
                    self._restarts += 1
                except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                    self._error = str(exc)
                continue

            try:
                # Continuously send the current pose so that TF does not go
                # stale between scans (RViz, wait times in SLAM).
                pose = self._scan_odo.pose()
                if pose is not None:
                    self._send_pose(time.time(), pose)

                measurement = self._scan_odo.measurement()
                if measurement is not None and measurement.nr != last_nr:
                    last_nr = measurement.nr
                    scan = measurement.scan
                    # Scan.timestamp is monotonic time; ROS needs wall-clock time.
                    wall = time.time() - (time.monotonic() - measurement.timestamp)
                    # Send the pose at exactly the measurement time along:
                    # slam_toolbox looks it up for the scan's timestamp.
                    self._send_pose(wall, measurement.pose)
                    duration = 1.0 / 5.5
                    if last_scan_wall is not None and 0 < wall - last_scan_wall < 0.5:
                        duration = wall - last_scan_wall
                    last_scan_wall = wall
                    self._send({
                        "typ": "scan", "t": wall, "dauer": round(duration, 4),
                        "r": grid(scan.angle, scan.ranges, config.MAP_SCAN_BEAMS),
                    })
                    self._scans += 1
                    self._last_scan = time.monotonic()
                    self._error = None
                    wait_time = config.RECONNECT_MIN_S
            except (OSError, ValueError) as exc:
                # Broken pipe or already closed: the node is gone.
                # The next round restarts it.
                if not self._stop.is_set():
                    self._error = f"Weitergabe an ROS unterbrochen: {exc}"
                    self._stop_process()
                continue

            self._stop.wait(tick)

    def _send_pose(self, t: float, pose) -> None:
        self._send({"typ": "odom", "t": t, "x": pose[0], "y": pose[1], "theta": pose[2]})

    def _send(self, message: dict) -> None:
        process = self._process
        if process is None or process.stdin is None:
            raise ValueError("Knoten nicht verbunden")
        process.stdin.write(json.dumps(message, separators=(",", ":")) + "\n")
        process.stdin.flush()


def grid(angle: list[float], ranges: list[float], rays: int) -> list[float]:
    """Put points onto a fixed angle grid from -180° to just under +180°.

    Our angles (0° forward, counterclockwise) already match the ROS
    convention, only the range is 0..360 instead of -180..180. If two points
    fall into the same beam, the nearer one counts -- missing an obstacle
    would be worse than entering one slightly too close.
    """
    values = [0.0] * rays
    step = 360.0 / rays
    for w, r in zip(angle, ranges):
        i = int(round(((w + 180.0) % 360.0) / step)) % rays
        if values[i] == 0.0 or r < values[i]:
            values[i] = r
    return [round(r, 3) for r in values]
