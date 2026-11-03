"""Read nav_msgs/OccupancyGrid from ROS 2 inside the x3 Docker container.

No ROS 2 Python package is required on the host. A small subscriber is started
inside the container with ``docker exec`` and sends compact JSON lines to this
process. Occupancy values are transported as base64-encoded signed int8 bytes.

The resulting map dictionary has this shape::

    {
        "topic": "/map",
        "frame_id": "map",
        "stamp": {"sec": 0, "nanosec": 0},
        "info": {
            "resolution": 0.05, "width": 100, "height": 100,
            "origin": {"position": {"x": 0, "y": 0, "z": 0},
                       "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
        },
        "data_b64": "...",
        "received_at": 1234567890.0
    }

FastAPI can return ``get_latest_map()`` from a WebSocket polling loop.
"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import threading
import time
from typing import Any


_JSON_PREFIX = "ROSMAP_JSON:"


_ROS_SUBSCRIBER_SCRIPT = r'''import base64
import json
import os
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

stop_requested = threading.Event()

def request_stop(_signum, _frame):
    stop_requested.set()

signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('rosmaster_web_map_bridge')
        topic = os.environ.get('ROSMAP_TOPIC', '/map')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self.on_map, qos
        )
        self.topic = topic
        self.get_logger().info('Listening for occupancy grids on ' + topic)

    def on_map(self, msg):
        p = msg.info.origin.position
        q = msg.info.origin.orientation
        payload = {
            'topic': self.topic,
            'frame_id': msg.header.frame_id,
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'info': {
                'resolution': float(msg.info.resolution),
                'width': int(msg.info.width),
                'height': int(msg.info.height),
                'origin': {
                    'position': {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)},
                    'orientation': {
                        'x': float(q.x), 'y': float(q.y),
                        'z': float(q.z), 'w': float(q.w),
                    },
                },
            },
            'data_b64': base64.b64encode(
                bytes((int(value) & 0xff) for value in msg.data)
            ).decode('ascii'),
        }
        print('ROSMAP_JSON:' + json.dumps(payload, separators=(',', ':')), flush=True)

rclpy.init()
node = MapSubscriber()
try:
    while rclpy.ok() and not stop_requested.is_set():
        rclpy.spin_once(node, timeout_sec=0.25)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''


class ROSMapBridge:
    """Run a ROS 2 OccupancyGrid subscriber inside a Docker container."""

    def __init__(
        self,
        container: str | None = None,
        setup_file: str | None = None,
        topic: str | None = None,
        docker_bin: str | None = None,
    ) -> None:
        self.container = container or os.getenv("ROSMAP_CONTAINER", "x3")
        self.setup_file = setup_file or os.getenv(
            "ROSMAP_ROS_SETUP", "/opt/ros/foxy/setup.bash"
        )
        self.topic = topic or os.getenv("ROSMAP_TOPIC", "/map")
        self.docker_bin = docker_bin or shutil.which("docker") or "docker"
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._latest_map: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._started_at: float | None = None
        self._stopping = False
        self._pid_file = "/tmp/rosmaster-map-bridge.pid"

    def _docker_exec(self, command: str, timeout: float = 8.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                self.docker_bin,
                "exec",
                self.container,
                "bash",
                "-lc",
                command,
            ],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )

    def start(self) -> None:
        """Start the in-container subscriber. Safe to call more than once."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return

            check = self._docker_exec(f"test -f {shlex.quote(self.setup_file)}")
            if check.returncode != 0:
                raise RuntimeError(
                    f"ROS-2-Setup nicht gefunden: {self.setup_file}. "
                    "Prüfe den ROS-Setup-Pfad im Docker-Container."
                )

            inner = (
                f"source {shlex.quote(self.setup_file)} && "
                f"export ROSMAP_TOPIC={shlex.quote(self.topic)} && "
                f"echo $$ > {shlex.quote(self._pid_file)} && "
                f"exec python3 -u -c {shlex.quote(_ROS_SUBSCRIBER_SCRIPT)}"
            )
            command = [
                self.docker_bin,
                "exec",
                self.container,
                "bash",
                "-lc",
                inner,
            ]
            try:
                self._process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    bufsize=1,
                )
            except OSError as exc:
                raise RuntimeError(f"ROS-Map-Bridge konnte nicht starten: {exc}") from exc

            self._last_error = None
            self._stopping = False
            self._started_at = time.time()
            self._reader = threading.Thread(
                target=self._read_output,
                name="rosmap-bridge-reader",
                daemon=True,
            )
            self._reader.start()

    def _read_output(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        try:
            for line in process.stdout:
                line = line.strip()
                if not line.startswith(_JSON_PREFIX):
                    if line and ("error" in line.lower() or "exception" in line.lower()):
                        with self._condition:
                            self._last_error = line[-1000:]
                    continue
                try:
                    payload = json.loads(line[len(_JSON_PREFIX) :])
                    payload["received_at"] = time.time()
                    with self._condition:
                        self._latest_map = payload
                        self._last_error = None
                        self._condition.notify_all()
                except (json.JSONDecodeError, TypeError) as exc:
                    with self._condition:
                        self._last_error = f"Ungültige Kartendaten vom ROS-Bridge-Prozess: {exc}"
        finally:
            return_code = process.poll()
            with self._condition:
                if not self._stopping and return_code not in (None, 0):
                    self._last_error = (
                        f"ROS-Map-Bridge wurde beendet (Exit-Code {return_code}). "
                        "Prüfe ROS-2-Logs und das Topic /map."
                    )
                self._condition.notify_all()

    def get_latest_map(self) -> dict[str, Any] | None:
        """Return the latest received map, or None before the first map."""
        with self._lock:
            if self._latest_map is None:
                return None
            # The payload is treated as immutable after publication.
            return dict(self._latest_map)

    def wait_for_map(self, timeout: float = 10.0) -> dict[str, Any] | None:
        """Wait until the first map arrives, up to ``timeout`` seconds."""
        with self._condition:
            if self._latest_map is None:
                self._condition.wait_for(
                    lambda: self._latest_map is not None or self._last_error is not None,
                    timeout=max(0.0, timeout),
                )
            return dict(self._latest_map) if self._latest_map is not None else None

    def status(self) -> dict[str, Any]:
        process = self._process
        running = process is not None and process.poll() is None
        latest = self.get_latest_map()
        with self._lock:
            return {
                "running": running,
                "topic": self.topic,
                "map_available": latest is not None,
                "last_map_at": latest.get("received_at") if latest else None,
                "started_at": self._started_at,
                "error": self._last_error,
            }

    def stop(self) -> None:
        """Stop the subscriber gracefully, then terminate the docker exec client."""
        with self._lock:
            process = self._process
            if process is None:
                return
            self._stopping = True

        # Signal the Python ROS node inside the container; this also releases
        # its ROS subscription cleanly rather than merely killing the CLI.
        try:
            self._docker_exec(
                f"if test -f {shlex.quote(self._pid_file)}; then "
                f"pid=\"$(cat {shlex.quote(self._pid_file)})\"; "
                "if test -r /proc/$pid/cmdline && tr \"\\0\" \" \" < /proc/$pid/cmdline | grep -Fq rosmaster_web_map_bridge; then "
                "kill -INT \"$pid\" 2>/dev/null || true; fi; "
                f"rm -f {shlex.quote(self._pid_file)}; fi",
                timeout=4.0,
            )
        except (OSError, subprocess.SubprocessError):
            pass

        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)

        with self._lock:
            self._process = None
            self._reader = None
            self._stopping = False
