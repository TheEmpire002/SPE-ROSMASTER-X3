"""High-level lifecycle manager for ROS 2 SLAM and the live map bridge.

The FastAPI app runs on the Raspberry Pi host. ROS 2 and SLAM run inside the
existing Docker container (default: ``x3``). The manager never starts mapping
just because the web page was opened; call ``start()`` from a Start button.

Environment variables:
    ROSMAP_CONTAINER       Docker container name (default: x3)
    ROSMAP_ROS_SETUP       ROS setup file inside container
                           (default: /opt/ros/foxy/setup.bash)
    ROSMAP_SLAM_PACKAGE    ROS package (default: slam_toolbox)
    ROSMAP_SLAM_LAUNCH     Launch file (default: online_async_launch.py)
    ROSMAP_SLAM_ARGS       Optional launch arguments, split with shell quoting
    ROSMAP_TOPIC           OccupancyGrid topic (default: /map)
    ROSMAP_STORAGE_DIR     Host folder for saved maps

The default launch uses slam_toolbox. It requires a working laser topic and TF
chain in ROS 2 (normally /scan and odom -> base frame). If Yahboom provides a
specific SLAM launch file, configure ROSMAP_SLAM_PACKAGE and ROSMAP_SLAM_LAUNCH
without changing this module.
"""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from .map_service import MapService
from .ros_bridge import ROSMapBridge


class MappingManager:
    """Start/stop SLAM, expose the latest map and save map snapshots."""

    def __init__(
        self,
        container: str | None = None,
        setup_file: str | None = None,
        slam_package: str | None = None,
        slam_launch: str | None = None,
        slam_args: str | None = None,
        map_topic: str | None = None,
        storage_dir: str | Path | None = None,
        docker_bin: str | None = None,
    ) -> None:
        self.container = container or os.getenv("ROSMAP_CONTAINER", "x3")
        self.setup_file = setup_file or os.getenv(
            "ROSMAP_ROS_SETUP", "/opt/ros/foxy/setup.bash"
        )
        self.slam_package = slam_package or os.getenv("ROSMAP_SLAM_PACKAGE", "slam_toolbox")
        self.slam_launch = slam_launch or os.getenv(
            "ROSMAP_SLAM_LAUNCH", "online_async_launch.py"
        )
        self.slam_args = slam_args if slam_args is not None else os.getenv(
            "ROSMAP_SLAM_ARGS", "use_sim_time:=false"
        )
        self.map_topic = map_topic or os.getenv("ROSMAP_TOPIC", "/map")
        self.docker_bin = docker_bin or shutil.which("docker") or "docker"
        self.bridge = ROSMapBridge(
            container=self.container,
            setup_file=self.setup_file,
            topic=self.map_topic,
            docker_bin=self.docker_bin,
        )
        self.map_service = MapService(storage_dir=storage_dir)
        self._lock = threading.RLock()
        self._started_at: float | None = None
        self._last_error: str | None = None
        self._slam_pid_file = "/tmp/rosmaster-slam.pid"
        self._slam_log_file = "/tmp/rosmaster-slam.log"

    def _docker(self, *args: str, timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [self.docker_bin, *args],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )

    def _container_running(self) -> bool:
        result = self._docker(
            "inspect", "-f", "{{.State.Running}}", self.container, timeout=5.0
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Docker-Container '{self.container}' wurde nicht gefunden: "
                f"{(result.stderr or result.stdout).strip()}"
            )
        return result.stdout.strip().lower() == "true"

    def _ensure_container(self) -> None:
        if not self._container_running():
            result = self._docker("start", self.container, timeout=30.0)
            if result.returncode != 0:
                raise RuntimeError(
                    f"Docker-Container '{self.container}' konnte nicht gestartet werden: "
                    f"{(result.stderr or result.stdout).strip()}"
                )
        check = self._docker(
            "exec", self.container, "bash", "-lc",
            f"test -f {shlex.quote(self.setup_file)}",
            timeout=8.0,
        )
        if check.returncode != 0:
            raise RuntimeError(
                f"ROS-2-Setup fehlt im Container: {self.setup_file}"
            )

    def _slam_is_running(self) -> bool:
        command = (
            f"if test -f {shlex.quote(self._slam_pid_file)}; then "
            f"pid=\"$(cat {shlex.quote(self._slam_pid_file)})\"; "
            "test -r /proc/$pid/cmdline && "
            "tr \"\\0\" \" \" < /proc/$pid/cmdline | grep -Fq 'ros2 launch'; "
            "else exit 1; fi"
        )
        result = self._docker(
            "exec", self.container, "bash", "-lc", command, timeout=5.0
        )
        return result.returncode == 0

    def _start_slam(self) -> None:
        if self._slam_is_running():
            return

        args = shlex.split(self.slam_args)
        ros_command = ["ros2", "launch", self.slam_package, self.slam_launch, *args]
        launch_command = " ".join(shlex.quote(part) for part in ros_command)
        inner = (
            f"source {shlex.quote(self.setup_file)} && "
            f"exec {launch_command}"
        )
        background = (
            f"rm -f {shlex.quote(self._slam_pid_file)}; "
            f"nohup bash -lc {shlex.quote(inner)} "
            f">{shlex.quote(self._slam_log_file)} 2>&1 </dev/null & "
            f"echo $! > {shlex.quote(self._slam_pid_file)}; "
            "sleep 0.3; "
            f"if kill -0 \"$(cat {shlex.quote(self._slam_pid_file)})\" 2>/dev/null; "
            "then echo STARTED; else "
            f"cat {shlex.quote(self._slam_log_file)} 2>/dev/null || true; exit 1; fi"
        )
        result = self._docker(
            "exec", self.container, "bash", "-lc", background, timeout=12.0
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                "SLAM konnte nicht gestartet werden. "
                f"Launch: ros2 launch {self.slam_package} {self.slam_launch}. "
                f"Docker-Ausgabe: {detail[-3000:]}"
            )

    def _stop_slam(self) -> None:
        if not self._container_running():
            return
        command = (
            f"if test -f {shlex.quote(self._slam_pid_file)}; then "
            f"pid=\"$(cat {shlex.quote(self._slam_pid_file)})\"; "
            "if test -r /proc/$pid/cmdline && "
            "tr \"\\0\" \" \" < /proc/$pid/cmdline | grep -Fq 'ros2 launch'; then "
            "kill -INT \"$pid\" 2>/dev/null || true; "
            "for n in 1 2 3 4 5 6 7 8 9 10; do "
            "kill -0 \"$pid\" 2>/dev/null || break; sleep 0.5; done; "
            "kill -TERM \"$pid\" 2>/dev/null || true; "
            "sleep 0.2; kill -KILL \"$pid\" 2>/dev/null || true; fi; "
            f"rm -f {shlex.quote(self._slam_pid_file)}; fi"
        )
        self._docker("exec", self.container, "bash", "-lc", command, timeout=10.0)

    def start(self) -> dict[str, Any]:
        """Ensure x3 is running, then start the map subscriber and SLAM."""
        with self._lock:
            try:
                self._ensure_container()
                self.bridge.start()
                self._start_slam()
                self._started_at = time.time()
                self._last_error = None
                return self.status()
            except Exception as exc:
                self._last_error = str(exc)
                # Avoid leaving the bridge running after a failed SLAM launch.
                self.bridge.stop()
                raise

    def stop(self) -> dict[str, Any]:
        """Stop SLAM and the ROS map subscriber; leave Docker container up."""
        with self._lock:
            try:
                self._stop_slam()
            finally:
                self.bridge.stop()
            self._started_at = None
            return self.status()

    def get_map(self) -> dict[str, Any] | None:
        return self.bridge.get_latest_map()

    def wait_for_map(self, timeout: float = 10.0) -> dict[str, Any] | None:
        return self.bridge.wait_for_map(timeout=timeout)

    def save_map(self, name: str | None = None) -> dict[str, Any]:
        """Save the most recent received map to PGM + YAML on the Pi host."""
        payload = self.get_map()
        if payload is None:
            raise RuntimeError(
                "Es liegt noch keine Karte vor. Starte das Mapping und warte, "
                "bis ROS ein /map-Topic veröffentlicht."
            )
        return self.map_service.save_map(payload, name=name)

    def list_maps(self) -> list[dict[str, Any]]:
        return self.map_service.list_maps()

    def status(self) -> dict[str, Any]:
        try:
            container_running = self._container_running()
        except Exception as exc:
            container_running = False
            docker_error = str(exc)
        else:
            docker_error = None
        slam_running = container_running and self._slam_is_running()
        bridge_status = self.bridge.status()
        return {
            "container": self.container,
            "container_running": container_running,
            "mapping_running": slam_running,
            "bridge_running": bridge_status["running"],
            "map_available": bridge_status["map_available"],
            "map_topic": self.map_topic,
            "last_map_at": bridge_status["last_map_at"],
            "started_at": self._started_at,
            "error": self._last_error or bridge_status.get("error") or docker_error,
            "slam_log_file": self._slam_log_file if container_running else None,
        }
