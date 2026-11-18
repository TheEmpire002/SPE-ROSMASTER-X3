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

/scan and TF come from our own server: it holds the lidar and the board and
feeds both into the container via ``scan_feed.py`` (E8 in
docs/decisions.md). sllidar_node and yahboomcar_bringup must not run for
this.
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

from .. import config
from .scan_feed import ScanFeed
from .occ_map import OccupancyMap, read_extra, write_extra
from .map_service import MapService
from .ros_bridge import ROSMapBridge
from .simulation import SimMapping


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
        lidar=None,
        scan_odometry=None,
        localization=None,
        simulation: bool = False,
    ) -> None:
        self.container = container or os.getenv("ROSMAP_CONTAINER", config.MAP_CONTAINER)
        self.setup_file = setup_file or os.getenv("ROSMAP_ROS_SETUP", config.MAP_ROS_SETUP)
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
        # Lidar odometry: provides position and heading for SLAM and the
        # mapping drive. Runs only while mapping (costs CPU).
        self.scan_odometry = scan_odometry
        # Without it (e.g. in tests) there is nothing to feed in; then ROS
        # must provide /scan and TF itself.
        self.scan_feed = (
            ScanFeed(lidar, scan_odometry, self.container, self.setup_file, self.docker_bin)
            if scan_odometry is not None
            else None
        )
        # Pose in the map (navigation/map_localization.py). It receives the
        # map from the server loop; here it is only cleared before a new
        # mapping run and supplied with saved maps.
        self.localization = localization
        # --mock: map in Python instead of slam_toolbox in the container.
        self.simulation = (
            SimMapping(scan_odometry) if simulation and scan_odometry is not None else None
        )
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
            # Some containers exit again immediately, e.g. when they were
            # created with a shell but without "-it". Without this check only
            # the next exec failed, and the message misleadingly said
            # "ROS-2-Setup fehlt".
            time.sleep(1.0)
            if not self._container_running():
                logs = self._docker("logs", "--tail", "10", self.container, timeout=5.0)
                output = (logs.stdout + logs.stderr).strip()[-800:]
                raise RuntimeError(
                    f"Docker-Container '{self.container}' beendet sich direkt nach dem "
                    "Start wieder. Mit 'docker inspect' Befehl (Cmd) und Tty prüfen. "
                    f"Letzte Ausgabe: {output or '(keine)'}"
                )
        check = self._docker(
            "exec", self.container, "bash", "-lc",
            f"test -f {shlex.quote(self.setup_file)}",
            timeout=8.0,
        )
        if check.returncode != 0:
            detail = (check.stderr or check.stdout).strip()
            raise RuntimeError(
                f"ROS-2-Setup fehlt im Container: {self.setup_file}"
                + (f" ({detail[-300:]})" if detail else "")
            )

    def _check_slam_package(self) -> None:
        """Check up front whether the SLAM package exists in the container.

        Otherwise ``ros2 launch`` appeared to start successfully and only
        failed afterwards, without this being visible anywhere.
        """
        result = self._docker(
            "exec", self.container, "bash", "-lc",
            f"source {shlex.quote(self.setup_file)} && "
            f"ros2 pkg prefix {shlex.quote(self.slam_package)}",
            timeout=20.0,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Das ROS-Paket '{self.slam_package}' fehlt im Container "
                f"'{self.container}'. Installieren mit: docker exec -it "
                f"{self.container} apt install ros-foxy-slam-toolbox "
                f"({(result.stderr or result.stdout).strip()[-300:]})"
            )

    def _slam_log(self) -> list[str]:
        """Last lines of the SLAM log that lives in the container."""
        result = self._docker(
            "exec", self.container, "bash", "-lc",
            f"tail -n {int(config.MAP_LOG_LINES)} "
            f"{shlex.quote(self._slam_log_file)} 2>/dev/null",
            timeout=5.0,
        )
        return [z for z in result.stdout.splitlines() if z.strip()]

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
            # If one is still running from an earlier run (server restarted
            # without stopping first), do not reuse it: it has the old map in
            # mind and an origin that does not match the position, which
            # starts fresh at (0, 0). Every start is a new map.
            self._stop_slam()

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
            # Every mapping run starts a new map frame at the current
            # location. A loaded map (including the pose in it) no longer
            # belongs to it. slam_toolbox maps in the lidar odometry frame;
            # with permanent odometry (user "immer") that frame keeps
            # running, otherwise it starts afresh at (0, 0).
            if self.localization is not None:
                self.localization.unload()
            if self.simulation is not None:
                if self.scan_odometry is not None:
                    self.scan_odometry.start()
                self.simulation.start()
                self._started_at = time.time()
                self._last_error = None
                return self.status()
            try:
                self._ensure_container()
                self._check_slam_package()
                # Scan feed first: slam_toolbox should already find scans
                # and TF when it comes up.
                if self.scan_odometry is not None:
                    self.scan_odometry.start()
                if self.scan_feed is not None:
                    self.scan_feed.start()
                self.bridge.start()
                self._start_slam()
                self._started_at = time.time()
                self._last_error = None
                return self.status()
            except Exception as exc:
                self._last_error = str(exc)
                # Avoid leaving the bridge running after a failed SLAM launch.
                self.bridge.stop()
                if self.scan_feed is not None:
                    self.scan_feed.stop()
                if self.scan_odometry is not None:
                    self.scan_odometry.stop()
                raise

    def stop(self) -> dict[str, Any]:
        """Stop SLAM and the ROS map subscriber; leave Docker container up."""
        with self._lock:
            if self.simulation is not None:
                self.simulation.stop()
                if self.scan_odometry is not None:
                    self.scan_odometry.stop()
                self._started_at = None
                return self.status()
            try:
                self._stop_slam()
            finally:
                self.bridge.stop()
                if self.scan_feed is not None:
                    self.scan_feed.stop()
                if self.scan_odometry is not None:
                    self.scan_odometry.stop()
            self._started_at = None
            return self.status()

    @property
    def active(self) -> bool:
        """Started and not stopped (SLAM may still have crashed)."""
        return self._started_at is not None

    def mapped_map(self) -> dict[str, Any] | None:
        """The map from the (current or last) mapping run."""
        if self.simulation is not None:
            return self.simulation.get_latest_map()
        return self.bridge.get_latest_map()

    def get_map(self) -> dict[str, Any] | None:
        """What the /map page should show: the map the robot is currently using.

        If a saved map is loaded, it is that one, otherwise the one from
        mapping.
        """
        if self.localization is not None and self.localization.occ_map is not None and not self.localization.live:
            return self.localization.occ_map.as_payload()
        return self.mapped_map()

    def wait_for_map(self, timeout: float = 10.0) -> dict[str, Any] | None:
        return self.bridge.wait_for_map(timeout=timeout)

    def save_map(self, name: str | None = None) -> dict[str, Any]:
        """Save the most recent received map to PGM + YAML on the Pi host.

        Plus a small JSON file with the map's starting point ("home"): after
        loading, the robot first searches for its own pose there, and homing
        with a map drives back there.
        """
        payload = self.get_map()
        if payload is None:
            raise RuntimeError(
                "Es liegt noch keine Karte vor. Starte das Mapping und warte, "
                "bis ROS ein /map-Topic veröffentlicht."
            )
        result = self.map_service.save_map(payload, name=name)
        start = list(self.localization.start) if self.localization is not None else [0.0, 0.0, 0.0]
        write_extra(result["yaml"], {"start": start, "gespeichert": time.time()})
        return result

    def list_maps(self) -> list[dict[str, Any]]:
        return self.map_service.list_maps()

    def load_map(self, name: str) -> dict[str, Any]:
        """Load a saved map to drive in it (localization then searches for the pose)."""
        if self.localization is None:
            raise RuntimeError("Ortung in der Karte ist nicht eingerichtet.")
        if self.active:
            raise RuntimeError("Erst das Mapping stoppen: Gerade entsteht eine neue Karte.")
        entry = next((m for m in self.list_maps() if m["name"] == name), None)
        if entry is None or not entry.get("image"):
            raise ValueError(f"Karte '{name}' nicht gefunden.")
        occ_map = OccupancyMap.load(entry["yaml"])
        start = read_extra(entry["yaml"]).get("start") or [0.0, 0.0, 0.0]
        self.localization.set_map(occ_map, start=tuple(float(v) for v in start[:3]), live=False)
        return {"name": occ_map.name, "breite": occ_map.width, "hoehe": occ_map.height,
                "aufloesung": occ_map.resolution, "start": start}

    def remember_start(self, start) -> None:
        """New starting point for the loaded map -- also in its file."""
        if self.localization is None or self.localization.occ_map is None:
            return
        self.localization.set_start(start)
        name = self.localization.occ_map.name
        if not name or self.localization.live:
            return
        entry = next((m for m in self.list_maps() if m["name"] == name), None)
        if entry is not None:
            data = read_extra(entry["yaml"])
            data["start"] = [float(v) for v in start[:3]]
            write_extra(entry["yaml"], data)

    def status(self) -> dict[str, Any]:
        if self.simulation is not None:
            return self._status_simulation()
        try:
            container_running = self._container_running()
        except Exception as exc:
            container_running = False
            docker_error = str(exc)
        else:
            docker_error = None
        slam_running = container_running and self._slam_is_running()
        bridge_status = self.bridge.status()
        scan_feed = self.scan_feed.status() if self.scan_feed is not None else None
        active = self._started_at is not None
        slam_log: list[str] = []
        if active and container_running:
            try:
                slam_log = self._slam_log()
            except (OSError, subprocess.SubprocessError):
                pass
        hint, hint_error = self._hint(
            active, slam_running, bridge_status, scan_feed
        )
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
            # From here on: feedback on what is currently blocking (for the /map page)
            "aktiv": active,
            "einspeisung": scan_feed,
            "lidar_odometrie": (
                self.scan_odometry.status() if self.scan_odometry is not None else None
            ),
            "slam_log": slam_log,
            "hinweis": hint,
            "hinweis_fehler": hint_error,
        }

    def _status_simulation(self) -> dict[str, Any]:
        """The same fields as ``status()``, without asking Docker."""
        sim = self.simulation.status()
        active = self._started_at is not None
        if not active:
            hint = "Mapping ist gestoppt."
        elif not sim["map_available"]:
            hint = "Simulation: warte auf die erste Karte …"
        else:
            hint = (f"Simulation: Karte aus dem simulierten Lidar "
                       f"({sim['scans']} Aufnahmen, ohne slam_toolbox).")
        return {
            "container": "simulation",
            "container_running": False,
            "mapping_running": sim["running"],
            "bridge_running": sim["running"],
            "map_available": sim["map_available"],
            "map_topic": self.map_topic,
            "last_map_at": sim["last_map_at"],
            "started_at": self._started_at,
            "error": self._last_error,
            "slam_log_file": None,
            "aktiv": active,
            "einspeisung": None,
            "lidar_odometrie": (
                self.scan_odometry.status() if self.scan_odometry is not None else None
            ),
            "slam_log": [],
            "hinweis": hint,
            "hinweis_fehler": False,
            "simulation": True,
        }

    def _hint(
        self,
        active: bool,
        slam_running: bool,
        bridge: dict[str, Any],
        scan_feed: dict[str, Any] | None,
    ) -> tuple[str, bool]:
        """In one sentence: what is happening right now or what is blocking.

        The checks follow the path of the data -- lidar, scan feed, SLAM,
        map. The first step that fails is the cause.
        """
        if not active:
            return "Mapping ist gestoppt.", False
        if not slam_running:
            return (
                "Der SLAM-Prozess hat sich beendet. Die Ursache steht im "
                "SLAM-Protokoll unten.", True,
            )
        if scan_feed is not None:
            lidar_age = scan_feed["lidar_scan_vor_s"]
            if lidar_age is None or lidar_age > 2.0:
                return (
                    "Der Lidar liefert keine Daten, ohne Scans keine Karte. "
                    "Zustand: /lidar/status", True,
                )
            if not scan_feed["laeuft"]:
                return (
                    "Die Weitergabe des Lidars an ROS ist unterbrochen und "
                    f"startet neu. {scan_feed['fehler'] or ''}".strip(), True,
                )
            if not scan_feed["bereit"]:
                return "Verbinde den Lidar mit ROS …", False
        if not bridge["running"]:
            return (
                "Der Kartenempfang ist beendet. "
                f"{bridge.get('error') or ''}".strip(), True,
            )
        if not bridge["map_available"]:
            since = time.time() - (self._started_at or time.time())
            scans = f" {scan_feed['scans']} Scans an ROS gegeben." if scan_feed else ""
            if since < config.MAP_FIRST_MAP_S:
                return f"SLAM läuft, warte auf die erste Karte ({since:.0f} s).{scans}", False
            return (
                f"Seit {since:.0f} s keine Karte.{scans} Die Ursache steht "
                "vermutlich im SLAM-Protokoll unten.", True,
            )
        age = time.time() - (bridge["last_map_at"] or time.time())
        return f"Karte wird aufgebaut, letzte Aktualisierung vor {age:.0f} s.", False
