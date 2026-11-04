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

/scan und TF kommen von unserem eigenen Server: Er hält Lidar und Board und
speist beides über ``einspeisung.py`` in den Container ein (E8 in
docs/entscheidungen.md). sllidar_node und yahboomcar_bringup dürfen dafür
nicht laufen.
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
from .einspeisung import Einspeisung
from .karte import Karte, zusatz_lesen, zusatz_schreiben
from .map_service import MapService
from .ros_bridge import ROSMapBridge
from .simulation import SimKartierung


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
        scan_odometrie=None,
        ortung=None,
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
        # Lidar-Odometrie: liefert Position und Blickrichtung für SLAM und
        # die Kartierfahrt. Läuft nur, solange kartiert wird (kostet CPU).
        self.scan_odometrie = scan_odometrie
        # Ohne sie (etwa in Tests) gibt es nichts einzuspeisen; dann muss
        # ROS /scan und TF selbst liefern.
        self.einspeisung = (
            Einspeisung(lidar, scan_odometrie, self.container, self.setup_file, self.docker_bin)
            if scan_odometrie is not None
            else None
        )
        # Lage in der Karte (navigation/kartenortung.py). Sie bekommt die
        # Karte vom Server-Takt; hier wird sie nur vor einer neuen Kartierung
        # geleert und mit gespeicherten Karten versorgt.
        self.ortung = ortung
        # --mock: Karte in Python statt slam_toolbox im Container.
        self.simulation = (
            SimKartierung(scan_odometrie) if simulation and scan_odometrie is not None else None
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
            # Manche Container beenden sich sofort wieder, etwa wenn sie mit
            # einer Shell ohne "-it" angelegt wurden. Ohne diese Prüfung
            # scheiterte erst das nächste exec, und die Meldung sagte
            # irreführend "ROS-2-Setup fehlt".
            time.sleep(1.0)
            if not self._container_running():
                logs = self._docker("logs", "--tail", "10", self.container, timeout=5.0)
                ausgabe = (logs.stdout + logs.stderr).strip()[-800:]
                raise RuntimeError(
                    f"Docker-Container '{self.container}' beendet sich direkt nach dem "
                    "Start wieder. Mit 'docker inspect' Befehl (Cmd) und Tty prüfen. "
                    f"Letzte Ausgabe: {ausgabe or '(keine)'}"
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

    def _slam_paket_pruefen(self) -> None:
        """Vorab prüfen, ob es das SLAM-Paket im Container gibt.

        Sonst startete ``ros2 launch`` scheinbar erfolgreich und scheiterte
        erst danach, ohne dass es irgendwo zu sehen war.
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
        """Letzte Zeilen des SLAM-Protokolls, das im Container liegt."""
        result = self._docker(
            "exec", self.container, "bash", "-lc",
            f"tail -n {int(config.MAP_LOG_ZEILEN)} "
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
            # Läuft noch einer aus einem früheren Lauf (Server neu gestartet,
            # ohne vorher zu stoppen), nicht weiterbenutzen: Er hat die alte
            # Karte im Kopf und einen Ursprung, der nicht zur neu bei (0, 0)
            # beginnenden Position passt. Jeder Start ist eine neue Karte.
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
            # Jede Kartierung beginnt einen neuen Kartenrahmen am jetzigen
            # Standort. Eine geladene Karte (samt Lage darin) gehört nicht
            # mehr dazu; und die Lidar-Odometrie soll neu bei (0, 0) beginnen,
            # wo auch slam_toolbox seinen Ursprung setzt.
            if self.ortung is not None:
                self.ortung.entladen()
            if self.simulation is not None:
                if self.scan_odometrie is not None:
                    self.scan_odometrie.start()
                self.simulation.start()
                self._started_at = time.time()
                self._last_error = None
                return self.status()
            try:
                self._ensure_container()
                self._slam_paket_pruefen()
                # Zuerst die Einspeisung: slam_toolbox soll beim Anlaufen
                # schon Scans und TF vorfinden.
                if self.scan_odometrie is not None:
                    self.scan_odometrie.start()
                if self.einspeisung is not None:
                    self.einspeisung.start()
                self.bridge.start()
                self._start_slam()
                self._started_at = time.time()
                self._last_error = None
                return self.status()
            except Exception as exc:
                self._last_error = str(exc)
                # Avoid leaving the bridge running after a failed SLAM launch.
                self.bridge.stop()
                if self.einspeisung is not None:
                    self.einspeisung.stop()
                if self.scan_odometrie is not None:
                    self.scan_odometrie.stop()
                raise

    def stop(self) -> dict[str, Any]:
        """Stop SLAM and the ROS map subscriber; leave Docker container up."""
        with self._lock:
            if self.simulation is not None:
                self.simulation.stop()
                if self.scan_odometrie is not None:
                    self.scan_odometrie.stop()
                self._started_at = None
                return self.status()
            try:
                self._stop_slam()
            finally:
                self.bridge.stop()
                if self.einspeisung is not None:
                    self.einspeisung.stop()
                if self.scan_odometrie is not None:
                    self.scan_odometrie.stop()
            self._started_at = None
            return self.status()

    @property
    def aktiv(self) -> bool:
        """Gestartet und nicht gestoppt (SLAM kann trotzdem abgestürzt sein)."""
        return self._started_at is not None

    def kartierte_karte(self) -> dict[str, Any] | None:
        """Die Karte aus der (laufenden oder letzten) Kartierung."""
        if self.simulation is not None:
            return self.simulation.get_latest_map()
        return self.bridge.get_latest_map()

    def get_map(self) -> dict[str, Any] | None:
        """Was die Seite /map zeigen soll: die Karte, mit der er gerade fährt.

        Ist eine gespeicherte Karte geladen, ist es diese, sonst die aus
        der Kartierung.
        """
        if self.ortung is not None and self.ortung.karte is not None and not self.ortung.live:
            return self.ortung.karte.als_payload()
        return self.kartierte_karte()

    def wait_for_map(self, timeout: float = 10.0) -> dict[str, Any] | None:
        return self.bridge.wait_for_map(timeout=timeout)

    def save_map(self, name: str | None = None) -> dict[str, Any]:
        """Save the most recent received map to PGM + YAML on the Pi host.

        Dazu kommt eine kleine JSON-Datei mit dem Ausgangspunkt der Karte
        ("zu Hause"): Dort wird nach dem Laden zuerst nach der eigenen Lage
        gesucht, und dorthin fährt die Heimfahrt mit Karte.
        """
        payload = self.get_map()
        if payload is None:
            raise RuntimeError(
                "Es liegt noch keine Karte vor. Starte das Mapping und warte, "
                "bis ROS ein /map-Topic veröffentlicht."
            )
        ergebnis = self.map_service.save_map(payload, name=name)
        start = list(self.ortung.start) if self.ortung is not None else [0.0, 0.0, 0.0]
        zusatz_schreiben(ergebnis["yaml"], {"start": start, "gespeichert": time.time()})
        return ergebnis

    def list_maps(self) -> list[dict[str, Any]]:
        return self.map_service.list_maps()

    def load_map(self, name: str) -> dict[str, Any]:
        """Gespeicherte Karte laden, um darin zu fahren (Ortung sucht dann die Lage)."""
        if self.ortung is None:
            raise RuntimeError("Ortung in der Karte ist nicht eingerichtet.")
        if self.aktiv:
            raise RuntimeError("Erst das Mapping stoppen: Gerade entsteht eine neue Karte.")
        eintrag = next((m for m in self.list_maps() if m["name"] == name), None)
        if eintrag is None or not eintrag.get("image"):
            raise ValueError(f"Karte '{name}' nicht gefunden.")
        karte = Karte.laden(eintrag["yaml"])
        start = zusatz_lesen(eintrag["yaml"]).get("start") or [0.0, 0.0, 0.0]
        self.ortung.karte_setzen(karte, start=tuple(float(v) for v in start[:3]), live=False)
        return {"name": karte.name, "breite": karte.breite, "hoehe": karte.hoehe,
                "aufloesung": karte.aufloesung, "start": start}

    def start_merken(self, start) -> None:
        """Neuer Ausgangspunkt für die geladene Karte -- auch in ihrer Datei."""
        if self.ortung is None or self.ortung.karte is None:
            return
        self.ortung.start_setzen(start)
        name = self.ortung.karte.name
        if not name or self.ortung.live:
            return
        eintrag = next((m for m in self.list_maps() if m["name"] == name), None)
        if eintrag is not None:
            daten = zusatz_lesen(eintrag["yaml"])
            daten["start"] = [float(v) for v in start[:3]]
            zusatz_schreiben(eintrag["yaml"], daten)

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
        einspeisung = self.einspeisung.status() if self.einspeisung is not None else None
        aktiv = self._started_at is not None
        slam_log: list[str] = []
        if aktiv and container_running:
            try:
                slam_log = self._slam_log()
            except (OSError, subprocess.SubprocessError):
                pass
        hinweis, hinweis_fehler = self._hinweis(
            aktiv, slam_running, bridge_status, einspeisung
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
            # Ab hier: Rückmeldung, woran es gerade hängt (für die Seite /map)
            "aktiv": aktiv,
            "einspeisung": einspeisung,
            "lidar_odometrie": (
                self.scan_odometrie.status() if self.scan_odometrie is not None else None
            ),
            "slam_log": slam_log,
            "hinweis": hinweis,
            "hinweis_fehler": hinweis_fehler,
        }

    def _status_simulation(self) -> dict[str, Any]:
        """Dieselben Felder wie ``status()``, ohne Docker zu fragen."""
        sim = self.simulation.status()
        aktiv = self._started_at is not None
        if not aktiv:
            hinweis = "Mapping ist gestoppt."
        elif not sim["map_available"]:
            hinweis = "Simulation: warte auf die erste Karte …"
        else:
            hinweis = (f"Simulation: Karte aus dem simulierten Lidar "
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
            "aktiv": aktiv,
            "einspeisung": None,
            "lidar_odometrie": (
                self.scan_odometrie.status() if self.scan_odometrie is not None else None
            ),
            "slam_log": [],
            "hinweis": hinweis,
            "hinweis_fehler": False,
            "simulation": True,
        }

    def _hinweis(
        self,
        aktiv: bool,
        slam_running: bool,
        bridge: dict[str, Any],
        einspeisung: dict[str, Any] | None,
    ) -> tuple[str, bool]:
        """In einem Satz: was gerade passiert oder woran es hängt.

        Die Prüfungen folgen dem Weg der Daten -- Lidar, Einspeisung, SLAM,
        Karte. Der erste Schritt, der hakt, ist die Ursache.
        """
        if not aktiv:
            return "Mapping ist gestoppt.", False
        if not slam_running:
            return (
                "Der SLAM-Prozess hat sich beendet. Die Ursache steht im "
                "SLAM-Protokoll unten.", True,
            )
        if einspeisung is not None:
            lidar_alter = einspeisung["lidar_scan_vor_s"]
            if lidar_alter is None or lidar_alter > 2.0:
                return (
                    "Der Lidar liefert keine Daten, ohne Scans keine Karte. "
                    "Zustand: /lidar/status", True,
                )
            if not einspeisung["laeuft"]:
                return (
                    "Die Weitergabe des Lidars an ROS ist unterbrochen und "
                    f"startet neu. {einspeisung['fehler'] or ''}".strip(), True,
                )
            if not einspeisung["bereit"]:
                return "Verbinde den Lidar mit ROS …", False
        if not bridge["running"]:
            return (
                "Der Kartenempfang ist beendet. "
                f"{bridge.get('error') or ''}".strip(), True,
            )
        if not bridge["map_available"]:
            seit = time.time() - (self._started_at or time.time())
            scans = f" {einspeisung['scans']} Scans an ROS gegeben." if einspeisung else ""
            if seit < config.MAP_ERSTE_KARTE_S:
                return f"SLAM läuft, warte auf die erste Karte ({seit:.0f} s).{scans}", False
            return (
                f"Seit {seit:.0f} s keine Karte.{scans} Die Ursache steht "
                "vermutlich im SLAM-Protokoll unten.", True,
            )
        alter = time.time() - (bridge["last_map_at"] or time.time())
        return f"Karte wird aufgebaut, letzte Aktualisierung vor {alter:.0f} s.", False
