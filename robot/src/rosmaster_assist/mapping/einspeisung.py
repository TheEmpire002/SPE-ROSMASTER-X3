"""Lidar und Position in den ROS-Container einspeisen.

slam_toolbox baut die Karte, braucht dafür aber zwei Dinge aus ROS: die
Lidar-Scans auf ``/scan`` und die Position des Roboters als TF
(``odom -> base_footprint``). Im Yahboom-Stack kämen sie von ``sllidar_node``
und ``yahboomcar_bringup``. Die dürfen hier aber nicht laufen, weil unser
Server Lidar und Board schon offen hat -- zwei Leser am selben Gerät
bekommen beide nur Bruchstücke (docs/hardware.md).

Deshalb läuft es andersherum: Unser Server liest wie immer und reicht die
Daten weiter. Im Container läuft dazu ein kleiner ROS-Knoten, gestartet über
``docker exec -i``. Er bekommt JSON-Zeilen über stdin und veröffentlicht sie
als ROS-Nachrichten. stdin als Kanal, weil es keine Netzwerkeinstellungen
zwischen Pi und Container braucht -- und endet unser Server, bekommt der
Knoten EOF und beendet sich von selbst, statt mit veralteten Daten
weiterzulaufen.

Zeilen an den Knoten (Zeiten als Unix-Zeit, der Container teilt die Uhr
mit dem Pi)::

    {"typ": "odom", "t": .., "x": .., "y": .., "theta": ..}
    {"typ": "scan", "t": .., "dauer": .., "r": [..]}     (0 = kein Echo)
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

# Rahmennamen wie in der Standard-Konfiguration von slam_toolbox (Foxy:
# base_frame = base_footprint). base_link wird zusätzlich veröffentlicht,
# damit auch eine Konfiguration mit base_link als Basis funktioniert.
_ODOM_FRAME = "odom"
_BASIS_FRAME = "base_footprint"
_LASER_FRAME = "laser"

# Steht im Knotennamen und damit in der Kommandozeile des Prozesses. Daran
# wird ein übrig gebliebener Knoten erkannt, bevor er beendet wird.
_KENNUNG = "rosmaster_scan_einspeisung"
_PID_DATEI = "/tmp/rosmaster-scan-einspeisung.pid"
_BEREIT = "ROSFEED_BEREIT"

# Wie lange der Start auf den Knoten wartet. ROS2 hochzufahren dauert am
# Pi ein, zwei Sekunden; scheitert es (fehlendes Paket), soll der Fehler
# gleich beim Starten auf der Seite stehen und nicht erst irgendwann.
_BEREIT_WARTEN_S = 10.0


_KNOTEN = r'''import json
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
# Der Lidar sitzt für uns in der Mitte und zeigt nach vorn: Einbaulage und
# Drehsinn sind schon in unserem Server herausgerechnet.
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


class Einspeisung:
    """Reicht Lidar-Scans und Position laufend an ROS im Container weiter."""

    def __init__(
        self, lidar, scan_odometrie, container: str, setup_file: str, docker_bin: str
    ) -> None:
        self._lidar = lidar
        # Liefert zu jeder Aufnahme die Position samt Blickrichtung aus dem
        # Lidar-Abgleich (navigation/scan_odometrie.py). Die Räder allein
        # taugten nicht: Am X3 kommt von ihnen keine brauchbare Drehung.
        self._scan_odo = scan_odometrie
        self._container = container
        self._setup_file = setup_file
        self._docker_bin = docker_bin

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._prozess: subprocess.Popen[str] | None = None
        self._bereit = threading.Event()
        self._ausgabe: deque[str] = deque(maxlen=20)

        self._scans = 0
        self._letzter_scan: float | None = None  # monotonic
        self._fehler: str | None = None
        self._neustarts = 0

    # --- Steuerung -----------------------------------------------------------

    def start(self) -> None:
        """Knoten im Container starten. Wirft, wenn er nicht hochkommt."""
        if self._lidar is None:
            raise RuntimeError(
                "Ohne Lidar keine Karte -- LIDAR_ENABLED ist in config.py ausgeschaltet."
            )
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop.clear()
        self._scans = 0
        self._letzter_scan = None
        self._fehler = None
        self._neustarts = 0

        # Ein Knoten aus einem abgestürzten früheren Lauf veröffentlichte
        # sonst weiter seine eigene, eingefrorene Position auf /tf -- zwei
        # widersprüchliche Positionen zerreißen die Karte.
        self._alten_beenden()
        self._prozess_starten()
        if not self._bereit.wait(_BEREIT_WARTEN_S):
            prozess = self._prozess
            if prozess is None or prozess.poll() is not None:
                ausgabe = self._ausgabe_ende()
                self._prozess_beenden()
                raise RuntimeError(
                    "Lidar-Einspeisung in ROS ist nicht angelaufen. "
                    f"Ausgabe im Container: {ausgabe or '(keine)'}"
                )
            # Läuft noch, ist nur langsam: weitermachen, der Status zeigt es.
            log.warning("Lidar-Einspeisung meldet sich nach %.0f s noch nicht bereit",
                        _BEREIT_WARTEN_S)

        self._thread = threading.Thread(
            target=self._schleife, name="ros-einspeisung", daemon=True
        )
        self._thread.start()
        log.info("Lidar-Einspeisung in ROS gestartet (Container %s)", self._container)

    def stop(self) -> None:
        self._stop.set()
        self._prozess_beenden()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=3.0)
        self._thread = None
        try:
            self._alten_beenden()
        except (OSError, subprocess.SubprocessError):
            # Docker schon weg (etwa beim Herunterfahren): dann ist es der
            # Knoten auch.
            pass

    def status(self) -> dict[str, Any]:
        prozess = self._prozess
        scan = self._lidar.latest() if self._lidar is not None else None
        jetzt = time.monotonic()
        return {
            "laeuft": (
                self._thread is not None
                and self._thread.is_alive()
                and prozess is not None
                and prozess.poll() is None
            ),
            "bereit": self._bereit.is_set(),
            "scans": self._scans,
            "letzter_scan_vor_s": (
                round(jetzt - self._letzter_scan, 1) if self._letzter_scan else None
            ),
            # Getrennt vom Obigen: so sieht man, ob der Lidar selbst nichts
            # liefert oder nur die Weitergabe hakt.
            "lidar_scan_vor_s": round(jetzt - scan.zeit, 1) if scan else None,
            "neustarts": self._neustarts,
            "fehler": self._fehler,
        }

    # --- Prozess im Container --------------------------------------------------

    def _prozess_starten(self) -> None:
        einstellungen = {
            "scan_topic": config.MAP_SCAN_TOPIC,
            "odom_frame": _ODOM_FRAME,
            "basis_frame": _BASIS_FRAME,
            "laser_frame": _LASER_FRAME,
            "range_min": config.LIDAR_MIN_M,
            "range_max": config.LIDAR_MAX_M,
        }
        innen = (
            f"source {shlex.quote(self._setup_file)} && "
            f"export ROSFEED_CONFIG={shlex.quote(json.dumps(einstellungen))} && "
            f"echo $$ > {shlex.quote(_PID_DATEI)} && "
            f"exec python3 -u -c {shlex.quote(_KNOTEN)}"
        )
        self._bereit.clear()
        self._ausgabe.clear()
        try:
            prozess = subprocess.Popen(
                [self._docker_bin, "exec", "-i", self._container, "bash", "-lc", innen],
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
            self._prozess = prozess
        threading.Thread(
            target=self._ausgabe_lesen, args=(prozess,), name="ros-einspeisung-ausgabe",
            daemon=True,
        ).start()

    def _ausgabe_lesen(self, prozess: subprocess.Popen[str]) -> None:
        """Ausgabe des Knotens mitlesen: Bereitmeldung und Fehlertexte."""
        if prozess.stdout is None:
            return
        for zeile in prozess.stdout:
            zeile = zeile.rstrip()
            if zeile == _BEREIT:
                self._bereit.set()
            elif zeile:
                self._ausgabe.append(zeile)

    def _ausgabe_ende(self) -> str:
        return " | ".join(list(self._ausgabe)[-5:])[-1500:]

    def _prozess_laeuft(self) -> bool:
        prozess = self._prozess
        return prozess is not None and prozess.poll() is None

    def _prozess_beenden(self) -> None:
        with self._lock:
            prozess, self._prozess = self._prozess, None
        if prozess is None:
            return
        # stdin schließen genügt eigentlich: der Knoten liest EOF und
        # beendet sich. Nur falls er hängt, härter nachhelfen.
        try:
            if prozess.stdin is not None:
                prozess.stdin.close()
        except OSError:
            pass
        try:
            prozess.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            prozess.terminate()
            try:
                prozess.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                prozess.kill()

    def _alten_beenden(self) -> None:
        """Einen übrig gebliebenen Knoten im Container beenden."""
        pid_datei = shlex.quote(_PID_DATEI)
        befehl = (
            f"if test -f {pid_datei}; then pid=\"$(cat {pid_datei})\"; "
            "if test -r /proc/$pid/cmdline && "
            f"tr \"\\0\" \" \" < /proc/$pid/cmdline | grep -Fq {_KENNUNG}; then "
            "kill -INT \"$pid\" 2>/dev/null || true; sleep 0.3; "
            "kill -KILL \"$pid\" 2>/dev/null || true; fi; "
            f"rm -f {pid_datei}; fi"
        )
        subprocess.run(
            [self._docker_bin, "exec", self._container, "bash", "-lc", befehl],
            capture_output=True, text=True, timeout=6.0, check=False,
        )

    # --- Weitergabe ------------------------------------------------------------

    def _schleife(self) -> None:
        takt = 1.0 / config.MAP_FEED_HZ
        wartezeit = config.RECONNECT_MIN_S
        letzte_nr: int | None = None
        letzte_scan_wand: float | None = None

        while not self._stop.is_set():
            if not self._prozess_laeuft():
                # Knoten ist weg (Container neu gestartet, ROS abgestürzt):
                # neu starten, mit wachsender Pause, damit ein dauerhaft
                # kaputter Container nicht im Sekundentakt angestoßen wird.
                ausgabe = self._ausgabe_ende()
                self._fehler = (
                    "Lidar-Einspeisung im Container beendet"
                    + (f": {ausgabe}" if ausgabe else "")
                )
                log.warning("%s -- neuer Versuch in %.0f s", self._fehler, wartezeit)
                self._prozess_beenden()
                if self._stop.wait(wartezeit):
                    break
                wartezeit = min(wartezeit * 2, config.RECONNECT_MAX_S)
                try:
                    self._alten_beenden()
                    self._prozess_starten()
                    self._neustarts += 1
                except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                    self._fehler = str(exc)
                continue

            try:
                # Laufend die aktuelle Position, damit TF zwischen den
                # Aufnahmen nicht veraltet (RViz, Wartezeiten in SLAM).
                pose = self._scan_odo.pose()
                if pose is not None:
                    self._senden_pose(time.time(), pose)

                messung = self._scan_odo.messung()
                if messung is not None and messung.nr != letzte_nr:
                    letzte_nr = messung.nr
                    scan = messung.scan
                    # Scan.zeit ist monotone Zeit; ROS braucht Uhrzeit.
                    wand = time.time() - (time.monotonic() - messung.zeit)
                    # Die Position genau zum Messzeitpunkt mitschicken:
                    # slam_toolbox fragt sie für den Zeitstempel des Scans ab.
                    self._senden_pose(wand, messung.pose)
                    dauer = 1.0 / 5.5
                    if letzte_scan_wand is not None and 0 < wand - letzte_scan_wand < 0.5:
                        dauer = wand - letzte_scan_wand
                    letzte_scan_wand = wand
                    self._senden({
                        "typ": "scan", "t": wand, "dauer": round(dauer, 4),
                        "r": raster(scan.winkel, scan.entfernungen, config.MAP_SCAN_STRAHLEN),
                    })
                    self._scans += 1
                    self._letzter_scan = time.monotonic()
                    self._fehler = None
                    wartezeit = config.RECONNECT_MIN_S
            except (OSError, ValueError) as exc:
                # Rohr gebrochen oder schon geschlossen: der Knoten ist weg.
                # Die nächste Runde startet ihn neu.
                if not self._stop.is_set():
                    self._fehler = f"Weitergabe an ROS unterbrochen: {exc}"
                    self._prozess_beenden()
                continue

            self._stop.wait(takt)

    def _senden_pose(self, t: float, pose) -> None:
        self._senden({"typ": "odom", "t": t, "x": pose[0], "y": pose[1], "theta": pose[2]})

    def _senden(self, nachricht: dict) -> None:
        prozess = self._prozess
        if prozess is None or prozess.stdin is None:
            raise ValueError("Knoten nicht verbunden")
        prozess.stdin.write(json.dumps(nachricht, separators=(",", ":")) + "\n")
        prozess.stdin.flush()


def raster(winkel: list[float], entfernungen: list[float], strahlen: int) -> list[float]:
    """Punkte auf ein festes Winkelraster von -180° bis knapp +180° legen.

    Unsere Winkel (0° vorn, gegen den Uhrzeigersinn) entsprechen schon der
    ROS-Konvention, nur der Bereich ist 0..360 statt -180..180. Fallen zwei
    Punkte in denselben Strahl, zählt der nähere -- ein Hindernis zu
    übersehen wäre schlimmer als eines etwas zu nah einzutragen.
    """
    werte = [0.0] * strahlen
    schritt = 360.0 / strahlen
    for w, r in zip(winkel, entfernungen):
        i = int(round(((w + 180.0) % 360.0) / schritt)) % strahlen
        if werte[i] == 0.0 or r < werte[i]:
            werte[i] = r
    return [round(r, 3) for r in werte]
