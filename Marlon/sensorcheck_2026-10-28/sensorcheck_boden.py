# Sensor-Check am Boden: Odometrie (vel_raw), Gyro und Lidar gegeneinander pruefen.
# Aufruf im Container:  python3 sensorcheck_boden.py  -> schreibt boden.txt / boden.json
# Braucht ca. 1 m frei vorne und links. Ablauf:
#   1. 4 s Ruhe (Gyro-Bias)
#   2. Drehen links 0,5 rad/s ~3,1 s (~90 Grad), Ruhe, Drehen rechts zurueck
#   3. Vor 0,15 m/s 2 s (~0,3 m), Ruhe, zurueck
#   4. Links 0,15 m/s 2 s (~0,3 m), Ruhe, zurueck
# Lidar: Rohwinkel gegen den Uhrzeigersinn, vorne = -90 Grad (Hand-Test 28.10.).
import json
import math
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32

VORNE = -90.0
LOG = []
DATA = {}


def log(msg):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, flush=True)
    LOG.append(line)


class Probe(Node):
    def __init__(self):
        super().__init__("sensorcheck_boden")
        self.scan = None
        self.imu = []
        self.vel = []
        self.volt = None
        self.create_subscription(LaserScan, "/scan", self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data_raw",
                                 lambda m: self.imu.append((time.time(), m.angular_velocity.z)), 10)
        self.create_subscription(Twist, "/vel_raw", lambda m: self.vel.append(
            (time.time(), m.linear.x, m.linear.y, m.angular.z)), 10)
        self.create_subscription(Float32, "/voltage", self.cb_volt, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def cb_scan(self, m):
        self.scan = (time.time(), m)

    def cb_volt(self, m):
        self.volt = m.data

    def spin_for(self, dur, cmd=None):
        t_end = time.time() + dur
        next_pub = 0.0
        while time.time() < t_end:
            if cmd is not None and time.time() >= next_pub:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = cmd
                self.pub.publish(t)
                next_pub = time.time() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.05)


def profile(m, nbins=360):
    """Kleinste Entfernung je 1-Grad-Bin (Rohwinkel), None = leer."""
    prof = [None] * nbins
    for i, r in enumerate(m.ranges):
        if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
            continue
        a = math.degrees(m.angle_min + i * m.angle_increment)
        b = int(math.floor(a + 180.0)) % nbins
        if prof[b] is None or r < prof[b]:
            prof[b] = r
    return prof


def best_shift(before, after, max_shift=150):
    """Um wie viel Grad ist die Umgebung im Scan gewandert (after[b] = before[b - s])?"""
    n = len(before)
    best = None
    for s in range(-max_shift, max_shift + 1):
        tot, cnt = 0.0, 0
        for b in range(n):
            a, c = after[b], before[(b - s) % n]
            if a is not None and c is not None:
                tot += min(abs(a - c), 0.5)
                cnt += 1
        if cnt > n // 4:
            e = tot / cnt
            if best is None or e < best[1]:
                best = (s, e)
    return best


def dist_at(m, deg, width=4.0):
    """Median-Entfernung um den Rohwinkel deg (+-width)."""
    vals = []
    for i, r in enumerate(m.ranges):
        if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
            continue
        a = math.degrees(m.angle_min + i * m.angle_increment)
        if abs((a - deg + 180.0) % 360.0 - 180.0) <= width:
            vals.append(r)
    return round(st.median(vals), 3) if vals else None


def fresh_scans(node, n=3):
    """n neue Scans im Stillstand sammeln, Profil = Median je Bin."""
    scans = []
    last_t = node.scan[0] if node.scan else 0.0
    t_end = time.time() + 5.0
    while len(scans) < n:
        if time.time() > t_end:
            log("FEHLER: keine neuen Scans mehr (USB-Reset?)")
            raise SystemExit
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.scan and node.scan[0] != last_t:
            last_t = node.scan[0]
            scans.append(node.scan[1])
    profs = [profile(m) for m in scans]
    med = []
    for b in range(360):
        v = [p[b] for p in profs if p[b] is not None]
        med.append(st.median(v) if v else None)
    return med, scans[-1]


def integrate(samples, t0, t1, idx):
    """Rechteck-Integration der Werte samples[k][idx] zwischen t0 und t1."""
    s = [x for x in samples if t0 <= x[0] <= t1]
    tot = 0.0
    for a, b in zip(s, s[1:]):
        tot += a[idx] * (b[0] - a[0])
    return tot


rclpy.init()
node = Probe()
try:
    node.spin_for(2.0)
    if node.scan is None or not node.vel or not node.imu:
        log("FEHLER: /scan, /vel_raw oder /imu/data_raw fehlt - Treiber/Lidar laufen?")
        raise SystemExit
    log(f"Akku {node.volt}")

    # --- 1. Gyro-Bias im Stillstand ---
    t0 = time.time()
    node.spin_for(4.0)
    gz = [z for t, z in node.imu if t >= t0]
    bias = st.mean(gz)
    DATA["gyro_bias_z"] = round(bias, 4)
    DATA["gyro_std_z"] = round(st.pstdev(gz), 4)
    log(f"Gyro-Bias z {bias:.4f} rad/s (std {st.pstdev(gz):.4f}, n={len(gz)})")

    # --- 2. Drehen ---
    DATA["drehen"] = {}
    for name, wz in (("links", 0.5), ("rechts", -0.5)):
        p0, _ = fresh_scans(node)
        t0 = time.time()
        node.spin_for(math.pi / 2 / abs(wz), (0.0, 0.0, wz))
        node.stop()
        node.spin_for(1.5)
        t1 = time.time()
        p1, _ = fresh_scans(node)
        odo = integrate(node.vel, t0, t1, 3)
        gyr = integrate([(t, z - bias) for t, z in node.imu], t0, t1, 1)
        gyr_raw = integrate(node.imu, t0, t1, 1)
        sh = best_shift(p0, p1)
        r = {"soll_deg": round(math.degrees(math.copysign(math.pi / 2, wz)), 1),
             "odom_deg": round(math.degrees(odo), 1),
             "gyro_deg": round(math.degrees(gyr), 1),
             "gyro_ohne_bias_korr_deg": round(math.degrees(gyr_raw), 1),
             "lidar_verschiebung_deg": sh and sh[0], "lidar_fehler_m": sh and round(sh[1], 3)}
        DATA["drehen"][name] = r
        log(f"Drehen {name}: {r}")

    # --- 3./4. Vor/zurueck, links/rechts ---
    DATA["fahren"] = {}
    for name, cmd, deg in (("vor", (0.15, 0.0, 0.0), VORNE),
                           ("zurueck", (-0.15, 0.0, 0.0), VORNE),
                           ("links", (0.0, 0.15, 0.0), VORNE + 90.0),
                           ("rechts", (0.0, -0.15, 0.0), VORNE + 90.0)):
        _, m0 = fresh_scans(node)
        d0 = dist_at(m0, deg)
        if name in ("vor", "links") and (d0 is None or d0 < 0.6):
            log(f"{name}: nur {d0} m frei bei Rohwinkel {deg} - uebersprungen")
            DATA["fahren"][name] = {"uebersprungen": d0}
            continue
        t0 = time.time()
        node.spin_for(2.0, cmd)
        node.stop()
        node.spin_for(1.5)
        t1 = time.time()
        _, m1 = fresh_scans(node)
        d1 = dist_at(m1, deg)
        idx = 1 if cmd[0] else 2
        odo = integrate(node.vel, t0, t1, idx)
        rot = integrate([(t, z - bias) for t, z in node.imu], t0, t1, 1)
        r = {"soll_m": round(max(cmd, key=abs) * 2.0, 3), "odom_m": round(odo, 3),
             "lidar_vorher_m": d0, "lidar_nachher_m": d1,
             "lidar_delta_m": d0 is not None and d1 is not None and round(d0 - d1, 3),
             "drift_gyro_deg": round(math.degrees(rot), 1)}
        DATA["fahren"][name] = r
        log(f"Fahren {name} {cmd}: {r}")
    log(f"fertig, Akku {node.volt}")
except (KeyboardInterrupt, SystemExit):
    log("abgebrochen")
finally:
    node.stop()
    with open("boden.json", "w") as f:
        json.dump(DATA, f, indent=1)
    with open("boden.txt", "w") as f:
        f.write("\n".join(LOG) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)
