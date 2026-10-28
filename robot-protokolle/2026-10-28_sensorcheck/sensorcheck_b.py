# Sensor-Check Phase B: ueber ROS2 (im Container x3, Treiber + Lidar laufen).
# Aufruf im Container:  python3 sensorcheck_b.py  -> schreibt phase_b.txt / phase_b.json
# Raeder in der Luft! Am Ende kurzer cmd_vel-Test (0,2 m/s, 1 rad/s).
import json
import math
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, MagneticField
from std_msgs.msg import Float32, Int32

LOG = []
DATA = {}


def log(msg):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, flush=True)
    LOG.append(line)


def stats(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    return {"mittel": round(st.mean(vals), 4), "std": round(st.pstdev(vals), 4),
            "min": round(min(vals), 4), "max": round(max(vals), 4), "n": len(vals)}


class Probe(Node):
    def __init__(self):
        super().__init__("sensorcheck_b")
        self.msgs = {k: [] for k in ("scan", "imu", "mag", "vel", "volt", "ed", "js")}
        sd = qos_profile_sensor_data
        self.create_subscription(LaserScan, "/scan", lambda m: self.rx("scan", m), sd)
        self.create_subscription(Imu, "/imu/data_raw", lambda m: self.rx("imu", m), 10)
        self.create_subscription(MagneticField, "/imu/mag", lambda m: self.rx("mag", m), 10)
        self.create_subscription(Twist, "/vel_raw", lambda m: self.rx("vel", m), 10)
        self.create_subscription(Float32, "/voltage", lambda m: self.rx("volt", m), 10)
        self.create_subscription(Float32, "/edition", lambda m: self.rx("ed", m), 10)
        self.create_subscription(JointState, "/joint_states", lambda m: self.rx("js", m), 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def rx(self, key, msg):
        self.msgs[key].append((time.time(), msg))

    def clear(self):
        for v in self.msgs.values():
            v.clear()

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


def rate(lst):
    if len(lst) < 2:
        return None
    dts = [b[0] - a[0] for a, b in zip(lst, lst[1:])]
    return {"hz": round((len(lst) - 1) / (lst[-1][0] - lst[0][0]), 2),
            "max_luecke_s": round(max(dts), 3), "n": len(lst)}


def scan_summary(lst):
    if not lst:
        return None
    m0 = lst[-1][1]
    out = {"frame_id": m0.header.frame_id,
           "angle_min_deg": round(math.degrees(m0.angle_min), 2),
           "angle_max_deg": round(math.degrees(m0.angle_max), 2),
           "inkrement_deg": round(math.degrees(m0.angle_increment), 4),
           "range_min": round(m0.range_min, 3), "range_max": round(m0.range_max, 3),
           "scan_time": round(m0.scan_time, 4), "time_increment": m0.time_increment,
           "intensitaeten": len(m0.intensities) > 0}
    npts, valid = [], []
    for _, m in lst:
        npts.append(len(m.ranges))
        ok = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
        valid.append(len(ok) / max(len(m.ranges), 1))
    out["punkte"] = stats(npts)
    out["anteil_gueltig"] = stats(valid)
    # Entfernung je 30-Grad-Sektor (Rohwinkel des Lidars), letzter Scan
    sek = {}
    for i, r in enumerate(m0.ranges):
        if not (math.isfinite(r) and m0.range_min <= r <= m0.range_max):
            continue
        a = math.degrees(m0.angle_min + i * m0.angle_increment)
        a = (a + 180.0) % 360.0 - 180.0
        k = int((a + 180.0) // 30) * 30 - 180
        sek.setdefault(k, []).append(r)
    out["sektoren_min_m"] = {f"{k}..{k + 30}": round(min(v), 3) for k, v in sorted(sek.items())}
    allr = [r for r in m0.ranges if math.isfinite(r) and m0.range_min <= r <= m0.range_max]
    out["naechster_m"] = round(min(allr), 3) if allr else None
    out["weitester_m"] = round(max(allr), 3) if allr else None
    # Winkel des naechsten Punkts (Rohwinkel), hilft bei der Ausrichtung
    if allr:
        i = min(range(len(m0.ranges)), key=lambda j: m0.ranges[j]
                if math.isfinite(m0.ranges[j]) and m0.ranges[j] >= m0.range_min else 1e9)
        out["naechster_rohwinkel_deg"] = round(
            math.degrees(m0.angle_min + i * m0.angle_increment), 1)
    return out


def imu_summary(lst):
    if not lst:
        return None
    m0 = lst[-1][1]
    g = lambda f: [f(m) for _, m in lst]
    return {"frame_id": m0.header.frame_id,
            "acc": [stats(g(lambda m: m.linear_acceleration.x)),
                    stats(g(lambda m: m.linear_acceleration.y)),
                    stats(g(lambda m: m.linear_acceleration.z))],
            "gyro": [stats(g(lambda m: m.angular_velocity.x)),
                     stats(g(lambda m: m.angular_velocity.y)),
                     stats(g(lambda m: m.angular_velocity.z))],
            "orientierung_w": stats(g(lambda m: m.orientation.w)),
            "kovarianz_acc0": m0.linear_acceleration_covariance[0]}


def vel_summary(lst):
    if not lst:
        return None
    g = lambda f: [f(m) for _, m in lst]
    return [stats(g(lambda m: m.linear.x)), stats(g(lambda m: m.linear.y)),
            stats(g(lambda m: m.angular.z))]


rclpy.init()
node = Probe()
try:
    log("Ruhe 20 s, alle Topics mitschreiben ...")
    node.spin_for(20.0)
    M = node.msgs
    DATA["raten"] = {k: rate(v) for k, v in M.items()}
    log(f"  Raten {DATA['raten']}")
    DATA["scan"] = scan_summary(M["scan"])
    log(f"  scan {DATA['scan']}")
    DATA["imu"] = imu_summary(M["imu"])
    log(f"  imu {DATA['imu']}")
    if M["mag"]:
        mm = [m.magnetic_field for _, m in M["mag"]]
        DATA["mag"] = [stats([v.x for v in mm]), stats([v.y for v in mm]), stats([v.z for v in mm])]
        log(f"  mag {DATA['mag']}")
    DATA["volt"] = stats([m.data for _, m in M["volt"]])
    DATA["edition"] = M["ed"][-1][1].data if M["ed"] else None
    log(f"  Akku {DATA['volt']}  Edition {DATA['edition']}")
    if M["js"]:
        js = M["js"][-1][1]
        DATA["joint_states"] = {"namen": list(js.name), "pos": [round(p, 3) for p in js.position]}
        log(f"  joint_states {DATA['joint_states']}")
    DATA["vel_ruhe"] = vel_summary(M["vel"])

    # cmd_vel-Kette: Latenz bis vel_raw reagiert + erreichte Geschwindigkeit
    DATA["cmd"] = {}
    for name, cmd in (("vor", (0.2, 0.0, 0.0)), ("links", (0.0, 0.2, 0.0)),
                      ("drehen_links", (0.0, 0.0, 1.0))):
        node.clear()
        t0 = time.time()
        node.spin_for(3.0, cmd)
        node.spin_for(1.5, (0.0, 0.0, 0.0))
        node.pub.publish(Twist())
        vel = node.msgs["vel"]
        idx = {"vor": lambda m: m.linear.x, "links": lambda m: m.linear.y,
               "drehen_links": lambda m: m.angular.z}[name]
        soll = max(cmd, key=abs)
        t_start = next((t for t, m in vel if abs(idx(m)) > 0.2 * abs(soll)), None)
        t_90 = next((t for t, m in vel if abs(idx(m)) > 0.9 * abs(soll)), None)
        stat = [idx(m) for t, m in vel if t0 + 1.5 < t < t0 + 3.0]
        t_stop = next((t for t, m in vel if t > t0 + 3.0 and abs(idx(m)) < 0.1 * abs(soll)), None)
        imu_z = [m.angular_velocity.z for t, m in node.msgs["imu"] if t0 + 1.5 < t < t0 + 3.0]
        r = {"soll": soll,
             "latenz_20%_s": t_start and round(t_start - t0, 3),
             "anstieg_90%_s": t_90 and round(t_90 - t0, 3),
             "ist_stationaer": stats(stat),
             "auslauf_s": t_stop and round(t_stop - t0 - 3.0, 3),
             "imu_gyro_z": stats(imu_z)}
        DATA["cmd"][name] = r
        log(f"cmd {name} {cmd}: {r}")
        node.spin_for(1.0)
    log("fertig")
except KeyboardInterrupt:
    log("abgebrochen")
finally:
    for _ in range(5):
        node.pub.publish(Twist())
        time.sleep(0.05)
    with open("phase_b.json", "w") as f:
        json.dump(DATA, f, indent=1, default=str)
    with open("phase_b.txt", "w") as f:
        f.write("\n".join(LOG) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)
