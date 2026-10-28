# Lidar-Ausrichtung mit Referenz-Scan: nur Punkte, die NEU naeher sind als im
# Referenz-Scan (Hand), werden gemeldet. Ablauf (55 s):
#   0-8 s   NICHTS in der Naehe (Referenz)
#   8-28 s  Hand VORNE (~20 cm)
#   28-35 s Hand weg
#   35-55 s Hand LINKS (~20 cm)
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

out = open("hand.txt", "w")


def say(line):
    print(line, flush=True)
    out.write(line + "\n")


class P(Node):
    def __init__(self):
        super().__init__("sensorcheck_hand")
        self.last = None
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.last = m


def valid(m, r):
    return math.isfinite(r) and m.range_min <= r <= m.range_max


rclpy.init()
n = P()
t0 = time.time()
ref = None                       # pro Strahl die kleinste Entfernung waehrend 0-8 s
next_t = t0 + 1.0
while time.time() - t0 < 55:
    rclpy.spin_once(n, timeout_sec=0.05)
    m = n.last
    if m is None:
        continue
    t = time.time() - t0
    if t < 8:
        cur = [r if valid(m, r) else 99.0 for r in m.ranges]
        ref = cur if ref is None else [min(a, b) for a, b in zip(ref, cur)]
    if time.time() >= next_t and ref:
        next_t += 1.0
        phase = ("REFERENZ" if t < 8 else "VORNE" if t < 28 else "weg"
                 if t < 35 else "LINKS")
        neu = [(m.ranges[i], i) for i in range(len(m.ranges))
               if valid(m, m.ranges[i]) and m.ranges[i] < 0.6
               and m.ranges[i] < ref[i] - 0.10]
        if len(neu) >= 3:
            angs = sorted(math.degrees(m.angle_min + i * m.angle_increment) for _, i in neu)
            mid = angs[len(angs) // 2]
            say(f"t={t:5.1f}s [{phase:8s}] neu: {len(neu):3d} Punkte, Mitte "
                f"Rohwinkel {mid:7.1f} Grad, naechster {min(neu)[0]:.3f} m")
        else:
            say(f"t={t:5.1f}s [{phase:8s}] nichts Neues")
out.close()
rclpy.shutdown()
