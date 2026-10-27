#!/usr/bin/env python3
"""
Autonomous explorer for the Yahboom ROSMASTER X3 (mecanum wheels + RPLIDAR).

Reads /scan, drives via /cmd_vel. Works with ROS1 (rospy) and ROS2 (rclpy) -
the version is picked from $ROS_VERSION.

Behaviour ("looks smart"):
  * looks around once at startup, then heads for the most open direction
  * evaluates 72 driving directions per tick, each checked as a full robot-wide
    corridor (not just a single beam), and picks the best trade-off of
    free space / wander goal / smoothness
  * uses the mecanum wheels to slide sideways around obstacles while turning
    the body into the direction of travel
  * keeps distance from walls, slows down smoothly near obstacles
  * dead end -> stop, beep, back up, turn towards the widest gap
  * pauses every now and then to "look around" (pan left/right)

Usage:
  python3 x3_explorer.py                 # drive
  python3 x3_explorer.py --calibrate     # only print where the lidar sees things
  python3 x3_explorer.py --help          # all parameters
"""

import argparse
import math
import os
import random
import signal
import sys
import time

TAU = 2.0 * math.pi

# Yahboom driver: /RGBLight (Int32) -> Rosmaster.set_colorful_effect(effect)
LED_OFF, LED_FLOW, LED_MARQUEE, LED_BREATH, LED_GRADIENT, LED_STAR = 0, 1, 2, 3, 4, 5


def wrap(a):
    """Wrap angle to [-pi, pi)."""
    return (a + math.pi) % TAU - math.pi


def angdiff(a, b):
    return abs(wrap(a - b))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def approach(cur, target, max_step):
    if target > cur:
        return min(target, cur + max_step)
    return max(target, cur - max_step)


# --------------------------------------------------------------------------- #
#  Pure logic (no ROS) - testable on any PC, see test_explorer_sim.py
# --------------------------------------------------------------------------- #

def scan_to_points(angle_min, angle_inc, ranges, range_min, range_max,
                   front_offset, ignore_below, flip=False, bin_deg=2.0):
    """LaserScan -> list of (x, y) obstacle points in the robot frame
    (x forward, y left). Keeps only the closest return per angular bin."""
    nbins = int(round(360.0 / bin_deg))
    best = [None] * nbins
    lo = max(range_min, ignore_below)
    for i, r in enumerate(ranges):
        if not (lo <= r <= range_max):  # also rejects nan/inf
            continue
        a = angle_min + i * angle_inc
        if flip:
            a = -a
        a = wrap(a - front_offset)
        b = int((a + math.pi) / TAU * nbins) % nbins
        if best[b] is None or r < best[b][0]:
            best[b] = (r, a)
    return [(r * math.cos(a), r * math.sin(a)) for r, a in filter(None, best)]


def corridor_free(points, theta, radius, cap):
    """How far a disc of `radius` can travel in direction `theta` before
    touching any point. Negative = already touching."""
    c, s = math.cos(theta), math.sin(theta)
    r2 = radius * radius
    free = cap
    for x, y in points:
        along = x * c + y * s
        if along <= 0.0:
            continue
        lat = y * c - x * s
        if -radius < lat < radius:
            d = along - math.sqrt(r2 - lat * lat)
            if d < free:
                free = d
    return free


class Cmd:
    __slots__ = ("vx", "vy", "wz")

    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx, self.vy, self.wz = vx, vy, wz


class Explorer:
    STARTUP, EXPLORE, LOOK, RECOVER = "UMSCHAUEN", "ERKUNDEN", "SCHAUEN", "AUSWEG"

    def __init__(self, p, rng=None):
        self.p = p
        self.rng = rng or random.Random()
        n = p.directions
        self.cands = [wrap(-math.pi + i * TAU / n) for i in range(n)]
        self.yaw = 0.0                 # dead-reckoned pose (world frame)
        self.px = self.py = 0.0
        self.visits = {}               # coarse "where have I been" memory
        self.visit_t = 0.0
        self.cmd = Cmd()               # currently commanded (smoothed) velocity
        self.state = None
        self.state_t = 0.0
        self.phase = 0
        self.phase_t = 0.0
        self.target_world = 0.0
        self.prev_world = 0.0
        self.wander_world = 0.0
        self.next_wander_t = 0.0
        self.next_look_t = 0.0
        self.blocked_since = None
        self.progress = []             # (t, distance) for stuck detection
        self.events = []               # ("led", effect) / ("beep", seconds)
        self.status = ""
        self.chosen_rel = 0.0
        self.chosen_free = 0.0
        self.free = [0.0] * n
        self.t = None

    # ---- helpers -------------------------------------------------------- #
    def _enter(self, state, now):
        self.state, self.state_t = state, now
        self.phase, self.phase_t = 0, now
        led = {self.STARTUP: LED_GRADIENT, self.EXPLORE: LED_FLOW,
               self.LOOK: LED_STAR, self.RECOVER: LED_BREATH}[state]
        self.events.append(("led", led))

    def _next_phase(self, now):
        self.phase += 1
        self.phase_t = now

    def _rel(self, world):
        return wrap(world - self.yaw)

    def _evaluate(self, pts):
        p = self.p
        self.free = [corridor_free(pts, th, p.radius + p.margin, p.look_ahead)
                     for th in self.cands]

    def _most_open_world(self, front_weight=0.15):
        n = len(self.cands)
        best, best_s = 0, -1e9
        for i, th in enumerate(self.cands):
            # average with neighbours -> prefer the middle of wide gaps
            f = (self.free[i - 1] + 2 * self.free[i] + self.free[(i + 1) % n]) / 4
            s = f / self.p.look_ahead - front_weight * abs(th) / math.pi
            if s > best_s:
                best, best_s = i, s
        return wrap(self.yaw + self.cands[best])

    def _cell(self, x, y):
        c = self.p.memory_cell
        return (int(math.floor(x / c)), int(math.floor(y / c)))

    def _remember(self, now):
        if now - self.visit_t < 0.5:
            return
        self.visit_t = now
        k = self._cell(self.px, self.py)
        self.visits[k] = self.visits.get(k, 0.0) + 1.0
        if len(self.visits) > 4000:     # forget the oldest-looking (weakest)
            for key in sorted(self.visits, key=self.visits.get)[:1000]:
                del self.visits[key]

    def _familiarity(self, th, free):
        """How often we've been where direction th (robot frame) leads."""
        a = self.yaw + th
        d = clamp(free, 0.3, 1.5)
        x, y = self.px + d * math.cos(a), self.py + d * math.sin(a)
        cx, cy = self._cell(x, y)
        tot = 0.0
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                tot += self.visits.get((cx + dx, cy + dy), 0.0)
        return min(tot / 30.0, 1.0)

    def _min_dist(self, pts):
        return min((math.hypot(x, y) for x, y in pts), default=99.0)

    # ---- main entry ----------------------------------------------------- #
    def update(self, pts, now):
        """pts: obstacle points in robot frame; returns Cmd to publish."""
        p = self.p
        if self.t is None:
            self.t = now
            self.next_look_t = now + self.rng.uniform(*p.look_every)
            self._enter(self.STARTUP if p.startup_spin else self.EXPLORE, now)
            self.events.append(("beep", 0.08))
        dt = clamp(now - self.t, 0.0, 0.2)
        self.t = now

        self._evaluate(pts)
        nearest = self._min_dist(pts)

        if self.state == self.STARTUP:
            target = self._startup(now)
        elif self.state == self.LOOK:
            target = self._look(now)
        elif self.state == self.RECOVER:
            target = self._recover(pts, now)
        else:
            target = self._explore(pts, now, nearest)

        # --- acceleration limits -> smooth, "intentional" motion ----------
        c = self.cmd
        c.vx = approach(c.vx, target.vx, p.acc_lin * dt)
        c.vy = approach(c.vy, target.vy, p.acc_lin * dt)
        c.wz = approach(c.wz, target.wz, p.acc_ang * dt)

        # --- final safety check on what we actually send -----------------
        speed = math.hypot(c.vx, c.vy)
        if speed > 0.02:
            d = corridor_free(pts, math.atan2(c.vy, c.vx), p.radius, 2.0)
            brake = speed * speed / (2 * p.acc_lin) + p.stop_margin
            if d < brake:
                # cannot stop in time with normal decel -> hard stop
                scale = clamp(d / brake, 0.0, 1.0) if d > 0 else 0.0
                c.vx *= scale
                c.vy *= scale

        self.px += (c.vx * math.cos(self.yaw) - c.vy * math.sin(self.yaw)) * dt
        self.py += (c.vx * math.sin(self.yaw) + c.vy * math.cos(self.yaw)) * dt
        self.yaw = wrap(self.yaw + c.wz * dt)
        self._remember(now)
        self.progress.append((now, math.hypot(c.vx, c.vy) * dt))
        while self.progress and self.progress[0][0] < now - p.stuck_time:
            self.progress.pop(0)

        self.status = (f"{self.state:<9} v={math.hypot(c.vx, c.vy):.2f}m/s "
                       f"w={c.wz:+.2f}rad/s Ziel={math.degrees(self.chosen_rel):+5.0f}° "
                       f"frei={self.chosen_free:.2f}m naechstes={nearest:.2f}m")
        return Cmd(c.vx, c.vy, c.wz)

    # ---- states ---------------------------------------------------------- #
    def _startup(self, now):
        """Full slow turn on the spot, then head for the most open direction."""
        p = self.p
        spin_time = TAU / p.look_speed
        if now - self.state_t < spin_time and min(self.free) > -0.02:
            return Cmd(0.0, 0.0, p.look_speed)
        self.wander_world = self.prev_world = self._most_open_world(0.0)
        self.next_wander_t = now + self.rng.uniform(*p.wander_every)
        self._enter(self.EXPLORE, now)
        return Cmd()

    def _look(self, now):
        """Theatre: stop, pan left, pan right, centre, then pick best direction."""
        p = self.p
        pan = math.radians(p.look_pan_deg)
        seq = [(0.0, 0.5), (+1, pan), (-1, 2 * pan), (+1, pan)]
        if self.phase >= len(seq):
            self.wander_world = self.prev_world = self._most_open_world(0.4)
            self.events.append(("beep", 0.05))
            self._enter(self.EXPLORE, now)
            return Cmd()
        sign, amount = seq[self.phase]
        if sign == 0.0:
            if now - self.phase_t >= amount:
                self._next_phase(now)
            return Cmd()
        if now - self.phase_t >= amount / p.look_speed:
            self._next_phase(now)
        return Cmd(0.0, 0.0, sign * p.look_speed)

    def _recover(self, pts, now):
        """Dead end: pause, back up if possible, rotate to the widest gap."""
        p = self.p
        el = now - self.phase_t
        if self.phase == 0:                      # short pause + beep
            if el > 0.4:
                self._next_phase(now)
            return Cmd()
        if self.phase == 1:                      # back up
            rear = corridor_free(pts, math.pi, p.radius + 0.02, 1.0)
            if el < p.backup_time and rear > 0.12:
                return Cmd(-p.backup_speed, 0.0, 0.0)
            self.target_world = self._most_open_world(0.05)
            self._next_phase(now)
            return Cmd()
        if self.phase == 2:                      # turn towards the gap
            err = self._rel(self.target_world)
            if abs(err) < math.radians(15) or el > 8.0:
                self.wander_world = self.prev_world = self.target_world
                self.next_wander_t = now + self.rng.uniform(*p.wander_every)
                self.blocked_since = None
                self.progress.clear()
                self._enter(self.EXPLORE, now)
                return Cmd()
            w = clamp(2.0 * err, -p.max_w, p.max_w)
            if abs(w) < 0.4:
                w = math.copysign(0.4, err)
            return Cmd(0.0, 0.0, w)
        return Cmd()

    def _explore(self, pts, now, nearest):
        p = self.p
        n = len(self.cands)

        if now >= self.next_wander_t:
            self.wander_world = wrap(self.wander_world +
                                     math.radians(self.rng.uniform(-100, 100)))
            self.next_wander_t = now + self.rng.uniform(*p.wander_every)

        if (p.look_around and now >= self.next_look_t
                and nearest > 0.6 and now - self.state_t > 5.0):
            self.next_look_t = now + self.rng.uniform(*p.look_every)
            self._enter(self.LOOK, now)
            return Cmd()

        wander_rel = self._rel(self.wander_world)
        prev_rel = self._rel(self.prev_world)
        best, best_s = None, -1e9
        for i, th in enumerate(self.cands):
            f = min(self.free[i - 1], self.free[i], self.free[(i + 1) % n])
            if f < p.min_free:
                continue
            fa = (self.free[i - 1] + 2 * self.free[i] + self.free[(i + 1) % n]) / 4
            s = (p.w_free * fa / p.look_ahead
                 - p.w_goal * angdiff(th, wander_rel) / math.pi
                 - p.w_smooth * angdiff(th, prev_rel) / math.pi
                 - p.w_front * abs(th) / math.pi
                 - p.w_novel * self._familiarity(th, self.free[i]))
            if s > best_s:
                best, best_s = i, s

        stuck = (now - self.state_t > p.stuck_time and
                 sum(d for _, d in self.progress) < p.stuck_dist)
        if best is None or stuck:
            if self.blocked_since is None:
                self.blocked_since = now
            if stuck or now - self.blocked_since > 0.3:
                self.events.append(("beep", 0.15))
                self._enter(self.RECOVER, now)
            return Cmd()
        self.blocked_since = None

        th = self.cands[best]
        free = self.free[best]
        self.chosen_rel, self.chosen_free = th, free
        self.prev_world = wrap(self.yaw + th)

        v = p.max_v * clamp((free - p.stop_dist) / (p.slow_dist - p.stop_dist), 0.0, 1.0)
        if abs(th) > math.radians(75):
            v *= 0.2                                 # mostly turn first
        else:
            v *= max(0.35, math.cos(th))
        vx, vy = v * math.cos(th), v * math.sin(th)

        # gentle push away from side walls (mecanum strafe)
        push = 0.0
        for x, y in pts:
            d = math.hypot(x, y)
            if d < p.wall_dist and abs(y) > abs(x) * 0.5:
                push -= math.copysign((p.wall_dist - d) / p.wall_dist, y)
        vy += clamp(push * p.wall_gain, -p.max_vy, p.max_vy)
        vy = clamp(vy, -p.max_vy, p.max_vy)
        if vx < 0.0:
            vx = 0.0                                 # explore forwards only

        wz = clamp(p.k_turn * th, -p.max_w, p.max_w)
        return Cmd(vx, vy, wz)


def make_parser():
    a = argparse.ArgumentParser(description="ROSMASTER X3 autonomous explorer")
    g = a.add_argument
    g("--calibrate", action="store_true",
      help="nicht fahren, nur anzeigen wo der Lidar das naechste Objekt sieht")
    # unser X3 (pi-bravo) kalibriert am 27.10.: --flip --front-offset 90
    g("--front-offset", type=float, default=90.0,
      help="Lidar-Winkel (Grad), der nach vorne zeigt (kalibriert: 90)")
    g("--flip", dest="flip", action="store_true", default=True,
      help="Lidar-Drehrichtung spiegeln (Standard, kalibriert)")
    g("--no-flip", dest="flip", action="store_false",
      help="Lidar-Drehrichtung nicht spiegeln")
    g("--ignore-below", type=float, default=0.12,
      help="Messungen naeher als dies (m) ignorieren (eigenes Chassis)")
    g("--max-v", type=float, default=0.30, help="max. Geschwindigkeit m/s")
    g("--max-vy", type=float, default=0.20, help="max. Seitwaerts m/s")
    g("--max-w", type=float, default=1.2, help="max. Drehrate rad/s")
    g("--radius", type=float, default=0.20, help="Roboter-Radius m")
    g("--margin", type=float, default=0.06, help="Sicherheitsabstand m")
    g("--stop-dist", type=float, default=0.12)
    g("--slow-dist", type=float, default=0.90)
    g("--min-free", type=float, default=0.25)
    g("--stop-margin", type=float, default=0.04)
    g("--look-ahead", type=float, default=2.5)
    g("--wall-dist", type=float, default=0.40)
    g("--wall-gain", type=float, default=0.06)
    g("--acc-lin", type=float, default=0.6, help="m/s^2")
    g("--acc-ang", type=float, default=2.5, help="rad/s^2")
    g("--k-turn", type=float, default=1.6)
    g("--look-speed", type=float, default=0.9, help="Drehrate beim Umschauen")
    g("--look-pan-deg", type=float, default=60.0)
    g("--backup-speed", type=float, default=0.12)
    g("--backup-time", type=float, default=1.5)
    g("--stuck-time", type=float, default=8.0)
    g("--stuck-dist", type=float, default=0.20)
    g("--no-startup-spin", dest="startup_spin", action="store_false")
    g("--no-look-around", dest="look_around", action="store_false")
    g("--no-sound", dest="sound", action="store_false")
    g("--no-led", dest="led", action="store_false")
    g("--rate", type=float, default=20.0, help="Regelfrequenz Hz")
    g("--scan-topic", default="/scan")
    g("--cmd-topic", default="/cmd_vel")
    g("--quiet", action="store_true")
    a.set_defaults(directions=72, w_free=1.0, w_goal=0.35, w_smooth=0.45,
                   w_front=0.45, w_novel=0.5, memory_cell=0.4, look_every=(40.0, 80.0), wander_every=(8.0, 18.0))
    return a


# --------------------------------------------------------------------------- #
#  ROS glue
# --------------------------------------------------------------------------- #

class Runner:
    """Version independent part: scan in, cmd out, LEDs, buzzer, console."""

    def __init__(self, p, io):
        self.p, self.io = p, io
        self.explorer = Explorer(p)
        self.scan = None
        self.scan_t = 0.0
        self.alive_t = 0.0
        self.beep_off_t = None
        self.last_print = 0.0
        self.warned = False

    def on_scan(self, msg):
        now = time.monotonic()
        # Lebenszeichen fuer den Lidar-Waechter in start_autonomy.sh (max. 1x/s)
        if now - self.alive_t > 1.0:
            self.alive_t = now
            try:
                with open("/tmp/x3_scan_ok", "w"):
                    pass
            except OSError:
                pass
        self.scan = msg
        self.scan_t = now

    def points(self):
        s, p = self.scan, self.p
        return scan_to_points(s.angle_min, s.angle_increment, s.ranges,
                              s.range_min, s.range_max,
                              math.radians(p.front_offset), p.ignore_below, p.flip)

    def tick(self):
        now = time.monotonic()
        if self.beep_off_t is not None and now >= self.beep_off_t:
            self.io.buzzer(False)
            self.beep_off_t = None

        if self.scan is None or now - self.scan_t > 0.6:
            self.io.cmd(0.0, 0.0, 0.0)
            if now - self.last_print > 2.0:
                print(f"[x3] warte auf Lidar-Daten auf {self.p.scan_topic} ...", flush=True)
                self.last_print = now
            return

        pts = self.points()
        if self.p.calibrate:
            self.calibrate(pts, now)
            return

        c = self.explorer.update(pts, now)
        self.io.cmd(c.vx, c.vy, c.wz)
        for kind, val in self.explorer.events:
            if kind == "led" and self.p.led:
                self.io.led(val)
            elif kind == "beep" and self.p.sound:
                self.io.buzzer(True)
                self.beep_off_t = now + val
        self.explorer.events.clear()
        self.io.marker(self.explorer.chosen_rel, self.explorer.chosen_free,
                       self.scan.header.frame_id)
        if not self.p.quiet and now - self.last_print > 0.5:
            print("[x3] " + self.explorer.status, flush=True)
            self.last_print = now

    def calibrate(self, pts, now):
        if now - self.last_print < 0.3:
            return
        self.last_print = now
        if not pts:
            print("[kalibrieren] keine gueltigen Punkte", flush=True)
            return
        x, y = min(pts, key=lambda q: math.hypot(*q))
        a = math.degrees(math.atan2(y, x))
        where = ("VORNE" if abs(a) < 30 else "HINTEN" if abs(a) > 150 else
                 "LINKS" if a > 0 else "RECHTS")
        print(f"[kalibrieren] naechstes Objekt: {math.hypot(x, y):.2f} m bei "
              f"{a:+6.1f}°  -> {where}", flush=True)

    def stop(self):
        for _ in range(3):
            try:
                self.io.cmd(0.0, 0.0, 0.0)
                self.io.buzzer(False)
            except Exception:
                pass
            time.sleep(0.05)


def run_ros1(p):
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Int32
    from visualization_msgs.msg import Marker

    rospy.init_node("x3_explorer")

    class IO:
        cmd_pub = rospy.Publisher(p.cmd_topic, Twist, queue_size=1)
        led_pub = rospy.Publisher("/RGBLight", Int32, queue_size=1)
        buz_pub = rospy.Publisher("/Buzzer", Bool, queue_size=1)
        mk_pub = rospy.Publisher("/x3_explorer/direction", Marker, queue_size=1)

        def cmd(self, vx, vy, wz):
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            self.cmd_pub.publish(t)

        def led(self, e):
            self.led_pub.publish(Int32(data=e))

        def buzzer(self, on):
            self.buz_pub.publish(Bool(data=on))

        def marker(self, rel, free, frame):
            self.mk_pub.publish(_marker(Marker(), rel, free, frame, p,
                                        stamp=rospy.Time.now()))

    r = Runner(p, IO())
    rospy.Subscriber(p.scan_topic, LaserScan, r.on_scan, queue_size=1)
    rospy.on_shutdown(r.stop)
    rate = rospy.Rate(p.rate)
    while not rospy.is_shutdown():
        r.tick()
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def run_ros2(p):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Int32
    from visualization_msgs.msg import Marker

    # Als Hintergrundjob von start_autonomy.sh ist SIGINT ignoriert, und SIGTERM
    # wuerde Python ohne "finally" beenden -> Roboter faehrt weiter. Beide
    # Signale daher als KeyboardInterrupt behandeln, damit r.stop() laeuft.
    def _interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGTERM, _interrupt)
    node = rclpy.create_node("x3_explorer")

    class IO:
        cmd_pub = node.create_publisher(Twist, p.cmd_topic, 1)
        led_pub = node.create_publisher(Int32, "/RGBLight", 1)
        buz_pub = node.create_publisher(Bool, "/Buzzer", 1)
        mk_pub = node.create_publisher(Marker, "/x3_explorer/direction", 1)

        def cmd(self, vx, vy, wz):
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
            self.cmd_pub.publish(t)

        def led(self, e):
            self.led_pub.publish(Int32(data=int(e)))

        def buzzer(self, on):
            self.buz_pub.publish(Bool(data=bool(on)))

        def marker(self, rel, free, frame):
            self.mk_pub.publish(_marker(Marker(), rel, free, frame, p,
                                        stamp=node.get_clock().now().to_msg()))

    r = Runner(p, IO())
    node.create_subscription(LaserScan, p.scan_topic, r.on_scan, qos_profile_sensor_data)
    node.create_timer(1.0 / p.rate, r.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)    # Stopp nicht unterbrechen
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            r.stop()
        except Exception as e:                          # z.B. Kontext schon beendet
            print(f"[x3] Stopp-Befehl nicht gesendet: {e}", file=sys.stderr)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _marker(m, rel, free, frame, p, stamp):
    """Arrow in the lidar frame showing the chosen direction (for RViz)."""
    a = rel + math.radians(p.front_offset)
    if p.flip:
        a = -a
    m.header.frame_id = frame or "laser"
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = "x3_explorer", 0, 0, 0   # ARROW, ADD
    m.pose.orientation.z = math.sin(a / 2)
    m.pose.orientation.w = math.cos(a / 2)
    m.scale.x, m.scale.y, m.scale.z = max(0.1, min(free, 2.0)), 0.05, 0.05
    m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 1.0
    return m


def main():
    # Alles ab --ros-args (ROS2) bzw. name:=wert (ROS1) gehoert ROS, der Rest
    # muss eine eigene Option sein - Tippfehler sonst stumm ignoriert.
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    p = make_parser().parse_args([a for a in argv if ":=" not in a])
    ver = os.environ.get("ROS_VERSION", "")
    if p.calibrate:
        print("[x3] KALIBRIERMODUS - Roboter faehrt nicht. Hand/Objekt vor den "
              "Roboter halten: es muss 'VORNE' und ca. 0° erscheinen.\n"
              "     Links muss ca. +90° sein. Sonst --front-offset / --flip anpassen.")
    if ver == "1":
        run_ros1(p)
    elif ver == "2":
        run_ros2(p)
    else:
        sys.exit("ROS_VERSION nicht gesetzt - bitte ROS-Umgebung sourcen "
                 "(oder start_autonomy.sh benutzen).")


if __name__ == "__main__":
    main()
