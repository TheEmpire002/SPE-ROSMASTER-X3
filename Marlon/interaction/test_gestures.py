"""Checks gesture detection, pose evaluation and behavior -- without robot.

    python Marlon/interaction/test_gestures.py

Tested:

* each gesture on its own with artificial keypoints, plus cases that must
  **not** trigger (standing still, hello wave, too short, occluded);
* the sequence of the simulated person from ``main.py --mock`` -- it must
  yield all four gestures in the right order;
* decoding the Hailo outputs (``decode_poses``) with built tensors;
* the behavior (follow, stop, obstacle) with fakes for watchdog, lidar and
  pose.

Needs only numpy.
"""

import asyncio
import math
import pathlib
import random
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

import numpy as np  # noqa: E402

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaction import gestures as gs  # noqa: E402
from rosmaster_assist.interaction import behavior as vh  # noqa: E402
from rosmaster_assist.sensors import pose  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, result)
    error += not condition


# --- Artificial person ---------------------------------------------------------
#
# Thought of in body units: shoulder width 1, shoulder center at (mx, sy).
# hand_l/hand_r as (u, v) relative to it (v downwards), None = occluded.

ASPECT = 4 / 3
WIDTH = 0.12  # shoulder width as a fraction of the image in x


def person(hand_l=(0.6, 1.4), hand_r=(-0.6, 1.4), mx=0.5, sy=0.3, shoulders=True, noise=0.0):
    sx = WIDTH  # in x image coordinates
    sy_unit = WIDTH * ASPECT  # the same length in y image coordinates

    def image(u, v):
        r = random.uniform(-noise, noise)
        return [mx + (u + r) * sx, sy + (v + r) * sy_unit, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = image(0.0, -0.7)
    kp[5] = image(0.5, 0.0) if shoulders else [0, 0, 0.1]
    kp[6] = image(-0.5, 0.0) if shoulders else [0, 0, 0.1]
    kp[9] = image(*hand_l) if hand_l else [0, 0, 0.1]
    kp[10] = image(*hand_r) if hand_r else [0, 0, 0.1]
    kp[11] = image(0.35, 1.8)
    kp[12] = image(-0.35, 1.8)
    fov = config.CAMERA_FOV_DEG
    return {
        "box": [mx - sx, sy - 0.1, mx + sx, sy + 0.6],
        "kp": kp,
        "score": 0.9,
        "winkel": (0.5 - mx) * fov,
        "seitenverhaeltnis": ASPECT,
    }


def play(producer, duration, hz=10.0, start=0.0, detection=None):
    """producer(t) -> person per frame. Returns [(t, gesture)]."""
    detection = detection or gs.GestureDetection()
    hits = []
    for i in range(int(duration * hz)):
        t = start + i / hz
        g = detection.update(producer(t), t)
        if g:
            hits.append((round(t, 1), g))
    return hits


def only(hits):
    return [g for _, g in hits]


random.seed(1)

# --- Individual gestures -------------------------------------------------------

t = play(lambda t: person(hand_r=(-0.4, -1.4)), 2.0)
check("follow: arm above the head", only(t) == [gs.FOLLOW], t)
check("follow: only after the hold time", t and t[0][0] >= 0.8 * config.GESTURE_HOLD_S, t)

t = play(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.5)
check("stop: both hands up", only(t) == [gs.STOP], t)
check("stop: faster than follow", t and t[0][0] < config.GESTURE_HOLD_S, t)


def wave(t):
    return person(hand_r=(-0.7, 0.5 + 0.4 * math.sin(2 * math.pi * 1.5 * t)))


t = play(wave, 2.5)
check("come here: hand waves up and down", only(t) == [gs.COME_HERE], t)


def swipe(t):
    return person(hand_r=(-(0.3 + 1.5 * ((t * 1.0) % 1.0)), 0.4))


t = play(swipe, 2.0)
check("go away: hand swipes outwards", only(t)[:1] == [gs.GO_AWAY], t)

# If the person walks sideways while waving, it stays come here -- the
# measurement is relative to the shoulder center.
t = play(lambda t: person(
    hand_r=(-0.7, 0.5 + 0.4 * math.sin(2 * math.pi * 1.5 * t)), mx=0.3 + 0.1 * t), 2.5)
check("come here: also while walking", only(t) == [gs.COME_HERE], t)

# Small person (far away) -- same gesture, different pixels
WIDTH, old = 0.04, WIDTH
t = play(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.5)
WIDTH = old
check("stop: also far away", only(t) == [gs.STOP], t)

# --- What must not trigger -----------------------------------------------------

t = []
for seed in range(10):  # several random sequences, one lucky hit is not enough
    random.seed(seed)
    t += play(lambda t: person(noise=0.08), 6.0)
check("standing still with jitter: nothing", t == [], t)

t = play(lambda t: person(hand_r=(-1.1 - 0.35 * math.sin(2 * math.pi * 2 * t), -0.3)), 4.0)
check("hello wave (sideways): nothing", t == [], t)

t = play(lambda t: person(hand_r=(-0.4, -1.4)) if t < 0.5 else person(), 3.0)
check("arm up only briefly: nothing", t == [], t)

t = play(lambda t: person(hand_r=(-0.4, -1.4), shoulders=False), 3.0)
check("shoulders occluded: nothing", t == [], t)

t = play(lambda t: person(hand_r=(-0.7, 0.5 + 0.1 * math.sin(2 * math.pi * 1.5 * t))), 3.0)
check("wave too small (jitter): nothing", t == [], t)

t = play(lambda t: person(hand_r=(-0.4, -1.4) if int(t * 10) % 2 else (-0.6, 1.4)), 3.0)
check("flickering detection: nothing", t == [], t)

# Person switch: on the left one holds the arm up, on the right another one
# without a gesture -- if the selection jumps back and forth, nothing may add up.
t = play(lambda t: person(hand_r=(-0.4, -1.4), mx=0.2) if int(t * 10) % 2 else person(mx=0.8), 3.0)
check("two people alternating: nothing", t == [], t)

# --- Pause and priority of stop -----------------------------------------------

e = gs.GestureDetection()
t = play(lambda t: person(hand_r=(-0.4, -1.4)), 4.0, detection=e)
check("held follow counts only once per pause", only(t).count(gs.FOLLOW) <= 2 and t[0][1] == gs.FOLLOW, t)

e = gs.GestureDetection()
play(lambda t: person(hand_r=(-0.4, -1.4)), 1.2, detection=e)
t = play(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.0, start=1.2, detection=e)
check("stop also works right after follow", only(t) == [gs.STOP], t)

e = gs.GestureDetection()
play(lambda t: person(hand_r=(-0.4, -1.4)), 1.2, detection=e)
t = play(wave, 1.5, start=1.2, detection=e)
check("other gesture during the pause: nothing", t == [], t)

check("direction changes are counted", gs.direction_changes([0, 1, 0, 1, 0], 0.5) == 3)
check("direction changes: jitter does not count", gs.direction_changes([0, 0.1, 0, 0.1, 0], 0.5) == 0)

# --- Simulated person from --mock ----------------------------------------------

duration = sum(d for d, _ in pose._SIM_SEQUENCE)
t = play(lambda t: pose._simulated_people(t)[0], duration)
order = [g for i, g in enumerate(only(t)) if i == 0 or only(t)[i - 1] != g]
check(
    "mock person shows all gestures in order",
    order == [gs.FOLLOW, gs.STOP, gs.COME_HERE, gs.GO_AWAY],
    t,
)

# --- decode_poses (Hailo outputs) ----------------------------------------------


def outputs(cells):
    """Built model outputs: cells = [(grid, gy, gx, score_logit, dist, kp_raw)]."""
    parts = []
    for grid in (80, 40, 20):
        box = np.zeros((grid, grid, 64), np.float32)
        score = np.full((grid, grid, 1), -10.0, np.float32)  # Logits
        kp = np.zeros((grid, grid, 51), np.float32)
        for r, gy, gx, s, dist, kp_raw in cells:
            if r != grid:
                continue
            score[gy, gx, 0] = s
            for side in range(4):  # sharp distribution at "dist" grid steps
                box[gy, gx, side * 16 + dist] = 20.0
            kp[gy, gx] = kp_raw
        # Order deliberately shuffled -- decode_poses sorts by shape.
        parts += [kp, box, score]
    return parts


kp_raw_buf = np.zeros(51, np.float32)
kp_raw_buf[0::3] = 0.25  # all points: (0.25*2 + gx) * step
kp_raw_buf[2::3] = 5.0
res = pose.decode_poses(np, outputs([(40, 10, 20, 4.0, 3, kp_raw_buf)]), 640, 0.5)
check("decode: one person", len(res) == 1, len(res))
if res:
    s, box, kp = res[0]
    # grid 40 at 640 px -> step 16; cell (20, 10) -> center (328, 168);
    # 3 steps per side -> 48 px
    check("decode: box", np.allclose(box, [280, 120, 376, 216], atol=0.5), box)
    check("decode: point x", abs(kp[0, 0] - (0.5 + 20) * 16) < 0.5, kp[0])
    check("decode: point y", abs(kp[0, 1] - 10 * 16) < 0.5, kp[0])
    check("decode: confidence", 0.99 < kp[0, 2] <= 1.0 and 0.98 < s <= 1.0, (s, kp[0, 2]))

# The same person in neighboring cells and a second grid -> NMS leaves one;
# a second person far away stays.
res = pose.decode_poses(np, outputs([
    (40, 10, 20, 4.0, 3, kp_raw_buf), (40, 10, 21, 3.0, 3, kp_raw_buf),
    (80, 21, 41, 2.0, 6, kp_raw_buf), (20, 15, 3, 3.0, 2, kp_raw_buf),
]), 640, 0.5)
check("decode: NMS merges", len(res) == 2, [round(e[0], 2) for e in res])
res = pose.decode_poses(np, outputs([(40, 10, 20, 0.0, 3, kp_raw_buf)]), 640, 0.6)
check("decode: too uncertain -> nothing", res == [], len(res))

# --- Behavior with fakes -------------------------------------------------------


class Watchdog:
    def __init__(self):
        self.commands, self.stops = [], 0

    def submit(self, cmd, source="hand"):
        self.commands.append(cmd)

    def stop(self):
        self.stops += 1


class Lidar:
    def __init__(self, objects):
        self.objects = objects  # [(angle_from, angle_to, distance)]

    def latest(self):
        angle = [float(w) for w in range(360)]
        r = [5.0] * 360
        for from_, until, d in self.objects:
            for w in range(360):
                diff = (w - from_) % 360
                if diff <= (until - from_) % 360:
                    r[w] = min(r[w], d)
        scan = Scan(angle, r)
        # Always fresh, no matter which time the test plays (otherwise the
        # result would depend on the computer's uptime since booting).
        scan.timestamp = float("inf")
        return scan


class Pose:
    available = True
    fov_deg = config.CAMERA_FOV_DEG  # like PoseDetection.fov_deg

    def __init__(self):
        self.entries, self.timestamp = [], 0.0

    def people(self):
        return self.entries

    def activate(self, an):
        pass


def interaction(objects, mode):
    wd, li, po = Watchdog(), Lidar(objects), Pose()
    ia = vh.Interaction(wd, li, po)
    ia._mode, ia._mode_since, ia._seen = mode, 0.0, 0.0
    return ia, wd, li, po


def with_time(p, z):
    return {**p, "zeit": z}


# Person straight ahead at 2 m (legs in the lidar) -> forward, barely turning
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("follow: person 2 m ahead -> forward", b and b.vx > 0.1 and abs(b.omega) < 0.05, b)

# Too close (0.6 m) -> back
ia, wd, li, po = interaction([(-5, 5, 0.6)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("follow: person too close -> backwards", b and b.vx < 0, b)

# At the target distance -> stand
ia, wd, li, po = interaction([(-5, 5, config.FOLLOW_DISTANCE_M)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("follow: at the target distance -> stands", b and b.vx == 0, b)

# Person far left -> only turn (left = positive)
ia, wd, li, po = interaction([(30, 40, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(mx=0.0), 0.1)]
b = ia._step(0.1)
check("follow: person left -> turns left, does not drive", b and b.vx == 0 and b.omega > 0, b)

# Obstacle in the driving corridor in front of the person -> do not drive into it
ia, wd, li, po = interaction([(-5, 5, 2.0), (-8, 8, 0.45)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("follow: obstacle in front -> barely moving", b and b.vx <= 0.06, b)

# Something very close -> do not move at all
ia, wd, li, po = interaction([(-5, 5, 2.0), (80, 90, 0.15)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("follow: something very close -> stands", b and b.vx == 0 and b.omega == 0, b)

# Person gone -> wait first, then search
ia, wd, li, po = interaction([], vh.FOLLOW)
ia._angle = 10.0
b = ia._step(1.0)
check("follow: person briefly gone -> holds", b and b.vx == 0 and b.omega == 0 and ia._mode == vh.FOLLOW, b)
ia._step(1.0 + config.FOLLOW_LOST_S + 0.1)
b = ia._step(1.2 + config.FOLLOW_LOST_S)
check("follow: person gone longer -> searches", ia._mode == vh.SEARCHING and b and b.omega > 0, (ia._mode, b))
start = 1.0 + config.FOLLOW_LOST_S + 0.1  # this is when the search began
b = ia._step(start + config.FOLLOW_SEARCH_TURN_S + 0.1)
check("search: hold still briefly after turning", ia._mode == vh.SEARCHING and b and b.omega == 0, b)
b = ia._step(start + config.FOLLOW_SEARCH_TURN_S + config.FOLLOW_SEARCH_PAUSE_S + 0.1)
check("search: then keep turning", b and b.omega > 0, b)
# Without a time limit (default): still searching even after minutes
ia._step(start + 300.0)
check("search without time limit -> keeps searching", config.FOLLOW_SEARCH_S == 0 and ia._mode == vh.SEARCHING, ia._mode)
# Whoever shows up during the search is the person (since 04.11. without
# confirmation by appearance) -- two frames: one for the tracker, one for
# the switch.
for i in range(2):
    t = start + 300.1 + 0.1 * i
    po.entries = [with_time(person(), t)]
    ia._step(t)
check("person back -> follows again", ia._mode == vh.FOLLOW, ia._mode)

# With a time limit: afterwards ready and stopped
old_search = config.FOLLOW_SEARCH_S
config.FOLLOW_SEARCH_S = 6.0
ia, wd, li, po = interaction([], vh.FOLLOW)
ia._angle = 10.0
ia._step(1.0)
ia._step(1.0 + config.FOLLOW_LOST_S + 0.1)
ia._step(1.3 + config.FOLLOW_LOST_S + config.FOLLOW_SEARCH_S)
check("search unsuccessful -> ready and stopped", ia._mode == vh.READY and wd.stops >= 1, ia._mode)
config.FOLLOW_SEARCH_S = old_search

# Stop gesture while following -> ready immediately, watchdog stopped, no more commands
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
for i in range(15):
    z = 0.1 * i
    po.entries = [with_time(person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), z)]
    b = ia._step(z)
check("stop gesture -> ready", ia._mode == vh.READY and wd.stops >= 1, ia._mode)
check("ready -> sends no drive commands", b is None, b)
check("stop gesture -> message", any("Stopp" in m for m in ia.fetch_messages()))

# Follow gesture from READY
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.READY)
for i in range(15):
    z = 0.1 * i
    po.entries = [with_time(person(hand_r=(-0.4, -1.4)), z)]
    ia._step(z)
check("follow gesture -> follows", ia._mode == vh.FOLLOW, ia._mode)

# Come here: arrived -> ready
ia, wd, li, po = interaction([(-5, 5, config.COME_HERE_DISTANCE_M)], vh.COME_HERE)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1)
check("come here: arrived -> ready", ia._mode == vh.READY, ia._mode)

# Go away: clear behind -> backwards; wall behind -> stand
ia, wd, li, po = interaction([(-5, 5, 0.8)], vh.GO_AWAY)
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("go away: clear behind -> backwards", b and b.vx < 0, b)
ia, wd, li, po = interaction([(-5, 5, 0.8), (170, 190, 0.3)], vh.GO_AWAY)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1)
check("go away: wall behind -> ready", ia._mode == vh.READY, ia._mode)
ia, wd, li, po = interaction([(-5, 5, 0.8)], vh.GO_AWAY)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1 + config.GO_AWAY_DISTANCE_M / config.GO_AWAY_SPEED + 0.1)
check("go away: distance done -> ready", ia._mode == vh.READY, ia._mode)

# Without lidar data -> stands
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
li.latest = lambda: None
po.entries = [with_time(person(), 0.1)]
b = ia._step(0.1)
check("without lidar -> stands", b and b.vx == 0 and b.omega == 0, b)

# --- Smooth driving (29.10.2026) -----------------------------------------------

# Briefly lost -> coasts instead of braking hard, after FOLLOW_HOLD_S it stands
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
full = ia._step(0.1)
po.entries = []
b1 = ia._step(0.1 + 0.4 * config.FOLLOW_HOLD_S)
b2 = ia._step(0.1 + config.FOLLOW_HOLD_S + 0.05)
check("coasting: briefly gone -> keeps driving slower", 0 < b1.vx < full.vx, (full, b1))
check("coasting: after the hold time -> stands", b2.vx == 0 and b2.omega == 0, b2)

# Coasting does not drive into an obstacle that has appeared in front in the meantime
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1)
po.entries, li.objects = [], [(-8, 8, 0.38)]
b = ia._step(0.2)
check("coasting: obstacle in front -> stands", b.vx == 0, b)

# Between FULL_ANGLE and MAX_ANGLE the speed decreases steadily instead of stopping abruptly
w = (config.FOLLOW_FULL_ANGLE + config.FOLLOW_MAX_ANGLE_DRIVE) / 2
ia, wd, li, po = interaction([(w - 8, w + 8, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(mx=0.5 - w / config.CAMERA_FOV_DEG), 0.1)]
b = ia._step(0.1)
check("follow: at an angle -> drives throttled and turns", 0 < b.vx < config.FOLLOW_MAX_SPEED and b.omega > 0, b)

# Small angle (image noise) -> do not turn after it
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(mx=0.5 - 2.0 / config.CAMERA_FOV_DEG), 0.1)]
b = ia._step(0.1)
check("follow: 2° off -> does not turn", b.omega == 0 and b.vx > 0, b)

# If the angle jumps for one frame, omega follows only partially
ia, wd, li, po = interaction([(-30, 30, 2.0)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1)
# (below INTERACTION_JUMP_DEG, otherwise it would count as another person)
po.entries = [with_time(person(mx=0.5 - 10 / config.CAMERA_FOV_DEG), 0.2)]
ia._step(0.2)
check("follow: angle jump is smoothed", 0 < ia._smoothed_angle < 10 * 0.5, ia._smoothed_angle)

# Near hysteresis: after "too close" stand a little longer, then continue
ia, wd, li, po = interaction([(-5, 5, 2.0), (80, 90, 0.15)], vh.FOLLOW)
po.entries = [with_time(person(), 0.1)]
ia._step(0.1)
li.objects = [(-5, 5, 2.0)]
po.entries = [with_time(person(), 0.2)]
b1 = ia._step(0.1 + 0.5 * config.FOLLOW_NEAR_HOLD_S)
po.entries = [with_time(person(), 0.3)]
b2 = ia._step(0.1 + config.FOLLOW_NEAR_HOLD_S + 0.05)
check("near hysteresis: right after -> still stands", b1.vx == 0 and b1.omega == 0, b1)
check("near hysteresis: afterwards -> drives again", b2.vx > 0, b2)

# --- Gestures debounced --------------------------------------------------------


def wave_hand(t):
    return (-0.7, 0.5 + 0.4 * math.sin(2 * math.pi * 1.5 * t))


# Walking towards the robot (person grows in the image) while swinging the
# arm -> must not be come here
def approach(t):
    global WIDTH
    old, WIDTH = WIDTH, 0.08 + 0.05 * t
    try:
        return person(hand_r=wave_hand(t))
    finally:
        WIDTH = old


t = play(approach, 3.0)
check("debounced: person comes closer + arm swing -> nothing", t == [], t)

# Walking briskly across the image while swinging -> nothing
t = play(lambda t: person(hand_r=wave_hand(t), mx=0.2 + 0.25 * t), 2.5)
check("debounced: person walks across + arm swing -> nothing", t == [], t)


# From behind (no nose) -> no moving gesture
def from_behind(t):
    p = person(hand_r=wave_hand(t))
    p["kp"][0] = [0, 0, 0.1]
    return p


t = play(from_behind, 3.0)
check("debounced: waving from behind -> nothing", t == [], t)


# Robot drives -> no come here, stop still works
def play_while_driving(producer, duration, hz=10.0):
    detection, hits = gs.GestureDetection(), []
    for i in range(int(duration * hz)):
        z = i / hz
        g = detection.update(producer(z), z, still=False)
        if g:
            hits.append((round(z, 1), g))
    return hits


t = play_while_driving(wave, 3.0)
check("debounced: robot drives -> no come here", t == [], t)
t = play_while_driving(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.5)
check("debounced: robot drives -> stop works", only(t) == [gs.STOP], t)

# Interaction asks the watchdog whether the robot is standing
ia, wd, li, po = interaction([(-5, 5, 2.0)], vh.READY)
wd.current_command = vh.DriveCommand(0.2, 0.0, 0.0)
check("robot drives -> not still", not ia._robot_still())
wd.current_command = vh.DriveCommand(0.0, 0.0, 0.0)
check("robot stands -> still", ia._robot_still())

# --- Ramp in the watchdog ------------------------------------------------------

from rosmaster_assist.safety import watchdog as wdm  # noqa: E402

r = wdm._ramp(0.0, 0.3, 0.8, 2.0, 0.05)
check("ramp: acceleration limited", abs(r - 0.04) < 1e-9, r)
r = wdm._ramp(0.3, 0.0, 0.8, 2.0, 0.05)
check("ramp: braking faster than accelerating", abs(r - 0.2) < 1e-9, r)
r = wdm._ramp(0.1, -0.3, 0.8, 2.0, 0.1)
check("ramp: direction change only down to zero first", r == 0.0, r)
r = wdm._ramp(0.29, 0.3, 0.8, 2.0, 0.05)
check("ramp: small remainder -> exactly at the target", r == 0.3, r)


class Driver:
    def __init__(self):
        self.driven, self.stops = [], 0

    def drive(self, cmd):
        self.driven.append(cmd)

    def stop(self):
        self.stops += 1


an = Driver()
w = wdm.SafetyWatchdog(an)
w._output_at = -1.0  # as if nothing was driven for long -> dt limited to RAMP_DT_MAX_S
first = w.submit(vh.DriveCommand(0.3, 0.0, 0.0))
check("watchdog: jump to 0.3 -> only part of it first", 0 < first.vx < 0.3, first)
for i in range(1, 20):
    w._emit(w._output_at + 0.05)
check("watchdog: ramp reaches the target", w.current_command.vx == 0.3, w.current_command)
w.stop()
check("watchdog: stop() immediately, without ramp", w.current_command.is_stop() and an.stops == 1, w.current_command)
w.submit(vh.DriveCommand(0.3, 0.0, 0.0))
w._last_command_at -= config.COMMAND_TIMEOUT_S + 0.1
w._check_timeout()
check("watchdog: dead man's switch still applies", w.current_command.is_stop() and an.stops == 2, w.current_command)

# --- Drive gone (USB reset at gesture start, 29.10.) ----------------------------
class StillRunning:
    """Stands in for the asyncio task so that `active` is true without an event loop."""

    def done(self):
        return False


connected = [False]
wd, li, po = Watchdog(), Lidar([(-5, 5, 2.0)]), Pose()
ia = vh.Interaction(wd, li, po, drive_connected=lambda: connected[0])
ia._task = StillRunning()
ia._messages = []
ia.follow()  # like the app button
check("drive gone: follow rejected, stays ready", ia._mode == vh.READY, ia._mode)
check("drive gone: message for the app",
       any("Antrieb" in m for m in ia.fetch_messages()))
check("drive gone: speech command come here rejected",
       not ia.switch_to(vh.COME_HERE, "per Sprache"), ia._mode)

connected[0] = True
check("drive back: follow works", ia.switch_to(vh.FOLLOW, "per Sprache"), ia._mode)
stops = wd.stops
connected[0] = False
ia.drive_lost()
check("drive_lost: stops, mode ready", ia._mode == vh.READY and wd.stops > stops,
       (ia._mode, wd.stops))
check("drive_lost: interaction stays on", ia.active)

connected[0] = True
ia._mode, ia._mode_since, ia._seen = vh.FOLLOW, 0.0, 0.0
po.entries = [with_time(person(), 0.1)]
connected[0] = False
b = ia._step(0.1)
check("step without drive: no drive command, ready",
       (b is None or b.is_stop()) and ia._mode == vh.READY, (b, ia._mode))

# --- Soft start (03.11.): warm up, announce, only then drive ---------------------
class VoiceStub:
    """Speaks every sentence for 0.3 s."""

    def __init__(self):
        self.said, self.until = [], 0.0

    def say(self, text, priority=False):
        self.said.append(text)
        self.until = time.monotonic() + 0.3
        return True

    @property
    def busy(self):
        return time.monotonic() < self.until


async def soft_start(stop_during_ramp=False):
    old = config.INTERACTION_STARTUP_S
    config.INTERACTION_STARTUP_S = 0.3
    st = VoiceStub()
    ia = vh.Interaction(Watchdog(), Lidar([(-5, 5, 2.0)]), Pose(), st)
    try:
        ia.start(vh.FOLLOW)
        await asyncio.sleep(0.15)
        if stop_during_ramp:
            ia.switch_to(vh.READY, "angehalten (per Sprache)")
        startup = (ia._mode, list(st.said))
        await asyncio.sleep(0.3)   # startup over, announcement running
        announcement = (ia._mode, list(st.said))
        await asyncio.sleep(0.45)  # finished speaking
        return startup, announcement, ia._mode
    finally:
        ia.cancel()
        config.INTERACTION_STARTUP_S = old


startup, announcement, after = asyncio.run(soft_start())
check("soft start: first silent and standing", startup == (vh.READY, []), startup)
check("soft start: then announcement, still standing",
       announcement == (vh.READY, ["Jetzt folge ich dir."]), announcement)
check("soft start: after the announcement it follows", after == vh.FOLLOW, after)
startup, announcement, after = asyncio.run(soft_start(stop_during_ramp=True))
check("soft start: stop during startup applies", after == vh.READY and announcement[1] != ["Jetzt folge ich dir."],
       (announcement, after))

print("\nall passed" if not error else f"\n{error} error(s)")
sys.exit(1 if error else 0)
