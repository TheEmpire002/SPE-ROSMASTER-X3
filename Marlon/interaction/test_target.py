"""Checks whom it follows while following (tracker, TargetChoice, behavior) -- without a robot.

    python Marlon/interaction/test_target.py

Synthetic scenes with several people:

* two people crossing (with a short occlusion at the crossing point);
* the robot turning on the spot: the people move across the image, tracks stay;
* short occlusion of the target while another person is visible;
* the target leaves the image and comes back -> it keeps following
  (the reason why people are no longer told apart since 04.11.);
* starting "follow" with two people -> the larger one;
* the follow gesture makes the gesturing person the target, not the largest one;
* a passer-by cannot redirect the following, but can stop it with a gesture;
* faster re-finding (F4): search earlier if the person disappeared at the image
  edge, and turn towards a person the lidar sees.

Only needs numpy (via the package imports).
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaction import behavior as vh  # noqa: E402
from rosmaster_assist.interaction import target as zl  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0


def check(name, condition, result=""):
    global error
    print(("OK    " if condition else "ERROR "), name, result)
    error += not condition


# --- Synthetic people ----------------------------------------------------------

FOV = config.CAMERA_FOV_DEG
ASPECT = 4 / 3
WIDTH = 0.12


def feature(color):
    """Histogram (16x8) with almost all weight in one bin: a "clothing color"."""
    h = [0.2 / 127] * 128
    h[color] = 0.8
    return h


RED, BLUE, GREEN = feature(3), feature(60), feature(100)


def person(angle, color=RED, timestamp=0.0, size=1.0, arm=None):
    """Person at camera angle ``angle`` (left positive).

    arm: None = arms down, "folgen" = right arm above the head,
    "stopp" = both hands up.
    """
    mx = 0.5 - angle / FOV
    sx = WIDTH * size
    sy = 0.3
    e = sx * ASPECT
    hand_l, hand_r = (0.6, 1.4), (-0.6, 1.4)
    if arm == "folgen":
        hand_r = (-0.4, -1.4)
    elif arm == "stopp":
        hand_l, hand_r = (0.7, -0.6), (-0.7, -0.6)

    def image(u, v):
        return [mx + u * sx, sy + v * e, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = image(0.0, -0.7)
    kp[5], kp[6] = image(0.5, 0.0), image(-0.5, 0.0)
    kp[9], kp[10] = image(*hand_l), image(*hand_r)
    kp[11], kp[12] = image(0.35, 1.8), image(-0.35, 1.8)
    return {
        "box": [mx - sx, sy - 0.1 * size, mx + sx, sy + 0.6 * size],
        "kp": kp,
        "score": 0.9,
        "winkel": angle,
        "seitenverhaeltnis": ASPECT,
        "merkmal": color,
        "zeit": timestamp,
    }


# --- Tracker: crossing ---------------------------------------------------------

# A walks from +20° to -20°, B the other way, 10 frames/s, 2 s. At the
# crossing point the camera only sees the person in front (B).
tr = zl.PersonTracker()
id_a = id_b = None
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    entries = [person(wb, BLUE, t)] if abs(wa - wb) < 5 else [person(wa, RED, t), person(wb, BLUE, t)]
    tr.update(entries, t)
    if i == 0:
        id_a = next(x.id for x in tr.tracks if x.feature == RED)
        id_b = next(x.id for x in tr.tracks if x.feature == BLUE)
ta, tb = tr.track(id_a), tr.track(id_b)
check("Crossing: track A is still alive", ta is not None and ta.measured, [x.id for x in tr.tracks])
check("Crossing: A arrived on the right", ta is not None and ta.person["winkel"] < -15,
       ta and ta.person["winkel"])
check("Crossing: A still wears red", ta is not None and ta.person["merkmal"] is RED)
check("Crossing: B arrived on the left and is blue",
       tb is not None and tb.person["winkel"] > 15 and tb.person["merkmal"] is BLUE)

# --- Tracker: turning on the spot ------------------------------------------------

# Robot turns left at 1 rad/s -> both (standing) people move right at
# ~57°/s. Only 20° between the two, 5 frames/s.
omega = 1.0
tr = zl.PersonTracker()
ids = None
stable = True
for i in range(6):
    t = i * 0.2
    shift = -57.3 * omega * t
    entries = [person(25 + shift, RED, t), person(5 + shift, BLUE, t)]
    tr.update(entries, t, omega=omega)
    assignment = {x.id: x.person["merkmal"] is RED for x in tr.tracks if x.measured}
    if ids is None:
        ids = assignment
    stable &= assignment == ids
check("Turning: IDs stay despite the rotation", stable and len(tr.tracks) == 2,
       [(x.id, round(x.w)) for x in tr.tracks])

check("Turning: people count as standing (v ~ 0)",
       all(abs(x.v) < 5 for x in tr.tracks), [round(x.v) for x in tr.tracks])

# Cross-check: without the turn rate the filter mistakes the rotation for
# movement of the people -- after the robot stops, the predictions would be off.
tr = zl.PersonTracker()
for i in range(6):
    t = i * 0.2
    shift = -57.3 * omega * t
    tr.update([person(25 + shift, RED, t), person(5 + shift, BLUE, t)], t, omega=0.0)
check("Turning: without turn rate wrong person velocity (cross-check)",
       all(x.v < -20 for x in tr.tracks), [round(x.v) for x in tr.tracks])


# --- TargetChoice --------------------------------------------------------------


def run(tr, tc, scenes, start=0.0, hz=10.0):
    """scenes: list of person lists per frame. Returns the assigned tracks."""
    result = []
    for i, entries in enumerate(scenes):
        t = start + i / hz
        for p in entries:
            p["zeit"] = t
        tr.update(entries, t)
        result.append(tc.assign(tr.tracks, t, t))
    return result


tr, tc = zl.PersonTracker(), zl.TargetChoice()
tc.acquire(None, 0.0)
run(tr, tc, [[person(0, RED)] for _ in range(5)])
check("Start: one person -> follows", tc.state == zl.FOLLOWING, tc.as_dict())
# B joins (right), then A is briefly (0.5 s) occluded, B stays visible.
run(tr, tc, [[person(0, RED), person(-25, BLUE)] for _ in range(5)], start=0.5)
path = run(tr, tc, [[person(-25, BLUE)] for _ in range(5)], start=1.0)
check("Short occlusion: B does not become the target", all(x is None for x in path), [x and x.id for x in path])
check("Short occlusion: state temporarily lost", tc.state == zl.TEMPORARILY_LOST, tc.state)
back = run(tr, tc, [[person(3, RED), person(-25, BLUE)] for _ in range(3)], start=1.5)
check("Short occlusion: red again", back[0] is not None and back[0].person["merkmal"] is RED
       and tc.state == zl.FOLLOWING, tc.as_dict())

# The actual case: red leaves the image (track expires) and comes back --
# with a new track number, but it keeps following her immediately.
tr, tc = zl.PersonTracker(), zl.TargetChoice()
tc.acquire(None, 0.0)
run(tr, tc, [[person(10, RED)] for _ in range(5)])
old_id = tc.id
run(tr, tc, [[] for _ in range(30)], start=0.5)
check("Gone and back: old track expired", tr.track(old_id) is None)
back = run(tr, tc, [[person(-5, RED)]], start=3.5)
check("Gone and back: target again immediately", back[0] is not None and tc.state == zl.FOLLOWING,
       tc.as_dict())
check("Gone and back: new track number, does not matter", tc.id != old_id, (old_id, tc.id))

# Gone for long, another person in the image: after TARGET_SWITCH_S it follows her.
tr, tc = zl.PersonTracker(), zl.TargetChoice()
tc.acquire(None, 0.0)
run(tr, tc, [[person(0, RED, size=1.3), person(-25, BLUE)] for _ in range(5)])
check("Start with two: the larger one (red)", tr.track(tc.id).person["merkmal"] is RED, tc.as_dict())
res = run(tr, tc, [[person(-25, BLUE)] for _ in range(20)], start=0.5)
n = int(round(config.TARGET_SWITCH_S * 10))
check("Gone for long: the other person only after TARGET_SWITCH_S",
       all(x is None for x in res[:n - 1]) and res[-1] is not None and res[-1].person["merkmal"] is BLUE,
       [x and x.id for x in res])

# Crossing: at the crossing point red is briefly occluded -- it stays with red.
tr, tc = zl.PersonTracker(), zl.TargetChoice()
tc.acquire(None, 0.0)
run(tr, tc, [[person(20, RED)] for _ in range(5)])
scenes = []
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    scenes.append([person(wb, BLUE)] if abs(wa - wb) < 5 else [person(wa, RED), person(wb, BLUE)])
res = run(tr, tc, scenes, start=0.5)
check("Crossing: target is never the blue person",
       all(x is None or x.person["merkmal"] is RED for x in res))
check("Crossing: at the end it follows red again", res[-1] is not None and res[-1].person["merkmal"] is RED,
       tc.as_dict())


# --- Behavior with stubs ---------------------------------------------------------


class Watchdog:
    def __init__(self):
        self.commands, self.stops = [], 0

    def submit(self, cmd, source="hand"):
        self.commands.append(cmd)

    def stop(self):
        self.stops += 1


class Lidar:
    """Legs of every person as an object at 2 m at their angle."""

    def __init__(self, pose):
        self.pose = pose

    def latest(self):
        angle = [float(w) for w in range(360)]
        r = [5.0] * 360
        for p in self.pose.entries:
            for d in range(-4, 5):
                r[int(round(p["winkel"] + d)) % 360] = 2.0
        return Scan(angle, r)


class Pose:
    available = True
    fov_deg = FOV

    def __init__(self):
        self.entries = []

    def people(self):
        return self.entries

    def activate(self, an):
        pass


class StubOdometry:
    def __init__(self):
        self.omega = 0.0

    def velocity(self):
        return (0.0, 0.0, self.omega)


def interaction():
    po = Pose()
    wd, li = Watchdog(), Lidar(po)
    ia = vh.Interaction(wd, li, po, odometry=StubOdometry())
    ia._mode, ia._mode_since, ia._seen = vh.READY, 0.0, 0.0
    return ia, wd, po


def play(ia, po, producer, from_, until, hz=10.0):
    """producer(t) -> list of people. Returns [(t, mode, command)]."""
    history = []
    n = int(round((until - from_) * hz))
    for i in range(n):
        t = from_ + i / hz
        po.entries = [{**p, "zeit": t} for p in producer(t)]
        history.append((t, ia._mode, ia._step(t)))
    return history


def follows_red(ia, po):
    """Start via button with exactly one person (red in front) until FOLLOWING."""
    ia._switch(vh.FOLLOW, "Test", 0.0)
    play(ia, po, lambda t: [person(0, RED)], 0.0, 1.5)


# App start (button) with two people -> follows the larger one
ia, wd, po = interaction()
two_large = lambda t: [person(10, RED), person(-15, BLUE, size=1.3)]
ia._switch(vh.FOLLOW, "Test", 0.0)
v = play(ia, po, two_large, 0.0, 1.5)
target = ia._choose_target(1.5)
check("App start with two people: follows", ia._mode == vh.FOLLOW, ia._mode)
check("App start with two people: the larger one (blue)", target is not None and target["merkmal"] is BLUE,
       target and target["winkel"])
check("App start with two people: turns to her (right)", v[-1][2].omega < 0, v[-1][2])

# The actual case: red leaves the image, it searches, red comes back
ia, wd, po = interaction()
follows_red(ia, po)
end = 1.5 + config.FOLLOW_LOST_S + 0.5
play(ia, po, lambda t: [], 1.5, end)
check("Search: red gone -> searches", ia._mode == vh.SEARCHING, ia._mode)
ia.fetch_messages()
play(ia, po, lambda t: [person(15, RED)], end, end + 0.5)
check("Search: red back -> follows again", ia._mode == vh.FOLLOW, ia._mode)
check("Search: reports 'wiedergefunden'", any("wiedergefunden" in m for m in ia.fetch_messages()))

# Someone appears during the search -- people are not told apart, it follows.
ia, wd, po = interaction()
follows_red(ia, po)
play(ia, po, lambda t: [], 1.5, end)
play(ia, po, lambda t: [person(-10, BLUE)], end, end + 0.5)
check("Search: whoever appears, it follows", ia._mode == vh.FOLLOW, ia._mode)

# Nobody comes back: with a time limit (default 0 = search endlessly) it stops
# afterwards, the reason is shown in the app.
search_s, config.FOLLOW_SEARCH_S = config.FOLLOW_SEARCH_S, 3.0
ia, wd, po = interaction()
follows_red(ia, po)
play(ia, po, lambda t: [], 1.5, end)
v = play(ia, po, lambda t: [], end, end + config.FOLLOW_SEARCH_S + 0.5)
check("Search: never drives forward meanwhile", all(b is None or b.vx <= 0 for _, _, b in v))
check("Search: stopped at the end, no target",
       ia._mode == vh.READY and ia._target_choice.state == zl.IDLE and wd.stops >= 1,
       ia.state()["ziel"])
check("Search: reason 'nicht wiedergefunden'", "wiedergefunden" in ia.state()["ziel"]["grund"],
       ia.state()["ziel"])
config.FOLLOW_SEARCH_S = search_s

# Follow gesture: the gesturing person becomes the target, not the larger person
ia, wd, po = interaction()
play(ia, po, lambda t: [person(15, BLUE, size=1.3), person(-15, RED, arm="folgen")], 0.0, 1.5)
target = ia._choose_target(1.5)
check("Gesture: follow triggered", ia._mode == vh.FOLLOW, ia._mode)
check("Gesture: target is the gesturing person (red)", target is not None and target["merkmal"] is RED,
       target and target["winkel"])

# Passer-by gestures while following: no switch via come-here, but stop works
ia, wd, po = interaction()
follows_red(ia, po)
play(ia, po, lambda t: [person(0, RED), person(-20, BLUE, arm="folgen")], 1.5, 3.0)
check("Passer-by: gesture does not change the target",
       ia._mode == vh.FOLLOW and ia._choose_target(3.0)["merkmal"] is RED,
       ia.state()["ziel"])
play(ia, po, lambda t: [person(0, RED), person(-20, BLUE, arm="stopp")], 3.0, 4.5)
check("Passer-by: stop gesture stops", ia._mode == vh.READY and wd.stops >= 1, ia._mode)

# Switching off clears everything
ia, wd, po = interaction()
follows_red(ia, po)
ia._forget_target()
check("Forget: no target, no tracks", ia._target_choice.state == zl.IDLE and not ia._tracker.tracks)

# /interaktion/pose: the state contains the target field (interface unchanged)
z = ia.state()
check("State: field ziel with zustand/id/sicherheit/grund",
       set(z["ziel"]) == {"zustand", "id", "sicherheit", "grund"}, z["ziel"])


# --- Control from the app ------------------------------------------------------


class StillRunning:
    """Stands in for the running interaction loop (active == True)."""

    def done(self):
        return False


def track_id(ia, color):
    return next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is color)


two = lambda t: [person(10, RED), person(-15, BLUE)]

# "Folge mir" button: no number in the text
ia, wd, po = interaction()
ia._task = StillRunning()
play(ia, po, two, 0.0, 0.3)
check("Button: accepted", ia.follow() is True)
check("Button: text without person number", ia.state()["text"] == "folgt (per App)", ia.state()["text"])

# Older clients still send ziel_id: that person then counts as the start target.
ia._switch(vh.READY, "Test", 0.3)
blue_id = track_id(ia, BLUE)
check("ziel_id (old): accepted", ia.follow(blue_id) is True)
v = play(ia, po, two, 0.3, 1.8)
check("ziel_id (old): follows blue", ia._mode == vh.FOLLOW and ia._target_choice.id == blue_id, ia.state()["ziel"])
check("ziel_id (old): turns to blue (right)", v[-1][2].omega < 0, v[-1][2])
check("ziel_id unknown: accepted anyway", ia.follow(999) is True)

# Pause: stands still, target stays remembered and tracked
check("Pause: accepted", ia.pause() is True)
check("Pause: state reports paused", ia.state()["pausiert"] is True)
# Longer than lost + search: without the pause it would long since be searching.
t0 = 1.8 + config.FOLLOW_LOST_S + config.FOLLOW_SEARCH_S + 1.0
v = play(ia, po, two, 1.8, t0)
check("Pause: does not drive", all(b is None or (b.vx == 0 and b.vy == 0 and b.omega == 0) for _, _, b in v),
       [b for _, _, b in v if b is not None][:2])
check("Pause: mode and target stay", ia._mode == vh.FOLLOW and ia._target_choice.id == blue_id,
       (ia._mode, ia.state()["ziel"]))
check("Pause: pausing twice is not possible", ia.pause() is False)

# Resume: drives to the same target again
check("Resume: accepted", ia.resume() is True)
v = play(ia, po, two, t0, t0 + 1.0)
check("Resume: follows blue again", ia._mode == vh.FOLLOW and ia._target_choice.id == blue_id
       and not ia.state()["pausiert"], ia.state()["ziel"])
check("Resume: turns to blue again", v[-1][2].omega < 0, v[-1][2])
check("Resume: rejected without pause", ia.resume() is False)

# Pause, then stop gesture: the pause ends with the mode
ia.pause()
play(ia, po, lambda t: [person(10, RED), person(-15, BLUE, arm="stopp")], t0 + 1.0, t0 + 2.5)
check("Pause + stop: ready, no longer paused",
       ia._mode == vh.READY and not ia.state()["pausiert"], (ia._mode, ia.state()["pausiert"]))
check("Pause while standing: rejected", ia.pause() is False)

# Pose for the app: id and ziel per person, no feature
ia, wd, po = interaction()
ia._task = StillRunning()
play(ia, po, two, 0.0, 0.3)
blue_id = track_id(ia, BLUE)
ia.follow(blue_id)
play(ia, po, two, 0.3, 1.0)
entries = ia.people_with_id(po.people())
check("Pose IDs: no feature", all("merkmal" not in p for p in entries))
check("Pose IDs: every person has an ID", all(isinstance(p["id"], int) for p in entries), entries)
check("Pose IDs: exactly blue is the target",
       [p["id"] for p in entries if p["ziel"]] == [blue_id], [(p["id"], p["ziel"]) for p in entries])
check("Pose IDs: order unchanged", [p["winkel"] for p in entries] == [10, -15],
       [p["winkel"] for p in entries])
stranger = ia.people_with_id([person(0, RED)])
check("Pose IDs: new frame before the tracker -> id None", stranger[0]["id"] is None and not stranger[0]["ziel"])

# Releasing also clears the reason of the last search
ia._switch(vh.READY, "Test", 1.0)
ia._target_choice.release("Person nicht wiedergefunden")
ia.follow_stop()
check("Release: no reason any more", ia._target_choice.state == zl.IDLE and not ia._target_choice.reason,
       ia.state()["ziel"])

# --- Faster re-finding (F4) ------------------------------------------------------

# Disappeared at the image edge -> search earlier than in the image center
margin = FOV / 2 - config.TARGET_EDGE_DEG + 3
for name, angle, expected in (("edge", margin, vh.SEARCHING), ("center", 0, vh.FOLLOW)):
    ia, wd, po = interaction()
    ia._switch(vh.FOLLOW, "Test", 0.0)
    play(ia, po, lambda t, w=angle: [person(w, RED)], 0.0, 1.5)
    path = 1.5 + config.FOLLOW_LOST_EDGE_S + 0.3
    play(ia, po, lambda t: [], 1.5, path)
    check(f"Image edge: last seen at {name} -> after {path - 1.5:.1f} s {expected}", ia._mode == expected,
           (ia._mode, ia._target_choice.state))


def om(b):
    return 0.0 if b is None else b.omega


class Human:
    def __init__(self, angle):
        self.category, self.angle, self.distance = vh.hd.HUMAN, angle, 2.0


def in_search(ia, po):
    """Follows red (slightly left), then red disappears -> search; returns the time."""
    ia._switch(vh.FOLLOW, "Test", 0.0)
    play(ia, po, lambda t: [person(5, RED)], 0.0, 1.5)
    end = 1.5 + config.FOLLOW_LOST_S + 0.5
    play(ia, po, lambda t: [], 1.5, end)
    return end


# Lidar sees a person rear right -> turns right, although red was last on the
# left; once she is in the image, it holds still briefly.
ia, wd, po = interaction()
end = in_search(ia, po)
check("Lidar hint: searches", ia._mode == vh.SEARCHING, ia._mode)
direction = [-120.0]
ia._obstacle_detection.current = lambda scan, *a: [Human(direction[0])]
v = play(ia, po, lambda t: [], end, end + 0.5)
check("Lidar hint: turns to the person (right)", all(om(b) < 0 for _, _, b in v), [om(b) for _, _, b in v][:3])
direction[0] = -5.0
v = play(ia, po, lambda t: [], end + 0.5, end + 0.5 + config.TARGET_HINT_S - 0.2)
check("Lidar hint: in the image -> looks still", all(om(b) == 0 for _, _, b in v), [om(b) for _, _, b in v][:3])
t1 = end + 0.5 + config.TARGET_HINT_S
v = play(ia, po, lambda t: [], t1, t1 + config.FOLLOW_SEARCH_TURN_S + config.FOLLOW_SEARCH_PAUSE_S)
check("Lidar hint: nobody recognized -> fixed search rhythm again", any(om(b) != 0 for _, _, b in v))

# No lidar human -> fixed rhythm towards the last side (left) as before
ia, wd, po = interaction()
end = in_search(ia, po)
ia._obstacle_detection.current = lambda scan, *a: []
v = play(ia, po, lambda t: [], end, end + 0.5)
check("Without lidar hint: turns to the last side (left)", om(v[0][2]) > 0, v[0][2])

print()
print("all passed" if not error else f"{error} errors")
sys.exit(1 if error else 0)
