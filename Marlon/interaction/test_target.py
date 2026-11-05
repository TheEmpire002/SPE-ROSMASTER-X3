"""Checks the target lock while following (tracker, TargetLock, behavior) -- without a robot.

    python Marlon/interaction/test_target.py

Synthetic scenes with several people:

* two people crossing (with a short occlusion at the crossing point);
* the robot turning on the spot: the people move across the image, IDs stay;
* short occlusion of the target while another person is visible;
* a second person enters during the search -> safe stop, do not follow;
* two identically dressed people after losing the target -> safe stop;
* starting "follow" without a selection with two people -> rejected;
* the follow gesture makes the gesturing person the target, not the largest one;
* a passer-by cannot redirect the following, but can stop it with a gesture;
* it remembers whom it follows: after a stop or safe stop, "follow" with two
  people picks the same one again; releasing and expiry clear that;
* faster re-finding: search earlier if the person disappeared at the image
  edge, turn towards a lidar person, hold still while confirming.

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


# --- TargetLock: occlusion -------------------------------------------------------


def run(tr, sp, scenes, start=0.0, hz=10.0):
    """scenes: list of person lists per frame. Returns the assigned tracks."""
    result = []
    for i, entries in enumerate(scenes):
        t = start + i / hz
        for p in entries:
            p["zeit"] = t
        tr.update(entries, t)
        result.append(sp.assign(tr.tracks, t, t))
    return result


tr, sp = zl.PersonTracker(), zl.TargetLock()
sp.acquire(None, 0.0)
run(tr, sp, [[person(0, RED)] for _ in range(15)])
check("Acquire: one person -> follows", sp.state == zl.FOLLOWING, sp.as_dict())
target_id = sp.id
# B joins (right), then A is occluded for 1 s, B stays visible.
run(tr, sp, [[person(0, RED), person(-25, BLUE)] for _ in range(5)], start=1.5)
path = run(tr, sp, [[person(-25, BLUE)] for _ in range(10)], start=2.0)
check("Occlusion: B never becomes the target", all(x is None for x in path), [x and x.id for x in path])
check("Occlusion: state temporarily lost", sp.state == zl.TEMPORARILY_LOST, sp.state)
back = run(tr, sp, [[person(3, RED), person(-25, BLUE)] for _ in range(3)], start=3.0)
check("Short occlusion: same track, target again immediately",
       back[0] is not None and sp.id == target_id and sp.state == zl.FOLLOWING, sp.as_dict())

# Long occlusion (2 s): the track is discarded, red comes back as a new track
# and first has to confirm itself over several frames.
run(tr, sp, [[person(-25, BLUE)] for _ in range(20)], start=3.3)
check("Long occlusion: old track discarded", tr.track(target_id) is None)
back = run(tr, sp, [[person(5, RED), person(-25, BLUE)] for _ in range(8)], start=5.3)
found = [x for x in back if x is not None]
check("Long occlusion: red is recognized again", found and found[-1].person["merkmal"] is RED
       and sp.state == zl.FOLLOWING and sp.id != target_id, sp.as_dict())
# Red is a clear match (different color from blue) -> the short confirmation suffices.
check("Long occlusion: only after confirmation",
       all(x is None for x in back[:config.TARGET_CONFIRM_FAST_FRAMES - 1]), [x and x.id for x in back])
check("Long occlusion: clear match confirmed quickly",
       back[config.TARGET_CONFIRM_FAST_FRAMES - 1] is not None, [x and x.id for x in back])

# Crossing with the lock: stay with the right one
tr, sp = zl.PersonTracker(), zl.TargetLock()
sp.acquire(None, 0.0)
run(tr, sp, [[person(20, RED)] for _ in range(15)])
scenes = []
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    scenes.append([person(wb, BLUE)] if abs(wa - wb) < 5 else [person(wa, RED), person(wb, BLUE)])
res = run(tr, sp, scenes, start=1.5)
check("Crossing: target is never the blue person",
       all(x is None or x.person["merkmal"] is RED for x in res))
check("Crossing: at the end it follows red again", res[-1] is not None and res[-1].person["merkmal"] is RED,
       sp.as_dict())


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


# App start (button, without tapping) with two people -> rejected
ia, wd, po = interaction()
po.entries = [person(10, RED), person(-15, BLUE)]
ia._switch(vh.FOLLOW, "Test", 0.0)
v = play(ia, po, lambda t: [person(10, RED), person(-15, BLUE)], 0.0, 0.5)
check("App start with two people: back to ready", ia._mode == vh.READY, ia._mode)
check("App start with two people: does not drive",
       all(b is None or (b.vx == 0 and b.omega == 0) for _, _, b in v), [b for _, _, b in v][:3])
check("App start with two people: reason for the app",
       "antippen" in ia.state()["ziel"]["grund"], ia.state()["ziel"])
check("App start with two people: message",
       any("antippen" in m for m in ia.fetch_messages()))

# App start with a selection (ID) with two people -> follows exactly that one
ia, wd, po = interaction()
play(ia, po, lambda t: [person(10, RED), person(-15, BLUE)], 0.0, 0.3)
blue_id = next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is BLUE)
ia._selected_id = blue_id
ia._switch(vh.FOLLOW, "Test", 0.3)
v = play(ia, po, lambda t: [person(10, RED), person(-15, BLUE)], 0.3, 1.8)
check("App start with selection: follows", ia._mode == vh.FOLLOW and ia._lock.id == blue_id,
       ia.state()["ziel"])
check("App start with selection: turns to the selected person (right)", v[-1][2].omega < 0, v[-1][2])

# A second person enters during the search -> safe stop, do not follow.
# The default is searching without a time limit; the safe stop at the end only
# exists with a limit, so set one here.
search_old, config.FOLLOW_SEARCH_S = config.FOLLOW_SEARCH_S, 6.0
ia, wd, po = interaction()
follows_red(ia, po)
check("Search: before that it follows red", ia._lock.state == zl.FOLLOWING, ia.state()["ziel"])
end = 1.5 + config.FOLLOW_LOST_S + 0.5
play(ia, po, lambda t: [], 1.5, end)
check("Search: red gone -> searches", ia._mode == vh.SEARCHING, ia._mode)
v = play(ia, po, lambda t: [person(-10, BLUE)], end, end + config.FOLLOW_SEARCH_S + 0.5)
check("Search: never follows blue", all(m in (vh.SEARCHING, vh.READY) for _, m, _ in v),
       sorted({m for _, m, _ in v}))
check("Search: never drives forward meanwhile", all(b is None or b.vx <= 0 for _, _, b in v))
check("Search: safely stopped at the end",
       ia._mode == vh.READY and ia._lock.state == zl.SAFE_STOP and wd.stops >= 1,
       ia.state()["ziel"])
config.FOLLOW_SEARCH_S = search_old

# Red comes back during the search, blue stands next to it -> follows red
ia, wd, po = interaction()
follows_red(ia, po)
end = 1.5 + config.FOLLOW_LOST_S + 0.5
play(ia, po, lambda t: [], 1.5, end)
play(ia, po, lambda t: [person(-12, BLUE), person(15, RED)], end, end + 1.0)
target = ia._choose_target(end + 1.0)
check("Search: red back -> follows red again",
       ia._mode == vh.FOLLOW and target is not None and target["merkmal"] is RED, (ia._mode, target and target["winkel"]))

# Two identically dressed people after losing the target -> do not guess
ia, wd, po = interaction()
follows_red(ia, po)
end = 1.5 + config.FOLLOW_LOST_S + 0.5
play(ia, po, lambda t: [], 1.5, end)
v = play(ia, po, lambda t: [person(-15, RED), person(15, RED)], end, end + 1.5)
check("Same clothing: follows nobody", all(m != vh.FOLLOW for _, m, _ in v[1:]),
       sorted({m for _, m, _ in v}))
check("Same clothing: safely stopped",
       ia._mode == vh.READY and ia._lock.state == zl.SAFE_STOP, ia.state()["ziel"])
check("Same clothing: reason 'nicht eindeutig'", "eindeutig" in ia.state()["ziel"]["grund"],
       ia.state()["ziel"])

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

# Switching off clears everything learned
ia, wd, po = interaction()
follows_red(ia, po)
ia._forget_target()
check("Forget: no target, no feature, no tracks",
       ia._lock.state == zl.IDLE and ia._lock.reference is None and not ia._tracker.tracks)

# /interaktion/pose: the state contains the target field
z = ia.state()
check("State: field ziel with zustand/id/sicherheit/grund",
       set(z["ziel"]) == {"zustand", "id", "sicherheit", "grund"}, z["ziel"])


# --- Control from the app (step C) -------------------------------------------------


class StillRunning:
    """Stands in for the running interaction loop (active == True)."""

    def done(self):
        return False


def track_id(ia, color):
    return next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is color)


two = lambda t: [person(10, RED), person(-15, BLUE)]

# Tapping: follow with ziel_id selects exactly this person
ia, wd, po = interaction()
ia._task = StillRunning()
play(ia, po, two, 0.0, 0.3)
blue_id = track_id(ia, BLUE)
check("Tap: accepted", ia.follow(blue_id) is True)
check("Tap: text names the person", f"Person {blue_id}" in ia.state()["text"], ia.state()["text"])
v = play(ia, po, two, 0.3, 1.8)
check("Tap: follows blue", ia._mode == vh.FOLLOW and ia._lock.id == blue_id, ia.state()["ziel"])
check("Tap: turns to blue (right)", v[-1][2].omega < 0, v[-1][2])

# Reselecting while following: tapping red replaces blue
red_id = track_id(ia, RED)
check("Reselect: accepted", ia.follow(red_id) is True)
v = play(ia, po, two, 1.8, 3.3)
check("Reselect: now follows red", ia._mode == vh.FOLLOW and ia._lock.id == red_id, ia.state()["ziel"])
check("Reselect: turns to red (left)", v[-1][2].omega > 0, v[-1][2])

# Unknown ID: rejected, target stays
ia.fetch_messages()
check("Unknown ID: rejected", ia.follow(999) is False)
check("Unknown ID: target stays red", ia._lock.id == red_id, ia.state()["ziel"])
check("Unknown ID: message", any("nicht mehr im Bild" in m for m in ia.fetch_messages()))

# Pause: stands still, target stays locked and tracked
check("Pause: accepted", ia.pause() is True)
check("Pause: state reports paused", ia.state()["pausiert"] is True)
# Longer than lost + search: without the pause it would long since be searching.
t0 = 3.3 + config.FOLLOW_LOST_S + config.FOLLOW_SEARCH_S + 1.0
v = play(ia, po, two, 3.3, t0)
check("Pause: does not drive", all(b is None or (b.vx == 0 and b.vy == 0 and b.omega == 0) for _, _, b in v),
       [b for _, _, b in v if b is not None][:2])
check("Pause: mode and target stay", ia._mode == vh.FOLLOW and ia._lock.id == red_id,
       (ia._mode, ia.state()["ziel"]))
check("Pause: pausing twice is not possible", ia.pause() is False)

# Resume: drives to the same target again
check("Resume: accepted", ia.resume() is True)
v = play(ia, po, two, t0, t0 + 1.0)
check("Resume: follows red again", ia._mode == vh.FOLLOW and ia._lock.id == red_id
       and not ia.state()["pausiert"], ia.state()["ziel"])
check("Resume: turns to red again", v[-1][2].omega > 0, v[-1][2])
check("Resume: rejected without pause", ia.resume() is False)

# Pause, then stop gesture: the pause ends with the mode
ia.pause()
play(ia, po, lambda t: [person(10, RED, arm="stopp"), person(-15, BLUE)], t0 + 1.0, t0 + 2.5)
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

# Releasing acknowledges a safe stop
ia._lock.safe_stop("Test", 1.0)
ia._mode = vh.READY
ia.follow_stop()
check("Release: safe_stop becomes idle", ia._lock.state == zl.IDLE, ia.state()["ziel"])

# --- Remembered person (F4) -------------------------------------------------------


def follow_red(start=0.0):
    tr, sp = zl.PersonTracker(), zl.TargetLock()
    sp.acquire(None, start)
    run(tr, sp, [[person(0, RED)] for _ in range(15)], start=start)
    return tr, sp


def restart(tr, sp, start, scene=lambda: [person(10, RED), person(-15, BLUE)], images=10):
    """"Follow" without tapping with several people; reports whether it became ambiguous."""
    sp.acquire(None, start)
    ambiguous = False
    for i in range(images):
        t = start + i / 10.0
        entries = [{**p, "zeit": t} for p in scene()]
        tr.update(entries, t)
        sp.assign(tr.tracks, t, t)
        ambiguous |= sp.ambiguous
    return ambiguous


tr, sp = follow_red()
check("Remember: remembered after following", sp.remembered is not None)
sp.release()
ambiguous = restart(tr, sp, 2.0)
target = tr.track(sp.id) if sp.id is not None else None
check("Remember: after release it picks red again with two people",
       not ambiguous and target is not None and target.person["merkmal"] is RED, sp.as_dict())

tr, sp = follow_red()
sp.safe_stop("Test", 1.5)
ambiguous = restart(tr, sp, 2.0)
target = tr.track(sp.id) if sp.id is not None else None
check("Remember: also after a safe stop",
       not ambiguous and target is not None and target.person["merkmal"] is RED, sp.as_dict())

tr, sp = follow_red()
sp.forget()
check("Remember: forget() clears", sp.remembered is None)
check("Remember: afterwards ambiguous again with two people", restart(tr, sp, 2.0))

tr, sp = follow_red()
sp.release()
check("Remember: expired after TARGET_REMEMBER_S -> ambiguous",
       restart(tr, sp, 1.5 + config.TARGET_REMEMBER_S + 1.0))

tr, sp = follow_red()
sp.release()
ambiguous = restart(tr, sp, 2.0, scene=lambda: [person(10, GREEN), person(-15, BLUE)])
check("Remember: remembered person not there -> ambiguous as before", ambiguous and sp.id is None,
       sp.as_dict())

# Tapping remembers the appearance immediately: if the tracker loses the
# person before even one frame is learned, it finds exactly her again.
tr, sp = zl.PersonTracker(), zl.TargetLock()
run(tr, sp, [[person(10, RED), person(-15, BLUE)] for _ in range(3)])
red_id = next(t.id for t in tr.tracks if t.person["merkmal"] is RED)
sp.acquire(red_id, 0.3, feature=RED)
run(tr, sp, [[person(-15, BLUE)] for _ in range(25)], start=0.3)
check("Tap remembers: old track discarded", tr.track(red_id) is None)
back = run(tr, sp, [[person(12, RED), person(-15, BLUE)] for _ in range(8)], start=2.8)
found = [x for x in back if x is not None]
check("Tap remembers: finds red again, never blue",
       found and all(x.person["merkmal"] is RED for x in found), sp.as_dict())

# Behavior: stop gesture/ready mode keeps the person, releasing does not
ia, wd, po = interaction()
ia._task = StillRunning()
follows_red(ia, po)
ia._switch(vh.READY, "Test", 1.5)
ia._switch(vh.FOLLOW, "Test", 1.6)
play(ia, po, two, 1.6, 3.0)
target = ia._choose_target(3.0)
check("Remember (behavior): follow with two people picks red again",
       ia._mode == vh.FOLLOW and target is not None and target["merkmal"] is RED,
       (ia._mode, ia.state()["ziel"]))
ia.follow_stop()
check("Remember (behavior): releasing forgets", ia._lock.remembered is None and ia._mode == vh.READY)
ia._switch(vh.FOLLOW, "Test", 3.1)
play(ia, po, two, 3.1, 3.6)
check("Remember (behavior): rejects again afterwards", ia._mode == vh.READY
       and "antippen" in ia.state()["ziel"]["grund"], ia.state()["ziel"])

ia, wd, po = interaction()
follows_red(ia, po)
ia._forget_target()
check("Remember (behavior): interaction off forgets", ia._lock.remembered is None)

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
           (ia._mode, ia._lock.state))


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

# While a candidate is being confirmed, hold still instead of turning further
ia, wd, po = interaction()
end = in_search(ia, po)
v = play(ia, po, lambda t: [person(20, RED), person(-15, BLUE)], end, end + 1.0)
checking = [b for _, m, b in v if m == vh.SEARCHING]
check("Checking candidate: confirmation takes several frames", len(checking) >= 1, len(checking))
check("Checking candidate: does not turn meanwhile", all(om(b) == 0 and (b is None or b.vx == 0) for b in checking),
       [om(b) for b in checking])
check("Checking candidate: then it follows red", ia._mode == vh.FOLLOW
       and ia._choose_target(end + 1.0)["merkmal"] is RED, ia.state()["ziel"])

print()
print("all passed" if not error else f"{error} errors")
sys.exit(1 if error else 0)
