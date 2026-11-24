"""Checks the healthbot (health check) -- without the robot.

    python Marlon/health/test_health.py

An imaginary world replaces drive, lidar, camera, homing, speaker and
microphone. The person lies where the mock simulation puts them
(``simulation.STANDARD_LOCATION``), as circles for the lidar and as a lying
skeleton for the camera.

Cases:

* assessment: ``interpret_answer`` and ``lying`` (standing/lying);
* detection: tracking the same person, lying lengthwise to the camera
  (hips above shoulders, shortened torso), majority over frames, motion
  threshold scaled with the body size, watch tolerating short misses;
* lies motionless, does not answer -> nudge twice, no reaction -> alarm,
  announcement, home, music; never deeper than ``HEALTH_NUDGE_DEPTH_M``
  into the person;
* answers "ja", to the follow-up "nein alles gut" -> responsive, home;
* moves, does not answer -> nudge -> reaction -> "hilfe" -> alarm;
* no person -> warning, home, no music;
* nudging switched off -> alarm without a nudge;
* person stands and does not answer -> unremarkable, no nudge;
* cancel in the middle -> stops, homing cancelled;
* watch: lying person detected on its own -> check starts;
* person lies below the lidar plane, only the drive guard (depth, memory)
  sees them -> still approaches and nudges (test log);
* on_finished: called once per outcome with a complete JSON record; a
  failing on_finished does not break the healthbot;
* someone upright next to the person on the floor: the lying one is
  checked and nudged;
* hidden from the arrival point: found from a viewpoint around the spot;
* answers to "geht es ihnen gut?": well -> responsive, unwell -> alarm;
* drive lost for a moment: pauses and goes on; for too long: cancelled;
* fetching help: finds an upright person, they follow, it leads them to
  the person; a helper who does not follow is called again, then it goes
  alone; a sentence that did not come out is said again;
* search.py: viewpoints, search points on a map, someone behind the robot.
"""

import asyncio
import json
import math
import os
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.health import assessment as am  # noqa: E402
from rosmaster_assist.health import procedure as pr  # noqa: E402
from rosmaster_assist.health import search  # noqa: E402
from rosmaster_assist.health import simulation as sim  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety.drive_guard import Check  # noqa: E402
from rosmaster_assist.sensors import pose as pose_module  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0


def check(name: str, condition: bool, info: str = "") -> None:
    global error
    if condition:
        print(f"  ok    {name}")
    else:
        error += 1
        print(f"  FEHLER {name} {info}")


# ---------------------------------------------------------------- world

class World:
    """Robot (x, y, theta) and person; integrates on every access."""

    ROOM = (-3.0, 5.0, -3.0, 3.0)

    def __init__(self, person: str | None = "liegend", moving: bool = False, lidar_sees: bool = True,
                 extras=None) -> None:
        # other people, upright: {"x", "y", "follows"}; a follower walks
        # behind the robot once asked (``following``)
        self.extras = [dict(e) for e in (extras or [])]
        self.following = False
        # where the person on the floor can be seen from, None: everywhere
        self.visible_from = None
        self.x = self.y = self.th = 0.0
        self.vx = self.omega = 0.0
        self.last_command = time.monotonic()
        self.last = time.monotonic()
        self.person = person
        self.moving = moving
        # false: the person lies flatter than the lidar plane; the scan looks
        # over them, only the drive guard (depth, memory) sees them
        self.lidar_sees = lidar_sees
        ox, oy = sim.STANDARD_LOCATION
        if person == "liegend":
            self.circles = [(ox + dx, oy + dy, r) for dx, dy, r in sim._CIRCLES]
        elif person == "stehend":
            self.circles = [(ox, oy, 0.15)]  # legs together, seen from above
        else:
            self.circles = []

    def update(self) -> None:
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        self.th += self.omega * dt
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        if now - self.last_command > 0.5:  # dead man's switch
            self.vx = self.omega = 0.0
        if self.following:
            for e in self.extras:
                if e.get("follows"):
                    e["x"] = self.x - 0.7 * math.cos(self.th)
                    e["y"] = self.y - 0.7 * math.sin(self.th)

    def all_circles(self):
        return self.circles + [(e["x"], e["y"], 0.15) for e in self.extras]

    def place(self, x: float, y: float, th: float) -> None:
        self.update()
        self.x, self.y, self.th = x, y, th
        self.vx = self.omega = 0.0

    def air_ahead(self) -> float:
        """Smallest distance lidar -> body edge in front of the robot (±35°)."""
        self.update()
        smallest = math.inf
        for cx, cy, r in self.all_circles():
            dx, dy = cx - self.x, cy - self.y
            w = math.degrees(math.atan2(dy, dx) - self.th)
            w = (w + 180.0) % 360.0 - 180.0
            if abs(w) <= 35.0:
                smallest = min(smallest, math.hypot(dx, dy) - r)
        return smallest

    def scan(self) -> Scan:
        self.update()
        angles = np.arange(0.0, 360.0, 1.0)
        a = self.th + np.radians(angles)
        ux, uy = np.cos(a), np.sin(a)
        x0, x1, y0, y1 = self.ROOM
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(ux > 0, (x1 - self.x) / ux, (x0 - self.x) / ux)
            ty = np.where(uy > 0, (y1 - self.y) / uy, (y0 - self.y) / uy)
        r = np.minimum(np.abs(tx), np.abs(ty))
        for cx, cy, rad in (self.all_circles() if self.lidar_sees
                            else [(e["x"], e["y"], 0.15) for e in self.extras]):
            # ray x + t*u hits circle: t^2 - 2 t (u.d) + |d|^2 - rad^2 = 0
            dx, dy = cx - self.x, cy - self.y
            b = ux * dx + uy * dy
            c = dx * dx + dy * dy - rad * rad
            disc = b * b - c
            t = np.where(disc >= 0, b - np.sqrt(np.maximum(disc, 0.0)), np.inf)
            t = np.where(t > 0, t, np.inf)
            r = np.minimum(r, t)
        return Scan(list(angles), list(r))


class Watchdog:
    def __init__(self, world: World) -> None:
        self.world = world
        self.last_check = None
        self.stops = 0
        self.smallest_air = math.inf
        self.commands = 0

    def submit(self, cmd: DriveCommand, source: str = "app", margin_m: float | None = None) -> DriveCommand:
        w = self.world
        air = w.air_ahead()
        margin = config.GUARD_MARGIN_M if margin_m is None else margin_m
        vx = cmd.vx
        free = air - config.ROBOT_FRONT_M - margin if vx > 0 and math.isfinite(air) else None
        reason = None
        if vx > 0 and free is not None and free < 0.0:
            vx, reason = 0.0, "Hindernis im Weg"
        command = DriveCommand(vx=vx, vy=cmd.vy, omega=cmd.omega)
        # like the real drive guard: distance until contact, margin included
        self.last_check = Check(command, 1.0 if reason is None else 0.0,
                                None if free is None else max(0.0, free), reason)
        w.vx, w.omega = vx, cmd.omega
        w.last_command = time.monotonic()
        self.smallest_air = min(self.smallest_air, air)
        self.commands += 1
        return command

    def stop(self) -> None:
        self.world.update()
        self.world.vx = self.world.omega = 0.0
        self.stops += 1


class Odometry:
    def __init__(self, world: World) -> None:
        self.world = world

    def pose(self):
        self.world.update()
        return self.world.x, self.world.y, math.degrees(self.world.th)

    def velocity(self):
        self.world.update()
        return self.world.vx, 0.0, self.world.omega


class Lidar:
    def __init__(self, world: World) -> None:
        self.world = world

    def latest(self):
        return self.world.scan()


class Pose:
    available = True

    def __init__(self, world: World) -> None:
        self.world = world
        self.fov_deg = config.CAMERA_FOV_DEG
        self.activated: list[tuple[bool, str]] = []

    def activate(self, on: bool, who: str) -> None:
        self.activated.append((on, who))

    def _image_x(self, x: float, y: float) -> float | None:
        w = self.world
        angle = math.degrees(math.atan2(y - w.y, x - w.x) - w.th)
        angle = (angle + 180.0) % 360.0 - 180.0
        if abs(angle) > self.fov_deg / 2:
            return None
        return 0.5 - angle / self.fov_deg

    def _upright(self, mx: float, distance: float) -> dict:
        # nearer people are larger in the image
        height = max(0.2, min(0.85, 1.2 / max(distance, 0.5)))
        kp, box = pose_module._sim_body(mx, max(0.02, 0.5 - height / 2), height, 0.03)
        return pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8,
                                   config.CAMERA_WIDTH / config.CAMERA_HEIGHT, self.fov_deg)

    def people(self) -> list[dict]:
        w = self.world
        w.update()
        found = []
        ox, oy = sim.STANDARD_LOCATION
        mx = self._image_x(ox, oy) if w.person is not None else None
        if w.visible_from is not None and not w.visible_from(w.x, w.y):
            mx = None
        if mx is not None:
            if w.person == "liegend":
                distance = max(0.05, min(math.hypot(cx - w.x, cy - w.y) - r for cx, cy, r in w.circles))
                found.append(pose_module._sim_lying(mx, distance, w.moving, time.monotonic(), self.fov_deg))
            else:
                kp, box = pose_module._sim_body(mx, 0.2, 0.6, 0.03)
                found.append(pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8,
                                                 config.CAMERA_WIDTH / config.CAMERA_HEIGHT, self.fov_deg))
        for e in w.extras:
            ex = self._image_x(e["x"], e["y"])
            if ex is not None:
                found.append(self._upright(ex, math.hypot(e["x"] - w.x, e["y"] - w.y)))
        found.sort(key=lambda p: p["box"][3] - p["box"][1], reverse=True)
        return found


class Homing:
    """Does not drive, but puts the robot at the goal after a short time.

    Goals other than the start it reaches like the real one with tolerance:
    0.9 m short of it, facing the spot.
    """

    # viewpoints and search points are reached exactly, not short of them
    EXACT = ("Blickpunkt", "Suchpunkt")

    def __init__(self, world: World) -> None:
        self.world = world
        self.goals: list[tuple[float, float, str]] = []
        self.cancelled: list[str] = []
        self.speeds: list[float | None] = []
        # drive time (s) per goal name, otherwise 0.1 s
        self.durations: dict[str, float] = {}
        self._goal = None
        self._name = None
        self._end = 0.0
        self.last_reason = None

    def start_to(self, x, y, tolerance=None, name="Startpunkt", blocked_enough_m=0.0, speed=None) -> bool:
        if self.active:
            return False
        self.goals.append((x, y, name))
        self.speeds.append(speed)
        self._goal = (x, y)
        self._name = name
        self._end = time.monotonic() + self.durations.get(name, 0.1)
        self.last_reason = None
        return True

    @property
    def active(self) -> bool:
        if self._goal is None:
            return False
        if time.monotonic() < self._end:
            return True
        x, y = self._goal
        w = self.world
        if (x, y) == (0.0, 0.0):
            w.place(0.0, 0.0, 0.0)
        else:
            w.update()
            d = max(1e-6, math.hypot(x - w.x, y - w.y))
            ux, uy = (x - w.x) / d, (y - w.y) / d
            short = 0.0 if self._name in self.EXACT else 0.9
            w.place(x - short * ux, y - short * uy, math.atan2(uy, ux))
        self._goal = None
        self.last_reason = "angekommen"
        return False

    def cancel(self, reason: str = "abgebrochen") -> None:
        self.cancelled.append(reason)
        self._goal = None
        self.last_reason = reason


class MapHoming(Homing):
    """Map-frame homing: the map frame is the world frame of the test."""

    def __init__(self, world: World, ready: bool = True) -> None:
        super().__init__(world)
        self.ready = ready
        self.map_goals: list[tuple[float, float] | None] = []
        self.enough: list[float] = []
        self.corridors: list = []

    @property
    def map_ready(self) -> bool:
        return self.ready

    def map_pose(self):
        if not self.ready:
            return None
        self.world.update()
        return self.world.x, self.world.y, self.world.th

    occ_map = None

    def start(self, target=None, angle=None, corridor=None, blocked_enough_m=0.0, speed=None) -> bool:
        if self.active:
            return False
        self._name = None
        self.speeds.append(speed)
        self.map_goals.append(None if target is None else tuple(target))
        self.enough.append(blocked_enough_m)
        self.corridors.append(corridor)
        self._goal = (0.0, 0.0) if target is None else tuple(target)
        self._end = time.monotonic() + 0.1
        self.last_reason = None
        return True

    def state(self) -> dict:
        return {"aktiv": self.active, "weg": [], "ziel": None}


class Speech:
    available = True

    def __init__(self, answers: dict[str, list[str]]) -> None:
        self.answers = answers
        self.entries: list[tuple[float, str]] = []

    def heard_since(self, since: float) -> list[str]:
        now = time.time()
        return [t for z, t in self.entries if since <= z <= now]


class Voice:
    available = True
    speaking = False

    def __init__(self, speech: Speech, world: World | None = None) -> None:
        self.speech = speech
        self.world = world
        self.said: list[str] = []
        # how many of the next sentences get lost (speaker dropout)
        self.drop = 0
        self.said_count = 0

    def say(self, text: str, priority: bool = False) -> bool:
        if self.drop > 0:
            self.drop -= 1
            return True  # queued, but never comes out
        self.said.append(text)
        self.said_count += 1
        if text == config.HEALTH_HELPER_CALL and self.world is not None:
            self.world.following = True
        # the answer comes shortly after the question -- after listening began
        for t in self.speech.answers.get(text, []):
            self.speech.entries.append((time.time() + 0.45, t))
        return True


class Mukke:
    def __init__(self) -> None:
        self.played: list[tuple[str, str]] = []
        self.running = False
        self.stops = 0

    def play_file(self, path: str, title: str) -> None:
        self.played.append((path, title))
        self.running = True

    def stop(self) -> None:
        self.running = False
        self.stops += 1


class Setup:
    def __init__(self, person: str | None = "liegend", moving: bool = False, answers=None,
                 lidar_sees: bool = True, on_finished=None, by_map: bool | None = None,
                 extras=None) -> None:
        self.world = World(person, moving, lidar_sees, extras)
        self.watchdog = Watchdog(self.world)
        # by_map: True map ready, False map homing without a pose, None odometry only
        self.homing = Homing(self.world) if by_map is None else MapHoming(self.world, by_map)
        self.pose = Pose(self.world)
        self.speech = Speech(answers or {})
        self.voice = Voice(self.speech, self.world)
        self.mukke = Mukke()
        self.messages: list[dict] = []
        self.records: list[dict] = []

        async def report(message):
            self.messages.append(message)

        self.hb = pr.Healthbot(
            self.watchdog, Odometry(self.world), self.homing, lidar=Lidar(self.world),
            pose=self.pose, voice=self.voice, speech=self.speech, mukke=self.mukke,
            report=report, on_finished=on_finished or self.records.append,
        )

    def levels(self) -> list[str]:
        return [m["level"] for m in self.messages if m.get("type") == "alert"]

    async def until_done(self, timeout_s: float = 60.0) -> None:
        end = time.monotonic() + timeout_s
        while self.hb.active and time.monotonic() < end:
            await asyncio.sleep(0.05)
        await asyncio.sleep(0)  # let the done callback run


def sign_state(hb, name: str) -> str:
    return next(c["zustand"] for c in hb.status()["checks"] if c["name"] == name)


_RECORD_KEYS = (
    "kind", "started_at", "ended_at", "duration_s", "source", "target", "outcome", "state", "verdict",
    "reason", "alarm", "timeline", "checks", "assessment", "thresholds", "answers_heard",
    "follow_up_interpretation", "nudges", "nudge_active", "spoken", "music", "person_last",
    "wellbeing_interpretation", "posture", "viewpoints", "helper",
)


def check_record(a: Setup, outcome: str) -> dict | None:
    """on_finished got exactly one complete JSON record with ``outcome``."""
    check(f"on_finished called once ({outcome})", len(a.records) == 1, str(len(a.records)))
    if not a.records:
        return None
    r = a.records[0]
    try:
        json.dumps(r)
        ok = True
    except (TypeError, ValueError) as e:
        ok, r_err = False, str(e)
    check("record is JSON", ok, "" if ok else r_err)
    missing = [k for k in _RECORD_KEYS if k not in r]
    check("record has all keys", not missing, str(missing))
    check(f"record outcome {outcome}", r.get("outcome") == outcome, str(r.get("outcome")))
    check("record kind health_check", r.get("kind") == "health_check")
    check("record assessment covers all signs", set(r.get("assessment", {})) == set(am.SIGNS),
          str(list(r.get("assessment", {}))))
    return r


# ---------------------------------------------------------------- cases

def test_assessment() -> None:
    print("Assessment")
    check("„keine Hilfe“ is no", am.interpret_answer(["nein danke keine hilfe"]) == "nein")
    check("„nein hilf mir“ is help", am.interpret_answer(["nein", "hilf mir"]) == "hilfe")
    check("„ja“ is help", am.interpret_answer(["ja"]) == "hilfe")
    check("unclear is None", am.interpret_answer(["der hund bellt"]) is None)
    kp, box = pose_module._sim_body(0.5, 0.2, 0.6, 0.03)
    standing = pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8, 4 / 3, 60)
    lying = pose_module._sim_lying(0.5, 1.0, False, 0.0, 60)
    check("standing is not lying", am.lying(standing)["zustand"] == am.OK, str(am.lying(standing)))
    check("lying is lying", am.lying(lying)["zustand"] == am.FLAGGED, str(am.lying(lying)))
    check("lying sign carries its threshold", am.lying(lying)["grenze"] == config.HEALTH_LYING_DEG,
          str(am.lying(lying)))
    check("no answer without microphone is unknown", am.answer(None)["zustand"] == am.UNKNOWN)


def _body(shoulders, hips, box, conf=0.8):
    """Person with only shoulders and hips set (0..1 image coordinates)."""
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    kp[pose_module.KP_SHOULDER_L] = [shoulders[0][0], shoulders[0][1], conf]
    kp[pose_module.KP_SHOULDER_R] = [shoulders[1][0], shoulders[1][1], conf]
    kp[pose_module.KP_HIP_L] = [hips[0][0], hips[0][1], conf]
    kp[pose_module.KP_HIP_R] = [hips[1][0], hips[1][1], conf]
    return pose_module._person(box, kp, conf, 4 / 3, 60)


def _figure(points: dict[int, tuple[float, float]], box, shift: float = 0.0, conf: float = 0.8) -> dict:
    """Person with the given keypoints (index -> x, y), all shifted by ``shift`` in x."""
    kp = [[0.0, 0.0, 0.0] for _ in range(17)]
    for i, (x, y) in points.items():
        kp[i] = [x + shift, y, conf]
    return pose_module._person(list(box), kp, conf, 4 / 3, 60)


# side view of someone working on an office chair, like the photo of the false
# alarm on 24.11.: torso ~79° upright, thighs horizontal, hands at the desk
SEATED_SIDE = {0: (0.47, 0.30), 5: (0.45, 0.40), 6: (0.46, 0.40), 7: (0.52, 0.52), 9: (0.58, 0.55),
               11: (0.43, 0.62), 12: (0.44, 0.62), 13: (0.56, 0.64), 14: (0.57, 0.64),
               15: (0.57, 0.85), 16: (0.58, 0.85)}
SEATED_BOX = (0.38, 0.25, 0.62, 0.9)


def test_posture() -> None:
    print("Posture and stillness")
    # typing: tiny jitter of the hand, the body still
    still = [_figure(SEATED_SIDE, SEATED_BOX, shift=0.001 * (i % 2)) for i in range(5)]
    check("seated side view is sitting", am.posture(still[0]) == "sitzt", str(am.posture(still[0])))
    s = am.motionless(still)
    check("sitting quietly on a chair is no warning sign", s["zustand"] == am.OK and s["haltung"] == "sitzt", str(s))
    check("the text says why", "sitzt aufrecht" in s["text"] and "kein Warnzeichen" in s["text"], s["text"])

    behind_desk = {i: p for i, p in SEATED_SIDE.items() if i not in (13, 14, 15, 16)}
    hidden = [_figure(behind_desk, (0.38, 0.25, 0.62, 0.65)) for _ in range(5)]
    check("legs behind the desk is upright", am.posture(hidden[0]) == "aufrecht", str(am.posture(hidden[0])))
    check("quiet upright behind a desk is no warning sign", am.motionless(hidden)["zustand"] == am.OK)

    kp, box = pose_module._sim_body(0.5, 0.2, 0.6, 0.03)
    standing = pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8, 4 / 3, 60)
    check("standing person is standing", am.posture(standing) == "steht", str(am.posture(standing)))
    check("standing still is no warning sign", am.motionless([standing] * 5)["zustand"] == am.OK)

    # slumped on the chair: head hangs below the shoulders
    slumped_points = dict(SEATED_SIDE)
    slumped_points[0] = (0.50, 0.45)
    slumped = [_figure(slumped_points, SEATED_BOX) for _ in range(5)]
    check("hanging head is slumped", am.posture(slumped[0]) == "zusammengesackt", str(am.posture(slumped[0])))
    s = am.motionless(slumped)
    check("slumped and still stays motionless", s["zustand"] == am.FLAGGED and "zusammengesackt" in s["text"], str(s))

    lying = [pose_module._sim_lying(0.5, 1.0, False, i * 0.1, 60) for i in range(6)]
    s = am.motionless(lying)
    check("lying still stays motionless", s["zustand"] == am.FLAGGED and s["haltung"] == "liegt", str(s))

    # the verdict never calls an emergency for someone sitting quietly
    checks = am.empty()
    checks["liegend"] = am.lying_over(still)
    checks["regungslos"] = am.motionless(still)
    checks["antwort"] = am.answer([])
    checks["boden"] = am.sign("boden", am.OK, 0.2, "Beine")
    v = am.overall_verdict(checks)
    check("sitting quietly without answer is no emergency", v["stufe"] != "notfall", str(v))


def test_detection() -> None:
    print("Detection")
    old = {n: getattr(config, n) for n in ("HEALTH_AUTO_S", "HEALTH_AUTO_GAP_S", "HEALTH_LYING_SHARE",
                                           "HEALTH_MOTIONLESS_REF_DIAG_PX")}
    config.HEALTH_AUTO_S, config.HEALTH_AUTO_GAP_S, config.HEALTH_LYING_SHARE = 3.0, 1.0, 0.6
    try:
        # tracking
        big = _body(((0.6, 0.3), (0.8, 0.3)), ((0.62, 0.6), (0.78, 0.6)), [0.5, 0.1, 0.9, 0.95])
        small = _body(((0.15, 0.4), (0.25, 0.4)), ((0.16, 0.6), (0.24, 0.6)), [0.1, 0.3, 0.3, 0.9])
        moved = dict(small, box=[0.12, 0.31, 0.32, 0.9])
        check("pick: largest without history", am.pick([big, small], None) is big)
        check("pick: keeps the tracked person", am.pick([big, moved], small) is moved)
        check("pick: falls back to the largest", am.pick([big], small) is big)
        check("pick: nobody", am.pick([], small) is None)

        # lying lengthwise: head towards the camera, hips above the shoulders
        head_near = _body(((0.4, 0.75), (0.6, 0.75)), ((0.43, 0.6), (0.57, 0.6)), [0.35, 0.5, 0.65, 0.85])
        s = am.lying(head_near)
        check("hips above shoulders is lying", s["zustand"] == am.FLAGGED and "Kopf zur Kamera" in s["text"], str(s))
        # head away from the camera: shortened torso, wide box
        head_far = _body(((0.42, 0.6), (0.58, 0.6)), ((0.45, 0.65), (0.55, 0.65)), [0.38, 0.45, 0.62, 0.7])
        s = am.lying(head_far)
        check("shortened torso is lying", s["zustand"] == am.FLAGGED and "verkürzt" in s["text"], str(s))
        check("shortened torso carries its threshold", s["grenze"] == config.HEALTH_FORESHORTEN_RATIO, str(s))
        # bent-over standing person: shortened torso, but a narrow box
        bowing = _body(((0.42, 0.4), (0.58, 0.4)), ((0.45, 0.46), (0.55, 0.46)), [0.42, 0.2, 0.58, 0.95])
        check("bent-over standing person stays upright", am.lying(bowing)["zustand"] == am.OK, str(am.lying(bowing)))
        kp, box = pose_module._sim_body(0.5, 0.2, 0.6, 0.03)
        standing = pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8, 4 / 3, 60)
        lying = pose_module._sim_lying(0.5, 1.0, False, 0.0, 60)

        # majority over frames
        s = am.lying_over([lying] * 5 + [standing] * 2)
        check("majority lying is lying", s["zustand"] == am.FLAGGED and "5 von 7" in s["text"], str(s))
        s = am.lying_over([lying] * 2 + [standing] * 5)
        check("majority standing is not lying", s["zustand"] == am.OK and "2 von 7" in s["text"], str(s))
        s = am.lying_over([standing, lying])
        check("a single lying frame among few still needs the share", s["zustand"] == am.OK, str(s))
        check("no frames falls back to the person", am.lying_over([], lying)["zustand"] == am.FLAGGED)
        check("no frames, no person is unknown", am.lying_over([], None)["zustand"] == am.UNKNOWN)

        # motion threshold scaled with the body size
        near = [pose_module._sim_lying(0.5, 0.6, False, i * 0.1, 60) for i in range(10)]
        far = [pose_module._sim_lying(0.5, 4.0, False, i * 0.1, 60) for i in range(10)]
        check("near person gets a larger motion threshold",
              am.motionless_limit_px(near) > am.motionless_limit_px(far),
              f"{am.motionless_limit_px(near):.1f} / {am.motionless_limit_px(far):.1f}")
        lo, hi = config.HEALTH_MOTIONLESS_PX * 0.5, config.HEALTH_MOTIONLESS_PX * 2.0
        check("motion threshold stays within its factors",
              all(lo <= am.motionless_limit_px(h) <= hi for h in (near, far)))
        check("motionless sign reports the scaled threshold",
              am.motionless(far)["grenze"] == round(am.motionless_limit_px(far), 1), str(am.motionless(far)))
        config.HEALTH_MOTIONLESS_REF_DIAG_PX = 0.0
        check("reference 0 switches the scaling off", am.motionless_limit_px(near) == config.HEALTH_MOTIONLESS_PX)

        # watch: short misses tolerated, long ones start over
        def run(pattern):
            w = am.LyingWatch()
            for i, flagged in enumerate(pattern):
                if w.update(i * 0.2, flagged):
                    return i * 0.2
            return None

        steady = [True] * 20
        check("watch: steady lying triggers after HEALTH_AUTO_S", abs((run(steady) or 0) - 3.0) < 0.01, str(run(steady)))
        short_gap = [True] * 8 + [False] * 3 + [True] * 9
        check("watch: a short miss does not restart", run(short_gap) is not None, str(run(short_gap)))
        long_gap = [True] * 8 + [False] * 7 + [True] * 10
        check("watch: a long miss restarts", run(long_gap) is None, str(run(long_gap)))
        sparse = [True, False, False] * 10
        check("watch: mostly misses never triggers", run(sparse) is None, str(run(sparse)))
    finally:
        for n, v in old.items():
            setattr(config, n, v)


async def case_motionless() -> None:
    print("Lies motionless, does not answer")
    a = Setup("liegend")
    check("starts", a.hb.start(*sim.STANDARD_LOCATION) is None)
    check("second start refused", a.hb.start() == "läuft schon")
    await a.until_done()
    s = a.hb.status()
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("alarm set", s["alarm"])
    check("emergency alert", "emergency" in a.levels(), str(a.levels()))
    check("verdict notfall", (s["urteil"] or {}).get("stufe") == "notfall", str(s["urteil"]))
    check("nudged twice", len(s["stupser"]) == config.HEALTH_NUDGE_COUNT, str(s["stupser"]))
    check("nudges end at the person or the distance limit",
          all(st["grund"] in ("Kontakt", pr.GUARD_HOLDS, "Weggrenze") for st in s["stupser"]),
          str(s["stupser"]))
    check("no reaction noticed", not any(st["reaktion"] for st in s["stupser"]))
    limit = config.ROBOT_FRONT_M - config.HEALTH_NUDGE_DEPTH_M - 0.01
    check("never deeper than the nudge depth into the person", a.watchdog.smallest_air >= limit,
          f"{a.watchdog.smallest_air:.3f} < {limit:.3f}")
    check("got close at all", a.watchdog.smallest_air <= config.ROBOT_FRONT_M + 0.02,
          f"{a.watchdog.smallest_air:.3f}")
    check("four signs checked", all(c["zustand"] != am.UNKNOWN for c in s["checks"]), str(s["checks"]))
    check("all four flagged", all(c["zustand"] == am.FLAGGED for c in s["checks"]),
          str([(c["name"], c["zustand"]) for c in s["checks"]]))
    check("help announced", config.HEALTH_ANNOUNCE_HELP in a.voice.said)
    check("music played", len(a.mukke.played) == 1, str(a.mukke.played))
    if a.mukke.played and not os.path.isfile(a.mukke.played[0][0]):
        # soft: the file is copied to the robot separately
        print(f"  note  music file missing: {a.mukke.played[0][0]}")
    check("first to the spot, then home",
          [g[:2] for g in a.homing.goals] == [sim.STANDARD_LOCATION, (0.0, 0.0)], str(a.homing.goals))
    check("stands at the start", math.hypot(a.world.x, a.world.y) < 0.01)
    check("stands still", a.world.vx == 0.0 and a.world.omega == 0.0)
    check("pose released again", a.pose.activated[-1] == (False, "healthbot"), str(a.pose.activated))
    check("status is JSON", bool(json.dumps(s)))
    order = [e["zustand"] for e in s["timeline"]]
    check("procedure in the right order",
          [z for i, z in enumerate(order) if i == 0 or order[i - 1] != z]
          == [pr.APPROACH, pr.OVERVIEW, pr.ADDRESS, pr.NUDGE, pr.HELP], str(order))
    r = check_record(a, "alarm")
    if r:
        check("record alarm and reason", r["alarm"] and bool(r["reason"]), str(r["reason"]))
        check("record source and target", r["source"] == "eingabe"
              and r["target"] == {"x": sim.STANDARD_LOCATION[0], "y": sim.STANDARD_LOCATION[1]}, str(r["target"]))
        check("record lying threshold", r["assessment"]["liegend"]["threshold"] == config.HEALTH_LYING_DEG
              and r["assessment"]["liegend"]["flagged"], str(r["assessment"]["liegend"]))
        check("record nudges", len(r["nudges"]) == config.HEALTH_NUDGE_COUNT, str(r["nudges"]))
        check("record thresholds", r["thresholds"].get("HEALTH_NUDGE_DEPTH_M") == config.HEALTH_NUDGE_DEPTH_M,
              str(r["thresholds"]))
        check("record music played", r["music"]["played"] is True, str(r["music"]))
        check("record spoken includes the question", any(x["text"] == config.HEALTH_QUESTION for x in r["spoken"]))
        check("record answers per phase",
              [x["phase"] for x in r["answers_heard"]] == ["question", "nudge_1", "nudge_2"],
              str(r["answers_heard"]))


async def case_responsive() -> None:
    print("Answers, needs no help")
    a = Setup("liegend", answers={config.HEALTH_QUESTION: ["ja"], config.HEALTH_QUESTION_HELP: ["nein alles gut"]})
    a.hb.start(*sim.STANDARD_LOCATION)
    await a.until_done()
    s = a.hb.status()
    check("state ansprechbar", s["zustand"] == pr.RESPONSIVE, s["zustand"] + " / " + s["schritt"])
    check("answer ok", sign_state(a.hb, "antwort") == am.OK)
    check("not nudged", not s["stupser"])
    check("no alarm", not s["alarm"] and "emergency" not in a.levels())
    check("asked again", config.HEALTH_QUESTION_HELP in a.voice.said)
    check("no music", not a.mukke.played)
    check("home", a.homing.goals[-1][:2] == (0.0, 0.0))
    # pose off before the drive home, and no frozen skeleton left on the page
    check("pose off before driving home", a.pose.activated.count((False, "healthbot")) >= 2, str(a.pose.activated))
    check("no person shown after the check", s["person"] is None, str(s["person"]))
    check("camera of the keypoints sent", "kamera" in s)
    r = check_record(a, "ok")
    if r:
        heard = {x["phase"]: x["texte"] for x in r["answers_heard"]}
        check("record heard „ja“ on the question", "ja" in heard.get("question", []), str(r["answers_heard"]))
        check("record follow-up answer", "nein alles gut" in heard.get("follow_up", []), str(r["answers_heard"]))
        check("record interpretation nein", r["follow_up_interpretation"] == "nein",
              str(r["follow_up_interpretation"]))
        check("record no alarm", r["alarm"] is False and r["music"]["played"] is False)


async def case_moving() -> None:
    print("Moves, answers only after the nudge")
    a = Setup("liegend", moving=True, answers={config.HEALTH_QUESTION_HELP: ["hilfe"]})
    a.hb.start(*sim.STANDARD_LOCATION)
    await a.until_done()
    s = a.hb.status()
    check("movement detected", sign_state(a.hb, "regungslos") == am.OK)
    check("nudged once, reaction", len(s["stupser"]) == 1 and s["stupser"][0]["reaktion"], str(s["stupser"]))
    check("asked again", config.HEALTH_QUESTION_HELP in a.voice.said)
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("reason: asks for help", (s["urteil"] or {}).get("text") == "bittet um Hilfe", str(s["urteil"]))
    check("music played", len(a.mukke.played) == 1)
    r = check_record(a, "alarm")
    if r:
        check("record reason bittet um Hilfe", r["reason"] == "bittet um Hilfe", str(r["reason"]))
        check("record interpretation hilfe", r["follow_up_interpretation"] == "hilfe")


async def case_no_person() -> None:
    print("No person there")
    a = Setup(None)
    a.hb.start()
    await a.until_done()
    s = a.hb.status()
    check("state keine_person", s["zustand"] == pr.NO_PERSON, s["zustand"] + " / " + s["schritt"])
    check("warning", "warning" in a.levels(), str(a.levels()))
    check("turned around", a.watchdog.commands > 5)
    check("home", a.homing.goals == [(0.0, 0.0, "Startpunkt")], str(a.homing.goals))
    check("no music", not a.mukke.played)
    r = check_record(a, "no_person")
    if r:
        check("record without target", r["target"] is None)


async def case_without_nudging() -> None:
    print("Nudging switched off")
    old = config.HEALTH_NUDGE_ACTIVE
    config.HEALTH_NUDGE_ACTIVE = False
    try:
        a = Setup("liegend")
        a.hb.start(*sim.STANDARD_LOCATION)
        await a.until_done()
        s = a.hb.status()
        check("state hilfe", s["zustand"] == pr.HELP, s["zustand"])
        check("no nudge", not s["stupser"])
        check("status reports it", s["stupsen_aktiv"] is False)
        check("did not drive close", a.watchdog.smallest_air > 0.5, f"{a.watchdog.smallest_air:.2f}")
        r = check_record(a, "alarm")
        if r:
            check("record nudge_active false", r["nudge_active"] is False)
    finally:
        config.HEALTH_NUDGE_ACTIVE = old


async def case_standing() -> None:
    print("Stands and does not answer")
    a = Setup("stehend")
    a.hb.start(*sim.STANDARD_LOCATION)
    await a.until_done()
    s = a.hb.status()
    check("state unauffaellig", s["zustand"] == pr.UNREMARKABLE, s["zustand"] + " / " + s["schritt"])
    check("lying ok", sign_state(a.hb, "liegend") == am.OK)
    check("floor ok", sign_state(a.hb, "boden") == am.OK, str(s["checks"][3]))
    check("no nudge", not s["stupser"])
    check("no alarm", not s["alarm"])
    check_record(a, "ok")


async def case_cancel() -> None:
    print("Cancel")
    a = Setup("liegend")
    a.hb.start(*sim.STANDARD_LOCATION)
    await asyncio.sleep(0.6)
    stops = a.watchdog.stops
    a.hb.cancel("Not-Stopp")
    await asyncio.sleep(0.2)
    s = a.hb.status()
    check("no longer active", not s["aktiv"])
    check("state abgebrochen", s["zustand"] == pr.CANCELLED, s["zustand"])
    check("stopped", a.watchdog.stops > stops and a.world.vx == 0.0 and a.world.omega == 0.0)
    check("homing cancelled", a.homing.cancelled == ["Not-Stopp"], str(a.homing.cancelled))
    check("pose released again", a.pose.activated[-1] == (False, "healthbot"))
    r = check_record(a, "cancelled")
    if r:
        check("record reason Not-Stopp", r["reason"] == "Not-Stopp", str(r["reason"]))


async def case_end_alarm() -> None:
    # test log: after the stop the alarm music kept playing and the robot
    # stayed in state hilfe; after _help the procedure is already over, so
    # cancel() alone did nothing
    print("Stop ends the alarm")
    a = Setup("liegend")
    a.hb.start(*sim.STANDARD_LOCATION)
    a.hb._task.cancel()
    await asyncio.sleep(0.05)
    await a.hb._help("Test")
    s = a.hb.status()
    check("alarm running, procedure over", s["alarm"] and not s["aktiv"] and a.mukke.running, str(s["alarm"]))
    await a.hb.end_alarm("Not-Stopp")
    s = a.hb.status()
    check("music off", not a.mukke.running and a.mukke.stops == 1, str(a.mukke.stops))
    check("alarm off", not s["alarm"])
    check("state bereit", s["zustand"] == pr.READY, s["zustand"] + " / " + s["schritt"])
    check("timeline notes it", "Alarm beendet" in s["timeline"][-1]["text"], s["timeline"][-1]["text"])
    await a.hb.end_alarm("Not-Stopp")
    check("second stop stops nothing more", a.mukke.stops == 1, str(a.mukke.stops))
    check("startable again", a.hb.start(*sim.STANDARD_LOCATION) is None)
    await asyncio.sleep(0.3)
    stops = a.watchdog.stops
    await a.hb.end_alarm("Not-Stopp")
    await asyncio.sleep(0)
    s = a.hb.status()
    check("running check cancelled", not s["aktiv"] and s["zustand"] == pr.CANCELLED, s["zustand"])
    check("stopped meanwhile", a.watchdog.stops > stops)
    check("other music not touched", a.mukke.stops == 1, str(a.mukke.stops))
    check("one record per check, both cancelled",
          [r["outcome"] for r in a.records] == ["cancelled", "cancelled"], str([r["outcome"] for r in a.records]))


async def case_below_lidar() -> None:
    print("Lies below the lidar plane, only the drive guard sees them")
    a = Setup("liegend", lidar_sees=False)
    check("starts", a.hb.start(*sim.STANDARD_LOCATION) is None)
    await a.until_done(90.0)
    s = a.hb.status()
    texts = " | ".join(e["text"] for e in s["timeline"])
    check("did not fail at approaching", "nicht nah genug" not in texts, texts)
    check("nudged twice", len(s["stupser"]) == config.HEALTH_NUDGE_COUNT, str(s["stupser"]))
    check("nudges report the drive guard as the end",
          all(st["grund"] == pr.GUARD_HOLDS for st in s["stupser"]), str(s["stupser"]))
    check("got close", a.watchdog.smallest_air <= config.ROBOT_FRONT_M + 0.02, f"{a.watchdog.smallest_air:.3f}")
    limit = config.ROBOT_FRONT_M - config.HEALTH_NUDGE_DEPTH_M - 0.01
    check("never deeper than the nudge depth into the person", a.watchdog.smallest_air >= limit,
          f"{a.watchdog.smallest_air:.3f} < {limit:.3f}")
    check("nudge distance limited although the lidar sees lots of air",
          all(st["weg_m"] <= config.HEALTH_NUDGE_DEPTH_M + 0.12 for st in s["stupser"]), str(s["stupser"]))
    check("timeline names the drive guard", "Fahrschutz" in texts, texts)
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"])
    check_record(a, "alarm")


async def case_watch() -> None:
    print("Watch detects a lying person")
    a = Setup("liegend")
    ox, oy = sim.STANDARD_LOCATION
    a.world.place(ox - 1.2, oy, 0.0)  # person 1.2 m ahead
    check("watch on", a.hb.set_watch(True) is None and a.hb.watch)
    end = time.monotonic() + config.HEALTH_AUTO_S + 3.0
    while not a.hb.active and time.monotonic() < end:
        await asyncio.sleep(0.05)
    s = a.hb.status()
    check("check started", s["aktiv"])
    check("source selbst erkannt", (s["ziel"] or {}).get("quelle") == "selbst erkannt", str(s["ziel"]))
    if s["ziel"]:
        check("target at the person", math.hypot(s["ziel"]["x"] - ox, s["ziel"]["y"] - oy) < 0.5, str(s["ziel"]))
    await asyncio.sleep(0.3)
    check("watch off afterwards", not a.hb.watch)
    a.hb.cancel("Test")
    await asyncio.sleep(0.1)
    check("watch pose released", (False, "healthbot-wache") in a.pose.activated)
    r = check_record(a, "cancelled")
    if r:
        check("record source selbst erkannt", r["source"] == "selbst erkannt", str(r["source"]))


async def case_without_camera() -> None:
    print("Without camera")
    a = Setup("liegend")
    a.pose.available = False
    check("start refused", a.hb.start() is not None)
    check("watch refused", a.hb.set_watch(True) is not None)
    check("no record without a check", not a.records)


async def case_failing_on_finished() -> None:
    print("Failing on_finished does not break the healthbot")
    calls = []

    def broken(record):
        calls.append(record["outcome"])
        raise RuntimeError("store broken")

    a = Setup(None, on_finished=broken)
    logger = pr.log
    old_disabled = logger.disabled
    logger.disabled = True  # the expected traceback would only clutter the output
    try:
        check("starts", a.hb.start() is None)
        await a.until_done()
        check("called once", calls == ["no_person"], str(calls))
        check("finished normally", a.hb.status()["zustand"] == pr.NO_PERSON, a.hb.status()["zustand"])
        check("startable again", a.hb.start() is None)
        a.hb.cancel("Test")
        await asyncio.sleep(0.1)
        check("called again on cancel", calls == ["no_person", "cancelled"], str(calls))
    finally:
        logger.disabled = old_disabled


async def until_done_watching(a: Setup, timeout_s: float = 60.0) -> dict:
    """Like until_done, but notes what the camera tilt would have been told."""
    seen = {"still": False, "tracked": False, "tracked_while_still": False}
    end = time.monotonic() + timeout_s
    while a.hb.active and time.monotonic() < end:
        still = a.hb.camera_still
        tracked = a.hb.tracked_person is not None
        seen["still"] |= still
        seen["tracked"] |= tracked
        await asyncio.sleep(0.02)
    await asyncio.sleep(0)
    return seen


async def case_map_frame() -> None:
    print("Point on the map: drives by map, faces the person, aims at the legs")
    a = Setup("liegend", by_map=True)
    a.hb.corridor = lambda: "brotkrumen"
    check("starts", a.hb.start(*sim.STANDARD_LOCATION, "karte", frame="karte") is None)
    view = a.hb.map_view()
    check("map view shows the point and area", view["aktiv"] and view["ziel"] == list(sim.STANDARD_LOCATION)
          and view["radius_m"] == config.HEALTH_AREA_RADIUS_M, str(view))
    seen = await until_done_watching(a)
    s = a.hb.status()
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("by map there and back to where it started", a.homing.map_goals == [sim.STANDARD_LOCATION, (0.0, 0.0)],
          str(a.homing.map_goals))
    check("both drives use the breadcrumb corridor", a.homing.corridors == ["brotkrumen"] * 2,
          str(a.homing.corridors))
    check("no dead-reckoning goals", a.homing.goals == [], str(a.homing.goals))
    check("arrives when blocked close enough", a.homing.enough[0] == config.HEALTH_BLOCKED_ENOUGH_M,
          str(a.homing.enough))
    check("target frame karte", (s["ziel"] or {}).get("rahmen") == "karte", str(s["ziel"]))
    steps = [e["text"] for e in s["timeline"]]
    check("looks into the face", "schaut der Person ins Gesicht" in steps, str(steps))
    check("aims at the legs", "zielt auf die Beine" in steps, str(steps))
    check("tilt held still while observing", seen["still"])
    check("tilt follows the person", seen["tracked"])
    check("tilt released at the end", not a.hb.camera_still and a.hb.tracked_person is None)
    check("nudged", len(s["stupser"]) == config.HEALTH_NUDGE_COUNT, str(s["stupser"]))
    check("never deeper than the nudge depth into the person",
          a.watchdog.smallest_air >= config.ROBOT_FRONT_M - config.HEALTH_NUDGE_DEPTH_M - 0.01,
          f"{a.watchdog.smallest_air:.3f}")
    r = check_record(a, "alarm")
    if r:
        check("record target frame", r["target_frame"] == "karte", str(r.get("target_frame")))


async def case_map_outside_area() -> None:
    print("Point on the map far from the person: nobody in the area")
    a = Setup("liegend", by_map=True)
    point = (0.5, 2.0)  # about 3.6 m from the person
    check("starts", a.hb.start(*point, "karte", frame="karte") is None)
    await a.until_done()
    s = a.hb.status()
    check("state keine_person", s["zustand"] == pr.NO_PERSON, s["zustand"] + " / " + s["schritt"])
    check("not nudged", not s["stupser"], str(s["stupser"]))
    goals = a.homing.map_goals
    check("searched the area by map, then home",
          goals[0] == point and goals[-1] == (0.0, 0.0) and len(goals) == 2 + config.HEALTH_VIEWPOINT_MAX,
          str(goals))


async def case_map_refused() -> None:
    print("Map frame without a pose in the map or without a point is refused")
    a = Setup("liegend", by_map=False)
    reason = a.hb.start(1.0, 1.0, "karte", frame="karte")
    check("refused without map pose", reason is not None and "Karte" in reason, str(reason))
    check("not active", not a.hb.active)
    b = Setup("liegend", by_map=True)
    reason = b.hb.start(None, None, "karte", frame="karte")
    check("refused without point", reason is not None, str(reason))
    c = Setup("liegend")
    reason = c.hb.start(1.0, 1.0, "karte", frame="karte")
    check("refused with odometry-only homing", reason is not None, str(reason))
    check("map view without point", c.hb.map_view()["ziel"] is None)


class FakeMap:
    """4 x 4 m, cells of 0.1 m, everything left of x = 1.5 m is a wall."""

    origin = (0.0, 0.0)
    width = height = 40
    resolution = 0.1

    def is_free(self, x: float, y: float) -> bool:
        return 1.5 <= x < 4.0 and 0.0 <= y < 4.0

    def distance_at(self, x: float, y: float) -> float:
        return 1.0


def test_search() -> None:
    print("Search points")
    x, y = search.position((1.0, 0.0, math.pi / 2), 90.0, 2.0)
    check("position: 90° left of facing +y is -x", abs(x + 1.0) < 1e-9 and abs(y) < 1e-9, f"{x}, {y}")
    pts = search.viewpoints((0.0, 0.0), (1.0, 0.0), 1.0, 3, 90.0)
    want = [(0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    check("viewpoints go around from the robot's side",
          all(math.dist(p, q) < 1e-9 for p, q in zip(pts, want)) and len(pts) == 3, str(pts))
    pts = search.viewpoints((2.0, 2.0), (3.0, 2.0), 1.0, 3, 90.0, FakeMap())
    want = [(2.0, 3.0), (2.0, 1.0)]
    check("viewpoints in walls dropped, at most one round",
          len(pts) == 2 and all(math.dist(p, q) < 1e-9 for p, q in zip(pts, want)), str(pts))
    pts = search.search_points(FakeMap(), (0.25, 0.25), 1.0, 0.3, 50)
    check("search points only where free", len(pts) == 12 and all(p[0] >= 1.5 for p in pts), str(pts))
    check("search points nearest first", pts and math.dist(pts[0], (1.5, 0.5)) < 1e-9, str(pts[:1]))
    check("search points limited", len(search.search_points(FakeMap(), (0.0, 0.0), 1.0, 0.3, 3)) == 3)
    check("no map, no search points", search.search_points(None, (0.0, 0.0), 1.0, 0.3, 5) == [])
    behind = [(-0.7, 0.0), (-0.7, 0.05), (-0.7, -0.05)]
    check("someone behind", search.someone_behind(behind, 1.6, 30.0))
    check("in front is not behind", not search.someone_behind([(0.7, 0.0)] * 3, 1.6, 30.0))
    check("the robot's own back is not someone", not search.someone_behind([(-0.15, 0.0)] * 3, 1.6, 30.0))
    check("too far is not following", not search.someone_behind([(-2.0, 0.0)] * 3, 1.6, 30.0))
    check("no scan, nobody behind", not search.someone_behind([], 1.6, 30.0))


def test_wellbeing_and_patient() -> None:
    print("Answers to „geht es ihnen gut?“, who is the patient")
    cases = [
        (["mir geht es gut"], "gut"), (["ja"], "gut"), (["alles okay"], "gut"),
        (["nein"], "hilfe"), (["mir geht es nicht gut"], "hilfe"), (["schlecht"], "hilfe"),
        (["ja aber hilfe"], "hilfe"), (["keine schmerzen alles gut"], "gut"),
        (["nein nein alles gut"], None), (["äh"], None), ([], None),
    ]
    for texts, want in cases:
        got = am.interpret_wellbeing(texts)
        check(f"„{' '.join(texts)}“ -> {want}", got == want, str(got))
    fov = config.CAMERA_FOV_DEG
    lying = pose_module._sim_lying(0.3, 2.0, False, time.time(), fov)
    kp, box = pose_module._sim_body(0.6, 0.05, 0.85, 0.03)
    upright = pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8,
                                  config.CAMERA_WIDTH / config.CAMERA_HEIGHT, fov)
    check("sim lying counts as down, standing does not", am.down(lying) and not am.down(upright))
    check("lying beats a larger upright person", am.pick_patient([upright, lying], None) is lying)
    check("only upright: that one", am.pick_patient([upright], None) is upright)
    check("nobody: None", am.pick_patient([], None) is None)
    check("stays with the lying one", am.pick_patient([upright, lying], lying) is lying)


def test_camera_tilt_hold() -> None:
    print("Camera tilt holds still while the healthbot observes")
    from rosmaster_assist.sensors.camera_tilt import CameraTilt

    class Driver:
        connected = False

    hold = {"on": True}
    head_high = {"winkel": 0.0, "zeit": 1.0, "kp": [[0.5, 0.05, 0.9]] * 17,
                 "box": [0.4, 0.0, 0.6, 0.9]}
    tilt = CameraTilt(Driver(), person_source=lambda: head_high, hold_source=lambda: hold["on"])
    tilt._target = tilt._angle + 10.0  # a move under way
    now = time.monotonic()
    tilt._track(now, 0.1)
    check("held: move stops where it is", tilt._target == tilt._angle, f"{tilt._target} vs {tilt._angle}")
    before = tilt._target
    tilt._track(now + 30.0, 0.1)
    check("held: no return to home and no tracking", tilt._target == before)
    hold["on"] = False
    tilt._track(now + 30.1, 0.1)
    check("released: tracks the head again", tilt._target != before, f"{tilt._target}")


async def case_lying_preferred() -> None:
    print("Someone upright next to the person on the floor: checks the lying one")
    ox, oy = sim.STANDARD_LOCATION
    # upright in the area and in view on arrival, larger in the image
    a = Setup("liegend", extras=[{"x": ox + 1.0, "y": oy + 0.5}])
    a.hb.start(ox, oy)
    await a.until_done(90.0)
    s = a.hb.status()
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("lying flagged", sign_state(a.hb, "liegend") == am.FLAGGED)
    check("nudged the lying one", len(s["stupser"]) == config.HEALTH_NUDGE_COUNT, str(s["stupser"]))
    check("drove at the healthbot speed", a.homing.speeds and a.homing.speeds[0] == config.HEALTH_DRIVE_SPEED,
          str(a.homing.speeds))
    check_record(a, "alarm")


async def case_hidden() -> None:
    print("Hidden from the arrival point: found from a viewpoint")
    ox, oy = sim.STANDARD_LOCATION
    a = Setup("liegend")
    # behind a table: not visible from where it arrives (north-west of the person)
    a.world.visible_from = lambda x, y: not (x < 2.0 and y > -1.2)
    a.hb.start(ox, oy)
    await a.until_done(90.0)
    s = a.hb.status()
    names = [g[2] for g in a.homing.goals]
    check("drove to a viewpoint", "Blickpunkt" in names, str(names))
    check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("nudged", len(s["stupser"]) == config.HEALTH_NUDGE_COUNT, str(s["stupser"]))
    r = check_record(a, "alarm")
    if r:
        check("record viewpoints", len(r["viewpoints"]) >= 1 and r["viewpoints"][0]["erreicht"], str(r["viewpoints"]))


async def case_wellbeing() -> None:
    print("Answers to „geht es ihnen gut?“ are acted on")
    a = Setup("stehend", answers={config.HEALTH_QUESTION: ["mir geht es gut"]})
    a.hb.start(*sim.STANDARD_LOCATION)
    await a.until_done(90.0)
    s = a.hb.status()
    check("well: state ansprechbar", s["zustand"] == pr.RESPONSIVE, s["zustand"] + " / " + s["schritt"])
    check("well: no follow-up question", config.HEALTH_QUESTION_HELP not in a.voice.said, str(a.voice.said))
    check("well: no alarm", not s["alarm"] and not a.mukke.played)
    r = check_record(a, "ok")
    if r:
        check("record interpretation gut", r["wellbeing_interpretation"] == "gut", str(r["wellbeing_interpretation"]))

    b = Setup("liegend", moving=True, answers={config.HEALTH_QUESTION: ["nein mir geht es schlecht"]})
    b.hb.start(*sim.STANDARD_LOCATION)
    await b.until_done(90.0)
    s = b.hb.status()
    check("unwell: state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check("unwell: not nudged", not s["stupser"], str(s["stupser"]))
    check("unwell: no follow-up question", config.HEALTH_QUESTION_HELP not in b.voice.said)
    check("unwell: reason", (s["urteil"] or {}).get("text") == "sagt, dass es ihr nicht gut geht", str(s["urteil"]))
    r = check_record(b, "alarm")
    if r:
        check("record interpretation hilfe", r["wellbeing_interpretation"] == "hilfe")


async def case_speaker_dropout() -> None:
    print("A sentence that did not come out is said again")
    a = Setup("liegend", moving=True, answers={config.HEALTH_QUESTION: ["mir geht es schlecht"]})
    a.voice.drop = 1  # the question gets lost once
    a.hb.start(*sim.STANDARD_LOCATION)
    await a.until_done(90.0)
    s = a.hb.status()
    texts = [e["text"] for e in s["timeline"]]
    check("said again", any(t.startswith("sagt nochmal") for t in texts), str(texts))
    check("question came out", config.HEALTH_QUESTION in a.voice.said)
    check("answer heard after the repeat", s["zustand"] == pr.HELP, s["zustand"])


async def case_drive_lost() -> None:
    print("Drive lost for a moment: pauses and goes on")
    a = Setup("liegend")
    a.homing.durations = {"gemeldete Stelle": 1.0}
    a.hb.start(*sim.STANDARD_LOCATION)
    await asyncio.sleep(0.4)
    a.hb.drive_lost()
    await asyncio.sleep(0.6)
    s = a.hb.status()
    check("still active while lost", s["aktiv"], s["zustand"])
    check("drive to the spot cancelled", len(a.homing.cancelled) == 1, str(a.homing.cancelled))
    check("stands still while lost", a.world.vx == 0.0 and a.world.omega == 0.0)
    a.hb.drive_back()
    await a.until_done(90.0)
    s = a.hb.status()
    names = [g[2] for g in a.homing.goals]
    check("drove on to the spot", names.count("gemeldete Stelle") == 2, str(names))
    check("check finished", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
    check_record(a, "alarm")

    print("Drive lost for too long: cancelled")
    old = config.HEALTH_DRIVE_RECONNECT_S
    config.HEALTH_DRIVE_RECONNECT_S = 0.5
    try:
        b = Setup("liegend")
        b.homing.durations = {"gemeldete Stelle": 1.0}
        b.hb.start(*sim.STANDARD_LOCATION)
        await asyncio.sleep(0.4)
        b.hb.drive_lost()
        await asyncio.sleep(1.2)
        s = b.hb.status()
        check("cancelled", not s["aktiv"] and s["zustand"] == pr.CANCELLED, s["zustand"])
        check("stands still", b.world.vx == 0.0 and b.world.omega == 0.0)
        check_record(b, "cancelled")
    finally:
        config.HEALTH_DRIVE_RECONNECT_S = old


# far enough from the person that they are not behind the robot after it
# drove up to the helper (the fake homing does not turn the robot)
_HELPER = {"x": -0.5, "y": 1.5}


async def case_fetch_help() -> None:
    print("Fetches help: the helper follows to the person")
    old = {n: getattr(config, n) for n in ("HEALTH_HELPER_LOOK_S", "HEALTH_HELPER_SEARCH_S")}
    config.HEALTH_FETCH_HELP = True
    # one full round while looking for a helper
    config.HEALTH_HELPER_LOOK_S = 2.0 * math.pi / config.HEALTH_SEARCH_OMEGA + 1.0
    config.HEALTH_HELPER_SEARCH_S = 40.0
    try:
        a = Setup("liegend", extras=[dict(_HELPER, follows=True)])
        a.hb.start(*sim.STANDARD_LOCATION)
        await a.until_done(120.0)
        s = a.hb.status()
        check("state hilfe", s["zustand"] == pr.HELP, s["zustand"] + " / " + s["schritt"])
        check("asked the helper to follow", config.HEALTH_HELPER_CALL in a.voice.said, str(a.voice.said))
        check("helper followed", a.world.following)
        check("told the helper on arrival", config.HEALTH_HELPER_ARRIVED in a.voice.said, str(a.voice.said))
        check("no music, a helper is there", not a.mukke.played)
        check("not driven home", (0.0, 0.0) not in [g[:2] for g in a.homing.goals], str(a.homing.goals))
        states = [e["zustand"] for e in s["timeline"]]
        check("searched and led", pr.SEEK_HELP in states and pr.LEAD in states, str(states))
        ox, oy = sim.STANDARD_LOCATION
        check("ends at the person", math.hypot(a.world.x - ox, a.world.y - oy) < 1.2,
              f"{a.world.x:.2f}, {a.world.y:.2f}")
        r = check_record(a, "alarm")
        if r:
            check("record helper found and followed", (r["helper"] or {}).get("gefunden") is True
                  and r["helper"].get("gefolgt") is True, str(r["helper"]))

        print("Fetches help: the helper does not follow")
        b = Setup("liegend", extras=[dict(_HELPER, follows=False)])
        b.homing.durations = {"Person": 2.5}
        b.hb.start(*sim.STANDARD_LOCATION)
        await b.until_done(120.0)
        s = b.hb.status()
        calls = b.voice.said.count(config.HEALTH_HELPER_WAIT_CALL)
        check("called again each time", calls == config.HEALTH_LEAD_TRIES, str(calls))
        check("went alone, music", len(b.mukke.played) == 1, str(b.mukke.played))
        check("state hilfe", s["zustand"] == pr.HELP, s["zustand"])
        r = check_record(b, "alarm")
        if r:
            check("record helper not followed", (r["helper"] or {}).get("gefolgt") is False, str(r["helper"]))

        print("Fetches help: nobody there")
        for n, v in old.items():
            setattr(config, n, v)
        c = Setup("liegend")
        c.hb.start(*sim.STANDARD_LOCATION)
        await c.until_done(120.0)
        check("says it finds nobody", config.HEALTH_HELPER_NONE in c.voice.said, str(c.voice.said))
        check("home and music", c.homing.goals[-1][:2] == (0.0, 0.0) and len(c.mukke.played) == 1,
              str(c.homing.goals))
    finally:
        config.HEALTH_FETCH_HELP = False
        for n, v in old.items():
            setattr(config, n, v)


async def run_all() -> None:
    await case_without_camera()
    await case_motionless()
    await case_responsive()
    await case_moving()
    await case_no_person()
    await case_without_nudging()
    await case_standing()
    await case_cancel()
    await case_end_alarm()
    await case_below_lidar()
    await case_watch()
    await case_failing_on_finished()
    await case_map_refused()
    await case_map_frame()
    await case_map_outside_area()
    await case_lying_preferred()
    await case_hidden()
    await case_wellbeing()
    await case_speaker_dropout()
    await case_drive_lost()
    await case_fetch_help()


def main() -> None:
    # shorter waits keep the test fast; the logic stays the same
    short = {
        "HEALTH_OBSERVE_S": 1.0, "HEALTH_ANSWER_S": 0.6, "HEALTH_SEARCH_S": 3.0, "HEALTH_AUTO_S": 0.5,
        # fetching help has its own cases; the others end with home and music
        "HEALTH_FETCH_HELP": False,
        "HEALTH_HELPER_LOOK_S": 2.0, "HEALTH_HELPER_SEARCH_S": 6.0, "HEALTH_VIEWPOINT_SEARCH_S": 1.5,
        "HEALTH_VIEWPOINT_MAX": 2, "HEALTH_AUDIO_WAIT_S": 0.5, "HEALTH_FOLLOW_LOST_S": 0.3,
        "HEALTH_LEAD_WAIT_S": 0.5,
    }
    old = {n: getattr(config, n) for n in short}
    old_reaction = pr._REACTION_S
    for n, v in short.items():
        setattr(config, n, v)
    pr._REACTION_S = 0.8
    try:
        test_assessment()
        test_detection()
        test_posture()
        test_camera_tilt_hold()
        test_search()
        test_wellbeing_and_patient()
        asyncio.run(run_all())
    finally:
        for n, v in old.items():
            setattr(config, n, v)
        pr._REACTION_S = old_reaction
    print()
    print("all cases pass" if error == 0 else f"{error} error(s)")
    sys.exit(1 if error else 0)


if __name__ == "__main__":
    main()
