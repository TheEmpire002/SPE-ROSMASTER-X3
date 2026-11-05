"""Checks the healthbot (health check) -- without the robot.

    python Marlon/health/test_health.py

An imaginary world replaces drive, lidar, camera, homing, speaker and
microphone. The person lies where the mock simulation puts them
(``simulation.STANDARD_LOCATION``), as circles for the lidar and as a lying
skeleton for the camera.

Cases:

* assessment: ``interpret_answer`` and ``lying`` (standing/lying);
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
  failing on_finished does not break the healthbot.
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

    def __init__(self, person: str | None = "liegend", moving: bool = False, lidar_sees: bool = True) -> None:
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

    def place(self, x: float, y: float, th: float) -> None:
        self.update()
        self.x, self.y, self.th = x, y, th
        self.vx = self.omega = 0.0

    def air_ahead(self) -> float:
        """Smallest distance lidar -> body edge in front of the robot (±35°)."""
        self.update()
        smallest = math.inf
        for cx, cy, r in self.circles:
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
        for cx, cy, rad in (self.circles if self.lidar_sees else []):
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

    def people(self) -> list[dict]:
        w = self.world
        w.update()
        if w.person is None:
            return []
        ox, oy = sim.STANDARD_LOCATION
        angle = math.degrees(math.atan2(oy - w.y, ox - w.x) - w.th)
        angle = (angle + 180.0) % 360.0 - 180.0
        if abs(angle) > self.fov_deg / 2:
            return []
        mx = 0.5 - angle / self.fov_deg
        if w.person == "liegend":
            distance = max(0.05, min(math.hypot(cx - w.x, cy - w.y) - r for cx, cy, r in w.circles))
            return [pose_module._sim_lying(mx, distance, w.moving, time.monotonic(), self.fov_deg)]
        kp, box = pose_module._sim_body(mx, 0.2, 0.6, 0.03)
        return [pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8,
                                    config.CAMERA_WIDTH / config.CAMERA_HEIGHT, self.fov_deg)]


class Homing:
    """Does not drive, but puts the robot at the goal after a short time.

    Goals other than the start it reaches like the real one with tolerance:
    0.9 m short of it, facing the spot.
    """

    def __init__(self, world: World) -> None:
        self.world = world
        self.goals: list[tuple[float, float, str]] = []
        self.cancelled: list[str] = []
        self._goal = None
        self._end = 0.0
        self.last_reason = None

    def start_to(self, x, y, tolerance=None, name="Startpunkt", blocked_enough_m=0.0) -> bool:
        if self.active:
            return False
        self.goals.append((x, y, name))
        self._goal = (x, y)
        self._end = time.monotonic() + 0.1
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
            d = math.hypot(x - w.x, y - w.y)
            ux, uy = (x - w.x) / d, (y - w.y) / d
            w.place(x - 0.9 * ux, y - 0.9 * uy, math.atan2(uy, ux))
        self._goal = None
        self.last_reason = "angekommen"
        return False

    def cancel(self, reason: str = "abgebrochen") -> None:
        self.cancelled.append(reason)
        self._goal = None
        self.last_reason = reason


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

    def __init__(self, speech: Speech) -> None:
        self.speech = speech
        self.said: list[str] = []

    def say(self, text: str, priority: bool = False) -> None:
        self.said.append(text)
        # the answer comes shortly after the question -- after listening began
        for t in self.speech.answers.get(text, []):
            self.speech.entries.append((time.time() + 0.45, t))


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
                 lidar_sees: bool = True, on_finished=None) -> None:
        self.world = World(person, moving, lidar_sees)
        self.watchdog = Watchdog(self.world)
        self.homing = Homing(self.world)
        self.pose = Pose(self.world)
        self.speech = Speech(answers or {})
        self.voice = Voice(self.speech)
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


def main() -> None:
    # shorter waits keep the test fast; the logic stays the same
    old = {n: getattr(config, n) for n in ("HEALTH_OBSERVE_S", "HEALTH_ANSWER_S", "HEALTH_SEARCH_S", "HEALTH_AUTO_S")}
    old_reaction = pr._REACTION_S
    config.HEALTH_OBSERVE_S = 1.0
    config.HEALTH_ANSWER_S = 0.6
    config.HEALTH_SEARCH_S = 3.0
    config.HEALTH_AUTO_S = 0.5
    pr._REACTION_S = 0.8
    try:
        test_assessment()
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
