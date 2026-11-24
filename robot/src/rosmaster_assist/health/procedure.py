"""Healthbot procedure: drive there, overview, address, nudge, get help.

States (``status()["zustand"]``)::

    bereit -> anfahrt -> gesamtbild -> ansprechen -+-> nachfrage --> hilfe | ansprechbar
                                                   +-> anstupsen -+
                                                   +-> unauffaellig (home)

Every step goes into ``timeline`` with time and reason; the four signs are
shown as traffic lights in ``checks``. The web app shows both so it stays
clear *why* the robot raises an alarm.

Driving only via ``SafetyWatchdog.submit(source="autonom")``: speed limit,
dead man's switch and drive guard apply here too. While nudging, the drive
guard margin is lowered to ``HEALTH_NUDGE_MARGIN_M`` -- otherwise the robot
would always stop a few centimetres short of the person and never touch them.

When a check ends (any outcome), ``on_finished(record)`` receives a complete,
JSON-serialisable record of it; see ``Healthbot._record``.
"""

from __future__ import annotations

import asyncio
import contextlib
import inspect
import logging
import math
import os
import time

import numpy as np

from .. import config
from ..protocol import DriveCommand, alert_message
from ..safety import drive_guard
from . import assessment as am

# nudge end when the drive guard holds (depth/memory see the person just in
# front of the robot) instead of the wheels pushing against them
GUARD_HOLDS = "Fahrschutz hält"

log = logging.getLogger(__name__)

READY = "bereit"
APPROACH = "anfahrt"
OVERVIEW = "gesamtbild"
ADDRESS = "ansprechen"
NUDGE = "anstupsen"
FOLLOW_UP = "nachfrage"
HELP = "hilfe"
RESPONSIVE = "ansprechbar"
UNREMARKABLE = "unauffaellig"
NO_PERSON = "keine_person"
CANCELLED = "abgebrochen"

_TICK_S = 0.1  # far below the watchdog's dead man time (0.5 s)
_SCAN_MAX_AGE_S = 1.0
_TURN_MAX_S = 10.0
_BACK_MAX_S = 4.0
_REACTION_S = 2.5  # after each nudge, watch this long for movement or words
_SPEAK_MAX_S = 15.0
_MUSIC_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")


class Healthbot:
    def __init__(self, watchdog, odometry, homing, lidar=None, pose=None,
                 voice=None, speech=None, mukke=None, report=None, on_finished=None) -> None:
        """``homing`` is a separate ``Homing`` instance (not the one of the
        homing drive) so its display is not overwritten by healthbot drives.
        ``report(message)`` is an async callback that sends a message to all
        clients. ``on_finished(record)`` is called once per check with its
        record (may be sync or async)."""
        self._watchdog = watchdog
        self._odometry = odometry
        self._homing = homing
        self._lidar = lidar
        self._pose = pose
        self._voice = voice
        self._speech = speech
        self._mukke = mukke
        self._report = report
        self._on_finished = on_finished
        # set by the server: breadcrumb corridor of the map control, so drives
        # by map use the same known-free trail as a goal drive from /map
        self.corridor = None
        # where the check started (frame of the target), the way home leads there
        self._origin: tuple[float, float, float] | None = None
        self._task: asyncio.Task | None = None
        self._watch_task: asyncio.Task | None = None
        self._watch_since: float | None = None
        self._state = READY
        self._step = "wartet auf Meldung"
        self._timeline: list[dict] = []
        self._checks = am.empty()
        self._verdict: dict | None = None
        self._target: dict | None = None
        self._alarm = False
        # remembers whether the running music is ours: the stop should end
        # the alarm, not music the user picked
        self._music_started = False
        self._person: dict | None = None
        # pose runs for the check only until the robot heads home
        self._pose_on = False
        # true while observing: a moving tilt servo would shift all keypoints
        # and look like the person moving
        self._camera_still = False
        self._nudges: list[dict] = []
        self._reset_record()

    def _reset_record(self) -> None:
        self._started_at: float | None = None
        self._source: str | None = None
        self._cancel_reason: str | None = None
        self._error: str | None = None
        self._help_reason: str | None = None
        self._answers: list[dict] = []
        self._spoken: list[dict] = []
        self._interpretation: str | None = None
        self._music: dict = {"played": False, "error": None}

    # ------------------------------------------------------------------ status

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def watch(self) -> bool:
        return self._watch_task is not None and not self._watch_task.done()

    def watch_status(self) -> dict:
        """The watch for lying people: on, and since when someone lies."""
        since = self._watch_since if self.watch else None
        return {
            "an": self.watch,
            "liegt_seit_s": round(time.monotonic() - since, 1) if since is not None else None,
            "noetig_s": config.HEALTH_AUTO_S,
        }

    def status(self) -> dict:
        return {
            "aktiv": self.active,
            "zustand": self._state,
            "schritt": self._step,
            "timeline": list(self._timeline),
            "checks": [dict(self._checks[n]) for n in am.SIGNS],
            "urteil": self._verdict,
            "ziel": self._target,
            "alarm": self._alarm,
            "stupser": list(self._nudges),
            "wache": self.watch,
            "stupsen_aktiv": config.HEALTH_NUDGE_ACTIVE,
            "person": self._person_summary(),
            "kamera": getattr(self._pose, "camera_name", None),
        }

    def map_view(self) -> dict:
        """For the /map page: the point under check, its area and the drive there."""
        by_map = self._by_map()
        return {
            "aktiv": self.active,
            "zustand": self._state,
            "schritt": self._step,
            "ziel": [self._target["x"], self._target["y"]] if by_map else None,
            "radius_m": config.HEALTH_AREA_RADIUS_M,
            "fahrt": self._homing.state() if by_map and self.active else None,
        }

    @property
    def tracked_person(self) -> dict | None:
        """The person under check for the camera tilt (eye contact), else None."""
        if not (self.active and self._pose_on):
            return None
        return self._seen()

    @property
    def camera_still(self) -> bool:
        """The tilt servo should hold its angle (observing the person)."""
        return self.active and self._camera_still

    def _person_summary(self) -> dict | None:
        # live frame instead of the stored one: the stored frame is seconds old
        # while the robot speaks or listens and would sit beside the person
        if not (self.active and self._pose_on):
            return None
        p = self._seen()
        return None if p is None else {"box": p["box"], "kp": p["kp"], "winkel": p["winkel"]}

    def _use_pose(self, on: bool) -> None:
        self._pose_on = on
        self._pose.activate(on, "healthbot")

    def _note(self, text: str, state: str | None = None) -> None:
        if state is not None:
            self._state = state
        self._step = text
        self._timeline.append({"zeit": time.time(), "zustand": self._state, "text": text})
        log.info("HEALTHBOT [%s] %s", self._state, text)

    def _check(self, c: dict) -> None:
        self._checks[c["name"]] = c
        self._note(f"{c['titel']}: {c['text']}")

    async def _send(self, level: str, text: str) -> None:
        if self._report is None:
            return
        try:
            await self._report(alert_message(level, text))
        except Exception:  # a broken connection must not stop the check
            log.exception("HEALTHBOT: alert not sent")

    # ------------------------------------------------------------------ control

    def start(self, x: float | None = None, y: float | None = None, source: str = "eingabe",
              frame: str = "odom") -> str | None:
        """Begin a check. Without a target: check a person near here.

        ``frame`` says what x/y mean: "odom" (odometry frame, dead reckoning)
        or "karte" (map frame: drives by map and only checks a person within
        HEALTH_AREA_RADIUS_M of the point). Returns None or the (German,
        user-facing) reason why it does not work.
        """
        if not config.HEALTH_ACTIVE:
            return "abgeschaltet (config.HEALTH_ACTIVE)"
        if self.active:
            return "läuft schon"
        if self._pose is None or not getattr(self._pose, "available", False):
            return "keine Pose-Erkennung -- ohne Kamera kein Gesamtbild"
        by_map = frame == "karte"
        if by_map and (x is None or y is None):
            return "für die Karte fehlt der Punkt (x, y)"
        if by_map and not getattr(self._homing, "map_ready", False):
            return "Lage in der Karte unbekannt -- erst auf der Karte zeigen, wo er steht"
        self._timeline = []
        self._checks = am.empty()
        self._verdict = None
        self._alarm = False
        self._person = None
        self._nudges = []
        self._reset_record()
        self._started_at = time.time()
        self._source = source
        self._target = None if x is None or y is None else {
            "x": round(float(x), 2), "y": round(float(y), 2), "quelle": source,
            "rahmen": "karte" if by_map else "odom",
        }
        self._origin = self._frame_pose()
        self._task = asyncio.create_task(self._procedure())
        # done callback instead of finally: it also fires when the task is
        # cancelled before it ever ran
        self._task.add_done_callback(self._finished)
        return None

    def cancel(self, reason: str = "abgebrochen") -> None:
        """Stop immediately. Also call this when the human intervenes."""
        if not self.active:
            return
        self._cancel_reason = reason
        self._task.cancel()
        self._homing.cancel(reason)
        self._watchdog.stop()
        self._note(f"abgebrochen: {reason}", CANCELLED)

    async def end_alarm(self, reason: str) -> None:
        """Stop by the human: cancel the check, alarm music off and back to
        ready. After the call for help the check is already over, cancel()
        alone would leave music and alarm running."""
        task = self._task if self.active else None
        self.cancel(reason)
        if task is not None:
            # wait until finally (stop, pose off) has run
            with contextlib.suppress(asyncio.CancelledError):
                await task
        if self._music_started and self._mukke is not None:
            self._music_started = False
            # stop() waits up to 2 s for mpv, so not in the event loop
            await asyncio.to_thread(self._mukke.stop)
        if self._alarm:
            self._alarm = False
            self._note(f"Alarm beendet: {reason} – bereit", READY)

    def set_watch(self, on: bool) -> str | None:
        """Look out for lying people on our own (does not drive off as long
        as none is seen). Returns None or the reason."""
        if not on:
            if self.watch:
                self._watch_task.cancel()
            self._watch_task = None
            if self._pose is not None:
                self._pose.activate(False, "healthbot-wache")
            return None
        if not config.HEALTH_AUTO_DETECT:
            return "abgeschaltet (config.HEALTH_AUTO_DETECT)"
        if self._pose is None or not getattr(self._pose, "available", False):
            return "keine Pose-Erkennung"
        if not self.watch:
            self._watch_task = asyncio.create_task(self._watch())
        return None

    def shutdown(self) -> None:
        """When the server shuts down."""
        self.set_watch(False)
        self.cancel("Server beendet")

    # ------------------------------------------------------------------ record

    def _outcome(self, task: asyncio.Task) -> str:
        if task.cancelled():
            return "cancelled"
        if self._error is not None or task.exception() is not None:
            return "error"
        if self._state == HELP or self._alarm:
            return "alarm"
        if self._state == NO_PERSON:
            return "no_person"
        if self._state in (RESPONSIVE, UNREMARKABLE):
            return "ok"
        return "error"  # ended without a final state: should not happen

    def _record(self, outcome: str) -> dict:
        """Everything about one check, JSON-serialisable.

        Top-level keys are English (the test-run store is new); nested
        objects keep the status format of the web app (German keys).
        """
        ended = time.time()
        if outcome == "cancelled":
            reason = self._cancel_reason or "abgebrochen"
        elif outcome == "error":
            reason = self._error or "unbekannter Fehler"
        elif outcome == "alarm":
            reason = self._help_reason or (self._verdict or {}).get("text")
        else:
            reason = (self._verdict or {}).get("text")
        assessment = {
            name: {
                "state": c["zustand"],
                "flagged": c["zustand"] == am.FLAGGED,
                "value": c["wert"],
                "unit": c.get("einheit"),
                "threshold": c.get("grenze"),
                "text": c["text"],
            }
            for name, c in self._checks.items()
        }
        return {
            "kind": "health_check",
            "started_at": self._started_at,
            "ended_at": ended,
            "duration_s": None if self._started_at is None else round(ended - self._started_at, 2),
            "source": self._source,
            "target": None if self._target is None else {"x": self._target["x"], "y": self._target["y"]},
            "target_frame": None if self._target is None else self._target["rahmen"],
            "outcome": outcome,
            "state": self._state,
            "verdict": None if self._verdict is None else dict(self._verdict),
            "reason": reason,
            "alarm": self._alarm,
            "timeline": [dict(t) for t in self._timeline],
            "checks": [dict(self._checks[n]) for n in am.SIGNS],
            "assessment": assessment,
            "thresholds": _thresholds(),
            "answers_heard": [dict(a, texte=list(a["texte"])) for a in self._answers],
            "follow_up_interpretation": self._interpretation,
            "nudges": [dict(n, gehoert=list(n.get("gehoert") or [])) for n in self._nudges],
            "nudge_active": config.HEALTH_NUDGE_ACTIVE,
            "spoken": [dict(s) for s in self._spoken],
            "music": dict(self._music),
            "person_last": self._person_summary(),
        }

    def _finished(self, task: asyncio.Task) -> None:
        if self._on_finished is None:
            return
        try:
            result = self._on_finished(self._record(self._outcome(task)))
            if inspect.isawaitable(result):
                asyncio.ensure_future(result).add_done_callback(_log_callback_error)
        except Exception:  # the store must never break the healthbot
            log.exception("HEALTHBOT: on_finished failed")

    # ------------------------------------------------------------------ helpers

    def _drive(self, vx: float = 0.0, omega: float = 0.0, margin_m: float | None = None) -> DriveCommand:
        return self._watchdog.submit(DriveCommand(vx=vx, omega=omega), source="autonom", margin_m=margin_m)

    def _points(self):
        """Current lidar scan as points, or None (no/old scan)."""
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or time.monotonic() - scan.timestamp > _SCAN_MAX_AGE_S:
            return None
        return scan.points()

    def _seen(self) -> dict | None:
        """The person under check: tracked from frame to frame, so a second
        person walking past does not take over."""
        people = self._pose.people() if self._pose is not None else []
        return am.pick(people, self._person)

    def _distance(self, person: dict | None, leg_angle: float | None = None) -> float | None:
        """Lidar distance to the person (nearest point of their piece),
        otherwise the nearest thing straight ahead. With ``leg_angle`` the
        distance to the legs in a narrow window."""
        points = self._points()
        if points is None:
            return None
        if leg_angle is not None:
            # only the nearest point in the window: the piece would grow over
            # the whole body and give the distance to its nearest part
            p = np.asarray(points, dtype=float)
            w = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
            near = p[np.abs((w - leg_angle + 180.0) % 360.0 - 180.0) <= config.HEALTH_LEG_HALF_DEG]
            if len(near):
                return float(np.min(np.hypot(near[:, 0], near[:, 1])))
        if person is not None:
            half = (person["box"][2] - person["box"][0]) * self._pose.fov_deg / 2
            piece = am.lidar_piece(points, person["winkel"], half)
            if piece is not None:
                return piece[0]
        scan = self._lidar.latest()
        d = scan.min_distance(0.0, 15.0) if scan is not None else math.inf
        return d if math.isfinite(d) else None

    def _corridor(self):
        """Breadcrumb corridor for drives by map, None without map control."""
        if self.corridor is None:
            return None
        try:
            return self.corridor()
        except Exception:  # without the corridor it still plans on the map alone
            log.exception("HEALTHBOT: corridor not available")
            return None

    def _by_map(self) -> bool:
        return self._target is not None and self._target["rahmen"] == "karte"

    def _frame_pose(self) -> tuple[float, float, float] | None:
        """Pose (x, y, radians) in the frame of the target."""
        if self._by_map():
            return self._homing.map_pose()
        x, y, deg = self._odometry.pose()
        return x, y, math.radians(deg)

    def _in_area(self, person: dict) -> bool:
        """Is the person within HEALTH_AREA_RADIUS_M of the point on the map?

        Their position comes from the lidar distance in their direction.
        Without a map point, pose or lidar distance it cannot tell and lets
        them count, as before.
        """
        if not self._by_map():
            return True
        pose = self._frame_pose()
        d = self._distance(person)
        if pose is None or d is None:
            return True
        a = pose[2] + math.radians(person["winkel"])
        px, py = pose[0] + d * math.cos(a), pose[1] + d * math.sin(a)
        return math.hypot(px - self._target["x"], py - self._target["y"]) <= config.HEALTH_AREA_RADIUS_M

    def _part_angle(self, person: dict | None, indices) -> float | None:
        assert self._pose is not None
        return am.part_angle(person, indices, self._pose.fov_deg)

    def _leg_angle(self, person: dict | None) -> float | None:
        if not config.HEALTH_NUDGE_AT_LEGS:
            return None
        return self._part_angle(person, am.LEG_POINTS)

    async def _aim(self, indices, duration: float) -> float | None:
        """Turn until the body part (keypoints) is centred in the image.

        Returns its angle at the end, None if it was not seen.
        """
        deadline = time.monotonic() + duration
        angle = None
        while time.monotonic() < deadline:
            p = self._seen()
            if p is not None:
                self._person = p
            angle = self._part_angle(p, indices)
            if angle is None or abs(angle) <= config.HEALTH_CENTERED_DEG:
                break
            limit = config.HEALTH_SEARCH_OMEGA
            self._drive(omega=max(-limit, min(limit, math.radians(angle) * 1.5)))
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()
        return angle

    def _travelled(self, start: tuple[float, float, float]) -> float:
        x, y, _ = self._odometry.pose()
        return math.hypot(x - start[0], y - start[1])

    def _microphone(self):
        s = self._speech
        if s is None or not getattr(s, "available", False) or not hasattr(s, "heard_since"):
            return None
        return s

    def _heard_since(self, since: float) -> list[str] | None:
        """What was heard since ``since`` (unix time); None without microphone."""
        s = self._microphone()
        return None if s is None else list(s.heard_since(since))

    async def _speak(self, text: str, priority: bool = False) -> None:
        """Say it and wait until it is said -- otherwise the microphone hears
        the robot itself while waiting for the answer."""
        v = self._voice
        if v is None or not getattr(v, "available", False):
            self._spoken.append({"zeit": time.time(), "text": text, "gesagt": False})
            self._note(f"(kein Lautsprecher) „{text}“")
            return
        v.say(text, priority=priority)
        self._spoken.append({"zeit": time.time(), "text": text, "gesagt": True})
        self._note(f"sagt: „{text}“")
        deadline = time.monotonic() + _SPEAK_MAX_S
        await asyncio.sleep(0.3)  # until the output really starts
        while v.speaking and time.monotonic() < deadline:
            await asyncio.sleep(_TICK_S)

    async def _listen(self, duration: float, phase: str) -> list[str] | None:
        """What is heard within ``duration`` seconds; None without microphone."""
        if self._microphone() is None:
            return None
        since = time.time()
        await asyncio.sleep(duration)
        heard = self._heard_since(since)
        self._answers.append({"phase": phase, "zeit": since, "texte": list(heard or [])})
        return heard

    async def _observe(self, duration: float) -> list[dict]:
        """Collect frames of the person for ``duration`` seconds. The robot
        stands still: its own motion would otherwise look like the person moving."""
        self._watchdog.stop()
        history: list[dict] = []
        last = None
        end = time.monotonic() + duration
        self._camera_still = True
        try:
            while time.monotonic() < end:
                p = self._seen()
                if p is not None and p.get("zeit") != last:
                    last = p.get("zeit")
                    history.append(p)
                    self._person = p
                await asyncio.sleep(_TICK_S)
        finally:
            self._camera_still = False
        return history

    async def _turn_by(self, degrees: float) -> None:
        """Turn by ``degrees`` (left positive), by odometry with a time limit."""
        if abs(degrees) < 10.0:
            return
        _, _, start = self._odometry.pose()
        omega = math.copysign(config.HEALTH_SEARCH_OMEGA * 1.5, degrees)
        deadline = time.monotonic() + min(_TURN_MAX_S, abs(math.radians(degrees) / omega) * 1.5 + 1.0)
        while time.monotonic() < deadline:
            _, _, now = self._odometry.pose()
            turned = (now - start + 180.0) % 360.0 - 180.0
            if abs(turned) >= abs(degrees) - 5.0:
                break
            self._drive(omega=omega)
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()

    async def _search_person(self) -> dict | None:
        """Turn until a person is centred in the image. None after one round."""
        deadline = time.monotonic() + config.HEALTH_SEARCH_S
        while time.monotonic() < deadline:
            people = self._pose.people() if self._pose is not None else []
            # with a map point only who is in that area: someone standing
            # elsewhere in the room is turned past
            p = am.pick([q for q in people if self._in_area(q)], self._person)
            if p is not None:
                self._person = p
                if abs(p["winkel"]) <= config.HEALTH_CENTERED_DEG:
                    self._watchdog.stop()
                    return p
                # swing in proportionally; limited so they do not slip out of the image
                omega = max(-config.HEALTH_SEARCH_OMEGA, min(config.HEALTH_SEARCH_OMEGA, math.radians(p["winkel"]) * 1.5))
                self._drive(omega=omega)
            else:
                self._drive(omega=config.HEALTH_SEARCH_OMEGA)
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()
        return None

    async def _home(self) -> None:
        # the check is over, nobody needs the skeleton on the way back
        self._use_pose(False)
        self._note("fährt zum Startpunkt zurück")
        origin = self._origin
        if self._by_map() and getattr(self._homing, "map_ready", False):
            # by map back to where the check started: dead reckoning over a
            # longer drive through rooms would drift
            if origin is not None:
                started = self._homing.start(target=origin[:2], angle=origin[2],
                                             corridor=self._corridor())
            else:
                started = self._homing.start(corridor=self._corridor())
        else:
            x, y = origin[:2] if origin is not None else (0.0, 0.0)
            started = self._homing.start_to(x, y, name="Startpunkt")
        if not started:
            self._note("Heimfahrt startet nicht (läuft schon eine?)")
            return
        while self._homing.active:
            await asyncio.sleep(0.2)
        self._note(f"Heimfahrt: {self._homing.last_reason or 'beendet'}")

    # ------------------------------------------------------------------ procedure

    async def _procedure(self) -> None:
        self._use_pose(True)
        try:
            await self._procedure_inner()
        except asyncio.CancelledError:
            raise
        except Exception as e:  # never leave a driving robot behind
            log.exception("HEALTHBOT: error in the procedure")
            self._error = f"{type(e).__name__}: {e}"
            self._watchdog.stop()
            self._note(f"Fehler: {e}", CANCELLED)
            await self._send("warning", f"Healthbot: Fehler ({e}) -- bitte selbst nachsehen")
        finally:
            self._watchdog.stop()
            self._use_pose(False)

    async def _procedure_inner(self) -> None:
        # 1. approach
        if self._target is not None:
            self._note(f"fährt zur gemeldeten Stelle ({self._target['x']:.2f}, {self._target['y']:.2f})", APPROACH)
            await self._send("info", "Healthbot: fahre zur gemeldeten Stelle")
            if self._by_map():
                started = self._homing.start(
                    target=(self._target["x"], self._target["y"]),
                    corridor=self._corridor(),
                    blocked_enough_m=config.HEALTH_BLOCKED_ENOUGH_M,
                )
            else:
                started = self._homing.start_to(
                    self._target["x"], self._target["y"], tolerance=config.HEALTH_GOAL_TOLERANCE_M,
                    name="gemeldete Stelle", blocked_enough_m=config.HEALTH_BLOCKED_ENOUGH_M,
                )
            if not started:
                self._note(f"Anfahrt startet nicht: {self._homing.last_reason or 'läuft schon eine Fahrt?'}")
            while self._homing.active:
                await asyncio.sleep(0.2)
            self._note(f"Anfahrt: {self._homing.last_reason or 'beendet'}")
            # turn towards the spot: the person lies there, not in the
            # direction of the last evasive manoeuvre
            pose = self._frame_pose()
            if pose is not None and math.hypot(self._target["x"] - pose[0], self._target["y"] - pose[1]) > 0.2:
                bearing = math.atan2(self._target["y"] - pose[1], self._target["x"] - pose[0])
                await self._turn_by((math.degrees(bearing - pose[2]) + 180.0) % 360.0 - 180.0)

        # 2. overview
        self._note("sucht die Person", OVERVIEW)
        person = await self._search_person()
        if person is None:
            self._note("keine Person gefunden", NO_PERSON)
            self._verdict = {"stufe": "unbekannt", "punkte": 0, "text": "keine Person gefunden"}
            await self._send("warning", "Healthbot: Keine Person gefunden – bitte selbst nachsehen")
            await self._home()
            return
        self._note(f"Person gefunden ({person['winkel']:+.0f}°), beobachtet {config.HEALTH_OBSERVE_S:.0f} s")
        history = await self._observe(config.HEALTH_OBSERVE_S)
        person = history[-1] if history else self._seen()
        # all frames vote: one frame with swapped keypoints must not decide
        self._check(am.lying_over(history, person))
        assert self._pose is not None  # start() requires pose detection
        self._check(am.floor(person, self._points(), self._pose.fov_deg))
        self._check(am.motionless(history))

        # 3. address
        self._note("spricht die Person an", ADDRESS)
        # face centred by turning, the tilt servo does the height: eye contact
        if await self._aim(am.HEAD_POINTS, config.HEALTH_FACE_S) is not None:
            self._note("schaut der Person ins Gesicht")
        await self._speak(config.HEALTH_QUESTION)
        heard = await self._listen(config.HEALTH_ANSWER_S, "question")
        self._check(am.answer(heard))
        self._verdict = am.overall_verdict(self._checks)

        if heard:
            await self._follow_up()
            return
        position = any(self._checks[n]["zustand"] == am.FLAGGED for n in ("liegend", "boden"))
        if not position:
            # whoever stands or sits is not nudged, even without an answer
            # (headphones, no interest, microphone misheard)
            self._note("liegt nicht -- Person scheint wohlauf", UNREMARKABLE)
            await self._send("info", "Healthbot: keine Auffälligkeit")
            await self._home()
            return

        # 4. nudge
        if config.HEALTH_NUDGE_ACTIVE:
            reaction = await self._nudge_all()
            self._verdict = am.overall_verdict(self._checks, reaction)
            if reaction:
                await self._follow_up()
                return
        else:
            self._note("Anstupsen abgeschaltet (config.HEALTH_NUDGE_ACTIVE)")
        await self._help()

    async def _follow_up(self) -> None:
        """The person reacts -- but do they need help?"""
        self._note("Person reagiert, fragt nach", FOLLOW_UP)
        await self._speak(config.HEALTH_QUESTION_HELP)
        heard = await self._listen(config.HEALTH_ANSWER_S, "follow_up")
        interpretation = am.interpret_answer(heard or [])
        self._interpretation = interpretation
        if heard:
            self._note(f"hört: „{' / '.join(heard)}“ -> {interpretation or 'unklar'}")
        if interpretation == "nein":
            self._note("Person ansprechbar", RESPONSIVE)
            self._verdict = {"stufe": "ansprechbar", "punkte": am.count(self._checks), "text": "Person ansprechbar, braucht keine Hilfe"}
            await self._send("info", "Healthbot: Person ansprechbar")
            await self._speak(config.HEALTH_ANNOUNCE_OK)
            await self._home()
            return
        # yes, help -- or no/unclear answer: when in doubt get help; a false
        # alarm costs a glance, a missed emergency more
        reason = "bittet um Hilfe" if interpretation == "hilfe" else "keine klare Antwort auf die Nachfrage"
        self._verdict = {"stufe": "notfall", "punkte": am.count(self._checks), "text": reason}
        await self._help(reason)

    async def _nudge_all(self) -> bool:
        """Nudge up to HEALTH_NUDGE_COUNT times. True on a reaction."""
        self._note("fährt heran, um anzustupsen", NUDGE)
        # say what is coming: a person lying there should not be startled by
        # a silent robot touching them
        await self._speak(config.HEALTH_NUDGE_ANNOUNCE)
        if not await self._approach():
            self._note("kommt nicht nah genug heran -- kein Stupsen möglich")
            return False
        for nr in range(1, config.HEALTH_NUDGE_COUNT + 1):
            if config.HEALTH_NUDGE_AT_LEGS:
                legs = await self._aim(am.LEG_POINTS, config.HEALTH_FACE_S)
                self._note("zielt auf die Beine" if legs is not None
                           else "Beine nicht zu sehen -- stupst die Körpermitte an")
            # call first, then nudge: the ear is deaf while the robot speaks,
            # so only what comes after the call counts as an answer
            await self._speak(config.HEALTH_NUDGE_CALL)
            since = time.time()
            result = await self._nudge(nr)
            await self._back_off()
            history = await self._observe(_REACTION_S)
            moved = am.motionless(history)
            heard = self._heard_since(since)
            if heard is not None:
                self._answers.append({"phase": f"nudge_{nr}", "zeit": since, "texte": list(heard)})
            heard = heard or []
            reaction = moved["zustand"] == am.OK or bool(heard)
            result.update({"reaktion": reaction, "bewegung": moved["wert"], "gehoert": heard})
            self._nudges.append(result)
            if reaction:
                what = f"„{' / '.join(heard)}“" if heard else f"Bewegung {moved['wert']} px"
                self._note(f"Stups {nr}: Reaktion ({what})")
                self._check(am.motionless(history))
                if heard:
                    self._check(am.answer(heard))
                return True
            self._note(f"Stups {nr}: keine Reaktion")
        return False

    async def _approach(self) -> bool:
        """Slowly up to HEALTH_NUDGE_START_M (lidar) to the person, keeping
        them centred. The normal drive guard stays on.

        a person lying on the floor is below the lidar plane, so the lidar
        distance is often the wall behind them: then only the drive guard
        (depth camera, memory) or the wheels stopping at the body end the
        approach. limited by distance and a generous time, not by a short
        timeout (robot test: gave up after 15 s while still on its way)
        """
        deadline = time.monotonic() + config.HEALTH_APPROACH_MAX_S
        start = self._odometry.pose()
        began = time.monotonic()
        still_since: float | None = None
        while time.monotonic() < deadline:
            if self._travelled(start) > config.HEALTH_APPROACH_MAX_M:
                self._watchdog.stop()
                self._note(f"nach {config.HEALTH_APPROACH_MAX_M:.1f} m noch nicht bei der Person -- bricht ab")
                return False
            p = self._seen()
            if p is not None:
                self._person = p
            legs = self._leg_angle(p)
            d = self._distance(p, legs)
            if d is None:
                self._watchdog.stop()
                self._note("kein Lidar-Abstand zur Person")
                return False
            if d <= config.HEALTH_NUDGE_START_M:
                self._watchdog.stop()
                return True
            limit = config.HEALTH_APPROACH_OMEGA_MAX
            # towards the legs when they are seen: that is where it nudges
            aim = legs if legs is not None else (None if p is None else p["winkel"])
            omega = 0.0 if aim is None else max(-limit, min(limit, math.radians(aim)))
            driven = self._drive(vx=config.HEALTH_APPROACH_SPEED, omega=omega)
            enough = config.HEALTH_NUDGE_START_M + config.HEALTH_APPROACH_ENOUGH_M
            if driven.vx <= 0.0 and self._watchdog.last_check is not None and d <= enough:
                # drive guard holds just short of the goal: close enough to nudge
                self._watchdog.stop()
                return True
            if driven.vx <= 0.0 and self._holds_close():
                # person lies lower than the lidar plane, so d measures the wall
                # behind; depth and memory see them and the guard holds just in
                # front (test log: only stopped, never nudged)
                self._watchdog.stop()
                self._note("Fahrschutz hält knapp vor der Person (Tiefe/Gedächtnis) -- nah genug zum Stupsen")
                return True
            # wheels stand although it drives: it touches the person already
            vx_actual = self._odometry.velocity()[0]
            if time.monotonic() - began > 1.0 and driven.vx > 0.0 and vx_actual < config.HEALTH_NUDGE_STILL_MS:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= config.HEALTH_NUDGE_CONTACT_S:
                    self._watchdog.stop()
                    self._note("Räder stehen an der Person -- nah genug zum Stupsen")
                    return True
            else:
                still_since = None
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()
        return False

    def _holds_close(self) -> bool:
        """Did the drive guard hold because of an obstacle just ahead?
        Missing lidar data does not count: then it knows nothing about the person."""
        c = self._watchdog.last_check
        reason = getattr(c, "reason", None)
        free = getattr(c, "free_m", None)
        return reason not in (None, drive_guard.NO_LIDAR) and free is not None and free <= config.HEALTH_APPROACH_ENOUGH_M

    async def _nudge(self, nr: int) -> dict:
        """One nudge: slowly forward until contact or distance limit.

        Distance limit = air to the person + HEALTH_NUDGE_DEPTH_M, measured
        by odometry -- so it pushes at most ~10 cm further than lidar or drive
        guard see air. Contact = the wheels stop turning although it should
        drive; if the drive guard holds first, the nudge ends with GUARD_HOLDS.
        """
        p = self._seen()
        d = self._distance(p, self._leg_angle(p))
        air = max(0.0, (d if d is not None else config.ROBOT_FRONT_M) - config.ROBOT_FRONT_M)
        # more air than _approach allows means the lidar looks over the
        # person at the wall; then do not trust the lidar value
        air = min(air, config.HEALTH_NUDGE_START_M + config.HEALTH_APPROACH_ENOUGH_M - config.ROBOT_FRONT_M)
        limit = air + config.HEALTH_NUDGE_DEPTH_M
        self._note(f"Stups {nr}: {air:.2f} m Luft, höchstens {limit:.2f} m vor")
        start = self._odometry.pose()
        began = time.monotonic()
        still_since: float | None = None
        reason = "Zeitgrenze"
        while time.monotonic() - began < config.HEALTH_NUDGE_MAX_S:
            if self._travelled(start) >= limit:
                reason = "Weggrenze"
                break
            driven = self._drive(vx=config.HEALTH_NUDGE_SPEED, margin_m=config.HEALTH_NUDGE_MARGIN_M)
            # the drive guard also sees what lies below the lidar plane: its
            # distance to contact limits the nudge depth as well
            free = getattr(self._watchdog.last_check, "free_m", None)
            if free is not None:
                limit = min(limit, self._travelled(start) + free + config.HEALTH_NUDGE_DEPTH_M)
            vx_actual = self._odometry.velocity()[0]
            # only watch for standstill after starting off (ramp)
            still = time.monotonic() - began > 0.6 and (driven.vx <= 0.0 or vx_actual < config.HEALTH_NUDGE_STILL_MS)
            if still:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= config.HEALTH_NUDGE_CONTACT_S:
                    if driven.vx > 0.0:
                        reason = "Kontakt"
                    elif getattr(self._watchdog.last_check, "reason", None) == drive_guard.NO_LIDAR:
                        # held because the lidar dropped out, not because of
                        # the person; the log should say so
                        reason = drive_guard.NO_LIDAR
                    else:
                        reason = GUARD_HOLDS
                    break
            else:
                still_since = None
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()
        distance = self._travelled(start)
        self._note(f"Stups {nr}: {reason} nach {distance:.2f} m")
        return {"nr": nr, "grund": reason, "weg_m": round(distance, 3), "zeit": time.time()}

    async def _back_off(self) -> None:
        """Back off at once so the person has room (rear has a drive guard too)."""
        start = self._odometry.pose()
        deadline = time.monotonic() + _BACK_MAX_S
        while time.monotonic() < deadline and self._travelled(start) < config.HEALTH_NUDGE_BACK_M:
            self._drive(vx=-config.HEALTH_APPROACH_SPEED)
            await asyncio.sleep(_TICK_S)
        self._watchdog.stop()

    async def _help(self, reason: str = "liegt regungslos und reagiert nicht") -> None:
        """Get help: alarm in the app, announcement, drive home, music."""
        self._alarm = True
        self._help_reason = reason
        self._note(f"HILFE: {reason}", HELP)
        if self._verdict is None or self._verdict.get("stufe") != "notfall":
            self._verdict = {"stufe": "notfall", "punkte": am.count(self._checks), "text": reason}
        place = ""
        if self._target is not None:
            place = f" bei ({self._target['x']:.1f}, {self._target['y']:.1f})"
        await self._send("emergency", f"Healthbot: Person{place} braucht Hilfe – {reason}")
        await self._speak(config.HEALTH_ANNOUNCE_HELP, priority=True)
        await self._home()
        # music even if the homing drive did not fully work: it should call
        # people over wherever the robot is
        await self._play_music()
        self._step = "Hilfe gerufen"

    async def _play_music(self) -> None:
        if self._mukke is None:
            self._music["error"] = "keine Musik (Lautsprecher fehlt)"
            self._note("keine Musik (Lautsprecher fehlt)")
            return
        from ..audio.mukke import MukkeError

        path = os.path.join(_MUSIC_DIR, config.HEALTH_MUSIC_FILE)
        # set before starting: a stop arriving while mpv is still starting
        # should still end the music
        self._music_started = True
        try:
            await asyncio.to_thread(self._mukke.play_file, path, config.HEALTH_MUSIC_TITLE)
            self._music["played"] = True
            self._note(f"spielt „{config.HEALTH_MUSIC_TITLE}“")
        except Exception as e:  # MukkeError or a broken player: the alarm stays an alarm
            if not isinstance(e, MukkeError):
                log.exception("HEALTHBOT: music failed")
            self._music_started = False
            self._music["error"] = str(e)
            self._note(f"Musik geht nicht: {e}")

    # ------------------------------------------------------------------ watch

    async def _watch(self) -> None:
        """Look out for lying people. Whoever is seen lying for HEALTH_AUTO_S
        gets checked; then the watch is off, otherwise the same person would
        trigger the next check right away."""
        # set_watch() only turns the watch on with pose detection
        assert self._pose is not None
        self._pose.activate(True, "healthbot-wache")
        self._watch_since = None
        watch = am.LyingWatch()
        seen = None
        try:
            while True:
                await asyncio.sleep(0.2)
                if self.active:
                    watch.reset()
                    self._watch_since = None
                    continue
                people = self._pose.people()
                p = am.pick(people, seen)
                seen = p or seen
                flagged = p is not None and am.lying(p)["zustand"] == am.FLAGGED
                # short misses (a lost frame) are tolerated, see LyingWatch
                due = watch.update(time.monotonic(), flagged)
                self._watch_since = watch.since
                if not due or p is None:
                    continue
                d = self._distance(p)
                x, y, heading = self._odometry.pose()
                if d is None:
                    target = None  # without distance: check from here
                else:
                    w = math.radians(heading + p["winkel"])
                    target = (x + d * math.cos(w), y + d * math.sin(w))
                error = self.start(*(target or (None, None)), source="selbst erkannt")
                if error is None:
                    self._note("liegende Person selbst erkannt -- Wache aus")
                    await self._send("warning", "Healthbot: liegende Person erkannt, prüfe")
                    return
                watch.reset()
                self._watch_since = None
        finally:
            self._pose.activate(False, "healthbot-wache")


def _thresholds() -> dict:
    """The config values a check depends on, for later review."""
    names = (
        "HEALTH_OBSERVE_S", "HEALTH_ANSWER_S", "HEALTH_LYING_DEG", "HEALTH_LYING_ASPECT",
        "HEALTH_LYING_SHARE", "HEALTH_FORESHORTEN_RATIO",
        "HEALTH_MOTIONLESS_PX", "HEALTH_MOTIONLESS_REF_DIAG_PX",
        "HEALTH_FLOOR_MIN_WIDTH_M", "HEALTH_FLOOR_IMAGE_BOTTOM",
        "HEALTH_NUDGE_ACTIVE", "HEALTH_NUDGE_COUNT", "HEALTH_NUDGE_SPEED", "HEALTH_NUDGE_DEPTH_M",
        "HEALTH_NUDGE_MARGIN_M", "HEALTH_NUDGE_START_M", "HEALTH_NUDGE_CONTACT_S",
        "HEALTH_NUDGE_STILL_MS", "HEALTH_NUDGE_MAX_S", "HEALTH_NUDGE_BACK_M",
        "HEALTH_APPROACH_SPEED", "HEALTH_APPROACH_ENOUGH_M", "HEALTH_GOAL_TOLERANCE_M",
        "HEALTH_BLOCKED_ENOUGH_M", "HEALTH_SEARCH_S", "HEALTH_CENTERED_DEG", "POSE_KP_MIN_CONF",
    )
    return {n: getattr(config, n, None) for n in names}


def _log_callback_error(future: asyncio.Future) -> None:
    if future.cancelled():
        return
    if future.exception() is not None:
        log.error("HEALTHBOT: on_finished failed", exc_info=future.exception())
