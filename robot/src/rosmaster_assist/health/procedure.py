"""Healthbot procedure: drive there, overview, address, nudge, get help.

States (``status()["zustand"]``)::

    bereit -> anfahrt -> gesamtbild -> ansprechen -+-> nachfrage --> hilfe | ansprechbar
                                                   +-> anstupsen -+     |
                                                   +-> unauffaellig     v
                                                       ansprechbar   hilfe_suchen -> hinfuehren -> hilfe

In the overview it looks for someone lying or slumped first, also from
viewpoints around the reported spot (behind a table); an upright person is
only checked when nobody lies there. After the alarm it looks for an
upright helper in the mapped area and leads them to the person.

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
from . import search

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
SEEK_HELP = "hilfe_suchen"
LEAD = "hinfuehren"
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
        # drive connection lost: since when (monotonic), total paused time and
        # whether a homing drive was cut off by it
        self._drive_lost_at: float | None = None
        self._paused_s = 0.0
        self._lost_during = False
        self._loss_timer: asyncio.Task | None = None
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
        self._wellbeing: str | None = None
        self._posture: str | None = None
        self._viewpoints: list[dict] = []
        self._helper: dict | None = None
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
        """For the /map page: the point under check, its area and the drive
        there; the person on the floor, viewpoints, helper search points and
        the people the camera sees, all in the map frame."""
        by_map = self._by_map()
        helper = self._helper if by_map and self._helper is not None else {}
        return {
            "aktiv": self.active,
            "zustand": self._state,
            "schritt": self._step,
            "ziel": [self._target["x"], self._target["y"]] if by_map else None,
            "radius_m": config.HEALTH_AREA_RADIUS_M,
            "fahrt": self._homing.state() if by_map and self.active else None,
            "opfer": helper.get("opfer"),
            "suchpunkte": list(helper.get("suchpunkte", [])),
            "suchpunkt": helper.get("punkt"),
            "blickpunkte": [[v["x"], v["y"], v["erreicht"]] for v in self._viewpoints] if by_map else [],
            "personen": self._people_on_map() if by_map else [],
        }

    def _people_on_map(self) -> list[dict]:
        """People the camera sees, placed by camera direction and lidar distance."""
        if not (self.active and self._pose_on and self._pose is not None):
            return []
        out = []
        try:
            for person in self._pose.people()[:6]:
                place = self._place_of(person)
                if place is not None:
                    out.append({"x": round(place[0], 2), "y": round(place[1], 2),
                                "haltung": am.posture(person), "liegt": bool(am.down(person))})
        except Exception:  # noqa: BLE001 -- the map view must never break the status
            log.debug("people on the map failed", exc_info=True)
        return out

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
        self._end_loss()
        self._paused_s = 0.0
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

    def drive_lost(self, reason: str = "Verbindung zum Antrieb verloren") -> None:
        """Drive connection gone (usb reset): stop and wait for it to come back.

        A short dropout used to end the whole check. Now it pauses and goes
        on after ``drive_back()``; if the drive stays away longer than
        HEALTH_DRIVE_RECONNECT_S, the check is cancelled.
        """
        if not self.active:
            return
        self._watchdog.stop()
        if self._drive_lost_at is not None:
            return
        self._drive_lost_at = time.monotonic()
        if self._homing.active:
            self._lost_during = True
            self._homing.cancel(reason)
        self._note(f"{reason} -- wartet bis zu {config.HEALTH_DRIVE_RECONNECT_S:.0f} s")
        self._loss_timer = asyncio.create_task(self._loss_deadline(reason))

    def drive_back(self) -> None:
        """Drive connection is back: a paused check goes on."""
        if self._drive_lost_at is None:
            return
        self._paused_s += time.monotonic() - self._drive_lost_at
        self._drive_lost_at = None
        if self._loss_timer is not None:
            self._loss_timer.cancel()
            self._loss_timer = None
        if self.active:
            self._note("Antrieb wieder verbunden -- macht weiter")

    async def _loss_deadline(self, reason: str) -> None:
        await asyncio.sleep(config.HEALTH_DRIVE_RECONNECT_S)
        if self._drive_lost_at is not None:
            self._loss_timer = None
            self._drive_lost_at = None
            self.cancel(f"{reason}, nach {config.HEALTH_DRIVE_RECONNECT_S:.0f} s nicht zurück")

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
            "wellbeing_interpretation": self._wellbeing,
            "posture": self._posture,
            "viewpoints": [dict(v) for v in self._viewpoints],
            "helper": None if self._helper is None else dict(self._helper),
            "nudges": [dict(n, gehoert=list(n.get("gehoert") or [])) for n in self._nudges],
            "nudge_active": config.HEALTH_NUDGE_ACTIVE,
            "spoken": [dict(s) for s in self._spoken],
            "music": dict(self._music),
            "person_last": self._person_summary(),
        }

    def _end_loss(self) -> None:
        # a timer left over from an earlier check must not cancel the next one
        self._drive_lost_at = None
        if self._loss_timer is not None:
            self._loss_timer.cancel()
            self._loss_timer = None

    def _finished(self, task: asyncio.Task) -> None:
        self._end_loss()
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
        if self._drive_lost_at is not None:
            return self._watchdog.submit(DriveCommand(), source="autonom")
        return self._watchdog.submit(DriveCommand(vx=vx, omega=omega), source="autonom", margin_m=margin_m)

    def _now(self) -> float:
        """Clock for drive time limits: stands still while the drive is lost."""
        paused = self._paused_s
        if self._drive_lost_at is not None:
            paused += time.monotonic() - self._drive_lost_at
        return time.monotonic() - paused

    async def _wait_drive(self) -> None:
        while self._drive_lost_at is not None:
            await asyncio.sleep(0.2)

    async def _tick(self) -> None:
        """One step of a drive loop; waits while the drive is lost."""
        await asyncio.sleep(_TICK_S)
        if self._drive_lost_at is not None:
            self._watchdog.stop()
            await self._wait_drive()

    async def _homing_drive(self, start, watch=None, watch_reason: str = "") -> bool:
        """Run a homing drive begun by ``start()``; after a drive loss it starts
        again. False if it does not start (the reason is in last_reason).

        ``watch()`` is asked during the drive; a true answer cancels it, with
        the answer as reason if it is a text, otherwise ``watch_reason``.
        """
        while True:
            await self._wait_drive()
            self._lost_during = False
            if not start():
                return False
            while self._homing.active:
                await asyncio.sleep(0.2)
                said = watch() if watch is not None and self._homing.active else None
                if said:
                    self._homing.cancel(said if isinstance(said, str) else watch_reason)
                    while self._homing.active:
                        await asyncio.sleep(0.05)
                    return True
            if not self._lost_during:
                return True
            await self._wait_drive()
            self._note("setzt die Fahrt fort")

    def _points(self):
        """Current lidar scan as points, or None (no/old scan)."""
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or time.monotonic() - scan.timestamp > _SCAN_MAX_AGE_S:
            return None
        return scan.points()

    def _front_x(self) -> float | None:
        """Median distance straight ahead (lidar, robot frame x), None
        without a scan or with too few points."""
        points = self._points()
        if points is None:
            return None
        p = np.asarray(points, dtype=float)
        if p.ndim != 2 or len(p) == 0:
            return None
        p = p[np.all(np.isfinite(p), axis=1)]
        r = np.hypot(p[:, 0], p[:, 1])
        w = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
        ahead = p[(np.abs(w) <= config.HEALTH_SLIP_HALF_DEG) & (r >= 0.15) & (r <= config.HEALTH_SLIP_MAX_RANGE_M)]
        if len(ahead) < config.HEALTH_SLIP_MIN_POINTS:
            return None
        return float(np.median(ahead[:, 0]))

    def _slipping(self, slip: "_SlipCheck", start) -> bool:
        """One step of the slip check while driving forward."""
        return slip.update(self._now(), self._travelled(start), self._front_x(), self._odometry.pose()[2])

    def _seen(self) -> dict | None:
        """The person under check: tracked from frame to frame, so a second
        person walking past does not take over."""
        people = self._pose.people() if self._pose is not None else []
        return am.pick_patient(people, self._person)

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
        if pose is None:
            return True
        tx, ty, r = self._target["x"], self._target["y"], config.HEALTH_AREA_RADIUS_M
        a = pose[2] + math.radians(person["winkel"])
        dx, dy = tx - pose[0], ty - pose[1]
        if am.down(person) and math.hypot(dx, dy) > r:
            # someone on the floor lies below the lidar plane, the lidar
            # measures the table in front or the wall behind: from outside
            # the area it is enough that the line of sight passes through it
            # (inside the area every direction does, there the lidar decides)
            along = dx * math.cos(a) + dy * math.sin(a)
            across = abs(-dx * math.sin(a) + dy * math.cos(a))
            return along > 0.0 and across <= r
        d = self._distance(person)
        if d is None:
            return True
        px, py = pose[0] + d * math.cos(a), pose[1] + d * math.sin(a)
        return math.hypot(px - tx, py - ty) <= r

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
        deadline = self._now() + duration
        angle = None
        while self._now() < deadline:
            p = self._seen()
            if p is not None:
                self._person = p
            angle = self._part_angle(p, indices)
            if angle is None or abs(angle) <= config.HEALTH_CENTERED_DEG:
                break
            limit = config.HEALTH_SEARCH_OMEGA
            self._drive(omega=max(-limit, min(limit, math.radians(angle) * 1.5)))
            await self._tick()
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
        if v is None or not await self._wait_for(lambda: getattr(v, "available", False)):
            self._spoken.append({"zeit": time.time(), "text": text, "gesagt": False})
            self._note(f"(kein Lautsprecher) „{text}“")
            return
        # the speaker drops out with the usb resets: a sentence that did not
        # come out is said once more (robot test: nobody heard the question)
        for attempt in (1, 2):
            before = getattr(v, "said_count", None)
            ok = v.say(text, priority=priority or attempt > 1)
            self._note(f"sagt: „{text}“" if attempt == 1 else f"sagt nochmal: „{text}“")
            deadline = time.monotonic() + _SPEAK_MAX_S
            await asyncio.sleep(0.3)  # until the output really starts
            while v.speaking and time.monotonic() < deadline:
                await asyncio.sleep(_TICK_S)
            said = ok is not False and (before is None or v.said_count > before)
            if said or attempt == 2:
                break
            self._note("Satz kam nicht aus dem Lautsprecher")
            await self._wait_for(lambda: getattr(v, "available", False))
        self._spoken.append({"zeit": time.time(), "text": text, "gesagt": bool(said)})

    def _call_out(self, text: str) -> None:
        """Say it without waiting, for calls while driving: the voice queues
        it, the drive goes on (a sentence takes seconds, the dead man's
        switch only half a second)."""
        v = self._voice
        ok = v is not None and getattr(v, "available", False) and v.say(text, priority=True) is not False
        self._spoken.append({"zeit": time.time(), "text": text, "gesagt": bool(ok)})
        self._note(f"ruft: „{text}“" if ok else f"(kein Lautsprecher) „{text}“")

    async def _wait_for(self, ready) -> bool:
        """Wait at most HEALTH_AUDIO_WAIT_S until ``ready()``; the audio devices
        come back a few seconds after a usb reset."""
        deadline = time.monotonic() + config.HEALTH_AUDIO_WAIT_S
        while not ready():
            if time.monotonic() >= deadline:
                return False
            await asyncio.sleep(0.2)
        return True

    async def _listen(self, duration: float, phase: str) -> list[str] | None:
        """What is heard within ``duration`` seconds; None without microphone."""
        if not await self._wait_for(lambda: self._microphone() is not None):
            self._note("(kein Mikrofon) hört nichts")
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
        end = self._now() + duration
        self._camera_still = True
        try:
            while self._now() < end:
                p = self._seen()
                if p is not None and p.get("zeit") != last:
                    last = p.get("zeit")
                    history.append(p)
                    self._person = p
                await self._tick()
        finally:
            self._camera_still = False
        return history

    async def _turn_by(self, degrees: float) -> None:
        """Turn by ``degrees`` (left positive), by odometry with a time limit."""
        if abs(degrees) < 10.0:
            return
        _, _, start = self._odometry.pose()
        omega = math.copysign(config.HEALTH_SEARCH_OMEGA * 1.5, degrees)
        deadline = self._now() + min(_TURN_MAX_S, abs(math.radians(degrees) / omega) * 1.5 + 1.0)
        while self._now() < deadline:
            _, _, now = self._odometry.pose()
            turned = (now - start + 180.0) % 360.0 - 180.0
            if abs(turned) >= abs(degrees) - 5.0:
                break
            self._drive(omega=omega)
            await self._tick()
        self._watchdog.stop()

    async def _search_person(self, duration: float | None = None, down_only: bool = False) -> dict | None:
        """Turn until a person is centred in the image. None after ``duration``.

        ``down_only``: only someone lying or slumped counts, upright people
        are turned past (robot test: it addressed someone sitting in front of
        the person on the floor). Once it centres someone it stays with them,
        even when one frame calls them sitting.
        """
        deadline = self._now() + (config.HEALTH_SEARCH_S if duration is None else duration)
        locked: dict | None = None
        missing = 0
        while self._now() < deadline:
            people = self._pose.people() if self._pose is not None else []
            # with a map point only who is in that area: someone standing
            # elsewhere in the room is turned past
            near = [q for q in people if self._in_area(q)]
            if locked is not None:
                again = am.track(near, locked) if near else None
                missing = 0 if again is not None else missing + 1
                locked = again or (locked if missing * _TICK_S < 1.0 else None)
                p = again
            else:
                p = None
            if locked is None:
                if down_only:
                    downs = [q for q in near if am.down(q)]
                    p = am.pick_patient(downs, None)
                else:
                    p = am.pick_patient(near, None)
                locked = p
            if p is not None:
                self._person = p
                if abs(p["winkel"]) <= config.HEALTH_CENTERED_DEG:
                    self._watchdog.stop()
                    return p
                # swing in proportionally; limited so they do not slip out of the image
                omega = max(-config.HEALTH_SEARCH_OMEGA, min(config.HEALTH_SEARCH_OMEGA, math.radians(p["winkel"]) * 1.5))
                self._drive(omega=omega)
            elif locked is not None:
                self._watchdog.stop()  # lost for a moment: hold still and look again
            else:
                self._drive(omega=config.HEALTH_SEARCH_OMEGA)
            await self._tick()
        self._watchdog.stop()
        return None

    async def _face_target(self) -> None:
        """Turn towards the reported spot: the person lies there, not in the
        direction of the last evasive manoeuvre."""
        if self._target is None:
            return
        pose = self._frame_pose()
        if pose is not None and math.hypot(self._target["x"] - pose[0], self._target["y"] - pose[1]) > 0.2:
            bearing = math.atan2(self._target["y"] - pose[1], self._target["x"] - pose[0])
            await self._turn_by((math.degrees(bearing - pose[2]) + 180.0) % 360.0 - 180.0)

    def _spot_watch(self):
        """Watch for the drive to the reported spot: ends it
        HEALTH_SPOT_STANDOFF_M before the spot, or once someone lying in its
        area is confirmed within HEALTH_SPOT_WATCH_M. The spot is where the
        person lies; driving onto it runs into them."""
        tx, ty = self._target["x"], self._target["y"]
        seen = {"good": 0}

        def watch() -> str | None:
            pose = self._frame_pose()
            if pose is None:
                return None
            left = math.hypot(tx - pose[0], ty - pose[1])
            if left <= config.HEALTH_SPOT_STANDOFF_M:
                return "kurz vor der Stelle"
            people = self._pose.people() if self._pose is not None and left <= config.HEALTH_SPOT_WATCH_M else []
            if any(am.down(q) and self._in_area(q) for q in people):
                seen["good"] += 1
            else:
                seen["good"] = 0
            return "liegende Person in Sicht" if seen["good"] >= config.HEALTH_SPOT_CONFIRM_FRAMES else None

        return watch

    def _drive_to(self, x: float, y: float, name: str, angle: float | None = None,
                  speed: float | None = None, blocked_enough_m: float = 0.0):
        """A start function for ``_homing_drive`` in the frame of the check."""
        speed = config.HEALTH_DRIVE_SPEED if speed is None else speed
        if self._by_map():
            return lambda: self._homing.start(target=(x, y), angle=angle, corridor=self._corridor(),
                                              blocked_enough_m=blocked_enough_m, speed=speed,
                                              goal_snap_m=config.HEALTH_GOAL_SNAP_M)
        return lambda: self._homing.start_to(x, y, tolerance=config.HEALTH_GOAL_TOLERANCE_M, name=name,
                                             blocked_enough_m=blocked_enough_m, speed=speed)

    async def _search_area(self) -> dict | None:
        """Nobody lying seen from here: look from viewpoints around the spot.

        The person may lie behind a table or a sofa; from the other side the
        camera sees them. The viewpoints go around the spot in one direction.
        """
        pose = self._frame_pose()
        if self._target is None or pose is None:
            return None
        tx, ty = self._target["x"], self._target["y"]
        occ = getattr(self._homing, "occ_map", None) if self._by_map() else None
        points = search.viewpoints((tx, ty), pose[:2], config.HEALTH_VIEWPOINT_RADIUS_M,
                                   config.HEALTH_VIEWPOINT_MAX, config.HEALTH_VIEWPOINT_STEP_DEG,
                                   occ, config.HEALTH_VIEWPOINT_CLEARANCE_M)
        if not points:
            self._note("kein freier Blickpunkt um die Stelle")
            return None
        for i, (x, y) in enumerate(points, 1):
            self._note(f"sucht im Bereich: Blickpunkt {i}/{len(points)} ({x:.2f}, {y:.2f})")
            angle = math.atan2(ty - y, tx - x)
            started = await self._homing_drive(self._drive_to(x, y, "Blickpunkt", angle))
            reason = self._homing.last_reason or ("beendet" if started else "startet nicht")
            self._viewpoints.append({"x": round(x, 2), "y": round(y, 2), "erreicht": bool(
                started and getattr(self._homing, "arrived", True)), "grund": reason})
            self._note(f"Blickpunkt {i}: {reason}")
            await self._face_target()
            p = await self._search_person(config.HEALTH_VIEWPOINT_SEARCH_S, down_only=True)
            if p is not None:
                return p
        return None

    async def _find_person(self) -> dict | None:
        """Someone lying or slumped first, from here and from viewpoints;
        only then anybody (an upright person who may still need help)."""
        person = await self._search_person(down_only=True)
        if person is None and self._target is not None:
            self._note("niemand liegt in Sicht -- sucht den Bereich ab")
            person = await self._search_area()
        if person is None:
            self._note("niemand liegt -- sucht irgendeine Person")
            # the down search turned away from the spot, start from there again
            await self._face_target()
            person = await self._search_person(down_only=False)
        return person

    async def _home(self) -> None:
        # the check is over, nobody needs the skeleton on the way back
        self._use_pose(False)
        self._note("fährt zum Startpunkt zurück")
        origin = self._origin
        if self._by_map() and getattr(self._homing, "map_ready", False):
            # by map back to where the check started: dead reckoning over a
            # longer drive through rooms would drift
            if origin is not None:
                start = lambda: self._homing.start(target=origin[:2], angle=origin[2],  # noqa: E731
                                                   corridor=self._corridor(),
                                                   speed=config.HEALTH_DRIVE_SPEED)
            else:
                start = lambda: self._homing.start(corridor=self._corridor(),  # noqa: E731
                                                   speed=config.HEALTH_DRIVE_SPEED)
        else:
            x, y = origin[:2] if origin is not None else (0.0, 0.0)
            start = lambda: self._homing.start_to(x, y, name="Startpunkt",  # noqa: E731
                                                  speed=config.HEALTH_DRIVE_SPEED)
        if not await self._homing_drive(start):
            self._note("Heimfahrt startet nicht (läuft schon eine?)")
            return
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
            started = await self._homing_drive(self._drive_to(
                self._target["x"], self._target["y"], "gemeldete Stelle",
                blocked_enough_m=config.HEALTH_BLOCKED_ENOUGH_M), watch=self._spot_watch())
            if not started:
                self._note(f"Anfahrt startet nicht: {self._homing.last_reason or 'läuft schon eine Fahrt?'}")
            else:
                self._note(f"Anfahrt: {self._homing.last_reason or 'beendet'}")
            await self._face_target()

        # 2. overview
        self._note("sucht die Person", OVERVIEW)
        person = await self._find_person()
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
        self._posture = am.posture_over(history)
        if self._posture is not None:
            self._note(f"Haltung: {am.POSTURE_TEXT.get(self._posture, self._posture)}")
        # lying, on the floor or slumped in a chair: whoever does not answer
        # gets nudged (robot test: a slumped person was never nudged)
        position = (any(self._checks[n]["zustand"] == am.FLAGGED for n in ("liegend", "boden"))
                    or self._posture == "zusammengesackt")

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
            # the answer to "geht es ihnen gut?" counts now (robot test:
            # "mir geht's gut" changed nothing)
            wellbeing = am.interpret_wellbeing(heard)
            self._wellbeing = wellbeing
            self._note(f"hört: „{' / '.join(heard)}“ -> {wellbeing or 'unklar'}")
            if wellbeing == "hilfe":
                reason = "sagt, dass es ihr nicht gut geht"
                self._verdict = {"stufe": "notfall", "punkte": am.count(self._checks), "text": reason}
                await self._help(reason)
                return
            if wellbeing == "gut" and not position:
                await self._responsive("Person sagt, es geht ihr gut")
                return
            # well but lying on the floor, or unclear: ask once more
            await self._follow_up()
            return
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
            await self._responsive("Person ansprechbar, braucht keine Hilfe")
            return
        # yes, help -- or no/unclear answer: when in doubt get help; a false
        # alarm costs a glance, a missed emergency more
        reason = "bittet um Hilfe" if interpretation == "hilfe" else "keine klare Antwort auf die Nachfrage"
        self._verdict = {"stufe": "notfall", "punkte": am.count(self._checks), "text": reason}
        await self._help(reason)

    async def _responsive(self, text: str) -> None:
        self._note("Person ansprechbar", RESPONSIVE)
        self._verdict = {"stufe": "ansprechbar", "punkte": am.count(self._checks), "text": text}
        await self._send("info", "Healthbot: Person ansprechbar")
        await self._speak(config.HEALTH_ANNOUNCE_OK)
        await self._home()

    async def _nudge_all(self) -> bool:
        """Nudge up to HEALTH_NUDGE_COUNT times. True on a reaction."""
        self._note("fährt heran, um anzustupsen", NUDGE)
        # say what is coming: a person lying there should not be startled by
        # a silent robot touching them
        await self._speak(config.HEALTH_NUDGE_ANNOUNCE)
        if not await self._approach():
            self._note("kommt nicht nah genug heran -- kein Stupsen möglich")
            await self._make_room()
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
        await self._make_room()
        return False

    async def _make_room(self) -> None:
        """Back off from the person before turning to look for help: pressed
        against them it has no room to turn (robot test: got stuck)."""
        self._note(f"setzt {config.HEALTH_HELP_BACK_M:.1f} m zurück")
        await self._back_off(config.HEALTH_HELP_BACK_M)

    async def _approach(self) -> bool:
        """Slowly up to HEALTH_NUDGE_START_M (lidar) to the person, keeping
        them centred. The normal drive guard stays on.

        a person lying on the floor is below the lidar plane, so the lidar
        distance is often the wall behind them: then only the drive guard
        (depth camera, memory) or the wheels stopping at the body end the
        approach. limited by distance and a generous time, not by a short
        timeout (robot test: gave up after 15 s while still on its way)
        """
        deadline = self._now() + config.HEALTH_APPROACH_MAX_S
        start = self._odometry.pose()
        began = self._now()
        still_since: float | None = None
        slip = _SlipCheck()
        # last lidar distance: (distance, travelled then, time)
        last: tuple[float, float, float] | None = None
        blind_since: float | None = None
        while self._now() < deadline:
            if self._travelled(start) > config.HEALTH_APPROACH_MAX_M:
                self._watchdog.stop()
                self._note(f"nach {config.HEALTH_APPROACH_MAX_M:.1f} m noch nicht bei der Person -- bricht ab")
                return False
            p = self._seen()
            if p is not None:
                self._person = p
            legs = self._leg_angle(p)
            d = self._distance(p, legs)
            now = self._now()
            if d is not None:
                last = (d, self._travelled(start), now)
                blind_since = None
            elif last is not None and now - last[2] <= config.HEALTH_APPROACH_BLIND_S:
                # one scan missing or stale: the distance shrinks by the way driven
                d = last[0] - (self._travelled(start) - last[1])
            else:
                blind_since = blind_since if blind_since is not None else now
                self._watchdog.stop()
                if now - blind_since > config.HEALTH_APPROACH_BLIND_S:
                    self._note("kein Lidar-Abstand zur Person")
                    return False
                await self._tick()
                continue
            if d <= config.HEALTH_NUDGE_START_M:
                self._watchdog.stop()
                return True
            limit = config.HEALTH_APPROACH_OMEGA_MAX
            # towards the legs when they are seen: that is where it nudges
            aim = legs if legs is not None else (None if p is None else p["winkel"])
            omega = 0.0 if aim is None else max(-limit, min(limit, math.radians(aim)))
            # faster from afar, slow just before the person
            share = (d - config.HEALTH_NUDGE_START_M) / config.HEALTH_APPROACH_SLOWDOWN_M
            vx = max(config.HEALTH_APPROACH_MIN_SPEED, config.HEALTH_APPROACH_SPEED * min(1.0, share))
            driven = self._drive(vx=vx, omega=omega)
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
            # wheels turn but the scene ahead does not come closer: it pushes
            # against the person (robot test: odometry counted on to 4 m)
            if driven.vx > 0.0 and self._slipping(slip, start):
                self._watchdog.stop()
                self._note("Räder drehen durch -- steht an der Person")
                return True
            if driven.vx <= 0.0:
                slip.reset()
            # wheels stand although it drives: it touches the person already
            vx_actual = self._odometry.velocity()[0]
            if self._now() - began > 1.0 and driven.vx > 0.0 and vx_actual < config.HEALTH_NUDGE_STILL_MS:
                still_since = still_since or self._now()
                if self._now() - still_since >= config.HEALTH_NUDGE_CONTACT_S:
                    self._watchdog.stop()
                    self._note("Räder stehen an der Person -- nah genug zum Stupsen")
                    return True
            else:
                still_since = None
            await self._tick()
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
        began = self._now()
        still_since: float | None = None
        # (time, way) samples: contact is little gain over HEALTH_NUDGE_CONTACT_S
        gains: list[tuple[float, float]] = []
        slip = _SlipCheck()
        reason = "Zeitgrenze"
        while self._now() - began < config.HEALTH_NUDGE_MAX_S:
            now = self._now()
            way = self._travelled(start)
            if way >= limit:
                reason = "Weggrenze"
                break
            driven = self._drive(vx=config.HEALTH_NUDGE_SPEED, margin_m=config.HEALTH_NUDGE_MARGIN_M)
            # the drive guard also sees what lies below the lidar plane: its
            # distance to contact limits the nudge depth as well
            free = getattr(self._watchdog.last_check, "free_m", None)
            if free is not None:
                limit = min(limit, way + free + config.HEALTH_NUDGE_DEPTH_M)
            gains.append((now, way))
            while len(gains) > 1 and now - gains[1][0] >= config.HEALTH_NUDGE_CONTACT_S:
                gains.pop(0)
            window_full = now - gains[0][0] >= config.HEALTH_NUDGE_CONTACT_S
            # a command scaled below the motor minimum does not move the
            # wheels: that is the guard holding, not the person
            pushing = driven.vx >= config.HEALTH_APPROACH_MIN_SPEED - 1e-6
            # only watch for standstill after starting off (ramp)
            if now - began > 0.6 and pushing and window_full and way - gains[0][1] < config.HEALTH_NUDGE_STALL_M:
                reason = "Kontakt"
                break
            # the wheels turn on against the body: contact as well (robot
            # test: it pushed against a leg and did not stop)
            if pushing and self._slipping(slip, start):
                reason = "Kontakt"
                break
            if not pushing:
                slip.reset()
            if now - began > 0.6 and not pushing:
                still_since = still_since or now
                if now - still_since >= config.HEALTH_NUDGE_CONTACT_S:
                    if getattr(self._watchdog.last_check, "reason", None) == drive_guard.NO_LIDAR:
                        # held because the lidar dropped out, not because of
                        # the person; the log should say so
                        reason = drive_guard.NO_LIDAR
                    else:
                        reason = GUARD_HOLDS
                    break
            else:
                still_since = None
            await self._tick()
        self._watchdog.stop()
        distance = self._travelled(start)
        self._note(f"Stups {nr}: {reason} nach {distance:.2f} m")
        return {"nr": nr, "grund": reason, "weg_m": round(distance, 3), "zeit": time.time()}

    async def _back_off(self, distance: float | None = None) -> None:
        """Back off at once so the person has room (rear has a drive guard too)."""
        distance = config.HEALTH_NUDGE_BACK_M if distance is None else distance
        start = self._odometry.pose()
        deadline = self._now() + _BACK_MAX_S
        while self._now() < deadline and self._travelled(start) < distance:
            self._drive(vx=-config.HEALTH_APPROACH_MIN_SPEED * 2.0)
            await self._tick()
        self._watchdog.stop()

    async def _help(self, reason: str = "liegt regungslos und reagiert nicht") -> None:
        """Get help: alarm in the app, announcement, then fetch a helper and
        lead them here; if nobody is found, drive home and play music."""
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
        led = await self._fetch_help() if config.HEALTH_FETCH_HELP else None
        if led:
            self._note("Helfer ist bei der Person", HELP)
            self._step = "Helfer zur Person gebracht"
            return
        if led is None:
            await self._home()
        # music even if the homing drive did not fully work: it should call
        # people over wherever the robot is (after leading alone: at the person)
        await self._play_music()
        self._note("ruft mit Musik um Hilfe", HELP)
        self._step = "Hilfe gerufen"

    # ------------------------------------------------------------------ fetch help

    def _victim_spot(self) -> tuple[float, float] | None:
        """Where the person lies, in the frame of the check: from the robot
        in front of them along their direction. The lidar often measures past
        someone on the floor, so the distance is kept short."""
        pose = self._frame_pose()
        if pose is None:
            return None if self._target is None else (self._target["x"], self._target["y"])
        p = self._person
        bearing = 0.0 if p is None else float(p["winkel"])
        d = self._distance(p) if p is not None else None
        d = 0.6 if d is None else min(max(d, 0.3), 1.2)
        return search.position(pose, bearing, d)

    def _place_of(self, person: dict) -> tuple[float, float] | None:
        pose = self._frame_pose()
        d = self._distance(person)
        if pose is None or d is None:
            return None
        return search.position(pose, person["winkel"], d)

    def _looks_like_helper(self, person: dict, victim) -> bool:
        """Upright and not in the direction of the person on the floor.

        an unclear posture does not count: the victim far away often has
        too few keypoints for "liegt" (robot test: taken as the helper)
        """
        if am.posture(person) not in am.UPRIGHT_POSTURES:
            return False
        pose = self._frame_pose()
        return pose is None or not search.sight_near(pose, person["winkel"], victim,
                                                     config.HEALTH_HELPER_SIGHT_M)

    def _helper_candidate(self, person: dict, victim) -> bool:
        if not self._looks_like_helper(person, victim):
            return False
        place = self._place_of(person)
        return place is not None and math.dist(place, victim) >= config.HEALTH_HELPER_MIN_FROM_VICTIM_M

    def _confirm_helper(self, victim, locked: dict | None, good: int) -> tuple[dict | None, int]:
        """One frame of confirming a helper: the person locked on and how many
        frames in a row they passed as helper."""
        people = self._pose.people() if self._pose is not None else []
        p = am.track(people, locked) if locked is not None and people else None
        if p is None:
            p = next((q for q in people if self._helper_candidate(q, victim)), None)
            return p, 0 if p is None else 1
        if self._helper_candidate(p, victim):
            return p, good + 1
        return None, 0

    async def _drive_watching(self, x: float, y: float, victim) -> dict | None:
        """Drive to a search point and watch for a helper on the way. Someone
        confirmed over a few frames ends the drive and is returned."""
        seen: dict = {"p": None, "good": 0}

        def watch() -> bool:
            seen["p"], seen["good"] = self._confirm_helper(victim, seen["p"], seen["good"])
            return seen["p"] is not None and seen["good"] >= config.HEALTH_HELPER_CONFIRM_FRAMES

        await self._homing_drive(self._drive_to(x, y, "Suchpunkt"), watch=watch,
                                 watch_reason="Person unterwegs gesehen")
        if seen["p"] is not None and seen["good"] >= config.HEALTH_HELPER_CONFIRM_FRAMES:
            return seen["p"]
        return None

    async def _look_for_helper(self, victim, duration: float) -> dict | None:
        """Turn on the spot until an upright person away from the victim is
        centred and confirmed over a few frames. Returns them, None after
        ``duration``."""
        deadline = self._now() + duration
        p: dict | None = None
        good = 0
        while self._now() < deadline:
            p, good = self._confirm_helper(victim, p, good)
            if p is not None:
                if abs(p["winkel"]) <= config.HEALTH_CENTERED_DEG and good >= config.HEALTH_HELPER_CONFIRM_FRAMES:
                    self._watchdog.stop()
                    return p
                limit = config.HEALTH_SEARCH_OMEGA
                self._drive(omega=max(-limit, min(limit, math.radians(p["winkel"]) * 1.5)))
            else:
                self._drive(omega=config.HEALTH_SEARCH_OMEGA)
            await self._tick()
        self._watchdog.stop()
        return None

    async def _go_to_helper(self, helper: dict, victim) -> bool:
        """Up to HEALTH_HELPER_TALK_M in front of the helper, keeping them centred.

        Walking away (HEALTH_HELPER_AWAY_M further than the nearest so far):
        it follows faster and calls them to stop every
        HEALTH_HELPER_STOP_CALL_S until it is close enough to talk.
        False if they are lost, no longer look upright or turn out to be in
        the direction of the person on the floor; then it searches on.
        """
        deadline = self._now() + config.HEALTH_HELPER_APPROACH_S
        lost_since: float | None = None
        held_since: float | None = None
        bad = 0
        nearest = math.inf
        chasing = False
        next_call = 0.0
        calls = 0
        bearing = float(helper["winkel"])
        try:
            while self._now() < deadline:
                now = self._now()
                people = self._pose.people() if self._pose is not None else []
                p = am.track(people, helper) if people else None
                if p is None and lost_since is not None and chasing:
                    # walking away they leave the picture and come back elsewhere in it
                    p = next((q for q in people if self._helper_candidate(q, victim)), None)
                if p is None:
                    lost_since = lost_since if lost_since is not None else now
                    if now - lost_since > config.HEALTH_HELPER_LOST_S:
                        self._note("Helfer aus dem Blick verloren -- sucht weiter")
                        return False
                    if chasing:
                        # turn to where they were last seen
                        self._drive(omega=math.copysign(config.HEALTH_SEARCH_OMEGA, bearing))
                    else:
                        self._watchdog.stop()
                    await self._tick()
                    continue
                lost_since = None
                helper = p
                bearing = float(p["winkel"])
                bad = 0 if self._looks_like_helper(p, victim) else bad + 1
                if bad >= config.HEALTH_HELPER_CONFIRM_FRAMES:
                    self._note("doch kein Helfer (liegt oder Richtung der Person) -- sucht weiter")
                    return False
                d = self._distance(p)
                if d is not None and d <= config.HEALTH_HELPER_TALK_M:
                    return True
                if d is not None:
                    nearest = min(nearest, d)
                    if not chasing and d > nearest + config.HEALTH_HELPER_AWAY_M:
                        chasing = True
                        deadline = max(deadline, now + config.HEALTH_HELPER_CHASE_S)
                        self._helper["hinterher"] = True
                        self._note("Helfer geht weg -- fährt hinterher und ruft")
                if chasing and now >= next_call:
                    texts = config.HEALTH_HELPER_STOP_CALLS
                    self._call_out(texts[calls % len(texts)])
                    calls += 1
                    next_call = now + config.HEALTH_HELPER_STOP_CALL_S
                limit = config.HEALTH_APPROACH_OMEGA_MAX
                speed = config.HEALTH_HELPER_CHASE_SPEED if chasing else config.HEALTH_LEAD_SPEED
                driven = self._drive(vx=speed, omega=max(-limit, min(limit, math.radians(p["winkel"]))))
                if driven.vx <= 0.0:
                    # drive guard holds: the helper or something next to them is just ahead
                    held_since = held_since if held_since is not None else now
                    if now - held_since >= config.HEALTH_HELPER_HOLD_S:
                        return True
                else:
                    held_since = None
                await self._tick()
            # still seen and upright after the time limit: talk from here
            return True
        finally:
            self._watchdog.stop()

    def _helper_stops(self, victim) -> list[tuple[float, float] | None]:
        """Where to look for a helper, None meaning right here.

        With a map: search points across it away from the person on the
        floor, so it first drives away from them. Without a map: here, then
        the place where the check started (someone reported it from there).
        """
        pose = self._frame_pose()
        occ = getattr(self._homing, "occ_map", None) if self._by_map() else None
        if occ is not None and pose is not None:
            points = search.search_points(occ, pose[:2], config.HEALTH_HELPER_POINT_SPACING_M,
                                          config.HEALTH_HELPER_POINT_CLEARANCE_M,
                                          config.HEALTH_HELPER_POINTS_MAX, avoid=victim,
                                          avoid_m=config.HEALTH_HELPER_POINT_MIN_FROM_VICTIM_M)
            if points:
                return list(points)
        stops: list[tuple[float, float] | None] = [None]
        if self._origin is not None and pose is not None and math.dist(self._origin[:2], pose[:2]) > 0.5:
            stops.append(self._origin[:2])
        return stops

    async def _fetch_help(self) -> bool | None:
        """Find an upright person, ask them to follow, lead them to the victim.

        None: nobody found (the caller drives home and plays music). False:
        the helper stopped following and it went on alone. True: led there.
        """
        victim = self._victim_spot()
        if victim is None:
            self._note("Ort der Person unbekannt -- kann niemanden hinführen")
            return None
        self._helper = {"opfer": [round(victim[0], 2), round(victim[1], 2)], "gefunden": False}
        self._note("sucht jemanden, der helfen kann", SEEK_HELP)
        await self._send("warning", "Healthbot: suche jemanden, der helfen kann")
        self._use_pose(True)
        until = self._now() + config.HEALTH_HELPER_SEARCH_S
        stops = self._helper_stops(victim)
        if stops[0] is not None:
            self._note("fährt erst von der Person weg und sucht dann")
        points = [s for s in stops if s is not None]
        self._helper["suchpunkte"] = [[round(x, 2), round(y, 2)] for x, y in points]
        # the points again and again until the search time is up (robot test:
        # one round, then it gave up after 18 s)
        rounds = 0
        while self._now() < until:
            rounds += 1
            self._helper["runden"] = rounds
            if rounds > 1:
                self._note(f"sucht weiter: Runde {rounds}")
                self._call_out(config.HEALTH_HELPER_ROUND_CALL)
            found = await self._search_round(victim, stops, points, until)
            if found is not None:
                return found
            await self._tick()
        self._note("niemanden zum Helfen gefunden")
        await self._speak(config.HEALTH_HELPER_NONE, priority=True)
        return None

    async def _search_round(self, victim, stops, points, until: float) -> bool | None:
        """One round over the search points; None if nobody was found,
        otherwise the result of leading the helper."""
        for stop in stops:
            if self._now() >= until:
                break
            helper = None
            if stop is not None:
                x, y = stop
                self._helper["punkt"] = points.index(stop)
                self._note(f"sucht Helfer: Punkt {points.index(stop) + 1}/{len(points)} ({x:.2f}, {y:.2f})")
                helper = await self._drive_watching(x, y, victim)
                if helper is not None:
                    self._note("Person unterwegs gesehen -- hält an")
            if helper is None:
                helper = await self._look_for_helper(victim, max(0.0, min(config.HEALTH_HELPER_LOOK_S,
                                                                          until - self._now())))
            if helper is None:
                continue
            self._note(f"Helfer gefunden ({helper['winkel']:+.0f}°), fährt hin")
            self._call_out(config.HEALTH_HELPER_HAIL)
            if not await self._go_to_helper(helper, victim):
                continue
            self._helper["gefunden"] = True
            await self._speak(config.HEALTH_HELPER_CALL, priority=True)
            return await self._lead(victim)
        return None

    def _followed(self) -> bool:
        return search.someone_behind(self._points(), config.HEALTH_FOLLOW_MAX_M, config.HEALTH_FOLLOW_HALF_DEG)

    async def _lead(self, victim) -> bool:
        """Drive to the victim slowly while the helper follows (lidar behind).

        Nobody behind for HEALTH_FOLLOW_LOST_S: stop, call and wait. After
        HEALTH_LEAD_TRIES calls it goes on alone. True if someone was still
        following on arrival.
        """
        self._note("führt den Helfer zur Person", LEAD)
        await self._send("info", "Healthbot: führe Helfer zur Person")
        tries = 0
        alone = False
        while True:
            await self._wait_drive()
            self._lost_during = False
            started = self._drive_to(victim[0], victim[1], "Person", speed=config.HEALTH_LEAD_SPEED,
                                     blocked_enough_m=config.HEALTH_BLOCKED_ENOUGH_M)()
            if not started:
                self._note(f"Fahrt zur Person startet nicht: {self._homing.last_reason or '?'}")
                break
            # the helper stands in front at first, the robot turns before they are behind it
            grace = time.monotonic() + 2.0 * config.HEALTH_FOLLOW_LOST_S
            lost_since: float | None = None
            stalled = False
            while self._homing.active:
                await asyncio.sleep(0.2)
                if alone or time.monotonic() < grace or self._drive_lost_at is not None:
                    continue
                if self._followed():
                    lost_since = None
                    continue
                lost_since = lost_since or time.monotonic()
                if time.monotonic() - lost_since >= config.HEALTH_FOLLOW_LOST_S:
                    stalled = True
                    self._homing.cancel("wartet auf den Helfer")
            while self._homing.active:
                await asyncio.sleep(0.1)
            if self._lost_during:
                await self._wait_drive()
                continue
            if not stalled:
                break
            tries += 1
            if tries > config.HEALTH_LEAD_TRIES:
                self._note("Helfer folgt nicht -- fährt allein zur Person")
                alone = True
                continue
            self._note(f"niemand folgt -- ruft und wartet ({tries}/{config.HEALTH_LEAD_TRIES})")
            await self._speak(config.HEALTH_HELPER_WAIT_CALL, priority=True)
            await self._wait_for_follower(config.HEALTH_LEAD_WAIT_S)
        self._note(f"bei der Person: {self._homing.last_reason or 'beendet'}")
        followed = not alone and self._followed()
        self._helper["gefolgt"] = followed
        if alone:
            await self._speak(config.HEALTH_ANNOUNCE_HELP, priority=True)
            return False
        await self._speak(config.HEALTH_HELPER_ARRIVED, priority=True)
        return True

    async def _wait_for_follower(self, duration: float) -> bool:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            # turn around would be nicer, but the rear lidar sees them anyway
            if self._followed():
                return True
            await asyncio.sleep(0.2)
        return False

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
                p = am.pick_patient(people, seen)
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
        "HEALTH_NUDGE_STILL_MS", "HEALTH_NUDGE_STALL_M", "HEALTH_NUDGE_MAX_S", "HEALTH_NUDGE_BACK_M",
        "HEALTH_APPROACH_SPEED", "HEALTH_APPROACH_ENOUGH_M", "HEALTH_GOAL_TOLERANCE_M",
        "HEALTH_BLOCKED_ENOUGH_M", "HEALTH_SEARCH_S", "HEALTH_CENTERED_DEG", "POSE_KP_MIN_CONF",
        "HEALTH_APPROACH_SLOWDOWN_M", "HEALTH_APPROACH_MIN_SPEED", "HEALTH_DRIVE_SPEED",
        "HEALTH_AREA_RADIUS_M", "HEALTH_VIEWPOINT_RADIUS_M", "HEALTH_VIEWPOINT_MAX",
        "HEALTH_VIEWPOINT_STEP_DEG", "HEALTH_VIEWPOINT_SEARCH_S", "HEALTH_WORDS_WELL",
        "HEALTH_WORDS_UNWELL", "HEALTH_FETCH_HELP", "HEALTH_HELPER_MIN_FROM_VICTIM_M",
        "HEALTH_HELPER_SEARCH_S", "HEALTH_HELPER_POINT_SPACING_M", "HEALTH_HELPER_AWAY_M",
        "HEALTH_HELPER_CHASE_SPEED", "HEALTH_LEAD_SPEED", "HEALTH_FOLLOW_MAX_M", "HEALTH_FOLLOW_HALF_DEG",
        "HEALTH_FOLLOW_LOST_S", "HEALTH_LEAD_TRIES", "HEALTH_HELP_BACK_M", "HEALTH_SLIP_WINDOW_S",
        "HEALTH_SLIP_MIN_WAY_M", "HEALTH_SLIP_RATIO", "HEALTH_SPOT_STANDOFF_M", "HEALTH_SPOT_WATCH_M",
    )
    return {n: getattr(config, n, None) for n in names}


class _SlipCheck:
    """Wheels turn but the scene ahead does not come closer: the robot
    pushes against something. Wheel slip fools odometry, the lidar does
    not slip."""

    def __init__(self) -> None:
        # (time, wheel way, distance ahead, heading in degrees)
        self._samples: list[tuple[float, float, float, float]] = []

    def reset(self) -> None:
        self._samples.clear()

    def update(self, now: float, way: float, ahead: float | None, heading: float) -> bool:
        if ahead is None:
            self.reset()
            return False
        self._samples.append((now, way, ahead, heading))
        while len(self._samples) > 1 and now - self._samples[1][0] >= config.HEALTH_SLIP_WINDOW_S:
            self._samples.pop(0)
        t0, w0, a0, h0 = self._samples[0]
        if now - t0 < config.HEALTH_SLIP_WINDOW_S:
            return False
        # turning shifts the points ahead, that says nothing about slip
        if abs((heading - h0 + 180.0) % 360.0 - 180.0) > config.HEALTH_SLIP_MAX_TURN_DEG:
            return False
        wheels = way - w0
        if wheels < config.HEALTH_SLIP_MIN_WAY_M:
            return False
        return a0 - ahead < config.HEALTH_SLIP_RATIO * wheels


def _log_callback_error(future: asyncio.Future) -> None:
    if future.cancelled():
        return
    if future.exception() is not None:
        log.error("HEALTHBOT: on_finished failed", exc_info=future.exception())
