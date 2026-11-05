"""React to hand signals: follow, come here, go away, stop.

When interaction mode is on, the robot looks for people and waits for a
gesture (``READY``). Recognised gestures switch the behavior:

``folgen``     stay behind the person at a distance of ``FOLLOW_DISTANCE_M``
``herkommen``  drive up to ``COME_HERE_DISTANCE_M``, then stand still
``weggehen``   ``GO_AWAY_DISTANCE_M`` backwards, then stand still
``stopp``      stop immediately, back to READY

Where each quantity comes from:

* **Direction** from the camera (angle of the person to the image centre).
  The camera knows *who* is a human, but cannot measure distance.
* **Distance** from the lidar, exactly in that direction. The lidar measures
  to the centimetre, but does not know what it sees. Together this gives
  "human, 15° left, 1.3 m".

Safety as with the explorer: every command goes through the
``SafetyWatchdog``; forwards only if the driving corridor is free, backwards
only if the rear is free; if something comes very close, it stands still;
without fresh lidar data it stands still. If this loop hangs, the watchdog
stops after 0.5 s.

Detour (03.11.2026, F3): if something is between it and the person (chair,
door frame), it slides diagonally past and keeps the person in view
(``avoidance.choose_direction``, mecanum). Whatever slips out of view to
the side while passing, it remembers briefly (short-term memory). If it is
lifted, it stops and goes to READY after being set down -- it does not
drive on by itself then.

Smooth driving (29.10.2026): jumps in the drive command draw current
spikes that trigger USB resets on the X3. That is why the camera angle is
smoothed, turn rate and speed grow steadily with the error (no hard
limits), short detection dropouts are coasted through instead of braking
hard, and the watchdog additionally limits the acceleration (``RAMP_*``).

In READY the loop sends **no** commands -- the robot stands still, and
nothing keeps the drive awake unnecessarily. Manual control from the app
ends the whole interaction mode (as with the explorer): whoever steers
themselves does not want a gesture to interfere.

Drive gone (29.10.2026): if the connection to the board drops (USB reset),
the interaction mode no longer ends but falls back to READY and keeps
watching. Driving modes are only accepted again once the board is back --
and even then only on a *new* gesture or a new command. It never starts
driving by itself after reconnecting.

Whom it follows (04.11.2026): ``target.py`` tracks the people across frames
(track IDs, own rotation removed) and keeps the gestures of two people
apart. They are no longer told apart: it follows the person in the frame,
and whoever was briefly gone and comes back is the target again. The target
lock with re-identification by appearance (03.11.) was removed for this --
it often took returning people for strangers and stood still.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from typing import TYPE_CHECKING, Callable

from .. import config
from ..navigation import avoidance
from ..navigation import obstacles as hd
from ..navigation.explorer import free_in_direction
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from . import gestures as gs
from .target import IDLE, TEMPORARILY_LOST, PersonTracker, TargetChoice

if TYPE_CHECKING:  # only for the type hints -- avoids circular imports
    from ..navigation.odometry import Odometry
    from ..sensors.lidar import LidarService
    from ..sensors.pose import PoseDetection
    from .voice import Voice

log = logging.getLogger(__name__)

#: Modes as they appear in the app (state.interaktion.modus).
READY = "bereit"
FOLLOW = "folgen"
COME_HERE = "herkommen"
GO_AWAY = "weggehen"
SEARCHING = "sucht"

#: Modes in which it drives -- these need a connected board.
_DRIVE_MODES = (FOLLOW, COME_HERE, GO_AWAY, SEARCHING)

#: Which gesture triggers which mode.
_GESTURE_TO_MODE = {
    gs.FOLLOW: FOLLOW,
    gs.COME_HERE: COME_HERE,
    gs.GO_AWAY: GO_AWAY,
    gs.STOP: READY,
}

#: Text for messages in the app.
_GESTURE_TEXT = {
    gs.FOLLOW: "Folgen",
    gs.STOP: "Stopp",
    gs.COME_HERE: "Herkommen",
    gs.GO_AWAY: "Weggehen",
}

#: What the robot says out loud when it has understood a gesture. Short and
#: in the first person -- the person should hear at once that it means them.
_GESTURE_SAY = {
    gs.FOLLOW: "Ich folge dir.",
    gs.STOP: "Okay, ich halte an.",
    gs.COME_HERE: "Ich komme.",
    gs.GO_AWAY: "Ich fahre zurück.",
}

#: Display text and sentence when the mode starts right away with a job
#: (app button "Folgen" or voice command "Robi komm her").
_START_TEXT = {
    READY: ("wartet auf ein Handzeichen", "Hallo! Zeig mir ein Handzeichen."),
    FOLLOW: ("folgt (per App)", "Jetzt folge ich dir."),
    COME_HERE: ("kommt her", "Ich komme."),
    GO_AWAY: ("nimmt Abstand", "Ich fahre zurück."),
}

#: How long (s) the last gesture stays in the display.
_GESTURE_DISPLAY_S = 3.0

#: If the person stands this precisely (°) in front of it, "herkommen" counts as done.
_TARGET_ANGLE = 8.0


class Interaction:
    """Interaction mode. Operated like ``Explorer`` (start/cancel/state)."""

    def __init__(
        self,
        watchdog: SafetyWatchdog,
        lidar: "LidarService | None",
        pose: "PoseDetection | None",
        voice: "Voice | None" = None,
        drive_connected: Callable[[], bool] | None = None,
        odometry: "Odometry | None" = None,
    ) -> None:
        self._watchdog = watchdog
        self._lidar = lidar
        self._pose = pose
        self._voice = voice
        # Measured turn rate for predicting the tracks (without it: drive command).
        self._odometry = odometry
        # Asks whether the board is currently connected. If not given (tests)
        # it counts as connected.
        self._drive_check = drive_connected
        self._gestures = gs.GestureDetection()
        self._task: asyncio.Task[None] | None = None
        # During the soft start (see _run): remember requests instead of
        # driving off right away.
        self._soft_start = False
        self._start_mode = READY
        self._reason: str | None = None
        self._messages: list[str] = []

        # For the display -- written by the loop, read by the status tick
        # (both in the same event loop, no lock needed).
        self._mode = READY
        self._text = ""
        self._angle: float | None = None
        self._distance: float | None = None
        self._count = 0

        self._mode_since = 0.0
        self._seen = 0.0  # when the target person was last visible
        self._last_side = 1.0  # +1 left, -1 right -- search goes that way
        self._search_for = FOLLOW  # switch to this mode after a successful search
        # Lidar hint while searching: direction (degrees, robot frame) in which
        # the person is expected -- updated with the robot's own rotation.
        self._search_direction = 0.0
        self._hint_in_view_since: float | None = None
        self._hint_off = False
        self._last_frame = -1.0
        self._no_lidar_since: float | None = None

        # Smooth driving
        self._smoothed_angle: float | None = None
        self._smoothed_frame = -1.0
        self._prev_command: DriveCommand | None = None
        self._distance_good: float | None = None
        self._distance_good_time = -1e9
        # Where the last distance came from ("Cluster: Mensch", "Minimum") -- for
        # tests and debugging. A separate detection, because the server calls
        # its own with the person angles: there even a chair in front of the
        # person would be called "Mensch", and that is exactly what we want to
        # tell apart here.
        self._distance_source: str | None = None
        self._obstacle_detection = hd.ObstacleDetection()
        self._last_near = -1e9
        # Avoidance (F3): the same building blocks as the explorer.
        self._memory = avoidance.ShortTermMemory(odometry)
        self._lift = avoidance.LiftDetector()
        self._lift_reported = False
        self._detour_side = 0.0  # +1 left, -1 right, 0: drives directly
        self._last_tick: float | None = None

        # Whom it follows (RAM only, cleared when switched off)
        self._tracker = PersonTracker()
        self._target_choice = TargetChoice()
        self._tracker_image = None  # image time the tracker last ran with
        self._selected_id: int | None = None  # whom to follow at the next start (gesture)
        self._gestures_per_track: dict[int, gs.GestureDetection] = {}
        self._gesture_display: tuple[str | None, float | None] = (None, None)
        # Pause from the app: the target stays remembered and keeps being
        # tracked, but it does not drive.
        self._paused = False

    # --- Control ----------------------------------------------------------------

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def last_reason(self) -> str | None:
        return self._reason

    @property
    def available(self) -> bool:
        """Needs pose detection (who, which gesture) and lidar (how far)."""
        return self._lidar is not None and self._pose is not None and self._pose.available

    @property
    def mode(self) -> str | None:
        return self._mode if self.active else None

    def start(self, mode: str = READY) -> bool:
        """Switch interaction mode on. False if already on or not possible."""
        if self.active or not self.available:
            return False
        self._reason = None
        self._task = asyncio.create_task(self._run(mode))
        return True

    def cancel(self, reason: str = "abgebrochen") -> None:
        """End immediately and stop."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._reason = reason
            log.info("Interaction %s", reason)
            self._say("Interaktion beendet.")
        self._task = None

    def follow(self, target_id: int | None = None) -> bool:
        """Follow via app button instead of gesture. Switches the mode on if necessary.

        It follows the person in the frame, the largest if there are several.
        ``target_id`` (track ID from ``/interaktion/pose``) names a specific
        one; the app no longer sends it since 04.11., older clients still may.
        If that person is no longer in the frame, it simply follows whoever
        is there.
        """
        if not self.active:
            # Pass the mode right away -- the loop only starts in the next
            # pass of the event loop and would otherwise reset it to READY.
            return self.start(FOLLOW)
        now = time.monotonic()
        if target_id is not None and self._tracker.track(target_id) is not None:
            if self._target_choice.id != target_id:
                self._target_choice.release()
            self._selected_id = target_id
        if self._soft_start:
            # Soft start still running: following begins after the
            # announcement. The selection above applies already, otherwise it
            # would be lost.
            self._start_mode = FOLLOW
            return True
        self._switch(FOLLOW, "folgt (per App)", now, say="Ich folge dir.")
        return True

    def pause(self) -> bool:
        """Stop, but keep the target. False if nothing is driving right now."""
        if not self.active or self._mode not in _DRIVE_MODES or self._paused:
            return False
        self._paused = True
        self._watchdog.stop()
        self._prev_command = None
        self._text = "pausiert -- Ziel bleibt gemerkt"
        log.info("Interaction: paused (%s)", self._mode)
        return True

    def resume(self) -> bool:
        """Drive on after a pause. False if not paused."""
        if not self.active or not self._paused:
            return False
        now = time.monotonic()
        self._paused = False
        # The pause does not count as lost or search time, otherwise it
        # would give up right after resuming.
        self._seen = self._mode_since = now
        log.info("Interaction: resumed (%s)", self._mode)
        return True

    @property
    def paused(self) -> bool:
        return self.active and self._paused

    def people_with_id(self, people: list[dict]) -> list[dict]:
        """People for the app: without feature, with track ``id`` and ``ziel``.

        The pose delivers the same dicts the tracker received; that is how
        the assignment is recognised. If the image is newer than the last
        tracker tick, ``id`` is briefly ``None``.
        """
        ids = {id(t.person): t.id for t in self._tracker.tracks if t.measured}
        is_following = self._target_choice.id if self._target_choice.state != IDLE else None
        out = []
        for p in people:
            tid = ids.get(id(p))
            entry = {k: v for k, v in p.items() if k != "merkmal"}
            entry["id"] = tid
            entry["ziel"] = tid is not None and tid == is_following
            out.append(entry)
        return out

    def drive_lost(self) -> None:
        """Board gone: stop, but keep watching (see module docstring).

        This used to end the whole interaction mode. Because a USB reset
        often happened on the X3 right when gesture control was switched on,
        it was then off again immediately -- and you had to restart.
        """
        if self.active and self._mode != READY:
            self._switch(READY, "Verbindung zum Antrieb verloren -- angehalten",
                           time.monotonic(), report=True)

    def switch_to(self, mode: str, text: str, say: str | None = None) -> bool:
        """Set the behavior from outside (voice command). False if not on."""
        if not self.active:
            return False
        if self._soft_start:
            # Still in the soft start: applies as soon as it is done.
            self._start_mode = mode
            return True
        self._switch(mode, text, time.monotonic(), say=say)
        return self._mode == mode

    def follow_stop(self) -> None:
        """End the behavior, interaction mode stays on."""
        if self.active and self._soft_start:
            self._start_mode = READY
        elif self.active and self._mode != READY:
            self._switch(READY, "angehalten (per App)", time.monotonic(), say="Okay, ich bleibe stehen.")
        if self.active and self._mode == READY:
            # "Loslassen" (release) also clears the reason of the last
            # search -- afterwards the app shows "kein Ziel" again.
            self._target_choice.release()

    def fetch_messages(self) -> list[str]:
        """Events since the last call -- for hints in the app."""
        new, self._messages = self._messages, []
        return new

    def state(self) -> dict:
        """For state.interaktion in the app."""
        active = self.active
        now = time.monotonic()
        gesture = None
        # Gestures come from the per-track detection or (old way) from the
        # single detection -- the more recent one is displayed.
        last = [
            (timestamp, g)
            for g, timestamp in ((self._gestures.last, self._gestures.last_time), self._gesture_display)
            if timestamp is not None
        ]
        if last:
            timestamp, g = max(last, key=lambda e: e[0])
            if now - timestamp < _GESTURE_DISPLAY_S:
                gesture = g
        person = None
        if active and self._angle is not None:
            person = {
                "winkel": round(self._angle, 1),
                "abstand": (
                    round(self._distance, 2)
                    if self._distance is not None and math.isfinite(self._distance)
                    else None
                ),
            }
        return {
            "aktiv": active,
            "verfuegbar": self.available,
            "modus": self._mode if active else None,
            "text": self._text if active else None,
            "person": person,
            "personen": self._count if active else 0,
            "geste": gesture if active else None,
            "grund": self._reason,
            "ziel": self._target_choice.as_dict(),
            "pausiert": self.paused,
        }

    # --- Loop --------------------------------------------------------------------

    async def _run(self, mode: str) -> None:
        tick = 1.0 / max(1, config.ODOMETRY_HZ)
        self._gestures = gs.GestureDetection()
        self._angle = self._distance = None
        self._last_frame = -1.0
        self._no_lidar_since = None
        self._smoothed_angle, self._smoothed_frame = None, -1.0
        self._distance_good, self._distance_good_time = None, -1e9
        self._last_near = -1e9
        self._memory.clear()
        self._lift.reset()
        self._lift_reported = False
        self._detour_side = 0.0
        self._last_tick = None
        self._forget_target()
        self._mode = READY
        self._start_mode = mode

        try:
            # Soft start: pose detection, announcement and motors one after
            # another instead of in the same second -- together they broke the
            # USB power supply on 03.11. (see config.INTERACTION_STARTUP_S).
            if self._pose is not None:
                self._pose.activate(True)
            log.info("Interaction started")
            self._soft_start = True
            self._text = "startet …"
            if config.INTERACTION_STARTUP_S > 0:
                await asyncio.sleep(config.INTERACTION_STARTUP_S)
            # Whatever was requested in the meantime applies (speech, app).
            mode = self._start_mode
            text, sentence = _START_TEXT.get(mode, _START_TEXT[READY])
            if mode in _DRIVE_MODES and self._voice is not None:
                # Finish speaking first, then drive off.
                self._say(sentence)
                await self._finish_speaking()
                sentence = None
                if self._start_mode != mode:  # e.g. "Stopp" during the announcement
                    mode = self._start_mode
                    text, sentence = _START_TEXT.get(mode, _START_TEXT[READY])
            self._soft_start = False
            self._switch(mode, text, time.monotonic(), say=sentence)

            while True:
                command = self._step(time.monotonic())
                # None means: send nothing (READY). The stop already happened
                # when switching to READY.
                if command is not None:
                    self._watchdog.submit(command, source="autonom")
                await asyncio.sleep(tick)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler in der Interaktion"
            log.exception("Interaction aborted")
        finally:
            self._soft_start = False
            # Do not keep the learned appearance beyond switching off.
            self._forget_target()
            if self._pose is not None:
                self._pose.activate(False)

    async def _finish_speaking(self) -> None:
        """Wait until the voice has spoken the sentence (at most VOICE_TIMEOUT_S)."""
        end = time.monotonic() + config.VOICE_TIMEOUT_S
        # Give it a moment until the sentence has arrived in the queue.
        await asyncio.sleep(0.1)
        while self._voice.busy and time.monotonic() < end:
            await asyncio.sleep(0.1)

    def _step(self, now: float) -> DriveCommand | None:
        """One tick: look at person and gesture, return the drive command."""
        stop_cmd = DriveCommand(0.0, 0.0, 0.0)

        # Own pose for the short-term memory -- every tick, even without a new scan.
        dt = 0.0 if self._last_tick is None else min(0.5, max(0.0, now - self._last_tick))
        self._last_tick = now
        self._memory.advance(dt)

        target = self._choose_target(now)
        self._check_gesture(target, now)

        if target is not None:
            self._smooth(target, now)
            self._seen = now
            self._angle = target["winkel"]
            if abs(self._angle) > 2.0:
                self._last_side = 1.0 if self._angle > 0 else -1.0

        if self._mode != READY and not self._drive_available():
            # Second safeguard besides drive_lost(): the connection
            # monitor only checks every RECONNECT_CHECK_S.
            return self._switch(READY, "Verbindung zum Antrieb verloren -- angehalten",
                                  now, report=True)

        if self._mode == READY:
            self._distance = self._distance_to(target, now=now)
            return None

        if self._paused:
            # Tracker and target choice keep running above, so the target is
            # still the same after resuming. No driving.
            self._distance = self._distance_to(target, now=now)
            self._text = "pausiert -- Ziel bleibt gemerkt"
            self._prev_command = None
            return stop_cmd

        if self._mode in (COME_HERE, GO_AWAY) and now - self._mode_since > config.INTERACTION_BEHAVIOR_MAX_S:
            return self._switch(READY, f"{self._mode}: Zeit abgelaufen", now, report=True,
                                 say="Das dauert zu lange. Ich bleibe stehen.")

        # No driving without a fresh lidar scan. Wait out short dropouts
        # (USB reset), end the behavior on longer ones.
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or now - scan.timestamp > config.EXPLORE_SCAN_MAX_AGE_S:
            if self._no_lidar_since is None:
                self._no_lidar_since = now
                log.warning("Interaction: no fresh lidar data -- stopping")
            if now - self._no_lidar_since > config.EXPLORE_LIDAR_WAIT_S:
                return self._switch(READY, "Lidar liefert keine Daten", now, report=True,
                                     say="Ich sehe nichts mehr und halte an.")
            self._text = "keine frischen Lidar-Daten"
            self._prev_command = None
            return stop_cmd
        self._no_lidar_since = None

        # Lifted or carried: stop. After being set down nothing is valid any
        # more (where the person is, what was in the way) -- so do not drive
        # on by itself but wait for a new gesture.
        pose = self._memory.pose if self._memory.measuring else None
        if self._lift.check(scan, pose):
            if not self._lift_reported:
                self._lift_reported = True
                self._memory.clear()
                self._detour_side = 0.0
                log.info("Interaction: lifted (%.0f %% of the lidar image changed)",
                         100 * self._lift.share)
            self._text = "hochgehoben -- wartet, bis er wieder steht"
            self._prev_command = None
            return stop_cmd
        if self._lift_reported:
            self._lift_reported = False
            return self._switch(READY, "hochgehoben -- angehalten", now, report=True,
                                 say="Ich wurde hochgehoben. Ich bleibe stehen.")

        # The raw scan for "very close right now"; with memory for driving --
        # there the door frame that has just slipped out of view beside it
        # counts too.
        raw = scan.points()
        points = self._memory.points(scan)
        self._distance = self._distance_to(target, scan, now)
        if self._distance is not None and math.isfinite(self._distance):
            self._distance_good, self._distance_good_time = self._distance, now
        elif target is not None and now - self._distance_good_time < config.FOLLOW_HOLD_S:
            # Briefly no lidar point in the direction of the person (legs
            # between two beams) -- take the last good value instead of
            # stopping and immediately starting again.
            self._distance = self._distance_good

        if self._mode == GO_AWAY:
            return self._go_away(now, target, points, raw)

        # Follow, come here, search: everything that drives forwards or turns
        # on the spot. Something very close -> do not move at all (the corners
        # swing out when turning). Only from three points on, one stray beam
        # does not count. After "near" stay put briefly, so it does not
        # flicker between stop and drive at the limit.
        if self._near(raw):
            self._last_near = now
        if now - self._last_near < config.FOLLOW_NEAR_HOLD_S:
            self._text = "etwas sehr nah -- warte"
            self._prev_command = None
            return stop_cmd

        if self._mode == SEARCHING:
            return self._search_step(now, target, scan, dt)

        if target is None:
            if now - self._seen > self._lost_deadline():
                self._search_for = self._mode
                return self._switch(SEARCHING, "Person verloren -- suche", now, report=True,
                                     say="Wo bist du?")
            coast = self._coast(now, points)
            if coast is not None:
                return coast
            self._text = "Person kurz nicht zu sehen -- warte"
            return stop_cmd

        requested = config.FOLLOW_DISTANCE_M if self._mode == FOLLOW else config.COME_HERE_DISTANCE_M
        if (
            self._mode == COME_HERE
            and self._distance is not None
            and self._distance <= requested + config.FOLLOW_DEADBAND_M
            and abs(self._angle) < _TARGET_ANGLE
        ):
            return self._switch(READY, "bin da", now, report=True, say="Da bin ich.")
        command = self._approach_target(points, requested, backwards=self._mode == FOLLOW)
        self._prev_command = command
        return command

    # --- Behavior ---------------------------------------------------------------

    def _approach_target(self, points, requested: float, backwards: bool) -> DriveCommand:
        """P controller: turn to the person and keep the distance ``requested``."""
        angle = self._smoothed_angle if self._smoothed_angle is not None else (self._angle or 0.0)
        # Do not correct small angles: the image noise of the pose (one to two
        # degrees) would otherwise make it twitch back and forth constantly.
        # Subtracted rather than clipped, so omega starts at 0 at the limit.
        rest = max(0.0, abs(angle) - config.FOLLOW_ANGLE_DEADBAND)
        omega = math.copysign(config.FOLLOW_KP_W * math.radians(rest), angle)
        omega = max(-config.FOLLOW_MAX_OMEGA, min(config.FOLLOW_MAX_OMEGA, omega))

        # Align first, then drive: driving off at an angle pushes it past the
        # person, and the driving corridor straight ahead says nothing about
        # the path to the person. Instead of cutting off hard at
        # FOLLOW_MAX_ANGLE_DRIVE, the speed decreases steadily from
        # FOLLOW_FULL_ANGLE -- an on/off at the limit was one of the jerks.
        magnitude = abs(angle)
        if magnitude <= config.FOLLOW_FULL_ANGLE:
            share = 1.0
        else:
            span = max(1e-6, config.FOLLOW_MAX_ANGLE_DRIVE - config.FOLLOW_FULL_ANGLE)
            share = max(0.0, (config.FOLLOW_MAX_ANGLE_DRIVE - magnitude) / span)

        vx = vy = 0.0
        detour: str | None = None
        distance = self._distance
        if distance is not None and math.isfinite(distance) and share > 0.0:
            error = distance - requested
            tot = config.FOLLOW_DEADBAND_M
            width = config.EXPLORE_HALF_WIDTH_M
            # Here too computed from the deadband, so vx starts at 0 there.
            if error > tot:
                v = min(config.FOLLOW_MAX_SPEED, config.FOLLOW_KP_V * (error - tot)) * share
                # With the same margin as for avoidance: otherwise it swings
                # back too early after sliding past and grazes the corner.
                front = free_in_direction(points, 0.0, width + config.AVOID_MARGIN_M)
                # Room for the next stretch. It ends in front of the person,
                # so the person never blocks their own path.
                needed = min(error - tot, config.DETOUR_LOOKAHEAD_M) + config.EXPLORE_STOP_M
                choice = None
                if front >= needed:
                    self._detour_side = 0.0
                else:
                    # Straight ahead blocked: slide past diagonally, the view
                    # (omega) stays on the person.
                    choice = avoidance.choose_direction(points, angle, needed, width, self._detour_side)
                if choice is None:
                    # Never closer than the stop limit to anything in the driving corridor.
                    vx = min(v, max(0.0, front - config.EXPLORE_STOP_M))
                else:
                    direction, free = choice
                    v = min(v, max(0.0, free - config.EXPLORE_STOP_M))
                    a = math.radians(direction)
                    vx, vy = v * math.cos(a), v * math.sin(a)
                    shift = avoidance._normalize_angle(math.radians(direction - angle))
                    if abs(shift) > 1e-6:
                        self._detour_side = math.copysign(1.0, shift)
                        detour = "links" if shift > 0 else "rechts"
            elif error < -tot and backwards:
                behind = free_in_direction(points, 180.0, width)
                if behind > config.EXPLORE_STOP_M:
                    vx = -min(config.FOLLOW_REVERSE_SPEED, config.FOLLOW_KP_V * (-error - tot)) * share

        if detour is not None and vx > 0:
            self._text = f"umfährt Hindernis ({detour}, {distance:.2f} m)"
        elif vx > 0:
            self._text = f"fährt zur Person ({distance:.2f} m)"
        elif vx < 0:
            self._text = f"weicht zurück ({distance:.2f} m)"
        elif distance is None or not math.isfinite(distance):
            self._text = "richtet sich aus (Abstand unbekannt)"
        else:
            self._text = f"hält Abstand ({distance:.2f} m)"
        return DriveCommand(vx, vy, omega)

    def _coast(self, now: float, points) -> DriveCommand | None:
        """Person gone for a moment: let the last command coast out.

        The pose often loses the person for one or two images (motion blur,
        occlusion). Stopping hard each time and immediately starting again
        was the main reason for the jerking. The driving corridor keeps being
        checked meanwhile.
        """
        if self._prev_command is None:
            return None
        since = now - self._seen
        if since >= config.FOLLOW_HOLD_S:
            return None
        share = 1.0 - since / config.FOLLOW_HOLD_S
        last = self._prev_command
        width = config.EXPLORE_HALF_WIDTH_M
        vx, vy = last.vx * share, last.vy * share
        if vx > 0:
            # Diagonal during a detour: check in the driving direction and
            # shorten both components together, so the direction is kept.
            direction = math.degrees(math.atan2(vy, vx))
            space = max(0.0, free_in_direction(points, direction, width) - config.EXPLORE_STOP_M)
            v = math.hypot(vx, vy)
            if v > space:
                vx, vy = vx * space / v, vy * space / v
        elif vx < 0 and free_in_direction(points, 180.0, width) <= config.EXPLORE_STOP_M:
            vx = vy = 0.0
        self._text = "Person kurz nicht zu sehen -- rollt aus"
        return DriveCommand(vx, vy, last.omega * share)

    def _go_away(self, now: float, target: dict | None, points, raw) -> DriveCommand:
        duration = config.GO_AWAY_DISTANCE_M / max(0.01, config.GO_AWAY_SPEED)
        if now - self._mode_since >= duration:
            self._switch(READY, "Abstand genommen", now, report=True, say="So, genug Abstand.")
            return DriveCommand(0.0, 0.0, 0.0)
        # Backwards only what is behind counts -- in front stands the very
        # person who sent it away.
        behind = free_in_direction(points, 180.0, config.EXPLORE_HALF_WIDTH_M)
        if behind <= config.EXPLORE_STOP_M or self._near(raw, rear_only=True):
            self._switch(READY, "hinten kein Platz -- bleibe stehen", now, report=True,
                          say="Hinter mir ist kein Platz.")
            return DriveCommand(0.0, 0.0, 0.0)
        # Keep the person in view meanwhile (gently, without forcing alignment).
        omega = 0.0
        if target is not None:
            omega = 0.5 * config.FOLLOW_KP_W * math.radians(target["winkel"])
            omega = max(-config.FOLLOW_MAX_OMEGA, min(config.FOLLOW_MAX_OMEGA, omega))
        self._text = "fährt zurück"
        return DriveCommand(-config.GO_AWAY_SPEED, 0.0, omega)

    def _search_step(self, now: float, target: dict | None, scan=None, dt: float = 0.0) -> DriveCommand:
        # Whoever appears now is the target (TargetChoice in target.py) --
        # even with a new track number, because they were briefly out of view.
        if target is not None:
            self._switch(self._search_for, "Person wiedergefunden", now, report=True,
                          say="Da bist du ja.")
            return DriveCommand(0.0, 0.0, 0.0)
        searching_since = now - self._mode_since
        if config.FOLLOW_SEARCH_S > 0 and searching_since > config.FOLLOW_SEARCH_S:
            self._target_choice.release("Person nicht wiedergefunden")
            self._switch(READY, "Person nicht wiedergefunden", now, report=True,
                          say="Ich habe dich verloren.")
            return DriveCommand(0.0, 0.0, 0.0)
        hint = self._lidar_hint(now, scan, dt)
        if hint is not None:
            return hint
        # Alternate turning and briefly holding still: while turning the image
        # blurs, standing still the pose detection sees sharply.
        tick = config.FOLLOW_SEARCH_TURN_S + config.FOLLOW_SEARCH_PAUSE_S
        if tick > 0 and searching_since % tick >= config.FOLLOW_SEARCH_TURN_S:
            self._text = "sucht die Person -- schaut"
            return DriveCommand(0.0, 0.0, 0.0)
        # Turn towards the side where the person was last -- that is usually
        # where they walked out of view. Without a time limit, round and round.
        self._text = "sucht die Person"
        return DriveCommand(0.0, 0.0, self._last_side * config.EXPLORE_TURN_SPEED)

    def _lost_deadline(self) -> float:
        """How long the person may be missing before it searches.

        Last seen at the image edge means: they walked out to the side, and
        every second of waiting increases the angle it has to turn after them.
        In the image centre, on the other hand, they usually disappear only
        briefly (occluded). Only if the target choice has just lost the
        person -- ``_angle`` may otherwise still be from the last follow.
        """
        if self._angle is None or self._target_choice.state != TEMPORARILY_LOST:
            return config.FOLLOW_LOST_S
        fov = self._pose.fov_deg if self._pose is not None else config.CAMERA_FOV_DEG
        if abs(self._angle) > fov / 2 - config.TARGET_EDGE_DEG:
            return min(config.FOLLOW_LOST_S, config.FOLLOW_LOST_EDGE_S)
        return config.FOLLOW_LOST_S

    def _lidar_hint(self, now: float, scan, dt: float) -> DriveCommand | None:
        """While searching, turn towards a lidar person cluster instead of blindly.

        The lidar sees all around, the camera only to the front: if someone is
        behind it, it turns towards them deliberately instead of in a fixed
        rhythm in one direction. It still only follows whoever the camera
        sees (TargetChoice) -- the lidar only says where it is worth looking.
        If the camera recognises nobody there after TARGET_HINT_S, the fixed
        rhythm applies again for the rest of the search (chair legs look like
        legs to the lidar).
        """
        # Carry the expected direction along with the robot's own rotation.
        direction = self._search_direction - math.degrees(self._self_rotation()) * dt
        self._search_direction = (direction + 180.0) % 360.0 - 180.0
        if self._hint_off or scan is None:
            return None
        humans = [h for h in self._obstacle_detection.current(scan) if h.category == hd.HUMAN]
        if not humans:
            self._hint_in_view_since = None
            return None

        def norm(w: float) -> float:
            return (w + 180.0) % 360.0 - 180.0

        # If several, the one closest to the expected direction.
        best = min(humans, key=lambda h: abs(norm(h.angle - self._search_direction)))
        angle = norm(best.angle)
        fov = self._pose.fov_deg if self._pose is not None else config.CAMERA_FOV_DEG
        if abs(angle) > fov / 2 - config.TARGET_EDGE_DEG:
            self._hint_in_view_since = None
            self._last_side = 1.0 if angle > 0 else -1.0
            self._text = "sucht die Person -- dreht zu einer Person (Lidar)"
            return DriveCommand(0.0, 0.0, self._last_side * config.EXPLORE_TURN_SPEED)
        if self._hint_in_view_since is None:
            self._hint_in_view_since = now
        if now - self._hint_in_view_since < config.TARGET_HINT_S:
            self._text = "sucht die Person -- schaut zur Person (Lidar)"
            return DriveCommand(0.0, 0.0, 0.0)
        self._hint_off = True
        return None

    # --- Helpers ----------------------------------------------------------------

    def _choose_target(self, now: float) -> dict | None:
        """The person in question -- with their track ID (``"id"``, internal only).

        In driving modes the one from the TargetChoice: the same as last
        time, and if that one is gone, the next one to appear there. In READY
        it is only for display: the largest person.
        """
        people = self._pose.people() if self._pose is not None else []
        self._count = len(people)
        # A new camera image is recognised by its image time. Without people
        # the tracker keeps running on the clock, so old tracks expire.
        image = people[0].get("zeit", now) if people else now
        if image != self._tracker_image:
            self._tracker_image = image
            self._tracker.update(people, image, self._self_rotation())

        if self._mode in _DRIVE_MODES:
            if self._target_choice.state == IDLE:
                # Mode was set without _switch running (tests).
                self._target_choice.acquire(self._selected_id, now)
                self._selected_id = None
            track = self._target_choice.assign(self._tracker.tracks, self._tracker_image, now)
        else:
            measured = [t for t in self._tracker.tracks if t.measured]
            track = max(measured, key=lambda t: t.height) if measured else None
        return dict(track.person, id=track.id) if track is not None else None

    def _self_rotation(self) -> float:
        """Turn rate of the robot (rad/s): measured, otherwise the drive command."""
        if self._odometry is not None:
            return float(self._odometry.velocity()[2])
        command = getattr(self._watchdog, "current_command", None)
        return float(command.omega) if command is not None else 0.0

    def _forget_target(self) -> None:
        """Clear tracks and target."""
        self._tracker.clear()
        self._target_choice.release()
        self._tracker_image = None
        self._selected_id = None
        self._gestures_per_track.clear()
        self._gesture_display = (None, None)

    def _check_gesture(self, target: dict | None, now: float) -> None:
        if target is not None and "id" not in target:
            # Person without a track (direct call, e.g. from test_voice).
            self._check_gesture_single(target, now)
            return
        if self._count == 0:
            self._gestures_per_track.clear()
            return
        # Only evaluate new camera images -- the pose comes at ~10 Hz, this
        # loop runs at 20 Hz.
        image = self._tracker_image
        if image == self._last_frame:
            return
        self._last_frame = image

        # Each person has their own detection: when the selection used to
        # jump between two people, their arm movements got mixed up.
        alive = {t.id for t in self._tracker.tracks}
        for tid in list(self._gestures_per_track):
            if tid not in alive:
                del self._gestures_per_track[tid]
        still = self._robot_still()
        hits = []
        for t in self._tracker.tracks:
            if not t.measured:
                continue
            detection = self._gestures_per_track.setdefault(t.id, gs.GestureDetection())
            gesture = detection.update(t.person, image, still=still)
            if gesture is not None:
                hits.append((t.id, gesture))
        if not hits:
            return

        if self._mode in _DRIVE_MODES:
            # While driving only the target counts -- a passer-by must not send
            # it away. Stop counts from anyone: stopping is always safe.
            target_id = self._target_choice.id
            hits = [(tid, g) for tid, g in hits if g == gs.STOP or tid == target_id]
            if not hits:
                return
        # Stop takes precedence over everything else.
        tid, gesture = min(hits, key=lambda e: e[1] != gs.STOP)
        self._gesture_display = (gesture, image)
        if self._mode == READY and gesture != gs.STOP:
            # Whoever makes the gesture becomes the target (not the largest person).
            self._selected_id = tid
        self._execute_gesture(gesture, now)

    def _check_gesture_single(self, target: dict | None, now: float) -> None:
        """Old way: one detection for the selected person."""
        if target is None:
            if self._count == 0:
                self._gestures.update(None, now)
            return
        # Only evaluate new camera images -- the pose comes at ~10 Hz, this
        # loop runs at 20 Hz.
        if target["zeit"] == self._last_frame:
            return
        self._last_frame = target["zeit"]
        gesture = self._gestures.update(target, target["zeit"], still=self._robot_still())
        if gesture is None:
            return
        self._execute_gesture(gesture, now)

    def _execute_gesture(self, gesture: str, now: float) -> None:
        log.info("Interaction: gesture %s recognised", gesture)
        self._messages.append(f"Geste erkannt: {_GESTURE_TEXT[gesture]}")
        mode = _GESTURE_TO_MODE[gesture]
        texts = {
            READY: "angehalten -- wartet auf ein Handzeichen",
            FOLLOW: "folgt",
            COME_HERE: "kommt her",
            GO_AWAY: "fährt zurück",
        }
        self._switch(mode, texts[mode], now)
        if self._mode == mode:  # otherwise refused (no drive), _switch says so
            # Stop takes precedence: whatever was queued before is no longer valid.
            self._say(_GESTURE_SAY[gesture], priority=gesture == gs.STOP)

    def _smooth(self, target: dict, now: float) -> None:
        """Low-pass filter the camera angle -- once per new camera image.

        The pose box jumps by a few degrees from image to image; unfiltered
        this becomes a fidgety omega. After a gap the filter starts over,
        otherwise it would drag along an outdated angle.
        """
        if target["zeit"] == self._smoothed_frame:
            return
        self._smoothed_frame = target["zeit"]
        raw = target["winkel"]
        if self._smoothed_angle is None or now - self._seen > config.FOLLOW_HOLD_S:
            self._smoothed_angle = raw
        else:
            self._smoothed_angle += config.FOLLOW_ANGLE_SMOOTHING * (raw - self._smoothed_angle)

    def _robot_still(self) -> bool:
        """Is the robot (almost) standing still? Only then do moving gestures count.

        When it drives, the camera shakes and the person wanders through the
        image -- to the detection that looked like waving.
        """
        command = getattr(self._watchdog, "current_command", None)
        if command is None:
            return True
        return (
            abs(command.vx) <= config.GESTURE_ROBOT_STILL_V
            and abs(command.vy) <= config.GESTURE_ROBOT_STILL_V
            and abs(command.omega) <= config.GESTURE_ROBOT_STILL_W
        )

    def _distance_to(self, target: dict | None, scan=None, now: float | None = None) -> float | None:
        """Lidar distance to the person: their cluster within the box's angle.

        Previously the minimum within the box's angle range -- then a chair in
        front of the person counted as their distance. Now, among the clusters
        in that range, the one most likely to be the person is chosen:

        1. two human legs ("Mensch") -- the strongest shape in the lidar,
        2. matches the last good distance (jump < FOLLOW_CLUSTER_JUMP_M),
        3. thick single legs or objects (skirt, coat) before furniture legs,
        4. angle closest to the box centre.

        If no cluster is found (person farther than OBSTACLE_RANGE_M, at the
        wall, only one beam), the minimum applies as before. Safety does not
        depend on it: ``_approach_target`` always limits forward driving by
        the free driving corridor, so a chair in front still stops it.
        """
        if target is None:
            self._distance_source = None
            return None
        if scan is None:
            scan = self._lidar.latest() if self._lidar is not None else None
            if scan is None:
                self._distance_source = None
                return None
        fov = self._pose.fov_deg if self._pose is not None else config.CAMERA_FOV_DEG
        from_ = (0.5 - target["box"][2]) * fov
        until = (0.5 - target["box"][0]) * fov
        # Half the box width, but at least 3°: for people far away the box is
        # narrow, and a lidar beam would otherwise miss.
        center = (from_ + until) / 2
        half = max(3.0, (until - from_) / 2)

        margin = config.FOLLOW_CLUSTER_EDGE_DEG
        candidates = [
            h for h in self._obstacle_detection.current(scan)
            if h.category != hd.WALL and abs((h.angle - center + 180.0) % 360.0 - 180.0) <= half + margin
        ]
        if not candidates:
            self._distance_source = "Minimum"
            return scan.min_distance(center, half)

        good = None
        if now is not None and now - self._distance_good_time < config.FOLLOW_CLUSTER_GOOD_S:
            good = self._distance_good

        def rank(h):
            implausible = good is not None and abs(h.distance - good) > config.FOLLOW_CLUSTER_JUMP_M
            furniture = h.category in (hd.CHAIR, hd.TABLE, hd.LEG)
            angle = abs((h.angle - center + 180.0) % 360.0 - 180.0)
            return (h.category != hd.HUMAN, implausible, furniture, angle)

        best = min(candidates, key=rank)
        self._distance_source = f"Cluster: {best.category}"
        return best.distance

    @staticmethod
    def _near(points, rear_only: bool = False) -> bool:
        if len(points) == 0:
            return False
        r = (points[:, 0] ** 2 + points[:, 1] ** 2) ** 0.5
        close = r < config.EXPLORE_NEAR_M
        if rear_only:
            close &= points[:, 0] < 0.0
        return int(close.sum()) >= 3

    def _drive_available(self) -> bool:
        return self._drive_check is None or bool(self._drive_check())

    def _say(self, text: str, priority: bool = False) -> None:
        if self._voice is not None:
            self._voice.say(text, priority=priority)

    def _switch(
        self, mode: str, text: str, now: float, report: bool = False, say: str | None = None
    ) -> None:
        if mode in _DRIVE_MODES and not self._drive_available():
            # Without the board every command would go nowhere, and after
            # reconnecting it would suddenly drive off. So do not switch at
            # all, but stand still and let the user know.
            log.warning("Interaction: %s refused -- no connection to the drive", mode)
            mode, text, report = READY, "wartet auf den Antrieb", True
            self._messages.append("Keine Verbindung zum Antrieb -- fahre nicht los")
            say = "Ich habe keine Verbindung zu meinen Rädern."
        # Every mode change (gesture, stop, loss) ends a pause.
        self._paused = False
        if mode == READY and self._mode != READY:
            # Stop explicitly once when switching to READY -- after that the
            # loop sends nothing any more.
            self._watchdog.stop()
        if mode != self._mode:
            log.info("Interaction: %s -> %s (%s)", self._mode, mode, text)
            # Coasting only applies within one behavior.
            self._prev_command = None
        if mode in (FOLLOW, COME_HERE):
            # Deadline for getting lost starts now -- otherwise after a button
            # press it searches immediately if the person is not in view.
            self._seen = now
        # Keep the target choice in step. A new target only comes when
        # starting from standstill, never when switching between driving modes.
        if mode == SEARCHING:
            self._target_choice.search(now)
            # The search begins where the person was last.
            self._search_direction = self._angle or 0.0
            self._hint_in_view_since = None
            self._hint_off = False
        elif mode == READY:
            if self._target_choice.state != IDLE:
                self._target_choice.release()
        elif mode in _DRIVE_MODES and self._target_choice.state == IDLE:
            self._target_choice.acquire(self._selected_id, now)
            self._selected_id = None
        self._mode = mode
        self._mode_since = now
        self._text = text
        if report:
            self._messages.append(text)
        if say:
            self._say(say)
        return None
