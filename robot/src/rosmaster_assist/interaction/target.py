"""Target lock while following: track people across frames, re-identify the target.

Following used to pick, in every frame, the person whose angle was closest
to the last one -- and after a search simply the largest. If someone
crossed, or the target was not the first to come back after an occlusion,
the robot followed the wrong person. This module holds two blocks, both
testable without hardware:

- ``PersonTracker`` gives every person an ID that persists across frames.
  Per track, a small Kalman filter predicts the camera angle and removes
  the robot's own rotation (board turn rate, not ``theta``: for a long time
  the angle came from the frozen IMU).
- ``TargetLock`` remembers which track is the target, and the look of the
  upper body (HSV histogram from ``sensors/pose.py``). After an occlusion,
  only someone who matches clearly better than everyone else over several
  frames counts as the target again. When in doubt it stops instead of
  guessing.

Everything stays in RAM and is deleted on release: no images, only a
histogram with 128 numbers per person.
"""

from __future__ import annotations

import math

from .. import config

# States of the target lock (the values go to the app as-is, see docs/api.md).
IDLE = "idle"
ACQUIRING = "acquiring_target"
FOLLOWING = "following"
TEMPORARILY_LOST = "target_temporarily_lost"
SEARCHING_TARGET = "searching_for_target"
SAFE_STOP = "safe_stop"

# Kalman model of the camera angle (degrees). Not settings but properties
# of the measurement: pose detection hits the box centre to about 1.5°; a
# walking person rarely changes their angle faster than 60°/s (at 1 m
# distance ~1 m/s sideways).
_MEAS_STD = 1.5
_ACCEL_STD = 60.0
_V_START_STD = 30.0
_V_MAX = 60.0
# Longest time span for one prediction: after a long pause (pose detection
# stalled) the uncertainty should not grow towards infinity.
_DT_MAX = 0.5
# How strongly box height and track feature are updated per measurement.
_HEIGHT_SMOOTHING = 0.3
_FEATURE_SMOOTHING = 0.3


def bhattacharyya(p, q) -> float | None:
    """Distance between two normalized histograms: 0 = equal, 1 = nothing in common.

    None if one is missing -- then nothing can be said about the look
    (e.g. torso not in the frame), and that must not count as a "match".
    """
    if not p or not q or len(p) != len(q):
        return None
    total = sum(math.sqrt(max(a, 0.0) * max(b, 0.0)) for a, b in zip(p, q))
    return math.sqrt(max(0.0, 1.0 - total))


def _height(person: dict) -> float:
    box = person["box"]
    return max(box[3] - box[1], 1e-3)


def _blend(old: list | None, new: list | None, share: float) -> list | None:
    if new is None:
        return old
    if old is None or len(old) != len(new):
        return list(new)
    return [a + share * (b - a) for a, b in zip(old, new)]


class Track:
    """One person across several frames: angle, angular velocity, look."""

    def __init__(self, track_id: int, person: dict, timestamp: float) -> None:
        self.id = track_id
        self.w = float(person["winkel"])
        self.v = 0.0
        # Covariance as (P00, P01, P10, P11) -- 2x2 is not worth numpy.
        self.p = (_MEAS_STD ** 2, 0.0, 0.0, _V_START_STD ** 2)
        self.height = _height(person)
        self.feature = _blend(None, person.get("merkmal"), 1.0)
        self.person = person
        self.timestamp = timestamp
        self.measured = True

    def predict(self, dt: float, omega: float) -> None:
        """Constant angular velocity; remove the robot's own rotation.

        If the robot turns counter-clockwise at omega, every person in the
        frame moves right by omega*dt (smaller angle).
        """
        if dt <= 0:
            return
        self.w += self.v * dt - math.degrees(omega) * dt
        p00, p01, p10, p11 = self.p
        q = _ACCEL_STD ** 2
        self.p = (
            p00 + dt * (p01 + p10) + dt * dt * p11 + q * dt ** 4 / 4,
            p01 + dt * p11 + q * dt ** 3 / 2,
            p10 + dt * p11 + q * dt ** 3 / 2,
            p11 + q * dt * dt,
        )

    def gate(self) -> float:
        """Association gate: at least TARGET_GATE_DEG, grows with the uncertainty."""
        g = config.TARGET_GATE_DEG
        return min(max(g, 3.0 * math.sqrt(self.p[0] + _MEAS_STD ** 2)), 3.0 * g)

    def measure(self, person: dict, timestamp: float) -> None:
        p00, p01, p10, p11 = self.p
        s = p00 + _MEAS_STD ** 2
        k0, k1 = p00 / s, p10 / s
        deviation = float(person["winkel"]) - self.w
        self.w += k0 * deviation
        self.v = max(-_V_MAX, min(_V_MAX, self.v + k1 * deviation))
        self.p = ((1 - k0) * p00, (1 - k0) * p01, p10 - k1 * p00, p11 - k1 * p01)
        self.height += _HEIGHT_SMOOTHING * (_height(person) - self.height)
        self.feature = _blend(self.feature, person.get("merkmal"), _FEATURE_SMOOTHING)
        self.person = person
        self.timestamp = timestamp
        self.measured = True


class PersonTracker:
    """Assigns the people in each frame to the known tracks."""

    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.tracks: list[Track] = []
        self._next_id = 1
        self._time: float | None = None

    def track(self, track_id) -> Track | None:
        return next((t for t in self.tracks if t.id == track_id), None)

    def update(self, people: list[dict], timestamp: float, omega: float = 0.0) -> list[Track]:
        """Process one frame. ``omega``: measured turn rate of the robot (rad/s)."""
        dt = 0.0 if self._time is None else min(max(timestamp - self._time, 0.0), _DT_MAX)
        self._time = timestamp if self._time is None else max(self._time, timestamp)
        for t in self.tracks:
            t.predict(dt, omega)
            t.measured = False

        # Greedy assignment by cost: with two or three people that is enough,
        # the Hungarian method would only add another dependency.
        pairs = []
        for i, t in enumerate(self.tracks):
            gate = t.gate()
            for j, p in enumerate(people):
                deviation = abs(float(p["winkel"]) - t.w)
                if deviation > gate:
                    continue
                h = _height(p)
                ratio = min(h, t.height) / max(h, t.height)
                if ratio < config.TARGET_HEIGHT_RATIO:
                    continue
                d = bhattacharyya(t.feature, p.get("merkmal"))
                cost = deviation / gate + 0.5 * abs(math.log(ratio)) + (d or 0.0)
                pairs.append((cost, i, j))
        pairs.sort()
        taken_t, taken_p = set(), set()
        for _, i, j in pairs:
            if i in taken_t or j in taken_p:
                continue
            taken_t.add(i)
            taken_p.add(j)
            self.tracks[i].measure(people[j], timestamp)

        for j, p in enumerate(people):
            if j not in taken_p:
                self.tracks.append(Track(self._next_id, p, timestamp))
                self._next_id += 1

        self.tracks = [t for t in self.tracks if timestamp - t.timestamp <= config.TARGET_TRACK_HOLD_S]
        return self.tracks


class TargetLock:
    """Which track is the target? State machine, see docs/interaction.md."""

    def __init__(self) -> None:
        # Look of the person who last triggered following or was being
        # followed. Survives release and safe stop, so that on the next
        # "follow" it takes the same person again instead of refusing when
        # several are in the frame. Only in RAM, deleted on forget() and
        # after TARGET_REMEMBER_S.
        self.remembered: list | None = None
        self._remembered_time = 0.0
        self.release()

    # --- Transitions from outside ----------------------------------------------

    def forget(self) -> None:
        """Release by the human: also delete the remembered person."""
        self.remembered = None
        self.release()

    def release(self, reason: str = "") -> None:
        """Release the target and discard what was learned (the remembered person stays)."""
        self.state = IDLE
        self.id = None
        self.reference: list | None = None
        self.reason = reason
        self.confidence = 0.0
        self.ambiguous = False
        self._never_locked = True
        self._samples = 0
        self._since = 0.0
        self._misses = 0
        self._last_frame = None
        self._unclear_images = 0
        self._new_candidate()

    def acquire(self, track_id, now: float, feature: list | None = None) -> None:
        """Acquire a new target: a specific ID or (None) the only person.

        ``feature`` is the look of the chosen person at this moment
        (tap/gesture). That way it knows them immediately -- if the tracker
        briefly loses them during the learning phase, it finds exactly them
        again instead of choosing anew.
        """
        self.release()
        self.state = ACQUIRING
        self.id = track_id
        self._never_locked = track_id is None
        self._since = now
        if track_id is not None and feature is not None:
            self.reference = list(feature)
            self._samples = 1
            self._remember(now)

    def search(self, now: float) -> None:
        if self.state in (IDLE, SAFE_STOP):
            return
        self.state = SEARCHING_TARGET
        self._since = now
        self._new_candidate()

    def safe_stop(self, reason: str, now: float) -> None:
        """Stand still until someone chooses again. What was learned is deleted."""
        self.release(reason)
        self.state = SAFE_STOP
        self._since = now

    def as_dict(self) -> dict:
        return {
            "zustand": self.state,
            "id": self.id,
            "sicherheit": round(self.confidence, 2),
            "grund": self.reason,
        }

    @property
    def checking_candidate(self) -> bool:
        """While searching, someone matches and is being confirmed -- hold still
        then, otherwise the robot turns the person out of the frame again."""
        return self._candidate is not None and self.state in (TEMPORARILY_LOST, SEARCHING_TARGET)

    # --- per tick ---------------------------------------------------------------

    def assign(self, tracks: list[Track], image_time, now: float) -> Track | None:
        """Return the target track -- only if it was measured in this frame.

        Called in the control tick; counting (confirmations, miss frames)
        only happens on a new frame (``image_time`` different from last time).
        """
        self.ambiguous = False
        if self.state in (IDLE, SAFE_STOP):
            return None
        new = image_time is not None and image_time != self._last_frame
        self._last_frame = image_time
        measured = [t for t in tracks if t.measured]

        if self.state == ACQUIRING and self.id is None:
            # Without a tap only if the choice is unambiguous -- or if it
            # remembered the person from the last follow and confidently
            # recognizes them among the visible ones.
            if len(measured) > 1:
                if not self._remembered_valid(now):
                    self.ambiguous = True
                    return None
                best = self._choose_remembered(measured, new)
                if best is None:
                    return None
                self.id = best.id
                self.reference = list(self.remembered)
                self._samples = 1
            elif not measured:
                return None
            else:
                self.id = measured[0].id
            self._never_locked = False

        target = next((t for t in tracks if t.id == self.id), None) if self.id is not None else None
        if target is not None and target.measured:
            return self._target_measured(target, measured, new, now)

        # Target without measurement (occluded, out of frame, track dropped)
        if self.state == ACQUIRING:
            if target is not None:
                return None
            if self.reference is None:
                # Nothing learned yet: like a fresh start.
                self.id = None
                self._never_locked = True
                return None
        if self.state in (ACQUIRING, FOLLOWING):
            self.state = TEMPORARILY_LOST
            self._since = now
        return self._reacquire(measured, new, now)

    # --- internal ---------------------------------------------------------------

    def _new_candidate(self) -> None:
        self._candidate = None
        self._candidate_frames = 0
        self._narrow_frames = 0

    def _remember(self, now: float) -> None:
        self.remembered = list(self.reference) if self.reference is not None else None
        self._remembered_time = now

    def _remembered_valid(self, now: float) -> bool:
        return self.remembered is not None and now - self._remembered_time <= config.TARGET_REMEMBER_S

    def _choose_remembered(self, measured: list[Track], new: bool) -> Track | None:
        """Confirm the remembered person among several visible ones."""
        hits = self._compare(self.remembered, measured, new)
        if hits is None:
            # Nobody matches (different clothes, different light): after a
            # short time refuse as before, so that someone taps in the image.
            if new:
                self._unclear_images += 1
            if self._unclear_images >= config.TARGET_CONFIRM_FRAMES:
                self.ambiguous = True
            return None
        self._unclear_images = 0
        if not self._confirm(*hits, new):
            return None
        self._new_candidate()
        return hits[0]

    def _compare(self, reference: list | None, measured: list[Track], new: bool) -> tuple[Track, bool] | None:
        """Best candidate for the feature and whether it clearly matches -- or None.

        Clearly means: well below the threshold and far ahead of the second
        best. Then fewer confirmation frames are enough.
        """
        scored = sorted(
            (d, t.id, t)
            for t in measured
            if (d := bhattacharyya(reference, t.person.get("merkmal"))) is not None
        )
        if not scored or scored[0][0] > config.TARGET_FEATURE_THRESHOLD:
            self._new_candidate()
            return None
        if len(scored) > 1 and scored[1][0] - scored[0][0] < config.TARGET_FEATURE_MARGIN:
            # Two match almost equally well (same clothes?): do not guess.
            if new:
                self._narrow_frames += 1
            if self._narrow_frames >= config.TARGET_CONFIRM_FRAMES:
                self.ambiguous = True
            return None
        self._narrow_frames = 0
        d0 = scored[0][0]
        self.confidence = 1.0 - d0
        clear = d0 < config.TARGET_FEATURE_THRESHOLD / 2 and (
            len(scored) == 1 or scored[1][0] - d0 >= 2 * config.TARGET_FEATURE_MARGIN
        )
        return scored[0][2], clear

    def _confirm(self, best: Track, clear: bool, new: bool) -> bool:
        """Counts consecutive frames for the same candidate; True when enough."""
        if self._candidate != best.id:
            self._candidate = best.id
            self._candidate_frames = 1 if new else 0
        elif new:
            self._candidate_frames += 1
        needed = config.TARGET_CONFIRM_FAST_FRAMES if clear else config.TARGET_CONFIRM_FRAMES
        return self._candidate_frames >= needed

    def _still_learning(self) -> bool:
        # If the target was lost during the learning phase, the look is
        # still thinly sampled -- after finding it again keep learning
        # instead of counting as confirmed right away.
        return self.reference is None or self._samples < config.TARGET_CONFIRM_FRAMES

    def _target_measured(self, target: Track, measured: list[Track], new: bool, now: float) -> Track | None:
        feature = target.person.get("merkmal")
        d = bhattacharyya(self.reference, feature)
        if d is not None and d > config.TARGET_FEATURE_THRESHOLD and self.state != ACQUIRING:
            # The tracker has probably put another person on this ID
            # (e.g. when crossing). Do not follow; if it persists, the ID is
            # given up and a new search starts.
            if new:
                self._misses += 1
            if self._misses >= config.TARGET_CONFIRM_FRAMES:
                self.id = None
                self._misses = 0
                if self.state == FOLLOWING:
                    self.state = TEMPORARILY_LOST
                    self._since = now
            return None
        self._misses = 0
        self._new_candidate()
        # If someone stands close by, the histogram could contain parts of
        # the other person -- then learn nothing.
        unambiguous = not any(
            abs(t.w - target.w) < 2 * config.TARGET_GATE_DEG for t in measured if t is not target
        )
        if self.state == ACQUIRING:
            if new and unambiguous and feature is not None:
                self._samples += 1
                self.reference = _blend(self.reference, feature, 1.0 / self._samples)
            if now - self._since >= config.TARGET_ACQUIRE_S and (
                self._samples >= config.TARGET_CONFIRM_FRAMES or feature is None
            ):
                self.state = FOLLOWING
        elif self.state in (TEMPORARILY_LOST, SEARCHING_TARGET):
            self.state = ACQUIRING if self._still_learning() else FOLLOWING
        elif new and unambiguous and d is not None and d < config.TARGET_FEATURE_THRESHOLD / 2:
            self.reference = _blend(self.reference, feature, config.TARGET_FEATURE_UPDATE)
        self.confidence = 1.0 - d if d is not None else 0.5
        if self.reference is not None:
            self._remember(now)
        return target

    def _reacquire(self, measured: list[Track], new: bool, now: float) -> Track | None:
        """Re-identify the target among the visible people -- only when confident."""
        if not measured:
            self._new_candidate()
            return None

        if self.reference is None:
            # Without a learned look never take over someone else. Only
            # exception: nobody was ever locked (start without a person in
            # the frame) and exactly one person appears steadily.
            if not (self.state == SEARCHING_TARGET and self._never_locked):
                return None
            if len(measured) > 1:
                self.ambiguous = True
                return None
            best, clear = measured[0], False
        else:
            hits = self._compare(self.reference, measured, new)
            if hits is None:
                return None
            best, clear = hits

        if not self._confirm(best, clear, new):
            return None

        self.id = best.id
        self._misses = 0
        self._new_candidate()
        if self.reference is None:
            # Very first lock: learn the look now.
            self._never_locked = False
            self.state = ACQUIRING
            self._since = now
        else:
            self.state = ACQUIRING if self._still_learning() else FOLLOWING
        return best
