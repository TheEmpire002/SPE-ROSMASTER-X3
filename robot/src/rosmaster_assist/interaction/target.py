"""Whom does it follow? Track people across frames without telling them apart.

Two blocks, both testable without hardware:

- ``PersonTracker`` assigns the people in each frame to each other. Per
  person, a small Kalman filter predicts the camera angle and removes the
  robot's own rotation (board turn rate, not ``theta``: for a long time the
  angle came from the frozen IMU). The track numbers are internal only:
  they keep the gestures of two people apart and keep following calm when
  someone walks past.
- ``TargetChoice`` says which track it follows. If the person is gone and
  comes back, it keeps following them, no matter which number they have now.

Until 04.11. the target was additionally re-identified by its appearance
(target lock), and people were called "Person 1, 2, ..." in the app. Whoever
left the frame and came back was often taken for a stranger, and the robot
stood still. That is why it was removed.

Everything stays in RAM and is deleted on switch-off.
"""

from __future__ import annotations

import math

from .. import config

# States of the target choice (the values go to the app as-is, see docs/api.md).
IDLE = "idle"
ACQUIRING = "acquiring_target"
FOLLOWING = "following"
TEMPORARILY_LOST = "target_temporarily_lost"
SEARCHING_TARGET = "searching_for_target"

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


class TargetChoice:
    """Whom it follows: the person in the frame -- without telling people apart.

    It stays with the same track as long as that track is measured;
    otherwise it would jump back and forth between two people in the same
    frame. If the track is gone, after ``TARGET_SWITCH_S`` it takes the
    person closest to the last angle -- even if the tracker has given them
    a new number. That is intended: whoever briefly leaves the frame and
    comes back is a new track for the tracker, but the same person.

    Until 04.11. there was a target lock here that had to re-identify the
    target by its appearance. If that failed (light, turning), the returning
    person counted as a stranger and the robot stood still.
    """

    def __init__(self) -> None:
        self.release()

    def release(self, reason: str = "") -> None:
        """Stop following."""
        self.state = IDLE
        self.id = None
        self.reason = reason
        self.confidence = 0.0
        self._angle: float | None = None
        self._gone_since: float | None = None

    def acquire(self, track_id, now: float) -> None:
        """Start following: the track that made the gesture, or (None) the largest person."""
        self.release()
        self.state = ACQUIRING
        self.id = track_id

    def search(self, now: float) -> None:
        """Person lost: the next one to appear is the target."""
        if self.state != IDLE:
            self.state = SEARCHING_TARGET

    def as_dict(self) -> dict:
        return {
            "zustand": self.state,
            "id": self.id,
            "sicherheit": round(self.confidence, 2),
            "grund": self.reason,
        }

    def assign(self, tracks: list[Track], image_time, now: float) -> Track | None:
        """The track it follows -- only if it was measured in the last frame."""
        if self.state == IDLE:
            return None
        measured = [t for t in tracks if t.measured]
        target = next((t for t in measured if t.id == self.id), None)
        if target is None and measured:
            # Briefly occluded (someone walking past)? Only take another
            # person after TARGET_SWITCH_S. At start and while searching, at once.
            gone_long = self._gone_since is not None and now - self._gone_since >= config.TARGET_SWITCH_S
            if self.id is None or self.state in (ACQUIRING, SEARCHING_TARGET) or gone_long:
                target = self._next_no(measured)
        if target is None:
            if self._gone_since is None:
                self._gone_since = now
            if self.state == FOLLOWING:
                self.state = TEMPORARILY_LOST
            return None
        self._gone_since = None
        self.id = target.id
        self._angle = target.w
        self.state = FOLLOWING
        self.confidence = 1.0
        return target

    def _next_no(self, measured: list[Track]) -> Track:
        """Where it last saw someone; without a clue, the largest person."""
        if self._angle is None:
            return max(measured, key=lambda t: t.height)
        return min(measured, key=lambda t: (abs(t.w - self._angle), -t.height))
