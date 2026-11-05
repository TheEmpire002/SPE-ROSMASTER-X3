"""Recognize hand signals from the body pose.

Four gestures (version 1, see docs/interaction.md):

``FOLLOW``     one arm above the head, held
``STOP``       both hands above the shoulders, held
``COME_HERE``  one hand waves up and down in front of the body
``GO_AWAY``    one hand swipes sideways away from the body

All measures are relative to the person's **shoulder width**, not in image
pixels: whether someone stands 1 m or 3 m away changes the pixels, but not
the ratio of hand to shoulders. Likewise everything is relative to the
shoulder center -- if the person moves sideways while waving, the hand
moves in the image, but not relative to the body.

Against false triggers: a held gesture only counts once it was visible in
most frames over ``GESTURE_HOLD_S``; moving gestures need several
direction changes or a long distance. After every detected gesture there is
a pause; only STOP always works.

Moving gestures are additionally debounced because the arms swing while
walking (looked like waving/swiping while following). They only count if
the person is standing in place (size and shoulder center barely change),
their face is visible (not from behind), the robot itself is standing still
and the gesture was detected in ``GESTURE_CONFIRM`` consecutive evaluations.

The module knows neither the camera nor Hailo -- it receives person dicts
from ``sensors/pose.py`` and can thus be tested without hardware
(``Marlon/interaction/test_gestures.py``).
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

from .. import config

FOLLOW = "folgen"
STOP = "stopp"
COME_HERE = "herkommen"
GO_AWAY = "weggehen"

# Indices of the COCO keypoints (as in sensors/pose.py; repeated here so
# this module stays importable without numpy/Hailo).
_NOSE = 0
_SHOULDER = (5, 6)
_HAND = (9, 10)
_HIP = (11, 12)

#: Without a detected hip: roughly this far (in shoulder widths) below the
#: shoulders it lies for an adult.
_HIP_FALLBACK = 1.8

#: Without a detected nose: roughly this far above the shoulders is the head.
_HEAD_FALLBACK = 0.6

#: The shoulder width is smoothed over this many frames (~1 s at 10 Hz).
_WIDTH_COUNT = 9


@dataclass
class BodyPose:
    """A pose converted into body measures (unit: shoulder width).

    u to the right in the image, v downwards, origin at the shoulder center.
    ``hands`` holds (u, v) for the left/right hand, or None if uncertain.
    """

    timestamp: float
    posture: str | None
    hands: tuple[tuple[float, float] | None, tuple[float, float] | None]
    head_v: float
    hip_v: float
    # For debouncing: shoulder center and measured (not smoothed) shoulder
    # width in image units, whether the face is visible and whether the
    # robot was standing still for this frame.
    center: tuple[float, float] | None = None
    width: float | None = None
    frontal: bool = False
    still: bool = True


def _read_point(person: dict):
    """Returns point(i) -> (x, y) in equal-length units, or None."""
    kp = person.get("kp") or []
    if len(kp) < 17:
        return None
    limit = config.POSE_KP_MIN_CONF
    # Stretch x by the aspect ratio so that a step in x is as long as one
    # in y (the coordinates are relative to the image width and height,
    # respectively).
    sv = float(person.get("seitenverhaeltnis") or 1.0)

    def point(i: int) -> tuple[float, float] | None:
        x, y, c = kp[i]
        return (x * sv, y) if c >= limit else None

    return point


def shoulder_width(person: dict) -> float | None:
    """Distance between the shoulders in image units, None if uncertain."""
    point = _read_point(person)
    if point is None:
        return None
    sl, sr = point(_SHOULDER[0]), point(_SHOULDER[1])
    if sl is None or sr is None:
        return None
    width = math.hypot(sl[0] - sr[0], sl[1] - sr[1])
    return width if width >= 1e-3 else None  # sideways to the camera -- unusable


def body_measure(person: dict, timestamp: float, width: float | None = None) -> BodyPose | None:
    """Person dict -> BodyPose. None if the shoulders are not certain.

    Without both shoulders there is no scale -- better to detect nothing
    than to work with the wrong size. ``width`` replaces the measured
    shoulder width with a smoothed value (see GestureDetection).
    """
    measured = shoulder_width(person)
    if measured is None:
        return None
    width = width or measured
    point = _read_point(person)
    sl, sr = point(_SHOULDER[0]), point(_SHOULDER[1])
    mx, my = (sl[0] + sr[0]) / 2, (sl[1] + sr[1]) / 2

    def rel(p: tuple[float, float] | None) -> tuple[float, float] | None:
        return None if p is None else ((p[0] - mx) / width, (p[1] - my) / width)

    hands = (rel(point(_HAND[0])), rel(point(_HAND[1])))
    nose = rel(point(_NOSE))
    head_v = nose[1] if nose is not None else -_HEAD_FALLBACK
    hips = [h for h in (rel(point(_HIP[0])), rel(point(_HIP[1]))) if h is not None]
    hip_v = sum(h[1] for h in hips) / len(hips) if hips else _HIP_FALLBACK

    # Only the nose, not the order of the shoulders: the camera may be
    # mirrored via CAMERA_ROLES, but a nose stays a nose either way.
    frontal = nose is not None
    return BodyPose(
        timestamp, posture(hands, head_v), hands, head_v, hip_v,
        center=(mx, my), width=measured, frontal=frontal,
    )


def posture(hands, head_v: float) -> str | None:
    """Held gesture in a single frame (STOP, FOLLOW or None)."""
    # v < 0 means: above the shoulder line (y points down).
    above_shoulder = [h is not None and h[1] < 0.0 for h in hands]
    above_head = [h is not None and h[1] < head_v for h in hands]
    if all(above_shoulder):
        return STOP
    # Exactly one arm above the head, the other down -- both up is STOP.
    if sum(above_head) == 1:
        return FOLLOW
    return None


def direction_changes(values: list[float], hub: float) -> int:
    """How often a sequence changes direction, jitter below ``hub`` not counted.

    Only once the value has moved at least ``hub`` away from the last
    turning point does the new direction count -- so keypoint noise does
    not turn into waving.
    """
    if not values:
        return 0
    direction = 0
    extreme = values[0]
    reversals = 0
    for w in values[1:]:
        if direction == 0:
            if abs(w - extreme) >= hub:
                direction = 1 if w > extreme else -1
                extreme = w
        elif direction > 0:
            if w > extreme:
                extreme = w
            elif extreme - w >= hub:
                direction, extreme, reversals = -1, w, reversals + 1
        else:
            if w < extreme:
                extreme = w
            elif w - extreme >= hub:
                direction, extreme, reversals = 1, w, reversals + 1
    return reversals


def waves(images: list[BodyPose], hand: int) -> bool:
    """COME_HERE: hand in front of the upper body, up and down several times."""
    samples = [
        b.hands[hand] for b in images
        if b.hands[hand] is not None and b.head_v < b.hands[hand][1] < b.hip_v
    ]
    # The hand must be visible for almost the whole window, otherwise gaps
    # create apparent jumps.
    if len(samples) < 0.6 * len(images) or len(samples) < 6:
        return False
    u = [p[0] for p in samples]
    v = [p[1] for p in samples]
    # Mostly vertical -- side-to-side motion is a hello wave.
    if max(u) - min(u) > max(v) - min(v):
        return False
    return direction_changes(v, config.GESTURE_WAVE_AMPLITUDE) >= config.GESTURE_WAVE_REVERSALS


def swipes(images: list[BodyPose], hand: int) -> bool:
    """GO_AWAY: hand moves far sideways away from the body."""
    samples = [
        b.hands[hand] for b in images
        if b.hands[hand] is not None and b.hands[hand][1] < b.hip_v
    ]
    if len(samples) < 4:
        return False
    # Largest increase of the lateral distance from earlier to later
    # (running minimum), with little up/down motion -- otherwise it is an
    # arm being raised sideways.
    segment = config.GESTURE_SWIPE_DISTANCE
    smallest = samples[0]
    for p in samples[1:]:
        if abs(p[0]) - abs(smallest[0]) >= segment:
            if abs(p[1] - smallest[1]) < 0.6 * (abs(p[0]) - abs(smallest[0])):
                return True
        if abs(p[0]) < abs(smallest[0]):
            smallest = p
    return False


class GestureDetection:
    """Remembers a person's recent poses and reports confirmed gestures."""

    def __init__(self) -> None:
        self._frame_buffer: deque[BodyPose] = deque()
        self._widths: deque[float] = deque(maxlen=_WIDTH_COUNT)
        self._angle: float | None = None
        self._previous: str | None = None
        self._last_time = -1e9
        self.last: str | None = None  # for the display
        self.last_time: float | None = None
        # Moving gesture of the recent evaluations (candidate, how many times in a row)
        self._candidate: str | None = None
        self._candidate_count = 0

    def reset(self) -> None:
        self._frame_buffer.clear()
        self._widths.clear()
        self._angle = None
        self._candidate, self._candidate_count = None, 0

    def update(self, person: dict | None, timestamp: float, still: bool = True) -> str | None:
        """New pose of the tracked person. Returns a confirmed gesture.

        Call only once per new camera frame -- the same frame twice would
        distort hold durations. ``still``: the robot was standing still for
        this frame (while it drives, only STOP and FOLLOW count).
        """
        if person is None:
            self.reset()
            return None
        angle = person.get("winkel")
        if (
            self._angle is not None and angle is not None
            and abs(angle - self._angle) > config.INTERACTION_JUMP_DEG
        ):
            # Large jump: probably a different person -- their arms must not
            # be mixed up with the arms of the previous one.
            self._frame_buffer.clear()
            self._widths.clear()
            self._candidate, self._candidate_count = None, 0
        self._angle = angle

        # The shoulder width is the scale for everything. If the shoulders
        # jitter from frame to frame, every hand would appear to move along
        # (a steady hand 1.4 widths below the shoulders "waves" by 0.4 with
        # 15 % jitter of the width) -- hence the median of the recent frames.
        measured = shoulder_width(person)
        if measured is not None:
            self._widths.append(measured)
        width = statistics.median(self._widths) if self._widths else None
        image = body_measure(person, timestamp, width)
        if image is None:
            # An incomplete frame counts as "no gesture" so that it interrupts
            # the hold duration instead of being skipped.
            image = BodyPose(timestamp, None, (None, None), -_HEAD_FALLBACK, _HIP_FALLBACK)
        image.still = still
        self._frame_buffer.append(image)
        while self._frame_buffer and timestamp - self._frame_buffer[0].timestamp > config.GESTURE_WINDOW_S:
            self._frame_buffer.popleft()

        gesture = self._detect(timestamp)
        if gesture is None:
            return None
        # Pause after every gesture so that one motion does not count twice.
        # STOP always works -- unless it was just detected.
        in_pause = timestamp - self._last_time < config.GESTURE_PAUSE_S
        if in_pause and (gesture != STOP or self._previous == STOP):
            return None
        self._previous, self._last_time = gesture, timestamp
        self.last, self.last_time = gesture, timestamp
        self._frame_buffer.clear()
        self._candidate, self._candidate_count = None, 0
        return gesture

    def _detect(self, timestamp: float) -> str | None:
        images = list(self._frame_buffer)
        # Safety first: STOP is checked before anything else.
        if self._held(images, timestamp, STOP, config.GESTURE_STOP_HOLD_S):
            return STOP
        if self._held(images, timestamp, FOLLOW, config.GESTURE_HOLD_S):
            return FOLLOW
        # Moving gestures need an almost full window.
        candidate = None
        if (
            images and timestamp - images[0].timestamp >= 0.7 * config.GESTURE_WINDOW_S
            and self._is_still(images)
        ):
            if any(waves(images, h) for h in (0, 1)):
                candidate = COME_HERE
            elif any(swipes(images, h) for h in (0, 1)):
                candidate = GO_AWAY
        # Only report once several consecutive evaluations see the same
        # thing -- a single window that happens to match is not enough.
        if candidate is not None and candidate == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = candidate
            self._candidate_count = 1 if candidate is not None else 0
        if candidate is not None and self._candidate_count >= config.GESTURE_CONFIRM:
            return candidate
        return None

    @staticmethod
    def _is_still(images: list[BodyPose]) -> bool:
        """Calm scene for moving gestures: robot still, person still, facing us."""
        share = config.GESTURE_SHARE
        if sum(b.still for b in images) < share * len(images):
            return False
        if sum(b.frontal for b in images) < share * len(images):
            return False
        good = [b for b in images if b.center is not None and b.width]
        if len(good) < 6:
            return False
        # Median of the first and last third -- single keypoint outliers
        # should not count as walking.
        n = max(2, len(good) // 3)
        head, end = good[:n], good[-n:]
        b0 = statistics.median(b.width for b in head)
        b1 = statistics.median(b.width for b in end)
        if max(b0, b1) > config.GESTURE_STILL_SIZE * min(b0, b1):
            return False  # approaching or moving away
        x0 = statistics.median(b.center[0] for b in head)
        y0 = statistics.median(b.center[1] for b in head)
        x1 = statistics.median(b.center[0] for b in end)
        y1 = statistics.median(b.center[1] for b in end)
        path = math.hypot(x1 - x0, y1 - y0) / statistics.median(b.width for b in good)
        return path <= config.GESTURE_STILL_DRIFT  # otherwise the person is walking sideways

    @staticmethod
    def _held(images: list[BodyPose], timestamp: float, gesture: str, duration: float) -> bool:
        part = [b for b in images if timestamp - b.timestamp <= duration]
        # The oldest frame must be far enough back, otherwise the hold
        # duration has not even been reached yet.
        if len(part) < 3 or timestamp - part[0].timestamp < 0.8 * duration:
            return False
        hits = sum(1 for b in part if b.posture == gesture)
        return hits >= config.GESTURE_SHARE * len(part)
