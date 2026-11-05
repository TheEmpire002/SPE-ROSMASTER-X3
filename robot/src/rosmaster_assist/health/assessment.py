"""The four signs of the health check -- pure computation, no hardware.

Each sign is a dict that the web app shows directly as a traffic light::

    {"name": "liegend", "titel": "Liegende Haltung",
     "zustand": "auffaellig" | "ok" | "unbekannt",
     "wert": 12.0, "text": "Rumpf 12° gegen die Waagerechte -- liegt",
     "grenze": 35.0, "einheit": "°"}

"unbekannt" means measurements are missing (no microphone, no keypoints,
no lidar). It counts neither for nor against an emergency, so that a
failed sensor alone can neither trigger an alarm nor prevent one.

``grenze`` (threshold) and ``einheit`` (unit of ``wert``) were added on the
English code base so a check can be reviewed later without the robot.
"""

from __future__ import annotations

import math
import re

import numpy as np

from .. import config
from ..sensors.pose import KP_HIP_L, KP_HIP_R, KP_SHOULDER_L, KP_SHOULDER_R

FLAGGED = "auffaellig"
OK = "ok"
UNKNOWN = "unbekannt"

#: order of the signs in the status and the web app
SIGNS = ("liegend", "regungslos", "antwort", "boden")
TITLES = {
    "liegend": "Liegende Haltung",
    "regungslos": "Regungslosigkeit",
    "antwort": "Keine Antwort",
    "boden": "Lage am Boden",
}

# two lidar points belong to the same object if they are closer than this;
# legs of a standing person (>= 15 cm gap) stay apart, a lying body is one piece
_GAP_M = 0.12
# extra margin (degrees) on the person's camera angle: camera and lidar
# are not mounted at the same spot
_ANGLE_MARGIN_DEG = 5.0


def sign(name: str, state: str, value, text: str, threshold=None, unit: str | None = None) -> dict:
    return {"name": name, "titel": TITLES[name], "zustand": state, "wert": value, "text": text,
            "grenze": threshold, "einheit": unit}


def empty() -> dict[str, dict]:
    """All signs still unchecked (start of a check)."""
    return {n: sign(n, UNKNOWN, None, "noch nicht geprüft") for n in SIGNS}


def _kp_px(person: dict, index: int) -> tuple[float, float] | None:
    """Keypoint in pixels of the 640 image, or None if too uncertain.

    In pixels because the image coordinates 0..1 have different lengths in
    x and y -- an angle computed from them would be distorted.
    """
    kp = person.get("kp") or []
    if index >= len(kp) or kp[index][2] < config.POSE_KP_MIN_CONF:
        return None
    return kp[index][0] * config.CAMERA_WIDTH, kp[index][1] * config.CAMERA_HEIGHT


def _middle(a, b):
    if a is None and b is None:
        return None
    if a is None or b is None:
        return a or b
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2


def torso_angle(person: dict) -> float | None:
    """Angle (degrees, 0..90) of the shoulder-hip axis against the horizontal."""
    shoulder = _middle(_kp_px(person, KP_SHOULDER_L), _kp_px(person, KP_SHOULDER_R))
    hip = _middle(_kp_px(person, KP_HIP_L), _kp_px(person, KP_HIP_R))
    if shoulder is None or hip is None:
        return None
    dx, dy = hip[0] - shoulder[0], hip[1] - shoulder[1]
    if math.hypot(dx, dy) < 5.0:
        return None  # points almost on top of each other: the direction is noise
    return math.degrees(math.atan2(abs(dy), abs(dx)))


def lying(person: dict | None) -> dict:
    """Is the person lying? From the torso first, otherwise from the box shape."""
    if person is None:
        return sign("liegend", UNKNOWN, None, "keine Person im Bild")
    angle = torso_angle(person)
    limit = config.HEALTH_LYING_DEG
    if angle is not None:
        if angle < limit:
            return sign("liegend", FLAGGED, round(angle, 1),
                        f"Rumpf {angle:.0f}° gegen die Waagerechte -- liegt", limit, "°")
        return sign("liegend", OK, round(angle, 1),
                    f"Rumpf {angle:.0f}° gegen die Waagerechte -- aufrecht", limit, "°")
    # fallback: box in pixels; less certain (someone sitting with legs apart
    # also has a wide box), so only when the torso is missing
    x1, y1, x2, y2 = person["box"]
    height = max(1e-6, (y2 - y1) * config.CAMERA_HEIGHT)
    ratio = (x2 - x1) * config.CAMERA_WIDTH / height
    limit = config.HEALTH_LYING_ASPECT
    if ratio > limit:
        return sign("liegend", FLAGGED, round(ratio, 2),
                    f"Umriss {ratio:.1f}× so breit wie hoch -- liegt vermutlich", limit, "×")
    return sign("liegend", OK, round(ratio, 2),
                f"Umriss {ratio:.1f}× so breit wie hoch -- eher aufrecht", limit, "×")


def movement_px(history: list[dict]) -> float | None:
    """How much the person in ``history`` (person dicts) has moved.

    Per keypoint the mean deviation (RMS, pixels) from its mean position;
    the result is the mean of the three most mobile ones. So a single hand
    that moves already counts -- the rest of the body may lie still.
    """
    if len(history) < 3:
        return None
    values = []
    for index in range(17):
        points = [p for p in (_kp_px(h, index) for h in history) if p is not None]
        # only points visible in most frames: a point that appears and
        # vanishes would jump between estimates
        if len(points) < max(3, len(history) // 2):
            continue
        a = np.asarray(points)
        values.append(float(np.sqrt(((a - a.mean(axis=0)) ** 2).sum(axis=1).mean())))
    if not values:
        return None
    values.sort(reverse=True)
    return float(np.mean(values[:3]))


def motionless(history: list[dict]) -> dict:
    m = movement_px(history)
    limit = config.HEALTH_MOTIONLESS_PX
    if m is None:
        return sign("regungslos", UNKNOWN, None, "zu wenige Bilder mit Keypoints", limit, "px")
    if m < limit:
        return sign("regungslos", FLAGGED, round(m, 1), f"Bewegung {m:.1f} px -- regungslos", limit, "px")
    return sign("regungslos", OK, round(m, 1), f"Bewegung {m:.1f} px -- bewegt sich", limit, "px")


def answer(heard: list[str] | None) -> dict:
    """``heard`` is None when there is no microphone."""
    if heard is None:
        return sign("antwort", UNKNOWN, None, "kein Mikrofon -- Antwort nicht prüfbar")
    if not heard:
        return sign("antwort", FLAGGED, "", "keine Antwort")
    text = " / ".join(heard)
    return sign("antwort", OK, text, f"hat geantwortet: „{text}“")


def _find(words: list[str], phrase: str) -> int:
    """Index where the phrase (one or more words) starts, otherwise -1."""
    part = phrase.lower().split()
    for i in range(len(words) - len(part) + 1):
        if words[i : i + len(part)] == part:
            return i
    return -1


def interpret_answer(texts: list[str]) -> str | None:
    """"hilfe", "nein" or None (nothing clear heard).

    Help wins: whoever says "nein ... hilf mir" gets help. Only "keine
    Hilfe" counts as no, otherwise exactly that sentence would raise an alarm.
    """
    words = re.findall(r"\w+", " ".join(texts).lower())
    help_ = False
    for w in config.HEALTH_WORDS_HELP:
        i = _find(words, w)
        if i >= 0 and not (i > 0 and words[i - 1] in ("keine", "kein")):
            help_ = True
    if help_:
        return "hilfe"
    if any(_find(words, w) >= 0 for w in config.HEALTH_WORDS_NO) or "keine" in words:
        return "nein"
    return None


def lidar_piece(points, angle_deg: float, half_width_deg: float):
    """The nearest connected object in the direction of the person.

    ``points`` Nx2 (x forward, y left, robot frame). Returns
    ``(distance_m, length_m)`` or None if there is nothing in that direction.
    """
    if points is None or len(points) == 0:
        return None
    p = np.asarray(points, dtype=float)
    w = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    diff = (w - angle_deg + 180.0) % 360.0 - 180.0
    window = p[np.abs(diff) <= half_width_deg + _ANGLE_MARGIN_DEG]
    if len(window) == 0:
        return None
    r = np.hypot(window[:, 0], window[:, 1])
    # grow from the nearest point; search all points (not only the window),
    # because a lying body usually extends beyond the image section
    start = window[int(np.argmin(r))]
    inside = np.zeros(len(p), dtype=bool)
    inside[int(np.argmin(np.hypot(*(p - start).T)))] = True
    while True:
        grown = inside.copy()
        for q in p[inside]:
            grown |= np.hypot(*(p - q).T) < _GAP_M
        if grown.sum() == inside.sum():
            break
        inside = grown
    piece = p[inside]
    length = float(np.max(np.hypot(*(piece[:, None, :] - piece[None, :, :]).transpose(2, 0, 1))))
    return float(np.min(r)), length


def floor(person: dict | None, points, fov_deg: float) -> dict:
    """Is the person lying on the floor? The lidar measures just above it.

    A lying body appears there as one long piece, a standing person only as
    two narrow legs. Without lidar the position in the image counts (lying
    and low in the image), but only as a weak sign.
    """
    if person is None:
        return sign("boden", UNKNOWN, None, "keine Person im Bild")
    x1, _, x2, y2 = person["box"]
    half = (x2 - x1) * fov_deg / 2
    limit = config.HEALTH_FLOOR_MIN_WIDTH_M
    if points is not None:
        piece = lidar_piece(points, person["winkel"], half)
        if piece is not None:
            distance, length = piece
            if length >= limit:
                return sign("boden", FLAGGED, round(length, 2),
                            f"Lidar: {length:.2f} m langes Stück in {distance:.2f} m knapp über dem Boden",
                            limit, "m")
            return sign("boden", OK, round(length, 2),
                        f"Lidar: nur {length:.2f} m breit in {distance:.2f} m -- eher Beine", limit, "m")
    angle = torso_angle(person)
    if angle is not None and angle < config.HEALTH_LYING_DEG and y2 > config.HEALTH_FLOOR_IMAGE_BOTTOM:
        return sign("boden", FLAGGED, round(y2, 2), "nur Kamera: liegt unten im Bild",
                    config.HEALTH_FLOOR_IMAGE_BOTTOM, "Bildhöhe")
    return sign("boden", UNKNOWN, None, "kein Lidar-Echo in Richtung der Person")


def count(checks: dict[str, dict], state: str = FLAGGED) -> int:
    return sum(1 for c in checks.values() if c["zustand"] == state)


def overall_verdict(checks: dict[str, dict], reaction: bool | None = None) -> dict:
    """Summary for the web app: level and one sentence.

    ``reaction`` is the result of nudging (None: not nudged). Emergency
    means: lying or on the floor, not moving and not answering -- and no
    reaction to nudging either, if nudged.
    """
    n = count(checks)
    position = checks["liegend"]["zustand"] == FLAGGED or checks["boden"]["zustand"] == FLAGGED
    still = checks["regungslos"]["zustand"] != OK
    silent = checks["antwort"]["zustand"] != OK
    if reaction is True or checks["antwort"]["zustand"] == OK:
        return {"stufe": "ansprechbar", "punkte": n, "text": "Person reagiert"}
    if position and still and silent and reaction is not True:
        return {"stufe": "notfall", "punkte": n, "text": "liegt regungslos und reagiert nicht"}
    if n == 0:
        return {"stufe": "unauffaellig", "punkte": n, "text": "keine Auffälligkeit"}
    return {"stufe": "auffaellig", "punkte": n, "text": f"{n} von 4 Zeichen auffällig"}
