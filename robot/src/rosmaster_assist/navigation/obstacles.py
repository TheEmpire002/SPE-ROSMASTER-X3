"""Find obstacles in the lidar scan and roughly name them.

The lidar only sees one plane ~20 cm above the floor. What shows up there,
however, has typical shapes:

- **Wand** (wall): long, straight row of points
- **Stuhl** (chair): several thin legs (2-5 cm) close together
- **Tisch** (table): thin legs 0.6-1.8 m apart
- **Mensch** (human): two thicker legs (8-25 cm) side by side -- or something
  the camera has recognised as a person
- everything else is called **Objekt** (object)

This is a rule of thumb, not object recognition: a waste-paper basket can
pass as "Bein (Mensch?)". For avoidance the naming does not matter anyway --
the robot avoids everything. It serves the display, and lets the robot wait
briefly in front of people instead of turning away immediately.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

from .. import config

#: These names end up in the app as they are (state.autonomie.hindernisse[].art).
WALL = "Wand"
CHAIR = "Stuhl"
TABLE = "Tisch"
HUMAN = "Mensch"
HUMAN_LEG = "Bein (Mensch?)"
LEG = "Bein (Stuhl/Tisch?)"
OBJECT = "Objekt"


@dataclass
class Obstacle:
    category: str
    angle: float     # degrees, 0 = front, counter-clockwise
    distance: float    # m, nearest point from the lidar centre
    width: float     # m
    x: float          # m, centre in robot coordinates (front)
    y: float          # m, (left)
    source: str = "Lidar"

    def as_dict(self) -> dict:
        # Explicit keys, not asdict(): these are protocol field names
        # (state.autonomie.hindernisse[]) and must not follow code renames.
        return {
            "art": self.category,
            "winkel": round(self.angle, 1),
            "abstand": round(self.distance, 2),
            "breite": round(self.width, 2),
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "quelle": self.source,
        }


def _points(angle: list[float], ranges: list[float]) -> list[tuple[float, float, float, float]]:
    """Sorted by angle: (angle, r, x, y), only within the detection radius."""
    raw = sorted(
        (w, r) for w, r in zip(angle, ranges) if 0.0 < r <= config.OBSTACLE_RANGE_M
    )
    return [
        (w, r, r * math.cos(math.radians(w)), r * math.sin(math.radians(w)))
        for w, r in raw
    ]


def _clusters(points: list) -> list[list]:
    """Group neighbouring points into objects.

    A new object begins where the distance to the previous point jumps. The
    threshold grows with range, because there the angular step alone already
    pulls the points apart.
    """
    if not points:
        return []
    groups: list[list] = [[points[0]]]
    for previous, p in zip(points, points[1:]):
        threshold = config.OBSTACLE_GAP_M + 0.03 * p[1]
        gap = math.hypot(p[2] - previous[2], p[3] - previous[3])
        if gap <= threshold and p[0] - previous[0] <= 5.0:
            groups[-1].append(p)
        else:
            groups.append([p])

    # Closed all around: an object right behind the robot lies at the start
    # and at the end of the list (-180°/+180°) -- that is one object.
    if len(groups) > 1:
        first, last = groups[0][0], groups[-1][-1]
        gap = math.hypot(first[2] - last[2], first[3] - last[3])
        if gap <= config.OBSTACLE_GAP_M + 0.03 * first[1] and (first[0] + 360.0 - last[0]) <= 5.0:
            groups[0] = groups.pop() + groups[0]
    return groups


def _deviation(group: list) -> tuple[float, int]:
    """Largest distance of a point from the line first-last point."""
    ax, ay = group[0][2], group[0][3]
    bx, by = group[-1][2], group[-1][3]
    length = math.hypot(bx - ax, by - ay) or 1e-9
    largest, index = 0.0, 0
    for i, p in enumerate(group):
        d = abs((bx - ax) * (ay - p[3]) - (ax - p[2]) * (by - ay)) / length
        if d > largest:
            largest, index = d, i
    return largest, index


def _split(group: list, depth: int = 0) -> list[list]:
    """Split corners: two walls at a right angle form one cluster, but should
    be recognised as two straight pieces."""
    if len(group) < 6 or depth >= 3:
        return [group]
    width = math.hypot(group[-1][2] - group[0][2], group[-1][3] - group[0][3])
    if width < config.OBSTACLE_WALL_MIN_M:
        return [group]
    dev, i = _deviation(group)
    if dev <= 0.05 or i in (0, len(group) - 1):
        return [group]
    return _split(group[: i + 1], depth + 1) + _split(group[i:], depth + 1)


def _obstacle(group: list, category: str) -> Obstacle:
    x = sum(p[2] for p in group) / len(group)
    y = sum(p[3] for p in group) / len(group)
    width = math.hypot(group[-1][2] - group[0][2], group[-1][3] - group[0][3])
    return Obstacle(
        category=category,
        angle=math.degrees(math.atan2(y, x)),
        distance=min(p[1] for p in group),
        width=width,
        x=x,
        y=y,
    )


def _merge(parts: list[Obstacle], category: str) -> Obstacle:
    """Combine several legs into one piece of furniture / one human."""
    x = sum(t.x for t in parts) / len(parts)
    y = sum(t.y for t in parts) / len(parts)
    width = max(
        (math.hypot(a.x - b.x, a.y - b.y) for a in parts for b in parts), default=0.0
    )
    return Obstacle(
        category=category,
        angle=math.degrees(math.atan2(y, x)),
        distance=min(t.distance for t in parts),
        width=width,
        x=x,
        y=y,
    )


def _group(parts: list[Obstacle], max_distance: float) -> list[list[Obstacle]]:
    """Connected groups: each member has a neighbour closer than max_distance."""
    rest = list(parts)
    groups = []
    while rest:
        group = [rest.pop()]
        i = 0
        while i < len(group):
            close = [t for t in rest if math.hypot(t.x - group[i].x, t.y - group[i].y) <= max_distance]
            for t in close:
                rest.remove(t)
            group.extend(close)
            i += 1
        groups.append(group)
    return groups


def _in_field_of_view(angle: float, people: list[tuple[float, float]]) -> bool:
    return any(lo - 5.0 <= angle <= hi + 5.0 for lo, hi in people)


def classify(
    angle: list[float],
    ranges: list[float],
    people_angles: list[tuple[float, float]] | None = None,
) -> list[Obstacle]:
    """Obstacles of a scan, nearest first.

    ``people_angles``: angle ranges (from, to) in degrees in which the camera
    currently sees a person. Whatever the lidar finds there is then called
    "Mensch", whatever its shape.
    """
    people_angles = people_angles or []
    pieces: list[Obstacle] = []
    for cluster in _clusters(_points(angle, ranges)):
        # Single points are mostly noise -- except very close, where a table
        # leg hit by just one beam is already dangerous.
        if len(cluster) < 2 and cluster[0][1] > 1.0:
            continue
        for part in _split(cluster):
            width = math.hypot(part[-1][2] - part[0][2], part[-1][3] - part[0][3])
            if width >= config.OBSTACLE_WALL_MIN_M and _deviation(part)[0] <= 0.06:
                category = WALL
            elif width <= config.OBSTACLE_LEG_MAX_M:
                category = LEG
            elif width <= config.OBSTACLE_HUMAN_LEG_MAX_M:
                category = HUMAN_LEG
            else:
                category = OBJECT
            pieces.append(_obstacle(part, category))

    result = [s for s in pieces if s.category in (WALL, OBJECT)]

    # Human legs: two thick legs close side by side.
    for group in _group([s for s in pieces if s.category == HUMAN_LEG], 0.45):
        result.append(_merge(group, HUMAN) if len(group) >= 2 else group[0])

    # Thin legs: close together a chair, farther apart a table.
    thin = [s for s in pieces if s.category == LEG]
    singles = []
    for group in _group(thin, 0.55):
        if len(group) >= 2:
            result.append(_merge(group, CHAIR))
        else:
            singles.extend(group)
    for group in _group(singles, 1.8):
        if len(group) >= 2:
            result.append(_merge(group, TABLE))
        else:
            result.extend(group)

    # The camera trumps the shape: if it sees a person in this direction,
    # it is one -- even if the legs look like an object to the lidar.
    for h in result:
        if h.category != WALL and _in_field_of_view(h.angle, people_angles):
            h.category = HUMAN
            h.source = "Kamera+Lidar"

    result.sort(key=lambda h: h.distance)
    return result


class ObstacleDetection:
    """Remembers the result per scan.

    Server (display) and explorer (driving) both ask -- yet it is computed
    only once per lidar revolution.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._scan_time: float | None = None
        self._people: list[tuple[float, float]] = []
        self._result: list[Obstacle] = []

    def current(self, scan, people_angles: list[tuple[float, float]] | None = None) -> list[Obstacle]:
        people_angles = people_angles or []
        with self._lock:
            if scan is None:
                return []
            if scan.timestamp == self._scan_time and people_angles == self._people:
                return self._result
        result = classify(scan.angle, scan.ranges, people_angles)
        with self._lock:
            self._scan_time = scan.timestamp
            self._people = people_angles
            self._result = result
        return result
