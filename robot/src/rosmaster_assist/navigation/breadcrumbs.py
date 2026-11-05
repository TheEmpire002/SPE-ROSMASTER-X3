"""The trail the robot drove, and named places on it -- for the way back (E14).

While the pose in the main map is confident, every ``BREADCRUMB_M`` a crumb
is dropped: "streng" if the integrate gate is writing at that moment
(the strictest test the pose passes), otherwise "sicher". When the pose is
lost the trail breaks there -- a gap is never bridged.

The way back uses it twice:

* the planner prefers the cells along the strict crumbs since the anchor
  (``corridor``), and lets the robot pass there with a little less wall
  clearance -- it was physically there;
* if the planner finds no path at all, the robot drives the crumbs back
  (``reversed_path``).

Anchors are named poses: ``home`` (start of the map), ``mission`` (where a
search began), later ``person``. All in the frame of the main map, so they
survive a restart (saved in the map's json).
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from .. import config

STRICT = "streng"
CONFIDENT = "sicher"


class Breadcrumbs:
    """Crumbs ``[x, y, strict, segment, id]`` and anchors in the main map's frame."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._crumbs: deque = deque(maxlen=config.BREADCRUMB_MAX)
        self._segment = 0
        self._next_id = 0
        self._broken = False
        #: name -> {"pose": [x, y, theta], "seit": first crumb id, "zeit": time}
        self.anchors: dict[str, dict] = {}

    # --- recording ------------------------------------------------------------------

    def add(self, pose, strict: bool) -> bool:
        """Drop a crumb if the robot moved far enough; True if one was added."""
        with self._lock:
            if self._crumbs and not self._broken:
                last = self._crumbs[-1]
                if math.hypot(pose[0] - last[0], pose[1] - last[1]) < config.BREADCRUMB_M:
                    if strict and not last[2]:
                        last[2] = True   # the same spot, now confirmed strictly
                        return True      # changed: worth saving
                    return False
            if self._broken:
                self._segment += 1
                self._broken = False
            self._crumbs.append([float(pose[0]), float(pose[1]), bool(strict), self._segment,
                                 self._next_id])
            self._next_id += 1
            return True

    def gap(self) -> None:
        """The pose is lost: the next crumb starts a new piece of trail."""
        with self._lock:
            self._broken = True

    def clear(self) -> None:
        with self._lock:
            self._crumbs.clear()
            self._broken = False
            self.anchors.clear()

    # --- anchors ----------------------------------------------------------------------

    def set_anchor(self, name: str, pose) -> None:
        with self._lock:
            self.anchors[name] = {
                "pose": [float(pose[0]), float(pose[1]), float(pose[2])],
                "seit": self._next_id,
                "zeit": time.time(),
            }

    def anchor(self, name: str) -> tuple[float, float, float] | None:
        with self._lock:
            a = self.anchors.get(name)
            return tuple(a["pose"]) if a else None

    # --- queries ------------------------------------------------------------------------

    def _since(self, name: str | None) -> list:
        """Crumbs since the anchor was set (lock held); all without anchor."""
        a = self.anchors.get(name) if name else None
        first = a["seit"] if a else -1
        return [c for c in self._crumbs if c[4] >= first]

    def corridor(self, since: str | None = None) -> np.ndarray:
        """Strict crumbs since the anchor, filled in between neighbours (Nx2)."""
        with self._lock:
            crumbs = [c for c in self._since(since) if c[2]]
        if not crumbs:
            return np.zeros((0, 2))
        step = config.NAV_PLAN_RESOLUTION_M / 2   # no planning cell is skipped
        out = [crumbs[0][:2]]
        for a, b in zip(crumbs, crumbs[1:]):
            gap = math.hypot(b[0] - a[0], b[1] - a[1])
            # only along the trail actually driven: not across a break, and
            # not between strict crumbs that had a weaker piece in between
            if a[3] == b[3] and b[4] == a[4] + 1 and gap <= config.BREADCRUMB_FILL_GAP_M:
                n = max(1, int(math.ceil(gap / step)))
                out.extend([[a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n]
                            for k in range(1, n + 1)])
            else:
                out.append(b[:2])
        return np.asarray(out, dtype=float)

    def reversed_path(self, since: str | None = None) -> list[tuple[float, float]]:
        """The trail since the anchor backwards, simplified -- the last resort way back.

        only the newest unbroken piece: a gap means the pose was lost there,
        and what lies before it may not fit
        """
        with self._lock:
            crumbs = self._since(since)
        if not crumbs:
            return []
        last_segment = crumbs[-1][3]
        piece = [(c[0], c[1]) for c in crumbs if c[3] == last_segment]
        return _simplify(piece[::-1], config.BREADCRUMB_SIMPLIFY_M)

    def near(self) -> np.ndarray:
        """All crumbs (Nx2) -- the localization searches near them first."""
        with self._lock:
            return np.asarray([c[:2] for c in self._crumbs], dtype=float).reshape(-1, 2)

    def length_m(self, since: str | None = None) -> float:
        with self._lock:
            crumbs = self._since(since)
        total = 0.0
        for a, b in zip(crumbs, crumbs[1:]):
            if a[3] == b[3]:
                total += math.hypot(b[0] - a[0], b[1] - a[1])
        return total

    def display(self, since: str | None = None) -> list[list]:
        """For the /map page: [x, y, strict, segment], the newest ``BREADCRUMB_DISPLAY_MAX``."""
        with self._lock:
            crumbs = self._since(since)[-config.BREADCRUMB_DISPLAY_MAX:]
        return [[round(c[0], 2), round(c[1], 2), c[2], c[3]] for c in crumbs]

    # --- saving -----------------------------------------------------------------------

    def to_json(self) -> dict:
        with self._lock:
            return {
                "spur": [[round(c[0], 3), round(c[1], 3), c[2], c[3], c[4]]
                         for c in list(self._crumbs)[-config.BREADCRUMB_SAVE_MAX:]],
                "anker": {k: dict(v) for k, v in self.anchors.items()},
            }

    def from_json(self, data: dict) -> None:
        """Take over a saved trail; a restart is a break in it."""
        with self._lock:
            self._crumbs.clear()
            self.anchors.clear()
            try:
                for c in data.get("spur") or []:
                    self._crumbs.append([float(c[0]), float(c[1]), bool(c[2]), int(c[3]),
                                         int(c[4])])
                for name, a in (data.get("anker") or {}).items():
                    self.anchors[str(name)] = {
                        "pose": [float(v) for v in a["pose"][:3]],
                        "seit": int(a.get("seit", 0)),
                        "zeit": float(a.get("zeit", 0.0)),
                    }
            except (TypeError, ValueError, KeyError, IndexError):
                self._crumbs.clear()
                self.anchors.clear()
            # the next crumb opens a new piece: add() counts up because of _broken
            self._segment = self._crumbs[-1][3] if self._crumbs else 0
            self._next_id = (self._crumbs[-1][4] + 1) if self._crumbs else 0
            # an anchor never lies ahead of the trail, or "since the anchor"
            # would stay empty until the ids catch up
            for a in self.anchors.values():
                a["seit"] = min(a["seit"], self._next_id)
            self._broken = True


def _simplify(points, tolerance: float) -> list[tuple[float, float]]:
    """Douglas-Peucker: drop points that lie within ``tolerance`` of the line."""
    if len(points) <= 2:
        return list(points)
    pts = np.asarray(points, dtype=float)
    keep = np.zeros(len(pts), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        length = float(np.hypot(*ab))
        seg = pts[i + 1:j]
        if length < 1e-9:
            d = np.hypot(*(seg - a).T)
        else:
            d = np.abs(ab[0] * (seg[:, 1] - a[1]) - ab[1] * (seg[:, 0] - a[0])) / length
        k = int(np.argmax(d))
        if d[k] > tolerance:
            keep[i + 1 + k] = True
            stack.append((i, i + 1 + k))
            stack.append((i + 1 + k, j))
    return [tuple(p) for p in pts[keep]]
