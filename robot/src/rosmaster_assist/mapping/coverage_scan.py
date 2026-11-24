"""Autonomous 3D scan: drive through a known map until a share of its walls is in the 3D map.

**What counts.** The walls of the 2D lidar map that border the free area the
robot can reach (thin lidar streaks through doors are opened away first). A
wall cell counts as seen once the 3D map (``Room3D``) has shown blocks above
it, at least ``COVERAGE_MIN_Z_M`` above the floor. So the share comes from the
stored blocks: it survives restarts and also counts what was recorded by hand.

**Where to look next.** Candidate spots on a grid over the reachable free
area, with room around them. From each, rays are cast through the 2D map in
all directions (the robot turns once around at every spot); a ray scores if
the first wall it hits is unseen and within the camera's range. The best spot
weighs that against the distance to drive there.

**Driving.** To the spot with ``Homing`` (path planner, obstacle avoidance,
through the watchdog), then once around in ``Maneuver`` turns with a pause
after each: the 3D map takes no images while turning faster than
``MAP3D_MAX_OMEGA``, and a block needs ``MAP3D_MIN_HITS`` images. Every manual
input, emergency stop or other drive mode ends the scan.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from .. import config
from ..navigation import maneuver as mv

log = logging.getLogger(__name__)


def _kernel(np, radius_cells: int):
    size = 2 * max(0, int(radius_cells)) + 1
    return np.ones((size, size), np.uint8)


def reachable_region(occ, robot_xy):
    """Free cells connected to the robot, without thin streaks (bool grid) or None"""
    import cv2
    import numpy as np

    res = occ.resolution
    radius = max(1, int(round(config.COVERAGE_OPEN_M / res / 2)))
    opened = cv2.morphologyEx(occ.free.astype(np.uint8), cv2.MORPH_OPEN, _kernel(np, radius))
    _, labels = cv2.connectedComponents(opened, connectivity=4)
    ix, iy = occ.cell(robot_xy[0], robot_xy[1])
    ix, iy = int(ix), int(iy)
    if not (0 <= ix < occ.width and 0 <= iy < occ.height):
        return None
    label = int(labels[iy, ix])
    if label == 0:
        # standing close to a wall the opening ate: the region nearby
        r = int(math.ceil(config.COVERAGE_OPEN_M / res))
        window = labels[max(0, iy - r):iy + r + 1, max(0, ix - r):ix + r + 1]
        found = window[window > 0]
        if not len(found):
            return None
        label = int(np.bincount(found).argmax())
    return labels == label


def wall_targets(occ, region):
    """Wall cells that border the reachable area: the ones a scan should see"""
    import cv2
    import numpy as np

    radius = max(1, int(round(config.COVERAGE_WALL_REACH_M / occ.resolution)))
    near = cv2.dilate(region.astype(np.uint8), _kernel(np, radius)).astype(bool)
    return occ.occupied & near


def seen_cells(occ, xs, ys):
    """Cells with 3D blocks above them, widened by ``COVERAGE_SEEN_TOL_M`` (the
    lidar wall and the depth wall are a few cm apart)"""
    import cv2
    import numpy as np

    seen = np.zeros((occ.height, occ.width), np.uint8)
    if len(xs):
        ix, iy = occ.cell(xs, ys)
        inside = occ.inside(ix, iy)
        seen[iy[inside], ix[inside]] = 1
    radius = int(round(config.COVERAGE_SEEN_TOL_M / occ.resolution))
    if radius > 0:
        seen = cv2.dilate(seen, _kernel(np, radius))
    return seen.astype(bool)


def best_view(occ, region, unseen, robot_xy, avoid) -> tuple[float, float, int] | None:
    """Spot (x, y, rays that hit unseen walls) to look around next, None if none is worth it"""
    import numpy as np

    res = occ.resolution
    clear = region & (occ.distance >= config.COVERAGE_CLEARANCE_M)
    step = max(1, int(round(config.COVERAGE_CANDIDATE_STEP_M / res)))
    grid = np.zeros_like(clear)
    grid[::step, ::step] = True
    cy, cx = np.nonzero(clear & grid)
    if len(cx) > config.COVERAGE_MAX_CANDIDATES:
        keep = np.linspace(0, len(cx) - 1, config.COVERAGE_MAX_CANDIDATES).astype(int)
        cx, cy = cx[keep], cy[keep]
    wx, wy = occ.world(cx, cy)
    # where it stands is a candidate too: the first look needs no drive
    wx = np.append(np.asarray(wx, float), robot_xy[0])
    wy = np.append(np.asarray(wy, float), robot_xy[1])
    if avoid:
        pts = np.asarray(avoid, float)
        d = np.hypot(wx[:, None] - pts[None, :, 0], wy[:, None] - pts[None, :, 1]).min(axis=1)
        far = d >= config.COVERAGE_VISITED_M
        wx, wy = wx[far], wy[far]
    if not len(wx):
        return None

    angles = np.linspace(0.0, 2 * math.pi, config.COVERAGE_RAYS, endpoint=False)
    ray_step = res * config.COVERAGE_RAY_STEP
    radii = np.arange(ray_step, config.COVERAGE_RANGE_M, ray_step)
    cos, sin = np.cos(angles)[None, :, None], np.sin(angles)[None, :, None]
    # unknown blocks the view too: behind it could be anything
    blocking = occ.occupied | (occ.occupancy < 0)
    gains = np.zeros(len(wx), int)
    min_index = int(np.searchsorted(radii, config.MAP3D_MIN_M))
    for start in range(0, len(wx), 64):
        px = wx[start:start + 64, None, None] + radii[None, None, :] * cos
        py = wy[start:start + 64, None, None] + radii[None, None, :] * sin
        ix, iy = occ.cell(px, py)
        inside = occ.inside(ix, iy)
        ix = np.clip(ix, 0, occ.width - 1)
        iy = np.clip(iy, 0, occ.height - 1)
        blocked = blocking[iy, ix] | ~inside
        first = blocked.argmax(axis=2)
        hit = blocked.any(axis=2)
        fx = np.take_along_axis(ix, first[..., None], axis=2)[..., 0]
        fy = np.take_along_axis(iy, first[..., None], axis=2)[..., 0]
        # a ray that leaves the map saw no wall, even if the border cell is one
        left = ~np.take_along_axis(inside, first[..., None], axis=2)[..., 0]
        good = hit & ~left & (first >= min_index) & unseen[fy, fx]
        gains[start:start + 64] = good.sum(axis=1)
    dist = np.hypot(wx - robot_xy[0], wy - robot_xy[1])
    score = gains / (1.0 + dist / config.COVERAGE_DISTANCE_SCALE_M)
    # only spots worth it compete: a poor one nearby must not hide a good one far away
    score[gains < config.COVERAGE_MIN_GAIN] = -1.0
    best = int(np.argmax(score))
    if score[best] < 0:
        return None
    return float(wx[best]), float(wy[best]), int(gains[best])


class CoverageScan:
    """Scans a known map by itself. Used like ``Panorama``: start, cancel, state."""

    def __init__(self, room3d, localization, homing, maneuver, corridor=None, busy=None) -> None:
        self._room = room3d
        self._localization = localization
        self._homing = homing
        self._maneuver = maneuver
        # breadcrumb trail for the planner (where it drove, it fits through)
        self._corridor = corridor
        # returns a reason (german) when another mode drives, else None
        self._busy = busy
        self._task: asyncio.Task[None] | None = None
        self._text = ""
        self.reason: str | None = None
        self.goal_pct = float(config.COVERAGE_DEFAULT_PCT)
        self._coverage: dict[str, Any] | None = None
        self._next: tuple[float, float] | None = None
        self._spots: list[tuple[float, float]] = []
        self._started = 0.0
        self._deadline = 0.0

    @property
    def available(self) -> bool:
        return (self._room is not None and self._room.available and self._maneuver is not None
                and self._maneuver.available and self._homing is not None)

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    def state(self) -> dict[str, Any]:
        active = self.active
        cov = self._coverage or {}
        return {
            "verfuegbar": self.available,
            "aktiv": active,
            "text": self._text if active else None,
            "ziel_prozent": self.goal_pct,
            "prozent": cov.get("prozent"),
            "gesehen": cov.get("gesehen"),
            "gesamt": cov.get("gesamt"),
            "stellen": len(self._spots),
            "naechste": [round(v, 2) for v in self._next] if active and self._next else None,
            "besucht": [[round(x, 2), round(y, 2)] for x, y in self._spots],
            "dauer_s": round(time.monotonic() - self._started) if active else None,
            "rest_s": max(0, round(self._deadline - time.monotonic())) if active else None,
            "grund": None if active else self.reason,
        }

    def start(self, goal_pct: float, max_minutes: float | None = None) -> str | None:
        """Start from here; returns an error text (german)"""
        if not self.available:
            return "Der 3D-Scan braucht Tiefenkamera und Lidar"
        if self.active:
            return "Der 3D-Scan läuft schon"
        try:
            goal_pct = float(goal_pct)
            minutes = float(config.COVERAGE_MAX_MIN if max_minutes is None else max_minutes)
        except (TypeError, ValueError):
            return "Prozent und Minuten müssen Zahlen sein"
        if not (math.isfinite(goal_pct) and 1.0 <= goal_pct <= 100.0):
            return "Ziel: 1 bis 100 %"
        if not (math.isfinite(minutes) and 1.0 <= minutes <= config.COVERAGE_MAX_MIN_LIMIT):
            return f"Zeit: 1 bis {config.COVERAGE_MAX_MIN_LIMIT:g} Minuten"
        if self._localization.occ_map is None:
            return "Keine Karte geladen"
        if not self._localization.confident or self._localization.pose() is None:
            return "Lage in der Karte unsicher -- erst auf der Karte zeigen, wo er steht"
        self.goal_pct = goal_pct
        self.reason = None
        self._spots = []
        self._next = None
        self._started = time.monotonic()
        self._deadline = self._started + minutes * 60.0
        self._task = asyncio.create_task(self._run())
        log.info("3D scan started: goal %.0f %%, at most %.0f min", goal_pct, minutes)
        return None

    def cancel(self, reason: str = "abgebrochen") -> None:
        if self.active:
            self._task.cancel()
            self.reason = reason
            self._homing.cancel(f"3D-Scan {reason}")
            self._maneuver.cancel(f"3D-Scan {reason}")
            log.info("3D scan %s", reason)
        self._task = None

    # --- coverage ---

    def _evaluate(self) -> dict[str, Any] | None:
        """Coverage now (worker thread): grids and numbers, None without map or pose"""
        import numpy as np

        occ = self._localization.occ_map
        pose = self._localization.pose()
        if occ is None or pose is None:
            return None
        region = reachable_region(occ, pose)
        if region is None:
            return None
        targets = wall_targets(occ, region)
        current, xs, ys = self._room.columns(config.COVERAGE_MIN_Z_M)
        if current is None or current[0] != getattr(occ, "name", None):
            xs = ys = np.zeros(0)
        seen = targets & seen_cells(occ, xs, ys)
        total = int(targets.sum())
        n_seen = int(seen.sum())
        pct = 100.0 * n_seen / total if total else 0.0
        self._coverage = {"prozent": round(pct, 1), "gesehen": n_seen, "gesamt": total}
        return {"occ": occ, "pose": pose, "region": region, "targets": targets, "seen": seen,
                "prozent": pct, "gesamt": total}

    def coverage(self) -> dict[str, Any]:
        """For /map: share and the wall cells, seen and still missing (worker thread)"""
        import numpy as np

        ev = self._evaluate()
        if ev is None:
            return {"prozent": None, "gesehen": [], "fehlt": [],
                    "text": "Keine Karte oder Lage unsicher"}
        occ = ev["occ"]

        def points(mask):
            iy, ix = np.nonzero(mask)
            if len(ix) > config.COVERAGE_DRAW_MAX:
                keep = np.linspace(0, len(ix) - 1, config.COVERAGE_DRAW_MAX).astype(int)
                ix, iy = ix[keep], iy[keep]
            x, y = occ.world(ix, iy)
            return [[round(float(a), 2), round(float(b), 2)] for a, b in zip(x, y)]

        return {"prozent": round(ev["prozent"], 1), "gesamt": ev["gesamt"],
                "gesehen": points(ev["seen"]), "fehlt": points(ev["targets"] & ~ev["seen"]),
                "text": None}

    # --- the drive ---

    def _set(self, text: str) -> None:
        self._text = text

    def _end(self, reason: str) -> None:
        self.reason = reason
        log.info("3D scan: %s", reason)

    def _other(self) -> str | None:
        return self._busy() if self._busy is not None else None

    async def _run(self) -> None:
        was_recording = self._room.recording
        self._room.set_recording(True)
        self._maneuver.quiet = True
        failed: list[tuple[float, float]] = []
        failures = 0
        try:
            self._set("Tiefenkamera startet …")
            await asyncio.sleep(config.COVERAGE_WARMUP_S)
            while True:
                other = self._other()
                if other:
                    return self._end(f"abgebrochen: {other}")
                if not await self._wait_confident():
                    return self._end("abgebrochen: Lage in der Karte unsicher")
                ev = await asyncio.to_thread(self._evaluate)
                if ev is None:
                    return self._end("abgebrochen: keine Karte oder Lage")
                if not ev["gesamt"]:
                    return self._end("keine Wände in der Karte gefunden")
                pct = ev["prozent"]
                if pct >= self.goal_pct:
                    return self._end(f"fertig: {pct:.0f} % der Wände erfasst")
                if time.monotonic() > self._deadline:
                    return self._end(f"Zeit abgelaufen bei {pct:.0f} %")
                self._set(f"{pct:.0f} % erfasst -- sucht die nächste Stelle")
                unseen = ev["targets"] & ~ev["seen"]
                view = await asyncio.to_thread(best_view, ev["occ"], ev["region"], unseen,
                                               ev["pose"][:2], self._spots + failed)
                if view is None:
                    return self._end(f"keine Stelle mehr, von der er mehr sieht ({pct:.0f} % erfasst)")
                x, y, gain = view
                self._next = (x, y)
                pose = self._localization.pose() or ev["pose"]
                if math.hypot(x - pose[0], y - pose[1]) > config.COVERAGE_ARRIVE_M:
                    self._set(f"{pct:.0f} % erfasst -- fährt zur nächsten Stelle")
                    reached = await self._drive_to(x, y)
                    if reached is None:
                        return self._end(f"abgebrochen: {self._homing.last_reason or 'Fahrt beendet'}")
                    if not reached:
                        failed.append((x, y))
                        failures += 1
                        log.info("3D scan: spot (%.2f, %.2f) not reached (%s)", x, y,
                                 self._homing.last_reason)
                        if failures >= config.COVERAGE_MAX_FAILURES:
                            return self._end(f"{failures} Stellen nicht erreicht ({pct:.0f} % erfasst)")
                        continue
                failures = 0
                error = await self._look_around(pct)
                if error:
                    return self._end(error)
                self._spots.append((x, y))
                log.info("3D scan: looked around at (%.2f, %.2f), %d rays expected new", x, y, gain)
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("3D scan failed")
            self.reason = "Fehler beim 3D-Scan"
        finally:
            self._maneuver.quiet = False
            self._room.set_recording(was_recording)
            self._next = None

    async def _wait_confident(self) -> bool:
        deadline = time.monotonic() + config.COVERAGE_LOST_WAIT_S
        while not (self._localization.confident and self._localization.pose() is not None):
            if time.monotonic() > deadline:
                return False
            self._set("wartet, bis er wieder weiß, wo er ist")
            await asyncio.sleep(config.COVERAGE_POLL_S)
        return True

    async def _drive_to(self, x: float, y: float) -> bool | None:
        """True arrived, False not reached (try another spot), None stopped from outside"""
        corridor = self._corridor() if self._corridor is not None else None
        if not self._homing.start(target=(x, y), corridor=corridor):
            return False
        while self._homing.active:
            await asyncio.sleep(config.COVERAGE_POLL_S)
        if self._homing.cancelled:
            return None
        return self._homing.arrived

    async def _look_around(self, pct: float) -> str | None:
        """Once around in steps, still for a moment after each; error text if it must stop"""
        steps = max(3, int(round(360.0 / config.COVERAGE_TURN_STEP_DEG)))
        step = 360.0 / steps
        for i in range(steps):
            self._set(f"{pct:.0f} % erfasst -- schaut sich um ({i + 1}/{steps})")
            # standing still: the depth images go into the 3d map
            await asyncio.sleep(config.COVERAGE_DWELL_S)
            other = self._other()
            if other:
                return f"abgebrochen: {other}"
            if i == steps - 1:
                break
            error = self._maneuver.start(mv.TURN, step)
            if error:
                return error
            while self._maneuver.active:
                await asyncio.sleep(config.COVERAGE_POLL_S)
            if self._maneuver.cancelled:
                return f"abgebrochen: {self._maneuver.last_reason or 'Drehung beendet'}"
            if not self._maneuver.succeeded:
                # no room to turn here: on to the next spot
                log.info("3D scan: turn failed (%s)", self._maneuver.last_reason)
                break
        return None
