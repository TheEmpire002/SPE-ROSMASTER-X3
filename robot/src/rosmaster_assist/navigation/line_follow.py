"""Line following: drive along tape or mats stuck to the floor.

Idea taken from Yahboom's ``yahboomcar_linefollow`` (learn the line colour,
mask it, steer towards it), but built for our setup:

1. **Bird's-eye view.** The Orbbec looks straight ahead from 22 cm, so the
   floor only appears in the lower quarter of the image, heavily
   foreshortened. The image is warped onto the floor plane (1 px = 1 cm,
   robot frame). Curves and corners then look like what they are, and
   steering works in metres instead of pixels.
2. **Learn the colour at the start.** Whatever stands out most from the
   floor closest in front of the robot is the line. Black, white or coloured
   tape all work without setting anything. While following, a pixel is line
   when it is closer to the learned colour than to the floor (the floor
   colour is re-measured in every image, the line colour adapts slowly).
3. **Trace instead of centroid.** A centroid of a band cannot tell a curve
   from a junction. Instead the line is walked in 5 cm steps: at every step a
   fan of directions is checked, each contiguous hit is a branch. One branch:
   curve or straight. Several: junction, choose by preference ("gerade",
   "links", "rechts").
4. **Remember the blind zone.** The first ~0.5 m in front of the robot are
   not visible. The traced line is stored in the odometry frame, so the robot
   keeps following it through that zone -- a 90° corner has long left the
   image when the robot reaches it.
5. **Pure pursuit** on that line: steer towards the point one look-ahead
   distance away, slower in tight curves, turn on the spot at sharp corners.
   When the line is used up and nothing is seen: turn to search, then stop
   (end of the course).

Drives only via ``SafetyWatchdog.submit(source="autonom")``, so speed limit,
dead man's switch and drive guard (lidar) apply as for every autonomous mode.
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from .. import config
from ..protocol import DriveCommand, STOP
from ..safety import SafetyWatchdog
from ..safety.drive_guard import NO_LIDAR

log = logging.getLogger(__name__)

STRAIGHT = "gerade"
LEFT = "links"
RIGHT = "rechts"
CHOICES = (STRAIGHT, LEFT, RIGHT)

# states for the app (user-facing, german)
OFF = "aus"
LEARN = "lernt"
FOLLOW = "folgt"
BLIND = "blind"      # drives on the remembered line, sees none right now
WAIT = "wartet"      # stands, waits for camera image or line
SEARCH = "sucht"
TURN = "dreht"       # sharp corner, turns on the spot


def _cv2():
    import cv2  # not on every laptop, only needed when the mode runs

    return cv2


def _runs(hits: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous True stretches as (first, last) index."""
    if not hits.any():
        return []
    padded = np.concatenate(([0], hits.astype(np.int8), [0]))
    d = np.diff(padded)
    starts = np.nonzero(d == 1)[0]
    ends = np.nonzero(d == -1)[0] - 1
    return list(zip(starts.tolist(), ends.tolist()))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _to_world(points, pose):
    x0, y0, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(x0 + c * x - s * y, y0 + s * x + c * y) for x, y in points]


def _to_robot(points, pose):
    x0, y0, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0)) for x, y in points]


@dataclass
class Detection:
    path: list[tuple[float, float]]   # robot frame at the image time, near to far
    width_m: float
    junction: bool


# --- Seeing the line -----------------------------------------------------------------


class GroundView:
    """Warps a camera image onto the floor in front of the robot.

    BEV pixel (row, col): row 0 = farthest, col 0 = leftmost, so the picture
    looks like a top view with "forward" up.
    """

    def __init__(self, width_px: int, height_px: int, fov_deg: float) -> None:
        cv2 = _cv2()
        self.res = config.LINE_BEV_RES_M
        self.x_min = config.LINE_VIEW_MIN_M
        self.x_max = config.LINE_VIEW_MAX_M
        self.y_half = config.LINE_VIEW_HALF_WIDTH_M
        self.rows = int(round((self.x_max - self.x_min) / self.res)) + 1
        self.cols = int(round(2 * self.y_half / self.res)) + 1

        self._f = (width_px / 2) / math.tan(math.radians(fov_deg) / 2)
        self._cx = (width_px - 1) / 2
        self._cy = (height_px - 1) / 2
        self._h = config.LINE_CAMERA_HEIGHT_M
        self._fwd = config.LINE_CAMERA_FORWARD_M
        self._tilt = math.radians(config.LINE_CAMERA_TILT_DEG)

        ground = [(self.x_max, self.y_half), (self.x_max, -self.y_half),
                  (self.x_min, -self.y_half), (self.x_min, self.y_half)]
        bev = [(0, 0), (self.cols - 1, 0), (self.cols - 1, self.rows - 1), (0, self.rows - 1)]
        src = np.float32([self._to_image(x, y) for x, y in ground])
        self.homography = cv2.getPerspectiveTransform(src, np.float32(bev))
        self.size = (width_px, height_px)

        full = np.full((height_px, width_px), 255, np.uint8)
        valid = cv2.warpPerspective(
            full, self.homography, (self.cols, self.rows),
            flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0,
        )
        # edge pixels are interpolated with the border, so they are not trusted
        valid = cv2.erode(valid, np.ones((5, 5), np.uint8))
        self.valid = valid > 0
        rows = np.nonzero(self.valid[:, self.cols // 2])[0]
        # nearest visible floor straight ahead
        self.near_x = self.x_max - rows.max() * self.res if len(rows) else self.x_min

    def _to_image(self, x: float, y: float) -> tuple[float, float]:
        """Floor point (robot frame) to image pixel (u right, v down)."""
        xc = x - self._fwd
        c, s = math.cos(self._tilt), math.sin(self._tilt)
        depth = xc * c + self._h * s
        right = -y
        down = -xc * s + self._h * c
        return self._cx + self._f * right / depth, self._cy + self._f * down / depth

    def warp(self, image: np.ndarray) -> np.ndarray:
        cv2 = _cv2()
        # replicate the border: a black border would look like black tape
        return cv2.warpPerspective(
            image, self.homography, (self.cols, self.rows),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE,
        )

    def to_pixel(self, x, y):
        return (self.x_max - np.asarray(x)) / self.res, (self.y_half - np.asarray(y)) / self.res

    def to_robot(self, rows, cols):
        return self.x_max - np.asarray(rows) * self.res, self.y_half - np.asarray(cols) * self.res


class LineDetector:
    """Learns the line colour and traces the line in single images."""

    def __init__(self) -> None:
        self._view: GroundView | None = None
        self._view_key = None
        self.line_lab: np.ndarray | None = None
        self.width_m = config.LINE_DEFAULT_WIDTH_M
        self._debug = None
        # a quick restart can leave the old worker thread still running
        self._work_lock = threading.Lock()
        self._debug_lock = threading.Lock()

    # --- preparation ---

    def _prepare(self, image: np.ndarray, fov_deg: float):
        cv2 = _cv2()
        h, w = image.shape[:2]
        key = (w, h, fov_deg, config.LINE_CAMERA_HEIGHT_M, config.LINE_CAMERA_FORWARD_M,
               config.LINE_CAMERA_TILT_DEG, config.LINE_BEV_RES_M, config.LINE_VIEW_MIN_M,
               config.LINE_VIEW_MAX_M, config.LINE_VIEW_HALF_WIDTH_M)
        if key != self._view_key:
            self._view = GroundView(w, h, fov_deg)
            self._view_key = key
            log.info("Line following: floor visible from %.2f m ahead", self._view.near_x)
        bev = self._view.warp(image)
        lab = cv2.cvtColor(bev, cv2.COLOR_BGR2LAB).astype(np.float32)
        return self._view, bev, lab

    @property
    def near_x(self) -> float:
        return self._view.near_x if self._view is not None else config.LINE_VIEW_MIN_M

    @staticmethod
    def _clean(mask: np.ndarray) -> np.ndarray:
        cv2 = _cv2()
        m = mask.astype(np.uint8)
        # close small gaps, then drop specks by area: an opening would also
        # erase tape thinner than ~3 px (electrical tape is 1.9 cm)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        min_px = config.LINE_MIN_SPECK_M2 / config.LINE_BEV_RES_M ** 2
        keep = np.zeros(n, bool)
        keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_px
        return keep[labels]

    # --- learning ---

    def learn(self, image: np.ndarray, fov_deg: float) -> str | None:
        """Learn the line colour from the image. Returns an error text (german)."""
        with self._work_lock:
            return self._learn(image, fov_deg)

    def _learn(self, image: np.ndarray, fov_deg: float) -> str | None:
        cv2 = _cv2()
        view, bev, lab = self._prepare(image, fov_deg)
        valid = view.valid
        if valid.sum() < 200:
            return "Kamera sieht keinen Boden (Einbau prüfen)"
        floor = np.median(lab[valid], axis=0)
        contrast = np.linalg.norm(lab - floor, axis=2)
        cand = self._clean(valid & (contrast > config.LINE_MIN_CONTRAST))
        share = cand.sum() / valid.sum()
        self._store_debug(view, bev, cand, [], None)
        if share > config.LINE_MAX_LINE_SHARE:
            return "Boden zu unruhig – keine Linie abgrenzbar"
        n, labels, stats, _ = cv2.connectedComponentsWithStats(cand.astype(np.uint8), connectivity=8)
        min_area = config.LINE_MIN_WIDTH_M * 0.10 / view.res ** 2
        ex, ey = view.near_x + config.LINE_EXPECT_MARGIN_M, 0.0
        best, best_d = None, None
        for k in range(1, n):
            if stats[k, cv2.CC_STAT_AREA] < min_area:
                continue
            rows, cols = np.nonzero(labels == k)
            xs, ys = view.to_robot(rows, cols)
            d = float(np.min(np.hypot(xs - ex, ys - ey)))
            if best_d is None or d < best_d:
                best, best_d = k, d
        if best is None:
            return "Keine Linie vor dem Roboter erkannt"
        pixels = labels == best
        self.line_lab = np.median(lab[pixels], axis=0)
        # width from the nearest piece, across the line
        rows, cols = np.nonzero(pixels)
        xs, ys = view.to_robot(rows, cols)
        i = int(np.argmin(np.hypot(xs - ex, ys - ey)))
        across = self._recenter(view, pixels, float(xs[i]), float(ys[i]), 0.0, 0.15)
        width = across[2] if across is not None else config.LINE_DEFAULT_WIDTH_M
        self.width_m = float(np.clip(width, config.LINE_MIN_WIDTH_M, config.LINE_MAX_WIDTH_M))
        log.info("Line following: learned line colour lab=%s (floor %s), width %.0f cm",
                 np.round(self.line_lab).astype(int).tolist(), np.round(floor).astype(int).tolist(),
                 self.width_m * 100)
        return None

    # --- following ---

    def detect(self, image: np.ndarray, fov_deg: float, expected: tuple[float, float],
               heading: float, radius: float | None, prefer: str) -> Detection | None:
        """Trace the line, starting near ``expected`` (robot frame)."""
        with self._work_lock:
            return self._detect(image, fov_deg, expected, heading, radius, prefer)

    def _detect(self, image, fov_deg, expected, heading, radius, prefer):
        if self.line_lab is None:
            return None
        view, bev, lab = self._prepare(image, fov_deg)
        valid = view.valid
        floor = np.median(lab[valid], axis=0)
        d_floor = np.linalg.norm(lab - floor, axis=2)
        d_line = np.linalg.norm(lab - self.line_lab, axis=2)
        mask = self._clean(
            valid & (d_line < d_floor) & (d_line < config.LINE_COLOR_TOLERANCE)
            & (d_floor > config.LINE_MIN_CONTRAST * 0.5)
        )
        path, junction = [], False
        start = self._start(view, mask, expected, heading, radius)
        if start is not None:
            path, junction = self._trace(view, mask, start[0], start[1], heading, prefer)
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        ok = length >= config.LINE_MIN_PATH_M
        self._store_debug(view, bev, mask, path if ok else [], None)
        if not ok:
            return None
        # lighting changes along the course: follow the line colour slowly
        on_line = mask & (d_line < config.LINE_COLOR_TOLERANCE * 0.7)
        if on_line.sum() > 20:
            a = config.LINE_COLOR_ADAPT
            self.line_lab = (1 - a) * self.line_lab + a * np.median(lab[on_line], axis=0)
        return Detection(path, self.width_m, junction)

    def _hit(self, view: GroundView, mask: np.ndarray, xs, ys) -> np.ndarray:
        r, c = view.to_pixel(xs, ys)
        r = np.rint(r).astype(int)
        c = np.rint(c).astype(int)
        inside = (r >= 0) & (r < view.rows) & (c >= 0) & (c < view.cols)
        out = np.zeros(r.shape, bool)
        out[inside] = mask[r[inside], c[inside]]
        return out

    def _recenter(self, view, mask, x, y, heading, half_span):
        """Move (x, y) to the middle of the line across ``heading``; also the width."""
        nx, ny = -math.sin(heading), math.cos(heading)
        offs = np.arange(-half_span, half_span + 1e-9, view.res / 2)
        hits = self._hit(view, mask, x + offs * nx, y + offs * ny)
        runs = _runs(hits)
        if not runs:
            return None

        def gap(run):
            lo, hi = offs[run[0]], offs[run[1]]
            return 0.0 if lo <= 0 <= hi else min(abs(lo), abs(hi))

        lo, hi = min(runs, key=gap)
        mid = (offs[lo] + offs[hi]) / 2
        return x + mid * nx, y + mid * ny, float(offs[hi] - offs[lo] + view.res / 2)

    def _start(self, view, mask, expected, heading, radius):
        rows, cols = np.nonzero(mask)
        if not len(rows):
            return None
        xs, ys = view.to_robot(rows, cols)
        d = np.hypot(xs - expected[0], ys - expected[1])
        i = int(np.argmin(d))
        if radius is not None and d[i] > radius:
            return None
        span = max(2 * self.width_m, 0.06)
        return self._recenter(view, mask, float(xs[i]), float(ys[i]), heading, span)

    def _branches(self, view, mask, x, y, heading, r) -> list[float]:
        """Directions (relative to heading) in which the line continues."""
        fan = config.LINE_TRACE_FAN_DEG
        rel = np.radians(np.arange(-fan, fan + 1e-6, config.LINE_TRACE_FAN_STEP_DEG))
        a = heading + rel
        hits = self._hit(view, mask, x + r * np.cos(a), y + r * np.sin(a))
        return [float(rel[s:e + 1].mean()) for s, e in _runs(hits) if e > s]

    @staticmethod
    def _choose(branches: list[float], prefer: str) -> float | None:
        if not branches:
            return None
        if len(branches) == 1:
            return branches[0]
        if prefer == LEFT:
            return max(branches)
        if prefer == RIGHT:
            return min(branches)
        straight = min(branches, key=abs)
        if abs(straight) <= math.radians(config.LINE_STRAIGHT_DEG):
            return straight
        return max(branches) if config.LINE_T_JUNCTION == LEFT else min(branches)

    def _trace(self, view, mask, x, y, heading, prefer):
        path = [(x, y)]
        junction = False
        # the step must be clearly wider than the line, otherwise the fan sees
        # the line's own edges as branches
        r = max(config.LINE_TRACE_STEP_M, 1.5 * self.width_m)
        span = max(2 * self.width_m, 0.04)
        length = 0.0
        while length < config.LINE_TRACE_MAX_M and len(path) < 200:
            branches = self._branches(view, mask, x, y, heading, r)
            junction = junction or len(branches) > 1
            rel = self._choose(branches, prefer)
            if rel is None:
                break
            h = heading + rel
            nxt = self._recenter(view, mask, x + r * math.cos(h), y + r * math.sin(h), h, span)
            if nxt is None:
                break
            nx, ny, _ = nxt
            step = math.hypot(nx - x, ny - y)
            if step < 0.3 * r:
                break
            heading = math.atan2(ny - y, nx - x)
            x, y = nx, ny
            path.append((x, y))
            length += step
        return path, junction

    # --- debug picture ---

    def _store_debug(self, view, bev, mask, path, target) -> None:
        with self._debug_lock:
            self._debug = (view, bev, mask, list(path), target)

    def set_target(self, target) -> None:
        with self._debug_lock:
            if self._debug is not None:
                self._debug = (*self._debug[:4], target)

    def debug_jpeg(self) -> bytes | None:
        """Bird's-eye view: line pixels green, traced line red, target yellow."""
        with self._debug_lock:
            debug = self._debug
        if debug is None:
            return None
        cv2 = _cv2()
        view, bev, mask, path, target = debug
        img = bev.copy()
        img[~view.valid] = (img[~view.valid] * 0.3).astype(np.uint8)
        img[mask] = (0.5 * img[mask] + 0.5 * np.array([60, 220, 60])).astype(np.uint8)
        scale = 3
        img = cv2.resize(img, (view.cols * scale, view.rows * scale), interpolation=cv2.INTER_NEAREST)

        def px(p):
            r, c = view.to_pixel(p[0], p[1])
            return int(c * scale), int(r * scale)

        if len(path) > 1:
            cv2.polylines(img, [np.array([px(p) for p in path], np.int32)], False, (40, 40, 240), 2)
        if target is not None:
            cv2.circle(img, px(target), 6, (0, 220, 255), 2)
        ok, data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return data.tobytes() if ok else None


def _decode(frame):
    cv2 = _cv2()
    return cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)


# --- Driving -------------------------------------------------------------------------


class LineFollower:
    """Follows a line on the floor. Used like ``Maneuver``: start, cancel, state."""

    def __init__(self, watchdog: SafetyWatchdog, camera, odometry) -> None:
        self._watchdog = watchdog
        self._camera = camera
        self._odometry = odometry
        self._detector = LineDetector()
        self._task: asyncio.Task[None] | None = None
        self._reason: str | None = None
        self._state = OFF
        self._text = ""
        self.prefer = STRAIGHT
        self.announcement: str | None = None
        self.finished = 0  # counts every self-ended run (for the server's message)
        self._junctions = 0
        self._in_junction = False
        self._distance = 0.0
        self._started = 0.0
        self._spinning = False
        self._last_side = 1.0
        self._error: str | None = None
        try:
            _cv2()
        except Exception as exc:
            self._error = f"OpenCV fehlt ({exc})"

    # --- control ---

    @property
    def available(self) -> bool:
        return self._camera is not None and self._error is None

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def last_reason(self) -> str | None:
        return self._reason

    def start(self, prefer: str | None = None) -> str | None:
        """Start following, or change the junction preference while running.
        Returns an error text."""
        if prefer is not None and prefer not in CHOICES:
            return f"abzweig muss {', '.join(CHOICES)} sein"
        if not self.available:
            return self._error or "Linie folgen geht nur mit Kamera"
        if prefer is not None:
            self.prefer = prefer
        if self.active:
            return None
        self.prefer = prefer or STRAIGHT
        self._reason = None
        self.announcement = None
        self._junctions = 0
        self._in_junction = False
        self._spinning = False
        self._distance = 0.0
        self._started = time.monotonic()
        self._task = asyncio.create_task(self._run())
        return None

    def cancel(self, reason: str = "abgebrochen") -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._reason = reason
            self._state = OFF
            self._text = reason
            log.info("Line following %s", reason)
        self._task = None

    def state(self) -> dict:
        active = self.active
        return {
            "aktiv": active,
            "verfuegbar": self.available,
            "zustand": self._state if active else OFF,
            "text": self._text,
            "grund": self._reason,
            "abzweig": self.prefer,
            "kreuzungen": self._junctions,
            "gefahren_m": round(self._distance, 2),
            "dauer_s": round(time.monotonic() - self._started, 1) if active else None,
            "linie_breite_cm": round(self._detector.width_m * 100, 1)
            if self._detector.line_lab is not None else None,
            "fehler": self._error,
        }

    def debug_jpeg(self) -> bytes | None:
        try:
            return self._detector.debug_jpeg()
        except Exception:
            log.debug("Line debug picture failed", exc_info=True)
            return None

    # --- sequence ---

    def _pose_now(self) -> tuple[float, float, float]:
        x, y, deg = self._odometry.pose()
        return x, y, math.radians(deg)

    def _set(self, state: str, text: str) -> None:
        self._state, self._text = state, text

    async def _run(self) -> None:
        tick = 1.0 / max(1, config.LINE_HZ)
        try:
            self._watchdog.stop()
            if not await self._learn(tick):
                return
            await self._follow(tick)
        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._reason = "Fehler beim Linienfolgen"
            self._state, self._text = OFF, self._reason
            self.finished += 1
            log.exception("Line following aborted")

    async def _learn(self, tick: float) -> bool:
        self._set(LEARN, "lerne die Linienfarbe")
        begin = time.monotonic()
        last_t = 0.0
        error = "Kamera liefert kein Bild"
        while time.monotonic() - begin < config.LINE_LEARN_S:
            frame, t = self._camera.latest_with_time()
            # only images taken after the start: the robot may just have been placed
            if frame is not None and t > last_t and t >= begin - config.LINE_LEARN_GRACE_S:
                last_t = t
                image = await asyncio.to_thread(_decode, frame)
                if image is not None:
                    error = await asyncio.to_thread(self._detector.learn, image, self._camera.fov_deg)
                    if error is None:
                        return True
            await asyncio.sleep(tick)
        self._end(error, "Ich sehe keine Linie vor mir.")
        return False

    async def _follow(self, tick: float) -> None:
        path_world: list[tuple[float, float]] = []
        last_seen = time.monotonic()
        last_t = 0.0
        last_arrival = time.monotonic()
        search: dict | None = None
        blocked_since: float | None = None
        previous_pose = self._pose_now()
        log.info("Line following started (junctions: %s)", self.prefer)

        while True:
            now = time.monotonic()
            frame, t = self._camera.latest_with_time()
            if frame is not None and t > last_t:
                last_t = t
                last_arrival = now
                if now - t <= config.LINE_FRAME_MAX_AGE_S:
                    pose_img = self._odometry.pose_at(t) or self._pose_now()
                    expected, heading, radius = self._expectation(path_world, pose_img, search)
                    det = await asyncio.to_thread(
                        self._detect, frame, expected, heading, radius, self.prefer
                    )
                    if det is not None:
                        path_world = self._merge(path_world, det.path, pose_img)
                        last_seen = time.monotonic()
                        if search is not None:
                            log.info("Line following: line found again")
                        search = None
                        if det.junction and not self._in_junction:
                            self._junctions += 1
                        self._in_junction = det.junction

            pose = self._pose_now()
            self._distance += math.dist(pose[:2], previous_pose[:2])
            previous_pose = pose
            now = time.monotonic()

            if now - last_arrival > config.LINE_CAMERA_GIVE_UP_S:
                return self._end("Kamera liefert keine Bilder mehr",
                                 "Meine Kamera zeigt nichts mehr, ich halte an.")
            if now - last_arrival > config.LINE_CAMERA_WAIT_S:
                self._set(WAIT, "wartet auf ein Kamerabild")
                self._watchdog.submit(STOP, source="autonom")
                await asyncio.sleep(tick)
                continue

            path = self._ahead(_to_robot(path_world, pose))
            cmd = None
            if search is None and path and math.hypot(*path[-1]) > config.LINE_END_REST_M:
                blind = now - last_seen > config.LINE_BLIND_S
                cmd = self._pursue(path, blind)
            elif search is None and now - last_seen < config.LINE_LOST_S:
                self._set(WAIT, "Linie kurz verloren")
                cmd = STOP
            else:
                if search is None:
                    search = {"start": pose[2], "side": self._last_side, "phase": 0}
                    path_world = []
                    log.info("Line following: line lost, searching")
                cmd = self._search(search, pose[2])
                if cmd is None:
                    return self._end("Linie nicht mehr gefunden (Ende der Strecke?)",
                                     "Ich finde die Linie nicht mehr.")

            self._watchdog.submit(cmd, source="autonom")
            verdict = self._watchdog.last_check
            if (verdict is not None and verdict.share == 0.0 and not cmd.is_stop()):
                blocked_since = blocked_since or now
                if now - blocked_since > config.LINE_BLOCKED_S:
                    reason = ("keine frischen Lidar-Daten" if verdict.reason == NO_LIDAR
                              else f"Fahrschutz: {verdict.reason}")
                    return self._end(f"{reason} -- angehalten",
                                     "Da ist etwas auf der Strecke, ich habe angehalten.")
                self._text += f" (Fahrschutz: {verdict.reason})"
            else:
                blocked_since = None
            await asyncio.sleep(tick)

    def _detect(self, frame, expected, heading, radius, prefer):
        image = _decode(frame)
        if image is None:
            return None
        return self._detector.detect(image, self._camera.fov_deg, expected, heading, radius, prefer)

    def _expectation(self, path_world, pose_img, search):
        """Where the line should enter the view, its direction there, search radius."""
        near = self._detector.near_x + config.LINE_EXPECT_MARGIN_M
        if search is not None or not path_world:
            return (near, 0.0), 0.0, None
        path = _to_robot(path_world, pose_img)
        for i, (x, y) in enumerate(path):
            if x >= near:
                # direction of the segment at that point
                a, b = (i, i + 1) if i + 1 < len(path) else (i - 1, i)
                heading = (math.atan2(path[b][1] - path[a][1], path[b][0] - path[a][0])
                           if a >= 0 else 0.0)
                return (x, y), heading, config.LINE_START_RADIUS_M
        # remembered line lies entirely in the blind zone (corner): look from its end
        if len(path) > 1:
            heading = math.atan2(path[-1][1] - path[-2][1], path[-1][0] - path[-2][0])
        else:
            heading = 0.0
        return path[-1], heading, 2 * config.LINE_START_RADIUS_M

    @staticmethod
    def _merge(old_world, new_robot, pose_img):
        """Remembered line up to where the new trace starts, then the new trace."""
        new_world = _to_world(new_robot, pose_img)
        if not old_world:
            return new_world
        here = pose_img[:2]
        i0 = min(range(len(old_world)), key=lambda i: math.dist(old_world[i], here))
        start = new_world[0]
        rest = old_world[i0:]
        j = min(range(len(rest)), key=lambda i: math.dist(rest[i], start))
        # the old line does not lead to the new one: rather forget it
        if math.dist(rest[j], start) > config.LINE_START_RADIUS_M:
            return new_world
        return rest[:j] + new_world

    @staticmethod
    def _ahead(path):
        """Drop the part the robot has already passed."""
        if not path:
            return []
        i0 = min(range(len(path)), key=lambda i: math.hypot(*path[i]))
        return path[i0:]

    def _pursue(self, path, blind: bool) -> DriveCommand:
        target = next((p for p in path if math.hypot(*p) >= config.LINE_LOOKAHEAD_M), path[-1])
        self._detector.set_target(target)
        tx, ty = target
        dist = max(math.hypot(tx, ty), 0.05)
        alpha = math.atan2(ty, tx)
        if abs(alpha) > math.radians(5):
            self._last_side = math.copysign(1.0, alpha)

        limit = math.radians(config.LINE_TURN_IN_PLACE_DEG)
        if abs(alpha) > limit or (self._spinning and abs(alpha) > limit / 2):
            self._spinning = True
            self._set(TURN, "dreht in die Kurve")
            return DriveCommand(0.0, 0.0, math.copysign(config.LINE_TURN_OMEGA, alpha))
        self._spinning = False

        curvature = 2 * math.sin(alpha) / dist
        v = config.LINE_SPEED / (1 + config.LINE_CURVE_SLOWDOWN * abs(curvature))
        if blind:
            v *= config.LINE_BLIND_SPEED_SHARE
        v = max(v, config.LINE_MIN_SPEED)
        w = v * curvature
        if abs(w) > config.LINE_MAX_OMEGA:
            v *= config.LINE_MAX_OMEGA / abs(w)
            w = math.copysign(config.LINE_MAX_OMEGA, w)
        if blind:
            self._set(BLIND, "fährt auf der gemerkten Linie")
        else:
            self._set(FOLLOW, "folgt der Linie" + (" (Abzweig: " + self.prefer + ")"
                                                   if self.prefer != STRAIGHT else ""))
        return DriveCommand(v, 0.0, w)

    def _search(self, search: dict, heading: float) -> DriveCommand | None:
        """Turn to one side, then to the other; None when both are done."""
        turned = _wrap(heading - search["start"])
        side = search["side"] if search["phase"] == 0 else -search["side"]
        if side * turned >= math.radians(config.LINE_SEARCH_DEG):
            if search["phase"] == 1:
                return None
            search["phase"] = 1
            side = -side
        self._detector.set_target(None)
        self._set(SEARCH, "sucht die Linie")
        return DriveCommand(0.0, 0.0, side * config.LINE_SEARCH_OMEGA)

    def _end(self, reason: str, announcement: str | None = None) -> None:
        self._watchdog.stop()
        self._reason = reason
        self._state, self._text = OFF, reason
        self.announcement = announcement
        self.finished += 1
        log.warning("Line following: %s", reason)
