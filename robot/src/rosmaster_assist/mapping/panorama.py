"""All-round panoramas at a point of the map.

The robot turns on the spot in ``PANORAMA_SHOTS`` steps and takes a photo
with the forward camera after each one. The turns are ``Maneuver`` turns:
measured by the lidar, through the watchdog with drive guard and the check
for room to turn, and ended by every stop or manual input like any other
drive command.

**Stitching by direction, not by features.** OpenCV's ``Stitcher`` looks
for matching features in neighbouring images; on white walls there are
none, and it costs a lot of CPU (load spikes trigger USB dropouts on the
X3). Here every shot is projected onto a cylinder at the heading the lidar
measured for it, and overlaps are blended softly. A heading error of a few
degrees shows as a small seam instead of a failed panorama.

Panorama columns run clockwise from left to right, like turning on the spot
to the right; the centre column is the heading of the first shot. The
result is stored with the map's photos (``MapPhotos.add_panorama``).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .. import config
from ..navigation import maneuver as mv

log = logging.getLogger(__name__)

OFF = "aus"
RUNNING = "laeuft"


def stitch(shots, hfov_deg: float):
    """Cylindrical panorama from ``[(image_bgr, heading_rad), ...]``.

    the centre column looks along the first shot's heading; returns the
    image (bgr) or None
    """
    import cv2
    import numpy as np

    if not shots:
        return None
    h, w = shots[0][0].shape[:2]
    half = math.radians(hfov_deg) / 2
    f = (w / 2) / math.tan(half)
    f_out = f * config.PANORAMA_SCALE
    width = int(round(2 * math.pi * f_out))
    # between two shots the cylinder reaches up to 1/cos(step/2) beyond the
    # image's top and bottom edge; slightly lower, so no black scallops remain
    height = int(round(h * config.PANORAMA_SCALE * 0.95))
    centre = shots[0][1]
    cx, cy = (w - 1) / 2, (h - 1) / 2

    # azimuth of every column: left edge = centre + 180 deg, then clockwise
    u = np.arange(width, dtype=np.float32)
    azimuth = centre + math.pi - (u + 0.5) * (2 * math.pi / width)
    # height on the cylinder as tangent of the elevation
    v = np.arange(height, dtype=np.float32)
    elevation = (v - (height - 1) / 2) / f_out

    layers = []
    for image, heading in shots:
        if image.shape[:2] != (h, w):
            image = cv2.resize(image, (w, h))
        rel = np.arctan2(np.sin(azimuth - heading), np.cos(azimuth - heading))
        cols = np.nonzero(np.abs(rel) < half * 0.98)[0]
        if not len(cols):
            continue
        a = rel[cols]
        # a point left of the camera's axis (a > 0) lies left in the image
        map_x = np.broadcast_to(cx - f * np.tan(a), (height, len(cols)))
        map_y = cy + elevation[:, None] * (f / np.cos(a))[None, :]
        warped = cv2.remap(image, map_x.astype(np.float32), map_y.astype(np.float32),
                           cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        inside = (map_y >= 0) & (map_y <= h - 1)
        layers.append((cols, warped.astype(np.float32), inside, a))

    gains = _gains(layers) if config.PANORAMA_GAIN else [1.0] * len(layers)
    total = np.zeros((height, width, 3), np.float32)
    weights = np.zeros((height, width), np.float32)
    for (cols, warped, inside, a), gain in zip(layers, gains):
        # nearly winner-take-all: the shot that looked most directly at a
        # column dominates, so close objects (parallax, the camera is not on
        # the turning axis) do not show twice; blended only near the seam
        weight = ((1.0 - np.abs(a) / half) ** config.PANORAMA_BLEND_POWER)[None, :] * inside
        total[:, cols] += warped * (gain * weight)[..., None]
        weights[:, cols] += weight
    seen = weights > 1e-6
    out = np.zeros_like(total)
    out[seen] = total[seen] / weights[seen][:, None]
    return out.clip(0, 255).astype(np.uint8)


def _gains(layers) -> list[float]:
    """One brightness factor per shot, so neighbours match in their overlap.

    the camera exposes every shot by itself (a window in view makes the
    next one darker). least squares like opencv's gain compensator: equal
    brightness in each overlap, factors close to 1
    """
    import numpy as np

    n = len(layers)
    if n < 2:
        return [1.0] * n
    alpha = 1.0 / config.PANORAMA_GAIN_NOISE ** 2
    beta = 1.0 / config.PANORAMA_GAIN_SPREAD ** 2
    A = np.zeros((n, n))
    b = np.zeros(n)
    for i in range(n):
        for j in range(i + 1, n):
            ci, wi, ii, _ = layers[i]
            cj, wj, ij, _ = layers[j]
            common, pi, pj = np.intersect1d(ci, cj, assume_unique=True, return_indices=True)
            if len(common) == 0:
                continue
            both = ii[:, pi] & ij[:, pj]
            count = float(both.sum())
            if count < 100:
                continue
            mi = float(wi[:, pi][both].mean())
            mj = float(wj[:, pj][both].mean())
            for a_, b_, ma, mb in ((i, j, mi, mj), (j, i, mj, mi)):
                A[a_, a_] += count * (alpha * ma * ma + beta)
                A[a_, b_] -= count * alpha * ma * mb
                b[a_] += count * beta
    try:
        gains = np.linalg.solve(A + np.eye(n) * 1e-9, b)
    except np.linalg.LinAlgError:
        return [1.0] * n
    lo, hi = config.PANORAMA_GAIN_LIMITS
    return [float(min(hi, max(lo, g))) if np.isfinite(g) else 1.0 for g in gains]


class Panorama:
    """Turns, shoots, stitches. Used like ``Maneuver``: start, cancel, state."""

    def __init__(self, photos, maneuver, localization, camera, busy=None) -> None:
        # returns a reason (german) when another mode drives, else None
        self._busy = busy
        self._photos = photos
        self._maneuver = maneuver
        self._localization = localization
        self._camera = camera
        self._task: asyncio.Task[None] | None = None
        self._state = OFF
        self._text = ""
        self._step = 0
        self.reason: str | None = None

    @property
    def available(self) -> bool:
        return (self._camera is not None and self._maneuver is not None
                and self._maneuver.available and self._photos is not None
                and self._photos.available)

    @property
    def active(self) -> bool:
        return self._task is not None and not self._task.done()

    def state(self) -> dict:
        active = self.active
        return {
            "verfuegbar": self.available,
            "aktiv": active,
            "text": self._text if active else None,
            "schritt": self._step if active else None,
            "von": config.PANORAMA_SHOTS,
            "grund": self.reason,
        }

    def start(self) -> str | None:
        """Start here; returns an error text (german)."""
        if not self.available:
            return "Panorama geht nur mit Kamera und Lidar"
        if self.active:
            return "Panorama läuft schon"
        if self._photos.map_here() is None:
            return "Keine Karte, in der er seine Lage kennt"
        if not self._localization.confident or self._localization.pose() is None:
            return "Lage in der Karte ist gerade unsicher"
        self.reason = None
        self._task = asyncio.create_task(self._run())
        return None

    def cancel(self, reason: str = "abgebrochen") -> None:
        if self.active:
            self._task.cancel()
            self.reason = reason
            self._maneuver.cancel(f"Panorama {reason}")
            log.info("Panorama %s", reason)
        self._task = None

    async def _run(self) -> None:
        shots_wanted = max(3, int(config.PANORAMA_SHOTS))
        step_deg = 360.0 / shots_wanted
        current = self._photos.map_here()
        start_pose = self._localization.pose()
        # turned so far, measured by the lidar during each turn
        turned = 0.0
        shots: list[tuple[bytes, float]] = []
        self._photos.paused = True
        self._maneuver.quiet = True
        try:
            for i in range(shots_wanted):
                self._step = i + 1
                self._set(f"Foto {i + 1} von {shots_wanted}")
                frame = await self._fresh_frame()
                if frame is None:
                    return self._end("kein frisches Kamerabild")
                shots.append((frame, start_pose[2] + turned))
                if i == shots_wanted - 1:
                    break
                other = self._busy() if self._busy is not None else None
                if other:
                    return self._end(f"abgebrochen: {other}")
                self._set(f"dreht weiter ({i + 1}/{shots_wanted})")
                # clockwise, so the panorama fills from left to right
                error = self._maneuver.start(mv.TURN, -step_deg)
                if error:
                    return self._end(error)
                while self._maneuver.active:
                    await asyncio.sleep(0.05)
                measured = self._maneuver.state()["gedreht_grad"]
                if abs(measured) < step_deg * 0.5:
                    return self._end(f"Drehung hat nicht geklappt ({self._maneuver.last_reason})")
                turned += math.radians(measured)
            self._set("setzt das Panorama zusammen …")
            jpeg = await asyncio.to_thread(self._build, shots)
            if jpeg is None:
                return self._end("Panorama ließ sich nicht zusammensetzen")
            entry = await asyncio.to_thread(self._photos.add_panorama, current, start_pose,
                                            jpeg, shots)
            if entry is None:
                return self._end(self._photos.last_error or "Panorama nicht gespeichert")
            self.reason = "fertig"
            log.info("Panorama %s with %d shots at x=%.2f y=%.2f", entry["id"], len(shots),
                     start_pose[0], start_pose[1])
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("Panorama failed")
            self.reason = "Fehler beim Panorama"
        finally:
            self._photos.paused = False
            self._maneuver.quiet = False
            self._state = OFF

    def _set(self, text: str) -> None:
        self._state, self._text = RUNNING, text

    def _end(self, reason: str) -> None:
        self.reason = reason
        log.warning("Panorama: %s", reason)

    async def _fresh_frame(self) -> bytes | None:
        """A camera image taken after the robot came to rest."""
        settled = time.monotonic() + config.PANORAMA_SETTLE_S
        deadline = settled + 2.0
        await asyncio.sleep(config.PANORAMA_SETTLE_S)
        while time.monotonic() < deadline:
            frame, taken = self._camera.latest_with_time()
            if frame is not None and taken >= settled and "jpeg" in str(frame[1]):
                return frame[0]
            await asyncio.sleep(0.05)
        return None

    def _build(self, shots) -> bytes | None:
        import cv2
        import numpy as np

        images = []
        for data, heading in shots:
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                images.append((image, heading))
        started = time.perf_counter()
        pano = stitch(images, self._camera.fov_deg)
        if pano is None:
            return None
        ok, out = cv2.imencode(".jpg", pano, [cv2.IMWRITE_JPEG_QUALITY, config.PANORAMA_JPEG_QUALITY])
        log.info("Panorama stitched in %.1f s (%d x %d)", time.perf_counter() - started,
                 pano.shape[1], pano.shape[0])
        return out.tobytes() if ok else None
