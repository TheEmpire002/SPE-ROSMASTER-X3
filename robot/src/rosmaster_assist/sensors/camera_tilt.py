"""Tilt the person camera (Logitech C270) up and down with a servo.

The C270 sits low on the robot. A person standing 1 m in front of it
reaches beyond the top of the picture: head and raised arms (gestures) are
cut off. A servo on the expansion board tilts the camera.

- **Angle.** 0 = horizontal, positive = up, limited to
  ``CAMERA_TILT_MIN_DEG`` .. ``CAMERA_TILT_MAX_DEG``. Mapped to the servo
  with ``CAMERA_TILT_SERVO_CENTER_DEG`` and ``CAMERA_TILT_SERVO_SIGN``.
- **Manual.** From the app (``camera_tilt`` with ``winkel`` or
  ``schritt``); switches auto off.
- **Auto.** While pose detection runs (interaction, following) it keeps the
  head of the person in question at ``CAMERA_TILT_HEAD_Y`` of the image
  height. The pose arrives with delay, so the correction starts from the
  tilt the camera had when the image was taken, not from the current one;
  otherwise it would correct the same error several times and overshoot.
  Nobody in view for ``CAMERA_TILT_RETURN_S``: back slowly to the home tilt.
- **Gentle.** At most ``CAMERA_TILT_SPEED_DEG_S``: a jerk blurs the picture,
  and a servo starting at full force pulls a current spike from the board
  (USB dropouts on the X3).
- **Dropouts.** Commands go through the driver, which reconnects by itself.
  After a reconnect the angle is sent again, the board may have lost it.

The servo does not drive the robot, so it does not go through the
watchdog.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections import deque
from typing import Callable

from .. import config
from ..drive.base import DriveBase

log = logging.getLogger(__name__)

# nose, left eye, right eye in the yolov8 pose keypoints
_HEAD_POINTS = (0, 1, 2)


def _clamp(angle: float) -> float:
    return max(config.CAMERA_TILT_MIN_DEG, min(config.CAMERA_TILT_MAX_DEG, float(angle)))


def head_y(person: dict) -> float | None:
    """Height of the head in the image (0 top .. 1 bottom), may be < 0.

    from nose and eyes if seen; otherwise estimated from the top of the box.
    a box touching the top edge means the head is above the picture
    """
    kp = person.get("kp") or []
    ys = [kp[i][1] for i in _HEAD_POINTS
          if i < len(kp) and len(kp[i]) > 2 and kp[i][2] >= config.CAMERA_TILT_KP_MIN]
    if ys:
        return sum(ys) / len(ys)
    box = person.get("box")
    if not box or len(box) < 4:
        return None
    if box[1] <= config.CAMERA_TILT_BOX_TOP_CUT:
        return config.CAMERA_TILT_HEAD_ABOVE_Y
    return box[1] + config.CAMERA_TILT_HEAD_BOX_SHARE * (box[3] - box[1])


def vertical_fov(horizontal_deg: float) -> float:
    half = math.radians(horizontal_deg) / 2
    return math.degrees(2 * math.atan(math.tan(half) * config.CAMERA_TILT_ASPECT))


class CameraTilt:
    def __init__(
        self,
        driver: DriveBase,
        person_source: Callable[[], dict | None] | None = None,
        fov_deg: float | None = None,
    ) -> None:
        self._driver = driver
        # the person whose head is kept in the frame, or None
        self._person_source = person_source
        self._vfov = vertical_fov(fov_deg if fov_deg else config.CAMERA_FOV_DEG)
        self.auto = bool(config.CAMERA_TILT_AUTO)
        home = _clamp(config.CAMERA_TILT_HOME_DEG)
        self._target = home
        self._angle = home      # where the servo is commanded to right now
        self._sent: int | None = None  # last servo value sent
        self._was_connected = False
        # commanded tilt over the last seconds, to look up the tilt of an image
        self._history: deque[tuple[float, float]] = deque(maxlen=4 * max(1, config.CAMERA_TILT_HZ))
        self._last_image = None
        self._last_person = time.monotonic()
        self._tracking = False
        self._task: asyncio.Task[None] | None = None

    @property
    def available(self) -> bool:
        return bool(config.CAMERA_TILT_ENABLED)

    # --- control ---

    def start(self) -> None:
        if self.available and self._task is None:
            self._task = asyncio.create_task(self._loop())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None

    def set(self, angle: float) -> None:
        """Manual angle (degrees, + = up); switches auto off."""
        self.auto = False
        self._target = _clamp(angle)

    def step(self, delta: float) -> None:
        self.set(self._target + delta)

    def set_auto(self, on: bool) -> None:
        self.auto = bool(on)
        # do not turn back home right away when nobody is seen yet
        self._last_person = time.monotonic()
        self._last_image = None

    def state(self) -> dict:
        return {
            "verfuegbar": self.available,
            "winkel": round(self._angle, 1),
            "ziel": round(self._target, 1),
            "auto": self.auto,
            "min": config.CAMERA_TILT_MIN_DEG,
            "max": config.CAMERA_TILT_MAX_DEG,
            "verfolgt": self.auto and self._tracking,
        }

    # --- loop ---

    async def _loop(self) -> None:
        tick = 1.0 / max(1, config.CAMERA_TILT_HZ)
        last = time.monotonic()
        while True:
            await asyncio.sleep(tick)
            now = time.monotonic()
            dt, last = min(0.5, now - last), now
            try:
                if self.auto:
                    self._track(now, dt)
                else:
                    self._tracking = False
                await self._move(now, dt)
            except Exception:
                log.exception("Camera tilt step failed")

    def _track(self, now: float, dt: float) -> None:
        person = self._person_source() if self._person_source is not None else None
        if person is None:
            self._tracking = False
            if now - self._last_person > config.CAMERA_TILT_RETURN_S:
                home = _clamp(config.CAMERA_TILT_HOME_DEG)
                step = config.CAMERA_TILT_RETURN_SPEED_DEG_S * dt
                self._target += max(-step, min(step, home - self._target))
            return
        self._last_person = now
        self._tracking = True
        image_time = person.get("zeit")
        if image_time is not None and image_time == self._last_image:
            return  # same image as last time, its error is already corrected
        self._last_image = image_time
        y = head_y(person)
        if y is None:
            return
        error = y - config.CAMERA_TILT_HEAD_Y
        if abs(error) < config.CAMERA_TILT_DEADBAND:
            return
        base = self._tilt_at(image_time) if image_time is not None else self._angle
        # image y points down: a head too high (error < 0) needs tilting up
        self._target = _clamp(base - error * self._vfov * config.CAMERA_TILT_GAIN)

    def _tilt_at(self, t: float) -> float:
        """Commanded tilt at time ``t`` (the servo follows within a few ms)."""
        angle = self._angle
        for when, a in reversed(self._history):
            angle = a
            if when <= t:
                break
        return angle

    async def _move(self, now: float, dt: float) -> None:
        step = config.CAMERA_TILT_SPEED_DEG_S * dt
        self._angle += max(-step, min(step, self._target - self._angle))
        self._history.append((now, self._angle))

        connected = self._driver.connected
        if connected and not self._was_connected:
            self._sent = None  # (re)connected: the board may have lost the angle
        self._was_connected = connected
        if not connected:
            return
        servo = round(config.CAMERA_TILT_SERVO_CENTER_DEG + config.CAMERA_TILT_SERVO_SIGN * self._angle)
        servo = max(0, min(180, servo))
        if servo == self._sent:
            return
        # serial write, keep it off the event loop
        if await asyncio.to_thread(self._driver.set_servo, config.CAMERA_TILT_CHANNEL, servo):
            self._sent = servo
