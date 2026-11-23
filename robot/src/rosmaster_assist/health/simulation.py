"""Mock only: put a lying person into the imaginary room.

Camera (pose simulation) and lidar (extra obstacles) see the same person at
the same spot, depending on where the robot *really* stands. So the whole
healthbot procedure can be played through at the desk: drive there, search,
observe, nudge.
"""

from __future__ import annotations

import math

#: where the person lies (world coordinates, m) -- free in the imaginary
#: room, away from wall and pillar
STANDARD_LOCATION = (2.5, -1.0)
# the body as a chain of overlapping circles across the view from the start:
# head/torso, hip, legs; about 1.4 m long like a lying human. the circles
# overlap so the lidar sees one continuous piece as with a real body
_CIRCLES = ((0.0, -0.55, 0.15), (0.0, -0.275, 0.17), (0.0, 0.0, 0.17),
            (0.0, 0.275, 0.15), (0.0, 0.55, 0.13))

SCENES = ("liegend_regungslos", "liegend_bewegt")


class Simulation:
    def __init__(self, pose, pose_source, set_obstacles=None) -> None:
        """``pose_source()`` returns (x, y, theta in radians) of the mock drive,
        ``set_obstacles(circles)`` comes from the mock lidar."""
        self._pose = pose
        self._pose_source = pose_source
        self._set_obstacles = set_obstacles
        self._scene: str | None = None
        self._spot = STANDARD_LOCATION

    @property
    def active(self) -> bool:
        return self._scene is not None

    @property
    def scene(self) -> str | None:
        return self._scene

    def status(self) -> dict:
        return {"szene": self._scene, "x": self._spot[0], "y": self._spot[1]}

    def circles(self) -> list[tuple[float, float, float]]:
        x, y = self._spot
        return [(x + dx, y + dy, r) for dx, dy, r in _CIRCLES]

    def set(self, scene: str | None, x: float | None = None, y: float | None = None) -> None:
        """Set the scene (None = no person any more)."""
        if scene is not None and scene not in SCENES:
            raise ValueError(f"unbekannte Szene {scene!r} (erlaubt: {', '.join(SCENES)})")
        if x is not None and y is not None:
            self._spot = (float(x), float(y))
        self._scene = scene
        if self._set_obstacles is not None:
            self._set_obstacles(self.circles() if scene else [])
        self._pose.set_simulation(scene, self._location if scene else None)

    def _location(self) -> tuple[float, float] | None:
        """(angle in degrees, left positive; distance to the body edge in m)
        from the robot -- what camera and lidar would see from here."""
        rx, ry, theta = self._pose_source()
        x, y = self._spot
        angle = math.degrees(math.atan2(y - ry, x - rx) - theta)
        angle = (angle + 180.0) % 360.0 - 180.0
        distance = min(math.hypot(kx - rx, ky - ry) - r for kx, ky, r in self.circles())
        return angle, max(0.05, distance)
