"""Obstacle memory: what the depth camera has seen, even when it no longer
sees it -- and where the robot got stuck.

The Astra only measures from ~0.6 m. A chair leg therefore appears at 1.5 m
and disappears from the image again at 0.6 m -- right before it matters.
That is why the obstacle points are remembered in a fixed frame and
transformed back into the robot frame using the robot's own motion.

A point is cleared when the camera sees free floor at its location again
(chair moved away, false measurement), when it is too old or too far away.
On top of that come **phantom obstacles** from the progress watch: if the
robot made no progress at a spot despite a drive command, something nobody
sees is there. The camera does not clear these points -- it sees nothing
there, after all --, they only expire with time.

The frame comes from ``pose_source``: the lidar odometry while mapping,
otherwise the odometry. If the source changes or the pose jumps (starting
point reset), the remembered points no longer fit and are discarded.
"""

from __future__ import annotations

import logging
import math

import numpy as np

from .. import config

log = logging.getLogger(__name__)

#: The pose does not jump further than this between two ticks -- otherwise it
#: was reset, and the remembered points belong somewhere else.
_MAX_JUMP_M = 0.5


def _to_world(points, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + points[:, 0] * c - points[:, 1] * s,
                            y + points[:, 0] * s + points[:, 1] * c))


def _to_robot(points, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = points[:, 0] - x, points[:, 1] - y
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


class ObstacleMemory:
    """Depth and phantom obstacles, retrievable in the robot frame."""

    def __init__(self, depth, pose_source) -> None:
        self._depth = depth
        self._pose_source = pose_source  # () -> (name, (x, y, th)) or None
        self._source: str | None = None
        self._pose: tuple | None = None
        self._frame_time: float | None = None
        self._world = np.zeros((0, 2))
        self._time = np.zeros(0)
        self._phantom = np.zeros(0, dtype=bool)
        #: Changes on every modification -- drive guard and explorer then
        #: recompute, otherwise they reuse the last result.
        self.version = 0
        self._robot_version = -1
        self._robot = np.zeros((0, 2))

    def clear(self) -> None:
        self._world = np.zeros((0, 2))
        self._time = np.zeros(0)
        self._phantom = np.zeros(0, dtype=bool)
        self.version += 1

    @property
    def count(self) -> int:
        return int(len(self._world))

    @property
    def phantoms(self) -> int:
        return int(self._phantom.sum())

    def update(self, now: float) -> None:
        """Track the pose, take over a new depth image, discard old points.

        May be called as often as desired; computation only happens for new data.
        """
        source = self._pose_source()
        if source is None:
            return
        name, pose = source
        if name != self._source or (
            self._pose is not None
            and math.hypot(pose[0] - self._pose[0], pose[1] - self._pose[1]) > _MAX_JUMP_M
        ):
            if self.count:
                log.info("Obstacle memory discarded (pose %s)",
                         "reset" if name == self._source else f"now from {name}")
            self.clear()
            self._source = name
        if pose != self._pose:
            self._pose = pose
            self.version += 1

        image = self._depth.image() if self._depth is not None else None
        if image is not None and image.timestamp != self._frame_time:
            self._frame_time = image.timestamp
            self._feed(image, pose)

        if self.count:
            age = now - self._time
            far = np.hypot(self._world[:, 0] - pose[0], self._world[:, 1] - pose[1])
            keep = (far <= config.DEPTH_MEMORY_M) & np.where(
                self._phantom, age <= config.PROGRESS_PHANTOM_S,
                age <= config.DEPTH_MEMORY_S)
            if not keep.all():
                self._keep(keep)

    def _feed(self, image, pose) -> None:
        cell = config.DEPTH_CELL_M
        timestamp = image.timestamp
        # Clearing: camera points at whose location it now sees free floor.
        if self.count and len(image.free):
            free = {(int(i), int(j)) for i, j in np.floor(image.free / cell).astype(int)}
            here = np.floor(_to_robot(self._world, pose) / cell).astype(int)
            seen = np.array([(int(i), int(j)) in free for i, j in here])
            path = seen & ~self._phantom
            if path.any():
                self._keep(~path)
        if len(image.obstacles) == 0:
            return
        new = _to_world(np.asarray(image.obstacles, dtype=float), pose)
        # Not the same world cell twice: the old camera point gives way to the new one.
        if self.count:
            new_cells = {(int(i), int(j)) for i, j in np.floor(new / cell).astype(int)}
            old_cells = np.floor(self._world / cell).astype(int)
            duplicate = np.array([(int(i), int(j)) in new_cells for i, j in old_cells])
            duplicate &= ~self._phantom
            if duplicate.any():
                self._keep(~duplicate)
        self._world = np.concatenate((self._world, new))
        self._time = np.concatenate((self._time, np.full(len(new), timestamp)))
        self._phantom = np.concatenate((self._phantom, np.zeros(len(new), dtype=bool)))
        self.version += 1

    def phantom(self, points_robot, now: float) -> None:
        """Enter an assumed obstacle at these locations (robot frame)."""
        if self._pose is None or len(points_robot) == 0:
            return
        new = _to_world(np.asarray(points_robot, dtype=float), self._pose)
        self._world = np.concatenate((self._world, new))
        self._time = np.concatenate((self._time, np.full(len(new), now)))
        self._phantom = np.concatenate((self._phantom, np.ones(len(new), dtype=bool)))
        self.version += 1

    def points(self) -> np.ndarray:
        """All remembered points in the current robot frame (Nx2)."""
        if self._pose is None or not self.count:
            return np.zeros((0, 2))
        if self._robot_version != self.version:
            self._robot = _to_robot(self._world, self._pose)
            self._robot_version = self.version
        return self._robot

    def _keep(self, mask) -> None:
        self._world = self._world[mask]
        self._time = self._time[mask]
        self._phantom = self._phantom[mask]
        self.version += 1

    def status(self) -> dict:
        return {"punkte": self.count, "phantome": self.phantoms, "rahmen": self._source}
