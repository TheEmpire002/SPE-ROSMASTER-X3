"""Align two lidar scans on top of each other (ICP).

The idea: when the start point is set, a scan records once what the room
looks like from there. Later the current scan is shifted and rotated until
it fits the reference as closely as possible. The shift needed for that
*is* the robot's position relative to the start point.

The big difference to dead reckoning: this method accumulates nothing.
Every measurement is valid on its own, an error does not propagate.
Spinning wheels do not matter to it -- it looks at the walls.

Method: Iterative Closest Point. Every point is assigned to the nearest
reference point, the best rotation and translation is computed from that,
and the whole thing is repeated a few times. This needs a usable initial
guess -- dead reckoning provides it.
"""

from __future__ import annotations

import logging
import math

log = logging.getLogger(__name__)

#: Point pairs farther apart than this are not considered to belong together.
#: Too large and unrelated points skew the solution, too small finds nothing.
MAX_PAIR_DISTANCE_M = 0.5

#: Below this many pairs the result is not meaningful.
MIN_PAIRS = 40


class MatchResult:
    """Result of a match."""

    __slots__ = ("x", "y", "theta", "residuum", "pairs")

    def __init__(self, x: float, y: float, theta: float, residuum: float, pairs: int):
        self.x = x
        self.y = y
        self.theta = theta          # radians
        self.residuum = residuum    # mean residual distance in metres
        self.pairs = pairs

    def __repr__(self) -> str:
        return (
            f"MatchResult(x={self.x:.3f}, y={self.y:.3f}, "
            f"theta={math.degrees(self.theta):.1f}°, "
            f"residuum={self.residuum:.3f} m, pairs={self.pairs})"
        )


def match(
    current,
    reference,
    start: tuple[float, float, float],
    iterations: int = 25,
) -> MatchResult | None:
    """Finds the alignment in which ``current`` best fits ``reference``.

    ``current``   Nx2 array, points in the robot coordinate frame
    ``reference`` Mx2 array, points in the coordinate frame of the start point
    ``start``     (x, y, angle in radians) as the initial guess

    Returns None if too few points find a partner -- for example because
    the robot is now standing in a different room.
    """
    import numpy as np

    if current is None or reference is None:
        return None
    if len(current) < MIN_PAIRS or len(reference) < MIN_PAIRS:
        return None

    A = np.asarray(current, dtype=float)
    R = np.asarray(reference, dtype=float)

    x, y, theta = start
    translation = np.array([x, y], dtype=float)

    residuum = float("inf")
    pairs = 0

    for _ in range(iterations):
        # Apply the current estimate.
        c, s = math.cos(theta), math.sin(theta)
        turn = np.array([[c, -s], [s, c]])
        turned = A @ turn.T + translation

        # Find the nearest reference point for every point. With a few
        # hundred points the full distance matrix is faster than any tree
        # structure -- and needs no extra package.
        diff = turned[:, None, :] - R[None, :, :]
        distances = np.einsum("ijk,ijk->ij", diff, diff)  # squared, saves the square root
        nearest = np.argmin(distances, axis=1)
        min_sq = distances[np.arange(len(turned)), nearest]

        valid = min_sq < MAX_PAIR_DISTANCE_M**2
        pairs = int(valid.sum())
        if pairs < MIN_PAIRS:
            return None

        P = turned[valid]
        Q = R[nearest[valid]]
        residuum = float(np.sqrt(min_sq[valid]).mean())

        # Best rotation and translation for this assignment, solvable in
        # closed form (Kabsch in the plane).
        pm, qm = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pm).T @ (Q - qm)
        dtheta = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])

        dc, ds = math.cos(dtheta), math.sin(dtheta)
        drot = np.array([[dc, -ds], [ds, dc]])
        dshift = qm - drot @ pm

        # Apply the correction on top of the previous estimate.
        translation = drot @ translation + dshift
        theta = _normalize(theta + dtheta)

        # If almost nothing moves any more, further iterations gain nothing.
        if abs(dtheta) < 1e-4 and float(np.hypot(*dshift)) < 1e-4:
            break

    return MatchResult(float(translation[0]), float(translation[1]), theta, residuum, pairs)


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
