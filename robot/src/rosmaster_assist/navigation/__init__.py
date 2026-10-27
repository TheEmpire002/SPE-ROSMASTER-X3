"""Positionsverfolgung und automatische Fahrten.

Derzeit nur Koppelnavigation und die Rückfahrt zum Startpunkt. Für echtes
autonomes Fahren fehlt die Hinderniserkennung -- dafür wäre der Lidar da,
siehe E1a in docs/entscheidungen.md.
"""

from .homing import Homing
from .odometry import Odometry

__all__ = ["Homing", "Odometry"]
