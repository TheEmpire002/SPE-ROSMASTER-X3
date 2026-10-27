"""Positionsverfolgung und automatische Fahrten.

Drei Bausteine:

``Odometry``   schreibt die Position aus der Fahrbewegung fort (driftet)
``Localizer``  zieht sie per Lidar-Abgleich wieder gerade (driftet nicht)
``Homing``     fährt damit zum Startpunkt zurück

Warum beide Ortungsverfahren: Die Koppelnavigation ist immer verfügbar,
aber ungenau. Der Lidar ist genau, braucht aber Wiedererkennung. Zusammen
ergänzen sie sich -- Details in ``localizer.py``.
"""

from .homing import Homing
from .localizer import Localizer
from .odometry import Odometry

__all__ = ["Homing", "Localizer", "Odometry"]
