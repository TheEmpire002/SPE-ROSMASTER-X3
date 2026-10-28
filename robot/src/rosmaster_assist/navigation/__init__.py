"""Positionsverfolgung und automatische Fahrten.

Die Bausteine:

``Odometry``            schreibt die Position aus der Fahrbewegung fort (driftet)
``Localizer``           zieht sie per Lidar-Abgleich wieder gerade (driftet nicht)
``Homing``              fährt damit zum Startpunkt zurück
``Hinderniserkennung``  benennt, was der Lidar sieht (Wand, Stuhl, Tisch, Mensch)
``Erkunder``            fährt autonom umher und weicht allem aus

Warum beide Ortungsverfahren: Die Koppelnavigation ist immer verfügbar,
aber ungenau. Der Lidar ist genau, braucht aber Wiedererkennung. Zusammen
ergänzen sie sich -- Details in ``localizer.py``.
"""

from .explorer import Erkunder
from .hindernisse import Hinderniserkennung
from .homing import Homing
from .localizer import Localizer
from .odometry import Odometry

__all__ = ["Erkunder", "Hinderniserkennung", "Homing", "Localizer", "Odometry"]
