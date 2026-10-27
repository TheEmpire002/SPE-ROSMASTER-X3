"""Positionsverfolgung und automatische Fahrten.

Die Bausteine:

``Odometry``        schreibt die Position aus der Fahrbewegung fort (driftet)
``Localizer``       zieht sie per Lidar-Abgleich gegen eine Karte gerade
``Rueckweg``        merkt sich die gefahrene Strecke
``MarkerAndocken``  richtet am Ziel per Kamera-Marker genau aus (optional)
``Homing``          fährt mit alledem zum Startpunkt zurück

Warum mehrere Ortungsverfahren: Die Koppelnavigation ist immer verfügbar,
aber ungenau. Der Lidar ist genau, braucht aber Wiedererkennung. Der Marker
ist am genauesten, aber nur in Sichtweite des Startpunkts. Zusammen
ergänzen sie sich -- Details in ``localizer.py`` und ``marker.py``.
"""

from .homing import Homing
from .localizer import Localizer
from .marker import MarkerAndocken
from .odometry import Odometry
from .rueckweg import Rueckweg

__all__ = ["Homing", "Localizer", "MarkerAndocken", "Odometry", "Rueckweg"]
