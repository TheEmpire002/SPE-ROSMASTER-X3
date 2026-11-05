"""Position tracking and automatic drives.

The building blocks:

``Odometry``            dead-reckons the position from the motion (drifts)
``Localizer``           pulls it straight again via lidar matching (does not drift)
``Homing``              uses it to drive back to the starting point
``ObstacleDetection``   names what the lidar sees (Wand, Stuhl, Tisch, Mensch)
``Explorer``            drives around autonomously and avoids everything
``Maneuver``            drives exactly "2 meters forward" or turns "90 degrees" (lidar measures)
``MappingDrive``        second mode of the explorer: covers the room like a robot vacuum
``ScanOdometry``        position including heading from the lidar (while mapping)

Why both localization methods: dead reckoning is always available but
inaccurate. The lidar is accurate but needs recognition. Together they
complement each other -- details in ``localizer.py``.
"""

from .explorer import Explorer
from .obstacles import ObstacleDetection
from .homing import Homing
from .localizer import Localizer
from .maneuver import Maneuver
from .odometry import Odometry

__all__ = ["Explorer", "ObstacleDetection", "Homing", "Localizer", "Maneuver", "Odometry"]
