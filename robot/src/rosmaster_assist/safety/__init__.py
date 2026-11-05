"""Safety layer: limits drive commands, stops when the radio link drops,
optionally stops in front of obstacles, brakes autonomous driving (drive
guard over the robot's footprint).
"""

from .collision import CollisionGuard
from .drive_guard import DriveGuard
from .watchdog import SafetyWatchdog

__all__ = ["CollisionGuard", "DriveGuard", "SafetyWatchdog"]
