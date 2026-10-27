"""Sicherheitsschicht: begrenzt Fahrbefehle, stoppt bei Funkabriss,
hält auf Wunsch vor Hindernissen an."""

from .collision import CollisionGuard
from .watchdog import SafetyWatchdog

__all__ = ["CollisionGuard", "SafetyWatchdog"]
