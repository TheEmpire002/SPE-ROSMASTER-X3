"""Sicherheitsschicht: begrenzt Fahrbefehle und stoppt bei Funkabriss."""

from .watchdog import SafetyWatchdog

__all__ = ["SafetyWatchdog"]
