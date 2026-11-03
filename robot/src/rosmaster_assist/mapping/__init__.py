"""Mapping subsystem for the ROSMASTER X3 FastAPI application.

The package deliberately keeps ROS 2 inside the existing Docker container.
The host-side FastAPI process communicates with it through ``docker exec``.
"""

from .manager import MappingManager
from .map_service import MapService
from .ros_bridge import ROSMapBridge

__all__ = ["MappingManager", "MapService", "ROSMapBridge"]
