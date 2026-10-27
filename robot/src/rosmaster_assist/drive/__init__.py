"""Antrieb: Schnittstelle, echter Treiber, Mock."""

from .base import DriveBase
from .mock_driver import MockDriver

__all__ = ["DriveBase", "MockDriver", "create_driver"]


def create_driver(use_mock: bool) -> DriveBase:
    """Liefert den passenden Treiber.

    Der echte Treiber wird erst hier importiert, damit `Rosmaster_Lib` auf
    Entwicklungsrechnern gar nicht installiert sein muss.
    """
    if use_mock:
        return MockDriver()

    from .rosmaster_driver import RosmasterDriver

    return RosmasterDriver()
