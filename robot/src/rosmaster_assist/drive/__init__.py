"""Drive: interface, real driver, mock."""

from .base import DriveBase
from .mock_driver import MockDriver

__all__ = ["DriveBase", "MockDriver", "create_driver"]


def create_driver(use_mock: bool) -> DriveBase:
    """Returns the matching driver.

    The real driver is only imported here so that `Rosmaster_Lib` does not
    have to be installed on development machines at all.
    """
    if use_mock:
        return MockDriver()

    from .rosmaster_driver import RosmasterDriver

    return RosmasterDriver()
