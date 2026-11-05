"""The interface every drive must implement.

The rest of the code knows only this class, never a concrete driver.
This makes the hardware swappable -- for the mock during development,
or later for a ROS2 connection -- without touching the server or the
watchdog.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..protocol import DriveCommand


class DriveBase(ABC):
    """A drive that can execute drive commands."""

    #: Short name for log output and status messages to the app.
    name: str = "base"

    @abstractmethod
    def connect(self) -> None:
        """Connect to the hardware. Must run before `drive`."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close the connection cleanly. Stops the robot first."""

    @abstractmethod
    def drive(self, cmd: DriveCommand) -> None:
        """Execute a drive command.

        Called often (several times per second) and must return quickly
        -- so it must not block.
        """

    @abstractmethod
    def stop(self) -> None:
        """Stop immediately.

        Must work even if something went wrong before, and must not raise
        an exception.
        """

    def beep(self, ms: int = 100) -> None:
        """Beep. Optional -- the default does nothing."""

    def monitor(self) -> None:
        """Check the connection and re-establish it after a dropout.

        Called regularly from a background thread and may block briefly.
        Whether the connection is up is reported afterwards by
        ``connected``. Default: nothing to do.
        """

    def read_motion(self) -> tuple[float, float, float] | None:
        """Actually driven velocity (vx, vy, omega), if measurable.

        Much better for position tracking than the commanded value:
        slipping wheels, slopes and start-up delay show up here.
        ``None`` means "not measurable" -- then the command is used
        instead.
        """
        return None

    def read_heading(self) -> float | None:
        """Heading in degrees from the IMU, if available.

        A measured angle is far superior to an integrated turn rate: when
        integrating, the error grows with every turn; here it does not.
        """
        return None

    def read_battery(self) -> float | None:
        """Battery voltage in volts, if measurable.

        For the diagnostics page: a weak battery is the most common cause
        of the board dropping out on the X3. ``None`` means "not measurable".
        """
        return None

    @property
    @abstractmethod
    def connected(self) -> bool:
        """Whether the connection to the hardware is up."""

    # Enables `with create_driver(...) as drv:` -- the robot stops even
    # when the program ends with an exception.
    def __enter__(self) -> "DriveBase":
        self.connect()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.disconnect()
