"""Drive via Yahboom's `Rosmaster_Lib` (real hardware).

This module is only imported on the robot. On development machines
`create_driver` takes the mock instead.

**Reconnecting.** If the supply at the USB hub dips briefly, the kernel
removes all devices and re-registers them two seconds later under new
numbers. Rosmaster_Lib notices none of this: its read thread dies with
"device reports readiness to read but returned no data", and every further
command goes to a file that no longer exists. ``monitor()`` detects this
and rebuilds the connection.
"""

from __future__ import annotations

import logging
import os
import threading
import time

from .base import DriveBase
from .. import config
from ..protocol import DriveCommand
from ..serial_ports import who_holds

log = logging.getLogger(__name__)


class RosmasterDriver(DriveBase):
    name = "rosmaster"

    def __init__(self, port: str | None = None) -> None:
        # The configured value stays separate from the port actually used:
        # after a USB dropout we have to search again, the numbers swap.
        self._configured_port = port if port is not None else config.SERIAL_PORT
        self._port: str | None = None
        self._bot = None
        self._last: DriveCommand | None = None
        self._lock = threading.Lock()
        # How a dead connection can be recognized -- see _alive().
        self._device: int | None = None
        self._receivers: list[threading.Thread] = []
        self._next_attempt = 0.0
        self._wait_time = config.RECONNECT_MIN_S

    def _determine_port(self) -> str | None:
        """Finds the expansion board among several serial devices.

        The ROSMASTER typically has two USB-serial adapters: the board
        (CH340) and the RPLIDAR (CP210x). Both appear as ``/dev/ttyUSB*``,
        and which one gets which number depends on the power-on order. If
        Rosmaster_Lib grabs the lidar, it reads its data stream as board
        replies -- which ends in
        "device reports readiness to read but returned no data".

        The lidar can be excluded reliably because a udev rule pins it to
        ``/dev/rplidar``. If exactly one device remains after that, it is
        the board.
        """
        import glob

        if self._configured_port:
            return self._configured_port

        # The same Yahboom udev rule pins the board (CH340, 1a86:7523) to
        # /dev/myserial. If that path exists, it is the safest choice -- it
        # depends on vendor and product ID, not on the order in which the
        # devices register.
        if os.path.exists("/dev/myserial"):
            return "/dev/myserial"

        devices = sorted(glob.glob("/dev/serial/by-id/*"))
        if not devices:
            return None  # nothing to decide, let the library search

        # Always exclude the lidar, even if it is the only device: on 28.10.,
        # after a USB dropout, it re-registered one second before the board,
        # /dev/rplidar did not exist yet, and the driver connected to the
        # lidar. At that moment only its name gives it away (CP210x from
        # Silicon Labs).
        lidar = os.path.realpath("/dev/rplidar") if os.path.exists("/dev/rplidar") else None
        candidates = [
            g for g in devices
            if os.path.realpath(g) != lidar and (lidar is not None or "CP210" not in g)
        ]

        if not candidates:
            # Only the lidar is there, the board is still registering.
            # None makes Rosmaster_Lib open /dev/myserial; that fails, and
            # monitor() tries again shortly.
            log.info("Only the lidar is registered -- waiting for the board")
            return None

        if len(candidates) == 1:
            log.info("Expansion board detected: %s", candidates[0])
            log.info("(lidar excluded via /dev/rplidar)")
            return candidates[0]

        log.warning(
            "%d serial devices, none can be assigned unambiguously -- "
            "Rosmaster_Lib picks one:",
            len(devices),
        )
        for g in devices:
            log.warning("    %s", g)
        log.warning("Enter the right one in config.py at SERIAL_PORT.")
        return None

    def connect(self) -> None:
        try:
            import Rosmaster_Lib  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "Rosmaster_Lib is not installed. On the development machine "
                "start with --mock instead. On the robot: see "
                "docs/hardware.md."
            ) from exc

        if not config.TELEMETRY_ENABLED:
            log.warning(
                "Telemetry disabled (TELEMETRY_ENABLED=False) -- "
                "no battery and IMU values"
            )
        self._open_device()

    def _open_device(self) -> None:
        """Establish the connection. Raises if the board is not reachable."""
        from Rosmaster_Lib import Rosmaster

        self._port = self._determine_port()
        # Rosmaster() without an argument searches for the port itself.
        bot = Rosmaster(com=self._port) if self._port else Rosmaster()

        try:
            # Stop first. After a dropout only the USB converter was gone, the
            # processor on the board kept running on the battery -- and may
            # still be executing the last drive command.
            bot.set_car_motion(0.0, 0.0, 0.0)

            # Starts the background thread that reads feedback (IMU, battery,
            # encoders) from the board. Without this call all read functions
            # of the library stay empty -- driving still works. The library
            # does not tell which thread it is; so check which one is new.
            receivers: list[threading.Thread] = []
            if config.TELEMETRY_ENABLED:
                previous = set(threading.enumerate())
                bot.create_receive_threading()
                receivers = [t for t in threading.enumerate() if t not in previous]
        except Exception:
            # Close the port again: otherwise every failed reconnect (USB
            # storm, 29.10.) leaves an open port behind, until the server runs
            # out of file handles or the port stays occupied.
            ser = getattr(bot, "ser", None)
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            raise

        with self._lock:
            self._bot = bot
            self._device = _device_of(bot)
            self._receivers = receivers
            self._last = None
        self._wait_time = config.RECONNECT_MIN_S

        log.info(
            "ROSMASTER connected (port: %s)",
            getattr(getattr(bot, "ser", None), "port", None) or self._port or "automatic",
        )
        # Check right when connecting: otherwise a second program on the port
        # only shows up when the read thread dies minutes later.
        self._report_others()

    def _report_others(self) -> bool:
        """Logs other processes on the board port. True if there are any."""
        path = self._port or "/dev/myserial"
        others = who_holds(path)
        if others:
            log.error("Another program has the board port %s open:", path)
            for line in others:
                log.error("    %s", line)
            log.error("This disturbs reception -- stop it with: kill <PID>")
        return bool(others)

    def _lost(self, reason: str) -> None:
        """Mark the connection as dead; ``monitor()`` rebuilds it."""
        with self._lock:
            bot, self._bot = self._bot, None
            self._receivers = []
        if bot is None:
            return  # already reported
        log.error("Connection to the board lost (%s) -- reconnecting", reason)
        self._report_others()
        # Close the old port, otherwise it stays occupied -- and if the board
        # comes back under the same number, it could not be opened.
        try:
            bot.ser.close()
        except Exception:
            log.debug("Closing the old port failed", exc_info=True)
        self._next_attempt = time.monotonic() + self._wait_time

    def _alive(self) -> bool:
        """Whether the existing connection still leads to the board.

        Two signs of a dead connection that need no write attempt:

        - The library's read thread has died. That is exactly the traceback
          you see in the terminal on a dropout.
        - The device path now points to a different device than the one
          opened. After re-registering, ``/dev/myserial`` has e.g. moved from
          ``ttyUSB1`` to ``ttyUSB0``.
        """
        with self._lock:
            bot, device, receivers = self._bot, self._device, self._receivers
        if bot is None:
            return False
        if receivers and not any(t.is_alive() for t in receivers):
            return False
        ser = getattr(bot, "ser", None)
        if ser is not None and not getattr(ser, "is_open", True):
            return False
        if device is not None:
            try:
                if os.stat(ser.port).st_rdev != device:
                    return False
            except OSError:
                return False  # path just disappeared
        return True

    def monitor(self) -> None:
        if self._bot is not None:
            if not self._alive():
                self._lost("USB device re-registered or read thread ended")
            return

        if time.monotonic() < self._next_attempt:
            return
        try:
            self._open_device()
            log.warning("Board reconnected")
        except Exception as exc:
            # Common: the device has not re-registered yet. Keep trying with
            # growing intervals instead of flooding the log.
            log.warning(
                "Board not reachable yet (%s) -- next attempt in %.0f s",
                exc,
                self._wait_time,
            )
            self._report_others()
            self._next_attempt = time.monotonic() + self._wait_time
            self._wait_time = min(self._wait_time * 2, config.RECONNECT_MAX_S)

    def disconnect(self) -> None:
        if self._bot is None:
            return
        self.stop()
        # Rosmaster_Lib has no close(); the serial port is released when the
        # process exits. All that matters is that the robot stands still.
        self._bot = None
        log.info("ROSMASTER disconnected")

    def drive(self, cmd: DriveCommand) -> None:
        bot = self._bot
        if bot is None:
            # While reconnecting: discard commands instead of raising. An
            # error here would tear down the connection to the app as well.
            return
        try:
            # Mecanum: set_car_motion(vx, vy, vz) expects m/s, m/s, rad/s.
            bot.set_car_motion(cmd.vx, cmd.vy, cmd.omega)
        except Exception as exc:
            self._lost(f"write failed: {exc}")
            return

        # Only log on change, otherwise 10 lines per second. Logging here at
        # all is intentional: otherwise a silent robot in real operation
        # looks exactly like one in the simulation, and you look for the
        # fault in the wrong place.
        if cmd != self._last:
            log.info("DRIVE  vx=%+.2f  vy=%+.2f  omega=%+.2f", cmd.vx, cmd.vy, cmd.omega)
            self._last = cmd

    def stop(self) -> None:
        # Must never raise -- stop() is the emergency exit and is also called
        # from error paths and during shutdown.
        bot = self._bot
        if bot is None:
            return
        try:
            bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception as exc:
            log.exception("Stopping failed -- check the hardware!")
            self._lost(f"stopping failed: {exc}")

    def beep(self, ms: int = 100) -> None:
        bot = self._bot
        if bot is None:
            return
        try:
            bot.set_beep(ms)
        except Exception:
            log.exception("Buzzer failed")

    def read_motion(self) -> tuple[float, float, float] | None:
        # Needs the receive thread; without it the library returns zeros
        # or -- after a dropout -- the last values before the disconnect.
        # Both would be worse than no measurement.
        bot = self._bot
        if bot is None or not config.TELEMETRY_ENABLED or not self._alive():
            return None
        try:
            vx, vy, omega = bot.get_motion_data()
            return float(vx), float(vy), float(omega)
        except Exception:
            return None

    def read_heading(self) -> float | None:
        # Caution: on the X3, get_imu_attitude_data() constantly returns 0
        # (sensor check 28.10.). The odometry therefore only uses the value
        # if config.ODOM_IMU_HEADING is set, and then checks it for
        # plausibility.
        bot = self._bot
        if bot is None or not config.TELEMETRY_ENABLED or not self._alive():
            return None
        try:
            _roll, _pitch, yaw = bot.get_imu_attitude_data()
            return float(yaw)
        except Exception:
            return None

    def read_battery(self) -> float | None:
        bot = self._bot
        if bot is None or not config.TELEMETRY_ENABLED or not self._alive():
            return None
        try:
            volt = float(bot.get_battery_voltage())
        except Exception:
            return None
        # Before the board's first data packet the library returns 0 --
        # that is not an empty battery but no measurement yet.
        return volt if volt > 1.0 else None

    @property
    def connected(self) -> bool:
        return self._bot is not None


def _device_of(bot) -> int | None:
    """ID of the device actually opened (major/minor number).

    Stays valid even if the device has long been removed -- exactly that
    makes it useful for comparing with what the path points to now.
    """
    try:
        return os.fstat(bot.ser.fileno()).st_rdev
    except Exception:
        return None
