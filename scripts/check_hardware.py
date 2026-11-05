#!/usr/bin/env python3
"""Shows which robot hardware is present on this machine.

Run on the Raspberry Pi when it is unclear which serial port is right
or whether the drivers are installed:

    python scripts/check_hardware.py
"""

from __future__ import annotations

import glob
import platform
import shutil
import subprocess
import sys
import time


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def check_system() -> None:
    headline("System")
    print(f"Python   {sys.version.split()[0]}")
    print(f"Platform {platform.platform()}")
    try:
        with open("/proc/device-tree/model") as f:
            print(f"Model    {f.read().strip(chr(0))}")
    except OSError:
        print("Model    (not a Raspberry Pi)")
    check_clock()


def check_clock() -> None:
    """CPU clock and throttling -- the USB resets on the X3 depend on them.

    At 2.4 GHz the board, lidar and camera dropped out on load spikes, which is
    why /boot/firmware/config.txt contains `arm_freq=2000`. If that line is
    missing (e.g. after a new image), you should see it here right away.
    """
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq") as f:
            mhz = int(f.read()) // 1000
    except (OSError, ValueError):
        return  # no Linux with cpufreq, e.g. Windows -- nothing to check
    hint = "" if mhz <= 2000 else "  [WARNING] above 2000 MHz -> USB resets possible, see docs/hardware.md"
    print(f"CPU max  {mhz} MHz{hint}")
    if shutil.which("vcgencmd") is None:
        return
    for args, name in ((["measure_temp"], "Temp    "), (["get_throttled"], "Throttle")):
        try:
            out = subprocess.run(["vcgencmd", *args], capture_output=True, text=True, timeout=5)
            value = out.stdout.strip().split("=", 1)[-1]
        except (subprocess.SubprocessError, OSError):
            continue
        # get_throttled: 0x0 = never throttled. Bit 0/16 undervoltage,
        # bit 3/19 temperature limit (currently/since boot).
        print(f"{name} {value}")


def _overcurrent_total() -> int | None:
    """Sum of the over-current counters on the Pi's root ports (readable without root).

    On the Pi 5 all four sockets share one power switch, so on an overload
    all ports count up at the same time.
    """
    values = []
    for path in glob.glob("/sys/bus/usb/devices/*-0:1.0/usb*-port*/over_current_count"):
        try:
            with open(path) as f:
                values.append(int(f.read()))
        except (OSError, ValueError):
            pass
    return sum(values) if values else None


def check_usb_power() -> None:
    """Shows whether the USB supply is collapsing right now and whether the Pi is compiling.

    On 28.10. a DKMS build (nvidia) ran twice in the background: 100 % CPU,
    over 80 °C, after which the USB supply kept dropping out in an endless loop.
    Both should be visible here right away before a test.
    """
    previous = _overcurrent_total()
    if previous is None:
        return  # no Linux or no Pi -- nothing to check
    headline("USB power")
    time.sleep(3)
    later = _overcurrent_total() or 0
    new = later - previous
    hint = (
        f"  [WARNING] {new} new in 3 s -> supply is collapsing right now, see docs/hardware.md"
        if new > 0 else "  [ok] none new in 3 s"
    )
    print(f"  Over-current since boot: {later}{hint}")
    try:
        out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True, timeout=5)
    except (subprocess.SubprocessError, OSError):
        return
    build = [z for z in out.stdout.splitlines()
             if any(w in z for w in ("dkms", "dpkg ", "apt-get", "apt ", "cc1", "make -j"))]
    if build:
        print("  [WARNING] The Pi is installing/compiling right now -- do not drive, do not test:")
        for line in build[:5]:
            print(f"    {line[:100]}")


def check_serial() -> None:
    headline("Serial ports")
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/myserial"]
    ports = sorted({p for pat in patterns for p in glob.glob(pat)})
    if ports:
        for p in ports:
            print(f"  {p}")
    else:
        print("  none found")
        print("\n-> Check the USB cable, is the robot switched on?")
        return

    # Expansion board and RPLIDAR both sit on /dev/ttyUSB*. Their numbering
    # can swap on reboot -- the by-id paths do not. So always use those.
    stable = sorted(glob.glob("/dev/serial/by-id/*"))
    if stable:
        print("\nStable paths (enter these in config.py):")
        for p in stable:
            print(f"  {p}")
        if len(ports) > 1:
            print("\n-> Several devices! To find out which is which,")
            print("   unplug one and run the script again.")
    else:
        print("\n-> /dev/serial/by-id/ is empty -- only available under Linux.")


def check_module(name: str, hint: str) -> None:
    try:
        __import__(name)
        print(f"  [ok]     {name}")
    except ImportError:
        print(f"  [missing] {name:<16} {hint}")


def check_python_packages() -> None:
    headline("Python packages")
    check_module("Rosmaster_Lib", "Yahboom driver -- see docs/hardware.md")
    check_module("fastapi", "pip install -r robot/requirements.txt")
    check_module("uvicorn", "pip install -r robot/requirements.txt")
    check_module("serial", "pip install pyserial")
    check_module("cv2", "optional, for the camera: pip install opencv-python")


def check_board() -> None:
    """The actual test: can the expansion board be opened?

    More meaningful than any search for device files -- Rosmaster_Lib finds
    the right port by itself, and its name differs depending on the image
    and the type of connection.
    """
    headline("Expansion Board")
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError:
        print("  [missing] Rosmaster_Lib -- nothing works without it")
        return

    try:
        Rosmaster()
    except Exception as exc:
        print(f"  [ERROR]  Board not reachable: {exc}")
        print("\n-> Robot switched on? Cable? Permissions (group 'dialout')?")
        return

    print("  [ok]     Board responds -- real hardware is usable")


def check_usb() -> None:
    headline("USB devices (lidar, camera)")
    if shutil.which("lsusb") is None:
        print("  lsusb not available (Linux only)")
        return
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
        print(out.stdout.rstrip() or "  none")
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"  Error: {exc}")


def check_cameras() -> None:
    headline("Cameras")
    devices = sorted(glob.glob("/dev/video*"))
    print("\n".join(f"  {d}" for d in devices) if devices else "  none found")


def main() -> int:
    check_system()
    check_usb_power()
    check_serial()
    check_python_packages()
    check_board()
    check_cameras()
    check_usb()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
