#!/usr/bin/env python3
"""Zeigt, was auf diesem Rechner an Roboter-Hardware vorhanden ist.

Auf dem Raspberry Pi ausführen, wenn unklar ist, welcher serielle Port stimmt
oder ob die Treiber installiert sind:

    python scripts/check_hardware.py
"""

from __future__ import annotations

import glob
import platform
import shutil
import subprocess
import sys


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def check_system() -> None:
    headline("System")
    print(f"Python   {sys.version.split()[0]}")
    print(f"Platform {platform.platform()}")
    try:
        with open("/proc/device-tree/model") as f:
            print(f"Modell   {f.read().strip(chr(0))}")
    except OSError:
        print("Modell   (kein Raspberry Pi)")


def check_serial() -> None:
    headline("Serielle Schnittstellen")
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/myserial"]
    ports = sorted({p for pat in patterns for p in glob.glob(pat)})
    if ports:
        for p in ports:
            print(f"  {p}")
    else:
        print("  keine gefunden")
        print("\n-> USB-Kabel prüfen, Roboter eingeschaltet?")
        return

    # Expansion Board und RPLIDAR hängen beide an /dev/ttyUSB*. Deren
    # Nummerierung kann sich beim Neustart vertauschen -- die by-id-Pfade
    # nicht. Deshalb immer die hier verwenden.
    stable = sorted(glob.glob("/dev/serial/by-id/*"))
    if stable:
        print("\nStabile Pfade (diese in config.py eintragen):")
        for p in stable:
            print(f"  {p}")
        if len(ports) > 1:
            print("\n-> Mehrere Geräte! Welches welches ist, findest du heraus,")
            print("   indem du eines absteckst und das Skript erneut ausführst.")
    else:
        print("\n-> /dev/serial/by-id/ ist leer -- nur unter Linux verfügbar.")


def check_module(name: str, hint: str) -> None:
    try:
        __import__(name)
        print(f"  [ok]     {name}")
    except ImportError:
        print(f"  [fehlt]  {name:<16} {hint}")


def check_python_packages() -> None:
    headline("Python-Pakete")
    check_module("Rosmaster_Lib", "Yahboom-Treiber -- siehe docs/hardware.md")
    check_module("fastapi", "pip install -r robot/requirements.txt")
    check_module("uvicorn", "pip install -r robot/requirements.txt")
    check_module("serial", "pip install pyserial")
    check_module("cv2", "optional, für die Kamera: pip install opencv-python")


def check_usb() -> None:
    headline("USB-Geräte (Lidar, Kamera)")
    if shutil.which("lsusb") is None:
        print("  lsusb nicht verfügbar (nur unter Linux)")
        return
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
        print(out.stdout.rstrip() or "  keine")
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"  Fehler: {exc}")


def check_cameras() -> None:
    headline("Kameras")
    devices = sorted(glob.glob("/dev/video*"))
    print("\n".join(f"  {d}" for d in devices) if devices else "  keine gefunden")


def main() -> int:
    check_system()
    check_serial()
    check_python_packages()
    check_cameras()
    check_usb()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
