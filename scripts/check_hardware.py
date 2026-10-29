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
import time


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
    check_takt()


def check_takt() -> None:
    """CPU-Takt und Drosselung -- am X3 hängen die USB-Resets daran.

    Bei 2,4 GHz fielen Board, Lidar und Kamera bei Lastsprüngen ab, deshalb
    steht in /boot/firmware/config.txt `arm_freq=2000`. Fehlt die Zeile
    (z. B. nach einem neuen Image), soll man das hier sofort sehen.
    """
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq") as f:
            mhz = int(f.read()) // 1000
    except (OSError, ValueError):
        return  # kein Linux mit cpufreq, z. B. Windows -- nichts zu prüfen
    hinweis = "" if mhz <= 2000 else "  [WARNUNG] über 2000 MHz -> USB-Resets möglich, siehe docs/hardware.md"
    print(f"CPU max  {mhz} MHz{hinweis}")
    if shutil.which("vcgencmd") is None:
        return
    for args, name in ((["measure_temp"], "Temp    "), (["get_throttled"], "Drossel ")):
        try:
            out = subprocess.run(["vcgencmd", *args], capture_output=True, text=True, timeout=5)
            wert = out.stdout.strip().split("=", 1)[-1]
        except (subprocess.SubprocessError, OSError):
            continue
        # get_throttled: 0x0 = nie gedrosselt. Bit 0/16 Unterspannung,
        # Bit 3/19 Temperaturgrenze (aktuell/seit dem Start).
        print(f"{name} {wert}")


def _overcurrent_summe() -> int | None:
    """Summe der Over-Current-Zähler an den Root-Ports des Pi (ohne root lesbar).

    Beim Pi 5 hängen alle vier Buchsen an einem gemeinsamen Stromschalter,
    deshalb zählen bei einer Überlast alle Ports gleichzeitig hoch.
    """
    werte = []
    for pfad in glob.glob("/sys/bus/usb/devices/*-0:1.0/usb*-port*/over_current_count"):
        try:
            with open(pfad) as f:
                werte.append(int(f.read()))
        except (OSError, ValueError):
            pass
    return sum(werte) if werte else None


def check_usb_strom() -> None:
    """Zeigt, ob die USB-Versorgung gerade zusammenbricht und ob der Pi kompiliert.

    Am 28.10. lief zweimal ein DKMS-Build (nvidia) im Hintergrund: 100 % CPU,
    über 80 °C, danach riss die USB-Versorgung in einer Dauerschleife ab.
    Beides soll man hier vor einem Test sofort sehen.
    """
    vorher = _overcurrent_summe()
    if vorher is None:
        return  # kein Linux bzw. kein Pi -- nichts zu prüfen
    headline("USB-Versorgung")
    time.sleep(3)
    nachher = _overcurrent_summe() or 0
    neu = nachher - vorher
    hinweis = (
        f"  [WARNUNG] {neu} neue in 3 s -> Versorgung bricht gerade zusammen, siehe docs/hardware.md"
        if neu > 0 else "  [ok] keine neuen in 3 s"
    )
    print(f"  Over-Current seit dem Start: {nachher}{hinweis}")
    try:
        out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True, timeout=5)
    except (subprocess.SubprocessError, OSError):
        return
    bauen = [z for z in out.stdout.splitlines()
             if any(w in z for w in ("dkms", "dpkg ", "apt-get", "apt ", "cc1", "make -j"))]
    if bauen:
        print("  [WARNUNG] Der Pi installiert/kompiliert gerade -- nicht fahren, nicht testen:")
        for zeile in bauen[:5]:
            print(f"    {zeile[:100]}")


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


def check_board() -> None:
    """Der eigentliche Test: lässt sich das Expansion Board öffnen?

    Aussagekräftiger als jede Suche nach Gerätedateien -- Rosmaster_Lib
    findet die richtige Schnittstelle selbst, und je nach Image und
    Anschlussart heißt sie anders.
    """
    headline("Expansion Board")
    try:
        from Rosmaster_Lib import Rosmaster
    except ImportError:
        print("  [fehlt]  Rosmaster_Lib -- ohne die geht gar nichts")
        return

    try:
        Rosmaster()
    except Exception as exc:
        print(f"  [FEHLER] Board nicht erreichbar: {exc}")
        print("\n-> Roboter eingeschaltet? Kabel? Rechte (Gruppe 'dialout')?")
        return

    print("  [ok]     Board antwortet -- echte Hardware ist nutzbar")


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
    check_usb_strom()
    check_serial()
    check_python_packages()
    check_board()
    check_cameras()
    check_usb()
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
