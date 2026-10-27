#!/usr/bin/env python3
"""Findet heraus, warum kein Kamerabild kommt.

Auf dem Roboter ausführen, mit dem Python der virtuellen Umgebung:

    robot/.venv/bin/python scripts/check_camera.py

Geprüft wird der Reihe nach alles, was zwischen Kamera und Bild stehen kann:
vorhandene Geräte, Zugriffsrechte, konkurrierende Prozesse, OpenCV und das
tatsächliche Öffnen jedes Kandidaten. Am Ende steht ein Urteil.

Der Server muss dafür **beendet** sein -- sonst hält er die Kamera selbst
und das Skript meldet sie als belegt.
"""

from __future__ import annotations

import glob
import os
import re
import shutil
import stat
import subprocess
import sys
from pathlib import Path

# Die Filterliste nicht doppelt pflegen: sie kommt aus dem Paket selbst.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot" / "src"))
try:
    from rosmaster_assist.sensors.camera import KEINE_KAMERA
except ImportError:
    KEINE_KAMERA = ("pispbe", "hevc", "codec", "isp", "cfe", "rpivid")

befunde: list[str] = []


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def geraetename(idx: int) -> str:
    try:
        with open(f"/sys/class/video4linux/video{idx}/name") as f:
            return f.read().strip()
    except OSError:
        return ""


def video_indizes() -> list[int]:
    out = []
    for pfad in sorted(glob.glob("/dev/video*")):
        treffer = re.search(r"(\d+)$", pfad)
        if treffer:
            out.append(int(treffer.group(1)))
    return sorted(out)


def ist_kamera(idx: int) -> bool:
    name = geraetename(idx).lower()
    return not any(wort in name for wort in KEINE_KAMERA)


# --- 1. Welche Geräte gibt es --------------------------------------------------


def pruefe_geraete() -> list[int]:
    headline("1. Geräte unter /dev/video*")
    alle = video_indizes()
    if not alle:
        print("  keine gefunden")
        befunde.append("Es gibt überhaupt keine /dev/video*-Geräte. "
                       "Kamera nicht eingesteckt oder nicht erkannt (lsusb prüfen).")
        return []

    kameras = []
    for idx in alle:
        name = geraetename(idx) or "?"
        if ist_kamera(idx):
            kameras.append(idx)
            print(f"  /dev/video{idx:<3} {name}   <- Kamera-Kandidat")
        else:
            print(f"  /dev/video{idx:<3} {name}")

    if not kameras:
        befunde.append("Alle gefundenen Geräte sind Codec-/Bildverarbeitungs-"
                       "Einheiten, keine Kamera darunter.")
    return kameras


# --- 2. Rechte ------------------------------------------------------------------


def pruefe_rechte(kandidaten: list[int]) -> None:
    headline("2. Zugriffsrechte")
    try:
        import grp

        gruppen = [grp.getgrgid(g).gr_name for g in os.getgroups()]
    except Exception:
        gruppen = []

    print(f"  Benutzer: {os.getlogin() if hasattr(os, 'getlogin') else '?'}")
    print(f"  Gruppen:  {', '.join(gruppen) or '?'}")

    if gruppen and "video" not in gruppen:
        print("  [!] nicht in der Gruppe 'video'")
        befunde.append("Der Benutzer ist nicht in der Gruppe 'video'. "
                       "Beheben: sudo usermod -aG video $USER, danach neu anmelden.")

    for idx in kandidaten:
        pfad = f"/dev/video{idx}"
        try:
            st = os.stat(pfad)
            rechte = stat.filemode(st.st_mode)
            lesbar = os.access(pfad, os.R_OK | os.W_OK)
            print(f"  {pfad}: {rechte}  zugreifbar={'ja' if lesbar else 'NEIN'}")
            if not lesbar:
                befunde.append(f"Kein Lese-/Schreibzugriff auf {pfad}.")
        except OSError as exc:
            print(f"  {pfad}: {exc}")


# --- 3. Hält jemand die Kamera? -------------------------------------------------


def pruefe_belegung(kandidaten: list[int]) -> None:
    headline("3. Belegt das jemand?")
    if shutil.which("fuser") is None:
        print("  fuser nicht verfügbar (sudo apt install psmisc)")
        return

    for idx in kandidaten:
        pfad = f"/dev/video{idx}"
        try:
            res = subprocess.run(
                ["fuser", "-v", pfad], capture_output=True, text=True, timeout=10
            )
            ausgabe = (res.stdout + res.stderr).strip()
        except (subprocess.SubprocessError, OSError) as exc:
            print(f"  {pfad}: {exc}")
            continue

        # fuser schreibt die Treffer nach stderr und liefert 1, wenn frei.
        if res.returncode != 0 and not ausgabe:
            print(f"  {pfad}: frei")
        else:
            print(f"  {pfad}:\n{ausgabe}")
            if ausgabe:
                befunde.append(
                    f"{pfad} wird von einem anderen Prozess gehalten (siehe oben). "
                    "Nur ein Programm kann die Kamera gleichzeitig öffnen."
                )


# --- 4. OpenCV ------------------------------------------------------------------


def pruefe_opencv() -> object | None:
    headline("4. OpenCV")
    try:
        import cv2
    except ImportError as exc:
        print(f"  nicht installiert: {exc}")
        befunde.append("OpenCV fehlt. Beheben: "
                       "robot/.venv/bin/pip install opencv-python")
        return None

    print(f"  Version: {cv2.__version__}")
    print(f"  Datei:   {cv2.__file__}")
    return cv2


# --- 5. Tatsächlich öffnen --------------------------------------------------------


def pruefe_oeffnen(cv2, kandidaten: list[int]) -> bool:
    headline("5. Öffnen und ein Bild holen")
    erfolg = False

    for idx in kandidaten:
        for name, api in (("V4L2", cv2.CAP_V4L2), ("Standard", cv2.CAP_ANY)):
            cap = cv2.VideoCapture(idx, api)
            if not cap.isOpened():
                print(f"  video{idx} über {name:<8}: öffnet nicht")
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ok, frame = cap.read()
            if ok and frame is not None:
                h, w = frame.shape[:2]
                print(f"  video{idx} über {name:<8}: OK -- Bild {w}x{h}")
                erfolg = True
            else:
                print(f"  video{idx} über {name:<8}: geöffnet, aber kein Bild")
            cap.release()

    if not erfolg and kandidaten:
        befunde.append("Kein Kandidat liefert ein Bild. Wenn oben niemand die "
                       "Kamera hält und die Rechte stimmen, hilft ein Blick auf "
                       "die unterstützten Formate (Abschnitt 6).")
    return erfolg


# --- 6. Welche Formate kann die Kamera -----------------------------------------------


def pruefe_formate(kandidaten: list[int]) -> None:
    headline("6. Unterstützte Formate")
    if shutil.which("v4l2-ctl") is None:
        print("  v4l2-ctl nicht installiert (sudo apt install v4l-utils)")
        return

    for idx in kandidaten[:2]:  # die ersten beiden genügen
        print(f"\n  /dev/video{idx}:")
        try:
            res = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}", "--list-formats"],
                capture_output=True, text=True, timeout=10,
            )
            for zeile in (res.stdout or res.stderr).splitlines():
                if zeile.strip():
                    print(f"    {zeile}")
        except (subprocess.SubprocessError, OSError) as exc:
            print(f"    {exc}")


# --- Hauptablauf ------------------------------------------------------------------------


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass

    print("\n  Kamera-Diagnose")
    print("  " + "=" * 40)
    print("  Der Server muss beendet sein, sonst hält er die Kamera selbst.")

    kandidaten = pruefe_geraete()
    if kandidaten:
        pruefe_rechte(kandidaten)
        pruefe_belegung(kandidaten)

    cv2 = pruefe_opencv()
    erfolg = False
    if cv2 is not None and kandidaten:
        erfolg = pruefe_oeffnen(cv2, kandidaten)
    if kandidaten and not erfolg:
        pruefe_formate(kandidaten)

    headline("Urteil")
    if erfolg:
        print("  Die Kamera funktioniert. Liefert der Server trotzdem kein Bild,")
        print("  lief er bei dieser Prüfung vermutlich noch und hielt sie selbst.")
    elif befunde:
        for i, b in enumerate(befunde, 1):
            print(f"  {i}. {b}")
    else:
        print("  Kein eindeutiger Grund gefunden -- bitte die ganze Ausgabe melden.")
    print()
    return 0 if erfolg else 1


if __name__ == "__main__":
    sys.exit(main())
