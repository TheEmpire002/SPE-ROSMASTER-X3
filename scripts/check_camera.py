#!/usr/bin/env python3
"""Finds out why no camera image arrives.

Run on the robot, with the Python of the virtual environment:

    robot/.venv/bin/python scripts/check_camera.py

It checks, in order, everything that can stand between camera and image:
existing devices, access rights, competing processes, OpenCV and actually
opening each candidate. At the end there is a verdict.

The server must be **stopped** for this -- otherwise it holds the camera
itself and the script reports it as busy.
"""

from __future__ import annotations

import glob
import os
import re
import shutil
import stat
import subprocess
import sys
import time
from pathlib import Path

# Do not maintain the filter list twice: it comes from the package itself.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot" / "src"))
try:
    from rosmaster_assist.sensors.camera import NOT_A_CAMERA
except ImportError:
    NOT_A_CAMERA = ("pispbe", "hevc", "codec", "isp", "cfe", "rpivid")

findings: list[str] = []


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def device_name(idx: int) -> str:
    try:
        with open(f"/sys/class/video4linux/video{idx}/name") as f:
            return f.read().strip()
    except OSError:
        return ""


def video_indices() -> list[int]:
    out = []
    for path in sorted(glob.glob("/dev/video*")):
        hits = re.search(r"(\d+)$", path)
        if hits:
            out.append(int(hits.group(1)))
    return sorted(out)


def is_camera(idx: int) -> bool:
    name = device_name(idx).lower()
    return not any(word in name for word in NOT_A_CAMERA)


# --- 1. Which devices exist ----------------------------------------------------


def check_devices() -> list[int]:
    headline("1. Devices under /dev/video*")
    all_ports = video_indices()
    if not all_ports:
        print("  none found")
        findings.append("There are no /dev/video* devices at all. "
                       "Camera not plugged in or not detected (check lsusb).")
        return []

    cameras = []
    for idx in all_ports:
        name = device_name(idx) or "?"
        if is_camera(idx):
            cameras.append(idx)
            print(f"  /dev/video{idx:<3} {name}   <- camera candidate")
        else:
            print(f"  /dev/video{idx:<3} {name}")

    if not cameras:
        findings.append("All devices found are codec/image-processing "
                       "units, no camera among them.")
    return cameras


# --- 2. Permissions -------------------------------------------------------------


def check_permissions(candidates: list[int]) -> None:
    headline("2. Access rights")
    try:
        import grp

        groups = [grp.getgrgid(g).gr_name for g in os.getgroups()]
    except Exception:
        groups = []

    print(f"  User:     {os.getlogin() if hasattr(os, 'getlogin') else '?'}")
    print(f"  Groups:   {', '.join(groups) or '?'}")

    if groups and "video" not in groups:
        print("  [!] not in group 'video'")
        findings.append("The user is not in group 'video'. "
                       "Fix: sudo usermod -aG video $USER, then log in again.")

    for idx in candidates:
        path = f"/dev/video{idx}"
        try:
            st = os.stat(path)
            perms = stat.filemode(st.st_mode)
            accessible = os.access(path, os.R_OK | os.W_OK)
            print(f"  {path}: {perms}  accessible={'yes' if accessible else 'NO'}")
            if not accessible:
                findings.append(f"No read/write access to {path}.")
        except OSError as exc:
            print(f"  {path}: {exc}")


# --- 3. Is someone holding the camera? ------------------------------------------


def check_in_use(candidates: list[int]) -> None:
    headline("3. Is it in use?")
    if shutil.which("fuser") is None:
        print("  fuser not available (sudo apt install psmisc)")
        return

    for idx in candidates:
        path = f"/dev/video{idx}"
        try:
            res = subprocess.run(
                ["fuser", "-v", path], capture_output=True, text=True, timeout=10
            )
            output = (res.stdout + res.stderr).strip()
        except (subprocess.SubprocessError, OSError) as exc:
            print(f"  {path}: {exc}")
            continue

        # fuser writes the hits to stderr and returns 1 when free.
        if res.returncode != 0 and not output:
            print(f"  {path}: free")
        else:
            print(f"  {path}:\n{output}")
            if output:
                findings.append(
                    f"{path} is held by another process (see above). "
                    "Only one program can open the camera at a time."
                )


# --- 4. OpenCV ------------------------------------------------------------------


def check_opencv() -> object | None:
    headline("4. OpenCV")
    try:
        import cv2
    except ImportError as exc:
        print(f"  not installed: {exc}")
        findings.append("OpenCV is missing. Fix: "
                       "robot/.venv/bin/pip install opencv-python")
        return None

    print(f"  Version: {cv2.__version__}")
    print(f"  File:    {cv2.__file__}")
    return cv2


# --- 5. Actually open ------------------------------------------------------------


VARIANTS = [
    ("MJPG", 640, 480),
    (None, 640, 480),
    ("YUYV", 640, 480),
    (None, 320, 240),
]


def warm_up(cap, seconds: float = 1.5):
    """Read repeatedly until a frame arrives. Cameras need time to start up."""
    deadline = time.monotonic() + seconds
    attempts = 0
    while time.monotonic() < deadline:
        attempts += 1
        ok, frame = cap.read()
        if ok and frame is not None and getattr(frame, "size", 0) > 0:
            return frame, attempts
        time.sleep(0.05)
    return None, attempts


def check_open(cv2, candidates: list[int]) -> bool:
    headline("5. Open and grab a frame")
    print("  (up to 1.5 s per variant -- cameras need start-up time)")
    success = False

    for idx in candidates:
        for api_name, api in (("V4L2", cv2.CAP_V4L2), ("Standard", cv2.CAP_ANY)):
            cap = cv2.VideoCapture(idx, api)
            if not cap.isOpened():
                print(f"  video{idx} / {api_name:<8}: does not open")
                cap.release()
                continue

            for fourcc, w, h in VARIANTS:
                if fourcc:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

                frame, attempts = warm_up(cap)
                label = f"{fourcc or 'Standard':<8} {w}x{h}"
                if frame is not None:
                    actual_h, actual_w = frame.shape[:2]
                    print(f"  video{idx} / {api_name:<8} / {label}: "
                          f"OK -- {actual_w}x{actual_h} after {attempts} attempt(s)")
                    success = True
                    break
                print(f"  video{idx} / {api_name:<8} / {label}: "
                      f"no frame ({attempts} attempts)")
            cap.release()
            if success:
                break
        if success:
            break

    if not success and candidates:
        findings.append("No combination of interface, format and resolution "
                       "delivers a frame. Section 6 shows what the camera "
                       "actually supports.")
    return success


# --- 6. Which formats the camera supports -------------------------------------------


def check_formats(candidates: list[int]) -> None:
    headline("6. Supported formats")
    if shutil.which("v4l2-ctl") is None:
        print("  v4l2-ctl not installed (sudo apt install v4l-utils)")
        return

    for idx in candidates[:2]:  # the first two are enough
        print(f"\n  /dev/video{idx}:")
        try:
            res = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}", "--list-formats"],
                capture_output=True, text=True, timeout=10,
            )
            for line in (res.stdout or res.stderr).splitlines():
                if line.strip():
                    print(f"    {line}")
        except (subprocess.SubprocessError, OSError) as exc:
            print(f"    {exc}")


# --- Main flow --------------------------------------------------------------------------


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass

    print("\n  Camera diagnostics")
    print("  " + "=" * 40)
    print("  The server must be stopped, otherwise it holds the camera itself.")

    candidates = check_devices()
    if candidates:
        check_permissions(candidates)
        check_in_use(candidates)

    cv2 = check_opencv()
    success = False
    if cv2 is not None and candidates:
        success = check_open(cv2, candidates)
    if candidates and not success:
        check_formats(candidates)

    headline("Verdict")
    if success:
        print("  The camera works. If the server still delivers no image, it was")
        print("  probably still running during this check and holding it itself.")
    elif findings:
        for i, b in enumerate(findings, 1):
            print(f"  {i}. {b}")
    else:
        print("  No clear cause found -- please report the whole output.")
    print()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
