"""Helpers for serial devices (board, lidar)."""

from __future__ import annotations

import glob
import os


def who_holds(path: str) -> list[str]:
    """Other processes that currently have the device open.

    For error messages: a dying reader thread, "Device or resource busy" or
    a lidar that only delivers garbage almost always mean that a second
    program is attached to the same port -- a forgotten second instance, a
    Jupyter notebook, a ROS node. Who that is should be in the log instead
    of having to hunt for it.

    Reads /proc directly so no lsof is needed. Processes of other users
    (root) stay invisible that way; our own are usually enough.
    """
    try:
        target = os.path.realpath(path)
    except OSError:
        return []
    own = str(os.getpid())
    hits = []
    for fd_folder in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_folder.split("/")[2]
        if pid == own:
            continue
        try:
            if not any(
                os.readlink(f"{fd_folder}/{fd}") == target for fd in os.listdir(fd_folder)
            ):
                continue
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                command = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue  # process has exited meanwhile or belongs to another user
        hits.append(f"PID {pid}: {command[:150]}")
    return hits
