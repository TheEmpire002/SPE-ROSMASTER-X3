"""Hilfen rund um serielle Geräte (Board, Lidar)."""

from __future__ import annotations

import glob
import os


def wer_haelt(pfad: str) -> list[str]:
    """Andere Prozesse, die das Gerät gerade offen haben.

    Für Fehlermeldungen: ein sterbender Lese-Thread, "Device or resource
    busy" oder ein Lidar, der nur Datenmüll liefert, heißen fast immer, dass
    ein zweites Programm am selben Port hängt -- eine vergessene zweite
    Instanz, ein Jupyter-Notebook, ein ROS-Knoten. Wer das ist, soll im Log
    stehen, statt dass man es suchen muss.

    Liest /proc direkt, damit kein lsof nötig ist. Prozesse anderer Benutzer
    (root) bleiben dabei unsichtbar, die eigenen genügen in aller Regel.
    """
    try:
        ziel = os.path.realpath(pfad)
    except OSError:
        return []
    eigene = str(os.getpid())
    treffer = []
    for fd_ordner in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_ordner.split("/")[2]
        if pid == eigene:
            continue
        try:
            if not any(
                os.readlink(f"{fd_ordner}/{fd}") == ziel for fd in os.listdir(fd_ordner)
            ):
                continue
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                befehl = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue  # Prozess inzwischen beendet oder fremder Benutzer
        treffer.append(f"PID {pid}: {befehl[:150]}")
    return treffer
