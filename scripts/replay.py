#!/usr/bin/env python3
"""Replays a saved drive on your own computer -- without the robot.

A drive is created with "Fahrt speichern" in the web app (drive recorder,
see ``rosmaster_assist/recording.py``). It can be downloaded via
``http://<roboter>:8000/aufzeichnung/liste`` and
``/aufzeichnung/datei/<name>``.

    python scripts/replay.py fahrt_20261103_141500.jsonl.gz
    python scripts/replay.py fahrt.jsonl.gz --svg fahrt.svg
    python scripts/replay.py fahrt.jsonl.gz --erkunder

Without options you get a summary: duration, scan rate, commands per
source, how often the braking-distance guard intervened, hard stops and
state changes with timestamps.

``--svg`` draws the path (odometry pose) and the lidar points of some
scans in that frame -- good for seeing doors and narrow passages.
Caution: the pose comes from odometry and drifts; if the robot was lifted,
it does not jump along.

``--erkunder`` feeds the recorded scans and velocities to the current
explorer and compares its commands with the recorded ones. This lets you
test a changed avoidance logic on real drives. A full match is not to be
expected: on the robot the explorer chooses randomly, and odometry is only
recorded at 20 Hz.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
from collections import Counter

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "robot" / "src"))

from rosmaster_assist import recording as az  # noqa: E402

#: From this deviation on (m/s or rad/s) a command counts as changed.
#: Smaller than any step the explorer chooses deliberately.
TOLERANCE = 0.02
#: At most this many scans end up in the SVG -- more becomes unreadable.
SVG_SCANS = 12


def _changed(a, b) -> bool:
    return any(abs(x - y) > TOLERANCE for x, y in zip(a, b))


def summary(header: dict, events: list[dict]) -> None:
    print(f"Recorded: {header.get('erstellt', '?')}  Drive: {header.get('antrieb', '?')}"
          f"  Simulation: {header.get('mock', '?')}")
    if header.get("notiz"):
        print(f"Note: {header['notiz']}")
    if not events:
        print("No events.")
        return
    duration = events[-1]["t"] - events[0]["t"]
    categories = Counter(e["art"] for e in events)
    scans = categories.get("scan", 0)
    print(f"Duration: {duration:.1f} s, {len(events)} events")
    print(f"Lidar: {scans} revolutions ({scans / duration:.1f} Hz)" if duration > 0 else f"Lidar: {scans} revolutions")

    # Gaps in the lidar data indicate USB dropouts or a hanging service.
    times = [e["zeit"] for e in events if e["art"] == "scan"]
    gaps = [(a, b) for a, b in zip(times, times[1:]) if b - a > 0.5]
    for a, b in gaps:
        print(f"  Lidar gap: {a:.1f}–{b:.1f} s ({b - a:.1f} s)")

    commands = [e for e in events if e["art"] == "befehl"]
    sources = Counter(e["quelle"] for e in commands)
    print("Drive commands: " + (", ".join(f"{q} {n}" for q, n in sources.items()) or "none"))
    limited = [e for e in commands if _changed(e["soll"], e["safe"])]
    if limited:
        print(f"  of which limited (speed/braking distance): {len(limited)}")
        for e in limited[:5]:
            print(f"    {e['t']:6.2f} s {e['quelle']}: {e['soll']} -> {e['safe']}")
    stops = [e["t"] for e in events if e["art"] == "stop"]
    if stops:
        print(f"Hard stops: {len(stops)} (at " + ", ".join(f"{t:.1f}" for t in stops[:10]) + " s)")

    print("States:")
    for e in events:
        if e["art"] == "zustand":
            fields = {k: v for k, v in e.items() if k not in ("t", "art") and v not in (None, False)}
            print(f"  {e['t']:6.2f} s  " + (", ".join(f"{k}={v}" for k, v in fields.items()) or "all off"))


def svg(events: list[dict], path: pathlib.Path) -> None:
    poses = [e["pose"] for e in events if e["art"] == "odo"]
    if not poses:
        sys.exit("No odometry in the recording.")
    scans = [e for e in events if e["art"] == "scan"]
    step = max(1, len(scans) // SVG_SCANS)
    points = []
    playback = az.Playback(events)
    for e in scans[::step]:
        playback.until(e["zeit"])
        x, y, deg = playback.pose()
        th = math.radians(deg)
        for w, r in zip(e["w"], e["r"]):
            a = th + math.radians(w / 10.0)
            points.append((x + r / 1000.0 * math.cos(a), y + r / 1000.0 * math.sin(a)))
    all_ports = [(p[0], p[1]) for p in poses] + points
    xs, ys = [p[0] for p in all_ports], [p[1] for p in all_ports]
    margin = 0.3
    x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    m = 600 / max(x1 - x0, y1 - y0)  # pixels per meter

    # SVG counts y downwards, the robot to the left -- hence mirrored.
    def px(x, y):
        return f"{(x - x0) * m:.1f}", f"{(y1 - y) * m:.1f}"

    def circle(x, y, r, color):
        cx, cy = px(x, y)
        return f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{color}"/>'

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{(x1 - x0) * m:.0f}" '
             f'height="{(y1 - y0) * m:.0f}" style="background:#111">']
    parts += [circle(x, y, 1.2, "#4da3ff") for x, y in points]
    parts.append('<polyline fill="none" stroke="#ffcc33" stroke-width="2" points="'
                 + " ".join(",".join(px(p[0], p[1])) for p in poses) + '"/>')
    parts.append(circle(poses[0][0], poses[0][1], 5, "#3c3"))
    parts.append(circle(poses[-1][0], poses[-1][1], 5, "#e33"))
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")
    print(f"SVG written: {path} (green start, red end, blue lidar)")


def explorer_comparison(events: list[dict]) -> None:
    from rosmaster_assist.navigation import explorer as ex
    from rosmaster_assist.navigation import obstacles as hd

    class NoDriver:
        """The replayed explorer must not output anything."""

        def submit(self, command, source=""):
            return command

        def stop(self):
            pass

    def build(playback):
        return ex.Explorer(NoDriver(), playback, hd.ObstacleDetection(), None,
                           odometry=playback)

    ticks = az.replay_explorer(events, build)
    if not ticks:
        print("No explorer section in the recording.")
        return
    different = [t for t in ticks if t["nachgespielt"] is None
              or _changed(t["aufgezeichnet"], t["nachgespielt"])]
    same = len(ticks) - len(different)
    print(f"Explorer replayed: {len(ticks)} ticks, {same} same "
          f"({100 * same / len(ticks):.0f} %), {len(different)} different")
    for t in different[:10]:
        print(f"  {t['t']:6.2f} s  then {t['aufgezeichnet']}  now {t['nachgespielt']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a saved drive")
    parser.add_argument("datei", type=pathlib.Path)
    parser.add_argument("--svg", type=pathlib.Path, help="draw path and lidar as SVG")
    parser.add_argument("--erkunder", action="store_true",
                        help="feed the scans to the current explorer and compare")
    args = parser.parse_args()

    header, events = az.load(args.file)
    summary(header, events)
    if args.svg:
        svg(events, args.svg)
    if args.explorer:
        explorer_comparison(events)


if __name__ == "__main__":
    main()
