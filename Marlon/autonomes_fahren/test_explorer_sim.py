#!/usr/bin/env python3
"""
Offline test of the explorer logic (no ROS, no robot needed).

Simulates a room with obstacles, a 360° lidar (mounted 90° rotated like on
the X3) and holonomic mecanum kinematics. Reports collisions and coverage.
Also checks that a mirrored lidar is caught during the startup turn.

  python test_explorer_sim.py            # 10 simulated minutes, 3 seeds
  python test_explorer_sim.py --plot     # also save sim_path.png (matplotlib)
"""

import argparse
import math
import random
import sys

from x3_explorer import Explorer, make_parser, scan_to_points, wrap

ROBOT_R = 0.17          # physical contact radius of the X3 body
# real X3 lidar (/scan of sllidar_ros2): robot angle = lidar angle + 90 deg


def build_world():
    segs = []

    def box(x0, y0, x1, y1):
        segs.extend([((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
                     ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))])

    box(0, 0, 6, 4)                      # room walls
    box(1.5, 1.2, 2.1, 1.8)              # table
    box(3.8, 0.0, 4.0, 1.6)              # wall stub -> corridor
    box(4.6, 2.6, 6.0, 2.8)              # shelf
    segs.append(((0.0, 3.0), (0.8, 4.0)))  # angled corner -> dead end-ish
    circles = [(3.0, 2.8, 0.15), (1.0, 0.7, 0.1), (5.0, 1.2, 0.2)]  # chair legs, bin
    return segs, circles


def ray(ox, oy, dx, dy, segs, circles, rmax):
    best = rmax
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        if 0 < t < best and 0 <= u <= 1:
            best = t
    for cx, cy, r in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if 0 < t < best:
                best = t
    return best


def clearance(x, y, segs, circles):
    d = 1e9
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        t = max(0, min(1, ((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey)))
        d = min(d, math.hypot(x - x1 - t * ex, y - y1 - t * ey))
    for cx, cy, r in circles:
        d = min(d, math.hypot(x - cx, y - cy) - r)
    return d


def simulate(seed, minutes, p, mirrored=False):
    rnd = random.Random(seed)
    segs, circles = build_world()
    x, y, th = 0.8, 2.0, rnd.uniform(-math.pi, math.pi)
    ex = Explorer(p, random.Random(seed))
    dt, n_rays = 0.05, 400
    pts = []
    collisions, min_clear = 0, 9.0
    visited = set()
    states = {}
    path = []
    for k in range(int(minutes * 60 / dt)):
        t = k * dt
        if k % 2 == 0:  # lidar at 10 Hz, noise + dropouts
            ranges = []
            for i in range(n_rays):
                a_l = -math.pi + i * 2 * math.pi / n_rays
                if mirrored:    # Lidar falsch herum eingestellt
                    a_w = th - a_l - math.pi / 2
                else:
                    a_w = th + a_l + math.pi / 2
                r = ray(x, y, math.cos(a_w), math.sin(a_w), segs, circles, 12.0)
                r += rnd.gauss(0, 0.01)
                if rnd.random() < 0.03:
                    r = float("inf")
                ranges.append(r)
            pts = scan_to_points(-math.pi, 2 * math.pi / n_rays, ranges, 0.15, 12.0,
                                 math.radians(p.front_offset), p.ignore_below, p.flip)
        c = ex.update(pts, t)
        # actual motion with 10% wheel slip noise
        s = 1 + rnd.gauss(0, 0.05)
        x += (c.vx * math.cos(th) - c.vy * math.sin(th)) * dt * s
        y += (c.vx * math.sin(th) + c.vy * math.cos(th)) * dt * s
        th = wrap(th + c.wz * dt * s)
        cl = clearance(x, y, segs, circles) - ROBOT_R
        min_clear = min(min_clear, cl)
        if cl < 0:
            collisions += 1
        visited.add((int(x / 0.25), int(y / 0.25)))
        states[ex.state] = states.get(ex.state, 0) + dt
        path.append((x, y))
    return collisions, min_clear, len(visited), states, path, ex.fault


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--minutes", type=float, default=10)
    a.add_argument("--seeds", type=int, default=3)
    a.add_argument("--plot", action="store_true")
    args = a.parse_args()
    p = make_parser().parse_args([])
    total_cells = int(6 / 0.25) * int(4 / 0.25)
    ok = True
    paths = []
    for seed in range(args.seeds):
        col, mc, cells, states, path, fault = simulate(seed, args.minutes, p)
        paths.append(path)
        st = ", ".join(f"{k}={v:.0f}s" for k, v in states.items())
        print(f"seed {seed}: Kollisionen={col}  min. Abstand={mc:.3f} m  "
              f"Abdeckung={cells}/{total_cells} Zellen  [{st}]")
        if fault:
            print(f"  FEHLER: Spiegel-Pruefung schlaegt falsch an: {fault}")
        ok &= col == 0 and not fault
    for seed in range(args.seeds):
        fault = simulate(seed, 0.5, p, mirrored=True)[5]
        print(f"seed {seed}: gespiegelter Lidar "
              f"{'erkannt' if fault else 'NICHT erkannt'}")
        ok &= bool(fault)
    if args.plot:
        import matplotlib.pyplot as plt
        segs, circles = build_world()
        fig, ax = plt.subplots(figsize=(9, 6))
        for (x1, y1), (x2, y2) in segs:
            ax.plot([x1, x2], [y1, y2], color="#333", lw=2)
        for cx, cy, r in circles:
            ax.add_patch(plt.Circle((cx, cy), r, color="#333"))
        for i, path in enumerate(paths):
            ax.plot(*zip(*path), lw=0.8, alpha=0.8, label=f"seed {i}")
        ax.set_aspect("equal")
        ax.legend()
        fig.savefig("sim_path.png", dpi=120, bbox_inches="tight")
        print("-> sim_path.png gespeichert")
    print("OK" if ok else "FEHLER")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
