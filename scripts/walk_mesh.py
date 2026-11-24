"""Walk through a 3D model from build_mesh.py in first person (on the pc).

    python scripts/walk_mesh.py live_20261124-125839.ply
    python scripts/walk_mesh.py modell.ply --from live_20261124-125839 --eye 1.0

Keys (hold for continuous movement):
    W / S           forward / back
    A / D           step left / right
    Left / Right    turn
    Up / Down       look up / down
    R / F           eye higher / lower
    + / -           faster / slower
    C               walls solid on/off
    H               print this help
    Q / Esc         quit

Starts where the robot took its first image (frames.jsonl of the recording
next to the .ply, or ``--from``), else in the middle of the model. Walls are
solid: a ray in the walking direction stops you ``--margin`` before anything
the model has there. Needs ``pip install open3d numpy``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

HELP = __doc__.split("Keys", 1)[1].split("Starts", 1)[0]


def start_pose(args, mesh, np):
    """(x, y, heading rad) of the first robot pose, or the model centre"""
    folder = Path(args.source) if args.source else Path(args.mesh).with_suffix("")
    frames = folder / "frames.jsonl"
    if frames.exists():
        for line in frames.read_text(encoding="utf-8").splitlines():
            if line.strip():
                pose = json.loads(line)["pose"]
                return float(pose[0]), float(pose[1]), float(pose[2])
    centre = np.asarray(mesh.get_axis_aligned_bounding_box().get_center())
    return float(centre[0]), float(centre[1]), 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mesh", help=".ply from build_mesh.py")
    parser.add_argument("--from", dest="source", help="recording folder for the start position")
    parser.add_argument("--eye", type=float, default=1.0, help="eye height above the floor (m)")
    parser.add_argument("--step", type=float, default=0.08, help="metres per key repeat")
    parser.add_argument("--turn", type=float, default=4.0, help="degrees per key repeat")
    parser.add_argument("--margin", type=float, default=0.25, help="distance kept to walls (m)")
    parser.add_argument("--fov", type=float, default=70.0, help="field of view (deg)")
    args = parser.parse_args()
    try:
        import numpy as np
        import open3d as o3d
    except ImportError:
        print("needs: pip install open3d numpy", file=sys.stderr)
        return 1

    mesh = o3d.io.read_triangle_mesh(args.mesh)
    if not len(mesh.triangles):
        print(f"no triangles in {args.mesh}", file=sys.stderr)
        return 1
    mesh.compute_vertex_normals()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

    x, y, heading = start_pose(args, mesh, np)
    state = {"x": x, "y": y, "z": args.eye, "yaw": heading, "pitch": 0.0, "step": args.step, "solid": True}

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"{Path(args.mesh).name} -- W A S D, arrows, H help", width=1500, height=900)
    vis.add_geometry(mesh)
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True
    opt.background_color = np.array([0.55, 0.65, 0.78])
    ctr = vis.get_view_control()
    ctr.change_field_of_view(step=args.fov - ctr.get_field_of_view())
    ctr.set_constant_z_near(0.03)
    ctr.set_constant_z_far(100.0)

    def apply() -> None:
        cy, sy = math.cos(state["yaw"]), math.sin(state["yaw"])
        cp, sp = math.cos(state["pitch"]), math.sin(state["pitch"])
        forward = np.array([cp * cy, cp * sy, sp])
        right = np.array([sy, -cy, 0.0])
        down = np.cross(forward, right)
        # open3d camera: x right, y down, z forward; extrinsic = world -> camera
        rot = np.stack([right, down, forward])
        eye = np.array([state["x"], state["y"], state["z"]])
        ext = np.eye(4)
        ext[:3, :3] = rot
        ext[:3, 3] = -rot @ eye
        params = ctr.convert_to_pinhole_camera_parameters()
        params.extrinsic = ext
        ctr.convert_from_pinhole_camera_parameters(params, allow_arbitrary=True)

    def blocked(dx: float, dy: float) -> bool:
        """something of the model within the step plus margin, at knee and eye height"""
        if not state["solid"]:
            return False
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return False
        ux, uy = dx / length, dy / length
        heights = (0.35, max(0.5, state["z"] - 0.1))
        rays = o3d.core.Tensor([[state["x"], state["y"], h, ux, uy, 0.0] for h in heights],
                               dtype=o3d.core.Dtype.Float32)
        hit = scene.cast_rays(rays)["t_hit"].numpy()
        return bool((hit < length + args.margin).any())

    def move(forward: float, side: float):
        def callback(_vis):
            cy, sy = math.cos(state["yaw"]), math.sin(state["yaw"])
            dx = state["step"] * (forward * cy + side * sy)
            dy = state["step"] * (forward * sy - side * cy)
            if not blocked(dx, dy):
                state["x"] += dx
                state["y"] += dy
            apply()
            return True
        return callback

    def turn(d_yaw: float, d_pitch: float):
        def callback(_vis):
            state["yaw"] += math.radians(d_yaw)
            state["pitch"] = max(-1.4, min(1.4, state["pitch"] + math.radians(d_pitch)))
            apply()
            return True
        return callback

    def lift(dz: float):
        def callback(_vis):
            state["z"] = max(0.1, min(5.0, state["z"] + dz))
            apply()
            return True
        return callback

    def speed(factor: float):
        def callback(_vis):
            state["step"] = max(0.01, min(1.0, state["step"] * factor))
            print(f"step {state['step']:.2f} m")
            return False
        return callback

    def toggle_solid(_vis):
        state["solid"] = not state["solid"]
        print("walls solid" if state["solid"] else "walking through walls")
        return False

    def show_help(_vis):
        print(HELP)
        return False

    keys = {
        ord("W"): move(1, 0), ord("S"): move(-1, 0), ord("A"): move(0, -1), ord("D"): move(0, 1),
        263: turn(args.turn, 0), 262: turn(-args.turn, 0),          # left, right arrow
        265: turn(0, args.turn * 0.6), 264: turn(0, -args.turn * 0.6),  # up, down arrow
        ord("R"): lift(0.05), ord("F"): lift(-0.05),
        ord("="): speed(1.25), 334: speed(1.25), ord("-"): speed(0.8), 333: speed(0.8),
        ord("C"): toggle_solid, ord("H"): show_help,
    }
    for key, callback in keys.items():
        vis.register_key_callback(key, callback)
    print(HELP)
    apply()
    vis.run()
    vis.destroy_window()
    return 0


if __name__ == "__main__":
    sys.exit(main())
