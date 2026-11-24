"""Coloured 3D model (mesh) from a raw recording of the robot -- runs on a pc.

On /map: „Rohaufnahme fürs 3D-Modell“, drive slowly through the room, stop,
download the ZIP. Then:

    pip install open3d opencv-python numpy
    python scripts/build_mesh.py rohaufnahme_20261124-153000.zip
    python scripts/build_mesh.py rohaufnahme_....zip --show        # look at it right away

Writes ``<recording>.ply`` (vertices with colour and normals), opens in
MeshLab, Blender, Windows 3D Viewer or PowerPoint (3D models).

How: every depth image is fused into a truncated signed distance volume
(open3d's ScalableTSDFVolume, the method behind KinectFusion) at the pose the
robot had in its map when the image was taken; the colour image is cut and
scaled onto the depth image with the same box as the robot's overlay view.
From the volume marching cubes extracts the surface. The pose is only 2D
(x, y, heading); height and tilt come from the camera mounting and the floor
plane the robot measured per image.

Not part of the robot software: needs open3d, which does not run on the pi
(and the fusion is too heavy for it anyway).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import zipfile
from pathlib import Path


def camera_to_world(meta: dict, pose, floor) -> "np.ndarray":
    """4x4 transform from the open3d camera frame (x right, y down, z forward)
    to the map frame (x, y on the floor, z up), as sensors/depth.robot_points
    plus the floor plane and the map pose"""
    import numpy as np

    p = math.radians(meta["kamera_neigung_grad"])
    # camera -> robot frame (x forward, y left, z up), tilt p down
    R = np.array([
        [0.0, -math.sin(p), math.cos(p)],
        [-1.0, 0.0, 0.0],
        [0.0, -math.cos(p), -math.sin(p)],
    ])
    robot = np.eye(4)
    robot[:3, :3] = R
    robot[:3, 3] = [meta["kamera_vorn_m"], 0.0, meta["kamera_hoehe_m"]]
    # height above the floor plane the robot saw (z' = z - (a x + b y + c)):
    # takes out the tipping while driving
    level = np.eye(4)
    if floor:
        a, b, c = floor
        level[2, :] = [-a, -b, 1.0, -c]
    x, y, th = pose
    world = np.array([
        [math.cos(th), -math.sin(th), 0.0, x],
        [math.sin(th), math.cos(th), 0.0, y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    return world @ level @ robot


def color_on_depth(cv2, np, path: Path, box, w: int, h: int):
    """Cut the colour image to the box where the depth image lies, scale it to
    the depth size; parts outside the colour image stay grey"""
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    x0, y0, bw, bh = box
    H, W = image.shape[:2]
    canvas = np.full((bh, bw, 3), 128, dtype=np.uint8)
    ax, ay, ex, ey = max(0, x0), max(0, y0), min(W, x0 + bw), min(H, y0 + bh)
    if ex > ax and ey > ay:
        canvas[ay - y0:ey - y0, ax - x0:ex - x0] = image[ay:ey, ax:ex]
    small = cv2.resize(canvas, (w, h), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2RGB)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("recording", help="ZIP from /map or the unpacked folder")
    parser.add_argument("--out", help="output .ply (default: beside the recording)")
    parser.add_argument("--voxel", type=float, default=0.02, help="volume cell (m), smaller = finer and slower")
    parser.add_argument("--trunc", type=float, default=0.08, help="truncation distance (m), about 3-4 cells")
    parser.add_argument("--every", type=int, default=1, help="use every n-th image")
    parser.add_argument("--max-range", type=float, help="ignore depth beyond this (m), default from the robot")
    parser.add_argument("--min-cluster", type=int, default=500,
                        help="drop loose pieces with fewer triangles (noise); 0 = keep all")
    parser.add_argument("--show", action="store_true", help="open a viewer at the end")
    args = parser.parse_args()

    try:
        import cv2
        import numpy as np
        import open3d as o3d
    except ImportError as exc:
        print(f"missing package ({exc.name}): pip install open3d opencv-python numpy", file=sys.stderr)
        return 1

    source = Path(args.recording)
    temp = None
    if source.suffix.lower() == ".zip":
        temp = tempfile.TemporaryDirectory()
        with zipfile.ZipFile(source) as z:
            z.extractall(temp.name)
        folders = [p.parent for p in Path(temp.name).rglob("meta.json")]
        if not folders:
            print("no meta.json in the zip", file=sys.stderr)
            return 1
        folder = folders[0]
    else:
        folder = source
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    frames = [json.loads(line) for line in (folder / "frames.jsonl").read_text(encoding="utf-8").splitlines()
              if line.strip()]
    frames = frames[::max(1, args.every)]
    w, h = meta["tiefe_breite"], meta["tiefe_hoehe"]
    fx = (w / 2.0) / math.tan(meta["hfov"] / 2.0)
    fy = (h / 2.0) / math.tan(meta["vfov"] / 2.0)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, w / 2.0, h / 2.0)
    max_range = args.max_range or meta.get("max_m", 3.0)
    min_mm = int(meta.get("min_m", 0.5) * 1000)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel, sdf_trunc=args.trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    grey = np.full((h, w, 3), 128, dtype=np.uint8)
    print(f"{len(frames)} images, {w}x{h}, map '{meta.get('karte')}'")
    for k, fr in enumerate(frames):
        stem = f"{fr['i']:05d}"
        depth = cv2.imread(str(folder / f"{stem}_tiefe.png"), cv2.IMREAD_UNCHANGED)
        if depth is None:
            continue
        depth = depth.astype(np.uint16)
        depth[depth < min_mm] = 0  # the astra measures nothing useful that close
        color = grey
        if fr.get("farbe") and (folder / f"{stem}_farbe.jpg").exists():
            color = color_on_depth(cv2, np, folder / f"{stem}_farbe.jpg", fr["farbe"]["box"], w, h)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(color)), o3d.geometry.Image(depth),
            depth_scale=1000.0, depth_trunc=max_range, convert_rgb_to_intensity=False)
        extrinsic = np.linalg.inv(camera_to_world(meta, fr["pose"], fr.get("boden")))
        volume.integrate(rgbd, intrinsic, extrinsic)
        if (k + 1) % 50 == 0:
            print(f"  {k + 1}/{len(frames)}")

    print("extracting the surface ...")
    mesh = volume.extract_triangle_mesh()
    if args.min_cluster > 0 and len(mesh.triangles):
        clusters, sizes, _ = mesh.cluster_connected_triangles()
        clusters, sizes = np.asarray(clusters), np.asarray(sizes)
        mesh.remove_triangles_by_mask(sizes[clusters] < args.min_cluster)
        mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    out = Path(args.out) if args.out else source.with_suffix(".ply")
    o3d.io.write_triangle_mesh(str(out), mesh)
    print(f"{len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles -> {out}")
    if args.show:
        o3d.visualization.draw_geometries([mesh], window_name=out.name, mesh_show_back_face=True)
    if temp is not None:
        temp.cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
