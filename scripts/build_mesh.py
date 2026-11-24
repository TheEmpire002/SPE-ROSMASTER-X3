"""Coloured 3D model (mesh) from the robot's depth images -- runs on a pc.

Offline, from a raw recording (/map: „Rohaufnahme fürs 3D-Modell“, then ZIP):

    pip install open3d opencv-python numpy
    python scripts/build_mesh.py rohaufnahme_20261124-153000.zip
    python scripts/build_mesh.py rohaufnahme_....zip --show        # look at it right away

Live, while the robot drives (also needs ``pip install websockets``):

    python scripts/build_mesh.py --live 10.42.0.62

The model grows in a window while the robot drives; close the window or press
Ctrl+C to stop. The received images are kept in ``live_<time>/`` (same format
as a raw recording, so it can be rebuilt offline with other settings) and the
mesh is written to ``live_<time>.ply``.

Writes ``.ply`` (vertices with colour and normals), opens in MeshLab,
Blender, Windows 3D Viewer or PowerPoint (3D models).

How: every depth image is fused into a truncated signed distance volume
(open3d's tensor VoxelBlockGrid, the method behind KinectFusion) at the pose the
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
import base64
import json
import math
import sys
import tempfile
import time
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


def color_on_depth(cv2, np, image, box, w: int, h: int):
    """Cut the colour image (bgr) to the box where the depth image lies, scale
    it to the depth size; parts outside the colour image stay grey. rgb out"""
    x0, y0, bw, bh = box
    H, W = image.shape[:2]
    canvas = np.full((bh, bw, 3), 128, dtype=np.uint8)
    ax, ay, ex, ey = max(0, x0), max(0, y0), min(W, x0 + bw), min(H, y0 + bh)
    if ex > ax and ey > ay:
        canvas[ay - y0:ey - y0, ax - x0:ex - x0] = image[ay:ey, ax:ex]
    small = cv2.resize(canvas, (w, h), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2RGB)


def map2d_from_payload(payload: dict) -> dict:
    """/api/mapping/map payload -> the karte2d dict the robot puts in meta.json"""
    info = payload["info"]
    pos = info["origin"]["position"]
    return {"aufloesung": info["resolution"], "ursprung": [pos["x"], pos["y"]],
            "breite": info["width"], "hoehe": info["height"], "frei_max": 19, "belegt_min": 65,
            "data_b64": payload["data_b64"]}


def fetch_map2d(host: str) -> dict:
    """The robot's current 2d map over http (for recordings made before the
    robot put it into meta.json)"""
    import urllib.request

    host = host if ":" in host else f"{host}:8000"
    with urllib.request.urlopen(f"http://{host}/api/mapping/map", timeout=10) as r:
        return map2d_from_payload(json.load(r))


def floor_mesh(o3d, np, cv2, map2d: dict, cell: float, seeds=None, colour=(0.55, 0.55, 0.56)):
    """Grey floor at height 0 under the free cells of the 2d map, merged to
    ``cell`` size; the depth camera often does not see shiny or dark floors.
    ``seeds`` (robot positions x, y): only the free areas the robot was in"""
    res = float(map2d["aufloesung"])
    grid = np.frombuffer(base64.b64decode(map2d["data_b64"]), dtype=np.int8).reshape(
        int(map2d["hoehe"]), int(map2d["breite"]))
    free = (grid >= 0) & (grid <= int(map2d["frei_max"]))
    # coarser cells: a cell is floor if most of the map cells in it are free
    k = max(1, int(round(cell / res)))
    h, w = (free.shape[0] // k) * k, (free.shape[1] // k) * k
    coarse = free[:h, :w].reshape(h // k, k, w // k, k).mean(axis=(1, 3)) >= 0.5
    size = res * k
    ox, oy = map2d["ursprung"][0], map2d["ursprung"][1]
    # lidar rays through doors and windows leave thin free streaks outside the room:
    # an opening removes them, then only regions the robot drove in stay
    mask = cv2.morphologyEx(coarse.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    if seeds is not None and len(seeds):
        _, labels = cv2.connectedComponents(mask, connectivity=4)
        seeds = np.asarray(seeds, dtype=float)[:, :2]
        sx = ((seeds[:, 0] - ox) / size).astype(int)
        sy = ((seeds[:, 1] - oy) / size).astype(int)
        inside = (sx >= 0) & (sy >= 0) & (sx < mask.shape[1]) & (sy < mask.shape[0])
        keep = np.unique(labels[sy[inside], sx[inside]])
        keep = keep[keep > 0]
        mask = np.isin(labels, keep).astype(np.uint8)
    iy, ix = np.nonzero(mask)
    mesh = o3d.geometry.TriangleMesh()
    if not len(ix):
        return mesh
    x0 = ox + ix * size
    y0 = oy + iy * size
    corners = np.stack([np.column_stack((x0 + dx, y0 + dy, np.zeros_like(x0)))
                        for dx, dy in ((0, 0), (size, 0), (size, size), (0, size))], axis=1).reshape(-1, 3)
    base = np.arange(len(ix)) * 4
    tris = np.concatenate([np.column_stack((base, base + 1, base + 2)),
                           np.column_stack((base, base + 2, base + 3))])
    mesh.vertices = o3d.utility.Vector3dVector(corners)
    mesh.triangles = o3d.utility.Vector3iVector(tris)
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.tile(colour, (len(corners), 1)))
    mesh.merge_close_vertices(1e-6)
    mesh.compute_vertex_normals()
    return mesh


def with_floor(o3d, np, cv2, mesh, meta: dict, args, seeds=None):
    """The fused mesh plus the floor from the 2d map, if wanted and known"""
    if args.no_floor:
        return mesh
    map2d = meta.get("karte2d")
    if map2d is None and args.karte_von:
        try:
            map2d = fetch_map2d(args.karte_von)
            print(f"2d map fetched from {args.karte_von}")
        except Exception as exc:
            print(f"2d map not fetched ({exc}) -- no floor", file=sys.stderr)
    if map2d is None:
        print("no 2d map in the recording -- no floor (--karte-von ROBOTER fetches it)")
        return mesh
    floor = floor_mesh(o3d, np, cv2, map2d, args.floor_cell, seeds)
    print(f"floor: {len(floor.triangles) // 2} cells of {args.floor_cell} m")
    return mesh + floor


class Fusion:
    """The tsdf volume plus what is needed to add one image to it.

    open3d's tensor VoxelBlockGrid: the older ScalableTSDFVolume stays empty
    with open3d 0.20 and numpy 2 (checked on windows), the grid works and
    grows by itself"""

    def __init__(self, o3d, np, cv2, meta: dict, args) -> None:
        import open3d.core as o3c

        self.o3d, self.np, self.cv2, self.o3c = o3d, np, cv2, o3c
        self.meta = meta
        self.args = args
        self.w, self.h = meta["tiefe_breite"], meta["tiefe_hoehe"]
        fx = (self.w / 2.0) / math.tan(meta["hfov"] / 2.0)
        fy = (self.h / 2.0) / math.tan(meta["vfov"] / 2.0)
        self.intrinsic = o3c.Tensor([[fx, 0.0, self.w / 2.0], [0.0, fy, self.h / 2.0], [0.0, 0.0, 1.0]],
                                    o3c.float64)
        self.max_range = args.max_range or meta.get("max_m", 3.0)
        self.min_mm = int(meta.get("min_m", 0.5) * 1000)
        # truncation in cells, as open3d wants it
        self.trunc_cells = max(1.0, args.trunc / args.voxel)
        # initial size only, the hash map grows when the room is larger
        self.grid = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
            attr_channels=((1), (1), (3)),
            voxel_size=args.voxel, block_resolution=16, block_count=10000,
            device=o3c.Device("CPU:0"))
        self.grey = np.full((self.h, self.w, 3), 128, dtype=np.uint8)
        self.count = 0
        self.skipped = 0
        self.poses: list = []   # where the robot was: the floor is kept around these

    def add(self, line: dict, depth, color_bgr) -> bool:
        """One image: depth (uint16 mm), colour (bgr or None), its frames.jsonl
        line; False if it had nothing in range (open3d aborts on those)"""
        np, o3d, o3c = self.np, self.o3d, self.o3c
        self.poses.append(line["pose"][:2])
        depth = depth.astype(np.uint16)
        depth[depth < self.min_mm] = 0  # the astra measures nothing useful that close
        if int(((depth > 0) & (depth < self.max_range * 1000)).sum()) < self.args.min_pixels:
            self.skipped += 1  # e.g. looking down a long hallway
            return False
        color = self.grey
        if color_bgr is not None and line.get("farbe"):
            color = color_on_depth(self.cv2, np, color_bgr, line["farbe"]["box"], self.w, self.h)
        extrinsic = o3c.Tensor(np.linalg.inv(camera_to_world(self.meta, line["pose"], line.get("boden"))),
                               o3c.float64)
        # depth uint16 with colour uint8 is one of the two combinations open3d takes
        depth_t = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(depth)))
        color_t = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(color)))
        try:
            blocks = self.grid.compute_unique_block_coordinates(
                depth_t, self.intrinsic, extrinsic, 1000.0, self.max_range, self.trunc_cells)
            self.grid.integrate(blocks, depth_t, color_t, self.intrinsic, self.intrinsic, extrinsic,
                                1000.0, self.max_range, self.trunc_cells)
        except RuntimeError:
            self.skipped += 1
            return False
        self.count += 1
        return True

    def mesh(self, clean: bool = True):
        np = self.np
        if not self.count:
            return self.o3d.geometry.TriangleMesh()
        mesh = self.grid.extract_triangle_mesh().to_legacy()
        if clean and self.args.min_cluster > 0 and len(mesh.triangles):
            clusters, sizes, _ = mesh.cluster_connected_triangles()
            clusters, sizes = np.asarray(clusters), np.asarray(sizes)
            mesh.remove_triangles_by_mask(sizes[clusters] < self.args.min_cluster)
            mesh.remove_unreferenced_vertices()
        # colours come as 0..1 with a few slightly above (interpolation): clip
        colors = np.asarray(mesh.vertex_colors)
        if len(colors):
            mesh.vertex_colors = self.o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
        mesh.compute_vertex_normals()
        return mesh


def offline(o3d, np, cv2, args) -> int:
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
    fusion = Fusion(o3d, np, cv2, meta, args)
    print(f"{len(frames)} images, {fusion.w}x{fusion.h}, map '{meta.get('karte')}'")
    for k, fr in enumerate(frames):
        stem = f"{fr['i']:05d}"
        depth = cv2.imread(str(folder / f"{stem}_tiefe.png"), cv2.IMREAD_UNCHANGED)
        if depth is None:
            continue
        color = None
        if fr.get("farbe") and (folder / f"{stem}_farbe.jpg").exists():
            color = cv2.imread(str(folder / f"{stem}_farbe.jpg"), cv2.IMREAD_COLOR)
        fusion.add(fr, depth, color)
        if (k + 1) % 50 == 0:
            print(f"  {k + 1}/{len(frames)}")
    print(f"fused {fusion.count}, skipped {fusion.skipped} (nothing within {fusion.max_range} m)")
    print("extracting the surface ...")
    mesh = with_floor(o3d, np, cv2, fusion.mesh(), meta, args, fusion.poses)
    out = Path(args.out) if args.out else source.with_suffix(".ply")
    o3d.io.write_triangle_mesh(str(out), mesh)
    print(f"{len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles -> {out}")
    if args.show:
        o3d.visualization.draw_geometries([mesh], window_name=out.name, mesh_show_back_face=True)
    if temp is not None:
        temp.cleanup()
    return 0


def robot_marker(o3d, pose):
    """Small coordinate cross where the robot is (red = forward)"""
    marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    marker.rotate(o3d.geometry.get_rotation_matrix_from_xyz((0.0, 0.0, pose[2])), center=(0, 0, 0))
    marker.translate((pose[0], pose[1], 0.0))
    return marker


def live(o3d, np, cv2, args) -> int:
    try:
        from websockets.exceptions import WebSocketException
        from websockets.sync.client import connect
    except ImportError:
        print("missing package: pip install websockets", file=sys.stderr)
        return 1
    host = args.live if ":" in args.live else f"{args.live}:8000"
    url = f"ws://{host}/api/mapping/raum3d/live"
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(args.out) if args.out else Path(f"live_{stamp}.ply")
    folder = out.parent / (out.stem if out.suffix.lower() == ".ply" else out.name)
    if out.suffix.lower() != ".ply":
        out = out.parent / f"{out.name}.ply"
    folder.mkdir(parents=True, exist_ok=True)
    print(f"connecting to {url} ... images go to {folder}/")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Roboter live -- {folder.name}", width=1280, height=800)
    vis.get_render_option().mesh_show_back_face = True
    shown: list = []
    fusion: Fusion | None = None
    floor = None
    last_mesh = 0.0
    last_pose = None
    first_view = True
    running = True
    received = 0

    def refresh() -> None:
        nonlocal shown, first_view, floor
        if fusion is None or not fusion.count:
            return
        geometries = [fusion.mesh(clean=False)]
        if not args.no_floor and fusion.meta.get("karte2d"):
            floor = floor_mesh(o3d, np, cv2, fusion.meta["karte2d"], args.floor_cell, fusion.poses)
        if floor is not None:
            geometries.append(floor)
        if last_pose is not None:
            geometries.append(robot_marker(o3d, last_pose))
        for g in shown:
            vis.remove_geometry(g, reset_bounding_box=False)
        for g in geometries:
            vis.add_geometry(g, reset_bounding_box=first_view)
        shown = geometries
        first_view = False

    try:
        with connect(url, max_size=None, open_timeout=10) as ws:
            while running:
                try:
                    message = ws.recv(timeout=0.05)
                except TimeoutError:
                    message = None
                if isinstance(message, str):
                    data = json.loads(message)
                    if data.get("type") == "meta":
                        if fusion is not None and (fusion.meta.get("karte"), fusion.meta.get("frame_id")) != (
                                data.get("karte"), data.get("frame_id")):
                            print("the robot switched maps -- stopping, positions would no longer fit")
                            break
                        meta = {k: v for k, v in data.items() if k != "type"}
                        if fusion is None:
                            fusion = Fusion(o3d, np, cv2, meta, args)
                            (folder / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
                            print(f"map '{meta.get('karte')}', depth {fusion.w}x{fusion.h} -- drive slowly")
                    elif data.get("type") == "status":
                        print(f"\r  robot: {data.get('text')}" + " " * 20, end="", flush=True)
                    elif data.get("type") == "fehler":
                        print(f"robot: {data.get('text')}", file=sys.stderr)
                        break
                elif isinstance(message, bytes) and fusion is not None:
                    n = int.from_bytes(message[:4], "little")
                    line = json.loads(message[4:4 + n])
                    start = 4 + n
                    png = message[start:start + line["tiefe_bytes"]]
                    jpeg = message[start + line["tiefe_bytes"]:start + line["tiefe_bytes"] + line["farbe_bytes"]]
                    depth = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
                    color = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if jpeg else None
                    # keep it like a raw recording: rebuild offline with other settings later
                    line = {k: v for k, v in line.items() if k not in ("tiefe_bytes", "farbe_bytes")}
                    line["i"] = received
                    stem = f"{received:05d}"
                    (folder / f"{stem}_tiefe.png").write_bytes(png)
                    if jpeg:
                        (folder / f"{stem}_farbe.jpg").write_bytes(jpeg)
                    else:
                        line["farbe"] = None
                    with open(folder / "frames.jsonl", "a", encoding="utf-8") as f:
                        f.write(json.dumps(line) + "\n")
                    received += 1
                    if depth is not None:
                        fusion.add(line, depth, color)
                        last_pose = line["pose"]
                    print(f"\r  {received} images" + " " * 30, end="", flush=True)
                # the surface every few seconds: extracting it costs more than adding an image
                if time.monotonic() - last_mesh > args.refresh:
                    last_mesh = time.monotonic()
                    refresh()
                if not vis.poll_events():
                    running = False  # window closed
                vis.update_renderer()
    except KeyboardInterrupt:
        pass
    except (OSError, WebSocketException) as exc:
        # robot stopped, wifi gone: keep what came, the mesh is still written
        print(f"\nconnection ended: {exc}", file=sys.stderr)
    print()
    vis.destroy_window()
    if fusion is None or not fusion.count:
        print("no images received -- is the robot localized in a map and the depth camera available?")
        return 1
    print("extracting the surface ...")
    mesh = with_floor(o3d, np, cv2, fusion.mesh(), fusion.meta, args, fusion.poses)
    o3d.io.write_triangle_mesh(str(out), mesh)
    print(f"{fusion.count} images, {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles -> {out}")
    if args.show:
        o3d.visualization.draw_geometries([mesh], window_name=out.name, mesh_show_back_face=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("recording", nargs="?", help="ZIP from /map or the unpacked folder")
    parser.add_argument("--live", metavar="HOST", help="build live from the robot (ip, optionally :port)")
    parser.add_argument("--out", help="output .ply (live: also the folder name for the images)")
    parser.add_argument("--voxel", type=float, default=0.02, help="volume cell (m), smaller = finer and slower")
    parser.add_argument("--trunc", type=float, default=0.08, help="truncation distance (m), about 3-4 cells")
    parser.add_argument("--every", type=int, default=1, help="offline: use every n-th image")
    parser.add_argument("--max-range", type=float, help="ignore depth beyond this (m), default from the robot")
    parser.add_argument("--min-cluster", type=int, default=500,
                        help="drop loose pieces with fewer triangles (noise); 0 = keep all")
    parser.add_argument("--min-pixels", type=int, default=500,
                        help="skip images with fewer measured pixels within range")
    parser.add_argument("--no-floor", action="store_true", help="no floor from the 2d map")
    parser.add_argument("--floor-cell", type=float, default=0.05, help="size of the floor tiles (m)")
    parser.add_argument("--karte-von", metavar="HOST",
                        help="fetch the 2d map from the robot (recordings without it in meta.json)")
    parser.add_argument("--refresh", type=float, default=3.0, help="live: rebuild the shown surface every n s")
    parser.add_argument("--show", action="store_true", help="open a viewer at the end")
    args = parser.parse_args()
    if not args.recording and not args.live:
        parser.error("give a recording (ZIP or folder) or --live HOST")

    try:
        import cv2
        import numpy as np
        import open3d as o3d
    except ImportError as exc:
        print(f"missing package ({exc.name}): pip install open3d opencv-python numpy", file=sys.stderr)
        return 1
    return live(o3d, np, cv2, args) if args.live else offline(o3d, np, cv2, args)


if __name__ == "__main__":
    sys.exit(main())
