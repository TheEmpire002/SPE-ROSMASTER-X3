"""Compare pose models on the Hailo: speed, CPU cost and keypoint confidence.

    python scripts/bench_pose.py                          # snapshot from the running server
    python scripts/bench_pose.py --image person.jpg -n 100
    python scripts/bench_pose.py --hef /usr/share/hailo-models/yolov8m_pose_h10.hef
    python scripts/bench_pose.py --no-upload              # print only

Runs on the robot next to the running server: the Hailo is opened "SHARED"
(as in ``sensors/pose.py``), so the server's pose detection keeps working,
both just share the chip. Nothing drives, the server is not touched. The
camera belongs to the server, so the image comes from its snapshot URL
(or a file).

Per model it measures the time per frame split into JPEG decode, letterbox,
Hailo inference and decoding (all as in ``pose.py``), the CPU time of this
process per frame, and the detections: number of people and the mean
confidence of the shoulder/hip keypoints that the healthbot relies on.

The results go to the server (``POST /zustand/benchmark``), which shows them
on the page ``/zustand``; the server runs as root, so a shared file in the
home folder would not work.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import statistics
import sys
import time
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.sensors import pose as ps  # noqa: E402

_DEFAULT_HEFS = [
    "/usr/share/hailo-models/yolov8s_pose_h10.hef",
    "/usr/share/hailo-models/yolov8m_pose_h10.hef",
]
_TORSO = (ps.KP_SHOULDER_L, ps.KP_SHOULDER_R, ps.KP_HIP_L, ps.KP_HIP_R)


def _image(args) -> bytes:
    if args.image:
        return pathlib.Path(args.image).read_bytes()
    url = f"{args.server}/camera/{config.PERSON_CAMERA}/snapshot"
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.read()


def _upload(server: str, image: str, models: list[dict]) -> None:
    body = json.dumps({"bild": image, "modelle": models}).encode("utf-8")
    request = urllib.request.Request(f"{server}/zustand/benchmark", data=body,
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=5):
            print(f"results shown at {server}/zustand")
    except Exception as exc:  # the measurement itself is printed above, an upload error is not fatal
        print(f"upload to the server failed: {exc}")


def _bench(hef: str, jpeg: bytes, n: int, cv2, np) -> dict:
    config.POSE_HEF = hef
    hailo = ps._Hailo(np)
    try:
        size = hailo.size
        phases = {"decode_ms": [], "letterbox_ms": [], "hailo_ms": [], "post_ms": [], "total_ms": []}
        cpu = 0.0
        people, torso_conf = [], []
        for i in range(n + 5):
            c0, t0 = time.process_time(), time.perf_counter()
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            t1 = time.perf_counter()
            h, w = image.shape[:2]
            scale = min(size / w, size / h)
            nw, nh = int(round(w * scale)), int(round(h * scale))
            dx, dy = (size - nw) // 2, (size - nh) // 2
            model_input = np.full((size, size, 3), 114, dtype=np.uint8)
            small = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_CUBIC)
            model_input[dy:dy + nh, dx:dx + nw] = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            t2 = time.perf_counter()
            outputs = hailo.infer(model_input)
            t3 = time.perf_counter()
            hits = ps.decode_poses(np, outputs, size, config.POSE_MIN_SCORE)
            t4 = time.perf_counter()
            if i < 5:
                continue  # warm-up
            cpu += time.process_time() - c0
            for key, value in zip(phases, (t1 - t0, t2 - t1, t3 - t2, t4 - t3, t4 - t0)):
                phases[key].append(value * 1000)
            people.append(len(hits))
            for _, _, kp in hits[:1]:
                torso_conf.append(float(np.mean([kp[k][2] for k in _TORSO])))
        result = {k: round(statistics.median(v), 1) for k, v in phases.items()}
        result.update({
            "fps_max": round(1000 / statistics.median(phases["total_ms"]), 1),
            "cpu_ms_per_frame": round(cpu / n * 1000, 1),
            "people_median": statistics.median(people) if people else 0,
            "torso_conf_mean": round(statistics.mean(torso_conf), 3) if torso_conf else None,
        })
        return result
    finally:
        hailo.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="compare pose models on the hailo")
    parser.add_argument("--hef", action="append", help="model file (repeatable), default s and m")
    parser.add_argument("--image", help="JPEG file instead of the server snapshot")
    parser.add_argument("--server", default="http://127.0.0.1:8000")
    parser.add_argument("-n", type=int, default=50, help="frames per model")
    parser.add_argument("--no-upload", action="store_true", help="do not send the results to the server")
    args = parser.parse_args()

    import cv2
    import numpy as np

    jpeg = _image(args)
    models = []
    for hef in args.hef or _DEFAULT_HEFS:
        name = pathlib.Path(hef).name
        if not pathlib.Path(hef).exists():
            print(f"{hef}: missing")
            continue
        try:
            result = _bench(hef, jpeg, args.n, cv2, np)
        except Exception as exc:
            print(f"{hef}: {exc.__class__.__name__}: {exc}", flush=True)
            models.append({"modell": name, "fehler": f"{exc.__class__.__name__}: {exc}"[:200]})
            continue
        print(name, result, flush=True)
        models.append({"modell": name, "bilder": args.n, **result})
    if models and not args.no_upload:
        _upload(args.server, args.image or "Schnappschuss der Personenkamera", models)


if __name__ == "__main__":
    main()
