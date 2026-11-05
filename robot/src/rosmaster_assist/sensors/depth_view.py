"""The depth image for viewing: colours by distance, obstacles marked.

For ``/tiefe/stream`` and the camera view of the app. This makes it possible
to check on the robot what the depth camera measures and what of it counts
as an obstacle -- without having to interpret numbers from ``/tiefe/status``.

* red = near, blue = far (``DEPTH_VIEW_NEAR_M`` to ``DEPTH_VIEW_FAR_M``)
* black = no measurement: closer than ~0.6 m, too far, reflective or black
* magenta = counts as an obstacle (4-40 cm above the floor, within range)

It is overlaid on the colour image of the same Orbbec. The two cameras have
different fields of view and sit slightly apart. The size comes from the
fields of view, the rest can be adjusted by hand
(``DEPTH_OVERLAY_SHIFT_PX``, ``DEPTH_OVERLAY_ZOOM``). So it only lines up
roughly -- good enough for viewing, nothing is computed from it.
"""

from __future__ import annotations

import math

from .. import config

_MAGENTA = (255, 0, 255)  # BGR


def _text(cv2, image, text: str, x: int, y: int, size: float = 0.5) -> None:
    """White with a black outline -- readable on any background."""
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, (255, 255, 255), 1, cv2.LINE_AA)


def _m(value: float) -> str:
    return f"{value:.2f} m".replace(".", ",")


def draw(image, hfov_depth: float, background: bytes | None = None,
             hfov_color_deg: float | None = None) -> bytes:
    """Evaluated depth image (``DepthFrame`` with ``raw``) -> JPEG.

    With ``background`` (JPEG of the colour image) semi-transparent on top
    of it, otherwise the depth image alone at double size.
    """
    import cv2
    import numpy as np

    raw = image.raw
    h, w = raw.shape
    meter = raw.astype(np.float32) / 1000.0
    valid = raw > 0

    close, far = config.DEPTH_VIEW_NEAR_M, config.DEPTH_VIEW_FAR_M
    share = np.clip((meter - close) / max(0.1, far - close), 0.0, 1.0)
    # JET: 0 = blue, 255 = red -- so invert it, making near red.
    colors = cv2.applyColorMap(((1.0 - share) * 255).astype(np.uint8), cv2.COLORMAP_JET)
    colors[~valid] = 0
    obstacle = np.zeros((h, w), dtype=bool)
    if image.mask is not None:
        # The mask has half resolution (every second point evaluated).
        full = np.repeat(np.repeat(image.mask, 2, axis=0), 2, axis=1)[:h, :w]
        obstacle[: full.shape[0], : full.shape[1]] = full
        colors[obstacle] = _MAGENTA

    color_image = None
    if background is not None:
        color_image = cv2.imdecode(np.frombuffer(background, dtype=np.uint8), cv2.IMREAD_COLOR)

    if color_image is None:
        output = cv2.resize(colors, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST)
        pose = (0, 0, w * 2, h * 2)  # where the depth image lies in the output
    else:
        output = color_image
        H, W = output.shape[:2]
        # Width of the depth image within the colour image, from the ratio of the fields of view.
        ratio = 1.0
        if hfov_depth and hfov_color_deg:
            ratio = math.tan(hfov_depth / 2.0) / math.tan(math.radians(hfov_color_deg) / 2.0)
        bw = max(1, int(W * ratio * config.DEPTH_OVERLAY_ZOOM))
        bh = max(1, int(bw * h / w))
        dx, dy = config.DEPTH_OVERLAY_SHIFT_PX
        x0, y0 = (W - bw) // 2 + int(dx), (H - bh) // 2 + int(dy)
        pose = (x0, y0, bw, bh)
        large = cv2.resize(colors, (bw, bh), interpolation=cv2.INTER_NEAREST)
        g_valid = cv2.resize(valid.astype(np.uint8), (bw, bh), interpolation=cv2.INTER_NEAREST) > 0
        g_obst = cv2.resize(obstacle.astype(np.uint8), (bw, bh), interpolation=cv2.INTER_NEAREST) > 0
        # Only the part that falls inside the colour image.
        ax, ay = max(0, x0), max(0, y0)
        ex, ey = min(W, x0 + bw), min(H, y0 + bh)
        if ex > ax and ey > ay:
            target = output[ay:ey, ax:ex].astype(np.float32)
            source = large[ay - y0:ey - y0, ax - x0:ex - x0].astype(np.float32)
            cover = np.where(g_obst[ay - y0:ey - y0, ax - x0:ex - x0], 0.8,
                            np.where(g_valid[ay - y0:ey - y0, ax - x0:ex - x0],
                                     config.DEPTH_OVERLAY_OPACITY, 0.0))[..., None]
            output[ay:ey, ax:ex] = (target * (1.0 - cover) + source * cover).astype(np.uint8)
        cv2.rectangle(output, (x0, y0), (x0 + bw - 1, y0 + bh - 1), (200, 200, 200), 1)

    # Distance at the image centre: median of a small window, so that a
    # single outlier does not make the number jump.
    window = meter[h // 2 - 4:h // 2 + 5, w // 2 - 4:w // 2 + 5]
    values = window[window > 0]
    lx, ly, lb, lh = pose
    mx, my = lx + lb // 2, ly + lh // 2
    cv2.drawMarker(output, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
    _text(cv2, output, _m(float(np.median(values))) if len(values) else "keine Messung",
          mx + 12, my - 8)

    lines = []
    if len(image.obstacles):
        distance = np.hypot(image.obstacles[:, 0], image.obstacles[:, 1])
        lines.append(f"naechstes Hindernis: {_m(float(distance.min()))}")
    else:
        lines.append("kein Hindernis")
    lines.append("Boden erkannt" if image.floor is not None else "Boden NICHT erkannt")
    for i, line in enumerate(lines):
        _text(cv2, output, line, 8, 20 + 20 * i)

    # Legend at the bottom: colour gradient near -> far, plus what black and magenta mean.
    H, W = output.shape[:2]
    width = min(200, W // 3)
    history = cv2.applyColorMap(np.linspace(255, 0, width).astype(np.uint8)[None, :], cv2.COLORMAP_JET)
    output[H - 22:H - 12, 8:8 + width] = np.repeat(history, 10, axis=0)
    _text(cv2, output, _m(close), 8, H - 28, 0.4)
    _text(cv2, output, _m(far), 8 + width - 45, H - 28, 0.4)
    _text(cv2, output, "schwarz: keine Messung (<0,6 m)  magenta: Hindernis",
          16 + width, H - 12, 0.4)

    ok, jpeg = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise RuntimeError("JPEG kodieren fehlgeschlagen")
    return jpeg.tobytes()
