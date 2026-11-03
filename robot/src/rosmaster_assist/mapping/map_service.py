"""Save and enumerate ROS OccupancyGrid maps as standard PGM + YAML files."""

from __future__ import annotations

import base64
import math
import os
import re
import time
from pathlib import Path
from typing import Any


class MapService:
    """Convert a ROS OccupancyGrid payload into map-server compatible files.

    Maps are saved as ``<name>.pgm`` plus ``<name>.yaml``. The default folder
    is ``rosmaster_assist/data/maps``; override it with ``ROSMAP_STORAGE_DIR``.
    """

    def __init__(self, storage_dir: str | Path | None = None) -> None:
        if storage_dir is None:
            storage_dir = os.getenv("ROSMAP_STORAGE_DIR") or (
                Path(__file__).resolve().parents[1] / "data" / "maps"
            )
        self.storage_dir = Path(storage_dir).expanduser().resolve()
        self.storage_dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _safe_name(name: str | None) -> str:
        raw = (name or "karte").strip()
        raw = re.sub(r"[^A-Za-z0-9ÄÖÜäöüß_-]+", "_", raw).strip("._-")
        if not raw:
            raw = "karte"
        return raw[:64]

    @staticmethod
    def _yaw_from_quaternion(q: dict[str, Any]) -> float:
        x = float(q.get("x", 0.0))
        y = float(q.get("y", 0.0))
        z = float(q.get("z", 0.0))
        w = float(q.get("w", 1.0))
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _occupancy_to_pgm(data: bytes, width: int, height: int) -> bytes:
        """Convert signed int8 occupancy values to top-down PGM grayscale."""
        pixels = bytearray(width * height)
        for y in range(height):
            source_row = (height - 1 - y) * width  # PGM rows start at the top.
            target_row = y * width
            for x in range(width):
                value = data[source_row + x]
                if value >= 128:
                    occupancy = value - 256  # signed int8: 255 means unknown (-1)
                else:
                    occupancy = value

                if occupancy < 0:
                    gray = 205  # unknown
                elif occupancy >= 65:
                    gray = 0  # occupied
                elif occupancy <= 19:
                    gray = 254  # free
                else:
                    gray = 205  # uncertain / intermediate
                pixels[target_row + x] = gray
        return b"P5\n" + f"{width} {height}\n255\n".encode("ascii") + bytes(pixels)

    def save_map(self, map_payload: dict[str, Any], name: str | None = None) -> dict[str, Any]:
        """Save a bridge payload; returns file names and basic map metadata."""
        if not isinstance(map_payload, dict):
            raise ValueError("Keine gültigen Kartendaten übergeben.")

        info = map_payload.get("info") or {}
        width = int(info.get("width", 0))
        height = int(info.get("height", 0))
        resolution = float(info.get("resolution", 0.0))
        if width <= 0 or height <= 0 or width * height > 100_000_000:
            raise ValueError("Ungültige Kartengröße.")
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError("Ungültige Kartenauflösung.")

        encoded = map_payload.get("data_b64")
        if not isinstance(encoded, str):
            raise ValueError("Kartendaten (data_b64) fehlen.")
        try:
            occupancy = base64.b64decode(encoded, validate=True)
        except (ValueError, base64.binascii.Error) as exc:
            raise ValueError("Kartendaten sind nicht gültig Base64-kodiert.") from exc
        if len(occupancy) != width * height:
            raise ValueError(
                f"Kartendaten unvollständig: erwartet {width * height} Zellen, "
                f"erhalten {len(occupancy)}."
            )

        safe_name = self._safe_name(name)
        pgm_path = self.storage_dir / f"{safe_name}.pgm"
        yaml_path = self.storage_dir / f"{safe_name}.yaml"

        origin = info.get("origin") or {}
        position = origin.get("position") or {}
        orientation = origin.get("orientation") or {}
        origin_x = float(position.get("x", 0.0))
        origin_y = float(position.get("y", 0.0))
        yaw = self._yaw_from_quaternion(orientation)
        if not all(math.isfinite(v) for v in (origin_x, origin_y, yaw)):
            raise ValueError("Ungültiger Kartenursprung.")

        pgm_bytes = self._occupancy_to_pgm(occupancy, width, height)
        yaml_text = (
            f"image: {pgm_path.name}\n"
            f"mode: trinary\n"
            f"resolution: {resolution:.8f}\n"
            f"origin: [{origin_x:.8f}, {origin_y:.8f}, {yaw:.8f}]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

        # Write temporary files and replace the destination to avoid exposing
        # partially written maps to a concurrent reader.
        temp_pgm = pgm_path.with_suffix(".pgm.tmp")
        temp_yaml = yaml_path.with_suffix(".yaml.tmp")
        try:
            temp_pgm.write_bytes(pgm_bytes)
            temp_yaml.write_text(yaml_text, encoding="utf-8")
            temp_pgm.replace(pgm_path)
            temp_yaml.replace(yaml_path)
        finally:
            temp_pgm.unlink(missing_ok=True)
            temp_yaml.unlink(missing_ok=True)

        return {
            "name": safe_name,
            "image": str(pgm_path),
            "yaml": str(yaml_path),
            "width": width,
            "height": height,
            "resolution": resolution,
            "saved_at": time.time(),
        }

    def list_maps(self) -> list[dict[str, Any]]:
        """Return saved map YAML files and their paired image files."""
        maps: list[dict[str, Any]] = []
        for yaml_path in sorted(self.storage_dir.glob("*.yaml"), key=lambda p: p.name.lower()):
            pgm_path = yaml_path.with_suffix(".pgm")
            maps.append(
                {
                    "name": yaml_path.stem,
                    "yaml": str(yaml_path),
                    "image": str(pgm_path) if pgm_path.exists() else None,
                    "modified_at": yaml_path.stat().st_mtime,
                }
            )
        return maps
