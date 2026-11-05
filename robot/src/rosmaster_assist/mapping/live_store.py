"""Saving and loading the main map of always-on mapping (E14).

Per map three kinds of files in the maps folder:

* ``<name>.npz``: the log-odds grid (float16), origin, resolution, frame id --
  what the mapper continues with; the pgm alone would lose how sure a cell is;
* ``<name>.pgm/.yaml``: the trinary map as before (``MapService``), so the
  map list, other tools and an emergency start without the npz still work;
* ``<name>.json``: starting point, last confident pose, frame id (later
  also anchors and the breadcrumb trail).

``_aktiv.json`` names the map that grows. Everything is written atomically
(``.tmp``, then ``os.replace``); a power cut leaves the old state. Before a
map is replaced there is a backup in ``backups/`` (not in the map list).
"""

from __future__ import annotations

import io
import json
import logging
import os
import shutil
import threading
import time
from pathlib import Path

import numpy as np

from .. import config
from .map_service import MapService
from .occ_map import OccupancyMap, read_extra, write_extra

log = logging.getLogger(__name__)

_ACTIVE = "_aktiv.json"


def _atomic_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    try:
        tmp.write_bytes(data)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)


class LiveStore:
    """Files of the main map in the maps folder of ``MapService``."""

    def __init__(self, map_service: MapService) -> None:
        self._service = map_service
        self.folder = map_service.storage_dir
        self.last_saved: float | None = None
        self.last_error: str | None = None
        # the json is read, changed and written from the loop and from threads
        self._lock = threading.RLock()

    # --- which map grows ---------------------------------------------------------

    def active_name(self) -> str:
        try:
            data = json.loads((self.folder / _ACTIVE).read_text(encoding="utf-8"))
            name = str(data.get("name") or "").strip()
            if name:
                return name
        except (OSError, ValueError, AttributeError):
            pass
        return config.LIVE_MAP_NAME

    def set_active(self, name: str) -> None:
        _atomic_bytes(self.folder / _ACTIVE,
                      json.dumps({"name": name}, ensure_ascii=False).encode("utf-8"))

    def paths(self, name: str) -> dict[str, Path]:
        return {k: self.folder / f"{name}.{k}" for k in ("npz", "pgm", "yaml", "json")}

    def exists(self, name: str) -> bool:
        p = self.paths(name)
        return p["npz"].exists() or (p["yaml"].exists() and p["pgm"].exists())

    # --- loading -------------------------------------------------------------------

    def load(self, name: str) -> dict | None:
        """The saved map: {"lo"|"occ_map", "origin", "resolution", "frame_id", "extra"}.

        a broken npz is renamed to .broken and the pgm is used instead -- the
        robot then starts with sure walls, only the finer certainty is lost
        """
        p = self.paths(name)
        extra = read_extra(p["yaml"]) if p["yaml"].exists() else {}
        if p["npz"].exists():
            try:
                with np.load(p["npz"], allow_pickle=False) as data:
                    lo = data["lo"].astype(np.float32)
                    origin = tuple(float(v) for v in data["origin"][:2])
                    resolution = float(data["resolution"])
                    frame_id = str(data["frame_id"]) if "frame_id" in data else None
                if lo.ndim != 2 or not np.isfinite(lo).all() or resolution <= 0:
                    raise ValueError("invalid grid")
                return {"lo": lo, "origin": origin, "resolution": resolution,
                        "frame_id": frame_id or extra.get("frame_id"), "extra": extra}
            except Exception as exc:
                broken = p["npz"].with_name(f"{name}.{int(time.time())}.broken")
                log.error("Main map %s unreadable (%s) -- renamed to %s, using the pgm",
                          p["npz"].name, exc, broken.name)
                try:
                    os.replace(p["npz"], broken)
                except OSError:
                    log.exception("could not rename the broken map")
        if p["yaml"].exists() and p["pgm"].exists():
            try:
                occ_map = OccupancyMap.load(p["yaml"])
            except Exception:
                log.exception("Map %s could not be loaded", name)
                return None
            return {"occ_map": occ_map, "origin": occ_map.origin,
                    "resolution": occ_map.resolution,
                    "frame_id": extra.get("frame_id"), "extra": extra}
        return None

    # --- saving --------------------------------------------------------------------

    def save(self, name: str, arrays, snapshot: OccupancyMap | None, frame_id: str | None,
             extra: dict) -> None:
        """Grid, trinary map and extras; ``extra`` is merged into the existing json."""
        p = self.paths(name)
        with self._lock:
            self._save(name, p, arrays, snapshot, frame_id, extra)

    def _save(self, name, p, arrays, snapshot, frame_id, extra) -> None:
        try:
            if arrays is not None:
                lo, origin, resolution = arrays
                buffer = io.BytesIO()
                np.savez_compressed(
                    buffer, lo=lo.astype(np.float16),
                    origin=np.asarray(origin, dtype=np.float64),
                    resolution=np.float64(resolution), frame_id=np.str_(frame_id or ""),
                )
                _atomic_bytes(p["npz"], buffer.getvalue())
            if snapshot is not None:
                self._service.save_map(snapshot.as_payload(), name=name)
            self.save_extra(name, dict(extra, frame_id=frame_id, gespeichert=time.time()))
            self.last_saved = time.time()
            self.last_error = None
        except Exception as exc:
            self.last_error = str(exc)
            raise

    def save_extra(self, name: str, changes: dict) -> None:
        p = self.paths(name)
        with self._lock:
            data = read_extra(p["yaml"]) if p["json"].exists() else {}
            data.update(changes)
            write_extra(p["yaml"], data)

    def backup(self, name: str) -> Path | None:
        """Copy the map's files to backups/; keep LIVE_MAP_BACKUPS per map."""
        files = [f for f in self.paths(name).values() if f.exists()]
        if not files:
            return None
        folder = self.folder / "backups"
        folder.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        for f in files:
            shutil.copy2(f, folder / f"{name}.{stamp}{f.suffix}")
        stamps = sorted({b.name[len(name) + 1:].split(".")[0]
                         for b in folder.glob(f"{name}.*")})
        for old in stamps[:-max(config.LIVE_MAP_BACKUPS, 1)]:
            for b in folder.glob(f"{name}.{old}.*"):
                b.unlink(missing_ok=True)
        return folder

    def remove(self, name: str) -> None:
        for f in self.paths(name).values():
            f.unlink(missing_ok=True)
