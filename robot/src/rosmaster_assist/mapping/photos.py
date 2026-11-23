"""Photos on the map: what the robot saw, and from where.

While the pose in a map is confident, the robot takes a photo with the
forward camera (``MAP_PHOTO_CAMERA``) every ``MAP_PHOTO_EVERY_M`` or after
turning ``MAP_PHOTO_EVERY_DEG`` -- not while turning fast (motion blur), and
not when there is already one from almost the same spot looking the same
way. "Foto hier" on the /map page takes one by hand.

Per map a folder ``<name>_bilder/`` beside the map files: the JPEGs as the
camera delivers them, plus ``index.json`` with position (m, map frame),
heading (degrees, 0 = map x axis, counter-clockwise), time and source.
The index also stores the map's frame id: when the main map is replaced
(new map, slam adopted) the old positions no longer fit, so the folder is
set aside as ``<name>_bilder.<stamp>.alt`` instead of showing photos in the
wrong places.

Only the event loop calls ``tick``/``take``; the files are written in a
thread, a slow sd card must not hold up localization.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import shutil
import threading
import time
from pathlib import Path
from typing import Any

from .. import config

log = logging.getLogger(__name__)

_INDEX = "index.json"
# names that may appear in urls: map names and photo files
_SAFE = re.compile(r"^[A-Za-z0-9_.\-äöüÄÖÜß]+$")


def _angle_diff(a: float, b: float) -> float:
    return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))


def safe_name(name: str) -> bool:
    return bool(_SAFE.match(name)) and ".." not in name


class MapPhotos:
    def __init__(self, storage_dir: Path, localization, camera, odometry=None) -> None:
        self._dir = Path(storage_dir)
        self._localization = localization
        self._camera = camera
        self._odometry = odometry
        self.auto = bool(config.MAP_PHOTO_AUTO)
        self._lock = threading.Lock()
        self._map: tuple[str, str | None] | None = None   # (name, frame id) of the index
        self._photos: list[dict[str, Any]] = []
        self._last_check: tuple[float, float, float] | None = None
        self._last_time = 0.0
        self._counter = 0
        # bumped on every change; the /map page reloads the list when it moves
        self.stand = 0
        self.last_error: str | None = None
        # no auto photos while a panorama turns the robot
        self.paused = False

    @property
    def available(self) -> bool:
        return bool(config.MAP_PHOTO_ENABLED) and self._camera is not None

    # --- which map ---

    def _current_map(self) -> tuple[str, str | None] | None:
        """Name and frame id of the map the pose refers to; None for the live slam map."""
        loc = self._localization
        occ_map = loc.occ_map
        if occ_map is None or loc.source not in ("dauer", "geladen"):
            return None
        name = getattr(occ_map, "name", None)
        if not name or not safe_name(str(name)):
            return None
        return str(name), getattr(occ_map, "frame_id", None)

    def map_here(self) -> tuple[str, str | None] | None:
        """The map photos would belong to right now, or None."""
        return self._current_map()

    def _folder(self, name: str) -> Path:
        return self._dir / f"{name}_bilder"

    def _switch(self, current: tuple[str, str | None]) -> None:
        """Load the index of ``current``; set it aside if it belongs to another frame."""
        if current == self._map:
            return
        name, frame_id = current
        folder = self._folder(name)
        photos: list[dict[str, Any]] = []
        try:
            data = json.loads((folder / _INDEX).read_text(encoding="utf-8"))
            saved_frame = data.get("frame_id")
            photos = [p for p in data.get("fotos", []) if isinstance(p, dict)]
            if photos and frame_id and saved_frame and saved_frame != frame_id:
                aside = folder.with_name(f"{folder.name}.{time.strftime('%Y%m%d-%H%M%S')}.alt")
                os.replace(folder, aside)
                log.warning("Map photos of '%s' belong to an older map -- moved to %s",
                            name, aside.name)
                photos = []
        except FileNotFoundError:
            pass
        except (OSError, ValueError):
            log.warning("Map photo index of '%s' unreadable -- starting a new one", name,
                        exc_info=True)
        with self._lock:
            self._map = current
            self._photos = photos
            self._last_check = None
            self.stand += 1

    # --- taking photos ---

    def tick(self) -> None:
        """Take a photo if the robot has moved on far enough (auto mode)."""
        if not self.available or not self.auto or self.paused:
            return
        current = self._current_map()
        if current is None or not self._localization.confident:
            return
        self._switch(current)
        pose = self._localization.pose()
        if pose is None:
            return
        now = time.monotonic()
        if now - self._last_time < config.MAP_PHOTO_MIN_INTERVAL_S:
            return
        if self._odometry is not None:
            if abs(self._odometry.velocity()[2]) > config.MAP_PHOTO_MAX_OMEGA:
                return
        last = self._last_check
        if last is not None and (
            math.hypot(pose[0] - last[0], pose[1] - last[1]) < config.MAP_PHOTO_EVERY_M
            and _angle_diff(pose[2], last[2]) < config.MAP_PHOTO_EVERY_DEG
        ):
            return
        self._last_check = tuple(pose)
        if self._similar(pose) or self._count_photos() >= config.MAP_PHOTO_MAX:
            return
        self._capture(current, pose, "auto")

    def take(self) -> dict[str, Any] | str:
        """Photo right now (by hand). Returns the entry or an error text (german)."""
        if not self.available:
            return "Fotos sind ausgeschaltet oder es gibt keine Kamera"
        current = self._current_map()
        if current is None:
            return "Keine Karte, in der er seine Lage kennt (Live-Karte von SLAM zählt nicht)"
        if not self._localization.confident:
            return "Lage in der Karte ist gerade unsicher"
        pose = self._localization.pose()
        if pose is None:
            return "Lage in der Karte unbekannt"
        self._switch(current)
        entry = self._capture(current, pose, "hand")
        return entry if entry is not None else (self.last_error or "Kein Kamerabild")

    def _similar(self, pose) -> bool:
        return any(
            p.get("art", "foto") == "foto"
            and math.hypot(p["x"] - pose[0], p["y"] - pose[1]) < config.MAP_PHOTO_SIMILAR_M
            and _angle_diff(math.radians(p["winkel"]), pose[2]) < config.MAP_PHOTO_SIMILAR_DEG
            for p in self._photos
        )

    def _count_photos(self) -> int:
        return sum(1 for p in self._photos if p.get("art", "foto") == "foto")

    def add_panorama(self, current, pose, data: bytes, shots,
                     camera: str | None = None) -> dict[str, Any] | None:
        """Store a stitched panorama taken at ``pose`` (centre = its heading).

        ``shots``: the single photos as ``[(jpeg, heading_rad, pitch_rad), ...]``; kept in
        ``<id>/`` with their headings for re-stitching and a 3d model later
        """
        if current is None:
            self.last_error = "Keine Karte für das Panorama"
            return None
        self._switch(current)
        self._counter += 1
        photo_id = f"pano-{time.strftime('%Y%m%d-%H%M%S')}-{self._counter}"
        entry = {
            "id": photo_id,
            "art": "panorama",
            "datei": f"{photo_id}.jpg",
            "x": round(float(pose[0]), 3),
            "y": round(float(pose[1]), 3),
            "winkel": round(math.degrees(pose[2]) % 360.0, 1),
            "zeit": round(time.time(), 1),
            "quelle": "hand",
            "kamera": camera or getattr(self._camera, "name", None),
            "aufnahmen": len(shots),
            "neigung": round(math.degrees(sum(s[2] for s in shots) / len(shots)), 1) if shots else 0.0,
            "einzelbilder": [
                {"datei": f"{photo_id}/{k:02d}.jpg", "winkel": round(math.degrees(hd) % 360.0, 2),
                 "neigung": round(math.degrees(pt), 1)}
                for k, (_, hd, pt) in enumerate(shots)
            ] if config.PANORAMA_KEEP_SHOTS else [],
        }
        with self._lock:
            self._photos.append(entry)
            photos = list(self._photos)
            self.stand += 1
        # called from a worker thread already, so write right here
        self._write(current, entry, data, photos)
        if self.last_error is None and config.PANORAMA_KEEP_SHOTS:
            try:
                folder = self._folder(current[0]) / photo_id
                folder.mkdir(parents=True, exist_ok=True)
                for k, (jpeg, _, _) in enumerate(shots):
                    (folder / f"{k:02d}.jpg").write_bytes(jpeg)
            except OSError:
                log.warning("Single shots of panorama %s not saved", photo_id, exc_info=True)
        return entry if self.last_error is None else None

    def _capture(self, current, pose, source: str) -> dict[str, Any] | None:
        frame, taken = self._camera.latest_with_time()
        if frame is None or time.monotonic() - taken > config.MAP_PHOTO_FRAME_MAX_AGE_S:
            self.last_error = "Kein frisches Kamerabild"
            return None
        data, mime = frame
        if "jpeg" not in str(mime):
            self.last_error = f"Kamera liefert {mime}, kein JPEG"
            return None
        self._last_time = time.monotonic()
        self._counter += 1
        photo_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{self._counter}"
        entry = {
            "id": photo_id,
            "art": "foto",
            "datei": f"{photo_id}.jpg",
            "x": round(float(pose[0]), 3),
            "y": round(float(pose[1]), 3),
            "winkel": round(math.degrees(pose[2]) % 360.0, 1),
            "zeit": round(time.time(), 1),
            "quelle": source,
            "kamera": getattr(self._camera, "name", None),
        }
        with self._lock:
            self._photos.append(entry)
            photos = list(self._photos)
            self.stand += 1
        # writing in a thread; the entry is shown at once, the file follows
        threading.Thread(target=self._write, args=(current, entry, data, photos),
                         name="kartenfoto", daemon=True).start()
        log.info("Map photo %s (%s) at x=%.2f y=%.2f %.0f°", photo_id, source,
                 entry["x"], entry["y"], entry["winkel"])
        return entry

    def _write(self, current, entry, data: bytes, photos) -> None:
        name, frame_id = current
        folder = self._folder(name)
        try:
            folder.mkdir(parents=True, exist_ok=True)
            (folder / entry["datei"]).write_bytes(data)
            self._write_index(folder, frame_id, photos)
            self.last_error = None
        except OSError as exc:
            self.last_error = f"Foto nicht gespeichert: {exc}"
            log.warning("Map photo could not be saved", exc_info=True)
            with self._lock:
                self._photos = [p for p in self._photos if p["id"] != entry["id"]]
                self.stand += 1

    def _write_index(self, folder: Path, frame_id, photos) -> None:
        tmp = folder / (_INDEX + ".tmp")
        tmp.write_text(json.dumps({"frame_id": frame_id, "fotos": photos}, ensure_ascii=False,
                                  indent=1), encoding="utf-8")
        os.replace(tmp, folder / _INDEX)

    # --- for the page ---

    def delete(self, photo_id: str) -> bool:
        if self._map is None:
            return False
        name, frame_id = self._map
        with self._lock:
            entry = next((p for p in self._photos if p["id"] == photo_id), None)
            if entry is None:
                return False
            self._photos = [p for p in self._photos if p["id"] != photo_id]
            photos = list(self._photos)
            self.stand += 1
        folder = self._folder(name)
        try:
            (folder / entry["datei"]).unlink(missing_ok=True)
            if entry.get("art") == "panorama":
                shutil.rmtree(folder / entry["id"], ignore_errors=True)
            self._write_index(folder, frame_id, photos)
        except OSError:
            log.warning("Map photo %s could not be deleted", photo_id, exc_info=True)
        return True

    def listing(self) -> dict[str, Any]:
        current = self._current_map()
        if current is not None:
            self._switch(current)
        name = current[0] if current is not None else None
        with self._lock:
            photos = list(self._photos) if current is not None else []
        return {
            "karte": name,
            "auto": self.auto,
            "verfuegbar": self.available,
            "fotos": [dict(p, url=f"/api/mapping/fotos/{name}/{p['datei']}") for p in photos],
        }

    def brief(self) -> dict[str, Any]:
        current = self._current_map()
        if current is not None:
            # another map loaded: its photos, and a new stand so the page reloads
            self._switch(current)
        return {
            "verfuegbar": self.available,
            "auto": self.auto,
            "karte": current[0] if current is not None else None,
            "anzahl": len(self._photos) if current is not None else 0,
            "stand": self.stand,
            "fehler": self.last_error,
        }

    def file(self, name: str, filename: str) -> Path | None:
        if not safe_name(name) or not safe_name(filename) or not filename.endswith(".jpg"):
            return None
        path = self._folder(name) / filename
        return path if path.is_file() else None
