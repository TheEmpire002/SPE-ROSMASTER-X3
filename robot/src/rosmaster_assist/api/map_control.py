"""Everything the /map page and the app can do with the map.

Three jobs that only the server can do, because only it knows all the
parties involved (explorer, homing, interaction, drive commands, driver):

1. **Start the mapping drive and keep it running.** It is a mode of the
   explorer; emergency stop, hand and drive failure apply unchanged. New
   (04.11.2026): it no longer ends as soon as a page closes briefly or the
   driver is briefly gone due to a USB reset -- it waits and drives on.
   Without any watcher (no app, no /map page) it still stops after
   ``MAP_WATCHER_WAIT_S`` as before.
2. **Use the map.** Load a saved map, show where the robot is, drive to a
   tapped point (``Homing.start(target=...)``).
3. **Tick.** Pass new maps from mapping to localization and track the pose
   (computed in a thread, not in the event loop).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from .. import config
from ..mapping.occ_map import OccupancyMap
from ..navigation.explorer import MAP
from ..navigation.path_planning import frontiers_for_display
from ..protocol import alert_message

log = logging.getLogger(__name__)


class MapControl:
    """Mapping drive, localization and goal drive -- for /map and the app."""

    def __init__(self, *, mapping_manager, localization, explorer, homing, interaction, maneuver,
                 watchdog, driver, hub) -> None:
        self._manager = mapping_manager
        self._localization = localization
        self._explorer = explorer
        self._homing = homing
        self._interaction = interaction
        self._maneuver = maneuver
        self._watchdog = watchdog
        self._driver = driver
        self._hub = hub
        #: Open /map pages (counted by the router).
        self.pages = 0
        self._watcher_task: asyncio.Task | None = None
        self._interrupted: float | None = None   # driver gone during the mapping drive
        self._frontiers: list[list[float]] = []
        self._frontiers_time = 0.0
        # further cancel(reason) callbacks this module does not know, e.g. the healthbot
        self.also_cancel: list = []

    # --- 1. Mapping drive (interface as before for mapping/web.py) --------------------

    @property
    def active(self) -> bool:
        return self._explorer.mode == MAP

    def state(self) -> dict[str, Any]:
        z = self._explorer.state()
        active = self.active
        k = z["kartierfahrt"] or {}
        return {
            "aktiv": active,
            "text": z["text"] if active else None,
            "dauer_s": z["dauer_s"] if active else None,
            "schritt": k.get("schritt") if active else None,
            "bahnen": k.get("bahnen") if active else None,
            "luecken": k.get("luecken") if active else None,
            # Waiting to drive on after a drive failure.
            "unterbrochen": self._interrupted is not None,
            # Why the last mapping drive ended (not the last random
            # drive).
            "grund": (
                self._explorer.last_reason
                if not active and self._explorer.last_mode == MAP
                else None
            ),
        }

    def _cancel_others(self, reason: str) -> None:
        """Only one program steers."""
        self._homing.cancel(reason)
        self._explorer.cancel(reason)
        self._interaction.cancel(reason)
        self._maneuver.cancel(reason)
        for cancel in self.also_cancel:
            cancel(reason)

    def start(self, resume: bool = False) -> str | None:
        if self.active:
            return None
        if not self._manager.active:
            return ("Erst das Mapping starten: Die Kartierfahrt lenkt mit der "
                    "Lidar-Odometrie, und die läuft nur beim Kartieren.")
        if not self._explorer.available:
            return "Die Kartierfahrt geht nur mit Lidar."
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb."
        self._cancel_others("Kartierfahrt gestartet")
        self._interrupted = None
        if not self._explorer.start(MAP, resume=resume):
            return "Die Kartierfahrt ließ sich nicht starten."
        self._report("info", "Kartierfahrt fortgesetzt" if resume else "Kartierfahrt gestartet")
        return None

    def stop(self, reason: str) -> None:
        self._interrupted = None
        if self.active:
            self._explorer.cancel(reason)
            self._watchdog.stop()

    # Watchers: /map pages and app connections.

    def page_opened(self) -> None:
        self.pages += 1

    def page_closed(self) -> None:
        self.pages = max(0, self.pages - 1)
        self.check_watchers("keine Seite mehr offen")

    def nobody_watching(self) -> bool:
        return self.pages <= 0 and self._hub.count <= 0

    def check_watchers(self, reason: str) -> None:
        """If nobody is watching any more, stop after a deadline (not immediately).

        A phone whose screen turns off drops the connection; if it comes back
        within ``MAP_WATCHER_WAIT_S``, the robot simply drives on.
        """
        if not self.nobody_watching():
            return
        if not (self.active or self._homing.active or self._interrupted is not None):
            return
        if self._watcher_task is not None and not self._watcher_task.done():
            return
        log.warning("Nobody is watching any more -- stopping in %.0f s unless someone comes back",
                    config.MAP_WATCHER_WAIT_S)
        self._watcher_task = asyncio.get_running_loop().create_task(self._stop_later(reason))

    async def _stop_later(self, reason: str) -> None:
        await asyncio.sleep(config.MAP_WATCHER_WAIT_S)
        if not self.nobody_watching():
            return
        log.warning("No watcher came back -- stopping (%s)", reason)
        self.stop(reason)
        self._homing.cancel(reason)
        self._watchdog.stop()

    # Drive failure (USB reset) during the mapping drive.

    def drive_gone(self) -> None:
        if self.active:
            self._interrupted = time.monotonic()
            log.warning("Driver gone during the mapping drive -- will drive on after reconnecting")

    def drive_back(self) -> None:
        if self._interrupted is None:
            return
        gone_since = time.monotonic() - self._interrupted
        self._interrupted = None
        if gone_since > config.MAPPING_DRIVE_RESUME_S:
            self._report("warning", f"Kartierfahrt nicht fortgesetzt: Antrieb war {gone_since:.0f} s weg")
            return
        if self.nobody_watching():
            return
        error = self.start(resume=True)
        if error:
            self._report("warning", f"Kartierfahrt nicht fortgesetzt: {error}")

    def _report(self, level: str, text: str) -> None:
        try:
            asyncio.get_running_loop().create_task(self._hub.broadcast(alert_message(level, text)))
        except RuntimeError:
            log.info(text)  # outside the event loop (tests)

    # --- 2. Use the map -----------------------------------------------------------------

    def target(self, x, y, angle_deg=None) -> str | None:
        """Drive to a point on the map. Error text or None."""
        try:
            x, y = float(x), float(y)
            angle = math.radians(float(angle_deg)) if angle_deg is not None else None
        except (TypeError, ValueError):
            return "x und y müssen Zahlen sein (Meter im Kartenrahmen)"
        if not all(math.isfinite(v) for v in (x, y)):
            return "x und y müssen endlich sein"
        if self._localization.occ_map is None:
            return "Keine Karte geladen"
        if not self._localization.confident:
            return "Lage in der Karte unbekannt -- erst auf der Karte zeigen, wo er steht"
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb"
        self._cancel_others("neues Ziel")
        if not self._homing.start(target=(x, y), angle=angle):
            return self._homing.last_reason or "Fahrt zum Ziel ließ sich nicht starten"
        self._report("info", f"Fahrt zum Ziel ({x:.2f}, {y:.2f}) gestartet")
        return None

    def goal_stop(self, reason: str = "vom Nutzer abgebrochen") -> None:
        self._homing.cancel(reason)
        self._watchdog.stop()

    def set_pose(self, x, y, angle_deg=None) -> str | None:
        try:
            x, y = float(x), float(y)
            angle = math.radians(float(angle_deg)) if angle_deg is not None else None
        except (TypeError, ValueError):
            return "x und y müssen Zahlen sein"
        if self._localization.occ_map is None:
            return "Keine Karte geladen"
        self._homing.cancel("Lage wird neu gesucht")
        self._localization.set_pose(x, y, angle)
        return None

    def load(self, name: str) -> dict[str, Any]:
        self._cancel_others("Karte wird geladen")
        return self._manager.load_map(name)

    def start_here(self) -> bool:
        """"Startpunkt hier setzen" (set start here) also applies to map-based homing when a map exists."""
        pose = self._localization.pose()
        if pose is None or not self._localization.confident:
            return False
        self._manager.remember_start(pose)
        return True

    def display(self) -> dict[str, Any]:
        """For the /map page: pose, trail, current drive, frontiers."""
        k = (self._explorer.state().get("kartierfahrt") or {}) if self.active else {}
        return {
            "ortung": self._localization.state(),
            "spur": self._localization.trail(),
            "fahrt": self._homing.state(),
            "kartierfahrt": {"weg": k.get("weg") or [], "ziel": k.get("ziel")},
            "luecken": self._frontiers,
        }

    def state_brief(self) -> dict[str, Any] | None:
        """For state.karte in the app -- without trail and paths, they are large."""
        o = self._localization.state()
        if not o["karte"]:
            return None
        f = self._homing.state()
        return {
            "name": o["name"],
            "live": o["live"],
            "pose": o["pose"],
            "sicher": o["sicher"],
            "text": o["text"],
            "fahrt": {k: f[k] for k in ("aktiv", "art", "ziel", "rest_m", "grund")},
        }

    # --- 3. Tick -----------------------------------------------------------------------

    async def tick_loop(self, report_error) -> None:
        """Take over the map and track the pose; runs as long as the server runs."""
        tick = 1.0 / max(config.NAV_LOCALIZATION_HZ, 0.5)
        last_map = None
        while True:
            await asyncio.sleep(tick)
            try:
                if self._manager.active:
                    payload = self._manager.mapped_map()
                    stamp = payload.get("received_at") if payload else None
                    if payload is not None and stamp != last_map:
                        last_map = stamp
                        occ_map = await asyncio.to_thread(_build_map, payload)
                        self._localization.set_map(occ_map, live=True)
                occ_map = self._localization.occ_map
                if occ_map is not None:
                    await asyncio.to_thread(self._localization.match_scan)
                    if self.pages > 0 and time.monotonic() - self._frontiers_time > 5.0:
                        self._frontiers_time = time.monotonic()
                        self._frontiers = await asyncio.to_thread(frontiers_for_display, occ_map)
                else:
                    self._frontiers = []
            except Exception:
                report_error("Ortung in der Karte")


def _build_map(payload) -> OccupancyMap:
    """Map including distance field -- compute the expensive part right here in the thread."""
    occ_map = OccupancyMap.from_payload(payload)
    _ = occ_map.gradient
    return occ_map
