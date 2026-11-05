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
4. **Always-on mapping** (E14). The main map is loaded at start; behind the
   integrate gate every drive extends it. Snapshots for localization and
   planner, autosave and the last confident pose run beside the tick.
   While slam maps, it pauses; afterwards the slam map can become the main
   map (``adopt_slam``) or be dropped (``discard_slam``).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from .. import config
from ..mapping.occ_map import OccupancyMap
from ..navigation.breadcrumbs import Breadcrumbs
from ..navigation.explorer import MAP
from ..navigation.map_localization import Stage
from ..navigation.path_planning import frontiers_for_display
from ..protocol import alert_message

log = logging.getLogger(__name__)


class MapControl:
    """Mapping drive, localization and goal drive -- for /map and the app."""

    def __init__(self, *, mapping_manager, localization, explorer, homing, interaction, maneuver,
                 watchdog, driver, hub, live_mapper=None, live_store=None,
                 scan_odometry=None) -> None:
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
        # always-on mapping
        self._mapper = live_mapper
        self._store = live_store
        self._scan_odometry = scan_odometry
        self._live_name = config.LIVE_MAP_NAME
        self._extra: dict[str, Any] = {}          # json of the main map
        self._snap_task: asyncio.Task | None = None
        self._save_task: asyncio.Task | None = None
        self._snap_time = 0.0
        self._save_time = time.monotonic()
        self._pose_time = 0.0
        self._pose_saved: tuple | None = None
        self._pose_dirty = False
        # mock only: map pose of the simulation's true frame
        self._sim_origin: tuple | None = None
        # main map -> odometry correction when slam started, and the odometry run
        self._slam_from: tuple | None = None
        # the trail driven in the main map, and the anchors on it (way back)
        self.crumbs = Breadcrumbs()
        self._crumbs_dirty = False
        self._crumb_confident = False
        localization.near_source = self.crumbs.near

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
        occ_map = self._localization.occ_map
        if occ_map is None:
            return "Keine Karte geladen"
        if not self._localization.confident:
            return "Lage in der Karte unbekannt -- erst auf der Karte zeigen, wo er steht"
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb"
        # only into space the map knows to be free: what was never scanned
        # could be a wall
        if occ_map.nearest_free(x, y, config.NAV_GOAL_KNOWN_M) is None:
            return "Ziel liegt außerhalb der bekannten Karte"
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
        if not self._live_ok:
            return self._manager.load_map(name)
        # always-on mapping: the chosen map becomes the one that grows
        if self._manager.active:
            raise RuntimeError("Erst das Mapping stoppen: Gerade entsteht eine neue Karte.")
        saved = self._store.load(name)
        if saved is None:
            raise ValueError(f"Karte '{name}' nicht gefunden.")
        self._save_now()
        self._take_saved(name, saved)
        self._store.set_active(name)
        occ_map = self._localization.occ_map
        start = self._extra.get("start") or [0.0, 0.0, 0.0]
        return {"name": name, "breite": occ_map.width if occ_map else None,
                "hoehe": occ_map.height if occ_map else None,
                "aufloesung": occ_map.resolution if occ_map else None, "start": start}

    def simulation_moved(self, before, after) -> bool:
        """Mock only: ``set_home`` put the imagined robot back to its start.

        for the localization that is a kidnapping -- the map would freeze and
        home would land in the wrong place. The simulation knows the true
        pose, so the new pose in the map is a hint, and it is home. The
        current map pose is only the fallback: after earlier teleports it
        may already be wrong.
        """
        origin = self._sim_origin
        if origin is None:
            pose = self._localization.pose()
            if pose is None:
                return False
            origin = _compose(pose, _inverse(before))
        x, y, angle = _compose(origin, after)
        self._localization.set_pose(x, y, angle)
        self._manager.remember_start((x, y, angle))
        return True

    # --- missions: remember a place, come back to it (E14) ---------------------------------

    def mission_start(self) -> str | None:
        """"Missionsstart hier": the way back leads here. Error text or None."""
        if self._localization.source != "dauer":
            return "Missionen gehen nur in der Hauptkarte (Dauerkartierung)"
        pose = self._localization.pose()
        if pose is None or not self._localization.confident:
            return "Lage in der Karte unsicher -- erst wenn sie sicher ist"
        self.crumbs.set_anchor("mission", pose)
        self._crumbs_dirty = True
        self._report("info", "Missionsstart gesetzt -- der Rückweg führt hierher")
        return None

    def return_to(self, goal: str = "mission") -> str | None:
        """Drive back to an anchor ("mission" or "home") along the trail. Error text or None."""
        if goal not in ("mission", "home"):
            return "ziel muss \"mission\" oder \"home\" sein"
        if self._localization.occ_map is None:
            return "Keine Karte geladen"
        if not self._localization.confident:
            return "Lage in der Karte unsicher -- erst wenn sie sicher ist"
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb"
        if goal == "mission":
            anchor = self.crumbs.anchor("mission")
            if anchor is None:
                return "Kein Missionsstart gesetzt"
            since = "mission"
        else:
            anchor = self._localization.start
            since = None
        self._cancel_others("Rückfahrt")
        if not self._homing.start(target=(anchor[0], anchor[1]), angle=anchor[2], kind="zurueck",
                                  corridor=self.crumbs.corridor(since),
                                  fallback_points=self.crumbs.reversed_path(since)):
            return self._homing.last_reason or "Rückfahrt ließ sich nicht starten"
        where = "zum Missionsstart" if goal == "mission" else "zum Ausgangspunkt"
        self._report("info", f"Rückfahrt {where} gestartet")
        return None

    def map_live(self, active: bool) -> str | None:
        """Switch always-on mapping on/off for the user (the pose is still tracked)."""
        if not self._live_ok:
            return "Dauerkartierung ist abgeschaltet (LIVE_MAP_ENABLED)"
        if active:
            self._mapper.resume("nutzer")
        else:
            self._mapper.pause("nutzer")
        return None

    def new_map(self) -> str | None:
        """Begin a new main map at the next standstill; the old one goes to the backups."""
        if not self._live_ok:
            return "Dauerkartierung ist abgeschaltet (LIVE_MAP_ENABLED)"
        if self._manager.active:
            return "Erst das Mapping stoppen"
        self._cancel_others("neue Karte")
        try:
            self._save_now()
            self._store.backup(self._live_name)
        except Exception:
            log.exception("Main map could not be backed up before starting a new one")
            return "Sicherung der alten Karte fehlgeschlagen -- neue Karte nicht begonnen"
        self._mapper.start_empty()
        self._localization.unload()
        self.crumbs.clear()
        self._extra = {}
        self._pose_saved = None
        self._sim_origin = None
        self._report("info", "Neue Karte: bitte kurz stillhalten, dann beginnt sie hier")
        return None

    # --- slam as the optional first mapping (E14) ---------------------------------------

    def before_slam(self) -> None:
        """Slam is about to start: save the main map, remember how it lies.

        slam maps in the lidar odometry frame, which keeps running; with the
        main map's correction at this moment both frames can be related later
        """
        self._slam_from = None
        if not self._live_ok:
            return
        if self._localization.source == "dauer" and self._scan_odometry is not None:
            correction = self._localization.correction()
            if correction is not None and self._localization.confident:
                self._slam_from = (correction, self._scan_odometry.run)
        # paused right away: the tick only notices slam a moment later
        self._mapper.pause("slam")
        try:
            self._save_now()
        except Exception:
            log.exception("Main map could not be saved before slam")

    @property
    def _slam_result(self) -> bool:
        """Slam has stopped and its map is still loaded: adopt or discard."""
        return (self._live_ok and not self._manager.active
                and self._localization.source == "slam")

    def _main_to_odometry(self) -> tuple | None:
        """Main map frame -> odometry (= slam) frame, if the odometry ran through."""
        if self._slam_from is None or self._scan_odometry is None:
            return None
        correction, run = self._slam_from
        return _inverse(correction) if run == self._scan_odometry.run else None

    def adopt_slam(self) -> str | None:
        """The slam map becomes the main map and grows from now on. Error text or None."""
        if not self._live_ok:
            return "Dauerkartierung ist abgeschaltet (LIVE_MAP_ENABLED)"
        if self._manager.active:
            return "Erst das Mapping stoppen"
        occ_map = self._localization.occ_map
        if occ_map is None or self._localization.source != "slam":
            return "Keine SLAM-Karte zum Übernehmen"
        self._cancel_others("SLAM-Karte wird übernommen")
        try:
            if self._mapper.has_map:
                self._save_now()
                self._store.backup(self._live_name)
        except Exception:
            log.exception("Main map could not be backed up before adopting the slam map")
            return "Sicherung der Hauptkarte fehlgeschlagen -- SLAM-Karte nicht übernommen"
        correction = self._localization.correction()
        slam_start = tuple(self._localization.start)
        old_start = _pose3(self._extra.get("start"))
        to_slam = self._main_to_odometry()
        self._mapper.import_occupancy(occ_map)
        # slam may have bent its frame a little (loop closure): good enough
        # for the trail and the anchors, the pose itself stays tracked
        if to_slam is not None:
            self.crumbs.transform(to_slam)
            start = _compose(to_slam, old_start) if old_start is not None else slam_start
        else:
            self.crumbs.clear()
            start = slam_start
        self._extra = {"start": list(start)}
        self._pose_saved = None
        self._sim_origin = None
        self._slam_from = None
        if correction is not None:
            self._attach_main(None, correction=correction, start=start)
        else:
            self._attach_main(startup_stages(self._extra), start=start)
        self._store.set_active(self._live_name)
        self._crumbs_dirty = True
        self._save_time = 0.0   # save the new main map soon
        self._report("info", "SLAM-Karte als Hauptkarte übernommen -- sie wächst ab jetzt "
                             "beim Fahren weiter")
        return None

    def discard_slam(self) -> str | None:
        """Back to the main map; the pose is searched where the odometry expects it."""
        if not self._live_ok:
            return "Dauerkartierung ist abgeschaltet (LIVE_MAP_ENABLED)"
        if self._manager.active:
            return "Erst das Mapping stoppen"
        if self._localization.source != "slam":
            return "Keine SLAM-Karte zum Verwerfen"
        self._cancel_others("SLAM-Karte verworfen")
        stages: list[Stage] = []
        odo = self._scan_odometry.pose() if self._scan_odometry is not None else None
        if self._main_to_odometry() is not None and odo is not None:
            guess = _compose(self._slam_from[0], odo)
            share = config.NAV_SEARCH_MIN_SHARE
            stages.append(Stage((guess[0], guess[1]), config.LIVE_START_SEARCH_NEAR_M, guess[2],
                                share, text="sucht die eigene Lage, wo die Odometrie ihn sieht"))
            stages.append(Stage((guess[0], guess[1]), config.NAV_SEARCH_RADIUS_M, None, share,
                                text="sucht die eigene Lage rund um diese Stelle"))
        extra = dict(self._extra)
        if self._pose_saved is not None:
            extra["last_pose"] = list(self._pose_saved)
        self._slam_from = None
        self._localization.unload()
        if self._mapper.has_map:
            self._attach_main(stages + startup_stages(extra))
        self._report("info", "SLAM-Karte verworfen -- zurück in der Hauptkarte")
        return None

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
            "kartieren": self.live_status(),
            "brotkrumen": self.crumbs.display(),
            "anker": {k: [round(v, 3) for v in a["pose"]] for k, a in self.crumbs.anchors.items()},
        }

    def state_brief(self) -> dict[str, Any] | None:
        """For state.karte in the app -- without trail and paths, they are large."""
        o = self._localization.state()
        if not o["karte"]:
            return None
        f = self._homing.state()
        mission = self.crumbs.anchor("mission")
        return {
            "name": o["name"],
            "live": o["live"],
            "pose": o["pose"],
            "sicher": o["sicher"],
            "text": o["text"],
            "fahrt": {k: f[k] for k in ("aktiv", "art", "ziel", "rest_m", "grund", "wartet")},
            "kartieren": self._live_brief(),
            "mission": [round(v, 3) for v in mission] if mission else None,
            "spur_m": round(self.crumbs.length_m("mission" if mission else None), 1),
        }

    def _live_brief(self) -> dict[str, Any] | None:
        if self._mapper is None:
            return None
        g = self._mapper.status()
        return {"aktiv": g["aktiv"], "zustand": g["zustand"], "grund": g["grund"]}

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
                self._live_switches()
                occ_map = self._localization.occ_map
                if occ_map is not None:
                    await asyncio.to_thread(self._track)
                    if self.pages > 0 and time.monotonic() - self._frontiers_time > 5.0:
                        self._frontiers_time = time.monotonic()
                        self._frontiers = await asyncio.to_thread(frontiers_for_display, occ_map)
                else:
                    self._frontiers = []
                    if self._founding:
                        await asyncio.to_thread(self._found)
                self._live_background(report_error)
            except Exception:
                report_error("Ortung in der Karte")

    def _track(self) -> None:
        """Match the scan, then let the mapper decide whether it goes into the map."""
        track = self._localization.match_scan()
        if self._growing:
            self._mapper.step(track)
        self._drop_crumb()

    def _drop_crumb(self) -> None:
        """Breadcrumbs while the pose in the main map is confident; a gap when it is not."""
        if self._localization.source != "dauer":
            return
        if self._localization.confident:
            pose = self._localization.pose()
            if pose is not None:
                strict = self._mapper is not None and self._mapper.integrating
                if self.crumbs.add(pose, strict):
                    self._crumbs_dirty = True
                self._crumb_confident = True
        elif self._crumb_confident:
            self.crumbs.gap()
            self._crumb_confident = False

    # --- 4. Always-on mapping ---------------------------------------------------------

    @property
    def _live_ok(self) -> bool:
        return self._mapper is not None and self._store is not None and config.LIVE_MAP_ENABLED

    @property
    def _growing(self) -> bool:
        """The loaded map is the main map, so tracking results may extend it."""
        if not self._live_ok or self._manager.active:
            return False
        occ_map = self._localization.occ_map
        return (occ_map is not None and self._localization.source == "dauer"
                and occ_map.frame_id is not None and occ_map.frame_id == self._mapper.frame_id)

    @property
    def _founding(self) -> bool:
        return (self._live_ok and not self._mapper.has_map and not self._manager.active
                and self._localization.occ_map is None)

    def live_startup(self) -> None:
        """Server start: the main map, searched at the last pose -- or a new one."""
        if not self._live_ok:
            return
        if self._scan_odometry is not None:
            # founding a map needs the lidar odometry before any map exists
            self._scan_odometry.start("dauer")
        name = self._store.active_name()
        saved = self._store.load(name)
        if saved is None:
            self._live_name = name
            self._mapper.name = name
            self._extra = {}
            self._mapper.start_empty()
            log.info("Always-on mapping: no main map '%s' yet -- founding one at standstill", name)
            return
        self._take_saved(name, saved)
        log.info("Always-on mapping: main map '%s' loaded", name)

    def _take_saved(self, name: str, saved: dict) -> None:
        if "lo" in saved:
            self._mapper.load_array(saved["lo"], saved["origin"], saved["resolution"],
                                    saved["frame_id"])
        else:
            # a map without grid (slam_toolbox, an old pgm) grows from now on and
            # will be overwritten: keep the original
            try:
                self._store.backup(name)
            except OSError:
                log.warning("No backup of map '%s' before mapping on", name, exc_info=True)
            self._mapper.import_occupancy(saved["occ_map"], saved["frame_id"])
        self._live_name = name
        self._mapper.name = name
        self._extra = dict(saved.get("extra") or {})
        self._pose_saved = None
        self._sim_origin = None
        self.crumbs.from_json(self._extra)
        self._attach_main(startup_stages(self._extra))

    def _attach_main(self, stages: list[Stage] | None, correction=None, start=None) -> None:
        snap = self._mapper.snapshot()
        if snap is None:
            return
        if start is None:
            start = _pose3(self._extra.get("start")) or (0.0, 0.0, 0.0)
        self._localization.set_map(snap, start=start, source="dauer", stages=stages,
                                   correction=correction)

    def _found(self) -> None:
        """No map at all: found one from still scans, here is home."""
        if self._scan_odometry is None:
            return
        m = self._scan_odometry.measurement()
        if not self._mapper.bootstrap(m):
            return
        self._extra = {"start": list(m.pose)}
        self.crumbs.clear()
        self._attach_main(None, correction=(0.0, 0.0, 0.0), start=tuple(m.pose))
        true = self._sim_true()
        # standing still, so the scan and the true pose belong together
        self._sim_origin = _compose(tuple(m.pose), _inverse(true)) if true else None
        self._mapper.gate.integrating_now(self._localization.map_gen)
        self._store.set_active(self._live_name)
        self._save_time = 0.0   # save the founded map soon
        log.info("Always-on mapping: new main map '%s' founded", self._live_name)

    def _live_switches(self) -> None:
        """Pause while slam maps; take the main map back when nothing else is loaded."""
        if not self._live_ok:
            return
        if self._manager.active:
            self._mapper.pause("slam")
            return
        self._mapper.resume("slam")
        if self._mapper.has_map and self._localization.occ_map is None:
            extra = dict(self._extra)
            if self._pose_saved is not None:
                extra["last_pose"] = list(self._pose_saved)
            self._attach_main(startup_stages(extra))

    def _live_background(self, report_error) -> None:
        """Snapshot, autosave and last pose -- each in its own thread, never waited for."""
        if not self._live_ok:
            return
        now = time.monotonic()
        growing = self._growing
        if (growing and now - self._snap_time >= config.LIVE_MAP_SNAPSHOT_S
                and (self._snap_task is None or self._snap_task.done())):
            self._snap_time = now
            self._snap_task = asyncio.get_running_loop().create_task(
                self._guarded(self._refresh, report_error, "Kartenschnappschuss"))
        if growing:
            self._remember_pose(now)
            if self._sim_origin is None and self._localization.confident:
                true, pose = self._sim_true(), self._localization.pose()
                if true is not None and pose is not None:
                    self._sim_origin = _compose(pose, _inverse(true))
        if ((self._mapper.changed or self._pose_dirty or self._crumbs_dirty)
                and now - self._save_time >= config.LIVE_MAP_SAVE_S
                and (self._save_task is None or self._save_task.done())):
            self._save_time = now
            self._save_task = asyncio.get_running_loop().create_task(
                self._guarded(self._save_now, report_error, "Karte speichern"))

    @staticmethod
    async def _guarded(job, report_error, what: str) -> None:
        try:
            await asyncio.to_thread(job)
        except Exception:
            report_error(what)

    def _refresh(self) -> None:
        snap = self._mapper.snapshot()
        if snap is not None and snap is not self._localization.occ_map:
            self._localization.update_map(snap)

    def _remember_pose(self, now: float) -> None:
        if now - self._pose_time < config.LIVE_POSE_SAVE_S or not self._localization.confident:
            return
        pose = self._localization.pose()
        if pose is None:
            return
        self._pose_time = now
        last = self._pose_saved
        if last is not None and (
            math.hypot(pose[0] - last[0], pose[1] - last[1]) < config.LIVE_POSE_SAVE_MOVE_M
            and abs(math.degrees(math.atan2(math.sin(pose[2] - last[2]),
                                            math.cos(pose[2] - last[2]))))
            < config.LIVE_POSE_SAVE_MOVE_DEG
        ):
            return
        self._pose_saved = tuple(pose)
        self._pose_dirty = True
        # small file, written right away (a power cut must not cost the pose),
        # but in a thread: a slow sd card must not hold up the loop
        asyncio.get_running_loop().create_task(asyncio.to_thread(self._write_pose, pose))

    def _write_pose(self, pose) -> None:
        try:
            self._store.save_extra(self._live_name, {"last_pose": [round(v, 4) for v in pose]})
            self._pose_dirty = False
        except OSError:
            log.warning("Last pose could not be saved", exc_info=True)

    def _save_now(self) -> None:
        """Save the main map if it changed (thread or shutdown)."""
        if not self._live_ok or not self._mapper.has_map:
            return
        if not self._mapper.changed and not self._pose_dirty and not self._crumbs_dirty:
            return
        self._mapper.changed = False
        self._crumbs_dirty = False
        arrays = self._mapper.arrays()
        snap = self._mapper.snapshot()
        if snap is not None and self._growing:
            self._localization.update_map(snap)
        extra: dict[str, Any] = {}
        if self._localization.source == "dauer":
            extra["start"] = [round(v, 4) for v in self._localization.start]
        if self._pose_saved is not None:
            extra["last_pose"] = [round(v, 4) for v in self._pose_saved]
        extra.update(self.crumbs.to_json())
        self._extra.update(extra)
        try:
            self._store.save(self._live_name, arrays, snap, self._mapper.frame_id, extra)
            self._pose_dirty = False
        except Exception:
            self._mapper.changed = True   # try again next time
            raise
        log.info("Main map '%s' saved", self._live_name)

    def _sim_true(self) -> tuple | None:
        true_pose = getattr(self._driver, "true_pose", None)
        return tuple(true_pose()) if true_pose is not None else None

    def live_shutdown(self) -> None:
        if not self._live_ok:
            return
        try:
            self._save_now()
        except Exception:
            log.exception("Main map could not be saved on shutdown")
        if self._scan_odometry is not None:
            self._scan_odometry.stop("dauer")

    def live_status(self) -> dict[str, Any] | None:
        if self._mapper is None:
            return None
        status = self._mapper.status()
        status["name"] = self._live_name
        status["waechst"] = self._growing
        status["gespeichert"] = self._store.last_saved if self._store else None
        status["speicherfehler"] = self._store.last_error if self._store else None
        status["slam_karte"] = self._slam_result
        return status


def _compose(a, b) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def _inverse(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def _pose3(value) -> tuple[float, float, float] | None:
    try:
        x, y, theta = (float(v) for v in value[:3])
    except (TypeError, ValueError):
        return None
    return (x, y, theta) if all(math.isfinite(v) for v in (x, y, theta)) else None


def startup_stages(extra: dict) -> list[Stage]:
    """Where to look after a start: last pose, around it, home, the whole map.

    the robot usually starts where it was switched off
    """
    share = config.NAV_SEARCH_MIN_SHARE
    stages = []
    last = _pose3(extra.get("last_pose"))
    if last is not None:
        stages.append(Stage((last[0], last[1]), config.LIVE_START_SEARCH_NEAR_M, last[2], share,
                            text="sucht die eigene Lage dort, wo er zuletzt stand"))
        stages.append(Stage((last[0], last[1]), config.NAV_SEARCH_RADIUS_M, None, share,
                            text="sucht die eigene Lage rund um die letzte Stelle"))
    start = _pose3(extra.get("start"))
    if start is not None:
        stages.append(Stage((start[0], start[1]), config.NAV_SEARCH_RADIUS_M, start[2], share,
                            text="sucht die eigene Lage am Ausgangspunkt der Karte"))
    stages.append(Stage(None, 0.0, None, share, text="sucht die eigene Lage in der ganzen Karte"))
    return stages


def _build_map(payload) -> OccupancyMap:
    """Map including distance field -- compute the expensive part right here in the thread."""
    occ_map = OccupancyMap.from_payload(payload)
    _ = occ_map.gradient
    return occ_map
