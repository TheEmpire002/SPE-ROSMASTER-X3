"""WebSocket server -- the robot's only input and output.

App and ESP32 both connect to `/ws` and speak the protocol from
`protocol.py`. In addition the server serves the web app under `/`, so only
one address is needed in the browser.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import queue
import re
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    Response,
    StreamingResponse,
)
from fastapi.staticfiles import StaticFiles

from .. import config, settings
from ..recording import Recorder
from ..diagnostics import Diagnostics, LogBuffer
from ..drive.base import DriveBase
from ..audio.mukke import Mukke, MukkeError
from ..interaction import speech as sp
from ..interaction import behavior as vh
from ..interaction.speech import SpeechRecognition
from ..interaction.voice import Voice
from ..mapping.live_mapper import LiveMapper
from ..mapping.live_store import LiveStore
from ..mapping.manager import MappingManager
from ..mapping.web import create_mapping_router
from ..interaction.behavior import Interaction
from ..protocol import (
    DriveCommand,
    ProtocolError,
    alert_message,
    collision_message,
    error_message,
    limits_message,
    state_message,
    vitals_message,
)
from ..navigation import Explorer, ObstacleDetection, Homing, Localizer, Maneuver, Odometry
from ..navigation import maneuver as mv
from ..navigation.explorer import MAP
from ..navigation.map_localization import MapLocalization
from ..navigation.mapping_drive import MappingDrive
from ..navigation.obstacle_memory import ObstacleMemory
from ..navigation.path_planning import BackgroundPlanner
from ..navigation.scan_odometry import ScanOdometry
from .map_control import MapControl
from ..safety import CollisionGuard, DriveGuard, SafetyWatchdog
from ..sensors.camera import CameraService, create_cameras
from ..sensors.lidar import LidarService, create_lidar
from ..sensors.people import PersonDetection
from ..sensors.pose import SKELETON, PoseDetection
from ..sensors.depth import create_depth
from ..sensors import depth_view

log = logging.getLogger(__name__)

# robot/src/rosmaster_assist/api/server.py -> project root -> webapp/
WEBAPP_DIR = Path(__file__).resolve().parents[4] / "webapp"

# Make the browser revalidate the web app with the robot before every use
# (for an unchanged file just a short 304 response). Without this header the
# browser may keep reusing files at its own discretion -- after an update a
# new index.html and an old app.js then did not fit together, the script
# aborted, and the app stayed on "getrennt" (disconnected).
_NO_CACHE = {"Cache-Control": "no-cache"}


class _NoStaleCache(StaticFiles):
    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers.update(_NO_CACHE)
        return response


def _page(name: str) -> HTMLResponse:
    """An HTML page of the web app, with version numbers on scripts and styles.

    no-cache alone is not enough: a copy the browser still has from before
    this header was set is reused without asking. With ``?v=...`` every
    changed file is a new address for it -- an old app.js with a new
    index.html (script aborts, app stays on "getrennt") can then no longer
    happen.
    """
    def version(hits: re.Match) -> str:
        path = WEBAPP_DIR / hits.group(2)
        try:
            v = int(path.stat().st_mtime)
        except OSError:
            return hits.group(0)
        return f'{hits.group(1)}{hits.group(2)}?v={v}"'

    html = (WEBAPP_DIR / name).read_text(encoding="utf-8")
    html = re.sub(r'(/static/)([\w./-]+\.(?:js|css))"', version, html)
    return HTMLResponse(html, headers=_NO_CACHE)


class ConnectionHub:
    """Holds all open connections and distributes messages to them."""

    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def add(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.add(ws)
        log.info("Client connected (%d active)", len(self._clients))

    async def remove(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)
        log.info("Client disconnected (%d active)", len(self._clients))

    async def broadcast(self, message: dict[str, Any]) -> None:
        """Send to everyone. Dead or hanging connections are weeded out.

        In parallel and with a timeout: a phone with poor Wi-Fi used to hold
        up the status loop and thereby all other clients -- the app froze,
        and it looked like a crash.
        """
        async with self._lock:
            targets = list(self._clients)
        if not targets:
            return

        async def send_to(ws: WebSocket) -> WebSocket | None:
            try:
                await asyncio.wait_for(ws.send_json(message), config.WS_SEND_TIMEOUT_S)
                return None
            except Exception:
                return ws

        for ws in await asyncio.gather(*(send_to(ws) for ws in targets)):
            if ws is not None:
                await self.remove(ws)
                # Close it so that this client's receive loop ends too (and
                # the robot is stopped when there are zero clients).
                asyncio.create_task(_close(ws))

    @property
    def count(self) -> int:
        return len(self._clients)


async def _close(ws: WebSocket) -> None:
    try:
        await asyncio.wait_for(ws.close(), 2.0)
    except Exception:
        pass


_error_times: dict[str, float] = {}


def _loop_error(name: str) -> None:
    """Log an error in a long-running loop -- throttled -- and carry on.

    Only call inside an except block. The first error used to end the loop
    for good (e.g. no more status messages) without anyone noticing.
    """
    now = time.monotonic()
    if now - _error_times.get(name, -1e9) >= config.LOOP_ERROR_LOG_S:
        _error_times[name] = now
        log.exception("Error in %s -- continuing", name)


def create_app(driver: DriveBase, use_mock_camera: bool = False) -> FastAPI:
    """Builds the server application around a driver."""

    # First: some services read their values only when created (e.g. the
    # initial state of the obstacle stop).
    settings.load()

    watchdog = SafetyWatchdog(driver)
    odometry = Odometry(driver)
    hub = ConnectionHub()
    cameras: dict[str, CameraService] = (
        {name: CameraService(k) for name, k in create_cameras(use_mock_camera).items()}
        if config.CAMERA_ENABLED
        else {}
    )
    # The default camera answers on the endpoints without a camera name
    # (/camera/stream etc.) -- so clients that only know one camera keep
    # working unchanged.
    camera = cameras.get(config.CAMERA_DEFAULT) or next(iter(cameras.values()), None)

    # The simulated lidar needs the true position to scan the imaginary
    # room. Only the mock driver knows it -- the real hardware of course
    # does not, it simply measures.
    true_pose_source = getattr(driver, "true_pose", None)
    lidar = (
        LidarService(create_lidar(use_mock_camera, true_pose_source))
        if config.LIDAR_ENABLED
        else None
    )
    localizer = Localizer(odometry, lidar)
    # Lidar odometry for mapping, the mapping drive and localization in the
    # map: the heading comes from the lidar because the IMU on the X3 does
    # not deliver one. With always-on mapping it runs permanently (user
    # "immer"), otherwise only while mapping or while a map is loaded.
    scan_odometry = ScanOdometry(lidar, odometry)
    # Pose in the map (E12) and paths through it: for continuing while
    # mapping and for drives to a point on the map.
    localization = MapLocalization(scan_odometry)
    # always-on mapping (E14): the main map grows behind the integrate gate;
    # wheel travel only for lift detection, and only when really measured
    live_mapper = LiveMapper(
        wheel_pose_at=lambda t: odometry.pose_at(t) if odometry.actually_measuring else None
    )
    planner = BackgroundPlanner()
    homing = Homing(watchdog, odometry, localizer, localization=localization, planner=planner)
    # Depth camera (Astra) for what lies below the lidar plane: chair legs,
    # table edges. Its memory moves the points along with the robot's own
    # motion -- using the lidar odometry when it runs, otherwise the
    # odometry.
    depth = create_depth(use_mock_camera)

    def pose_source():
        so = scan_odometry
        if so.running:
            age = so.age()
            p = so.pose()
            if p is not None and age is not None and age < 1.0:
                return "lidar", p
        x, y, deg = odometry.pose()
        return "rad", (x, y, math.radians(deg))

    obstacles = ObstacleMemory(depth, pose_source)
    # Drive guard: checks every autonomous command against the robot's
    # footprint (in the watchdog, source="autonom") and manual control when
    # the obstacle stop is on. Without lidar it does not exist -- then only
    # homing (blind as before) and manual control drive.
    drive_guard = (
        DriveGuard(lidar, extra=obstacles) if lidar is not None else None
    )
    watchdog.drive_guard = drive_guard
    guard = CollisionGuard(localizer, drive_guard)
    detection = ObstacleDetection()
    # Person detection and pose detection (gestures, follow) both look
    # through the person camera (C270), see docs/interaction.md.
    person_camera = cameras.get(config.PERSON_CAMERA, camera)
    people = PersonDetection(person_camera)
    explorer = Explorer(
        watchdog, lidar, detection, people, odometry=odometry,
        mapping_drive=MappingDrive(scan_odometry, localization=localization, planner=planner),
        obstacles=obstacles, depth=depth,
    )
    maneuver = Maneuver(watchdog, driver, lidar)
    pose = PoseDetection(person_camera, mock=use_mock_camera)
    mukke = Mukke()
    # Voice and music share the speaker: while the robot speaks, the music
    # pauses and afterwards resumes at the same position.
    voice = Voice(
        mock=use_mock_camera, before_speaking=mukke.interrupt, after_speech=mukke.resume
    )
    # The driver is asked so that the interaction does not even start
    # driving without the board (and not by itself after reconnecting).
    interaction = Interaction(
        watchdog, lidar, pose, voice, drive_connected=lambda: driver.connected, odometry=odometry
    )
    # Gets the voice so it can stop listening while the robot itself speaks
    # (otherwise it would understand itself), and MUKKE: while music plays
    # only stop and "Musik aus" count, otherwise the singing turns into
    # commands.
    speech = SpeechRecognition(mock=use_mock_camera, voice=voice, mukke=mukke)

    # The diagnostics page also shows the server messages. An old buffer
    # (create_app called a second time, e.g. in tests) is replaced, otherwise
    # every line would arrive twice.
    package_log = logging.getLogger("rosmaster_assist")
    for h in [h for h in package_log.handlers if isinstance(h, LogBuffer)]:
        package_log.removeHandler(h)
    log_buffer = LogBuffer(config.DIAG_LOG_LINES)
    package_log.addHandler(log_buffer)
    diagnostics = Diagnostics(
        driver, watchdog, lidar, cameras, pose, log_buffer, mock=use_mock_camera
    )
    diagnostics.setup_crash_handling()

    # Recorder: keeps the last minute in memory, saving only happens at the
    # push of a button (see recording.py).
    recording = Recorder() if config.RECORDING_ACTIVE else None
    if recording is not None:
        recording.header = {
            "antrieb": driver.name, "mock": bool(use_mock_camera), "lidar": lidar is not None,
        }
        watchdog.recording = recording

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        try:
            driver.connect()
        except Exception:
            # Start anyway without the board: diagnostics and app should show
            # what is going on; the connection monitor keeps trying.
            log.exception("Driver could not be connected at startup -- will keep trying")
        if driver.name == "mock":
            log.warning("Driver: SIMULATION — the motors stay still")
        else:
            log.warning("Driver: %s — the robot can move", driver.name)
        watchdog.start()
        for camera in cameras.values():
            camera.start()
        people.start()
        pose.start()  # only computes once the interaction runs
        if depth is not None:
            depth.start()  # camera stays off until a drive needs it
        voice.start()
        # Audible self-test: whoever switches on the robot knows right away
        # whether the speaker works.
        voice.say("Ich bin bereit.")
        speech.start()  # loads the model in its own thread, does not block
        if lidar is not None:
            lidar.start()
            if config.LIVE_LOCALIZATION_ALWAYS:
                scan_odometry.start("immer")
            # Give the lidar one revolution, then calibrate the start.
            # Without a reference there would be no correction until the
            # first "set_home" -- and experience shows nobody presses the
            # button before something goes wrong.
            diagnostics.watch_task(
                asyncio.create_task(_catch_up_reference(localizer, odometry)),
                "Startpunkt einmessen", endless=False,
            )
        # main map from disk, the pose searched where the robot was switched off
        try:
            await asyncio.to_thread(map_control.live_startup)
        except Exception:
            log.exception("Main map could not be loaded -- always-on mapping stays off")
        diagnostics.start()
        asyncio.get_running_loop().set_exception_handler(diagnostics.loop_error)

        status_task = asyncio.create_task(
            _status_loop(
                hub, watchdog, driver, odometry, homing, localizer, guard,
                explorer, detection, lidar, people, interaction, voice, speech, maneuver,
                map_control=map_control,
            )
        )
        speech_task = asyncio.create_task(_speech_loop())
        odo_task = asyncio.create_task(_odometry_loop(watchdog, odometry))
        match_task = asyncio.create_task(_match_loop(localizer))
        hw_task = asyncio.create_task(
            _hardware_loop(hub, watchdog, driver, homing, explorer, interaction, maneuver,
                           map_control=map_control)
        )
        pulse_task = asyncio.create_task(_pulse_loop(diagnostics))
        map_task = asyncio.create_task(map_control.tick_loop(_loop_error))
        for task, name in [
            (status_task, "Statusmeldungen"), (speech_task, "Sprachbefehle"),
            (odo_task, "Positionsverfolgung"), (hw_task, "Verbindungsüberwachung"),
            (pulse_task, "Puls"), (map_task, "Ortung in der Karte"),
        ]:
            diagnostics.watch_task(task, name)
        rec_task = None
        if recording is not None:
            rec_task = asyncio.create_task(
                _recording_loop(
                    recording, watchdog, odometry, lidar, explorer, interaction, homing
                )
            )
            diagnostics.watch_task(rec_task, "Recorder")
        # Without lidar the matching ends by itself -- only report errors.
        diagnostics.watch_task(match_task, "Lidar-Abgleich", endless=False)
        try:
            yield
        finally:
            # First record that this is an intended shutdown: if cleanup
            # hangs and the process is then killed, that should not count
            # as a crash.
            diagnostics.mark_clean_exit()
            pulse_task.cancel()
            if rec_task is not None:
                rec_task.cancel()
            # Order matters: first end status messages, then the watchdog
            # (which stops the robot), then disconnect the hardware.
            status_task.cancel()
            odo_task.cancel()
            match_task.cancel()
            hw_task.cancel()
            speech_task.cancel()
            map_task.cancel()
            homing.cancel("Server wird beendet")
            explorer.cancel("Server wird beendet")
            maneuver.cancel("Server wird beendet")
            interaction.cancel("Server wird beendet")
            await watchdog.shutdown()
            # Only after stopping: shutting down the container can take a
            # while, and an error in it (Docker already gone, or not present
            # at all on the laptop) must not prevent disconnecting the
            # hardware.
            try:
                mapping_manager.stop()
            except Exception:
                log.exception("Mapping could not be stopped cleanly on shutdown")
            map_control.live_shutdown()
            localization.unload()
            scan_odometry.stop("immer")
            people.stop()
            pose.stop()
            mukke.stop()
            speech.stop()
            voice.stop()
            for camera in cameras.values():
                camera.stop()
            if depth is not None:
                depth.stop()
            if lidar is not None:
                lidar.stop()
            # Stop it before disconnecting, otherwise it would still report
            # the shutdown as "driver lost".
            diagnostics.stop()
            driver.disconnect()

    app = FastAPI(title="ROSMASTER X3 Hilfsroboter", lifespan=lifespan)
    # Other front ends should be able to call the HTTP endpoints from the
    # browser even if they are not served by this server (see docs/api.md).
    # There is no login anyway -- whoever is on the Wi-Fi can control the
    # robot.
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["GET"])

    # Gets the lidar and lidar odometry to feed them into ROS: slam_toolbox
    # in the container needs /scan and TF, and this server holds both. In
    # the simulation the map is built in Python.
    mapping_manager = MappingManager(
        lidar=lidar, scan_odometry=scan_odometry, localization=localization,
        simulation=lidar is not None and lidar.is_simulation,
    )
    app.state.mapping_manager = mapping_manager
    if config.SPEECH_OFF_WHILE_MAPPING:
        speech.set_pause(lambda: "Mapping läuft" if mapping_manager.active else None)

    # Mapping drive, loading a map, showing the pose, goal drive -- for /map
    # and the app. The mapping drive is a mode of the explorer; that way
    # emergency stop, manual intervention and connection loss apply without
    # extra code paths.
    map_control = MapControl(
        mapping_manager=mapping_manager, localization=localization, explorer=explorer, homing=homing,
        interaction=interaction, maneuver=maneuver, watchdog=watchdog, driver=driver, hub=hub,
        live_mapper=live_mapper, live_store=LiveStore(mapping_manager.map_service),
        scan_odometry=scan_odometry,
    )
    app.state.map_control = map_control
    app.include_router(create_mapping_router(mapping_manager, map_control))

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        await hub.add(ws)
        # Limits first: the app scales its slider positions to them and
        # therefore has to know them before the first command goes out.
        await ws.send_json(
            limits_message(config.MAX_SPEED_X, config.MAX_SPEED_Y, config.MAX_SPEED_OMEGA)
        )
        # The server holds the setting, not the browser: otherwise every tab
        # would have its own idea of what currently applies.
        await ws.send_json(collision_message(guard.state()))
        try:
            while True:
                msg = await ws.receive_json()
                try:
                    reply = await _handle(
                        msg, watchdog, driver, hub, odometry, homing, localizer, guard,
                        explorer, interaction, voice, speech, recording,
                        maneuver=maneuver, sender=ws, map_control=map_control,
                    )
                except Exception:
                    # A faulty command used to drop the connection -- the
                    # robot stopped, and it looked like a crash.
                    kind = msg.get("type") if isinstance(msg, dict) else None
                    log.exception("Command %r failed", kind)
                    reply = alert_message("warning", f"Befehl {kind!r} fehlgeschlagen (siehe /diagnose)")
                if reply is not None:
                    await ws.send_json(reply)
        except WebSocketDisconnect:
            pass
        except Exception:
            log.exception("Error in the connection")
        finally:
            await hub.remove(ws)
            # Last client gone? Then nobody is in control any more.
            if hub.count == 0:
                # Exception: mapping drive and map-based drives. They run for
                # a long time and are often watched from /map; if that is
                # closed too, they only stop after a deadline -- a phone whose
                # screen turns off should not end them (MapControl).
                with_map = explorer.mode == MAP or homing.by_map
                log.warning("No client connected any more -- stopping"
                            + (" (map-based drive only after a deadline)" if with_map else ""))
                if not with_map:
                    homing.cancel("Verbindung verloren")
                    explorer.cancel("Verbindung verloren")
                maneuver.cancel("Verbindung verloren")
                interaction.cancel("Verbindung verloren")
                if with_map:
                    map_control.check_watchers("Verbindung verloren")
                else:
                    watchdog.stop()

    # --- Camera ---
    #
    # MJPEG: a never-ending HTTP response of concatenated images. The
    # browser shows it directly in an <img> -- without JavaScript and
    # without an extra library.

    BOUNDARY = "frame"

    def _stream(camera: CameraService | None):
        if camera is None:
            return Response("Kamera abgeschaltet", status_code=503)

        async def images():
            last = None
            while True:
                frame = camera.latest()
                # Do not send the same image twice: with a slow camera this
                # saves noticeable bandwidth.
                if frame is not None and frame is not last:
                    data, mime = frame
                    last = frame
                    yield (
                        f"--{BOUNDARY}\r\nContent-Type: {mime}\r\n"
                        f"Content-Length: {len(data)}\r\n\r\n".encode()
                        + data
                        + b"\r\n"
                    )
                # Check twice as often as images arrive: otherwise every new
                # image sits around for half a period on average. Read anew
                # each time because the frame rate is adjustable.
                await asyncio.sleep(0.5 / max(1, config.CAMERA_FPS))

        return StreamingResponse(
            images(),
            media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
            # Without this some proxies buffer the stream until nothing
            # arrives any more -- the image would then simply stay black.
            headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"},
        )

    def _snapshot(camera: CameraService | None):
        if camera is None:
            return Response("Kamera abgeschaltet", status_code=503)
        frame = camera.latest()
        if frame is None:
            return Response("Noch kein Bild", status_code=503)
        data, mime = frame
        return Response(data, media_type=mime, headers={"Cache-Control": "no-store"})

    def _unknown(name: str):
        return Response(
            f"Keine Kamera {name!r} -- vorhanden: {', '.join(cameras) or 'keine'}",
            status_code=404,
        )

    # Without a camera name: the default camera (config.CAMERA_DEFAULT).

    @app.get("/camera/stream")
    async def camera_stream():
        return _stream(camera)

    @app.get("/camera/status")
    async def camera_status():
        """Why an image arrives -- or why not.

        Callable from the browser, so the question can be answered without
        searching the log on the robot.
        """
        if camera is None:
            return {"kamera": None, "hinweis": "CAMERA_ENABLED ist False"}
        return camera.status()

    @app.get("/camera/snapshot")
    async def camera_snapshot():
        """A single image -- useful for testing with curl."""
        return _snapshot(camera)

    # With a camera name: each camera individually.

    @app.get("/cameras")
    async def cameras_list():
        """All cameras with role, state and the addresses of their images."""
        return {
            "standard": camera.name if camera is not None else None,
            "personen": people.status().get("kamera"),
            # Views of the depth camera -- a separate list because they are not
            # cameras with role and settings (the settings page lists "kameras").
            "tiefe": [
                {"name": "tiefe", "kamera": "Tiefe", "basis": "/tiefe"},
                {"name": "tiefe-ueber", "kamera": "Tiefe + Farbbild", "basis": "/tiefe/ueberlagert"},
            ] if depth is not None else [],
            "kameras": [
                {
                    **k.status(),
                    "stream": f"/camera/{name}/stream",
                    "snapshot": f"/camera/{name}/snapshot",
                }
                for name, k in cameras.items()
            ],
        }

    @app.get("/camera/{name}/stream")
    async def camera_stream_name(name: str):
        if name not in cameras:
            return _unknown(name)
        return _stream(cameras[name])

    @app.get("/camera/{name}/snapshot")
    async def camera_snapshot_name(name: str):
        if name not in cameras:
            return _unknown(name)
        return _snapshot(cameras[name])

    @app.get("/camera/{name}/status")
    async def camera_status_name(name: str):
        if name not in cameras:
            return _unknown(name)
        return cameras[name].status()

    # --- Lidar ---

    @app.get("/lidar/status")
    async def lidar_status():
        """What the lidar is doing and whether localization works with it."""
        if lidar is None:
            return {"lidar": None, "hinweis": "LIDAR_ENABLED ist False"}
        return {**lidar.status(), "ortung": localizer.status()}

    # --- Depth camera ---

    @app.get("/tiefe/status")
    async def depth_status():
        """Depth camera and obstacle memory (chair legs, phantoms)."""
        return {
            "kamera": depth.status() if depth is not None else None,
            "hinweis": None if depth is not None else (
                "Simulation" if use_mock_camera else
                "aus (DEPTH_ACTIVE) oder OpenNI2 fehlt, siehe scripts/setup_orbbec.sh"
            ),
            "gedaechtnis": obstacles.status(),
            "fortschritt_aktiv": config.PROGRESS_ACTIVE,
        }

    @app.post("/tiefe/test")
    async def depth_test():
        """Switch the camera on for 20 s -- to check floor detection and mounting
        position while standing. Otherwise it only runs during autonomous drives."""
        if depth is None:
            return {"ok": False, "hinweis": "keine Tiefenkamera"}
        depth.activate("test", True)
        asyncio.get_running_loop().call_later(20.0, depth.activate, "test", False)
        return {"ok": True, "hinweis": "an für 20 s, dann /tiefe/status abfragen"}

    # Depth image for viewing (sensors/depth_view.py): alone or over the
    # Orbbec's color image. Whoever watches switches the camera on.

    def _depth_jpeg(image, overlay: bool) -> bytes:
        background = fov = None
        if overlay:
            color_camera = cameras.get(config.DEPTH_OVERLAY_CAMERA)
            frame = color_camera.latest() if color_camera is not None else None
            background = frame[0] if frame is not None else None
            fov = config.CAMERA_ROLES.get(config.DEPTH_OVERLAY_CAMERA, {}).get("fov_deg")
        return depth_view.draw(image, depth.hfov, background, fov)

    def _depth_view_missing():
        if depth is None:
            return Response("Keine Tiefenkamera (siehe /tiefe/status)", status_code=503)
        try:
            import cv2  # noqa: F401
        except ImportError:
            return Response("OpenCV fehlt -- ohne lässt sich das Tiefenbild nicht zeichnen",
                            status_code=503)
        return None

    watcher_ids = iter(range(1, 1_000_000_000))

    def _depth_stream(overlay: bool):
        missing = _depth_view_missing()
        if missing is not None:
            return missing
        # Every watcher subscribes individually: if one closes the image,
        # the camera stays on for the others.
        who = f"anzeige-{next(watcher_ids)}"

        async def images():
            depth.activate(who, True)
            last = None
            try:
                while True:
                    b = depth.image()
                    if b is not None and b.raw is not None and b.timestamp != last:
                        last = b.timestamp
                        data = await asyncio.to_thread(_depth_jpeg, b, overlay)
                        yield (
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(data)}\r\n\r\n".encode() + data + b"\r\n"
                        )
                    await asyncio.sleep(0.05)
            finally:
                depth.activate(who, False)

        return StreamingResponse(
            images(),
            media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
            headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"},
        )

    snapshot_off_timers: list = []  # timer that switches the camera off after snapshots

    async def _depth_snapshot(overlay: bool):
        missing = _depth_view_missing()
        if missing is not None:
            return missing
        # Snapshots (Safari) come in quick succession: the camera stays on
        # until none has been fetched for 10 s.
        depth.activate("anzeige-einzelbild", True)
        for old in snapshot_off_timers:
            old.cancel()
        snapshot_off_timers[:] = [asyncio.get_running_loop().call_later(
            10.0, depth.activate, "anzeige-einzelbild", False)]
        deadline = time.monotonic() + 3.0  # switching on takes about 1-2 s
        b = depth.image()
        while (b is None or b.raw is None) and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            b = depth.image()
        if b is None or b.raw is None:
            return Response("Noch kein Tiefenbild", status_code=503)
        data = await asyncio.to_thread(_depth_jpeg, b, overlay)
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/tiefe/stream")
    async def depth_stream():
        """Depth image as MJPEG: red near, blue far, magenta obstacle."""
        return _depth_stream(False)

    @app.get("/tiefe/snapshot")
    async def depth_snapshot():
        return await _depth_snapshot(False)

    @app.get("/tiefe/ueberlagert/stream")
    async def depth_overlay_stream():
        """The same, semi-transparent over the Orbbec's color image."""
        return _depth_stream(True)

    @app.get("/tiefe/ueberlagert/snapshot")
    async def depth_overlay_snapshot():
        return await _depth_snapshot(True)

    @app.get("/lidar/scan")
    async def lidar_scan():
        """The last revolution as raw data -- for display in the app."""
        if lidar is None:
            return {"punkte": [], "hinweis": "kein Lidar"}
        scan = lidar.latest()
        if scan is None or time.monotonic() - scan.timestamp > 3.0:
            # Tell the app why nothing arrives -- "no data" alone leaves one
            # guessing whether the lidar, the app or the Wi-Fi is at fault.
            st = lidar.status()
            reason = st["letzter_fehler"] or (
                "wartet auf erste Umdrehung" if st["geoeffnet"] else "nicht geöffnet"
            )
            return {"punkte": [], "hinweis": f"Lidar: {reason}"}
        # Transmit rounded: two decimal places in meters are one
        # centimeter, the display needs nothing finer -- and it halves the
        # amount of data.
        return {
            "punkte": [
                [round(w, 1), round(r, 2)] for w, r in zip(scan.angle, scan.ranges)
            ],
            "alter_s": round(time.monotonic() - scan.timestamp, 2),
        }

    # --- Autonomous driving ---

    @app.get("/autonomie/status")
    async def autonomy_status():
        """Explorer, detected obstacles and person detection at a glance."""
        return _autonomy(explorer, detection, lidar, people)

    # --- Settings ---
    #
    # Read via GET like everything else; change via POST because it changes
    # state on the robot (and is saved). The CORS setup above only allows
    # GET -- so only someone who loaded the page from the robot itself can
    # change settings, not any arbitrary web page in the browser.

    async def _apply_settings(changed: list[str]) -> None:
        """Trigger whatever does not take effect by itself on next use."""
        if any(k.startswith("MAX_SPEED_") for k in changed):
            # The app scales its sliders to the limits.
            await hub.broadcast(
                limits_message(config.MAX_SPEED_X, config.MAX_SPEED_Y, config.MAX_SPEED_OMEGA)
            )
        if "COLLISION_ENABLED" in changed or "COLLISION_DISTANCE_M" in changed:
            # The guard holds its own state (the switch in the app changes
            # it without config) -- so pass it explicitly.
            guard.configure(
                config.COLLISION_ENABLED if "COLLISION_ENABLED" in changed else None,
                config.COLLISION_DISTANCE_M if "COLLISION_DISTANCE_M" in changed else None,
            )
            await hub.broadcast(collision_message(guard.state()))
        if "CAMERA_RESOLUTION" in changed:
            for camera in cameras.values():
                camera.reopen()
        if "VOLUME" in changed:
            # The voice reads the value for every sentence, mpv has to be told.
            await asyncio.to_thread(mukke.apply_volume)

    async def _settings_reply(changed: list[str]) -> dict[str, Any]:
        await _apply_settings(changed)
        restart = settings.restart_needed(changed)
        if changed:
            text = f"Einstellungen geändert: {', '.join(changed)}"
            if restart:
                text += " (wirkt nach Neustart)"
            await hub.broadcast(alert_message("info", text))
        return {**settings.entries(), "geaendert": changed, "neustart_noetig": restart}

    @app.get("/settings")
    async def settings_get():
        """All adjustable settings by block, with limits."""
        return settings.entries()

    @app.post("/settings")
    async def settings_set(request: Request):
        """Body: {"KEY": value, ...}. All or none are applied."""
        try:
            new = await request.json()
        except ValueError:
            return JSONResponse({"fehler": "JSON erwartet"}, status_code=400)
        if not isinstance(new, dict):
            return JSONResponse({"fehler": "Objekt {KEY: wert} erwartet"}, status_code=400)
        try:
            changed = settings.apply(new)
        except settings.SettingError as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await _settings_reply(changed)

    @app.post("/settings/reset")
    async def settings_reset(request: Request):
        """Body: {"keys": [...]} for individual ones, empty or {} for all."""
        try:
            data = await request.json()
        except ValueError:
            data = {}
        keys = data.get("keys") if isinstance(data, dict) else None
        try:
            changed = settings.reset(keys)
        except settings.SettingError as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await _settings_reply(changed)

    # --- Interaction ---

    @app.get("/interaktion/status")
    async def interaction_status():
        """Mode, tracked person, last gesture and whether pose detection runs."""
        return {**interaction.state(), "pose": pose.status()}

    @app.get("/manoever/status")
    async def maneuver_status():
        """Exact drive command: what runs, how far along it is per lidar, how it ended."""
        return {"verfuegbar": maneuver.available, **maneuver.state()}

    @app.get("/interaktion/pose")
    async def interaction_pose():
        """Detected body keypoints for overlaying on the camera image.

        Empty while the interaction is off -- then the Hailo does not compute.
        """
        # The appearance feature (color histogram) stays in the robot -- the
        # app does not need it, and it bloats every response. Added are the
        # track ID (internal only) and whether it is following this person.
        people = interaction.people_with_id(pose.people())
        return {"personen": people, "skelett": SKELETON}

    # --- Recorder ---

    @app.get("/aufzeichnung/status")
    async def recording_status():
        """Buffer fill level and last saved file."""
        if recording is None:
            return {"aktiv": False}
        return recording.status()

    @app.get("/aufzeichnung/liste")
    async def recording_list():
        """Saved drives, newest first."""
        if recording is None:
            return {"dateien": []}
        return {"dateien": await asyncio.to_thread(recording.entries)}

    @app.get("/aufzeichnung/datei/{name}")
    async def recording_file(name: str):
        """Download a saved drive (for replay on a computer)."""
        path = recording.file(name) if recording is not None else None
        if path is None:
            return JSONResponse({"fehler": "keine solche Aufzeichnung"}, status_code=404)
        data = await asyncio.to_thread(path.read_bytes)
        return Response(
            data,
            media_type="application/gzip",
            headers={"Content-Disposition": f'attachment; filename="{path.name}"'},
        )

    # --- Diagnostics ---

    @app.get("/diagnose/status")
    async def diagnostics_status():
        """Overall state, problems with suggested fixes, readings, timeline."""
        return diagnostics.status()

    @app.get("/diagnose/protokoll")
    async def diagnostics_log(after: int = Query(0, alias="nach")):
        """Server log after line ``nach`` (exclusive), so only new lines come."""
        lines, last_id = log_buffer.lines(after)
        return {"zeilen": lines, "letzte_id": last_id}

    @app.post("/diagnose/quittieren")
    async def diagnostics_acknowledge(request: Request):
        """Body: {"id": "..."} for one problem or {"alle": true}."""
        try:
            data = await request.json()
        except ValueError:
            data = None
        if not isinstance(data, dict) or not (data.get("alle") or data.get("id")):
            return JSONResponse({"fehler": '{"id": ...} oder {"alle": true} erwartet'},
                                status_code=400)
        n = diagnostics.acknowledge(None if data.get("alle") else str(data["id"]))
        return {"quittiert": n}

    # --- Music ("MUKKE") ---
    #
    # HTTP like the settings: changes via POST, so that foreign web pages
    # (CORS only allows them GET) cannot play anything on the speaker.

    async def _mukke_json(request: Request) -> dict:
        try:
            data = await request.json()
        except ValueError:
            return {}
        return data if isinstance(data, dict) else {}

    async def _mukke(action) -> Any:
        """Run the action; errors as 400, otherwise the new state."""
        try:
            # Starting mpv and querying it over its socket takes a moment --
            # not in the event loop, otherwise control would stutter.
            await asyncio.to_thread(action)
        except MukkeError as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await asyncio.to_thread(mukke.status)

    @app.get("/mukke/status")
    async def mukke_status():
        """Whether something plays, which title, volume, and whether mpv/yt-dlp exist."""
        return await asyncio.to_thread(mukke.status)

    @app.post("/mukke/play")
    async def mukke_play(request: Request):
        """Body: {"url": "https://youtu.be/..."}. Replaces whatever is playing."""
        url = (await _mukke_json(request)).get("url", "")
        response = await _mukke(lambda: mukke.play(url))
        if not isinstance(response, JSONResponse):
            await hub.broadcast(alert_message("info", "MUKKE: spielt ab"))
        return response

    @app.post("/mukke/stop")
    async def mukke_stop():
        return await _mukke(mukke.stop)

    @app.post("/mukke/pause")
    async def mukke_pause():
        """Toggle pause."""
        return await _mukke(mukke.toggle_pause)

    @app.post("/mukke/volume")
    async def mukke_volume(request: Request):
        """Body: {"lautstaerke": 0..100}."""
        value = (await _mukke_json(request)).get("lautstaerke")
        return await _mukke(lambda: mukke.set_volume(value))

    # --- Voice ---

    @app.get("/stimme/status")
    async def voice_status():
        """Which speech output runs (Piper, espeak-ng, text only), last sentence, errors."""
        return voice.status()

    # --- Speech commands ---

    @app.get("/sprache/status")
    async def speech_status():
        """Microphone, model, wake word, last heard text and recognized commands."""
        return speech.status()

    @app.get("/sprache/live")
    async def speech_live():
        """Live view: level, partial text and the last sentences with verdict."""
        return speech.live()

    @app.websocket("/sprache/ton")
    async def speech_sound(ws: WebSocket) -> None:
        """Listen in: raw audio from the microphone as binary messages (16 bit, mono).

        Before that and on every change of sample rate a text message
        {"rate": Hz} is sent; without audio also every second as a heartbeat.
        Audio is only read while someone is connected.
        """
        await ws.accept()
        subscription = speech.sound_subscribe()
        rate: int | None = -1  # -1: nothing reported yet, the first report always goes out
        try:
            while True:
                if speech.sound_rate != rate:
                    rate = speech.sound_rate
                    await ws.send_json({"rate": rate})
                try:
                    # Blocking wait in a thread so the event loop stays free.
                    data = await asyncio.to_thread(subscription.get, True, 1.0)
                except queue.Empty:
                    # Microphone currently gone (USB) or simulation. Still send
                    # a heartbeat: that is how the app knows the connection is
                    # up; otherwise it reconnects by itself.
                    await ws.send_json({"rate": rate})
                    continue
                await ws.send_bytes(data)
        except (WebSocketDisconnect, RuntimeError):
            pass  # browser closed or page left
        finally:
            speech.sound_unsubscribe(subscription)

    @app.websocket("/sprache/durchsage")
    async def speech_announcement(ws: WebSocket) -> None:
        """Announcement: audio from the app live over the robot's speaker.

        First a text message {"rate": Hz}, then raw audio as binary messages
        (16 bit little-endian, mono). Reply {"bereit": bool, "grund": ...}.
        """
        await ws.accept()
        rate = 16000
        announcement = None
        retry_at: float | None = None  # after a speaker error, only from this time on
        ready = voice is not None and voice.available
        await ws.send_json({"bereit": ready, "grund": None if ready else "kein Lautsprecher"})
        try:
            while True:
                message = await ws.receive()
                if message["type"] == "websocket.disconnect":
                    break
                if message.get("text"):
                    try:
                        rate = max(8000, min(48000, int(json.loads(message["text"]).get("rate", rate))))
                    except (ValueError, TypeError, AttributeError):
                        pass
                    continue
                data = message.get("bytes")
                if not data or not ready:
                    continue
                # After a pause in speaking or a stop the old announcement is
                # over; then simply begin a new one. If the speaker failed, only
                # retry after a pause -- otherwise every audio packet would
                # start a new aplay.
                if announcement is None or announcement.finished:
                    if announcement is not None and announcement.failed:
                        if retry_at is None:
                            retry_at = time.monotonic() + config.ANNOUNCE_PAUSE_S
                            await ws.send_json({"bereit": False, "grund": "Lautsprecher geht gerade nicht"})
                        if time.monotonic() < retry_at:
                            continue
                    retry_at = None
                    announcement = voice.announcement(rate)
                    await ws.send_json({"bereit": True, "grund": None})
                announcement.feed(data)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if announcement is not None:
                announcement.end()

    async def _speech_loop() -> None:
        """Fetch and execute recognized speech commands.

        Recognition runs in its own thread; driving, however, only happens
        here in the event loop -- along the same paths as the app's buttons.
        """
        answered_at: float | None = None
        while True:
            await asyncio.sleep(0.1)
            if answered_at is not None and time.monotonic() >= answered_at:
                answered_at = None
                # If the command already came during the pause, the window is
                # consumed and the "Ja?" is unnecessary.
                if speech.awake and config.SPEECH_WAKE_REPLY:
                    voice.say(config.SPEECH_WAKE_REPLY)
            for entry in speech.fetch_commands():
                if entry["befehl"] == sp.WAKE:
                    answered_at = time.monotonic() + config.SPEECH_WAKE_REPLY_AFTER_S
                    continue
                try:
                    await _speech_command(entry["befehl"], entry["text"], entry.get("wie"))
                except Exception:  # a broken command must not switch off the ear
                    log.exception("Speech command %s failed", entry["befehl"])

    async def _issue_command(kind: str, **fields: Any) -> bool:
        """Execute an app command. False if it was refused with a warning."""
        response = await _handle(
            {"type": kind, **fields}, watchdog, driver, hub, odometry, homing, localizer, guard,
            explorer, interaction, voice, speech, maneuver=maneuver, map_control=map_control,
        )
        if response is None:
            return True
        await hub.broadcast(response)
        if response.get("level") == "warning":
            # Whoever speaks is not necessarily looking at the app.
            voice.say("Das geht gerade nicht.")
            return False
        return True

    async def _interaction_with(mode: str, text: str, sentence: str) -> None:
        """Come here/go away by speech; switches the interaction on if needed."""
        if interaction.active:
            interaction.switch_to(mode, text, say=sentence)
            return
        if not interaction.available:
            await _issue_command("interaction_start")  # yields the matching warning
            return
        homing.cancel("Interaktion gestartet")
        explorer.cancel("Interaktion gestartet")
        maneuver.cancel("Interaktion gestartet")
        if interaction.start(mode):
            await hub.broadcast(alert_message("info", f"Interaktion an -- {text}"))

    async def _speech_command(command: str, text: str, details: dict | None = None) -> None:
        await hub.broadcast(alert_message("info", f"Sprachbefehl: {text}"))
        if command == sp.STOP:
            if interaction.active:
                # Stop, but keep listening and watching: "Robi, folgen"
                # should work again right afterwards.
                homing.cancel("Not-Stopp")
                explorer.cancel("Not-Stopp")
                maneuver.cancel("Not-Stopp")
                interaction.switch_to(vh.READY, "angehalten (per Sprache)")
                watchdog.stop()
                voice.say("Okay, ich halte an.", priority=True)
            else:
                await _issue_command("stop")
        elif command == sp.FOLLOW:
            await _issue_command("follow")
        elif command == sp.COME_HERE:
            await _interaction_with(vh.COME_HERE, "kommt her (per Sprache)", "Ich komme.")
        elif command == sp.GO_AWAY:
            await _interaction_with(vh.GO_AWAY, "nimmt Abstand (per Sprache)", "Ich fahre zurück.")
        elif command == sp.START:
            if interaction.active:
                voice.say("Ja? Ich bin da.")
            else:
                await _issue_command("interaction_start")
        elif command == sp.MANEUVER and details:
            # "Robi, fahr zwei Meter vor" / "Robi, dreh dich um neunzig Grad"
            kind = "turn" if details["art"] == mv.TURN else "move"
            fields = {"grad": details["wert"]} if kind == "turn" else {"richtung": details["art"], "meter": details["wert"]}
            if await _issue_command(kind, **fields) and maneuver.active:
                if kind == "turn":
                    voice.say(f"Ich drehe mich um {abs(details['wert']):g} Grad "
                                 f"{'nach links' if details['wert'] > 0 else 'nach rechts'}.")
                else:
                    voice.say(f"Ich fahre {_say_meters(details['wert'])} "
                                 f"{mv._DIRECTION_TEXT[details['art']]}.")
        elif command == sp.OFF:
            homing.cancel("vom Nutzer beendet")
            explorer.cancel("vom Nutzer beendet")
            maneuver.cancel("vom Nutzer beendet")
            await _issue_command("interaction_stop")
            voice.say("Gute Nacht.")
        elif command == sp.GO_HOME:
            if await _issue_command("home") and homing.active:
                voice.say("Ich fahre nach Hause.")
        elif command == sp.EXPLORE:
            if await _issue_command("explore") and explorer.active:
                voice.say("Ich schaue mich mal um.")
        elif command == sp.MUSIC_OFF:
            try:
                await asyncio.to_thread(mukke.stop)
            except MukkeError as exc:
                await hub.broadcast(alert_message("warning", str(exc)))
            voice.say("Musik ist aus.")
        elif command in (sp.LADADA, sp.BEATBOX):
            # Look it up only at command time: that way a changed config applies.
            url = {
                sp.LADADA: config.SPEECH_LADADA_URL,
                sp.BEATBOX: config.SPEECH_BEATBOX_URL,
            }[command]
            # Do not announce anything: the voice would interrupt the song
            # right away.
            playing = await asyncio.to_thread(mukke.status)
            if playing["laeuft"] and playing["url"] == url:
                # The microphone hears the song too -- if it sings "la la la"
                # itself, it would otherwise start over again and again.
                return
            try:
                await asyncio.to_thread(mukke.play, url)
            except MukkeError as exc:
                await hub.broadcast(alert_message("warning", str(exc)))
                voice.say("Das geht gerade nicht.")

    # Serve the web app directly so one address is enough.
    if WEBAPP_DIR.is_dir():
        app.mount("/static", _NoStaleCache(directory=WEBAPP_DIR), name="static")

        @app.get("/")
        async def index() -> HTMLResponse:
            return _page("index.html")

        @app.get("/einstellungen")
        async def settings_page() -> HTMLResponse:
            return _page("einstellungen.html")

        # /logs as a second name -- that is what people look for when something goes wrong.
        @app.get("/diagnose")
        @app.get("/logs")
        async def diagnostics_page() -> HTMLResponse:
            return _page("diagnose.html")
    else:
        log.warning("webapp/ not found at %s -- only /ws available", WEBAPP_DIR)

    return app


# Which connection last actually steered, and when (monotonic).
_pilot: dict[str, Any] = {"wer": None, "zeit": 0.0}


def _say_meters(meter: float) -> str:
    """Distance the way it is spoken ("einen halben Meter", "30 Zentimeter")."""
    if abs(meter - 1.0) < 1e-6:
        return "einen Meter"
    if abs(meter - 0.5) < 1e-6:
        return "einen halben Meter"
    if meter < 1.0:
        return f"{round(meter * 100)} Zentimeter"
    return f"{meter:g} Meter".replace(".", ",")


async def _handle(
    msg: dict[str, Any],
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    hub: ConnectionHub,
    odometry: Odometry,
    homing: Homing,
    localizer: Localizer,
    guard: CollisionGuard,
    explorer: Explorer | None = None,
    interaction: Interaction | None = None,
    voice: Voice | None = None,
    speech: SpeechRecognition | None = None,
    recording: Recorder | None = None,
    maneuver: Maneuver | None = None,
    sender: object | None = None,
    map_control: MapControl | None = None,
) -> dict[str, Any] | None:
    """Process an incoming message.

    Returns the reply to the sender, or None if none is needed.
    ``sender`` is the connection it came from (for drive priority).
    """
    kind = msg.get("type")

    if kind == "drive":
        try:
            cmd = DriveCommand.from_message(msg)
        except ProtocolError as exc:
            return error_message(str(exc))

        # Drive priority: whoever is steering is not slowed down by the idle
        # zeros of a second open app (see DRIVE_PRIORITY_S).
        if sender is not None:
            now = time.monotonic()
            if not cmd.is_stop():
                _pilot["wer"], _pilot["zeit"] = sender, now
            elif (
                _pilot["wer"] not in (None, sender)
                and now - _pilot["zeit"] < config.DRIVE_PRIORITY_S
            ):
                return None

        # The human has priority. The app keeps sending even at standstill,
        # so only an actual input cancels -- otherwise homing would end
        # immediately.
        if homing.active and not cmd.is_stop():
            homing.cancel("von Hand übersteuert")
            await hub.broadcast(alert_message("info", "Heimfahrt abgebrochen"))
        elif homing.active:
            return None  # ignore idle messages, homing is steering

        # Same for autonomous driving: intervening ends it, the app's idle
        # zeros must not slow it down.
        if explorer is not None and explorer.active:
            if cmd.is_stop():
                return None
            # _status_loop sends the message with the reason.
            explorer.cancel("von Hand übersteuert")

        # Interaction too: whoever steers by hand does not want the robot
        # to follow a person at the same time.
        if interaction is not None and interaction.active:
            if cmd.is_stop():
                return None
            interaction.cancel("von Hand übersteuert")

        # Exact drive command ("2 Meter vor") likewise.
        if maneuver is not None and maneuver.active:
            if cmd.is_stop():
                return None
            maneuver.cancel("von Hand übersteuert")

        allowed, obstacle = guard.check(cmd)
        if obstacle is not None and guard.report():
            await hub.broadcast(
                alert_message(
                    "warning", f"Hindernis in {obstacle:.2f} m -- Fahrt gesperrt"
                )
            )
        watchdog.submit(allowed)
        return None  # status goes out on the tick anyway

    if kind == "collision":
        guard.configure(msg.get("enabled"), msg.get("distance"))
        await hub.broadcast(collision_message(guard.state()))
        return None

    if kind == "stop":
        homing.cancel("Not-Stopp")
        if map_control is not None:
            # Also discard a pending resume after a USB reset.
            map_control.stop("Not-Stopp")
        if explorer is not None:
            explorer.cancel("Not-Stopp")
        if interaction is not None:
            interaction.cancel("Not-Stopp")
        if maneuver is not None:
            maneuver.cancel("Not-Stopp")
        # no spoken "Halt": the app also sends stop when it is left, and
        # hearing that every time is annoying
        watchdog.stop()
        return None

    if kind in ("move", "turn"):
        if maneuver is None:
            return alert_message("warning", "Genaue Fahrbefehle gehen hier nicht")
        if kind == "turn":
            category, value = mv.TURN, msg.get("grad")
        else:
            category, value = str(msg.get("richtung", mv.FORWARD)), msg.get("meter")
        # Only one program steers: homing, explorer and interaction give way.
        homing.cancel("Fahrbefehl gestartet")
        if explorer is not None:
            explorer.cancel("Fahrbefehl gestartet")
        if interaction is not None:
            interaction.cancel("Fahrbefehl gestartet")
        error = maneuver.start(category, value)
        if error is not None:
            return alert_message("warning", error)
        await hub.broadcast(alert_message("info", f"Fahrbefehl: {mv.describe(category, float(value))}"))
        return None

    if kind == "explore":
        if explorer is None or not explorer.available:
            return alert_message("warning", "Autonomes Fahren geht nur mit Lidar")
        if explorer.active:
            return None
        homing.cancel("autonomes Fahren gestartet")
        if interaction is not None:
            interaction.cancel("autonomes Fahren gestartet")
        if maneuver is not None:
            maneuver.cancel("autonomes Fahren gestartet")
        if explorer.start():
            await hub.broadcast(alert_message("info", "Autonomes Fahren gestartet"))
        return None

    if kind == "explore_stop":
        if explorer is not None:
            explorer.cancel("vom Nutzer beendet")  # message comes from _status_loop
        watchdog.stop()
        return None

    if kind in ("interaction_start", "follow"):
        if interaction is None or not interaction.available:
            return alert_message(
                "warning",
                "Interaktion geht nur mit Lidar und Posenerkennung (siehe /interaktion/status)",
            )
        if not interaction.active:
            # Only one program steers: explorer and homing give way.
            homing.cancel("Interaktion gestartet")
            if explorer is not None:
                explorer.cancel("Interaktion gestartet")
            if maneuver is not None:
                maneuver.cancel("Interaktion gestartet")
        if kind == "follow":
            was_active = interaction.active
            target_id = msg.get("ziel_id")
            if isinstance(target_id, bool) or not isinstance(target_id, (int, type(None))):
                return alert_message("warning", "ziel_id muss eine Zahl sein")
            if interaction.follow(target_id) and not was_active:
                await hub.broadcast(alert_message("info", "Interaktion an -- folge dir"))
        elif interaction.start():
            await hub.broadcast(
                alert_message("info", "Interaktion an -- ich reagiere auf Handzeichen")
            )
        return None

    if kind == "follow_stop":
        if interaction is not None:
            interaction.follow_stop()
        return None

    if kind in ("follow_pause", "follow_resume"):
        # Pause stops, the target stays remembered -- unlike follow_stop.
        if interaction is not None:
            if kind == "follow_pause":
                interaction.pause()
            else:
                interaction.resume()
        return None

    if kind == "record_save":
        if recording is None:
            return alert_message("warning", "Fahrtenschreiber ist ausgeschaltet")
        note = msg.get("notiz", "")
        if not isinstance(note, str):
            note = ""
        # gzip and the SD card take a few hundred milliseconds -- not in the
        # event loop, otherwise the watchdog tick would stutter.
        path = await asyncio.to_thread(recording.save, note)
        return alert_message("info", f"Fahrt gespeichert: {path.name}")

    if kind == "interaction_stop":
        if interaction is not None:
            interaction.cancel("vom Nutzer beendet")  # message comes from _status_loop
        watchdog.stop()
        return None

    if kind == "home":
        by_map = homing.map_ready
        if not odometry.actually_measuring and not by_map:
            log.warning("Homing without motion measurement -- position is estimated")
        distance = odometry.distance_to_start()
        if explorer is not None:
            explorer.cancel("Heimfahrt gestartet")
        if interaction is not None:
            interaction.cancel("Heimfahrt gestartet")
        if maneuver is not None:
            maneuver.cancel("Heimfahrt gestartet")
        if homing.start():
            text = (
                "Heimfahrt nach Karte gestartet (zum Ausgangspunkt der Karte)"
                if by_map else f"Heimfahrt gestartet ({distance:.2f} m)"
            )
            await hub.broadcast(alert_message("info", text))
        return None

    if kind == "goto":
        if map_control is None:
            return alert_message("warning", "Fahrt zum Ziel geht hier nicht")
        error = map_control.target(msg.get("x"), msg.get("y"), msg.get("winkel"))
        if error:
            return alert_message("warning", error)
        return None  # message comes from MapControl

    if kind == "goto_cancel":
        homing.cancel("vom Nutzer abgebrochen")
        watchdog.stop()
        return None

    if kind == "mission_start":
        if map_control is None:
            return alert_message("warning", "Keine Karte eingerichtet")
        error = map_control.mission_start()
        return alert_message("warning", error) if error else None  # message from MapControl

    if kind == "return":
        if map_control is None:
            return alert_message("warning", "Keine Karte eingerichtet")
        error = map_control.return_to(msg.get("ziel") or "mission")
        return alert_message("warning", error) if error else None

    if kind == "map_live":
        if map_control is None:
            return alert_message("warning", "Keine Karte eingerichtet")
        active = bool(msg.get("aktiv", True))
        error = map_control.map_live(active)
        if error:
            return alert_message("warning", error)
        return alert_message("info", "Dauerkartierung an" if active else
                             "Dauerkartierung pausiert -- die Karte ändert sich nicht")

    if kind == "map_pose":
        if map_control is None:
            return alert_message("warning", "Keine Karte eingerichtet")
        error = map_control.set_pose(msg.get("x"), msg.get("y"), msg.get("winkel"))
        if error:
            return alert_message("warning", error)
        return alert_message("info", "Suche die Lage an der gezeigten Stelle")

    if kind == "map_load":
        if map_control is None:
            return alert_message("warning", "Keine Karte eingerichtet")
        name = msg.get("name")
        if not isinstance(name, str) or not name:
            return alert_message("warning", "map_load braucht einen Kartennamen (name)")
        try:
            info = await asyncio.to_thread(map_control.load, name)
        except (ValueError, RuntimeError, OSError) as exc:
            return alert_message("warning", f"Karte nicht geladen: {exc}")
        return alert_message("info", f"Karte „{info['name']}“ geladen -- suche die eigene Lage")

    if kind == "home_cancel":
        homing.cancel("vom Nutzer abgebrochen")
        watchdog.stop()
        await hub.broadcast(alert_message("info", "Heimfahrt abgebrochen"))
        return None

    if kind == "set_home":
        homing.cancel("Startpunkt wurde verschoben")
        if explorer is not None:
            explorer.cancel("Startpunkt wurde verschoben")
        if interaction is not None:
            interaction.cancel("Startpunkt wurde verschoben")
        if maneuver is not None:
            maneuver.cancel("Startpunkt wurde verschoben")
        # Only present in the simulation: there the imaginary reality also
        # has to go back to the start, otherwise the robot would be outside
        # the simulated room.
        zero = getattr(driver, "reset_true_pose", None)
        moved_in_map = False
        if zero is not None:
            before = driver.true_pose()
            zero()
            if map_control is not None:
                moved_in_map = map_control.simulation_moved(before, driver.true_pose())
        odometry.reset()
        # The old reference scan belongs to the old start and would
        # otherwise pull the position back there.
        calibrated = localizer.capture_reference()
        text = (
            "Startpunkt gesetzt und per Lidar eingemessen"
            if calibrated
            else "Startpunkt gesetzt (ohne Lidar -- Position wird nur geschätzt)"
        )
        # With a map: this is also the goal of map-based homing.
        if moved_in_map or (map_control is not None and map_control.start_here()):
            text += ", auch in der Karte"
        await hub.broadcast(alert_message("info", text))
        return None

    if kind == "say":
        if voice is None:
            return alert_message("warning", "Keine Sprachausgabe")
        text = msg.get("text")
        if not isinstance(text, str) or not text.strip():
            return error_message("say braucht ein Feld text")
        voice.say(text, priority=bool(msg.get("vorrang", False)))
        if not voice.available:
            return alert_message(
                "warning", "Kein Lautsprecher oder keine Sprachsynthese (siehe /stimme/status)"
            )
        return None

    if kind == "say_stop":
        if voice is not None:
            voice.mute()
        return None

    if kind == "speech":
        # As if spoken into the microphone: for trying things out without a
        # microphone and in --mock. Executed in _speech_loop, as with real
        # hearing.
        if speech is None:
            return alert_message("warning", "Keine Spracherkennung")
        text = msg.get("text")
        if not isinstance(text, str) or not text.strip():
            return error_message("speech braucht ein Feld text")
        if speech.hear(text) is None:
            return alert_message(
                "info", f"Kein Befehl erkannt in {text!r} (Weckwort: {config.SPEECH_WAKE_WORD})"
            )
        return None

    if kind == "ping":
        return {"type": "pong"}

    if kind == "beep":
        driver.beep(int(msg.get("ms", 100)))
        return None

    if kind == "vitals":
        # From the ESP: pass on to all clients so the app displays them.
        # Deliberately without evaluation -- see docs/decisions.md, E3.
        await hub.broadcast(vitals_message(msg.get("bpm"), msg.get("motion")))
        return None

    if kind == "box":
        # TODO: implement box/ once the mechanics are in place.
        return alert_message("info", "Box ist noch nicht angebaut")

    return error_message(f"Unbekannter Nachrichtentyp: {kind!r}")


async def _catch_up_reference(
    localizer: Localizer, odometry: Odometry, attempts: int = 120
) -> None:
    """Takes the reference scan as soon as the lidar has its first revolution.

    Generous deadline: at boot the lidar can take a while until it answers
    cleanly. Calibration only happens while the robot is still at the start
    -- saving a scan from somewhere else as the "start" would lead every
    later homing to the wrong place.
    """
    for _ in range(attempts):
        await asyncio.sleep(1.0)
        if localizer.has_reference:
            return  # set via "set_home" in the meantime
        x, y, _angle = odometry.pose()
        if math.hypot(x, y) > 0.05:
            break
        if localizer.capture_reference():
            return
    log.warning(
        "Start could not be calibrated -- localization without "
        "lidar correction until \"Startpunkt hier setzen\" is pressed"
    )


async def _match_loop(localizer: Localizer) -> None:
    """Regularly matches the position against the reference scan."""
    if not localizer.available:
        return
    while True:
        # Read anew each time: adjustable on the settings page.
        await asyncio.sleep(1.0 / max(0.1, config.MATCH_HZ))
        try:
            # The matching computes with a few hundred points and would
            # otherwise briefly block the event loop -- which shows up as
            # jerky control.
            await asyncio.to_thread(localizer.match_scan)
        except Exception:
            _loop_error("Lidar-Abgleich")


async def _hardware_loop(
    hub: ConnectionHub,
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    homing: Homing,
    explorer: Explorer | None = None,
    interaction: Interaction | None = None,
    maneuver: Maneuver | None = None,
    map_control: MapControl | None = None,
) -> None:
    """Keeps the connection to the driver alive.

    After a USB dropout the driver re-establishes it by itself; this makes
    sure nothing keeps driving until then and that the app finds out.
    """
    connected = driver.connected
    while True:
        await asyncio.sleep(config.RECONNECT_CHECK_S)
        try:
            # Reconnecting can take a while -- not in the event loop,
            # otherwise control would stutter.
            await asyncio.to_thread(driver.monitor)

            now = driver.connected
            if now == connected:
                continue
            connected = now
            if not now:
                homing.cancel("Verbindung zum Antrieb verloren")
                if map_control is not None:
                    # Remembers the running mapping drive in order to resume
                    # after reconnecting (MAPPING_DRIVE_RESUME_S).
                    map_control.drive_gone()
                if explorer is not None:
                    explorer.cancel("Verbindung zum Antrieb verloren")
                if maneuver is not None:
                    maneuver.cancel("Verbindung zum Antrieb verloren")
                if interaction is not None:
                    # Do not end it, only stop: gesture control stays on and
                    # is ready again right after reconnecting. It used to be
                    # off after every USB reset.
                    interaction.drive_lost()
                watchdog.stop()
                await hub.broadcast(
                    alert_message(
                        "warning",
                        "Verbindung zum Antrieb verloren -- verbinde neu. "
                        "Häufigste Ursache: Stromversorgung bzw. USB-Überlast "
                        "(siehe /diagnose).",
                    )
                )
            else:
                await hub.broadcast(alert_message("info", "Antrieb wieder verbunden"))
                if map_control is not None:
                    map_control.drive_back()
        except Exception:
            # This used to be the end: no more reconnecting until a restart.
            _loop_error("Verbindungsüberwachung")


async def _odometry_loop(watchdog: SafetyWatchdog, odometry: Odometry) -> None:
    """Keeps updating the position.

    Must run permanently, not only during homing: otherwise, when the home
    button is pressed, the robot would not know where it is by now.
    """
    tick = 1.0 / max(1, config.ODOMETRY_HZ)
    while True:
        await asyncio.sleep(tick)
        try:
            odometry.update(watchdog.current_command)
        except Exception:
            _loop_error("Positionsverfolgung")


async def _recording_loop(
    recording: Recorder,
    watchdog: SafetyWatchdog,
    odometry: Odometry,
    lidar: LidarService | None,
    explorer: Explorer,
    interaction: Interaction,
    homing: Homing,
) -> None:
    """Feeds the recorder with scans, ego-motion and states.

    At the odometry tick, so that every drive command has an odometry value
    nearby. Drive commands themselves come directly from the watchdog.
    """
    tick = 1.0 / max(1, config.ODOMETRY_HZ)
    while True:
        await asyncio.sleep(tick)
        try:
            if lidar is not None:
                recording.scan(lidar.latest())
            recording.odo(
                odometry.velocity(), odometry.pose(),
                watchdog.current_command, odometry.actually_measuring,
            )
            e = explorer.state()
            i = interaction.state()
            # Explicit string keys: they are the field names of the
            # recording file and must stay readable for old recordings.
            recording.state(**{
                "erkunder": e.get("zustand"),
                "erkunder_text": e.get("text"),
                "erkunder_grund": e.get("grund"),
                "interaktion": i.get("modus") if interaction.active else None,
                "heimfahrt": bool(homing.active),
            })
        except Exception:
            _loop_error("Recorder")


async def _pulse_loop(diagnostics: Diagnostics) -> None:
    """Heartbeat of the event loop for diagnostics.

    If the sleep wakes up clearly too late, the loop was blocked (a hang).
    If the pulse stops entirely, diagnostics records from its own thread
    where the loop is stuck.
    """
    tick = config.DIAG_PULSE_S
    while True:
        previous = time.monotonic()
        await asyncio.sleep(tick)
        diagnostics.pulse(max(0.0, time.monotonic() - previous - tick))


async def _status_loop(
    hub: ConnectionHub,
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    odometry: Odometry,
    homing: Homing,
    localizer: Localizer,
    guard: CollisionGuard,
    explorer: Explorer | None = None,
    detection: ObstacleDetection | None = None,
    lidar: LidarService | None = None,
    people: PersonDetection | None = None,
    interaction: Interaction | None = None,
    voice: Voice | None = None,
    speech: SpeechRecognition | None = None,
    maneuver: Maneuver | None = None,
    map_control: MapControl | None = None,
) -> None:
    """Regularly sends the state to all clients."""
    last_active = False
    last_explored = False
    last_interacted = False
    last_maneuver = 0
    while True:
        await asyncio.sleep(config.STATUS_INTERVAL_S)
        try:
            # Report the end of homing -- it ends by itself without anyone
            # asking.
            if last_active and not homing.active and homing.last_reason:
                await hub.broadcast(
                    alert_message("info", f"{homing.label}: {homing.last_reason}")
                )
            last_active = homing.active

            # Likewise the end of autonomous driving (lidar gone, stuck).
            if explorer is not None:
                if last_explored and not explorer.active and explorer.last_reason:
                    await hub.broadcast(
                        alert_message(
                            "info", f"Autonomes Fahren beendet: {explorer.last_reason}"
                        )
                    )
                last_explored = explorer.active

            # Interaction: recognized gestures and mode changes as a notice,
            # plus the end (e.g. lidar gone).
            if interaction is not None:
                for text in interaction.fetch_messages():
                    await hub.broadcast(alert_message("info", text))
                if last_interacted and not interaction.active and interaction.last_reason:
                    await hub.broadcast(
                        alert_message("info", f"Interaktion beendet: {interaction.last_reason}")
                    )
                last_interacted = interaction.active

            # Exact drive commands: report the end, and if it did not work,
            # also say so (started by speech, nobody is looking at the app).
            # Via the counter, not via "active": "zu wenig Platz" is over
            # faster than one tick of this loop.
            if maneuver is not None and maneuver.finished != last_maneuver:
                last_maneuver = maneuver.finished
                if maneuver.last_reason:
                    await hub.broadcast(alert_message(
                        "info" if maneuver.succeeded else "warning",
                        f"Fahrbefehl: {maneuver.last_reason}",
                    ))
                    if not maneuver.succeeded and voice is not None:
                        voice.say(maneuver.announcement or "Das hat nicht ganz geklappt.")

            if hub.count:
                autonomy = (
                    _autonomy(explorer, detection, lidar, people)
                    if explorer is not None and detection is not None
                    else None
                )
                await hub.broadcast(
                    state_message(
                        watchdog.current_command,
                        driver.name,
                        driver.connected,
                        pose=odometry.pose(),
                        homing=homing.active,
                        pose_measured=odometry.actually_measuring,
                        collision=guard.state(),
                        autonomy=autonomy,
                        interaction=interaction.state() if interaction is not None else None,
                        voice=voice.state() if voice is not None else None,
                        speech=speech.state() if speech is not None else None,
                        occ_map=map_control.state_brief() if map_control is not None else None,
                    )
                )
        except Exception:
            # This used to end the status messages for good: the app showed
            # frozen values, it looked like a crash.
            _loop_error("Statusmeldungen")


def _autonomy(
    explorer: Explorer,
    detection: ObstacleDetection,
    lidar: LidarService | None,
    people: PersonDetection | None,
) -> dict[str, Any]:
    """State for state.autonomie and /autonomie/status.

    The obstacles are included even without exploration running -- so one
    can see before the start what the robot detects.
    """
    scan = lidar.latest() if lidar is not None else None
    if scan is not None and time.monotonic() - scan.timestamp > 3.0:
        scan = None  # do not output an old scan as the current situation
    angle = people.angle_ranges() if people is not None else []
    obstacles = detection.current(scan, angle) if scan is not None else []
    return {
        **explorer.state(),
        # The nearest ones are enough for display and list; all of them
        # would needlessly bloat the status message in cluttered rooms.
        "hindernisse": [h.as_dict() for h in obstacles[:12]],
        "personen": people.boxes() if people is not None else [],
        "personenerkennung": people.status() if people is not None else None,
    }
