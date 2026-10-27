"""WebSocket-Server -- der einzige Ein- und Ausgang des Roboters.

App und ESP32 verbinden sich beide auf `/ws` und sprechen das Protokoll aus
`protocol.py`. Zusätzlich liefert der Server die Web-App unter `/` aus, damit
man nur eine Adresse im Browser braucht.
"""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .. import config
from ..drive.base import DriveBase
from ..protocol import (
    DriveCommand,
    ProtocolError,
    alert_message,
    error_message,
    limits_message,
    state_message,
    vitals_message,
)
from ..navigation import Homing, Odometry
from ..safety import SafetyWatchdog
from ..sensors.camera import CameraService, create_camera

log = logging.getLogger(__name__)

# robot/src/rosmaster_assist/api/server.py -> Projektwurzel -> webapp/
WEBAPP_DIR = Path(__file__).resolve().parents[4] / "webapp"


class ConnectionHub:
    """Hält alle offenen Verbindungen und verteilt Nachrichten an sie."""

    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def add(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.add(ws)
        log.info("Client verbunden (%d aktiv)", len(self._clients))

    async def remove(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)
        log.info("Client getrennt (%d aktiv)", len(self._clients))

    async def broadcast(self, message: dict[str, Any]) -> None:
        """An alle senden. Tote Verbindungen werden dabei aussortiert."""
        async with self._lock:
            targets = list(self._clients)

        dead = []
        for ws in targets:
            try:
                await ws.send_json(message)
            except Exception:
                dead.append(ws)

        for ws in dead:
            await self.remove(ws)

    @property
    def count(self) -> int:
        return len(self._clients)


def create_app(driver: DriveBase, use_mock_camera: bool = False) -> FastAPI:
    """Baut die Server-Anwendung um einen Antrieb herum."""

    watchdog = SafetyWatchdog(driver)
    odometry = Odometry(driver)
    homing = Homing(watchdog, odometry)
    hub = ConnectionHub()
    camera = (
        CameraService(create_camera(use_mock_camera)) if config.CAMERA_ENABLED else None
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        driver.connect()
        if driver.name == "mock":
            log.warning("Antrieb: SIMULATION — die Motoren bleiben stehen")
        else:
            log.warning("Antrieb: %s — der Roboter kann sich bewegen", driver.name)
        watchdog.start()
        if camera is not None:
            camera.start()
        status_task = asyncio.create_task(
            _status_loop(hub, watchdog, driver, odometry, homing)
        )
        odo_task = asyncio.create_task(_odometry_loop(watchdog, odometry))
        try:
            yield
        finally:
            # Reihenfolge ist wichtig: erst Statusmeldungen beenden, dann
            # Watchdog (der hält an), dann die Hardware trennen.
            status_task.cancel()
            odo_task.cancel()
            homing.abbrechen("Server wird beendet")
            await watchdog.shutdown()
            if camera is not None:
                camera.stop()
            driver.disconnect()

    app = FastAPI(title="ROSMASTER X3 Hilfsroboter", lifespan=lifespan)

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        await hub.add(ws)
        # Zuerst die Grenzwerte: die App rechnet ihre Reglerstellung darauf
        # um und muss sie deshalb kennen, bevor der erste Befehl rausgeht.
        await ws.send_json(
            limits_message(config.MAX_SPEED_X, config.MAX_SPEED_Y, config.MAX_SPEED_OMEGA)
        )
        try:
            while True:
                msg = await ws.receive_json()
                reply = await _handle(msg, watchdog, driver, hub, odometry, homing)
                if reply is not None:
                    await ws.send_json(reply)
        except WebSocketDisconnect:
            pass
        except Exception:
            log.exception("Fehler in der Verbindung")
        finally:
            await hub.remove(ws)
            # Letzter Client weg? Dann hat niemand mehr die Kontrolle.
            if hub.count == 0:
                log.warning("Kein Client mehr verbunden -- halte an")
                homing.abbrechen("Verbindung verloren")
                watchdog.stop()

    # --- Kamera ---
    #
    # MJPEG: eine nie endende HTTP-Antwort aus aneinandergehängten Bildern.
    # Der Browser zeigt das direkt in einem <img> an -- ohne JavaScript und
    # ohne zusätzliche Bibliothek.

    GRENZE = "frame"

    @app.get("/camera/stream")
    async def camera_stream():
        if camera is None:
            return Response("Kamera abgeschaltet", status_code=503)

        async def bilder():
            interval = 1.0 / max(1, config.CAMERA_FPS)
            zuletzt = None
            while True:
                frame = camera.latest()
                # Dasselbe Bild nicht doppelt schicken: spart bei einer
                # langsamen Kamera spürbar Bandbreite.
                if frame is not None and frame is not zuletzt:
                    data, mime = frame
                    zuletzt = frame
                    yield (
                        f"--{GRENZE}\r\nContent-Type: {mime}\r\n"
                        f"Content-Length: {len(data)}\r\n\r\n".encode()
                        + data
                        + b"\r\n"
                    )
                await asyncio.sleep(interval)

        return StreamingResponse(
            bilder(),
            media_type=f"multipart/x-mixed-replace; boundary={GRENZE}",
            # Ohne das puffern manche Proxys den Stream, bis nichts mehr
            # ankommt -- das Bild bliebe dann einfach schwarz.
            headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"},
        )

    @app.get("/camera/status")
    async def camera_status():
        """Warum kommt ein Bild -- oder warum nicht.

        Im Browser aufrufbar, damit die Frage beantwortbar ist, ohne auf
        dem Roboter im Log zu suchen.
        """
        if camera is None:
            return {"kamera": None, "hinweis": "CAMERA_ENABLED ist False"}
        return camera.status()

    @app.get("/camera/snapshot")
    async def camera_snapshot():
        """Ein einzelnes Bild -- nützlich zum Testen per curl."""
        if camera is None:
            return Response("Kamera abgeschaltet", status_code=503)
        frame = camera.latest()
        if frame is None:
            return Response("Noch kein Bild", status_code=503)
        data, mime = frame
        return Response(data, media_type=mime, headers={"Cache-Control": "no-store"})

    # Die Web-App direkt mitausliefern, damit eine Adresse genügt.
    if WEBAPP_DIR.is_dir():
        app.mount("/static", StaticFiles(directory=WEBAPP_DIR), name="static")

        @app.get("/")
        async def index() -> FileResponse:
            return FileResponse(WEBAPP_DIR / "index.html")
    else:
        log.warning("webapp/ nicht gefunden unter %s -- nur /ws verfügbar", WEBAPP_DIR)

    return app


async def _handle(
    msg: dict[str, Any],
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    hub: ConnectionHub,
    odometry: Odometry,
    homing: Homing,
) -> dict[str, Any] | None:
    """Eine eingehende Nachricht verarbeiten.

    Gibt die Antwort an den Absender zurück, oder None wenn keine nötig ist.
    """
    kind = msg.get("type")

    if kind == "drive":
        try:
            cmd = DriveCommand.from_message(msg)
        except ProtocolError as exc:
            return error_message(str(exc))

        # Der Mensch hat Vorrang. Die App sendet auch im Stillstand weiter,
        # deshalb bricht nur eine tatsächliche Eingabe ab -- sonst wäre die
        # Heimfahrt sofort wieder beendet.
        if homing.aktiv and not cmd.is_stop():
            homing.abbrechen("von Hand übersteuert")
            await hub.broadcast(alert_message("info", "Heimfahrt abgebrochen"))
        elif homing.aktiv:
            return None  # Leerlauf-Nachrichten ignorieren, Heimfahrt steuert

        watchdog.submit(cmd)
        return None  # Status geht ohnehin im Takt raus

    if kind == "stop":
        homing.abbrechen("Not-Stopp")
        watchdog.stop()
        return None

    if kind == "home":
        if not odometry.misst_tatsaechlich:
            log.warning("Heimfahrt ohne Bewegungsmessung -- Position ist geschätzt")
        abstand = odometry.distanz_zum_start()
        if homing.start():
            await hub.broadcast(
                alert_message("info", f"Heimfahrt gestartet ({abstand:.2f} m)")
            )
        return None

    if kind == "home_cancel":
        homing.abbrechen("vom Nutzer abgebrochen")
        watchdog.stop()
        await hub.broadcast(alert_message("info", "Heimfahrt abgebrochen"))
        return None

    if kind == "set_home":
        homing.abbrechen("Startpunkt wurde verschoben")
        odometry.reset()
        await hub.broadcast(alert_message("info", "Startpunkt ist jetzt hier"))
        return None

    if kind == "ping":
        return {"type": "pong"}

    if kind == "beep":
        driver.beep(int(msg.get("ms", 100)))
        return None

    if kind == "vitals":
        # Vom ESP: an alle Clients weiterreichen, damit die App sie anzeigt.
        # Bewusst ohne Bewertung -- siehe docs/entscheidungen.md, E3.
        await hub.broadcast(vitals_message(msg.get("bpm"), msg.get("motion")))
        return None

    if kind == "box":
        # TODO: box/ implementieren, sobald die Mechanik steht.
        return alert_message("info", "Box ist noch nicht angebaut")

    return error_message(f"Unbekannter Nachrichtentyp: {kind!r}")


async def _odometry_loop(watchdog: SafetyWatchdog, odometry: Odometry) -> None:
    """Schreibt die Position fort.

    Muss dauerhaft laufen, nicht nur während der Heimfahrt: sonst wüsste der
    Roboter beim Druck auf den Home-Knopf nicht, wo er inzwischen steht.
    """
    takt = 1.0 / max(1, config.ODOMETRY_HZ)
    try:
        while True:
            await asyncio.sleep(takt)
            odometry.update(watchdog.current_command)
    except asyncio.CancelledError:
        raise
    except Exception:
        log.exception("Positionsverfolgung abgestürzt -- Heimfahrt unbrauchbar")


async def _status_loop(
    hub: ConnectionHub,
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    odometry: Odometry,
    homing: Homing,
) -> None:
    """Schickt regelmäßig den Zustand an alle Clients."""
    zuletzt_aktiv = False
    try:
        while True:
            await asyncio.sleep(config.STATUS_INTERVAL_S)

            # Das Ende der Heimfahrt melden -- sie beendet sich selbst,
            # ohne dass jemand danach fragt.
            if zuletzt_aktiv and not homing.aktiv and homing.letzter_grund:
                await hub.broadcast(
                    alert_message("info", f"Heimfahrt: {homing.letzter_grund}")
                )
            zuletzt_aktiv = homing.aktiv

            if hub.count:
                await hub.broadcast(
                    state_message(
                        watchdog.current_command,
                        driver.name,
                        driver.connected,
                        pose=odometry.pose(),
                        homing=homing.aktiv,
                    )
                )
    except asyncio.CancelledError:
        raise
