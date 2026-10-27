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
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .. import config
from ..drive.base import DriveBase
from ..protocol import (
    DriveCommand,
    ProtocolError,
    alert_message,
    error_message,
    state_message,
    vitals_message,
)
from ..safety import SafetyWatchdog

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


def create_app(driver: DriveBase) -> FastAPI:
    """Baut die Server-Anwendung um einen Antrieb herum."""

    watchdog = SafetyWatchdog(driver)
    hub = ConnectionHub()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        driver.connect()
        watchdog.start()
        status_task = asyncio.create_task(_status_loop(hub, watchdog, driver))
        try:
            yield
        finally:
            # Reihenfolge ist wichtig: erst Statusmeldungen beenden, dann
            # Watchdog (der hält an), dann die Hardware trennen.
            status_task.cancel()
            await watchdog.shutdown()
            driver.disconnect()

    app = FastAPI(title="ROSMASTER X3 Hilfsroboter", lifespan=lifespan)

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        await hub.add(ws)
        try:
            while True:
                msg = await ws.receive_json()
                reply = await _handle(msg, watchdog, driver, hub)
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
                watchdog.stop()

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
) -> dict[str, Any] | None:
    """Eine eingehende Nachricht verarbeiten.

    Gibt die Antwort an den Absender zurück, oder None wenn keine nötig ist.
    """
    kind = msg.get("type")

    if kind == "drive":
        try:
            watchdog.submit(DriveCommand.from_message(msg))
        except ProtocolError as exc:
            return error_message(str(exc))
        return None  # Status geht ohnehin im Takt raus

    if kind == "stop":
        watchdog.stop()
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


async def _status_loop(
    hub: ConnectionHub, watchdog: SafetyWatchdog, driver: DriveBase
) -> None:
    """Schickt regelmäßig den Zustand an alle Clients."""
    try:
        while True:
            await asyncio.sleep(config.STATUS_INTERVAL_S)
            if hub.count:
                await hub.broadcast(
                    state_message(watchdog.current_command, driver.name, driver.connected)
                )
    except asyncio.CancelledError:
        raise
