"""FastAPI routes for the ROSMASTER mapping UI and API."""

from __future__ import annotations

import asyncio
import logging
import subprocess
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field
from starlette.concurrency import run_in_threadpool

from .manager import MappingManager

log = logging.getLogger(__name__)
_TEMPLATE = Path(__file__).resolve().parents[1] / "templates" / "map.html"


class SaveMapRequest(BaseModel):
    """Optional map name sent by the browser when saving a map."""

    name: str | None = Field(default=None, max_length=64)


class PointRequest(BaseModel):
    """A point on the map (m, map frame), heading optional (degrees)."""

    x: float
    y: float
    # Alias: the JSON field stays "winkel" (API contract), whatever the code calls it.
    angle: float | None = Field(default=None, alias="winkel")


class LoadRequest(BaseModel):
    name: str = Field(max_length=64)


class LiveRequest(BaseModel):
    aktiv: bool


class ReturnRequest(BaseModel):
    ziel: str = Field(default="mission", pattern="^(mission|home)$")


def create_mapping_router(manager: MappingManager | None = None, drive=None) -> APIRouter:
    """Create the mapping routes.

    The manager is intentionally created here (not started here). Opening
    /map therefore does not start SLAM; only POST /api/mapping/start does.

    ``drive`` is the server's ``MapControl`` (api/map_control.py): start/stop
    the mapping drive, load a map, drive to a point, set the pose. Only the
    server knows the watchdog, homing and interaction, which have to give way
    for this. Without it (tests) there is only mapping and the map.
    """
    mapping_manager = manager or MappingManager()
    router = APIRouter()

    def _control():
        if drive is None:
            raise HTTPException(status_code=503, detail="Kartensteuerung ist nicht eingerichtet.")
        return drive

    def _status() -> dict[str, Any]:
        status = mapping_manager.status()
        status["fahrt"] = drive.state() if drive is not None else None
        return status

    @router.get("/map", include_in_schema=False)
    async def map_page() -> FileResponse:
        if not _TEMPLATE.is_file():
            raise HTTPException(status_code=500, detail="templates/map.html fehlt.")
        return FileResponse(_TEMPLATE, media_type="text/html; charset=utf-8")

    @router.get("/api/mapping/status")
    async def mapping_status() -> dict[str, Any]:
        try:
            return await run_in_threadpool(_status)
        except Exception as exc:  # report Docker/ROS problems as an API response
            log.exception("Could not read mapping status")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router.post("/api/mapping/start")
    async def mapping_start() -> dict[str, Any]:
        try:
            return await run_in_threadpool(mapping_manager.start)
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            log.exception("Could not start mapping")
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except Exception as exc:
            log.exception("Unexpected error while starting mapping")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.post("/api/mapping/stop")
    async def mapping_stop() -> dict[str, Any]:
        # Stop first: without SLAM and scan odometry it drives blind.
        if drive is not None:
            drive.stop("Mapping gestoppt")
        try:
            return await run_in_threadpool(mapping_manager.stop)
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            log.exception("Could not stop mapping")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router.post("/api/mapping/fahrt/start")
    async def drive_start() -> dict[str, Any]:
        if drive is None:
            raise HTTPException(status_code=503, detail="Kartierfahrt ist nicht eingerichtet.")
        error = drive.start()
        if error:
            raise HTTPException(status_code=409, detail=error)
        return drive.state()

    @router.post("/api/mapping/fahrt/stop")
    async def drive_stop() -> dict[str, Any]:
        if drive is None:
            raise HTTPException(status_code=503, detail="Kartierfahrt ist nicht eingerichtet.")
        drive.stop("vom Nutzer beendet")
        return drive.state()

    @router.get("/api/mapping/map")
    async def mapping_map() -> dict[str, Any]:
        payload = await run_in_threadpool(mapping_manager.get_map)
        if payload is None:
            raise HTTPException(status_code=404, detail="Noch keine Karte verfügbar.")
        return payload

    @router.post("/api/mapping/save")
    async def mapping_save(request: SaveMapRequest) -> dict[str, Any]:
        try:
            return await run_in_threadpool(mapping_manager.save_map, request.name)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except OSError as exc:
            log.exception("Could not save map")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.get("/api/mapping/maps")
    async def mapping_maps() -> list[dict[str, Any]]:
        return await run_in_threadpool(mapping_manager.list_maps)

    # --- Using the map ------------------------------------------------------------

    @router.post("/api/mapping/laden")
    async def mapping_load(request: LoadRequest) -> dict[str, Any]:
        try:
            return await run_in_threadpool(_control().load, request.name)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except OSError as exc:
            log.exception("Could not load map")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.get("/api/mapping/ortung")
    async def mapping_localization() -> dict[str, Any]:
        return _control().display()

    @router.post("/api/mapping/ziel")
    async def mapping_goal(request: PointRequest) -> dict[str, Any]:
        control = _control()
        error = control.target(request.x, request.y, request.angle)
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.display()

    @router.post("/api/mapping/ziel/stop")
    async def mapping_goal_stop() -> dict[str, Any]:
        control = _control()
        control.goal_stop()
        return control.display()

    @router.post("/api/mapping/pose")
    async def mapping_pose(request: PointRequest) -> dict[str, Any]:
        control = _control()
        error = control.set_pose(request.x, request.y, request.angle)
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.display()

    # --- Always-on mapping and the way back (E14) ----------------------------------

    @router.get("/api/mapping/live")
    async def mapping_live() -> dict[str, Any]:
        return _control().live_status()

    @router.post("/api/mapping/live")
    async def mapping_live_switch(request: LiveRequest) -> dict[str, Any]:
        control = _control()
        error = control.map_live(request.aktiv)
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.live_status()

    @router.post("/api/mapping/mission/start")
    async def mapping_mission_start() -> dict[str, Any]:
        control = _control()
        error = control.mission_start()
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.display()

    @router.post("/api/mapping/zurueck")
    async def mapping_return(request: ReturnRequest) -> dict[str, Any]:
        control = _control()
        error = control.return_to(request.ziel)
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.display()

    @router.post("/api/mapping/hauptkarte/neu")
    async def mapping_new_main() -> dict[str, Any]:
        control = _control()
        error = await run_in_threadpool(control.new_map)
        if error:
            raise HTTPException(status_code=409, detail=error)
        return control.display()

    @router.websocket("/api/mapping/ws")
    async def mapping_websocket(websocket: WebSocket) -> None:
        """Push the latest map to the browser when a new map arrives."""
        await websocket.accept()
        last_map_received_at: float | None = None
        last_status_at = 0.0
        last_drive: dict[str, Any] | None = None
        last_localization: dict[str, Any] | None = None
        # The page counts as a watcher: if all of them close (and no app is
        # open), a mapping drive or goal drive stops after a deadline.
        if drive is not None:
            drive.page_opened()
        try:
            while True:
                now = asyncio.get_running_loop().time()

                # Status includes Docker inspection; query it less often than
                # the map so the Pi is not needlessly polling Docker.
                if now - last_status_at >= 3.0:
                    try:
                        status = await run_in_threadpool(_status)
                        await websocket.send_json({"type": "status", "data": status})
                    except Exception as exc:
                        await websocket.send_json(
                            {"type": "error", "message": f"Statusfehler: {exc}"}
                        )
                    last_status_at = now

                # Report the mapping drive more often: it does not query Docker,
                # and whoever is watching wants to know what the robot is doing.
                if drive is not None:
                    state = drive.state()
                    if state != last_drive:
                        await websocket.send_json({"type": "fahrt", "data": state})
                        last_drive = state
                    # Where it is on the map, where it is heading.
                    localization = drive.display()
                    if localization != last_localization:
                        await websocket.send_json({"type": "ortung", "data": localization})
                        last_localization = localization

                payload = await run_in_threadpool(mapping_manager.get_map)
                if payload is not None:
                    received_at = payload.get("received_at")
                    if received_at != last_map_received_at:
                        await websocket.send_json({"type": "map", "data": payload})
                        last_map_received_at = received_at

                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            return
        except Exception:
            log.exception("Mapping WebSocket closed unexpectedly")
            try:
                await websocket.close(code=1011)
            except Exception:
                pass
        finally:
            if drive is not None:
                drive.page_closed()

    # Expose the instance for optional application shutdown handling.
    router.mapping_manager = mapping_manager  # type: ignore[attr-defined]
    return router
