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


def create_mapping_router(manager: MappingManager | None = None, fahrt=None) -> APIRouter:
    """Create the mapping routes.

    The manager is intentionally created here (not started here). Opening
    /map therefore does not start SLAM; only POST /api/mapping/start does.

    ``fahrt`` steuert die Kartierfahrt (automatisches Abfahren des Raums).
    Der Server reicht ein Objekt mit ``starten() -> str | None`` (Fehlertext
    oder None), ``stoppen(grund)``, ``aktiv`` und ``zustand()`` herein: Nur
    er kennt Watchdog, Heimfahrt und Interaktion, die dafür weichen müssen.
    """
    mapping_manager = manager or MappingManager()
    router = APIRouter()
    # Offene /map-Seiten. Die Kartierfahrt wird von dort gestartet und
    # braucht jemanden, der zusieht: Schließt die letzte, hält er an --
    # dieselbe Totmannschaltung wie bei der App.
    seiten = {"offen": 0}

    def _status() -> dict[str, Any]:
        status = mapping_manager.status()
        status["fahrt"] = fahrt.zustand() if fahrt is not None else None
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
            log.exception("Mapping-Status konnte nicht gelesen werden")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router.post("/api/mapping/start")
    async def mapping_start() -> dict[str, Any]:
        try:
            return await run_in_threadpool(mapping_manager.start)
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            log.exception("Mapping konnte nicht gestartet werden")
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except Exception as exc:
            log.exception("Unerwarteter Fehler beim Mapping-Start")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.post("/api/mapping/stop")
    async def mapping_stop() -> dict[str, Any]:
        # Erst anhalten: Ohne SLAM und Lidar-Odometrie fährt er blind.
        if fahrt is not None:
            fahrt.stoppen("Mapping gestoppt")
        try:
            return await run_in_threadpool(mapping_manager.stop)
        except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
            log.exception("Mapping konnte nicht gestoppt werden")
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    @router.post("/api/mapping/fahrt/start")
    async def fahrt_start() -> dict[str, Any]:
        if fahrt is None:
            raise HTTPException(status_code=503, detail="Kartierfahrt ist nicht eingerichtet.")
        fehler = fahrt.starten()
        if fehler:
            raise HTTPException(status_code=409, detail=fehler)
        return fahrt.zustand()

    @router.post("/api/mapping/fahrt/stop")
    async def fahrt_stop() -> dict[str, Any]:
        if fahrt is None:
            raise HTTPException(status_code=503, detail="Kartierfahrt ist nicht eingerichtet.")
        fahrt.stoppen("vom Nutzer beendet")
        return fahrt.zustand()

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
            log.exception("Karte konnte nicht gespeichert werden")
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @router.get("/api/mapping/maps")
    async def mapping_maps() -> list[dict[str, Any]]:
        return await run_in_threadpool(mapping_manager.list_maps)

    @router.websocket("/api/mapping/ws")
    async def mapping_websocket(websocket: WebSocket) -> None:
        """Push the latest map to the browser when a new map arrives."""
        await websocket.accept()
        last_map_received_at: float | None = None
        last_status_at = 0.0
        letzte_fahrt: dict[str, Any] | None = None
        seiten["offen"] += 1
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

                # Die Kartierfahrt öfter melden: Sie fragt kein Docker ab,
                # und wer zusieht, will wissen, was der Roboter gerade tut.
                if fahrt is not None:
                    zustand = fahrt.zustand()
                    if zustand != letzte_fahrt:
                        await websocket.send_json({"type": "fahrt", "data": zustand})
                        letzte_fahrt = zustand

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
            log.exception("Mapping-WebSocket wurde unerwartet beendet")
            try:
                await websocket.close(code=1011)
            except Exception:
                pass
        finally:
            seiten["offen"] -= 1
            if seiten["offen"] <= 0 and fahrt is not None and fahrt.aktiv:
                log.warning("Keine /map-Seite mehr offen -- Kartierfahrt hält an")
                fahrt.stoppen("Seite /map geschlossen")

    # Expose the instance for optional application shutdown handling.
    router.mapping_manager = mapping_manager  # type: ignore[attr-defined]
    return router
