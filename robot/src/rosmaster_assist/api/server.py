"""WebSocket-Server -- der einzige Ein- und Ausgang des Roboters.

App und ESP32 verbinden sich beide auf `/ws` und sprechen das Protokoll aus
`protocol.py`. Zusätzlich liefert der Server die Web-App unter `/` aus, damit
man nur eine Adresse im Browser braucht.
"""

from __future__ import annotations

import asyncio
import logging
import math
import re
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import (
    HTMLResponse,
    JSONResponse,
    Response,
    StreamingResponse,
)
from fastapi.staticfiles import StaticFiles

from .. import config, settings
from ..diagnose import Diagnose, LogPuffer
from ..drive.base import DriveBase
from ..audio.mukke import Mukke, MukkeFehler
from ..interaktion import sprache as sp
from ..interaktion import verhalten as vh
from ..interaktion.sprache import Spracherkennung
from ..interaktion.stimme import Stimme
from ..interaktion.verhalten import Interaktion
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
from ..navigation import Erkunder, Hinderniserkennung, Homing, Localizer, Odometry
from ..safety import CollisionGuard, SafetyWatchdog
from ..sensors.camera import CameraService, create_cameras
from ..sensors.lidar import LidarService, create_lidar
from ..sensors.personen import Personenerkennung
from ..sensors.pose import SKELETT, Posenerkennung

log = logging.getLogger(__name__)

# robot/src/rosmaster_assist/api/server.py -> Projektwurzel -> webapp/
WEBAPP_DIR = Path(__file__).resolve().parents[4] / "webapp"

# Die Web-App vor jeder Verwendung beim Roboter nachfragen lassen (bei
# unveränderter Datei nur eine kurze 304-Antwort). Ohne diese Angabe darf
# der Browser Dateien nach eigenem Ermessen weiterverwenden -- nach einem
# Update passten dann eine neue index.html und eine alte app.js nicht
# zusammen, das Skript brach ab, und die App blieb auf "getrennt".
_KEIN_CACHE = {"Cache-Control": "no-cache"}


class _OhneVeraltetenCache(StaticFiles):
    async def get_response(self, path, scope):
        antwort = await super().get_response(path, scope)
        antwort.headers.update(_KEIN_CACHE)
        return antwort


def _seite(name: str) -> HTMLResponse:
    """Eine HTML-Seite der Web-App, Skripte und Styles mit Versionsnummer.

    no-cache allein reicht nicht: eine Kopie, die der Browser noch von vor
    dieser Einstellung hat, verwendet er ungefragt weiter. Mit ``?v=...``
    ist jede geänderte Datei für ihn eine neue Adresse -- eine alte app.js
    zur neuen index.html (Skript bricht ab, App bleibt auf "getrennt")
    kann es dann nicht mehr geben.
    """
    def version(treffer: re.Match) -> str:
        pfad = WEBAPP_DIR / treffer.group(2)
        try:
            v = int(pfad.stat().st_mtime)
        except OSError:
            return treffer.group(0)
        return f'{treffer.group(1)}{treffer.group(2)}?v={v}"'

    html = (WEBAPP_DIR / name).read_text(encoding="utf-8")
    html = re.sub(r'(/static/)([\w./-]+\.(?:js|css))"', version, html)
    return HTMLResponse(html, headers=_KEIN_CACHE)


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

    # Zuerst: einige Dienste lesen ihre Werte nur beim Anlegen (etwa der
    # Startzustand des Hindernisstopps).
    settings.laden()

    watchdog = SafetyWatchdog(driver)
    odometry = Odometry(driver)
    hub = ConnectionHub()
    kameras: dict[str, CameraService] = (
        {name: CameraService(k) for name, k in create_cameras(use_mock_camera).items()}
        if config.CAMERA_ENABLED
        else {}
    )
    # Die Standardkamera antwortet unter den Endpunkten ohne Kameranamen
    # (/camera/stream usw.) -- so laufen Clients, die nur eine Kamera
    # kennen, unverändert weiter.
    camera = kameras.get(config.CAMERA_DEFAULT) or next(iter(kameras.values()), None)

    # Der simulierte Lidar braucht die wahre Position, um den gedachten Raum
    # abzutasten. Die kennt nur der Mock-Antrieb -- die echte Hardware
    # natürlich nicht, die misst einfach.
    pose_quelle = getattr(driver, "wahre_pose", None)
    lidar = (
        LidarService(create_lidar(use_mock_camera, pose_quelle))
        if config.LIDAR_ENABLED
        else None
    )
    localizer = Localizer(odometry, lidar)
    homing = Homing(watchdog, odometry, localizer)
    guard = CollisionGuard(localizer)
    erkennung = Hinderniserkennung()
    # Personenerkennung und Posenerkennung (Gesten, Folgen) schauen beide
    # durch die Personenkamera (C270), siehe docs/interaktion.md.
    personen_kamera = kameras.get(config.PERSON_CAMERA, camera)
    personen = Personenerkennung(personen_kamera)
    erkunder = Erkunder(watchdog, lidar, erkennung, personen)
    pose = Posenerkennung(personen_kamera, mock=use_mock_camera)
    mukke = Mukke()
    # Stimme und Musik teilen sich den Lautsprecher: Solange der Roboter
    # spricht, pausiert die Musik und läuft danach an derselben Stelle weiter.
    stimme = Stimme(
        mock=use_mock_camera, vor_sprechen=mukke.unterbrechen, nach_sprechen=mukke.fortsetzen
    )
    # Der Antrieb wird gefragt, damit die Interaktion ohne Board gar nicht
    # erst losfährt (und nach dem Neuverbinden nicht von selbst).
    interaktion = Interaktion(watchdog, lidar, pose, stimme, antrieb=lambda: driver.connected)
    # Bekommt die Stimme, um wegzuhören, solange der Roboter selbst spricht
    # (sonst versteht er sich selbst).
    sprache = Spracherkennung(mock=use_mock_camera, stimme=stimme)

    # Die Diagnoseseite zeigt auch die Server-Meldungen. Ein alter Puffer
    # (create_app ein zweites Mal, etwa in Tests) wird ersetzt, sonst
    # käme jede Zeile doppelt an.
    paket_log = logging.getLogger("rosmaster_assist")
    for h in [h for h in paket_log.handlers if isinstance(h, LogPuffer)]:
        paket_log.removeHandler(h)
    log_puffer = LogPuffer(config.DIAGNOSE_LOG_ZEILEN)
    paket_log.addHandler(log_puffer)
    diagnose = Diagnose(
        driver, watchdog, lidar, kameras, pose, log_puffer, mock=use_mock_camera
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        driver.connect()
        if driver.name == "mock":
            log.warning("Antrieb: SIMULATION — die Motoren bleiben stehen")
        else:
            log.warning("Antrieb: %s — der Roboter kann sich bewegen", driver.name)
        watchdog.start()
        for kamera in kameras.values():
            kamera.start()
        personen.start()
        pose.start()  # rechnet erst, wenn die Interaktion läuft
        stimme.start()
        # Hörbarer Selbsttest: Wer den Roboter einschaltet, weiß sofort,
        # ob der Lautsprecher geht.
        stimme.sagen("Ich bin bereit.")
        sprache.start()  # lädt das Modell im eigenen Thread, blockiert nicht
        if lidar is not None:
            lidar.start()
            # Dem Lidar eine Umdrehung Zeit geben, dann den Startpunkt
            # einmessen. Ohne Referenz gäbe es bis zum ersten "set_home"
            # keine Korrektur -- und den Knopf drückt erfahrungsgemäß
            # niemand, bevor etwas schiefgeht.
            asyncio.create_task(_referenz_nachholen(localizer, odometry))
        diagnose.start()

        status_task = asyncio.create_task(
            _status_loop(
                hub, watchdog, driver, odometry, homing, localizer, guard,
                erkunder, erkennung, lidar, personen, interaktion, stimme, sprache,
            )
        )
        sprach_task = asyncio.create_task(_sprach_loop())
        odo_task = asyncio.create_task(_odometry_loop(watchdog, odometry))
        match_task = asyncio.create_task(_match_loop(localizer))
        hw_task = asyncio.create_task(
            _hardware_loop(hub, watchdog, driver, homing, erkunder, interaktion)
        )
        try:
            yield
        finally:
            # Reihenfolge ist wichtig: erst Statusmeldungen beenden, dann
            # Watchdog (der hält an), dann die Hardware trennen.
            status_task.cancel()
            odo_task.cancel()
            match_task.cancel()
            hw_task.cancel()
            sprach_task.cancel()
            homing.abbrechen("Server wird beendet")
            erkunder.abbrechen("Server wird beendet")
            interaktion.abbrechen("Server wird beendet")
            await watchdog.shutdown()
            personen.stop()
            pose.stop()
            mukke.stoppen()
            sprache.stop()
            stimme.stop()
            for kamera in kameras.values():
                kamera.stop()
            if lidar is not None:
                lidar.stop()
            # Vor dem Trennen beenden, sonst meldete sie das Abschalten
            # noch als "Antrieb verloren".
            diagnose.stop()
            driver.disconnect()

    app = FastAPI(title="ROSMASTER X3 Hilfsroboter", lifespan=lifespan)
    # Andere Oberflächen sollen die HTTP-Endpunkte aus dem Browser abrufen
    # können, auch wenn sie nicht von diesem Server ausgeliefert werden
    # (siehe docs/api.md). Eine Anmeldung gibt es ohnehin nicht -- wer im
    # WLAN ist, kann den Roboter steuern.
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["GET"])

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        await hub.add(ws)
        # Zuerst die Grenzwerte: die App rechnet ihre Reglerstellung darauf
        # um und muss sie deshalb kennen, bevor der erste Befehl rausgeht.
        await ws.send_json(
            limits_message(config.MAX_SPEED_X, config.MAX_SPEED_Y, config.MAX_SPEED_OMEGA)
        )
        # Der Server hält die Einstellung, nicht der Browser: sonst hätte
        # jeder Tab seine eigene Vorstellung davon, was gerade gilt.
        await ws.send_json(collision_message(guard.zustand()))
        try:
            while True:
                msg = await ws.receive_json()
                reply = await _handle(
                    msg, watchdog, driver, hub, odometry, homing, localizer, guard,
                    erkunder, interaktion, stimme, sprache,
                )
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
                erkunder.abbrechen("Verbindung verloren")
                interaktion.abbrechen("Verbindung verloren")
                watchdog.stop()

    # --- Kamera ---
    #
    # MJPEG: eine nie endende HTTP-Antwort aus aneinandergehängten Bildern.
    # Der Browser zeigt das direkt in einem <img> an -- ohne JavaScript und
    # ohne zusätzliche Bibliothek.

    GRENZE = "frame"

    def _stream(camera: CameraService | None):
        if camera is None:
            return Response("Kamera abgeschaltet", status_code=503)

        async def bilder():
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
                # Doppelt so oft nachsehen wie Bilder kommen: sonst liegt
                # jedes neue Bild im Mittel eine halbe Periode herum. Jedes
                # Mal neu gelesen, weil die Bildrate verstellbar ist.
                await asyncio.sleep(0.5 / max(1, config.CAMERA_FPS))

        return StreamingResponse(
            bilder(),
            media_type=f"multipart/x-mixed-replace; boundary={GRENZE}",
            # Ohne das puffern manche Proxys den Stream, bis nichts mehr
            # ankommt -- das Bild bliebe dann einfach schwarz.
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

    def _unbekannt(name: str):
        return Response(
            f"Keine Kamera {name!r} -- vorhanden: {', '.join(kameras) or 'keine'}",
            status_code=404,
        )

    # Ohne Kameranamen: die Standardkamera (config.CAMERA_DEFAULT).

    @app.get("/camera/stream")
    async def camera_stream():
        return _stream(camera)

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
        return _snapshot(camera)

    # Mit Kameranamen: jede Kamera einzeln.

    @app.get("/cameras")
    async def cameras_liste():
        """Alle Kameras mit Rolle, Zustand und Adressen ihrer Bilder."""
        return {
            "standard": camera.name if camera is not None else None,
            "personen": personen.status().get("kamera"),
            "kameras": [
                {
                    **k.status(),
                    "stream": f"/camera/{name}/stream",
                    "snapshot": f"/camera/{name}/snapshot",
                }
                for name, k in kameras.items()
            ],
        }

    @app.get("/camera/{name}/stream")
    async def camera_stream_name(name: str):
        if name not in kameras:
            return _unbekannt(name)
        return _stream(kameras[name])

    @app.get("/camera/{name}/snapshot")
    async def camera_snapshot_name(name: str):
        if name not in kameras:
            return _unbekannt(name)
        return _snapshot(kameras[name])

    @app.get("/camera/{name}/status")
    async def camera_status_name(name: str):
        if name not in kameras:
            return _unbekannt(name)
        return kameras[name].status()

    # --- Lidar ---

    @app.get("/lidar/status")
    async def lidar_status():
        """Was der Lidar tut und ob die Ortung damit arbeitet."""
        if lidar is None:
            return {"lidar": None, "hinweis": "LIDAR_ENABLED ist False"}
        return {**lidar.status(), "ortung": localizer.status()}

    @app.get("/lidar/scan")
    async def lidar_scan():
        """Die letzte Umdrehung als Rohdaten -- für die Anzeige in der App."""
        if lidar is None:
            return {"punkte": [], "hinweis": "kein Lidar"}
        scan = lidar.latest()
        if scan is None or time.monotonic() - scan.zeit > 3.0:
            # Der App sagen, warum nichts kommt -- "keine Daten" allein
            # lässt einen raten, ob der Lidar, die App oder das WLAN hakt.
            st = lidar.status()
            grund = st["letzter_fehler"] or (
                "wartet auf erste Umdrehung" if st["geoeffnet"] else "nicht geöffnet"
            )
            return {"punkte": [], "hinweis": f"Lidar: {grund}"}
        # Gerundet übertragen: zwei Nachkommastellen bei Metern sind ein
        # Zentimeter, feiner braucht es die Anzeige nicht -- und es
        # halbiert die Datenmenge.
        return {
            "punkte": [
                [round(w, 1), round(r, 2)] for w, r in zip(scan.winkel, scan.entfernungen)
            ],
            "alter_s": round(time.monotonic() - scan.zeit, 2),
        }

    # --- Autonomes Fahren ---

    @app.get("/autonomie/status")
    async def autonomie_status():
        """Erkunder, erkannte Hindernisse und Personenerkennung auf einen Blick."""
        return _autonomie(erkunder, erkennung, lidar, personen)

    # --- Einstellungen ---
    #
    # Lesen per GET wie alles andere; Ändern per POST, weil es Zustand auf
    # dem Roboter verändert (und gespeichert wird). Das CORS-Setup oben
    # erlaubt nur GET -- ändern kann also nur, wer die Seite vom Roboter
    # selbst geladen hat, nicht jede beliebige Webseite im Browser.

    async def _einstellungen_wirken(geaendert: list[str]) -> None:
        """Was nicht von selbst bei der nächsten Verwendung greift, anstoßen."""
        if any(k.startswith("MAX_SPEED_") for k in geaendert):
            # Die App rechnet ihre Regler auf die Grenzen um.
            await hub.broadcast(
                limits_message(config.MAX_SPEED_X, config.MAX_SPEED_Y, config.MAX_SPEED_OMEGA)
            )
        if "COLLISION_ENABLED" in geaendert or "COLLISION_DISTANCE_M" in geaendert:
            # Der Wächter hält seinen Zustand selbst (der Schalter in der
            # App ändert ihn ohne config) -- deshalb ausdrücklich übergeben.
            guard.einstellen(
                config.COLLISION_ENABLED if "COLLISION_ENABLED" in geaendert else None,
                config.COLLISION_DISTANCE_M if "COLLISION_DISTANCE_M" in geaendert else None,
            )
            await hub.broadcast(collision_message(guard.zustand()))
        if "CAMERA_RESOLUTION" in geaendert:
            for kamera in kameras.values():
                kamera.neu_oeffnen()

    async def _einstellungen_antwort(geaendert: list[str]) -> dict[str, Any]:
        await _einstellungen_wirken(geaendert)
        neustart = settings.neustart_noetig(geaendert)
        if geaendert:
            text = f"Einstellungen geändert: {', '.join(geaendert)}"
            if neustart:
                text += " (wirkt nach Neustart)"
            await hub.broadcast(alert_message("info", text))
        return {**settings.liste(), "geaendert": geaendert, "neustart_noetig": neustart}

    @app.get("/settings")
    async def settings_lesen():
        """Alle verstellbaren Einstellungen nach Blöcken, mit Grenzen."""
        return settings.liste()

    @app.post("/settings")
    async def settings_setzen(request: Request):
        """Body: {"KEY": wert, ...}. Alle oder keiner werden übernommen."""
        try:
            neu = await request.json()
        except ValueError:
            return JSONResponse({"fehler": "JSON erwartet"}, status_code=400)
        if not isinstance(neu, dict):
            return JSONResponse({"fehler": "Objekt {KEY: wert} erwartet"}, status_code=400)
        try:
            geaendert = settings.setzen(neu)
        except settings.EinstellungsFehler as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await _einstellungen_antwort(geaendert)

    @app.post("/settings/reset")
    async def settings_zuruecksetzen(request: Request):
        """Body: {"keys": [...]} für einzelne, leer oder {} für alle."""
        try:
            daten = await request.json()
        except ValueError:
            daten = {}
        keys = daten.get("keys") if isinstance(daten, dict) else None
        try:
            geaendert = settings.zuruecksetzen(keys)
        except settings.EinstellungsFehler as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await _einstellungen_antwort(geaendert)

    # --- Interaktion ---

    @app.get("/interaktion/status")
    async def interaktion_status():
        """Modus, verfolgte Person, letzte Geste und ob die Posenerkennung läuft."""
        return {**interaktion.zustand(), "pose": pose.status()}

    @app.get("/interaktion/pose")
    async def interaktion_pose():
        """Erkannte Körperpunkte zum Einblenden über dem Kamerabild.

        Leer, solange die Interaktion aus ist -- dann rechnet der Hailo nicht.
        """
        return {"personen": pose.personen(), "skelett": SKELETT}

    # --- Diagnose ---

    @app.get("/diagnose/status")
    async def diagnose_status():
        """Gesamtzustand, Probleme mit Lösungsvorschlag, Messwerte, Zeitleiste."""
        return diagnose.status()

    @app.get("/diagnose/protokoll")
    async def diagnose_protokoll(nach: int = 0):
        """Server-Log ab Zeile ``nach`` (exklusiv), damit nur Neues kommt."""
        zeilen, letzte_id = log_puffer.zeilen(nach)
        return {"zeilen": zeilen, "letzte_id": letzte_id}

    @app.post("/diagnose/quittieren")
    async def diagnose_quittieren(request: Request):
        """Body: {"id": "..."} für ein Problem oder {"alle": true}."""
        try:
            daten = await request.json()
        except ValueError:
            daten = None
        if not isinstance(daten, dict) or not (daten.get("alle") or daten.get("id")):
            return JSONResponse({"fehler": '{"id": ...} oder {"alle": true} erwartet'},
                                status_code=400)
        n = diagnose.quittieren(None if daten.get("alle") else str(daten["id"]))
        return {"quittiert": n}

    # --- Musik ("MUKKE") ---
    #
    # HTTP wie die Einstellungen: Ändern per POST, damit fremde Webseiten
    # (CORS erlaubt ihnen nur GET) nichts auf dem Lautsprecher abspielen.

    async def _mukke_json(request: Request) -> dict:
        try:
            daten = await request.json()
        except ValueError:
            return {}
        return daten if isinstance(daten, dict) else {}

    async def _mukke(aktion) -> Any:
        """Aktion ausführen, Fehler als 400, sonst den neuen Zustand."""
        try:
            # mpv starten und per Socket fragen dauert kurz -- nicht in der
            # Ereignisschleife, sonst stockt die Steuerung.
            await asyncio.to_thread(aktion)
        except MukkeFehler as exc:
            return JSONResponse({"fehler": str(exc)}, status_code=400)
        return await asyncio.to_thread(mukke.status)

    @app.get("/mukke/status")
    async def mukke_status():
        """Läuft etwas, welcher Titel, Lautstärke, und ob mpv/yt-dlp da sind."""
        return await asyncio.to_thread(mukke.status)

    @app.post("/mukke/play")
    async def mukke_play(request: Request):
        """Body: {"url": "https://youtu.be/..."}. Ersetzt, was gerade läuft."""
        url = (await _mukke_json(request)).get("url", "")
        antwort = await _mukke(lambda: mukke.abspielen(url))
        if not isinstance(antwort, JSONResponse):
            await hub.broadcast(alert_message("info", "MUKKE: spielt ab"))
        return antwort

    @app.post("/mukke/stop")
    async def mukke_stop():
        return await _mukke(mukke.stoppen)

    @app.post("/mukke/pause")
    async def mukke_pause():
        """Pause an/aus."""
        return await _mukke(mukke.pause_umschalten)

    @app.post("/mukke/volume")
    async def mukke_volume(request: Request):
        """Body: {"lautstaerke": 0..100}."""
        wert = (await _mukke_json(request)).get("lautstaerke")
        return await _mukke(lambda: mukke.lautstaerke_setzen(wert))

    # --- Stimme ---

    @app.get("/stimme/status")
    async def stimme_status():
        """Welche Sprachausgabe läuft (Piper, espeak-ng, nur Text), letzter Satz, Fehler."""
        return stimme.status()

    # --- Sprachbefehle ---

    @app.get("/sprache/status")
    async def sprache_status():
        """Mikrofon, Modell, Weckwort, zuletzt Gehörtes und erkannte Befehle."""
        return sprache.status()

    async def _sprach_loop() -> None:
        """Erkannte Sprachbefehle abholen und ausführen.

        Die Erkennung läuft im eigenen Thread; gefahren wird aber nur hier
        im Event-Loop -- über dieselben Wege wie mit den Knöpfen der App.
        """
        while True:
            await asyncio.sleep(0.1)
            for eintrag in sprache.befehle_abholen():
                try:
                    await _sprachbefehl(eintrag["befehl"], eintrag["text"])
                except Exception:  # ein kaputter Befehl darf das Ohr nicht abschalten
                    log.exception("Sprachbefehl %s fehlgeschlagen", eintrag["befehl"])

    async def _befehlen(typ: str) -> bool:
        """App-Befehl ausführen. False, wenn er mit einer Warnung abgelehnt wurde."""
        antwort = await _handle(
            {"type": typ}, watchdog, driver, hub, odometry, homing, localizer, guard,
            erkunder, interaktion, stimme, sprache,
        )
        if antwort is None:
            return True
        await hub.broadcast(antwort)
        if antwort.get("level") == "warning":
            # Wer spricht, schaut nicht unbedingt auf die App.
            stimme.sagen("Das geht gerade nicht.")
            return False
        return True

    async def _interaktion_mit(modus: str, text: str, satz: str) -> None:
        """Herkommen/Weggehen per Sprache; schaltet die Interaktion nötigenfalls ein."""
        if interaktion.aktiv:
            interaktion.wechseln_zu(modus, text, sagen=satz)
            return
        if not interaktion.verfuegbar:
            await _befehlen("interaction_start")  # liefert die passende Warnung
            return
        homing.abbrechen("Interaktion gestartet")
        erkunder.abbrechen("Interaktion gestartet")
        if interaktion.start(modus):
            await hub.broadcast(alert_message("info", f"Interaktion an -- {text}"))

    async def _sprachbefehl(befehl: str, text: str) -> None:
        await hub.broadcast(alert_message("info", f"Sprachbefehl: {text}"))
        if befehl == sp.STOPP:
            if interaktion.aktiv:
                # Anhalten, aber weiter zuhören und zuschauen: "Robi, folgen"
                # soll danach gleich wieder gehen.
                homing.abbrechen("Not-Stopp")
                erkunder.abbrechen("Not-Stopp")
                interaktion.wechseln_zu(vh.BEREIT, "angehalten (per Sprache)")
                watchdog.stop()
                stimme.sagen("Okay, ich halte an.", vorrang=True)
            else:
                await _befehlen("stop")
        elif befehl == sp.FOLGEN:
            await _befehlen("follow")
        elif befehl == sp.HERKOMMEN:
            await _interaktion_mit(vh.HERKOMMEN, "kommt her (per Sprache)", "Ich komme.")
        elif befehl == sp.WEGGEHEN:
            await _interaktion_mit(vh.WEGGEHEN, "nimmt Abstand (per Sprache)", "Ich fahre zurück.")
        elif befehl == sp.START:
            if interaktion.aktiv:
                stimme.sagen("Ja? Ich bin da.")
            else:
                await _befehlen("interaction_start")
        elif befehl == sp.AUS:
            homing.abbrechen("vom Nutzer beendet")
            erkunder.abbrechen("vom Nutzer beendet")
            await _befehlen("interaction_stop")
            stimme.sagen("Gute Nacht.")
        elif befehl == sp.NACH_HAUSE:
            if await _befehlen("home") and homing.aktiv:
                stimme.sagen("Ich fahre nach Hause.")
        elif befehl == sp.ERKUNDEN:
            if await _befehlen("explore") and erkunder.aktiv:
                stimme.sagen("Ich schaue mich mal um.")
        elif befehl == sp.MUSIK_AUS:
            try:
                await asyncio.to_thread(mukke.stoppen)
            except MukkeFehler as exc:
                await hub.broadcast(alert_message("warning", str(exc)))
            stimme.sagen("Musik ist aus.")
        elif befehl == sp.LADADA:
            # Nichts ansagen: Die Stimme würde das Lied gleich wieder
            # unterbrechen.
            laufend = await asyncio.to_thread(mukke.status)
            if laufend["laeuft"] and laufend["url"] == config.SPRACHE_LADADA_URL:
                # Das Mikrofon hört das Lied mit -- singt es selbst "la la
                # la", finge es sonst immer wieder von vorn an.
                return
            try:
                await asyncio.to_thread(mukke.abspielen, config.SPRACHE_LADADA_URL)
            except MukkeFehler as exc:
                await hub.broadcast(alert_message("warning", str(exc)))
                stimme.sagen("Das geht gerade nicht.")

    # Die Web-App direkt mitausliefern, damit eine Adresse genügt.
    if WEBAPP_DIR.is_dir():
        app.mount("/static", _OhneVeraltetenCache(directory=WEBAPP_DIR), name="static")

        @app.get("/")
        async def index() -> HTMLResponse:
            return _seite("index.html")

        @app.get("/einstellungen")
        async def einstellungen_seite() -> HTMLResponse:
            return _seite("einstellungen.html")

        # /logs als Zweitname -- danach sucht man, wenn etwas schiefgeht.
        @app.get("/diagnose")
        @app.get("/logs")
        async def diagnose_seite() -> HTMLResponse:
            return _seite("diagnose.html")
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
    localizer: Localizer,
    guard: CollisionGuard,
    erkunder: Erkunder | None = None,
    interaktion: Interaktion | None = None,
    stimme: Stimme | None = None,
    sprache: Spracherkennung | None = None,
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

        # Beim autonomen Fahren genauso: Eingreifen beendet es, die
        # Leerlauf-Nullen der App dürfen es nicht ausbremsen.
        if erkunder is not None and erkunder.aktiv:
            if cmd.is_stop():
                return None
            # Die Meldung mit Grund schickt _status_loop.
            erkunder.abbrechen("von Hand übersteuert")

        # Interaktion ebenso: wer von Hand lenkt, will nicht, dass der
        # Roboter gleichzeitig einer Person folgt.
        if interaktion is not None and interaktion.aktiv:
            if cmd.is_stop():
                return None
            interaktion.abbrechen("von Hand übersteuert")

        erlaubt, hindernis = guard.pruefen(cmd)
        if hindernis is not None and guard.melden():
            await hub.broadcast(
                alert_message(
                    "warning", f"Hindernis in {hindernis:.2f} m -- Fahrt gesperrt"
                )
            )
        watchdog.submit(erlaubt)
        return None  # Status geht ohnehin im Takt raus

    if kind == "collision":
        guard.einstellen(msg.get("enabled"), msg.get("distance"))
        await hub.broadcast(collision_message(guard.zustand()))
        return None

    if kind == "stop":
        homing.abbrechen("Not-Stopp")
        if erkunder is not None:
            erkunder.abbrechen("Not-Stopp")
        if interaktion is not None:
            interaktion.abbrechen("Not-Stopp")
        watchdog.stop()
        if stimme is not None:
            # Vorrang: alles, was noch in der Schlange stand, ist überholt.
            stimme.sagen("Halt.", vorrang=True)
        return None

    if kind == "explore":
        if erkunder is None or not erkunder.verfuegbar:
            return alert_message("warning", "Autonomes Fahren geht nur mit Lidar")
        if erkunder.aktiv:
            return None
        homing.abbrechen("autonomes Fahren gestartet")
        if interaktion is not None:
            interaktion.abbrechen("autonomes Fahren gestartet")
        if erkunder.start():
            await hub.broadcast(alert_message("info", "Autonomes Fahren gestartet"))
        return None

    if kind == "explore_stop":
        if erkunder is not None:
            erkunder.abbrechen("vom Nutzer beendet")  # Meldung kommt aus _status_loop
        watchdog.stop()
        return None

    if kind in ("interaction_start", "follow"):
        if interaktion is None or not interaktion.verfuegbar:
            return alert_message(
                "warning",
                "Interaktion geht nur mit Lidar und Posenerkennung (siehe /interaktion/status)",
            )
        if not interaktion.aktiv:
            # Nur ein Programm lenkt: Erkunder und Heimfahrt weichen.
            homing.abbrechen("Interaktion gestartet")
            if erkunder is not None:
                erkunder.abbrechen("Interaktion gestartet")
        if kind == "follow":
            war_aktiv = interaktion.aktiv
            if interaktion.folgen() and not war_aktiv:
                await hub.broadcast(alert_message("info", "Interaktion an -- folge dir"))
        elif interaktion.start():
            await hub.broadcast(
                alert_message("info", "Interaktion an -- ich reagiere auf Handzeichen")
            )
        return None

    if kind == "follow_stop":
        if interaktion is not None:
            interaktion.folgen_stop()
        return None

    if kind == "interaction_stop":
        if interaktion is not None:
            interaktion.abbrechen("vom Nutzer beendet")  # Meldung kommt aus _status_loop
        watchdog.stop()
        return None

    if kind == "home":
        if not odometry.misst_tatsaechlich:
            log.warning("Heimfahrt ohne Bewegungsmessung -- Position ist geschätzt")
        abstand = odometry.distanz_zum_start()
        if erkunder is not None:
            erkunder.abbrechen("Heimfahrt gestartet")
        if interaktion is not None:
            interaktion.abbrechen("Heimfahrt gestartet")
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
        if erkunder is not None:
            erkunder.abbrechen("Startpunkt wurde verschoben")
        if interaktion is not None:
            interaktion.abbrechen("Startpunkt wurde verschoben")
        # Nur in der Simulation vorhanden: dort muss auch die gedachte
        # Wirklichkeit auf den Startpunkt zurück, sonst stünde der Roboter
        # außerhalb des simulierten Raums.
        nullen = getattr(driver, "wahre_pose_nullen", None)
        if nullen is not None:
            nullen()
        odometry.reset()
        # Die alte Referenzaufnahme gehört zum alten Startpunkt und würde
        # die Position sonst dorthin zurückziehen.
        eingemessen = localizer.referenz_aufnehmen()
        text = (
            "Startpunkt gesetzt und per Lidar eingemessen"
            if eingemessen
            else "Startpunkt gesetzt (ohne Lidar -- Position wird nur geschätzt)"
        )
        await hub.broadcast(alert_message("info", text))
        return None

    if kind == "say":
        if stimme is None:
            return alert_message("warning", "Keine Sprachausgabe")
        text = msg.get("text")
        if not isinstance(text, str) or not text.strip():
            return error_message("say braucht ein Feld text")
        stimme.sagen(text, vorrang=bool(msg.get("vorrang", False)))
        if not stimme.verfuegbar:
            return alert_message(
                "warning", "Kein Lautsprecher oder keine Sprachsynthese (siehe /stimme/status)"
            )
        return None

    if kind == "say_stop":
        if stimme is not None:
            stimme.verstummen()
        return None

    if kind == "speech":
        # Wie ins Mikrofon gesprochen: zum Ausprobieren ohne Mikrofon und
        # in --mock. Ausgeführt wird im _sprach_loop, wie beim echten Hören.
        if sprache is None:
            return alert_message("warning", "Keine Spracherkennung")
        text = msg.get("text")
        if not isinstance(text, str) or not text.strip():
            return error_message("speech braucht ein Feld text")
        if sprache.hoeren(text) is None:
            return alert_message(
                "info", f"Kein Befehl erkannt in {text!r} (Weckwort: {config.SPRACHE_WECKWORT})"
            )
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


async def _referenz_nachholen(
    localizer: Localizer, odometry: Odometry, versuche: int = 120
) -> None:
    """Nimmt die Referenzaufnahme, sobald der Lidar die erste Runde hat.

    Großzügige Frist: der Lidar kann beim Hochfahren etwas brauchen, bis er
    sauber antwortet. Eingemessen wird aber nur, solange der Roboter noch am
    Startpunkt steht -- eine Aufnahme von woanders als "Startpunkt" zu
    speichern, würde jede spätere Heimfahrt an die falsche Stelle führen.
    """
    for _ in range(versuche):
        await asyncio.sleep(1.0)
        if localizer.hat_referenz:
            return  # inzwischen per "set_home" gesetzt
        x, y, _winkel = odometry.pose()
        if math.hypot(x, y) > 0.05:
            break
        if localizer.referenz_aufnehmen():
            return
    log.warning(
        "Startpunkt konnte nicht eingemessen werden -- Ortung ohne "
        "Lidar-Korrektur, bis \"Startpunkt hier setzen\" gedrückt wird"
    )


async def _match_loop(localizer: Localizer) -> None:
    """Gleicht die Position regelmäßig mit der Referenzaufnahme ab."""
    if not localizer.verfuegbar:
        return
    try:
        while True:
            # Jedes Mal neu lesen: auf der Einstellungsseite verstellbar.
            await asyncio.sleep(1.0 / max(0.1, config.MATCH_HZ))
            # Der Abgleich rechnet mit einigen hundert Punkten und würde
            # die Ereignisschleife sonst kurz blockieren -- das macht sich
            # in ruckelnder Steuerung bemerkbar.
            await asyncio.to_thread(localizer.abgleichen)
    except asyncio.CancelledError:
        raise
    except Exception:
        log.exception("Lidar-Abgleich abgestürzt -- Ortung nur noch per Odometrie")


async def _hardware_loop(
    hub: ConnectionHub,
    watchdog: SafetyWatchdog,
    driver: DriveBase,
    homing: Homing,
    erkunder: Erkunder | None = None,
    interaktion: Interaktion | None = None,
) -> None:
    """Hält die Verbindung zum Antrieb am Leben.

    Nach einem USB-Aussetzer baut der Treiber sie selbst neu auf; hier wird
    dafür gesorgt, dass bis dahin nichts weiterfährt und die App es erfährt.
    """
    verbunden = driver.connected
    try:
        while True:
            await asyncio.sleep(config.RECONNECT_CHECK_S)
            # Neu verbinden kann eine Weile dauern -- nicht in der
            # Ereignisschleife, sonst stockt die Steuerung.
            await asyncio.to_thread(driver.ueberwachen)

            jetzt = driver.connected
            if jetzt == verbunden:
                continue
            verbunden = jetzt
            if not jetzt:
                homing.abbrechen("Verbindung zum Antrieb verloren")
                if erkunder is not None:
                    erkunder.abbrechen("Verbindung zum Antrieb verloren")
                if interaktion is not None:
                    # Nicht beenden, nur anhalten: Die Gestensteuerung bleibt
                    # an und ist nach dem Neuverbinden sofort wieder bereit.
                    # Früher war sie nach jedem USB-Reset aus.
                    interaktion.antrieb_verloren()
                watchdog.stop()
                await hub.broadcast(
                    alert_message(
                        "warning",
                        "Verbindung zum Antrieb verloren -- verbinde neu. "
                        "Häufigste Ursache: Akku schwach.",
                    )
                )
            else:
                await hub.broadcast(alert_message("info", "Antrieb wieder verbunden"))
    except asyncio.CancelledError:
        raise
    except Exception:
        log.exception("Verbindungsüberwachung abgestürzt -- kein Neuverbinden mehr")


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
    localizer: Localizer,
    guard: CollisionGuard,
    erkunder: Erkunder | None = None,
    erkennung: Hinderniserkennung | None = None,
    lidar: LidarService | None = None,
    personen: Personenerkennung | None = None,
    interaktion: Interaktion | None = None,
    stimme: Stimme | None = None,
    sprache: Spracherkennung | None = None,
) -> None:
    """Schickt regelmäßig den Zustand an alle Clients."""
    zuletzt_aktiv = False
    zuletzt_erkundet = False
    zuletzt_interagiert = False
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

            # Ebenso das Ende des autonomen Fahrens (Lidar weg, festgefahren).
            if erkunder is not None:
                if zuletzt_erkundet and not erkunder.aktiv and erkunder.letzter_grund:
                    await hub.broadcast(
                        alert_message(
                            "info", f"Autonomes Fahren beendet: {erkunder.letzter_grund}"
                        )
                    )
                zuletzt_erkundet = erkunder.aktiv

            # Interaktion: erkannte Gesten und Moduswechsel als Hinweis,
            # dazu das Ende (z. B. Lidar weg).
            if interaktion is not None:
                for text in interaktion.meldungen_abholen():
                    await hub.broadcast(alert_message("info", text))
                if zuletzt_interagiert and not interaktion.aktiv and interaktion.letzter_grund:
                    await hub.broadcast(
                        alert_message("info", f"Interaktion beendet: {interaktion.letzter_grund}")
                    )
                zuletzt_interagiert = interaktion.aktiv

            if hub.count:
                autonomie = (
                    _autonomie(erkunder, erkennung, lidar, personen)
                    if erkunder is not None and erkennung is not None
                    else None
                )
                await hub.broadcast(
                    state_message(
                        watchdog.current_command,
                        driver.name,
                        driver.connected,
                        pose=odometry.pose(),
                        homing=homing.aktiv,
                        pose_gemessen=odometry.misst_tatsaechlich,
                        collision=guard.zustand(),
                        autonomie=autonomie,
                        interaktion=interaktion.zustand() if interaktion is not None else None,
                        stimme=stimme.zustand() if stimme is not None else None,
                        sprache=sprache.zustand() if sprache is not None else None,
                    )
                )
    except asyncio.CancelledError:
        raise


def _autonomie(
    erkunder: Erkunder,
    erkennung: Hinderniserkennung,
    lidar: LidarService | None,
    personen: Personenerkennung | None,
) -> dict[str, Any]:
    """Zustand für state.autonomie und /autonomie/status.

    Die Hindernisse kommen auch ohne laufende Erkundung mit -- so sieht man
    schon vor dem Start, was der Roboter erkennt.
    """
    scan = lidar.latest() if lidar is not None else None
    if scan is not None and time.monotonic() - scan.zeit > 3.0:
        scan = None  # alte Runde nicht als aktuelle Lage ausgeben
    winkel = personen.winkelbereiche() if personen is not None else []
    hindernisse = erkennung.aktuell(scan, winkel) if scan is not None else []
    return {
        **erkunder.zustand(),
        # Die nächsten genügen für Anzeige und Liste; alle würden die
        # Statusnachricht in vollen Räumen unnötig aufblähen.
        "hindernisse": [h.als_dict() for h in hindernisse[:12]],
        "personen": personen.boxen() if personen is not None else [],
        "personenerkennung": personen.status() if personen is not None else None,
    }
