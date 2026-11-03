"""Startpunkt der Roboter-Software.

    python -m rosmaster_assist.main --mock    # ohne Hardware, zum Entwickeln
    python -m rosmaster_assist.main           # auf dem Roboter
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import shutil
import subprocess
import sys

import uvicorn

from . import config
from .api import create_app
from .drive import create_driver

log = logging.getLogger("rosmaster_assist")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ROSMASTER X3 Hilfsroboter")
    p.add_argument(
        "--mock",
        action="store_true",
        help="Ohne Hardware starten (Fahrbefehle nur ins Log)",
    )
    p.add_argument("--host", default=config.HOST, help=f"Standard: {config.HOST}")
    p.add_argument("--port", type=int, default=config.PORT, help=f"Standard: {config.PORT}")
    p.add_argument("--debug", action="store_true", help="Ausführliche Logausgabe")
    return p.parse_args(argv)


def _zertifikat() -> tuple[str, str] | None:
    """Selbst unterschriebenes Zertifikat für HTTPS; erzeugt es beim ersten Mal.

    None, wenn es nicht geht -- dann läuft eben nur HTTP.
    """
    ordner = os.path.expanduser(config.HTTPS_ORDNER)
    zert, schluessel = os.path.join(ordner, "cert.pem"), os.path.join(ordner, "key.pem")
    if os.path.isfile(zert) and os.path.isfile(schluessel):
        return zert, schluessel
    if shutil.which("openssl") is None:
        log.warning("HTTPS aus: openssl fehlt (Durchsage vom Handy geht dann nicht)")
        return None
    try:
        os.makedirs(ordner, exist_ok=True)
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650",
             "-subj", "/CN=rosmaster", "-keyout", schluessel, "-out", zert],
            check=True, capture_output=True, timeout=60,
        )
        # Unter sudo gestartet: Die Dateien sollen trotzdem dem Benutzer
        # gehören, sonst kann ein späterer Start ohne sudo sie nicht lesen.
        uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
        if uid and gid and hasattr(os, "chown"):
            for pfad in (ordner, zert, schluessel):
                os.chown(pfad, int(uid), int(gid))
    except (OSError, subprocess.SubprocessError) as exc:
        log.warning("HTTPS aus: Zertifikat ließ sich nicht erzeugen (%s)", exc)
        return None
    log.info("HTTPS: neues Zertifikat in %s", ordner)
    return zert, schluessel


class _NebenServer(uvicorn.Server):
    """Zweiter Zugang (HTTPS) zur selben App.

    Fängt keine Signale ab: Strg+C gehört dem Haupt-Server, der beim Beenden
    die Motoren anhält. Der Neben-Server wird danach einfach abgeräumt.
    """

    @contextlib.contextmanager
    def capture_signals(self):
        yield


async def _neben_sicher(neben: uvicorn.Server) -> None:
    """HTTPS darf ausfallen, ohne den Server mitzureißen.

    Ist der Port belegt, beendet uvicorn mit sys.exit(1) -- in einer Aufgabe
    risse das den ganzen Prozess mit, samt Fahren und Lidar (am 03.11.
    passiert, als ein zweiter Server lief). Dann eben nur HTTP.
    """
    try:
        await neben.serve()
    except (SystemExit, OSError) as exc:
        log.warning("HTTPS aus: Port %d geht nicht (%s) -- Durchsage vom Handy fehlt, "
                    "sonst läuft alles", config.HTTPS_PORT, exc)


async def _mit_https(haupt: uvicorn.Server, neben: uvicorn.Server) -> None:
    aufgabe = asyncio.create_task(_neben_sicher(neben))
    try:
        await haupt.serve()
    finally:
        aufgabe.cancel()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s  %(levelname)-7s %(name)-28s %(message)s",
        datefmt="%H:%M:%S",
    )

    try:
        driver = create_driver(use_mock=args.mock)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 1

    if args.mock:
        log.warning("MOCK-MODUS -- es bewegt sich nichts")

    # Ohne Hardware auch kein Kamerabild erwarten: dann das Testbild.
    app = create_app(driver, use_mock_camera=args.mock)

    host_text = "localhost" if args.host == "0.0.0.0" else args.host
    log.info("Web-App: http://%s:%d", host_text, args.port)

    # log_config=None: uvicorn soll unser Logging-Format nicht überschreiben.
    # timeout_graceful_shutdown: offene Streams nach kurzer Zeit kappen,
    # damit Strg+C immer beendet und die Motoren sicher stoppen.
    gemeinsam = dict(log_config=None, timeout_graceful_shutdown=config.SHUTDOWN_TIMEOUT_S)
    https = _zertifikat() if config.HTTPS_PORT else None
    if https is None:
        uvicorn.run(app, host=args.host, port=args.port, **gemeinsam)
        return 0

    haupt = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, **gemeinsam))
    # lifespan="off": Hochfahren und Aufräumen (Antrieb, Lidar, Kamera) nur
    # einmal, über den Haupt-Server -- beide teilen sich dieselbe App.
    neben = _NebenServer(uvicorn.Config(
        app, host=args.host, port=config.HTTPS_PORT, lifespan="off",
        ssl_certfile=https[0], ssl_keyfile=https[1], **gemeinsam,
    ))
    log.info("Web-App mit Mikrofon (Durchsage): https://%s:%d", host_text, config.HTTPS_PORT)
    try:
        asyncio.run(_mit_https(haupt, neben))
    except KeyboardInterrupt:
        pass  # wie uvicorn.run: Strg+C ist ein normales Ende
    return 0


if __name__ == "__main__":
    sys.exit(main())
