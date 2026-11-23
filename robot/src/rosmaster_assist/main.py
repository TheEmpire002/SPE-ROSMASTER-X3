"""Entry point of the robot software.

    python -m rosmaster_assist.main --mock    # without hardware, for development
    python -m rosmaster_assist.main           # on the robot
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
    p = argparse.ArgumentParser(description="ROSMASTER X3 helper robot")
    p.add_argument(
        "--mock",
        action="store_true",
        help="Start without hardware (drive commands only go to the log)",
    )
    p.add_argument("--host", default=config.HOST, help=f"Default: {config.HOST}")
    p.add_argument("--port", type=int, default=config.PORT, help=f"Default: {config.PORT}")
    p.add_argument("--debug", action="store_true", help="Verbose log output")
    return p.parse_args(argv)


def _certificate() -> tuple[str, str] | None:
    """Self-signed certificate for HTTPS; creates it the first time.

    None if that is not possible -- then only HTTP runs.
    """
    folder = os.path.expanduser(config.HTTPS_FOLDER)
    cert, key = os.path.join(folder, "cert.pem"), os.path.join(folder, "key.pem")
    if os.path.isfile(cert) and os.path.isfile(key):
        return cert, key
    if shutil.which("openssl") is None:
        log.warning("HTTPS off: openssl missing (announcements from the phone will not work)")
        return None
    try:
        os.makedirs(folder, exist_ok=True)
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650",
             "-subj", "/CN=rosmaster", "-keyout", key, "-out", cert],
            check=True, capture_output=True, timeout=60,
        )
        # Started under sudo: the files should still belong to the user,
        # otherwise a later start without sudo cannot read them.
        uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
        if uid and gid and hasattr(os, "chown"):
            for path in (folder, cert, key):
                os.chown(path, int(uid), int(gid))
    except (OSError, subprocess.SubprocessError) as exc:
        log.warning("HTTPS off: could not create certificate (%s)", exc)
        return None
    log.info("HTTPS: new certificate in %s", folder)
    return cert, key


class _SecondaryServer(uvicorn.Server):
    """Second access point (HTTPS) to the same app.

    Does not capture signals: Ctrl+C belongs to the main server, which stops
    the motors on shutdown. The secondary server is simply cleaned up after that.
    """

    @contextlib.contextmanager
    def capture_signals(self):
        yield


async def _secondary_guarded(secondary: uvicorn.Server) -> None:
    """HTTPS may fail without taking the server down with it.

    If the port is in use, uvicorn exits with sys.exit(1) -- inside a task that
    would take down the whole process, including driving and lidar (happened
    on 03.11. while a second server was running). Then just HTTP.
    """
    try:
        await secondary.serve()
    except (SystemExit, OSError) as exc:
        log.warning("HTTPS off: port %d not usable (%s) -- announcements from the phone "
                    "unavailable, everything else runs", config.HTTPS_PORT, exc)


async def _with_https(main_server: uvicorn.Server, secondary: uvicorn.Server) -> None:
    task = asyncio.create_task(_secondary_guarded(secondary))
    try:
        await main_server.serve()
    finally:
        task.cancel()


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
        log.warning("MOCK MODE -- nothing moves")

    # Without hardware, do not expect a camera image either: use the test image.
    app = create_app(driver, use_mock_camera=args.mock)

    host_text = "localhost" if args.host == "0.0.0.0" else args.host
    log.info("Web app: http://%s:%d", host_text, args.port)

    # log_config=None: uvicorn must not overwrite our logging format.
    # timeout_graceful_shutdown: cut open streams after a short time so that
    # Ctrl+C always exits and the motors stop reliably.
    common = dict(log_config=None, timeout_graceful_shutdown=config.SHUTDOWN_TIMEOUT_S)
    https = _certificate() if config.HTTPS_PORT else None
    if https is None:
        uvicorn.run(app, host=args.host, port=args.port, **common)
        return 0

    main_server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, **common))
    # lifespan="off": startup and cleanup (drive, lidar, camera) only once,
    # via the main server -- both share the same app.
    secondary = _SecondaryServer(uvicorn.Config(
        app, host=args.host, port=config.HTTPS_PORT, lifespan="off",
        ssl_certfile=https[0], ssl_keyfile=https[1], **common,
    ))
    log.info("Web app with microphone (announcements): https://%s:%d", host_text, config.HTTPS_PORT)
    try:
        asyncio.run(_with_https(main_server, secondary))
    except KeyboardInterrupt:
        pass  # like uvicorn.run: Ctrl+C is a normal exit
    return 0


if __name__ == "__main__":
    sys.exit(main())
