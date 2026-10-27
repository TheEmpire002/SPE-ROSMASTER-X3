"""Startpunkt der Roboter-Software.

    python -m rosmaster_assist.main --mock    # ohne Hardware, zum Entwickeln
    python -m rosmaster_assist.main           # auf dem Roboter
"""

from __future__ import annotations

import argparse
import logging
import sys

import uvicorn

from . import config
from .api import create_app
from .drive import create_driver


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


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s  %(levelname)-7s %(name)-28s %(message)s",
        datefmt="%H:%M:%S",
    )
    log = logging.getLogger("rosmaster_assist")

    try:
        driver = create_driver(use_mock=args.mock)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 1

    if args.mock:
        log.warning("MOCK-MODUS -- es bewegt sich nichts")

    # Ohne Hardware auch kein Kamerabild erwarten: dann das Testbild.
    app = create_app(driver, use_mock_camera=args.mock)

    log.info("Web-App: http://%s:%d", "localhost" if args.host == "0.0.0.0" else args.host, args.port)

    # log_config=None: uvicorn soll unser Logging-Format nicht überschreiben.
    uvicorn.run(app, host=args.host, port=args.port, log_config=None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
