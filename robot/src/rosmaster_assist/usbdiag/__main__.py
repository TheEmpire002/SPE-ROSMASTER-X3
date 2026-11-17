"""Start the USB diagnosis: python -m rosmaster_assist.usbdiag [--mock] [--port N]."""

import argparse
import os

from . import config, daemon, store


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="usbdiag", description="USB-Diagnose für den Roboter")
    parser.add_argument("--mock", action="store_true", help="ohne Hardware, mit simulierten Ausfällen")
    parser.add_argument("--port", type=int, default=config.HTTP_PORT, help="Port der Diagnose-Seite")
    parser.add_argument("--log-dir", help="Ordner für Protokoll und Vorfälle")
    args = parser.parse_args(argv)

    if args.mock:
        from .mock import MockSources
        sources = MockSources()
        folder = args.log_dir or os.path.expanduser(config.MOCK_LOG_DIR)
        boot_id = config.MOCK_BOOT_ID
    else:
        sources = daemon.RealSources()
        folder = args.log_dir or config.LOG_DIR
        boot_id = store.read_boot_id()

    st = store.Store(folder, boot_id, config.LOG_FILE_BYTES, config.LOG_FILE_COUNT,
                     config.INCIDENT_FILES_KEEP, config.SUMMARY_BOOTS_KEEP)
    print(f"USB-Diagnose läuft auf http://{config.HTTP_HOST}:{args.port}/ (Protokoll in {folder})", flush=True)
    try:
        daemon.run(sources, st, config.HTTP_HOST, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        st.close()


if __name__ == "__main__":
    main()
