"""Small stdlib HTTP server for the German USB page and its JSON state."""

import json
import logging
import pathlib
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATIC_DIR = pathlib.Path(__file__).parent / "web"

# url -> (file in STATIC_DIR, content type); nothing outside this table is ever served
STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/usbdiag.js": ("usbdiag.js", "text/javascript; charset=utf-8"),
    "/usbdiag.css": ("usbdiag.css", "text/css; charset=utf-8"),
}

NOT_FOUND_TEXT = "Diese Seite gibt es hier nicht. Die USB-Diagnose liegt unter /"
UNAVAILABLE_TEXT = "Die Diagnose-Daten sind gerade nicht verfügbar, bitte gleich noch einmal laden."


def make_server(state, host: str, port: int, clock=time.monotonic,
                logger: logging.Logger | None = None) -> ThreadingHTTPServer:
    """Serve the page; clock must be the same time base the daemon feeds into state."""

    class Handler(BaseHTTPRequestHandler):
        server_version = "usbdiag"

        def _send(self, code: int, ctype: str, body: bytes, no_store: bool = False) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            if no_store:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _text(self, code: int, text: str) -> None:
            self._send(code, "text/plain; charset=utf-8", text.encode("utf-8"), no_store=True)

        def do_GET(self):  # noqa: N802 (stdlib name)
            path = self.path.split("?", 1)[0].split("#", 1)[0]
            if path == "/api/state":
                try:
                    # page() takes state.lock itself, so it must not be held here
                    body = json.dumps(state.page(clock()), ensure_ascii=False).encode("utf-8")
                except Exception:  # noqa: BLE001
                    if logger is not None:
                        logger.exception("page model failed")
                    self._text(503, UNAVAILABLE_TEXT)
                    return
                self._send(200, "application/json; charset=utf-8", body, no_store=True)
                return
            entry = STATIC_FILES.get(path)
            if entry is None:
                self._text(404, NOT_FOUND_TEXT)
                return
            try:
                body = (STATIC_DIR / entry[0]).read_bytes()
            except OSError:
                self._text(404, NOT_FOUND_TEXT)
                return
            self._send(200, entry[1], body, no_store=True)

        def log_message(self, format, *args):  # noqa: A002 (stdlib signature)
            pass  # one line per poll would flood the journal

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
