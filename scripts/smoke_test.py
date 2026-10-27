#!/usr/bin/env python3
"""Prüft gegen einen laufenden Server, ob die Grundfunktionen stimmen.

Der Server muss dafür bereits laufen (am besten im Mock-Modus):

    Terminal 1:   python -m rosmaster_assist.main --mock
    Terminal 2:   python scripts/smoke_test.py

Benötigt zusätzlich das Paket `websockets`:

    pip install websockets

Getestet wird bewusst von außen über echtes HTTP und echtes WebSocket --
also genau der Weg, den die App auch nimmt.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import urllib.error
import urllib.request

try:
    import websockets
except ImportError:
    sys.exit("Fehlt: pip install websockets")

# Muss zu config.py passen. Weicht einer der Werte ab, schlägt der
# Begrenzungstest fehl -- was richtig so ist: dann stimmt etwas nicht überein.
MAX_SPEED_X = 0.4
COMMAND_TIMEOUT_S = 0.5

_passed = 0
_failed = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global _passed, _failed
    if ok:
        _passed += 1
        print(f"  [ok]    {name}")
    else:
        _failed += 1
        print(f"  [FEHLER] {name}  {detail}")


async def next_of(ws, kind: str, limit: int = 10) -> dict | None:
    """Wartet auf die nächste Nachricht eines bestimmten Typs.

    Nötig, weil der Server unabhängig davon regelmäßig Statusmeldungen
    schickt -- die gewünschte Antwort kommt also selten als Erstes.
    """
    for _ in range(limit):
        msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3))
        if msg.get("type") == kind:
            return msg
    return None


def test_http(base: str) -> None:
    print("\nHTTP")
    for path in ("/", "/static/app.js", "/static/style.css"):
        try:
            with urllib.request.urlopen(base + path, timeout=5) as r:
                check(f"GET {path}", r.status == 200 and len(r.read()) > 0)
        except urllib.error.URLError as exc:
            check(f"GET {path}", False, str(exc))


async def test_websocket(ws_url: str) -> None:
    print("\nWebSocket")
    async with websockets.connect(ws_url) as ws:
        await ws.send(json.dumps({"type": "ping"}))
        check("ping -> pong", await next_of(ws, "pong") is not None)

        # Überhöhter Fahrbefehl muss auf das Tempolimit gekappt werden.
        await ws.send(json.dumps({"type": "drive", "vx": 99.0, "vy": 0.0, "omega": 0.0}))
        state = await next_of(ws, "state")
        check(
            f"vx=99.0 wird auf {MAX_SPEED_X} begrenzt",
            state is not None and abs(state["vx"] - MAX_SPEED_X) < 1e-6,
            f"bekommen: {state}",
        )

        # Keine weiteren Befehle -> der Watchdog muss von selbst anhalten.
        await asyncio.sleep(COMMAND_TIMEOUT_S * 3)
        await ws.send(json.dumps({"type": "ping"}))
        state = await next_of(ws, "state")
        check(
            "Watchdog hält ohne Befehle an",
            state is not None and state["vx"] == 0.0 and state["omega"] == 0.0,
            f"bekommen: {state}",
        )

        # ESP-Daten müssen unverändert an alle Clients zurückkommen.
        await ws.send(json.dumps({"type": "vitals", "bpm": 72, "motion": True}))
        vitals = await next_of(ws, "vitals")
        check(
            "Vitaldaten werden weitergereicht",
            vitals is not None and vitals["bpm"] == 72 and vitals["motion"] is True,
            f"bekommen: {vitals}",
        )

        # Unsinn darf den Server nicht umbringen.
        await ws.send(json.dumps({"type": "gibtsnicht"}))
        err = await next_of(ws, "error")
        check("Unbekannter Typ liefert Fehlermeldung", err is not None)


async def main_async(host: str, port: int) -> int:
    base = f"http://{host}:{port}"
    print(f"Teste {base}")

    test_http(base)
    try:
        await test_websocket(f"ws://{host}:{port}/ws")
    except (OSError, asyncio.TimeoutError) as exc:
        check("WebSocket-Verbindung", False, f"{exc} -- läuft der Server?")

    print(f"\n{_passed} ok, {_failed} fehlgeschlagen\n")
    return 1 if _failed else 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    return asyncio.run(main_async(args.host, args.port))


if __name__ == "__main__":
    sys.exit(main())
