#!/usr/bin/env python3
"""Prüft gegen einen laufenden Server, ob die Grundfunktionen stimmen.

Der Server muss dafür bereits laufen (am besten im Mock-Modus):

    Terminal 1:   python -m rosmaster_assist.main --mock
    Terminal 2:   python scripts/smoke_test.py

Benötigt zusätzlich das Paket `websockets`:

    pip install websockets

Getestet wird bewusst von außen über echtes HTTP und echtes WebSocket --
also genau der Weg, den die App auch nimmt.

Auf dem Roboter: Das Skript erkennt am gemeldeten Antrieb, ob echte Hardware
dranhängt, und überspringt die Fahrtests dann. Wer sie trotzdem will, bockt
die Räder auf und hängt `--allow-motion` an.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
import urllib.error
import urllib.request

try:
    import websockets
except ImportError:
    sys.exit("Fehlt: pip install websockets")

# Das Tempolimit holt sich der Test beim Server ab, statt es hier zu
# wiederholen -- sonst müsste man es bei jeder Änderung an zwei Stellen
# nachziehen und merkt das Vergessen erst am rot gewordenen Test.
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


def test_camera(base: str) -> None:
    print("\nKamera")
    try:
        with urllib.request.urlopen(base + "/camera/snapshot", timeout=10) as r:
            data = r.read()
            typ = r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as exc:
        # 503 heißt: Kamera abgeschaltet oder noch kein Bild. Das ist kein
        # Fehler im Sinne dieses Tests -- nur eine Information.
        print(f"  [--]     kein Bild ({exc.code}) -- Kamera aus oder noch nicht bereit")
        return
    except urllib.error.URLError as exc:
        check("Einzelbild abrufbar", False, str(exc))
        return

    check(
        f"Einzelbild abrufbar ({typ}, {len(data)} Bytes)",
        len(data) > 100 and typ.startswith("image/"),
    )

    # Ein stehendes Bild sieht im Browser aus wie ein laufender Stream.
    # Deshalb zwei Aufnahmen vergleichen.
    time.sleep(1.0)
    with urllib.request.urlopen(base + "/camera/snapshot", timeout=10) as r:
        zweites = r.read()
    check("Bild aktualisiert sich", zweites != data, "zweimal dasselbe Bild")


async def drive_until(ws, cmd: dict, treffer, seconds: float = 3.0) -> dict | None:
    """Schickt einen Fahrbefehl im Takt und wartet auf den passenden Status.

    Ein einzelner Befehl reicht nicht: der Watchdog hält nach einer halben
    Sekunde an, und Statusmeldungen kommen ebenfalls nur alle halbe Sekunde.
    Ein Einzelbefehl kann also längst gestoppt sein, bevor der erste Status
    ihn zeigt. Die App sendet dauerhaft -- der Test macht es genauso.
    """
    loop = asyncio.get_running_loop()
    ende = loop.time() + seconds
    letzter = None

    while loop.time() < ende:
        await ws.send(json.dumps(cmd))
        try:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.1))
        except asyncio.TimeoutError:
            continue
        if msg.get("type") == "state":
            letzter = msg
            if treffer(msg):
                return msg

    return letzter


async def wait_for_state(ws, treffer, seconds: float = 3.0) -> dict | None:
    """Wartet auf einen Zustand, der ``treffer`` erfüllt -- ohne zu senden.

    Nötig, weil nach einer Fahrphase noch ältere Statusmeldungen im Puffer
    liegen. Die nächstbeste zu nehmen würde den Zustand von vor ein paar
    hundert Millisekunden prüfen.
    """
    loop = asyncio.get_running_loop()
    ende = loop.time() + seconds
    letzter = None

    while loop.time() < ende:
        try:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.2))
        except asyncio.TimeoutError:
            continue
        if msg.get("type") == "state":
            letzter = msg
            if treffer(msg):
                return msg

    return letzter


async def _test_motion(ws, driver: str | None, max_vx: float) -> None:
    """Fahrtests. Bewegt bei echtem Antrieb tatsächlich den Roboter."""
    if driver != "mock":
        print(f"  [!] Antrieb '{driver}' -- der Roboter fährt jetzt kurz an.")

    # Überhöhter Fahrbefehl muss auf das Tempolimit gekappt werden.
    state = await drive_until(
        ws,
        {"type": "drive", "vx": 99.0, "vy": 0.0, "omega": 0.0},
        lambda m: abs(m["vx"] - max_vx) < 1e-6,
    )
    check(
        f"vx=99.0 wird auf {max_vx} begrenzt",
        state is not None and abs(state["vx"] - max_vx) < 1e-6,
        f"bekommen: {state}",
    )

    # Keine weiteren Befehle -> der Watchdog muss von selbst anhalten.
    # Dabei liegen noch Statusmeldungen aus der Fahrphase im Puffer; es wird
    # also auf den Stillstand gewartet, statt die nächstbeste zu nehmen.
    state = await wait_for_state(ws, lambda m: m["vx"] == 0.0 and m["omega"] == 0.0)
    check(
        "Watchdog hält ohne Befehle an",
        state is not None and state["vx"] == 0.0 and state["omega"] == 0.0,
        f"bekommen: {state}",
    )

    # Sicherheitshalber explizit stoppen, falls oben etwas schiefging.
    await ws.send(json.dumps({"type": "stop"}))

    await _test_collision(ws)
    await _test_homing(ws)


async def _test_collision(ws) -> None:
    """Kollisionsvermeidung: sperrt zum Hindernis, lässt den Rest zu.

    Beginnt mit ``set_home``: Der Tempolimit-Test davor fährt mehrere Meter
    weit, und in der Simulation stünde der Roboter danach außerhalb des
    gedachten Raums -- dort sieht der Lidar nichts, was er sperren könnte.
    """
    await ws.send(json.dumps({"type": "set_home"}))
    await asyncio.sleep(1.0)

    async def fahren(vx, vy, omega, sek=1.2):
        loop = asyncio.get_running_loop()
        ende = loop.time() + sek
        letzter = None
        while loop.time() < ende:
            await ws.send(json.dumps({"type": "drive", "vx": vx, "vy": vy, "omega": omega}))
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.15))
                if msg.get("type") == "state":
                    letzter = msg
            except asyncio.TimeoutError:
                pass
        await ws.send(json.dumps({"type": "stop"}))
        return letzter or {}

    await ws.send(json.dumps({"type": "collision", "enabled": True, "distance": 1.5}))
    await next_of(ws, "collision")

    # Auf ein Hindernis zufahren, bis gesperrt wird. Nicht einfach einmal
    # Gas geben und messen: am Startpunkt kann die freie Strecke größer
    # sein als der eingestellte Mindestabstand -- dann wäre "fährt" das
    # richtige Verhalten und der Test prüfte nichts.
    gesperrt = False
    for _ in range(6):
        zustand = await fahren(0.4, 0.25, 0.0, 0.8)
        if abs(zustand.get("vx", 1.0)) < 1e-6 and abs(zustand.get("vy", 1.0)) < 1e-6:
            gesperrt = True
            break
    check("hält vor einem Hindernis an", gesperrt, "fuhr ungebremst weiter")

    # Drehen darf nie gesperrt werden -- sonst kommt man nicht mehr frei.
    gedreht = await fahren(0.0, 0.0, 0.5, 1.0)
    check(
        "Drehen bleibt trotzdem erlaubt",
        abs(gedreht.get("omega", 0.0)) > 0.1,
        f"omega={gedreht.get('omega')}",
    )

    await ws.send(json.dumps({"type": "collision", "enabled": False}))
    z = await next_of(ws, "collision")
    check("Abschalten wird übernommen", bool(z) and z.get("enabled") is False)

    frei = await fahren(0.5, 0.0, 0.0)
    check(
        "ausgeschaltet fährt er wieder",
        abs(frei.get("vx", 0.0)) > 0.1,
        f"vx={frei.get('vx')}",
    )


async def _test_homing(ws) -> None:
    """Wegfahren, Home drücken, zurückkommen.

    Prüft die Positionsverfolgung und die Heimfahrt zusammen -- einzeln
    sagen sie wenig aus.
    """
    await ws.send(json.dumps({"type": "set_home"}))
    await asyncio.sleep(0.3)

    # Diagonal wegfahren: so wird auch die Seitwärtsfahrt mitgeprüft.
    loop = asyncio.get_running_loop()
    ende = loop.time() + 2.0
    while loop.time() < ende:
        await ws.send(json.dumps({"type": "drive", "vx": 0.5, "vy": 0.3, "omega": 0.0}))
        await asyncio.sleep(0.1)
    await ws.send(json.dumps({"type": "stop"}))

    weg = await wait_for_state(ws, lambda m: _abstand(m) > 0.5, seconds=5)
    entfernung = _abstand(weg) if weg else 0.0
    check(f"Position wird mitgeführt ({entfernung:.2f} m entfernt)", entfernung > 0.5)

    await ws.send(json.dumps({"type": "home"}))
    gestartet = await wait_for_state(ws, lambda m: m.get("homing") is True, seconds=5)
    check("Heimfahrt startet", bool(gestartet and gestartet.get("homing")))

    fertig = await wait_for_state(ws, lambda m: m.get("homing") is False, seconds=40)
    rest = _abstand(fertig) if fertig else 99.0
    check(f"kommt zum Startpunkt zurück (Restabweichung {rest:.2f} m)", rest < 0.25)


def _abstand(state: dict | None) -> float:
    if not state or not state.get("pose"):
        return 0.0
    p = state["pose"]
    return (p["x"] ** 2 + p["y"] ** 2) ** 0.5


async def test_websocket(ws_url: str, allow_motion: bool) -> None:
    print("\nWebSocket")
    async with websockets.connect(ws_url) as ws:
        # Die Grenzwerte kommen unaufgefordert direkt nach dem Verbinden.
        lim = await next_of(ws, "limits")
        check("Server nennt seine Grenzwerte", lim is not None, f"bekommen: {lim}")
        max_vx = lim["max_vx"] if lim else 0.0

        await ws.send(json.dumps({"type": "ping"}))
        check("ping -> pong", await next_of(ws, "pong") is not None)

        # Welcher Antrieb läuft dort drüben? Entscheidet, ob wir fahren dürfen.
        state = await next_of(ws, "state")
        driver = state.get("driver") if state else None
        check("Server meldet seinen Antrieb", driver is not None, f"bekommen: {state}")

        if driver == "mock" or allow_motion:
            await _test_motion(ws, driver, max_vx)
        else:
            print(f"  [übersprungen] Fahrtests -- echter Antrieb '{driver}' erkannt")
            print("                 Der Roboter würde sich bewegen.")
            print("                 Räder aufbocken, dann: --allow-motion")

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


async def main_async(host: str, port: int, allow_motion: bool) -> int:
    base = f"http://{host}:{port}"
    print(f"Teste {base}")

    test_http(base)
    test_camera(base)
    try:
        await test_websocket(f"ws://{host}:{port}/ws", allow_motion)
    except (OSError, asyncio.TimeoutError) as exc:
        check("WebSocket-Verbindung", False, f"{exc} -- läuft der Server?")

    print(f"\n{_passed} ok, {_failed} fehlgeschlagen\n")
    return 1 if _failed else 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument(
        "--allow-motion",
        action="store_true",
        help="Fahrtests auch mit echter Hardware. NUR mit aufgebockten Rädern!",
    )
    args = p.parse_args()
    return asyncio.run(main_async(args.host, args.port, args.allow_motion))


if __name__ == "__main__":
    sys.exit(main())
