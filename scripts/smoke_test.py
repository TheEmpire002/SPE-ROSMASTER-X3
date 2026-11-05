#!/usr/bin/env python3
"""Checks against a running server whether the basic functions work.

The server must already be running for this (ideally in mock mode):

    Terminal 1:   python -m rosmaster_assist.main --mock
    Terminal 2:   python scripts/smoke_test.py

Additionally needs the package `websockets`:

    pip install websockets

Testing deliberately happens from outside over real HTTP and real
WebSocket -- exactly the path the app takes as well.

On the robot: the script recognizes from the reported driver whether real
hardware is attached, and then skips the drive tests. Anyone who wants them
anyway jacks up the wheels and adds `--allow-motion`.
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
    sys.exit("Missing: pip install websockets")

# The test fetches the speed limit from the server instead of repeating it
# here -- otherwise every change would have to be made in two places, and a
# forgotten one would only show up as a red test.
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
        print(f"  [ERROR] {name}  {detail}")


async def next_of(ws, kind: str, limit: int = 10) -> dict | None:
    """Waits for the next message of a given type.

    Needed because the server sends status messages regularly regardless --
    so the desired answer rarely comes first.
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
    print("\nCamera")
    try:
        with urllib.request.urlopen(base + "/camera/snapshot", timeout=10) as r:
            data = r.read()
            kind = r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as exc:
        # 503 means: camera disabled or no image yet. That is not a
        # failure in the sense of this test -- just information.
        print(f"  [--]     no image ({exc.code}) -- camera off or not ready yet")
        return
    except urllib.error.URLError as exc:
        check("snapshot retrievable", False, str(exc))
        return

    check(
        f"snapshot retrievable ({kind}, {len(data)} bytes)",
        len(data) > 100 and kind.startswith("image/"),
    )

    # A frozen image looks like a running stream in the browser.
    # So compare two snapshots.
    time.sleep(1.0)
    with urllib.request.urlopen(base + "/camera/snapshot", timeout=10) as r:
        second = r.read()
    check("image updates", second != data, "same image twice")


async def drive_until(ws, cmd: dict, hits, seconds: float = 3.0) -> dict | None:
    """Sends a drive command repeatedly and waits for the matching status.

    A single command is not enough: the watchdog stops after half a second,
    and status messages also only come every half second. A single command
    may therefore have stopped long before the first status shows it. The
    app sends continuously -- the test does the same.
    """
    loop = asyncio.get_running_loop()
    end = loop.time() + seconds
    last = None

    while loop.time() < end:
        await ws.send(json.dumps(cmd))
        try:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.1))
        except asyncio.TimeoutError:
            continue
        if msg.get("type") == "state":
            last = msg
            if hits(msg):
                return msg

    return last


async def wait_for_state(ws, hits, seconds: float = 3.0) -> dict | None:
    """Waits for a state that satisfies ``hits`` -- without sending.

    Needed because after a drive phase older status messages are still in
    the buffer. Taking the next one would check the state from a few hundred
    milliseconds ago.
    """
    loop = asyncio.get_running_loop()
    end = loop.time() + seconds
    last = None

    while loop.time() < end:
        try:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.2))
        except asyncio.TimeoutError:
            continue
        if msg.get("type") == "state":
            last = msg
            if hits(msg):
                return msg

    return last


async def _test_motion(ws, driver: str | None, max_vx: float) -> None:
    """Drive tests. With a real driver this actually moves the robot."""
    if driver != "mock":
        print(f"  [!] driver '{driver}' -- the robot will now move briefly.")

    # An excessive drive command must be capped to the speed limit.
    # Since the acceleration ramp in the watchdog (RAMP_* in config) the limit
    # no longer shows up in the status right away, only after a few seconds
    # of accelerating. The simulation waits for that; on the real robot that
    # would be a full-throttle drive through the room -- there it is enough
    # to show that the value never exceeds the limit.
    # The simulation drives in a circle meanwhile (omega at the limit):
    # straight ahead it would cover a good 15 m until reaching the limit, and
    # the robot would then be outside the imaginary room for the following
    # tests.
    if driver == "mock":
        state = await drive_until(
            ws,
            {"type": "drive", "vx": 99.0, "vy": 0.0, "omega": 99.0},
            lambda m: abs(m["vx"] - max_vx) < 1e-6,
            seconds=10.0,
        )
        ok = state is not None and abs(state["vx"] - max_vx) < 1e-6
    else:
        state = await drive_until(
            ws,
            {"type": "drive", "vx": 99.0, "vy": 0.0, "omega": 0.0},
            lambda m: False,
            seconds=1.0,
        )
        ok = state is not None and 0.0 < state["vx"] <= max_vx + 1e-6
    check(f"vx=99.0 is capped to {max_vx}", ok, f"got: {state}")

    # No further commands -> the watchdog must stop on its own.
    # Status messages from the drive phase are still in the buffer, so we
    # wait for standstill instead of taking the next one.
    state = await wait_for_state(ws, lambda m: m["vx"] == 0.0 and m["omega"] == 0.0)
    check(
        "watchdog stops without commands",
        state is not None and state["vx"] == 0.0 and state["omega"] == 0.0,
        f"got: {state}",
    )

    # Stop explicitly to be safe, in case something went wrong above.
    await ws.send(json.dumps({"type": "stop"}))

    await _test_collision(ws)
    await _test_homing(ws)


async def _test_collision(ws) -> None:
    """Collision avoidance: blocks towards the obstacle, allows the rest.

    Starts with ``set_home``: the speed limit test before drives several
    meters, and in the simulation the robot would then be outside the
    imaginary room -- there the lidar sees nothing it could block on.
    """
    await ws.send(json.dumps({"type": "set_home"}))
    await asyncio.sleep(1.0)

    async def simulate_drive(vx, vy, omega, sect=1.2):
        loop = asyncio.get_running_loop()
        end = loop.time() + sect
        last = None
        while loop.time() < end:
            await ws.send(json.dumps({"type": "drive", "vx": vx, "vy": vy, "omega": omega}))
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.15))
                if msg.get("type") == "state":
                    last = msg
            except asyncio.TimeoutError:
                pass
        await ws.send(json.dumps({"type": "stop"}))
        return last or {}

    await ws.send(json.dumps({"type": "collision", "enabled": True, "distance": 1.5}))
    await next_of(ws, "collision")

    # Drive towards an obstacle until it blocks. Do not just accelerate
    # once and measure: at the start the free distance can be larger than
    # the configured minimum distance -- then "drives" would be the correct
    # behavior and the test would check nothing.
    blocked = False
    for _ in range(6):
        state = await simulate_drive(0.4, 0.25, 0.0, 0.8)
        if abs(state.get("vx", 1.0)) < 1e-6 and abs(state.get("vy", 1.0)) < 1e-6:
            blocked = True
            break
    check("stops in front of an obstacle", blocked, "kept driving unbraked")

    # Turning must never be blocked -- otherwise the robot can no longer get free.
    turned = await simulate_drive(0.0, 0.0, 0.5, 1.0)
    check(
        "turning is still allowed",
        abs(turned.get("omega", 0.0)) > 0.1,
        f"omega={turned.get('omega')}",
    )

    await ws.send(json.dumps({"type": "collision", "enabled": False}))
    z = await next_of(ws, "collision")
    check("disabling is applied", bool(z) and z.get("enabled") is False)

    free = await simulate_drive(0.5, 0.0, 0.0)
    check(
        "when disabled it drives again",
        abs(free.get("vx", 0.0)) > 0.1,
        f"vx={free.get('vx')}",
    )


async def _test_homing(ws) -> None:
    """Drive away, press home, come back.

    Checks position tracking and homing together -- on their own they say
    little.
    """
    await ws.send(json.dumps({"type": "set_home"}))
    await asyncio.sleep(0.3)

    # Drive away diagonally: that also checks sideways driving.
    loop = asyncio.get_running_loop()
    end = loop.time() + 2.0
    while loop.time() < end:
        await ws.send(json.dumps({"type": "drive", "vx": 0.5, "vy": 0.3, "omega": 0.0}))
        await asyncio.sleep(0.1)
    await ws.send(json.dumps({"type": "stop"}))

    path = await wait_for_state(ws, lambda m: _distance(m) > 0.5, seconds=5)
    dist = _distance(path) if path else 0.0
    check(f"position is tracked ({dist:.2f} m away)", dist > 0.5)

    await ws.send(json.dumps({"type": "home"}))
    started = await wait_for_state(ws, lambda m: m.get("homing") is True, seconds=5)
    check("homing starts", bool(started and started.get("homing")))
    by_map = bool((_map_display().get("fahrt") or {}).get("nach_karte"))

    done = await wait_for_state(ws, lambda m: m.get("homing") is False, seconds=40)
    rest = _distance(done) if done else 99.0
    # with a confident pose in the (always-on) map homing drives by the
    # lidar to the map's start; the wheel odometry slips in the simulation,
    # so there the map is the better ruler
    in_map = _map_rest() if done and by_map else None
    if in_map is not None:
        check(f"returns to the start (remaining deviation in the map {in_map:.2f} m, "
              f"by wheel odometry {rest:.2f} m)", in_map < 0.25)
    else:
        check(f"returns to the start (remaining deviation {rest:.2f} m)", rest < 0.25)


def _map_display() -> dict:
    try:
        with urllib.request.urlopen(_base + "/api/mapping/ortung", timeout=5) as r:
            return json.loads(r.read()) or {}
    except (urllib.error.URLError, ValueError):
        return {}


def _map_rest() -> float | None:
    """Distance to the map's start if the pose in a map is confident, else None."""
    o = _map_display().get("ortung") or {}
    if not o.get("sicher") or not o.get("pose") or not o.get("start"):
        return None
    p, s = o["pose"], o["start"]
    return ((p["x"] - s[0]) ** 2 + (p["y"] - s[1]) ** 2) ** 0.5


def _distance(state: dict | None) -> float:
    if not state or not state.get("pose"):
        return 0.0
    p = state["pose"]
    return (p["x"] ** 2 + p["y"] ** 2) ** 0.5


async def test_websocket(ws_url: str, allow_motion: bool) -> None:
    print("\nWebSocket")
    async with websockets.connect(ws_url) as ws:
        # The limits arrive unrequested right after connecting.
        lim = await next_of(ws, "limits")
        check("server reports its limits", lim is not None, f"got: {lim}")
        max_vx = lim["max_vx"] if lim else 0.0

        await ws.send(json.dumps({"type": "ping"}))
        check("ping -> pong", await next_of(ws, "pong") is not None)

        # Which driver is running over there? Decides whether we may drive.
        state = await next_of(ws, "state")
        driver = state.get("driver") if state else None
        check("server reports its driver", driver is not None, f"got: {state}")

        if driver == "mock" or allow_motion:
            await _test_motion(ws, driver, max_vx)
        else:
            print(f"  [skipped] drive tests -- real driver '{driver}' detected")
            print("            The robot would move.")
            print("            Jack up the wheels, then: --allow-motion")

        # ESP data must come back unchanged to all clients.
        await ws.send(json.dumps({"type": "vitals", "bpm": 72, "motion": True}))
        vitals = await next_of(ws, "vitals")
        check(
            "vital data is passed on",
            vitals is not None and vitals["bpm"] == 72 and vitals["motion"] is True,
            f"got: {vitals}",
        )

        # Nonsense must not kill the server.
        await ws.send(json.dumps({"type": "gibtsnicht"}))
        err = await next_of(ws, "error")
        check("unknown type returns an error message", err is not None)


_base = ""


async def main_async(host: str, port: int, allow_motion: bool) -> int:
    global _base
    base = _base = f"http://{host}:{port}"
    print(f"Testing {base}")

    test_http(base)
    test_camera(base)
    try:
        await test_websocket(f"ws://{host}:{port}/ws", allow_motion)
    except (OSError, asyncio.TimeoutError) as exc:
        check("WebSocket connection", False, f"{exc} -- is the server running?")

    print(f"\n{_passed} ok, {_failed} failed\n")
    return 1 if _failed else 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument(
        "--allow-motion",
        action="store_true",
        help="Drive tests also with real hardware. ONLY with the wheels jacked up!",
    )
    args = p.parse_args()
    return asyncio.run(main_async(args.host, args.port, args.allow_motion))


if __name__ == "__main__":
    sys.exit(main())
