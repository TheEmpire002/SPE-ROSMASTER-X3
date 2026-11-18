"""Tests the usbdiag DiagState and the German page model.

    python Marlon/diagnostics/test_usbdiag_state.py
"""

import json
import pathlib
import re
import struct
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import config, kmsg, mock, sampler, state, texts, topology, usbmon  # noqa: E402

FIXTURES = pathlib.Path(__file__).resolve().parents[2] / "robot/src/rosmaster_assist/usbdiag/fixtures"

RAW_PATTERNS = [r"-\d{2,3}\b", r"\b[0-9a-f]{4}:[0-9a-f]{4}\b", r"\busb\d", r"\b\d-\d(\.\d)*\b",
                r"ttyUSB", r"error", r"\b200\b"]
# identifiers that the page script uses but never shows
HIDDEN_KEYS = {"id", "from", "to", "kind"}

_calls = [0]


def fixture_topology(tmp, drop=(), add=None):
    """Scan the Task 3 fixture with ports removed or devices added (fresh tree per call)."""
    tree = json.loads((FIXTURES / "topology.json").read_text())
    for path in drop:
        tree["devices"].pop(path, None)
        hub, _, num = path.rpartition(".")
        port = tree["ports"].get(f"{hub}-port{num}")
        if port:
            port.update(state="not attached", device=False)
    for i, (path, (vid, pid, speed)) in enumerate((add or {}).items()):
        tree["devices"][path] = {"idVendor": vid, "idProduct": pid, "product": "Camera", "speed": str(speed),
                                 "bMaxPower": "500mA", "devnum": str(120 + i), "busnum": path.split("-")[0]}
    _calls[0] += 1
    root = pathlib.Path(tmp) / f"sys{_calls[0]}"
    mock.write_fake_sysfs(tree, root)
    return topology.scan(str(root))


def walk_strings(obj, skip=()):
    if isinstance(obj, str):
        yield obj
    elif isinstance(obj, dict):
        for key, value in obj.items():
            if key not in skip:
                yield from walk_strings(value, skip)
    elif isinstance(obj, (list, tuple)):
        for value in obj:
            yield from walk_strings(value, skip)


def main_view_strings(page):
    details = {k: dict(v, technical=[]) for k, v in page["details"].items()}
    return list(walk_strings({**page, "details": details}, skip=HIDDEN_KEYS))


def tile_by_id(page, tile_id):
    for module in page["modules"]:
        for tile in module["tiles"]:
            if tile["id"] == tile_id:
                return tile
    raise KeyError(tile_id)


def motion_hex(vx, vy, vz, car_type=1):
    body = bytes([0xFF, 0xFC, 10, 0x12, car_type]) + struct.pack("<hhh", int(vx * 1000), int(vy * 1000),
                                                                int(vz * 1000))
    frame = body + bytes([(sum(body) + 257 - 0xFC) & 0xFF])
    return " ".join(frame[i:i + 4].hex() for i in range(0, len(frame), 4))


def loaded_state(tmp):
    st = state.DiagState(None, boot_wall=1_700_000_000.0, daemon_start_mono=0.0)
    with st.lock:
        st.apply_topology(fixture_topology(tmp), now=10.0)
        for line in (FIXTURES / "kmsg_burst.txt").read_text().splitlines():
            e = kmsg.parse_record(line, 1_700_000_000.0)
            if e:
                st.apply_event(e, backlog=True)
        st.apply_sample(sampler.Sample(10.0, 0, 5.06788, {}, {}, 61.0, 2000.0, 12.0, 0.4))
    return st


def test_page_shape_and_modules():
    with tempfile.TemporaryDirectory() as d:
        st = loaded_state(pathlib.Path(d))
        with st.lock:
            st.tick(130.0)  # the backlog burst is long over
        p = st.page(now=130.0)
    assert [m["id"] for m in p["modules"]] == ["pi", "board", "motor"]
    assert p["modules"][0]["note"] == "Lage der Buchsen noch nicht geprüft"
    assert {"from": "pi-usb3-a", "to": "board", "level": "ok"} in p["links"]
    assert set(p["details"]) >= {"pi-usb3-a", "board-p4", "motor"}
    v = {x["label"]: x for x in p["vitals"]}
    assert v["5V Pi"]["value"] == "5,07 V" and v["5V Pi"]["judgement"] == "gut"


def test_pi_sockets_carry_their_config_slot():
    with tempfile.TemporaryDirectory() as d:
        p = loaded_state(pathlib.Path(d)).page(now=20.0)
    pi = p["modules"][0]["tiles"]
    assert {t["id"]: t["slot"] for t in pi} == {s["id"]: s["slot"] for s in config.PI_SOCKETS}


def test_link_stays_red_while_board_incident_is_open():
    with tempfile.TemporaryDirectory() as d:
        tmp = pathlib.Path(d)
        st = state.DiagState(None, 0.0, 0.0)
        with st.lock:
            st.apply_topology(fixture_topology(tmp), now=1.0)
            st.tick(1.0)
        assert [x["level"] for x in st.page(1.0)["links"]] == ["ok", "ok"]
        with st.lock:
            st.apply_event(kmsg.Event(30.0, 0, "disconnect", "1-1.4", {}, "r"))
            st.apply_topology(fixture_topology(tmp, drop=["1-1.4"]), now=30.2)
            st.tick(30.2)
        p = st.page(30.3)
        assert [x["level"] for x in p["links"]] == ["bad", "bad"] and tile_by_id(p, "board-p4")["level"] == "bad"
        # plugged back in, but the incident is still open
        with st.lock:
            st.apply_event(kmsg.Event(33.0, 0, "connect", "1-1.4", {}, "r"))
            st.apply_topology(fixture_topology(tmp), now=33.1)
            st.tick(35.0)
        p = st.page(35.0)
        assert st.tracker.current is not None
        assert p["links"][0]["level"] == "bad" and tile_by_id(p, "board-p4")["level"] == "bad"
        with st.lock:
            st.tick(33.0 + config.INCIDENT_QUIET_S + 1)
        p = st.page(33.0 + config.INCIDENT_QUIET_S + 1)
        assert st.tracker.current is None
        assert [x["level"] for x in p["links"]] == ["ok", "ok"]


def test_reconnect_row_counts_since_boot_and_since_start():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, daemon_start_mono=50.0)
        with st.lock:
            st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        rows = {r["label"]: r["value"] for r in st.page(2.0)["details"]["board-p4"]["rows"]}
        assert rows["Neu verbunden"] == "nie seit Boot"
        with st.lock:
            st.apply_event(kmsg.Event(9.0, 0, "disconnect", "1-1.4", {}, "r"), backlog=True)
            st.apply_event(kmsg.Event(10.0, 0, "connect", "1-1.4", {}, "r"), backlog=True)
            st.apply_event(kmsg.Event(59.0, 0, "disconnect", "1-1.4", {}, "r"))
            st.apply_event(kmsg.Event(60.0, 0, "connect", "1-1.4", {}, "r"))
        rows = {r["label"]: r["value"] for r in st.page(61.0)["details"]["board-p4"]["rows"]}
        assert rows["Neu verbunden"] == "2× seit Boot · 1× seit Start der Diagnose", rows


def test_main_view_has_no_raw_codes():
    with tempfile.TemporaryDirectory() as d:
        p = loaded_state(pathlib.Path(d)).page(now=20.0)
    strings = main_view_strings(p)
    assert strings
    for s in strings:
        for pat in RAW_PATTERNS:
            assert not re.search(pat, s), (pat, s)


def test_missing_board_turns_red_and_verdict_bad():
    with tempfile.TemporaryDirectory() as d:
        st = loaded_state(pathlib.Path(d))
        with st.lock:
            st.apply_event(kmsg.Event(30.0, 0, "disconnect", "1-1.4", {}, "r"))
            st.apply_topology(fixture_topology(pathlib.Path(d), drop=["1-1.4"]), now=30.5)
        p = st.page(now=31.0)
    tile = tile_by_id(p, "board-p4")
    assert tile["level"] == "bad" and p["verdict"]["level"] == "bad"
    assert "Board-Mikrocontroller" in p["verdict"]["text"]
    assert "fehlt seit" in tile["line"]
    motor_tile = tile_by_id(p, "motor")
    assert "fehlt seit" in motor_tile["line"] and motor_tile["level"] == "bad"
    assert {"from": "board-p4", "to": "motor", "level": "bad"} in p["links"]


def test_capture_refused_when_camera_on_board_bus():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d), add={"1-1.1": ("2bc5", "060f", 480)}), now=1.0)
        assert st.capture_target() is None
        p = st.page(2.0)
        assert "Kamera am selben USB-Bus" in p["motor"]["capture"]
        for tid in ("board-p4", "motor"):
            assert "Kamera am selben USB-Bus" in tile_by_id(p, tid)["line"]
        rows = " ".join(r["value"] for r in p["details"]["board-p4"]["rows"] + p["details"]["motor"]["rows"])
        assert "Kamera am selben USB-Bus" in rows
        assert "kein Mithören" not in rows
        # traffic is ignored while the camera shares the bus
        st.set_capture("on")
        h = motion_hex(0.0, 0.0, 0.3)
        st.apply_urbs([usbmon.parse_line(f"ffff 1000000 S Bo:1:114:2 -115 12 = {h}")], now=3.0)
        assert st.last_command_mono is None and not st.traffic


def test_verdict_bad_while_incident_open():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=0.5)
        st.set_capture("on")
        h = motion_hex(0.0, 0.0, 0.0)
        for i in range(31):
            t = 1.0 + i * 0.1
            st.apply_urbs([usbmon.parse_line(f"ffff {int(t * 1e6)} S Bo:1:114:2 -115 12 = {h}")], now=t)
            st.tick(t)
        assert st.tracker.current is not None
        assert st.page(4.2)["verdict"]["level"] == "bad"


def test_capture_target_is_board_and_lidar():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        assert st.capture_target() is None
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        assert st.capture_target() == (1, {113, 114})


def test_board_reports_feed_motor_block():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        urbs = [usbmon.parse_line(line) for line in (FIXTURES / "usbmon_board.txt").read_text().splitlines()]
        st.apply_urbs([u for u in urbs if u], now=5.0)
        m = st.page(6.0)["motor"]
    assert m["battery"].startswith("11,8 V") and m["capture"] == "Mithören aktiv"


def test_board_silent_raises_incident():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=0.5)
        st.set_capture("on")
        # standing-still commands keep the board polled without counting as a motor start
        hexcmd = motion_hex(0.0, 0.0, 0.0)
        for i in range(31):
            t = 1.0 + i * 0.1
            st.apply_urbs([usbmon.parse_line(f"ffff {int(t * 1e6)} S Bo:1:114:2 -115 12 = {hexcmd}")], now=t)
            st.tick(t)
        p = st.page(4.2)
        assert any("sendet keine Meldungen" in e["text"] for e in p["details"]["board-p4"]["events"])
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=20.0)
        st.tick(20.0)
        assert st.page(20.0)["incidents"][0]["reason"] == "Board-Mikrocontroller still"


def test_backlog_storms_become_separate_incidents():
    lines = mock._kmsg_records((FIXTURES / "kmsg_storms.txt").read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 1_700_000_000.0, 0.0)
        last = 0.0
        with st.lock:
            st.apply_topology(fixture_topology(pathlib.Path(d)), now=10.0)
            for line in lines:
                e = kmsg.parse_record(line, 1_700_000_000.0)
                if e:
                    st.apply_event(e, backlog=True)
                    last = max(last, e.mono)
            st.tick(last + config.INCIDENT_QUIET_S + 1)
        done = st.finished
    starts = sorted(i["start_mono"] for i in done)
    # the 4.2 s and 11.0 s storms are only 1.8 s apart in the real log, so they stay one incident
    for want in (4.2, 78.2, 3344.5):
        assert any(abs(s - want) <= 1 for s in starts), (want, starts)
    assert len(done) >= 3, starts
    assert all(i["end_mono"] - i["start_mono"] <= config.INCIDENT_MAX_S for i in done), starts


def test_capture_without_traffic_says_server_off():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        assert st.page(10.0)["motor"]["capture"] == "kein Verkehr (Server aus)"


def test_motor_without_traffic_says_server_off():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        now = 100.0
        assert st.page(now)["motor"]["state"] == texts.MOTOR_NO_TRAFFIC == "Kein Verkehr – Server aus?"
        st.last_urb = now - 1
        assert st.page(now)["motor"]["state"] == "Motoren stehen"
        st.last_urb = now - config.CAPTURE_IDLE_S - 1
        assert st.page(now)["motor"]["state"] == texts.MOTOR_NO_TRAFFIC


def test_counts_follow_the_port_when_the_board_returns():
    with tempfile.TemporaryDirectory() as d:
        tmp = pathlib.Path(d)
        st = state.DiagState(None, 0.0, 0.0)
        # the daemon started during a storm: no board hub in the first scan
        st.apply_topology(fixture_topology(tmp, drop=["1-1", "1-1.2", "1-1.4"]), now=1.0)
        for i, port in enumerate(("1-1.4", "1-1.2")):
            st.apply_event(kmsg.Event(5.0 + i, 0, "disconnect", port, {}, "r"), backlog=True)
            st.apply_event(kmsg.Event(6.0 + i, 0, "connect", port, {}, "r"), backlog=True)
        st.apply_topology(fixture_topology(tmp), now=30.0)
        st.tick(31.0)  # closes the storm incident
        page = st.page(31.0)
        for tile_id in ("board-p4", "board-p2"):
            tile = tile_by_id(page, tile_id)
            assert "1× neu verbunden" in tile["line"], (tile_id, tile["line"])
            rows = {r["label"]: r["value"] for r in page["details"][tile_id]["rows"]}
            assert rows["Neu verbunden"].startswith("1× seit Boot"), (tile_id, rows)
        usb3 = tile_by_id(page, "pi-usb3-a")
        assert "neu verbunden" not in usb3["line"], usb3["line"]
        rows = {r["label"]: r["value"] for r in page["details"]["pi-usb3-a"]["rows"]}
        assert rows.get("Neu verbunden", "nie seit Boot") == "nie seit Boot", rows


def test_finished_incident_ends_at_its_last_event():
    with tempfile.TemporaryDirectory() as d:
        tmp = pathlib.Path(d)
        st = state.DiagState(None, 0.0, 0.0)
        with st.lock:
            st.apply_topology(fixture_topology(tmp), now=1.0)
            st.apply_event(kmsg.Event(30.0, 0, "disconnect", "1-1.4", {}, "r"))
            st.apply_topology(fixture_topology(tmp, drop=["1-1.4"]), now=30.2)
            st.apply_event(kmsg.Event(32.0, 0, "connect", "1-1.4", {}, "r"))
            st.apply_topology(fixture_topology(tmp), now=32.1)
            for t in (28.0, 31.0, 40.0):
                st.apply_sample(sampler.Sample(t, 0, 5.06, {}, {}, 55.0, 2000.0, 12.0, 0.4))
            st.tick(32.0 + config.INCIDENT_QUIET_S + 5)
        now = 32.0 + config.INCIDENT_QUIET_S + 6
        p = st.page(now)
        inc = st.finished[0]
        # past the verdict window: 3635 s since the reconnect, 3625 s if the quiet wait counted
        later = st.page(32.0 + config.VERDICT_WINDOW_S + 35)
    assert st.tracker.current is None
    # duration, snapshot and the stable-since text all end with the reconnect, not after the quiet wait
    assert inc["end_mono"] == 32.0 and p["incidents"][0]["duration"] == "2 s", inc["duration"]
    assert [s["mono"] for s in inc["snapshot"]] == [28.0, 31.0], inc["snapshot"]
    assert later["verdict"]["text"] == "Alles stabil seit 1 h 1 min", later["verdict"]


def test_missing_usbmon_has_its_own_text():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("usbmon_missing")
        m = st.page(2.0)["motor"]
    assert m["capture"] == texts.CAPTURE_TEXTS["usbmon_missing"] != texts.CAPTURE_TEXTS["unavailable"]
    assert "usbmon ist nicht geladen" in m["capture"]


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print("bad ", name, repr(exc))
    sys.exit(1 if failed else 0)
