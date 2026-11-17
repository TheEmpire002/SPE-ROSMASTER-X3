"""Tests the usbdiag daemon, the mock sources and the web server.

    python Marlon/diagnostics/test_usbdiag_web.py
"""

import json
import pathlib
import socket
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import __main__ as entry  # noqa: E402
from rosmaster_assist.usbdiag import config, daemon, kmsg, mock, store, texts  # noqa: E402

FIXTURE_KMSG = mock.FIXTURES / "kmsg_burst.txt"
LIVE_DISCONNECT = "6,9001,161000000,-;usb 1-1.2: USB disconnect, device number 112"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def get(port, path):
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=2) as resp:
            return resp.read()
    except Exception:  # noqa: BLE001
        return None


def status(port, path):
    # http.client keeps "/../config.py" as is, urllib would too, but go raw to be sure
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        s.sendall(f"GET {path} HTTP/1.0\r\nHost: x\r\n\r\n".encode("ascii"))
        head = s.recv(64).decode("latin-1")
    return int(head.split()[1])


def wait_until(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            if cond():
                return
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.1)
    raise AssertionError("condition not met in time")


def start_mock():
    src = mock.MockSources(speed=50.0)
    d = tempfile.mkdtemp()
    st = store.Store(d, "mock", 1_000_000, 3, 200, 20)
    stop = threading.Event()
    srv_port = free_port()
    th = threading.Thread(target=daemon.run, args=(src, st, "127.0.0.1", srv_port, stop), daemon=True)
    th.start()
    wait_until(lambda: get(srv_port, "/api/state") is not None, 5)
    return srv_port, stop, d


def test_serves_page_and_state():
    port, stop, d = start_mock()
    try:
        html = get(port, "/").decode("utf-8")
        assert "USB-Diagnose" in html and "usbdiag.js" in html
        assert b"fetch" in get(port, "/usbdiag.js")
        assert get(port, "/usbdiag.css")
        page = json.loads(get(port, "/api/state"))
        assert [m["id"] for m in page["modules"]] == ["pi", "board", "motor"]
        assert status(port, "/../config.py") == 404
        assert status(port, "/config.py") == 404
    finally:
        stop.set()


def test_page_script_places_sockets_by_slot_and_makes_modules_buttons():
    js = (pathlib.Path(daemon.__file__).parent / "web" / "usbdiag.js").read_text(encoding="utf-8")
    # no socket ids in the script, so a changed layout in config shows on the page
    assert "pi-usb" not in js and "t.slot" in js
    assert '"data-module": mod.id' in js
    for kind in ("module", "chip", "tile"):
        start = js.index(f'class: "{kind}",')
        assert 'role: "button"' in js[start:start + 200] and 'tabindex: "0"' in js[start:start + 200], kind
    assert 'closest("[data-tile], [data-module]")' in js


def test_mock_produces_incidents_and_files():
    port, stop, d = start_mock()
    try:
        wait_until(lambda: json.loads(get(port, "/api/state"))["incidents"], 15)
        wait_until(lambda: list(pathlib.Path(d, "incidents").glob("*.json")), 15)
        assert pathlib.Path(d, "events.jsonl").stat().st_size > 0
        assert pathlib.Path(d, "usb.log").exists()
    finally:
        stop.set()


def new_store(d):
    return store.Store(d, "mock", 1_000_000, 3, 200, 20)


def run_in_thread(src, d, port, stop):
    th = threading.Thread(target=daemon.run, args=(src, new_store(d), "127.0.0.1", port, stop), daemon=True)
    th.start()
    wait_until(lambda: get(port, "/api/state") is not None, 5)
    return th


def incident_files(d):
    return sorted(pathlib.Path(d, "incidents").glob("*.json"))


def fixture_events(boot_wall):
    records = [r for r in FIXTURE_KMSG.read_text(encoding="utf-8").splitlines() if r[:1].isdigit()]
    return [e for e in (kmsg.parse_record(r, boot_wall) for r in records) if e]


def reconnects_of(events, port):
    # a connect counts only after a disconnect of the same port
    gone, count = False, 0
    for e in events:
        if e.port == port and e.kind == "disconnect":
            gone = True
        elif e.port == port and e.kind == "connect" and gone:
            gone, count = False, count + 1
    return count


def test_restart_does_not_duplicate_backlog():
    d = tempfile.mkdtemp()
    stop = threading.Event()
    th = run_in_thread(mock.MockSources(speed=50.0), d, free_port(), stop)
    time.sleep(1)
    stop.set()
    th.join(5)
    n1 = len(pathlib.Path(d, "events.jsonl").read_text().splitlines())
    files1 = incident_files(d)
    # second run on the same folder replays the same backlog: no line and no incident may be added for it
    src = mock.MockSources(speed=0.0)  # speed 0: backlog only, no live bursts
    port2 = free_port()
    stop2 = threading.Event()
    th2 = run_in_thread(src, d, port2, stop2)
    try:
        page = json.loads(get(port2, "/api/state"))
    finally:
        stop2.set()
        th2.join(5)
    assert n1 > 0 and files1
    assert len(pathlib.Path(d, "events.jsonl").read_text().splitlines()) == n1
    # (a) same incident files, (b) the page lists every incident of this boot once
    assert incident_files(d) == files1, (files1, incident_files(d))
    assert page["incident_count"] == len(files1), (page["incident_count"], files1)
    backlog_files = [f for f in files1 if json.loads(f.read_text(encoding="utf-8"))["start_mono"] < config.MOCK_START_MONO]
    assert len(backlog_files) == 1, backlog_files
    # (c) the backlog reconnects count toward "seit Boot"
    events = fixture_events(src.boot_wall())
    expected = reconnects_of(events, mock.BOARD)
    assert expected > 0
    rows = {r["label"]: r["value"] for r in page["details"]["board-p4"]["rows"]}
    assert rows["Neu verbunden"] == f"{expected}× seit Boot · 0× seit Start der Diagnose", rows
    # (d) the backlog incident carries the wall clock of its first kernel record
    first = next(e for e in events if e.kind in ("disconnect", "overcurrent", "reset_failed"))
    inc = json.loads(backlog_files[0].read_text(encoding="utf-8"))
    assert inc["start_mono"] == first.mono, (inc["start_mono"], first.mono)
    # the second run rewrote the file with its own boot_wall estimate
    assert abs(inc["start_wall"] - (src.boot_wall() + first.mono)) < 1e-6, (inc["start_wall"], src.boot_wall())
    assert inc["time"] == texts.fmt_clock(src.boot_wall() + first.mono)
    assert page["incidents"][-1]["time"] == texts.fmt_clock(src.boot_wall() + first.mono)


class ManualLoop(daemon._Loop):
    """Daemon loop on a hand-driven clock: every wake moves the mock clock one step."""

    def __init__(self, src, d, step, until, on_wake=None):
        self.clock = [config.MOCK_START_MONO]
        src.now = lambda: self.clock[0]
        super().__init__(src, new_store(d), threading.Event())
        self.step = step
        self.until = until
        self.on_wake = on_wake
        self.wakes = 0

    def wait(self):
        if self.on_wake is not None and self.wakes:
            self.on_wake(self)
        self.wakes += 1
        self.clock[0] += self.step
        if self.clock[0] >= self.until:
            self.stop.set()


def test_burst_turns_board_and_link_red():
    seen = []

    def snap(loop):
        page = loop.state.page(loop.clock[0])
        board = next(t for m in page["modules"] for t in m["tiles"] if t["id"] == "board-p4")
        seen.append((loop.clock[0], mock.BOARD in loop.state.present, board["level"],
                     [x["level"] for x in page["links"]]))

    burst = config.MOCK_START_MONO + config.MOCK_FIRST_BURST_S
    loop = ManualLoop(mock.MockSources(speed=1.0, seed=1), tempfile.mkdtemp(), 0.1, burst + 40, snap)
    loop.start()
    loop.run()
    before = [s for s in seen if s[0] < burst - 1]
    unplugged = [s for s in seen if not s[1]]
    assert before and all(s[3] == ["ok", "ok"] for s in before), before[-1]
    # unplug window: board tile and both links are red
    assert unplugged and all(s[2] == "bad" and s[3] == ["bad", "bad"] for s in unplugged), unplugged
    # board back but its incident still open: tile and pi link stay red
    back = [s for s in seen if s[1] and unplugged[-1][0] < s[0] < unplugged[-1][0] + 5]
    assert back and all(s[2] == "bad" and s[3][0] == "bad" for s in back), back
    assert seen[-1][3] == ["ok", "ok"], seen[-1]


def test_mock_bursts_show_motor_start_and_reenum_loop():
    # the first burst follows a drive command, the second one only the depth camera loop
    second = config.MOCK_START_MONO + config.MOCK_FIRST_BURST_S + config.MOCK_BURST_PERIOD_S
    loop = ManualLoop(mock.MockSources(speed=1.0, seed=1), tempfile.mkdtemp(), 0.25, second + 30)
    loop.start()
    loop.run()
    reasons = [i["reason"] for i in loop.state.page(loop.clock[0])["incidents"]]
    assert "Motoranlauf" in reasons and "Wiedereinschalt-Schleife" in reasons, reasons


def test_tick_runs_at_most_at_the_configured_rate():
    loop = ManualLoop(mock.MockSources(speed=0.0), tempfile.mkdtemp(), 0.01, config.MOCK_START_MONO + 2.0)
    loop.start()
    ticks = []
    real_tick = loop.state.tick

    def counting_tick(now):
        ticks.append(now)
        real_tick(now)
    loop.state.tick = counting_tick
    loop.run()
    assert loop.wakes >= 199, loop.wakes
    expected = 2.0 / config.TICK_INTERVAL_S
    assert expected - 1 <= len(ticks) <= expected + 1, len(ticks)


def test_failing_state_call_does_not_drop_kernel_events():
    d = tempfile.mkdtemp()
    src = mock.MockSources(speed=0.0)
    loop = ManualLoop(src, d, 0.1, config.MOCK_START_MONO + 3.0)
    loop.start()
    ev = kmsg.parse_record(LIVE_DISCONNECT, src.boot_wall())
    script = [[], [ev]]
    src.read_kmsg = lambda: script.pop(0) if script else []
    real_sample = loop.state.apply_sample
    calls = []

    def apply_sample(s):
        calls.append(s)
        if len(calls) == 1:
            raise ValueError("broken sample")
        real_sample(s)
    loop.state.apply_sample = apply_sample
    loop.run()
    assert len(calls) >= 2
    assert ev.raw in pathlib.Path(d, "events.jsonl").read_text(encoding="utf-8")
    cur = loop.state.tracker.current
    assert cur is not None and cur.events[0].raw == ev.raw
    log = pathlib.Path(d, "usb.log").read_text(encoding="utf-8")
    assert log.count("state.apply_sample") == 1, log


def test_shutdown_finishes_open_incident():
    d = tempfile.mkdtemp()
    src = mock.MockSources(speed=0.0)
    ev = kmsg.parse_record(LIVE_DISCONNECT, src.boot_wall())
    backlog = src.read_kmsg()
    script = [backlog, [ev]]
    src.read_kmsg = lambda: script.pop(0) if script else []
    port = free_port()
    stop = threading.Event()
    th = run_in_thread(src, d, port, stop)
    wait_until(lambda: any(i.get("events") and i["events"][0]["time"] == texts.fmt_clock(ev.wall)
                           and not i["recovered"] for i in json.loads(get(port, "/api/state"))["incidents"]), 5)
    stop.set()
    th.join(5)
    assert not th.is_alive()
    found = [json.loads(f.read_text(encoding="utf-8")) for f in incident_files(d)]
    inc = next((i for i in found if i["start_mono"] == ev.mono), None)
    assert inc is not None, [i["start_mono"] for i in found]
    assert inc["recovered"] is False


def test_sigterm_sets_the_stop_event():
    import signal
    old = signal.getsignal(signal.SIGTERM)
    stop = threading.Event()
    try:
        entry.stop_on_sigterm(stop)
        signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    finally:
        signal.signal(signal.SIGTERM, old)
    assert stop.is_set()


def test_source_errors_do_not_stop_the_loop():
    src = mock.MockSources(speed=50.0)

    def broken():
        raise OSError("gone")
    src.scan_topology = broken
    d = tempfile.mkdtemp()
    st = store.Store(d, "mock", 1_000_000, 3, 200, 20)
    stop = threading.Event()
    port = free_port()
    threading.Thread(target=daemon.run, args=(src, st, "127.0.0.1", port, stop), daemon=True).start()
    try:
        wait_until(lambda: get(port, "/api/state") is not None, 5)
        time.sleep(1.5)
        assert get(port, "/api/state") is not None
        log = pathlib.Path(d, "usb.log").read_text(encoding="utf-8")
        assert log.count("topology") == 1, log
    finally:
        stop.set()


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
