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

from rosmaster_assist.usbdiag import daemon, mock, store  # noqa: E402


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


def test_mock_produces_incidents_and_files():
    port, stop, d = start_mock()
    try:
        wait_until(lambda: json.loads(get(port, "/api/state"))["incidents"], 15)
        wait_until(lambda: list(pathlib.Path(d, "incidents").glob("*.json")), 15)
        assert pathlib.Path(d, "events.jsonl").stat().st_size > 0
        assert pathlib.Path(d, "usb.log").exists()
    finally:
        stop.set()


def test_restart_does_not_duplicate_backlog():
    port, stop, d = start_mock()
    time.sleep(1)
    stop.set()
    time.sleep(0.5)
    n1 = len(pathlib.Path(d, "events.jsonl").read_text().splitlines())
    # second run on the same folder replays the same backlog: no line may be added for it
    src = mock.MockSources(speed=0.0)  # speed 0: backlog only, no live bursts
    st = store.Store(d, "mock", 1_000_000, 3, 200, 20)
    stop2 = threading.Event()
    threading.Thread(target=daemon.run, args=(src, st, "127.0.0.1", free_port(), stop2), daemon=True).start()
    time.sleep(1)
    stop2.set()
    assert n1 > 0
    assert len(pathlib.Path(d, "events.jsonl").read_text().splitlines()) == n1


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
