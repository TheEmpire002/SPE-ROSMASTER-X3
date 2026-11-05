"""Tests the problem rules of the diagnostics (robot/src/rosmaster_assist/diagnostics.py)
without a robot: battery, acknowledging, incidents, kernel log, log buffer.

    python Marlon/diagnostics/test_diagnostics.py

Instead of the background thread the checks are called directly; driver
and watchdog are dummies with exactly the fields the diagnostics read.
"""

import logging
import pathlib
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist import diagnostics as d  # noqa: E402


class Driver:
    name = "mock"
    connected = True

    def __init__(self) -> None:
        self.volt = 12.4

    def read_battery(self):
        return self.volt


def new_diagnostics():
    watchdog = SimpleNamespace(current_command=SimpleNamespace(vx=0.0, vy=0.0, omega=0.0))
    driver = Driver()
    return d.Diagnostics(driver, watchdog, mock=True), driver


def problem(diag, pid):
    return next((p for p in diag.status()["probleme"] if p["id"] == pid), None)


def test_battery_full_is_ok():
    diag, _ = new_diagnostics()
    diag._check_battery(time.time())
    s = diag.status()
    assert s["gesamt"]["stufe"] == "ok", s["gesamt"]
    assert s["probleme"] == []


def test_battery_critical_and_acknowledge():
    diag, driver = new_diagnostics()
    driver.volt = config.BATTERY_CRITICAL_V - 0.2
    diag._check_battery(time.time())
    p = problem(diag, "akku_kritisch")
    assert p and p["besteht"] and p["zustand"] and not p["quittiert"]
    assert diag.status()["gesamt"]["stufe"] == "kritisch"
    # "Low" at the same time would be redundant.
    assert problem(diag, "akku_schwach") is None

    assert diag.acknowledge("akku_kritisch") == 1
    assert diag.status()["gesamt"]["stufe"] == "ok"

    # Still present -> stays acknowledged, no new red on every measurement.
    diag._check_battery(time.time())
    assert problem(diag, "akku_kritisch")["quittiert"]

    # Recovers and drops again -> new occurrence, open again.
    driver.volt = 12.4
    diag._check_battery(time.time())
    assert not problem(diag, "akku_kritisch")["besteht"]
    time.sleep(0.01)
    driver.volt = config.BATTERY_CRITICAL_V - 0.2
    diag._check_battery(time.time())
    p = problem(diag, "akku_kritisch")
    assert not p["quittiert"] and p["anzahl"] == 2, p


def test_voltage_drop():
    diag, driver = new_diagnostics()
    now = time.time()
    diag._check_battery(now)
    driver.volt = 12.4 - config.VOLTAGE_DROP_V - 0.1
    diag._check_battery(now + 1)
    assert problem(diag, "spannungseinbruch") is not None
    categories = [e["art"] for e in diag.status()["ereignisse"]]
    assert "spannung" in categories


def test_incident_counts_and_expires():
    diag, _ = new_diagnostics()
    now = time.time()
    for i in range(3):
        diag._incident("usb_ueberlast", "warnung", "Titel", "was", ["tun"], timestamp=now + i)
    p = problem(diag, "usb_ueberlast")
    assert p["anzahl"] == 3 and not p["besteht"] and not p["zustand"]

    # Forgotten outside the window.
    diag._problems["usb_ueberlast"].last = now - config.PROBLEM_WINDOW_S - 1
    assert problem(diag, "usb_ueberlast") is None


def test_acknowledged_incident_recurs():
    diag, _ = new_diagnostics()
    now = time.time()
    diag._incident("x", "warnung", "T", "w", [], timestamp=now)
    diag.acknowledge()
    assert problem(diag, "x")["quittiert"]
    diag._incident("x", "warnung", "T", "w", [], timestamp=now + 5)
    assert not problem(diag, "x")["quittiert"]


def test_kernel_usb_disconnect():
    diag, _ = new_diagnostics()
    diag._kernel_line(time.time(), "usb 3-1.2: USB disconnect, device number 7")
    assert problem(diag, "usb_abmeldung") is not None
    texts = [e["text"] for e in diag.status()["ereignisse"]]
    assert any("usb 3-1.2" in t for t in texts), texts


def test_undervoltage_from_vcgencmd():
    diag, _ = new_diagnostics()
    diag._check_throttle(0x50005)  # undervoltage now + since boot
    assert problem(diag, "unterspannung")["schwere"] == "kritisch"
    assert problem(diag, "unterspannung_frueher") is not None
    diag._check_throttle(0x50000)  # over, only "since boot" remains
    assert not problem(diag, "unterspannung")["besteht"]


def test_log_buffer_last_id():
    buffer = d.LogBuffer(3)
    lg = logging.getLogger("rosmaster_assist.test")
    lg.addHandler(buffer)
    lg.setLevel(logging.INFO)
    try:
        for i in range(5):
            lg.warning("Line %d", i)
        lg.error("broken")
    finally:
        lg.removeHandler(buffer)
    lines, last = buffer.lines()
    assert last == 6 and [z["id"] for z in lines] == [4, 5, 6]
    # Browser is "ahead" of the server (restart) -> real last number.
    lines, last = buffer.lines(99999)
    assert lines == [] and last == 6
    assert buffer.error()[0] == 1


def test_battery_percent():
    assert d.battery_percent(None) is None
    assert d.battery_percent(12.6) == 100
    assert d.battery_percent(9.0) == 0


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    error = 0
    for name, f in tests:
        try:
            f()
            print(f"ok      {name}")
        except AssertionError as e:
            error += 1
            print(f"ERROR   {name}: {e}")
    print(f"\n{len(tests) - error}/{len(tests)} passed")
    sys.exit(1 if error else 0)
