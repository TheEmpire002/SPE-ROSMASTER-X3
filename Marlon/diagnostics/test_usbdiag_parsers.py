"""Tests the usbdiag config, German texts and kernel log parser.

    python Marlon/diagnostics/test_usbdiag_parsers.py

fixtures/kmsg_burst.txt is synthetic (its first line says so and the parser
skips it) until a real recording from the Pi replaces it.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import kmsg, texts  # noqa: E402

FIXTURES = pathlib.Path(__file__).resolve().parents[2] / "robot/src/rosmaster_assist/usbdiag/fixtures"
REC = "3,5202,173531911,-;usb 3-2: clear tt 4 (9032) error -71"


def test_normalize_port():
    assert kmsg.normalize_port("usb3-port2") == "3-2"
    assert kmsg.normalize_port("1-1-port4") == "1-1.4"
    assert kmsg.normalize_port("1-1.4") == "1-1.4"


def test_parse_kinds():
    cases = {
        "6,900,100000000,-;usb 1-1.4: USB disconnect, device number 113": ("disconnect", "1-1.4"),
        "6,901,100100000,-;usb 1-1.4: new full-speed USB device number 114 using xhci-hcd": ("new_device", "1-1.4"),
        "6,902,100200000,-;usb 1-1.4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64": ("connect", "1-1.4"),
        "4,903,100300000,-;usb usb1-port1: over-current change #54": ("overcurrent", "1-1"),
        "3,904,100400000,-;usb 3-1-port2: cannot reset (err = -71)": ("reset_failed", "3-1.2"),
        "3,905,100500000,-;usb 3-1: device descriptor read/64, error -71": ("enum_failed", "3-1"),
        REC: ("protocol_error", "3-2"),
        "6,906,100600000,-;usb 1-1.4: ch341-uart converter now attached to ttyUSB1": ("driver_attached", "1-1.4"),
        "6,907,100700000,-;ch341-uart ttyUSB1: ch341-uart converter now disconnected from ttyUSB1": ("driver_detached", None),
        "4,908,100800000,-;hwmon hwmon1: Undervoltage detected!": ("undervoltage", None),
    }
    for rec, (kind, port) in cases.items():
        ev = kmsg.parse_record(rec, boot_wall=1_000_000.0)
        assert ev is not None and (ev.kind, ev.port) == (kind, port), rec


def test_parse_details_and_time():
    ev = kmsg.parse_record("6,902,100200000,-;usb 1-1.4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64\n SUBSYSTEM=usb", 1_000_000.0)
    assert ev.details == {"vid": "1a86", "pid": "7523"}
    assert abs(ev.mono - 100.2) < 1e-6 and abs(ev.wall - 1_000_100.2) < 1e-6
    assert kmsg.parse_record(REC, 0).details["error"] == -71
    assert kmsg.parse_record("6,906,1,-;usb 1-1.4: ch341-uart converter now attached to ttyUSB1", 0).details["tty"] == "ttyUSB1"


def test_parse_ignores_other_lines():
    assert kmsg.parse_record("6,1,1,-;eth0: link up", 0) is None
    assert kmsg.parse_record("garbage", 0) is None
    assert kmsg.parse_record(" SUBSYSTEM=usb", 0) is None


def test_fixture_burst_parses():
    lines = (FIXTURES / "kmsg_burst.txt").read_text().splitlines()
    events = [kmsg.parse_record(l, 0) for l in lines]
    assert sum(e is not None for e in events) >= 20
    kinds = {e.kind for e in events if e is not None}
    assert kinds >= {"disconnect", "new_device", "connect", "overcurrent", "reset_failed",
                     "enum_failed", "protocol_error", "driver_attached", "driver_detached",
                     "undervoltage"}


def test_texts_have_units_and_judgement():
    assert texts.fmt_volts(5.06788) == "5,07 V"
    assert texts.fmt_volts(None) == "unbekannt"
    assert texts.fmt_duration(72) == "1 min 12 s"
    assert texts.fmt_duration(7500) == "2 h 5 min"
    assert texts.judge_ext5v(5.06) == ("gut", "ok")
    assert texts.judge_ext5v(4.85)[1] == "warn"
    assert texts.judge_ext5v(4.70)[1] == "bad"
    assert texts.judge_temp(None) == ("unbekannt", "unknown")


def test_event_sentences_are_plain_german():
    for kind in ["disconnect", "connect", "overcurrent", "reset_failed", "enum_failed",
                 "protocol_error", "driver_attached", "driver_detached", "undervoltage", "board_silent"]:
        s = texts.event_sentence(kind, "Lidar", {"error": -71, "tty": "ttyUSB0"})
        assert s and "-71" not in s and "err" not in s and "tty" not in s.lower()
    assert texts.event_sentence("disconnect", "Lidar", {}) == "Lidar hat sich abgemeldet"
    assert set(texts.REASONS) == {"motor_start", "reenum_loop", "heat", "undervoltage",
                                 "cpu_spike", "board_mcu_silent", "idle_unclear"}
    assert all(r["title"] and r["action"] for r in texts.REASONS.values())


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
