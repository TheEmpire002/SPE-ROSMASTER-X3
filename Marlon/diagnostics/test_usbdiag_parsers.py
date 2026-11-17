"""Tests the usbdiag config, German texts and kernel log parser.

    python Marlon/diagnostics/test_usbdiag_parsers.py

fixtures/kmsg_burst.txt is synthetic (its first line says so and the parser
skips it) until a real recording from the Pi replaces it; the same holds for
fixtures/pmic.txt and fixtures/usbmon_board.txt.
"""

import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import kmsg, sampler, texts, usbmon  # noqa: E402

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


def checksum_report(ext_len, ext_type, data):
    return (ext_len + ext_type + sum(data)) % 256


def report_0a(vx, vy, vz, batt_dv):
    data = list(struct.pack("<hhhB", int(vx*1000), int(vy*1000), int(vz*1000), batt_dv))
    ext_len = len(data) + 2
    return bytes([0xFF, 0xFB, ext_len, 0x0A] + data + [checksum_report(ext_len, 0x0A, data)])


def motion_cmd(vx, vy, vz, car_type=1):
    body = [0xFF, 0xFC, 0x00, 0x12, car_type] + list(struct.pack("<hhh", int(vx*1000), int(vy*1000), int(vz*1000)))
    body[2] = len(body) - 1
    return bytes(body + [(sum(body) + 257 - 0xFC) & 0xFF])  # Rosmaster_Lib: sum(cmd, 257 - DEVICE_ID) & 0xff


def test_parse_usbmon_line():
    u = usbmon.parse_line("ffff8880c2a1e000 3091248115 C Bi:1:114:2 0 11 = fffb090a 00000000 00007601")
    assert (u.event, u.xfer, u.bus, u.dev, u.ep, u.status, u.length) == ("C", "Bi", 1, 114, 2, 0, 11)
    assert u.data[:4] == bytes.fromhex("fffb090a")
    assert usbmon.parse_line("ffff8880c2a1e000 3091248115 S Bi:1:114:2 -115 32 <").data == b""
    assert usbmon.parse_line("ffff 1 S Ci:1:001:0 s a3 00 0000 0001 0004 4 <").status is None
    assert usbmon.parse_line("nonsense") is None


def test_frame_decoder_report_and_battery():
    f = usbmon.FrameDecoder().feed(report_0a(0.1, 0.0, -0.6, 118))
    assert len(f) == 1 and f[0].type == 0x0A
    r = usbmon.decode_report(f[0])
    assert abs(r["vx"] - 0.1) < 1e-9 and abs(r["vz"] + 0.6) < 1e-9 and abs(r["battery_v"] - 11.8) < 1e-9


def test_frame_decoder_split_and_resync():
    frame = report_0a(0.0, 0.0, 0.0, 120)
    d = usbmon.FrameDecoder()
    assert d.feed(frame[:5]) == [] and len(d.feed(frame[5:])) == 1
    bad = bytearray(frame); bad[-1] ^= 0xFF
    out = d.feed(bytes(bad) + b"\x00\x13" + frame)
    assert len(out) == 1 and usbmon.decode_report(out[0])["battery_v"] == 12.0
    assert d.feed(frame[:7]) == [] and len(d.feed(frame)) == 1  # truncated urb then a full frame


def test_decode_motion_command():
    cmds = usbmon.decode_commands(motion_cmd(0.0, 0.0, 0.6))
    assert cmds[0].name == "MOTION" and abs(cmds[0].params["vz"] - 0.6) < 1e-9


def test_parse_pmic_and_throttled():
    rails = sampler.parse_pmic((FIXTURES / "pmic.txt").read_text())
    assert abs(rails["EXT5V_V"] - 5.06788) < 1e-6 and "VDD_CORE_A" in rails
    t = sampler.parse_throttled("throttled=0x50005")
    assert t["under_voltage_now"] and t["throttled_now"] and t["under_voltage_boot"] and t["throttled_boot"]
    assert not t["freq_capped_now"]
    assert sampler.parse_throttled("garbage") == {}


def test_cpu_percent():
    assert sampler.cpu_percent((100, 1000), (150, 1100)) == 50.0
    assert sampler.read_cpu_times("cpu  10 0 10 80 0 0 0 0 0 0\ncpu0 1 1 1 1") == (20, 100)


def test_usbmon_fixture_decodes():
    dec = usbmon.FrameDecoder()
    frames, cmds = [], []
    for line in (FIXTURES / "usbmon_board.txt").read_text().splitlines():
        u = usbmon.parse_line(line)
        if u is None or u.dev != 114:
            continue
        if u.xfer == "Bi" and u.event == "C":
            frames += dec.feed(u.data)
        elif u.xfer == "Bo" and u.event == "S":
            cmds += usbmon.decode_commands(u.data)
    assert frames and all(abs(usbmon.decode_report(f)["battery_v"] - 11.8) < 1e-9 for f in frames)
    assert any(c.name == "MOTION" and abs(c.params["vz"] - 0.6) < 1e-9 for c in cmds)


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
