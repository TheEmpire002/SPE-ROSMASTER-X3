# USB diagnosis daemon Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A stdlib-only background daemon on the Pi that records every USB event of the three modules (Pi 5, expansion board, motor controller), explains each crash in plain German and shows it all on a clickable, graphical page on port 8001.

**Architecture:** New subpackage `robot/src/rosmaster_assist/usbdiag/` with relative imports only, so the install script can copy it to `/opt/rosmaster-usbdiag/usbdiag/` and run it with the system `python3 -m usbdiag`. Pure parsers and logic (kmsg, sysfs topology, usbmon, PMIC, incidents, German texts) feed one in-memory `DiagState`. `DiagState` produces a single JSON document with every string already formatted in German, and a static HTML/SVG page renders it. A mock mode replays recorded Pi data so everything runs and is tested on Windows.

**Tech Stack:** Python ≥3.11 standard library only (`http.server`, `select`, `subprocess`, `logging.handlers`, `json`, `dataclasses`), vanilla JS + inline SVG, systemd.

**Spec:** `docs/superpowers/specs/2026-11-17-usb-diagnose-design.md`

## Global Constraints

- Stdlib only. Inside `usbdiag/` no import of `rosmaster_assist.*` outside the subpackage, only relative imports (`from . import config`).
- Code, identifiers, comments, docstrings, log lines, tests, docs and commits in English. Comments are concise lowercase fragments without a trailing period that explain the why.
- Everything on the page is German. The main view never shows kernel errors (`-71`), URB status codes, USB IDs, sysfs paths or HTTP codes. Raw text appears only in the collapsed "Technische Details" box.
- Values are always shown with a unit and a judgement word ("5,06 V – gut"). Decimal comma.
- All numbers live in `usbdiag/config.py`.
- The daemon never opens `/dev/myserial`, `/dev/rplidar` or any `ttyUSB*`, and never writes to the board.
- CPU budget: well below 3 % of one core on average. usbmon captures only the board's bus, and only when no camera is on that bus.
- Logs go to `/home/pi/.rosmaster/diagnose/usb/` and are bounded: `usb.log` 2 MB × 3, `events.jsonl` 2 MB × 3, the newest 200 incident files. Files are readable by user `pi` (chown to `pi` when running as root).
- Must survive any watched file, device or sysfs directory disappearing in the middle of a read.
- Tests are plain `test_*` functions with a `__main__` runner, in `Marlon/diagnostics/test_usbdiag_*.py`, run by `python scripts/run_tests.py usbdiag`. The runner flags any output line containing "FEHLER", so test output must not print that word.
- Default timing: incident quiet period 10 s, snapshot 30 s before, incident timeout 120 s, motor window 2 s.
- Port 8001. Install to `/opt/rosmaster-usbdiag/`; the unit runs it as root with `/usr/bin/python3`.
- Every work step goes into `Marlon/PROJECT_LOG.md`; technical details go into `docs/hardware.md` and `docs/architecture.md`.
- Pi workflow: commit and push on the laptop, `git pull` on the Pi. Ask Marlon before any state change on the Pi (`modprobe`, install, enabling the unit).

## Review Focus

1. **Board hub moves to a bus that also carries a camera.** usbmon must then not capture (CPU anti-goal), and the board and motor tiles say "Mithören abgeschaltet: Kamera am selben USB-Bus". Test in Task 6.
2. **Board vanishes in the middle of an incident.** The board and motor modules must stay on the page, turn red and show "fehlt seit …", not disappear because the CH340 is no longer in sysfs. Test in Task 3.
3. **A sysfs directory or file disappears between `listdir` and `open`.** Scanning returns the remaining topology, never raises. Test in Task 3.
4. **A board frame split across two URBs, or truncated by the 32-byte usbmon cap.** The decoder resynchronises at the next `FF FB` and never yields a frame with a wrong checksum. Test in Task 2.
5. **The daemon starts long after boot.** The kmsg backlog (hundreds of records) must count toward "seit Boot" counters and incidents with correct wall-clock times, without flooding `events.jsonl` with duplicates on a daemon restart. Test in Task 7.

---

## File structure

```
robot/src/rosmaster_assist/usbdiag/
  __init__.py        package docstring only
  __main__.py        argparse: --mock, --port, --log-dir; calls daemon.run
  config.py          every number, path, threshold, known-device table, Pi socket table
  texts.py           German formatting, judgement words, event sentences, reason titles and actions
  kmsg.py            Event dataclass, kmsg record parser, port normalisation, KmsgReader
  topology.py        sysfs scan → Topology (ports, devices); dropout-safe reads
  layout.py          Topology → Layout (modules, tiles, links); board detection; keeps the board while it is gone
  usbmon.py          usbmon text line parser, board frame decoder, command decoder, UsbmonTap
  sampler.py         PMIC/throttled/proc parsers, Sampler, process scan
  incidents.py       IncidentTracker, Context, classify() rules
  state.py           DiagState: applies all inputs, keeps rings and counters, builds the page JSON
  store.py           usb.log, events.jsonl, incident files, summary.json, chown to pi
  web.py             ThreadingHTTPServer: static files + /api/state
  mock.py            fake sysfs tree writer, MockSources that replay fixtures
  daemon.py          main loop wiring the real or mock sources, error isolation, re-open with backoff
  web/index.html, web/usbdiag.css, web/usbdiag.js
  fixtures/topology.json, fixtures/kmsg_burst.txt, fixtures/usbmon_board.txt, fixtures/pmic.txt
robot/deploy/usbdiag/rosmaster-usbdiag.service
robot/deploy/usbdiag/install.sh
Marlon/diagnostics/test_usbdiag_parsers.py
Marlon/diagnostics/test_usbdiag_topology.py
Marlon/diagnostics/test_usbdiag_incidents.py
Marlon/diagnostics/test_usbdiag_state.py
Marlon/diagnostics/test_usbdiag_store.py
Marlon/diagnostics/test_usbdiag_web.py
Modify: webapp/diagnose.html (header link), webapp/diagnose.js (set the link's host)
Modify: docs/hardware.md, docs/architecture.md, Marlon/PROJECT_LOG.md
```

Every test file starts like `Marlon/diagnostics/test_diagnostics.py`:
`sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))`, then `from rosmaster_assist.usbdiag import …`, plain `test_*` functions, and the same `__main__` runner. That runner loops over the `test_*` globals, prints `ok      name` / `ERROR   name: e` and `n/m passed`, and exits 1 on any error.

---

### Task 1: Config, German texts and the kmsg parser

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/__init__.py`, `config.py`, `texts.py`, `kmsg.py`
- Create: `robot/src/rosmaster_assist/usbdiag/fixtures/kmsg_burst.txt`
- Test: `Marlon/diagnostics/test_usbdiag_parsers.py`

**Interfaces:**
- Produces in `config.py` (names are fixed, values as given):
  - `HTTP_PORT = 8001`, `HTTP_HOST = "0.0.0.0"`, `POLL_HINT_MS = 1000`
  - `LOG_DIR = "/home/pi/.rosmaster/diagnose/usb"`, `LOG_FILE_BYTES = 2_000_000`, `LOG_FILE_COUNT = 3`, `INCIDENT_FILES_KEEP = 200`, `SUMMARY_BOOTS_KEEP = 20`, `LOG_OWNER = "pi"`
  - `TOPOLOGY_POLL_S = 1.0`, `SAMPLE_INTERVAL_S = 1.0`, `PROCESS_INTERVAL_S = 5.0`, `SUMMARY_INTERVAL_S = 30.0`
  - `INCIDENT_QUIET_S = 10.0`, `INCIDENT_MAX_S = 120.0`, `SNAPSHOT_BEFORE_S = 30.0`, `SAMPLE_RING_S = 900.0`, `PORT_EVENT_RING = 60`, `HISTORY_BUCKET_S = 300`, `HISTORY_BUCKETS = 12`
  - `MOTOR_WINDOW_S = 2.0`, `MOTOR_DELTA = 0.05` (m/s or rad/s change that counts as a start), `BATTERY_DROP_V = 0.3`, `LOOP_REENUM_WINDOW_S = 3.0`, `LOOP_RESET_FAILS = 3`, `CPU_SPIKE_PCT = 80.0`, `CPU_WINDOW_S = 5.0`, `BOARD_SILENT_S = 2.0`, `RECENT_RECONNECT_S = 300.0`
  - judgement thresholds: `EXT5V_GOOD_V = 4.95`, `EXT5V_LOW_V = 4.80`, `BATTERY_GOOD_V = 11.3`, `BATTERY_LOW_V = 10.5`, `TEMP_WARM_C = 70.0`, `TEMP_HOT_C = 80.0`
  - `BOARD_ID = ("1a86", "7523")`
  - `KNOWN_DEVICES: dict[tuple[str, str], tuple[str, str]]`, mapping (vid, pid) → (German name, role):
    - `("1a86","7523")`: ("Board-Mikrocontroller", "board")
    - `("10c4","ea60")`: ("Lidar", "lidar")
    - `("2bc5","060f")`: ("Tiefenkamera", "camera")
    - `("2bc5","050f")`: ("Farbkamera", "camera")
    - `("046d","0825")`: ("Webcam C270", "camera")
    - `("2109","2817")`: ("Expansion-Board-Hub", "hub")
    - `("2109","0817")`: ("Expansion-Board-Hub (USB3)", "hub")
    - `("05e3","0608")`: ("Kamera-Hub", "hub")
  - `PI_SOCKETS: list[dict]`, one entry per physical socket, keys `id`, `label`, `kind`, `lanes` (root ports in device-path form), `slot` (SVG position):
    - `{"id": "pi-usb3-a", "label": "USB3 oben", "kind": "usb3", "lanes": ["1-1", "2-1"], "slot": 0}`
    - `{"id": "pi-usb3-b", "label": "USB3 unten", "kind": "usb3", "lanes": ["3-1", "4-1"], "slot": 1}`
    - `{"id": "pi-usb2-a", "label": "USB2 oben", "kind": "usb2", "lanes": ["1-2"], "slot": 2}`
    - `{"id": "pi-usb2-b", "label": "USB2 unten", "kind": "usb2", "lanes": ["3-2"], "slot": 3}`
  - `SOCKETS_VERIFIED = False`
  - `BUSY_PROCESSES = ("apt", "apt-get", "dpkg", "unattended-upgr", "dkms", "cc1", "cc1plus", "make")`, `SERVER_MARKER = "rosmaster_assist"`
- Produces in `kmsg.py`:
  - `@dataclass Event: mono: float` (s since boot), `wall: float` (epoch s), `kind: str`, `port: str | None` (device-path form, e.g. `"1-1.4"`, `"1-1"`), `details: dict`, `raw: str`
  - kinds: `"disconnect"`, `"new_device"`, `"connect"`, `"overcurrent"`, `"reset_failed"`, `"enum_failed"`, `"protocol_error"`, `"driver_attached"`, `"driver_detached"`, `"undervoltage"`, plus the synthetic `"board_silent"` made by `DiagState`
  - `normalize_port(name: str) -> str`: `"usb3-port2"` → `"3-2"`, `"1-1-port4"` → `"1-1.4"`, `"1-1.4"` → `"1-1.4"`
  - `parse_record(record: str, boot_wall: float) -> Event | None`
  - `class KmsgReader(path="/dev/kmsg", boot_wall: float)` with `fileno() -> int` and `read_available() -> list[Event]`
- Produces in `texts.py`:
  - `fmt_duration(seconds: float) -> str`, `fmt_volts(v: float | None) -> str`, `fmt_clock(wall: float) -> str` ("11:02:33")
  - `judge_ext5v(v) / judge_battery(v) / judge_temp(c) / judge_cpu(pct) -> tuple[str, str]` returning (German word, level), level ∈ `"ok" | "warn" | "bad" | "unknown"`
  - `event_sentence(kind: str, device: str, details: dict) -> str`
  - `REASONS: dict[str, dict]` with keys `motor_start`, `reenum_loop`, `heat`, `undervoltage`, `cpu_spike`, `board_mcu_silent`, `idle_unclear`, each `{"title": str, "action": str}`

- [ ] **Step 1: Write the failing tests**

```python
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

def test_fixture_burst_parses():
    lines = (FIXTURES / "kmsg_burst.txt").read_text().splitlines()
    events = [kmsg.parse_record(l, 0) for l in lines]
    assert sum(e is not None for e in events) >= 20

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
```

(`driver_attached` says "… ist bereit", so the tty name stays in the technical details.)

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_parsers.py`
Expected: `ModuleNotFoundError: No module named 'rosmaster_assist.usbdiag'`

- [ ] **Step 3: Write `config.py` and `texts.py` with the names above**

The `texts.py` copy is fixed:
- event sentences:
  - disconnect "{d} hat sich abgemeldet"
  - connect "{d} hat sich angemeldet"
  - overcurrent "Überlast an den Pi-Buchsen – der gemeinsame Stromschalter hat abgeschaltet"
  - reset_failed "Neustart des Anschlusses ist fehlgeschlagen (Leitung gestört)"
  - enum_failed "{d} konnte sich nicht anmelden (Leitung gestört)"
  - protocol_error "Übertragungsfehler auf der Leitung zu {d}"
  - driver_attached "{d} ist bereit"
  - driver_detached "{d} ist nicht mehr nutzbar"
  - undervoltage "Der Pi meldet Unterspannung am Netzteil"
  - board_silent "Das Board sendet keine Meldungen mehr, obwohl es noch angeschlossen ist"
- reason titles: motor_start "Motoranlauf", reenum_loop "Wiedereinschalt-Schleife", heat "Hitze / Taktbremse", undervoltage "Unterspannung Pi-Netzteil", cpu_spike "CPU-Lastspitze", board_mcu_silent "Board-Mikrocontroller still", idle_unclear "Im Stand, Ursache unklar (Rückspeisung?)"
- reason actions:
  - motor_start "Motoren sanfter anfahren lassen (Rampe) und Board/Hub getrennt vom Motorstrom versorgen"
  - reenum_loop "Roboter einmal ganz ausschalten; Tiefenkamera über eigenen, versorgten Hub anschließen"
  - heat "Lüfter prüfen und nicht fahren, während der Pi installiert oder kompiliert"
  - undervoltage "Pi-Netzteil und Kabel prüfen (offizielles 5-V-/5-A-Netzteil)"
  - cpu_spike "Rechenlast reduzieren; nicht fahren, während apt oder ein Compiler läuft"
  - board_mcu_silent "Akku und Board-Stromversorgung prüfen – der Mikrocontroller ist neu gestartet"
  - idle_unclear "Rückspeisung über den Board-Hub prüfen (VBUS trennen oder Hub ohne Rückspeisung)"

Decimal comma comes from formatting with `f"{v:.2f}".replace(".", ",")`. `fmt_duration` shows "N s" under 60 s, "M min S s" under 1 h, and "H h M min" above that.

- [ ] **Step 4: Write `kmsg.py`**

A record is `prio,seq,usec,flags;message`, possibly followed by continuation lines that start with a space; only the first line counts. `mono = usec / 1e6`, `wall = boot_wall + mono`. One compiled regex per kind, with these exact patterns:
- `usb (\S+): USB disconnect, device number (\d+)`
- `usb (\S+): new .*USB device number (\d+) using`
- `usb (\S+): New USB device found, idVendor=(\w+), idProduct=(\w+)`
- `usb (usb\d+-port\d+): over-current change #(\d+)`
- `usb (\S+-port\d+): cannot reset \(err = (-\d+)\)`
- `usb (\S+): device descriptor read/\S+, error (-\d+)`
- `usb (\S+): clear tt \d+ \(\w+\) error (-\d+)`
- `usb (\S+): \S+ converter now attached to (ttyUSB\d+)`
- `\S+ (ttyUSB\d+): \S+ converter now disconnected from ttyUSB\d+`
- `Undervoltage detected`

`KmsgReader` opens the path with `os.open(path, os.O_RDONLY | os.O_NONBLOCK)`. It starts at the beginning of the ring, so boot-time events are included. `read_available()` loops `os.read(fd, 8192)`: every read returns exactly one record. It stops on `BlockingIOError`, and on `BrokenPipeError` (records were overwritten) it continues.

- [ ] **Step 5: Create `fixtures/kmsg_burst.txt`**

Record about 40 real lines read-only from the Pi, then convert each to the kmsg format (`prio,seq,usec,-;msg`). Use `ssh -i ~/.ssh/id_x3 -o BatchMode=yes pi@10.42.0.62 "sudo dmesg -r --time-format raw | grep -E 'usb|ttyUSB|Undervoltage' | tail -400"`. Pick one complete burst: over-current changes, disconnects of `1-1.2`/`1-1.4`/`3-1.1`, `cannot reset`, `clear tt`, re-enumeration with `New USB device found` and `converter now attached`.

- [ ] **Step 6: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_parsers.py`
Expected: all `ok`, exit 0

- [ ] **Step 7: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag Marlon/diagnostics/test_usbdiag_parsers.py
git commit -m "Add usbdiag config, German texts and kernel log parser"
```

---

### Task 2: usbmon parser, board frame decoder and system sampler parsers

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/usbmon.py`, `sampler.py`, `fixtures/usbmon_board.txt`, `fixtures/pmic.txt`
- Test: `Marlon/diagnostics/test_usbdiag_parsers.py` (extend)

**Interfaces:**
- Consumes: `config` from Task 1
- Produces in `usbmon.py`:
  - `@dataclass Urb: ts: float` (s), `event: str` (`"S"|"C"|"E"`), `xfer: str` (`"Bi"|"Bo"|"Ci"|"Co"|"Ii"|"Io"|"Zi"|"Zo"`), `bus: int`, `dev: int`, `ep: int`, `status: int | None`, `length: int`, `data: bytes`
  - `parse_line(line: str) -> Urb | None`
  - `@dataclass Frame: type: int`, `payload: bytes` (without checksum)
  - `class FrameDecoder` with `feed(data: bytes) -> list[Frame]`, buffering across calls
  - `decode_report(frame: Frame) -> dict`, which gives:
    - `0x0A` → `{"vx", "vy", "vz", "battery_v"}`
    - `0x0C` → `{"roll", "pitch", "yaw"}`
    - `0x0D` → `{"encoders": [int, int, int, int]}`
    - anything else → `{}`
  - `@dataclass Command: func: int`, `name: str`, `params: dict`
  - `decode_commands(data: bytes) -> list[Command]` for host→board bytes
  - `FUNC_NAMES: dict[int, str]` (the Rosmaster function codes below)
  - `class UsbmonTap(root="/sys/kernel/debug/usb/usbmon")` with:
    - `ensure_loaded() -> bool`
    - `open(bus: int) -> None`, `close() -> None`
    - `fileno() -> int | None`
    - `read_available() -> list[Urb]`
    - `bus: int | None`
- Produces in `sampler.py`:
  - `parse_pmic(text: str) -> dict[str, float]` (keys like `"EXT5V_V"`, `"VDD_CORE_A"`)
  - `parse_throttled(text: str) -> dict[str, bool]` with keys `under_voltage_now`, `freq_capped_now`, `throttled_now`, `soft_temp_now`, `under_voltage_boot`, `freq_capped_boot`, `throttled_boot`, `soft_temp_boot`
  - `cpu_percent(prev: tuple[int, int], cur: tuple[int, int]) -> float` over (busy, total) jiffies
  - `read_cpu_times(text: str) -> tuple[int, int]` from the first `/proc/stat` line
  - `@dataclass Sample: mono, wall, ext5v: float | None, rails: dict, throttled: dict, temp_c: float | None, freq_mhz: float | None, cpu_pct: float | None, load1: float | None`
  - `class Sampler` with `sample() -> Sample`, which runs `vcgencmd pmic_read_adc` and `vcgencmd get_throttled` with a 2 s timeout and reads `/sys/class/thermal/thermal_zone0/temp`, `/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq`, `/proc/stat` and `/proc/loadavg`. Every failing source gives `None`, never an exception.
  - `scan_processes(proc="/proc") -> dict`, returning `{"server_running": bool, "busy": list[str]}`

- [ ] **Step 1: Write the failing tests**

```python
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
```

The reference for the frame layout is Rosmaster_Lib `__receive_data` / `__parse_data` on the Pi (`/home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py`):
- board→host: `FF FB ext_len ext_type data…`, with `ext_len - 2` bytes after `ext_type`;
- the last of those bytes is the checksum, `(ext_len + ext_type + sum(data[:-1])) % 256`.

Host→board: `FF FC len func …`, where `len = len(frame_without_checksum) - 1` and the checksum is `(sum(frame_without_checksum) + 257 - 0xFC) & 0xFF`. `decode_commands` skips frames whose checksum does not match. Before implementing, check both layouts once, read-only, against the file with `ssh … "sed -n '/def set_car_motion/,/^    def /p;/def __parse_data/,/^    def /p' /home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py"`.

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_parsers.py`
Expected: the new tests fail with `ImportError` for `usbmon`/`sampler`

- [ ] **Step 3: Implement `usbmon.py` and `sampler.py`**

- `FUNC_NAMES` = `{0x01:"AUTO_REPORT", 0x02:"BEEP", 0x03:"PWM_SERVO", 0x04:"PWM_SERVO_ALL", 0x05:"RGB", 0x06:"RGB_EFFECT", 0x0F:"RESET_STATE", 0x10:"MOTOR", 0x11:"CAR_RUN", 0x12:"MOTION", 0x13:"SET_MOTOR_PID", 0x14:"SET_YAW_PID", 0x15:"SET_CAR_TYPE", 0x20:"UART_SERVO", 0x23:"ARM_CTRL", 0x50:"REQUEST_DATA", 0x51:"VERSION", 0xA0:"RESET_FLASH"}`.
- MOTION params: `car_type`, `vx`, `vy`, `vz` (int16 little-endian / 1000). MOTOR params: `speeds` (four int8).
- Report `0x0A` uses `struct.unpack_from("<hhhB", payload)`. `0x0C` uses three int16 / 10000. `0x0D` uses `"<iiii"`.
- usbmon data words are hex groups after `=`. Join them and use `bytes.fromhex`. `<` or `>` means no data. A status token `s` (setup packet) gives `status=None`.
- `UsbmonTap.ensure_loaded()` returns True if `root` exists. Otherwise it runs `modprobe usbmon` (timeout 5 s) and checks again. `open(bus)` opens `f"{root}/{bus}u"` with `O_RDONLY | O_NONBLOCK`. `read_available()` reads up to 64 kB per call, splits on newlines and keeps a partial last line.
- `parse_pmic` matches `^\s*(\S+) (?:volt|current)\(\d+\)=([\d.]+)[VA]$` per line. The throttled bits are 0, 1, 2, 3 for now and 16, 17, 18, 19 for since boot, in the key order listed above.

`fixtures/usbmon_board.txt`: about 30 lines in the text format above, built by a throwaway script. Use real `C Bi` lines carrying `report_0a` frames (battery 11.8 V, some split across two URBs) and `S Bo` lines carrying motion commands (a stop, then a turn with vz 0.6 at a known timestamp). Use bus 1, device 114 for the board and device 113 for the lidar, plus lidar `C Bi` lines with 32 bytes each. `fixtures/pmic.txt`: the real `vcgencmd pmic_read_adc` output from the Pi, recorded read-only.

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_parsers.py`
Expected: all `ok`

- [ ] **Step 5: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag Marlon/diagnostics/test_usbdiag_parsers.py
git commit -m "Add usbmon board frame decoder and system sampler parsers"
```

---

### Task 3: sysfs topology and module layout

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/topology.py`, `layout.py`, `mock.py` (only `write_fake_sysfs` here), `fixtures/topology.json`
- Test: `Marlon/diagnostics/test_usbdiag_topology.py`

**Interfaces:**
- Consumes: `config.KNOWN_DEVICES`, `config.PI_SOCKETS`, `config.BOARD_ID`
- Produces in `topology.py`:
  - `@dataclass DeviceInfo: path: str, vid: str, pid: str, product: str, speed_mbps: int, max_power_ma: int, devnum: int, busnum: int, driver: str | None, tty: str | None`
  - `@dataclass PortInfo: path: str` (the device path a device on this port gets, e.g. `"1-1.4"`), `hub: str` (`"usb1"` for root ports, else the hub path), `number: int`, `state: str`, `over_current: int`, `peer: str | None` (device-path form), `device: DeviceInfo | None`
  - `@dataclass Topology: ports: dict[str, PortInfo]`, `devices: dict[str, DeviceInfo]`
  - `scan(root: str = "/sys/bus/usb/devices") -> Topology`
  - `device_name(dev: DeviceInfo | None) -> str` (German: "nichts" for None, else from the known table, else `„{product}“` stripped, else "Unbekanntes USB-Gerät")
  - `device_role(dev) -> str` (`"board" | "lidar" | "camera" | "hub" | "other"`)
- Produces in `layout.py`:
  - `@dataclass Tile: id: str, module: str, label: str, lanes: list[str]` (port paths), `kind: str` (`"socket" | "hubport" | "mcu" | "motor"`)
  - `@dataclass Layout: tiles: list[Tile]`, `board_hub: str | None` (e.g. `"1-1"`), `board_port: str | None` (e.g. `"1-1.4"`), `board_socket: str | None` (tile id of the Pi socket feeding the board), `board_present: bool`
  - `build_layout(topo: Topology, previous: Layout | None) -> Layout`
  - `tile_for_port(layout: Layout, port: str) -> str | None`. This is the tile whose lane is the port or the closest ancestor of it, so `"3-1.1"` → `"pi-usb3-b"` and `"1-1.4"` → `"board-p4"`.
- Produces in `mock.py`: `write_fake_sysfs(tree: dict, root: pathlib.Path) -> None`

The fake sysfs uses plain files instead of symlinks (Windows): `device` is a file whose content is the target path, and so are `peer` and `driver`. `topology` therefore reads every link through `_link_name(path) -> str | None`: `os.readlink` if it is a link, the file content if it is a file, and the basename of the target either way.

`fixtures/topology.json` describes the real Pi from 2026-11-17 (see the spec and the dump below). Its shape is `{"devices": {path: {attrs…, "interfaces": {"1.0": {"driver": "ch341", "tty": "ttyUSB1"}}}}, "ports": {sysfs_port_dir: {"state", "over_current_count", "peer", "device": bool}}}`:
- root ports: `usb1-port1` (state configured, oc 54, peer usb2-port1, device), `usb1-port2` (not attached, 31), `usb2-port1` (configured, 62, device), `usb3-port1` (configured, 56, peer usb4-port1, device), `usb3-port2` (not attached, 34), `usb4-port1` (not attached, 66)
- devices:
  - `1-1` 2109:2817 "USB2.0 Hub" 480 Mbit/s 0mA
  - `2-1` 2109:0817 "USB3.0 Hub" 5000
  - `1-1.2` 10c4:ea60 "CP2102 USB to UART Bridge Controller" 12 100mA cp210x ttyUSB0
  - `1-1.4` 1a86:7523 "USB Serial" 12 104mA ch341 ttyUSB1
  - `3-1` 05e3:0608 "USB2.0 Hub" 480 100mA
  - `3-1.1` 2bc5:060f "ORBBEC Depth Sensor" 480 500mA
  - `3-1.2` 2bc5:050f "USB 2.0 Camera" 480 500mA uvcvideo
- hub ports: `1-1:1.0/1-1-port1..4` (1, 3 not attached; 2, 4 configured; each with peer `2-1-portM`), `2-1:1.0/2-1-port1..4` (all not attached, peer `1-1-portM`), `3-1:1.0/3-1-port1..4` (1, 2 configured)

- [ ] **Step 1: Write the failing tests**

```python
def make_tree(tmp):
    tree = json.loads((FIXTURES / "topology.json").read_text())
    mock.write_fake_sysfs(tree, tmp)
    return tmp

def test_scan_real_pi_fixture():
    with tempfile.TemporaryDirectory() as d:
        topo = topology.scan(str(make_tree(pathlib.Path(d))))
    assert topo.ports["1-1"].over_current == 54 and topo.ports["1-1"].peer == "2-1"
    assert topo.ports["1-1.4"].device.vid == "1a86" and topo.ports["1-1.4"].device.tty == "ttyUSB1"
    assert topo.ports["1-1.4"].device.driver == "ch341" and topo.ports["1-1.4"].device.max_power_ma == 104
    assert topo.ports["1-1.1"].device is None and topo.ports["1-1.1"].state == "not attached"
    assert topo.ports["3-1.1"].device.product == "ORBBEC Depth Sensor"

def test_scan_survives_vanishing_files():
    with tempfile.TemporaryDirectory() as d:
        root = make_tree(pathlib.Path(d))
        (root / "1-1.4" / "idVendor").unlink()
        shutil.rmtree(root / "3-1.2")
        topo = topology.scan(str(root))
        assert "1-1.2" in topo.devices
        assert topology.scan(str(root / "missing")).ports == {}

def test_layout_detects_board_and_modules():
    with tempfile.TemporaryDirectory() as d:
        lay = layout.build_layout(topology.scan(str(make_tree(pathlib.Path(d)))), None)
    assert (lay.board_hub, lay.board_port, lay.board_socket, lay.board_present) == ("1-1", "1-1.4", "pi-usb3-a", True)
    ids = [t.id for t in lay.tiles]
    assert ids[:4] == ["pi-usb3-a", "pi-usb3-b", "pi-usb2-a", "pi-usb2-b"]
    assert {"board-p1", "board-p2", "board-p3", "board-p4", "motor"} <= set(ids)
    assert next(t for t in lay.tiles if t.id == "board-p4").kind == "mcu"
    assert next(t for t in lay.tiles if t.id == "board-p2").lanes == ["1-1.2", "2-1.2"]
    assert layout.tile_for_port(lay, "3-1.1") == "pi-usb3-b"
    assert layout.tile_for_port(lay, "1-1.4") == "board-p4"
    assert layout.tile_for_port(lay, "1-1") == "pi-usb3-a"

def test_layout_keeps_board_while_it_is_gone():
    with tempfile.TemporaryDirectory() as d:
        root = make_tree(pathlib.Path(d))
        first = layout.build_layout(topology.scan(str(root)), None)
        for p in ["1-1", "1-1.2", "1-1.4", "2-1"]:
            shutil.rmtree(root / p)
        gone = layout.build_layout(topology.scan(str(root)), first)
    assert gone.board_present is False and gone.board_hub == "1-1"
    assert "board-p4" in [t.id for t in gone.tiles] and "motor" in [t.id for t in gone.tiles]

def test_layout_follows_board_to_other_socket():
    tree = json.loads((FIXTURES / "topology.json").read_text())
    tree = rename_prefix(tree, "1-1", "3-2")  # helper in the test file: moves the board hub to black socket B
    with tempfile.TemporaryDirectory() as d:
        mock.write_fake_sysfs(tree, pathlib.Path(d))
        lay = layout.build_layout(topology.scan(d), None)
    assert lay.board_socket == "pi-usb2-b" and lay.board_port == "3-2.4"

def test_device_names():
    dev = topology.DeviceInfo("1-1.2", "10c4", "ea60", "CP2102", 12, 100, 5, 1, "cp210x", "ttyUSB0")
    assert topology.device_name(dev) == "Lidar" and topology.device_role(dev) == "lidar"
    assert topology.device_name(None) == "nichts"
    odd = topology.DeviceInfo("3-2", "dead", "beef", "Stick  ", 480, 100, 7, 3, None, None)
    assert topology.device_name(odd) == "„Stick“"
```

`rename_prefix` (in the test file) rewrites every device path, port dir name and peer that starts with `1-1` to `3-2`, and moves the root port entry from `usb1-port1` to `usb3-port2`.

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_topology.py`
Expected: `ImportError`

- [ ] **Step 3: Implement `write_fake_sysfs`, `topology.scan`, `build_layout`, `tile_for_port`**

Scan algorithm:
- For each `usbN` directory, the ports are `usbN/N-0:1.0/usbN-portM`, giving port path `N-M`.
- For each device directory `X` that has `X:1.0/X-portM`, the port path is `X.M`.
- Port attributes are `state`, `over_current_count` and the `peer` link, normalised with `kmsg.normalize_port`.
- Device attributes come from `root/<path>`: `idVendor`, `idProduct`, `product` (strip it), `speed`, `bMaxPower` ("104mA" → 104), `devnum`, `busnum`. `driver` and `tty` come from the first interface dir `<path>:*` that has a `driver` entry and a `ttyUSB*` child.
- Every read goes through one helper that returns a default on `OSError` or `ValueError`.

Layout:
- Board hub = parent of the port whose device has `BOARD_ID`. Board socket = the socket whose lanes contain `path.split(".")[0]`.
- Board tiles are `board-p1…pN` for each port of the board hub, with lanes `[hub.M, peer]`. The CH340's tile gets kind `mcu` and the label "Port N · Board-MCU"; other hub ports are labelled "Port N".
- Then one tile `motor` (kind `motor`, lanes `[board_port]`).
- If no board is found and `previous` had one, copy its board tiles and fields with `board_present=False`.

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_topology.py`
Expected: all `ok`

- [ ] **Step 5: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag Marlon/diagnostics/test_usbdiag_topology.py
git commit -m "Add sysfs topology scan and automatic module layout"
```

---

### Task 4: Incident tracking and crash reasons

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/incidents.py`
- Test: `Marlon/diagnostics/test_usbdiag_incidents.py`

**Interfaces:**
- Consumes: `kmsg.Event`, `sampler.Sample`, `usbmon.Command`, `config` thresholds, `texts.REASONS`
- Produces:
  - `TRIGGER_KINDS = {"disconnect", "overcurrent", "reset_failed", "enum_failed", "board_silent", "undervoltage"}` (`protocol_error` alone never starts an incident; it is evidence)
  - `@dataclass Incident: id: str` (`time.strftime("%Y-%m-%d_%H-%M-%S", localtime(start_wall))`), `start_mono: float`, `start_wall: float`, `last_trigger_mono: float`, `end_mono: float | None`, `events: list[Event]`, `ports: set[str]`, `missing: set[str]` (ports with a device before that are empty now), `recovered: bool`, `reasons: list[Reason]`
  - `@dataclass Reason: key: str, score: float, evidence: list[str]` (German sentences, no raw codes)
  - `class IncidentTracker(quiet_s: float, max_s: float)` with:
    - `on_event(ev: Event, present: set[str]) -> Incident | None` (returns the new incident when one starts)
    - `tick(now_mono: float, present: set[str]) -> Incident | None` (returns the incident when it ends)
    - `current: Incident | None`
  - `@dataclass Context: samples: list[Sample]`, `commands: list[tuple[float, Command]]`, `battery: list[tuple[float, float]]`, `busy: list[str]`, `server_running: bool`, `names: dict[str, str]` (port → German device name)
  - `classify(inc: Incident, ctx: Context) -> list[Reason]`, sorted by score descending

Tracker rules:
- A trigger with no open incident opens one. `present` (ports with a device right now, from `DiagState`) is recorded as the baseline, and disconnected ports are added to `missing`.
- Further events are appended. Triggers update `last_trigger_mono`, and `connect` removes a port from `missing`.
- `tick` ends the incident when `now - last_trigger_mono >= quiet_s` and every baseline port is in `present` again (`recovered=True`), or when `now - start_mono >= max_s` (`recovered=False`).

Classification rules (each makes one `Reason` with a score; evidence is German, with values formatted via `texts`):
- `motor_start` (score 0.9): a MOTION/MOTOR/CAR_RUN command in `[start - MOTOR_WINDOW_S, start]` whose speed changes by at least `MOTOR_DELTA` against the previous command. Evidence: "Fahrbefehl (Drehung 0,6 rad/s) 0,8 s vorher". Add +0.1 and the evidence "Akku fiel um 0,4 V" if the battery dropped by at least `BATTERY_DROP_V` within the window.
- `reenum_loop` (0.8): within the incident, a `connect` followed by an `overcurrent` or `disconnect` within `LOOP_REENUM_WINDOW_S`, or at least `LOOP_RESET_FAILS` `reset_failed` events. Evidence: "Tiefenkamera meldete sich an, 0,4 s später neue Überlast", "3× Neustart eines Anschlusses fehlgeschlagen".
- `heat` (0.7): max `temp_c` ≥ `TEMP_HOT_C` in the snapshot, or `throttled_now`/`soft_temp_now`.
- `undervoltage` (0.85): min `ext5v` < `EXT5V_LOW_V`, `under_voltage_now`, or an `undervoltage` event in the incident.
- `cpu_spike` (0.6): `cpu_pct` ≥ `CPU_SPIKE_PCT` within `CPU_WINDOW_S` before, or `busy` not empty ("apt lief gerade").
- `board_mcu_silent` (0.9): the incident contains a `board_silent` event.
- `idle_unclear` (0.5): added when none of `motor_start`, `heat`, `undervoltage`, `cpu_spike` or `board_mcu_silent` apply. Evidence: "keine Fahrbefehle in den 30 s davor", plus "Server lief nicht" when `server_running` is False.

- [ ] **Step 1: Write the failing tests**

```python
def ev(t, kind, port=None, **details):
    return kmsg.Event(mono=t, wall=1_700_000_000 + t, kind=kind, port=port, details=details, raw="")

def sample(t, ext5v=5.06, temp=55.0, cpu=20.0, **thr):
    return sampler.Sample(t, 0, ext5v, {}, thr, temp, 2000.0, cpu, 0.5)

def motion(vz):
    return usbmon.Command(0x12, "MOTION", {"vx": 0.0, "vy": 0.0, "vz": vz, "car_type": 1})

def started_incident(at):
    return incidents.IncidentTracker(10, 120).on_event(ev(at, "overcurrent", "1-1"), {"1-1"})

def test_burst_is_one_incident():
    tr = incidents.IncidentTracker(quiet_s=10, max_s=120)
    present = {"1-1", "1-1.2", "1-1.4"}
    started = tr.on_event(ev(100, "overcurrent", "1-1"), present)
    assert started is not None
    for i in range(30):
        assert tr.on_event(ev(100.1 + i * 0.05, "overcurrent", "1-1"), present) is None
    tr.on_event(ev(101, "disconnect", "1-1.4"), present - {"1-1.4"})
    assert tr.tick(105, present - {"1-1.4"}) is None
    tr.on_event(ev(103, "connect", "1-1.4"), present)
    done = tr.tick(113.5, present)
    assert done is started and done.recovered and "1-1.4" in done.ports

def test_protocol_errors_alone_do_not_start():
    tr = incidents.IncidentTracker(10, 120)
    assert tr.on_event(ev(5, "protocol_error", "3-2", error=-71), set()) is None

def test_timeout_ends_unrecovered():
    tr = incidents.IncidentTracker(10, 120)
    tr.on_event(ev(0, "disconnect", "1-1.4"), {"1-1.4"})
    done = tr.tick(121, set())
    assert done and not done.recovered and done.missing == {"1-1.4"}

def test_classify_motor_start():
    inc = started_incident(at=50.0)   # helper: tracker + one overcurrent at t=50
    ctx = incidents.Context([sample(49)], [(40.0, motion(0.0)), (49.2, motion(0.6))],
                            [(48.0, 11.9), (49.6, 11.4)], [], True, {})
    top = incidents.classify(inc, ctx)[0]
    assert top.key == "motor_start" and any("0,8 s vorher" in e for e in top.evidence)
    assert any("Akku" in e for e in top.evidence)

def test_classify_loop_and_idle():
    tr = incidents.IncidentTracker(10, 120)
    inc = tr.on_event(ev(10, "overcurrent", "1-1"), {"3-1.1"})
    tr.on_event(ev(12, "connect", "3-1.1", vid="2bc5", pid="060f"), {"3-1.1"})
    tr.on_event(ev(12.4, "overcurrent", "3-1"), {"3-1.1"})
    for t in (13, 14, 15):
        tr.on_event(ev(t, "reset_failed", "3-1.2", error=-71), {"3-1.1"})
    ctx = incidents.Context([sample(9)], [], [], [], False, {"3-1.1": "Tiefenkamera"})
    keys = [r.key for r in incidents.classify(inc, ctx)]
    assert keys[0] == "reenum_loop" and "idle_unclear" in keys
    loop = incidents.classify(inc, ctx)[0]
    assert any("Tiefenkamera meldete sich an" in e for e in loop.evidence)
    assert all("-71" not in e for r in incidents.classify(inc, ctx) for e in r.evidence)

def test_classify_heat_undervoltage_cpu():
    inc = started_incident(at=50.0)
    ctx = incidents.Context([sample(48, ext5v=4.70, temp=82.0, cpu=95.0)], [], [], ["dpkg"], True, {})
    keys = {r.key for r in incidents.classify(inc, ctx)}
    assert {"heat", "undervoltage", "cpu_spike"} <= keys and "idle_unclear" not in keys
```

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_incidents.py`
Expected: `ImportError`

- [ ] **Step 3: Implement `incidents.py`**

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_incidents.py`
Expected: all `ok`

- [ ] **Step 5: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag/incidents.py Marlon/diagnostics/test_usbdiag_incidents.py
git commit -m "Add USB incident tracking and crash reason rules"
```

---

### Task 5: Log files

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/store.py`
- Test: `Marlon/diagnostics/test_usbdiag_store.py`

**Interfaces:**
- Consumes: `config` retention values, `kmsg.Event`
- Produces:
  - `read_boot_id(path="/proc/sys/kernel/random/boot_id") -> str` (`"mock"` if unreadable)
  - `class Store(folder: str, boot_id: str, file_bytes: int, file_count: int, keep_incidents: int, keep_boots: int)` with:
    - `logger: logging.Logger` (writes `usb.log` through `RotatingFileHandler`)
    - `append_event(ev: Event) -> None`: one JSON line `{"boot", "mono", "wall", "kind", "port", "device", "details", "raw"}` to `events.jsonl`, rotated to `.1` and `.2` when the size would exceed `file_bytes`
    - `seen_event(boot_id: str, mono: float, raw: str) -> bool`, used on restart to skip kmsg backlog that was already written. At start it reads the highest `mono` of this boot from the tail of `events.jsonl`; an event counts as seen when its `mono` is at or below that value
    - `write_incident(data: dict) -> str`: writes `incidents/<id>.json` and prunes to the newest `keep_incidents`. Returns the path.
    - `load_incidents(boot_id: str) -> list[dict]`, newest first
    - `write_summary(data: dict) -> None`: merges into `summary.json` under `{"boots": {boot_id: data}}`, keeps `keep_boots`, writes atomically via tmp + `os.replace`
  - All created files and dirs get mode 0o644 / 0o755. When `os.geteuid() == 0` and `pwd.getpwnam(config.LOG_OWNER)` exists, they are chowned to that user. On Windows (no `pwd`) this is skipped silently.

- [ ] **Step 1: Write the failing tests**

```python
def test_events_rotate_and_stay_bounded():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", file_bytes=2000, file_count=3, keep_incidents=5, keep_boots=2)
        for i in range(200):
            st.append_event(kmsg.Event(i, 1e9 + i, "disconnect", "1-1.4", {}, "x" * 50))
        files = sorted(p.name for p in pathlib.Path(d).glob("events.jsonl*"))
        assert files == ["events.jsonl", "events.jsonl.1", "events.jsonl.2"]
        assert all(p.stat().st_size <= 2000 for p in pathlib.Path(d).glob("events.jsonl*"))

def test_seen_event_after_restart():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        st.append_event(kmsg.Event(42.0, 1e9, "disconnect", "1-1.4", {}, "r"))
        again = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        assert again.seen_event("boot-a", 42.0, "r") and not again.seen_event("boot-a", 43.0, "s")
        assert not again.seen_event("boot-b", 1.0, "r")

def test_incident_pruning_and_loading():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", 10_000, 3, keep_incidents=3, keep_boots=2)
        for i in range(5):
            st.write_incident({"id": f"2026-11-17_11-0{i}-00", "boot": "boot-a", "reasons": []})
        names = sorted(p.name for p in (pathlib.Path(d) / "incidents").iterdir())
        assert names == ["2026-11-17_11-02-00.json", "2026-11-17_11-03-00.json", "2026-11-17_11-04-00.json"]
        assert [i["id"] for i in st.load_incidents("boot-a")][0] == "2026-11-17_11-04-00"

def test_summary_keeps_boots_and_is_valid_json():
    with tempfile.TemporaryDirectory() as d:
        for b in ["b1", "b2", "b3"]:
            store.Store(d, b, 10_000, 3, 5, keep_boots=2).write_summary({"incidents": 1})
        data = json.loads((pathlib.Path(d) / "summary.json").read_text())
        assert list(data["boots"]) == ["b2", "b3"]
```

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_store.py`
Expected: `ImportError`

- [ ] **Step 3: Implement `store.py`**

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_store.py`
Expected: all `ok`

- [ ] **Step 5: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag/store.py Marlon/diagnostics/test_usbdiag_store.py
git commit -m "Add bounded USB diagnosis log files"
```

---

### Task 6: DiagState and the page JSON

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/state.py`
- Test: `Marlon/diagnostics/test_usbdiag_state.py`

**Interfaces:**
- Consumes: everything from Tasks 1–5 (`Topology`, `build_layout`, `tile_for_port`, `Event`, `Urb`, `FrameDecoder`, `decode_report`, `decode_commands`, `Sample`, `IncidentTracker`, `classify`, `Store`, `texts.*`)
- Produces:
  - `class DiagState(store: Store | None, boot_wall: float, daemon_start_mono: float)` with:
    - `apply_topology(topo: Topology, now: float) -> None`
    - `apply_event(ev: Event, backlog: bool = False) -> None`
    - `apply_urbs(urbs: list[Urb], now: float) -> None`
    - `apply_sample(s: Sample) -> None`
    - `apply_processes(p: dict) -> None`
    - `set_capture(status: str) -> None`, where status ∈ `"on" | "unavailable" | "camera_on_bus" | "no_board"`
    - `capture_target() -> tuple[int, set[int]] | None`: (bus, devnums to decode) for the board CH340 and the lidar. Returns None when there is no board, or when any device on the board's bus has role `camera` or speed ≥ 5000 (then `set_capture("camera_on_bus")`).
    - `tick(now: float) -> None`: ends incidents, raises `board_silent`, writes the summary every `SUMMARY_INTERVAL_S`
    - `page(now: float) -> dict`
  - all methods are called under one `threading.Lock` owned by `DiagState` (`self.lock`), used by the HTTP thread for `page()`

Page JSON (fixed; the JS in Task 7 renders it):

```
{
 "verdict": {"level": "ok|warn|bad", "text": "Alles stabil seit 2 h 5 min"},
 "uptime": "38 min", "running": "seit 12 min", "incident_count": 6,
 "vitals": [{"label": "5V Pi", "value": "5,07 V", "judgement": "gut", "level": "ok"}, ...],
 "sampling_note": "Spannung wird 1× pro Sekunde gemessen; sehr kurze Einbrüche sieht man nur an der Taktbremse-Anzeige.",
 "modules": [{"id": "pi|board|motor", "title": "Raspberry Pi 5|Expansion Board|Motorsteuerung",
              "note": "Lage der Buchsen noch nicht geprüft" | null,
              "tiles": [{"id", "label", "kind", "level": "ok|warn|bad|empty",
                         "device": "Expansion-Board-Hub", "line": "14× neu verbunden · seit 1 min 12 s"}]}],
 "links": [{"from": "pi-usb3-a", "to": "board", "level": "ok|bad"}, {"from": "board-p4", "to": "motor", "level": ...}],
 "details": {tile_id: {"title": "USB3 oben · Raspberry Pi 5",
                       "rows": [{"label": "Angeschlossen", "value": "..."}, ...],
                       "history": [int × HISTORY_BUCKETS], "events": [{"time": "11:02:33", "text": "..."}],
                       "reasons": [{"title", "evidence": [str], "action"}], "technical": [str]}},
 "incidents": [{"id", "time", "where": "alle Ports|Board-MCU|…", "reason": "Wiedereinschalt-Schleife",
                "duration": "12 s", "recovered": bool,
                "reasons": [{"title", "evidence", "action"}], "events": [{"time", "text"}]}],
 "motor": {"state": "Motoren stehen|Motoren laufen", "battery": "11,8 V – gut",
           "battery_curve": [float], "commanded": "vorwärts 0,10 m/s · Drehung 0,00 rad/s",
           "reported": "...", "last_command": "vor 4 min", "near_motor_start": "2 von 6 Vorfällen kurz nach Motorstart",
           "capture": "Mithören aktiv|kein Verkehr (Server aus)|Mithören nicht verfügbar|Mithören abgeschaltet: Kamera am selben USB-Bus"}
}
```

Detail rows per tile kind:
- `socket` / `hubport`: Angeschlossen (name, speed in Mbit/s or Gbit/s, "zieht bis zu 104 mA", Treiber in plain words), Dahinter (child devices), Zustand, Überlast (with the note "Pi-Buchsen teilen sich einen 1,6-A-Schalter" on Pi sockets), Verkehr (only on the capture bus), Spannung
- `mcu`: Zustand, Meldungen vom Board (Hz), Akku, letzte Fahrbefehle
- `motor`: the motor block

Tile level rules:
- `bad` when a device was there and is missing now, or an incident is open on it;
- `warn` when it reconnected within `RECENT_RECONNECT_S` or has protocol errors in the last `RECENT_RECONNECT_S`;
- `empty` when nothing was ever seen;
- `ok` otherwise.

Verdict:
- `bad` "Gerade USB-Absturz: {names} fehlen" while an incident is open;
- `warn` "Seit {time} {n} USB-Abstürze, wahrscheinlich {top reason}" when there are incidents in the last hour;
- else `ok` "Alles stabil seit {duration}".

Per-port counters:
- reconnects since boot come from kmsg `connect` events, counting the backlog;
- reconnects since daemon start skip backlog events;
- also kept per port: last connect `mono`, protocol errors, URB errors (`status < 0` in `C` URBs on capture devices), bytes/s.

Event → tile goes through `tile_for_port`, and the device name comes from the last known device of that port. On `driver_detached`, the tile is found through the remembered tty → port map.

`board_silent`: when the board port is present, a host→board command arrived in the last `BOARD_SILENT_S` and no board report arrived for `BOARD_SILENT_S`, emit one synthetic Event (once until reports resume).

Every new event is stored with `store.append_event` unless `store.seen_event` says it was already written. Each finished incident is classified with a `Context` cut from the rings (`SNAPSHOT_BEFORE_S` before start). The result is written with `store.write_incident` (incident dict = the page incident entry plus `"boot"`, `"start_wall"`, `"end_wall"`, `"snapshot"` with the raw samples and `"technical"` with the raw kmsg lines).

- [ ] **Step 1: Write the failing tests**

```python
RAW_PATTERNS = [r"-\d{2,3}\b", r"\b[0-9a-f]{4}:[0-9a-f]{4}\b", r"\busb\d", r"\b\d-\d(\.\d)*\b", r"ttyUSB", r"error", r"\b200\b"]

def main_view_strings(page):
    page = dict(page); details = {k: dict(v, technical=[]) for k, v in page.pop("details").items()}
    return [s for s in walk_strings({**page, "details": details})]  # helper: all str leaves

def loaded_state(tmp):
    st = state.DiagState(None, boot_wall=1_700_000_000.0, daemon_start_mono=0.0)
    with st.lock:
        st.apply_topology(fixture_topology(tmp), now=10.0)
        for line in (FIXTURES / "kmsg_burst.txt").read_text().splitlines():
            e = kmsg.parse_record(line, 1_700_000_000.0)
            if e: st.apply_event(e, backlog=True)
        st.apply_sample(sampler.Sample(10.0, 0, 5.06788, {}, {}, 61.0, 2000.0, 12.0, 0.4))
    return st

def test_page_shape_and_modules():
    with tempfile.TemporaryDirectory() as d:
        p = loaded_state(pathlib.Path(d)).page(now=20.0)
    assert [m["id"] for m in p["modules"]] == ["pi", "board", "motor"]
    assert p["modules"][0]["note"] == "Lage der Buchsen noch nicht geprüft"
    assert {"from": "pi-usb3-a", "to": "board", "level": "ok"} in p["links"]
    assert set(p["details"]) >= {"pi-usb3-a", "board-p4", "motor"}
    v = {x["label"]: x for x in p["vitals"]}
    assert v["5V Pi"]["value"] == "5,07 V" and v["5V Pi"]["judgement"] == "gut"

def test_main_view_has_no_raw_codes():
    with tempfile.TemporaryDirectory() as d:
        p = loaded_state(pathlib.Path(d)).page(now=20.0)
    for s in main_view_strings(p):
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
    assert {"from": "board-p4", "to": "motor", "level": "bad"} in p["links"]

def test_capture_refused_when_camera_on_board_bus():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d), add={"1-1.1": ("2bc5", "060f", 480)}), now=1.0)
        assert st.capture_target() is None
        assert "Kamera am selben USB-Bus" in st.page(2.0)["motor"]["capture"]

def test_board_reports_feed_motor_block():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        urbs = [usbmon.parse_line(l) for l in (FIXTURES / "usbmon_board.txt").read_text().splitlines()]
        st.apply_urbs([u for u in urbs if u], now=5.0)
        m = st.page(6.0)["motor"]
    assert m["battery"].startswith("11,8 V") and m["capture"] == "Mithören aktiv"

def test_board_silent_raises_incident():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        hexcmd = motion_hex(0.0, 0.0, 0.3)   # helper: usbmon data words of one MOTION frame
        for i in range(31):
            t = 1.0 + i * 0.1
            st.apply_urbs([usbmon.parse_line(f"ffff {int(t * 1e6)} S Bo:1:114:2 -115 12 = {hexcmd}")], now=t)
            st.tick(t)
        p = st.page(4.2)
        assert any("sendet keine Meldungen" in e["text"] for e in p["details"]["board-p4"]["events"])
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=20.0)
        st.tick(20.0)
        assert st.page(20.0)["incidents"][0]["reason"] == "Board-Mikrocontroller still"

def test_capture_without_traffic_says_server_off():
    with tempfile.TemporaryDirectory() as d:
        st = state.DiagState(None, 0.0, 0.0)
        st.apply_topology(fixture_topology(pathlib.Path(d)), now=1.0)
        st.set_capture("on")
        assert st.page(10.0)["motor"]["capture"] == "kein Verkehr (Server aus)"
```

The board_silent incident ends once reports resume or, as here, when the board is still present and the quiet period has passed: `board_silent` adds no port to `missing`, so the incident recovers after `INCIDENT_QUIET_S`.

The test file also defines the helpers `fixture_topology(tmp, drop=(), add={})` (writes the Task 3 fixture with ports removed or devices added, then scans it), `walk_strings`, `tile_by_id` and `motion_hex` (builds a MOTION frame like Task 2's `motion_cmd` and formats it as usbmon hex words).

Capture text rule: `"on"` with no URB from the board for 5 s → "kein Verkehr (Server aus)"; `"on"` with traffic → "Mithören aktiv"; `"unavailable"` → "Mithören nicht verfügbar"; `"camera_on_bus"` → "Mithören abgeschaltet: Kamera am selben USB-Bus"; `"no_board"` → "Board nicht angeschlossen".

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_state.py`
Expected: `ImportError`

- [ ] **Step 3: Implement `state.py`**

Rings: samples as a `deque` of `SAMPLE_RING_S / SAMPLE_INTERVAL_S` entries; commands and battery points with time-based trimming at `SAMPLE_RING_S`; per-port events as `deque(maxlen=PORT_EVENT_RING)`. The history row counts incidents per `HISTORY_BUCKET_S` bucket over the last `HISTORY_BUCKETS` buckets.

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_state.py`
Expected: all `ok`

- [ ] **Step 5: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag/state.py Marlon/diagnostics/test_usbdiag_state.py
git commit -m "Add USB diagnosis state and German page model"
```

---

### Task 7: Daemon, web server, mock mode and the page

**Files:**
- Create: `robot/src/rosmaster_assist/usbdiag/web.py`, `daemon.py`, `__main__.py`; complete `mock.py`
- Create: `robot/src/rosmaster_assist/usbdiag/web/index.html`, `web/usbdiag.css`, `web/usbdiag.js`
- Test: `Marlon/diagnostics/test_usbdiag_web.py`

**Interfaces:**
- Consumes: `DiagState` (Task 6), `KmsgReader`, `UsbmonTap`, `Sampler`, `scan_processes`, `topology.scan`, `Store`, `read_boot_id`
- Produces:
  - `web.make_server(state: DiagState, host: str, port: int) -> ThreadingHTTPServer`, serving:
    - `GET /` → `index.html`
    - `GET /usbdiag.js` and `GET /usbdiag.css` → the files from `web/`
    - `GET /api/state` → `json.dumps(state.page(time.monotonic()))` under `state.lock`, with `Cache-Control: no-store`
    - anything else → 404 with a German plain-text body
  - `class Sources` (protocol used by the daemon loop), with:
    - `kmsg_fd() -> int | None`, `read_kmsg() -> list[Event]`
    - `scan_topology() -> Topology`, `sample() -> Sample`, `processes() -> dict`
    - `ensure_capture() -> bool`, `open_capture(bus: int) -> None`, `capture_fd() -> int | None`, `read_capture() -> list[Urb]`
    - `now() -> float` (monotonic), `boot_wall() -> float`
  - `daemon.RealSources()` and `mock.MockSources(speed: float = 1.0)` implement `Sources`.
    - The mock writes the fake sysfs from `fixtures/topology.json` into a temp dir.
    - It replays the kmsg backlog at start.
    - Every 90 s it plays a burst: overcurrent, disconnect of `1-1.2`/`1-1.4`/`3-1.1` (it removes those dirs from the fake sysfs and restores them 2–4 s later), `connect` events, and one loop round of the Orbbec. Every third burst comes 1 s after a motion command, so `motor_start` shows up.
    - It feeds `usbmon_board.txt` lines in a loop at their recorded rate.
    - Samples are synthetic: EXT5V 5.02–5.07 V, 55–62 °C, CPU 10–30 %.
  - `daemon.run(sources: Sources, store: Store, host: str, port: int, stop: threading.Event | None = None) -> None`
  - `__main__`: `python -m rosmaster_assist.usbdiag [--mock] [--port 8001] [--log-dir DIR]`. In mock mode the default log dir is `~/.rosmaster/diagnose/usb-mock`.

Daemon loop:
- One thread runs the HTTP server; the main thread runs the loop.
- `select` on the kmsg and capture fds with a 0.25 s timeout. Topology is scanned every `TOPOLOGY_POLL_S`, and immediately after any kmsg USB event (debounced 0.2 s). Samples are taken every `SAMPLE_INTERVAL_S`, processes every `PROCESS_INTERVAL_S`, and `state.tick` runs every loop.
- After each topology scan, `capture_target()` decides whether to (re)open the tap on the board's bus.
- Each source call sits in its own `try/except Exception`. On error the daemon logs one line to `usb.log` (repeats of the same error are rate-limited to one per minute), drops that fd and retries after 5 s. A source error never ends the loop.
- On start, the kmsg backlog is applied with `backlog=True` before the first page is served.

Page (`web/`, German, dark look with the colour variables from `webapp/style.css`, copied because the daemon is installed on its own):
- Header: "USB-Diagnose", uptime, incident count, and a link "← Roboter-Diagnose" to `http://<host>:8000/diagnose`.
- Verdict bar in the level colour, with one large sentence.
- Vitals strip: chips "5V Pi 5,07 V · gut".
- Centre: an inline SVG wiring diagram, drawn by JS from `modules` and `links`:
  - the Pi 5 as a board shape with four sockets, placed by `slot` (2×2, blue USB3 / black USB2 frames);
  - a cable path to the expansion board box with N hub ports;
  - a cable from `board-p4` to the motor controller box with four little wheels;
  - peripheral devices as labelled chips next to their socket;
  - every socket, port and module is a `<g>` with `role="button"`, `tabindex="0"` and fill by level;
  - links with level `bad` get `stroke-dasharray` and the danger colour, and an open incident pulses (CSS animation, disabled under `prefers-reduced-motion`);
  - the note "Lage der Buchsen noch nicht geprüft" sits under the Pi drawing.
- Detail panel (right side on wide screens, below on narrow): title, rows as a definition list, a history bar row (12 bars, the last hour), recent events, reasons with evidence and action, and a collapsed `<details><summary>Technische Details</summary>` with the `technical` lines in monospace.
- Incident list: newest first. Each row (time, where, reason, duration) expands in place to its reasons and events.
- JS polls `/api/state` every 1 s, keeps the selected tile and the open incidents across refreshes, and on fetch failure shows "Verbindung zum Diagnose-Dienst verloren – versuche erneut …" in the verdict bar without clearing the last data.
- Use the frontend-design skill for this page.

- [ ] **Step 1: Write the failing tests**

```python
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
        page = json.loads(get(port, "/api/state"))
        assert [m["id"] for m in page["modules"]] == ["pi", "board", "motor"]
        assert status(port, "/../config.py") == 404
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
    port, stop, d = start_mock(); time.sleep(1); stop.set(); time.sleep(0.5)
    n1 = len(pathlib.Path(d, "events.jsonl").read_text().splitlines())
    # second run on the same folder replays the same backlog: no line may be added for it
    src = mock.MockSources(speed=0.0)  # speed 0: backlog only, no live bursts
    st = store.Store(d, "mock", 1_000_000, 3, 200, 20)
    stop2 = threading.Event(); threading.Thread(target=daemon.run, args=(src, st, "127.0.0.1", free_port(), stop2), daemon=True).start()
    time.sleep(1); stop2.set()
    assert len(pathlib.Path(d, "events.jsonl").read_text().splitlines()) == n1

def test_source_errors_do_not_stop_the_loop():
    src = mock.MockSources(speed=50.0)
    def broken():
        raise OSError("gone")
    src.scan_topology = broken
    d = tempfile.mkdtemp()
    st = store.Store(d, "mock", 1_000_000, 3, 200, 20)
    stop = threading.Event(); port = free_port()
    threading.Thread(target=daemon.run, args=(src, st, "127.0.0.1", port, stop), daemon=True).start()
    try:
        wait_until(lambda: get(port, "/api/state") is not None, 5)
        time.sleep(1.5)
        assert get(port, "/api/state") is not None
        log = pathlib.Path(d, "usb.log").read_text(encoding="utf-8")
        assert log.count("topology") == 1
    finally:
        stop.set()
```

`free_port`, `get` (returns the body or None on any error), `status` and `wait_until` are small helpers in the test file (`urllib.request`, socket bind to port 0). `MockSources(speed=0.0)` replays only the kmsg backlog, with no live bursts and no usbmon lines.

- [ ] **Step 2: Run the tests and check they fail**

Run: `python Marlon/diagnostics/test_usbdiag_web.py`
Expected: `ImportError`

- [ ] **Step 3: Implement `web.py`, `daemon.py`, `mock.MockSources`, `__main__.py`**

`RealSources.boot_wall()` = `time.time() - float(open("/proc/uptime").read().split()[0])`.

- [ ] **Step 4: Run the tests and check they pass**

Run: `python Marlon/diagnostics/test_usbdiag_web.py`
Expected: all `ok`

- [ ] **Step 5: Build the page with the frontend-design skill**

- [ ] **Step 6: Look at it in the mock**

Run: `python -m rosmaster_assist.usbdiag --mock` (from `robot/src`), open `http://localhost:8001/`. Check:
- the diagram shows Pi, board and motor, with the board cable on "USB3 oben";
- clicking each socket and port opens its details;
- a mock burst turns the board red and dashed, then yellow;
- the incident list shows "Wiedereinschalt-Schleife" and "Motoranlauf";
- the main view has no codes;
- "Technische Details" is collapsed.

Take a screenshot for the project log.

- [ ] **Step 7: Run all tests**

Run: `python scripts/run_tests.py`
Expected: all scripts pass

- [ ] **Step 8: Commit**

```bash
git add robot/src/rosmaster_assist/usbdiag Marlon/diagnostics/test_usbdiag_web.py
git commit -m "Add USB diagnosis daemon, mock mode and clickable German page"
```

---

### Task 8: systemd unit, install script, server link, docs

**Files:**
- Create: `robot/deploy/usbdiag/rosmaster-usbdiag.service`, `robot/deploy/usbdiag/install.sh`
- Modify: `webapp/diagnose.html:22-25` (header nav), `webapp/diagnose.js` (set the link target)
- Modify: `docs/hardware.md` (new section "USB diagnosis" after "USB resets (findings from 28.10.)"), `docs/architecture.md` (module table row and one paragraph), `Marlon/PROJECT_LOG.md` (new entry after the last `###` of section 2, plus open points in section 5)

**Interfaces:**
- Consumes: `python3 -m usbdiag` from Task 7 (in `/opt/rosmaster-usbdiag` the package is top-level)

- [ ] **Step 1: Write the unit file**

```ini
[Unit]
Description=ROSMASTER X3 USB diagnosis (page on port 8001)
After=local-fs.target

[Service]
Type=simple
WorkingDirectory=/opt/rosmaster-usbdiag
ExecStart=/usr/bin/python3 -m usbdiag
Restart=always
RestartSec=5
Nice=10
IOSchedulingClass=idle
CPUQuota=10%
MemoryMax=150M

[Install]
WantedBy=multi-user.target
```

- [ ] **Step 2: Write `install.sh`**

`set -euo pipefail`. Find the repo root from the script path. Then:
- `sudo rm -rf /opt/rosmaster-usbdiag/usbdiag` and `sudo mkdir -p /opt/rosmaster-usbdiag`;
- `sudo cp -r robot/src/rosmaster_assist/usbdiag /opt/rosmaster-usbdiag/`;
- copy the unit to `/etc/systemd/system/`, then `daemon-reload`, `enable --now rosmaster-usbdiag`;
- print `USB-Diagnose: http://$(hostname -I | awk '{print $1}'):8001/`.

`install.sh --uninstall` disables the unit and removes the unit file and `/opt/rosmaster-usbdiag`, but keeps the logs. The script never touches `/dev/myserial` or the server.

- [ ] **Step 3: Check that the copied package runs standalone**

Run (laptop, Git Bash): `d=$(mktemp -d) && cp -r robot/src/rosmaster_assist/usbdiag "$d"/ && cd "$d" && timeout 5 python -m usbdiag --mock --port 8011; echo exit=$?`
Expected: no `ImportError`; `exit=124` (stopped by timeout while serving)

- [ ] **Step 4: Add the link to the server's diagnosis page**

In `webapp/diagnose.html`, inside `<nav class="header-nav">` before "Einstellungen", add `<a id="usb-diag-link" class="header-link" href="#" title="Alle USB-Anschlüsse und Abstürze im Detail">USB-Diagnose</a>`. In `webapp/diagnose.js`, at start, set `document.getElementById("usb-diag-link").href = location.protocol + "//" + location.hostname + ":8001/"`.

- [ ] **Step 5: Write the docs**

`docs/hardware.md`, section "USB diagnosis":
- what it watches and why it is independent of the server;
- install and update (`git pull` + `robot/deploy/usbdiag/install.sh`);
- the page URL;
- the log files and how to read an incident file;
- what the crash reasons mean;
- the usbmon limitation (no traffic while the server is off);
- the 1 Hz voltage caveat;
- the **one-time socket check**: plug a USB stick into each Pi socket in turn, note which tile lights up, then fix the labels in `usbdiag/config.py` `PI_SOCKETS` and set `SOCKETS_VERIFIED = True`.

`docs/architecture.md`: add a table row for `usbdiag/` and two sentences on why it is a separate stdlib-only service installed to `/opt`.

`Marlon/PROJECT_LOG.md`: an entry "### 17.11.2026 – USB diagnosis daemon with its own page (Marlon, with Claude)" covering what, why, the result (mock screenshot, test count) and problems. Section 5 gets the open points: Pi installation, socket check, first incident analysis.

- [ ] **Step 6: Run all tests**

Run: `python scripts/run_tests.py`
Expected: all scripts pass

- [ ] **Step 7: Commit and push**

```bash
git add robot/deploy/usbdiag webapp/diagnose.html webapp/diagnose.js docs/hardware.md docs/architecture.md Marlon/PROJECT_LOG.md
git commit -m "Add USB diagnosis service install, server link and docs"
git push -u origin usb-diagnose
```

---

### Task 9: Verification on the Pi (ask Marlon before each state change)

**Files:**
- Modify: `robot/src/rosmaster_assist/usbdiag/fixtures/*`, `docs/hardware.md`, `Marlon/PROJECT_LOG.md` (results only)

- [ ] **Step 1: Pull on the Pi without switching the running checkout's branch for good**

Run: `ssh -i ~/.ssh/id_x3 -o BatchMode=yes pi@10.42.0.62 "cd ~/gitpipeline/SPE-ROSMASTER-X3 && git fetch && git worktree add -f /tmp/usbdiag-src origin/usb-diagnose"`
Expected: the worktree is created; the main checkout is unchanged (`git status` shows the same branch as before)

- [ ] **Step 2: After Marlon says yes, install**

Run: `ssh … "cd /tmp/usbdiag-src && bash robot/deploy/usbdiag/install.sh && systemctl is-active rosmaster-usbdiag"`
Expected: `active`, and the URL is printed

- [ ] **Step 3: Check the real decoding and load**

Run: `ssh … "curl -s localhost:8001/api/state | python3 -m json.tool | head -60; ls -la /home/pi/.rosmaster/diagnose/usb; ps -o %cpu,rss,cmd -C python3 | grep usbdiag; journalctl -u rosmaster-usbdiag -n 20 --no-pager"`
Expected:
- modules pi/board/motor; the board socket is "USB3 oben";
- `events.jsonl` is owned by `pi`;
- CPU below 3 % after a minute (re-check with `top -b -n 3 -d 10`);
- no tracebacks.

- [ ] **Step 4: Server on and off**

With the server off, the motor capture says "kein Verkehr (Server aus)". After Marlon starts the server (`python main.py`), the battery voltage appears (≈ the value shown on `/diagnose`) and a "Motoren stehen" state follows. Record whether a turn command shows up as "Motoren laufen". If the battery voltage differs from `/diagnose`, fix the byte offset in `decode_report` and add a test with the real frame.

- [ ] **Step 5: Real fixtures and docs**

Replace `fixtures/pmic.txt` and part of `usbmon_board.txt` with real recordings (`sudo cat /sys/kernel/debug/usb/usbmon/1u | head -200`). Re-run the tests on the laptop. Note the CPU figure, the first real incidents and their reasons in `PROJECT_LOG.md`. Commit and push.

- [ ] **Step 6: Clean up the Pi worktree**

Run: `ssh … "cd ~/gitpipeline/SPE-ROSMASTER-X3 && git worktree remove /tmp/usbdiag-src"`
Expected: removed; `/opt/rosmaster-usbdiag` keeps running
