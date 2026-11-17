# USB diagnosis daemon: design

Date: 2026-11-17 · Branch: `usb-diagnose` (from `main`) · Status: approved by Marlon 2026-11-17

## Intent

The X3 keeps losing its USB devices. When that happens the board (drive), the
lidar and the camera drop off together, the robot stops being usable, and
today nobody can say afterwards *why* it happened. The existing `/diagnose`
page of the server only counts over-currents and only works while the server
runs, which is exactly when the information is least needed.

We want a diagnosis that runs permanently in the background, independent of
the server, records every USB event with its context, and explains it on a
web page that is extremely easy to read: one tile per USB port of each of the
three modules (Raspberry Pi 5, Yahboom expansion board, motor controller).
Clicking a port shows what is connected, what is happening on it right now,
what happened before, the relevant voltages, and the most likely reason for
each crash. The goal is to finally find the root cause of the resets and to
prove a fix works.

### What we saw on the Pi on 2026-11-17 (read-only check)

- 29 minutes after boot: 161 `over-current change` messages and 96
  `USB disconnect` in `dmesg`, in bursts (10:36, 10:37, 10:40, 10:44,
  11:02, 11:03). Device numbers already at 90.
- All Pi ports report over-current together (counters 24 to 49 each), which
  matches the known shared 1.6 A power switch of the Pi 5.
- The resets happened **while the server was not running**, so without any
  motor command from us. Motor load alone does not explain them.
- EXT5V (Pi supply) read 4.98 to 5.07 V at rest, `throttled=0x0`.
- Topology after re-enumeration:
  - `usb2-port1` (USB3 lane of a blue socket): VIA hub `2109:0817`
  - `usb1-port1` (USB2 lane of the same socket): VIA hub `2109:2817`, with
    `1-1.2` lidar CP2102 (`10c4:ea60`, `/dev/rplidar`) and `1-1.4` board
    CH340 (`1a86:7523`, `/dev/myserial`)
  - `usb3-port1`: Orbbec depth `3-1.1` (`2bc5:060f`) and camera `3-1.2`
    (`2bc5:050f`)
- `usbmon.ko` exists but is not loaded; `/sys/kernel/debug/usb` is mounted.
- `vcgencmd pmic_read_adc` works and gives all PMIC rails.

## Goals

1. A background service (systemd, starts at boot, restarts on failure) that
   is completely independent of the `rosmaster_assist` server.
2. It writes log files that survive a reboot and are useful without the web
   page (for the project log and for comparing before/after a fix).
3. Its own web page on port **8001**, reachable even when the server is down
   or crashing.
4. One tile per USB port, grouped by module:
   - **Raspberry Pi 5**: the four physical sockets (2× USB3 blue, 2× USB2
     black), each showing both its USB2 and USB3 lane.
   - **Expansion board**: the ports of its hub (VIA, 4 ports), plus the
     board's own microcontroller link (CH340 → STM32).
   - **Motor controller**: no USB of its own; shown as a module tile fed
     from the board link (see below).
5. Click on a tile → detail view: connected device (name, IDs, driver,
   `/dev` path, speed, declared power draw), current state, time since last
   (re)connect, number of reconnects, over-current counters, data rate,
   transfer errors, recent events, voltages at that moment, and the crash
   reasons for each incident on that port.
6. The port mapping is **detected automatically** from the USB topology, so
   moving a cable to another socket needs no code change. Today the board's
   hub sits on a blue USB3 socket and everything else is peripherals; the page
   must show that without it being hard-coded.
7. Each incident gets a **crash reason** in plain German with the evidence
   behind it (what happened in the seconds before), not just a counter.

## Non-goals

- No additional hardware (no USB power meter, no oscilloscope, no ADC).
- No fix of the resets themselves. The tool finds and proves; the fix is a
  separate step.
- No replacement of the server's `/diagnose` page; it only gets a link to
  the new page.
- No ROS, no third-party Python packages (stdlib only, so the service runs
  even if the server's venv is broken).

## Anti-goals (would make it a failure even if it works)

- **The diagnosis must not cause resets.** The known trigger is CPU load
  spikes. The daemon must stay well below 3 % of one core on average and
  must never capture high-bandwidth traffic (camera streams).
- **It must never touch the drive path.** It does not open `/dev/myserial`
  or `/dev/rplidar`, does not send anything to the board, and cannot make
  the robot move or stop.
- **It must not fill the SD card.** All logs are size-bounded.
- **It must not need anyone to read raw kernel logs.** If the page only
  shows numbers that need interpretation, it has failed.

## Constraints

- Code, comments, logs, tests, docs and commits in English; everything the
  user sees on the page in German (AGENTS.md).
- Settings in `config.py` (or a section of it the daemon can import without
  pulling in the server; see "Left to the builder").
- Runs as root under systemd (needed for usbmon and `/dev/kmsg`); files it
  writes must be readable by user `pi`.
- Must survive the USB devices it watches disappearing and reappearing at
  any moment, including in the middle of a read.
- Must run on a laptop in a mock mode, so tests run with
  `python scripts/run_tests.py` on Windows.
- Only one program may own a serial port: the daemon never opens one.

## Approaches considered

**A. Extend the server's existing diagnostics.** Cheapest, reuses
`diagnostics.py` and `/diagnose`. Rejected: dies with the server, and the
server is not running during most resets (as seen today).

**B. Standalone daemon with its own small web page on port 8001.**
*(chosen by Marlon)* Survives server crashes and is running from boot, so it
also sees resets during boot, while compiling, and with the server stopped.
Cost: a second HTTP server and a systemd unit.

**C. Standalone daemon that only writes files; the server shows them.**
Keeps one web page, but the page is gone exactly when it matters (server
down). Rejected for that reason.

### Reading the board without opening the serial port (chosen: passive usbmon)

Marlon asked whether the daemon can read the board's serial port. Opening it
would collide with the server (only one reader per port). Instead the daemon
**listens passively with Linux usbmon** to the CH340's USB traffic on the
board's bus. That sees both directions without taking part:

- board → Pi: the auto reports (`FF FB len type data… checksum`). Report
  type `0x0A` carries vx, vy, vz (int16 /1000) and the **battery voltage**
  (byte 6, in 0.1 V). Other types: `0x0B`/`0x0E` raw IMU, `0x0C` attitude,
  `0x0D` encoders (verify the exact layout against
  `/home/pi/software/py_install/Rosmaster_Lib/Rosmaster_Lib.py`).
- Pi → board: every command the server sends (`FF FC len func …`),
  including motion commands, so the daemon knows exactly when the motors
  were told to start.
- URB status codes per device (e.g. `-71` protocol error, `-108` shutdown),
  which show a dying link before the disconnect.

Limitation, stated on the page: usbmon only sees data while some program has
the port open. With the server stopped the board link shows "kein Verkehr
(Server aus)" and the battery voltage is unknown. That is acceptable; it is
also evidence (resets with no traffic at all rule out motor commands).

Fallback if usbmon cannot be loaded: everything else still works; board and
motor tiles say "Mithören nicht verfügbar".

usbmon is loaded by the daemon at start (`modprobe usbmon`). Capture is
limited to the bus the board's CH340 is on (today bus 1, found from sysfs at
runtime; it moves if the cable moves). Other buses are watched through sysfs
and the kernel log only, never captured, to keep CPU load low.

## Design

### Data sources

| Source | What it gives | Rate |
|---|---|---|
| sysfs `/sys/bus/usb/devices` | topology, IDs, product strings, driver, speed, `bMaxPower`, `devnum`, port `over_current_count` (root hub and VIA hub ports), port state | poll ~1 s, plus immediately after each kernel USB message |
| `/dev/kmsg` | every kernel USB line with monotonic timestamp: connect, disconnect, over-current change, `cannot reset (err = -71)`, enumeration failures, driver attach (`ttyUSB*`) | streamed |
| usbmon on the board's bus | board reports (battery, speeds, IMU), host commands (motor start/stop), URB errors, bytes/s for lidar and board | streamed |
| `vcgencmd pmic_read_adc` | EXT5V_V and the other PMIC rails (voltages, currents) | ~1 s |
| `vcgencmd get_throttled` | under-voltage / throttling, current and sticky bits | ~1 s |
| `/sys/class/thermal`, cpufreq, `/proc/stat`, `/proc/loadavg` | temperature, clock, CPU load | ~1 s |
| process list | server running? `apt`/`dpkg`/compiler running? | ~5 s |

A sampled voltage at 1 Hz misses millisecond dips; the page must say so. The
sticky throttling bits and the board's battery voltage stream fill some of
that gap.

### Model

- **Module** → **Port** → **Device**. A port is identified by its sysfs path
  (`1-1.4`), a physical socket by its root-hub port. The four Pi 5 sockets
  are a fixed table (each blue socket = one USB2 + one USB3 root port, each
  black socket = one USB2 root port); the builder verifies the table on the
  real Pi (labels "oben/unten", "links/rechts" as seen from the robot's back).
- **Module detection**: the hub whose subtree contains the CH340
  `1a86:7523` is the expansion board hub; the CH340 port is the board's
  microcontroller link; the motor controller hangs off that link. Everything
  else is "Peripherie" and labelled by a known-device table (lidar, Orbbec,
  C270, microphone, speaker, VIA hub) with a generic fallback.
- **Port history**: each port keeps a ring of its last events and counters
  since boot and since daemon start (reconnects, over-current, URB errors,
  time present/absent).

### Incidents and crash reasons

An **incident** starts with the first disconnect or over-current after a quiet
period and ends after the devices are back (or a timeout). Bursts of dozens of
kernel lines become one incident, not dozens.

On each incident the daemon freezes a snapshot of the last ~30 s before and
the time until recovery (from in-memory ring buffers) and classifies it. First
version of the rules (German labels on the page, each with the evidence shown):

| Reason | Evidence |
|---|---|
| Motoranlauf | a motion command / speed change in the ~2 s before, battery voltage drop |
| Wiedereinschalt-Schleife | the first device re-enumerated triggers the next over-current within seconds (Orbbec, lidar motor inrush); repeated `err = -71` |
| Hitze / Taktbremse | temperature high or throttling bits set |
| Unterspannung Pi-Netzteil | EXT5V low or under-voltage bit |
| CPU-Lastspitze | CPU load jump just before, compiler/apt running |
| Board-Mikrocontroller still | CH340 still enumerated but board reports stopped (STM32 brownout or reset) instead of a USB loss |
| Im Stand, Ursache unklar (Rückspeisung?) | none of the above, no traffic, robot idle: points at backfeed from the board's hub (known suspect, docs/hardware.md) |

Several reasons can apply at once. The page shows them ranked with the
evidence, never just a word. The rules live in one place so they can be
refined from recorded incidents.

### Log files

Under `/home/pi/.rosmaster/diagnose/usb/` (next to the server's diagnostics,
owned by `pi`):

- `usb.log`: human-readable daemon log, rotated (same size limit as the
  server's, 3 files).
- `events.jsonl`: one line per USB event (time, port, device, kind, details),
  rotated by size.
- `incidents/YYYY-MM-DD_HH-MM-SS.json`: one file per incident with the
  snapshot, the reasons and the evidence; the newest N are kept.
- `summary.json`: per boot: uptime, number of incidents, resets per hour,
  per port, by reason. Used for before/after comparisons of a fix.

All times as wall clock plus monotonic; boot ID recorded so logs from
different boots are never mixed.

### Presentation rules (Marlon, 2026-11-17: "sehr übersichtlich, Fehler für Menschen verständlich, keine 200-Codes, grafisch schön")

- **No raw codes in the main view.** Kernel errors, URB status codes, USB
  IDs, sysfs paths and HTTP status never appear as the message. Every event
  is a plain German sentence ("Das Board hat sich kurz abgemeldet und war
  nach 2 Sekunden wieder da", not "usb 1-1.4: USB disconnect" or "-71").
  Raw details exist only in a collapsed "Technische Details" box at the
  bottom of the detail view, for debugging.
- **Graphical, clickable wiring diagram** as the centre of the page: an SVG
  drawing of the three modules (Pi 5 with its four sockets where they really
  are, the cable to the expansion board, its hub ports, the motor
  controller behind the board), each port and module clickable, coloured by
  state, cables drawn between them, a failing link drawn red/dashed. The
  tile lists in the mockups below are the same information; the diagram
  replaces them visually.
- **One-sentence verdict on top**: "Alles stabil seit 2 h" or "Seit 10:36
  sechs USB-Abstürze, wahrscheinlich Wiedereinschalt-Schleife". The user
  should know the state without clicking.
- Voltages, counters and times always with units and a judgement word
  ("5,06 V – gut"), never a bare number.
- Recommended action in plain words for each crash reason.

### Web page (port 8001, German)

Served by the daemon itself (stdlib HTTP), static HTML/JS plus a JSON
endpoint polled about once per second. Same look as the existing
`/diagnose` page (`webapp/style.css`). No login, LAN only, like the server.

```
┌ USB-Diagnose ─────────────────────────── Läuft seit 29 min · 6 Vorfälle ┐
│ Zustand: ⚠ USB-Sturm vor 2 min (Wiedereinschalt-Schleife)               │
│ 5V Pi 5,06 V · Akku 11,8 V · 61 °C · 2,0 GHz · keine Taktbremse         │
├ Raspberry Pi 5 ─────────────────────────────────────────────────────────┤
│ [USB3 oben ● Expansion-Board-Hub]  [USB3 unten ● Orbbec]                 │
│ [USB2 oben ○ frei]                 [USB2 unten ○ frei]                   │
├ Expansion Board ────────────────────────────────────────────────────────┤
│ [Port 1 ○ frei] [Port 2 ● Lidar] [Port 3 ○ frei] [Port 4 ● Board-MCU]    │
├ Motorsteuerung ─────────────────────────────────────────────────────────┤
│ [● Motoren stehen · Akku 11,8 V · letzte Fahrbefehle vor 4 min]          │
├ Vorfälle ───────────────────────────────────────────────────────────────┤
│ 11:02:33  alle Ports  Wiedereinschalt-Schleife   12 s  ▸                 │
│ 10:44:10  alle Ports  Im Stand, Ursache unklar    3 s  ▸                 │
└─────────────────────────────────────────────────────────────────────────┘
```

Tile colour: green = connected and quiet, yellow = reconnected recently or
errors, red = missing now or in a loop, grey = empty. Each tile shows device
name, reconnects since boot and time since last reconnect.

Detail view of a port (opens on click):

```
USB3 oben · Raspberry Pi 5                                      [schließen]
Angeschlossen: VIA USB3.0 Hub (2109:0817), Treiber hub, 5 Gbit/s
               Dahinter: Lidar (Port 2), Board-MCU (Port 4)
Zustand:       verbunden seit 1 min 12 s · 14× neu verbunden seit Boot
Überlast:      46× seit Boot (Pi-Buchsen teilen sich einen 1,6-A-Schalter)
Verkehr:       Board 1,2 kB/s · Lidar 9,8 kB/s · 0 Übertragungsfehler
Spannung:      5V Pi 5,06 V (min. heute 4,93 V) · Akku 11,8 V
Verlauf:       ▁▁▁█▁▁▁▁██▁▁  (Abstürze der letzten 60 min)
Letzte Ereignisse:
  11:02:33  Überlast an allen Pi-Buchsen
  11:02:34  getrennt: Lidar, Board-MCU
  11:02:36  wieder da: Board-MCU (ttyUSB1)
Absturzgründe (letzter Vorfall):
  1. Wiedereinschalt-Schleife: Orbbec meldete sich an, 0,4 s später neue
     Überlast; 3× „cannot reset (err = -71)“
  2. Im Stand: keine Fahrbefehle in den 30 s davor
```

The motor controller tile shows: battery voltage (= motor supply) with a
short curve, commanded vs reported speeds, motors running/stopped, and how
many incidents started within 2 s of a motor start.

### Packaging

- New subpackage in `robot/src/rosmaster_assist/` (e.g. `usbdiag/`), run as
  `python -m rosmaster_assist.usbdiag`, `--mock` for the laptop.
- systemd unit file in the repo plus an install script; `docs/hardware.md`
  gets an "USB diagnosis" section (install, page URL, log files, how to read
  an incident); `docs/architecture.md` mentions the daemon.
- The server's `/diagnose` page gets a link "USB-Diagnose" to
  `http://<host>:8001/`.
- Project log entry in `Marlon/PROJECT_LOG.md`.

## Left to the builder

- Module layout inside the subpackage, the exact JSON shape of the page
  endpoint, polling vs. server-sent updates.
- usbmon text vs. binary interface, as long as board frames are decoded
  reliably at the report rate and CPU stays within the anti-goal.
- How config values are shared: importing `rosmaster_assist.config` is fine
  if it stays stdlib-only at import time; otherwise a small config section of
  its own.
- Exact incident start/end timing and ring buffer lengths (defaults:
  quiet period 10 s, snapshot 30 s before).
- The known-device table and the mock data (recorded from the real Pi:
  `lsusb -t`, sysfs dump, the `dmesg` of today's bursts).
- Retention numbers (default: logs 2 MB × 3, 200 incident files).

## Calls made after the builder check (Claude, 2026-11-17; Marlon can override)

- **Install location: a fixed copy, not the git checkout.** The Pi's single
  checkout switches branches (`main`, `interaktion`, …); a unit pointing into
  it would break or silently change version on every checkout and spoil
  before/after comparisons. An install script copies the stdlib-only
  subpackage to `/opt/rosmaster-usbdiag/` and the unit runs it with the
  system `python3`. Updating = `git pull` + run the install script again.
  This also means the branch need not be checked out on the Pi permanently.
- **Own config module inside the subpackage** (`usbdiag/config.py`), because
  the copied daemon must not depend on the server's `config.py` and its
  imports. Same spirit as the AGENTS.md rule: all numbers in one config
  file, none in the code.
- **Physical socket labels are verified with a person at the robot.** SSH
  cannot tell which socket is which. The socket table ships with the
  labels derived from the Pi 5 layout and is marked unverified on the page
  ("Lage der Buchse noch nicht geprüft") until Marlon confirms it with the
  one-time check described in `docs/hardware.md` (plug one device into each
  socket and watch the page). Everything else is verified over SSH: report
  layout, mock recordings, usbmon decoding, CPU load, server on/off.
