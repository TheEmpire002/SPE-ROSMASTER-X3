# SPE-ROSMASTER-X3

Helper robot based on a **Yahboom ROSMASTER X3**.

The robot is meant to drive autonomously indoors, observe a person via ESP
sensors (motion, pulse) and fetch help in an emergency. In addition, it can be
controlled manually via a web app and deliver aids using a box mounted on top.

> **Note:** The vital-sign capture is a prototype and **not a medical
> device**. The robot makes no medical decisions — it always alerts a human.

## Subsystems

| Folder    | Runs on          | Purpose                                              |
|-----------|------------------|------------------------------------------------------|
| `robot/`  | Raspberry Pi 5   | Drive, sensors, safety logic, WebSocket server       |
| `webapp/` | Browser          | Remote control, status display, emergency alert      |
| `esp/`    | ESP32            | Person tracking and vital signs                      |
| `docs/`   | —                | Architecture, hardware, decisions                    |
| `scripts/`| Raspberry Pi 5   | Helper and diagnostic scripts                        |

Details: [`docs/architecture.md`](docs/architecture.md)

**Connecting your own user interface or client?** The complete interface
(driving, lidar, camera, homing) is in [`docs/api.md`](docs/api.md).
Notes for AI assistants: [`AGENTS.md`](AGENTS.md).

## Quick start

One command, everywhere — on the laptop as on the robot:

```bash
python main.py
```

The first time, the script sets everything up by itself (virtual environment,
dependencies), then detects whether real hardware is connected, starts the
server and opens the web app in the browser.

- **On the laptop** it finds no hardware and automatically switches to the
  simulation. Drive commands only end up on the console — so you can develop
  the complete app without occupying the robot.
- **On the Raspberry Pi** it detects the driver and the serial port and starts
  up with real hardware. The app is then reachable on the Wi-Fi at
  `http://<IP-of-the-Pi>:8000` (IP: `hostname -I`). If the board is not
  reachable at startup (e.g. a brief USB dropout), it still starts for real,
  only warns, and connects as soon as the board is back. It only goes into
  the simulation without `Rosmaster_Lib` or with `--mock`.

Options:

| Option          | Effect                                          |
|-----------------|-------------------------------------------------|
| `--mock`        | Force simulation, even on the robot             |
| `--real`        | Force hardware, abort without Rosmaster_Lib     |
| `--port 8001`   | Different port                                  |
| `--no-browser`  | Do not open the browser (e.g. over SSH)         |
| `--setup-only`  | Only set up, do not start                       |
| `--debug`       | Verbose log output                              |

### Without the automation

If you want to start the server directly:

```bash
cd robot
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e .
python -m rosmaster_assist.main --mock
```

## Camera

The image appears at the top of the web app without any extra steps. The
software finds the camera by itself: first a CSI camera, then a USB camera,
otherwise a moving test image. Which one it picked is shown in the log at
startup.

A **USB camera** needs OpenCV:

```bash
robot/.venv/bin/pip install opencv-python
```

Directly accessible, even without the app: `http://<IP-of-the-Pi>:8000/camera/stream`

**No image?** Stop the server, then:

```bash
robot/.venv/bin/python scripts/check_camera.py
```

It checks devices, permissions, competing processes, OpenCV and actually
opening the camera — and names the cause at the end.

Settings (resolution, frame rate, quality): `config.py`, camera section.
Details in [`docs/hardware.md`](docs/hardware.md#camera).

## Lidar

The RPLIDAR A1 is read directly — without ROS2. It does two things:

1. **Localization.** When the start point is set, a snapshot is taken once of
   what the room looks like from there. Later the current scan is matched
   against it; the shift needed is the position. This method does not drift
   — unlike dead reckoning.
2. **Obstacles.** During homing, the area in the direction of travel is
   monitored. The robot does not swerve; it stops and reports it.

This needs one package:

```bash
robot/.venv/bin/pip install rplidar-roboticia
```

The app shows the point cloud as a top view. State and hit rate of the
matching: `http://<IP-of-the-Pi>:8000/lidar/status`

**Important:** ROS2 must not run in parallel — `sllidar_ros2` occupies the
same port. See [`docs/hardware.md`](docs/hardware.md).

## Driving home

In the app there are two buttons under "Startpunkt":

- **Startpunkt hier setzen** (set start point here) — the current spot
  becomes the origin
- **Nach Hause** (go home) — the robot drives back there by itself

In between, the app continuously shows distance, position and heading.
During the drive, the same button becomes the cancel button.

**Two limitations you need to know:**

The position comes from two sources: dead reckoning (drifts, but is always
available) and lidar matching (does not drift, but needs a view of the start
area). Measured in a simulation run over 2.3 m: **30 cm error without lidar,
7 cm with**. If the app shows "geschätzt" (estimated) after the distance, the
lidar or the motion measurement is missing — then the worse value applies.

Beyond `MATCH_RADIUS_M` (4 m) the lidar no longer recognizes the start area
and does not correct. Odometry brings the robot back there, and the lidar
then guides it in precisely.

The robot **does not avoid obstacles** — it stops in front of them and
reports it. Avoiding without a map easily ends in a corner. That is also the
reason for the leisurely speed (`HOME_SPEED`) and the time limit.

Homing is cancelled by any drive input, the space bar, the cancel button or a
lost connection.

## Setting the speed

The web app has two sliders:

- **Max-Tempo** (max speed) — the maximum in m/s you allow yourself. Indoors
  for practice e.g. 0.3, outdoors full throttle. The setting is stored in
  the browser.
- **Tempo** (speed) — what percentage of it the keys use.

Sideways driving and turning are throttled in the same ratio, so that the
robot does not still rotate hectically with a tame max speed.

The **hard upper limit** is on the robot in
`robot/src/rosmaster_assist/config.py` (`MAX_SPEED_X` and co.) and cannot be
exceeded from the app. The server sends it along when connecting; the max
speed slider never goes beyond it.

## Does everything still work?

With the server running, in a second terminal:

```bash
pip install websockets
python scripts/smoke_test.py          # --port 8099 if needed
```

It checks HTTP, WebSocket, the speed limit and the watchdog from the outside
— exactly the path the app takes. Run it briefly after changes to the
server.

### On the robot

```bash
ssh pi@<IP-of-the-Pi>
cd ~/SPE-ROSMASTER-X3 && git pull

# Terminal 1 — server with real hardware
cd robot && source .venv/bin/activate
python -m rosmaster_assist.main

# Terminal 2 — test
cd ~/SPE-ROSMASTER-X3
robot/.venv/bin/python scripts/smoke_test.py
```

The driving tests are **skipped** here: the script recognizes from the
reported drive that real hardware is attached and does not move the robot on
its own. If you need them anyway, **jack up the wheels** and add
`--allow-motion`.

It also works from the laptop: `--host <IP-of-the-Pi>`.

## What is installed?

If it is unclear which drivers are present on the Pi:

```bash
python scripts/check_hardware.py
```

## Project status

- [x] Basic framework, driver abstraction, protocol
- [x] Manual control via web app
- [x] Camera image in the app
- [x] Return to the start point (dead reckoning)
- [x] Lidar: localization and obstacle detection
- [ ] ESP connection (motion, pulse)
- [x] Autonomous driving, test 1: explorer with obstacle naming (without SLAM)
- [x] Interaction, stages 2+3: gestures (Hailo pose) and behaviors follow/stop/come here/go away (see [`docs/interaction.md`](docs/interaction.md))
- [ ] Interaction: speech, voice, "ghost"
- [x] SLAM / map: mapping drive room by room, pose in the map, drive to a point on the map (`/map` page; still to be verified on the robot)
- [ ] Box mechanics
