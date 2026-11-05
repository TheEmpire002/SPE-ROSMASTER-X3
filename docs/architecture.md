# Architecture

## Overview

```
   ┌──────────────┐                    ┌──────────────┐
   │   Web app    │                    │    ESP32     │
   │  (browser)   │                    │ pulse/motion │
   └──────┬───────┘                    └──────┬───────┘
          │ WebSocket (JSON)                  │ HTTP/WebSocket (JSON)
          │                                   │
   ┌──────▼───────────────────────────────────▼───────┐
   │           Raspberry Pi 5  ·  robot/              │
   │                                                  │
   │   api/server.py        input and output          │
   │        │                                         │
   │   safety/watchdog.py   stops on radio loss       │
   │        │                                         │
   │   drive/  sensors/  box/                         │
   └──────┬───────────────────────────────────────────┘
          │ USB serial
   ┌──────▼───────┐
   │ X3 Expansion │  motors · IMU · servos · buzzer
   │    Board     │
   └──────────────┘
```

## Guiding principles

**1. One channel for everything.**
App and ESP speak the same JSON protocol over WebSocket
(`rosmaster_assist/protocol.py`). New features — opening the box, camera
image, emergency alert — are new message types, not new infrastructure.

**2. Hardware is interchangeable.**
Every driver implements an abstract base class (`drive/base.py`). Next to it
is a mock that fulfils the same interface but only writes to the console.
This lets the whole project run on any laptop, and several people can work
in parallel although there is only one robot.

**3. Safety sits at the bottom, not at the top.**
The watchdog sits between server and driver. No matter which error happens
above — crashed app, Wi-Fi gone, browser tab closed — the robot stops on
its own after a short time. This must not be a feature the app has to
remember.

Since 29.10. it also contains a **start-up ramp**: drive commands are
accelerated and braked with limited acceleration (`RAMP_*` in `config.py`)
instead of jumping onto the motors. Jumps between driving and stopping
caused current spikes that reset the USB hub on the X3. Because the ramp
lives in the watchdog, it applies equally to manual control, explorer,
homing and interaction. `stop()` bypasses it and stops immediately.

Since 03.11. the watchdog additionally checks autonomous drive commands
(`source="autonom"`: explorer, mapping drive, homing, following, precise
drive commands) with the **drive guard** (`safety/drive_guard.py`, decision
E10): it predicts where the robot's footprint will go with this command,
including on an arc and while turning, and lowers the speed so that the
stopping distance (reaction time plus age of the oldest scan point plus
braking distance) fits into free space. Driving away remains allowed.
Without a fresh scan no autonomous mode drives. Braking goes through the
ramp, not `stop()`: a hard stop creates exactly the current spikes that
reset the USB hub. The modules still do their own checks; the guard is the
safety net underneath. Settings: `ROBOT_*`, `GUARD_*`, can be switched off
with `AUTONOMOUS_GUARD_ACTIVE`. Until the merge on 03.11. this was done by
an `AutonomSchutz` using a strip in the driving direction, which did not
check rotations.

Since 04.11. the drive guard receives, in addition to the lidar points,
those of the obstacle memory: what the depth camera saw below the lidar
plane, and phantom obstacles from the progress watch (decision E11).

**4. The emergency logic does not decide, it reports.**
If the system detects a possible emergency, it triggers an alert to a
human. It does not assess a health condition.

## Modules in `robot/src/rosmaster_assist/`

| Module         | Responsibility                                             |
|----------------|------------------------------------------------------------|
| `config.py`    | All settings in one place (speed, ports, timeouts)         |
| `settings.py`  | Which of them the settings page may change at runtime; saves changes to `robot/settings.json` |
| `audio/`       | Sound on the USB speaker: `mukke.py` plays YouTube links with mpv |
| `protocol.py`  | Message format — the common language of all parties        |
| `drive/`       | Driving. `base.py` = interface, plus real driver + mock    |
| `safety/`      | Watchdog, emergency stop, speed limiting, drive guard based on the footprint (`drive_guard.py`) |
| `sensors/`     | Lidar, camera, depth camera (`depth.py`, OpenNI2; display `depth_view.py`), person detection, body pose (Hailo) |
| `navigation/`  | Odometry, lidar localization, homing (with a map also goal drive), explorer, mapping drive, avoidance (`avoidance.py`), obstacle naming, obstacle memory for depth points (`obstacle_memory.py`), progress watch against getting stuck (`progress.py`), pose in the map (`map_localization.py`), paths and frontiers (`path_planning.py`), path following (`path_follower.py`) |
| `mapping/`     | Mapping: slam_toolbox in the container including the feed, save/load the map (`occ_map.py`), map in the simulation (`simulation.py`), `/map` page (`web.py`) |
| `interaction/` | Gestures, behavior (follow, come here, go away), voice (speech output) and speech commands with the wake word "Robi"; planned: mind ("Geist") — see [`interaction.md`](interaction.md) |
| `health/` | Healthbot: drive to a person, overview from the camera pose, address by voice, nudge, call for help (`procedure.py`, state machine), the four signs as pure computation (`assessment.py`), a lying person in the mock (`simulation.py`) |
| `test_runs.py` | Guided, reproducible camera/pose/microphone test runs for the `/tests` page; stores each run under `~/.rosmaster/tests/<name>/` and uploads it to the `testdaten` branch |
| `diagnostics.py`  | Collects failures (battery, USB, undervoltage, temperature, devices, log) and turns them into understandable problems for the `/diagnose` page |
| `recording.py` | Recorder: last minute (scans, commands, odometry, states) in memory, saved to a file at the push of a button; plus loading and replaying for `scripts/replay.py` |
| `box/`         | Motor/servo of the transport box *(still empty)*           |
| `api/`         | WebSocket server, also serves the web app; `map_control.py` ties together mapping drive, localization and goal drive for `/map` and the app |
| `main.py`      | Entry point, wires everything together                     |

## Autonomous driving — deliberately later

The base system deliberately contains **no** ROS. For the reasoning see
[`decisions.md`](decisions.md).

When SLAM and navigation come up, there are two options:

- **ROS2 in Docker** on the Pi, connected via the same WebSocket channel.
  The rest of the project stays unchanged.
- **Own navigation in Python** directly on the lidar data. Less powerful,
  but without ROS complexity and easier to understand.

This decision is only made once the lidar delivers data — before that, the
basis for it is missing.

**Status 28.10.:** The second option was chosen. The explorer
(`navigation/explorer.py`) drives directly in the server on the lidar data,
with the same watchdog as manual control. The same goes for the precise
drive commands ("2 Meter vor", "90 Grad drehen", `navigation/maneuver.py`):
they measure distance and angle with the lidar instead of by time. The
older ROS2 project is kept for comparison under `Marlon/autonomous_driving/`.

**Status 03.11.:** The explorer turns by the measured angle (turn rate of
the board via odometry), decides only on a fresh scan after turning, and
unsticks itself instead of stopping: sideways or backwards when points are
too close or it is not making progress (board speed or an unchanged lidar
picture all around). Test without hardware:
`Marlon/autonomous_driving/test_explorer.py` (simulated room).

**Recorder (03.11.):** Before avoidance gets smarter, real drives are
needed for checking. `recording.py` listens in at two places: at the
watchdog (every command before and after the limits, every stop) and in a
loop of its own at the odometry rate (scan, board speed, pose, states). An
error while recording must never disturb driving, so every call is guarded.
`scripts/replay.py` summarizes a file, draws it as SVG and feeds the scans
to a current explorer (`replay_explorer`). In the test with the same random
seed, the replayed explorer issues the same commands tick by tick
(`Marlon/autonomous_driving/test_recording.py`); for real drives only the
agreement rate counts, because the explorer on the robot chooses randomly.

**Avoidance (03.11., F2):** `navigation/avoidance.py` contains the building
blocks; the explorer only puts them together. All of them are pure logic on
lidar points and testable without hardware:

- `ShortTermMemory` keeps points for a few seconds and moves them along
  with the board speed. Without a map and without localization, therefore
  only close points and only briefly (drift).
- `plan_arc` predicts arcs and checks the robot's outline. Heading,
  clearance, speed, range and smooth steering are scored.
- `gaps` returns the centres of free areas (doors).
- `LiftDetector` compares scans in 10° sectors; with the measured pose over
  a window of one second, so that slow carrying is noticed too.

Test: `Marlon/autonomous_driving/test_avoidance.py` (doors, corridor,
lifting, room without false alarm).

**Detour with a goal (03.11., F3):** Follow, come here, go away and homing
have a target direction. `avoidance.choose_direction` looks for the
direction closest to it that has enough room for the driving corridor plus
margin; in narrow passages it takes the middle, and the most recently
chosen side gets a head start (against oscillating). Both use the
`ShortTermMemory` and the `LiftDetector` from F2. When lifted, the behavior
goes to READY and homing aborts. Settings: `DETOUR_*`,
`HOME_DETOUR_WAIT_S`. Test: `Marlon/autonomous_driving/test_detour.py`
(chair and door while following, obstacle and dead end during homing,
lifting).

**Status 03.11.:** For the map, ROS2 is added after all, but only as the map
builder. slam_toolbox runs in the container `x3`. Our server remains the
only reader of lidar and board and feeds scans and odometry in via
`docker exec -i` (`mapping/scan_feed.py`, reasoning: E8). Operation and
feedback via the `/map` page. The heading comes from the lidar
(`navigation/scan_odometry.py`), and the mapping drive
(`navigation/mapping_drive.py`) covers the room as a mode of the explorer:
first around along the wall, then in lanes (E9).

**Status 04.11.: The map is used (E12).** The data path:

```
slam_toolbox ──/map──► ros_bridge ──► MappingManager.mapped_map()
   (or in --mock: mapping/simulation.py)              │
                                                      ▼
                              MapControl.tick_loop (4 Hz, thread)
                                   │  OccupancyMap.from_payload + distance field
                                   ▼
scan odometry ──pose──► MapLocalization ──pose in the map──► Homing (goal/home)
                        (match against          │                MappingDrive (frontiers)
                         the distance field)    │                state.karte, /map
                                                ▼
                                   path_planning (A*, frontiers) ──► path_follower
```

The mapping drive does not stop after the room, but drives to the next
frontier into the unknown until the map is complete. Once the map is
finished (or loaded), `Homing` drives along a planned path to any point of
the map; the `/map` page shows pose, trail, path and frontiers. Test:
`Marlon/autonomous_driving/test_map.py`.

## Interaction

Gestures, speech and a "Geist" (mind) layer sit as a separate level above
the behaviors and also drive only through the watchdog. Structure, gestures
and stage plan are described in [`interaction.md`](interaction.md); the
reasoning is in E5 and E6 in [`decisions.md`](decisions.md).

**Status 28.10.:** Stages 2 and 3 run in the server. `sensors/pose.py`
computes the body pose on the Hailo, `interaction/gestures.py` recognizes
the hand signals, `interaction/behavior.py` drives accordingly. The wiring
in `api/server.py` follows the explorer exactly: built in `create_app`,
cancelled on emergency stop, manual control, homing, connection loss and
drive failure, messages via `_status_loop`.

In addition there is `interaction/voice.py`: speech output on the USB
speaker in its own thread. The server builds it in `create_app` and passes
it to the behavior, which then announces its mode changes. The voice cannot
drive; it gets no watchdog. It blocks neither the event loop nor the drive
loop: `say()` only puts the sentence into a queue.
