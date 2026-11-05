# Notes for AI assistants and new contributors

Helper robot on a Yahboom ROSMASTER X3 (Raspberry Pi 5, mecanum wheels,
RPLIDAR A1, Orbbec camera). Pure Python, no ROS. A FastAPI server on the Pi
is the only way in and out; the web app and the ESP32 talk to it over
WebSocket.

## Read first

- [`docs/api.md`](docs/api.md): **the robot's interface.** Driving, lidar,
  camera, homing, message format, conventions. Anyone who only builds a user
  interface or a client needs nothing else.
- [`docs/architecture.md`](docs/architecture.md): structure and modules
- [`docs/decisions.md`](docs/decisions.md): why things are the way they
  are (please read before larger refactorings)
- [`docs/hardware.md`](docs/hardware.md): devices, ports, pitfalls on the Pi

## Starting

```bash
python main.py --mock     # on your own computer, everything simulated
python main.py            # on the robot
```

The simulation covers the drive, the lidar (an imaginary room) and the camera
(test image). All endpoints behave as they do on the robot.

Tests: `python scripts/run_tests.py` runs all test scripts in parallel and
shows details only for failures (filter: `... run_tests.py target`).
In addition, `python scripts/smoke_test.py` against a running mock server.
Matching subagents for Claude Code are in `.claude/agents/`.

## Rules for changes

- **`protocol.py` is the contract.** Do not rename or reinterpret existing
  messages and field names; add new ones instead. Register new commands in
  `protocol.py`, `api/server.py` (`_handle`) and `docs/api.md`.
- **Drive commands only via `SafetyWatchdog.submit()`**, never directly to
  the driver. Otherwise the speed limit and the dead man's switch do not
  apply.
- **Settings belong in `config.py`**, not as numbers in the code.
- **Conventions:** `vx` forward, `vy` left, `omega` counter-clockwise; lidar
  angle 0° = front, 90° = left. Details in `docs/api.md`.
- **Language:** applies to every agent and contributor, whatever tool writes
  the code (Claude Code reads this file via `CLAUDE.md`).
  - Code (identifiers, comments, docstrings, log messages), tests, docs and
    commit messages are in **English**. Comments explain the *why*.
  - **Code comments** are concise English sentence fragments, no trailing
    periods, all lowercase except proper nouns, absolutely clear. Example:
    `# wheels slip on turns, so the heading comes from the lidar`, not
    `# The heading is computed from the lidar because the wheels slip.`
    This applies to new and changed comments; existing ones are adapted
    when their code is touched.
  - Everything users see stays **German**: the web app, the `/map` page,
    alert texts, the `"text"`/`"grund"`/`"hinweis"` fields of the state,
    diagnostics texts, settings labels; also the robot's spoken sentences and
    the speech commands (German Vosk model).
  - Protocol/API names (WebSocket message types, JSON field names and
    values, HTTP URLs) stay unchanged even when they are German: they are
    the contract with the web app and the ESP.
- **Hardware code must survive dropouts.** USB devices on the X3 occasionally
  disconnect and reconnect (voltage dip). Drivers and sensors reconnect on
  their own; new hardware integrations should do the same and must never
  leave a robot driving.

## On the robot

- Board and lidar are both on USB serial. The lidar has the fixed path
  `/dev/rplidar`, the board `/dev/myserial` (udev rules from Yahboom); if the
  latter is missing, the driver searches `/dev/serial/by-id/`.
- Only **one** program may use the board or the lidar at a time. Stop ROS2
  nodes (`sllidar_node`, `yahboomcar_bringup`) and old server instances
  first: `pkill -f rosmaster_assist; sudo pkill -f sllidar_node`.
- Diagnostics without digging through logs: the `/diagnose` page (everything
  at a glance), individually `/lidar/status`, `/camera/status`,
  `/tiefe/status`, `/stimme/status` (speaker) and `/sprache/status`
  (microphone).
- Before tests, run `python scripts/check_hardware.py`: it warns about new
  USB over-current messages and when the Pi is currently installing or
  compiling. Do not drive while `apt`/`dpkg` are running (heat → USB
  dropouts).

## Documenting

Every work step also goes into the project log
[`Marlon/PROJECT_LOG.md`](Marlon/PROJECT_LOG.md): what, why, result,
problems. It is the basis for the final report and the presentation.
Technical details still belong in `docs/`.
