# Project log – helper robot on the ROSMASTER X3

This file is our **living final documentation**. Every work step is recorded
here: what we did, why, what came out of it and what went wrong. At the end of
the project it is the basis for the final report and the presentation.

> **Rule:** Append new entries at the bottom of the project diary (newest
> last), with a date. Technical details for developers are in `docs/` – this
> file holds the story behind them and what we learned from it.

---

## 1. Goal and starting point

**Goal:** A helper robot that moves around an apartment, perceives people and
interacts with them. It is controlled via an app, drives back to its start
point on its own, explores autonomously and is meant to react to gestures and
speech in the future.

**Platform:** Yahboom ROSMASTER X3 with these components:
- mecanum wheels,
- Raspberry Pi 5 (16 GB) with Hailo-10H AI accelerator (AI HAT+ 2),
- RPLIDAR A1,
- Orbbec depth camera and Logitech C270,
- expansion board (STM32) for motors, IMU and lights.

**Team and focus areas:**

| Who | Focus |
|---|---|
| Ben (Tr3nben) | FastAPI server, web app, driver, lidar localization, homing, API docs |
| Marlon Wehr | Autonomous driving, sensor check and hardware analysis, interaction/AI |
| Magnus Luciano Schell | Repo maintenance, ESP32 CSI (branch `ESP32-CSI`) |

**Fundamental decision:** We do not use ROS in the main system, but pure
Python with a FastAPI server as the only way in and out (see
`docs/decisions.md`, E1/E2). ROS2 Foxy is still in Docker on the Pi, but only
serves as a comparison.

---

## 2. Project diary

### 27.10.2026 – Basic framework (Ben)
- **What:**
  - FastAPI server with WebSocket protocol, web app with drive controls and camera image.
  - Watchdog (dead man's switch after 0.5 s), speed limit.
  - Lidar localization via scan matching, homing to the start point, collision avoidance when driving by hand.
- **Why:** one channel for everything, and everyone can develop without the robot (`--mock`).
- **Result:** The robot can be controlled from a browser at `http://10.42.0.62:8000`.
- **Problems:**
  - At first the Orbbec camera delivered only a single image and had to be forced to MJPG.
  - USB devices disconnected every now and then. Ben therefore built in automatic reconnecting.

### 27.10.2026 – First autonomy attempt with ROS2 (Marlon)
- **What:** a lidar explorer (`Marlon/autonomous_driving/x3_explorer.py`) in the
  ROS2 Docker container, started with one command via `x3.sh`.
- **Result:** The robot drove, but collided.
- **Cause:** The lidar was mirrored left/right and rotated by −90°
  (fix in commit 59013a5: `--no-flip --front-offset -90`).
- **Problems:**
  - The driver node hung after USB resets without reporting an error.
  - The start order lidar → driver mattered.
- **Lesson:** The mounting orientation of every sensor is checked first with a simple hand test.

### 28.10.2026 – Sensor check and hardware analysis (Marlon)
All results are in [`sensorcheck_2026-10-28/SUMMARY.md`](sensorcheck_2026-10-28/SUMMARY.md),
the raw data in `robot-protokolle/2026-10-28_sensorcheck/`.
- **USB resets – cause found:**
  - They did not happen because of a flat battery, but on **CPU load spikes at 2.4 GHz**.
  - The over-current message came on all ports at the same time. Presumably the hub feeds back through the expansion board (backfeed).
  - At 2.0 GHz the resets were gone in the measurement. In the later test, however, a few lidar resets did occur, see below.
- **Lidar:** front = raw angle −90°. The real scan rate is 7.5–7.8 Hz.
- **Gyro z:** large bias of +0.389 rad/s and wrong sign. Only usable with correction.
- **Odometry:** It is accurate. The drive command alone overestimates the motion by 10–30 %.
- **Battery:** 10.1–10.3 V – charge soon.

### 28.10.2026 – Autonomous driving test 1 in Ben's server (Marlon)
- **What:**
  - The explorer now runs directly in the FastAPI server instead of in ROS2: `navigation/explorer.py`.
  - It drives randomly, picks free directions and names obstacles by their lidar shape (wall, chair, table, person): `navigation/obstacles.py`.
  - It detects people via the camera (OpenCV HOG): `sensors/people.py`.
  - The web app has its own panel for it; the commands are `explore` and `explore_stop`.
- **Why:** one system instead of two. The same safety (watchdog, emergency stop) applies to all ways of driving.
- **Test without the robot:** `python Marlon/autonomous_driving/test_obstacles.py` simulates a room. All cases pass.
- **Result:**
  - PR #3 is merged into main.
  - On the real robot, lidar, camera, person detection and autonomy status work.
  - The lidar orientation matches the camera image.
- **Problems:**
  - At the first real start the robot was standing **on the table**. We aborted, because the lidar cannot see edges or drops. From now on autonomous driving runs **only on the floor**.
  - Even at 2.0 GHz there were two lidar resets (11:12 and 11:13). Ben's code reconnects on its own. The real solution, however, remains in hardware: a hub without backfeed.

### 28.10.2026 – Concept "interactive robot" (Marlon)
- **Inventory on the Pi:**
  - The AI chip is a **Hailo-10H** (40 TOPS, 8 GB of its own memory). It can run image models and small language models.
  - A pose model ships with it (`yolov8s_pose_h10.hef`, 17 body keypoints).
  - The Logitech C270 is mounted at the front and tilted slightly upwards.
  - For listening and speaking, a USB microphone and a USB speaker have been mounted since 28.10.
- **Decisions:**
  - Gestures for version 1: follow/stop and come here/go away.
  - They are detected from the body pose on the Hailo instead of with a hand model. That also works at 1–4 m distance and leaves the CPU free.
  - Speech: it listens (Vosk, offline) and answers with a voice (Piper, offline) through the speaker, and additionally as text in the app.
  - "Thinking" runs **hybrid**: Claude API when there is internet, otherwise locally (later the LLM on the Hailo), as a last resort template sentences.
  - **The language model never controls directly.** Driving always goes through behavior and watchdog.
- **"Consciousness":**
  - A mind layer with inner state: energy, curiosity, mood, attention.
  - Plus a memory of recent events and a visible stream of thoughts in the app.
  - Plus expressive movements and the voice.
- **Plan in stages:**
  1. Docs
  2. Perception
  3. Behavior
  4. Speech
  5. Mind and thinking
  6. Local LLM, faces
- **Developer docs:** [`docs/interaction.md`](../docs/interaction.md)

### 28.10.2026 – Rebuild: microphone and speaker, docs created (Marlon)
- **What:**
  - A USB microphone (C-Media) and a USB speaker ("UACDemoV1.0") are mounted. The robot therefore gets a voice already in version 1 (Piper, offline).
  - The C270 was unplugged during the rebuild and goes back to its place at the front.
  - This project log, `docs/interaction.md` and decisions E5/E6 are created.
- **Findings:**
  - After the reboot the Pi ran at 2.4 GHz again, because the clock limit only applied until the next reboot.
  - Shortly after booting, `dmesg` already showed 61 over-current messages on all buses.
  - The USB power supply thus remains the biggest hardware risk.

### 28.10.2026 – Clock permanently at 2.0 GHz, overheating (Marlon, agreed with Ben)
- **What:**
  - `/boot/firmware/config.txt` now contains `arm_freq=2000`, with a comment and a reference to `docs/hardware.md`.
  - The backup is called `config.txt.vor-arm_freq-2026-10-28`. To undo: delete the line.
- **Why:** The limit set at runtime was lost on every reboot (see previous entry).
- **Code changes:**
  - Ben's server needs no change, because its loops are timed by the clock and not by computing time.
  - `scripts/check_hardware.py` now shows CPU clock, temperature and throttling. It warns when the limit is missing, e.g. after a new image.
- **Findings:**
  - Another session had started `sudo dpkg --configure -a`. That built an nvidia driver module (DKMS), which is pointless on the Pi.
  - As a result the Pi ran at 84 °C and throttled to 1.5 GHz (`throttled=0xe0006`). Board and lidar kept dropping off USB.
  - Lesson: do not drive and do not test while the Pi is compiling. The USB resets also depend on temperature, not only on the clock.

### 28.10.2026 – Interaction stages 2 and 3: hand signals and behavior (Marlon)
- **What:**
  - `sensors/pose.py`: body pose with YOLOv8 pose on the Hailo-10H, 10 frames/s, 17 body keypoints per person. The Hailo is only active while the interaction is running and is reopened after errors. In the mock, a simulated person acts out the gestures.
  - `interaction/gestures.py`: detects follow (arm above the head), stop (both hands above the shoulders), come here (waving) and go away (sideways swipe).
  - `interaction/behavior.py`: follow at 1.0 m, come here up to 0.7 m, go away 1 m backwards, stop. Direction from the camera, distance from the lidar.
  - Server and web app: commands `interaction_start/stop`, `follow/follow_stop`, field `state.interaktion`, endpoints `/interaktion/status` and `/interaktion/pose`. New panel "Interaktion"; the skeleton is drawn over the camera image.
- **Why this way:**
  - Gestures from the body pose instead of a hand model (E5): the Hailo computes them, the CPU stays free, and arms are visible even from 2–3 m away.
  - The wiring is a copy of the explorer pattern. Only one program steers at a time, and any manual input cancels.
- **Problem and solution:**
  - In the first test, waving also triggered "go away". Cause: all measures are computed in shoulder widths. If the person briefly turns sideways, the shoulder width becomes tiny in one frame, and a small hand movement looks huge.
  - Solution: take the median of the shoulder width over the time window.
  - The "arm held horizontally forward" as stop was dropped. In 2D it looks almost like a hanging arm.
- **Result (local, mock):**
  - `test_gestures.py` and `test_obstacles.py`: all passed. Smoke test: 19 of 19.
  - Sequence in the mock: come here → "bin da", go away → "Abstand genommen", follow, stop. Cancelling via manual control and via "Autonom fahren" works.
- **Open:** check on the Pi whether the Hailo delivers the pose (`/interaktion/status`, fps). Then the driving test **on the floor**.

### 28.10.2026 – Speech output through the USB speaker (Marlon)
- **What:**
  - `interaction/voice.py`: The robot speaks through the USB speaker (ALSA "UACDemoV10"). Chain: Piper (offline, voice `de_DE-thorsten-medium`) → `aplay`. If Piper is missing, espeak-ng speaks, otherwise it only logs.
  - The behavior announces what it is doing: „Ich folge dir.“, „Ich komme.“, „Da bin ich.“, „Wo bist du?“, „Da bist du ja.“, „Okay, ich halte an.“. On an emergency stop it says „Halt.“, on server start „Ich bin bereit.“ (audible self-test).
  - Server: new commands `say {text, vorrang}` and `say_stop`, field `state.stimme`, endpoint `/stimme/status`. Settings `VOICE_*` in `config.py`.
  - Web app: display "Stimme" (what it is saying right now) and a field "Sagen" in the interaction area.
  - Docs: `docs/api.md`, `interaction.md`, `hardware.md` (setting up Piper), `architecture.md`, new decision E7.
- **Why this way:**
  - Offline, so that announcements and „Halt.“ work without internet too, like the reflexes (E6).
  - A separate thread with a queue: a sentence takes 1–2 s, and the server and the drive loop must not wait that long.
  - Piper and aplay run as separate processes. If one crashes or the speaker is missing, the server keeps running, and a stop can cut off the current sentence immediately.
  - The speaker is addressed by its name instead of the card number and thus survives USB resets.
  - Identical sentences within 4 s are swallowed, the queue holds at most 3. Otherwise it would keep saying „Wo bist du? Da bist du ja.“ non-stop while following.
- **Problem and solution:**
  - Anyone typing in the new text field would have steered the robot: the web app drives with the arrow/WASD keys, and the space bar is the emergency stop. Solution: keyboard control ignores keys while a text field has focus.
  - Without a speaker the voice reported `spricht = true` although nothing could be heard. Later that would have muted speech recognition for no reason. Now `spricht` is only true when sound actually comes out.
- **Result (local, without speaker):**
  - `Marlon/interaction/test_voice.py`: 17 checks passed (queue, priority, fallback to text, sentences on gestures). `test_gestures.py` still passes.
  - Smoke test of the server in the mock: „Ich bin bereit.“ at start, `say`, greeting on "Interaktion an", announcements for the gestures, „Halt.“ on stop.
- **Open:** install Piper and the model on the Pi (instructions in `docs/hardware.md`). Then check whether the speaker can be heard, how long the delay until sound is and whether the volume is right.

### 28.10.2026 – Full check on the Pi, camera fixed to the C270 (Marlon)
- **What:** Checked everything on the Pi, with `robot-protokolle/` as reference. Plus a test of the image chain camera → Hailo pose without server and without drive.
- **Findings from the previous start:**
  - The C270 was missing, only the Orbbec was there.
  - 299 over-current messages in 9 minutes. Board, lidar and hub kept dropping off, and the server got no image.
  - After rebooting with the C270 plugged back in, this was gone.
- **Findings now:**
  - Clock 2.0 GHz active, not throttled, ~51 °C.
  - All devices present: C270 (`video0`/`video1`), Orbbec RGB (`video2`), board (CH340), lidar (CP2102, `/dev/rplidar`), microphone FongLun, speaker UACDemoV10.
  - Over-current only during boot, plus once at s 241 on all ports at the same time. That fits the hub's backfeed.
  - Afterwards `ttyUSB0`/`ttyUSB1` swapped places. The driver still finds the board, because it searches via `/dev/serial/by-id` and excludes the lidar via `/dev/rplidar`.
  - espeak-ng is installed, Piper is still missing. So for now the voice speaks with the fallback voice.
- **Result of the pose test:**
  - The C270 was selected automatically (V4L2, MJPG 640×480).
  - Hailo ready, stable **10 fps** (393 frames in 40 s), 1–2 people detected.
  - **Not a single new over-current message under Hailo load.**
- **Problem and solution:**
  - Until now the camera was selected only by index (`CAMERA_INDEX = 0`). If the Orbbec registered first after a reset, it would be `video0`, and gestures would be searched for in the wrong camera.
  - New: `CAMERA_NAME = "C270"` in `config.py`. The camera with this name is tried first. If it is missing, the old order applies.

### 28.10.2026 – "Everything crashes": cause NVIDIA build, packages removed (Marlon)
- **Occasion:** Camera, motor and lidar kept failing for the team.
- **Findings (uptime ~21 min):**
  - `lsusb` showed only the root hubs. Since s 1234 there were constant over-current messages, ~4900 counts at the end.
  - 83 °C, throttling (`throttled=0xe0008`), load ~6.7.
  - The cause was again `sudo dpkg --configure -a` from a laptop session. An `apt install mpg123` had failed at 13:32 because dpkg still wanted to configure the nvidia package, and afterwards `dkms` built the nvidia module with `make -j4`. That happens again with every `apt` call.
  - **The loop sustained itself:** every re-registration of the devices (Orbbec, lidar motor) triggered the next overload. The hub ports reported `cannot reset (err -71)`. This time the C270 dropped off too, although it is plugged directly into the Pi.
- **What:**
  - Stopped the build. Removed `nvidia-kernel-dkms` with `dpkg --purge` so that it does not start again. Then `dpkg --configure -a` and deleted all 44 nvidia/cuda packages (`apt-get purge`). Checked beforehand with `apt-get -s`: no Hailo package affected, `hailo1x_pci` stays in DKMS, `/dev/hailo0` is there.
  - Installed `mpg123`, `dpkg --audit` is clean.
  - `scripts/check_hardware.py` now has the section "USB power". It shows whether new over-current messages are coming in right now and whether the Pi is installing or compiling in the background.
- **Result:**
  - About 2 min after the build ended, the loop stopped on its own; no reboot was needed.
  - All devices are back (hub, board, lidar, Orbbec, C270, microphone, speaker), the symlinks are correct.
  - Afterwards 0 new over-current messages, 61 °C.
- **New insight:** On the Pi 5, all four USB sockets hang on **one** shared power switch (1.6 A in total). That "all ports at the same time" report therefore does not prove backfeed. It only shows that the sum of all devices exceeded the limit. Hardware recommendations in `docs/hardware.md`, section "USB resets".

### 28.10.2026 – Stress test without Orbbec, with a full battery (Marlon)
- **Occasion:** Check whether the first hardware measures (battery full at 12.0 V, Orbbec unplugged) are enough.
- **What:** Planned a 15-min test: lidar, voice and driving alternating (announcement, 3 s each turning left and right with omega ±0.6, 2 s each forward and back at 0.1 m/s, 4 s pause). Every 10 s the over-current counter, USB disconnects (`journalctl`), temperature and load were recorded. The C270 was not plugged in, **so Hailo pose and gestures were not tested**.
- **Result:**
  - At rest (only lidar, voice, board) **no** new message for minutes.
  - While driving, **+160 over-current messages and 83 USB disconnects** in 4.5 min, with increasing frequency. The speaker dropped out at times. Test aborted after 4.5 min so that no endless loop develops.
  - **Every reset began 0–5 s after the start of a turn**, i.e. when all four motors start up at the same time. Driving straight at 0.1 m/s did not trigger any.
  - 55–62 °C, load ≤ 1.5, no throttling. So heat and computing load are not the cause this time.
- **Conclusion:** The inrush current of the motors pulls down the 5 V supply that the Pi and the hub also run on. This cannot be fixed with software alone. It needs the separate 5 V supply from `docs/hardware.md` ("What to do on the robot", point 4).
- **Software bugs found along the way, fixed:**
  - **Drive commands could go to the lidar.** After a dropout the lidar re-registered one second before the board. Ben's driver then took "the only device" and connected to the lidar. Now the driver picks `/dev/myserial` first (udev rule via vendor and product ID) and always excludes the lidar. Checked on the Pi: every reconnect went through `/dev/myserial`.
  - **Ctrl+C did not stop the server.** uvicorn waited forever for the camera stream to end, which never ends. Because of that the cleanup (motors off) did not run either. Now open connections are cut after 3 s (`SHUTDOWN_TIMEOUT_S`). Checked on the Pi with an open stream: stopped after 4 s, cleanup complete.
- **Afterwards: endless loop even when standing still.** After restarting the server (old version) the counter rose from 5129 to 5752 in 4 min without the robot driving. With the server stopped, the counter stopped immediately. Cause: after every successful reconnect the lidar motor starts up, the inrush current triggers the next dip, and the wait time started at 1 s again every time. So the server restarted the motor every few seconds.
  - **Fixed:** If the lidar fails again within 60 s after reconnecting (`LIDAR_STABLE_S`), the wait time keeps growing (1 → 2 → 4 … 30 s) instead of starting over. Checked locally with a fake lidar.
  - The lidar port on the hub (1-1.4) then stopped registering at all. Then the only help is replugging the lidar cable or switching the robot off and on.

### 29.10.2026 – Cross-check: lab power supply instead of battery (Marlon)
- **Occasion:** Check whether the resets while turning are caused by the battery. For this the robot is connected to a lab power supply instead of the battery. The Orbbec was plugged in again, the C270 was not.
- **Setup:** Robot **on a pedestal** so that it does not roll away. The wheels spin freely in the air, so the motors run without load.
- **What:** Started the new version from the laptop on the Pi (with the lidar fix `LIDAR_STABLE_S`), first watched 2 min at rest, then the same stress test as on 28.10. for 5 min (210 s driving, turns with omega ±0.6, forward/back at 0.1 m/s).
- **Result:**
  - At rest: no message. The 25 over-current messages since the start all came from switching on.
  - Driving: **0 over-current messages, 0 USB disconnects** (battery on 28.10.: +160 and 83 in 4.5 min). Lidar without reconnecting (2333 revolutions), 51 °C, no throttling. It really did drive: the odometry moved, and the lidar localization corrected 1277 times.
  - When plugging and unplugging the C270 and the speaker during operation, all USB devices dropped off for 3 s shortly afterwards (+18 messages). The server survives that. *Clarified later:* the event coincided with stopping the server, and the cause was a software bug (Hailo start during shutdown, see next entry), not the replugging.
- **Conclusion (at first):** The cause was supposedly the supply from the battery; the power supply held the voltage.
- **Corrected by the battery cross-check (next entry):** On the pedestal there are 0 resets with the battery as well. So the difference to 28.10. is not battery versus power supply, but mainly the **load on the wheels**. Orbbec and computing load are still ruled out.

### 29.10.2026 – Cross-check: full battery on the pedestal, shutdown bug fixed (Marlon)
- **Occasion:** The power supply test ran on the pedestal. So it was open whether the power supply helps or just the missing load. Therefore the same test with the battery in the same setup.
- **Setup:**
  - Robot **on the pedestal**, wheels free in the air (as in the power supply test).
  - **Battery charged to 100 %**, measured via the board (`get_battery_voltage`): 12.3 V before the test, 12.2–12.3 V afterwards.
  - Orbbec **and** C270 plugged in, speaker plugged in. The server fetched images from the C270 the whole time.
  - Version from the laptop (with `LIDAR_STABLE_S`), Hailo pose off because the interaction was not running.
- **What:** Watched 2 min at rest, then the same stress test as on 28.10. and with the power supply: 5 min, of which 210 s driving (announcement, 3 s each turning with omega ±0.6, 2 s each forward and back at 0.1 m/s, 4 s pause).
- **Result:**
  - At rest: no new message.
  - Driving: **0 over-current messages, 0 USB disconnects** (counter stayed at 19 from switching on).
  - Lidar 3335 revolutions without reconnecting, camera 6566 frames without reopening, 50–53 °C, no throttling (`throttled=0x0`).
- **Comparison:**

  | Date | Setup | Supply | Voltage | Over-current / disconnects while driving |
  |---|---|---|---|---|
  | 28.10. | no pedestal, wheels under load | battery | 12.0 V | +160 / 83 in 4.5 min |
  | 29.10. | pedestal, wheels free | lab power supply | – | 0 / 0 in 210 s |
  | 29.10. | pedestal, wheels free | battery | 12.3 V | 0 / 0 in 210 s |

- **Conclusion:** Without load on the wheels, the battery is enough too. The resets occur when the motors start up against load: then the inrush current is much higher than with freely spinning wheels. Whether the power supply absorbs this under load will only be shown by a **test on the floor**. First on the power supply, then on the battery. Other differences to 28.10.: the lidar fix `LIDAR_STABLE_S` was not yet in place then, and the battery had 12.0 instead of 12.3 V.
- **Shutdown bug found and fixed:**
  - **Observation:** When stopping the server, all USB devices dropped off twice for about 2 s (most recently +12 messages at 08:15:06). The event from the power supply test, which we had blamed on the replugging, also happened during shutdown.
  - **Cause:** `PoseDetection.stop()` wakes up the pose thread so that it exits. But the thread did not check afterwards whether it should stop, and first opened the Hailo. The HailoRT log says "Creating vdevice" at 08:15:06.890, in the same second as the overload. Starting up the Hailo briefly draws a lot of current.
  - **Fixed** in `sensors/pose.py`: after waking up, the thread exits if `stop()` woke it. Started and stopped twice on the Pi: counter 31 → 31, no more Hailo start during shutdown.
### 29.10.2026 – Diagnostics page: all failures in one place (Marlon)
- **What:**
  - New page `/diagnose` (also `/logs`), reachable via the button "Diagnose" at the top of the main page; "Steuerung" leads back from there. A dot on the button already shows on the main page when something is wrong (yellow = warning, blinking red = critical).
  - `diagnostics.py` collects in the background: battery voltage (new: `read_battery()` in the driver) with history and voltage drops, the Pi's USB overload counter, undervoltage and throttling (`vcgencmd get_throttled`), temperature and clock, USB disconnects from the kernel log, state of drive, lidar, cameras and Hailo, errors from the server log, hanging build processes (`dpkg`, `cc1` …).
  - From this it derives **problems in plain language**: what happened, what it means, "Was tun?" as steps. Problems can be marked as seen; if one occurs again afterwards, it reappears.
  - Plus a timeline (what happened when – to spot connections like "voltage drop while driving → USB disconnect"), the server log with filter and search, and "Bericht kopieren" (everything as text for a chat or a report).
  - The warning thresholds (battery, voltage drop, temperature) are on the settings page in the block "Diagnose & Warnschwellen" and take effect immediately.
- **Why:** With every dropout we had to look into `dmesg`, `vcgencmd` and the server output via SSH and piece the times together ourselves (see battery and power supply tests). Now everyone on the team sees at a glance what is going on and what to do.
- **Problems and solutions:**
  - Right after start the page reported lidar and camera as failed because they were still starting up. Solution: 15 s startup time (`DIAG_STARTUP_S`).
  - After a server restart the browser would no longer have fetched new log lines (line numbers start at 1 again). Solution: the server always reports its last number; if it is smaller than the known one, the browser reloads.
- **Result (local, mock):** `Marlon/diagnostics/test_diagnostics.py` 9 of 9 passed (battery, acknowledging, incidents, kernel log, undervoltage, log buffer); smoke test 19 of 19; all pages respond.
- **Merging with main:** In parallel Ben had built in the music feature "MUKKE", in the same places (server endpoints, `config.py`, button at the top left, CSS, `docs/api.md`). The conflicts were only "both appended something"; both parts are kept, the buttons Diagnose and MUKKE sit side by side. Afterwards again: tests 9/9, smoke test 19/19, `/diagnose` and `/mukke/status` respond.
- **Open:** check on the Pi whether the kernel log, the overload counter and `vcgencmd` are read. For the kernel log, `pi` must be in the group `systemd-journal` (`sudo usermod -aG systemd-journal pi`, then log in again).

### 29.10.2026 – Smooth driving and debounced gestures (Marlon)
- **What:**
  - **Acceleration ramp in the watchdog:** drive commands no longer jump onto the motors but are ramped up with limited acceleration (0.8 m/s², turning 3 rad/s²) and braked faster (2 m/s², 6 rad/s²). A change of direction first brakes to zero. `stop()` (emergency stop, stop gesture, dead man's switch) stays immediate. Settings `RAMP_*` in `config.py`, can be switched off with `RAMP_ACTIVE = False`.
  - **Calmer following:** camera angle smoothed, dead zone of 3°, smooth transition between turning and driving instead of hard switching, smaller turning gain (`FOLLOW_KP_W` 1.2). If the person is briefly gone, the last command coasts out over 0.5 s instead of jumping to zero (with a check whether the front or back is free). If the lidar distance is missing in one tick, the last good value applies briefly. The state "person very close" has a hysteresis.
  - **Debounced gestures:** come here and go away only count when the person is standing still (shoulders barely move, size barely changes), facing the camera, and the robot itself is standing; plus confirmed twice in a row. Stop and follow are exempt.
- **Why:**
  - When steering by gesture and while following, the USB hub failed and the motors ran jerkily. Both have the same cause: hard jumps in the drive command (person briefly not detected → immediately 0, back again → immediately full speed; jittering angle → `omega` jumps back and forth). Every step draws a current spike from the battery, the voltage drops, the hub resets (see sensor check and diagnostics page).
  - While following, the robot sometimes switched behavior without anyone giving a signal: people walking swing their arms, and that looked like come here/go away. When the robot drives, the whole image also shakes.
- **Result (local, mock):** `Marlon/interaction/test_gestures.py` 69 checks passed (new: coasting, coasting in front of an obstacle, smooth transition, dead zone, smoothing, hysteresis, walking with swinging arms, walking past, from behind, moving robot, ramp in the watchdog); `test_diagnostics.py` 9/9; smoke test 19/19.
- **Problems and solutions:**
  - The gesture test already crashed before the change (`'Pose' object has no attribute 'fov_deg'`): since the camera switch, the behavior asks the pose detection for the field of view, and the test dummy did not know it yet. Dummy extended; the real code was fine.
  - The smoke test (Ben's `scripts/smoke_test.py`) checked the speed limit by waiting for `vx = 5.0` in the status. With the ramp that takes a good 6 s. Adjusted: in the simulation it waits up to 10 s and drives in a circle meanwhile (straight ahead would add up to 15 m and the robot would end up outside the imaginary room); on the real robot it only checks for 1 s that the limit is never exceeded, instead of full speed across the room.
  - "homing starts" in the smoke test occasionally fails when the same mock server is tested several times in a row: the simulated robot drives through the imaginary walls, the localization then ends up near the start point, and homing is finished before the next status shows it. For comparison without the ramp: 3 of 5 runs failed, with the ramp 1 of 4. So this existed before; with a freshly started server the test passes.
- **Open:** test on the Pi on the floor whether following runs more calmly and the USB resets while following stay away (keep `dmesg -w` running, watch `/diagnose`). If acceleration feels too sluggish: increase `RAMP_ACCEL_XY`.

### 29.10.2026 – Voice commands "Robi" and drive-loss abort on gesture start fixed (Marlon)
- **What:**
  - **Versions merged:** the not yet pushed commits from the branch `interaktion` (gestures, voice, diagnostics, smooth driving) went into the new branch `sprache`, which is based on the current `main`.
  - **Hailo stays warm:** the pose detection opens the Hailo already at server start and keeps it open (`POSE_KEEP_WARM = True`). It still only computes while the interaction is on.
  - **Drive gone ≠ interaction off:** if the board drops out, the behavior stops and goes to `bereit`; the interaction stays on. Drive modes (follow, come here, go away, search) are refused as long as `driver.connected` is false; it then says „Ich habe keine Verbindung zu meinen Rädern.“. After reconnecting, it continues with the next gesture.
  - **Voice commands with the wake word "Robi"** (`interaction/speech.py`): USB microphone → Vosk (offline, small German model) with a fixed grammar. Commands: follow („Robi, folgen“, „folge mir“, „komm mit“), come here, go away/back, drive home, explore, music off/quiet, „Hallo Robi“ (interaction on), „Robi, gute Nacht“ (everything off). „Stopp“, „Halt“, „Bleib stehen“ also work without the wake word. They are executed through the same paths as the app commands. New: command `speech {text}` (text instead of microphone, also in the mock), `state.sprache`, `GET /sprache/status`, field "Hören" in the app. Dependencies as the optional extra `pip install -e "robot[sprache]"`.
- **Why:**
  - When switching on gesture control, "Keine Verbindung zum Antrieb" often came up and the interaction was immediately off again. Cause 1: the Hailo was only started up when switching on; that briefly draws a lot of current, and the board disconnects at the hub (same pattern as when stopping the server). Cause 2: on every drive loss, the server ended the whole interaction mode instead of just stopping.
  - Voice commands are the second input besides gestures (stage 4 in the concept). The wake word prevents conversations in the room from making the robot drive off; stop does not need it, so that stopping always works.
- **Result (local, mock):** `test_speech.py` 41 checks (wake word, wake window, precedence, grammar against the model's vocabulary, no crash without Vosk/microphone); `test_gestures.py` 76 (new: drive modes refused without drive, `drive_lost` leaves the interaction on); `test_voice.py` 17; `test_diagnostics.py` 9/9. `/sprache/status` and `speech` checked in the mock.
- **Problems and solutions:**
  - „Musik stopp“ was recognized as stop: the commands now have a precedence, music off comes before stop.
  - Words the Vosk model does not know would make the grammar invalid. They are filtered against `graph/words.txt`; for "Robi" there are variants („robbie“, „ruby“ …).
  - Music (mpv) and voice (aplay) share the speaker exclusively, and the microphone hears the music. „Robi, Musik aus“ helps; a clean solution (shared mixer) is open.  *Addendum: solved, music pauses while speaking (see below).*
  - Smoke test "homing starts" is still flaky as described on 29.10. (repeated runs against the same mock server); not caused by this change.
- **Addendum – "Sanft fahren" in the settings:** the acceleration ramp can now be switched off in the app under settings → "Fahren & Motoren" ("Sanft fahren") and adjusted (Anfahren, Bremsen, Andrehen, Drehung stoppen). Until now that was only possible in `config.py` with a restart. The watchdog reads the values every tick, so the change takes effect immediately and survives a restart via `robot/settings.json`. Checked: a jump to 0.3 m/s arrives as 0.08 with the ramp, as 0.3 immediately without; values outside the limits are rejected.
- **Addendum – "microphone not detected":** on the Pi, `/sprache/status` showed the error "sounddevice fehlt": `main.py` only installed the required packages, not the speech packages. Now `main.py` sets up the voice commands itself when starting with real hardware (`vosk`, `sounddevice`, model to `~`; without internet the server simply starts without speech). Only `libportaudio2` still needs a one-time `sudo apt`, which `main.py` points out; if it is missing, `/sprache/status` now reports this clearly instead of the listening thread dying silently. Testing with the real model revealed a second bug: the small model has no `graph/words.txt`, so the filter for unknown words never applied (316 Vosk warnings; the test had created the file itself). Now the server asks Vosk directly (`vosk_model_find_word`); as wake words "robbie", "ruby", "robin" remain, all command phrases are in the model. Checked on the laptop: installation, download, unpacking, second start without download (1.6 s), grammar without warnings.
- **Open:** install Vosk and the model on the Pi (`docs/hardware.md`, "Microphone and voice commands"), check `/sprache/status`, test „Robi, folgen“ on the floor. When switching on gestures, keep `dmesg -w` running: does the reset stay away?

---

### 29.10.2026 – `sprache` in `main`, music pauses while speaking, "La da da"
- **What:**
  - Merged branch `sprache` into `main` (no conflicts; MUKKE, diagnostics page and camera roles from `main` stay).
  - **Music and voice share the speaker:** when the robot speaks, MUKKE stops mpv before the first sentence, remembers the position and restarts there after the last sentence (`Mukke.interrupt()`/`resume()`, callbacks `before_speaking`/`after_speech` of the voice). In the app the song keeps running during that time.
  - **Voice command „La da da“** (`ladada`, also „la la la“, without wake word): plays the song from `SPEECH_LADADA_URL` via MUKKE. If it is already playing, nothing happens.
- **Why:** The speaker's ALSA device can only be opened once: when music was playing, the voice failed with "Device or resource busy". Pausing is not enough, mpv keeps the device open even while paused.
- **Problems and solutions:** The microphone hears the music; if the song itself contains „la la la“, it would start over again and again. Therefore „La da da“ is ignored while exactly this song is playing.
- **Addendum – „Robi, Beatbox“:** plays the song from `SPEECH_BEATBOX_URL` (with wake word). The song commands now go through a shared branch in the server. "Beatbox" is probably not in the Vosk model; variants („beat box“, „bit box“ …) are stored, unknown ones are dropped.
- **Open:** test on the Pi: music on, „Robi, hallo“ → music pauses, answer, music continues. „La da da“ without Robi. Whether Vosk reliably hears „la da da“ is shown by `/sprache/status` (`zuletzt_gehoert`).

### 29.10.2026 – Hunting "random crashes" via SSH (Marlon)
- **What:** After replugging to the other board, the robot kept crashing seemingly at random (on the power supply, on the pedestal). Measured on the Pi via SSH and reviewed the code for crash sources.
- **Findings on the Pi (no server running, wheels still):**
  - Supply and system healthy: `throttled=0x0`, 5.05–5.08 V, 47 °C, 14 GB RAM free.
  - **Permanent USB overload:** 49–65 over-current messages per 10 s, from the first second after booting. All 6 root ports count up, the hub ports do not.
  - The lidar does not register at all (`error -32`/`-71`), `/dev/rplidar` is missing.
  - The journal was only kept in RAM; logs from earlier crashes were gone.
- **Result:** The cause is hardware or rather the USB power supply (backfeed via the hub on the board, or a defective lidar/cable), not the code. Every overload briefly disconnects all devices, which looks like a crash in the app. Narrowing it down in four steps (unplug all USB devices, plug them back in one by one with the lidar last, swap the lidar cable, unplug the board USB), see `docs/hardware.md`.
- **Made the code more robust** (weak spots that could look like crashes):
  - Server loops (status, hardware, odometry, map matching) catch errors and keep running instead of ending silently; all background tasks are monitored.
  - A hanging WebSocket client no longer slows down the status for everyone else (parallel sending with timeout `WS_SEND_TIMEOUT_S`).
  - An error in a single app message no longer drops the connection.
  - If the board fails at start, the server starts anyway (the driver reconnects later); the serial port is closed again in that case.
  - `main.py` explains how the server ended (SIGKILL, segfault, SIGHUP …).
- **Crash recording** (`crash.py`, `/diagnose` → "Abstürze & Hänger"): every server run with boot ID, log as a file, segfault dump, event loop heartbeat with stack on hangs, memory history, USB overload per minute and per port. At the next start the page says whether the Pi rebooted, the server was killed hard or crashed. The findings of this investigation are shown permanently on the page.
- **Checked on the Pi (branch `fehlersuche-abstuerze`):** the server starts even without the board, `/diagnose` shows "Abstürze & Hänger" and the findings. Measured live: **247 USB overloads per minute while standing still**, all six root ports keep counting, cameras, lidar and board keep disconnecting. The message `Rosmaster object has no attribute ser` in the log is harmless (the Yahboom library does not clean up properly after a failed open).
- **Open:** the four narrowing-down steps on the robot; make the journal persistent (command in `docs/hardware.md`); start the server in `tmux` so that a closed SSH window does not stop it.
- **Addendum – started in mock mode:** `main.py` checks at start whether the board answers. Because of the USB overload, `/dev/myserial` was often missing at exactly that moment; then the server ran in the simulation („Es bewegt sich nichts“), and `--real` aborted. Now: if `Rosmaster_Lib` is installed (= we are on the robot), it always starts for real and only warns; the driver connects as soon as the board is back. Simulation only on Windows, without the lib or with `--mock`.

### 03.11.2026 – Robot switches the interaction on by itself
- **What:** Speech recognition checks every sentence before it counts as a command (`SpeechRecognition._interpret`). While music is playing, only stop and „Musik aus“ count. If Vosk is unsure about a word (`SPEECH_MIN_CONFIDENCE` = 0.7) or more than half of the sentence was unknown (`SPEECH_MAX_UNK_SHARE`), only stop counts. The same applies to word salad: wake word twice, or words besides the wake word and the phrase (`SPEECH_MAX_EXTRA_WORDS` = 0). `/sprache/status` now shows `konfidenz`, `verworfen` and `zuletzt_verworfen`.
- **Why:** The robot started the interaction without anyone doing anything. Cause: Vosk with a fixed grammar turns *any* noise into an allowed sentence. The microphone hears its own music (MUKKE) and conversations in the room. Until now it was only muted while the voice was speaking. The server log from the morning (analysed via SSH) contains four such commands within two minutes that nobody said: „aufwachen morgen“ → `start`, „robin hallo hause zu weg ruby“ → `weggehen`, „heim ruby zurück“ → `weggehen`, „ruby komm da fahr mir la ruby leise box“ → `herkommen`. Each of them switched the interaction on, two made the robot drive.
- **Result:** Checked on the Pi with the original sentences: all four are rejected; „robbie komm her“, „folge mir robbie“, „robbie fahr nach hause“, „stopp“ and „halt an“ still work.
- **Open:** check on the Pi which confidences real commands get (`/sprache/status`, field `konfidenz`). If real commands are rejected, lower `SPEECH_MIN_CONFIDENCE`. The C270 was used as the microphone (the USB microphone was not plugged in); it picks up from further away and with more noise.

### 03.11.2026 – USB overload investigated via SSH
- **What:** Analysed the kernel log, the supply and the server log on the Pi (uptime 15 min, server running while standing still).
- **Result:**
  - Over 400 "over-current" messages in 15 min, always on **all** root ports at the same time (including the empty `usb4-port1`). The shared 1.6 A switch of the Pi's sockets trips, not a single device.
  - The Pi's own supply is fine: `throttled=0x0`, EXT5V 5.09 V, 51 °C, `arm_freq=2000`, `usb_max_current_enable=1`, `PSU_MAX_CURRENT=5000`. Nothing more can be unlocked in software.
  - Already 24 overloads **before** the server start (during boot all devices register at the same time). The first collapse during the run came in the same second in which the Hailo, the lidar motor, both cameras and the announcement „Ich bin bereit“ started. After that an endless loop (devices register, draw inrush current, the switch trips again).
  - The **Orbbec is plugged in again** (it had been unplugged since 28.10.). It is the biggest consumer on the hub and brings its own internal hub (Genesys `05e3:0608`).
  - **The hub:** VIA VL817 (`2109:2817`/`0817`, firmware 0.50), reports itself as *self powered* (0 mA from the Pi). We were told it is the outdated V1 with power problems. That fits the pattern (overload on all Pi ports at the same time, including empty ones). The version cannot be read out via software.
  - The USB microphone is not plugged in; speech recognition uses the C270's microphone.
  - Pitfall when reading the logs: the Pi has no real-time clock. Until network time kicks in, the log shows old times (today "29.10. 15:04" instead of 07:5x).
- **Conclusion:** As on 28./29.10.: the cause is the shared 5 V supply, only hardware helps (`docs/hardware.md`, "What to do on the robot", point 4: active hub with its own supply, or cut VBUS between Pi and hub). Until then, unplug the Orbbec again. Given the hint about the V1 hub, replacing it with a newer version or a hub with a clean supply of its own is the obvious step. Cross-check first: remove the hub, connect board and lidar directly to the Pi.
- **Addendum – driver board V1:** the hint "V1 with power problems" referred to the driver board (Yahboom ROS Driver Board). According to Yahboom, only V3.0 supports the Pi 5's power protocol (5.1 V/5 A). Confirmed on the Pi: nothing was negotiated (`usbpd_power_data_objects` = 0). The 5 A are only set by hand in the EEPROM (`PSU_MAX_CURRENT=5000`), so the Pi allows 1.6 A for USB, which the V1 board does not deliver under load. Board firmware 2.4, battery 11.9 V. A firmware update does not help: the firmware controls motors and IMU, the supply is hardware. Remedy: board V3.0 or a separate 5 V/5 A converter with the Pi 5 protocol for the Pi.

### 03.11.2026 – "Robi" audible again, more natural voice
- **What:**
  - **"Robi" after a command:** the last command in the test was „Robi, Beatbox“. After that, the rule from the morning only let stop and „Musik aus“ through while music was playing; even the wake word was dropped. So the robot was deaf until the end of the song. Now, while music plays, any command counts if it has the wake word and Vosk is confident (`SPEECH_MIN_CONFIDENCE_MUSIC` = 0.85).
  - **"Robi" alone gets an answer:** after 1 s it says „Ja?“, unless the command has already come. Until now there was no audible feedback; you did not know whether it had listened. The wake window (5 s) only starts counting down once it has finished speaking. A "Robi" in the middle of word salad no longer wakes it.
  - **Voice:** set up Piper with the voice "Thorsten medium" on the Pi (`~/piper/`). Until now Piper was missing and the server spoke with espeak-ng, which is why it sounded so robotic. The server now also finds Piper in the release folder (`VOICE_PIPER`).
- **Result:** Speed measured on the Pi: "medium" about twice as fast as real time, "high" slower than real time (5.7 s for 3.5 s of speech), hence medium. Test scripts `test_speech.py` and `test_voice.py` passed on the Pi.
- **Open:** try on the robot: „Robi“ → „Ja?“ → „folge mir“; during a song „Robi, stopp“ and „Robi, komm her“. Judge the sound of the new voice in the room.
- **Side finding:** the crash recording (`~/.rosmaster/diagnose/server.log`) only exists on the branch `fehlersuche-abstuerze`. On `main` the server therefore wrote no log, only to the terminal. Merged into `main` the same day (conflicts only in the docs, both sides kept).

### 03.11.2026 – Female voice, and why Thorsten did not speak
- **What:** Switched the voice to "Kerstin" (female, `de_DE-kerstin-low`). Piper now computes on only one CPU core (`VOICE_CORES`, via `taskset`), the volume is at 70 instead of 80 %. When started with `sudo`, `main.py` uses the home directory of the actual user.
- **Why:** Thorsten did not speak for two reasons:
  1. **Over-current while speaking:** in the log every longer sentence broke off with `aplay: write error: No such device`. In the same second the kernel log showed an over-current storm; all USB devices dropped off, including the speaker. Piper on four cores plus the speaker at 80 % was a load jump that the weak supply (driver board V1) could not absorb.
  2. **Start with `sudo`:** the server then ran as root, `~` pointed to `/root`, Piper was not found, and it spoke with espeak again.
- **Result:** Measured on the Pi: with only one core Piper is hardly slower (1.27 instead of 1.28 s for 3.5 s of speech). A test sentence with Kerstin at 70 % played completely and without a single new over-current message (7 before, 7 after). `test_voice.py` passed.
- **Open:** watch during operation whether USB failures still occur while speaking (`/diagnose`). The real remedy remains the supply (board V3.0 or a separate 5 V/5 A converter).

### 03.11.2026 – Crash on "folge mir": soft start, one volume control
- **What:**
  - **Soft start of the interaction:** first only the pose detection warms up for 5 s (`INTERACTION_STARTUP_S`, silent, standing still), then it says „Jetzt folge ich dir.“, and it only drives once the sentence has finished. Commands during this time (e.g. stop) apply afterwards.
  - **One volume control:** `VOLUME` on the settings page (group "Ton & Lautstärke") applies to voice and music. The speaker's mixer is always at 100 % (`SOUND_MIXER`) and is set again before every sequence of sentences (after a USB dropout the speaker comes back with its default value). Lowering the volume happens in software: the server scales down Piper audio, espeak gets `-a`, mpv gets music control × overall volume.
- **Why:** At 08:50:25 the log showed everything in the same second: „ruby folge mir“ → interaction on, pose detection on, announcement „Ich folge dir.“, motors on. One second later the USB supply collapsed, shortly after that the whole Pi rebooted (uptime afterwards 5 min). One after the other instead of all at once spreads out the load jumps.
- **Result:** On the Pi: `test_gestures.py` (with new checks for the soft start), `test_speech.py` and `test_voice.py` passed. Real sentence via the `Voice` class with volume 60 % in software: Piper speaks, mixer is at 100 %, no new over-current message (16 before, 16 after).
- **Open:** test on the floor: „Robi, folge mir“ → 5 s silence → „Jetzt folge ich dir.“ → drive off, watching `/diagnose` meanwhile. Adjust the volume in the room.

### 03.11.2026 – Microphone live in the app
- **What:** Two switches in the "Interaktion" area of the web app:
  - **Live transcript ("Mitlesen"):** level bar, what Vosk currently understands in the ongoing sentence, and the last 15 sentences with a verdict (command, rejected with reason, wake word, no command). New: `GET /sprache/live`, the app polls 4x per second.
  - **Live audio ("Mithören"):** the sound of the robot's microphone in the browser. New: WebSocket `/sprache/ton` (raw audio 16 bit mono, playback with the Web Audio API). If playback lags behind, it jumps ahead instead of getting later and later.
- **Why:** With the false triggers, what the robot had heard was only visible in the log. Now you can listen live and see why a sentence did (not) count as a command. This also helps when tuning the confidence thresholds.
- **Cost:** the server only computes level and interim text while someone is reading along (it stops 3 s after the last request), and only sends audio to connected listeners. Switched off, neither costs anything.
- **Result:** Checked on the Pi with a second server in simulation mode: `/sprache/live` delivers, `mikro.js` is served, the audio WebSocket responds (without a microphone with `{"rate": null}`; found and fixed a bug along the way: the first message was missing, so the app would have shown "verbinde …" forever). Typed sentences appear in the feed with a verdict. Test scripts passed.
- **Open:** try with a real microphone after restarting the server (audio, delay).

### 03.11.2026 – Voice commands after a crash: failsafe
- **What:** Speech recognition now also recovers from a USB dropout when no error message comes:
  - The microphone is read via a callback with a queue instead of a blocking `read()`. If no audio comes for 3 s (`SPEECH_HANG_S`), it is reopened. If closing the dead stream hangs, it is left behind.
  - 5 s of nothing but exact zeros (`SPEECH_SILENT_S`) count as a dead stream.
  - If recognition runs on the C270 after a dropout, it switches back to the USB microphone as soon as that has registered again.
- **Why:** After a crash the robot no longer reacted to speech. On the running server, `/sprache/live` showed a frozen level, state "hoert" and no new sentences; `/sprache/ton` delivered no audio. In the kernel log the USB microphone had disconnected and reconnected at 09:12 and 09:13. The listening thread hung in `stream.read()`, without an error. The old failsafe (retry every 3 s) only applies on an error message and therefore never kicked in.
- **Result:** On the Pi: reading via callback with the real microphone works (no pure zeros, so no false triggering of the silence detection). New test in `test_speech.py`: a stream that never delivers is detected after the deadline and closed. All three test scripts passed.
- **Open:** restart the server and, on a real dropout, look for "stream hangs" and "microphone back" in the log.

### 03.11.2026 – Live audio stays on, even after commands and hangs
- **What:**
  - **Server:** a guard over the listening thread. If it makes no progress for 15 s (e.g. even opening the microphone hangs in PortAudio), the guard starts a new one, at most three times in a row. The Vosk model is not reloaded for this. Without audio, the audio WebSocket sends a sign of life every second.
  - **App:** live audio reconnects by itself when nothing arrives for 5 s, even if the connection was not closed cleanly (Pi hangs). The live transcript polls with a 2 s deadline. If the browser pauses the audio, it resumes with the next packet.
- **Why:** After a command you could no longer listen in. The robot answers a command, the speaker draws current, and the USB microphone drops off (eight times in eight seconds in the kernel log). With the old code the listening thread then hung forever.
- **Result:** On the running server after the restart: „Robi, folge mir“ → microphone drops off while speaking → after 3 s "microphone back", recognition keeps listening. New test: a listening thread that hangs forever when opening is replaced three times, after that `/sprache/status` reports „Server neu starten“. All test scripts passed.
- **Open:** The real cause remains the supply (board V3.0 or a separate 5 V/5 A converter). Until then there is a gap of about 3 s in the live audio after every sentence the robot says.

### 03.11.2026 – Slow after following: drive priority; search until found
- **What:**
  - **Drive priority:** while one device steers, the server ignores the zeros of all other connections for 0.5 s (`DRIVE_PRIORITY_S`). "stop" still works from every device.
  - **Search:** if it loses the person while following, it turns in a circle until it sees someone again (`FOLLOW_SEARCH_S` = 0, previously it gave up after 6 s). It turns in steps of about 40° with a short pause in between, because the camera image blurs while turning.
- **Why slow:** In the log the drive commands alternated several times per second: full speed, 0, full speed, 0. At the same time two devices were connected (10.42.0.60 and 10.42.0.206). The web app sends "vx=0" ten times per second even when standing still (dead man's switch). One device steered, the other braked in between, and it drove stuttering at about half speed. It only became noticeable after following, because during the interaction the server discards the zeros anyway. The acceleration ramp was switched off on the settings page, so every zero hit with full force.
- **Result:** On the Pi: test for the drive priority (device A drives, device B sends zeros → only A counts, A lets go → stop). `test_gestures.py` with new checks: search pauses briefly after turning, keeps turning, is still searching after 5 min, follows as soon as someone is there. With a time limit it still gives up. All test scripts passed.
- **Side finding:** The Pi rebooted again at about 09:41 (uptime afterwards 4 min), with many emergency stops in the log before that.
- **Open:** test on the floor: lose the person, it searches in a circle, finds them again. Driving with two open apps.

### 03.11.2026 – Announcement: speaking through the robot
- **What:** New switch "Über den Roboter sprechen" in the app. The microphone of a laptop or phone goes live through the robot's speaker.
  - Browser: microphone with echo and noise suppression, downsampled to 16 kHz, as raw audio over the WebSocket `/sprache/durchsage`.
  - Robot: the announcement sits in the voice's queue. Music pauses, speech recognition stops listening, a stop cancels, the central volume applies. After 2 s of silence the speaker is free again.
  - Additionally HTTPS on port 8443 with a self-generated certificate. Without HTTPS, browsers do not grant access to the microphone; under `http://` the app shows a link there.
- **Why:** The wish to talk to people through the robot without standing next to it.
- **Result:** On the real speaker: played a test tone via the announcement, speech recognition stops listening meanwhile, no new over-current message. With a second server (simulation): HTTP on 8100 and HTTPS on 8443 run at the same time, the certificate was generated, the announcement arrives via `wss://`, Ctrl+C stops cleanly. `test_voice.py` with new checks: the announcement keeps "spricht" set, ends with its end, after a pause and on a stop. Found and fixed: if the speaker was busy, every audio packet would have started a new aplay. Now the server waits 2 s and reports it to the app.
- **Open:** try it in the browser on a laptop and a phone (certificate warning, delay, volume).

### 03.11.2026 – Precise drive commands: "2 meters forward", "turn 90 degrees"
- **What:** New commands that drive a distance or an angle precisely:
  - App: panel "Genau fahren" (enter distance or angle, choose direction), messages `move` and `turn`.
  - Speech: „Robi, fahr zwei Meter vor“, „Robi, einen halben Meter nach links“, „Robi, dreh dich um neunzig Grad nach rechts“, „Robi, dreh dich um“. It announces what it is doing and speaks up if it did not work.
  - New module `navigation/maneuver.py`.
- **How:**
  - The lidar measures: every scan is laid onto the one from the start (ICP); from that follows how far it really drove or turned. Wheels and drive command only estimate the time between two scans.
  - When driving straight, the controller also keeps the lateral deviation and the heading at zero. Mecanum compensates sideways.
  - At the end it measures again while standing still and corrects. While driving, a lidar revolution is distorted.
  - The camera is not needed: it only looks forward and does not measure distances.
- **Why lidar:** Driving by time is no good. The drive command overestimates the motion by 10–30 %, mecanum slips when turning, and the gyro has a bias and the wrong sign (sensor check 28.10.).
- **Result:** Simulation with a deliberately slipping drive (turns 20 % less than commanded, the wheels report even less), `Marlon/autonomous_driving/test_maneuver.py`:
  - 1 m forward, 0.5 m sideways, 0.3 m back: 1–2 cm off, straight.
  - Turning 90°, 180°, 360°: 1–2° off. Without the lidar it would have been about 18° at 90°.
  - In front of the wall it stops and reports it. Cancelling stops immediately.
  - With a simulation server on the Pi: command from the app and voice command go through, the announcement comes. The larger Vosk grammar (2209 sentences) builds in 0.01 s.
- **Problems along the way:**
  - At first the reference scan changed every 25°. Each change added about 0.2° of error; after 360° it was 3.4° off. Now it matches against the start scan and only changes on a bad match.
  - It stopped just inside the tolerance (2.9 of 3 cm). Now it drives up to half the tolerance.
  - Found: if port 8443 was taken, the HTTPS part brought down the whole server. Now HTTP keeps running anyway.
- **Open:** test on the floor (real lidar, carpet/smooth floor). Adjust speed and tolerances if needed. In large empty rooms or long corridors the lidar finds less to hold on to.

### 03.11.2026 – Floor test of precise drive commands, turning improved
- **What:** First test on the floor, via app, via speech and via remote control over SSH.
- **Result:**
  - 1 m and 2 m forward: 1 cm off according to the lidar. Voice commands („Robi, fahr zwei Meter nach vorne“) go through.
  - Turning in open space: 45° accurate to 0.1°, 90° left to 0.9°, 90° right to 1.1°, 180° to 0.4°, 360° to 0.0°. Meanwhile it drifts 3–12 cm (mecanum).
- **Problems:**
  - When turning it often said „Das hat nicht ganz geklappt“. There were two causes behind this:
    - Something was closer than 30 cm (legs, furniture). It then does not turn, for safety reasons.
    - It corrected three times (+1.3°, +2.2°, +1.7°) without moving. The estimate already considered 1–2° done after one tick, although the ramp was only outputting 0.3 rad/s and the motors did not start from standstill.
  - Solved:
    - From standstill, a turn only counts after `MANEUVER_STARTUP_S` (0.25 s).
    - It now says the reason: „Zum Drehen ist es mir hier zu eng“, „Da ist etwas im Weg …“, „Fast geschafft, ich liege 2 Grad daneben“.
  - Found along the way:
    - `motor.py` in the repo on the Pi drives the board directly. If it runs next to the server, both fight over the port and the motion measurement fails (log 10:51).
    - `server.log` belongs to root; since the server runs as `pi`, nothing ends up in it anymore.
- **Open:** check the improved version on the floor. Cross-check the angle with a line on the floor; so far the lidar only checks itself.

### 03.11.2026 – Speech recognition only via the plugged-in microphone
- **Problem:** Sometimes speech recognition listened through the microphone of the Logitech C270. That sounds so bad that hardly any command got through.
- **Cause:** `SPEECH_MICROPHONE_NAMES` contained "C270" as the last fallback. If the USB microphone was briefly gone after a USB dropout, the retry took the C270.
- **What:** Removed "C270" from the list; the plugged-in FongLun ("USB Microphone", card 5 on the Pi) comes first. New `SPEECH_MICROPHONE_NEVER` (C270, Logi, WebCam, Orbbec): the automatic search never takes these devices, even if a generic name like "Microphone" matches. If the USB microphone is gone, recognition would rather be off and searches again every 3 s.
- **Result:** Checked on the Pi with the real device names: C270 and Orbbec are filtered out; only "pulse" and "default" remain, which match no name. The running server keeps the USB microphone occupied (`/sprache/status`: "USB Microphone (hw:5,0)").
- **Open:** after the next server restart, check in `/sprache/status` that `mikrofon` is the USB microphone. Unplug the microphone briefly: recognition must report "kein Mikrofon" instead of switching to the C270.

### 03.11.2026 – Autonomy step A: heading, timestamps, speed limits (Marlon, with Claude)
- **What:** First step of the improvement plan for following and exploring (branch `autonom`). Fix the basics before rebuilding the target lock and the explorer.
- **Why:** The board's IMU attitude permanently delivers 0 (sensor check 28.10.). The odometry took this value as the heading, so `theta` stood still while turning. That explains "theta in the server ≈ 0", why the explorer only turns by time and why homing can compute the wrong direction.
- **Changes:**
  - Odometry integrates the turn rate measured by the board (`ODOM_IMU_HEADING = False`). If the IMU is switched on, a plausibility check tests it: if its angle stands still while turning, it is discarded, with a warning in the log (visible on `/diagnose`). New: `odometry.velocity()` for the later person tracker.
  - Timestamps: the camera remembers the capture time per frame (`latest_with_time()`, `latest()` stays). The pose `timestamp` is now the capture time instead of the processing time. `Scan` knows its duration and `mid_time`.
  - Speed limits: `MAX_SPEED_*` from an ineffective 5.0 to 1.0 m/s and 3.0 rad/s. Following, exploring and homing register with the watchdog as `source="autonom"` and are limited to 0.35 m/s / 1.2 rad/s, below `BATTERY_WARN_V` to half of that.
- **Result:** New test `Marlon/autonomous_driving/test_odometry.py` passes (90° turn despite IMU = 0, driving straight afterwards in +y, frozen IMU discarded, manual and autonomous limits, battery throttling). `test_gestures`, `test_voice`, `test_speech`, `test_obstacles` and `test_diagnostics` still pass. Smoke test 18/19, exactly as before; "homing starts" fails even without the changes (see open points).
- **Problems:** The first smoke runs showed 14/19, because `main.py --mock` automatically opens a browser: the web app connects and sends its own drive and stop commands in between. With `--no-browser` the tests run cleanly.
- **Open:** 90° turn test on the floor against the lidar (is `theta` right?).

### 03.11.2026 – Autonomy step B: target lock while following (Marlon, with Claude)
- **What:** Person tracker with track IDs and a target lock with a state machine (`interaction/target.py`, new). The pose delivers a color feature of the torso; the mock pose shows a second person at times.
- **Why:** While following, the robot jumped over to other people. The association only used the camera angle, and after every search it took the largest person in the image.
- **Changes:**
  - Tracker: Kalman on the angle, own rotation subtracted using the odometry, greedy association.
  - `TargetLock` with the states idle → acquiring_target → following ↔ target_temporarily_lost → searching_for_target → safe_stop. Re-finding only with a matching feature, a clear margin and 5 confirmed frames. When in doubt, it stands still.
  - Gestures are detected per person: the follow gesture makes the person gesturing the target. While following, only the target's gestures count; anyone may say stop.
  - New field `state.interaktion.ziel`. The feature stays in the robot (not in `/interaktion/pose`), only in RAM.
- **Result:** New test `Marlon/interaction/test_target.py` passes, as do all previous tests (`test_gestures`, `test_speech`, `test_voice`, `test_obstacles`, `test_odometry`, `test_diagnostics`, `test_explorer_sim`). Smoke test 18/19, unchanged.
- **Problems:** Two test assumptions were wrong, not the code. Without own rotation the association does not break, because the gate grows along; the test now checks the estimated velocity. After 1 s of occlusion the track is still alive, so the target is back immediately; it is now checked short (immediately) and long (2 s, new confirmation).
- **Deliberately different from the plan:** the times are still called `FOLLOW_LOST_S`/`FOLLOW_SEARCH_S`. It already starts driving during acquisition as soon as the target is unambiguous.
- **Open:** ~~tapping in the web app (step C)~~ (done, see below), floor test with two people (crossing, occluding, same clothes).

### 03.11.2026 – Autonomy step C: choose target, pause and release in the web app (Marlon, with Claude)
- **What:** The web app can determine the target when following: tap a person in the camera image, pause/resume, release, display of the target state.
- **Why:** The robot is controlled via the web interface. With several people, the target lock from step B refuses a start without a selection; that needs an unambiguous selection in the app.
- **Changes:**
  - `protocol.py` only extended: `follow` with optional `ziel_id`, new `follow_pause` and `follow_resume`. New field `state.interaktion.pausiert`.
  - `/interaktion/pose` delivers `id` (track ID) and `ziel` per person; the feature still stays in the robot.
  - `behavior.py`: `follow(target_id)` also changes the target while following; a lost ID is refused. Pause stops, tracker and lock keep running. Every mode change ends the pause. `follow_stop` acknowledges a `safe_stop`.
  - Web app: people can be tapped (hit test on the box, smallest wins), target green, others gray, "Person N" above the head, row "Ziel", buttons "Pause/Weiter" and "Loslassen". A separate button "Neu wählen" is not needed, because tapping does exactly that.
- **Result:** `test_target.py` extended by 27 checks, everything passes; all other tests too. Smoke test 19/19. Checked in the mock: wrong `ziel_id` is rejected, pause/resume, unknown ID, `id`/`ziel` in `/interaktion/pose`.
- **Problems:** Two test failures were due to the test: the text "folgt Person N" is replaced by the drive text in the next tick, and one test phase ran backwards in time (the pause took longer than assumed). The stop gesture during the pause works.
- **Open:** try tapping on a phone/tablet (hit area), floor test with two people.

### 03.11.2026 – Autonomy step D: explorer without getting stuck (Marlon, with Claude)
- **What:** The explorer (`navigation/explorer.py`) turns by measured angle, waits for a fresh scan after turning, frees itself when points are too close and detects when it is stuck.
- **Why:** Until now it stood still without a time limit when points were closer than 0.22 m, turned only by time (the heading was missing, see step A) and gave up after six failed attempts with "festgefahren".
- **Changes:**
  - Turning: target from the board's turn rate, stop 8° early (overrun), time limit as an emergency brake, without measurement by time as before.
  - Close points: first wait 3 s (usually a person), then slowly sideways/backwards in the freest direction, without turning. New state `befreit sich`, counter `befreien` in `state.autonomie`.
  - Stuck detection: the board reports hardly any speed despite a drive command, or the lidar picture all around (12 beams) changes in at most one direction for 1.5 s. Then unsticking; only after three unsuccessful unstick attempts does the drive end.
  - All values as `EXPLORE_*` in `config.py`.
- **Result:** New test `Marlon/autonomous_driving/test_explorer.py` (simulated room with walls and obstacles, with and without board measurement): turn angle, unsticking instead of a permanent stop, getting stuck, no false alarms in the room and in the mock room. All other tests pass, smoke test 19/19. 20 s of exploring in the mock: drives, no unsticking, turn +163° for a target of +170°.
- **Problems:** The first stuck check only looked forward. If it drives diagonally along a wall, that distance stays the same: false alarm. Hence now the picture all around. In the mock there was still a false alarm afterwards, because beams at the range limit dropped out and only the unchanging side walls remained; now a beam appearing/disappearing also counts as motion.
- **Limits:** It does not detect spinning wheels in open space without board measurement (too few beams in range). A person moving next to it can mask the lidar check; the board and close-range checks remain.
- **Open:** floor test: turn angle against the lidar, unsticking from a corner, getting stuck (e.g. on a chair leg).

### 03.11.2026 – Autonomy step E: braking distance check and follow distance (Marlon, with Claude)
- **What:** A central guard in the watchdog brakes all autonomous drive commands according to braking distance. When following, the distance to the person comes from their lidar cluster instead of the smallest value in the field of view.
- **Why:** Explorer, homing and following each checked obstacles themselves, with different rules; a bug in one module would have taken effect unbraked. When following, a chair in front of the person counted as their distance, and the robot stopped although they kept walking.
- **Changes:**
  - `AutonomSchutz` (`safety/collision.py`): allowed speed from `v·t + v²/(2a) = free − margin`, `t` = latency + age of the scan, free path in the drive corridor in the direction of travel (`free_in_direction`). The direction is kept, only the speed drops. Without a scan or with a scan older than 0.5 s, only turning. Only applies with `source="autonom"`, manual control unchanged. Can be switched off with `AUTONOMOUS_GUARD_ACTIVE`.
  - Braking goes through the ramp, not `stop()`, because of the current spikes (USB resets).
  - Following (`interaction/behavior.py`): clusters from `navigation/obstacles.py` within the box area, precedence person → matches the last distance → not a furniture leg → angle. Without clusters, the minimum as before. Its own obstacle detection, because the server's works with person angles and there a chair would also be called "Mensch".
  - Values: `AUTONOM_SCHUTZ_*`, `FOLLOW_CLUSTER_*` in `config.py`.
- **Result:** New test `Marlon/autonomous_driving/test_guard.py`: braking formula, wall in front/to the side, backwards and sideways free, diagonal keeps its direction, outdated scan, watchdog only for autonomous; following: person behind a chair leg (1.45 m instead of 0.79 m), two objects with/without last distance, only a wall → minimum. All other tests pass, smoke test 19/19. 40 s of exploring in the mock: drives, no intervention by the guard, no getting stuck.
- **Problems:** The first test scenes were chosen too strictly (with a wall at 0.5 m, 0.52 m/s is still allowed by the formula); scenes adjusted, the formula stayed. `Scan.min_distance` does not skip beams without an echo (r = 0); avoided in the test, check on the robot whether such beams arrive.
- **Open:** floor test; measure `AUTONOM_SCHUTZ_BREMSEN` and `AUTONOM_SCHUTZ_LATENZ_S` on the robot (braking distance from 0.3 m/s).

### 03.11.2026 – Autonomy step F1: drive recorder and replay (Marlon, with Claude)
- **What:** The server always keeps the last minute of driving in memory (lidar scans, drive commands before and after the limits, stops, board speed, pose, states). The button "Fahrt speichern" in the web app writes it to a file. `scripts/replay.py` replays it on your own computer.
- **Why:** Next, the robot is supposed to drive around doors and obstacles skilfully and notice when it is lifted. That needs real drives for checking; otherwise the new logic is only built against the simulation. A lidar map or GPS localization was deliberately postponed for this; indoors GPS does not help.
- **Changes:**
  - New module `recording.py`: ring buffer (`RECORDING_BUFFER_S` = 60 s), saving as gzip JSON lines via a temporary file, only the newest `RECORDING_MAX_FILES` are kept; plus `load`, `Playback` (plays back scans and odometry with correct timing, with the same interface as lidar and odometry) and `replay_explorer`.
  - The watchdog reports every command and every stop to the recorder; an error there is caught and does not disturb driving. Scans, odometry and states are collected by a separate loop in the server at the odometry rate.
  - New command `record_save` (only added) and endpoints `/aufzeichnung/status`, `/liste`, `/datei/<name>` (only valid file names, no path tricks).
  - `scripts/replay.py`: summary (scan rate, gaps in the lidar, how often the braking distance guard braked, state changes), SVG with path and lidar points, comparison with the current explorer.
- **Result:** New test `Marlon/autonomous_driving/test_recording.py`: buffer, duplicate scans, save/load, cleanup, file names, watchdog hook (also with a deliberately broken recorder), playback. A 30 s explorer drive in the simulation is recorded, saved and replayed: 600 of 600 ticks identical. On the mock server: saving, listing, downloading and playing back work. All other tests pass.
- **Problems:** Playback is only identical with the same randomness. Without it, it was 59 % in the simulation (the explorer rolls dice for wander times); for real drives therefore only the rate counts. The smoke test "homing starts" fails in about 2 of 3 runs, with and without this change (cross-checked against the version of step E); cause still open.
- **Open:** record a few drives on the robot with doors, narrow spots and lifting; check whether recording puts a noticeable load on the Pi. Then step F2: local avoidance, checked against these recordings. Investigate the flaky smoke test "homing starts".

### 03.11.2026 – Autonomy step F2: local avoidance and lifting (Marlon, with Claude)
- **What:** The explorer drives around obstacles in arcs instead of stopping and turning. It approaches doors through the middle. It briefly remembers what slips out of view next to it, and detects when it is lifted or carried somewhere else.
- **Why:** In everyday use the robot gets picked up, and doors and other obstacles are in the way. Before, it turned around in front of every obstacle, got through doors badly and, after being put down, continued with outdated assumptions. GPS does not help indoors, a lidar map comes later. Hence local, using only what lidar and board deliver.
- **Changes:**
  - New module `navigation/avoidance.py`:
    - Short-term memory for lidar points (3 s, 1 m radius), carried along using the board speed.
    - Arc planning: turn rates and some sideways shifting computed a good second ahead, the robot's outline checked with a margin, then back onto the course.
    - Gap or door search (middle of free areas).
    - Lift detection (share of changed 10° sectors, own motion subtracted).
  - `explorer.py`:
    - Drives with `plan_arc`; only turns when no safe arc fits.
    - Picks gap centers when turning.
    - New state `hochgehoben`: stops, then forgets memory, course and counters.
    - No random change of direction while something is closer than 0.45 m (`AVOID_NARROW_M`).
  - Values as `AVOID_*` in `config.py`; `docs/api.md` and `docs/architecture.md` extended.
- **Result:**
  - New test `Marlon/autonomous_driving/test_avoidance.py` (simulated world with walls, circles, tilting and carrying), all cases pass:
    - Door search: door found, the direction safely passes both posts.
    - Avoiding without turning around, with and without measured turn rate; corridor.
    - Three door passages through the middle (shift ≤ 2 cm, ≥ 0.33 m clearance).
    - Free door search from the room: 4 of 4 runs through within 120 s.
    - Lifting and putting down detected (before 5.6 s); no false alarm in the room.
  - All previous tests pass; replay still 600/600 identical. Smoke test 19/19, apart from the known flakiness of "homing starts".
- **Problems:**
  - The free door search never succeeded at first. It arrived in front of the door in the middle and then turned around exactly there. The reason was not the avoidance, but the random change of direction that happened to be due right then. Solution: the change waits until the narrow spot is behind it.
  - Slow carrying went unnoticed from scan to scan, because the picture changed too little. Solution: with a measured pose it compares over a window of 1 s (`LIFT_WINDOW_S`). Measured: driving in the room at most 11 % changed sectors, carrying 40–83 %.
  - **Limits:**
    - Without board speed only the scan-to-scan comparison remains. Then its own driving already changes up to 22 %, so slow carrying can go unnoticed.
    - Lifted vertically, without tilting, the lidar picture hardly changes. It does not detect that.
  - The door center of the free area lies a few degrees off the geometric center (30° instead of 26.6°). The post still stays outside the drive corridor, so the test checks this clearance instead of the angle.
- **Open:**
  - Record drives through doors and with lifting on the robot and check them against the new explorer with `scripts/replay.py`.
  - Re-measure the rear dimension (`AVOID_REAR_M`).
  - Tune the weights of the arc scoring on real drives.

---

### 03.11.2026 – Autonomy step F3: detour while following and homing (Marlon, with Claude)
- **What:** The avoidance from F2 now also applies to follow, come here, go away and homing. If something is in the way, it shifts diagonally past it and takes doors through the middle. If it is lifted meanwhile, it stops.
- **Why:** Until now only the explorer could avoid obstacles. When following, it stopped in front of a chair and lost the person; homing stopped in front of every obstacle.
- **Changes:**
  - `navigation/avoidance.py`: new function `choose_direction` (free direction as close as possible to the goal, with margin, middle in narrow spots, preference for the side chosen last).
  - `interaction/behavior.py`: detour and lift detection for follow, come here and go away. Lifted → READY with an announcement.
  - `navigation/homing.py`: detour instead of stopping. Only when there is no room anywhere for 3 s does it stop and report how far it still is. Lifted → abort. Without a fresh scan, as before.
  - `navigation/localizer.py`: `scan()` returns the last fresh scan.
  - Values `DETOUR_*` and `HOME_DETOUR_WAIT_S` in `config.py`; `docs/api.md` and `docs/architecture.md` extended.
- **Result:**
  - New test `Marlon/autonomous_driving/test_detour.py`, all cases pass:
    - Following around a chair: swerves sideways, at least 0.25 m of clearance, gets to 1.17 m from the person.
    - Following through a door: 0.25 m clearance to the posts, final distance 1.16 m.
    - Homing around a box: gets to 0.12 m from the start point, 0.25 m clearance.
    - Dead end: stops with "Hindernis in 0.40 m, noch 3.00 m bis zum Startpunkt".
    - Lifting detected while following and while homing.
  - All 13 test scripts pass. Smoke test 18/19, only the known flakiness of "homing starts".
- **Problems:**
  - When following around the chair, it stopped at its corner with "etwas sehr nah". After shifting past, it checked straight ahead without a margin and swung back too early. Solution: the straight-ahead check uses the same margin as the avoidance.
  - A test case "wall across in front" was set up wrong: diagonally past a finite wall, 1 m really is free. The test now requires 2 m.
  - **Limits:** Homing has no map. It gets around individual obstacles, not through labyrinths. After being lifted it aborts, because the position is no longer right.
- **Open:**
  - Record following around chairs and through doors on the robot and check with `scripts/replay.py`.

### 03.11.2026 – Branch `autonom` merged into `main`
- **What:** Merged autonomy steps A to F3 into `main`. In case of contradictions, `main` takes precedence.
- **Conflicts:** All of them in `server.py`, `config.py`, `settings.py` and `behavior.py` were additions from both sides. Both are kept:
  - `server.py`: precise drive commands (`Maneuver`, drive priority via `sender`) next to `AutonomSchutz`, the drive recorder and odometry for explorer and interaction.
  - `behavior.py`: the soft start from `main` takes precedence. Following via the app during the startup only starts after the announcement; after that, target selection by tapping applies. When searching, `FOLLOW_SEARCH_S = 0` stays "without time limit" (from `main`), and in addition the target lock stops safely when giving up.
- **Adjusted:** The precise drive commands now send `source="autonom"`. So the tighter speed limit and the central braking distance check apply to them as for following, exploring and homing. Before, they went through as manual control.
- **Result:** All changed files parsed on the Pi, the whole package can be imported.
- **Open:** check on the floor: precise drive commands, following with soft start and tapping. The branch `feat-mapping` has its own central check with the drive guard. At the next sync with `main`, the drive guard and `AutonomSchutz` must be merged into one layer.

### 03.11.2026 – Mapping: SLAM got no lidar data
- **Problem:** On `/map` mapping could be started, but no map ever appeared, and there was no feedback.
- **Cause:** The mapping module only started slam_toolbox in the container `x3`. That is only the map builder and needs `/scan` and the position as TF. Those would come from `sllidar_node` and `yahboomcar_bringup`, which must not run on our setup because our server has lidar and board open. So slam_toolbox waited forever. That nobody noticed was due to three things: the start counted as successful if the process was still alive after 0.3 s. The SLAM log lay unnoticed in the container. And there was no time limit for the first map.
- **What:**
  - New `mapping/scan_feed.py`: our server passes scans and odometry via `docker exec -i` as JSON lines to a small ROS node in the container. It publishes `/scan`, `/odom`, `/tf` and `/tf_static` for slam_toolbox (decision E8).
  - The scan is put onto a fixed grid of 360 beams, because slam_toolbox discards scans with a changing beam count. The position at the time of measurement is sent along. Jumps caused by a newly set start point are masked out.
  - Before the start it checks whether slam_toolbox is installed. The status names the first step that fails (lidar, feed, SLAM, map). The page shows this together with the SLAM log.
  - When the server stops, stopping the mapping only runs after the robot has stopped, and inside a `try`. An error in it (Docker not there) used to skip disconnecting the hardware.
- **Result:** Tested on the laptop with dummies instead of Docker and ROS: grid, jump compensation, feeding the real node script (against re-created ROS classes), restart after a crashed node, all hint texts. Not yet on the robot.
- **Open:** check on the Pi: `docker exec x3 bash -lc "source /opt/ros/foxy/setup.bash; ros2 pkg prefix slam_toolbox"`, then start mapping, drive through the room, save the map.

### 03.11.2026 – Mapping drive like a robot vacuum, rotation from the lidar
- **Problem 1:** During mapping the robot did not drive. SLAM builds the map only from what the lidar sees; standing still, that is a single viewpoint with shadows behind every piece of furniture.
- **Problem 2:** The odometry knows no rotation. The IMU always reports 0 as the heading, so the position does not rotate when the robot turns. SLAM would have twisted the map at the first turn.
- **What:**
  - `navigation/scan_odometry.py`: every lidar scan is laid onto one taken shortly before via ICP. That yields distance and above all rotation. The wheels only provide the initial guess. The feed into ROS now uses this position.
  - `navigation/mapping_drive.py` as a second mode of the explorer: to the nearest wall, once all the way around with the wall on the right, then lanes 0.8 m apart up to the opposite wall. On the lanes it does not turn but drives forward, slides sideways, drives backwards (mecanum). It holds the lane direction to the direction of the walls, which it measures anew in every scan.
  - Page `/map`: checkbox "nach dem Start automatisch abfahren", button "Kartierfahrt starten/anhalten", display of the current step. If the last `/map` page closes, it stops.
  - Because the drive is a mode of the explorer, emergency stop, manual intervention, homing, interaction and drive loss end it as before.
- **Result (simulation, room 8 × 6 m with a pillar, 12 % wheel slip):** complete drive in 4 min: 27 m around, then 6 lanes, ending at the opposite wall. Deviation at the end: lidar odometry 0.3 m and 4.5°, wheels alone 4.1 m and 49°. Found and fixed along the way:
  - The pillar was the nearest thing; it drove around it and took that for the room → island detection (loop under 5 m).
  - Lanes were slanted because they followed the drift → alignment with the walls.
  - Wall direction failed at 3 cm of measurement noise → longer wall segments, since then at most 1° error.
- **Open:** check on the floor. First watch at low speed: wall following, corners, shift, backward lanes. Then check whether "once all the way around" is detected, and what the map looks like.

### 03.11.2026 – Drive guard based on the footprint, old map gone
- **Problem 1:** Collision avoidance did not work reliably. It checked a cone from the lidar center (misses table legs next to the wheels) and blocked hard at the limit (with 0.22 s of delay it keeps rolling). Turning was always allowed, scans could be 2 s old. Autonomous drives only checked individual directions.
- **Problem 2:** After a new start the old map was still there. If SLAM from an earlier run was still running in the container, it was simply reused, with the old map and the old origin. Map reception and page kept the last map.
- **What:**
  - New `safety/drive_guard.py`: for every command it computes ahead where the robot's footprint will go, also in an arc and when turning. It brakes continuously so that the stopping distance fits into free space. Driving away stays allowed.
  - The guard always applies to explorer, mapping drive, homing and interaction (in the watchdog), and to manual control when the obstacle stop is on. The control keeps its meaning.
  - New values in `config.py` (`ROBOT_*`, `GUARD_*`). The clearance for autonomous driving is adjustable on the settings page; the old cone angle is gone.
  - If an autonomous drive does not get past the obstacle for 10 s, it gives up.
  - Every mapping start is a new map: end the old SLAM process, clear map reception and page.
  - Mapping drive: if it is too narrow sideways during the shift (furniture instead of a wall), it drives back up to 2.5 m along the lane and shifts there. Before, it stopped too early at the pillar.
- **Result (simulation):** full speed towards a wall with 0.22 s of command delay: at 0.2, 0.5 and 0.8 m/s it stops 6 cm in front of the edge every time. A table leg at the front right is detected; the old cone would have missed it. Turning close to a wall is blocked, driving away is not. The mapping drive with the guard now covers the whole room (6 lanes, also next to the pillar). At the end the lidar odometry is 0.19 m and 3° off reality.
- **Open:** re-measure `ROBOT_REAR_M` on the robot. Check on the floor: switch on the obstacle stop in the app and drive towards a wall, a chair and a table leg. The lidar does not see low obstacles.

### 03.11.2026 – Mapping branch brought onto main, precise drive commands with drive guard
- **What:** Rebased the branch `feat-mapping` onto the current state of `main`. In the meantime the precise drive commands (`maneuver.py`) had been added there. There were conflicts only in this diary, in `server.py` and in `navigation/__init__.py`. In all three cases both sides are kept.
- **Adapted to the drive guard:**
  - The precise drive commands sent their commands to the watchdog without the drive guard. Now, like all autonomous drives, they run with `schutz=True`. If the guard blocks for 1.5 s in a row (`MANEUVER_BLOCKED_S`), the command gives up and says that something is in the way.
  - Before turning, a circle of 30 cm around the lidar used to be checked. In the floor test it often blocked for no reason when legs or furniture were in front, although the robot only protrudes 15 cm at the front. Now `DriveGuard.turn_free()` checks the footprint over the whole angle. The old circle only applies when there is no drive guard.
  - When a mapping drive starts, it cancels a running precise drive command. Before, two programs would have steered at the same time.
  - `test_maneuver.py` wires up the drive guard like the server.
- **Open:** check on the floor: "90 Grad drehen" with furniture 20–30 cm in front of the robot (must work now) and right next to a wall (must be refused). Run `test_maneuver.py` on the Pi.

### 03.11.2026 – One guard layer: drive guard and AutonomSchutz merged
- **Problem:** After merging `autonom` into `main` there were two central checks for autonomous driving in the watchdog: Marlon's `AutonomSchutz` (`quelle="autonom"`) and the drive guard from `feat-mapping` (`schutz=True`). Both would have checked twice and differently.
- **Comparison:**
  - `AutonomSchutz`: speed according to the braking distance to the nearest point in a strip in the direction of travel. Turning always allowed, arcs not computed ahead. Well integrated: speed limit, battery throttle, drive recorder, can be switched off.
  - Drive guard: computes the footprint ahead, also in an arc and when turning. Exactly then the X3 bumps its corners (exploring, turning around while mapping, "90 Grad drehen").
- **Decision:** Marlon's interface stays (`source="autonom"`, `AUTONOMOUS_GUARD_ACTIVE`), and inside it the drive guard does the checking. `AutonomSchutz` and `schutz=True` are dropped. Taken over from `AutonomSchutz`: age of the oldest scan point, preparing the points once per scan, counting interventions and logging them debounced (decision E10).
- **Shared footprint:** the arc planner and the drive guard computed with different dimensions (rear 0.15 instead of 0.17 m, sides 0.13 instead of 0.125 m) and margins (5 and 6 cm). At the sides and the rear the drive guard would have been stricter than the planner and would have braked its arcs. Now `AVOID_*` uses the `ROBOT_*` values, and the drive guard margin is lowered to 4 cm, below the planner's 5 cm.
- **Rebase:** rebased `feat-mapping` onto `main`. In the explorer, Marlon's sequence (lidar check, lifting, close range, unsticking) remains the frame; the mapping drive hooks in after it. So unsticking and lifting also apply while mapping.
- **Result:** On the Pi the whole package can be imported, the test scripts compile. `test_guard.py` now checks the drive guard (new as well: turning next to the wall, table leg next to the wheel).
- **Open:** run `test_guard.py`, `test_explorer.py`, `test_avoidance.py`, `test_detour.py` and `test_maneuver.py` on the Pi. On the floor: exploring and following in narrow rooms. The drive guard now also blocks turns in which a corner would bump, which `AutonomSchutz` always let through. Re-measure the rear dimension `ROBOT_REAR_M`.

### 04.11.2026 – Depth camera and progress watch against getting stuck
- **Problem:** While mapping, the robot got stuck on chair legs and similar things that the lidar does not see on its plane, or sees too late. The wheels turned, it did not move, and the map stopped growing. The explorer's stuck detection only applied to random driving, not to the mapping drive.
- **What:**
  - **Depth camera** (`sensors/depth.py`): the depth channel of the Orbbec Astra runs via our own ctypes binding to Orbbec's OpenNI2 (from the Yahboom container, `scripts/setup_orbbec.sh`). The pip package `openni` does not work under Python 3.13. Every frame becomes obstacle points: everything 4–40 cm above the floor. It looks for the floor as a plane in the image and uses it to compensate for the mounting orientation. The camera only runs during autonomous drives, because its projector draws a lot of current on the USB hub.
  - **Obstacle memory** (`navigation/obstacle_memory.py`): the Astra only measures from 0.6 m. So a chair leg disappears from the image shortly before it matters. The memory keeps the points and moves them along with its own motion, using the lidar odometry while mapping. Explorer, mapping drive and drive guard see them like lidar points.
  - **Progress watch** (`navigation/progress.py`): it compares the motion output by the watchdog with the motion measured by lidar matching (ICP). If less than 30 % arrives over 2 s, it is stuck. Then it places a phantom obstacle at that spot (30 s) and pushes itself free in the opposite direction. Applies in both modes. So that it does not judge wrongly in long corridors, the match is computed twice and must be unambiguous.
  - New: `GET /tiefe/status`, `POST /tiefe/test` (camera on for 20 s), the fields `tiefe` and `festgefahren` in `state.autonomie`. Decision E11.
  - **Viewing the depth image:** `/tiefe/stream` shows the depth in color (red near, blue far, black no measurement), obstacles in magenta, plus the distance in the image center. `/tiefe/ueberlagert/stream` overlays this on the Orbbec's color image. Both can be selected in the app's camera switcher. So you can see directly what the camera measures and what counts as an obstacle.
- **Result (on the Pi, standing still):** the camera connects, delivers ~7.5 frames/s, switches off when nobody needs it, and reports obstacles at 0.58–1.33 m. The floor was not detected during the test (robot was standing raised).
- **Open:**
  - On the floor `/tiefe/test` and `/tiefe/status`: must `boden_erkannt` be true? Is `kamera_hoehe_m` right? Hand on the left → `naechstes` with positive y.
  - Re-measure camera height and position (`DEPTH_CAMERA_*`).
  - Mapping drive with chairs in the room.
  - Watch the log for "Progress watch" to see whether it errs in open space.
  - Watch for USB resets while the camera is running.

### 04.11.2026 – Faster tests and helpers for working with AI (Marlon, with Claude)
- **What:**
  - New script `scripts/run_tests.py`: it starts all test scripts at the same time, writes one line per script and shows the full output only for failures. With a name filter only individual tests run (`python scripts/run_tests.py target gestures`).
  - Four subagents for Claude Code in `.claude/agents/`: `test-runner` (run tests), `smoke-tester` (check the server in the mock on port 8037), `rule-checker` (diff against the AGENTS.md rules) and `doc-writer` (project diary and `docs/`).
- **Why:** One after the other, the 14 tests took over ten minutes, and the pages of output from the passed checks used up a lot of context when working with the AI. The subagents do routine work with smaller models without filling up the main conversation.
- **Result:** All 14 tests in about 6.5 minutes. The slowest script (`test_avoidance.py`) determines the duration.
- **Problems:** In the first run after merging `main`, two tests failed. On `main` the robot now searches without a time limit, and a re-find only counts after several confirmed frames. The tests were adjusted accordingly.

### 04.11.2026 – Four tests repaired after the mapping merge (Marlon, with Claude)
- **What:** `test_explorer.py`, `test_avoidance.py`, `test_recording.py` and `test_guard.py` failed on `main`, even on the unchanged state of `origin/main`. All four were outdated tests, not a bug in the robot code.
- **Why:**
  - The new progress watch (`navigation/progress.py`) reads `watchdog.current_command`. The watchdog dummies in the three explorer tests did not have this field. They now remember the last accepted command, like the real watchdog.
  - `test_guard.py` "wall on the left: braked sideways to the left": the drive guard margin was deliberately reduced from 6 to 4 cm. With a wall at 0.30 m that leaves 13.5 cm of room, and the stopping distance at 0.25 m/s is about 13 cm. So the drive guard rightly does not have to brake there. The test now places the wall at 0.25 m; there it brakes to 0.16 m/s.
- **Result:** 14/14 tests passed.
- **Problems:** The full run now takes about 12.5 instead of 6.5 minutes. The progress watch now really runs along in the explorer tests, and `test_avoidance.py` alone needs 747 s.
- **Open:** check whether `test_avoidance.py` can be shortened.

### 04.11.2026 – Autonomy F4: remembered person and faster re-finding (Marlon, with Claude)
- **What:** The robot remembers the appearance of the person who last triggered following, and takes them again at the next "follow" without tapping. If it loses the person, it finds them again faster: shorter deadline if they disappeared at the image edge, lidar hint for the search direction, holding still while checking a candidate, and faster confirmation on a clear match.
- **Why:** User request: „er soll auch der selben person hinterherfahren und sich merken, wer den folgenbefehl gegeben hat. außerdem, wenn er die person verloren hat, muss diese schneller gefunden werden.“ Until now, with several people in the image, it always refused without tapping, and after losing someone it turned blindly at a fixed rhythm, regardless of where the person last was or what the lidar saw.
- **Changes:**
  - `TargetLock` (`interaction/target.py`): new field `remembered` (color histogram of the torso, RAM only) with a timestamp. Starting "follow" without tapping with several people visible: if the remembered person clearly matches, they are taken, otherwise "bitte antippen" as before. When tapping (`acquire(..., feature=...)`), the feature of the chosen person is the reference immediately, so a brief track loss during acquisition does not lose them. `forget()` deletes the remembered person in addition to the normal `release()`; it is kept on the stop gesture and a safe stop, and expires after `TARGET_REMEMBER_S` = 120 s.
  - If a candidate clearly matches when re-finding (distance below half the threshold, margin to the second best at least twice as large), `TARGET_CONFIRM_FAST_FRAMES` = 3 instead of 5 confirmation frames are enough.
  - `behavior.py`: if the person disappeared near the image edge in the state "kurz weg" (`|angle| > FOV/2 − TARGET_EDGE_DEG`, 8°), the search already starts after `FOLLOW_LOST_EDGE_S` = 0.7 s instead of `FOLLOW_LOST_S`. When searching, it turns towards a lidar person cluster (pair of legs) instead of blindly towards the last side if the cluster lies outside the camera image; if it is in the image, it holds still for `TARGET_HINT_S` = 1.5 s to look, then back to the fixed search rhythm. The lidar never takes over the target, only the viewing direction. While the lock checks a candidate (`checking_candidate`), the robot holds still instead of turning further.
  - New values in `config.py`: `TARGET_REMEMBER_S`, `TARGET_CONFIRM_FAST_FRAMES`, `TARGET_EDGE_DEG`, `FOLLOW_LOST_EDGE_S`, `TARGET_HINT_S`.
  - `docs/api.md` stays unchanged, no new command and no new field in the protocol.
- **Result:** `Marlon/interaction/test_target.py` extended with cases for remembering, expiry after `TARGET_REMEMBER_S`, deletion on release, reference directly on tapping, image-edge deadline, lidar hint, holding still while checking and fast confirmation. All tests passed.
- **Problems:** The shortened deadline at the image edge would have kicked in right after a mode change with a still outdated angle; therefore it only applies in the state "kurz weg" (`TEMPORARILY_LOST`), not right at the start of the search.
- **Open:** floor test: have the same person selected again via "follow" without tapping after releasing; check the lidar hint with a real pair of legs behind the robot.

### 04.11.2026 – Mapping no longer stops, the map gets used: pose, goal drive (with Claude)
- **What:** Two wishes: „Das Mapping soll nicht stoppen“ and „wenn die Karte fertig ist, soll man darauf sehen, wo er ist, und ihn zu einem Punkt schicken können“.
  - **Mapping drive room by room:** after the room (or when it cannot get any further there), it drives to the next spot where the map turns into the unknown ("frontier": open door, corridor, shadow behind furniture), until no reachable one is left. Then it drives back to the origin and reports "Karte vollständig".
  - **Perseverance:** stuck or blocked by the drive guard now means "give up this frontier, take the next one" instead of the end. Without watchers (no `/map` page, no app) it only stops after 20 s, not at the first connection drop. After a USB reset of the drive it continues by itself within 60 s. It waits 30 instead of 3 s for the lidar.
  - **Pose in the map** (`navigation/map_localization.py`): the lidar scan is laid onto a precomputed distance field of the map 4 times per second (`mapping/occ_map.py`); this compensates for the drift of the lidar odometry. After loading a saved map it searches for itself at the origin; if it does not find itself, you show it on the map ("Hier stehe ich").
  - **Goal drive** (`navigation/path_planning.py` with A*, `navigation/path_follower.py`): a point on the map, and it drives a planned path there, through the middle of doors, around unexpected things (blocked spots count as a wall for 60 s, then replan). Runs as homing with a goal, so that emergency stop, manual control and connection loss end it without extra code paths. With a map, "Nach Hause" also drives this path.
  - **Page `/map`:** robot, driven trail, planned path, goal and open frontiers on the map; buttons "Hierhin fahren", "Hier stehe ich" (drag = heading), "Zum Ausgangspunkt", "Fahrt abbrechen"; saved maps with "Laden".
  - **Protocol:** new commands `goto`, `goto_cancel`, `map_pose`, `map_load`, new field `state.karte`, new endpoints under `/api/mapping/` (`docs/api.md`). Rationale: E12.
  - **Simulation:** in `--mock` the map is built in Python (`mapping/simulation.py`), without Docker. With `ROSMASTER_MOCK_WOHNUNG=1` the lidar simulates two rooms with a door, a cupboard and a sofa.
- **Why:** The map was being built, but nobody used it. The mapping drive ended after the first room or at the first obstacle. It also ended when the phone screen turned off or the board briefly dropped off USB; on the X3 that happens often (voltage drops).
- **Result:**
  - `Marlon/autonomous_driving/test_map.py`: 29 checks passed. They cover the distance field, save/load, localization (pulling back, search, ambiguity), drift compensation, planning through the door, frontiers, path follower and the continuation of the mapping drive.
  - End to end in the simulation (apartment, real time, via the HTTP API like the `/map` page):
    - Room covered, 3 frontiers visited, drove into the neighboring room, "Karte vollständig", back at the origin. That took 7 minutes.
    - The pose in the map was within 1–6 cm and under 1.5° of the truth the whole time.
    - The map matches the true walls.
    - Drive to a point in the neighboring room: arrived, 5 cm off the truth.
    - Saved map loaded: where it was not standing at the origin, it did not find itself on its own (as it should be). After "Hier stehe ich" the pose was there within 1 s, and the drive back into the first room worked.
- **Problems:**
  - In the first end-to-end run the simulated map was rotated by about 30° and smeared. It had been entered with the pose of the lidar odometry, whose small rotation errors add up with every reference renewal. On the robot slam_toolbox compensates for this with loop closure. The simulation therefore now enters with the true pose, like a good SLAM, and the localization has to compensate for the odometry drift. That is exactly the situation on the robot.
  - Two empty rooms of the same size cannot be told apart by the localization (rotated by 180° or shifted by 4 m, a scan matches to 91 %). The search therefore requires 80 % hits and refuses when a clearly different pose fits almost as well. That is why there is furniture in the test apartment; real rooms are never empty.
  - The planner ended in the middle of a 10 cm cell instead of exactly at the goal (19 cm off); now it ends exactly at the requested point.
- **Open:** check on the robot (see open points). The existing bug "`dauer_s` right after the start = time since server start" was fixed along the way.

### 04.11.2026 – Microphone off during mapping (Marlon, with Claude)
- **What:** While mapping is running, speech recognition closes the microphone and accepts no voice commands. Afterwards it reopens it by itself (`SpeechRecognition.set_pause()`, setting `SPEECH_OFF_WHILE_MAPPING`). For this, `/sprache/status` and `state.sprache` have the new field `pause`; the app shows "Mikrofon aus: Mapping läuft".
- **Why:** The microphone stream loads the USB bus that the lidar also hangs on, and Vosk needs CPU that SLAM is missing. A misheard command („folgen“) would also cancel the mapping drive.
- **How:** The listening thread keeps asking whether mapping is active, instead of being notified at every start and stop. That way the pause ends on every path (page, app, crash, shutdown).
- **Downside:** "Stopp" by voice does not work during mapping either. Stop via the app, `/map` or by hand.
- **Open:** check on the Pi: start mapping, "Speech recognition paused" in the log, after stopping "pause over, reopening the microphone".

### 04.11.2026 – No longer telling people apart when following
- **Problem:** When following, every person got a number („Person 1, 2 …“ in the app). If the followed person left the image and came back, they had to be recognized by their appearance (colour of the upper body). That often failed: they counted as a new person, and the robot stopped with „bitte neu wählen“.
- **What:** The target lock (03.11.) is replaced by a simple `TargetChoice` (`interaction/target.py`). It follows the person in the image. If they are gone, it waits `TARGET_SWITCH_S` (1 s), then follows the person standing where the last one was. During the search it follows the next one who appears. The tracker stays internal, so the gestures of two people stay separate and it does not jump when someone walks past.
  - App: no numbers above the heads, no tapping, target text without number and percentage.
  - Setting „Aussehen-Schwelle“ replaced by „Wechsel nach“ (`TARGET_SWITCH_S`).
  - Interface unchanged: `follow` still accepts `ziel_id` (older clients), `state.interaktion.ziel` and `/interaktion/pose` keep their fields. `safe_stop` no longer occurs.
- **Result:** `test_target.py` rewritten (57 checks, including walking away and coming back), passed on the Pi; `test_gestures.py`, `test_speech.py`, `test_voice.py` as well.
- **Downside:** If several people are there and the followed one walks away, it follows the other one after a second.
- **Merged with F4 (Marlon, same day):** F4 had extended the recognition by appearance (remembered person, faster confirmation). That goes away with the target choice. Kept are the two parts that do not depend on appearance: searching earlier when the person disappeared at the image edge, and turning towards a person the lidar sees while searching. Agreed with the user; please let Marlon know.

### 04.11.2026 – False "lifted" alarms (with Claude)
- **What:** The lift detection (`LiftDetector`, `navigation/avoidance.py`) reported "hochgehoben" now and then although nobody had touched the robot. It now only counts a scan as changed if the change cannot be explained by the robot moving on the floor.
- **Why:** The detector compares each scan with the one from 1 s earlier and takes the own motion out using the wheel odometry. Whatever the odometry got wrong looked like a jump in the scan: rotation errors on a slippery floor, wheels spinning while stuck. Sectors that only sometimes return an echo (window, open door, black furniture) counted as changed as well. A bump over a door sill tilts the lidar for about two scans, which already met the threshold.
- **Changes:**
  - Before raising an alarm, the old scan is aligned onto the new one with the existing scan matcher (starting from the wheel estimate turned by up to ±40° and from "did not move"). If most points fit (`LIFT_FIT_MIN_SHARE` 70 %) and the lidar shows no displacement beyond what the wheels reported (`LIFT_CARRY_M` 0.25 m), it was the odometry, not a lift. A tilted lidar sees floor or ceiling and does not fit; being carried fits, but as a real displacement.
  - A sector seen in only one of the two scans only counts if what was seen is closer than `LIFT_NEAR_M` (3 m).
  - Three changed scans in a row instead of two (`LIFT_SCANS`), with a measured pose. Without one only consecutive scans can be compared, then two stay (`LIFT_SCANS_UNMEASURED`).
- **Result:** new test `Marlon/autonomous_driving/test_lift.py`: turning with the wheels reporting half the rotation, wheels spinning at 0.4 m/s while stuck, flickering far sectors and a two-scan tilt give no alarm any more (two of them did before). Being carried and a tilt of two seconds are still detected, and the alarm clears after setting down. `test_avoidance.py` (lift cases) passes.
- **Problems:** No recording of a real false alarm existed; the cause was narrowed down in simulation. The Recorder ("Fahrt speichern" in the app right after a false alarm) would show the real case.
- **Open:** observe on the robot whether "hochgehoben" still appears without being lifted; if so, save a recording.

### 04.11./05.11.2026 – Code, tests and docs in English (with Claude)
- **What:** The whole codebase is now in English: identifiers, module and file names, comments, docstrings, log messages, tests and documentation (`docs/`, README, AGENTS.md, this log). What users see stays German (web app, `/map` page, alert texts, diagnostics texts, settings labels), as do the robot's spoken sentences and the speech commands. Protocol and API names (message types, JSON fields and values, URLs) are unchanged.
- **Why:** User request ("write all code in english", then "keep the user interface in german"). The new language rule is in AGENTS.md and in E13 (`docs/decisions.md`).
- **How:**
  - Identifiers: a shared glossary and a rename map (about 2270 names, e.g. `Kartierfahrt` → `MappingDrive`, `KARTE_*` → `NAV_*`, `interaktion/` → `interaction/`). A script renamed exactly the code tokens (not strings, not comments) and reported name collisions per scope. Strings that name code (`getattr`, `__slots__`, settings keys, quoted type annotations) were renamed with it; JSON keys stay.
  - Contract checks: the JSON key structure of all GET endpoints and WebSocket messages was recorded before and compared after: unchanged. Request fields and query parameters that come from identifiers keep their German names via aliases (`winkel`, `nach`); obstacle dicts and recorder state keys are now built with explicit German keys.
  - Settings saved on the robot (`robot/settings.json`) still use the old names; `settings.load()` maps them to the new ones (`_OLD_NAMES`).
  - Comments, docstrings, logs and docs were translated afterwards; a token comparison confirmed that no code token changed.
  - Module files renamed (e.g. `kartierfahrt.py` → `mapping_drive.py`), test files and folders too (`Marlon/autonomous_driving/test_map.py`), scripts (`replay.py`, `run_tests.py`, `setup_orbbec.sh`), docs (`architecture.md`, `decisions.md`, `interaction.md`, this file `PROJECT_LOG.md`) and the Claude Code subagents (`test-runner`, `rule-checker`, `doc-writer`).
- **Problems:**
  - Seven `getattr` strings stayed German at first (the script looked only three tokens back). The simulated lidar then got no true pose and the position snapped back to zero; the smoke test showed it, a token comparison of old and new code found the rest.
  - Keyword arguments that become dict keys (`dict(p, zeit=t)` in tests) changed their key with the rename and were rewritten with explicit string keys.
  - Config name prefixes in the recorder header (`"RAMPE_"`) are not whole names and were fixed by hand.
  - A smoke-test check ("Drehen bleibt trotzdem erlaubt") failed only while about 40 Python processes from earlier mock servers were running ("Server hing 1.2 s"); on a quiet machine it passes.
  - Ben's commits from 04.11. (microphone off while mapping, depth camera crash) were made on the old file names; they are contained in the translated files.
- **Open:** Teammates' branches based on the old names will conflict on merge; merge them soon or rebase. The old-name settings migration in `settings.py` can be removed once all robots have saved settings with the new names.

---

### 05.11.2026 – Always-on mapping, reliable localization, tap a goal, way back (Marlon, with Claude)
- **What:** The main map now grows during every drive, not only in slam mode. Lidar odometry runs permanently; a gate decides which scans are written; the map is saved and loaded at start; tapping a goal on `/map` plans a path through known free space; the way back follows the breadcrumbs. slam_toolbox stays as an optional first mapping ("Als Hauptkarte übernehmen" or "Verwerfen"). 9 commits on `feat/always-on-mapping`. Rationale: E14 in `docs/decisions.md`.
- **Why:** The map only grew in slam mode, we could not control which scan was written, and "localization confident" only meant "last match under 4 s ago".
- **Changes:**
  - `MapLocalization` hardened (`TrackResult`, constraint and degeneracy from scan normals, search stages, blind distance `NAV_BLIND_MAX_M`, confirm after search).
  - `navigation/integrate_gate.py`: FROZEN → CONFIRMING (10 strict matches, at least 3 s) → INTEGRATING, lag buffer of 2, carried detection by wheel vs lidar speed, max omega 0.15 rad/s.
  - `mapping/live_mapper.py` (log-odds grid, established-cell cap), `mapping/live_store.py` (npz/pgm/yaml/json, 3 backups, atomic writes), incremental distance field `OccupancyMap.derived`.
  - Start search: last pose 0.5 m → 1.5 m → home → whole map; new map from 3 still scans.
  - `navigation/breadcrumbs.py`, corridor planning, homing waits for the pose (turn ±30° after 5 s, give up after 20 s), fallback along the reversed trail.
  - `/map` page: always-on mapping card, mission buttons, trail and freeze overlay.
- **Result:**
  - Simulation test `test_live_sim`: at least 95 % of the wall cells within 10 cm over 6 seeds, pose error 2–4 cm.
  - Kidnap, dropout, carried and corridor cases leave the map bit-identical until the pose is confirmed.
  - Smoke test including mission start → return: 0.12–0.14 m off the start.
  - Slam adopt and discard checked in the mock.
- **Problems:**
  - The gate froze at every turn: the odometry consistency check had a fixed limit, but lidar odometry drifts a little with every turn. Fixed by allowing `LIVE_GATE_DRIFT_DEG_PER_RAD` of heading drift per radian turned.
  - Door turns wrote a rotation error into the map. Fixed with max omega 0.15 rad/s.
  - Corridor ends gave a false constraint. Fixed by taking the constraint from the scan normals.
  - A carried robot wrote scans before the lift detector fired. Fixed with the wheel speed vs lidar speed criterion.
  - Pose saving and autosave wrote the map's json at the same time and collided on the temp file. Fixed with a lock (read-modify-write) and unique temp file names.
  - `from_json` incremented the segment counter twice.
- **Open:** Test on the robot: CPU on `/diagnose` (estimate 10–20 % of one core), map grows when driving by hand, lifting the robot → frozen, restart finds the last pose, mission start → next room → return. No loop closure: large loops may freeze; do the first mapping with slam.
- **First robot test (05.11. afternoon):** the map hardly grew, it froze every few seconds. In the log about half the freezes said "bewegt sich ohne die Räder (getragen?)" at standstill: scans come 0.1–0.15 s apart and the A1's 1–2 cm matching jitter looked like 0.1 m/s; the carry check now measures both speeds over 0.4 s. The other half said "Abgleich widerspricht der Odometrie (5–10 cm)": now such a scan is only skipped, the map freezes only at twice the allowance (8 cm / 3° plus drift) or after 3 s of disagreement; coming back needs 5 strict matches over 1.5 s instead of 10 over 3 s. The spoken "Halt." on every stop is gone (the app sends stop whenever it loses focus).

---

## 3. Results and measurements

| Topic | Result | Source |
|---|---|---|
| USB resets | Triggered by CPU load spikes at 2.4 GHz; much rarer at 2.0 GHz | Sensor check §1 |
| Lidar | front = −90° raw, 7.5–7.8 Hz, 1080 points/scan | Sensor check §2 |
| Gyro z | Bias +0.389 rad/s, wrong sign | Sensor check §3 |
| Odometry | 0.27 m measured for 0.30 m target, sideways ~70 % | Sensor check §4 |
| Obstacle detection | Wall, chair, table and person correct in the simulation | `test_obstacles.py` |
| Autonomous driving | starts and drives on the real robot; floor test still pending | 28.10. |
| Gesture detection and behavior | 4 gestures, negative cases (jitter, hello wave, flicker, two people), Hailo decoding and drive logic: 45 checks passed | `Marlon/interaction/test_gestures.py` |
| Hailo pose on the Pi | C270, 10 fps, 1–2 people detected, no over-current message under load | Full check 28.10. |
| Stress test (battery full, without Orbbec) | At rest: 0 resets; driving: +160 over-current in 4.5 min, each 0–5 s after the start of a turn | 28.10. |
| Stress test (lab power supply, with Orbbec, **on pedestal**) | At rest and driving (210 s, same turns): 0 over-current, 0 disconnects | 29.10. |
| Stress test (battery 100 %/12.3 V, Orbbec and C270, **on pedestal**) | At rest and driving (210 s): 0 over-current, 0 disconnects, lidar and camera without reconnecting | 29.10. |
| Speech output | Queue, priority/silencing, fallback without Piper, behavior announcements: 17 checks passed; not yet heard on the speaker | `Marlon/interaction/test_voice.py` |
| Smooth driving and debounced gestures | Ramp, coasting, smoothing, walking/walking past without false triggers: 69 checks passed (29.10.) | `Marlon/interaction/test_gestures.py` |
| Voice commands "Robi" | Wake word, wake window, precedence, grammar, fault tolerance: 41 checks; gestures incl. drive loss: 76 checks (29.10.); not yet tested with the microphone | `Marlon/interaction/test_speech.py` |
| Using the map (simulation, apartment with 2 rooms) | Mapping drive until "Karte vollständig" in 7 min; pose in the map 1–6 cm / under 1.5° off the truth; drive into the neighboring room 5 cm off the goal; 29 checks passed (04.11.) | `Marlon/autonomous_driving/test_map.py` |

---

## 4. Problems and solutions

| Problem | Cause | Solution / status |
|---|---|---|
| Devices drop off USB | CPU load spike at 2.4 GHz, hub backfeed | `arm_freq=2000` permanently in config.txt (28.10.), drivers reconnect on their own. **Open:** hub without backfeed or cut VBUS |
| Collisions with the first explorer | Lidar mirrored and rotated by 90° | `LIDAR_ANGLE_OFFSET_DEG = -90`, hand test |
| Inaccurate turning | Gyro bias/sign, theta in the server ≈ 0 | Explorer turns by time; gyro correction open |
| Robot on the table | Lidar cannot see edges | Autonomous driving only on the floor |
| USB failures when hot | Pi above 80 °C (compiling), throttling to 1.5 GHz | Do not drive during builds; `check_hardware.py` shows temperature and throttling |
| Everything keeps crashing (camera, motor, lidar) | Hanging nvidia DKMS build that restarted with every `apt` → heat → USB overload as an endless loop | nvidia packages removed (28.10.), `check_hardware.py` warns about running builds and new over-current messages |
| Waving triggered "go away" | Shoulder width collapses when turning sideways, measures in shoulder widths become huge | Median of the shoulder width over the window (28.10.) |
| Typing in the "Sagen" field would have driven the robot | The app's keyboard control reacts to all keys, space bar = emergency stop | Keys in text fields are ignored (28.10.) |
| Wrong camera possible | `/dev/video*` numbers swap after USB resets, the Orbbec could become `video0` | Select the camera by name, `CAMERA_NAME = "C270"` (28.10.) |
| Server crashes (code -11, segfault in `depth.py`), three times in a row, each ~10 s after start | Orbbec firmware hung (opening fails, then "busy"); the code closed the handle of the failed open, which makes OpenNI2 crash. Reproduced on the Pi (04.11.) | Forget the handle after a failed open instead of closing it; hanging camera: unplug and replug the Orbbec |
| Resets while turning, despite a full battery and without Orbbec | Inrush current of all four motors under load pulls down the supply. On the pedestal (wheels free) 0 resets, with battery as with power supply (29.10.) | **Open:** test on the floor (power supply, then battery); if resets remain, separate 5 V supply (DC/DC) for Pi or hub, see `docs/hardware.md` |
| All USB devices briefly drop off when stopping the server | Pose thread still opened the Hailo during shutdown, whose startup briefly draws a lot of current | `_stop` check in `sensors/pose.py`; checked twice on the Pi, 0 messages (29.10.) |
| Driver connected to the lidar | After a dropout the lidar was briefly the only serial device | Board via `/dev/myserial`, lidar always excluded (28.10.) |
| USB endless loop even when standing still | Lidar motor started again immediately after every reconnect, wait time started at 1 s | Wait time keeps growing if it fails again shortly afterwards (`LIDAR_STABLE_S`, 28.10.) |
| Ctrl+C did not stop the server | uvicorn waited for the endless camera stream, motor stop did not run | `SHUTDOWN_TIMEOUT_S = 3` (28.10.) |
| USB resets and jerking while following | Hard jumps in the drive command → current spikes → voltage drop | Acceleration ramp in the watchdog, smoothed following with coasting (29.10.). **Open:** floor test on the Pi |
| Behavior switches while following without a signal | Swinging arms when walking look like come here/go away | Come here/go away only with a person standing still and facing the robot, and the robot standing (29.10.) |
| "Keine Verbindung zum Antrieb" when starting gesture control, interaction off immediately | The Hailo only started up when switching on (current spike → USB reset), and every drive loss ended the whole interaction | Open the Hailo at server start and keep it warm (`POSE_KEEP_WARM`); on drive loss only stop, refuse drive modes without the board (29.10.). **Open:** check on the Pi |
| „Musik stopp“ recognized as stop | The stop phrase is contained in the music command | Precedence of the voice commands, music off first (29.10.) |
| "Random crashes" also on the other board and on the power supply | Permanent USB overload even at idle from boot (all root ports), lidar does not register; not a code bug (29.10.) | Server loops more robust, crash recording on `/diagnose`. **Open:** narrow down the culprit (devices one by one, lidar cable, board USB), see `docs/hardware.md` |
| Server starts in the simulation on the robot | Board briefly gone during the start check (USB overload), `main.py` fell back to mock (29.10.) | With `Rosmaster_Lib` installed always start for real, only warn; the driver reconnects |
| Failures only traceable via SSH | Kernel log, vcgencmd and server log separate | Diagnostics page `/diagnose` with problems, timeline and log (29.10.) |
| Push from the laptop hangs | Git Credential Manager waits for a login window | User pushes themselves, the Pi only pulls (`git pull`) |
| MUKKE: mpv reports "Failed to recognize file format" | mpv's ytdl script does not find yt-dlp in the venv and passes the YouTube link on raw | The server resolves the link itself with the yt-dlp library, mpv only gets the direct audio track; YouTube errors appear in the window |
| Robot starts the interaction by itself | Vosk with a grammar turns music and conversations into allowed sentences („robbie hallo“) | Confidence and `[unk]` check; while music plays only stop and „Musik aus“ (03.11.) |
| Turning/distance by time inaccurate | Drive command overestimates by 10–30 %, mecanum slips, gyro unusable | Precise drive commands measure with the lidar (`navigation/maneuver.py`, 03.11.) |
| Drives only slowly and stuttering after following | A second open app keeps sending "vx=0" at idle between the commands of the device steering | Drive priority for the steering device, `DRIVE_PRIORITY_S` (03.11.) |
| Mapping starts, but no map and no feedback | slam_toolbox got neither `/scan` nor TF, because our server holds lidar and board; the start counted as successful after 0.3 s | The server feeds scans and odometry into the container, the status names the failing step (E8, 03.11.). **Open:** check on the Pi |
| No more voice commands after a crash | The microphone briefly dropped off USB, `stream.read()` hung forever without an error | Reading via callback with a deadline, reopening after 3 s without audio; switching back from the C270 to the USB microphone (03.11.) |

---

## 5. Open points and outlook

- **Narrow down the USB overload at idle** (29.10.): unplug all devices, plug them back in one by one (lidar last), swap the lidar cable, unplug the board USB; value "USB overloads per minute" on `/diagnose`. Make the journal on the Pi persistent.
- Floor test of the explorer with obstacles.
- Mapping on the Pi: is slam_toolbox present in the container `x3`? Then start it on `/map`, watch the mapping drive on the floor, save the map (E8, E9).
- Using the map on the robot (E12): mapping drive over two rooms until "Karte vollständig"; then check on `/map` whether the displayed pose is right (push the robot to known spots). "Hierhin fahren" through a real door, save the map, restart the server, load the map: does it find itself at the origin? Watch the CPU load with a loaded map on `/diagnose` (lidar odometry then runs permanently).
- **Stress test on the floor** (wheels under load): first on the power supply, then on the battery. On the pedestal there are 0 resets with both (29.10.), so it is open whether the power supply helps under load. If resets remain on the floor, the separate 5 V supply for Pi or hub is needed (see `docs/hardware.md`, "What to do on the robot").
- Repeat the stress test with the interaction switched on (Hailo pose and gestures under driving load).
- Interaction: check the Hailo pose on the Pi, driving test follow/come here on the floor; then stages 4–5 (speech, mind).
- Voice: Piper has been on the Pi since 03.11.; check speaker and volume in the room.
- Voice commands on the Pi: install `libportaudio2`, `pip install -e "robot[sprache]"` and the Vosk model, check `/sprache/status`, test „Robi, folgen“ on the floor; if Vosk recognizes "Robi" poorly, add variants to `SPEECH_WAKE_WORD_VARIANTS`.
- Sharing the speaker: MUKKE and voice block each other (exclusive ALSA device); shared mixer (dmix/PipeWire) open.
- Check whether the drive now stays connected when gestures are switched on (warm Hailo).
- Later: local LLM on the Hailo, recognizing faces.
- Check the diagnostics page on the Pi (kernel log permission, real battery and overload values).
- Check smooth driving on the floor: following without USB resets, ramp not too sluggish.
- Smoke test "homing starts" fails at times in the mock (also without the changes from 03.11.; once 19/19 on the afternoon of 03.11.): the drive before it only covers ~0.02 m, homing ends immediately. Investigate the cause.
- 90° turn test on the floor: check the heading from the board's turn rate against the lidar.
- Calibrate the depth camera on the floor (`/tiefe/test`, floor detected? left/right?) and test the mapping drive with chairs (E11).
- Following on the floor: walking away and coming back (must keep following), crossing with two people and briefly occluding. If it switches to others too quickly, increase `TARGET_SWITCH_S`.

---

## 6. Presentation outline

1. **Task:** a helper robot that moves around and deals with people.
2. **Platform:** X3, Pi 5, Hailo-10H, lidar, cameras (labeled photo).
3. **Architecture:** one server, one protocol, safety at the very bottom (diagram from `docs/architecture.md`).
4. **The way there:** ROS2 attempt → problems → our own Python server (rationale E1).
5. **Debugging as engineering work:**
   - USB resets: hypotheses, measurement, cause (table of clock frequencies).
   - Lidar orientation via hand test.
6. **Autonomous driving:** choosing directions, naming obstacles, live demo or video.
7. **Interaction:** gestures → behavior, stream of thoughts, "seems conscious" (demo).
8. **Lessons learned:**
   - measure first, then guess,
   - anchor safety at the bottom,
   - autonomous only on the floor.
9. **Outlook.**
