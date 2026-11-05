# Decisions

Short log of the architecture decisions: *what* was decided, *why*, and
*when to re-evaluate it*. Append new entries at the bottom.

---

## E1 — No ROS in the base system, pure Python

**Date:** 2026-10-27
**Status:** active

**Context.** The robot runs on a Raspberry Pi 5 with AI HAT+. The OS for it
is Raspberry Pi OS (Bookworm, Debian 12). ROS2 has no official packages for
this combination: Humble targets Ubuntu 22.04, Jazzy Ubuntu 24.04. ROS2 on
the Pi 5 therefore means Docker or a source build. The team already has
Python experience, but no ROS experience.

**Decision.** The base system uses Yahboom's `Rosmaster_Lib` directly from
Python. No ROS.

**Consequences.**
- *Good:* App control, ESP connection and box work right away, without
  anyone having to learn ROS first. The project also runs on a laptop (mock).
- *Bad:* For SLAM and navigation there is nothing in pure Python that comes
  close to Nav2. This part becomes more work.
- *Mitigated by:* The driver abstraction in `drive/base.py`. A later ROS2
  container can hide behind the same interface.

**Re-evaluate when:** autonomous driving begins and the lidar delivers data.

---

## E1a — Correction: ROS2 is available

**Date:** 2026-10-27
**Status:** active, supplements E1

**What changed.** The assumption in E1 that there is no usable ROS2 for the
Pi 5 was wrong. The robot has **ROS2 Foxy** under `/opt/ros/foxy`, plus a
set-up Yahboom workspace (`/root/yahboomcar_ros2_ws`) with:

- `yahboomcar_bringup` — drive node, talks to the same `Rosmaster_Lib`
- `sllidar_ros2` — working driver for the RPLIDAR under `/dev/rplidar`
- `yahboomcar_ctrl` — joystick control

**Consequence for the base system: none.** The decision from E1 remains
right for app control, camera and ESP — they run without ROS, are finished
and work.

**Consequence for autonomous driving: significant.** Lidar and odometry are
already solved there. Rewriting our own navigation in Python instead would
be considerably more work for a worse result.

**Important:** `yahboomcar_bringup` and our server **cannot run at the same
time**. Both open the same serial port, and two readers each get only
fragments. The ROS2 stack must be stopped before our server starts — and
vice versa.

**Decided in E4:** The lidar is read directly, without ROS2.

---

## E2 — WebSocket protocol instead of REST

**Date:** 2026-10-27
**Status:** active

**Context.** The app must control the robot smoothly and receive status and
alerts at the same time. Drive commands arrive several times per second.

**Decision.** One persistent WebSocket connection with JSON messages, shared
by app and ESP.

**Consequences.**
- *Good:* The robot can send on its own — required for emergency alerts.
  An open connection is also the natural basis for the watchdog: if it
  drops, the robot stops.
- *Bad:* Somewhat more effort than REST, and the app has to handle
  connection drops.

---

## E3 — Vital data does not trigger an autonomous reaction

**Date:** 2026-10-27
**Status:** active

**Context.** A pulse sensor on the ESP32 is inaccurate and prone to
interference. Wrong readings are the normal case, not the exception.

**Decision.** Abnormal vital data triggers an alert to a human. The system
does not assess a health condition and does not call emergency services on
its own.

**Consequences.** The emergency chain always needs a recipient — app,
message, beep. This has to be planned for when designing the alerting.

---

## E4 — Read the lidar directly, localize by scan matching

**Date:** 2026-10-27
**Status:** active, closes the open question from E1a

**Context.** The drive back to the start used dead reckoning and drifted
accordingly: about 30 cm over two and a half metres in the simulation, more
in reality. The RPLIDAR A1 was present but unused.

**Decision.** The lidar is read **directly from Python** via the package
`rplidar`, without ROS2. Localization uses **no SLAM**, but matches the
current scan against a single reference scan taken at the start
(Iterative Closest Point, `navigation/scan_matching.py`).

**Why no ROS2, although it is installed.** It would bring a second process
with its own claim on the port, a bridge between both worlds and an EOL
dependency (Foxy, unmaintained since 2023). Reading the lidar itself, by
contrast, is a manageable piece of code. For exactly one kind of sensor the
extra effort is not worth it.

**Why no SLAM.** Building a map and localizing in it is many times the
effort. For "back to the start" a comparison with *one* place -- the start
-- is enough.

**Division of labour between the two localization methods.** Dead
reckoning always runs and is never lost, but drifts. The lidar match does
not drift, but needs recognition. So: odometry brings the robot *close*,
and the lidar then pins it down precisely. Beyond `MATCH_RADIUS_M` no match
is attempted at all -- a false match would be worse than no correction.

**Measured result (simulation, 12 % wheel slip, 2.3 m distance driven):**

| | Error while driving | at standstill |
|---|---|---|
| dead reckoning only | 30.5 cm | 30.5 cm |
| with lidar match | 7.3 cm | 5.6 cm |

The remaining centimetres are the angular resolution of the A1: one degree
at four metres distance corresponds to seven centimetres.

**Consequences.**
- *Good:* Considerably more accurate, without a second process and without
  ROS knowledge. Obstacle checking during homing comes as a by-product.
- *Bad:* Only works within sight of the start. Driving through several
  rooms needs a map after all.
- *Open:* The depth channel of the Orbbec remains unused -- it would need
  the Orbbec SDK (not a `pip` package). It would be interesting for low
  obstacles that the lidar does not see at its height.

**Re-evaluate when:** the robot is supposed to move between rooms. Then
there is no way around a map.

---

## E5 — Gestures from the body pose on the Hailo, not from a hand model

**Date:** 2026-10-28
**Status:** decided, implementation in stage 2 (see [`interaction.md`](interaction.md))

**Context.** The robot should react to hand signals: follow, stop, come
here, go away. Common hand models (e.g. MediaPipe Hands) recognize finger
positions, but need the hand large in the image, i.e. at about one metre,
and run on the CPU. The CPU is a sensitive point on the X3, because load
spikes trigger USB resets (see [`hardware.md`](hardware.md)). The Pi has a
Hailo-10H, and a pose model ships with it (`yolov8s_pose_h10.hef`,
17 body keypoints).

**Decision.** Gestures are determined from the **body pose**: from the
position of wrists, elbows and shoulders and their movement over a short
time. The model runs on the Hailo. The gesture logic consists of pure
Python functions over the keypoints.

**Consequences.**
- *Good:* This works at 1–4 m, i.e. the distance from which one waves at a
  moving robot. The CPU stays free. The gesture logic can be tested without
  hardware, and the same pose provides the direction to the person for
  following.
- *Bad:* There are no finger gestures (thumbs up, showing a number), only
  arm gestures. The C270 is tilted only slightly upwards, so a standing
  person is only fully in the image from about 1.5 m.
- *Mitigated by:* The gestures are deliberately chosen to be large and
  unambiguous (arm over the head, waving, swiping). A gesture only counts
  after several stable frames.

**Re-evaluate when:** finer gestures are needed. Then a hand model is added
as a second stage, but only on the image crop of the hand that the pose
provides.

---

## E6 — Hybrid thinking; the language model never controls directly

**Date:** 2026-10-28
**Status:** decided, implementation in stage 5

**Context.** The robot should be able to hold a conversation and come
across as having an inner life. A large language model (Claude) sounds most
natural, but needs internet and costs money per call. Only small models run
locally on the Hailo-10H. Above all, language models can be wrong, and an
error must never lead to a drive movement.

**Decision.** Thinking runs **hybrid**, with one interface and three
stages: the Claude API when internet is available, then a local LLM on the
Hailo (later), and finally template sentences. The language model only
returns `gedanke` (thought), `antwort` (answer) and optionally an `absicht`
(intent) from a fixed list. Whether this intent is carried out is decided
by the mind ("Geist"). Drive commands are created exclusively in the
behavior classes and go through the watchdog. Gestures and speech commands
("stopp") work without a language model.

**Consequences.**
- *Good:* The conversation feels natural when internet is available, and
  without internet everything still works. A wrong or malicious model
  output cannot make the robot drive uncontrolled. "Stopp" never depends on
  a network connection.
- *Bad:* With templates the robot comes across as much plainer. The Claude
  API needs a key and a budget.
- *Mitigated by:* Thoughts are only generated on events or every ~20 s. The
  key lives only in the environment variable `ANTHROPIC_API_KEY` and never
  in the repo.

**Re-evaluate when:** the local LLM on the Hailo answers well enough that
the cloud becomes dispensable.

## E7 — Offline speech output with Piper, via aplay, in its own thread

**Date:** 2026-10-28
**Status:** implemented (`interaction/voice.py`), still to be checked on the robot

**Context.** A USB speaker is now attached to the Pi. The robot should
announce what it is doing, and later speak answers. The voice must be
German and intelligible and must also work without internet, just like the
reflexes. It must not slow down the server: a sentence takes one or two
seconds, the drive loop runs at 10 Hz.

**Decision.**
- Speech is produced with **Piper** (offline, voice `de_DE-thorsten-medium`). Output goes via **`aplay`** to the speaker, which is addressed by its ALSA name.
- Piper and aplay run as **subprocesses**, controlled from a dedicated thread with a small queue.
- If Piper is missing, espeak-ng speaks; if that is missing too, it stays in the log.
- Cloud voices were not chosen because they need internet. A Python audio library (`sounddevice`, `pyaudio`) was not chosen either: it would be an additional dependency, and `aplay` is there anyway.

**Consequences.**
- *Good:*
  - It works without internet and without new pip packages.
  - A crashed Piper or an unplugged speaker does not take the server down with it.
  - An emergency stop can simply end the current sentence: `kill` on the subprocess.
  - Thanks to the card name, output survives USB resets.
- *Bad:*
  - Piper has to be installed by hand, program plus model.
  - Every sentence restarts Piper. Loading the model takes a moment on the Pi 5 before the sound comes.
- *Mitigated by:* The sentences are short and rare. Identical sentences within a few seconds are swallowed. If the delay becomes annoying, Piper can run permanently and receive sentences via stdin. That would be a change in `voice.py` only.

**Re-evaluate when:** the delay in conversation (stage 5) becomes annoying,
or when speech recognition and output together need an audio framework.

---

## E8 — Map with slam_toolbox in the container, our server feeds in lidar and position

**Date:** 2026-11-03
**Status:** implemented (`mapping/scan_feed.py`), still to be checked on the robot

**Context.** The mapping module (`mapping/`) only started slam_toolbox in
the Docker container `x3`. But that is only the map builder: it needs lidar
scans on `/scan` and the position as TF (`odom → base_footprint`). In the
Yahboom stack these come from `sllidar_node` and `yahboomcar_bringup`.
Neither runs in our setup, and neither may, because our server has lidar
and board open (E1a, E4). slam_toolbox therefore waited forever, no map was
ever created, and nothing reported it.

**Decision.** Our server remains the only reader of lidar and board. It
passes scans and odometry on to a small ROS node in the container. The node
is started via `docker exec -i`, reads JSON lines from stdin and publishes
`/scan`, `/odom`, `/tf` and `/tf_static`. slam_toolbox runs unchanged with
its default configuration.

Other options were not chosen:
- *Stop our server for mapping and start the Yahboom stack:* then app,
  watchdog and dead-man switch are gone during mapping.
- *Build the map ourselves in Python:* less effort, but without loop
  closure. The map drifts with the odometry.

**Details that matter.**
- The A1 delivers slightly different points on every revolution, but
  slam_toolbox discards every scan whose beam count differs from the first.
  Therefore the scan is put onto a fixed grid (`MAP_SCAN_BEAMS` = 360), with
  no echo as `inf`.
- Each scan is sent together with the position at the time of measurement
  (`pose_at`), because slam_toolbox queries TF for exactly the timestamp of
  the scan.
- When the start is reset, the odometry jumps to (0, 0). This jump is
  masked out (`MAP_JUMP_M`), otherwise SLAM would enter the walls twice.
- If the node dies, the feed restarts it with increasing pauses. When our
  server ends, the node gets EOF and exits on its own.

**Consequences.**
- *Good:* The map is built with loop closure. Driving, watchdog and app
  keep running normally meanwhile.
- *Good:* The `/map` page now shows at which step things are stuck (lidar,
  forwarding, SLAM, map) and displays the SLAM log.
- *Bad:* We do depend on ROS2 Foxy (EOL) and the container `x3` after all.
  slam_toolbox must be installed there.
- *Superseded by E9:* The position for SLAM no longer comes from dead
  reckoning but from scan odometry.

**Re-evaluate when:** the map is to be used for navigation. Then the
position from SLAM (`map → odom`) belongs back in our server as well.

---

## E9 — Heading from the lidar, mapping drive like a robot vacuum

**Date:** 2026-11-03
**Status:** implemented (`navigation/scan_odometry.py`, `navigation/mapping_drive.py`), checked in simulation, not yet on the robot

**Context.** For E8, SLAM needed the position including heading. But dead
reckoning on the X3 does not deliver a usable rotation: the library's IMU
always reports 0 as the heading (sensor check 28.10.), and on the floor the
robot turns only about three quarters of what was commanded. For SLAM that
would mean: when it turns, the map believes it is standing still. In
addition, nobody drove the robot during mapping, and a map from a single
viewpoint has shadows everywhere.

**Decision.**
- **Scan odometry:** Each scan is aligned via ICP onto one taken shortly
  before (existing `scan_matching.py`). The wheels only provide the initial
  guess. The comparison is against a reference scan that is only renewed
  after 0.25 m or 10°; that way no noise accumulates at standstill. It only
  runs during mapping, because the matching costs CPU.
- **Mapping drive** as a second mode of the explorer:
  1. to the nearest wall,
  2. once around with the wall on the right,
  3. lanes parallel to this wall up to the opposite one.
  On the lanes it does not turn, but drives forward, slides sideways and
  drives backwards (mecanum). It keeps the direction of the lanes anchored
  to the walls: room walls are almost always at right angles, their
  direction can be read from every single scan and therefore does not
  drift.

Other options were not chosen:
- *Fix the IMU:* The library does not deliver the value; that is out of our
  hands.
- *Integrate the wheel turn rate:* slips by a quarter on every turn.
- *Get the position back from slam_toolbox:* For that we would have to read
  TF from the container. That is a second bridge in the other direction,
  and without a map (at the beginning) there is nothing there either.
- *Turn around like a robot vacuum:* Every rotation is the least accurate
  part of the motion on the X3 and the most delicate one for SLAM.

**Measurement (simulation, room 8 × 6 m with a pillar, 12 % wheel slip, 4 min drive):**

| | Error at the end |
|---|---|
| wheels only | 4.1 m, 49° |
| scan odometry | 0.3 m, 4.5° |

At first it took the pillar for the nearest wall and drove once around it.
This led to the island detection: a lap shorter than
`MAPPING_DRIVE_ISLAND_M` was not a wall. The lanes were initially skewed
because they followed the slow drift of the odometry. Since they are
aligned with the wall direction, they run parallel.

**Consequences.**
- *Good:* SLAM gets usable rotations. The lanes stay parallel without
  anything being integrated. All safety paths of the explorer apply
  unchanged.
- *Bad:* In rooms without clear right angles (sloped ceilings, curved
  walls) lane guidance falls back to odometry and drifts slowly.
- *Bad:* Whether the wall follow detects "einmal rundherum" (once around)
  depends on odometry. If it does not get close enough to the beginning,
  it only ends after `MAPPING_DRIVE_WALL_FOLLOW_MAX_S`.
- *Open:* A1 scans are slightly distorted while turning (one revolution
  takes 0.18 s). This does not exist in the simulation.

**Re-evaluate when:** homing needs the same rotation. Scan odometry could
then run permanently and replace dead reckoning, if the CPU allows it.

---

## E10 — Drive guard based on the footprint instead of a cone in the driving direction

**Date:** 2026-11-03
**Status:** implemented (`safety/drive_guard.py`), tested without the robot

**Context.** Collision avoidance did not work reliably. For manual control
it checked a cone of ±35° from the lidar centre:
- Close to the robot such a cone is narrow and misses the table leg in
  front of the wheel. Far away it is wide and stops in front of walls the
  robot would pass by.
- It only blocked hard at the limit. With 0.22 s command latency the robot
  then keeps rolling.
- Turning was always allowed, although the corners swing out.
- It accepted scans up to 2 s old.

The autonomous drives each checked only individual directions. Nobody
checked a mix of forward, sideways and turning as a whole.

**Decision.** One common check for all movements. For the command, it
predicts where the rectangular footprint of the robot will be in the next
seconds, including on an arc and while turning. It has to brake as soon as
a lidar point falls below the margin *and* gets closer. The speed is then
reduced until the stopping distance (reaction time plus scan age plus
braking distance) fits into free space. All components of the command are
scaled equally, so the path stays the same, only slower.
- Always applies to explorer, mapping drive, homing, interaction and the
  precise drive commands (`maneuver.py`): in the watchdog on
  `submit(..., source="autonom")`. For turns by a fixed angle,
  `DriveGuard.turn_free()` checks the footprint over the whole angle in
  advance; this replaces the 30 cm circle around the lidar that blocked
  for no reason in the floor test.
- Applies to manual control when the obstacle stop is on. The `distance`
  slider keeps its meaning (forward from the lidar centre).
- Autonomous drives do not drive without a fresh scan
  (`GUARD_SCAN_MAX_S`). In that case the guard lets manual control pass:
  whoever drives by hand can see the robot.

**Measured** (simulation, drive with 0.22 s latency, full throttle towards
a wall): at 0.2, 0.5 and 0.8 m/s it stops 6 cm before the edge every time,
which is exactly the margin `GUARD_MARGIN_M` (6 cm at the time). The mapping
drive runs through with the guard just as it does without.

**Merged with `AutonomSchutz` (03.11.).** In parallel, a second layer had
been created in `main`: `AutonomSchutz` limited the speed by the braking
distance to the nearest point in a strip in the driving direction and let
every rotation pass. Two layers in the watchdog would have checked twice
and differently. One remains:
- The interface from `main`: `source="autonom"` (plus a tighter speed
  limit, battery throttling, recorder), can be switched off with
  `AUTONOMOUS_GUARD_ACTIVE`.
- Inside it the drive guard does the checking, because it sees arcs and
  swinging corners. Taken over from `AutonomSchutz`: age of the oldest point
  of the scan instead of its timestamp, preparing the points once per scan,
  counted interventions logged with debouncing.
- One footprint (`ROBOT_*`) for drive guard and arc planning (`AVOID_*`).
  The drive guard margin (4 cm) is deliberately below that of the planner
  (5 cm): the safety layer is the inner limit. If it were stricter than the
  planner, it would choke off the planner's arcs.

**Consequences.**
- *Good:* One place for everything. It brakes softly instead of hard and
  does not hold the robot in place next to obstacles.
- *Bad:* It only sees what the lidar sees at its height. Low things
  (cables, chair skids, bed frame) and overhanging ones (table top) remain
  invisible.
- *Bad:* The footprint dimensions are partly estimated (`ROBOT_REAR_M`).
  They have to be measured on the robot.

**Re-evaluate when:** the Orbbec depth is integrated. Its points can go into
the same check.

**Addendum 04.11.:** Depth is integrated (E11). Its points go into the same
check via the obstacle memory (`GUARD_WITH_DEPTH`).

---

## E11 — Depth camera for avoidance, progress watch against getting stuck

**Date:** 2026-11-04
**Status:** implemented (`sensors/depth.py`, `navigation/obstacle_memory.py`,
`navigation/progress.py`), not yet tested on the robot while driving

**Context.** During mapping and exploring, the robot got stuck on chair legs
and similarly low or thin things. The lidar only sees its single plane. For
the drive logic the front was free, the command went out, but the robot did
not move. The mapping drive had no detection for this (the explorer's stuck
detection only ran in random driving), so it stood still, and the map did
not grow.

**Decision.** Two independent aids, both for all autonomous modes:
1. **See what the lidar does not see.** The depth channel of the Astra
   delivers points lying 4–40 cm above the floor it finds itself. Because
   the Astra only measures from 0.6 m, an obstacle memory keeps the points
   in the pose frame (scan odometry during mapping, odometry otherwise) and
   returns them in the robot frame. If the camera sees free floor at a spot
   again, the point is dropped. Explorer, mapping drive and drive guard
   simply get the points added to the lidar points, without logic of their
   own.
2. **Notice that it is not making progress.** Every 0.5 s the progress watch
   compares the commanded motion of the last 2 s (what the watchdog actually
   output after ramp and drive guard) with the motion measured by the lidar
   (ICP between two scans). If less than 30 % arrives, the robot counts as
   stuck. The watch then places a phantom obstacle in front of the
   footprint in the driving direction and pushes the robot free away from
   it.

**Why this way.**
- *Lidar instead of wheels as the measurement:* Odometry does not see
  spinning or blocked wheels; it calculates from the motors. Matching the
  lidar scans looks at the walls.
- *Commanded after the watchdog:* If the drive guard brakes to zero in
  front of a wall, the expectation is zero as well. Standing in front of an
  obstacle is not being stuck.
- *Double match:* ICP is computed once starting from "not moved" and once
  from "as commanded". If the results diverge (corridor, bare wall), the
  watch does not judge. A false "stuck" would trigger unstick manoeuvres in
  the middle of open space.
- *Depth not in the map:* The map (E8) stays a pure lidar map. Depth points
  have a different height and more noise; in slam_toolbox they would
  disturb the matching.
- *Camera only while driving:* The projector draws a lot of current from
  the hub, which loses USB devices on voltage drops (hardware.md, USB
  resets).
- *Own ctypes binding to OpenNI2:* The pip package does not run under
  Python 3.13; a ROS node would contradict "no ROS in the server".

**Consequences.**
- *Good:* Chair legs are seen from ~1.5 m and driven around. What nobody
  sees, the watch detects after ~2 s and avoids the spot for 30 s.
- *Bad:* Below 0.6 m the camera is blind. Anything newly appearing there is
  only detected by the progress watch.
- *Bad:* In random mode the memory uses wheel odometry. If the robot slips
  while stuck, remembered points shift somewhat.
- *Bad:* Without a detected floor (dark floor) it becomes less accurate and
  more cautious (only from 9 cm, only up to 1.3 m).

**Re-evaluate when:** the watch is often wrong in operation (then adjust
`PROGRESS_SHARE`/`PROGRESS_WINDOW_S`) or a sensor for the close range
(ultrasound, bumper) is added.

---

## E12 — Using the map: own localization in the map, path planning, mapping drive room by room

**Date:** 2026-11-04
**Status:** implemented (`mapping/occ_map.py`, `navigation/map_localization.py`,
`navigation/path_planning.py`, `navigation/path_follower.py`,
`api/map_control.py`), checked in tests and simulation, not yet on the robot

**Context.** Since E8/E9 a map is being built, but nobody used it: the
robot did not know where it was in it and could not drive anywhere. The
mapping drive stopped after the first room, and even before that at every
spot where it could not continue ("keine Wand in Sicht", stuck, corner). It
also ended as soon as the `/map` page closed briefly (phone screen off) or
the drive was briefly gone due to a USB reset. E8 had named as the reason
to re-evaluate: "when the map is to be used for navigation".

**Decision.**
1. **Localization in the map in Python, not slam_toolbox in localization
   mode.** Several times per second the lidar scan is aligned onto a
   precomputed distance field of the map (Gauss-Newton, points pulled
   towards the nearest wall). Only a correction between scan odometry and
   map is kept. After loading a map, the pose is searched for at the
   map's origin (grid search, then refined); if the robot cannot find
   itself, one shows it on the map.
2. **Path planning with A\*** on a 10 cm grid: only known free space with
   0.25 m clearance to the wall is drivable, proximity to walls costs extra
   (paths through the middle of doors).
3. **Goal drive as part of homing** (`Homing.start(target=...)`). With a
   map, `home` also drives the planned path.
4. **Mapping drive room by room:** After the room (or when it cannot
   continue there) it drives to the next frontier (free spot at the edge of
   the unknown), until no reachable one is left, then back.
5. **Persistence:** Without a watcher, only stop after 20 s; after a drive
   failure, resume within 60 s; wait 30 s for the lidar.
6. **Map in the simulation in Python** (`mapping/simulation.py`), so that
   all of this runs at the desk without Docker.

Other options were not chosen:
- *Let slam_toolbox localize and get `map → odom` from the container:*
  a second bridge in the other direction, and every drive with a map would
  depend on Docker and ROS2 Foxy (EOL). This way only mapping needs the
  container.
- *Nav2:* would additionally need odometry and drive in ROS, i.e. exactly
  the duplication that E1/E4 avoid.
- *Localization with a particle filter (AMCL):* more robust with
  ambiguity, but considerably more computing time on the Pi. Because scan
  odometry is already good (E9), a single match per scan is enough here.
- *Goal drive as a separate class:* Every existing cancellation (emergency
  stop, manual control, connection lost, another drive starts) would have
  had to be added individually. As homing with a goal they all apply
  automatically.
- *Mapping drive with frontiers only, without wall follow and lanes:* The
  wall follow is tested and brings the room boundaries into the map early
  and completely; the frontiers only take care of what is missing after
  that.

**Details that matter.**
- The slam_toolbox map starts where mapping starts, with the same pose as
  scan odometry. That is why the MappingManager clears the localization
  before every mapping run, and scan odometry starts anew. It now runs as
  long as mapping is in progress **or** a map is loaded (users register and
  unregister).
- Points in the unknown part of the map do not count in the match -- during
  mapping that is the space the robot is only just seeing.
- The search only accepts a clear hit (80 % of the points on walls) and
  rejects when a clearly different pose fits almost as well. An empty
  rectangular room fits just as well rotated by 180°; without furniture the
  robot then only finds itself with a hint (heading or "hier stehe ich").
  Shown in the test setup: two identical empty rooms were
  indistinguishable, with a cupboard and sofa unambiguous.
- During mapping, driving on to the next frontier is a separate phase of
  the mapping drive; planning happens in a thread, and the drive loop
  (20 Hz) pauses meanwhile.

**Consequences.**
- *Good:* The map can be used immediately after mapping: show the pose,
  drive somewhere, go home via the map -- also into other rooms, which
  homing without a map never could (E4).
- *Good:* The mapping drive ends with "Karte vollständig" instead of after
  the first obstacle.
- *Bad:* With a loaded map scan odometry runs permanently and costs CPU
  (E9 had named exactly that as a condition for re-evaluation).
- *Bad:* A single match per scan can misjudge in long, uniform corridors;
  then the pose becomes "unsicher" (uncertain), and goal drives stop.
- *Open:* Check everything on the robot: localization with a real A1 map,
  planning through real doors, driving on in an apartment.

**Re-evaluate when:** localization on the robot often becomes "unsicher"
(then particle filter or `map → odom` from slam_toolbox), or the CPU load
with a loaded map leads to dropouts.

---

## E13 — Code in English, user interface in German

**Date:** 2026-11-05
**Status:** implemented (whole repository)

**Context.** Until now, code, comments, log messages and docs were German
(rule in AGENTS.md). The team decided to switch the code to English; the
people using the robot and the web app are German speakers.

**Decision.**
- Code (identifiers, module and file names, comments, docstrings, log
  messages), tests, docs and commit messages are in English.
- Everything users see stays German: web app, `/map` page, alert texts,
  `"text"`/`"grund"`/`"hinweis"` fields, diagnostics texts, settings labels,
  the robot's spoken sentences and the speech commands (German Vosk model).
- Protocol/API names stay unchanged even where they are German (message
  types, JSON field names and values, HTTP URLs such as `/interaktion/status`):
  they are the contract with the web app and the ESP (E2).

**Consequences.**
- *Good:* Code reads in one language; English names fit the libraries and
  are easier for outside readers and tools.
- *Bad:* Mixed languages remain at the edges: German protocol fields in
  English code (`msg.get("winkel")`), German UI strings next to English
  comments. That is intended -- the contract and the UI did not change.
- *Bad:* Every branch based on the old German names conflicts on merge.
- Settings saved under old names are migrated on load (`settings._OLD_NAMES`).

**Re-evaluate when:** the protocol gets a new major version -- then field
names could become English too, with app and ESP updated together.

---

## E14 — Always-on mapping with our own Python mapper; slam_toolbox only optional for a first map

**Date:** 2026-11-05
**Status:** implemented (`navigation/integrate_gate.py`, `mapping/live_mapper.py`,
`mapping/live_store.py`, `navigation/breadcrumbs.py`, `navigation/map_localization.py`,
`api/map_control.py`), checked in tests and simulation, not yet on the robot

**Context.** E8 made slam_toolbox the only map source, E9 kept scan odometry
off outside mapping, and E12 used the finished map for localization and goal
drives. In practice the map only grew while slam mode ran, and we had no
control over which scan was written into it. Localization "confident" meant
only "last match less than 4 s ago", not "the pose is really right".

**Relation to E8/E9/E12.** E14 re-evaluates three clauses. E8: "the map comes
from slam_toolbox" now holds only for an optional first map. E9: "scan
odometry only runs while mapping or with a loaded map" (named there as a
condition for re-evaluation) becomes "always on". E12: its localization is kept
and hardened; a single match per scan is still enough, but a scan only counts
as good when the new quality criteria say so.

**Decision.**
1. **Lidar odometry always on** (user `"dauer"`), so the pose exists at any time.
2. **`MapLocalization` hardened:** `TrackResult` per match; `constraint()` and
   degeneracy from the scan normals (a corridor end does not constrain the
   direction along the corridor); search stages; blind distance
   `NAV_BLIND_MAX_M` (how far odometry alone may carry the pose); a found
   pose is confirmed after a search.
3. **`IntegrateGate`** decides whether a scan may be written. Hard and soft
   criteria; states FROZEN → CONFIRMING (10 strict matches, at least 3 s) →
   INTEGRATING. A lag buffer of 2 scans means the last scan before a failure
   never lands in the map. "Carried" is also detected by comparing wheel speed
   with lidar speed (before the lift detector fires). While turning faster
   than 0.15 rad/s nothing is written.
4. **`LiveMapper`:** own log-odds grid. Established cells are capped in how far
   a session may change them, so a bad session can weaken a wall at most to
   unknown, never erase it. `OccupancyMap.derived` keeps the distance field up
   to date incrementally instead of recomputing it.
5. **`LiveStore`:** npz (float16) plus pgm/yaml plus json, `_aktiv.json`, atomic
   writes, 3 backups; a broken npz is renamed `.broken` and the pgm is used
   instead.
6. **Start:** search at the last pose with 0.5 m, then 1.5 m, then home, then
   the whole map. With no map, a new one is founded from 3 still scans.
7. **Way back:** breadcrumbs and corridor planning along the trail. Homing
   waits for the pose: stop, relocalize, turn ±30° after 5 s, resume after 3
   good matches, give up after 20 s. Fallback: along the reversed trail.
8. **Tapped goal** must lie in known free space. **Loading** a saved map makes
   it the main map, which keeps growing.
9. **slam_toolbox** is optional for a first map: pause, then "Als Hauptkarte
   übernehmen" (backup first; frames are related through the odometry
   correction remembered at slam start) or "Verwerfen".

Other options were not chosen:
- *Keep slam_toolbox as the only mapper:* no say over which scans are written,
  and every map depends on Docker and ROS2 Foxy (EOL).
- *Write every scan that matches:* a match is not proof of a good pose; turns
  and carrying wrote drift and rotation errors (see the project log).

**Consequences.**
- *Good:* the map grows during every drive; a bad moment freezes the map
  instead of damaging it.
- *Good:* localization says "unsicher" for the right reasons.
- *Bad:* no loop closure. Large loops may freeze instead of producing double
  walls. Recommendation: do the first mapping with slam, then adopt the map.
- *Bad:* CPU, estimated 10–20 % of one core (to be measured on the robot).
- Switches in `config.py`: `LIVE_MAP_ENABLED`, `LIVE_LOCALIZATION_ALWAYS`, `LIVE_MAP_*`.
- *Open:* everything on the robot (CPU, growth by hand drive, lift → frozen,
  restart finds the last pose, mission start → next room → return).

**Re-evaluate when:** the map often stays frozen in normal rooms (then loop
closure or a looser gate), or the CPU load causes dropouts.
