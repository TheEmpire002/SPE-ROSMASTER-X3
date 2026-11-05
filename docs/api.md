# Robot API

Everything you need to connect your own user interface, autonomy block or
script to the robot, without reading the server code.

The robot offers two channels:

| Channel | Address | Used for |
|---|---|---|
| **WebSocket** | `ws://<Pi-IP>:8000/ws` | Driving, homing, settings; live state from the robot |
| **HTTP** | `http://<Pi-IP>:8000/...` | Camera image, lidar scan, diagnostics |

`hostname -I` on the Pi shows the IP. Port 8000 is the default
(`--port` at startup, `PORT` in `config.py`).

There is no authentication: anyone on the same Wi-Fi can control the robot.
HTTP endpoints allow all origins via CORS, so a browser app does not have to
be served by the robot.

---

## Rules you need to know

These five points determine the behavior. Overlook them and you will wonder
why the robot keeps stopping.

1. **Repeat drive commands continuously.** If no `drive` arrives for
   `COMMAND_TIMEOUT_S` (0.5 s), the robot stops on its own. This is the
   dead man's switch for Wi-Fi dropouts and crashed apps. The web app
   sends at **10 Hz**, and that is also the recommendation. A single
   `drive` therefore moves the robot only for a moment.
   Multiple clients: whoever is currently steering (a `drive` other than 0)
   has priority. Zeros from other connections only count again
   `DRIVE_PRIORITY_S` (0.5 s) after that client's last real command;
   otherwise the idle zeros of a second open app would slow it down.
   `stop` always works, from any connection.
2. **Last client gone means stop.** When the last WebSocket connection
   closes, the robot stops and cancels any homing in progress.
3. **The server limits, not the client.** Every command is clamped to the
   values from the `limits` message. Physically the chassis only manages
   about 1 m/s anyway.
4. **Collision avoidance can intervene.** When it is enabled, the drive
   guard checks whether the robot's footprint would touch anything with
   this command. It then brakes smoothly so that the robot, accounting for
   reaction time and braking distance, stops in time, and finally blocks
   the motion. Driving away and turning without any corner hitting
   something remain allowed. `distance` is measured forward from the lidar
   center; all around, the robot's edge keeps `distance - ROBOT_FRONT_M`
   of clearance. What the robot actually executes is in the `state`
   message, not necessarily what you sent. Autonomous drives (explore,
   mapping drive, homing, interaction, precise drive commands) are always
   checked by the drive guard, with `GUARD_MARGIN_M`, and they do not move
   without a fresh lidar scan.
5. **All clients are equal.** There is no control ownership. If two clients
   drive at the same time, the most recent command wins. State and
   messages go to everyone.

---

## Conventions

The same everywhere, for drive commands, position and lidar:

| Quantity | Unit | Direction |
|---|---|---|
| `vx` | m/s | forward positive |
| `vy` | m/s | **left** positive (mecanum: strafe sideways without turning) |
| `omega` | rad/s | counterclockwise positive (= turn left) |
| Lidar angle | degrees, 0–360 | 0° = front (camera), 90° = left, 180° = rear, 270° = right |
| Lidar distance | meters | only 0.15–8 m, everything else is already discarded |
| Position `x`, `y` | meters | relative to the start point: `x` in the heading at start, `y` to the left of it |
| Position `winkel` | **degrees** | counterclockwise, 0 = heading at start |

The start point is where the server was started or where `set_home` was
last sent.

---

## Quick start

### Python: drive one meter forward, then query the lidar

```python
import asyncio, json, urllib.request
import websockets  # pip install websockets

ROBOT = "192.168.1.50:8000"   # enter the Pi's IP

async def main():
    async with websockets.connect(f"ws://{ROBOT}/ws") as ws:
        limits = json.loads(await ws.recv())          # always the first message
        print("Limits:", limits)

        # 0.2 m/s for 5 s = 1 m. Repeat at 10 Hz, otherwise the watchdog kicks in.
        for _ in range(50):
            await ws.send(json.dumps({"type": "drive", "vx": 0.2, "vy": 0, "omega": 0}))
            await asyncio.sleep(0.1)
        await ws.send(json.dumps({"type": "stop"}))

asyncio.run(main())

scan = json.load(urllib.request.urlopen(f"http://{ROBOT}/lidar/scan"))
front = [m for deg, m in scan["punkte"] if deg < 15 or deg > 345]
print("Nearest obstacle in front:", min(front) if front else "nothing", "m")
```

### Browser (JavaScript)

```js
const ROBOT = "192.168.1.50:8000";
const ws = new WebSocket(`ws://${ROBOT}/ws`);

ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  if (msg.type === "state") console.log("Position", msg.pose);
  if (msg.type === "alert") console.log(msg.level, msg.text);
};

// Driving: send at a steady rate for as long as the robot should move.
let motion = { vx: 0, vy: 0, omega: 0 };
setInterval(() => {
  if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "drive", ...motion }));
}, 100);

// Camera image: simply embed as <img>.
// <img src="http://192.168.1.50:8000/camera/stream">
```

### Command line

```bash
curl http://<IP>:8000/lidar/scan
curl http://<IP>:8000/lidar/status
curl -o image.jpg http://<IP>:8000/camera/snapshot
```

FastAPI provides an automatically generated overview of all HTTP endpoints
(without WebSocket) at `http://<IP>:8000/docs`.

---

## WebSocket `/ws`

All messages are JSON objects with a `type` field. The binding contract is
in
[`robot/src/rosmaster_assist/protocol.py`](../robot/src/rosmaster_assist/protocol.py),
the handling in `_handle()` in
[`api/server.py`](../robot/src/rosmaster_assist/api/server.py).

### Sequence on connect

Right after connecting, the robot sends on its own, in this order:

1. `limits`
2. `collision` (current setting)
3. then `state` every 0.5 s, plus `alert`, `vitals` and `collision` as needed

### To the robot

| `type` | Fields | Effect | Response |
|---|---|---|---|
| `drive` | `vx`, `vy`, `omega` (numbers, missing = 0) | Drive. See rules 1, 3, 4. Cancels a running homing as soon as a value is ≠ 0; zero commands are ignored during homing. | none; `error` for invalid values |
| `stop` | – | Stop immediately; cancel homing, autonomous driving and interaction. | none |
| `explore` | – | Start autonomous driving (see below). Needs the lidar; cancels homing. | `alert` to all; without lidar `alert` (warning) |
| `explore_stop` | – | End autonomous driving and stop. | `alert` to all |
| `move` | `richtung`: `"vor"` (forward) / `"zurueck"` (back) / `"links"` (left) / `"rechts"` (right), `meter` (0 < m ≤ 5) | Drive exactly this far; left/right strafes sideways (see "Precise drive commands in detail"). Needs the lidar; cancels homing, autonomous driving and interaction. Cancel with `stop` or a drive input ≠ 0. | `alert` to all, at the end an `alert` with the result (warning if not achieved); otherwise `alert` (warning) |
| `turn` | `grad` (+ = left, − = right, magnitude 1–720) | Turn on the spot by exactly this angle (degrees). Otherwise like `move`. | like `move` |
| `home` | – | Drive back to the start point autonomously (see below). With a map and a confident pose in it: along a planned path to the map's origin. | `alert` to all |
| `home_cancel` | – | Cancel homing and stop. | `alert` to all |
| `goto` | `x`, `y` (m in the map frame), `winkel` (degrees, optional: heading at the goal) | Drive to this point of the map, along a planned path around walls (new 04.11., see "Using the map" under Mapping). Needs a map and a confident pose in it; cancels other drives. Technically runs as homing with a goal: `state.homing` is `true`; `stop`, `home_cancel` and drive inputs end it. | `alert` to all; without map/pose `alert` (warning) with reason |
| `goto_cancel` | – | Cancel the goal drive and stop (like `home_cancel`). | – |
| `map_pose` | `x`, `y` (m), `winkel` (degrees, optional) | "I am here": localization searches for the robot's pose around this point (narrower with `winkel`). For the case where it does not find itself after loading a map. | `alert` (info); without map `alert` (warning) |
| `map_load` | `name` (saved map) | Load a saved map to drive in it. Localization then searches for the pose at the map's origin. Not during mapping. | `alert` (info/warning) |
| `interaction_start` | – | Interaction mode on: the robot waits for hand signals (see "Interaction in detail"). Needs lidar and pose detection; cancels homing and autonomous driving. | `alert` to all; otherwise `alert` (warning) |
| `interaction_stop` | – | Interaction mode off and stop. | `alert` to all |
| `follow` | `ziel_id` (optional, number) | "Follow me" without a gesture. Turns interaction mode on if needed. It follows the person in view, the largest one if there are several. `ziel_id` (the `id` from `/interaktion/pose`) makes that person the starting target; the app no longer sends it since 04.11., an unknown ID is ignored. | `alert` to all |
| `follow_pause` | – | Stop, the target stays remembered (new 03.11.). Only in `folgen`, `herkommen`, `weggehen`, `sucht`. | – |
| `follow_resume` | – | After `follow_pause`, continue to the same target. | – |
| `follow_stop` | – | End following, stop, release the target; interaction mode stays on. | `alert` to all |
| `record_save` | `notiz` (string, optional, at most 200 characters; a note) | Save the last minute of the drive recorder to a file (new 03.11., see "Drive recorder" under HTTP). Does not drive. | `alert` (info) „Fahrt gespeichert: <name>“ (drive saved); when disabled `alert` (warning) |
| `say` | `text` (string, at most 300 characters), `vorrang` (bool, optional) | Speak through the loudspeaker (see "Voice in detail"). `vorrang: true` (priority) interrupts the current sentence and clears the queue. | none; without `text` `error`, without loudspeaker/synthesis `alert` (warning) |
| `say_stop` | – | Go silent immediately, clear the queue. | none |
| `speech` | `text` (string) | Treat the text as if it had been spoken into the microphone (see "Speech commands in detail"). For trying things out without a microphone and in `--mock`. | `alert` „Sprachbefehl: …“ to all if a command was recognized; otherwise `alert` (info) „Kein Befehl erkannt …“ (no command recognized); without `text` `error` |
| `set_home` | – | The current position becomes the new start point (0, 0, 0°), the lidar reference is re-recorded. With a map and a confident pose, the position also becomes the map's origin (and is remembered in its file). | `alert` to all |
| `collision` | `enabled` (bool, optional), `distance` (m, optional) | Collision avoidance on/off, minimum distance. `distance` is clamped to 0.15–1.5 m. | `collision` to all |
| `health_check` | `x`, `y` (m in the odometry frame like homing, not the map frame; optional: both or neither) | Healthbot check (new 05.11., see "Healthbot" under HTTP): drive to the spot, look at the person, address them, nudge them if they do not answer, call for help. Without `x`/`y` it checks a person near the robot. **Drives and nudges, only on the floor.** Cancels homing, autonomous driving, interaction and precise drive commands. Any drive input ≠ 0, `stop`, `move`/`turn`, `explore`, `home`, `interaction_start` or a voice „Robi stopp“ cancels it. | `alert` to all; otherwise `alert` (warning) with reason |
| `health_stop` | – | Cancel the check, stop alarm music, stop. | none |
| `health_wache` | `an` (bool) | Watch: look out for a lying person on its own and then start a check. Does not drive off as long as nobody is seen. | none; `alert` (warning) if not possible |
| `beep` | `ms` (int, default 100) | Buzzer. | none |
| `vitals` | `bpm` (number/null), `motion` (bool/null) | Readings from the ESP32, relayed to all. | `vitals` to all |
| `box` | `action`: `"open"` / `"close"` | Not built yet. | `alert` „Box ist noch nicht angebaut“ (box not attached yet) |
| `ping` | – | Check the connection. | `pong` |

An unknown `type` returns `{"type": "error", "text": "Unbekannter Nachrichtentyp: …"}`.

### From the robot

**`limits`**: once on connect.

```json
{"type": "limits", "max_vx": 1.0, "max_vy": 1.0, "max_omega": 3.0}
```

These are the limits for manual control (`MAX_SPEED_*`). Following, exploring
and homing are additionally limited to `AUTONOMOUS_MAX_SPEED` (0.35 m/s) and
`AUTONOMOUS_MAX_OMEGA` (1.2 rad/s), halved when the battery is low.
In addition, the robot slows autonomous driving so that it can stop before
the nearest lidar point in the direction of travel; without fresh lidar data
it only turns.

**`state`**: every 0.5 s to all clients.

```json
{
  "type": "state",
  "driver": "rosmaster",
  "connected": true,
  "homing": false,
  "vx": 0.2, "vy": 0.0, "omega": 0.0,
  "pose": {"x": 1.234, "y": -0.05, "winkel": 12.5, "gemessen": true},
  "collision": {"enabled": false, "distance": 0.35, "verfuegbar": true, "blockiert": false},
  "autonomie": {"aktiv": false, "verfuegbar": true, "zustand": null, "...": "siehe unten"},
  "interaktion": {"aktiv": false, "verfuegbar": true, "modus": null, "...": "siehe unten"},
  "stimme": {"verfuegbar": true, "spricht": false, "text": "Ich bin bereit."},
  "sprache": {"verfuegbar": true, "geweckt": false, "gehoert": "robi folgen", "befehl": "folgen"},
  "karte": {"name": "wohnung", "live": false, "pose": {"x": 2.1, "y": -0.4, "winkel": 87.0},
            "sicher": true, "text": "Lage in der Karte bekannt",
            "fahrt": {"aktiv": true, "art": "ziel", "ziel": [3.5, -1.0], "rest_m": 2.4, "grund": null}}
}
```

| Field | Meaning |
|---|---|
| `driver` | `"rosmaster"` = real motors, `"mock"` = simulation |
| `connected` | Connection to the motor board is up. `false` after a USB dropout; the server reconnects on its own. |
| `homing` | Homing is running |
| `vx`, `vy`, `omega` | what the robot is **currently executing**, after limiting and collision avoidance. Since the acceleration ramp in the watchdog (29.10.), the value rises gradually after a drive command (`RAMP_*` in `config.py`, can be turned off and adjusted under Settings → „Fahren & Motoren“); a stop takes effect immediately. |
| `pose` | estimated position, see Conventions. `gemessen: false` (not measured) means: computed from the commands only, considerably less accurate. |
| `collision.verfuegbar` | whether a lidar is present that can see obstacles (available) |
| `collision.blockiert` | collision avoidance is currently holding back a motion (blocked) |
| `autonomie` | autonomous driving and detected obstacles, see "Autonomous driving in detail" |
| `interaktion` | interaction mode (hand signals, following), see "Interaction in detail". Missing on older servers. |
| `stimme` | voice output: `verfuegbar` (sound comes out), `spricht` (speaking a sentence right now), `text` (last sentence spoken, or `null`). See "Voice in detail". Missing on older servers. |
| `sprache` | speech commands: `verfuegbar` (microphone and model ready), `geweckt` (awake: „Robi“ was just said on its own, the next sentence needs no name), `gehoert` (last sentence understood, or `null`), `befehl` (recognized command, for a few seconds, otherwise `null`), `pause` (why the microphone is deliberately closed right now, e.g. „Mapping läuft“; then `verfuegbar` is `false`, otherwise `null`). See "Speech commands in detail". Missing on older servers. |
| `karte` | Only present when a map exists (new 04.11.): `name` (saved map, `null` for the live map during mapping), `live`, `pose` (pose **in the map frame**, not to be confused with `pose` above in the odometry frame; `null` while unknown), `sicher` (confident; `goto` only works then), `text`, `fahrt` (running or last map-based drive: `art` (kind) `"heim"` (home) / `"ziel"` (goal), `ziel` (goal), `rest_m` (remaining distance), `grund` (reason)). See "Using the map" under Mapping. |
| `healthbot` | Healthbot short form (new 05.11.): `aktiv`, `zustand` (state: `bereit`, `anfahrt`, `gesamtbild`, `ansprechen`, `anstupsen`, `nachfrage`, `hilfe`, `ansprechbar`, `unauffaellig`, `keine_person`, `abgebrochen`), `schritt` (current step as text), `alarm`, `wache` (watch on). Everything else in `GET /healthbot/status`. Missing on older servers. |

**`collision`**: on connect and after every change, fields as in
`state.collision`.

**`alert`**: messages for humans.

```json
{"type": "alert", "level": "warning", "text": "Hindernis in 0.31 m -- Fahrt gesperrt"}
```

`level` is `"info"`, `"warning"` or `"emergency"`. `emergency` is intended
for emergencies but is currently not triggered anywhere. The text is meant
for display, not for parsing. To react to an event, read the fields in
`state` instead (for example, `homing` switches to `false` when homing
ends).

**`vitals`**: relayed ESP data: `{"type": "vitals", "bpm": 72, "motion": true}`

**`error`**: response to an unusable message: `{"type": "error", "text": "…"}`

**`pong`**: response to `ping`.

### Homing in detail

`home` drives straight to the start point. Thanks to the mecanum wheels the
robot does not need to turn for this. The values are in `config.py`:

- speed `HOME_SPEED` (0.25 m/s), slower from 0.4 m before the goal
- arrived when closer than `HOME_TOLERANCE_M` (0.12 m)
- gives up after `HOME_TIMEOUT_S` (60 s)
- **drives around obstacles** (new 03.11., F3): if something is in the way
  to the start point, it slides diagonally past it (up to 75° off the goal
  direction, centered in doorways) and then returns to the connecting line.
  Without a map, i.e. around individual obstacles, not through mazes. If
  there is no room anywhere for 3 s, it stops: „Hindernis in 0.40 m, noch
  3.00 m bis zum Startpunkt“ or „kein Weg frei, …“
- **if it is lifted**, it aborts („hochgehoben -- Position stimmt nicht
  mehr“): dead reckoning knows nothing about being carried
- without a fresh lidar scan it stops, as before, as soon as something is
  closer than 0.45 m in the direction of travel
- ends with an `alert` „Heimfahrt: …“ and `state.homing = false`

**With a map (new 04.11.):** if a map is loaded and `state.karte.sicher`
is set, `home` no longer drives in a straight line but along a planned path
to the **map's origin** (where mapping began, or where `set_home` was last
pressed) -- through doors, into other rooms. The same drive is behind
`goto`. Differences from the drive without a map:
- speed `NAV_DRIVE_SPEED` (0.25 m/s), turns slowly into the direction of
  travel, at the goal into the requested heading if one was given
- arrived at `NAV_GOAL_TOLERANCE_M` (0.15 m)
- if something the map does not know is in the way, it slides past as
  above; if that does not work for `NAV_BLOCKED_S` (3 s), it marks the spot
  as a wall for 60 s and plans around it (at most 5 times)
- if it loses its pose in the map for more than 4 s, or is lifted, it stops
  (`alert` „… Lage in der Karte verloren“)
- time limit based on path length (12 s per meter, at least 60 s)
- ends with `alert` „Heimfahrt: …“ or „Fahrt zum Ziel: …“

### Autonomous driving in detail

`explore` starts test 1: the robot drives randomly through the room and
avoids everything the lidar sees. Values in `config.py` (`EXPLORE_*`):

- straight ahead at `EXPLORE_SPEED` (0.2 m/s), slower from 0.9 m before an
  obstacle; slides sideways away from nearby walls
- "free" means a 40 cm wide corridor (`EXPLORE_HALF_WIDTH_M`), not a cone
  -- so it does not squeeze into gaps narrower than itself
- below `EXPLORE_STOP_M` (0.4 m) in front: stop and turn toward the most
  open direction. The turn is controlled by the measured angle from the
  board's turn rate (since 03.11.); it stops `EXPLORE_TURN_LEAD_DEG`
  (8°) before the target because it coasts. A time limit
  (`EXPLORE_TURN_TIME_FACTOR`) catches hangs. Only without a measurement
  does it turn by time as before (`EXPLORE_TURN_FACTOR`). Afterwards it
  waits for a fresh scan instead of continuing on the old picture.
- points closer than `EXPLORE_NEAR_M`: it first waits `EXPLORE_NEAR_WAIT_S`
  (3 s), because often a person has stepped up. If it is still not clear,
  it **unsticks itself** (state `befreit sich`): slowly sideways or
  backward toward the most open direction, without turning, so the rear
  does not swing out.
- **Stuck detection:** if it is commanded to drive but the board reports
  hardly any speed, or the lidar picture all around does not change over
  `EXPLORE_STUCK_S` (1.5 s), it unsticks itself the same way.
- **Depth camera (since 04.11.):** during every autonomous drive the depth
  channel of the Orbbec Astra runs as well. Anything standing between 4 and
  40 cm above the floor (chair legs, table edges, boxes below the lidar
  plane) becomes obstacle points. The Astra only measures from ~0.6 m.
  Therefore an **obstacle memory** remembers the points and updates them
  with the robot's own motion until it has passed (`DEPTH_MEMORY_S`, 30 s).
  Avoidance, mapping drive and drive guard see them like lidar points.
  Without the camera or in the simulation, only this addition is missing.
- **Progress watch (since 04.11., both modes):** every 0.5 s it compares
  the motion the watchdog sent to the motors over the last ~2 s with the
  motion actually driven according to lidar matching (ICP). If less than
  30 % arrived (`PROGRESS_SHARE`), it is stuck. It then places a **phantom
  obstacle** at that spot (expires after `PROGRESS_PHANTOM_S`, 30 s) and
  unsticks itself away from the obstacle. If the surroundings are ambiguous
  for matching (long corridor), it makes no judgment.
- Only after `EXPLORE_UNSTICK_MAX` (3) consecutive failed unstick attempts
  does the drive end with „festgefahren“ (stuck). During mapping the
  attempts are reset when none was needed for `PROGRESS_RECOVERED_S` (15 s).
- **Detours in arcs (since 03.11., F2):** in front of an obstacle it
  simulates possible arcs (turn rate, some sideways sliding) a little more
  than a second ahead and takes the best one where its outline keeps
  clearance everywhere. Afterwards it steers back onto the old course. Only
  when no safe arc fits does it stop and turn.
- **Doors and gaps:** when turning, it picks the middle of an open area
  instead of a direction just past the frame.
- **Short-term memory:** points that slip out of the field of view while
  driving past (e.g. a door frame beside it) are kept for a few seconds
  (`AVOID_MEMORY_S`).
- every 4–10 s it turns off randomly so it does not always follow the same
  track -- but not while something is closer than `AVOID_NARROW_M` (0.45 m)
  (door, narrow passage): first get through
- **Lifted** (state `hochgehoben`): if the lidar picture all around changes
  at once (tilted, carried), it stops. Once it stands still again, it starts
  over and forgets memory and course. No alarm; the drive continues.
- if a **person** stands up to 1 m in front of it, it waits
  `EXPLORE_PERSON_WAIT_S` (2 s) before avoiding
- if the lidar delivers nothing for more than 3 s, the drive ends
- any drive input ≠ 0, `stop`, `home`, `set_home` or the disconnection of
  the last client ends it. Zero commands from the app are ignored.
- ends with `alert` „Autonomes Fahren beendet: …“ and `autonomie.aktiv = false`

Obstacles are detected even without a running drive (up to 3 m) and roughly
classified: `Wand` (wall), `Stuhl` (chair: thin legs close together),
`Tisch` (table: thin legs 0.6–1.8 m apart), `Mensch` (person: two thick legs
or a camera hit), `Bein (Mensch?)` (leg, person?), `Bein (Stuhl/Tisch?)`
(leg, chair/table?), `Objekt` (object). This is a rule of thumb on a single
lidar plane, not object recognition.

If OpenCV is installed, the person camera (Logitech C270, see
[Camera](#camera)) looks for people during the drive (HOG detector, 2 frames/s). Lidar obstacles in their direction are then labeled
`Mensch` with `quelle: "Kamera+Lidar"`. Without OpenCV everything else keeps working.

`state.autonomie`:

```json
{
  "aktiv": true,
  "verfuegbar": true,
  "zustand": "faehrt",
  "text": "frei: 1.8 m",
  "frei_vorn": 1.83,
  "dauer_s": 42,
  "ausweichen": 5,
  "menschen": 1,
  "befreien": 0,
  "tiefe": 14,
  "festgefahren": 0,
  "grund": null,
  "hindernisse": [
    {"art": "Stuhl", "winkel": 31.5, "abstand": 0.84, "breite": 0.41,
     "x": 0.71, "y": 0.44, "quelle": "Lidar"}
  ],
  "personen": [{"x1": 0.41, "y1": 0.1, "x2": 0.62, "y2": 0.95, "score": 1.2}],
  "personenerkennung": {"verfuegbar": true, "aktiv": true, "fehler": null,
                        "kamera": "personen", "letzte_erkennung_vor_s": 0.3}
}
```

| Field | Meaning |
|---|---|
| `aktiv` | an autonomous drive is running (active) |
| `verfuegbar` | lidar present, `explore` possible (available) |
| `modus` | `zufall` (random, `explore`) or `karte` (mapping drive, see [Mapping](#mapping-slam)); `null` when off |
| `kartierfahrt` | only in mode `karte`: `{"schritt", "text", "randweg_m", "bahnen"}` (step, text, wall-follow distance in m, lanes) |
| `zustand` | state: `faehrt` (driving), `dreht` (turning), `wartet` (waiting), `wartet auf Mensch` (waiting for person), `wartet auf Lidar` (waiting for lidar), `befreit sich` (unsticking, new 03.11.), `hochgehoben` (lifted, new 03.11., F2), `kartiert` (mapping); `null` when off |
| `text` | explanation for display |
| `frei_vorn` | free distance in the corridor in front of the robot (m) |
| `dauer_s` | duration of the current drive (s) |
| `ausweichen`, `menschen`, `befreien` | counters for this drive: avoidance maneuvers, people encountered, unstick attempts (`befreien` new 03.11., missing on older servers) |
| `tiefe` | points in the obstacle memory (depth camera and phantoms); `null` without depth camera (new 04.11.) |
| `festgefahren` | how often the progress watch detected being stuck during this drive (new 04.11.) |
| `grund` | why the last drive ended (reason) |
| `hindernisse` | the 12 nearest obstacles, nearest first. `art` (kind), `winkel` in degrees as for the lidar, `abstand` (distance, m), `breite` (width, m), `x`/`y` center in robot coordinates (m), `quelle` (source) |
| `personen` | person boxes in the person camera image, relative to the image size (0..1) |
| `personenerkennung` | whether camera person detection works; `fehler` (error) e.g. „opencv fehlt“; `kamera`: which camera image the boxes refer to (name as in `/cameras`) |

### Interaction in detail

`interaction_start` turns interaction mode on. The robot then stands still
and waits for a hand signal in front of the camera (mode `bereit`, ready).
The body keypoints are computed by the Hailo chip (YOLOv8-Pose); it only
works while the mode is on. **Use only on the floor.**

**Soft start:** after switching on, only pose detection warms up for
`INTERACTION_STARTUP_S` (5 s); the robot stands still and stays silent
(`text`: „startet …“). Then it says the start sentence (e.g.
„Jetzt folge ich dir.“) and only starts moving once the sentence is
finished. Commands during this time (follow, stop …) apply afterwards.
Reason: pose detection, speaking and motors at the same time made the USB
power supply collapse.

| Hand signal | How | Mode afterwards |
|---|---|---|
| Follow | hold one arm above your head (1 s) | `folgen` (following): stays `FOLLOW_DISTANCE_M` (1.0 m) behind the person |
| Stop | hold both hands above your shoulders (0.4 s) | `bereit` (ready): stops immediately |
| Come here | wave up and down in front of your body | `herkommen` (come here): approaches to `COME_HERE_DISTANCE_M` (0.7 m), then `bereit` |
| Go away | swipe your hand sideways | `weggehen` (go away): `GO_AWAY_DISTANCE_M` (1.0 m) backward, then `bereit` |

- The direction to the person comes from the camera, the distance from the
  lidar (the person's cluster in that direction; a chair in front does not
  count).
- If it loses the person while following for longer than `FOLLOW_LOST_S`
  (2 s), it searches by turning (mode `sucht`, searching): turn for
  `FOLLOW_SEARCH_TURN_S`, hold still for `FOLLOW_SEARCH_PAUSE_S` (the image
  blurs while turning). It searches until it sees someone again; with
  `FOLLOW_SEARCH_S` > 0 it gives up after that many seconds.
- Safety as for autonomous driving: forward only with a free driving lane,
  backward only with free space behind, without fresh lidar data it stands
  still. `herkommen` and `weggehen` end after at most
  `INTERACTION_BEHAVIOR_MAX_S` (20 s).
- Any drive input ≠ 0, `stop`, `explore`, `home`, `set_home` or the
  disconnection of the last client ends the mode. Zero commands from the app
  are ignored.
- Recognized gestures and mode changes arrive as `alert` (info), the end as
  „Interaktion beendet: …“.

`state.interaktion`:

```json
{
  "aktiv": true,
  "verfuegbar": true,
  "modus": "folgen",
  "text": "hält Abstand (1.20 m)",
  "person": {"winkel": -8.5, "abstand": 1.2},
  "personen": 1,
  "geste": "folgen",
  "grund": null,
  "pausiert": false,
  "ziel": {"zustand": "following", "id": 3, "sicherheit": 1.0, "grund": null}
}
```

| Field | Meaning |
|---|---|
| `aktiv` | interaction mode is on (active) |
| `verfuegbar` | lidar and pose detection present, `interaction_start` possible (available) |
| `modus` | `bereit` (ready), `folgen` (following), `herkommen` (coming here), `weggehen` (going away), `sucht` (searching); `null` when off |
| `text` | explanation for display; while following, coming here and going away, new (03.11., F3) e.g. „umfährt Hindernis (links, 0.62 m)“ (detouring around obstacle) or „hochgehoben -- wartet, bis er wieder steht“ (lifted, waiting until it stands again) |
| `person` | tracked person: `winkel` in degrees as for the lidar, `abstand` (distance) in m (`null` if the lidar sees nothing there); `null` without a person |
| `personen` | how many people the camera currently sees |
| `geste` | last recognized gesture (`folgen`, `stopp`, `herkommen`, `weggehen`), for a few seconds, otherwise `null` |
| `grund` | why the mode last ended (reason); „hochgehoben -- angehalten“ (new 03.11., F3) if it was lifted while driving -- afterwards `bereit` and the announcement „Ich wurde hochgehoben. Ich bleibe stehen.“ |
| `pausiert` | `true` after `follow_pause` (paused): stands still, mode and target are kept. Any mode change (including the stop gesture) ends the pause. Missing on older servers. |
| `ziel` | whom it follows (target lock 03.11., simplified 04.11.): `zustand` (state) is `idle`, `acquiring_target`, `following`, `target_temporarily_lost` or `searching_for_target` (`safe_stop` only from servers before 04.11.). `id` is the internal track ID (changes when the person was briefly out of view; do not display it), `sicherheit` (confidence) is 1 while following, `grund` says why it stopped (e.g. „Person nicht wiedergefunden“). Details in `docs/interaction.md`. Missing on older servers. |

### Voice in detail

A USB loudspeaker is attached to the robot. The robot briefly says on its
own what it is doing: „Ich bin bereit.“ at startup, „Ich folge dir.“,
„Ich komme.“, „Da bin ich.“, „Wo bist du?“, „Okay, ich halte an.“, and
„Halt.“ on `stop`. With `say` any text can be read aloud.

**Volume:** one slider for everything, `VOLUME` (0–100 %) on the settings
page (group „Ton & Lautstärke“, `POST /einstellungen`). The loudspeaker's
mixer is fixed at 100 %; volume is reduced in software. The music volume
(`/mukke/volume`) is a fraction of it.

- Speech is synthesized with Piper (offline, German voice). If Piper is
  missing, espeak-ng speaks; if that is missing too, the text only goes to
  the log (`verfuegbar: false`). In the simulation (`--mock`) it is only
  logged, but `spricht` behaves as for real.
- Sentences go into a small queue (`VOICE_MAX_QUEUE`, 3). When it is full,
  the oldest sentence is dropped. The same sentence within
  `VOICE_REPEAT_S` (4 s) is swallowed.
- `spricht` stays true for `VOICE_TAIL_S` (0.4 s) after each sentence.
  Speech recognition does not listen during this time.

### Precise drive commands in detail

`move` and `turn` drive a specific distance or turn by a specific angle,
and really do so: the measurement is done with the lidar, not by time or
wheel revolutions (`navigation/maneuver.py`).

- At the start it records a lidar scan while standing still. Each further
  scan is matched against it (ICP); this yields how far it has really moved
  and turned. Slipping mecanum wheels and the unusable gyro therefore do not
  matter. Wheels and drive command only estimate the time between two
  scans.
- **Driving straight:** besides the remaining distance, the controller also
  keeps the lateral deviation and the heading at zero. It corrects
  deviations by strafing sideways (mecanum) instead of steering.
- **At the end it measures again while standing still** (while driving, a
  lidar revolution is distorted) and corrects up to
  `MANEUVER_MAX_CORRECTIONS` times. Target: `MANEUVER_TOLERANCE_M` (3 cm)
  and `MANEUVER_TOLERANCE_DEG` (2°). In the simulation with deliberately
  slipping drive it was off by 1–2 cm and 1–2°.
- Speed `MANEUVER_SPEED` (0.2 m/s), turning `MANEUVER_TURN` (0.6 rad/s).
- **Safety:** it stops when something comes closer than `EXPLORE_STOP_M`
  (0.4 m) in the direction of travel and when the lidar fails to recognize
  the surroundings for `MANEUVER_BLIND_S` (2 s). It only turns if, over the
  whole angle, no corner of the footprint comes closer than
  `GUARD_MARGIN_M` to anything (checked beforehand). In addition, every
  command passes through the drive guard; if it blocks for
  `MANEUVER_BLOCKED_S` (1.5 s) in a row, it gives up and says why.
- `GET /manoever/status`: `aktiv` (active), `art` (kind), `wert` (value),
  `text` (progress), `grund` (how the last one ended), `erfolgreich`
  (successful), `gefahren_m` (distance driven), `gedreht_grad` (angle
  turned; according to the lidar, relative to the start), `abgleiche`
  (matches), `fehlschlaege` (failures).

### Speech commands in detail

The robot listens through the USB microphone (Vosk, offline). Commands only
count with the wake word **„Robi“** in the same sentence or up to 5 s
(`SPEECH_WAKE_WINDOW_S`) after „Robi“ was said on its own. „Stopp“,
„Halt“ and „Bleib stehen“ also count without the wake word, as does „La da da“.
If Vosk is unsure (word confidence below `SPEECH_MIN_CONFIDENCE` or mostly
unknown words) or it hears more than wake word plus phrase (word salad such
as „robin hallo hause zu weg ruby“), only stop counts. While MUKKE is
playing, every command except stop needs the wake word and a higher
confidence (`SPEECH_MIN_CONFIDENCE_MUSIC`). Otherwise music and
conversations would turn into commands. To „Robi“ on its own it answers
after 1 s with „Ja?“ (`SPEECH_WAKE_REPLY`), unless a command arrived by
then. The 5 s for the command only start counting down once it has
finished speaking.

| Command | Examples (German phrases) | Acts like |
|---|---|---|
| `folgen` | „Robi, folgen“, „Robi, folge mir“, „Robi, komm mit“ | `follow` |
| `herkommen` | „Robi, komm her“ | `interaction_start`, then mode `herkommen` |
| `weggehen` | „Robi, geh weg“, „Robi, zurück“ | `interaction_start`, then mode `weggehen` |
| `stopp` | „Stopp“, „Halt“ | during interaction: mode `bereit` (interaction stays on); otherwise `stop` |
| `nach_hause` | „Robi, fahr nach Hause“ | `home` |
| `erkunden` | „Robi, erkunde“ | `explore` |
| `musik_aus` | „Robi, Musik aus“, „Robi, Ruhe“ | `POST /mukke/stop` |
| `ladada` | „La da da“ (without wake word) | `POST /mukke/play` with `SPEECH_LADADA_URL` |
| `beatbox` | „Robi, Beatbox“ | `POST /mukke/play` with `SPEECH_BEATBOX_URL` |
| `start` | „Hallo Robi“ | `interaction_start` |
| `aus` | „Robi, gute Nacht“ | `interaction_stop`, cancel homing/explorer |
| `manoever` | „Robi, fahr zwei Meter vor“, „Robi, einen halben Meter nach links“, „Robi, dreh dich um neunzig Grad nach rechts“, „Robi, dreh dich um“ (180°) | `move` or `turn`; it announces what it is doing |

- Commands are executed through the same paths as the app's commands, with
  the same checks and messages. If a command is refused, the warning goes to
  all clients and the robot says „Das geht gerade nicht.“.
- Every recognized command produces `alert` (info) „Sprachbefehl: <Text>“.
- Without Vosk, model or microphone, recognition is off
  (`sprache.verfuegbar: false`); everything else keeps running.
  `GET /sprache/status` gives the reason.

---

## HTTP

### Camera

The robot operates all cameras it finds, each with its own image. Each one
has a name:

| Name | Camera | Used for |
|---|---|---|
| `orientierung` | Orbbec, color image | Orientation, driving. **Default camera.** |
| `personen` | Logitech C270 | Person detection |
| `usb1`, `usb2`, … | further USB cameras | nothing fixed |
| `csi` | camera on the ribbon cable, if present | nothing fixed |

`orientierung` and `personen` are always listed, even if the camera is
currently missing (then `geoeffnet: false`, not opened). The robot keeps
looking for them in the background. Assignment is by USB ID
(`config.CAMERA_ROLES`).

| Endpoint | Returns |
|---|---|
| `GET /cameras` | all cameras with state and addresses, see below |
| `GET /camera/<name>/stream` | MJPEG stream (`multipart/x-mixed-replace`) of this camera. Usable directly in `<img src>`, up to `CAMERA_FPS` frames/s (default 15, at most 30, adjustable on the settings page). |
| `GET /camera/<name>/snapshot` | a single frame (usually `image/jpeg`). `503` as long as none is available yet. Good for image processing: just fetch at the desired rate. |
| `GET /camera/<name>/status` | diagnostics for this camera, see below |
| `GET /camera/stream`, `/camera/snapshot`, `/camera/status` | the same for the default camera (`config.CAMERA_DEFAULT`) |

An unknown name returns `404`. Default resolution 640×480. If the camera is
disabled (`CAMERA_ENABLED = False`), `stream` and `snapshot` respond with
`503`. In simulation mode each camera delivers a moving test image as
`image/png`, with its own bar color.

`/cameras`:

```json
{
  "standard": "orientierung",
  "personen": "personen",
  "tiefe": [
    {"name": "tiefe", "kamera": "Tiefe", "basis": "/tiefe"},
    {"name": "tiefe-ueber", "kamera": "Tiefe + Farbbild", "basis": "/tiefe/ueberlagert"}
  ],
  "kameras": [
    {"name": "orientierung", "kamera": "Orbbec (Farbbild)", "geoeffnet": true,
     "stream": "/camera/orientierung/stream",
     "snapshot": "/camera/orientierung/snapshot", "...": "wie /camera/status"},
    {"name": "personen", "kamera": "Logitech C270", "geoeffnet": false,
     "letzter_fehler": "Öffnen fehlgeschlagen: nicht angeschlossen (USB 046d)",
     "...": "..."}
  ]
}
```

`standard` is the default camera. `personen` is the name of the camera
whose image the person boxes from `state.autonomie.personen` refer to.
`tiefe` (new 04.11.) lists the views of the depth camera, empty without it.
Image at `<basis>/stream` or `<basis>/snapshot`, see [Depth camera](#depth-camera).

`/camera/status` or `/camera/<name>/status`:

```json
{
  "kamera": "Orbbec (Farbbild)", "geoeffnet": true, "thread_laeuft": true,
  "bilder_gesamt": 5321, "letztes_bild_vor_s": 0.1,
  "bildgroesse_bytes": 41230, "format": "image/jpeg",
  "letzter_fehler": null, "neu_geoeffnet": 0,
  "name": "orientierung", "geraet": "/dev/video0", "fov_deg": 66.0,
  "fps_ist": 14.8, "fps_soll": 15
}
```

`geraet` is the device currently in use. It can change after a USB dropout.
`fov_deg` is the horizontal field of view in degrees. `fps_soll` is the
configured frame rate (`CAMERA_FPS`), `fps_ist` the rate actually achieved.
The cameras manage at most 30 fps, the Logitech only about 15 in low light.
Other fields: `thread_laeuft` (capture thread running), `bilder_gesamt`
(total frames), `letztes_bild_vor_s` (seconds since the last frame),
`bildgroesse_bytes` (frame size), `letzter_fehler` (last error),
`neu_geoeffnet` (number of reopens).

### Lidar

**`GET /lidar/scan`**: the last complete revolution.

```json
{"punkte": [[0.0, 1.52], [1.1, 1.53], [358.9, 1.51]], "alter_s": 0.12}
```

- `punkte` (points): list of `[angle_deg, distance_m]`, conventions see
  above. A few hundred points per revolution; the lidar spins about 5.5
  times per second. Angle rounded to 0.1°, distance to 1 cm.
- `alter_s`: age of the scan in seconds.
- If no current scan is available (older than 3 s or lidar faulty), you get
  `{"punkte": [], "hinweis": "Lidar: <Grund>"}` instead (`hinweis` = hint
  with the reason).

Converting to x/y (meters, robot at the center, x forward, y left):

```python
import math
xy = [(m * math.cos(math.radians(g)), m * math.sin(math.radians(g))) for g, m in scan["punkte"]]
```

**`GET /lidar/status`**: diagnostics and localization state.

```json
{
  "lidar": "RPLIDAR A1", "geoeffnet": true, "thread_laeuft": true,
  "umdrehungen": 1204, "punkte_letzte_runde": 312, "letzte_runde_vor_s": 0.1,
  "letzter_fehler": null, "neu_verbunden": 0,
  "ortung": {
    "lidar_vorhanden": true, "referenz_aufgenommen": true, "referenzpunkte": 318,
    "korrekturen": 85, "verworfen": 3,
    "letztes_residuum_m": 0.031, "letzte_korrektur_m": 0.004
  }
}
```

`ortung` is the localization against the start-point scan.
`letztes_residuum_m` (last residual) says how well the current scan fits
the reference from the start point. Below 0.05 is good.

### Depth camera

The camera only runs during autonomous drives (power on the USB hub). It is
integrated via OpenNI2; for setup see [hardware.md](hardware.md#depth-camera-orbbec-astra).

**`GET /tiefe/status`**:

```json
{
  "kamera": {
    "verfuegbar": true, "verbunden": true, "aktiv_fuer": ["erkunder"],
    "bilder_pro_s": 7.6, "hindernisse": 12, "frei_zellen": 230, "naechstes": [0.71, 0.12],
    "boden_erkannt": true, "kamera_hoehe_m": 0.214, "neigung_grad": -1.5,
    "bildwinkel_grad": 57.9, "fehler": null
  },
  "hinweis": null,
  "gedaechtnis": {"punkte": 14, "phantome": 0, "rahmen": "lidar"},
  "fortschritt_aktiv": true
}
```

- `kamera` is `null` in the simulation or without OpenNI2 (the reason is
  then in `hinweis`).
- `boden_erkannt` (floor detected): a floor plane was found in the image.
  Then the heights are correct, and `kamera_hoehe_m`/`neigung_grad` show the
  measured mounting height and tilt. Without a floor it computes more
  cautiously: 5 cm more minimum height, only up to 1.3 m.
- `gedaechtnis.rahmen`: the frame in which the obstacle memory stores the
  points, `lidar` (lidar odometry, during mapping) or `rad` (wheel
  odometry).
- `fortschritt_aktiv`: whether the progress watch is running.

**`POST /tiefe/test`**: switches the camera on for 20 s to check floor
detection and mounting pose while standing still.

**Viewing the depth image** (new 04.11., needs OpenCV on the Pi):

| Endpoint | Returns |
|---|---|
| `GET /tiefe/stream` | MJPEG: depth image in color |
| `GET /tiefe/snapshot` | a single frame of it (JPEG) |
| `GET /tiefe/ueberlagert/stream` | the same, semi-transparent over the Orbbec's color image (`DEPTH_OVERLAY_CAMERA`) |
| `GET /tiefe/ueberlagert/snapshot` | a single frame of it |

Red is near, blue far (`DEPTH_VIEW_NEAR_M` to `DEPTH_VIEW_FAR_M`). Black
means no measurement: closer than ~0.6 m, too far, reflective. Magenta
counts as an obstacle. In addition there is the distance at the image center
(crosshair), the nearest obstacle and whether the floor is detected. Anyone
watching switches the camera on; after closing, it turns off again. The
overlay only roughly lines up, because the cameras have different fields of
view and sit 2–3 cm apart. It can be adjusted with `DEPTH_OVERLAY_SHIFT_PX`
and `DEPTH_OVERLAY_ZOOM`. In the app both views can be selected in the
camera switcher, but not under „Alle“ (all).

### Autonomous driving

| Endpoint | Returns |
|---|---|
| `GET /autonomie/status` | the same as `state.autonomie`, for checking without WebSocket |

### Settings

The settings from `config.py` that can be changed at runtime (speed, camera
frame rate, lidar, autonomy …). In the web app the gear icon at the top
right opens them as a dialog; as a separate page they are available at
`/einstellungen`.

| Endpoint | Returns |
|---|---|
| `GET /settings` | all settings grouped in blocks, with value, default and limits |
| `POST /settings` | Body `{"CAMERA_FPS": 20, "EXPLORE_SPEED": 0.15}`: sets the values. All or nothing: if one is invalid, you get `400` with `{"fehler": "…"}` (error), and nothing is changed. |
| `POST /settings/reset` | Body `{"keys": ["CAMERA_FPS"]}` for individual values, `{}` for all: back to the values from `config.py` |

`GET /settings`:

```json
{
  "gruppen": [
    {"id": "kamera", "titel": "Kameras", "text": "…", "einstellungen": [
      {"key": "CAMERA_FPS", "label": "Bilder pro Sekunde", "typ": "int",
       "wert": 15, "standard": 15, "min": 1, "max": 30, "schritt": 1,
       "einheit": "fps", "optionen": null, "hilfe": "…", "neustart": false}
    ]}
  ]
}
```

- `gruppen` are the blocks (groups), each with `id`, `titel` (title),
  `text` and `einstellungen` (settings). Per setting: `key`, `label`,
  `typ` (type), `wert` (value), `standard` (default), `min`, `max`,
  `schritt` (step), `einheit` (unit), `optionen` (options), `hilfe` (help
  text), `neustart` (restart).
- `typ`: `bool`, `int`, `float` or `auswahl` (choice; the allowed values
  are then in `optionen`).
- `neustart: true`: only takes effect after a server restart.
- The response to both `POST`s is the same list, plus `geaendert` (which
  values actually changed) and `neustart_noetig` (restart needed).

Changes take effect immediately and are saved in `robot/settings.json`.
The file only contains deviations from the defaults. Delete it and you are
back to the values from `config.py`. If the speed limits change, the robot
sends a new `limits` to all clients. It also reports every change as an
`alert` (`info`).

Changes are only possible from a page loaded from the robot itself: CORS
only allows `GET` for foreign pages.

### Interaction

| Endpoint | Returns |
|---|---|
| `GET /interaktion/status` | the same as `state.interaktion`, plus `pose`: `verfuegbar` (available), `simuliert` (simulated), `aktiv` (active), `fehler` (error), `fps`, `bilder` (frames), `letzte_erkennung_vor_s` (seconds since last detection), `kamera` (which camera image the keypoints refer to, name as in `/cameras`) |
| `GET /interaktion/pose` | `personen`: detected people, largest first, each with `box` [x1, y1, x2, y2] and `kp` (17 × [x, y, confidence], COCO order), relative to the image size (0..1), plus `score`, `winkel` (angle), `id` (internal track ID, `null` while the tracker does not know the person yet; do not display it as a person number, it changes after every gap) and `ziel` (`true` for the person it follows); `skelett`: point pairs to connect (skeleton). Empty while the mode is off. |

### Drive recorder

New 03.11. The server always keeps the last minute (`RECORDING_BUFFER_S`)
in memory: every lidar scan, every drive command before and after the
watchdog's limits (`soll`, `safe`, `quelle`), hard stops, board speed and
pose (20 Hz) and state changes (explorer, interaction, homing). Only
`record_save` (button „Fahrt speichern“ in the web app) writes it to
`RECORDING_FOLDER` (default `~/.rosmaster/aufzeichnungen/`); the newest
`RECORDING_MAX_FILES` are kept. Purpose: review conspicuous drives on your
own computer with `scripts/replay.py` and test new avoidance logic against
them.

| Endpoint | Returns |
|---|---|
| `GET /aufzeichnung/status` | `aktiv` (active), `puffer_s` (buffer length), `ereignisse` (events), `scans`, `dauer_s` (what is currently in the buffer), `ordner` (folder), `gespeichert` (saved since start), `letzte_datei` (last file). When disabled only `{"aktiv": false}`. |
| `GET /aufzeichnung/liste` | `dateien` (files): each with `name`, `bytes`, `zeit` (time), newest first |
| `GET /aufzeichnung/datei/<name>` | the file for download (`application/gzip`). Unknown or invalid name `404`. |

File format: gzip-compressed JSON Lines. The first line is the header
(`version`, `erstellt` (created), `notiz` (note), `dauer_s`, `antrieb`
(drive), `mock`, `lidar`, `config` with the driving-related settings),
followed by one event per line with `t` (seconds since the first event) and
`art` (kind):

| `art` | Fields |
|---|---|
| `scan` | `zeit` (end of revolution), `dauer` (duration), `w` (angle in tenths of a degree), `r` (distance in mm) |
| `befehl` | command: `quelle` (source), `soll` (requested) and `safe` (each `[vx, vy, omega]`) |
| `stop` | – (hard stop via the watchdog) |
| `odo` | `v` (board speed), `pose` (`[x, y, theta_grad]`, theta in degrees), `ist` (current command), `misst` (whether `v` is measured rather than estimated) |
| `zustand` | state: `erkunder` (explorer), `erkunder_text`, `erkunder_grund`, `interaktion`, `heimfahrt` (homing) – only on change |

```bash
curl -O http://<robot>:8000/aufzeichnung/datei/fahrt_20261103_141500.jsonl.gz
python scripts/replay.py fahrt_20261103_141500.jsonl.gz --svg fahrt.svg --erkunder
```

### Diagnostics

Collects everything that can go wrong on the X3 (battery, voltage drops,
USB overcurrent and disconnects, undervoltage, temperature, device
failures, errors in the log) and turns it into problems with an explanation
and steps to take. The page for it is `/diagnose`.

| Endpoint | Returns |
|---|---|
| `GET /diagnose/status` | overall state, problems, readings, devices, timeline, battery history (see below) |
| `GET /diagnose/protokoll?nach=N` | server log lines with number > `N`: `{"zeilen": [{"id", "zeit", "stufe", "quelle", "text"}], "letzte_id"}` (lines with id, time, level, source, text). `letzte_id` is always the server's latest number; if it is smaller than `N`, the server was restarted. |
| `POST /diagnose/quittieren` | Body `{"id": "akku_schwach"}` or `{"alle": true}`: mark as seen (acknowledge). Without either, `400`. |

`GET /diagnose/status` (abridged):

```json
{
  "zeit": 1793260800.0,
  "simulation": false,
  "gesamt": {"stufe": "warnung", "text": "Läuft, aber etwas stimmt nicht."},
  "probleme": [
    {"id": "akku_schwach", "schwere": "warnung", "titel": "Akku schwach (11.0 V)",
     "was": "…", "tun": ["Bald laden.", "…"], "seit": 1793260700.0,
     "zuletzt": 1793260800.0, "anzahl": 1, "besteht": true, "zustand": true,
     "quittiert": false, "vor_s": 0.0}
  ],
  "messwerte": {"akku_v": 11.0, "akku_prozent": 23, "temp_c": 61.2, "…": "…"},
  "geraete": [{"name": "Lidar", "ok": true, "text": "7.6 Hz", "neu": 0}],
  "ereignisse": [{"id": 3, "zeit": 1793260790.0, "art": "spannung",
                  "schwere": "warnung", "text": "Spannungseinbruch: 12.1 V → 11.3 V (beim Fahren)"}],
  "akku_verlauf": [[1793260700.0, 11.4]],
  "schwellen": {"akku_warn_v": 11.1, "akku_kritisch_v": 10.5, "temp_warn_c": 75.0, "cpu_max_mhz": 2000},
  "quellen": {"kernel_log": "ok", "usb_zaehler": true, "vcgencmd": true, "akku": true}
}
```

- Top-level fields: `gesamt` (overall), `probleme` (problems), `messwerte`
  (readings), `geraete` (devices), `ereignisse` (events), `akku_verlauf`
  (battery history), `schwellen` (thresholds), `quellen` (data sources).
  Per problem: `titel` (title), `was` (what happened), `tun` (what to do),
  `seit` (since), `zuletzt` (last seen), `anzahl` (count), `quittiert`
  (acknowledged), `vor_s` (seconds ago).
- `gesamt.stufe`: `kritisch` (critical), `warnung` (warning) or `ok`; only
  counts unacknowledged problems.
- `schwere` (severity): `kritisch`, `warnung` or `hinweis` (hint).
- `zustand: true`: a condition (battery, lidar gone …); `besteht` says
  whether it currently applies. `zustand: false`: a one-off incident (USB
  overcurrent), visible for `PROBLEM_WINDOW_S` seconds.
- Acknowledged lasts until the next *new* occurrence.
- `quellen.kernel_log`: `ok`, `keine Berechtigung` (no permission: add the
  user to the `systemd-journal` group), `nicht vorhanden` (not present:
  Windows, simulation) or `Fehler beim Lesen` (read error).
- The warning thresholds can be set via `/settings` (block `diagnose`).
- `absturz` (since 29.10.): crash and hang capture. Fields:
  - `aktiv`, `ordner`: whether it records and where to
    (`~/.rosmaster/diagnose/`).
  - `boot`, `journal_dauerhaft`: boot ID; whether the kernel log survives
    reboots (persistent journal).
  - `vorher`: explanation if the previous run did not end cleanly
    (`art`: `pi_neustart` (Pi reboot), `segfault`, `speicher` (memory),
    `prozess_weg` (process gone); plus `titel`, `was`, `tun`, log lines,
    `dump`), otherwise `null`.
  - `laeufe`: the last server runs (`pid`, `boot`, `start`, `zuletzt`,
    `ende`; `ende: null` = not ended cleanly).
  - `haenger`, `puls_max_s`, `puls_alter_s`: how often and how long the
    event loop stalled; `blockiert` (`zeit`, `dauer_s`, `stapel` = stack)
    shows where it last hung.
  - `speicher` (`server_mb`, `threads`, `dateien`, `frei_mb`, `gesamt_mb`)
    and `speicher_verlauf`: to detect a memory leak.
  - `usb_ports`, `usb_ports_neu`, `usb_pro_min`: overcurrent counters per
    root port, new ones since startup and rate per minute. From
    `USB_STORM_PER_MIN` on, the page reports a persistent USB overload.

### Music („MUKKE“)

Plays the audio of a YouTube link on the robot's USB loudspeaker. Playback
happens on the robot, not in the browser. In the web app, the „MUKKE“
button at the top left opens the dialog for it.

| Endpoint | Returns |
|---|---|
| `GET /mukke/status` | `laeuft` (playing), `pausiert` (paused), `titel` (title), `url`, `lautstaerke` (volume, 0–100), `fehler` (why nothing played last time), `verfuegbar` (available) and `hinweis` (what is missing, e.g. „mpv fehlt“) |
| `POST /mukke/play` | Body `{"url": "https://youtu.be/…"}`. Replaces whatever is playing. Only links from youtube.com, music.youtube.com and youtu.be, otherwise `400` with `{"fehler": "…"}`. |
| `POST /mukke/pause` | toggle pause |
| `POST /mukke/stop` | stop |
| `POST /mukke/volume` | Body `{"lautstaerke": 70}`: fraction of the overall volume `VOLUME` (mpv gets both multiplied) |

All `POST`s respond with the new state, like `/mukke/status`. It takes a few
seconds until the title is known (yt-dlp first has to find the audio track).
Requirements on the Pi: `mpv` and `yt-dlp`, see `docs/hardware.md`.

### Voice

| Endpoint | Returns |
|---|---|
| `GET /stimme/status` | `aktiviert` (enabled), `verfuegbar` (available), `simuliert` (simulated), `ausgabe` (output: `piper`, `espeak`, `text`, `simuliert`), `geraet` (ALSA name), `spricht` (speaking), `warteschlange` (queued sentences), `zuletzt` (last sentence), `gesagt` (sentences spoken), `verworfen` (sentences dropped), `fehler` (error, e.g. a message from aplay if the loudspeaker is missing) |

### Speech commands

| Endpoint | Returns |
|---|---|
| `GET /sprache/status` | `aktiviert` (enabled), `verfuegbar` (available), `simuliert` (simulated), `zustand` (state: `aus` off, `laedt` loading, `hoert` listening, `fehler` error, `simuliert`, `pausiert` paused), `pause` (why the microphone is deliberately closed right now, e.g. „Mapping läuft“, otherwise `null`), `mikrofon` (device name), `modell` (path), `weckwort` (wake word), `geweckt` (awake), `zuletzt_gehoert` (last heard), `letzter_befehl` (last command), `erkannt` (recognized count), `konfidenz` (lowest word confidence of the last sentence, 0..1), `verworfen` (rejected count), `zuletzt_verworfen` (last rejected: command, text, reason), `befehle` (all phrases per command), `fehler` (e.g. „vosk fehlt (pip install vosk sounddevice)“, „Vosk-Modell fehlt: …“ or „kein Mikrofon gefunden (vorhanden: …)“), `filter` (speech filter on the open microphone stream, `null` when off or not listening: `aktiv`, `verstaerkung_db` gain in dB, `snr_db` speech-to-noise of the last block, `uebersteuert` share of clipped input samples — above 0 means the capture gain is too high) |
| `GET /sprache/live` | Live transcript (app: switch „Mikrofon mitlesen“, polls 4 times per second): `verfuegbar`, `zustand`, `mikrofon`, `rate` (Hz), `pegel` (level, 0..1, −60 dBFS to full scale), `teil` (partial: what Vosk understands of the current sentence so far), `taub` (deaf: it is speaking itself and evaluates nothing), `geweckt`, `ereignisse` (last sentences: `zeit` (Unix time), `text`, `ergebnis` (result) = `befehl` (command) / `verworfen` (rejected) / `weckwort` (wake word) / `ohne` (none), plus `befehl` and, for `verworfen`, the `grund` (reason)). The server only computes level and `teil` while someone is polling. |
| `WS /sprache/ton` | Live listening (app: switch „Mikrofon mithören“): first a text message `{"rate": 16000}` (`null` while no microphone is open; resent on every change and, without audio, every second as a heartbeat: if nothing at all arrives for 5 s, the app reconnects), then the audio as binary messages (what the speech recognition hears, i.e. after the speech filter when it is on), 16 bit little-endian, mono, blocks of 0.25 s. If the browser cannot keep up, blocks are dropped (`SPEECH_SOUND_BUFFER`). Data is only sent while someone is connected. |
| `WS /sprache/durchsage` | Announcement (app: switch „Über den Roboter sprechen“): audio from the browser microphone live through the robot's loudspeaker. The server first replies `{"bereit": true|false, "grund": …}` (ready, reason), and again later if the loudspeaker fails or comes back. The client sends `{"rate": 16000}` and then raw audio as binary messages (16 bit little-endian, mono). Runs in the voice queue: music pauses, speech recognition stops listening for the duration, a stop aborts it. After `ANNOUNCE_PAUSE_S` (2 s) without audio, the loudspeaker is free again. Browsers only grant microphone access over HTTPS, so the same app additionally runs at `https://<robot>:8443` (`HTTPS_PORT`, self-signed certificate in `~/.rosmaster/https`; confirm the browser warning once). |

### Mapping (SLAM)

The map is built by slam_toolbox in the ROS container `x3`. The server
feeds in lidar and position itself (E8 in [`decisions.md`](decisions.md)).
User interface: `GET /map`.

| Endpoint | Returns |
|---|---|
| `GET /api/mapping/status` | state (see below) |
| `POST /api/mapping/start` | starts scan feed, map reception and SLAM; errors come back as 503 with text |
| `POST /api/mapping/stop` | stops everything again (including the mapping drive); the last map remains available |
| `POST /api/mapping/fahrt/start` | start the mapping drive (only while mapping is running, otherwise 409 with reason) |
| `POST /api/mapping/fahrt/stop` | stop the mapping drive |
| `GET /api/mapping/map` | last map (`nav_msgs/OccupancyGrid`, cells as Base64), 404 before the first one |
| `POST /api/mapping/save` | saves the last map as PGM + YAML, body `{"name": "wohnzimmer"}` |
| `GET /api/mapping/maps` | saved maps |
| `POST /api/mapping/laden` | load a saved map to drive in it, body `{"name": "wohnzimmer"}`; 404 unknown, 409 during mapping |
| `GET /api/mapping/ortung` | pose in the map, trail, running drive, open frontiers (see "Using the map") |
| `POST /api/mapping/ziel` | drive to a point, body `{"x": 2.5, "y": -1.0, "winkel": 90}` (`winkel` optional); 409 with reason if not possible |
| `POST /api/mapping/ziel/stop` | cancel the goal drive |
| `POST /api/mapping/pose` | "I am here", body as for `ziel`: search for the pose there |
| `WS /api/mapping/ws` | `{"type": "status"}` every 3 s, `{"type": "map"}` on every new map, `{"type": "fahrt"}` on changes of the mapping drive, `{"type": "ortung"}` on changes of the pose (content as `GET /api/mapping/ortung`) |

`GET /api/mapping/map` and the `map` packets deliver the map **the robot is
currently driving with**: the live map of the mapping run or, if a saved map
is loaded, that one. Next to `<name>.pgm` and `<name>.yaml`, `save` writes a
`<name>.json` with the map's origin (`{"start": [x, y, radians]}`).

For feedback in the status:

```json
{
  "aktiv": true, "mapping_running": true, "map_available": false,
  "hinweis": "SLAM läuft, warte auf die erste Karte (6 s). 31 Scans an ROS gegeben.",
  "hinweis_fehler": false,
  "einspeisung": {"laeuft": true, "bereit": true, "scans": 31,
                  "letzter_scan_vor_s": 0.1, "lidar_scan_vor_s": 0.1,
                  "neustarts": 0, "fehler": null},
  "slam_log": ["[INFO] [launch]: ...", "..."]
}
```

`hinweis` (hint) names the first step that is stuck: lidar, hand-off to
ROS, SLAM process, map reception. With `hinweis_fehler` the cause is usually
in `slam_log`. `einspeisung` is the scan feed into ROS. In addition there
are `lidar_odometrie` (position from lidar matching, number of successful
and rejected matches) and `fahrt` (`aktiv`, `text`, `schritt` (step),
`bahnen` (lanes), `dauer_s`, `grund` (reason) of the last mapping drive).

**Mapping drive.** The robot covers the room like a robot vacuum: to the
nearest wall, once all the way around with the wall on its right, then lanes
parallel to that wall, each `MAPPING_DRIVE_LANE_SPACING_M` (0.8 m) further
into the room, up to the opposite wall. On the lanes it does not turn
around, but drives forward, slides sideways and drives back in reverse
(mecanum). A lap shorter than `MAPPING_DRIVE_ISLAND_M` (5 m) counts as an
obstacle in the middle of the room: it leaves it and looks for the wall.
Values: `MAPPING_DRIVE_*` in `config.py`.

**Room by room (new 04.11.):** when the room is done, it does not stop. It
drives to the next spot where the map turns into the unknown (open door,
corridor, shadow behind furniture; a "frontier", „Lücke“ in the UI), looks
around there and seeks the next one -- until no reachable one is left. Then
it drives back to the origin and reports „Kartierfahrt fertig -- Karte
vollständig (n Lücken angefahren), zurück am Ausgangspunkt“. Even when it
cannot progress in the room (no wall in sight, stuck, the drive guard lets
nothing through), the drive no longer ends but moves on to the next
frontier. It skips unreachable frontiers; only after `FRONTIER_MAX_ERRORS`
(6) consecutive failures does it give up. `fahrt.luecken` counts the
frontiers visited. Values: `FRONTIER_*`, `MAPPING_DRIVE_RETURN`. Maximum
duration is now `MAPPING_DRIVE_MAX_S` = 1 h.

Technically it is a mode of the explorer (`state.autonomie.modus =
"karte"`). Therefore the same things end it as `explore`: `stop`,
`explore_stop`, any drive input ≠ 0, `home`, `set_home`, interaction and
`POST /api/mapping/stop`. The end is reported by `alert` „Autonomes Fahren
beendet: Kartierfahrt fertig -- …“. What **no longer** ends it (new 04.11.):
- *Page closed:* only when neither a `/map` page nor the app is connected
  does it stop, and even then only after `MAP_WATCHER_WAIT_S` (20 s). A
  phone whose screen briefly turns off therefore no longer ends it.
- *Drive briefly gone (USB reset):* it stops and continues after
  reconnecting (directly to the next frontier) if that happens within
  `MAPPING_DRIVE_RESUME_S` (60 s). `fahrt.unterbrochen` (interrupted) is
  `true` during that time. A `stop` discards the resumption.
- *Lidar briefly gone:* it waits up to `MAPPING_DRIVE_LIDAR_WAIT_S` (30 s);
  for random exploring it stays at 3 s.

**Using the map (new 04.11.).** When a map exists, the robot knows where it
is in it. During mapping this applies immediately; afterwards (mapping
stopped) the map and the pose are kept, so it can drive in it right away. A
saved map is loaded with `POST /api/mapping/laden` or `map_load`; it then
looks for its pose at the map's origin, and if it cannot find itself there,
you show it with `POST /api/mapping/pose` or `map_pose` (on the `/map`
page: „Hier stehe ich“, tap, drag for the heading). Localization matches the
lidar scan against the map several times per second and thereby corrects
the odometry drift (E12). It would rather say "don't know" than something
wrong: it cannot tell apart two identical empty rooms (or one that looks the
same rotated by 180°); then it asks.

`GET /api/mapping/ortung`:

```json
{
  "ortung": {"karte": true, "name": "wohnung", "live": false,
             "pose": {"x": 2.1, "y": -0.4, "winkel": 87.0}, "sicher": true, "sucht": false,
             "text": "Lage in der Karte bekannt", "anteil": 0.93, "residuum_m": 0.031,
             "start": [0.0, 0.0, 0.0], "abgleiche": 412, "verworfen": 3},
  "spur": [[0.0, 0.0], [0.1, 0.02], "..."],
  "fahrt": {"aktiv": true, "art": "ziel", "nach_karte": true, "ziel": [3.5, -1.0],
            "weg": [[1.2, 1.8], [3.5, -1.0]], "rest_m": 2.4, "grund": null},
  "kartierfahrt": {"weg": [], "ziel": null},
  "luecken": [[1.4, 1.85]]
}
```

`anteil` (share) is the fraction of lidar points that hit a wall of the
map; below `NAV_MIN_SHARE` (0.45) a match does not count. `sicher`
(confident) means: one matched within the last 4 s. `sucht`: it is
searching for its pose. `start` is the map's origin, `abgleiche`/`verworfen`
count accepted/rejected matches. `spur` (trail) is the path driven (one
point per 10 cm), `fahrt.weg` the planned path, `luecken` (frontiers) are the
open transitions into the unknown (only while a `/map` page is open). The
`/map` page shows all of this on the map and has the buttons „Hierhin
fahren“ (drive here), „Hier stehe ich“ (I am here), „Zum Ausgangspunkt“ (to
the origin) and „Fahrt abbrechen“ (cancel drive). Values: `NAV_*` in
`config.py`.

**In the simulation** (`--mock`) there is neither Docker nor slam_toolbox;
the server then builds the map itself from the simulated lidar
(`mapping/simulation.py`, `status.simulation = true`). With
`ROSMASTER_MOCK_WOHNUNG=1` the imaginary room becomes an apartment with two
rooms, a door and furniture -- for trying out the room-to-room continuation.

### Healthbot

The healthbot checks whether a person is fine: drive there, overview from the
camera pose (lying? on the floor? moving?), address them by voice, nudge them
gently if there is no answer, call for help (alarm music, `alert`
`emergency`). Commands go over the WebSocket (`health_check`, `health_stop`,
`health_wache`); the page `GET /healthbot` shows the run.

| Endpoint | Returns |
|---|---|
| `GET /healthbot` | page with the run: steps with reasons, traffic lights of the four signs, camera image with the person |
| `GET /healthbot/status` | `aktiv`, `zustand`, `schritt`, `timeline` (steps `{zeit, zustand, text}`), `checks` (the four signs, each `{name, titel, zustand, text, ...}`), `urteil` (verdict), `ziel` (target), `alarm`, `stupser` (nudges), `wache`, `stupsen_aktiv` (nudging enabled), `person` (`{box, kp, winkel}` or `null`); in `--mock` also `simulation` and `szenen` (scene names) |
| `POST /healthbot/simulation` | only `--mock`: `{"szene": "liegt", "x": 2.0, "y": 0.5}` puts a simulated person into the scene (`x`/`y` optional). 400 with `fehler` for an unknown scene, 404 without mock. |

Every finished check (any outcome) is stored as a run like the tests below
(`typ: "health"`), so it can be looked at later without the robot.

### Test runs

Reproducible measurements for camera, pose and microphone: the robot announces
each phase („Stell dich links neben mich …“; left/right always seen from the
test person) and records readings, images and sound. The page `GET /tests`
guides through it. While a microphone test runs, recognized speech commands are
not executed (this includes a spoken „Robi, stopp“; use the stop button).

Tests never drive, with one exception: `mikro_fahrt` (new 05.11.) spins the
wheels while the robot stands on its stand, to measure how much motor noise
hurts speech recognition (stopped, slow 0.15 m/s, fast 0.35 m/s, turning
1.0 rad/s; `TESTS_MOTOR_*`; forward and turning only). It needs `podest: true`
in the start request, runs through the watchdog (dead man's switch), and ends
on `POST /tests/stop`, any drive input ≠ 0, `stop`, `move`/`turn`,
`explore`, `interaction_start`/`follow`, `home`, `goto`, `map_load`,
`health_check`/`health_stop`/`health_wache`, and when the last client
disconnects. The verdict lists hits per wheel condition, e.g.
`aus 2/2, langsam 2/2, schnell 1/2, drehen 0/2`.

| Endpoint | Returns |
|---|---|
| `GET /tests` | the tests page |
| `GET /tests/katalog` | `{"kategorien": {key: name}, "tests": [{id, kategorie, titel, kurz, art, faehrt?, anleitung[], phasen[{name, titel, anweisung, dauer_s, satz?, bedingung?, fahrt?}]}]}`; `faehrt: true` = the test spins the wheels, then each phase has `bedingung` (wheel condition) and `fahrt` (`{vx, vy, omega}` or `null`) |
| `GET /tests/status` | the running test: `aktiv`, `test`, `titel`, `art`, `phase`, `phase_titel`, `anweisung`, `satz`, `rest_s`, `live` (current readings), `gehoert` (microphone); always `letzter` (last run), `meldung`, `ordner` (folder on the Pi), `simulation`, `token` (GitHub token present) and `upload` (`auto`, `laeuft`, `ausstehend` = runs not uploaded yet, `letzter`, `fehler`, `hinweis`) |
| `GET /tests/liste` | `{"laeufe": [...]}`, newest first: `name`, `typ` (`"test"` / `"health"`), `test`, `titel`, `kategorie`, `beginn`, `abgebrochen`, `urteil`, `notiz`, `phasen[{titel, kurz}]`, `bytes`, `hochgeladen`, `upload_commit`, `upload_zeit`, `upload_fehler` |
| `POST /tests/start` | `{"test": "<id>", "podest": true}` starts a test (`podest` only needed for tests with `faehrt`); 400 with `fehler` if unknown, already running, a sensor is missing, the stand was not confirmed, or (for `faehrt`) another drive or the healthbot watch is active |
| `POST /tests/stop` | cancel the running test (the run is still stored, marked `abgebrochen`) |
| `POST /tests/notiz` | `{"name": "<run>", "text": "…"}`: note on a run (what stood out); counts as a change, so the run is uploaded again |
| `GET /tests/zip?name=<run>` | the run folder as a ZIP |
| `POST /tests/hochladen` | `{"namen": [...]}` (optional, without = all new ones) uploads runs to GitHub; `{ok, hochgeladen[], commit, fehler}` |
| `POST /tests/loeschen` | `{"namen": [...]}` or `{"alle": true}` deletes runs on the robot (copies on the `testdaten` branch stay); refused while a test runs or an upload is in progress; `{ok, geloescht[], fehler}`, 400 if nothing was deleted |

**Storage.** Each run is a folder `~/.rosmaster/tests/<date>_<time>_<test>/`
on the Pi (`TESTS_FOLDER`) with `ergebnis.json` (readings, summary per phase,
verdict, note, and in `meta` the code commit/branch, the computer, all scalar
settings and the test definition, so old runs stay comparable after
thresholds change), `bilder/` and, for microphone tests, two WAV files:
`ton.wav` is what the speech recognition hears (after the speech filter),
`ton_roh.wav` the unfiltered input, to see clipping and to replay the run
with other filter settings. Each microphone sample also stores `filter`
(the values from `/sprache/status`), and the phase summary adds
`uebersteuert_max` (largest clipped share; above 0 the capture gain is too
high) and `verstaerkung_db` (filter gain distribution).

**Upload.** After every run the robot pushes all runs that are not uploaded
yet to the branch `testdaten` of the repository (`TESTS_GIT_BRANCH`,
`TESTS_GIT_AUTO_UPLOAD`), each into `tests/<name>/`, plus `index.json` and
`README.md` (a table GitHub renders). The Pi's working tree and branch stay
untouched (separate git index, `commit-tree`). It needs a GitHub token with
write access in `~/.rosmaster/github_token` (`TESTS_GIT_TOKEN_FILE`, file only
readable by the user); the token travels as an HTTP header, never in the URL,
the log or the git config. Without a token, or when GitHub is not reachable,
runs stay pending and go along with the next upload. In `--mock` nothing is
uploaded automatically.

**Results without the robot.** Everything uploaded is on GitHub:

- Browse: <https://github.com/TheEmpire002/SPE-ROSMASTER-X3/tree/testdaten>
  (README with all runs, newest first).
- Machine-readable: `index.json` with `{branch, erzeugt, anzahl, laeufe: [{name, datum, typ, test, titel, urteil, abgebrochen, simulation, notiz, code_commit, pfad}]}`.
  Raw: `https://raw.githubusercontent.com/TheEmpire002/SPE-ROSMASTER-X3/testdaten/index.json`,
  one run: `.../testdaten/tests/<name>/ergebnis.json`.
- Locally without touching your branch:

```bash
git fetch origin testdaten
git show origin/testdaten:index.json
git worktree add ../testdaten origin/testdaten   # all files as a folder
```

```python
import json, urllib.request
base = "https://raw.githubusercontent.com/TheEmpire002/SPE-ROSMASTER-X3/testdaten/"
index = json.load(urllib.request.urlopen(base + "index.json"))
run = json.load(urllib.request.urlopen(base + index["laeufe"][0]["pfad"] + "ergebnis.json"))
```

### Miscellaneous

| Endpoint | Returns |
|---|---|
| `GET /` | the bundled web app |
| `GET /einstellungen` | the web app's settings page |
| `GET /diagnose`, `GET /logs` | the web app's diagnostics page |
| `GET /tests`, `GET /healthbot` | test runs and healthbot pages (see above) |
| `GET /static/...` | its files (`webapp/`) |
| `GET /docs` | automatically generated HTTP overview (FastAPI) |

---

## Developing without the robot

```bash
python main.py --mock
```

starts the same server on your own computer at `http://localhost:8000`.
All endpoints and messages behave as on the robot:

- **Drive:** simulated, with deliberate wheel slip. Drive commands appear
  in the log.
- **Lidar:** scans an imaginary room (8 × 6 m with a pillar) that moves in
  accordance with the motion. Localization, homing and collision avoidance
  can be fully tried out with it. With `ROSMASTER_MOCK_WOHNUNG=1`: two rooms
  with a door, a cupboard and a sofa.
- **Mapping:** without Docker; the map is built in Python from the
  simulated lidar. Mapping drive, localization and goal drive work as on the
  robot.
- **Camera:** one moving test image per camera (`orientierung`, `personen`).

`driver: "mock"` in the `state` message shows that you are in the
simulation.

---

## Extending

If you add new features in your own branch:

**New WebSocket command**
1. Add it to the header comment in `protocol.py` (that is the contract).
2. Add a branch `if kind == "…":` in `_handle()` in `api/server.py`.
3. Update this file (`docs/api.md`).

**New HTTP endpoint**: in `create_app()` in `api/server.py`, next to the
existing `@app.get(...)`. It then appears automatically under `/docs`.

**Do not restructure existing messages.** The web app and the ESP rely on
the field names. Add a new field or a new `type` instead; additional fields
do not disturb any existing client.

**Always drive through the watchdog.** Anyone generating drive commands
themselves (such as an autonomy block in the server) calls
`watchdog.submit(DriveCommand(...))`, never the driver directly. Only then
do limiting and the dead man's switch apply. See `navigation/homing.py` as
the model.

**Where things are** (`robot/src/rosmaster_assist/`):

| File | Content |
|---|---|
| `protocol.py` | message format, `DriveCommand` |
| `api/server.py` | all endpoints, WebSocket handling, background loops |
| `config.py` | all limits and settings in one place |
| `safety/watchdog.py` | limiting, dead man's switch and acceleration ramp |
| `safety/collision.py` | collision avoidance during manual driving (via the drive guard) |
| `safety/drive_guard.py` | footprint-based collision check for all drives |
| `navigation/odometry.py` | position estimation |
| `navigation/localizer.py` | lidar matching against the start-point scan |
| `navigation/homing.py` | homing |
| `navigation/maneuver.py` | precise drive commands (`move`, `turn`); the lidar measures distance and angle |
| `sensors/lidar.py` | reading the lidar (real and simulated) |
| `sensors/camera.py` | camera (CSI, USB, test image) |
| `drive/` | motor drivers (real and simulated) |
