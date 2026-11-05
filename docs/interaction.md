# Interaction: gestures, speech, "mind"

> **Status 2026-10-29:** Stages 1–3 are implemented (pose on the Hailo,
> gestures, follow/come here/go away/stop). From stage 4, **speech output**
> (`voice.py`, USB speaker) and **speech commands** with the wake word
> "Robi" (`speech.py`, Vosk) are done. Free text and the mind ("Geist") are
> still a concept.
> The project history for this is in `Marlon/PROJECT_LOG.md`.

The robot should perceive people, react to hand signals and calls, and in
doing so appear to have an inner life of its own. Everything runs **inside
the existing FastAPI server**. There is no second process on the board,
lidar or camera.

## Overview

```
 Perception           Interpretation    Mind                     Behavior         Safety
 ──────────           ──────────────    ────                     ────────         ──────
 C270 ─► Hailo pose ─► gestures.py ──┐                             ┌► Follow    ┐
 Mic  ─► Vosk ──────► speech.py ───┼─► geist.py                 ─┼► Come here ├─► SafetyWatchdog.submit()
 Lidar ────────────────────────────┘   attention, mood,          ├► Go away   │   + lidar stop
                                       memory, intention         └► Stop      ┘
                                            │
                                            └─► denken.py  (Claude API / local / templates)
                                                 → thoughts and answers: text in the app + voice (Piper)
```

## Guiding principles

**1. Reflexes are local and deterministic.**
Gestures and speech commands are recognized without a language model and
trigger fixed behaviors. "Stop" must take effect immediately, even without
internet.

**2. The language model never steers directly.**
It formulates thoughts and answers. At most it may propose an intention from
a fixed list (`folgen`, `stopp`, `herkommen`, `weggehen`, `umschauen`), and
the mind checks it against the current state. Drive commands are created
exclusively in the behavior classes. Rationale: see E6 in
[`decisions.md`](decisions.md).

**3. The same safety as everywhere else.**
Every behavior drives via `SafetyWatchdog.submit()`. The lidar stop
(`EXPLORE_STOP_M`) always applies, also while following. Emergency stop,
manual control, homing and connection loss cancel every behavior, exactly
as with the explorer.

**4. Movement only on the floor.**
The lidar cannot see table edges. The interaction mode is therefore **off**
by default and must be switched on deliberately in the app.

**5. If something is missing, the rest keeps running.**
Without the Hailo there are no gestures, without microphone or Vosk no
speech, without a speaker only text, without internet templates answer
instead of Claude. No missing part may stop the server.

## Modules (in `robot/src/rosmaster_assist/`)

| Module | Responsibility |
|---|---|
| `sensors/pose.py` | **done.** Body pose on the Hailo-10H (`POSE_HEF`, 17 COCO keypoints). Thread like `sensors/people.py`, `POSE_HZ` (10 Hz). The Hailo device is opened at server start and kept warm (`POSE_KEEP_WARM`, see "Drive lost when gestures start"), and reopened after errors. Delivers people with box, keypoints and angle to the image center. In the mock a simulated person performs the gestures one after another. |
| `interaction/gestures.py` | **done.** `GestureDetection` over a short keypoint history (`GESTURE_WINDOW_S`, 1.5 s). Testable without hardware. |
| `interaction/speech.py` | **done.** `SpeechRecognition`: USB microphone (`sounddevice`) → Vosk (offline, small German model) with a fixed grammar. Own thread, reconnects after dropouts. Commands only with the wake word "Robi", stop also without it. The server fetches the commands (`fetch_commands`) and executes them in the event loop. Free text after the wake word is still open. |
| `interaction/voice.py` | **done.** Speech output on the USB speaker (ALSA `UACDemoV10`): Piper → `aplay`, fallback espeak-ng, otherwise text only. Own thread with a small queue (`VOICE_*` in `config.py`). `speaking` tells the speech recognition when it has to stop listening. In the mock it only logs. |
| `interaction/behavior.py` | **done.** Class `Interaction` with the modes `bereit`, `folgen`, `herkommen`, `weggehen`, `sucht`; operated like `navigation/explorer.py` (`start`, `follow`, `follow_stop`, `cancel`, `active`, `last_reason`, `state`, `fetch_messages`). |
| `interaction/geist.py` | *planned.* Inner state, memory, choice of intention, expressive movements, stream of thoughts. Rate ~2 Hz. |
| `interaction/denken.py` | *planned.* Hybrid language core with the same interface for all stages: Claude API, later a local LLM on the Hailo, templates. |

## Gestures (version 1)

The gestures are determined from the body keypoints (shoulders, elbows,
wrists, nose). Image coordinates: y grows downwards. All measures are
computed in **shoulder widths** and relative to the shoulder center, so the
person's distance does not matter. The shoulder width is the median over
the window: if the person briefly turns sideways while waving, the width
becomes tiny in one frame, and without the median every small hand movement
would look huge.

| Gesture | Detection | Effect |
|---|---|---|
| **Follow** | One wrist above the head, held ≥ 1 s | Behavior *follow* starts |
| **Stop** | Both wrists above the shoulders, held ≥ 0.4 s. (The "arm stretched horizontally forward" from the concept can hardly be told apart from hanging arms in 2D and was dropped.) | Every behavior ends immediately |
| **Come here** | One wrist moves up and down several times (waving towards the body) | Towards the person, down to 0.7 m |
| **Go away** | One wrist swipes sideways away from the body | ~1 m back, then stand still |

A gesture only counts once it has been detected stably over several
consecutive frames (`GESTURE_*` in `config.py`). A single bad frame
therefore triggers nothing. **Stop** has priority over everything else.

**Debounced (29.10.):** While following, the robot sometimes switched to a
different behavior without anyone giving a signal. People who walk swing
their arms, and that looked like come here or go away. Therefore the
following additionally applies to **come here** and **go away**:

- The person must stand still: shoulder center and shoulder width may
  hardly change over the window (`GESTURE_STILL_DRIFT`, `GESTURE_STILL_SIZE`,
  in shoulder widths). Someone walking towards or past the robot does not
  count.
- The person must face the camera (nose visible). There are no gestures
  from behind.
- The robot itself must be nearly stationary (`GESTURE_ROBOT_STILL_V`,
  `GESTURE_ROBOT_STILL_W`, read from `watchdog.current_command`). If it is
  driving, the whole image moves and arm movements cannot be separated.
- The same gesture must be detected `GESTURE_CONFIRM` times in a row.

**Stop** and **follow** are exempt from this. Stop must always take effect
immediately, and follow (arm above the head, held) is not triggered by
accident while walking.

Because the C270 is tilted only slightly upwards, it sees a standing person
completely only from about 1.5–2 m away. The gestures need at least
shoulders and wrists in the image.

## Behaviors

- **Follow:**
  - The target is the person who made the gesture. The direction comes from the camera, the distance from the lidar: from the lidar cluster within the box's angle range (see "Distance to the person").
  - Target distance `FOLLOW_DISTANCE_M` (1.0 m), P controller on `vx` and `omega`, limited by `FOLLOW_MAX_SPEED`.
  - If the person is gone for more than 2 s, it says "Wo bist du?" ("Where are you?") and searches: it turns towards the side where the person disappeared, alternating 1.2 s of turning and 0.8 s of standing still (`FOLLOW_SEARCH_TURN_S`, `FOLLOW_SEARCH_PAUSE_S`). While turning the camera image blurs; when standing, the pose detection sees sharply. Since 03.11. it keeps searching in a circle until it sees someone again (`FOLLOW_SEARCH_S` = 0) and then follows that person. Before, it gave up after 6 s. End it with stop, the stop gesture or "Robi, gute Nacht" ("Robi, good night").
- **Come here:** drives towards the person down to 0.7 m and stops.
- **Go away:** backs off ~1 m and stops.
- Only **one** behavior is ever active. Explorer, homing and interaction cancel each other.
- Safety as with the explorer: forwards only with a free driving corridor (`free_in_direction`), backwards only with free space behind, and it stands still without fresh lidar data or when something is very close. In mode `bereit` it sends no commands.
- Which person it follows is decided by the **target lock** (see below). Another person is never taken over on its own.
- `herkommen` and `weggehen` end after `INTERACTION_BEHAVIOR_MAX_S` at the latest.
- **Without drive** (board gone, USB reset) it stops and goes to `bereit`, but the interaction mode stays on. Driving modes are rejected as long as `driver.connected` is false; once the board is back, it continues with the next gesture or the next speech command.
- All values: `FOLLOW_*`, `COME_HERE_*`, `GO_AWAY_*` in `config.py`.

### Distance to the person (03.11.)

Previously the smallest lidar value within the box's angle range counted as
the distance. If a chair or table stood in front of the person, the robot
read its distance, stopped and "got stuck" while the person walked on.

Now the clusters from `navigation/obstacles.py` within the box's range
(plus `FOLLOW_CLUSTER_EDGE_DEG`, because camera and lidar do not measure at
the same time) are considered, walls excluded. They are chosen in this
order:

1. two human legs ("Mensch"),
2. matches the last good distance (jump below `FOLLOW_CLUSTER_JUMP_M`, as
   long as that value is younger than `FOLLOW_CLUSTER_GOOD_S`),
3. other objects before furniture ("Stuhl", "Tisch", "Bein": chair,
   table, leg),
4. angle closest to the box center.

If there is no cluster (person too far away, standing at the wall), the
minimum applies as before. Safety does not depend on this: driving forward
is always limited by the free driving corridor, and the drive guard in the
watchdog brakes according to the braking distance (see
`docs/architecture.md`).

### Driving smoothly (29.10.)

While following, the robot drove jerkily, and at the same time USB resets
occurred. The cause was hard jumps in the drive command: person briefly not
detected → immediately 0, back again → immediately full speed; on top of
that a jittering camera angle that jerked `omega` back and forth. Each of
these steps produces a current spike.

| Measure | Effect | Setting |
|---|---|---|
| Acceleration ramp in the watchdog | no command jumps straight to the motors any more, applies to everything; can be switched off in the app under settings → "Sanft fahren" | `RAMP_*` |
| Smoothing the angle | moving average over the frames, small jitters disappear | `FOLLOW_ANGLE_SMOOTHING` |
| Dead band | below a few degrees it does not turn to correct | `FOLLOW_ANGLE_DEADBAND` |
| Soft transition turning/driving | between `FOLLOW_FULL_ANGLE` and `FOLLOW_MAX_ANGLE_DRIVE` the forward speed gradually decreases instead of dropping hard to 0 | `FOLLOW_FULL_ANGLE` |
| Gentler turning | smaller P gain on `omega` | `FOLLOW_KP_W` |
| Coasting | if the person is briefly gone, the last command ramps down linearly over `FOLLOW_HOLD_S`, checking for free space | `FOLLOW_HOLD_S` |
| Holding the lidar distance | if the lidar distance is missing in one cycle, the last good value applies briefly | `FOLLOW_HOLD_S` |
| Hysteresis "near" | the state "person very near" only ends after `FOLLOW_NEAR_HOLD_S`, so it does not toggle back and forth every cycle | `FOLLOW_NEAR_HOLD_S` |

Emergency stop, the stop gesture and the end of a behavior call
`watchdog.stop()` and stop immediately, without a ramp. On an obstacle or
missing lidar data the behavior sends a halt command; that goes through the
braking ramp (`RAMP_BRAKE_XY` = 2 m/s², so from 0.3 m/s in 0.15 s over
about 2 cm). The braking ramp is deliberately much steeper than the
acceleration ramp.

### Drive lost when gestures start (29.10.)

When switching on gesture control, "Keine Verbindung zum Antrieb" ("no
connection to the drive") often appeared, and the interaction was
immediately off again. Two causes:

1. **Load spike:** The Hailo was only opened and the network loaded when
   switching on. That briefly draws a lot of current, and on the shaky USB
   hub the board then disconnects (see [`hardware.md`](hardware.md), USB
   resets). **Solution:** The Hailo is already opened at server start,
   while the robot is standing still, and stays open
   (`POSE_KEEP_WARM = True`). Inference still only runs while the
   interaction is on.
2. **Overreaction:** If the drive was lost, the server ended the whole
   interaction mode. **Solution:** `Interaction.drive_lost()` only stops
   (mode `bereit`). The interaction gets `driver.connected` as a callback
   and rejects driving modes as long as the board is missing. The driver
   reconnects by itself, after which things continue without a restart.

### Target lock (03.11.)

Previously the robot jumped over to a second person while following. The
assignment was based only on the camera angle, and after every search it
took the largest person in the image. Now the rule is: **when in doubt,
stand still.**

Code: `interaction/target.py` (pure logic, testable without hardware),
integrated in `interaction/behavior.py`.

- **`PersonTracker`** assigns track IDs across frames. Each track has a
  Kalman filter on angle and angular velocity. The prediction subtracts the
  robot's **own rotation** (measured turn rate from the odometry, otherwise
  the drive command); otherwise every turn would look like a walking
  person. Greedy assignment by a cost made of angular distance (gate
  `TARGET_GATE_DEG`, grows with the time without measurement), box height
  and appearance. A track without measurement lives on for
  `TARGET_TRACK_HOLD_S`.
- **Appearance:** `sensors/pose.py` computes an HSV histogram (16 × 8) of
  the torso between shoulders and hips for each person (`merkmal`). They
  are compared via the Bhattacharyya distance.
- **`TargetLock`** holds the target: track ID, reference feature (mean from
  the acquisition, afterwards only updated slowly when the scene is
  unambiguous, `TARGET_FEATURE_UPDATE`) and the state:

| State (`ziel.zustand`) | Meaning | Robot |
|---|---|---|
| `idle` | no target | no commands |
| `acquiring_target` | target is being acquired, feature is being learned | already drives as soon as the target is unambiguous |
| `following` | target confirmed | follows only the target track |
| `target_temporarily_lost` | target track without measurement | coasts, stands |
| `searching_for_target` | lost for longer (`FOLLOW_LOST_S`) | turns slowly towards the last side |
| `safe_stop` | not found again or not unambiguous | stands, `grund` says why |

- **Initial choice:** the person making the follow gesture (gestures are
  detected per track), or the one chosen in the app. If you start without
  a choice while several people are visible, it refuses: "mehrere Personen
  – bitte im Bild antippen" ("several people – please tap one in the
  image").
- **Finding again:** During a short occlusion the same track immediately
  belongs to the target again. A new track must match the feature
  (`TARGET_FEATURE_THRESHOLD`), clearly better than any other candidate
  (`TARGET_FEATURE_MARGIN`), and do so in `TARGET_CONFIRM_FRAMES`
  consecutive frames.
- **Safe stop:** After `FOLLOW_SEARCH_S` without a confirmed candidate, or
  when two people match equally well, it goes to `safe_stop`. It only
  leaves that state through a new choice. The color feature cannot tell
  identically dressed people apart; then it deliberately stops instead of
  guessing.
- **Gestures of other people:** While following, only the target's
  gestures count. The exception is **stop**: anyone may do that.
- **Privacy:** Feature and track IDs are kept in RAM only. They are deleted
  on release, at the end of the interaction and at server shutdown. No
  images are stored and no permanent person ID is assigned.
  `/interaktion/pose` does not hand out the feature.

Deviations from the plan:

- The times are still called `FOLLOW_LOST_S` and `FOLLOW_SEARCH_S`.
  Separate `TARGET_*` names would only have produced duplicate settings.
- In `acquiring_target` it already starts driving as soon as the target is
  unambiguous. The safety comes from the unambiguity, not from a waiting
  time while standing; the feature keeps being learned meanwhile.

Test: `Marlon/interaction/test_target.py`. It checks crossing with
occlusion, own rotation, short and long occlusion, a second person during
the search, identical clothing, app start with two people, the gesture
choosing the gesturing person, and stop by passers-by.

### Remembered person and faster re-finding (04.11.)

- **Remembered person:** After following, `TargetLock` keeps the last used
  feature (`remembered`, RAM only) together with a timestamp. If "follow"
  starts without a tap while several people are visible, it first checks
  whether the remembered person is clearly among them (same criteria as
  for finding again: `TARGET_FEATURE_THRESHOLD`, `TARGET_FEATURE_MARGIN`);
  only then does it take that person, otherwise it still refuses with
  "bitte antippen" ("please tap"). Valid for `TARGET_REMEMBER_S` (120 s)
  from the last assignment. It is deleted on "Loslassen" (release,
  `follow_stop`) and at the end of the interaction, not on the stop gesture
  or `safe_stop` — there the next "follow" should take the same person
  again. On a tap or the gesture, the feature of the chosen person is taken
  over as the reference immediately, so a short track loss in
  `acquiring_target` does not lose that person right away.
- **Confirmed faster:** If a candidate clearly matches while re-finding
  (distance below half the threshold, lead over the second best at least
  twice the margin), `TARGET_CONFIRM_FAST_FRAMES` (3) consecutive frames
  suffice instead of the usual `TARGET_CONFIRM_FRAMES` (5). While a
  candidate is being checked (`TargetLock.checking_candidate`), the robot
  holds still during the search instead of turning the person out of the
  image again by turning further.
- **Shorter search delay at the image edge:** If the person disappears in
  state `target_temporarily_lost` close to the image edge (`|angle| > FOV/2
  − TARGET_EDGE_DEG`, 8°), the search already starts after
  `FOLLOW_LOST_EDGE_S` (0.7 s) instead of after `FOLLOW_LOST_S`. In the
  image center the longer delay still applies, since there the person is
  usually only briefly occluded. Directly after a mode change (no fresh
  angle yet) or when already in `searching_for_target`, the normal delay
  always applies.
- **Lidar hint while searching:** If the obstacle detection sees a person
  cluster (pair of legs) outside the camera image, the robot turns there
  instead of turning towards the last known side in the fixed rhythm. If
  the cluster is already in the image, it holds still for `TARGET_HINT_S`
  (1.5 s) so the camera can recognize the person; if the camera finds no
  one there, the fixed turn-pause rhythm applies again for the rest of the
  search. The target is still recognized only via the camera
  (appearance) — the lidar only provides the viewing direction, never the
  assignment itself.

### Operation in the web app (03.11.)

Control happens via the web app; gestures and speech remain secondary
paths.

- **Tapping a person:** While the interaction is running, the people in the
  image of the people camera can be tapped. A tap sends
  `follow {ziel_id}` with the track ID from `/interaktion/pose`. This also
  works while following: then the target changes. A separate button "Neu
  wählen" ("choose again") would be redundant and was dropped. If boxes
  overlap, the smaller one wins, usually the person standing further back
  whom you meant on purpose.
- **Display:** The target is green, all others gray, and above each head is
  "Person N". The line "Ziel" shows the state (e.g. "folgt Person 3 ·
  82 %", "sucht Person 3 …", "sicher angehalten – bitte neu wählen").
- **Pause / resume:** `follow_pause` stops; tracker and lock keep running so
  that after `follow_resume` the same person is still the target. During
  the pause no time limits apply (lost, searching); the stop gesture ends
  the pause like any mode.
- **Release:** `follow_stop` releases the target and also acknowledges a
  `safe_stop`.
- If you tap a person the tracker has just lost, it refuses ("bitte neu
  antippen", "please tap again") instead of taking someone else.

Test: also `test_target.py` (tapping, switching, unknown ID, pause beyond
the search time, resume, stop during the pause, IDs in the pose, release
from `safe_stop`).

## Speech

- **Input:** the USB microphone (FongLun "USB Microphone" or C-Media, `SPEECH_MICROPHONE_NAMES`; the microphones in the C270 and the Orbbec are excluded via `SPEECH_MICROPHONE_NEVER`, the C270's understands too poorly), recognized with **Vosk** and the small German model (`SPEECH_MODEL`) on the CPU. If Vosk, the model or the microphone is missing, recognition is off (`/sprache/status` says why), and the server keeps running.
- **Fixed grammar:** Vosk may only recognize the words of the commands and the wake word; everything else becomes `[unk]`. This makes recognition fast and robust against conversations in the room. Words the model does not know (e.g. "robi") drop out of the grammar; that is why there are variants (`SPEECH_WAKE_WORD_VARIANTS`: "robbie", "ruby" …).
- **Wake word "Robi":** A command only counts if "Robi" occurs in the same sentence ("Robi, folge mir", "folgen Robi") or was said on its own at most `SPEECH_WAKE_WINDOW_S` (5 s) before. A command uses up the window.
- **Stop always works:** "Stopp", "Halt", "Bleib stehen" (stop, halt, stand still) work even without the wake word (`SPEECH_STOP_WITHOUT_WAKE_WORD`) and win against any drive command in the same sentence.
- **Not listening to itself:** While the voice is speaking (`Voice.speaking` plus `VOICE_TAIL_S`), the microphone input is discarded.
- **Downside of the grammar:** Vosk squeezes every noise into an allowed sentence. Music and conversations thus became "robbie hallo" and switched the interaction on (03.11.). Therefore a sentence only counts fully if every word has at least `SPEECH_MIN_CONFIDENCE` and at most `SPEECH_MAX_UNK_SHARE` of the words are `[unk]`; otherwise only stop counts. The same goes for word salad: a spoken command arrives as exactly one grammar sentence, whereas from noise Vosk strings several together ("robin hallo hause zu weg ruby" → go away, "ruby komm da fahr mir la ruby leise box" → come here, both in the log on 03.11.). If the wake word appears twice, or more than `SPEECH_MAX_EXTRA_WORDS` (0) words stand next to the wake word and the phrase, only stop counts. While MUKKE is playing, every command except stop needs the wake word and at least `SPEECH_MIN_CONFIDENCE_MUSIC`. At first only stop and "Musik aus" (music off) counted during music, and then the robot was deaf after "Robi, Beatbox" until the song ended. A wake word rejected this way opens no window, and neither does a wake word in word salad ("robin la"). Rejected input is shown in the log and in `/sprache/status`.
- **"Robi" alone gets an answer:** After `SPEECH_WAKE_REPLY_AFTER_S` (1 s) it says "Ja?" ("Yes?", `SPEECH_WAKE_REPLY`), unless the command has already arrived. The pause keeps the "Ja?" from interrupting someone who says "Robi … folge mir" with a short break, because while it speaks, it does not listen. For the same reason the wake window only starts running out once it has finished speaking.
- **Off while mapping (04.11.):** While mapping is running (`/api/mapping/start` until `stop`, with or without a mapping drive), the listening thread closes the microphone and waits; afterwards it reopens it by itself. Reasons: the stream loads the USB bus the lidar is also on, Vosk takes CPU away from SLAM, and a misheard "folgen" would cancel the mapping drive. State `pausiert`, reason in `pause` (`/sprache/status`, `state.sprache`); the app shows "Mikrofon aus: Mapping läuft" ("microphone off: mapping running"). **Downside:** "Stopp" by voice does not work then either; stop via the app, `/map` or by hand. To disable: `SPEECH_OFF_WHILE_MAPPING = False`.
- **Live feed in the app:** Two switches in the "Interaktion" section. *Mitlesen* (read along) shows the level, Vosk's partial text and the last sentences with a verdict (command, rejected with reason, wake word, no command), via `GET /sprache/live`. *Mithören* (listen in) plays the microphone audio in the browser (WebSocket `/sprache/ton`, playback with the Web Audio API). Both only cost something on the robot while switched on. Meant for troubleshooting: you hear what arrives at the robot and see why a sentence did (not) count as a command.
- **Announcement ("Über den Roboter sprechen", speak through the robot):** Third switch. What you say into the microphone of a laptop or phone comes live out of the robot's speaker (WebSocket `/sprache/durchsage`, 16 kHz mono, delay about half a second). The announcement goes into the same queue as the voice's sentences: music pauses, the speech recognition stops listening (otherwise it would take the announcement as a command), a stop cancels it, and the volume is the same (`VOLUME`). After 2 s of silence it releases the speaker.
  - **Only over HTTPS:** Browsers only grant microphone access on secure pages. Under `http://10.42.0.62:8000` the app therefore shows a link to `https://10.42.0.62:8443`. The same app runs there with a certificate the server generates itself on first start. The browser warns once ("nicht sicher", not secure); confirm the warning.
  - Switching on *Mithören* at the same time causes echo: the robot's microphone hears its own speaker.
- **Failsafe after USB dropouts:** The microphone drops out too during an over-current storm. On an error message, the recognition searches again after `SPEECH_RETRY_S` (3 s); that already existed. New (03.11.) are three cases without an error message:
  - *Stream hangs:* After a dropout, `stream.read()` never returned. The level was frozen, the state stayed "hoert" (listening), and no more commands came until the server was restarted. Reading now happens via a callback into a queue with a deadline: if no block arrives for `SPEECH_HANG_S` (3 s), it reopens. If closing the dead stream hangs too, the stream is abandoned (`SPEECH_CLOSE_S`) instead of blocking the retry.
  - *Stream dead:* only exact zeros for `SPEECH_SILENT_S` (5 s) → reopen. A real microphone always has some noise.
  - *Wrong microphone:* After a dropout the C270 often came back before the USB microphone, and the recognition then ran on it. Since 03.11. it is not used at all any more; if the USB microphone is gone, recognition is off until it is back. If the recognition runs on a less preferred entry from `SPEECH_MICROPHONE_NAMES`, it keeps checking `/proc/asound/cards` every `SPEECH_MICROPHONE_CHECK_S` and switches back as soon as the better one is back.
  - *Listening thread stuck:* If even opening the microphone hangs in PortAudio, the deadlines above no longer help. A watchdog thread then starts a new listening thread after `SPEECH_THREAD_HANG_S` (15 s) without progress, at most `SPEECH_THREAD_MAX_RESTARTS` (3) times in a row. After that, `/sprache/status` says "Server neu starten" (restart the server).
  - *Listening in the app:* Without audio, the server sends a heartbeat every second. If nothing at all arrives for 5 s (Pi hangs, Wi-Fi gone, server restarted), the app reconnects by itself, until the robot answers again. *Mitlesen* polls with a 2 s deadline so a hanging request does not stop it.
  - Measured on 03.11.: After "Robi, folge mir" the microphone dropped out while it was speaking ("Stream hängt" at 09:22:51); 3 s later the recognition was listening again. The trigger is the power supply: when it speaks, the speaker draws current, and the microphone sometimes disconnects and reconnects eight times in eight seconds.
- **Execution:** The server fetches the commands every 0.1 s and uses the same paths as the app (`_handle`). If a command is rejected (e.g. camera missing), the warning appears in the app and it says "Das geht gerade nicht." ("That doesn't work right now.").

The phrases are German because the speech model is German; the English in
brackets is only an explanation.

| Say (examples) | Command | Effect |
|---|---|---|
| "Robi, folgen", "Robi, folge mir", "Robi, komm mit" (follow me, come along) | `folgen` | like `follow`: interaction on, it follows the nearest person |
| "Robi, komm her", "Robi, komm zu mir" (come here, come to me) | `herkommen` | interaction on (if off), drives towards the person |
| "Robi, geh weg", "Robi, zurück" (go away, back) | `weggehen` | interaction on (if off), backs off |
| "Stopp", "Halt", "Bleib stehen" (without Robi) | `stopp` | interaction on: stop, keep watching ("Okay, ich halte an."); otherwise emergency stop like `stop` |
| "Robi, fahr nach Hause", "Robi, heim" (go home) | `nach_hause` | like `home` |
| "Robi, erkunde", "Robi, schau dich um" (explore, look around) | `erkunden` | like `explore` |
| "Robi, Musik aus", "Robi, Ruhe" (music off, quiet) | `musik_aus` | stop MUKKE |
| "La da da", "La la la" (without Robi) | `ladada` | plays the song from `SPEECH_LADADA_URL` via MUKKE; if it is already playing, nothing happens |
| "Robi, Beatbox" | `beatbox` | plays the song from `SPEECH_BEATBOX_URL` via MUKKE; if it is already playing, nothing happens |
| "Hallo Robi", "Robi, wach auf" (hello, wake up) | `start` | interaction on (without driving); if already on: "Ja? Ich bin da." ("Yes? I'm here.") |
| "Robi, gute Nacht", "Robi, Feierabend" (good night, end of the day) | `aus` | cancel homing/explorer, interaction off, "Gute Nacht." |
| "Robi, fahr zwei Meter vor", "Robi, einen halben Meter nach links", "Robi, fahr fünfzig Zentimeter zurück" (two meters forward, half a meter left, fifty centimeters back) | `manoever` | drive exactly that far, measured by the lidar (like `move`, see [`api.md`](api.md)) |
| "Robi, dreh dich um neunzig Grad nach rechts", "Robi, dreh dich um" (180°), "Robi, dreh dich nach links" (90°) (turn ninety degrees right, turn around, turn left) | `manoever` | turn exactly by that angle (like `turn`); without a side, counter-clockwise |

Precise drive commands carry a number and are therefore not in `COMMANDS`;
instead they are parsed by `detect_maneuver()`. The whole sentence must
match ("fahr zwei Meter vor bitte", with a trailing "please", does not
count), so that word salad does not make it drive. Vosk only recognizes the
spelled-out numbers from `_maneuver_phrases()` (1–5 meters, half and one and
a half meters, 20/50 centimeters; 30–360 degrees). Typed (field "Hören",
`speech`), any number works, also "fahr 1,5 m vor". "Robi, fahr zwei Meter
zurück" is a drive command, "Robi, fahr zurück" stays go away. If a drive
command fails, it says the reason ("Zum Drehen ist es mir hier zu eng" –
too narrow to turn here, "Da ist etwas im Weg, ich habe angehalten" –
something is in the way, I stopped, "Fast geschafft, ich liege 2 Grad
daneben" – almost done, I'm 2 degrees off). Turning needs 30 cm of space
all around (`MANEUVER_TURN_FREE_M`).

The phrases are in `speech.py` (`COMMANDS`); their order there is the order
of precedence. That is why "Musik stopp" (music stop) comes before "Stopp".

**Trying it without a microphone:** In the app, the field "Hören" (command
`speech {text}`) takes text as if it had been spoken into the microphone.
This also works in `--mock`.

**Driving by voice does not need an open app.** That is intended (like a
call across the room). The watchdog's dead man's switch applies to manual
control; the behaviors drive with their own cycle and the lidar stop, as
with the explorer.

**Microphone and speaker:** MUKKE (mpv) and the voice (aplay) use the same
speaker, and the ALSA device can only be opened once. When the robot
speaks, MUKKE briefly ends mpv and resumes at the same position after the
last sentence (pausing alone does not release the device). The microphone
also hears the music; "Robi, Musik aus" helps.

- **Free text** will later go to `denken.py`, only after the wake word.
- **Output (implemented):** via the USB speaker with **Piper** (offline, female voice `de_DE-kerstin-low`, on one CPU core only). If Piper is missing, espeak-ng speaks; if that is missing too, it stays with the log. The last sentence spoken is in `state.stimme` and in the app. Setup: [`hardware.md`](hardware.md).
  - It briefly says what it is doing: "Ich folge dir." (I'm following you), "Ich komme." (I'm coming), "Da bin ich." (here I am), "Wo bist du?" (where are you?), "Okay, ich halte an." (okay, I'm stopping) and on emergency stop "Halt.". The sentences are in `behavior.py` (`_GESTURE_SAY` and the calls of `_switch(..., say=…)`).
  - **Soft start:** When switched on, the pose detection first warms up for `INTERACTION_STARTUP_S` (5 s), quietly and standing still. Then comes the start sentence ("Jetzt folge ich dir.", now I'll follow you), and it only drives once that is finished (`Voice.busy`). Requests during this time (speech, app) are remembered by the interaction (`_start_mode`) and executed afterwards. Reason: on 03.11. pose, announcement and motors started in the same second, the USB power supply collapsed, and the Pi rebooted.
  - At start it says "Ich bin bereit." (I'm ready). That way you hear right away whether the speaker works.
  - From the app: field "Sagen" in the interaction section (command `say`).
  - Identical sentences within `VOICE_REPEAT_S` are swallowed. The queue holds at most `VOICE_MAX_QUEUE` sentences; the oldest is dropped. Stop has priority and cuts off the current sentence.

## Mind: how the robot appears "conscious"

The robot has no consciousness. But it should appear that way in a
comprehensible manner. These four building blocks take care of that:

- **Inner state:**
  - energy (for now from the runtime, later from the battery),
  - curiosity (rises when nothing happens),
  - mood (praise raises it, being ignored lowers it),
  - attention (which person it is currently looking at).
- **Memory:**
  - short-term: the last ~50 events ("Person erschienen", "Geste Folgen", "Hindernis").
  - long-term: a small file `~/.rosmaster/erinnerung.json`.
- **Intention:** From gestures, speech and state, the mind chooses what happens next and starts or stops the behaviors.
- **Expression:**
  - Small movements: turn towards the person, briefly "wobble" when happy, back off when startled. They go through the watchdog and only when no behavior is driving.
  - The voice: it speaks answers and now and then says a thought out loud.
  - A **stream of thoughts** in the app makes the inner life visible.

`denken.py` receives the state and the latest events and returns JSON:
`{"gedanke": "...", "antwort": "...", "absicht": "folgen" | null}`.

Thoughts are only produced on events or every ~20 s. That keeps the API
costs low.

The API key comes from the environment variable `ANTHROPIC_API_KEY` and
**never** goes into the repo.

## Interface

Implemented and described in [`api.md`](api.md) ("Interaction in detail"):

- Commands `interaction_start` / `interaction_stop`, `follow {ziel_id?}` / `follow_pause` / `follow_resume` / `follow_stop`.
- `state.interaktion` with `aktiv`, `verfuegbar`, `modus`, `text`, `person{winkel, abstand}`, `personen`, `geste`, `grund`, `pausiert`, `ziel{zustand, id, sicherheit, grund}`.
- `GET /interaktion/status` and `GET /interaktion/pose` (skeleton for overlaying; the web app draws it over the camera image).
- Voice: commands `say {text, vorrang}` and `say_stop`, `state.stimme` with `verfuegbar`, `spricht`, `text`, diagnostics under `GET /stimme/status`.
- Speech: command `speech {text}`, `state.sprache` with `verfuegbar`, `geweckt`, `gehoert`, `befehl`, diagnostics under `GET /sprache/status`.

Planned for stages 4/5 (additions only): command `chat {text}`, fields `stimmung`, `aufmerksamkeit`, `gedanke`, `letzte_antwort`, message `type: "gedanke"`.

## Stages

| Stage | Content | Status |
|---|---|---|
| 1 | Docs (this file, project log, decisions E5/E6) | done |
| 2 | Perception: `pose.py`, `gestures.py`, skeleton in the app, **without driving** | done (mock + tests), Hailo still to be checked on the Pi |
| 3 | Behavior: follow/stop, come here/go away, **only on the floor** | done (mock), drive test on the floor open |
| 4 | Speech: Vosk commands, voice (Piper), chat in the app | voice and speech commands ("Robi …") done (mock + tests), still to be checked on the Pi; chat open |
| 5 | Mind and thinking: state, stream of thoughts, Claude/templates, expression | open |
| 6 | Later: local LLM on the Hailo, faces, Orbbec depth | – |

## Testing

- The gesture logic is checked without hardware, with synthetic keypoint sequences per gesture plus negative cases: `python Marlon/interaction/test_gestures.py`. The pattern is the same as in `test_obstacles.py`.
- Speech output (queue, priority, falling back to text, the behavior's sentences) is checked by `python Marlon/interaction/test_voice.py`, also without a speaker.
- The speech commands (wake word, wake window, precedence, grammar against the model's vocabulary, no crash without Vosk) are checked by `python Marlon/interaction/test_speech.py`, without microphone and without Vosk.
- `python main.py --mock` provides a simulated person who performs come here, go away, follow and stop one after another. In the app press "Interaktion an" and watch the messages.
- On the Pi:
  - `/interaktion/status` shows whether the Hailo is available, and the pose rate.
  - Meanwhile keep an eye on CPU load and `dmesg` so that no USB resets occur (see [`hardware.md`](hardware.md)).
