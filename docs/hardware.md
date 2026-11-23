# Hardware

## Inventory

| Component       | Model                 | Connection       | Status      |
|-----------------|-----------------------|------------------|-------------|
| Computer        | Raspberry Pi 5 (16 GB) + AI HAT+ 2 (Hailo-10H) | PCIe (`/dev/hailo0`) | present, Hailo planned for interaction |
| Chassis         | Yahboom ROSMASTER X3  | USB serial       | present     |
| Expansion Board | Yahboom (STM32)       | USB serial       | present     |
| Lidar           | Slamtec RPLIDAR A1    | USB serial       | integrated (localization + obstacles) |
| Depth camera    | Orbbec (color + depth channel) | USB (via hub) | color image integrated (camera `orientierung`), depth since 04.11. (obstacles during autonomous driving) |
| Webcam          | Logitech C270 (front, tilted slightly up) | USB, own bus (directly on the Pi) | integrated (camera `personen`: person detection, gestures) |
| Microphone      | USB microphone (FongLun, C-Media `0d8c:0016`); more in the C270 and Orbbec | USB audio, own bus | integrated (voice commands "Robi …", `SPEECH_MICROPHONE`) |
| Speaker         | USB speaker "UACDemoV1.0" | USB audio, directly on the Pi | integrated (MUKKE: music from YouTube; speech output, `VOICE_DEVICE`) |
| ESP32           | *(enter model)*       | Wi-Fi            | planned     |
| Pulse sensor    | *(enter model)*       | ESP32            | planned     |
| Box motor       | *(enter model)*       | servo on the X3  | planned     |

> Please enter the model names here when installing parts. Without them it
> is impossible to tell later which driver is needed.

## What is connected via USB

Output of `lsusb` on this robot, explained:

| USB ID | Device | Purpose |
|---|---|---|
| `1a86:7523` | QinHeng CH340 | **Expansion Board**: drive |
| `10c4:ea60` | Silicon Labs CP210x | **RPLIDAR A1** |
| `2bc5:050f` | Orbbec "USB 2.0 Camera" | RGB image, camera `orientierung` |
| `2bc5:060f` | Orbbec Depth Sensor | depth channel of the same camera |
| `046d:0825` | Logitech C270 HD WebCam | camera `personen`, microphone (own USB bus, not on the hub) |
| `0d8c:0016` | USB microphone (C-Media) | speech input (ALSA "Microphone") |
| `4c4a:4155` | Jieli "UACDemoV1.0" | USB speaker, speech output (ALSA "UACDemoV10"), not a camera |

**The most important consequence:** there are **two** USB serial adapters.
Board and lidar both show up as `/dev/ttyUSB*`, and which one gets which
number depends on the power-on order. If you leave the search to
`Rosmaster_Lib`, it may pick the lidar. Then nothing moves, and no error
message appears.

So set the port explicitly:

```bash
ls -l /dev/serial/by-id/
```

Put the entry containing `1a86` or `CH340` into `config.py` as
`SERIAL_PORT`. The lidar already has a fixed name, `/dev/rplidar`.

The **Orbbec depth camera** shows up as two USB devices. Its RGB channel is
used as an ordinary USB camera. The depth channel is not a video device (it
does not appear in `v4l2-ctl --list-devices`); it runs via Orbbec's
OpenNI2, see [Depth camera](#depth-camera-orbbec-astra).

Since 28.10. a **Logitech C270** is additionally mounted at the front. It is
connected directly to the Pi, not to the hub: the hub already carries the
board, lidar and Orbbec, and another camera there makes voltage drops more
likely. When it is plugged in, the kernel reports a few
`Failed to query (GET_INFO) UVC control …` and `cannot get freq`. That is
normal and harmless for the C270.

The `/dev/video*` numbers depend on the plug-in order. The server therefore
assigns the cameras by their USB ID, not by number (see [Camera](#camera)).

## Drive

The X3 has **mecanum wheels**. This lets it drive sideways without turning.
Drive commands therefore have three values instead of two:

| Value   | Meaning                      | Unit    |
|---------|------------------------------|---------|
| `vx`    | forward (+) / backward (−)   | m/s     |
| `vy`    | left (+) / right (−)         | m/s     |
| `omega` | turn counterclockwise (+) / clockwise (−) | rad/s |

Limits and default speed are in `robot/src/rosmaster_assist/config.py`.

## Rosmaster_Lib

Yahboom's Python driver talks to the Expansion Board via USB serial.

```python
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()                     # default: /dev/myserial or /dev/ttyUSB0
bot.create_receive_threading()        # starts the receive thread
bot.set_car_motion(vx, vy, omega)     # drive
bot.set_beep(100)                     # buzzer, duration in ms
bot.set_colorful_lamps(0xFF, r, g, b) # LED strip
```

If the library is missing, it is part of the Yahboom delivery or in the
official Yahboom GitHub repository for the ROSMASTER.

**Serial port.** The device name differs depending on the image and the
connection type (`/dev/myserial`, `/dev/ttyUSB0`, `/dev/ttyAMA0`, …). Usually
that does not matter: **`Rosmaster()` without an argument finds the board by
itself.** That is why `SERIAL_PORT` in `config.py` is `None` and should stay
that way unless there is a reason against it.

A fixed port is only needed if the automatic search picks the wrong device,
which realistically happens when the RPLIDAR is on the same bus. In that
case enter the `by-id` path, not `/dev/ttyUSB0` (see the lidar section
below).

Whether the board is reachable is answered by `python scripts/check_hardware.py`
under "Expansion Board". It actually tries to open the board instead of just
looking for device files.

**Only one program per port.** Technically the serial port can be opened
several times, but it cannot be shared sensibly: two readers each get
fragments. Typical message:

```
SerialException: device reports readiness to read but returned no data
(device disconnected or multiple access on port?)
```

To see who else is reading:

```bash
ps aux | grep -i python | grep -v grep
sudo fuser -v /dev/ttyUSB* /dev/ttyAMA* /dev/myserial 2>&1
```

The usual suspects:

- **The ROS2 stack.** `yahboomcar_bringup` opens the same board via the same
  library. The most common cause for us:

  ```bash
  sudo pkill -f yahboomcar_bringup
  sudo pkill -f x3_explorer       # otherwise drives on its own in parallel
  sudo pkill -f sllidar_node      # occupies /dev/rplidar
  ```

  Either ROS2 **or** our server, never both.
- **Yahboom app** (`app.py` with Flask, MakerControl). Starts at boot on
  Yahboom images. `sudo systemctl stop <name>`, additionally `disable` if
  needed.
- **Serial console** on the GPIO UART. Switch it off via
  `sudo raspi-config` → *Interface Options* → *Serial Port* → login shell
  **no**, hardware **yes**.
- **ModemManager**, which automatically probes new USB serial devices:
  `sudo systemctl disable --now ModemManager`

As a workaround, `TELEMETRY_ENABLED = False` can be set in `config.py`. Then
our software does not read at all and the conflict disappears. Driving
still works, but battery and IMU values are missing.

**Permissions.** If access fails with "Permission denied":

```bash
sudo usermod -aG dialout $USER   # log in again afterwards
```

## Lidar: Slamtec RPLIDAR A1

360° laser scanner, range about 12 m, around 8000 measurement points per
second at 5.5 revolutions/s. For each point it delivers angle, distance and
signal quality, nothing more. Building a map from that is the software's
job.

**Connection.** The A1 is connected to the Pi via a USB serial adapter
(CP2102) and runs at **115200 baud**. This Pi already has the fixed path
**`/dev/rplidar`** (udev rule from the Yahboom setup). Use it, not
`/dev/ttyUSB0`. The motor is controlled via the DTR line; if the scanner
does not spin, that is usually the reason.

**Integrated.** The lidar is read directly from Python, without ROS2
(rationale: E4 in [`decisions.md`](decisions.md)). This requires:

```bash
robot/.venv/bin/pip install rplidar-roboticia
```

**ROS2 must not run at the same time.** `sllidar_ros2` occupies the same
port. It is one or the other:

```bash
sudo pkill -f sllidar_node
```

Check the state: `http://<IP-of-the-Pi>:8000/lidar/status`. It shows how many
revolutions have arrived, whether the start point has been calibrated and
how well the match fits (`letztes_residuum_m`; below 5 cm is good).

**⚠ Port mix-up: important.** The X3 Expansion Board *also* uses a USB serial
port. With two devices, Linux assigns `/dev/ttyUSB0` and `/dev/ttyUSB1` in
the order in which they are detected, and that order can change on every
reboot. Then the drive suddenly receives lidar commands.

So use fixed names based on the device ID instead of `ttyUSB0`:

```bash
ls -l /dev/serial/by-id/
# -> /dev/serial/by-id/usb-Silicon_Labs_CP2102_...-if00-port0
```

This path stays the same regardless of the boot order. Enter the Expansion
Board's path in `config.py` as `SERIAL_PORT`, and the lidar's path later in
the lidar driver accordingly.

**Power supply.** The A1 briefly draws noticeable current when its motor
starts. If it is connected to the Pi through an unpowered USB hub, dropouts
can occur. When in doubt, connect it directly to the Pi.

**Library.** For pure Python, `rplidar-roboticia` is an option (a fork of
the no longer maintained `rplidar`). Not yet chosen; see the open points
in `robot/src/rosmaster_assist/sensors/__init__.py`.

## Camera

The software runs **all** cameras it finds, each with its own image:

| Variant | Requirement |
|---|---|
| USB cameras | `pip install opencv-python` |
| CSI camera on the ribbon cable | `picamera2` (included in Raspberry Pi OS) |
| Test image if there is no camera at all | nothing, runs everywhere |

Which USB camera is used for what is set in `config.py` in `CAMERA_ROLES`:
`orientierung` is the Orbbec (USB ID `2bc5`), `personen` the Logitech
(`046d`). Assignment is by USB ID, not by `/dev/videoN`, because the numbers
get swapped after a reboot or re-plugging. If one of these cameras is
missing, the server keeps looking for it in the background. Further USB
cameras run as `usb1`, `usb2`, …

At startup the log shows what was found (a line "Cameras: orientierung (…),
personen (…)" and, per camera, "Camera '…' (USB …, /dev/videoN) ready").
Without the log: open `/cameras` in the browser.

Resolution, frame rate and JPEG quality can be changed in the settings (gear
icon at the top right of the app); the default values are in
`robot/src/rosmaster_assist/config.py`. If the image stutters over Wi-Fi,
lower `CAMERA_FPS` first, and only then the resolution.

**Check a USB camera:**

```bash
v4l2-ctl --list-devices # one block per camera; if it is missing here, it is not detected
lsusb                   # USB ID, must match CAMERA_ROLES
pip install opencv-python
```

**Check a CSI camera:**

```bash
rpicam-hello --list-cameras
```

The stream is at `/camera/<name>/stream` (MJPEG) and a single frame at
`/camera/<name>/snapshot`, e.g. `/camera/personen/stream`. Both can be opened
directly in the browser, even without the app. `/camera/stream` without a
name shows the default camera (`orientierung`).

## Depth camera (Orbbec Astra)

Since 04.11. autonomous driving uses the depth channel (`sensors/depth.py`)
to see what lies below the lidar plane: chair and table legs, boxes, bags.
The robot used to get stuck on a chair leg.

**Setup (once on the Pi):** `bash scripts/setup_orbbec.sh`.
The script copies `libOpenNI2.so` and the Orbbec driver from the Yahboom
container to `~/orbbec_openni2` (`DEPTH_OPENNI_PATH`). The pip package
`openni` does not run under Python 3.13, so there is a small ctypes binding
of our own. Yahboom's udev rule `56-orbbec-usb.rules` allows access without
root. If the library is missing, the server keeps running without depth and
reports this in the log and under `/tiefe/status`.

**What you need to know:**

- The Astra only measures from **~0.6 m** (`DEPTH_MIN_M`). Anything closer
  the robot only knows from memory, i.e. from earlier, when it was farther
  away. A chair placed right next to it stays invisible. The progress watch
  helps against that (see `docs/api.md`).
- Frames are read at 320×240. 8 frames/s are evaluated (`DEPTH_HZ`); the
  camera delivers 30.
- An obstacle is anything 4–40 cm above the **floor**. The floor is fitted
  as a plane in every frame. This compensates for the mounting position
  (`DEPTH_CAMERA_HEIGHT_M` and `…_TILT_DEG` are only starting values). If
  no floor is found (dark or reflective floor, robot jacked up), only
  points from 9 cm height upward and only up to 1.3 m count.
- **Power:** the projector is one of the biggest consumers on the hub (see
  USB resets below). That is why the camera is only on during autonomous
  drives and closed otherwise. If it drops off USB, it reconnects after
  `DEPTH_RETRY_S`. Meanwhile the drive continues with the lidar alone.
- The depth points deliberately do **not** go into the map (SLAM). The map
  stays a lidar map; the points are only used for avoidance.
- Viewing: in the app via the camera switcher "Tiefe" or "Tiefe + Farbbild"
  (depth, or depth over the color image; or `/tiefe/stream` in the
  browser), colors see `docs/api.md`.
- Check while standing: `curl -X POST <IP>:8000/tiefe/test`, then
  `/tiefe/status`. Standing on the floor, `boden_erkannt` should be true and
  `kamera_hoehe_m` should show roughly the real height. To check left/right,
  hold a hand to the left in front of the camera (more than 60 cm away):
  `naechstes` must then show a positive y (convention: y to the left).
- **Camera hangs** (04.11.): `/tiefe/status` reports "Gerät öffnen: … Failed
  to open the file!", then "Resource busy!". The firmware registers on USB
  but no longer answers commands. A USB reset via software does not help,
  only **unplugging and re-plugging the Orbbec** (cutting its power).
  Previously the whole server crashed in this case (segfault in OpenNI2,
  because the handle of a failed open was closed). Now it keeps running,
  and the drive continues with the lidar alone.

## Speaker and music ("MUKKE")

The USB speaker is called `UACDemoV10` in ALSA. The app plays YouTube links
through it (button "MUKKE"). One-time setup on the Pi:

```bash
sudo apt install mpv
robot/.venv/bin/pip install -U yt-dlp
aplay -l                      # the speaker must show up as "UACDemoV10"
speaker-test -D plughw:CARD=UACDemoV10,DEV=0 -c 2 -t sine -l 1   # test tone
```

yt-dlp from pip, not from apt: YouTube keeps changing things, and the
version from the package sources is usually too old. If suddenly nothing
plays any more, `robot/.venv/bin/pip install -U yt-dlp` almost always helps.

The output device is set in `config.py` as `MUKKE_AUDIO_DEVICE`. If there is
no sound, the MUKKE window shows the reason: the error message from YouTube
(yt-dlp) if the link cannot be resolved, otherwise the last message from
mpv. Cross-check on the Pi: `robot/.venv/bin/yt-dlp -f bestaudio -g "<Link>"`.

## Speaker and speech output

The USB speaker registers as ALSA card **"UACDemoV10"**. The server
addresses it by name (`VOICE_DEVICE =
"plughw:CARD=UACDemoV10,DEV=0"`), not by card number. The number changes
when devices re-register after a USB reset; the name stays the same.
`plughw` converts sample rate and format as needed.

Setup on the Pi (once):

```bash
sudo apt install alsa-utils espeak-ng      # aplay/amixer, fallback voice
aplay -L | grep UACDemo                    # speaker present?

# Piper: unpack the aarch64 release to ~/piper/piper/ (the server looks
# there first, see VOICE_PIPER), plus the female voice Kerstin.
mkdir -p ~/piper && cd ~/piper
curl -fsSL https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_aarch64.tar.gz | tar xz
B=https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/de/de_DE/kerstin/low
curl -fsSLO $B/de_DE-kerstin-low.onnx && curl -fsSLO $B/de_DE-kerstin-low.onnx.json

# Test without the server (Kerstin uses 16 kHz, as stated in the .onnx.json):
echo "Hallo, ich bin Robi." \
  | taskset -c 3 ~/piper/piper/piper --model ~/piper/de_DE-kerstin-low.onnx --output-raw \
  | aplay -D plughw:CARD=UACDemoV10,DEV=0 -r 16000 -f S16_LE -c 1 -t raw
```

The test sentence ("Hello, I am Robi.") stays German because the voice is
German.

- Set up on the Pi on 03.11.; Thorsten (male, `de_DE-thorsten-medium`) is
  also installed there. Switch via `VOICE_MODEL`. Measured for 3.5 s of
  speech: Kerstin 1.3 s, Thorsten medium 1.1 s, Thorsten high 5.7 s (slower
  than real time, therefore not used).
- **Piper computes on one core only** (`VOICE_CORES`, via `taskset`). On all
  four cores, on 03.11. all USB devices dropped off exactly while speaking
  (over-current, `aplay: No such device`). With one core Piper is hardly
  slower, and a test sentence at 70 % ran without a new over-current message.
- **Do not start with `sudo`.** Under sudo, `~` is `/root`, so Piper, the
  Vosk model and `~/.rosmaster` are missing (the robot then spoke with
  espeak again). `main.py` now redirects this to the actual user by itself,
  but sudo is not needed.
- Piper is a system program, not a pip dependency. If it (or the model) is
  missing, the server falls back to espeak-ng and reports this in the log.
  If that is missing too, it does not speak but otherwise runs normally.
- The server sets the mixer via `amixer` to a fixed `SOUND_MIXER` (100 %),
  again before every sequence of sentences, because after a USB dropout the
  speaker comes back with its default value. Lower volume is done in
  software (`VOLUME`, settings page): the server scales Piper audio down
  itself, espeak gets `-a`, mpv `--volume`. At startup it says "Ich bin
  bereit." ("I am ready."). If you hear that, the whole chain works.
- Diagnostics: `curl localhost:8000/stimme/status`. `ausgabe` shows which
  voice is used and `fehler` the last message from aplay (e.g.
  "No such device" when the speaker is missing).
- After a USB dropout, at most the sentence currently playing is lost. The
  next one finds the speaker again under the same name.

## Microphone and voice commands

Listening uses the USB microphone (FongLun or C-Media `0d8c:0016`). The
server looks it up by name (`SPEECH_MICROPHONE_NAMES`: "USB Microphone",
"USB PnP", "C-Media", "Microphone", in that order), not by number. It never
uses the microphones in the C270 and Orbbec (`SPEECH_MICROPHONE_NEVER`): the
C270's sounds so bad that Vosk hardly understands anything. Recognition uses
Vosk with the small German model (about 45 MB, runs on the CPU).

Setup on the Pi: `python main.py` does this by itself when starting with real
hardware (packages `vosk`/`sounddevice` and the model to `~`, needs
internet once). Only PortAudio remains manual, because it needs `sudo`;
`main.py` points this out if it is missing. To reproduce by hand:

```bash
sudo apt install libportaudio2             # for sounddevice
cd ~/gitpipeline/SPE-ROSMASTER-X3
robot/.venv/bin/pip install -e "robot[sprache]"   # vosk + sounddevice

cd ~ && wget https://alphacephei.com/vosk/models/vosk-model-small-de-0.15.zip
unzip vosk-model-small-de-0.15.zip         # -> ~/vosk-model-small-de-0.15 (SPEECH_MODEL)

arecord -l                                 # microphone present?
arecord -D plughw:CARD=Device,DEV=0 -d 3 -f S16_LE -r 16000 test.wav && aplay test.wav
```

- The card name in the `arecord` test may be different; `arecord -l` shows
  it. The server does not need it.
- The small model has no `words.txt`; the server asks Vosk directly which
  wake word variants it knows (currently "robbie", "ruby", "robin").
- If something is missing, the server keeps running without voice commands.
  `curl localhost:8000/sprache/status` shows `zustand` and `fehler`.
- After a USB dropout, recognition tries to reopen the microphone every
  `SPEECH_RETRY_S` (3 s).
- Speaker and microphone sit close together. While the robot is speaking,
  the microphone input is discarded. It does hear music (MUKKE), though;
  "Robi, Musik aus" ("Robi, music off") usually still gets through.

### Speech filter (mic tests 05.11.)

The microphone carries only speech commands, so every block passes
`interaction/audio_filter.py` (`SpeechFilter`) before Vosk:

1. band-pass 200–4500 Hz,
2. noise suppression,
3. automatic gain on speech frames only,
4. a soft limiter.

The settings are `SPEECH_FILTER_*`, and `SPEECH_FILTER_ENABLED = False`
falls back to raw audio. `/sprache/status` shows `filter` with
`verstaerkung_db`, `snr_db` and `uebersteuert` (share of clipped samples).
Microphone test runs save both the filtered (`ton.wav`) and the raw
(`ton_roh.wav`) recording, so clipping and filter settings can be checked
offline.

What the three runs `mikro_pegel`, `mikro_befehle` and `mikro_fahrt`
showed:

- **Hum:** a constant hum at 46/52 Hz and its harmonics at about
  −19 dBFS. It is louder than the voice, which sat at −28 to −35 dBFS in
  the speech band.
- **Recognition (offline replay through Vosk, 18 command phases,
  hit = confidence ≥ 0.7):**

  | Input | Hits |
  |---|---|
  | raw | 0 (1 weak) |
  | band-pass + gain only | 0 |
  | filter with floor 0.1 | 1 |
  | filter with floor 0.3 (chosen) | 3–4 |

  None of these variants triggered on the 7 phases without a command
  (silence, background talk, driving).
- **Output levels with the filter:** silence drops from −19 to about
  −48 dBFS, and speech peaks reach about −20 dBFS.
- **Driving fast or turning:** the raw signal clips at −1 to −3 dBFS. No
  software can undo clipping. **Lower the ALSA capture gain** of the USB
  microphone (`alsamixer` → F4 "Capture", or `amixer -c <card> scontrols`
  for the control name) until `uebersteuert` stays at 0 while driving.
  Then speak closer: the commands from a distance were mostly not heard.
- **Remaining limits:** the small model is unsure about "stopp" when
  spoken quietly (confidence 0.4–0.5), and it often hears the command word
  as a second "ruby"/"robbie".

## AI HAT+

> **Addendum 28.10.:** The installed unit is an **AI HAT+ 2 with Hailo-10H**
> (PCIe, `/dev/hailo0`, firmware 5.1.1, 40 TOPS, 8 GB own memory). HailoRT
> 5.1.1 is installed; `hailo_platform` can be imported in the venv.
> Bundled models are in `/usr/share/hailo-models/`, among them
> `yolov8s_pose_h10.hef` (body pose, 17 points) and `yolov8m_h10.hef`
> (objects). The Hailo-10H can also run small language models (Hailo GenAI,
> not yet installed). Planned use: gesture recognition via the body pose,
> see [`interaction.md`](interaction.md) and E5 in
> [`decisions.md`](decisions.md).

The AI HAT+ contains a Hailo accelerator. It is interesting for later
camera analysis (person detection, fall detection) and is not yet used in
the current base framework. Whether person detection runs via the camera or
via the ESP is still open; the AI HAT+ would be the much more reliable
approach.

## USB resets (findings from 28.10.)

Devices on the USB hub (lidar, board, Orbbec) occasionally drop off at the
same time. `dmesg` then shows "over-current" on all ports. The cause is
**CPU load spikes at 2.4 GHz**, probably combined with backfeed of the hub
via the Expansion Board. An empty battery is not the cause.

- At 2.0 GHz the resets occur much less often, but they do not go away
  completely. **Since 28.10. `arm_freq=2000` is therefore set in
  `/boot/firmware/config.txt`** (agreed with Ben, backup
  `config.txt.vor-arm_freq-2026-10-28`). The server needs no changes for
  this: all loops run by wall-clock time, not by computing time, and the
  most expensive parts (person detection 2 Hz, JPEG 15 fps) have enough
  headroom.
- `python scripts/check_hardware.py` shows clock, temperature and throttling
  and warns if the limit is missing.
- If the Pi gets hot (above ~80 °C, e.g. while compiling), it throttles
  itself to 1.5 GHz and the USB devices drop off en masse. So do not let the
  robot drive while compiling.
- The real fix is in hardware: a hub without backfeed or a separated VBUS
  line.
- The C270 is on its own *data bus*, but **not on its own power supply**. On
  the Pi 5 all four sockets share one power switch (1.6 A total). If it
  trips, all ports report "over-current" at the same time, and during a
  longer overload the C270 drops off too (as happened on 28.10.).

**Endless loop.** Once the supply has collapsed, it can sustain itself: when
re-registering, the Orbbec and the lidar motor draw inrush current, the
switch trips again, and so on. `dmesg` then shows `cannot reset (err = -71)`
on the hub ports. Usually the loop ends once the cause is gone (load/heat);
otherwise switch the robot off once.

**Quick check before every test** (`python scripts/check_hardware.py`):

| Check | Good | Bad → do |
|---|---|---|
| USB supply | `[ok]`, no new messages in 3 s | new over-current messages → do not drive, find the cause (next lines) |
| Is the Pi installing/compiling? | no warning | `dpkg`/`apt`/`dkms`/`cc1` running → wait or cancel, do not drive |
| Temperature | below ~70 °C | above 80 °C → check the fan, reduce load |
| Clock | 2000 MHz | higher → `arm_freq=2000` missing in `config.txt` |

On 28.10. a pointless nvidia driver build (DKMS) ran twice in the
background, triggered by a hanging `dpkg --configure -a`. The nvidia
packages have been removed since. Anyone installing something on the Pi
with `apt`, please do not do it during a test, and check `dpkg --audit`
afterwards (empty = good).

**Stress test on 28.10. (battery full at 12.0 V, Orbbec unplugged).** At
rest (lidar, board, speaker) there was no message for minutes. While
driving there were 160 over-current messages and 83 USB disconnects in
4.5 min, and **every one started 0–5 s after the start of a turn** (omega
±0.6, all four motors start at the same time). Driving straight at 0.1 m/s
stayed quiet. Temperature 55–62 °C, no throttling. So the trigger is the
inrush current of the motors. It pulls down the 5 V supply that the Pi or
the hub hang on. Points 1 and 2 below are therefore not enough, **point 4
is necessary**.

The software survives the dropouts (board and lidar reconnect, the board
always via `/dev/myserial`, never via the lidar port), but the robot does
not drive reliably like this.

The lidar can also drive the loop while standing still: its motor starts
after every reconnect. That is why the server waits longer and longer if
the lidar fails again shortly after reconnecting (`LIDAR_STABLE_S`). If the
lidar does not register at all any more (`dmesg`: repeated "new full-speed
USB device" on 1-1.4), re-plug the lidar cable.

**Cross-checks on 29.10. on the stand.** The robot stood on a stand, the
wheels spun freely in the air, i.e. without load. Same test as on 28.10.:
5 min, of which 210 s driving with the same turns.

| Supply | Devices | Over-current / disconnects |
|---|---|---|
| Lab power supply | Orbbec, without C270 | 0 / 0 |
| Battery, 100 % (12.3 V) | Orbbec and C270 (images streaming) | 0 / 0 |

Lidar and camera did not reconnect a single time in either run, 51–53 °C,
no throttling. On 28.10. the robot was not on the stand, so the wheels had
load: +160 messages in 4.5 min. **So without load the battery is enough
too.** The resets come from the motors' inrush current *under load*, which
is considerably higher than with freely spinning wheels. Whether a power
supply can absorb that will only be shown by a test on the floor. Until
then, point 4 below remains the safe route.

**Over-current when stopping the server (fixed 29.10.).** Twice, all USB
devices dropped off for about 2 s exactly when the server was stopped.
Cause: the pose thread (`sensors/pose.py`) was woken by `stop()` and then
still opened the Hailo instead of exiting. Starting up the Hailo ("Creating
vdevice" in the HailoRT log, in the same second as the overload) briefly
draws so much current that the shared 1.6 A switch trips. Now the thread
first checks after waking up whether it should stop. Checked twice on the
Pi: no more messages when stopping. This also means: opening the Hailo is a
load spike. When switching on interaction while driving, watch for new
messages.

**"Random crashes" with the other board (findings from 29.10.).** After
switching to the other Expansion Board, seemingly random crashes continued,
on the power supply and on the stand. Measured via SSH, **without the
server running** and without any wheel turning:

| Measurement | Value |
|---|---|
| `vcgencmd get_throttled` | `0x0` (never undervoltage, never throttled) |
| Pi supply (PMIC `EXT5V`) | 5.05–5.08 V |
| Temperature, free RAM | 47 °C, 14 GB free |
| Over-current messages at idle | **49–65 per 10 s**, continuously |
| Counter `over_current_count` | all 6 root ports count up (160–325), the hub ports stay at 0 |
| Onset | within the first second after booting |
| Lidar | does not register (`device descriptor read … error -32`, plus `-71`), `/dev/rplidar` missing |

Conclusions:

- **The code is not the cause.** The overload is already running before the
  server starts, and without any computing or driving load. The Pi itself
  has enough voltage and is cool.
- That *all* root ports count matches the shared power switch of the Pi 5
  (see above): one device or cable draws too much or feeds back, and the
  switch reports it for all. Suspects are the hub on the board (VIA 2109,
  backfeed via VBUS) and the lidar or its cable, which does not even
  register.
- Every overload briefly disconnects the devices. To the server this looks
  like a crash: lidar, board and camera disappear, the app shows "Keine
  Verbindung zum Antrieb" ("no connection to the drive").

**Narrowing it down (on the robot, in this order).** After each step, check
the value *USB-Überlast pro Minute* (USB overload per minute) on `/diagnose`
under "Abstürze & Hänger" (crashes & hangs), or use `dmesg -w`:

1. Unplug all USB devices from the Pi. If the overload stops, a device is
   the cause; if it keeps counting, it is the Pi or its supply.
2. Plug the devices back in one by one, the lidar last. The device after
   which the counters start running again is the culprit.
3. Test the lidar with a different cable or on the other adapter.
4. Unplug the USB cable from the board to the Pi (rules out backfeed via the
   board). If it then stays quiet, implement point 4 below.

So that the kernel log is still there after a reboot next time, make the
journal persistent (on the Pi it was only in RAM):

```bash
sudo mkdir -p /var/log/journal/$(cat /etc/machine-id)
sudo systemd-tmpfiles --create --prefix /var/log/journal
sudo systemctl restart systemd-journald
```

The server also records things itself (`~/.rosmaster/diagnose/`): every run
with boot ID (`laeufe.json`), the log as a file (`server.log`), segfaults
(`absturz.txt`) and hangs of the event loop (`blockiert.txt`). On the next
start, the `/diagnose` page classifies whether the Pi rebooted, the server
was killed hard, or it crashed.

**Findings from 03.11. (standing still, Orbbec plugged in again).** Over 400
over-current messages in 15 min, always all root ports at the same time.
There was no undervoltage (`throttled=0x0`, EXT5V 5.09 V), and
`usb_max_current_enable=1` and `PSU_MAX_CURRENT=5000` were already set. Even
during boot there were 24 messages. The first collapse during the server
run came at startup, where the Hailo, lidar motor, cameras and the first
announcement all start in the same second. After that an endless loop ran.
Conclusion: as above, point 4 is necessary; until then unplug the Orbbec.
When reading the logs, note: the Pi has no real-time clock, so until network
time is available the log shows old timestamps.

**The hub.** The USB hub on the X3 is a VIA VL817 (`2109:2817` USB 2,
`2109:0817` USB 3, firmware 0.50). According to its descriptor it is *self
powered* (0 mA from the Pi), so it is supplied via the Expansion Board. We
were told it is the old version V1 with known power problems. That matches
the pattern above: a self-powered hub whose rail collapses or feeds back
makes the Pi's shared switch trip on all ports, including empty ones and the
C270, which is not on the hub. The Genesys hub (`05e3:0608`) behind it is the
Orbbec's internal hub, not a second hub. Cross-check: remove the hub,
connect board and lidar directly to the Pi (Orbbec unplugged). If the
messages disappear, the hub is the cause.

**The driver board (Expansion Board).** We were told it is version V1.
According to Yahboom, only V3.0 supports the Pi 5 power protocol (5.1 V/5 A
via Type-C); V1.0 only supplies "5 V" without stating the current. Confirmed
on the Pi (03.11.): `/proc/device-tree/chosen/power/usbpd_power_data_objects`
is all 0, nothing was negotiated. The Pi only "knows" the 5 A because
`PSU_MAX_CURRENT=5000` was set by hand in the EEPROM. With that it allows
1.6 A for USB, which the V1 board cannot reliably deliver under load, and on
every load spike the switch trips. Board firmware: 2.4 (`get_version()`).

A **firmware update does not help here.** The firmware runs on the STM32
(motors, IMU, servos); the 5 V supply and the Pi 5 protocol are hardware
(voltage regulator and protocol chip of V3.0). Yahboom advises against
flashing without need (Windows tool mcuisp/FlyMcu, BOOT0 + RESET by hand),
and new firmware may change the protocol to `Rosmaster_Lib` and our driver.
Remedy: board V3.0, or supply the Pi from the battery via a separate
5 V/5 A converter with the Pi 5 protocol (USB PD).

### What to do on the robot (hardware)

Sorted by effect, simplest first:

1. **Charge the battery.** During the sensor check it was at 10.1–10.3 V,
   i.e. almost empty (3S battery). Hub and board are probably supplied from
   the battery via the Expansion Board. If this rail sags, the devices draw
   their current through the USB cable from the Pi, and the 1.6 A switch
   trips. This is a hypothesis, but it is easy to check: test with a full
   battery. *Checked on 28.10.: with a full battery the resets during turns
   remain.*
2. **Unplug the Orbbec while nobody uses it.** It is the biggest consumer on
   the hub (depth sensor with projector, own internal hub), and our code
   does not use it. *Unplugged since 28.10., not enough on its own.*
3. **Only necessary devices directly on the Pi.** Everything on the four Pi
   sockets shares 1.6 A. Currently these are the C270, speaker, USB
   microphone and the hub. The C270 has its own microphone. If the resets
   continue, try unplugging the additional microphone and not turning the
   speaker all the way up.
4. **Power the hub properly (the real fix):** use an active hub with its own
   5 V power supply, on the robot fed from the battery via a DC/DC converter
   (5 V, at least 3 A). The hub must not feed back. If the hub is already
   powered via the board, an alternative is to cut the 5 V wire (VBUS, red)
   in the cable from the Pi to the hub, e.g. with a "data only" USB adapter.
   Then no current flows between Pi and hub any more.
5. **Board and lidar** directly on the (powered) hub, not via the Orbbec or
   a hub cascade.
6. **Check cooling:** the Pi's fan must run. 83 °C while compiling indicates
   insufficient cooling.

Measurements and derivation: `Marlon/sensorcheck_2026-10-28/SUMMARY.md`.
