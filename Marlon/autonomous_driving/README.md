# ROSMASTER X3 – autonomous exploring with lidar

After starting, the robot drives around on its own, avoids obstacles using the
RPLIDAR and uses the mecanum wheels to glide sideways past obstacles.

## What it does

| State       | Behavior                                               | LED        |
|-------------|--------------------------------------------------------|------------|
| UMSCHAUEN (look around) | at startup, turn slowly 360° once, then set off in the most open direction | color gradient |
| ERKUNDEN (explore) | drives smoothly, prefers areas it has not visited yet, keeps its distance from walls, brakes gently in front of obstacles | running light |
| SCHAUEN (glance) | every 40–80 s: stop, "look around" left/right, choose a new direction | starlight |
| AUSWEG (way out) | dead end: beep, back up a bit, turn towards the largest gap | breathing |

Technique: in every tick (20 Hz), 72 driving directions are evaluated. Each is
checked as a robot-wide corridor, not just as a single laser beam.
Accelerations are limited, which is why the motion looks "deliberate" rather
than jerky.

## Starting – one command

On the Pi (user `pi`, **not** inside the Docker container), in the repo folder:

```bash
cd ~/gitpipeline/SPE-ROSMASTER-X3   # or wherever the repo is
git pull
bash Marlon/autonomous_driving/x3.sh              # container + driver + lidar + drive off
```
**Stop: Ctrl+C**. The robot is stopped and all processes are terminated.

`x3.sh` does everything by itself:
1. stops the Yahboom factory app (`rosmaster_main.py`), which otherwise occupies the serial ports,
2. starts the ROS container `x3` (image `yahboomtechnology/ros-foxy:4.0.8`) with
   `-v /dev:/dev`, `--ipc=host` and this repo under the same path,
3. runs `start_autonomy.sh` inside it (driver → wait 8 s → lidar → `x3_explorer.py`).

| Command | Effect |
|--------|---------|
| `bash Marlon/autonomous_driving/x3.sh` | drive off (max. 0.30 m/s) |
| `bash Marlon/autonomous_driving/x3.sh --max-v 0.2` | slower; all options are passed to `x3_explorer.py` |
| `bash Marlon/autonomous_driving/x3.sh --calibrate` | check the lidar alignment, does **not** drive |
| `bash Marlon/autonomous_driving/x3.sh stop` | stop (e.g. from a second terminal) |
| `bash Marlon/autonomous_driving/x3.sh neustart` | stop all ROS containers, start fresh, drive off (`restart` works too) |
| `bash Marlon/autonomous_driving/x3.sh shell` | terminal inside the container (`ros2 topic echo /scan` …) |
| `bash Marlon/autonomous_driving/x3.sh hochladen` | commit and push the logs (`upload` works too) |

**Important:**
- If another ROS container is still running (e.g. from `~/run_docker.sh`), `x3.sh`
  aborts with a message → `bash Marlon/autonomous_driving/x3.sh neustart`.
- If the web app is running (`python main.py`, port 8000), the start aborts: it uses
  the same motor board and the same lidar. Stop the web app first.
- The Yahboom app restarts after every boot (`~/.config/autostart/`); `x3.sh`
  stops it every time.

## 1. Calibrating (once, the robot does NOT drive)

```bash
bash Marlon/autonomous_driving/x3.sh --calibrate
```
Hold a hand about 30 cm **in front of** the robot: `VORNE` and about `0°` must appear.
Hand on the **left**: about `+90°`.

Our X3 is already set up (`--front-offset -90`, not mirrored).
If it no longer fits after a rebuild: adjust `--front-offset` (degrees), or use
`--flip` if left and right are swapped. **Always check the left side too** –
a mirror error is not noticeable at the front and back (as on 27.10.: the robot
swerved to the wrong side and kept bumping into things).

In addition, while looking around at startup, the robot checks whether the
lidar image rotates the right way round. If not, it stops with
`FEHLER: Lidar-Bild dreht sich …` and beeps. Result in the log:
`Spiegel-Pruefung: …x passend, …x gespiegelt`.

## 2. Driving

More options: `--no-look-around`, `--no-sound`, `--no-led`, `--radius`,
`--margin`. Show all with `python3 x3_explorer.py --help`.
Unknown options (typos) are rejected.

In RViz, a green arrow on `/x3_explorer/direction` shows the chosen driving
direction. Good for a presentation.

## Logs

Every run writes to `Marlon/autonomous_driving/protokoll/` (the previous run is overwritten):

| File | Content |
|-------|--------|
| `start.txt` | complete output of the startup and the explorer |
| `bringup.txt` | driver (`yahboomcar_bringup`) |
| `treiber.txt` | driver node restarted on its own (only after a USB dropout; messages from the driver watchdog are in `start.txt`) |
| `lidar.txt` | lidar (`sllidar_node`) |
| `diagnose.txt` | devices, processes, topics at startup and on errors |

Pushing is done from the laptop only. Read the logs directly on the Pi (e.g. via SSH).
`bash Marlon/autonomous_driving/x3.sh hochladen` still exists, but is not needed for this.

## If something does not start

The script detects ROS1/ROS2 automatically and starts:

| | ROS2 | ROS1 |
|-|------|------|
| Driver | `yahboomcar_bringup yahboomcar_bringup_X3_launch.py` | `yahboomcar_bringup bringup.launch` |
| Lidar  | `ros2 run sllidar_ros2 sllidar_node` with `serial_port:=/dev/rplidar` | `rplidar_ros rplidar.launch` |

If the launch files are named differently on your image (e.g. RPLIDAR S2), override them
(inside the container: `bash Marlon/autonomous_driving/x3.sh shell`, then `cd` into the repo folder – same path as on the Pi):

```bash
LIDAR_LAUNCH="sllidar_ros2 sllidar_s2_launch.py" bash Marlon/autonomous_driving/start_autonomy.sh
```
If the driver/lidar are already running (e.g. started by hand), they are not started twice.
A hanging `sllidar_node` from the last run is stopped beforehand.

## Testing on the PC (without the robot)

```bash
python test_explorer_sim.py
```
Simulates a room with a table, a shelf and chair legs, lidar noise and wheel slip.
Current result: 0 collisions in 3×5 simulated minutes.
