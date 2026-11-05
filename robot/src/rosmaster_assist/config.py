"""Central settings.

Every value you might want to adjust while experimenting lives here -- not
scattered across the code. Anyone tuning the driving behavior only needs to
open this file.
"""

import os
from pathlib import Path

# --- Serial connection to the expansion board --------------------------------

# The port name differs depending on the image. Which one is right for you is
# shown by `python scripts/check_hardware.py`. None = Rosmaster_Lib searches
# by itself.
SERIAL_PORT: str | None = None

# Back channel from the expansion board (battery voltage, IMU, encoders).
#
# For this, Rosmaster_Lib starts a thread that continuously reads from the
# serial port. If a second program reads from the same port, this thread dies
# with "device reports readiness to read but returned no data". Driving
# still works -- only the readings are missing.
#
# If you cannot resolve the conflict right away, switch this off: we do not
# use the data anywhere yet. The proper fix remains to stop the second
# program (see docs/hardware.md).
TELEMETRY_ENABLED = True

# --- Reconnecting after USB dropouts ------------------------------------------

# If the 5 V supply on the USB hub dips briefly (weak battery, motors and
# camera drawing current at the same time), board, lidar and camera
# disconnect and shortly afterwards reappear under new numbers. The server
# notices this and reconnects -- the robot stands still meanwhile.

# How often to check whether the connection to the board is still up.
RECONNECT_CHECK_S = 1.0

# Interval between retries: starts short and doubles up to the maximum as
# long as the device does not reappear.
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 30.0

# If the lidar drops out again within this time after reconnecting, the wait
# does not start over but keeps growing. Otherwise its motor restarts every
# few seconds and keeps the USB dropouts going.
LIDAR_STABLE_S = 60.0

# If no new camera frame arrives for this long, the camera is reopened.
# The wait in between grows with every failure: if starting up the camera is
# itself what triggers the voltage dip, retrying quickly would restart the
# whole hub over and over.
CAMERA_REOPEN_S = 5.0
CAMERA_REOPEN_MAX_S = 120.0

# --- Driving -------------------------------------------------------------------

# Upper limits. The watchdog clamps every incoming command to these, no
# matter what the app sends. The app fetches these values when it connects
# -- so they only need to be set here, not additionally in the browser.
#
# NOTE: The chassis manages about 1 m/s, and the expansion board clamps
# internally to its own range. Values above that do not change the actual
# speed -- the app then shows more than the robot actually drives.
MAX_SPEED_X = 1.0      # m/s  forward / backward
MAX_SPEED_Y = 1.0      # m/s  sideways (mecanum)
MAX_SPEED_OMEGA = 3.0  # rad/s  turning

# Additional upper limit for everything the robot decides by itself
# (following, exploring, homing). The individual modes have their own,
# usually lower values -- this limit only catches calculation errors, so
# that no autonomous mode ever drives faster than the lidar can see
# obstacles in time (7.5 revolutions/s, plus braking distance). Manual
# control is exempt; MAX_SPEED_* applies to it.
AUTONOMOUS_MAX_SPEED = 0.35        # m/s   vx and vy
AUTONOMOUS_MAX_OMEGA = 1.2         # rad/s

# Battery almost empty: autonomous modes drive slower. Under load the
# voltage dips further, and that is exactly when the USB hub on the X3
# resets -- at half speed the drive draws much less current.
# Factor on AUTONOMOUS_MAX_SPEED once the battery is below BATTERY_WARN_V.
AUTONOMOUS_BATTERY_FACTOR = 0.5

# Last brake for all autonomous commands in the watchdog: the drive guard
# (safety/drive_guard.py, values GUARD_* further down). The modes stop
# earlier on their own; this catches what they miss. Until 03.11. this was
# AutonomSchutz (a strip in the direction of travel, without rotation), see
# E10.
AUTONOMOUS_GUARD_ACTIVE = True

# --- Safety ----------------------------------------------------------------------

# If no new drive command arrives within this time, the robot stops.
# Protects against WiFi loss, a closed browser tab and app crashes.
# Too short = jerky driving, too long = robot keeps driving uncontrolled.
COMMAND_TIMEOUT_S = 0.5

# Drive priority with several open apps: every app sends "vx=0" ten times
# per second even when idle. While one was steering, the zeros of the other
# came in between, and it drove stuttering at half speed (03.11., two
# devices; only noticed after following, because the interaction discards
# zeros anyway). For this long (s) after the last real drive command, zeros
# only count from the device that is currently steering. "stop" always
# works, from any device.
DRIVE_PRIORITY_S = 0.5

# Interval at which the watchdog checks. Must be well below the timeout.
WATCHDOG_INTERVAL_S = 0.1

# Acceleration and braking ramp. Without it, every drive command hits the
# motors unramped: when following and with gestures the command switches
# between driving and holding at 20 Hz, the motors draw a current surge each
# time, the voltage dips and the USB hub resets (see sensor check 28.10.:
# resets only under motor load). The ramp limits how fast vx/vy/omega may
# change per second.
#
# Braking may be faster than accelerating -- stopping in front of an
# obstacle matters more than a smooth start. watchdog.stop() (emergency
# stop, timeout, abort) always bypasses the ramp and stops immediately.
RAMP_ACTIVE = True
RAMP_ACCEL_XY = 0.8       # m/s²   0 -> 0.3 m/s in just under 0.4 s
RAMP_BRAKE_XY = 2.0        # m/s²   0.3 m/s -> 0 in 0.15 s (about 2 cm distance)
RAMP_ACCEL_OMEGA = 3.0    # rad/s²
RAMP_BRAKE_OMEGA = 6.0     # rad/s²
# Longest time step the ramp takes at once. After a pause (robot was
# standing, first command in a long time) the whole pause would otherwise
# count as "time to accelerate" and the command would jump after all.
RAMP_DT_MAX_S = 0.1

# --- Server --------------------------------------------------------------------

# 0.0.0.0 = reachable from the whole WiFi (needed so the phone can connect).
HOST = "0.0.0.0"
PORT = 8000

# Additionally the same app over HTTPS (0 = off). Browsers only allow the
# microphone on secure pages, so not under http://10.42.0.62:8000 -- the
# announcement from the phone needs https://10.42.0.62:8443. The server
# creates the certificate itself on first start (openssl); the browser warns
# once because nobody has signed it.
HTTPS_PORT = 8443
HTTPS_FOLDER = "~/.rosmaster/https"

# Interval at which the robot sends status messages to the app on its own.
STATUS_INTERVAL_S = 0.5

# How long the server waits for open connections when shutting down
# (Ctrl+C). The camera stream never ends by itself: without a limit, uvicorn
# waited forever on 28.10., and the cleanup (motors off, stop lidar) never
# even started.
SHUTDOWN_TIMEOUT_S = 3

# --- Lidar ---------------------------------------------------------------------

LIDAR_ENABLED = True

# Fixed device path from the udev rule. Do not use /dev/ttyUSB0: the
# expansion board is on the same bus and the numbers can swap.
LIDAR_PORT = "/dev/rplidar"
LIDAR_BAUDRATE = 115200

# Mounting orientation: by how many degrees the lidar's 0° direction deviates
# from "front" (camera). On the X3 it is mounted rotated by 90° -- without
# correction, front appeared on the left in the app, and both the obstacle
# stop and localization looked in the wrong direction. To check: hold a hand
# in front of the camera, it must appear at the top in the app. If it appears
# at the bottom instead, add 180.
LIDAR_ANGLE_OFFSET_DEG = -90.0

# Points outside these limits are discarded. Too close is usually parts of
# the robot itself, too far is noise.
LIDAR_MIN_M = 0.15
LIDAR_MAX_M = 8.0

# --- Lidar-based localization -------------------------------------------------

# Only within this distance of the starting point is the scan compared with
# the reference scan. Further away there is too little overlap, and a
# mismatch would be worse than no correction at all.
MATCH_RADIUS_M = 4.0

# Mean residual distance after matching up to which the result is accepted.
# Above that, the match counts as failed.
MATCH_MAX_RESIDUAL_M = 0.18

# How strongly a correction is applied per pass (0..1). Low, so that a
# single mismatch does not throw off the position.
MATCH_TRUST = 0.55

# Matching rate. More often brings little -- the lidar only spins about five
# times per second.
MATCH_HZ = 4.0

# --- Mapping (SLAM in the ROS container) ----------------------------------------

# The map is built by slam_toolbox in ROS2 Foxy in the Docker container. But
# our server holds the lidar and the board; it feeds scans and pose into the
# container (mapping/scan_feed.py, rationale: E8). The environment variables
# ROSMAP_* from mapping/manager.py override the first values.
MAP_CONTAINER = "x3"
MAP_ROS_SETUP = "/opt/ros/foxy/setup.bash"
MAP_SCAN_TOPIC = "/scan"

# How often (Hz) the pose is sent to ROS. slam_toolbox needs the pose at
# exactly the time of each scan; so it must be denser than the lidar
# revolution (5.5 Hz).
MAP_FEED_HZ = 20.0

# Angular grid of the fed scan: this many beams per revolution.
# slam_toolbox fixes the beam count at the first scan and discards every
# later one with a different count. But the A1 delivers a slightly different
# number of points every time -- hence the fixed grid.
MAP_SCAN_BEAMS = 360

# If the wheel pose jumps further than this between two lidar revolutions,
# it was not driving but a reset of the starting point. SLAM does not get to
# see the jump -- it would take it for real motion and tear the map apart.
MAP_JUMP_M = 0.3

# Lidar odometry (navigation/scan_odometry.py): the heading comes from
# matching consecutive scans, because the IMU on the X3 does not provide a
# heading angle. Up to this mean residual distance a match counts as
# successful; two scans taken shortly after each other usually match to
# 2-4 cm.
MAP_ICP_MAX_RESIDUAL_M = 0.08
# After this much motion the reference scan is renewed. Larger: less drift,
# but the scans overlap less well.
MAP_REF_M = 0.25
MAP_REF_DEG = 10.0

# --- Mapping drive (navigation/mapping_drive.py) ---
#
# When mapping it drives like a robot vacuum: once around with the wall on
# the right, then lanes parallel to the wall. Leisurely, so that SLAM knows
# every spot from several scans.

# Speed on lanes and along the wall (m/s), sideways when shifting, turning.
MAPPING_DRIVE_SPEED = 0.15
MAPPING_DRIVE_SIDE_SPEED = 0.12
MAPPING_DRIVE_TURN_SPEED = 0.5  # rad/s; below this the motors do not start reliably

# Distance from the lidar center to the wall during the wall-following lap.
# Half the robot width (0.12 m) plus room for steering errors.
MAPPING_DRIVE_WALL_DISTANCE_M = 0.40

# Spacing of the lanes. The lidar sees many meters, so for the map the lanes
# do not need to be as dense as when vacuuming -- only dense enough that it
# looks behind furniture.
MAPPING_DRIVE_LANE_SPACING_M = 0.8

# Half the robot length plus margin: this much room must be free to the side
# when it slides over to the next lane.
MAPPING_DRIVE_HALF_LENGTH_M = 0.22

# Wall-following lap is done when it is this close to the start again, after
# having driven at least MAPPING_DRIVE_LAP_MIN_M ...
MAPPING_DRIVE_LAP_M = 0.6
MAPPING_DRIVE_LAP_MIN_M = 3.0
# If the lap is shorter, it was not a wall but an obstacle in the middle of
# the room (pillar, sofa, table): it leaves it and looks for the wall. A
# 2 x 2 m room already has an 8 m perimeter, a good 5 m when driven along the
# wall.
MAPPING_DRIVE_ISLAND_M = 5.0
# ... but at the latest after this many seconds (open doors, angled rooms:
# then it might never come back).
MAPPING_DRIVE_WALL_FOLLOW_MAX_S = 300.0

# Maximum duration of the whole mapping drive (s), after which it stops.
# With continuing into other rooms (FRONTIER_*) an apartment takes longer
# than a single room.
MAPPING_DRIVE_MAX_S = 3600.0

# Once the first room is covered (or it cannot get any further there), it
# does not stop but heads for the next spot where the map turns into the
# unknown -- open doors, hallways, shadows behind furniture. Only when no
# such spot is reachable anymore is the map finished.
FRONTIER_ACTIVE = True
# Smaller transitions into the unknown (m) are measurement gaps, not
# passages.
FRONTIER_MIN_M = 0.4
# A new frontier this close (m) to a spot already visited is no longer a
# candidate: what is still unknown after driving there cannot be seen from
# there (mirror, glass, too far away).
FRONTIER_DONE_M = 0.8
# Arriving at the frontier: getting this close (m) is enough, the lidar sees
# far into it from there.
FRONTIER_NEAR_M = 0.5
# This many frontiers in a row may be missed (blocked, no path) before the
# mapping drive ends.
FRONTIER_MAX_ERRORS = 6
# At the end, drive back to the map's starting point -- it is then
# positioned correctly for the next drive with this map.
MAPPING_DRIVE_RETURN = True

# Wait longer for the lidar when mapping than when driving randomly (s): on
# the X3 the A1 disconnects and reconnects during voltage dips, and a long
# mapping drive should not end because of that.
MAPPING_DRIVE_LIDAR_WAIT_S = 30.0
# If the drive fails during the mapping drive (USB reset), it stops and
# continues after reconnecting, provided that happens within this many
# seconds. SLAM and the map keep running in the meantime.
MAPPING_DRIVE_RESUME_S = 60.0
# If all pages (/map and app) close, it waits this long (s) before
# stopping: a phone whose screen briefly turns off should not end the drive.
# After that the rule applies again: without a watcher it does not drive.
MAP_WATCHER_WAIT_S = 20.0

# --- Using the map: localization, planning, driving to a goal -------------------
#
# Once a map exists (live while mapping, saved afterwards), the robot knows
# where it is in it and, on request, drives to a point on the map
# (rationale: E12 in docs/decisions.md).

# How often (Hz) the lidar scan is matched against the map.
NAV_LOCALIZATION_HZ = 4.0
# Up to this distance (m) a map point counts as hit. The map's distance
# field ends at NAV_DISTANCE_MAX_M.
NAV_HIT_M = 0.15
NAV_DISTANCE_MAX_M = 1.0
# Minimum share of points hit for a match to count. Furniture that was moved
# and people hit nothing -- hence not 100 %.
NAV_MIN_SHARE = 0.45
# A match may not shift the pose by more than this (m, degrees), otherwise
# it has probably found a wrong, similar spot.
NAV_MAX_JUMP_M = 0.4
NAV_MAX_JUMP_DEG = 20.0
# Weight of a successful match (0..1). Smaller: calmer, but follows more
# slowly.
NAV_TRUST = 0.5
# After this long (s) without a successful match the pose counts as
# uncertain. Drives to a goal then stop.
NAV_CONFIDENT_S = 4.0
# Searching for its own pose (after loading a map, after being lifted):
# this far (m) around the hint, on this grid.
NAV_SEARCH_RADIUS_M = 1.5
NAV_SEARCH_GRID_M = 0.1
NAV_SEARCH_DEG = 5.0
# Minimum share of points hit for a search to count: when it searches on its
# own (after loading, after being lifted), it must be a clear hit --
# rectangular rooms look alike, a wrong pose would be worse than none. If
# someone has shown on the map where it is, less is enough.
NAV_SEARCH_MIN_SHARE = 0.8
NAV_HINT_MIN_SHARE = 0.6
# If a clearly different pose (further than this many m or degrees) fits
# almost as well (within this share), the search is ambiguous: better ask
# back then.
NAV_AMBIGUOUS_M = 0.5
NAV_AMBIGUOUS_DEG = 30.0
NAV_AMBIGUOUS_SHARE = 0.05

# Path planning: on a coarser grid (m) than the map; that is enough for
# paths and computes on the Pi in fractions of a second.
NAV_PLAN_RESOLUTION_M = 0.1
# No path passes a wall closer than this (m, from the lidar center): half
# the robot diagonal (0.21 m) plus margin. A passage therefore needs 0.5 m.
NAV_SAFETY_M = 0.25
# Up to this distance (m) being close to a wall costs extra -- so paths run
# through the middle of a door and not along its edge.
NAV_PREFERRED_DISTANCE_M = 0.5
NAV_WALL_COST = 3.0
# If the goal lies in a wall or too close to one, it is moved up to this far
# (m) to the nearest drivable spot.
NAV_GOAL_SNAP_M = 0.6

# Driving along the path.
NAV_DRIVE_SPEED = 0.25
NAV_TURN_MAX = 0.6  # rad/s, for aligning with the direction of travel
# Waypoint reached when it is this close (m); goal reached at
# NAV_GOAL_TOLERANCE_M.
NAV_WAYPOINT_M = 0.3
NAV_GOAL_TOLERANCE_M = 0.15
NAV_GOAL_DEG = 8.0
# Strayed further than this (m) from the path: replan.
NAV_DEVIATION_M = 0.7
# If the path is blocked for this long (s), it replans around it; the
# blocked spot counts as a wall for NAV_BLOCK_S.
NAV_BLOCKED_S = 3.0
NAV_BLOCK_S = 60.0
NAV_BLOCK_RADIUS_M = 0.3
# Replan this many times in a row, then give up.
NAV_MAX_REPLANS = 5
# Maximum duration of a drive to a goal: this many seconds per meter of
# path, but at least NAV_DRIVE_MIN_S.
NAV_DRIVE_S_PER_M = 12.0
NAV_DRIVE_MIN_S = 60.0

# Stored trail in the display: one point per this many meters, at most this
# many points.
NAV_TRAIL_M = 0.1
NAV_TRAIL_MAX = 3000

# Mapping in the simulation (--mock): no Docker, no slam_toolbox. The map is
# then built in Python from the simulated lidar, so that the /map page and
# everything that uses the map works at the desk.
MAP_SIM_RESOLUTION_M = 0.05
MAP_SIM_HZ = 2.0
# Imaginary apartment instead of a single room: a partition wall with a
# door. For continuing into the neighboring room (--mock, tests). Switch on
# without changing the file: ROSMASTER_MOCK_WOHNUNG=1 python main.py --mock
MOCK_APARTMENT = os.environ.get("ROSMASTER_MOCK_WOHNUNG") == "1"

# If no map arrives this long (s) after starting, the page reports it along
# with the SLAM log. slam_toolbox publishes the map every 5 s; so the first
# one should be there after a few seconds.
MAP_FIRST_MAP_S = 20.0

# The page shows this many last lines of the SLAM log.
MAP_LOG_LINES = 15

# --- Collision avoidance during manual control ----------------------------------

# Off by default: whoever steers by hand usually sees the robot and does not
# want to be patronized. Can be switched on in the app.
COLLISION_ENABLED = False

# Minimum distance in the direction of travel, measured from the lidar
# center as before. Adjustable with a slider in the app. Checked via the
# drive guard (below): the robot's edge keeps (distance - ROBOT_FRONT_M)
# clearance all around, i.e. exactly this distance from the lidar center at
# the front.
COLLISION_DISTANCE_M = 0.35

# Slider limits. Below 15 cm the lidar sees nothing reliable anymore; above
# 1.5 m something would almost always be in the way in a room.
COLLISION_MIN_M = 0.15
COLLISION_MAX_M = 1.5

# --- Drive guard: collision check using the footprint (safety/drive_guard.py) ---
#
# Applies to everything that drives by itself (quelle="autonom": explorer,
# mapping drive, homing, interaction, precise drive commands), and to manual
# control when the obstacle stop is on there. For every command it predicts
# where the footprint will go -- also on arcs and when turning; if a lidar
# point lies inside, it brakes in time.

# Footprint, measured from the lidar center. The X3 protrudes ~0.15 m at
# the front, the width is 0.245 m. The rear is estimated -- measure it on
# the robot. This is the only place for it: arc planning (AVOID_*) uses the
# same values, otherwise the drive guard would brake arcs the planner
# considers safe.
ROBOT_FRONT_M = 0.15
ROBOT_REAR_M = 0.17
ROBOT_HALF_WIDTH_M = 0.125

# Minimum clearance the robot's edge keeps when driving autonomously.
# Must be smaller than AVOID_MARGIN_M (planner): the safety layer is the
# inner limit. With equal values, every bit of noise in the scan would choke
# planned arcs.
GUARD_MARGIN_M = 0.04

# On the X3 ~0.22 s pass from command to effect (sensor check 28.10.); plus
# the age of the scan. During that time it keeps driving without braking.
GUARD_REACTION_S = 0.25

# Deceleration assumed for braking. Deliberately below the braking ramp
# (RAMP_BRAKE_XY): mecanum wheels slip when braking.
GUARD_BRAKE = 1.0       # m/s²
GUARD_BRAKE_TURN = 3.0  # rad/s²

# The scan must not be older than this, otherwise it stands still (it does
# not drive blind).
GUARD_SCAN_MAX_S = 0.5

# Below this share of the speed it stops completely: the motors no longer
# translate such slow speeds into motion on the floor.
GUARD_MIN_SHARE = 0.2

# If an autonomous drive cannot proceed for this long (s) because of the
# drive guard, it gives up instead of standing in front of the obstacle
# forever.
GUARD_STUCK_S = 10.0

# The obstacles from the depth camera (including the memory, below) feed
# into the drive guard -- it then also brakes for chair legs and cable
# ducts that lie below the lidar plane.
GUARD_WITH_DEPTH = True

# --- Depth camera (Orbbec Astra, sensors/depth.py) -------------------------------
#
# The lidar only sees in its own plane (~17 cm above the floor). Below that
# -- chair legs, skids, cable ducts, bed frames -- it runs into things and
# gets stuck. The depth camera sees everything ahead between the floor and
# the top of the robot. Accessed via Orbbec's OpenNI2 build (located in the
# Yahboom container, see scripts/setup_orbbec.sh), without ROS.

DEPTH_ACTIVE = True
# Folder containing libOpenNI2.so and OpenNI2/Drivers/liborbbec.so.
DEPTH_OPENNI_PATH = "~/orbbec_openni2"
# 320x240 is enough for obstacles and saves computing time and USB bandwidth.
DEPTH_WIDTH = 320
DEPTH_HEIGHT_PX = 240
# How often (Hz) a frame is evaluated; the camera delivers 30.
DEPTH_HZ = 8.0
# Mounting, measured from the lidar center on the floor: height of the
# camera above the floor, forward offset, downward tilt. Measure on the
# robot -- the floor detection compensates for rough errors as long as it
# sees the floor.
DEPTH_CAMERA_HEIGHT_M = 0.22
DEPTH_CAMERA_FORWARD_M = 0.05
DEPTH_CAMERA_TILT_DEG = 0.0
# Only this range in front of the robot is evaluated (m). Below ~0.6 m the
# Astra Pro does not measure at all (minimum range), further away it becomes
# inaccurate, and there the lidar sees things anyway before they matter.
DEPTH_MIN_M = 0.55
DEPTH_MAX_M = 1.8
# Anything this high above the detected floor counts as an obstacle (m). Not
# below ~4 cm at the bottom, otherwise every carpet edge and the noise of the
# floor becomes an obstacle; at the top up to just above the top of the
# robot.
DEPTH_MIN_HEIGHT_M = 0.04
DEPTH_MAX_HEIGHT_M = 0.40
# If the camera sees no floor (dark, shiny), only the mounting values count.
# Then be more cautious: start higher and do not look as far -- 2° of wrong
# tilt is already 6 cm at 1.8 m.
DEPTH_NO_FLOOR_MARGIN_M = 0.05
DEPTH_NO_FLOOR_MAX_M = 1.3
# Grid cell (m) and minimum number of points per cell: single outliers in
# the depth image should not create an obstacle, a chair leg has dozens.
DEPTH_CELL_M = 0.05
DEPTH_MIN_POINTS_PER_CELL = 4
# A depth frame must not be older than this, otherwise the camera counts as
# off.
DEPTH_MAX_AGE_S = 1.0
# Wait time (s) before the next connection attempt after an error.
DEPTH_RETRY_S = 5.0

# Memory (navigation/obstacle_memory.py): below 0.6 m the camera is blind
# -- exactly where the obstacle matters. What it saw further ahead is
# therefore carried along with the robot's own motion until it has passed.
# A point stays for at most this long (s), and only up to this distance.
DEPTH_MEMORY_S = 30.0
DEPTH_MEMORY_M = 2.5

# Display (/tiefe/stream): color scale from near (red) to far (blue), in m.
DEPTH_VIEW_NEAR_M = 0.5
DEPTH_VIEW_FAR_M = 3.0
# Overlaid on this color image (same Orbbec, the camera next to it).
DEPTH_OVERLAY_CAMERA = "orientierung"
# The two cameras sit ~2.5 cm apart and have different fields of view. The
# scaling comes from the fields of view; whatever is still off, shift it by
# hand here (pixels in the color image, + = right/down) and zoom (1.0 =
# field of view only).
DEPTH_OVERLAY_SHIFT_PX = (0, 0)
DEPTH_OVERLAY_ZOOM = 1.0
# Opacity of the depth colors over the color image (0..1).
DEPTH_OVERLAY_OPACITY = 0.45

# --- Progress watch (navigation/progress.py) ------------------------------------
#
# Compares what was driven (command after ramp and drive guard) with the
# motion the lidar sees (ICP of two scans). If it turns its wheels but does
# not get anywhere, it is stuck on something nobody can see.
PROGRESS_ACTIVE = True
# Comparison window (s) and how often (s) a scan is remembered.
PROGRESS_WINDOW_S = 2.0
PROGRESS_TICK_S = 0.5
# Only judged from this much commanded motion within the window onwards.
PROGRESS_MIN_DIST_M = 0.12
PROGRESS_MIN_TURN_DEG = 25.0
# Stuck if, according to the lidar, less than this share of it happened.
PROGRESS_SHARE = 0.3
# An imaginary obstacle is placed where it got stuck, so that it does not
# drive right back into it. It stays for this long (s).
PROGRESS_PHANTOM_S = 30.0
# After this long without a new recovery, the attempts are counted afresh
# when mapping (when driving randomly, EXPLORE_FREE_RESET_S takes care of
# this).
PROGRESS_RECOVERED_S = 15.0

# --- Homing ------------------------------------------------------------------------

# Rate at which the pose is updated. Higher = more accurate, but the
# inaccuracy comes from wheel slip anyway, not from this rate.
ODOMETRY_HZ = 20

# Take the heading from the board's IMU attitude? Off: the sensor check of
# 28.10. shows that get_imu_attitude_data() permanently returns 0 on the X3.
# That froze the angle (theta stayed at 0, only lidar matching moved it),
# and homing, exploring and localization computed with the wrong direction.
# Instead, the turn rate measured by the board is integrated.
# Only switch on if the IMU demonstrably reports a real angle -- even then
# it is discarded as soon as it stands still while turning.
ODOM_IMU_HEADING = False

# Above this turn rate (rad/s) an IMU angle must change visibly; if it does
# not do so over ODOM_IMU_CHECK_S, the IMU counts as unusable.
ODOM_IMU_CHECK_OMEGA = 0.3
ODOM_IMU_CHECK_S = 1.0

# Speed of automatic homing. Deliberately leisurely: the pose comes from
# dead reckoning, and detouring (DETOUR_*) needs time to look.
HOME_SPEED = 0.25        # m/s

# Within this distance of the starting point the drive counts as finished.
# Going smaller is not worth it -- dead reckoning is less accurate than that.
HOME_TOLERANCE_M = 0.12

# Slow down over the last centimeters, otherwise it overshoots the goal.
HOME_SLOWDOWN_M = 0.4

# Emergency brake: homing aborts after this time. Prevents the robot from
# driving around endlessly with a drifted pose.
HOME_TIMEOUT_S = 60.0

# Distance that must stay free in the direction of travel. If it is not, it
# slides past diagonally (DETOUR_*); only if there is no room anywhere for
# HOME_DETOUR_WAIT_S does it stop and report it.
HOME_STOP_DISTANCE_M = 0.45

# Half the opening angle of the monitored area, in degrees. Too narrow
# misses table legs beside the driving line, too wide stops for walls that
# could easily be passed.
HOME_CONE_DEG = 35.0

# --- Precise drive commands ("2 Meter vor", "90 Grad drehen") ---------------------
# See navigation/maneuver.py. Measured with the lidar: each scan is matched
# against one from the start, which shows how far it has really driven or
# turned. Wheels and IMU only provide the estimate between two scans (the
# IMU rotation is unreliable on the X3).

MANEUVER_SPEED = 0.20         # m/s  at most, when driving
MANEUVER_MIN_SPEED = 0.05     # m/s  below this the motors do not start reliably
MANEUVER_KP_V = 1.2           # (m/s)/m  speed per meter of remaining distance
MANEUVER_TURN = 0.6           # rad/s at most, when turning
# Below 0.4 rad/s the motors do not start reliably on the floor (explorer).
MANEUVER_MIN_TURN = 0.4
MANEUVER_KP_W = 1.5           # (rad/s)/rad
# For this long (s) after starting to turn from standstill, the commanded
# rotation does not count yet. Otherwise the estimate considered small
# corrections (1-2°) done after a single tick -- the ramp was only putting
# out 0.3 rad/s at that point, the motors did not start, and the lidar
# measured the same remainder again while standing (floor test 03.11.: three
# times in vain, then "fast erledigt").
MANEUVER_STARTUP_S = 0.25
# Done when remaining distance and remaining angle are below these ...
MANEUVER_TOLERANCE_M = 0.03
MANEUVER_TOLERANCE_DEG = 2.0
# ... and a lidar scan taken while standing confirms it as well: while
# driving, one revolution of the A1 (0.18 s) is distorted. At most this many
# corrections.
MANEUVER_MAX_CORRECTIONS = 3
MANEUVER_SETTLE_S = 0.3         # after stopping wait this long, then measure
# Match as much as possible against the scan from the start: the lidar sees
# all around, so after a turn it sees the same walls. Every change of the
# reference scan brings some error (in the simulation ~0.2° per change; with
# a change every 25° it was off by 3.4° after 360°).
# The reference is only changed when matching gets worse -- it drives out of
# the room, the old walls disappear: mean residual distance larger than ...
MANEUVER_NEW_REF_RESIDUAL_M = 0.06
# ... or less than this share of the points finds a partner.
MANEUVER_NEW_REF_PAIRS = 0.6
# While driving, get within this fraction of the tolerance -- otherwise it
# stopped just inside (2.9 of 3 cm), and any inaccuracy tipped it outside.
# Not much smaller: with one third (1 cm) the goal was within the
# measurement accuracy, and it corrected three times in vain.
MANEUVER_GOAL_SHARE = 0.5
# Matching may fail continuously for this long (s), then it stops
# (unfamiliar surroundings, lidar gone) -- driving blind would be inaccurate
# and unsafe.
MANEUVER_BLIND_S = 2.0
# Limits for one command.
MANEUVER_MAX_M = 5.0
MANEUVER_MAX_DEG = 720.0
# Only turn if there is this much room all around (corners swing out), m
# from the lidar center. Only a fallback without the drive guard now;
# otherwise the guard checks the footprint over the whole angle
# (GUARD_MARGIN_M clearance).
MANEUVER_TURN_FREE_M = 0.30
# If the drive guard blocks continuously for this long (s), the command
# gives up. Short, because whoever says "2 Meter vor" is waiting for an
# answer.
MANEUVER_BLOCKED_S = 1.5
# At most this many lidar points for matching (computing time on the Pi).
MANEUVER_MAX_POINTS = 300

# --- Camera ---------------------------------------------------------------------

CAMERA_ENABLED = True

# Which USB camera is there for what. All cameras found always run; this
# only defines which one carries which name and who evaluates its image.
#
# Each one is found by its USB ID as in `lsusb` -- "vvvv" matches every
# model of the manufacturer, "vvvv:pppp" only exactly one. Not via
# /dev/videoN: the numbers swap after a reboot or replugging, and person
# detection would silently get the wrong image.
#
# A role whose camera is missing keeps being searched for in the background
# -- if the camera reappears after a USB dropout, it continues running.
# Additional USB cameras without a role also run as "usb1", "usb2", ...
#
# fov_deg: horizontal field of view at 640x480 according to the data sheet.
# This is used to map a person in the image to a direction in the lidar --
# measure it if needed.
#
# flip: compensates the mounting orientation. None, "horizontal" (mirror
# left/right), "vertikal" (top/bottom) or "180" (both = upside down). The
# image is rotated on the robot before anyone sees it -- including person
# and pose detection. Adjustable on the settings page.
CAMERA_ROLES = {
    # Orbbec depth camera, color channel. For orientation and driving. The
    # depth channel (2bc5:060f) is not a video device and needs the Orbbec SDK.
    "orientierung": {"usb_id": "2bc5", "label": "Orbbec (Farbbild)", "fov_deg": 66.0,
                     "flip": None},
    # Logitech C270: person detection. Mounted the wrong way round (29.10.2026).
    "personen": {"usb_id": "046d", "label": "Logitech C270", "fov_deg": 45.0,
                 "flip": "horizontal"},
}

# Which camera answers under /camera/stream, /camera/snapshot and
# /camera/status (the endpoints without a camera name) and is shown first
# in the app.
CAMERA_DEFAULT = "orientierung"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Frames per second in the stream. Higher costs WiFi bandwidth and CPU; with
# a stuttering image over WiFi, rather go down here than lower the
# resolution.
CAMERA_FPS = 15

# What the cameras themselves deliver at most: Logitech C270 and Orbbec
# (color image) each manage 30 fps at 640x480 -- in low light the C270
# lengthens the exposure and only reaches ~15. The camera runs at this rate;
# CAMERA_FPS only throttles the fetching.
CAMERA_MAX_FPS = 30

# JPEG quality 1..100. Below this the image gets mushy, above it gets large.
CAMERA_QUALITY = 70

# --- Autonomous driving (exploring) ----------------------------------------------
#
# Test 1: the robot drives around randomly, avoiding everything the lidar
# sees. The values come from the sensor check of 28.10.2026
# (robot-protokolle/2026-10-28_sensorcheck): commands are executed with
# ~0.22 s delay and only to 70-90 % -- hence the leisurely pace.

# Cruising speed straight ahead. More than 0.25 m/s is not worth it in a
# room: at 7.5 lidar revolutions per second it would otherwise travel 3-4 cm
# between two scans.
EXPLORE_SPEED = 0.20      # m/s

# Turn rate when turning on the spot. Below 0.4 rad/s the motors do not
# start reliably on the floor.
EXPLORE_TURN_SPEED = 0.6  # rad/s

# Measured from the lidar center. The X3 protrudes ~0.15 m beyond it at the
# front -- so 0.40 m leaves ~25 cm clearance, enough for the braking
# distance at 0.2 m/s plus delay.
EXPLORE_STOP_M = 0.40

# From here on, slow down in front of an obstacle.
EXPLORE_SLOW_M = 0.90

# This much room must be free ahead for it to start driving after turning.
# Larger than EXPLORE_STOP_M, otherwise it stops again right away and
# oscillates.
EXPLORE_GO_M = 0.70

# Half the width of the driving corridor: half the robot width
# (0.245 m / 2) plus safety margin. This way table legs just beside the
# driving line count too.
EXPLORE_HALF_WIDTH_M = 0.20

# If something comes closer than this from the side, the mecanum drive
# gently moves the robot away from it instead of stopping.
EXPLORE_SIDE_M = 0.30

# If something anywhere around comes closer than this (a person steps up),
# it stays put instead of turning into the person.
EXPLORE_NEAR_M = 0.22

# Random changes of direction so that it does not always drive the same
# lane: at the earliest/latest after this many seconds of driving straight.
EXPLORE_WANDER_MIN_S = 4.0
EXPLORE_WANDER_MAX_S = 10.0

# On the floor it only turns ~75 % of what is commanded (sensor check: 90°
# commanded, 65° turned). The turn duration is extended accordingly. Only
# applies as a fallback when the board provides no turn rate -- otherwise
# it turns by measured angle (EXPLORE_TURN_*).
EXPLORE_TURN_FACTOR = 1.3

# Turning by angle: end the turn command this many degrees before the
# target. Command delay and braking ramp turn it a bit further.
EXPLORE_TURN_LEAD_DEG = 8.0

# Upper limit of a measured turn as a multiple of the calculated duration
# -- catches the case where it does not turn (e.g. wedged in).
EXPLORE_TURN_TIME_FACTOR = 2.0

# After turning, it only decides using a lidar revolution that started this
# long after the end of the turn (the robot still coasts briefly).
EXPLORE_COAST_S = 0.25

# A lidar revolution must not be older than this, otherwise it drives blind.
# The real lidar delivers a revolution every ~0.13 s.
EXPLORE_SCAN_MAX_AGE_S = 1.0

# Wait this long for fresh lidar data before the drive ends.
EXPLORE_LIDAR_WAIT_S = 3.0

# If a person stands this close in front of it, it waits first instead of
# turning right away -- maybe the person walks on.
EXPLORE_PERSON_WAIT_S = 2.0

# Total duration of an exploration; 0 = until someone stops it.
EXPLORE_TIMEOUT_S = 0.0

# If something comes closer than EXPLORE_NEAR_M, it waits this long (maybe a
# person who is about to walk on) and then pushes itself free.
EXPLORE_NEAR_WAIT_S = 3.0

# Unsticking: slide in the freest direction without turning (mecanum). Slow
# enough that the rear does not bump anything, long enough to make a
# difference.
EXPLORE_UNSTICK_SPEED = 0.08
EXPLORE_UNSTICK_S = 1.5
# At least this much must be free in the chosen direction (from the lidar
# center; the robot itself extends ~0.15 m to the front and rear) ...
EXPLORE_UNSTICK_FREE_M = 0.45
# ... and below this value it aborts the sliding.
EXPLORE_UNSTICK_STOP_M = 0.30
# This many unsticking attempts in a row without free driving in between,
# then it gives up.
EXPLORE_UNSTICK_MAX = 3
# It must drive straight unobstructed for this long for failed attempts and
# unsticking attempts to start from zero again.
EXPLORE_FREE_RESET_S = 3.0

# Stuck detection: if it drives (command vx at least EXPLORE_STUCK_MIN_VX)
# for this long while the board reports no motion (below
# EXPLORE_STUCK_V_MS) or the lidar image all around changes by more than
# EXPLORE_STUCK_DIST_M (or 2 % of the distance, measurement noise) in at
# most one direction, it is stuck. Must stay below the 3 s of the
# odometry, after which it discards a measurement reporting standstill
# entirely.
EXPLORE_STUCK_S = 1.5
EXPLORE_STUCK_MIN_VX = 0.05
EXPLORE_STUCK_V_MS = 0.02
EXPLORE_STUCK_DIST_M = 0.03

# --- Avoidance (explorer, step F2) -----------------------------------------------
#
# Local avoidance without a map: short memory for points that slip out of
# the field of view, arcs instead of turning in front of obstacles,
# approaching doors and gaps through the middle, detecting being lifted. See
# navigation/avoidance.py.

# Short-term memory: keep lidar points for this long and within this
# radius. The pose is extrapolated from the board velocity, which drifts --
# hence only a few seconds and only nearby points.
AVOID_MEMORY_S = 3.0
AVOID_MEMORY_M = 1.0
# A remembered point is dropped when the lidar now sees this much further in
# that direction (the thing is gone or was a measurement error).
AVOID_CLEAR_M = 0.10

# Outline of the robot around the lidar center (m) for arc planning: the
# same as for the drive guard (ROBOT_*); measure the rear on the robot.
AVOID_FRONT_M = ROBOT_FRONT_M
AVOID_REAR_M = ROBOT_REAR_M
AVOID_HALF_WIDTH_M = ROBOT_HALF_WIDTH_M

# Arc planning: predict this far ahead (s) in these steps. An arc is only
# usable if the outline has at least AVOID_MARGIN_M clearance everywhere
# (larger than GUARD_MARGIN_M, see there).
AVOID_HORIZON_S = 1.2
AVOID_STEP_S = 0.1
AVOID_MARGIN_M = 0.05
# Candidates: turn rates (rad/s) and sideways velocities (m/s).
AVOID_OMEGAS = (-0.6, -0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6)
AVOID_VYS = (-0.08, 0.0, 0.08)
# Scoring weights: back onto the chosen heading, clearance to the nearest
# obstacle, speed, range in the direction of travel at the end of the arc,
# smooth steering (jumps in omega), sideways sliding only when needed. Keep
# JERK small, otherwise it always outweighs the heading correction and the
# robot never steers back; SIDEWAYS such that it still slides sideways away
# from obstacles next to it (door frame, chair leg) -- as EXPLORE_SIDE_M
# used to do.
AVOID_WEIGHT_HEADING = 1.0
AVOID_WEIGHT_CLEARANCE = 1.0
AVOID_WEIGHT_SPEED = 0.5
AVOID_WEIGHT_RANGE = 1.5
AVOID_WEIGHT_JERK = 0.2
AVOID_WEIGHT_SIDEWAYS = 0.25
# Up to this distance (m) more clearance or range counts as better.
AVOID_CLEARANCE_GOOD_M = 0.30
AVOID_RANGE_GOOD_M = 2.0

# Gaps and doors: check directions on this grid (degrees). Of a free area it
# takes the middle and directions at least AVOID_GAP_EDGE_DEG away from the
# edge -- not right next to the door frame.
AVOID_GAP_GRID_DEG = 2
AVOID_GAP_EDGE_DEG = 10
# If something is closer than this (door frame, narrow spot), it postpones
# the random change of direction until it is through.
AVOID_NARROW_M = 0.45

# Lift: compare consecutive scans (own motion compensated) in 10° sectors.
# If a sector changes by more than LIFT_JUMP_M, it counts as changed. From
# LIFT_SHARE of changed sectors in LIFT_SCANS scans in a row it counts as
# lifted; set down again when the scans stay below LIFT_STILL_SHARE for
# LIFT_REST_S. A person standing close by covers only ~4 of 36 sectors.
# With a measured pose it compares over LIFT_WINDOW_S instead of from scan
# to scan -- so slow carrying, which the wheels do not report, is noticed
# too.
LIFT_JUMP_M = 0.30
LIFT_SHARE = 0.5
# Three scans in a row (~0.55 s), not two: a bump over a door sill or hard
# braking tilts the lidar for about two scans and is not a lift. This holds
# with a measured pose, where each scan is compared with the one from
# LIFT_WINDOW_S ago and a lift shows in every scan. Without a measured pose
# only consecutive scans can be compared; a steady tilt then only shows
# while it changes, so two must do (LIFT_SCANS_UNMEASURED).
LIFT_SCANS = 3
LIFT_SCANS_UNMEASURED = 2
LIFT_STILL_SHARE = 0.15
LIFT_REST_S = 2.0
LIFT_WINDOW_S = 1.0

# Lift detection, rigid-fit check (navigation/avoidance.py). Before a scan
# counts as "jumped", the old scan is aligned onto the new one. If a rigid
# motion of the robot on the floor explains the change, it was the wheel
# odometry that was wrong (rotation error, wheels spinning while stuck),
# not a lift. A tilted lidar sees floor or ceiling and no longer fits;
# being carried fits, but as a real displacement the wheels did not report.
# Points closer than this (m) to the old scan after alignment count as fitting.
LIFT_FIT_INLIER_M = 0.10
# At least this share of points must fit for the change to count as rigid.
LIFT_FIT_MIN_SHARE = 0.7
# A real displacement (m, within LIFT_WINDOW_S) beyond what the
# wheels reported counts as being carried.
LIFT_CARRY_M = 0.25
# A sector that has points in one scan and none in the other only counts as
# changed if those points are closer than this (m). Far sectors often have
# no echo (window, open door, black furniture) and would flicker.
LIFT_NEAR_M = 3.0

# Detouring while following and during homing (step F3). There is a goal
# there; if the way to it is blocked, it slides past diagonally
# (avoidance.choose_direction). At most LOOKAHEAD_M plus the stop distance
# must be free -- further ahead it will see it again in time. It searches
# up to MAX_DEG beside the goal direction on a grid of GRID_DEG. The side
# chosen last gets a head start of SIDE_DEG (otherwise it oscillates in
# front of wide obstacles), and it keeps EDGE_DEG away from the edge of the
# free area; in narrow passages it takes the middle.
DETOUR_LOOKAHEAD_M = 0.8
DETOUR_MAX_DEG = 75.0
DETOUR_GRID_DEG = 5.0
DETOUR_SIDE_DEG = 20.0
DETOUR_EDGE_DEG = 10.0

# Homing: there may be no room in any direction for this long before it
# gives up. Short, so that a person walking past does not end it.
HOME_DETOUR_WAIT_S = 3.0

# --- Obstacle detection ------------------------------------------------------------

# If the distance between two neighboring points jumps by more than this, a
# new object begins. Grows with distance, because the points lie further
# apart there (1° at 3 m is 5 cm).
OBSTACLE_GAP_M = 0.10

# Only objects within this radius are named -- further away there are too
# few points for a meaningful classification.
OBSTACLE_RANGE_M = 3.0

# Chair and table legs are thin clusters of points in the lidar (2-5 cm).
OBSTACLE_LEG_MAX_M = 0.07

# Human legs are 8-20 cm wide (ankle to calf, with trousers).
OBSTACLE_HUMAN_LEG_MAX_M = 0.25

# From this length and straightness on an object counts as a wall.
OBSTACLE_WALL_MIN_M = 0.60

# --- Person detection (camera) ---------------------------------------------------

# Uses the HOG person detector built into OpenCV -- needs no model from the
# internet. Without OpenCV, everything runs without camera detection.
PERSON_DETECTION_ENABLED = True

# Frames per second for detection. One frame costs ~0.1-0.2 s CPU on the
# Pi 5; more would slow down the lidar and control.
PERSON_DETECT_HZ = 2.0

# Detect on images scaled down to this width: faster, and people in a room
# are large in the image anyway.
PERSON_DETECT_WIDTH = 320

# Minimum weight of the HOG detector. Higher = fewer false alarms, but
# people seen from the side are more likely to be missed.
PERSON_MIN_SCORE = 0.5

# Which camera from CAMERA_ROLES looks for people. If the role is missing
# from CAMERA_ROLES, detection uses the default camera.
PERSON_CAMERA = "personen"

# Horizontal field of view in degrees for cameras without their own entry in
# CAMERA_ROLES (CSI camera, additional USB cameras).
CAMERA_FOV_DEG = 60.0

# --- Body pose (Hailo-10H) ---------------------------------------------------------

# Gestures need the body keypoints (shoulders, wrists ...), not just a box.
# The model runs on the AI HAT+ 2 and costs the CPU almost nothing --
# important, because CPU load spikes trigger USB resets on the X3.
POSE_ENABLED = True

# Shipped with the Hailo package (hailo-models). Expects 640x640 RGB.
POSE_HEF = "/usr/share/hailo-models/yolov8s_pose_h10.hef"

# Frames per second. The camera delivers 15; waving needs ~10 so that an
# up-down motion spans several frames.
POSE_HZ = 10.0

# Minimum score from which a detection counts as a person (0..1).
POSE_MIN_SCORE = 0.5

# Keypoints below this confidence count as "not seen" -- hidden hands
# should not trigger a gesture.
POSE_KP_MIN_CONF = 0.4

# Open the Hailo at server start and keep it open, instead of opening it
# only when gesture control starts and releasing it afterwards. Opening is
# a load spike that can reset the USB hub on the X3 -- then the board was
# gone ("Keine Verbindung zum Antrieb"), exactly at the moment the
# interaction was supposed to start. At server start the robot is standing
# still, and it only happens once. False releases the chip when the
# interaction is off (e.g. for hailo-apps).
POSE_KEEP_WARM = True

# --- Gestures ----------------------------------------------------------------------

# How far back to look to see movements (waving, swiping).
GESTURE_WINDOW_S = 1.5

# Follow (one arm above the head) must be held this long -- otherwise
# every hair adjustment triggers it.
GESTURE_HOLD_S = 1.0

# Stop (both arms up) should act quickly, hence shorter.
GESTURE_STOP_HOLD_S = 0.4

# Share of frames in the hold period that must show the posture.
# Not 100 %: single frames with a lost wrist are normal.
GESTURE_SHARE = 0.8

# Waving (come here): minimum number of direction reversals of the wrist in
# the window and minimum amplitude, measured in shoulder widths (thus
# independent of the person's distance).
GESTURE_WAVE_REVERSALS = 2
GESTURE_WAVE_AMPLITUDE = 0.35

# Swiping (go away): the hand must move outward sideways by this many
# shoulder widths.
GESTURE_SWIPE_DISTANCE = 1.0

# After a detected gesture, no new one for this long -- a held gesture
# should not trigger again every second.
GESTURE_PAUSE_S = 2.0

# Debouncing the moving gestures (come here, go away). When walking the
# arms swing along, and from behind or while walking the person's size
# changes -- to the detection that looked too often like waving or swiping
# (while following it switched without a clear sign). So they only count
# if ...
#
# ... the person is standing roughly in place: size (shoulder width) between
# the start and end of the window differs by at most this factor ...
GESTURE_STILL_SIZE = 1.2
# ... and the shoulder center has moved at most this many shoulder widths;
GESTURE_STILL_DRIFT = 0.8
# ... the robot itself is standing still (its own driving and turning blur
# the image and shift the person): limits for the command currently being
# driven in m/s and rad/s;
GESTURE_ROBOT_STILL_V = 0.05
GESTURE_ROBOT_STILL_W = 0.3
# ... and the gesture was detected in this many evaluations in a row.
GESTURE_CONFIRM = 2
# In addition, the face (nose) must be visible in GESTURE_SHARE of the
# frames: whoever turns their back to the robot is not giving it a sign.
# STOP and FOLLOW (held postures) are exempt from this -- STOP must always
# work.

# --- Interaction (follow, come here, go away) --------------------------------------

# Desired distance when following. 1 m: close enough not to lose the
# person, far enough that they do not feel crowded.
FOLLOW_DISTANCE_M = 1.0

# Maximum speed when following. Leisurely walking is ~1 m/s; the robot does
# not have to keep up, it catches up when the person stops.
FOLLOW_MAX_SPEED = 0.30

# Only slowly backwards -- it is less careful when moving towards the rear.
FOLLOW_REVERSE_SPEED = 0.12

# Controller: speed per meter of distance error and turn rate per radian of
# angle error.
FOLLOW_KP_V = 0.6
# 1.2 instead of 1.5: the camera delivers the angle with ~0.1-0.2 s delay;
# tuned too aggressively it overshoots and oscillates (29.10.2026).
FOLLOW_KP_W = 1.2
FOLLOW_MAX_OMEGA = 0.8

# Smoothing of the camera angle (0..1, share of the new value per frame).
# The keypoints jitter by a few degrees from frame to frame; unsmoothed this
# turns into a twitching turn rate.
FOLLOW_ANGLE_SMOOTHING = 0.4

# Below this angle error (degrees) it does not correct -- the transition is
# continuous (the controller computes from the edge of the dead band), so
# that it does not jerk.
FOLLOW_ANGLE_DEADBAND = 3.0

# Within this distance error it stays put -- otherwise it jerks back and
# forth with every breath of the person.
FOLLOW_DEADBAND_M = 0.10

# Only drive straight when the person is roughly in front; otherwise turn
# first (otherwise it drives past them at an angle).
FOLLOW_MAX_ANGLE_DRIVE = 25.0
# Full speed up to this angle, gradually less above it up to
# FOLLOW_MAX_ANGLE_DRIVE -- previously vx jumped hard between 0 and full at
# the limit, which jerked and strained the power supply.
FOLLOW_FULL_ANGLE = 10.0

# If the person disappears only briefly (one or two frames without
# detection), it keeps coasting with the last command for this long instead
# of braking hard and starting again right away. Then it stops. The last
# good lidar distance also stays valid for this long if there is briefly no
# point in the person's direction.
FOLLOW_HOLD_S = 0.5

# Distance to the person from the lidar cluster instead of the smallest
# value within the box's angle: a chair or table in front of the person
# would otherwise be read as their distance, and the robot would stand
# still while they walk on. Clusters (navigation/obstacles.py) are searched
# within the box's angular range, widened by this margin -- camera and lidar
# do not see at exactly the same time; when turning this shifts by a few
# degrees.
FOLLOW_CLUSTER_EDGE_DEG = 5.0
# If a cluster deviates by more than this from the last good distance, it
# is probably not the person (at a walking pace of 0.3 m/s they cover at
# most ~0.3 m in FOLLOW_CLUSTER_GOOD_S). Only applies while the last good
# distance is younger than FOLLOW_CLUSTER_GOOD_S.
FOLLOW_CLUSTER_JUMP_M = 0.5
FOLLOW_CLUSTER_GOOD_S = 1.0

# If something was very close (EXPLORE_NEAR_M), it stays put for this long
# after it is clear -- otherwise it flickers at the boundary between
# stopping and driving.
FOLLOW_NEAR_HOLD_S = 0.5

# The person may be out of the image for this long before it searches, and
# it searches for this long (turning in the direction where they
# disappeared). 0 = until it sees someone again; cancel with stop or
# "Robi, gute Nacht".
FOLLOW_LOST_S = 2.0
FOLLOW_SEARCH_S = 0.0
# When searching, alternate between turning and holding still (s): during
# the turn the camera image blurs and pose detection misses the person.
# 1.2 s at EXPLORE_TURN_SPEED (0.6 rad/s) is about 40°, less than the field
# of view of the C270 -- so there is no gap in the all-round view.
FOLLOW_SEARCH_TURN_S = 1.2
FOLLOW_SEARCH_PAUSE_S = 0.8

# Come here: approach up to this distance, then stop.
COME_HERE_DISTANCE_M = 0.70

# Go away: back off this far, at this speed.
GO_AWAY_DISTANCE_M = 1.0
GO_AWAY_SPEED = 0.15

# Come here/go away end after this time at the latest, even if the goal has
# not been reached -- no behavior should run forever.
INTERACTION_BEHAVIOR_MAX_S = 20.0

# If the tracked person shifts by more than this angle in one frame, it is
# probably someone else -- then gesture detection starts over.
INTERACTION_JUMP_DEG = 15.0

# Soft start (s): when the interaction is switched on, at first only pose
# detection warms up, the robot stands still and stays silent. Then it says
# the start sentence ("Jetzt folge ich dir.") and only starts driving when
# the sentence is finished. On 03.11. pose detection, speech and motors
# started in the same second, and the USB supply collapsed. 0 = immediately.
INTERACTION_STARTUP_S = 5.0

# --- Whom it follows (interaction/target.py) ---------------------------------------
# A tracker assigns the people from frame to frame. They are not told
# apart -- whoever was briefly out of the image is the target again on
# coming back.

# If the followed person is not visible for this long (s), it follows
# another person in the image (the one closest to the last angle). Covered
# for less time, e.g. by someone walking past: it waits for them.
TARGET_SWITCH_S = 1.0

# Angular window around the predicted position within which a detection may
# be assigned to the track (grows with uncertainty up to three times).
TARGET_GATE_DEG = 15.0

# A track without a measurement is kept (predicted) for this long before it
# is discarded.
TARGET_TRACK_HOLD_S = 1.5

# The box height may change at most this much between prediction and
# measurement (ratio small/large), otherwise it is not the same track.
TARGET_HEIGHT_RATIO = 0.6

# If the person disappears closer than this many degrees to the image edge,
# they have almost certainly walked out sideways -- then it already searches
# after FOLLOW_LOST_EDGE_S instead of FOLLOW_LOST_S. In the middle of the
# image the longer period remains (there it is more likely a brief
# occlusion).
TARGET_EDGE_DEG = 8.0
FOLLOW_LOST_EDGE_S = 0.7

# When searching, it turns towards a lidar person cluster (two legs) instead
# of turning on blindly. If the cluster is in the image, it waits this long
# for the camera; if the camera sees nobody there, it keeps turning as
# usual.
TARGET_HINT_S = 1.5

# --- Diagnostics (page /diagnose) ------------------------------------------------
# How often the fast measurements run (battery, USB over-current counter,
# device state) and how often the slow ones (temperature, vcgencmd, kernel
# log, process list). The slow ones start subprocesses -- every second would
# be unnecessary load on a Pi whose USB suffers from load spikes anyway.
DIAG_INTERVAL_S = 1.0
DIAG_SLOW_S = 5.0

# Battery thresholds for the X3's 3S battery (full 12.6 V). Below 11.1 V the
# motors' inrush current is enough to make the 5 V dip briefly; below
# 10.5 V the battery gets damaged and the board keeps disconnecting.
BATTERY_WARN_V = 11.1
BATTERY_CRITICAL_V = 10.5

# If the voltage falls this far below the maximum of the last minute, it
# counts as a voltage dip (typically: motors starting at the same time).
VOLTAGE_DROP_V = 0.6

# From this CPU temperature on the page warns. The Pi 5 throttles itself at
# 85 °C -- you should see it before that (on 28.10. a build ran up to
# 83 °C).
TEMP_WARN_C = 75.0

# Above this clock speed USB resets occurred on the X3, see docs/hardware.md.
CPU_MAX_MHZ = 2000

# A problem counts as over when it has not occurred for this long.
PROBLEM_WINDOW_S = 600.0

# After server start, lidar and cameras need a few seconds until the first
# data. Until then "delivers nothing" does not count as a problem yet.
DIAG_STARTUP_S = 15.0

# The server keeps this many log lines and events in memory.
DIAG_LOG_LINES = 1000
DIAG_EVENTS = 300

# Battery history: one point per fast measurement, this many points (30 min).
DIAG_BATTERY_POINTS = 1800

# At start, look back this far in the kernel log (USB disconnects that
# happened before server start, e.g. during boot).
DIAG_KERNEL_LOOKBACK_S = 3600.0

# --- Crash capture (since 29.10.) ---
# "Random crashes" could not be attributed so far: was it the server
# (error, memory, SSH closed), the whole Pi (reset, power) or just a hang?
# For this, the diagnostics record in this folder what survives a crash:
# run log, server log, stack dumps.
DIAG_FOLDER = Path.home() / ".rosmaster" / "diagnose"

# The run log (laeufe.json) keeps this many server runs.
DIAG_RUNS = 20

# Size of one server log file; two old files are kept in addition.
DIAG_LOGFILE_BYTES = 2_000_000

# Event loop pulse: every DIAG_PULSE_S the server checks how punctually it
# was woken up. If it is more than DIAG_HANG_S late, something has blocked
# the loop (then the emergency stop does not react either). If the pulse
# stops entirely for DIAG_BLOCKED_S, the diagnostics write down what the
# loop is currently stuck on.
DIAG_PULSE_S = 0.5
DIAG_HANG_S = 1.0
DIAG_BLOCKED_S = 3.0

# From here on the diagnostics warn: little free memory (otherwise the
# kernel kills the largest process -- usually the server) and too many open
# files (a leak that eventually makes every open fail).
DIAG_MEMORY_WARN_MB = 200
DIAG_FILES_WARN = 800

# A hanging browser must not hold up the status messages to everyone else:
# whoever has not accepted anything after this many seconds is dropped.
WS_SEND_TIMEOUT_S = 2.0

# Background loops catch errors and keep running. So that an error that
# occurs on every tick does not flood the log, it appears at most this
# often.
LOOP_ERROR_LOG_S = 10.0

# USB storm: this many over-current messages per minute *while standing* are
# no longer inrush current but a persistent fault in the power supply of the
# USB ports. On 29.10. there were 300-400 per minute while idle.
USB_STORM_PER_MIN = 20

# --- Drive recorder (recording.py, since 03.11.) ----------------------------
# Keeps the last seconds of a drive (lidar revolutions, drive commands,
# board velocity, states) in memory. Only "Fahrt speichern" in the app
# writes them to the SD card; evaluate them with scripts/replay.py.
RECORDING_ACTIVE = True
# One minute is enough to see the situation before an error and needs about
# 10 MB of memory (450 lidar revolutions with ~360 points each).
RECORDING_BUFFER_S = 60.0
RECORDING_FOLDER = Path.home() / ".rosmaster" / "aufzeichnungen"
# One file is a few hundred KB compressed; older ones are deleted.
RECORDING_MAX_FILES = 20

# --- Music ("MUKKE") ----------------------------------------------------------------

# YouTube links from the app are played with mpv on the USB speaker, audio
# only. Prerequisites on the Pi:
#   sudo apt install mpv
#   robot/.venv/bin/pip install -U yt-dlp
# yt-dlp from pip instead of apt: YouTube keeps changing things, and the
# version from the package sources is usually too old to still play.
MUKKE_ENABLED = True

# Output device for mpv. In ALSA the USB speaker is called "UACDemoV10"
# (see `aplay -l`). plughw instead of hw: converts sample rate and format
# that the speaker cannot handle itself. "auto" uses the default device.
MUKKE_AUDIO_DEVICE = "alsa/plughw:CARD=UACDemoV10,DEV=0"

# Music volume in percent (0..100), as a share of VOLUME.
# Adjustable in the app.
MUKKE_VOLUME = 60

# --- Sound ---------------------------------------------------------------------

# Master volume in percent (0..100): the single slider for everything that
# comes out of the speaker (voice and music). Computed in software; the
# speaker's mixer is fixed at SOUND_MIXER. Adjustable on the settings page.
# Quieter also means: less current on the Pi's USB.
VOLUME = 70

# Hardware mixer of the speaker (amixer) in percent. Always stays the same,
# so that there is only one slider (VOLUME).
SOUND_MIXER = 100

# --- Voice (USB speaker) ------------------------------------------------------
# Speech output via the USB speaker "UACDemoV1.0". See docs/interaction.md
# and docs/hardware.md (setup).

# Can be switched off, e.g. during a lecture, without touching the code.
VOICE_ENABLED = True

# Piper speaks offline and sounds much more natural than espeak-ng.
# If Piper or the model is missing, the voice falls back to espeak-ng and
# finally to plain log output -- the server starts in any case.
# Locations for the program, the first one found wins. The release from
# github.com/rhasspy/piper is unpacked to ~/piper/piper/.
VOICE_PIPER = ("~/piper/piper/piper", "piper")
# Female voice "Kerstin" (16 kHz). Thorsten (male) is also available under
# ~/piper/de_DE-thorsten-medium.onnx; to switch, only change it here.
VOICE_MODEL = "~/piper/de_DE-kerstin-low.onnx"
VOICE_ESPEAK = "espeak-ng"

# Let Piper compute only on these CPU cores (taskset; None = all).
# On all four cores every sentence was a load spike, and on 03.11. all USB
# devices dropped out exactly while speaking (over-current). With one core
# Piper is hardly slower (measured 1.27 instead of 1.28 s for 3.5 s of
# speech).
VOICE_CORES = "3"

# ALSA device via the card name instead of the number: the number changes
# when USB devices re-register after a voltage dip.
# "plughw" converts sample rate and channels as needed.
VOICE_DEVICE = "plughw:CARD=UACDemoV10,DEV=0"

# Speaking rate for Piper: >1 slower. Slightly slower is easier to
# understand in a room with motor noise.
VOICE_SPEED = 1.1

# No more sentences than this wait in line -- otherwise the robot is still
# talking about something that is long over. Older ones are dropped.
VOICE_MAX_QUEUE = 3

# The same sentence is not repeated within this time (e.g. when the person
# is lost and found again several times in a row).
VOICE_REPEAT_S = 4.0

# Longer texts are cut off; the app should not have novels read aloud.
VOICE_MAX_CHARS = 300

# No sentence may block the speech thread for longer than this.
VOICE_TIMEOUT_S = 20.0

# For this long after the end of a sentence the voice still counts as
# "speaking": reverberation in the room and buffering in the speaker --
# speech recognition should not take that as input.
VOICE_TAIL_S = 0.4

# Announcement: speak live from the app (laptop/phone microphone) through
# the robot's speaker (WebSocket /sprache/durchsage). If no audio arrives
# for this long (s), it counts as finished and the speaker is released ...
ANNOUNCE_PAUSE_S = 2.0
# ... and no more than this much (s) audio is buffered. If the announcement
# was queued behind a sentence, the beginning is dropped instead of it
# lagging seconds behind permanently.
ANNOUNCE_MAX_BUFFER_S = 0.5
# aplay buffer (µs). Smaller = less delay, but every WiFi stutter becomes
# audible.
ANNOUNCE_APLAY_BUFFER_US = 200000

# --- Speech commands (USB microphone, Vosk) ---------------------------------------
# See interaction/speech.py and docs/interaction.md. Without vosk/sounddevice,
# model or microphone, recognition is off; the server runs anyway.
SPEECH_ENABLED = True

# Small German Vosk model (~45 MB, runs on the Pi's CPU).
# Download: see docs/hardware.md.
SPEECH_MODEL = "~/vosk-model-small-de-0.15"

# None = search automatically (by SPEECH_MICROPHONE_NAMES), otherwise part
# of the device name (str) or the sounddevice device number (int).
SPEECH_MICROPHONE = None

# Order = preference. Only plugged-in USB microphones: the FongLun
# ("USB Microphone", hw:5 on 03.11.) or C-Media ("USB PnP Sound Device").
# The camera microphones are deliberately not included: the C270's sounds
# so bad that Vosk understands hardly anything -- better deaf and honestly
# "kein Mikrofon" in /sprache/status than half hearing.
SPEECH_MICROPHONE_NAMES = ("USB Microphone", "USB PnP", "C-Media", "Microphone")

# The automatic search never takes devices with these name parts, even if
# an entry above matches (such as "Webcam Microphone"). An explicitly set
# SPEECH_MICROPHONE still applies.
SPEECH_MICROPHONE_NEVER = ("C270", "Logi", "WebCam", "ORBBEC", "Depth Sensor")

# The robot's name. Vosk does not know "robi" as a word; depending on
# pronunciation it hears one of the variants. Only those in the model are
# used.
SPEECH_WAKE_WORD = "robi"
SPEECH_WAKE_WORD_VARIANTS = ("robbie", "robby", "roby", "robi", "ruby", "rubi", "robin")

# For this long (s) after a lone "Robi", the command alone is enough.
SPEECH_WAKE_WINDOW_S = 5.0

# "Stopp"/"halt" also works without the name: stopping must never fail
# because the name was missed. A false stop does no harm.
SPEECH_STOP_WITHOUT_WAKE_WORD = True

# "La-da-da" plays this song via MUKKE. Without wake word: whoever sings
# "la da da" wants music -- and a false alarm does not drive anywhere. Only
# the video address, without &list=...: the song, not the whole playlist.
SPEECH_LADADA_URL = "https://www.youtube.com/watch?v=j-S_w3tK3dw"
SPEECH_LADADA_WITHOUT_WAKE_WORD = True

# "Robi, Beatbox" plays this song (with wake word, like the other commands).
SPEECH_BEATBOX_URL = "https://www.youtube.com/watch?v=EfSp8EjZikE"

# With a fixed grammar, Vosk turns every noise into an allowed sentence:
# music, TV and conversations became "robbie hallo" and switched the
# interaction on. So a sentence only counts if Vosk is at least this
# confident (0..1) about every word ...
SPEECH_MIN_CONFIDENCE = 0.7
# ... and at most this share of the words was unknown ([unk]). More means:
# someone was talking, and "komm" was just one word in it.
SPEECH_MAX_UNK_SHARE = 0.5
# ... and at most this many further words came besides the wake word and
# the phrase. On 03.11. noise turned into "robin hallo hause zu weg ruby"
# (-> go away); a spoken command is exactly one grammar sentence.
SPEECH_MAX_EXTRA_WORDS = 0
# Uncertain input only lets stop through. When tuning, "konfidenz" and
# "zuletzt_verworfen" in /sprache/status help.

# While music is playing, the microphone hears the song too. Then a command
# only counts with "Robi" and with this higher confidence (stop and
# "Musik aus" always).
SPEECH_MIN_CONFIDENCE_MUSIC = 0.85

# Reply to a lone "Robi" (None = silent). It is only said after this pause
# (s): whoever says "Robi ... folge mir" with a short pause should not be
# interrupted by the "Ja?" -- while the robot is speaking, it does not
# listen.
SPEECH_WAKE_REPLY = "Ja?"
SPEECH_WAKE_REPLY_AFTER_S = 1.0

# Live feed from the microphone for the app (transcript and listen-in, both
# switchable in the app). The transcript shows this many recently heard
# sentences ...
SPEECH_LIVE_EVENTS = 15
# ... and for this long (s) after the last request the listening thread
# still computes level and partial text. If nobody is reading along, it
# costs nothing.
SPEECH_LIVE_HOLD_S = 3.0
# Listen-in: buffer this many blocks (0.25 s each) before some are dropped.
# Keep it small, otherwise you hear with a delay.
SPEECH_SOUND_BUFFER = 8

# Vosk models are trained on 16 kHz. If the microphone cannot do that, its
# own rate is used and Vosk resamples.
SPEECH_SAMPLE_RATE = 16000

# Wait time (s) before the next attempt when the microphone is gone.
SPEECH_RETRY_S = 3.0

# Failsafe after USB dropouts. After a crash the C270 came back faster than
# the USB microphone, and recognition then stayed stuck on the worse C270
# (which is why it is now excluded entirely). If it runs on a less
# preferred microphone, it checks this often (s) whether a better one has
# registered again, and switches ...
SPEECH_MICROPHONE_CHECK_S = 5.0
# ... if it was already there when opening and PortAudio still did not find
# it, it only tries again after this time (s) (no constant switching).
SPEECH_MICROPHONE_RETRY_S = 30.0
# If the microphone delivers only exact zeros for this long (s), the stream
# is dead (the device is gone without PortAudio reporting an error): reopen.
SPEECH_SILENT_S = 5.0
# If no block at all arrives for this long (s), the stream hangs: after a
# USB dropout, reading stopped forever without an error (level frozen,
# state "hoert", no more commands). Then reopen.
SPEECH_HANG_S = 3.0
# The listening thread buffers this many blocks (0.25 s each) when the
# evaluation briefly cannot keep up (Piper computing); older ones are
# dropped.
SPEECH_BLOCK_BUFFER = 20
# Closing a dead stream may take this long (s). If that hangs as well, it
# is abandoned so that the retry does not hang along with it.
SPEECH_CLOSE_S = 2.0
# If the listening thread itself makes no progress for this long (s) (e.g.
# even opening the microphone hangs in PortAudio), the watchdog starts a
# new one. Must be longer than a whole retry (deadline + closing + waiting).
SPEECH_THREAD_HANG_S = 15.0
# At most this many times in a row; after that state "fehler" with the hint
# to restart the server (every hanging thread is left behind).
SPEECH_THREAD_MAX_RESTARTS = 3
# During mapping the microphone is closed (no speech commands): the stream
# loads the USB bus that the lidar is also on, and Vosk loads the CPU that
# SLAM needs; a misheard "folgen" would also abort the mapping drive.
SPEECH_OFF_WHILE_MAPPING = True
# How often (s) the listening thread checks during the pause whether it is
# over.
SPEECH_PAUSE_CHECK_S = 0.5
