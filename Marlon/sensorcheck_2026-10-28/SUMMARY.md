# Sensor check ROSMASTER X3 (pi-bravo) – 28.10.2026

Tests: phase A (Rosmaster_Lib directly, wheels in the air), phase B (ROS2, wheels in the air),
hand test (lidar alignment), floor test (odometry/gyro/lidar against each other).
Raw data: `phase_a.*`, `phase_b.*`, `hand.txt`, `boden.*`, `dmesg_*.txt`, scripts `sensorcheck_*.py`.

## 1. Key finding: cause of the USB resets

Until now we thought: power from the battery/hub or the Pi's 600 mA limit. Neither is true.

- **The trigger is a CPU load spike at 2.4 GHz.** Measured without ROS (`while :; do :; done` on 4 cores, 5 s each):

  | Max. CPU clock | Idle 20 s | Full-load burst |
  |---|---|---|
  | 2.4 GHz (default) | 0 over-current | ~15 over-current per burst, hub drops out |
  | 2.0 GHz | – | 0 |
  | 1.8 GHz | – | 0 |
  | 1.5 GHz | – | 0 |

- That is why the resets always came when a new Python/ROS process started. This happened when starting the hand test, with `ros2 topic echo/list`, at boot, when starting the lidar and when colleagues started something.
- The 5 V supply only drops from 5.09 V to 5.01 V. `throttled=0x0`, so no real undervoltage.
- The over-current message appears simultaneously on all root ports, even empty ones. This points to **backfeed**: the hub is probably powered via the expansion board. If the Pi rail sags a little, current flows backwards into the Pi's VBUS switch.
- The EEPROM setting `PSU_MAX_CURRENT=5000` (1.6 A USB) is active, but does not help against the resets.
- **After the reset:** the ttyUSB numbers change, but the symlinks `/dev/myserial` and `/dev/rplidar` stay correct. The driver dies with `SerialException ... multiple access on port`. The lidar node hangs without printing an error message.
- **Software fix:** limit the CPU clock to max. 2.0 GHz.
  - Set at runtime for the test (`scaling_max_freq=2000000`). After that there were 0 resets, including during the hand test, the floor test and lidar and driver restarts.
  - The setting is lost after a reboot. It could be made permanent with `arm_freq=2000` in `/boot/firmware/config.txt` (also affects Ben).
- **Hardware (cleaner):** prevent the backfeed, i.e. use a hub without backfeed or disconnect VBUS to the Pi.

## 2. Lidar

- RPLIDAR (A1 series), S/N 74BEED95C4E493CBA5E69EF0E56E4B6F, FW 1.29, HW rev 7, Sensitivity mode at 8 kHz, max. 12 m.
- `/scan`: frame_id `laser`, 1080 points, angle −179…180°, increment 0.333°, range_min 0.15, range_max 12. Intensities are present, ~83 % of the values valid.
- **Actual scan rate 7.5–7.8 Hz** instead of the configured 10 Hz (scan_time 0.1345 s).
- **Alignment (hand test 3, unambiguous):**
  - A hand in front stayed constantly at raw angle **−87…−90°** for 20 s.
  - Left was at about −26°: body/arm diagonally front left, about ~80° wide. That is close to 0° and not −180°.
  - The floor test confirms the direction of rotation: on a left turn the surroundings move by −65°, odometry and lidar agree.
  - → Raw angles run counter-clockwise, **front = −90°, left = 0°**. The setting **`--no-flip --front-offset -90`** (commit 59013a5) is correct.
- **Start order:** start the lidar first, wait ~10 s, then the driver. With 2.0 GHz there were no more resets when starting the lidar.

## 3. IMU (`/imu/data_raw`, 10 Hz, frame `imu_link`)

- **Gyro z has a large constant bias: +0.389 rad/s (≈ 22°/s).**
  - It is very stable (std 0.0007) and the same whether in the air, with the wheels running or on the floor.
  - Without correction the rotation is unusable.
- **Gyro z has the wrong sign.**
  - On a left turn (+65° according to lidar and odometry) the gyro measures −70° after subtracting the bias.
  - On a right turn (−64°) it measures +67°.
  - → To use it: `yaw_rate = -(gz - 0.389)`. The scale is ~1.05–1.08 too large; the lidar resolution of 1° does not allow a more precise statement.
- Gyro bias x/y: 0.112 and 0.085 rad/s.
- Acceleration: (0.17, −0.07, 10.61). z is ~8 % too high.
- Orientation always w=1 (no fusion), covariances 0. `attitude` from the lib is always 0.
- **Magnetometer:** e.g. (−241, −21, 176) with std ~13. It deviates strongly between measurements (phase A: −246, −117, 213). **Unusable as a compass** (motors/battery nearby).

## 4. Odometry (`/vel_raw`, 10 Hz) and drive commands

- **vel_raw matches the real motion well** (measured on the floor with the lidar):

  | Command (2 s or 90°) | Target | Odometry | Lidar (real) |
  |---|---|---|---|
  | forward 0.15 m/s | 0.30 m | 0.266 m | 0.248 m |
  | backward | −0.30 m | −0.269 m | −0.272 m |
  | left 0.15 m/s | 0.30 m | 0.212 m | 0.208 m |
  | right | −0.30 m | −0.205 m | −0.200 m |
  | turn left 0.5 rad/s | 90° | 68.6° | 65° |
  | turn right | −90° | −66.9° | −64° |

- **The command alone overestimates the motion by 10–30 %.**
  - Reasons: latency ~0.22 s, rise to 90 % after ~0.72 s, coast-down ~0.23 s. Sideways the robot only reaches ~70 %.
  - In the air, the steady-state speed was ≈ the command (vx 0.206 / vy 0.206 / wz 1.04 at 0.2/0.2/1.0).
- **Consequence for `x3_explorer.py`:** line 292 (`self.yaw += c.wz * dt`) integrates the *command*. As a result, the estimated rotation overshoots by ~30 % on short turns. It would be better to integrate `vel_raw.angular.z`, or to use the gyro with bias and sign correction.
- Driving sideways rotates the robot slightly as well, by ~3° over 0.2 m.
- `--latency 0.25` fits the measurements.

## 5. Motors / board (phase A)

- Firmware 2.4, car_type 1, PID [0.8, 0.06, 0.5].
- Mapping: M1 front left, M2 rear left, M3 front right, M4 rear right.
- Encoder: ~1320 counts/revolution, ~5250 counts/m. PWM 100 ≈ 6400 counts/s (~1.2 m/s in the air).
- All motors at full load, lidar off: no USB resets.
- ROS topics: /scan, /imu/data_raw, /imu/mag, /vel_raw, /voltage, /edition (2.4), /cmd_vel, /Buzzer, /RGBLight. `/joint_states` is created but not published.

## 6. Battery and system

- Battery 10.3 V → 10.1 V during the tests, min. 10.2 V under load. **Charge soon.**
- Pi 5 with 16 GB, Debian 13, kernel 6.18.50, 46 °C, RAM and disk free.
- Running on the host: jupyter-lab, wayvnc, yahboom_oled. The Yahboom app is not running.
- Other SSH sessions: 10.42.0.100 (`nano autonom_ausweichen.py`) and 10.42.0.206 (`sudo su`). 10.42.0.194 was our own laptop.

## 7. Changed system settings (outside Marlon/, also affect Ben)

- **EEPROM:** added `PSU_MAX_CURRENT=5000`.
  - Revert with `sudo rpi-eeprom-config --apply ~/sensorcheck/eeprom_vorher.txt` and a reboot.
- **`/boot/firmware/config.txt`:** back to its original state. The test with `usb_max_current_enable` had no effect because the firmware ignores the option.
- **CPU clock:** limited to max. 2.0 GHz at runtime, back to 2.4 GHz after a reboot.
