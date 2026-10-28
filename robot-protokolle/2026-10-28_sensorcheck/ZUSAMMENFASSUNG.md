# Sensor-Check ROSMASTER X3 (pi-bravo) – 28.10.2026

Tests: Phase A (Rosmaster_Lib direkt, schwebend), Phase B (ROS2, schwebend),
Hand-Test (Lidar-Ausrichtung), Bodentest (Odometrie/Gyro/Lidar gegeneinander).
Rohdaten: `phase_a.*`, `phase_b.*`, `hand.txt`, `boden.*`, `dmesg_*.txt`, Skripte `sensorcheck_*.py`.

## 1. Wichtigster Befund: Ursache der USB-Resets

Bisher dachten wir: Strom vom Akku/Hub oder die 600-mA-Grenze des Pi. Beides stimmt nicht.

- **Auslöser ist ein CPU-Lastsprung bei 2,4 GHz.** Messung ohne ROS (`while :; do :; done` auf 4 Kernen, je 5 s):

  | CPU-Takt max. | Ruhe 20 s | Volllast-Stoß |
  |---|---|---|
  | 2,4 GHz (Standard) | 0 Over-Current | ~15 Over-Current je Stoß, Hub fällt ab |
  | 2,0 GHz | – | 0 |
  | 1,8 GHz | – | 0 |
  | 1,5 GHz | – | 0 |

- Daher kamen die Resets immer dann, wenn ein neuer Python-/ROS-Prozess startete. Das passierte beim Start des Hand-Tests, bei `ros2 topic echo/list`, beim Booten, beim Lidar-Start und wenn Kollegen etwas starteten.
- Die 5-V-Versorgung fällt dabei nur von 5,09 V auf 5,01 V. `throttled=0x0`, also keine echte Unterspannung.
- Die Over-Current-Meldung kommt gleichzeitig auf allen Root-Ports, auch auf leeren. Das spricht für **Rückspeisung (Backfeed)**: Der Hub wird vermutlich über das Expansion-Board versorgt. Sinkt die Pi-Schiene etwas, fließt Strom rückwärts in den VBUS-Schalter des Pi.
- Die EEPROM-Einstellung `PSU_MAX_CURRENT=5000` (1,6 A USB) ist aktiv, hilft aber nicht gegen die Resets.
- **Nach dem Reset:** Die ttyUSB-Nummern wechseln, die Symlinks `/dev/myserial` und `/dev/rplidar` stimmen aber weiter. Der Treiber stirbt mit `SerialException ... multiple access on port`. Der Lidar-Knoten bleibt hängen, ohne eine Fehlermeldung auszugeben.
- **Lösung per Software:** CPU-Takt auf max. 2,0 GHz begrenzen.
  - Zum Test zur Laufzeit gesetzt (`scaling_max_freq=2000000`). Danach gab es 0 Resets, auch beim Hand-Test, beim Bodentest und bei Lidar- und Treiber-Neustarts.
  - Die Einstellung ist nach einem Neustart weg. Dauerhaft ginge es mit `arm_freq=2000` in `/boot/firmware/config.txt` (betrifft auch Ben).
- **Hardware (sauberer):** Die Rückspeisung unterbinden, also einen Hub ohne Backfeed verwenden oder VBUS zum Pi trennen.

## 2. Lidar

- RPLIDAR (A1-Reihe), S/N 74BEED95C4E493CBA5E69EF0E56E4B6F, FW 1.29, HW Rev 7, Modus Sensitivity mit 8 kHz, max. 12 m.
- `/scan`: frame_id `laser`, 1080 Punkte, Winkel −179…180°, Inkrement 0,333°, range_min 0,15, range_max 12. Intensitäten sind vorhanden, ~83 % der Werte gültig.
- **Echte Scanrate 7,5–7,8 Hz** statt der eingestellten 10 Hz (scan_time 0,1345 s).
- **Ausrichtung (Hand-Test 3, eindeutig):**
  - Hand vorne lag 20 s lang konstant bei Rohwinkel **−87…−90°**.
  - Links lag bei ca. −26°: Körper/Arm schräg links vorn, über ~80° breit. Das ist nahe 0° und nicht −180°.
  - Der Bodentest bestätigt den Drehsinn: Bei einer Linksdrehung wandert die Umgebung um −65°, Odometrie und Lidar stimmen überein.
  - → Rohwinkel laufen gegen den Uhrzeigersinn, **vorne = −90°, links = 0°**. Die Einstellung **`--no-flip --front-offset -90`** (Commit 59013a5) ist richtig.
- **Start-Reihenfolge:** zuerst den Lidar starten, ~10 s warten, dann den Treiber. Mit 2,0 GHz traten beim Lidar-Start keine Resets mehr auf.

## 3. IMU (`/imu/data_raw`, 10 Hz, frame `imu_link`)

- **Gyro z hat einen großen konstanten Bias: +0,389 rad/s (≈ 22°/s).**
  - Er ist sehr stabil (std 0,0007) und gleich, egal ob schwebend, mit laufenden Rädern oder am Boden.
  - Ohne Korrektur ist die Drehung unbrauchbar.
- **Gyro z hat das falsche Vorzeichen.**
  - Bei einer Linksdrehung (laut Lidar und Odometrie +65°) misst der Gyro nach Bias-Abzug −70°.
  - Bei einer Rechtsdrehung (−64°) misst er +67°.
  - → Zum Verwenden gilt `yaw_rate = -(gz - 0,389)`. Die Skala ist ~1,05–1,08 zu groß; mehr lässt die Lidar-Auflösung von 1° nicht genau sagen.
- Gyro-Bias x/y: 0,112 und 0,085 rad/s.
- Beschleunigung: (0,17, −0,07, 10,61). z liegt ~8 % zu hoch.
- Orientierung immer w=1 (keine Fusion), Kovarianzen 0. `attitude` aus der Lib ist immer 0.
- **Magnetometer:** z. B. (−241, −21, 176) mit std ~13. Zwischen den Messungen weicht es stark ab (Phase A: −246, −117, 213). **Als Kompass unbrauchbar** (Motoren/Akku in der Nähe).

## 4. Odometrie (`/vel_raw`, 10 Hz) und Fahrbefehle

- **vel_raw stimmt gut mit der echten Bewegung überein** (am Boden mit dem Lidar gemessen):

  | Befehl (2 s bzw. 90°) | Soll | Odometrie | Lidar (echt) |
  |---|---|---|---|
  | vor 0,15 m/s | 0,30 m | 0,266 m | 0,248 m |
  | zurück | −0,30 m | −0,269 m | −0,272 m |
  | links 0,15 m/s | 0,30 m | 0,212 m | 0,208 m |
  | rechts | −0,30 m | −0,205 m | −0,200 m |
  | drehen links 0,5 rad/s | 90° | 68,6° | 65° |
  | drehen rechts | −90° | −66,9° | −64° |

- **Der Befehl allein überschätzt die Bewegung um 10–30 %.**
  - Gründe: Latenz ~0,22 s, Anstieg auf 90 % nach ~0,72 s, Auslauf ~0,23 s. Seitwärts erreicht der Roboter nur ~70 %.
  - Schwebend war die stationäre Geschwindigkeit ≈ Befehl (vx 0,206 / vy 0,206 / wz 1,04 bei 0,2/0,2/1,0).
- **Folge für `x3_explorer.py`:** Zeile 292 (`self.yaw += c.wz * dt`) integriert den *Befehl*. Dadurch läuft die geschätzte Drehung bei kurzen Drehungen ~30 % zu weit. Besser wäre es, `vel_raw.angular.z` zu integrieren, oder den Gyro mit Bias und Vorzeichen-Korrektur zu verwenden.
- Seitwärtsfahrt dreht den Roboter leicht mit, um ~3° auf 0,2 m.
- `--latency 0,25` passt zu den Messungen.

## 5. Motoren / Board (Phase A)

- Firmware 2.4, car_type 1, PID [0,8, 0,06, 0,5].
- Zuordnung: M1 vorne links, M2 hinten links, M3 vorne rechts, M4 hinten rechts.
- Encoder: ~1320 Counts/Umdrehung, ~5250 Counts/m. PWM 100 ≈ 6400 Counts/s (~1,2 m/s in der Luft).
- Volllast aller Motoren, Lidar aus: keine USB-Resets.
- ROS-Topics: /scan, /imu/data_raw, /imu/mag, /vel_raw, /voltage, /edition (2.4), /cmd_vel, /Buzzer, /RGBLight. `/joint_states` ist angelegt, wird aber nicht veröffentlicht.

## 6. Akku und System

- Akku 10,3 V → 10,1 V während der Tests, unter Last min. 10,2 V. **Bald laden.**
- Pi 5 mit 16 GB, Debian 13, Kernel 6.18.50, 46 °C, RAM und Platte frei.
- Auf dem Host laufen jupyter-lab, wayvnc, yahboom_oled. Die Yahboom-App läuft nicht.
- Andere SSH-Sitzungen: 10.42.0.100 (`nano autonom_ausweichen.py`) und 10.42.0.206 (`sudo su`). 10.42.0.194 war der eigene Laptop.

## 7. Geänderte Systemeinstellungen (außerhalb von Marlon/, betreffen auch Ben)

- **EEPROM:** `PSU_MAX_CURRENT=5000` ergänzt.
  - Zurück mit `sudo rpi-eeprom-config --apply ~/sensorcheck/eeprom_vorher.txt` und Neustart.
- **`/boot/firmware/config.txt`:** wieder im Originalzustand. Der Test mit `usb_max_current_enable` hatte keine Wirkung, weil die Firmware die Option ignoriert.
- **CPU-Takt:** zur Laufzeit auf max. 2,0 GHz begrenzt, nach einem Neustart wieder 2,4 GHz.
