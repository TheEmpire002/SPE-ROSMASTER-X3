# ROSMASTER X3 – Autonomes Erkunden mit Lidar

Der Roboter fährt nach dem Start selbstständig herum, umfährt Hindernisse mit dem
RPLIDAR und nutzt die Mecanum-Räder, um seitlich an Hindernissen vorbeizugleiten.

## Was er macht

| Zustand     | Verhalten                                              | LED        |
|-------------|--------------------------------------------------------|------------|
| UMSCHAUEN   | beim Start einmal langsam 360° drehen, dann losfahren in die offenste Richtung | Farbverlauf |
| ERKUNDEN    | fährt flüssig, bevorzugt Bereiche, wo er noch nicht war, hält Abstand zu Wänden, bremst sanft vor Hindernissen | Lauflicht |
| SCHAUEN     | alle 40–80 s: anhalten, nach links/rechts „umsehen“, neue Richtung wählen | Sternenlicht |
| AUSWEG      | Sackgasse: piepen, ein Stück zurück, zur größten Lücke drehen | Atmen |

Technik: pro Takt (20 Hz) werden 72 Fahrtrichtungen bewertet. Jede wird als
roboterbreiter Korridor geprüft, nicht nur als einzelner Laserstrahl.
Beschleunigungen sind begrenzt, deshalb wirkt die Bewegung „überlegt“ statt ruckartig.

## Starten – ein Befehl

Auf dem Pi (User `pi`, **nicht** im Docker-Container), im Repo-Ordner:

```bash
cd ~/SPE-ROSMASTER-X3          # bzw. wo das Repo liegt
git pull
bash Marlon/x3.sh              # Container + Treiber + Lidar + losfahren
```
**Stoppen: Ctrl+C**. Der Roboter wird angehalten und alle Prozesse beendet.

`x3.sh` erledigt alles selbst:
1. beendet die Yahboom-Werks-App (`rosmaster_main.py`), die sonst die seriellen Ports belegt,
2. startet den ROS-Container `x3` (Image `yahboomtechnology/ros-foxy:4.0.8`) mit
   `-v /dev:/dev`, `--ipc=host` und diesem Repo unter demselben Pfad,
3. startet darin `start_autonomy.sh` (Treiber → 8 s warten → Lidar → `x3_explorer.py`).

| Befehl | Wirkung |
|--------|---------|
| `bash Marlon/x3.sh` | losfahren (max. 0,30 m/s) |
| `bash Marlon/x3.sh --max-v 0.2` | langsamer; alle Optionen gehen an `x3_explorer.py` |
| `bash Marlon/x3.sh --calibrate` | Lidar-Ausrichtung prüfen, fährt **nicht** |
| `bash Marlon/x3.sh stop` | anhalten (z. B. aus einem zweiten Terminal) |
| `bash Marlon/x3.sh neustart` | alle ROS-Container beenden, frisch starten, losfahren |
| `bash Marlon/x3.sh shell` | Terminal im Container (`ros2 topic echo /scan` …) |
| `bash Marlon/x3.sh hochladen` | Protokolle committen und pushen |

**Wichtig:**
- Läuft noch ein anderer ROS-Container (z. B. aus `~/run_docker.sh`), bricht `x3.sh`
  mit einer Meldung ab → `bash Marlon/x3.sh neustart`.
- Läuft die Web-App (`python main.py`, Port 8000), bricht der Start ab: sie benutzt
  dieselbe Motorplatine und denselben Lidar. Erst die Web-App beenden.
- Die Yahboom-App startet nach jedem Booten neu (`~/.config/autostart/`); `x3.sh`
  beendet sie jedes Mal.

## 1. Kalibrieren (einmalig, Roboter fährt NICHT)

```bash
bash Marlon/x3.sh --calibrate
```
Hand ca. 30 cm **vor** den Roboter halten: es muss `VORNE` und ca. `0°` erscheinen.
Hand **links**: ca. `+90°`.

Unser X3 ist bereits eingestellt (`--front-offset 90`, gespiegelt).
Passt es nach einem Umbau nicht mehr: `--front-offset` (Grad) anpassen bzw.
`--no-flip`, falls links und rechts vertauscht sind.

## 2. Fahren

Weitere Optionen: `--no-look-around`, `--no-sound`, `--no-led`, `--radius`,
`--margin`. Alle anzeigen mit `python3 x3_explorer.py --help`.
Unbekannte Optionen (Tippfehler) werden abgelehnt.

In RViz zeigt ein grüner Pfeil auf `/x3_explorer/direction` die gewählte Fahrtrichtung.
Gut für eine Präsentation.

## Protokolle

Jeder Lauf schreibt nach `Marlon/protokoll/` (der vorige Lauf wird überschrieben):

| Datei | Inhalt |
|-------|--------|
| `start.txt` | komplette Ausgabe von Start und Explorer |
| `bringup.txt` | Treiber (`yahboomcar_bringup`) |
| `lidar.txt` | Lidar (`sllidar_node`) |
| `diagnose.txt` | Geräte, Prozesse, Topics bei Start und bei Fehlern |

Hochladen, damit man sie am PC lesen kann:
```bash
bash Marlon/x3.sh hochladen
```

## Wenn etwas nicht startet

Das Skript erkennt ROS1/ROS2 automatisch und startet:

| | ROS2 | ROS1 |
|-|------|------|
| Treiber | `yahboomcar_bringup yahboomcar_bringup_X3_launch.py` | `yahboomcar_bringup bringup.launch` |
| Lidar   | `ros2 run sllidar_ros2 sllidar_node` mit `serial_port:=/dev/rplidar` | `rplidar_ros rplidar.launch` |

Heißen die Launch-Dateien auf eurem Image anders (z. B. RPLIDAR S2), überschreiben
(im Container: `bash Marlon/x3.sh shell`, dann `cd` in den Repo-Ordner – gleicher Pfad wie auf dem Pi):

```bash
LIDAR_LAUNCH="sllidar_ros2 sllidar_s2_launch.py" bash Marlon/start_autonomy.sh
```
Laufen Treiber/Lidar schon (z. B. selbst gestartet), werden sie nicht doppelt gestartet.
Ein hängender `sllidar_node` vom letzten Lauf wird vorher beendet.

## Am PC testen (ohne Roboter)

```bash
python test_explorer_sim.py
```
Simuliert einen Raum mit Tisch, Regal und Stuhlbeinen, Lidar-Rauschen und Radschlupf.
Ergebnis aktuell: 0 Kollisionen in 3×5 simulierten Minuten.
