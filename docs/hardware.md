# Hardware

## Bestand

| Komponente      | Modell                | Anschluss        | Status      |
|-----------------|-----------------------|------------------|-------------|
| Rechner         | Raspberry Pi 5 + AI HAT+ | —             | vorhanden   |
| Fahrgestell     | Yahboom ROSMASTER X3  | USB-Serial       | vorhanden   |
| Expansion Board | Yahboom (STM32)       | USB-Serial       | vorhanden   |
| Lidar           | Slamtec RPLIDAR A1    | USB-Seriell      | vorhanden, nicht eingebunden |
| Kamera          | *(Modell eintragen)*  | USB / CSI        | eingebunden (CSI, USB oder Testbild) |
| ESP32           | *(Modell eintragen)*  | WLAN             | geplant     |
| Pulssensor      | *(Modell eintragen)*  | ESP32            | geplant     |
| Box-Motor       | *(Modell eintragen)*  | Servo am X3      | geplant     |

> Bitte beim Einbauen die Modellbezeichnungen hier eintragen — ohne die lässt
> sich später nicht nachvollziehen, welcher Treiber gebraucht wird.

## Was am USB hängt

Ausgabe von `lsusb` auf diesem Roboter, übersetzt:

| USB-ID | Gerät | Wofür |
|---|---|---|
| `1a86:7523` | QinHeng CH340 | **Expansion Board** — Antrieb |
| `10c4:ea60` | Silicon Labs CP210x | **RPLIDAR A1** |
| `2bc5:050f` | Orbbec „USB 2.0 Camera" | RGB-Bild → `/dev/video0` |
| `2bc5:060f` | Orbbec Depth Sensor | Tiefenkanal derselben Kamera |

**Daraus folgt das Wichtigste:** Es gibt **zwei** USB-Seriell-Adapter. Board
und Lidar erscheinen beide als `/dev/ttyUSB*`, und welcher welche Nummer
bekommt, hängt von der Einschaltreihenfolge ab. Überlässt man
`Rosmaster_Lib` die Suche, kann sie den Lidar erwischen — dann bewegt sich
nichts, ohne dass eine Fehlermeldung erscheint.

Deshalb den Port fest eintragen:

```bash
ls -l /dev/serial/by-id/
```

Den Eintrag mit `1a86` bzw. `CH340` in `config.py` bei `SERIAL_PORT`
hinterlegen. Der Lidar hat mit `/dev/rplidar` bereits einen festen Namen.

Die Kamera ist eine **Orbbec-Tiefenkamera**. Genutzt wird aktuell nur ihr
RGB-Kanal als gewöhnliche USB-Kamera. Der Tiefenkanal wäre für die
Hinderniserkennung interessant, braucht aber das Orbbec-SDK.

## Antrieb

Der X3 hat **Mecanum-Räder**. Er kann dadurch seitwärts fahren, ohne sich zu
drehen. Die Fahrbefehle haben deshalb drei Werte statt zwei:

| Wert    | Bedeutung                    | Einheit |
|---------|------------------------------|---------|
| `vx`    | vorwärts (+) / rückwärts (−) | m/s     |
| `vy`    | links (+) / rechts (−)       | m/s     |
| `omega` | drehen gegen (+) / im (−) Uhrzeigersinn | rad/s |

Grenzwerte und Standard-Tempo stehen in `robot/src/rosmaster_assist/config.py`.

## Rosmaster_Lib

Yahboons Python-Treiber spricht über USB-Serial mit dem Expansion Board.

```python
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()                     # Standard: /dev/myserial bzw. /dev/ttyUSB0
bot.create_receive_threading()        # startet den Empfangs-Thread
bot.set_car_motion(vx, vy, omega)     # fahren
bot.set_beep(100)                     # Buzzer, Dauer in ms
bot.set_colorful_lamps(0xFF, r, g, b) # LED-Streifen
```

Falls die Bibliothek fehlt, liegt sie im Yahboom-Lieferumfang bzw. im
offiziellen Yahboom-GitHub-Repository zum ROSMASTER.

**Serielle Schnittstelle.** Der Gerätename unterscheidet sich je nach Image
und Anschlussart (`/dev/myserial`, `/dev/ttyUSB0`, `/dev/ttyAMA0`, …). Das ist
aber meist egal: **`Rosmaster()` ohne Argument findet das Board selbst.**
Deshalb steht `SERIAL_PORT` in `config.py` auf `None` und sollte dort auch
bleiben, solange nichts dagegen spricht.

Ein fester Port ist nur nötig, wenn die Automatik das falsche Gerät erwischt —
realistisch dann, wenn der RPLIDAR am selben Bus hängt. In dem Fall den
`by-id`-Pfad eintragen, nicht `/dev/ttyUSB0` (siehe Lidar-Abschnitt unten).

Ob das Board erreichbar ist, beantwortet `python scripts/check_hardware.py`
unter „Expansion Board" — das versucht tatsächlich, es zu öffnen, statt nur
nach Gerätedateien zu suchen.

**Nur ein Programm pro Port.** Der serielle Port lässt sich technisch
mehrfach öffnen, aber nicht sinnvoll teilen: zwei Leser bekommen jeweils
Bruchstücke. Typische Meldung:

```
SerialException: device reports readiness to read but returned no data
(device disconnected or multiple access on port?)
```

Wer noch mitliest, zeigt:

```bash
ps aux | grep -i python | grep -v grep
sudo fuser -v /dev/ttyUSB* /dev/ttyAMA* /dev/myserial 2>&1
```

Die üblichen Verdächtigen:

- **Der ROS2-Stack.** `yahboomcar_bringup` öffnet dasselbe Board über
  dieselbe Bibliothek. Häufigste Ursache bei uns:

  ```bash
  sudo pkill -f yahboomcar_bringup
  sudo pkill -f x3_explorer       # fährt sonst nebenher selbst
  sudo pkill -f sllidar_node      # belegt /dev/rplidar
  ```

  Entweder ROS2 **oder** unser Server — niemals beide.
- **Yahboom-App** (`app.py` mit Flask, MakerControl). Startet auf
  Yahboom-Images beim Booten. `sudo systemctl stop <name>`, bei Bedarf
  zusätzlich `disable`.
- **Serielle Konsole** auf der GPIO-UART. Abschalten über
  `sudo raspi-config` → *Interface Options* → *Serial Port* → Login-Shell
  **nein**, Hardware **ja**.
- **ModemManager**, der neue USB-Seriell-Geräte automatisch abklopft:
  `sudo systemctl disable --now ModemManager`

Als Übergangslösung lässt sich in `config.py` `TELEMETRY_ENABLED = False`
setzen. Dann liest unsere Software gar nicht mehr und der Konflikt
verschwindet — Fahren funktioniert weiter, Akku- und IMU-Werte fehlen.

**Rechte.** Wenn der Zugriff an „Permission denied" scheitert:

```bash
sudo usermod -aG dialout $USER   # danach neu anmelden
```

## Lidar: Slamtec RPLIDAR A1

360°-Laserscanner, Reichweite ca. 12 m, rund 8000 Messpunkte pro Sekunde bei
5,5 Umdrehungen/s. Liefert pro Messpunkt Winkel, Entfernung und Signalqualität
— mehr nicht. Daraus eine Karte zu bauen ist Aufgabe der Software.

**Anschluss.** Der A1 hängt über einen USB-Seriell-Adapter (CP2102) am Pi und
läuft mit **115200 Baud**. Auf diesem Pi existiert bereits der feste Pfad
**`/dev/rplidar`** (udev-Regel aus dem Yahboom-Setup) — den verwenden, nicht
`/dev/ttyUSB0`. Der Motor wird über die DTR-Leitung gesteuert; dreht sich der
Scanner nicht, liegt es meist daran.

**Es gibt schon einen Treiber.** Unter ROS2 Foxy läuft `sllidar_ros2` auf
diesem Pi bereits. Bevor jemand einen eigenen Python-Treiber schreibt, lohnt
der Blick dorthin — siehe E1a in [`entscheidungen.md`](entscheidungen.md).

**⚠ Portverwechslung — wichtig.** Das X3-Expansion-Board nutzt *ebenfalls*
einen USB-Seriell-Port. Bei zwei Geräten vergibt Linux `/dev/ttyUSB0` und
`/dev/ttyUSB1` in der Reihenfolge, in der sie erkannt werden — und die kann
sich bei jedem Neustart ändern. Dann bekommt der Antrieb plötzlich
Lidar-Befehle.

Deshalb feste Namen über die Geräte-ID verwenden statt `ttyUSB0`:

```bash
ls -l /dev/serial/by-id/
# -> /dev/serial/by-id/usb-Silicon_Labs_CP2102_...-if00-port0
```

Dieser Pfad bleibt gleich, egal in welcher Reihenfolge gebootet wird. Den
Pfad des Expansion Boards in `config.py` bei `SERIAL_PORT` eintragen, den des
Lidars später entsprechend beim Lidar-Treiber.

**Stromversorgung.** Der A1 zieht beim Anlaufen des Motors kurz spürbar
Strom. Hängt er über einen unversorgten USB-Hub am Pi, kann es zu Aussetzern
kommen — im Zweifel direkt an den Pi.

**Bibliothek.** Für reines Python kommt `rplidar-roboticia` in Frage (Fork des
nicht mehr gepflegten `rplidar`). Noch nicht ausgewählt — siehe offene Punkte
in `robot/src/rosmaster_assist/sensors/__init__.py`.

## Kamera

Die Software probiert beim Start drei Wege in dieser Reihenfolge durch und
nimmt den ersten, der funktioniert:

| Reihenfolge | Variante | Voraussetzung |
|---|---|---|
| 1 | CSI-Kamera am Flachbandkabel | `picamera2` (bei Raspberry Pi OS dabei) |
| 2 | USB-Kamera | `pip install opencv-python` |
| 3 | Testbild | nichts — läuft überall |

Es ist also nichts zu konfigurieren, solange eine Kamera erkannt wird. Welche
genommen wurde, steht beim Start im Log („Kamera (CSI) bereit" bzw. „Kamera
(USB, Index 0) bereit"). Steht dort „Testbild", wurde keine gefunden.

Auflösung, Bildrate und JPEG-Qualität stehen in
`robot/src/rosmaster_assist/config.py`. Bei ruckelndem Bild über WLAN zuerst
`CAMERA_FPS` senken, erst danach die Auflösung.

**USB-Kamera prüfen:**

```bash
ls /dev/video*          # taucht hier nichts auf, wird sie nicht erkannt
pip install opencv-python
```

**CSI-Kamera prüfen:**

```bash
rpicam-hello --list-cameras
```

Der Stream liegt unter `/camera/stream` (MJPEG) und ein Einzelbild unter
`/camera/snapshot` — beides direkt im Browser aufrufbar, auch ohne die App.

## AI HAT+

Der AI HAT+ enthält einen Hailo-Beschleuniger. Er ist für die spätere
Kamera-Auswertung interessant (Personenerkennung, Sturzerkennung) und wird im
aktuellen Grundgerüst noch nicht genutzt. Ob die Personenerkennung über die
Kamera oder über den ESP läuft, ist noch offen — der AI HAT+ wäre das deutlich
zuverlässigere Verfahren.
