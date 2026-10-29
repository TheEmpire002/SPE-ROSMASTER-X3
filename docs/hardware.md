# Hardware

## Bestand

| Komponente      | Modell                | Anschluss        | Status      |
|-----------------|-----------------------|------------------|-------------|
| Rechner         | Raspberry Pi 5 (16 GB) + AI HAT+ 2 (Hailo-10H) | PCIe (`/dev/hailo0`) | vorhanden, Hailo für Interaktion geplant |
| Fahrgestell     | Yahboom ROSMASTER X3  | USB-Serial       | vorhanden   |
| Expansion Board | Yahboom (STM32)       | USB-Serial       | vorhanden   |
| Lidar           | Slamtec RPLIDAR A1    | USB-Seriell      | eingebunden (Ortung + Hindernisse) |
| Tiefenkamera    | Orbbec (Farb- + Tiefenkanal) | USB (über Hub) | Farbbild eingebunden (Kamera `orientierung`), Tiefe noch nicht |
| Webcam          | Logitech C270 (vorn, leicht nach oben) | USB, eigener Bus (direkt am Pi) | eingebunden (Kamera `personen`: Personenerkennung, Gesten) |
| Mikrofon        | USB-Mikrofon (FongLun, C-Media `0d8c:0016`); weitere in C270 und Orbbec | USB-Audio, eigener Bus | für Sprache geplant |
| Lautsprecher    | USB-Lautsprecher „UACDemoV1.0“ | USB-Audio, direkt am Pi | eingebunden (MUKKE: Musik von YouTube), Sprachausgabe geplant |
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
| `2bc5:050f` | Orbbec „USB 2.0 Camera" | RGB-Bild, Kamera `orientierung` |
| `2bc5:060f` | Orbbec Depth Sensor | Tiefenkanal derselben Kamera |
| `046d:0825` | Logitech C270 HD WebCam | Kamera `personen`, Mikrofon (eigener USB-Bus, nicht am Hub) |
| `0d8c:0016` | USB-Mikrofon (C-Media) | Spracheingabe (ALSA „Microphone“) |
| `4c4a:4155` | Jieli „UACDemoV1.0“ | USB-Lautsprecher, Sprachausgabe (ALSA „UACDemoV10“), keine Kamera |

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

Die **Orbbec-Tiefenkamera** meldet sich als zwei USB-Geräte. Genutzt wird
aktuell nur ihr RGB-Kanal als gewöhnliche USB-Kamera. Der Tiefenkanal ist
für Orientierung und Kollisionsvermeidung gedacht, ist aber kein
Videogerät (taucht in `v4l2-ctl --list-devices` nicht auf) und braucht das
Orbbec-SDK.

Seit dem 28.10. ist zusätzlich eine **Logitech C270** vorn montiert. Sie
hängt direkt am Pi, nicht am Hub: Am Hub hängen schon Board, Lidar und
Orbbec, und eine weitere Kamera dort macht Spannungseinbrüche
wahrscheinlicher. Beim Einstecken meldet der Kernel einige
`Failed to query (GET_INFO) UVC control …` und `cannot get freq`. Das ist
bei der C270 normal und harmlos.

Die `/dev/video*`-Nummern hängen von der Einsteckreihenfolge ab. Der Server
ordnet die Kameras deshalb über die USB-Kennung zu, nicht über die Nummer
(siehe [Kamera](#kamera)).

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

**Eingebunden.** Der Lidar wird direkt aus Python ausgelesen, ohne ROS2
(Begründung: E4 in [`entscheidungen.md`](entscheidungen.md)). Nötig ist dafür:

```bash
robot/.venv/bin/pip install rplidar-roboticia
```

**ROS2 darf nicht gleichzeitig laufen.** `sllidar_ros2` belegt denselben
Anschluss — entweder das eine oder das andere:

```bash
sudo pkill -f sllidar_node
```

Zustand prüfen: `http://<IP-des-Pi>:8000/lidar/status`. Dort steht, wie viele
Umdrehungen angekommen sind, ob der Startpunkt eingemessen wurde und wie gut
der Abgleich passt (`letztes_residuum_m`; unter 5 cm ist gut).

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

Die Software betreibt **alle** Kameras, die sie findet, jede mit eigenem
Bild:

| Variante | Voraussetzung |
|---|---|
| USB-Kameras | `pip install opencv-python` |
| CSI-Kamera am Flachbandkabel | `picamera2` (bei Raspberry Pi OS dabei) |
| Testbild, wenn gar keine da ist | nichts, läuft überall |

Welche USB-Kamera wofür da ist, steht in `config.py` bei `CAMERA_ROLES`:
`orientierung` ist die Orbbec (USB-Kennung `2bc5`), `personen` die Logitech
(`046d`). Zugeordnet wird über die USB-Kennung, nicht über `/dev/videoN`,
denn die Nummern vertauschen sich nach Neustart oder Neu-Einstecken. Fehlt
eine dieser Kameras, sucht der Server sie im Hintergrund weiter. Weitere
USB-Kameras laufen als `usb1`, `usb2`, …

Beim Start steht im Log, was gefunden wurde („Kameras: orientierung (…),
personen (…)“ und je Kamera „Kamera '…' (USB …, /dev/videoN) bereit“).
Ohne Log: `/cameras` im Browser aufrufen.

Auflösung, Bildrate und JPEG-Qualität lassen sich in den Einstellungen (Zahnrad
oben rechts in der App) verstellen; die Standardwerte stehen in
`robot/src/rosmaster_assist/config.py`. Bei ruckelndem Bild über WLAN zuerst
`CAMERA_FPS` senken, erst danach die Auflösung.

**USB-Kamera prüfen:**

```bash
v4l2-ctl --list-devices # je Kamera ein Block; fehlt sie hier, wird sie nicht erkannt
lsusb                   # USB-Kennung, muss zu CAMERA_ROLES passen
pip install opencv-python
```

**CSI-Kamera prüfen:**

```bash
rpicam-hello --list-cameras
```

Der Stream liegt unter `/camera/<name>/stream` (MJPEG) und ein Einzelbild
unter `/camera/<name>/snapshot`, z. B. `/camera/personen/stream`. Beides ist
direkt im Browser aufrufbar, auch ohne die App. `/camera/stream` ohne Namen
zeigt die Standardkamera (`orientierung`).

## Lautsprecher und Musik („MUKKE“)

Der USB-Lautsprecher heißt bei ALSA `UACDemoV10`. Die App spielt darüber
YouTube-Links ab (Knopf „MUKKE“). Einmalig auf dem Pi einrichten:

```bash
sudo apt install mpv
robot/.venv/bin/pip install -U yt-dlp
aplay -l                      # der Lautsprecher muss als "UACDemoV10" auftauchen
speaker-test -D plughw:CARD=UACDemoV10,DEV=0 -c 2 -t sine -l 1   # Testton
```

yt-dlp aus pip, nicht aus apt: YouTube ändert laufend etwas, und die
Version aus den Paketquellen ist meist zu alt. Spielt plötzlich nichts mehr,
hilft fast immer `robot/.venv/bin/pip install -U yt-dlp`.

Das Ausgabegerät steht in `config.py` bei `MUKKE_AUDIO_DEVICE`. Kommt kein
Ton, zeigt das MUKKE-Fenster den Grund an: die Fehlermeldung von YouTube
(yt-dlp), wenn der Link nicht aufgelöst werden kann, sonst die letzte Meldung
von mpv. Gegenprobe am Pi: `robot/.venv/bin/yt-dlp -f bestaudio -g "<Link>"`.

## AI HAT+

> **Nachtrag 28.10.:** Verbaut ist ein **AI HAT+ 2 mit Hailo-10H** (PCIe,
> `/dev/hailo0`, Firmware 5.1.1, 40 TOPS, 8 GB eigener Speicher). HailoRT
> 5.1.1 ist installiert; `hailo_platform` lässt sich im venv importieren.
> Mitgelieferte Modelle liegen in `/usr/share/hailo-models/`, u. a.
> `yolov8s_pose_h10.hef` (Körperpose, 17 Punkte) und `yolov8m_h10.hef`
> (Objekte). Der Hailo-10H kann auch kleine Sprachmodelle ausführen (Hailo
> GenAI, noch nicht installiert). Geplante Nutzung: Gestenerkennung über
> die Körperpose, siehe [`interaktion.md`](interaktion.md) und E5 in
> [`entscheidungen.md`](entscheidungen.md).

Der AI HAT+ enthält einen Hailo-Beschleuniger. Er ist für die spätere
Kamera-Auswertung interessant (Personenerkennung, Sturzerkennung) und wird im
aktuellen Grundgerüst noch nicht genutzt. Ob die Personenerkennung über die
Kamera oder über den ESP läuft, ist noch offen — der AI HAT+ wäre das deutlich
zuverlässigere Verfahren.

## USB-Resets (Befund vom 28.10.)

Geräte am USB-Hub (Lidar, Board, Orbbec) melden sich gelegentlich gleichzeitig
ab. `dmesg` zeigt dann „over-current“ auf allen Ports. Die Ursache sind
**CPU-Lastsprünge bei 2,4 GHz**, vermutlich mit Rückspeisung (Backfeed) des
Hubs über das Expansion-Board. Ein leerer Akku ist es nicht.

- Mit 2,0 GHz treten die Resets deutlich seltener auf, ganz weg sind sie
  aber nicht. **Seit dem 28.10. steht deshalb `arm_freq=2000` in
  `/boot/firmware/config.txt`** (mit Ben abgestimmt, Backup
  `config.txt.vor-arm_freq-2026-10-28`). Der Server braucht dafür keine
  Anpassung: Alle Schleifen laufen nach Uhrzeit, nicht nach Rechenzeit, und
  die teuersten Teile (Personenerkennung 2 Hz, JPEG 15 fps) haben genug Luft.
- `python scripts/check_hardware.py` zeigt Takt, Temperatur und Drosselung
  und warnt, wenn die Begrenzung fehlt.
- Wird der Pi heiß (über ~80 °C, z. B. beim Kompilieren), drosselt er selbst
  auf 1,5 GHz und die USB-Geräte fallen massenhaft ab. Beim Kompilieren also
  Roboter nicht fahren lassen.
- Die eigentliche Lösung liegt in der Hardware: ein Hub ohne Rückspeisung
  oder eine getrennte VBUS-Leitung.
- Die C270 hängt an einem eigenen Bus und ist nicht betroffen.

Messungen und Herleitung: `Marlon/sensorcheck_2026-10-28/ZUSAMMENFASSUNG.md`.
