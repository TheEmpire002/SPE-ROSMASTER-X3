# SPE-ROSMASTER-X3

Hilfsroboter auf Basis eines **Yahboom ROSMASTER X3**.

Der Roboter soll in Innenräumen autonom fahren, eine Person per ESP-Sensorik
beobachten (Bewegung, Puls) und im Notfall Hilfe holen. Zusätzlich kann er per
Web-App manuell gesteuert werden und über eine aufgesetzte Box Hilfsmittel
ausliefern.

> **Hinweis:** Die Vitaldaten-Erfassung ist ein Prototyp und **kein
> Medizinprodukt**. Der Roboter trifft keine medizinischen Entscheidungen — er
> alarmiert immer einen Menschen.

## Teilsysteme

| Ordner    | Läuft auf        | Zweck                                                  |
|-----------|------------------|--------------------------------------------------------|
| `robot/`  | Raspberry Pi 5   | Antrieb, Sensorik, Sicherheits-Logik, WebSocket-Server |
| `webapp/` | Browser          | Fernsteuerung, Statusanzeige, Notfall-Alarm            |
| `esp/`    | ESP32            | Personentracking und Vitaldaten                        |
| `docs/`   | —                | Architektur, Hardware, Entscheidungen                  |
| `scripts/`| Raspberry Pi 5   | Hilfs- und Diagnoseskripte                             |

Details: [`docs/architektur.md`](docs/architektur.md)

**Eigene Oberfläche oder eigenen Client anbinden?** Die komplette Schnittstelle
(Fahren, Lidar, Kamera, Heimfahrt) steht in [`docs/api.md`](docs/api.md).
Hinweise für KI-Assistenten: [`AGENTS.md`](AGENTS.md).

## Schnellstart

Ein Befehl, überall — auf dem Laptop wie auf dem Roboter:

```bash
python main.py
```

Das Skript richtet beim ersten Mal alles selbst ein (virtuelle Umgebung,
Abhängigkeiten), erkennt dann, ob echte Hardware angeschlossen ist, startet
den Server und öffnet die Web-App im Browser.

- **Auf dem Laptop** findet es keine Hardware und schaltet automatisch in die
  Simulation. Fahrbefehle landen nur auf der Konsole — du kannst also die
  komplette App entwickeln, ohne den Roboter zu belegen.
- **Auf dem Raspberry Pi** erkennt es Treiber und serielle Schnittstelle und
  fährt mit echter Hardware hoch. Die App ist dann im WLAN erreichbar unter
  `http://<IP-des-Pi>:8000` (IP: `hostname -I`).

Optionen:

| Option          | Wirkung                                        |
|-----------------|------------------------------------------------|
| `--mock`        | Simulation erzwingen, auch auf dem Roboter     |
| `--real`        | Hardware erzwingen, Abbruch wenn sie fehlt     |
| `--port 8001`   | anderer Port                                   |
| `--no-browser`  | Browser nicht öffnen (z.B. über SSH)           |
| `--setup-only`  | nur einrichten, nicht starten                  |
| `--debug`       | ausführliche Logausgabe                        |

### Ohne Automatik

Wenn du den Server direkt starten willst:

```bash
cd robot
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -e .
python -m rosmaster_assist.main --mock
```

## Kamera

Das Bild erscheint oben in der Web-App, ohne Zutun. Die Software sucht sich
die Kamera selbst: zuerst eine CSI-Kamera, dann eine USB-Kamera, sonst ein
bewegtes Testbild. Welche es wurde, steht beim Start im Log.

Für eine **USB-Kamera** wird OpenCV gebraucht:

```bash
robot/.venv/bin/pip install opencv-python
```

Direkt aufrufbar, auch ohne App: `http://<IP-des-Pi>:8000/camera/stream`

**Kein Bild?** Server beenden, dann:

```bash
robot/.venv/bin/python scripts/check_camera.py
```

Prüft Geräte, Rechte, konkurrierende Prozesse, OpenCV und das tatsächliche
Öffnen — und nennt am Ende die Ursache.

Einstellungen (Auflösung, Bildrate, Qualität): `config.py`, Abschnitt Kamera.
Details in [`docs/hardware.md`](docs/hardware.md#kamera).

## Lidar

Der RPLIDAR A1 wird direkt ausgelesen — ohne ROS2. Er leistet zweierlei:

1. **Ortung.** Beim Setzen des Startpunkts wird einmal aufgenommen, wie der
   Raum von dort aussieht. Später wird die aktuelle Aufnahme damit
   abgeglichen; die nötige Verschiebung ist die Position. Dieses Verfahren
   driftet nicht — anders als die Koppelnavigation.
2. **Hindernisse.** Während der Heimfahrt wird der Bereich in Fahrtrichtung
   überwacht. Der Roboter weicht nicht aus, er hält an und meldet es.

Dafür wird ein Paket gebraucht:

```bash
robot/.venv/bin/pip install rplidar-roboticia
```

Die App zeigt die Punktwolke als Draufsicht. Zustand und Trefferquote des
Abgleichs: `http://<IP-des-Pi>:8000/lidar/status`

**Wichtig:** ROS2 darf nicht parallel laufen — `sllidar_ros2` belegt
denselben Anschluss. Siehe [`docs/hardware.md`](docs/hardware.md).

## Nach Hause fahren

In der App gibt es unter „Startpunkt" zwei Knöpfe:

- **Startpunkt hier setzen** — die aktuelle Stelle wird der Nullpunkt
- **Nach Hause** — der Roboter fährt selbsttätig dorthin zurück

Dazwischen zeigt die App laufend Entfernung, Position und Blickrichtung.
Während der Fahrt wird derselbe Knopf zum Abbrechen.

**Zwei Einschränkungen, die man kennen muss:**

Die Position kommt aus zwei Quellen: der Koppelnavigation (driftet, ist aber
immer da) und dem Lidar-Abgleich (driftet nicht, braucht aber Sicht auf den
Startbereich). Gemessen im Simulationslauf über 2,3 m: **30 cm Abweichung
ohne Lidar, 7 cm mit**. Steht in der App „geschätzt" hinter der Entfernung,
fehlt der Lidar oder die Bewegungsmessung — dann gilt der schlechtere Wert.

Jenseits von `MATCH_RADIUS_M` (4 m) erkennt der Lidar den Startbereich nicht
wieder und korrigiert nicht. Die Odometrie bringt den Roboter dorthin
zurück, der Lidar weist ihn dann genau ein.

Der Roboter **weicht Hindernissen nicht aus** — er hält davor an und meldet
es. Ausweichen ohne Karte endet leicht in einer Ecke. Deshalb auch das
gemächliche Tempo (`HOME_SPEED`) und die Zeitbegrenzung.

Abgebrochen wird die Heimfahrt durch jede Fahreingabe, die Leertaste, den
Abbrechen-Knopf oder eine verlorene Verbindung.

## Tempo einstellen

In der Web-App gibt es zwei Regler:

- **Max-Tempo** — das Maximum in m/s, das du dir gibst. Drinnen zum Üben
  z.B. 0.3, draußen Vollgas. Die Einstellung bleibt im Browser gespeichert.
- **Tempo** — wie viel Prozent davon die Tasten ausreizen.

Seitwärtsfahrt und Drehen werden im selben Verhältnis gedrosselt, damit der
Roboter bei zahmem Max-Tempo nicht trotzdem hektisch rotiert.

Die **harte Obergrenze** steht auf dem Roboter in
`robot/src/rosmaster_assist/config.py` (`MAX_SPEED_X` und Co.) und lässt sich
aus der App nicht überschreiten. Der Server schickt sie beim Verbinden mit;
der Max-Tempo-Regler geht nie darüber hinaus.

## Funktioniert alles noch?

Bei laufendem Server in einem zweiten Terminal:

```bash
pip install websockets
python scripts/smoke_test.py          # --port 8099 falls nötig
```

Prüft HTTP, WebSocket, Tempobegrenzung und Watchdog von außen — also genau
den Weg, den die App auch nimmt. Nach Änderungen am Server kurz laufen lassen.

### Auf dem Roboter

```bash
ssh pi@<IP-des-Pi>
cd ~/SPE-ROSMASTER-X3 && git pull

# Terminal 1 — Server mit echter Hardware
cd robot && source .venv/bin/activate
python -m rosmaster_assist.main

# Terminal 2 — Test
cd ~/SPE-ROSMASTER-X3
robot/.venv/bin/python scripts/smoke_test.py
```

Die Fahrtests werden dabei **übersprungen**: das Skript erkennt am gemeldeten
Antrieb, dass echte Hardware dranhängt, und bewegt den Roboter nicht von
selbst. Wer sie trotzdem braucht, **bockt die Räder auf** und hängt
`--allow-motion` an.

Vom Laptop aus geht es auch: `--host <IP-des-Pi>`.

## Was ist installiert?

Wenn unklar ist, welche Treiber auf dem Pi vorhanden sind:

```bash
python scripts/check_hardware.py
```

## Projektstand

- [x] Grundgerüst, Treiber-Abstraktion, Protokoll
- [x] Manuelle Steuerung per Web-App
- [x] Kamerabild in der App
- [x] Rückfahrt zum Startpunkt (Koppelnavigation)
- [x] Lidar: Ortung und Hinderniserkennung
- [ ] ESP-Anbindung (Bewegung, Puls)
- [x] Autonomes Fahren, Test 1: Erkunder mit Hindernis-Benennung (ohne SLAM)
- [x] Interaktion, Stufe 2+3: Gesten (Hailo-Pose) und Verhalten Folgen/Stopp/Herkommen/Weggehen (siehe [`docs/interaktion.md`](docs/interaktion.md))
- [ ] Interaktion: Sprache, Stimme, „Geist“
- [ ] SLAM / Karte
- [ ] Box-Mechanik
