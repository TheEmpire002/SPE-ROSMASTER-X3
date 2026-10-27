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
- [ ] Lidar einbinden
- [ ] Kamera-Stream
- [ ] ESP-Anbindung (Bewegung, Puls)
- [ ] Autonomes Fahren / SLAM
- [ ] Box-Mechanik
