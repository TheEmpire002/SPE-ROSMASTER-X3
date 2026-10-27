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
