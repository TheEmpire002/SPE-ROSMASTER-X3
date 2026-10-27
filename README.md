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

## Schnellstart (ohne Roboter, zum Entwickeln)

Du brauchst keinen Roboter, um an der App zu arbeiten. Der Mock-Treiber
simuliert die Hardware und gibt alle Fahrbefehle auf der Konsole aus.

```bash
cd robot
python -m venv .venv

# Windows:
.venv\Scripts\activate
# Linux / Raspberry Pi:
source .venv/bin/activate

pip install -e .
python -m rosmaster_assist.main --mock
```

Danach im Browser öffnen: <http://localhost:8000>

## Start auf dem Roboter

```bash
cd robot
source .venv/bin/activate
pip install -e .
python -m rosmaster_assist.main          # ohne --mock = echte Hardware
```

Die App ist dann aus dem WLAN erreichbar unter `http://<IP-des-Pi>:8000`.
Die IP findest du mit `hostname -I`.

## Funktioniert alles noch?

Bei laufendem Server in einem zweiten Terminal:

```bash
pip install websockets
python scripts/smoke_test.py          # --port 8099 falls nötig
```

Prüft HTTP, WebSocket, Tempobegrenzung und Watchdog von außen — also genau
den Weg, den die App auch nimmt. Nach Änderungen am Server kurz laufen lassen.

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
