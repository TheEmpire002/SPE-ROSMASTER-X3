# Architektur

## Überblick

```
   ┌──────────────┐                    ┌──────────────┐
   │   Web-App    │                    │    ESP32     │
   │  (Browser)   │                    │ Puls/Bewegung│
   └──────┬───────┘                    └──────┬───────┘
          │ WebSocket (JSON)                  │ HTTP/WebSocket (JSON)
          │                                   │
   ┌──────▼───────────────────────────────────▼───────┐
   │           Raspberry Pi 5  ·  robot/              │
   │                                                  │
   │   api/server.py        Ein- und Ausgang          │
   │        │                                         │
   │   safety/watchdog.py   stoppt bei Funkabriss     │
   │        │                                         │
   │   drive/  sensors/  box/                         │
   └──────┬───────────────────────────────────────────┘
          │ USB-Serial
   ┌──────▼───────┐
   │ X3 Expansion │  Motoren · IMU · Servos · Buzzer
   │    Board     │
   └──────────────┘
```

## Leitgedanken

**1. Ein Kanal für alles.**
App und ESP sprechen dasselbe JSON-Protokoll über WebSocket
(`rosmaster_assist/protocol.py`). Neue Funktionen — Box öffnen, Kamerabild,
Notfall-Alarm — sind neue Nachrichtentypen, keine neue Infrastruktur.

**2. Hardware ist austauschbar.**
Jeder Treiber implementiert eine abstrakte Basisklasse (`drive/base.py`).
Daneben steht ein Mock, der dieselbe Schnittstelle erfüllt, aber nur auf die
Konsole schreibt. Dadurch läuft das gesamte Projekt auf jedem Laptop, und
mehrere Leute können parallel arbeiten, obwohl es nur einen Roboter gibt.

**3. Sicherheit liegt unten, nicht oben.**
Der Watchdog sitzt zwischen Server und Treiber. Egal welcher Fehler oben
passiert — abgestürzte App, WLAN weg, Browser-Tab zu — der Roboter bleibt
nach kurzer Zeit von selbst stehen. Das darf keine Funktion sein, an die
die App denken muss.

**4. Die Notfall-Logik entscheidet nicht, sie meldet.**
Erkennt das System einen möglichen Notfall, löst es einen Alarm an einen
Menschen aus. Es bewertet keinen Gesundheitszustand.

## Module in `robot/src/rosmaster_assist/`

| Modul          | Verantwortung                                              |
|----------------|------------------------------------------------------------|
| `config.py`    | Alle Einstellungen an einer Stelle (Tempo, Ports, Timeouts) |
| `protocol.py`  | Nachrichtenformat — die gemeinsame Sprache aller Beteiligten |
| `drive/`       | Fahren. `base.py` = Schnittstelle, dazu echter Treiber + Mock |
| `safety/`      | Watchdog, Not-Aus, Geschwindigkeitsbegrenzung               |
| `sensors/`     | Lidar, Kamera, Vitaldaten vom ESP *(noch leer)*             |
| `box/`         | Motor/Servo der Transportbox *(noch leer)*                  |
| `api/`         | WebSocket-Server, liefert auch die Web-App aus              |
| `main.py`      | Startpunkt, baut alles zusammen                             |

## Autonomes Fahren — bewusst später

Das Grundgerüst enthält bewusst **kein** ROS. Begründung siehe
[`entscheidungen.md`](entscheidungen.md).

Wenn SLAM und Navigation drankommen, gibt es zwei Wege:

- **ROS2 in Docker** auf dem Pi, angebunden über denselben WebSocket-Kanal.
  Der Rest des Projekts bleibt unverändert.
- **Eigene Navigation in Python** direkt auf den Lidar-Daten. Weniger
  mächtig, aber ohne ROS-Komplexität und besser zu verstehen.

Diese Entscheidung fällt erst, wenn der Lidar Daten liefert — vorher fehlt
die Grundlage dafür.
