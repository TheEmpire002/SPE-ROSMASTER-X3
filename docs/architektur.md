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

Seit dem 29.10. sitzt dort auch eine **Anfahrrampe**: Fahrbefehle werden mit
begrenzter Beschleunigung angefahren und gebremst (`RAMPE_*` in
`config.py`), statt auf die Motoren zu springen. Sprünge zwischen Fahren und
Halten erzeugten Stromspitzen, die am X3 den USB-Hub zurücksetzten. Weil
die Rampe im Watchdog liegt, gilt sie für Handsteuerung, Erkunder,
Heimfahrt und Interaktion gleichermaßen. `stop()` umgeht sie und hält
sofort an.

Seit dem 03.11. prüft der Watchdog autonome Fahrbefehle (`quelle="autonom"`:
Erkunder, Heimfahrt, Folgen) zusätzlich mit dem **`AutonomSchutz`**
(`safety/collision.py`): Das Tempo in Fahrtrichtung wird so begrenzt, dass
der Roboter vor dem nächsten Lidar-Punkt im Fahrschlauch noch anhalten kann
(`v·t + v²/(2a) ≤ frei − Rand`, mit Latenz plus Alter des Scans). Ohne
frischen Scan bleibt nur die Drehung erlaubt. Gebremst wird über die Rampe,
nicht mit `stop()`: ein harter Halt erzeugt genau die Stromspitzen, die den
USB-Hub zurücksetzen. Die Module prüfen weiterhin selbst; der Schutz ist
das Netz darunter. Werte: `AUTONOM_SCHUTZ_*` in `config.py`.

**4. Die Notfall-Logik entscheidet nicht, sie meldet.**
Erkennt das System einen möglichen Notfall, löst es einen Alarm an einen
Menschen aus. Es bewertet keinen Gesundheitszustand.

## Module in `robot/src/rosmaster_assist/`

| Modul          | Verantwortung                                              |
|----------------|------------------------------------------------------------|
| `config.py`    | Alle Einstellungen an einer Stelle (Tempo, Ports, Timeouts) |
| `settings.py`  | Welche davon die Einstellungsseite zur Laufzeit ändern darf; speichert Änderungen in `robot/settings.json` |
| `audio/`       | Ton auf dem USB-Lautsprecher: `mukke.py` spielt YouTube-Links mit mpv ab |
| `protocol.py`  | Nachrichtenformat — die gemeinsame Sprache aller Beteiligten |
| `drive/`       | Fahren. `base.py` = Schnittstelle, dazu echter Treiber + Mock |
| `safety/`      | Watchdog, Not-Aus, Geschwindigkeitsbegrenzung, Bremsweg-Prüfung autonom (`AutonomSchutz`) |
| `sensors/`     | Lidar, Kamera, Personenerkennung, Körperpose (Hailo)        |
| `navigation/`  | Odometrie, Lidar-Ortung, Heimfahrt, Erkunder, Ausweichen (`ausweichen.py`), Hindernis-Benennung |
| `interaktion/` | Gesten, Verhalten (Folgen, Herkommen, Weggehen), Stimme (Sprachausgabe) und Sprachbefehle mit dem Weckwort „Robi“; geplant: Geist — siehe [`interaktion.md`](interaktion.md) |
| `diagnose.py`  | Sammelt Ausfälle (Akku, USB, Unterspannung, Temperatur, Geräte, Log) und macht daraus verständliche Probleme für die Seite `/diagnose` |
| `aufzeichnung.py` | Fahrtenschreiber: letzte Minute (Scans, Befehle, Odometrie, Zustände) im Speicher, auf Knopfdruck als Datei; dazu Laden und Nachspielen für `scripts/abspielen.py` |
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

**Stand 28.10.:** Gewählt wurde der zweite Weg. Der Erkunder
(`navigation/explorer.py`) fährt direkt im Server auf den Lidar-Daten,
mit demselben Watchdog wie die Handsteuerung. Das ältere ROS2-Projekt liegt
zum Vergleich unter `Marlon/autonomes_fahren/`.

**Stand 03.11.:** Der Erkunder dreht nach dem gemessenen Winkel (Drehrate
des Boards über die Odometrie), entscheidet nach dem Drehen erst auf einem
frischen Scan und befreit sich selbst, statt stehen zu bleiben: seitlich
oder rückwärts, wenn Punkte zu nah sind oder er nicht vorankommt (Board-
Tempo oder unverändertes Lidar-Bild rundum). Test ohne Hardware:
`Marlon/autonomes_fahren/test_erkunder.py` (simulierter Raum).

**Fahrtenschreiber (03.11.):** Bevor das Ausweichen klüger wird, braucht
es echte Fahrten zum Prüfen. `aufzeichnung.py` hört an zwei Stellen mit:
am Watchdog (jeder Befehl vor und nach den Grenzen, jeder Halt) und in
einer eigenen Schleife im Odometrie-Takt (Scan, Board-Geschwindigkeit,
Pose, Zustände). Ein Fehler beim Mitschreiben darf nie das Fahren stören,
deshalb ist jeder Aufruf abgefangen. `scripts/abspielen.py` fasst eine
Datei zusammen, zeichnet sie als SVG und legt die Scans einem aktuellen
Erkunder vor (`erkunder_nachspielen`). Im Test mit gleichem Zufall gibt
der nachgespielte Erkunder Takt für Takt dieselben Befehle
(`Marlon/autonomes_fahren/test_aufzeichnung.py`); bei echten Fahrten zählt
nur die Übereinstimmungsquote, weil der Erkunder am Roboter zufällig wählt.

**Ausweichen (03.11., F2):** `navigation/ausweichen.py` enthält die
Bausteine, der Erkunder setzt sie nur zusammen. Alle sind reine Logik auf
Lidar-Punkten und ohne Hardware testbar:

- `Kurzgedaechtnis` behält Punkte wenige Sekunden und rechnet sie mit der
  Board-Geschwindigkeit mit. Ohne Karte und ohne Ortung, deshalb nur nahe
  Punkte und nur kurz (Drift).
- `bogen_planen` rechnet Bögen voraus und prüft den Umriss des Roboters.
  Bewertet werden Kurs, Luft, Tempo, Weite und ruhiges Lenken.
- `luecken` liefert die Mitten freier Bereiche (Türen).
- `HochhebeErkennung` vergleicht Runden in 10°-Sektoren; mit gemessener
  Lage über ein Fenster von einer Sekunde, damit auch langsames Tragen
  auffällt.

Test: `Marlon/autonomes_fahren/test_ausweichen.py` (Türen, Flur,
Hochheben, Zimmer ohne Fehlalarm).

## Interaktion

Gesten, Sprache und eine „Geist“-Schicht sitzen als eigene Ebene über den
Verhalten und fahren ebenfalls nur über den Watchdog. Aufbau, Gesten und
Stufenplan beschreibt [`interaktion.md`](interaktion.md), die Begründung
steht in E5 und E6 in [`entscheidungen.md`](entscheidungen.md).

**Stand 28.10.:** Stufen 2 und 3 laufen im Server. `sensors/pose.py`
rechnet die Körperpose auf dem Hailo, `interaktion/gesten.py` erkennt die
Handzeichen, `interaktion/verhalten.py` fährt danach. Die Verdrahtung in
`api/server.py` folgt genau dem Erkunder: bauen in `create_app`, Abbruch
bei Not-Stopp, Handsteuerung, Heimfahrt, Verbindungsverlust und
Antriebsausfall, Meldungen über `_status_loop`.

Dazu kommt `interaktion/stimme.py`: Sprachausgabe auf dem USB-Lautsprecher
in einem eigenen Thread. Der Server baut sie in `create_app` und gibt sie
dem Verhalten mit, das dann seine Moduswechsel ansagt. Fahren kann die
Stimme nicht, sie bekommt keinen Watchdog. Sie blockiert weder den
Event-Loop noch die Fahrschleife: `sagen()` stellt den Satz nur in eine
Warteschlange.
