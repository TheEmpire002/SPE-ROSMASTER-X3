# Hinweise für KI-Assistenten und neue Mitwirkende

Hilfsroboter auf einem Yahboom ROSMASTER X3 (Raspberry Pi 5, Mecanum-Räder,
RPLIDAR A1, Orbbec-Kamera). Reines Python, kein ROS. Ein FastAPI-Server auf
dem Pi ist der einzige Ein- und Ausgang; Web-App und ESP32 sprechen ihn per
WebSocket an.

## Zuerst lesen

- [`docs/api.md`](docs/api.md): **die Schnittstelle des Roboters.** Fahren,
  Lidar, Kamera, Heimfahrt, Nachrichtenformat, Konventionen. Wer nur eine
  Oberfläche oder einen Client baut, braucht nichts anderes.
- [`docs/architektur.md`](docs/architektur.md): Aufbau und Module
- [`docs/entscheidungen.md`](docs/entscheidungen.md): warum etwas so ist,
  wie es ist (bitte vor größeren Umbauten lesen)
- [`docs/hardware.md`](docs/hardware.md): Geräte, Ports, Stolperfallen am Pi

## Starten

```bash
python main.py --mock     # auf dem eigenen Rechner, alles simuliert
python main.py            # auf dem Roboter
```

Die Simulation deckt Antrieb, Lidar (gedachter Raum) und Kamera (Testbild)
ab. Alle Endpunkte verhalten sich wie am Roboter.

## Regeln für Änderungen

- **`protocol.py` ist der Vertrag.** Bestehende Nachrichten und Feldnamen
  nicht umbenennen oder umdeuten, sondern neue ergänzen. Neue Befehle in
  `protocol.py`, `api/server.py` (`_handle`) und `docs/api.md` eintragen.
- **Fahrbefehle nur über `SafetyWatchdog.submit()`**, nie direkt an den
  Treiber. Sonst greifen Tempolimit und Totmannschaltung nicht.
- **Einstellungen gehören nach `config.py`**, nicht als Zahlen in den Code.
- **Konventionen:** `vx` vorwärts, `vy` links, `omega` gegen den
  Uhrzeigersinn; Lidar-Winkel 0° = vorn, 90° = links. Details in
  `docs/api.md`.
- **Sprache:** Code-Kommentare, Bezeichner, Log-Meldungen, Doku und
  Commit-Nachrichten sind auf **Deutsch**. Kommentare erklären das *Warum*.
- **Hardware-Code muss Ausfälle überstehen.** USB-Geräte melden sich am X3
  gelegentlich ab und wieder an (Spannungseinbruch). Treiber und Sensoren
  verbinden sich selbst neu; neue Hardware-Anbindungen sollten das ebenso
  tun und dabei nie einen fahrenden Roboter zurücklassen.

## Am Roboter

- Board und Lidar hängen beide an USB-Seriell. Der Lidar hat den festen
  Pfad `/dev/rplidar`; das Board wird über `/dev/serial/by-id/` gefunden.
- Nur **ein** Programm darf gleichzeitig an Board bzw. Lidar. ROS2-Knoten
  (`sllidar_node`, `yahboomcar_bringup`) und alte Server-Instanzen vorher
  beenden: `pkill -f rosmaster_assist; sudo pkill -f sllidar_node`.
- Diagnose ohne Log-Suche: `/lidar/status`, `/camera/status` und
  `/stimme/status` (Lautsprecher).

## Dokumentieren

Jeder Arbeitsschritt kommt zusätzlich ins Projekttagebuch
[`Marlon/PROJEKTDOKU.md`](Marlon/PROJEKTDOKU.md): was, warum, Ergebnis,
Probleme. Es ist die Grundlage für Abschlussbericht und Präsentation.
Technische Details gehören weiterhin nach `docs/`.
