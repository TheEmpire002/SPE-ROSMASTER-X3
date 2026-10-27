# Entscheidungen

Kurzprotokoll der Architekturentscheidungen: *was* wurde entschieden, *warum*,
und *wann es neu zu bewerten ist*. Neue Einträge unten anhängen.

---

## E1 — Kein ROS im Grundgerüst, reines Python

**Datum:** 2026-10-27
**Status:** aktiv

**Kontext.** Der Roboter läuft auf einem Raspberry Pi 5 mit AI HAT+. Dafür gibt
es Raspberry Pi OS (Bookworm, Debian 12). ROS2 hat keine offiziellen Pakete für
diese Kombination: Humble zielt auf Ubuntu 22.04, Jazzy auf Ubuntu 24.04. ROS2
auf dem Pi 5 bedeutet also Docker oder Source-Build. Im Team ist bereits
Python-Erfahrung vorhanden, ROS-Erfahrung nicht.

**Entscheidung.** Das Grundgerüst nutzt Yahboons `Rosmaster_Lib` direkt aus
Python. Kein ROS.

**Konsequenzen.**
- *Gut:* App-Steuerung, ESP-Anbindung und Box funktionieren sofort, ohne dass
  jemand erst ROS lernen muss. Das Projekt läuft auch auf einem Laptop (Mock).
- *Schlecht:* Für SLAM und Navigation gibt es in reinem Python nichts, was an
  Nav2 heranreicht. Dieser Teil wird aufwendiger.
- *Abgefedert durch:* Die Treiber-Abstraktion in `drive/base.py`. Ein späterer
  ROS2-Container kann sich hinter derselben Schnittstelle verbergen.

**Neu bewerten, wenn:** das autonome Fahren beginnt und der Lidar Daten liefert.

---

## E1a — Korrektur: ROS2 ist vorhanden

**Datum:** 2026-10-27
**Status:** aktiv, ergänzt E1

**Was sich geändert hat.** Die Annahme in E1, für den Pi 5 gebe es kein
nutzbares ROS2, war falsch. Auf dem Roboter liegt **ROS2 Foxy** unter
`/opt/ros/foxy`, dazu ein eingerichteter Yahboom-Workspace
(`/root/yahboomcar_ros2_ws`) mit:

- `yahboomcar_bringup` — Antriebsknoten, spricht dieselbe `Rosmaster_Lib`
- `sllidar_ros2` — laufender Treiber für den RPLIDAR unter `/dev/rplidar`
- `yahboomcar_ctrl` — Joystick-Steuerung

**Folge für das Grundgerüst: keine.** Die Entscheidung aus E1 bleibt für
App-Steuerung, Kamera und ESP richtig — die laufen ohne ROS, sind fertig und
funktionieren.

**Folge für das autonome Fahren: erheblich.** Lidar und Odometrie sind dort
bereits gelöst. Eine eigene Navigation in Python dagegen neu zu schreiben,
wäre deutlich mehr Arbeit bei schlechterem Ergebnis.

**Wichtig:** `yahboomcar_bringup` und unser Server können **nicht
gleichzeitig** laufen. Beide öffnen denselben seriellen Port, und zwei Leser
bekommen jeweils nur Bruchstücke. Vor dem Start unseres Servers muss der
ROS2-Stack beendet sein — und umgekehrt.

**Offene Frage.** Beim autonomen Fahren entweder unser Server spricht über
eine Brücke mit ROS2 (dann besitzt ROS2 den Port), oder wir lesen den Lidar
selbst aus. Zu entscheiden, wenn es soweit ist. Foxy ist seit 2023 ohne
Unterstützung, was für ein Schulprojekt verkraftbar ist, aber ein Argument
gegen zu viel Aufbau darauf bleibt.

---

## E2 — WebSocket-Protokoll statt REST

**Datum:** 2026-10-27
**Status:** aktiv

**Kontext.** Die App muss den Roboter flüssig steuern und gleichzeitig Status
und Alarme empfangen. Fahrbefehle kommen mehrmals pro Sekunde.

**Entscheidung.** Eine dauerhafte WebSocket-Verbindung mit JSON-Nachrichten,
gemeinsam genutzt von App und ESP.

**Konsequenzen.**
- *Gut:* Der Roboter kann von sich aus senden — nötig für Notfall-Alarme.
  Eine offene Verbindung ist außerdem die natürliche Grundlage für den
  Watchdog: bricht sie ab, hält der Roboter an.
- *Schlecht:* Etwas mehr Aufwand als REST, und Verbindungsabbrüche müssen in
  der App behandelt werden.

---

## E3 — Vitaldaten lösen keine autonome Reaktion aus

**Datum:** 2026-10-27
**Status:** aktiv

**Kontext.** Ein Pulssensor am ESP32 ist ungenau und störanfällig. Falsche
Messwerte sind der Normalfall, nicht die Ausnahme.

**Entscheidung.** Auffällige Vitaldaten lösen einen Alarm an einen Menschen
aus. Das System bewertet keinen Gesundheitszustand und ruft nicht selbstständig
einen Rettungsdienst.

**Konsequenzen.** Die Notfallkette braucht immer einen Empfänger — App,
Nachricht, Signalton. Das ist beim Design der Alarmierung einzuplanen.
