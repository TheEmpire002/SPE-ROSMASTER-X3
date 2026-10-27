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
