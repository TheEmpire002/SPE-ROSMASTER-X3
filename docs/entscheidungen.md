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

**Entschieden in E4:** Der Lidar wird direkt ausgelesen, ohne ROS2.

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

---

## E4 — Lidar direkt auslesen, Ortung per Aufnahmeabgleich

**Datum:** 2026-10-27
**Status:** aktiv, schließt die offene Frage aus E1a. Die Ortung gegen nur
*eine* Referenzaufnahme ist durch E5 erweitert.

**Kontext.** Die Rückfahrt zum Startpunkt lief über Koppelnavigation und
driftete entsprechend: in der Simulation rund 30 cm auf zweieinhalb Metern,
in Wirklichkeit eher mehr. Der RPLIDAR A1 war vorhanden, aber ungenutzt.

**Entscheidung.** Der Lidar wird **direkt aus Python** ausgelesen, über das
Paket `rplidar`, ohne ROS2. Zur Ortung dient **kein SLAM**, sondern der
Abgleich der aktuellen Aufnahme mit einer einzigen Referenzaufnahme vom
Startpunkt (Iterative Closest Point, `navigation/scan_matching.py`).

**Warum kein ROS2, obwohl es installiert ist.** Es brächte einen zweiten
Prozess mit eigenem Port-Anspruch, eine Brücke zwischen beiden Welten und
eine EOL-Abhängigkeit (Foxy, seit 2023 ohne Pflege). Das Auslesen selbst ist
dagegen ein überschaubares Stück Code. Für genau eine Sensorart lohnt der
Mehraufwand nicht.

**Warum kein SLAM.** Eine Karte aufzubauen und darin zu lokalisieren ist das
Vielfache an Aufwand. Für „zurück zum Startpunkt" genügt der Vergleich mit
*einem* Ort -- dem Startpunkt.

**Arbeitsteilung der beiden Ortungsverfahren.** Die Koppelnavigation läuft
immer und ist nie verloren, driftet aber. Der Lidar-Abgleich driftet nicht,
braucht aber Wiedererkennung. Also: Die Odometrie bringt den Roboter *in die
Nähe*, der Lidar weist ihn dort genau ein. Jenseits von
`MATCH_RADIUS_M` wird gar nicht erst abgeglichen -- ein Fehlabgleich wäre
schlimmer als keine Korrektur.

**Messergebnis (Simulation, 12 % Radschlupf, 2,3 m Fahrstrecke):**

| | Abweichung während der Fahrt | im Stand |
|---|---|---|
| nur Koppelnavigation | 30,5 cm | 30,5 cm |
| mit Lidar-Abgleich | 7,3 cm | 5,6 cm |

Die verbleibenden Zentimeter sind die Winkelauflösung des A1: ein Grad
entspricht auf vier Meter Entfernung sieben Zentimetern.

**Konsequenzen.**
- *Gut:* Deutlich genauer, ohne zweiten Prozess und ohne ROS-Kenntnisse.
  Nebenbei fällt die Hinderniskontrolle während der Heimfahrt ab.
- *Schlecht:* Funktioniert nur in Sichtweite des Startpunkts. Wer durch
  mehrere Räume fahren will, braucht doch eine Karte.
- *Offen:* Der Tiefenkanal der Orbbec bleibt ungenutzt -- er bräuchte das
  Orbbec-SDK (kein `pip`-Paket). Interessant wäre er für niedrige
  Hindernisse, die der Lidar auf seiner Höhe nicht sieht.

**Neu bewerten, wenn:** der Roboter zwischen Räumen wechseln soll. Dann
führt an einer Karte kein Weg vorbei.

---

## E5 — Karte aus Schlüsselbildern, Rückweg, Marker zum Andocken

**Datum:** 2026-10-27
**Status:** aktiv, erweitert E4 (Branch `feature/return_home`)

**Kontext.** Am Roboter kam die Heimfahrt bei Fahrten über 4–5 m oder mit
Drehungen weit daneben heraus. Ursachen im Code: Jenseits von
`MATCH_RADIUS_M` (4 m) gab es keinen Abgleich mehr; Aufnahmen während
schneller Drehungen sind verzerrt, wurden bei der großzügigen Toleranz von
18 cm aber übernommen; und die Luftlinie zum Start führte durch Wände.

**Entscheidung.**
- **Karte aus Schlüsselbildern** statt einer Einzelaufnahme: alle 0,6 m eine
  Aufnahme samt Position, aber nur aus sehr guten Abgleichen. Abgeglichen
  wird gegen die nächstgelegenen drei. Kein Ausgleich der Karte im
  Nachhinein (kein vollständiges SLAM).
- **Strengere Übernahme:** Restabstand ≤ 10 cm, mindestens die Hälfte der
  Punkte mit Partner, kein Abgleich bei schneller Drehung, keine weiten
  Sprünge. Nach verlorener Spur wird mit mehreren Startwinkeln neu gesucht.
- **IMU-Drehsinn** wird beim ersten Drehen gegen die Befehle geprüft.
- **Rückweg** entlang der aufgezeichneten Strecke, Schleifen gestrichen,
  Abkürzung bei freier Lidar-Sicht.
- **Feinausrichtung im Stand** und Wiederherstellen der Blickrichtung.
- **ArUco-Marker** (umschaltbar) für das letzte Stück: Die Kamera merkt sich
  beim Setzen, wie der Marker liegt, und regelt am Ende auf genau diese
  Ansicht. Keine Kalibrierung nötig, weil Referenz und Messung denselben
  Fehler teilen.

**Warum der Marker auf der CPU läuft und nicht auf dem KI-Modul.** Die
Erkennung braucht wenige Millisekunden je Bild. Das KI-Modul bleibt für
Aufgaben frei, die ein neuronales Netz brauchen (Personenerkennung).

**Konsequenzen.**
- *Gut:* Keine Reichweitengrenze mehr; Türen und Ecken auf dem Hinweg sind
  auch auf dem Rückweg passierbar; mit Marker zentimetergenau.
- *Schlecht:* Fehler, die sich auf dem Hinweg in die Karte einschleichen,
  bleiben dort. Die Karte lebt nur im Speicher und beginnt bei jedem
  Serverstart bzw. „Startpunkt setzen“ neu.

**Neu bewerten, wenn:** die Karte über Neustarts hinweg halten oder
mehrere Räume dauerhaft abdecken soll. Dann lohnt echtes SLAM mit
Schleifenschluss.
