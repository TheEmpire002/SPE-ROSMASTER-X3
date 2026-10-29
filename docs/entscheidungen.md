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
**Status:** aktiv, schließt die offene Frage aus E1a

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

## E5 — Gesten über die Körperpose auf dem Hailo, nicht über ein Handmodell

**Datum:** 2026-10-28
**Status:** beschlossen, Umsetzung in Stufe 2 (siehe [`interaktion.md`](interaktion.md))

**Kontext.** Der Roboter soll auf Handzeichen reagieren: Folgen, Stopp,
Herkommen, Weggehen. Übliche Handmodelle (z. B. MediaPipe Hands) erkennen
Fingerstellungen, brauchen die Hand aber groß im Bild, also auf etwa einen
Meter, und laufen auf der CPU. Die CPU ist am X3 heikel, denn Lastsprünge
lösen USB-Resets aus (siehe [`hardware.md`](hardware.md)). Auf dem Pi
steckt ein Hailo-10H, und mitgeliefert ist ein Pose-Modell
(`yolov8s_pose_h10.hef`, 17 Körperpunkte).

**Entscheidung.** Gesten werden aus der **Körperpose** bestimmt: aus der
Lage von Handgelenken, Ellbogen und Schultern und ihrer Bewegung über eine
kurze Zeit. Das Modell läuft auf dem Hailo. Die Gestenlogik besteht aus
reinen Python-Funktionen über die Keypoints.

**Konsequenzen.**
- *Gut:* Das funktioniert auf 1–4 m, also auf dem Abstand, aus dem man
  einem fahrenden Roboter winkt. Die CPU bleibt frei. Die Gestenlogik ist
  ohne Hardware testbar, und dieselbe Pose liefert die Richtung zur Person
  fürs Folgen.
- *Schlecht:* Es gibt keine Fingergesten (Daumen hoch, Zahl zeigen), nur
  Armgesten. Die C270 ist nur leicht nach oben geneigt, deshalb ist eine
  stehende Person erst ab etwa 1,5 m ganz im Bild.
- *Abgefedert durch:* Die Gesten sind bewusst groß und eindeutig gewählt
  (Arm über den Kopf, winken, wischen). Eine Geste zählt erst nach mehreren
  stabilen Bildern.

**Neu bewerten, wenn:** feinere Gesten gebraucht werden. Dann kommt ein
Handmodell als zweite Stufe dazu, aber nur auf den Bildausschnitt der
Hand, den die Pose liefert.

---

## E6 — Hybrides Denken; das Sprachmodell steuert nie direkt

**Datum:** 2026-10-28
**Status:** beschlossen, Umsetzung in Stufe 5

**Kontext.** Der Roboter soll sich unterhalten können und so wirken, als
hätte er ein Innenleben. Ein großes Sprachmodell (Claude) klingt am
natürlichsten, braucht aber Internet und kostet pro Aufruf. Lokal auf dem
Hailo-10H laufen nur kleine Modelle. Vor allem aber können Sprachmodelle
sich irren, und ein Irrtum darf nie zu einer Fahrbewegung führen.

**Entscheidung.** Das Denken läuft **hybrid**, mit einer Schnittstelle und
drei Stufen: Claude-API, wenn Internet da ist, dann ein lokales LLM auf dem
Hailo (später) und schließlich Vorlagensätze. Das Sprachmodell liefert nur
`gedanke`, `antwort` und optional eine `absicht` aus einer festen Liste. Ob
diese Absicht ausgeführt wird, entscheidet der Geist. Fahrbefehle entstehen
ausschließlich in den Verhaltensklassen und laufen über den Watchdog.
Gesten und Sprachbefehle („stopp“) wirken ohne Sprachmodell.

**Konsequenzen.**
- *Gut:* Die Unterhaltung wirkt natürlich, wenn Internet da ist, und ohne
  Internet funktioniert trotzdem alles. Ein falsches oder böswilliges
  Modell-Ergebnis kann den Roboter nicht unkontrolliert fahren lassen.
  „Stopp“ hängt nie an einer Netzverbindung.
- *Schlecht:* Mit Vorlagen wirkt der Roboter deutlich schlichter. Die
  Claude-API braucht einen Schlüssel und ein Budget.
- *Abgefedert durch:* Gedanken entstehen nur bei Ereignissen oder alle
  ~20 s. Der Schlüssel liegt nur in der Umgebungsvariable
  `ANTHROPIC_API_KEY` und nie im Repo.

**Neu bewerten, wenn:** das lokale LLM auf dem Hailo gut genug antwortet,
dass die Cloud verzichtbar wird.

## E7 — Sprachausgabe offline mit Piper, über aplay, im eigenen Thread

**Datum:** 2026-10-28
**Status:** umgesetzt (`interaktion/stimme.py`), am Roboter noch zu prüfen

**Kontext.** Am Pi hängt jetzt ein USB-Lautsprecher. Der Roboter soll
ansagen, was er tut, und später Antworten sprechen. Die Stimme muss
deutsch und verständlich sein und auch ohne Internet gehen, genau wie die
Reflexe. Sie darf den Server nicht ausbremsen: Ein Satz dauert ein, zwei
Sekunden, die Fahrschleife läuft mit 10 Hz.

**Entscheidung.**
- Gesprochen wird mit **Piper** (offline, Stimme `de_DE-thorsten-medium`). Die Ausgabe geht über **`aplay`** an den Lautsprecher, der über seinen ALSA-Namen angesprochen wird.
- Piper und aplay laufen als **Unterprozesse**, gesteuert aus einem eigenen Thread mit kleiner Warteschlange.
- Fehlt Piper, spricht espeak-ng, fehlt auch das, bleibt es beim Log.
- Cloud-Stimmen wurden nicht genommen, weil sie Internet brauchen. Eine Python-Audiobibliothek (`sounddevice`, `pyaudio`) wurde auch nicht genommen: Sie wäre eine zusätzliche Abhängigkeit, und `aplay` ist ohnehin da.

**Konsequenzen.**
- *Gut:*
  - Es geht ohne Internet und ohne neue pip-Pakete.
  - Ein abgestürzter Piper oder ein abgezogener Lautsprecher reißt den Server nicht mit.
  - Ein Not-Stopp kann den laufenden Satz einfach beenden: `kill` auf den Unterprozess.
  - Über den Kartennamen übersteht die Ausgabe USB-Resets.
- *Schlecht:*
  - Piper muss von Hand installiert werden, Programm plus Modell.
  - Jeder Satz startet Piper neu. Das Modell zu laden kostet am Pi 5 einen Moment, bevor der Ton kommt.
- *Abgefedert durch:* Die Sätze sind kurz und selten. Gleiche Sätze innerhalb weniger Sekunden werden verschluckt. Wird die Verzögerung störend, kann Piper dauerhaft laufen und Sätze über stdin bekommen. Das wäre eine Änderung nur in `stimme.py`.

**Neu bewerten, wenn:** die Verzögerung im Gespräch (Stufe 5) stört, oder
wenn Spracherkennung und Ausgabe gemeinsam ein Audio-Framework brauchen.
