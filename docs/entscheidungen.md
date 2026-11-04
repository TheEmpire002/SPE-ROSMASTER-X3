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

---

## E8 — Karte mit slam_toolbox im Container, unser Server speist Lidar und Position ein

**Datum:** 2026-11-03
**Status:** umgesetzt (`mapping/einspeisung.py`), am Roboter noch zu prüfen

**Kontext.** Das Mapping-Modul (`mapping/`) startete im Docker-Container `x3`
nur slam_toolbox. Das ist aber nur der Kartenbauer: Er braucht Lidar-Scans auf
`/scan` und die Position als TF (`odom → base_footprint`). Die liefern im
Yahboom-Stack `sllidar_node` und `yahboomcar_bringup`. Beide laufen bei uns
nicht und dürfen es auch nicht, weil unser Server Lidar und Board offen hat
(E1a, E4). slam_toolbox wartete deshalb endlos, es entstand nie eine Karte,
und keine Stelle meldete das.

**Entscheidung.** Unser Server bleibt der einzige Leser von Lidar und Board.
Er reicht Scans und Odometrie an einen kleinen ROS-Knoten im Container weiter.
Der Knoten wird über `docker exec -i` gestartet, liest JSON-Zeilen von stdin
und veröffentlicht `/scan`, `/odom`, `/tf` und `/tf_static`. slam_toolbox
läuft unverändert mit seiner Standard-Konfiguration.

Andere Wege wurden nicht genommen:
- *Zum Kartieren unseren Server beenden und den Yahboom-Stack starten:* Dann
  fallen App, Watchdog und Totmannschaltung während der Kartierung weg.
- *Karte selbst in Python bauen:* weniger Aufwand, aber ohne Schleifenschluss.
  Die Karte driftet mit der Odometrie.

**Einzelheiten, die zählen.**
- Der A1 liefert jede Umdrehung etwas andere Punkte, slam_toolbox verwirft
  aber jeden Scan mit anderer Strahlzahl als der erste. Deshalb geht der Scan
  auf ein festes Raster (`MAP_SCAN_STRAHLEN` = 360), ohne Echo als `inf`.
- Zu jedem Scan geht die Position zum Messzeitpunkt mit (`pose_bei`), denn
  slam_toolbox fragt TF genau für den Zeitstempel des Scans ab.
- Wird der Startpunkt neu gesetzt, springt die Odometrie auf (0, 0). Dieser
  Sprung wird ausgeblendet (`MAP_SPRUNG_M`, `MAP_SPRUNG_GRAD`), sonst trüge
  SLAM die Wände doppelt ein.
- Stirbt der Knoten, startet die Einspeisung ihn mit wachsender Pause neu.
  Endet unser Server, bekommt er EOF und beendet sich selbst.

**Konsequenzen.**
- *Gut:* Die Karte entsteht mit Schleifenschluss. Fahren, Watchdog und App
  laufen dabei normal weiter.
- *Gut:* Die Seite `/map` sagt jetzt, an welchem Schritt es hängt (Lidar,
  Weitergabe, SLAM, Karte), und zeigt das SLAM-Protokoll.
- *Schlecht:* Wir hängen doch an ROS2 Foxy (EOL) und am Container `x3`.
  slam_toolbox muss dort installiert sein.
- *Überholt durch E9:* Die Position für SLAM kommt nicht mehr aus der
  Koppelnavigation, sondern aus der Lidar-Odometrie.

**Neu bewerten, wenn:** die Karte zur Navigation genutzt werden soll. Dann
gehört auch die Position aus SLAM (`map → odom`) zurück in unseren Server.

---

## E9 — Blickrichtung aus dem Lidar, Kartierfahrt wie ein Saugroboter

**Datum:** 2026-11-03
**Status:** umgesetzt (`navigation/scan_odometrie.py`, `navigation/kartierfahrt.py`), in der Simulation geprüft, am Roboter noch nicht

**Kontext.** Für E8 brauchte SLAM die Position samt Blickrichtung. Die
Koppelnavigation liefert am X3 aber keine brauchbare Drehung: Die IMU der
Bibliothek meldet als Kurswinkel immer 0 (Sensor-Check 28.10.), und am
Boden dreht er nur rund drei Viertel des Befohlenen. Für SLAM hieße das:
Dreht er sich, glaubt die Karte, er stehe still. Außerdem fuhr niemand den
Roboter beim Kartieren, und eine Karte aus einem Blickwinkel hat überall
Schatten.

**Entscheidung.**
- **Lidar-Odometrie:** Jede Aufnahme wird per ICP auf eine kurz zuvor
  gemachte gelegt (vorhandenes `scan_matching.py`). Die Räder liefern nur den
  Startwert. Verglichen wird mit einer Bezugsaufnahme, die erst nach 0,25 m
  oder 10° erneuert wird; so summiert sich im Stand kein Rauschen auf. Sie
  läuft nur beim Kartieren, weil der Abgleich CPU kostet.
- **Kartierfahrt** als zweiter Modus des Erkunders:
  1. zur nächsten Wand,
  2. mit der Wand rechts einmal rundherum,
  3. Bahnen parallel zu dieser Wand bis zur gegenüberliegenden.
  Auf den Bahnen dreht er nicht, sondern fährt vor, rutscht seitlich und
  fährt rückwärts (Mecanum). Die Richtung der Bahnen hält er an den Wänden
  fest: Zimmerwände stehen fast immer rechtwinklig, ihre Richtung lässt sich
  aus jeder einzelnen Aufnahme ablesen und driftet daher nicht.

Andere Wege wurden nicht genommen:
- *IMU reparieren:* Die Bibliothek liefert den Wert nicht, das ist
  außerhalb unserer Hand.
- *Drehrate der Räder aufsummieren:* rutscht bei jeder Drehung um ein Viertel.
- *Position von slam_toolbox zurückholen:* Dafür müssten wir TF aus dem
  Container lesen. Das ist eine zweite Brücke in die andere Richtung, und
  ohne Karte (am Anfang) gibt es auch dort nichts.
- *Wenden wie ein Saugroboter:* Jede Drehung ist am X3 der ungenaueste Teil
  der Bewegung und für SLAM der heikelste.

**Messung (Simulation, Raum 8 × 6 m mit Säule, 12 % Radschlupf, 4 min Fahrt):**

| | Abweichung am Ende |
|---|---|
| nur Räder | 4,1 m, 49° |
| Lidar-Odometrie | 0,3 m, 4,5° |

Die Säule hielt er zuerst für die nächste Wand und fuhr einmal um sie
herum. Daraus entstand die Inselerkennung: Eine Runde unter
`MAP_FAHRT_INSEL_M` war keine Wand. Die Bahnen lagen anfangs schräg, weil
sie der langsamen Drift der Odometrie folgten. Seit sie an der
Wandrichtung ausgerichtet sind, laufen sie parallel.

**Konsequenzen.**
- *Gut:* SLAM bekommt brauchbare Drehungen. Die Bahnen bleiben parallel,
  ohne dass etwas aufsummiert wird. Alle Sicherheitswege des Erkunders
  gelten unverändert.
- *Schlecht:* In Räumen ohne klare rechte Winkel (Dachschrägen, runde
  Wände) fällt die Bahnführung auf die Odometrie zurück und driftet langsam.
- *Schlecht:* Ob die Randfahrt „einmal rundherum“ erkennt, hängt an der
  Odometrie. Kommt er nicht nah genug an den Anfang zurück, endet sie erst
  nach `MAP_FAHRT_RAND_MAX_S`.
- *Offen:* Die A1-Aufnahmen sind beim Drehen leicht verzerrt (eine Umdrehung
  dauert 0,18 s). Das gibt es in der Simulation nicht.

**Neu bewerten, wenn:** die Heimfahrt dieselbe Drehung braucht. Die
Lidar-Odometrie könnte dann dauerhaft laufen und die Koppelnavigation
ersetzen, wenn die CPU es hergibt.

---

## E10 — Fahrschutz über die Grundfläche statt Kegel in Fahrtrichtung

**Datum:** 2026-11-03
**Status:** umgesetzt (`safety/fahrschutz.py`), getestet ohne Roboter

**Kontext.** Die Kollisionsvermeidung lief nicht zuverlässig. Bei der
Handsteuerung prüfte sie einen Kegel von ±35° ab Lidar-Mitte:
- Nah am Roboter ist so ein Kegel schmal und übersieht das Tischbein vor dem
  Rad. Weit weg ist er breit und hält vor Wänden, an denen der Roboter
  vorbeikäme.
- Sie sperrte hart erst an der Grenze. Bei 0,22 s Befehlsverzögerung rollt
  der Roboter dann noch weiter.
- Drehen war immer erlaubt, obwohl die Ecken dabei ausschwenken.
- Sie nahm Scans an, die bis zu 2 s alt waren.

Die selbstständigen Fahrten prüften jeweils nur einzelne Richtungen. Eine
Mischung aus vor, seitwärts und drehen prüfte niemand als Ganzes.

**Entscheidung.** Eine gemeinsame Prüfung für alle Bewegungen. Für den
Befehl wird vorausgerechnet, wo die rechteckige Grundfläche des Roboters in
den nächsten Sekunden ist, auch im Bogen und beim Drehen. Bremsen muss er,
sobald ein Lidar-Punkt den Rand unterschreitet *und* näher kommt. Das Tempo
wird dann so weit gesenkt, dass der Anhalteweg (Reaktionszeit plus Alter
des Scans plus Bremsweg) ins Freie passt. Alle Anteile des Befehls werden
gleich skaliert, die Bahn bleibt also dieselbe, nur langsamer.
- Gilt immer für Erkunder, Kartierfahrt, Heimfahrt, Interaktion und die
  genauen Fahrbefehle (`manoever.py`): im Watchdog bei
  `submit(..., quelle="autonom")`. Für Drehungen mit festem Winkel prüft
  `Fahrschutz.drehen_frei()` vorab die Grundfläche über den ganzen Winkel;
  das ersetzt den 30-cm-Kreis um den Lidar, der im Bodentest grundlos
  sperrte.
- Gilt für die Handsteuerung, wenn der Hindernisstopp an ist. Der Regler
  `distance` behält seine Bedeutung (ab Lidar-Mitte nach vorn).
- Selbstständige Fahrten fahren ohne frischen Scan (`FAHRSCHUTZ_SCAN_MAX_S`)
  nicht. Die Handsteuerung lässt der Schutz dann durch: Wer von Hand fährt,
  sieht den Roboter.

**Gemessen** (Simulation, Antrieb mit 0,22 s Verzögerung, Vollgas auf eine
Wand): Bei 0,2, 0,5 und 0,8 m/s steht er jedes Mal 6 cm vor der Kante, das
ist genau der Rand `FAHRSCHUTZ_RAND_M` (damals 6 cm). Die Kartierfahrt läuft
mit Schutz genauso durch wie ohne.

**Zusammengeführt mit `AutonomSchutz` (03.11.).** Parallel war in `main` eine
zweite Schicht entstanden: `AutonomSchutz` begrenzte das Tempo nach dem
Bremsweg bis zum nächsten Punkt in einem Streifen in Fahrtrichtung und ließ
jede Drehung durch. Zwei Schichten im Watchdog hätten doppelt und
unterschiedlich geprüft. Übrig bleibt eine:
- Die Schnittstelle aus `main`: `quelle="autonom"` (dazu engere Tempogrenze,
  Akku-Drossel, Fahrtenschreiber), abschaltbar mit `AUTONOM_SCHUTZ_AKTIV`.
- Darin prüft der Fahrschutz, weil er Bögen und ausschwenkende Ecken sieht.
  Übernommen aus `AutonomSchutz`: Alter des ältesten Punkts der Runde statt
  ihres Zeitstempels, Aufbereiten der Punkte einmal pro Runde, gezählte und
  entprellt geloggte Eingriffe.
- Eine Grundfläche (`ROBOTER_*`) für Fahrschutz und Bogenplanung
  (`AUSWEICH_*`). Der Fahrschutz-Rand (4 cm) liegt bewusst unter dem des
  Planers (5 cm): Die Sicherheitsschicht ist die innere Grenze. Wäre sie
  strenger als der Planer, würgte sie dessen Bögen ab.

**Konsequenzen.**
- *Gut:* Eine Stelle für alles. Sie bremst weich statt hart und hält den
  Roboter nicht neben Hindernissen fest.
- *Schlecht:* Sie sieht nur, was der Lidar auf seiner Höhe sieht. Niedrige
  Dinge (Kabel, Stuhlkufen, Bettgestell) und überhängende (Tischplatte)
  bleiben unsichtbar.
- *Schlecht:* Die Maße der Grundfläche sind teils geschätzt
  (`ROBOTER_HINTEN_M`). Sie müssen am Roboter nachgemessen werden.

**Neu bewerten, wenn:** die Orbbec-Tiefe eingebunden wird. Ihre Punkte
können in dieselbe Prüfung gehen.

**Nachtrag 04.11.:** Die Tiefe ist eingebunden (E11). Ihre Punkte gehen
über das Hindernisgedächtnis in dieselbe Prüfung (`FAHRSCHUTZ_MIT_TIEFE`).

---

## E11 — Tiefenkamera fürs Ausweichen, Fortschrittswache gegen Festhängen

**Datum:** 2026-11-04
**Status:** umgesetzt (`sensors/tiefe.py`, `navigation/hindernisgedaechtnis.py`,
`navigation/fortschritt.py`), am Roboter noch nicht im Fahrbetrieb getestet

**Kontext.** Beim Kartieren und Erkunden blieb der Roboter an Stuhlbeinen
und ähnlich niedrigen oder dünnen Dingen hängen. Der Lidar sieht nur seine
eine Ebene. Für die Fahrlogik war vorn frei, der Befehl ging raus, nur
bewegte er sich nicht. Die Kartierfahrt hatte dafür keine Erkennung (die
Festfahr-Erkennung des Erkunders lief nur beim Zufallsfahren), also stand er,
und die Karte wuchs nicht.

**Entscheidung.** Zwei unabhängige Hilfen, beide für alle autonomen Modi:
1. **Sehen, was der Lidar nicht sieht.** Der Tiefenkanal der Astra liefert
   Punkte, die 4–40 cm über dem selbst gefundenen Boden liegen. Weil die
   Astra erst ab 0,6 m misst, merkt sich ein Hindernisgedächtnis die Punkte
   im Lage-Rahmen (Lidar-Odometrie beim Kartieren, sonst Odometrie) und
   gibt sie im Roboterrahmen zurück. Sieht die Kamera an einer Stelle
   wieder freien Boden, fliegt der Punkt raus. Erkunder, Kartierfahrt und
   Fahrschutz bekommen die Punkte einfach zu den Lidar-Punkten dazu, ohne
   eigene Logik.
2. **Merken, dass es nicht vorangeht.** Die Fortschrittswache vergleicht
   alle 0,5 s die befohlene Bewegung der letzten 2 s (was der Watchdog nach
   Rampe und Fahrschutz wirklich ausgab) mit der vom Lidar gemessenen (ICP
   zwischen zwei Aufnahmen). Kommt weniger als 30 % an, gilt er als
   festgefahren. Dann setzt sie ein Phantom-Hindernis vor die Grundfläche
   in Fahrtrichtung und schiebt den Roboter von dort weg frei.

**Warum so.**
- *Lidar statt Rad als Messung:* Durchdrehende oder blockierte Räder sieht
  die Odometrie nicht; sie rechnet aus den Motoren. Der Abgleich der
  Lidar-Aufnahmen schaut auf die Wände.
- *Befohlen nach dem Watchdog:* Bremst der Fahrschutz vor einer Wand auf
  null, ist auch die Erwartung null. Stehen vor einem Hindernis ist kein
  Festhängen.
- *Doppelter Abgleich:* ICP wird einmal ab „nicht bewegt“ und einmal ab
  „wie befohlen“ gerechnet. Liegen die Ergebnisse auseinander (Flur, kahle
  Wand), urteilt die Wache nicht. Ein falsches „festgefahren“ würde mitten
  im freien Raum Befreiungsmanöver auslösen.
- *Tiefe nicht in die Karte:* Die Karte (E8) bleibt eine reine Lidar-Karte.
  Tiefenpunkte haben eine andere Höhe und mehr Rauschen; in slam_toolbox
  würden sie den Abgleich stören.
- *Kamera nur während der Fahrt:* Der Projektor zieht viel Strom am Hub,
  der bei Spannungseinbrüchen USB-Geräte verliert (hardware.md, USB-Resets).
- *Eigene ctypes-Anbindung an OpenNI2:* Das pip-Paket läuft nicht unter
  Python 3.13, ein ROS-Knoten widerspräche „kein ROS im Server“.

**Konsequenzen.**
- *Gut:* Stuhlbeine werden ab ~1,5 m gesehen und umfahren. Was keiner
  sieht, erkennt die Wache nach ~2 s und meidet die Stelle 30 s lang.
- *Schlecht:* Unter 0,6 m ist die Kamera blind. Was dort neu auftaucht,
  erkennt erst die Fortschrittswache.
- *Schlecht:* Im Zufallsmodus rechnet das Gedächtnis mit der Rad-Odometrie.
  Rutscht er beim Festhängen, verschieben sich gemerkte Punkte etwas.
- *Schlecht:* Ohne erkannten Boden (dunkler Boden) wird es ungenauer und
  vorsichtiger (erst ab 9 cm, nur bis 1,3 m).

**Neu bewerten, wenn:** sich die Wache im Betrieb oft irrt (dann
`FORTSCHRITT_ANTEIL`/`_FENSTER_S` anpassen) oder ein Sensor für den
Nahbereich (Ultraschall, Bumper) dazukommt.
