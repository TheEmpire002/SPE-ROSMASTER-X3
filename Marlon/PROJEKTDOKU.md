# Projektdokumentation – Hilfsroboter auf dem ROSMASTER X3

Diese Datei ist unsere **lebende Abschlussdokumentation**. Jeder Arbeitsschritt
wird hier festgehalten: was wir gemacht haben, warum, was herauskam und was
schiefging. Am Projektende ist sie die Grundlage für Abschlussbericht und
Präsentation.

> **Regel:** Neue Einträge unten im Projekttagebuch anhängen (neuestes zuletzt),
> mit Datum. Technische Details für Entwickler stehen in `docs/` – hier steht
> die Geschichte dazu und das, was wir daraus gelernt haben.

---

## 1. Ziel und Ausgangslage

**Ziel:** Ein Hilfsroboter, der sich in einer Wohnung bewegt, Menschen
wahrnimmt und mit ihnen interagiert. Er wird per App gesteuert, fährt selbst
zum Startpunkt zurück, erkundet autonom und soll künftig auf Gesten und
Sprache reagieren.

**Plattform:** Yahboom ROSMASTER X3 mit diesen Komponenten:
- Mecanum-Räder,
- Raspberry Pi 5 (16 GB) mit Hailo-10H-KI-Beschleuniger (AI HAT+ 2),
- RPLIDAR A1,
- Orbbec-Tiefenkamera und Logitech C270,
- Expansion-Board (STM32) für Motoren, IMU und Licht.

**Team und Schwerpunkte:**

| Wer | Schwerpunkt |
|---|---|
| Ben (Tr3nben) | FastAPI-Server, Web-App, Treiber, Lidar-Ortung, Heimfahrt, API-Doku |
| Marlon Wehr | Autonomes Fahren, Sensor-Check und Hardware-Analyse, Interaktion/KI |
| Magnus Luciano Schell | Repo-Pflege, ESP32-CSI (Branch `ESP32-CSI`) |

**Grundsatzentscheidung:** Wir verwenden kein ROS im Hauptsystem, sondern reines
Python mit einem FastAPI-Server als einzigem Ein- und Ausgang (siehe
`docs/entscheidungen.md`, E1/E2). ROS2 Foxy liegt zwar im Docker auf dem Pi,
dient aber nur noch als Vergleich.

---

## 2. Projekttagebuch

### 27.10.2026 – Grundgerüst (Ben)
- **Was:**
  - FastAPI-Server mit WebSocket-Protokoll, Web-App mit Fahrsteuerung und Kamerabild.
  - Watchdog (Totmannschaltung nach 0,5 s), Tempolimit.
  - Lidar-Ortung per Aufnahmeabgleich, Heimfahrt zum Startpunkt, Kollisionsvermeidung beim Fahren von Hand.
- **Warum:** ein Kanal für alles, und jeder kann ohne Roboter entwickeln (`--mock`).
- **Ergebnis:** Der Roboter ist per Browser steuerbar unter `http://10.42.0.62:8000`.
- **Probleme:**
  - Die Orbbec-Kamera lieferte zunächst nur ein Bild und musste auf MJPG gezwungen werden.
  - USB-Geräte meldeten sich zwischendurch ab. Ben hat deshalb ein automatisches Neuverbinden eingebaut.

### 27.10.2026 – Erster Autonomie-Versuch mit ROS2 (Marlon)
- **Was:** ein Lidar-Erkunder (`Marlon/autonomes_fahren/x3_explorer.py`) im
  ROS2-Docker-Container, gestartet mit einem Befehl über `x3.sh`.
- **Ergebnis:** Der Roboter fuhr, kollidierte aber.
- **Ursache:** Der Lidar war links/rechts gespiegelt und um −90° verdreht
  (Fix in Commit 59013a5: `--no-flip --front-offset -90`).
- **Probleme:**
  - Der Treiberknoten blieb nach USB-Resets hängen, ohne einen Fehler zu melden.
  - Die Startreihenfolge Lidar → Treiber war wichtig.
- **Lehre:** Die Einbaulage jedes Sensors wird zuerst mit einem einfachen Hand-Test geprüft.

### 28.10.2026 – Sensor-Check und Hardware-Analyse (Marlon)
Alle Ergebnisse stehen in [`sensorcheck_2026-10-28/ZUSAMMENFASSUNG.md`](sensorcheck_2026-10-28/ZUSAMMENFASSUNG.md),
die Rohdaten in `robot-protokolle/2026-10-28_sensorcheck/`.
- **USB-Resets – Ursache gefunden:**
  - Sie traten nicht wegen eines leeren Akkus auf, sondern bei **CPU-Lastsprüngen mit 2,4 GHz**.
  - Die Over-Current-Meldung kam auf allen Ports gleichzeitig. Vermutlich speist der Hub über das Expansion-Board zurück (Backfeed).
  - Bei 2,0 GHz waren die Resets in der Messung weg. Beim späteren Test traten aber vereinzelt doch Lidar-Resets auf, siehe unten.
- **Lidar:** vorne = Rohwinkel −90°. Die echte Scanrate liegt bei 7,5–7,8 Hz.
- **Gyro z:** großer Bias von +0,389 rad/s und falsches Vorzeichen. Er ist nur mit Korrektur verwendbar.
- **Odometrie:** Sie stimmt gut. Der Fahrbefehl allein überschätzt die Bewegung um 10–30 %.
- **Akku:** 10,1–10,3 V – bald laden.

### 28.10.2026 – Autonomes Fahren Test 1 in Bens Server (Marlon)
- **Was:**
  - Der Erkunder läuft jetzt direkt im FastAPI-Server statt in ROS2: `navigation/explorer.py`.
  - Er fährt zufällig, wählt freie Richtungen und benennt Hindernisse per Lidar-Form (Wand, Stuhl, Tisch, Mensch): `navigation/hindernisse.py`.
  - Personen erkennt er über die Kamera (OpenCV-HOG): `sensors/personen.py`.
  - Die Web-App hat dafür ein eigenes Panel, die Befehle heißen `explore` und `explore_stop`.
- **Warum:** ein System statt zwei. Dieselbe Sicherheit (Watchdog, Not-Stopp) gilt für alle Fahrarten.
- **Test ohne Roboter:** `python Marlon/autonomes_fahren/test_hindernisse.py` simuliert einen Raum. Alle Fälle bestehen.
- **Ergebnis:**
  - PR #3 ist auf main gemerged.
  - Am echten Roboter laufen Lidar, Kamera, Personenerkennung und Autonomie-Status.
  - Die Lidar-Ausrichtung passt zum Kamerabild.
- **Probleme:**
  - Beim ersten echten Start stand der Roboter **auf dem Tisch**. Wir haben abgebrochen, denn der Lidar sieht keine Kanten oder Abgründe. Ab jetzt läuft autonomes Fahren **nur am Boden**.
  - Auch bei 2,0 GHz gab es zwei Lidar-Resets (11:12 und 11:13 Uhr). Bens Code verbindet sich selbst neu. Die eigentliche Lösung bleibt aber in der Hardware: ein Hub ohne Rückspeisung.

### 28.10.2026 – Konzept „interaktiver Roboter“ (Marlon)
- **Bestandsaufnahme am Pi:**
  - Der KI-Chip ist ein **Hailo-10H** (40 TOPS, 8 GB eigener Speicher). Er kann Bildmodelle und kleine Sprachmodelle ausführen.
  - Mitgeliefert ist ein Pose-Modell (`yolov8s_pose_h10.hef`, 17 Körperpunkte).
  - Die Logitech C270 ist vorn montiert und leicht nach oben geneigt.
  - Zum Hören und Sprechen sind seit dem 28.10. ein USB-Mikrofon und ein USB-Lautsprecher montiert.
- **Entscheidungen:**
  - Gesten für Version 1: Folgen/Stopp und Herkommen/Weggehen.
  - Erkannt werden sie über die Körperpose auf dem Hailo statt über ein Handmodell. Das funktioniert auch auf 1–4 m Abstand und lässt die CPU frei.
  - Sprache: Er hört zu (Vosk, offline) und antwortet mit Stimme (Piper, offline) über den Lautsprecher, zusätzlich als Text in der App.
  - „Denken“ läuft **hybrid**: Claude-API, wenn Internet da ist, sonst lokal (später das LLM auf dem Hailo), notfalls Vorlagensätze.
  - **Das Sprachmodell steuert nie direkt.** Fahren läuft immer über Verhalten und Watchdog.
- **„Bewusstsein“:**
  - Eine Geist-Schicht mit innerem Zustand: Energie, Neugier, Stimmung, Aufmerksamkeit.
  - Dazu ein Gedächtnis der letzten Ereignisse und ein sichtbarer Gedankenstrom in der App.
  - Dazu kommen Ausdrucksbewegungen und die Stimme.
- **Plan in Stufen:**
  1. Doku
  2. Wahrnehmung
  3. Verhalten
  4. Sprache
  5. Geist und Denken
  6. Lokales LLM, Gesichter
- **Entwickler-Doku:** [`docs/interaktion.md`](../docs/interaktion.md)

### 28.10.2026 – Umbau: Mikrofon und Lautsprecher, Doku angelegt (Marlon)
- **Was:**
  - Ein USB-Mikrofon (C-Media) und ein USB-Lautsprecher („UACDemoV1.0“) sind montiert. Der Roboter bekommt deshalb schon in Version 1 eine Stimme (Piper, offline).
  - Die C270 war beim Umbau abgesteckt und kommt wieder an ihren Platz vorn.
  - Diese Projektdoku, `docs/interaktion.md` und die Entscheidungen E5/E6 sind angelegt.
- **Befund:**
  - Nach dem Neustart lief der Pi wieder mit 2,4 GHz, denn die Taktbegrenzung galt nur bis zum Neustart.
  - Kurz nach dem Booten zeigte `dmesg` bereits 61 over-current-Meldungen auf allen Bussen.
  - Die USB-Versorgung bleibt damit das größte Hardware-Risiko.

---

## 3. Ergebnisse und Messwerte

| Thema | Ergebnis | Quelle |
|---|---|---|
| USB-Resets | Auslöser sind CPU-Lastsprünge bei 2,4 GHz; mit 2,0 GHz deutlich seltener | Sensor-Check §1 |
| Lidar | vorne = −90° roh, 7,5–7,8 Hz, 1080 Punkte/Runde | Sensor-Check §2 |
| Gyro z | Bias +0,389 rad/s, Vorzeichen falsch | Sensor-Check §3 |
| Odometrie | 0,27 m gemessen bei 0,30 m Soll, seitwärts ~70 % | Sensor-Check §4 |
| Hinderniserkennung | Wand, Stuhl, Tisch und Mensch in der Simulation korrekt | `test_hindernisse.py` |
| Autonomes Fahren | startet und fährt am echten Roboter; Bodentest steht noch aus | 28.10. |

---

## 4. Probleme und Lösungen

| Problem | Ursache | Lösung / Stand |
|---|---|---|
| Geräte fallen vom USB ab | CPU-Lastsprung bei 2,4 GHz, Backfeed des Hubs | Takt auf 2,0 GHz (zur Laufzeit), Treiber verbinden sich selbst neu. **Offen:** Hub ohne Backfeed bzw. VBUS trennen, `arm_freq=2000` dauerhaft (mit Ben absprechen) |
| Kollisionen beim ersten Erkunder | Lidar gespiegelt und um 90° verdreht | `LIDAR_ANGLE_OFFSET_DEG = -90`, Hand-Test |
| Drehung ungenau | Gyro-Bias/-Vorzeichen, theta im Server ≈ 0 | Erkunder dreht nach Zeit; Gyro-Korrektur offen |
| Roboter auf dem Tisch | Lidar sieht keine Kanten | Autonomes Fahren nur am Boden |
| Push aus dem Laptop hängt | Git Credential Manager wartet auf Login-Fenster | Nutzer pusht selbst, der Pi zieht nur (`git pull`) |

---

## 5. Offene Punkte und Ausblick

- Bodentest des Erkunders mit Hindernissen.
- Hardware-Fix der USB-Versorgung.
- Interaktion, Stufen 2–5: Gesten, Folgen, Sprache, Geist.
- Später: lokales LLM auf dem Hailo, Gesichter wiedererkennen, Orbbec-Tiefe für niedrige Hindernisse.
- Akkuspannung im Server anzeigen.

---

## 6. Gliederung für die Präsentation

1. **Aufgabe:** Hilfsroboter, der sich bewegt und mit Menschen umgeht.
2. **Plattform:** X3, Pi 5, Hailo-10H, Lidar, Kameras (Foto mit Beschriftung).
3. **Architektur:** ein Server, ein Protokoll, Sicherheit ganz unten (Diagramm aus `docs/architektur.md`).
4. **Weg dorthin:** ROS2-Versuch → Probleme → eigener Python-Server (Begründung E1).
5. **Fehlersuche als Ingenieursarbeit:**
   - USB-Resets: Hypothesen, Messung, Ursache (Tabelle der Taktfrequenzen).
   - Lidar-Ausrichtung per Hand-Test.
6. **Autonomes Fahren:** Richtungswahl, Hindernisse benennen, Live-Demo bzw. Video.
7. **Interaktion:** Gesten → Verhalten, Gedankenstrom, „wirkt bewusst“ (Demo).
8. **Lessons Learned:**
   - zuerst messen, dann vermuten,
   - Sicherheit unten verankern,
   - autonom nur am Boden.
9. **Ausblick.**
