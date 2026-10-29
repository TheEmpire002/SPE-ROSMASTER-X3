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

### 28.10.2026 – Takt dauerhaft auf 2,0 GHz, Überhitzung (Marlon, mit Ben abgesprochen)
- **Was:**
  - In `/boot/firmware/config.txt` steht jetzt `arm_freq=2000`, mit Kommentar und Verweis auf `docs/hardware.md`.
  - Die Sicherung heißt `config.txt.vor-arm_freq-2026-10-28`. Rückgängig machen: die Zeile löschen.
- **Warum:** Die Begrenzung zur Laufzeit ging bei jedem Neustart verloren (siehe Eintrag davor).
- **Anpassung am Code:**
  - Bens Server braucht keine Änderung, weil seine Schleifen nach Uhrzeit takten und nicht nach Rechenzeit.
  - `scripts/check_hardware.py` zeigt jetzt CPU-Takt, Temperatur und Drosselung an. Es warnt, wenn die Begrenzung fehlt, z. B. nach einem neuen Image.
- **Befund:**
  - Eine andere Sitzung hatte `sudo dpkg --configure -a` gestartet. Das baute ein nvidia-Treibermodul (DKMS), das auf dem Pi sinnlos ist.
  - Der Pi lief dadurch mit 84 °C und drosselte auf 1,5 GHz (`throttled=0xe0006`). Board und Lidar fielen dabei dauernd vom USB.
  - Lehre: Nicht fahren und nicht testen, solange der Pi kompiliert. Die USB-Resets hängen auch an der Temperatur, nicht nur am Takt.

### 28.10.2026 – Interaktion Stufe 2 und 3: Handzeichen und Verhalten (Marlon)
- **Was:**
  - `sensors/pose.py`: Körperpose mit YOLOv8-Pose auf dem Hailo-10H, 10 Bilder/s, 17 Körperpunkte pro Person. Der Hailo ist nur aktiv, solange die Interaktion läuft, und wird nach Fehlern neu geöffnet. Im Mock spielt eine simulierte Person die Gesten vor.
  - `interaktion/gesten.py`: erkennt Folgen (Arm über dem Kopf), Stopp (beide Hände über den Schultern), Herkommen (winken) und Weggehen (seitlich wischen).
  - `interaktion/verhalten.py`: Folgen auf 1,0 m, Herkommen bis 0,7 m, Weggehen 1 m rückwärts, Stopp. Richtung aus der Kamera, Abstand aus dem Lidar.
  - Server und Web-App: Befehle `interaction_start/stop`, `follow/follow_stop`, Feld `state.interaktion`, Endpunkte `/interaktion/status` und `/interaktion/pose`. Neues Panel „Interaktion“; das Skelett wird über das Kamerabild gezeichnet.
- **Warum so:**
  - Gesten über die Körperpose statt über ein Handmodell (E5): Der Hailo rechnet sie, die CPU bleibt frei, und Arme sieht man auch aus 2–3 m Entfernung.
  - Die Verdrahtung ist eine Kopie des Erkunder-Musters. Es lenkt immer nur ein Programm, und jede Handeingabe bricht ab.
- **Problem und Lösung:**
  - Im ersten Test löste Winken auch „Weggehen“ aus. Ursache: Alle Maße sind in Schulterbreiten gerechnet. Dreht sich die Person kurz seitlich, wird die Schulterbreite in einem Bild winzig, und eine kleine Handbewegung wirkt riesig.
  - Lösung: den Median der Schulterbreite über das Zeitfenster nehmen.
  - Der „Arm waagerecht nach vorn“ als Stopp ist entfallen. In 2D sieht er fast so aus wie ein hängender Arm.
- **Ergebnis (lokal, Mock):**
  - `test_gesten.py` und `test_hindernisse.py`: alles bestanden. Smoke-Test: 19 von 19.
  - Ablauf im Mock: Herkommen → „bin da“, Weggehen → „Abstand genommen“, Folgen, Stopp. Abbruch durch Handsteuerung und durch „Autonom fahren“ funktioniert.
- **Offen:** am Pi prüfen, ob der Hailo die Pose liefert (`/interaktion/status`, fps). Danach der Fahrtest **am Boden**.

### 29.10.2026 – Diagnoseseite: alle Ausfälle an einer Stelle (Marlon)
- **Was:**
  - Neue Seite `/diagnose` (auch `/logs`), erreichbar über den Knopf „Diagnose“ oben auf der Hauptseite; von dort führt „Steuerung“ zurück. Ein Punkt am Knopf zeigt schon auf der Hauptseite, wenn etwas nicht stimmt (gelb = Warnung, rot blinkend = kritisch).
  - `diagnose.py` sammelt im Hintergrund: Akkuspannung (neu: `read_battery()` im Treiber) mit Verlauf und Spannungseinbrüchen, USB-Überlast-Zähler des Pi, Unterspannung und Drosselung (`vcgencmd get_throttled`), Temperatur und Takt, USB-Abmeldungen aus dem Kernel-Log, Zustand von Antrieb, Lidar, Kameras und Hailo, Fehler aus dem Server-Log, hängende Build-Prozesse (`dpkg`, `cc1` …).
  - Daraus entstehen **Probleme in Alltagssprache**: was ist passiert, was bedeutet es, „Was tun?“ als Schritte. Probleme lassen sich als gesehen markieren; tritt eins danach erneut auf, erscheint es wieder.
  - Dazu eine Zeitleiste (was ist wann passiert – zum Erkennen von Zusammenhängen wie „Spannungseinbruch beim Fahren → USB-Abmeldung“), das Server-Log mit Filter und Suche und „Bericht kopieren“ (alles als Text für Chat oder Protokoll).
  - Die Warnschwellen (Akku, Spannungseinbruch, Temperatur) stehen auf der Einstellungsseite im Block „Diagnose & Warnschwellen“ und wirken sofort.
- **Warum:** Bei jedem Aussetzer mussten wir per SSH in `dmesg`, `vcgencmd` und die Serverausgabe schauen und die Zeiten selbst zusammensuchen (siehe Akku- und Netzteiltests). Jetzt sieht jeder im Team auf einen Blick, was los ist und was zu tun ist.
- **Probleme und Lösungen:**
  - Direkt nach dem Start meldete die Seite Lidar und Kamera als ausgefallen, weil sie noch hochfuhren. Lösung: 15 s Anlaufzeit (`DIAGNOSE_ANLAUF_S`).
  - Nach einem Server-Neustart hätte der Browser keine neuen Log-Zeilen mehr geholt (Zeilennummern beginnen wieder bei 1). Lösung: Der Server meldet immer seine letzte Nummer; ist sie kleiner als die bekannte, lädt der Browser neu.
- **Ergebnis (lokal, Mock):** `Marlon/diagnose/test_diagnose.py` 9 von 9 bestanden (Akku, Quittieren, Vorfälle, Kernel-Log, Unterspannung, Log-Puffer); Smoke-Test 19 von 19; alle Seiten antworten.
- **Zusammenführen mit main:** Ben hatte parallel die Musikfunktion „MUKKE“ eingebaut, an denselben Stellen (Server-Endpunkte, `config.py`, Knopf oben links, CSS, `docs/api.md`). Die Konflikte waren nur „beide haben etwas angehängt“; beide Teile bleiben erhalten, die Knöpfe Diagnose und MUKKE stehen nebeneinander. Danach erneut: Tests 9/9, Smoke-Test 19/19, `/diagnose` und `/mukke/status` antworten.
- **Offen:** am Pi prüfen, ob Kernel-Log, Überlast-Zähler und `vcgencmd` gelesen werden. Für das Kernel-Log muss `pi` in der Gruppe `systemd-journal` sein (`sudo usermod -aG systemd-journal pi`, danach neu anmelden).

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
| Gestenerkennung und Verhalten | 4 Gesten, Negativfälle (Zittern, Hallo-Winken, Flackern, zwei Personen), Hailo-Dekodierung und Fahrlogik: 45 Prüfungen bestanden | `Marlon/interaktion/test_gesten.py` |

---

## 4. Probleme und Lösungen

| Problem | Ursache | Lösung / Stand |
|---|---|---|
| Geräte fallen vom USB ab | CPU-Lastsprung bei 2,4 GHz, Backfeed des Hubs | `arm_freq=2000` dauerhaft in config.txt (28.10.), Treiber verbinden sich selbst neu. **Offen:** Hub ohne Backfeed bzw. VBUS trennen |
| Kollisionen beim ersten Erkunder | Lidar gespiegelt und um 90° verdreht | `LIDAR_ANGLE_OFFSET_DEG = -90`, Hand-Test |
| Drehung ungenau | Gyro-Bias/-Vorzeichen, theta im Server ≈ 0 | Erkunder dreht nach Zeit; Gyro-Korrektur offen |
| Roboter auf dem Tisch | Lidar sieht keine Kanten | Autonomes Fahren nur am Boden |
| USB-Ausfälle bei Hitze | Pi über 80 °C (Kompilieren), Drosselung auf 1,5 GHz | Während Builds nicht fahren; `check_hardware.py` zeigt Temperatur und Drosselung |
| Winken löste „Weggehen“ aus | Schulterbreite bricht beim seitlichen Drehen ein, Maße in Schulterbreiten werden riesig | Median der Schulterbreite über das Fenster (28.10.) |
| Ausfälle nur per SSH auffindbar | Kernel-Log, vcgencmd und Serverlog getrennt | Diagnoseseite `/diagnose` mit Problemen, Zeitleiste und Log (29.10.) |
| Push aus dem Laptop hängt | Git Credential Manager wartet auf Login-Fenster | Nutzer pusht selbst, der Pi zieht nur (`git pull`) |
| MUKKE: mpv meldet „Failed to recognize file format“ | mpvs ytdl-Skript findet yt-dlp in der venv nicht und gibt den YouTube-Link roh weiter | Server löst den Link selbst mit der yt-dlp-Bibliothek auf, mpv bekommt nur noch die direkte Tonspur; YouTube-Fehler erscheinen im Fenster |

---

## 5. Offene Punkte und Ausblick

- Bodentest des Erkunders mit Hindernissen.
- Hardware-Fix der USB-Versorgung.
- Interaktion: Hailo-Pose am Pi prüfen, Fahrtest Folgen/Herkommen am Boden; danach Stufen 4–5 (Sprache, Geist).
- Später: lokales LLM auf dem Hailo, Gesichter wiedererkennen, Orbbec-Tiefe für niedrige Hindernisse.
- Diagnoseseite am Pi prüfen (Kernel-Log-Berechtigung, echte Akku- und Überlastwerte).

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
