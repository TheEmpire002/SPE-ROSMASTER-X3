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

### 28.10.2026 – Sprachausgabe über den USB-Lautsprecher (Marlon)
- **Was:**
  - `interaktion/stimme.py`: Der Roboter spricht über den USB-Lautsprecher (ALSA „UACDemoV10“). Kette: Piper (offline, Stimme `de_DE-thorsten-medium`) → `aplay`. Fehlt Piper, spricht espeak-ng, sonst wird nur geloggt.
  - Das Verhalten sagt an, was es tut: „Ich folge dir.“, „Ich komme.“, „Da bin ich.“, „Wo bist du?“, „Da bist du ja.“, „Okay, ich halte an.“. Beim Not-Stopp kommt „Halt.“, beim Serverstart „Ich bin bereit.“ (hörbarer Selbsttest).
  - Server: neue Befehle `say {text, vorrang}` und `say_stop`, Feld `state.stimme`, Endpunkt `/stimme/status`. Einstellungen `STIMME_*` in `config.py`.
  - Web-App: Anzeige „Stimme“ (was er gerade sagt) und ein Feld „Sagen“ im Interaktionsbereich.
  - Doku: `docs/api.md`, `interaktion.md`, `hardware.md` (Einrichtung von Piper), `architektur.md`, neue Entscheidung E7.
- **Warum so:**
  - Offline, damit Ansagen und „Halt.“ auch ohne Internet gehen, wie die Reflexe (E6).
  - Ein eigener Thread mit Warteschlange: Ein Satz dauert 1–2 s, so lange dürfen Server und Fahrschleife nicht warten.
  - Piper und aplay laufen als eigene Prozesse. Stürzt einer ab oder fehlt der Lautsprecher, läuft der Server weiter, und ein Stopp kann den laufenden Satz sofort abbrechen.
  - Der Lautsprecher wird über seinen Namen statt über die Kartennummer angesprochen und übersteht damit USB-Resets.
  - Gleiche Sätze innerhalb von 4 s werden verschluckt, die Warteschlange hält höchstens 3. Sonst redet er beim Folgen ununterbrochen „Wo bist du? Da bist du ja.“
- **Problem und Lösung:**
  - Wer im neuen Textfeld tippte, hätte den Roboter gesteuert: Die Web-App fährt mit den Pfeil-/WASD-Tasten, und die Leertaste ist der Not-Stopp. Lösung: Tastatursteuerung ignoriert Tasten, solange ein Textfeld den Fokus hat.
  - Ohne Lautsprecher meldete die Stimme `spricht = true`, obwohl nichts zu hören war. Später hätte das die Spracherkennung grundlos taub geschaltet. Jetzt gilt `spricht` nur, wenn wirklich Ton herauskommt.
- **Ergebnis (lokal, ohne Lautsprecher):**
  - `Marlon/interaktion/test_stimme.py`: 17 Prüfungen bestanden (Warteschlange, Vorrang, Ausweichen auf Text, Sätze bei Gesten). `test_gesten.py` weiter bestanden.
  - Smoke-Test des Servers im Mock: „Ich bin bereit.“ beim Start, `say`, Begrüßung bei „Interaktion an“, Ansagen zu den Gesten, „Halt.“ beim Stopp.
- **Offen:** am Pi Piper und das Modell installieren (Anleitung in `docs/hardware.md`). Dann prüfen, ob man den Lautsprecher hört, wie lang die Verzögerung bis zum Ton ist und ob die Lautstärke passt.

### 28.10.2026 – Gesamtprüfung am Pi, Kamera fest auf die C270 (Marlon)
- **Was:** Alles am Pi geprüft, mit `robot-protokolle/` als Referenz. Dazu ein Test der Bildkette Kamera → Hailo-Pose ohne Server und ohne Antrieb.
- **Befund vorheriger Start:**
  - Die C270 fehlte, nur die Orbbec war da.
  - In 9 Minuten 299 Over-Current-Meldungen. Board, Lidar und Hub fielen immer wieder ab, und der Server bekam kein Bild.
  - Nach dem Neustart mit wieder eingesteckter C270 war das weg.
- **Befund jetzt:**
  - Takt 2,0 GHz aktiv, nicht gedrosselt, ~51 °C.
  - Alle Geräte da: C270 (`video0`/`video1`), Orbbec RGB (`video2`), Board (CH340), Lidar (CP2102, `/dev/rplidar`), Mikrofon FongLun, Lautsprecher UACDemoV10.
  - Over-Current nur beim Hochfahren, plus einmal bei s 241 auf allen Ports gleichzeitig. Das passt zum Backfeed des Hubs.
  - Danach haben `ttyUSB0`/`ttyUSB1` die Plätze getauscht. Der Treiber findet das Board trotzdem, weil er über `/dev/serial/by-id` sucht und den Lidar über `/dev/rplidar` ausschließt.
  - espeak-ng ist installiert, Piper fehlt noch. Die Stimme spricht also vorerst mit der Ersatzstimme.
- **Ergebnis Pose-Test:**
  - Die C270 wurde automatisch gewählt (V4L2, MJPG 640×480).
  - Hailo bereit, stabil **10 fps** (393 Bilder in 40 s), 1–2 Personen erkannt.
  - **Keine einzige neue Over-Current-Meldung unter Hailo-Last.**
- **Problem und Lösung:**
  - Die Kamera wurde bisher nur über den Index gewählt (`CAMERA_INDEX = 0`). Meldet sich nach einem Reset die Orbbec zuerst an, wäre sie `video0`, und die Gesten würden in der falschen Kamera gesucht.
  - Neu: `CAMERA_NAME = "C270"` in `config.py`. Die Kamera mit diesem Namen wird zuerst probiert. Fehlt sie, gilt die alte Reihenfolge.

### 28.10.2026 – „Alles stürzt ab“: Ursache NVIDIA-Build, Pakete entfernt (Marlon)
- **Anlass:** Kamera, Motor und Lidar fielen im Team immer wieder aus.
- **Befund (Uptime ~21 min):**
  - `lsusb` zeigte nur noch die Root-Hubs. Seit s 1234 kamen dauernd Over-Current-Meldungen, am Ende ~4900 Zählerstände.
  - 83 °C, Drosselung (`throttled=0xe0008`), Last ~6,7.
  - Ursache war wieder `sudo dpkg --configure -a` aus einer Laptop-Sitzung. Ein `apt install mpg123` war um 13:32 gescheitert, weil dpkg noch das nvidia-Paket konfigurieren wollte, und danach baute `dkms` das nvidia-Modul mit `make -j4`. Das passiert bei jedem `apt`-Aufruf wieder.
  - **Die Schleife hielt sich selbst:** Jedes Neu-Anmelden der Geräte (Orbbec, Lidar-Motor) löste die nächste Überlast aus. Die Hub-Ports meldeten `cannot reset (err -71)`. Diesmal fiel auch die C270 ab, obwohl sie direkt am Pi steckt.
- **Was:**
  - Build beendet. `nvidia-kernel-dkms` mit `dpkg --purge` entfernt, sodass er nicht noch einmal anläuft. Danach `dpkg --configure -a` und alle 44 nvidia/cuda-Pakete gelöscht (`apt-get purge`). Vorher mit `apt-get -s` geprüft: kein Hailo-Paket betroffen, `hailo1x_pci` bleibt in DKMS, `/dev/hailo0` ist da.
  - `mpg123` installiert, `dpkg --audit` ist sauber.
  - `scripts/check_hardware.py` hat jetzt den Abschnitt „USB-Versorgung“. Er zeigt, ob gerade neue Over-Current-Meldungen kommen und ob der Pi im Hintergrund installiert oder kompiliert.
- **Ergebnis:**
  - Etwa 2 min nach dem Ende des Builds hörte die Schleife von selbst auf, ein Neustart war nicht nötig.
  - Alle Geräte sind zurück (Hub, Board, Lidar, Orbbec, C270, Mikrofon, Lautsprecher), die Symlinks stimmen.
  - Danach 0 neue Over-Current-Meldungen, 61 °C.
- **Neue Erkenntnis:** Beim Pi 5 hängen alle vier USB-Buchsen an **einem** gemeinsamen Stromschalter (1,6 A gesamt). Dass „alle Ports gleichzeitig“ melden, beweist deshalb noch keine Rückspeisung. Es zeigt nur, dass die Summe aller Geräte die Grenze überschritten hat. Empfehlungen zur Hardware in `docs/hardware.md`, Abschnitt „USB-Resets“.

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
| Hailo-Pose am Pi | C270, 10 fps, 1–2 Personen erkannt, keine Over-Current-Meldung unter Last | Gesamtprüfung 28.10. |
| Sprachausgabe | Warteschlange, Vorrang/Verstummen, Ausweichen ohne Piper, Ansagen des Verhaltens: 17 Prüfungen bestanden; am Lautsprecher noch nicht gehört | `Marlon/interaktion/test_stimme.py` |

---

## 4. Probleme und Lösungen

| Problem | Ursache | Lösung / Stand |
|---|---|---|
| Geräte fallen vom USB ab | CPU-Lastsprung bei 2,4 GHz, Backfeed des Hubs | `arm_freq=2000` dauerhaft in config.txt (28.10.), Treiber verbinden sich selbst neu. **Offen:** Hub ohne Backfeed bzw. VBUS trennen |
| Kollisionen beim ersten Erkunder | Lidar gespiegelt und um 90° verdreht | `LIDAR_ANGLE_OFFSET_DEG = -90`, Hand-Test |
| Drehung ungenau | Gyro-Bias/-Vorzeichen, theta im Server ≈ 0 | Erkunder dreht nach Zeit; Gyro-Korrektur offen |
| Roboter auf dem Tisch | Lidar sieht keine Kanten | Autonomes Fahren nur am Boden |
| USB-Ausfälle bei Hitze | Pi über 80 °C (Kompilieren), Drosselung auf 1,5 GHz | Während Builds nicht fahren; `check_hardware.py` zeigt Temperatur und Drosselung |
| Alles stürzt dauernd ab (Kamera, Motor, Lidar) | Hängender nvidia-DKMS-Build, der bei jedem `apt` neu startete → Hitze → USB-Überlast als Dauerschleife | nvidia-Pakete entfernt (28.10.), `check_hardware.py` warnt bei laufenden Builds und neuen Over-Current-Meldungen |
| Winken löste „Weggehen“ aus | Schulterbreite bricht beim seitlichen Drehen ein, Maße in Schulterbreiten werden riesig | Median der Schulterbreite über das Fenster (28.10.) |
| Tippen im „Sagen“-Feld hätte den Roboter gefahren | Tastatursteuerung der App reagiert auf alle Tasten, Leertaste = Not-Stopp | Tasten in Textfeldern werden ignoriert (28.10.) |
| Falsche Kamera möglich | `/dev/video*`-Nummern tauschen nach USB-Resets, Orbbec könnte `video0` werden | Kamera über den Namen wählen, `CAMERA_NAME = "C270"` (28.10.) |
| Push aus dem Laptop hängt | Git Credential Manager wartet auf Login-Fenster | Nutzer pusht selbst, der Pi zieht nur (`git pull`) |

---

## 5. Offene Punkte und Ausblick

- Bodentest des Erkunders mit Hindernissen.
- Hardware-Fix der USB-Versorgung: Akku laden, Orbbec abziehen solange ungenutzt, Hub eigenständig versorgen (siehe `docs/hardware.md`, „Was am Roboter tun“).
- Interaktion: Hailo-Pose am Pi prüfen, Fahrtest Folgen/Herkommen am Boden; danach Stufen 4–5 (Sprache, Geist).
- Stimme: Piper am Pi installieren, Lautsprecher und Lautstärke prüfen. Bei der Spracherkennung (Vosk) `Stimme.spricht` nutzen, damit er sich nicht selbst zuhört.
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
