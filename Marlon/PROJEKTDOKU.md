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

### 28.10.2026 – Stresstest ohne Orbbec, mit vollem Akku (Marlon)
- **Anlass:** Prüfen, ob die ersten Hardware-Maßnahmen (Akku voll mit 12,0 V, Orbbec abgezogen) reichen.
- **Was:** 15-min-Test geplant: Lidar, Stimme und Fahren im Wechsel (Ansage, je 3 s links- und rechtsherum drehen mit omega ±0,6, je 2 s vor und zurück mit 0,1 m/s, 4 s Pause). Alle 10 s Over-Current-Zähler, USB-Abmeldungen (`journalctl`), Temperatur und Last mitgeschrieben. Die C270 war nicht angesteckt, **Hailo-Pose und Gesten sind deshalb nicht mitgetestet**.
- **Ergebnis:**
  - In Ruhe (nur Lidar, Stimme, Board) minutenlang **keine** neue Meldung.
  - Beim Fahren in 4,5 min **+160 Over-Current-Meldungen und 83 USB-Abmeldungen**, mit steigender Häufigkeit. Der Lautsprecher fiel zeitweise aus. Test nach 4,5 min abgebrochen, damit keine Dauerschleife entsteht.
  - **Jeder Reset begann 0–5 s nach dem Start einer Drehung**, also wenn alle vier Motoren gleichzeitig anlaufen. Geradeausfahren mit 0,1 m/s löste keinen aus.
  - 55–62 °C, Last ≤ 1,5, keine Drosselung. Hitze und Rechenlast sind diesmal also nicht die Ursache.
- **Schluss:** Der Anlaufstrom der Motoren zieht die 5-V-Versorgung herunter, über die auch der Pi bzw. der Hub laufen. Mit Software allein ist das nicht zu beheben. Es braucht die getrennte 5-V-Versorgung aus `docs/hardware.md` („Was am Roboter tun“, Punkt 4).
- **Dabei gefundene Fehler in der Software, behoben:**
  - **Fahrbefehle konnten an den Lidar gehen.** Nach einem Aussetzer meldete sich der Lidar eine Sekunde vor dem Board wieder an. Bens Treiber nahm dann „das einzige Gerät“ und verband sich mit dem Lidar. Jetzt wählt der Treiber zuerst `/dev/myserial` (udev-Regel über Hersteller- und Produktnummer) und schließt den Lidar immer aus. Am Pi geprüft: Jedes Neu-Verbinden lief über `/dev/myserial`.
  - **Strg+C beendete den Server nicht.** uvicorn wartete ewig darauf, dass der Kamera-Stream endet, der nie endet. Dadurch lief auch das Aufräumen (Motoren aus) nicht an. Jetzt werden offene Verbindungen nach 3 s gekappt (`SHUTDOWN_TIMEOUT_S`). Am Pi mit offenem Stream geprüft: beendet nach 4 s, Aufräumen vollständig.
- **Danach: Dauerschleife schon im Stand.** Nach dem Neustart des Servers (alter Stand) stieg der Zähler in 4 min von 5129 auf 5752, ohne dass der Roboter fuhr. Mit gestopptem Server stand der Zähler sofort still. Ursache: Nach jedem erfolgreichen Neu-Verbinden läuft der Lidar-Motor an, der Anlaufstrom löst den nächsten Einbruch aus, und die Wartezeit begann jedes Mal wieder bei 1 s. Der Server startete den Motor also alle paar Sekunden neu.
  - **Behoben:** Fällt der Lidar innerhalb von 60 s nach dem Neu-Verbinden wieder aus (`LIDAR_STABIL_S`), wächst die Wartezeit weiter (1 → 2 → 4 … 30 s), statt neu zu beginnen. Lokal mit einem Schein-Lidar geprüft.
  - Der Lidar-Port am Hub (1-1.4) meldete sich danach gar nicht mehr an. Dann hilft nur Lidar-Kabel neu stecken oder Roboter aus- und einschalten.

### 29.10.2026 – Gegenprobe: Labornetzteil statt Akku (Marlon)
- **Anlass:** Prüfen, ob die Resets beim Drehen am Akku liegen. Der Roboter hängt dafür an einem Labornetzteil statt am Akku. Die Orbbec war wieder angesteckt, die C270 nicht.
- **Aufbau:** Roboter **auf einem Podest**, damit er nicht wegrollt. Die Räder drehen frei in der Luft, die Motoren laufen also ohne Last.
- **Was:** Neuer Stand vom Laptop am Pi gestartet (mit Lidar-Fix `LIDAR_STABIL_S`), erst 2 min Ruhe beobachtet, dann derselbe Stresstest wie am 28.10. für 5 min (210 s Fahren, Drehungen mit omega ±0,6, vor/zurück mit 0,1 m/s).
- **Ergebnis:**
  - Ruhe: keine Meldung. Die 25 Over-Current-Meldungen seit dem Start entstanden alle beim Einschalten.
  - Fahren: **0 Over-Current-Meldungen, 0 USB-Abmeldungen** (Akku am 28.10.: +160 bzw. 83 in 4,5 min). Lidar ohne Neu-Verbinden (2333 Umdrehungen), 51 °C, keine Drosselung. Gefahren ist er wirklich: Die Odometrie hat sich bewegt, und die Lidar-Ortung hat 1277-mal korrigiert.
  - Beim Ein- und Ausstecken von C270 und Lautsprecher im laufenden Betrieb fielen kurz danach alle USB-Geräte für 3 s ab (+18 Meldungen). Der Server übersteht das. *Später geklärt:* Das Ereignis fiel mit dem Beenden des Servers zusammen, und die Ursache war ein Software-Fehler (Hailo-Start beim Beenden, siehe nächster Eintrag), nicht das Umstecken.
- **Schluss (zuerst):** Die Ursache sei die Versorgung aus dem Akku, das Netzteil halte die Spannung.
- **Korrigiert durch die Akku-Gegenprobe (nächster Eintrag):** Auf dem Podest gibt es auch mit Akku 0 Resets. Der Unterschied zum 28.10. ist also nicht Akku gegen Netzteil, sondern vor allem die **Last auf den Rädern**. Orbbec und Rechenlast scheiden weiterhin aus.

### 29.10.2026 – Gegenprobe: voller Akku auf dem Podest, Fehler beim Beenden behoben (Marlon)
- **Anlass:** Der Netzteiltest lief auf dem Podest. Damit war offen, ob das Netzteil hilft oder nur die fehlende Last. Deshalb derselbe Test mit Akku im selben Aufbau.
- **Aufbau:**
  - Roboter **auf dem Podest**, Räder frei in der Luft (wie beim Netzteiltest).
  - **Akku auf 100 % geladen**, gemessen über das Board (`get_battery_voltage`): 12,3 V vor dem Test, 12,2–12,3 V danach.
  - Orbbec **und** C270 angesteckt, Lautsprecher angesteckt. Der Server holte durchgehend Bilder von der C270.
  - Stand vom Laptop (mit `LIDAR_STABIL_S`), Hailo-Pose aus, weil die Interaktion nicht lief.
- **Was:** 2 min Ruhe beobachtet, dann derselbe Stresstest wie am 28.10. und beim Netzteil: 5 min, davon 210 s Fahren (Ansage, je 3 s Drehen mit omega ±0,6, je 2 s vor und zurück mit 0,1 m/s, 4 s Pause).
- **Ergebnis:**
  - Ruhe: keine neue Meldung.
  - Fahren: **0 Over-Current-Meldungen, 0 USB-Abmeldungen** (Zähler blieb bei 19 aus dem Einschalten).
  - Lidar 3335 Umdrehungen ohne Neu-Verbinden, Kamera 6566 Bilder ohne Neu-Öffnen, 50–53 °C, keine Drosselung (`throttled=0x0`).
- **Vergleich:**

  | Datum | Aufbau | Versorgung | Spannung | Over-Current / Abmeldungen beim Fahren |
  |---|---|---|---|---|
  | 28.10. | ohne Podest, Räder mit Last | Akku | 12,0 V | +160 / 83 in 4,5 min |
  | 29.10. | Podest, Räder frei | Labornetzteil | – | 0 / 0 in 210 s |
  | 29.10. | Podest, Räder frei | Akku | 12,3 V | 0 / 0 in 210 s |

- **Schluss:** Ohne Last auf den Rädern reicht auch der Akku. Die Resets entstehen, wenn die Motoren gegen Last anlaufen: Dann ist der Anlaufstrom viel höher als bei frei drehenden Rädern. Ob das Netzteil das unter Last abfängt, zeigt erst ein **Test am Boden**. Erst am Netzteil, dann am Akku. Weitere Unterschiede zum 28.10.: Der Lidar-Fix `LIDAR_STABIL_S` war da noch nicht drauf, und der Akku hatte 12,0 statt 12,3 V.
- **Fehler beim Beenden gefunden und behoben:**
  - **Beobachtung:** Beim Beenden des Servers fielen zweimal alle USB-Geräte für etwa 2 s ab (zuletzt +12 Meldungen um 08:15:06). Das Ereignis vom Netzteiltest, das wir aufs Umstecken geschoben hatten, passierte ebenfalls beim Beenden.
  - **Ursache:** `Posenerkennung.stop()` weckt den Pose-Thread auf, damit er sich beendet. Der Thread prüfte danach aber nicht, ob er aufhören soll, und öffnete erst noch den Hailo. Im HailoRT-Log steht „Creating vdevice“ um 08:15:06.890, in derselben Sekunde wie die Überlast. Das Hochfahren des Hailo zieht kurz viel Strom.
  - **Behoben** in `sensors/pose.py`: Nach dem Aufwecken bricht der Thread ab, wenn `stop()` ihn geweckt hat. Am Pi zweimal gestartet und beendet: Zähler 31 → 31, kein Hailo-Start mehr beim Beenden.
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

### 29.10.2026 – Sanftes Fahren und entprellte Gesten (Marlon)
- **Was:**
  - **Anfahrrampe im Watchdog:** Fahrbefehle springen nicht mehr auf die Motoren, sondern werden mit begrenzter Beschleunigung angefahren (0,8 m/s², Drehen 3 rad/s²) und schneller gebremst (2 m/s², 6 rad/s²). Ein Richtungswechsel bremst erst auf null. `stop()` (Not-Stopp, Stopp-Geste, Totmann) bleibt sofort. Einstellungen `RAMPE_*` in `config.py`, mit `RAMPE_AKTIV = False` abschaltbar.
  - **Folgen ruhiger:** Kamerawinkel geglättet, Totzone von 3°, weicher Übergang zwischen Drehen und Fahren statt hartem Umschalten, kleinerer Drehanteil (`FOLGEN_KP_W` 1,2). Ist die Person kurz weg, rollt der letzte Befehl über 0,5 s aus, statt auf null zu springen (mit Prüfung, ob vorn bzw. hinten frei ist). Fehlt der Lidar-Abstand in einem Takt, gilt kurz der letzte gute Wert. Der Zustand „Person sehr nah“ hat eine Hysterese.
  - **Gesten entprellt:** Herkommen und Weggehen zählen nur noch, wenn die Person stillsteht (Schultern wandern kaum, Größe ändert sich kaum), zur Kamera schaut und der Roboter selbst steht; dazu zweimal hintereinander bestätigt. Stopp und Folgen sind ausgenommen.
- **Warum:**
  - Beim Steuern per Geste und beim Folgen fiel der USB-Hub aus, und die Motoren liefen ruckelig. Beides hat dieselbe Ursache: harte Sprünge im Fahrbefehl (Person kurz nicht erkannt → sofort 0, wieder da → sofort volle Fahrt; zitternder Winkel → `omega` springt hin und her). Jede Stufe zieht eine Stromspitze aus dem Akku, die Spannung bricht ein, der Hub setzt zurück (siehe Sensor-Check und Diagnoseseite).
  - Beim Folgen wechselte der Roboter manchmal das Verhalten, ohne dass jemand ein Zeichen gegeben hatte: Wer läuft, schwingt die Arme, und das sah nach Herkommen/Weggehen aus. Fährt der Roboter, wackelt außerdem das ganze Bild.
- **Ergebnis (lokal, Mock):** `Marlon/interaktion/test_gesten.py` 69 Prüfungen bestanden (neu: Ausrollen, Ausrollen vor Hindernis, weicher Übergang, Totzone, Glättung, Hysterese, Laufen mit schwingenden Armen, Vorbeigehen, von hinten, fahrender Roboter, Rampe im Watchdog); `test_diagnose.py` 9/9; Smoke-Test 19/19.
- **Probleme und Lösungen:**
  - Der Gestentest stürzte schon vor der Änderung ab (`'Pose' object has no attribute 'fov_deg'`): Seit dem Kamerawechsel fragt das Verhalten die Posenerkennung nach dem Bildwinkel, die Test-Attrappe kannte ihn noch nicht. Attrappe ergänzt; der echte Code war in Ordnung.
  - Der Smoke-Test (Bens `scripts/smoke_test.py`) prüfte das Tempolimit, indem er auf `vx = 5,0` im Status wartete. Mit der Rampe dauert das gut 6 s. Angepasst: In der Simulation wartet er bis zu 10 s und fährt dabei im Kreis (geradeaus kämen 15 m zusammen und der Roboter stünde außerhalb des gedachten Raums); am echten Roboter prüft er nur 1 s lang, dass das Limit nie überschritten wird, statt Vollgas durch den Raum.
  - „Heimfahrt startet“ im Smoke-Test schlägt gelegentlich fehl, wenn derselbe Mock-Server mehrmals hintereinander getestet wird: Der simulierte Roboter fährt durch die gedachten Wände, die Ortung landet dann nahe am Startpunkt, und die Heimfahrt ist fertig, bevor der nächste Status sie zeigt. Zum Vergleich ohne Rampe: 3 von 5 Läufen fehlgeschlagen, mit Rampe 1 von 4. Das lag also schon vorher vor; mit frisch gestartetem Server läuft der Test durch.
- **Offen:** am Pi am Boden testen, ob Folgen ruhiger läuft und die USB-Resets beim Folgen ausbleiben (`dmesg -w` mitlaufen lassen, `/diagnose` beobachten). Falls das Anfahren zu träge wirkt: `RAMPE_ANFAHREN_XY` erhöhen.

### 29.10.2026 – Sprachbefehle „Robi“ und Antrieb-Abbruch beim Gestenstart behoben (Marlon)
- **Was:**
  - **Stand zusammengeführt:** Die noch nicht gepushten Commits vom Branch `interaktion` (Gesten, Stimme, Diagnose, sanftes Fahren) sind in den neuen Branch `sprache` eingeflossen, der auf dem aktuellen `main` aufsetzt.
  - **Hailo bleibt warm:** Die Posenerkennung öffnet den Hailo schon beim Serverstart und hält ihn offen (`POSE_WARM_HALTEN = True`). Gerechnet wird weiter nur, solange die Interaktion an ist.
  - **Antrieb weg ≠ Interaktion aus:** Fällt das Board weg, hält das Verhalten an und geht nach `bereit`, die Interaktion bleibt an. Fahrmodi (Folgen, Herkommen, Weggehen, Suchen) werden abgelehnt, solange `driver.connected` falsch ist; er sagt dann „Ich habe keine Verbindung zu meinen Rädern.“. Nach dem Neuverbinden geht es mit der nächsten Geste weiter.
  - **Sprachbefehle mit Weckwort „Robi“** (`interaktion/sprache.py`): USB-Mikrofon → Vosk (offline, kleines deutsches Modell) mit fester Grammatik. Befehle: folgen („Robi, folgen“, „folge mir“, „komm mit“), komm her, geh weg/zurück, fahr nach Hause, erkunde, Musik aus/Ruhe, „Hallo Robi“ (Interaktion an), „Robi, gute Nacht“ (alles aus). „Stopp“, „Halt“, „Bleib stehen“ gelten auch ohne Weckwort. Ausgeführt wird über dieselben Wege wie die App-Befehle. Neu: Befehl `speech {text}` (Text statt Mikrofon, auch im Mock), `state.sprache`, `GET /sprache/status`, Feld „Hören“ in der App. Abhängigkeiten als optionales Extra `pip install -e "robot[sprache]"`.
- **Warum:**
  - Beim Einschalten der Gestensteuerung kam oft „Keine Verbindung zum Antrieb“, und die Interaktion war sofort wieder aus. Ursache 1: Erst beim Einschalten wurde der Hailo hochgefahren, das zieht kurz viel Strom, und am Hub meldet sich das Board ab (gleiches Muster wie beim Beenden des Servers). Ursache 2: Der Server beendete bei jedem Antriebsverlust den ganzen Interaktionsmodus, statt nur anzuhalten.
  - Sprachbefehle sind die zweite Eingabe neben den Gesten (Stufe 4 im Konzept). Das Weckwort verhindert, dass Gespräche im Raum den Roboter losfahren lassen; Stopp braucht es nicht, damit Anhalten immer geht.
- **Ergebnis (lokal, Mock):** `test_sprache.py` 41 Prüfungen (Weckwort, Weckfenster, Rangfolge, Grammatik gegen den Wortschatz des Modells, kein Absturz ohne Vosk/Mikrofon); `test_gesten.py` 76 (neu: Fahrmodi ohne Antrieb abgelehnt, `antrieb_verloren` lässt die Interaktion an); `test_stimme.py` 17; `test_diagnose.py` 9/9. `/sprache/status` und `speech` im Mock geprüft.
- **Probleme und Lösungen:**
  - „Musik stopp“ wurde als Stopp erkannt: Die Befehle haben jetzt eine Rangfolge, Musik aus steht vor Stopp.
  - Wörter, die das Vosk-Modell nicht kennt, würden die Grammatik ungültig machen. Sie werden gegen `graph/words.txt` gefiltert; für „Robi“ gibt es Varianten („robbie“, „ruby“ …).
  - Musik (mpv) und Stimme (aplay) teilen sich den Lautsprecher exklusiv, und das Mikrofon hört die Musik mit. „Robi, Musik aus“ hilft; eine saubere Lösung (gemeinsamer Mixer) ist offen.
  - Smoke-Test „Heimfahrt startet“ wackelt weiter wie am 29.10. beschrieben (wiederholte Läufe gegen denselben Mock-Server); nicht durch diese Änderung.
- **Nachtrag – „Sanft fahren“ in den Einstellungen:** Die Anfahrrampe lässt sich jetzt in der App unter Einstellungen → „Fahren & Motoren“ abschalten („Sanft fahren“) und einstellen (Anfahren, Bremsen, Andrehen, Drehung stoppen). Bisher ging das nur in `config.py` mit Neustart. Der Watchdog liest die Werte bei jedem Takt, die Änderung wirkt also sofort und übersteht über `robot/settings.json` einen Neustart. Geprüft: Sprung auf 0,3 m/s kommt mit Rampe als 0,08 an, ohne sofort als 0,3; Werte außerhalb der Grenzen werden abgelehnt.
- **Offen:** Am Pi Vosk und Modell installieren (`docs/hardware.md`, „Mikrofon und Sprachbefehle“), `/sprache/status` prüfen, „Robi, folgen“ am Boden testen. Beim Einschalten der Gesten `dmesg -w` mitlaufen lassen: Bleibt der Reset aus?

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
| Stresstest (Akku voll, ohne Orbbec) | Ruhe: 0 Resets; Fahren: +160 Over-Current in 4,5 min, jeder 0–5 s nach Beginn einer Drehung | 28.10. |
| Stresstest (Labornetzteil, mit Orbbec, **auf Podest**) | Ruhe und Fahren (210 s, gleiche Drehungen): 0 Over-Current, 0 Abmeldungen | 29.10. |
| Stresstest (Akku 100 %/12,3 V, Orbbec und C270, **auf Podest**) | Ruhe und Fahren (210 s): 0 Over-Current, 0 Abmeldungen, Lidar und Kamera ohne Neu-Verbinden | 29.10. |
| Sprachausgabe | Warteschlange, Vorrang/Verstummen, Ausweichen ohne Piper, Ansagen des Verhaltens: 17 Prüfungen bestanden; am Lautsprecher noch nicht gehört | `Marlon/interaktion/test_stimme.py` |
| Sanftes Fahren und entprellte Gesten | Rampe, Ausrollen, Glättung, Laufen/Vorbeigehen ohne Fehlauslösung: 69 Prüfungen bestanden (29.10.) | `Marlon/interaktion/test_gesten.py` |
| Sprachbefehle „Robi“ | Weckwort, Weckfenster, Rangfolge, Grammatik, Ausfallsicherheit: 41 Prüfungen; Gesten inkl. Antriebsverlust: 76 Prüfungen (29.10.); am Mikrofon noch nicht getestet | `Marlon/interaktion/test_sprache.py` |

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
| Resets beim Drehen, trotz vollem Akku und ohne Orbbec | Anlaufstrom aller vier Motoren unter Last zieht die Versorgung herunter. Auf dem Podest (Räder frei) 0 Resets, mit Akku wie mit Netzteil (29.10.) | **Offen:** Test am Boden (Netzteil, dann Akku); falls Resets bleiben, getrennte 5-V-Versorgung (DC/DC) für Pi bzw. Hub, siehe `docs/hardware.md` |
| Alle USB-Geräte fallen beim Beenden des Servers kurz ab | Pose-Thread öffnete beim Beenden noch den Hailo, dessen Hochfahren kurz viel Strom zieht | `_stop`-Prüfung in `sensors/pose.py`; am Pi zweimal geprüft, 0 Meldungen (29.10.) |
| Treiber verband sich mit dem Lidar | Nach einem Aussetzer war der Lidar kurz das einzige serielle Gerät | Board über `/dev/myserial`, Lidar immer ausgeschlossen (28.10.) |
| USB-Dauerschleife auch im Stand | Lidar-Motor lief nach jedem Neu-Verbinden sofort wieder an, Wartezeit begann bei 1 s | Wartezeit wächst weiter, wenn er kurz danach wieder ausfällt (`LIDAR_STABIL_S`, 28.10.) |
| Strg+C beendete den Server nicht | uvicorn wartete auf den endlosen Kamera-Stream, Motoren-Stopp lief nicht an | `SHUTDOWN_TIMEOUT_S = 3` (28.10.) |
| USB-Resets und Ruckeln beim Folgen | harte Sprünge im Fahrbefehl → Stromspitzen → Spannungseinbruch | Anfahrrampe im Watchdog, geglättetes Folgen mit Ausrollen (29.10.). **Offen:** Bodentest am Pi |
| Verhalten wechselt beim Folgen ohne Zeichen | schwingende Arme beim Laufen sehen wie Herkommen/Weggehen aus | Herkommen/Weggehen nur bei stillstehender, zugewandter Person und stehendem Roboter (29.10.) |
| „Keine Verbindung zum Antrieb“ beim Start der Gestensteuerung, Interaktion sofort aus | Hailo fuhr erst beim Einschalten hoch (Stromspitze → USB-Reset), und jeder Antriebsverlust beendete die ganze Interaktion | Hailo beim Serverstart öffnen und warm halten (`POSE_WARM_HALTEN`); bei Antriebsverlust nur anhalten, Fahrmodi ohne Board ablehnen (29.10.). **Offen:** am Pi prüfen |
| „Musik stopp“ als Stopp erkannt | Stopp-Wendung steckt im Musikbefehl | Rangfolge der Sprachbefehle, Musik aus zuerst (29.10.) |
| Ausfälle nur per SSH auffindbar | Kernel-Log, vcgencmd und Serverlog getrennt | Diagnoseseite `/diagnose` mit Problemen, Zeitleiste und Log (29.10.) |
| Push aus dem Laptop hängt | Git Credential Manager wartet auf Login-Fenster | Nutzer pusht selbst, der Pi zieht nur (`git pull`) |
| MUKKE: mpv meldet „Failed to recognize file format“ | mpvs ytdl-Skript findet yt-dlp in der venv nicht und gibt den YouTube-Link roh weiter | Server löst den Link selbst mit der yt-dlp-Bibliothek auf, mpv bekommt nur noch die direkte Tonspur; YouTube-Fehler erscheinen im Fenster |

---

## 5. Offene Punkte und Ausblick

- Bodentest des Erkunders mit Hindernissen.
- **Stresstest am Boden** (Räder mit Last): erst am Netzteil, dann am Akku. Auf dem Podest gibt es mit beiden 0 Resets (29.10.), deshalb ist offen, ob das Netzteil unter Last hilft. Bleiben am Boden Resets, braucht es die getrennte 5-V-Versorgung für Pi bzw. Hub (siehe `docs/hardware.md`, „Was am Roboter tun“).
- Stresstest mit eingeschalteter Interaktion (Hailo-Pose und Gesten unter Fahrlast) wiederholen.
- Interaktion: Hailo-Pose am Pi prüfen, Fahrtest Folgen/Herkommen am Boden; danach Stufen 4–5 (Sprache, Geist).
- Stimme: Piper am Pi installieren, Lautsprecher und Lautstärke prüfen.
- Sprachbefehle am Pi: `libportaudio2`, `pip install -e "robot[sprache]"` und Vosk-Modell installieren, `/sprache/status` prüfen, „Robi, folgen“ am Boden testen; erkennt Vosk „Robi“ schlecht, Varianten in `SPRACHE_WECKWORT_VARIANTEN` ergänzen.
- Lautsprecher teilen: MUKKE und Stimme blockieren sich gegenseitig (exklusives ALSA-Gerät); gemeinsamer Mixer (dmix/PipeWire) offen.
- Prüfen, ob der Antrieb beim Einschalten der Gesten jetzt verbunden bleibt (warmer Hailo).
- Später: lokales LLM auf dem Hailo, Gesichter wiedererkennen, Orbbec-Tiefe für niedrige Hindernisse.
- Diagnoseseite am Pi prüfen (Kernel-Log-Berechtigung, echte Akku- und Überlastwerte).
- Sanftes Fahren am Boden prüfen: Folgen ohne USB-Resets, Rampe nicht zu träge.

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
