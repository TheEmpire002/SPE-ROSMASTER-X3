"""Zentrale Einstellungen.

Alle Werte, die man beim Ausprobieren verstellen will, stehen hier -- nicht
verstreut im Code. Wer am Fahrverhalten schraubt, muss nur diese Datei öffnen.
"""

from pathlib import Path

# --- Serielle Verbindung zum Expansion Board ---------------------------------

# Je nach Image heißt die Schnittstelle anders. Welche bei euch stimmt, zeigt
# `python scripts/check_hardware.py`. None = Rosmaster_Lib sucht selbst.
SERIAL_PORT: str | None = None

# Rückkanal vom Expansion Board (Akkuspannung, IMU, Encoder).
#
# Dafür startet Rosmaster_Lib einen Thread, der dauerhaft vom seriellen Port
# liest. Liest ein zweites Programm am selben Port mit, stirbt dieser Thread
# mit "device reports readiness to read but returned no data". Fahren
# funktioniert weiterhin -- nur die Messwerte fehlen.
#
# Wer den Konflikt nicht sofort beseitigen kann, schaltet das hier ab: aktuell
# nutzen wir die Daten noch nirgends. Die richtige Lösung bleibt, das zweite
# Programm zu beenden (siehe docs/hardware.md).
TELEMETRY_ENABLED = True

# --- Wiederverbinden nach USB-Aussetzern ------------------------------------

# Bricht die 5-V-Versorgung am USB-Hub kurz ein (schwacher Akku, Motoren und
# Kamera ziehen gleichzeitig an), melden sich Board, Lidar und Kamera ab und
# kurz darauf unter neuen Nummern wieder an. Der Server bemerkt das und
# verbindet neu -- der Roboter steht so lange.

# Wie oft nachgesehen wird, ob die Verbindung zum Board noch steht.
RECONNECT_CHECK_S = 1.0

# Abstand der Neuversuche: beginnt kurz und verdoppelt sich bis zum
# Höchstwert, solange das Gerät nicht wieder auftaucht.
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 30.0

# Fällt der Lidar innerhalb dieser Zeit nach dem Neu-Verbinden wieder aus,
# beginnt die Wartezeit nicht von vorn, sondern wächst weiter. Sonst startet
# sein Motor alle paar Sekunden neu und hält die USB-Ausfälle am Laufen.
LIDAR_STABIL_S = 60.0

# Kommt so lange kein neues Kamerabild, wird die Kamera neu geöffnet.
# Die Wartezeit dazwischen wächst bei jedem Fehlschlag: ist das Anlaufen der
# Kamera selbst der Auslöser des Spannungseinbruchs, würde ein schnelles
# Wiederholen den ganzen Hub im Takt neu starten.
CAMERA_REOPEN_S = 5.0
CAMERA_REOPEN_MAX_S = 120.0

# --- Fahren ------------------------------------------------------------------

# Obergrenzen. Der Watchdog begrenzt jeden eingehenden Befehl darauf, egal was
# die App schickt. Die App holt sich diese Werte beim Verbinden ab -- sie sind
# also nur hier einzustellen, nicht zusätzlich im Browser.
#
# ACHTUNG: Das Fahrgestell schafft rund 1 m/s, und das Expansion Board kappt
# intern auf seinen eigenen Bereich. Werte darüber ändern am tatsächlichen
# Tempo nichts -- die App zeigt dann mehr an, als der Roboter fährt.
MAX_SPEED_X = 1.0      # m/s  vorwärts / rückwärts
MAX_SPEED_Y = 1.0      # m/s  seitwärts (Mecanum)
MAX_SPEED_OMEGA = 3.0  # rad/s  drehen

# Zusätzliche Obergrenze für alles, was der Roboter selbst entscheidet
# (Folgen, Erkunden, Heimfahrt). Die einzelnen Modi haben eigene, meist
# niedrigere Werte -- diese Grenze fängt nur Rechenfehler ab, damit kein
# autonomer Modus je schneller fährt, als der Lidar Hindernisse rechtzeitig
# sieht (7,5 Umdrehungen/s, plus Bremsweg). Die Handsteuerung ist davon
# ausgenommen; für sie gelten MAX_SPEED_*.
AUTONOM_MAX_SPEED = 0.35        # m/s   vx und vy
AUTONOM_MAX_OMEGA = 1.2         # rad/s

# Akku fast leer: autonome Modi fahren langsamer. Unter Last bricht die
# Spannung zusätzlich ein, und genau dann setzt sich am X3 der USB-Hub
# zurück -- mit halbem Tempo zieht der Antrieb deutlich weniger Strom.
# Faktor auf AUTONOM_MAX_SPEED, sobald der Akku unter AKKU_WARN_V liegt.
AUTONOM_AKKU_FAKTOR = 0.5

# Bremsweg-Prüfung für alle autonomen Befehle im Watchdog (AutonomSchutz in
# safety/collision.py). Erlaubt ist nur ein Tempo, bei dem der Roboter vor
# dem nächsten Hindernis in Fahrtrichtung noch steht:
#   frei - RAND >= v * (LATENZ + Scan-Alter) + v² / (2 * BREMSEN)
# Die Modi halten selbst früher an; das hier ist die letzte Bremse.
AUTONOM_SCHUTZ_AKTIV = True
# Abstand Lidar-Mitte -> Hindernis, bei dem v = 0 ist. Kleiner als
# EXPLORE_BEFREIEN_STOP_M (0,30) und EXPLORE_STOP_M (0,40), sonst würde die
# Schicht das Befreien und das normale Erkunden blockieren.
AUTONOM_SCHUTZ_RAND_M = 0.22
AUTONOM_SCHUTZ_HALBE_BREITE_M = 0.20  # wie EXPLORE_HALF_WIDTH_M
# Zeit von der Messung bis die Motoren reagieren (Rechnen, Board, Rampe),
# zusätzlich zum Alter des Scans.
AUTONOM_SCHUTZ_LATENZ_S = 0.15
# Angenommene Bremsverzögerung. Bewusst kleiner als RAMPE_BREMSEN_XY (2,0):
# Mecanum-Räder rutschen auf glattem Boden, und gemessen ist der Wert nicht.
AUTONOM_SCHUTZ_BREMSEN = 1.0      # m/s²
# Älterer Scan = kein Fahren mehr (nur Drehen). Bei 7,5 Umdrehungen/s ist
# ein Scan im Betrieb höchstens ~0,15 s alt; 0,5 s heißt: der Lidar hängt.
AUTONOM_SCHUTZ_SCAN_MAX_S = 0.5

# --- Sicherheit ---------------------------------------------------------------

# Kommt innerhalb dieser Zeit kein neuer Fahrbefehl, hält der Roboter an.
# Schützt vor WLAN-Abriss, geschlossenem Browser-Tab und App-Abstürzen.
# Zu kurz = ruckeliges Fahren, zu lang = Roboter fährt unkontrolliert weiter.
COMMAND_TIMEOUT_S = 0.5

# Fahrvorrang bei mehreren offenen Apps: Jede App schickt auch im Stillstand
# zehnmal pro Sekunde "vx=0". Lenkt eine, kamen die Nullen der anderen
# dazwischen, und er fuhr stotternd mit halbem Tempo (03.11., zwei Geräte,
# fiel erst nach dem Folgen auf, weil die Interaktion Nullen ohnehin
# verwirft). So lange (s) nach dem letzten echten Fahrbefehl zählen Nullen
# nur von dem Gerät, das gerade lenkt. "stop" wirkt immer, von jedem Gerät.
DRIVE_VORRANG_S = 0.5

# Takt, in dem der Watchdog prüft. Muss deutlich kleiner als das Timeout sein.
WATCHDOG_INTERVAL_S = 0.1

# Anfahr- und Bremsrampe. Ohne sie springt jeder Fahrbefehl ungebremst auf die
# Motoren: bei Folgen und Gesten wechselt der Befehl im 20-Hz-Takt zwischen
# Fahren und Halten, die Motoren ziehen jedes Mal einen Stromstoß, die
# Spannung bricht ein und der USB-Hub setzt sich zurück (siehe Sensor-Check
# 28.10.: Resets nur unter Motorlast). Die Rampe begrenzt, wie schnell sich
# vx/vy/omega pro Sekunde ändern dürfen.
#
# Bremsen darf schneller sein als Anfahren -- Anhalten vor einem Hindernis ist
# wichtiger als sanftes Anfahren. watchdog.stop() (Not-Stopp, Timeout,
# Abbruch) umgeht die Rampe immer und steht sofort.
RAMPE_AKTIV = True
RAMPE_ANFAHREN_XY = 0.8       # m/s²   0 -> 0,3 m/s in knapp 0,4 s
RAMPE_BREMSEN_XY = 2.0        # m/s²   0,3 m/s -> 0 in 0,15 s (rund 2 cm Weg)
RAMPE_ANFAHREN_OMEGA = 3.0    # rad/s²
RAMPE_BREMSEN_OMEGA = 6.0     # rad/s²
# Längster Zeitschritt, den die Rampe auf einmal macht. Nach einer Pause
# (Roboter stand, erster Befehl seit Langem) würde sonst die ganze Pause als
# "Zeit zum Beschleunigen" zählen und der Befehl doch wieder springen.
RAMPE_DT_MAX_S = 0.1

# --- Server --------------------------------------------------------------------

# 0.0.0.0 = aus dem ganzen WLAN erreichbar (nötig, damit das Handy rankommt).
HOST = "0.0.0.0"
PORT = 8000

# Zusätzlich dieselbe App über HTTPS (0 = aus). Browser geben das Mikrofon
# nur auf sicheren Seiten frei, also nicht unter http://10.42.0.62:8000 --
# für die Durchsage vom Handy braucht es https://10.42.0.62:8443. Das
# Zertifikat erzeugt der Server beim ersten Start selbst (openssl); der
# Browser warnt einmal, weil es niemand beglaubigt hat.
HTTPS_PORT = 8443
HTTPS_ORDNER = "~/.rosmaster/https"

# Takt, in dem der Roboter von sich aus Statusmeldungen an die App schickt.
STATUS_INTERVAL_S = 0.5

# So lange wartet der Server beim Beenden (Strg+C) auf offene Verbindungen.
# Der Kamera-Stream endet von selbst nie: Ohne Grenze wartete uvicorn am
# 28.10. ewig darauf, und das Aufräumen (Motoren aus, Lidar stoppen) lief
# gar nicht erst an.
SHUTDOWN_TIMEOUT_S = 3

# --- Lidar ---------------------------------------------------------------------

LIDAR_ENABLED = True

# Fester Gerätepfad aus der udev-Regel. Nicht /dev/ttyUSB0 verwenden: das
# Expansion Board hängt am selben Bus und die Nummern können tauschen.
LIDAR_PORT = "/dev/rplidar"
LIDAR_BAUDRATE = 115200

# Einbaulage: um wie viel Grad die 0°-Richtung des Lidars von "vorn" (Kamera)
# abweicht. Am X3 ist er um 90° gedreht montiert -- ohne Korrektur lag in der
# App vorn links, und Hindernisstopp wie Ortung schauten in die falsche
# Richtung. Prüfen: Hand vor die Kamera halten, sie muss in der App oben
# erscheinen. Liegt sie stattdessen unten, 180 addieren.
LIDAR_ANGLE_OFFSET_DEG = -90.0

# Messpunkte außerhalb dieser Grenzen werden verworfen. Zu nah sind meist
# Teile des Roboters selbst, zu weit ist Rauschen.
LIDAR_MIN_M = 0.15
LIDAR_MAX_M = 8.0

# --- Lidar-gestützte Ortung -----------------------------------------------------

# Nur in dieser Entfernung zum Startpunkt wird mit der Referenzaufnahme
# verglichen. Weiter weg überlappt zu wenig, und ein Fehlabgleich wäre
# schlimmer als gar keine Korrektur.
MATCH_RADIUS_M = 4.0

# Mittlerer Restabstand nach dem Abgleich, bis zu dem das Ergebnis
# übernommen wird. Darüber gilt der Abgleich als misslungen.
MATCH_MAX_RESIDUAL_M = 0.18

# Wie stark eine Korrektur je Durchgang einfließt (0..1). Niedrig, damit
# ein einzelner Fehlabgleich die Position nicht verreißt.
MATCH_TRUST = 0.55

# Takt des Abgleichs. Öfter bringt wenig -- der Lidar dreht sich nur
# rund fünfmal pro Sekunde.
MATCH_HZ = 4.0

# --- Kollisionsvermeidung bei manueller Steuerung -------------------------------

# Standardmäßig aus: wer von Hand steuert, sieht den Roboter meistens und
# will nicht bevormundet werden. In der App zuschaltbar.
COLLISION_ENABLED = False

# Mindestabstand in Fahrtrichtung. In der App per Regler verstellbar.
COLLISION_DISTANCE_M = 0.35

# Grenzen des Reglers. Unter 15 cm sieht der Lidar nichts Verlässliches
# mehr, über 1,5 m wäre in einem Zimmer fast immer etwas im Weg.
COLLISION_MIN_M = 0.15
COLLISION_MAX_M = 1.5

# Halber Öffnungswinkel des überwachten Bereichs in Grad. Siehe
# HOME_CONE_DEG -- dieselbe Abwägung zwischen "übersieht Tischbeine" und
# "hält vor jeder Wand an, an der man vorbeifahren will".
COLLISION_CONE_DEG = 35.0

# --- Heimfahrt ---------------------------------------------------------------------

# Takt, in dem die Position fortgeschrieben wird. Höher = genauer, aber die
# Ungenauigkeit kommt ohnehin vom Radschlupf, nicht von diesem Takt.
ODOMETRY_HZ = 20

# Blickrichtung aus der IMU-Lage des Boards nehmen? Aus: der Sensor-Check
# vom 28.10. zeigt, dass get_imu_attitude_data() am X3 dauerhaft 0 liefert.
# Damit fror der Winkel ein (theta blieb bei 0, nur der Lidar-Abgleich
# bewegte ihn), und Heimfahrt, Erkunden und Ortung rechneten mit falscher
# Richtung. Stattdessen wird die vom Board gemessene Drehrate aufsummiert.
# Nur einschalten, wenn die IMU nachweislich einen echten Winkel meldet --
# selbst dann wird sie verworfen, sobald sie beim Drehen stillsteht.
ODOM_IMU_WINKEL = False

# Ab dieser Drehrate (rad/s) muss sich ein IMU-Winkel sichtbar ändern; tut
# er das über ODOM_IMU_PRUEF_S nicht, gilt die IMU als unbrauchbar.
ODOM_IMU_PRUEF_OMEGA = 0.3
ODOM_IMU_PRUEF_S = 1.0

# Tempo der automatischen Heimfahrt. Bewusst gemächlich: die Position stammt
# aus der Koppelnavigation, und Umfahren (UMFAHREN_*) braucht Zeit zum Schauen.
HOME_SPEED = 0.25        # m/s

# Ab dieser Entfernung zum Startpunkt gilt die Fahrt als beendet. Kleiner
# zu werden lohnt nicht -- die Koppelnavigation ist ungenauer als das.
HOME_TOLERANCE_M = 0.12

# Auf den letzten Zentimetern langsamer werden, sonst schießt er übers Ziel.
HOME_SLOWDOWN_M = 0.4

# Notbremse: nach dieser Zeit bricht die Heimfahrt ab. Schützt davor, dass
# der Roboter bei abgedrifteter Position endlos umherfährt.
HOME_TIMEOUT_S = 60.0

# Abstand, der in Fahrtrichtung frei bleiben muss. Ist er es nicht, schiebt
# er schräg vorbei (UMFAHREN_*); nur wenn HOME_AUSWEICH_WARTEN_S lang
# nirgends Platz ist, hält er an und meldet es.
HOME_STOP_DISTANCE_M = 0.45

# Halber Öffnungswinkel des überwachten Bereichs, in Grad. Zu schmal
# übersieht Tischbeine seitlich der Fahrlinie, zu breit hält wegen
# Wänden an, an denen man problemlos vorbeifährt.
HOME_CONE_DEG = 35.0

# --- Genaue Fahrbefehle ("2 Meter vor", "90 Grad drehen") -------------------------
# Siehe navigation/manoever.py. Gemessen wird mit dem Lidar: Jede Aufnahme
# wird auf eine vom Anfang gelegt, daraus folgt, wie weit er wirklich
# gefahren bzw. gedreht ist. Räder und IMU liefern nur die Schätzung
# zwischen zwei Aufnahmen (die IMU-Drehung ist am X3 unzuverlässig).

MANOEVER_SPEED = 0.20         # m/s  höchstens, beim Fahren
MANOEVER_MIN_SPEED = 0.05     # m/s  darunter laufen die Motoren nicht sicher an
MANOEVER_KP_V = 1.2           # (m/s)/m  Tempo je Meter Restweg
MANOEVER_DREH = 0.6           # rad/s höchstens, beim Drehen
# Unter 0,4 rad/s laufen die Motoren am Boden nicht zuverlässig an (Erkunder).
MANOEVER_MIN_DREH = 0.4
MANOEVER_KP_W = 1.5           # (rad/s)/rad
# So lange (s) nach dem Losdrehen aus dem Stand zählt die befohlene Drehung
# noch nicht. Sonst hielt die Schätzung kleine Nachbesserungen (1-2°) schon
# nach einem Takt für erledigt -- die Rampe gab da erst 0,3 rad/s aus, die
# Motoren liefen nicht an, und das Lidar maß im Stand denselben Rest wieder
# (Bodentest 03.11.: dreimal vergeblich, dann "fast erledigt").
MANOEVER_ANLAUF_S = 0.25
# Fertig, wenn Restweg und Restwinkel darunter liegen ...
MANOEVER_TOLERANZ_M = 0.03
MANOEVER_TOLERANZ_GRAD = 2.0
# ... und das auch eine Lidar-Aufnahme bestätigt, die im Stand entstand:
# Im Fahren ist eine Umdrehung des A1 (0,18 s) verzerrt. So oft wird
# höchstens nachgebessert.
MANOEVER_NACHBESSERN = 3
MANOEVER_RUHE_S = 0.3         # nach dem Anhalten so lange warten, dann messen
# Möglichst alles gegen die Aufnahme vom Start abgleichen: Das Lidar sieht
# rundum, nach einer Drehung also dieselben Wände. Jeder Wechsel der
# Bezugsaufnahme bringt etwas Fehler mit (in der Simulation ~0,2° je
# Wechsel, mit Wechsel alle 25° lag er nach 360° um 3,4° daneben).
# Gewechselt wird nur, wenn der Abgleich schlechter wird -- er fährt aus dem
# Raum, die alten Wände verschwinden: mittlerer Restabstand größer als ...
MANOEVER_NEUER_BEZUG_RESIDUUM_M = 0.06
# ... oder weniger als dieser Anteil der Punkte findet einen Partner.
MANOEVER_NEUER_BEZUG_PAARE = 0.6
# Im Fahren bis auf diesen Teil der Toleranz heran -- sonst hielt er knapp
# innerhalb an (2,9 von 3 cm), und jede Ungenauigkeit kippte es nach außen.
# Nicht viel kleiner: Mit einem Drittel (1 cm) lag das Ziel in der
# Messgenauigkeit, und er besserte dreimal vergeblich nach.
MANOEVER_ZIEL_ANTEIL = 0.5
# So lange (s) darf der Abgleich am Stück scheitern, dann hält er an
# (fremde Umgebung, Lidar weg) -- blind fahren wäre ungenau und unsicher.
MANOEVER_BLIND_S = 2.0
# Grenzen für einen Befehl.
MANOEVER_MAX_M = 5.0
MANOEVER_MAX_GRAD = 720.0
# Drehen nur, wenn ringsum so viel Platz ist (Ecken schwenken aus), m ab
# Lidar-Mitte.
MANOEVER_DREH_FREI_M = 0.30
# Für den Abgleich höchstens so viele Lidar-Punkte (Rechenzeit am Pi).
MANOEVER_MAX_PUNKTE = 300

# --- Kamera ---------------------------------------------------------------------

CAMERA_ENABLED = True

# Welche USB-Kamera wofür da ist. Es laufen immer alle gefundenen Kameras;
# hier wird nur festgelegt, welche welchen Namen trägt und wer ihr Bild
# auswertet.
#
# Gefunden wird jede über ihre USB-Kennung wie in `lsusb` -- "vvvv" passt auf
# jedes Modell des Herstellers, "vvvv:pppp" nur auf genau eines. Nicht über
# /dev/videoN: die Nummern vertauschen sich nach Neustart oder
# Neu-Einstecken, und die Personenerkennung bekäme still das falsche Bild.
#
# Eine Rolle, deren Kamera fehlt, wird im Hintergrund weiter gesucht --
# meldet sich die Kamera nach einem USB-Aussetzer wieder, läuft sie weiter.
# Weitere USB-Kameras ohne Rolle laufen zusätzlich als "usb1", "usb2", ...
#
# fov_deg: horizontaler Bildwinkel bei 640x480 laut Datenblatt. Damit wird
# eine Person im Bild einer Richtung im Lidar zugeordnet -- bei Bedarf
# nachmessen.
#
# flip: Einbaulage ausgleichen. None, "horizontal" (links/rechts spiegeln),
# "vertikal" (oben/unten) oder "180" (beides = auf dem Kopf). Gedreht wird
# auf dem Roboter, bevor irgendwer das Bild sieht -- auch die Personen- und
# Posenerkennung. Auf der Einstellungsseite verstellbar.
CAMERA_ROLES = {
    # Orbbec-Tiefenkamera, Farbkanal. Für Orientierung und Fahren. Der
    # Tiefenkanal (2bc5:060f) ist kein Videogerät und braucht das Orbbec-SDK.
    "orientierung": {"usb_id": "2bc5", "label": "Orbbec (Farbbild)", "fov_deg": 66.0,
                     "flip": None},
    # Logitech C270: Personenerkennung. Hängt falsch herum (29.10.2026).
    "personen": {"usb_id": "046d", "label": "Logitech C270", "fov_deg": 45.0,
                 "flip": "horizontal"},
}

# Welche Kamera unter /camera/stream, /camera/snapshot und /camera/status
# antwortet (die Endpunkte ohne Kameranamen) und in der App zuerst zu sehen
# ist.
CAMERA_DEFAULT = "orientierung"

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Bilder pro Sekunde im Stream. Höher kostet WLAN-Bandbreite und CPU; bei
# ruckelndem Bild über WLAN eher hier heruntergehen als die Auflösung senken.
CAMERA_FPS = 15

# Was die Kameras selbst höchstens liefern: Logitech C270 und Orbbec
# (Farbbild) schaffen bei 640x480 je 30 fps -- bei wenig Licht verlängert
# die C270 die Belichtung und kommt nur noch auf ~15. Mit dieser Rate läuft
# die Kamera; CAMERA_FPS drosselt nur das Abholen.
CAMERA_MAX_FPS = 30

# JPEG-Qualität 1..100. Darunter wird das Bild matschig, darüber groß.
CAMERA_QUALITY = 70

# --- Autonomes Fahren (Erkunden) ------------------------------------------------
#
# Test 1: Der Roboter fährt zufällig umher und weicht dabei allem aus, was der
# Lidar sieht. Die Werte stammen aus dem Sensor-Check vom 28.10.2026
# (robot-protokolle/2026-10-28_sensorcheck): Der Befehl wird mit ~0,22 s
# Verzögerung und nur zu 70-90 % umgesetzt -- deshalb gemächlich.

# Reisetempo geradeaus. Mehr als 0,25 m/s lohnt im Zimmer nicht: Bei 7,5
# Lidar-Umdrehungen pro Sekunde fährt er sonst zwischen zwei Scans 3-4 cm.
EXPLORE_SPEED = 0.20      # m/s

# Drehrate beim Wenden auf der Stelle. Langsamer als 0,4 rad/s laufen die
# Motoren am Boden nicht zuverlässig an.
EXPLORE_TURN_SPEED = 0.6  # rad/s

# Gemessen wird ab der Lidar-Mitte. Der X3 ragt nach vorn ~0,15 m darüber
# hinaus -- 0,40 m lassen also ~25 cm Luft, genug für den Bremsweg bei
# 0,2 m/s plus Verzögerung.
EXPLORE_STOP_M = 0.40

# Ab hier vor einem Hindernis langsamer werden.
EXPLORE_SLOW_M = 0.90

# So viel Platz muss vorn frei sein, damit er nach dem Drehen losfährt.
# Größer als EXPLORE_STOP_M, sonst hält er gleich wieder an und pendelt.
EXPLORE_GO_M = 0.70

# Halbe Breite des Fahrschlauchs: halbe Roboterbreite (0,245 m / 2) plus
# Sicherheitsrand. Tischbeine knapp neben der Fahrlinie zählen so mit.
EXPLORE_HALF_WIDTH_M = 0.20

# Kommt seitlich etwas näher als das, schiebt ihn der Mecanum-Antrieb
# sachte davon weg, statt anzuhalten.
EXPLORE_SIDE_M = 0.30

# Rückt etwas rundherum näher als das (Mensch tritt heran), bleibt er stehen,
# statt sich in die Person hineinzudrehen.
EXPLORE_NAH_M = 0.22

# Zufällige Richtungswechsel, damit er nicht immer dieselbe Bahn fährt:
# frühestens/spätestens nach so vielen Sekunden Geradeausfahrt.
EXPLORE_WANDER_MIN_S = 4.0
EXPLORE_WANDER_MAX_S = 10.0

# Am Boden dreht er nur ~75 % dessen, was befohlen wird (Sensor-Check:
# 90° befohlen, 65° gedreht). Die Drehdauer wird damit verlängert. Gilt nur
# noch als Rückfall, wenn das Board keine Drehrate liefert -- sonst dreht er
# nach gemessenem Winkel (EXPLORE_DREH_*).
EXPLORE_TURN_FACTOR = 1.3

# Drehen nach Winkel: so viele Grad vor dem Ziel den Drehbefehl beenden.
# Befehlsverzögerung und Bremsrampe drehen ihn noch ein Stück weiter.
EXPLORE_DREH_VORHALT_GRAD = 8.0

# Obergrenze einer gemessenen Drehung als Vielfaches der rechnerischen
# Dauer -- fängt ab, dass er sich nicht dreht (z. B. eingeklemmt).
EXPLORE_DREH_ZEIT_FAKTOR = 2.0

# Nach dem Drehen entscheidet er erst mit einer Lidar-Runde, die so lange
# nach Drehende begonnen hat (der Roboter rollt noch kurz nach).
EXPLORE_NACHLAUF_S = 0.25

# Älter darf eine Lidar-Umdrehung nicht sein, sonst fährt er blind. Der
# echte Lidar liefert alle ~0,13 s eine Runde.
EXPLORE_SCAN_MAX_AGE_S = 1.0

# So lange auf frische Lidar-Daten warten, bevor die Fahrt endet.
EXPLORE_LIDAR_WAIT_S = 3.0

# Steht ein Mensch so nah vor ihm, wartet er erst, statt sofort zu wenden --
# vielleicht geht die Person ja weiter.
EXPLORE_PERSON_WAIT_S = 2.0

# Gesamtdauer einer Erkundung; 0 = bis jemand stoppt.
EXPLORE_TIMEOUT_S = 0.0

# Rückt etwas näher als EXPLORE_NAH_M, wartet er so lange (vielleicht ein
# Mensch, der gleich weitergeht) und schiebt sich dann frei.
EXPLORE_NAH_WARTEN_S = 3.0

# Befreien: ohne Drehung in die freieste Richtung schieben (Mecanum). So
# langsam, dass das Heck nichts anstößt, so lange, dass es etwas bringt.
EXPLORE_BEFREIEN_SPEED = 0.08
EXPLORE_BEFREIEN_S = 1.5
# In der gewählten Richtung muss mindestens so viel frei sein (ab
# Lidar-Mitte, der Roboter selbst reicht ~0,15 m nach vorn und hinten) ...
EXPLORE_BEFREIEN_FREI_M = 0.45
# ... und unter diesem Wert bricht er das Schieben ab.
EXPLORE_BEFREIEN_STOP_M = 0.30
# So viele Befreiungen in Folge ohne freie Fahrt dazwischen, dann gibt er auf.
EXPLORE_BEFREIEN_MAX = 3
# So lange muss er ungehindert geradeaus fahren, damit Fehlversuche und
# Befreiungen wieder bei null beginnen.
EXPLORE_FREI_ZURUECK_S = 3.0

# Festfahr-Erkennung: Fährt er (Befehl vx mindestens EXPLORE_FEST_MIN_VX)
# so lange, ohne dass das Board Bewegung meldet (unter EXPLORE_FEST_V_MS)
# oder sich das Lidar-Bild rundum in höchstens einer Richtung um mehr als
# EXPLORE_FEST_WEG_M (bzw. 2 % des Abstands, Messrauschen) ändert, steckt er
# fest. Muss unter den 3 s der Odometrie bleiben, nach denen sie eine
# Stillstand meldende Messung ganz verwirft.
EXPLORE_FEST_S = 1.5
EXPLORE_FEST_MIN_VX = 0.05
EXPLORE_FEST_V_MS = 0.02
EXPLORE_FEST_WEG_M = 0.03

# --- Ausweichen (Erkunder, Schritt F2) -------------------------------------------
#
# Lokales Ausweichen ohne Karte: kurzes Gedächtnis für Punkte, die aus dem
# Blickfeld rutschen, Bögen statt Wenden vor Hindernissen, Türen und Lücken
# mittig anfahren, Hochheben erkennen. Siehe navigation/ausweichen.py.

# Kurzgedächtnis: Lidar-Punkte so lange und in diesem Umkreis behalten. Die
# Lage wird aus der Board-Geschwindigkeit fortgeschrieben, die driftet --
# deshalb nur wenige Sekunden und nur nahe Punkte.
AUSWEICH_GEDAECHTNIS_S = 3.0
AUSWEICH_GEDAECHTNIS_M = 1.0
# Ein gemerkter Punkt fällt weg, wenn der Lidar jetzt in dieser Richtung um
# so viel weiter sieht (das Ding ist weg oder war ein Messfehler).
AUSWEICH_FREIRAEUMEN_M = 0.10

# Umriss des Roboters um die Lidar-Mitte (m) für die Bogenplanung. Breite
# 0,245 m; vorn ~0,15 m wie bei EXPLORE_STOP_M. Heck am Roboter nachmessen.
AUSWEICH_VORN_M = 0.15
AUSWEICH_HINTEN_M = 0.15
AUSWEICH_HALBE_BREITE_M = 0.13

# Bogenplanung: so weit (s) und in diesen Schritten vorausrechnen. Ein Bogen
# taugt nur, wenn der Umriss überall mindestens AUSWEICH_RAND_M Luft hat.
AUSWEICH_HORIZONT_S = 1.2
AUSWEICH_SCHRITT_S = 0.1
AUSWEICH_RAND_M = 0.05
# Kandidaten: Drehraten (rad/s) und seitliche Geschwindigkeiten (m/s).
AUSWEICH_OMEGAS = (-0.6, -0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6)
AUSWEICH_VYS = (-0.08, 0.0, 0.08)
# Gewichte der Bewertung: zurück auf den gewählten Kurs, Luft zum nächsten
# Hindernis, Tempo, Weite in Fahrtrichtung am Ende des Bogens, ruhiges
# Lenken (Sprünge in omega), seitliches Schieben nur wenn nötig. RUCK klein
# halten, sonst wiegt er die Kurskorrektur immer auf und der Roboter lenkt
# nie zurück; SEITLICH so, dass er neben Hindernissen (Türrahmen,
# Stuhlbein) noch seitlich wegschiebt -- wie früher bei EXPLORE_SIDE_M.
AUSWEICH_GEWICHT_KURS = 1.0
AUSWEICH_GEWICHT_LUFT = 1.0
AUSWEICH_GEWICHT_TEMPO = 0.5
AUSWEICH_GEWICHT_WEITE = 1.5
AUSWEICH_GEWICHT_RUCK = 0.2
AUSWEICH_GEWICHT_SEITLICH = 0.25
# Bis zu diesem Abstand (m) zählt mehr Luft bzw. Weite als besser.
AUSWEICH_LUFT_GUT_M = 0.30
AUSWEICH_WEITE_GUT_M = 2.0

# Lücken und Türen: Richtungen in diesem Raster (Grad) prüfen. Von einem
# freien Bereich nimmt er die Mitte und Richtungen mit mindestens
# AUSWEICH_LUECKE_RAND_GRAD Abstand zum Rand -- nicht knapp am Türrahmen.
AUSWEICH_LUECKE_RASTER_GRAD = 2
AUSWEICH_LUECKE_RAND_GRAD = 10
# Ist etwas näher als das (Türrahmen, Engstelle), verschiebt er den
# zufälligen Richtungswechsel, bis er hindurch ist.
AUSWEICH_ENG_M = 0.45

# Hochheben: Aufeinanderfolgende Runden (Eigenbewegung herausgerechnet) in
# 10°-Sektoren vergleichen. Ändert sich ein Sektor um mehr als
# AUSWEICH_HOCH_SPRUNG_M, zählt er als verändert. Ab AUSWEICH_HOCH_ANTEIL
# veränderter Sektoren in AUSWEICH_HOCH_RUNDEN Runden hintereinander gilt
# er als hochgehoben; wieder abgesetzt, wenn die Runden AUSWEICH_HOCH_RUHE_S
# lang unter AUSWEICH_HOCH_RUHIG_ANTEIL bleiben. Ein Mensch dicht daneben
# deckt nur ~4 von 36 Sektoren ab. Mit gemessener Lage vergleicht er über
# AUSWEICH_HOCH_FENSTER_S statt von Runde zu Runde -- so fällt auch
# langsames Tragen auf, das die Räder nicht melden.
AUSWEICH_HOCH_SPRUNG_M = 0.30
AUSWEICH_HOCH_ANTEIL = 0.5
AUSWEICH_HOCH_RUNDEN = 2
AUSWEICH_HOCH_RUHIG_ANTEIL = 0.15
AUSWEICH_HOCH_RUHE_S = 2.0
AUSWEICH_HOCH_FENSTER_S = 1.0

# Umfahren beim Folgen und auf der Heimfahrt (Schritt F3). Dort gibt es ein
# Ziel; ist der Weg dorthin versperrt, schiebt er schräg daran vorbei
# (ausweichen.richtung_waehlen). Frei sein müssen höchstens VORAUS_M plus
# Stoppabstand -- weiter vorn sieht er es rechtzeitig wieder. Gesucht wird
# bis MAX_GRAD neben der Zielrichtung im Raster RASTER_GRAD. Die zuletzt
# gewählte Seite bekommt SEITE_GRAD Vorsprung (sonst Pendeln vor breiten
# Hindernissen), und zum Rand des freien Bereichs hält er RAND_GRAD Abstand,
# in schmalen Durchgängen nimmt er die Mitte.
UMFAHREN_VORAUS_M = 0.8
UMFAHREN_MAX_GRAD = 75.0
UMFAHREN_RASTER_GRAD = 5.0
UMFAHREN_SEITE_GRAD = 20.0
UMFAHREN_RAND_GRAD = 10.0

# Heimfahrt: So lange darf in keiner Richtung Platz sein, bevor sie aufgibt.
# Kurz, damit ein vorbeigehender Mensch sie nicht beendet.
HOME_AUSWEICH_WARTEN_S = 3.0

# --- Hinderniserkennung ----------------------------------------------------------

# Springt der Abstand zwischen zwei Nachbarpunkten um mehr als das, beginnt
# ein neues Objekt. Wächst mit der Entfernung, weil die Punkte dort weiter
# auseinanderliegen (1° bei 3 m sind 5 cm).
OBSTACLE_GAP_M = 0.10

# Nur Objekte in diesem Umkreis werden benannt -- weiter weg sind es zu
# wenige Punkte für eine sinnvolle Einordnung.
OBSTACLE_RANGE_M = 3.0

# Stuhl- und Tischbeine sind im Lidar dünne Punkthaufen (2-5 cm).
OBSTACLE_LEG_MAX_M = 0.07

# Menschenbeine sind 8-20 cm breit (Knöchel bis Wade, mit Hose).
OBSTACLE_HUMAN_LEG_MAX_M = 0.25

# Ab dieser Länge und Geradheit gilt ein Objekt als Wand.
OBSTACLE_WALL_MIN_M = 0.60

# --- Personenerkennung (Kamera) ------------------------------------------------

# Nutzt den in OpenCV eingebauten HOG-Personendetektor -- braucht kein
# Modell aus dem Netz. Fehlt OpenCV, läuft alles ohne Kamera-Erkennung.
PERSON_DETECTION_ENABLED = True

# Bilder pro Sekunde für die Erkennung. Ein Bild kostet auf dem Pi 5
# ~0,1-0,2 s CPU; mehr würde Lidar und Steuerung ausbremsen.
PERSON_DETECT_HZ = 2.0

# Auf diese Breite verkleinert erkennen: schneller, und Personen im Zimmer
# sind ohnehin groß im Bild.
PERSON_DETECT_WIDTH = 320

# Mindestgewicht des HOG-Detektors. Höher = weniger Fehlalarme, aber
# Personen von der Seite werden eher übersehen.
PERSON_MIN_SCORE = 0.5

# Welche Kamera aus CAMERA_ROLES nach Personen sucht. Fehlt die Rolle in
# CAMERA_ROLES, nimmt die Erkennung die Standardkamera.
PERSON_CAMERA = "personen"

# Horizontaler Bildwinkel in Grad für Kameras ohne eigene Angabe in
# CAMERA_ROLES (CSI-Kamera, zusätzliche USB-Kameras).
CAMERA_FOV_DEG = 60.0

# --- Körperpose (Hailo-10H) ------------------------------------------------------

# Gesten brauchen die Körperpunkte (Schultern, Handgelenke ...), nicht nur
# eine Box. Das Modell läuft auf dem AI HAT+ 2 und kostet die CPU fast
# nichts -- wichtig, weil CPU-Lastsprünge am X3 USB-Resets auslösen.
POSE_ENABLED = True

# Mitgeliefert vom Hailo-Paket (hailo-models). Erwartet 640x640 RGB.
POSE_HEF = "/usr/share/hailo-models/yolov8s_pose_h10.hef"

# Bilder pro Sekunde. Die Kamera liefert 15; für Winken braucht es ~10,
# damit eine Auf-Ab-Bewegung mehrere Bilder umfasst.
POSE_HZ = 10.0

# Mindestwert, ab dem ein Treffer als Person gilt (0..1).
POSE_MIN_SCORE = 0.5

# Körperpunkte unter dieser Sicherheit zählen als "nicht gesehen" --
# verdeckte Hände sollen keine Geste auslösen.
POSE_KP_MIN_CONF = 0.4

# Hailo beim Serverstart öffnen und offen lassen, statt ihn erst beim
# Start der Gestensteuerung zu öffnen und danach wieder freizugeben. Das
# Öffnen ist eine Lastspitze, die am X3 den USB-Hub zurücksetzen kann --
# dann war das Board weg ("Keine Verbindung zum Antrieb"), genau in dem
# Moment, in dem die Interaktion losgehen sollte. Beim Serverstart steht
# der Roboter, und es passiert nur einmal. False gibt den Chip frei, wenn
# die Interaktion aus ist (z. B. für hailo-apps).
POSE_WARM_HALTEN = True

# --- Gesten ------------------------------------------------------------------------

# So weit reicht der Blick zurück, um Bewegungen (Winken, Wischen) zu sehen.
GESTE_FENSTER_S = 1.5

# Folgen (ein Arm über dem Kopf) muss so lange gehalten werden -- sonst
# löst jedes Haare-Richten aus.
GESTE_HALTEN_S = 1.0

# Stopp (beide Arme hoch) soll schnell wirken, deshalb kürzer.
GESTE_STOPP_HALTEN_S = 0.4

# Anteil der Bilder im Haltezeitraum, die die Haltung zeigen müssen.
# Nicht 100 %: einzelne Bilder mit verlorenem Handgelenk sind normal.
GESTE_ANTEIL = 0.8

# Winken (Herkommen): Mindestzahl Richtungswechsel des Handgelenks im
# Fenster und Mindesthub, gemessen in Schulterbreiten (so unabhängig vom
# Abstand der Person).
GESTE_WINK_WECHSEL = 2
GESTE_WINK_HUB = 0.35

# Wischen (Weggehen): so viele Schulterbreiten muss die Hand seitlich nach
# außen wandern.
GESTE_WISCH_STRECKE = 1.0

# Nach einer erkannten Geste so lange keine neue -- eine gehaltene Geste
# soll nicht jede Sekunde neu auslösen.
GESTE_PAUSE_S = 2.0

# Entprellen der bewegten Gesten (Herkommen, Weggehen). Beim Gehen schwingen
# die Arme mit, und von hinten oder im Laufen wechselt die Größe der Person
# -- das sah der Erkennung zu oft nach Winken oder Wischen aus (beim Folgen
# wechselte er ohne klares Zeichen). Deshalb zählen sie nur, wenn ...
#
# ... die Person ungefähr am Platz steht: Größe (Schulterbreite) zwischen
# Anfang und Ende des Fensters höchstens um diesen Faktor verschieden ...
GESTE_RUHE_GROESSE = 1.2
# ... und die Schultermitte höchstens so viele Schulterbreiten gewandert;
GESTE_RUHE_WEG = 0.8
# ... der Roboter selbst steht (eigene Fahrt und Drehung verwackeln das
# Bild und verschieben die Person): Grenzen für den gerade gefahrenen
# Befehl in m/s bzw. rad/s;
GESTE_ROBOTER_RUHIG_V = 0.05
GESTE_ROBOTER_RUHIG_W = 0.3
# ... und die Geste in so vielen Auswertungen hintereinander erkannt wurde.
GESTE_BESTAETIGEN = 2
# Außerdem muss das Gesicht (Nase) in GESTE_ANTEIL der Bilder zu sehen
# sein: Wer dem Roboter den Rücken zuwendet, gibt ihm kein Zeichen.
# STOPP und FOLGEN (gehaltene Haltungen) sind davon ausgenommen -- STOPP
# muss immer wirken.

# --- Interaktion (Folgen, Herkommen, Weggehen) -------------------------------------

# Wunschabstand beim Folgen. 1 m: nah genug, um die Person nicht zu
# verlieren, weit genug, dass sie sich nicht bedrängt fühlt.
FOLGEN_ABSTAND_M = 1.0

# Höchstgeschwindigkeit beim Folgen. Gemütliches Gehen ist ~1 m/s; der
# Roboter muss nicht mithalten, er holt beim Stehenbleiben auf.
FOLGEN_MAX_SPEED = 0.30

# Rückwärts nur langsam -- hinten ist er weniger umsichtig unterwegs.
FOLGEN_RUECK_SPEED = 0.12

# Regler: Geschwindigkeit pro Meter Abstandsfehler bzw. Drehrate pro
# Radiant Winkelfehler.
FOLGEN_KP_V = 0.6
# 1,2 statt 1,5: Die Kamera liefert den Winkel mit ~0,1-0,2 s Verzug; zu
# scharf eingestellt überdreht er und pendelt (29.10.2026).
FOLGEN_KP_W = 1.2
FOLGEN_MAX_OMEGA = 0.8

# Glättung des Kamerawinkels (0..1, Anteil des neuen Werts je Bild). Die
# Körperpunkte zittern von Bild zu Bild um einige Grad; ungeglättet wird
# daraus ein Zappeln der Drehrate.
FOLGEN_WINKEL_GLAETTUNG = 0.4

# Unter diesem Winkelfehler (Grad) dreht er nicht nach -- der Übergang ist
# stetig (Regler rechnet ab dem Rand der Totzone), damit es nicht ruckt.
FOLGEN_WINKEL_TOTZONE = 3.0

# Innerhalb dieses Abstandsfehlers bleibt er stehen -- sonst ruckelt er
# bei jedem Atemzug der Person vor und zurück.
FOLGEN_TOTZONE_M = 0.10

# Erst geradeaus fahren, wenn die Person ungefähr vorn ist; sonst zuerst
# drehen (sonst fährt er schräg an ihr vorbei).
FOLGEN_MAX_WINKEL_FAHREN = 25.0
# Bis zu diesem Winkel volle Fahrt, darüber bis FOLGEN_MAX_WINKEL_FAHREN
# gleitend weniger -- früher sprang vx an der Grenze hart zwischen 0 und
# voll, das ruckte und belastete die Stromversorgung.
FOLGEN_VOLL_WINKEL = 10.0

# Verschwindet die Person nur kurz (ein, zwei Bilder ohne Erkennung),
# rollt er so lange mit dem letzten Befehl weiter aus, statt hart zu
# bremsen und gleich wieder anzufahren. Danach hält er an. Ebenso lange
# gilt der letzte gute Lidar-Abstand, wenn in Richtung der Person kurz
# kein Punkt liegt.
FOLGEN_HALTEN_S = 0.5

# Abstand zur Person aus dem Lidar-Cluster statt aus dem kleinsten Wert im
# Bildwinkel der Box: Ein Stuhl oder Tisch vor der Person würde sonst als
# ihr Abstand gelesen, und der Roboter bliebe stehen, während sie
# weitergeht. Gesucht werden Cluster (navigation/hindernisse.py) im
# Winkelbereich der Box, erweitert um diesen Rand -- Kamera und Lidar sehen
# nicht exakt zur selben Zeit, beim Drehen verschiebt sich das um einige Grad.
FOLGEN_CLUSTER_RAND_GRAD = 5.0
# Weicht ein Cluster um mehr als das vom letzten guten Abstand ab, ist er
# wahrscheinlich nicht die Person (bei 0,3 m/s Schritttempo legt sie in
# FOLGEN_CLUSTER_GUT_S höchstens ~0,3 m zurück). Gilt nur, solange der
# letzte gute Abstand jünger ist als FOLGEN_CLUSTER_GUT_S.
FOLGEN_CLUSTER_SPRUNG_M = 0.5
FOLGEN_CLUSTER_GUT_S = 1.0

# War etwas sehr nah (EXPLORE_NAH_M), bleibt er noch so lange stehen,
# nachdem es frei ist -- sonst flackert er an der Grenze zwischen Halt
# und Fahrt.
FOLGEN_NAH_HALTEN_S = 0.5

# So lange darf die Person aus dem Bild sein, bevor er sucht, und so lange
# sucht er (drehen in die Richtung, in der sie verschwand). 0 = so lange,
# bis er wieder jemanden sieht; abbrechen mit Stopp oder "Robi, gute Nacht".
FOLGEN_VERLOREN_S = 2.0
FOLGEN_SUCHEN_S = 0.0
# Beim Suchen abwechselnd drehen und stillhalten (s): Während der Drehung
# verwischt das Kamerabild, und die Posenerkennung übersieht die Person.
# 1,2 s bei EXPLORE_TURN_SPEED (0,6 rad/s) sind rund 40°, weniger als das
# Bildfeld der C270 -- so bleibt keine Lücke im Rundblick.
FOLGEN_SUCHEN_DREH_S = 1.2
FOLGEN_SUCHEN_PAUSE_S = 0.8

# Herkommen: bis zu diesem Abstand heranfahren, dann stehen bleiben.
HERKOMMEN_ABSTAND_M = 0.70

# Weggehen: so weit zurückweichen, mit dieser Geschwindigkeit.
WEGGEHEN_STRECKE_M = 1.0
WEGGEHEN_SPEED = 0.15

# Herkommen/Weggehen enden spätestens nach dieser Zeit, auch wenn das Ziel
# nicht erreicht ist -- kein Verhalten soll endlos laufen.
INTERAKTION_VERHALTEN_MAX_S = 20.0

# Wechselt die verfolgte Person um mehr als diesen Winkel in einem Bild,
# ist es vermutlich eine andere -- dann beginnt die Gestenerkennung neu.
INTERAKTION_SPRUNG_GRAD = 15.0

# Sanfter Start (s): Wird die Interaktion eingeschaltet, läuft erst nur die
# Posenerkennung warm, der Roboter steht still und schweigt. Danach sagt er
# den Startsatz ("Jetzt folge ich dir.") und fährt erst los, wenn der Satz
# zu Ende ist. Am 03.11. starteten Posenerkennung, Sprechen und Motoren in
# derselben Sekunde, und die USB-Versorgung brach zusammen. 0 = sofort.
INTERAKTION_ANLAUF_S = 5.0

# --- Ziel-Lock beim Folgen (interaktion/ziel.py) -----------------------------
# Früher folgte er immer der nächstgelegenen bzw. größten Person -- nach
# einer Suche oder beim Kreuzen sprang er so auf Fremde über. Jetzt hält
# ein Tracker IDs über Bilder hinweg, und das Ziel wird zusätzlich an der
# Farbe des Oberkörpers (HSV-Histogramm) wiedererkannt. Im Zweifel bleibt
# er stehen, statt zu raten.

# So lange wird das Aussehen des Ziels nach dem Start gelernt, bevor es als
# bestätigt gilt (er fährt währenddessen schon).
ZIEL_ERFASSEN_S = 1.0

# So viele Bilder in Folge muss ein Kandidat passen, bevor er nach einer
# Verdeckung wieder als Ziel gilt -- und so viele Fehlbilder in Folge
# lösen das Ziel von seinem Track.
ZIEL_BESTAETIGEN_BILDER = 5

# Größter Bhattacharyya-Abstand (0 = gleich, 1 = völlig verschieden), bei
# dem ein Merkmal noch als „dieselbe Person“ gilt. Am Roboter prüfen: hängt
# stark von Licht und Kleidung ab.
ZIEL_MERKMAL_SCHWELLE = 0.5

# Beim Wiederfinden muss der beste Kandidat um mindestens so viel besser
# passen als der zweitbeste. Sonst sind zwei Personen zu ähnlich (z. B.
# gleich gekleidet) -- dann hält er sicher an.
ZIEL_MERKMAL_MARGE = 0.15

# Winkelfenster um die vorhergesagte Position, in dem eine Erkennung dem
# Track zugeordnet werden darf (wächst mit der Unsicherheit bis zum
# Dreifachen).
ZIEL_GATE_GRAD = 15.0

# Wie stark das gelernte Merkmal je Bild nachgeführt wird (Licht ändert
# sich beim Gehen durch den Raum). Nur bei eindeutiger Szene und gutem
# Treffer -- sonst lernt er schleichend eine fremde Person.
ZIEL_MERKMAL_NACHFUEHREN = 0.05

# So lange bleibt ein Track ohne Messung erhalten (vorhergesagt), bevor er
# verworfen wird.
ZIEL_TRACK_HALTEN_S = 1.5

# Höchstens so stark darf sich die Boxhöhe zwischen Vorhersage und Messung
# ändern (Verhältnis klein/groß), sonst ist es nicht derselbe Track.
ZIEL_HOEHE_VERHAELTNIS = 0.6

# --- Diagnose (Seite /diagnose) ------------------------------------------------
# Wie oft die schnellen Messungen laufen (Akku, USB-Überlastzähler, Geräte-
# zustand) und wie oft die langsamen (Temperatur, vcgencmd, Kernel-Log,
# Prozessliste). Die langsamen starten Unterprozesse -- jede Sekunde wäre
# unnötige Last auf einem Pi, dessen USB ohnehin unter Lastsprüngen leidet.
DIAGNOSE_INTERVAL_S = 1.0
DIAGNOSE_LANGSAM_S = 5.0

# Akkuschwellen für den 3S-Akku des X3 (voll 12,6 V). Unter 11,1 V reicht
# der Anlaufstrom der Motoren, um die 5 V kurz einbrechen zu lassen; unter
# 10,5 V nimmt der Akku Schaden und das Board meldet sich ständig ab.
AKKU_WARN_V = 11.1
AKKU_KRITISCH_V = 10.5

# Fällt die Spannung so weit unter das Maximum der letzten Minute, zählt
# das als Spannungseinbruch (typisch: Motoren laufen gleichzeitig an).
SPANNUNGSEINBRUCH_V = 0.6

# Ab dieser CPU-Temperatur warnt die Seite. Der Pi 5 drosselt selbst bei
# 85 °C -- vorher soll man es sehen (am 28.10. lief ein Build bis 83 °C).
TEMP_WARN_C = 75.0

# Über diesem Takt traten am X3 USB-Resets auf, siehe docs/hardware.md.
CPU_MAX_MHZ = 2000

# Ein Problem gilt als vorbei, wenn es so lange nicht mehr aufgetreten ist.
PROBLEM_FENSTER_S = 600.0

# Nach dem Serverstart brauchen Lidar und Kameras ein paar Sekunden bis zu
# den ersten Daten. So lange gilt "liefert nichts" noch nicht als Problem.
DIAGNOSE_ANLAUF_S = 15.0

# So viele Log-Zeilen und Ereignisse hält der Server im Speicher.
DIAGNOSE_LOG_ZEILEN = 1000
DIAGNOSE_EREIGNISSE = 300

# Akkuverlauf: ein Punkt je schnelle Messung, so viele Punkte (30 min).
DIAGNOSE_AKKU_PUNKTE = 1800

# Beim Start so weit im Kernel-Log zurückschauen (USB-Abmeldungen, die vor
# dem Serverstart passiert sind, z. B. beim Booten).
DIAGNOSE_KERNEL_RUECKBLICK_S = 3600.0

# --- Absturzerfassung (seit 29.10.) ---
# "Random Abstürze" ließen sich bisher nicht zuordnen: War es der Server
# (Fehler, Speicher, SSH zu), der ganze Pi (Reset, Strom) oder nur ein
# Hänger? Dafür schreibt die Diagnose in diesen Ordner mit, was ein
# Absturz überlebt: Laufprotokoll, Server-Log, Stack-Dumps.
DIAGNOSE_ORDNER = Path.home() / ".rosmaster" / "diagnose"

# So viele Serverläufe behält das Laufprotokoll (laeufe.json).
DIAGNOSE_LAEUFE = 20

# Größe einer Server-Logdatei; zwei alte Dateien bleiben zusätzlich liegen.
DIAGNOSE_LOGDATEI_BYTES = 2_000_000

# Puls der Ereignisschleife: Alle DIAGNOSE_PULS_S schaut der Server nach, wie
# pünktlich er geweckt wurde. Kommt er mehr als DIAGNOSE_HAENGER_S zu spät,
# hat etwas die Schleife blockiert (dann reagiert auch der Not-Stopp
# nicht). Bleibt der Puls DIAGNOSE_BLOCKIERT_S ganz aus, schreibt die
# Diagnose auf, woran die Schleife gerade hängt.
DIAGNOSE_PULS_S = 0.5
DIAGNOSE_HAENGER_S = 1.0
DIAGNOSE_BLOCKIERT_S = 3.0

# Ab hier warnt die Diagnose: wenig freier Arbeitsspeicher (der Kernel
# beendet sonst den größten Prozess -- meist den Server) und zu viele offene
# Dateien (ein Leck, das irgendwann jedes Öffnen scheitern lässt).
DIAGNOSE_SPEICHER_WARN_MB = 200
DIAGNOSE_DATEIEN_WARN = 800

# Ein hängender Browser darf die Statusmeldungen an alle anderen nicht
# aufhalten: Wer nach so vielen Sekunden nichts angenommen hat, fliegt raus.
WS_SENDE_TIMEOUT_S = 2.0

# Hintergrundschleifen fangen Fehler ab und laufen weiter. Damit ein Fehler,
# der jeden Takt kommt, nicht das Log flutet, steht er höchstens so oft drin.
SCHLEIFENFEHLER_LOG_S = 10.0

# USB-Sturm: So viele Überlast-Meldungen pro Minute *im Stand* sind kein
# Anlaufstrom mehr, sondern ein Dauerfehler an der Stromversorgung der
# USB-Buchsen. Am 29.10. waren es im Leerlauf 300-400 pro Minute.
USB_STURM_PRO_MIN = 20

# --- Fahrtenschreiber (aufzeichnung.py, seit 03.11.) ---------------------------
# Hält die letzten Sekunden einer Fahrt (Lidar-Runden, Fahrbefehle,
# Board-Geschwindigkeit, Zustände) im Speicher. Erst "Fahrt speichern" in der
# App schreibt sie auf die Karte; ausgewertet wird mit scripts/abspielen.py.
AUFZEICHNUNG_AKTIV = True
# Eine Minute reicht, um die Lage vor einem Fehler zu sehen, und braucht
# rund 10 MB Speicher (450 Lidar-Runden mit je ~360 Punkten).
AUFZEICHNUNG_PUFFER_S = 60.0
AUFZEICHNUNG_ORDNER = Path.home() / ".rosmaster" / "aufzeichnungen"
# Eine Datei hat gepackt einige hundert KB; ältere werden gelöscht.
AUFZEICHNUNG_MAX_DATEIEN = 20

# --- Musik ("MUKKE") ----------------------------------------------------------------

# YouTube-Links aus der App werden mit mpv auf dem USB-Lautsprecher
# abgespielt, nur der Ton. Voraussetzung auf dem Pi:
#   sudo apt install mpv
#   robot/.venv/bin/pip install -U yt-dlp
# yt-dlp aus pip statt apt: YouTube ändert laufend etwas, und die Version aus
# den Paketquellen ist meist zu alt, um noch abzuspielen.
MUKKE_ENABLED = True

# Ausgabegerät für mpv. Der USB-Lautsprecher heißt bei ALSA "UACDemoV10"
# (siehe `aplay -l`). plughw statt hw: wandelt Abtastrate und Format um,
# die der Lautsprecher nicht selbst kann. "auto" nimmt das Standardgerät.
MUKKE_AUDIO_DEVICE = "alsa/plughw:CARD=UACDemoV10,DEV=0"

# Lautstärke der Musik in Prozent (0..100), als Anteil von LAUTSTAERKE.
# In der App verstellbar.
MUKKE_LAUTSTAERKE = 60

# --- Ton -----------------------------------------------------------------------

# Gesamtlautstärke in Prozent (0..100): der eine Regler für alles, was aus dem
# Lautsprecher kommt (Stimme und Musik). Gerechnet wird in Software, der
# Mixer des Lautsprechers steht fest auf TON_MIXER. Auf der Einstellungsseite
# verstellbar. Leiser heißt auch: weniger Strom am USB des Pi.
LAUTSTAERKE = 70

# Hardware-Mixer des Lautsprechers (amixer) in Prozent. Bleibt immer gleich,
# damit es nur einen Regler gibt (LAUTSTAERKE).
TON_MIXER = 100

# --- Stimme (USB-Lautsprecher) ------------------------------------------------
# Sprachausgabe über den USB-Lautsprecher „UACDemoV1.0“. Siehe
# docs/interaktion.md und docs/hardware.md (Einrichtung).

# Abschaltbar, z.B. in der Vorlesung, ohne den Code anzufassen.
STIMME_ENABLED = True

# Piper spricht offline und klingt deutlich natürlicher als espeak-ng.
# Fehlt Piper oder das Modell, weicht die Stimme auf espeak-ng aus und
# zuletzt auf reine Log-Ausgabe -- der Server startet in jedem Fall.
# Orte für das Programm, der erste gefundene gilt. Das Release von
# github.com/rhasspy/piper wird nach ~/piper/piper/ entpackt.
STIMME_PIPER = ("~/piper/piper/piper", "piper")
# Weibliche Stimme "Kerstin" (16 kHz). Liegt auch noch Thorsten (männlich)
# unter ~/piper/de_DE-thorsten-medium.onnx, zum Umschalten nur hier ändern.
STIMME_MODELL = "~/piper/de_DE-kerstin-low.onnx"
STIMME_ESPEAK = "espeak-ng"

# Piper nur auf diesen CPU-Kernen rechnen lassen (taskset; None = alle).
# Auf allen vier Kernen war jeder Satz ein Lastsprung, und am 03.11. fielen
# genau beim Sprechen alle USB-Geräte ab (Over-Current). Mit einem Kern ist
# Piper kaum langsamer (gemessen 1,27 statt 1,28 s für 3,5 s Sprache).
STIMME_KERNE = "3"

# ALSA-Gerät über den Kartennamen statt der Nummer: die Nummer ändert sich,
# wenn sich USB-Geräte nach einem Spannungseinbruch neu anmelden.
# „plughw“ wandelt Abtastrate und Kanäle passend um.
STIMME_GERAET = "plughw:CARD=UACDemoV10,DEV=0"

# Sprechtempo für Piper: >1 langsamer. Etwas langsamer ist im Raum mit
# Motorgeräusch besser zu verstehen.
STIMME_TEMPO = 1.1

# Mehr Sätze warten nicht -- sonst redet der Roboter noch über etwas, das
# längst vorbei ist. Ältere fallen dann weg.
STIMME_MAX_WARTESCHLANGE = 3

# Derselbe Satz wird innerhalb dieser Zeit nicht wiederholt (z.B. wenn die
# Person mehrmals hintereinander verloren und wiedergefunden wird).
STIMME_WIEDERHOLUNG_S = 4.0

# Längere Texte werden abgeschnitten; die App soll keine Romane vorlesen.
STIMME_MAX_ZEICHEN = 300

# Kein Satz darf den Sprech-Thread länger blockieren.
STIMME_TIMEOUT_S = 20.0

# So lange nach dem Satzende gilt die Stimme noch als "spricht": Nachhall
# im Raum und Puffer im Lautsprecher -- die Spracherkennung soll das nicht
# als Eingabe verstehen.
STIMME_NACHLAUF_S = 0.4

# Durchsage: aus der App (Laptop/Handy-Mikrofon) live über den Lautsprecher
# des Roboters sprechen (WebSocket /sprache/durchsage). Kommt so lange (s)
# kein Ton, gilt sie als beendet und der Lautsprecher wird frei ...
DURCHSAGE_PAUSE_S = 2.0
# ... und mehr als so viel (s) Ton wird nicht gepuffert. Stand die Durchsage
# hinter einem Satz an, fällt der Anfang weg, statt dass sie dauerhaft um
# Sekunden hinterherhinkt.
DURCHSAGE_MAX_PUFFER_S = 0.5
# Puffer von aplay (µs). Kleiner = weniger Verzögerung, aber jedes Stocken
# im WLAN wird hörbar.
DURCHSAGE_APLAY_PUFFER_US = 200000

# --- Sprachbefehle (USB-Mikrofon, Vosk) -------------------------------------------
# Siehe interaktion/sprache.py und docs/interaktion.md. Ohne vosk/sounddevice,
# Modell oder Mikrofon ist die Erkennung aus; der Server läuft trotzdem.
SPRACHE_ENABLED = True

# Kleines deutsches Vosk-Modell (~45 MB, läuft auf der CPU des Pi).
# Herunterladen: siehe docs/hardware.md.
SPRACHE_MODELL = "~/vosk-model-small-de-0.15"

# None = selbst suchen (nach SPRACHE_MIKROFON_NAMEN), sonst ein Teil des
# Gerätenamens (str) oder die Gerätenummer von sounddevice (int).
SPRACHE_MIKROFON = None

# Reihenfolge = Vorliebe. Nur angesteckte USB-Mikrofone: das FongLun
# ("USB Microphone", hw:5 am 03.11.) bzw. C-Media ("USB PnP Sound Device").
# Die Kamera-Mikrofone sind bewusst nicht dabei: Das der C270 klingt so
# schlecht, dass Vosk kaum etwas versteht -- lieber taub und ehrlich
# "kein Mikrofon" in /sprache/status als halb hörend.
SPRACHE_MIKROFON_NAMEN = ("USB Microphone", "USB PnP", "C-Media", "Microphone")

# Geräte mit diesen Namensteilen nimmt die automatische Suche nie, auch wenn
# ein Eintrag oben passt (etwa "Webcam Microphone"). Ein fest eingetragenes
# SPRACHE_MIKROFON gilt trotzdem.
SPRACHE_MIKROFON_NIE = ("C270", "Logi", "WebCam", "ORBBEC", "Depth Sensor")

# Der Name des Roboters. Vosk kennt "robi" nicht als Wort; es hört je nach
# Aussprache eine der Varianten. Nur die, die im Modell stehen, werden benutzt.
SPRACHE_WECKWORT = "robi"
SPRACHE_WECKWORT_VARIANTEN = ("robbie", "robby", "roby", "robi", "ruby", "rubi", "robin")

# So lange (s) nach einem einzelnen "Robi" reicht der Befehl allein.
SPRACHE_WECK_FENSTER_S = 5.0

# "Stopp"/"halt" gilt auch ohne Namen: Anhalten darf nie daran scheitern,
# dass der Name überhört wurde. Ein falscher Stopp schadet nicht.
SPRACHE_STOPP_OHNE_WECKWORT = True

# "La-da-da" spielt dieses Lied über MUKKE. Ohne Weckwort: Wer "la da da"
# singt, will Musik -- und ein Fehlalarm fährt nirgends hin. Nur die
# Video-Adresse, ohne &list=...: das Lied, nicht die ganze Playlist.
SPRACHE_LADADA_URL = "https://www.youtube.com/watch?v=j-S_w3tK3dw"
SPRACHE_LADADA_OHNE_WECKWORT = True

# "Robi, Beatbox" spielt dieses Lied (mit Weckwort, wie die anderen Befehle).
SPRACHE_BEATBOX_URL = "https://www.youtube.com/watch?v=EfSp8EjZikE"

# Mit fester Grammatik macht Vosk aus jedem Geräusch einen erlaubten Satz:
# Musik, Fernseher und Gespräche wurden zu "robbie hallo" und schalteten die
# Interaktion ein. Deshalb gilt ein Satz nur, wenn Vosk sich bei jedem Wort
# mindestens so sicher ist (0..1) ...
SPRACHE_MIN_KONFIDENZ = 0.7
# ... und höchstens dieser Anteil der Wörter unbekannt war ([unk]). Mehr
# heißt: Da hat jemand geredet, und "komm" war nur ein Wort darin.
SPRACHE_MAX_UNK_ANTEIL = 0.5
# ... und neben Weckwort und Wendung höchstens so viele weitere Wörter
# kamen. Am 03.11. wurde aus Geräuschen "robin hallo hause zu weg ruby"
# (-> weggehen); ein gesprochener Befehl ist genau ein Grammatik-Satz.
SPRACHE_MAX_ZUSATZWOERTER = 0
# Unsicher Gehörtes lässt nur Stopp durch. Beim Einstellen helfen
# "konfidenz" und "zuletzt_verworfen" in /sprache/status.

# Läuft Musik, hört das Mikrofon das Lied mit. Dann gilt ein Befehl nur mit
# "Robi" und mit dieser höheren Konfidenz (Stopp und "Musik aus" immer).
SPRACHE_MIN_KONFIDENZ_MUSIK = 0.85

# Antwort auf ein einzelnes "Robi" (None = still). Gesagt wird sie erst nach
# dieser Pause (s): Wer "Robi ... folge mir" mit kurzer Pause sagt, dem soll
# das "Ja?" nicht ins Wort fallen -- während der Roboter spricht, hört er nicht.
SPRACHE_WECK_ANTWORT = "Ja?"
SPRACHE_WECK_ANTWORT_NACH_S = 1.0

# Live-Feed vom Mikrofon für die App (Mitlesen und Mithören, beides in der
# App schaltbar). So viele zuletzt gehörte Sätze zeigt das Mitlesen ...
SPRACHE_LIVE_EREIGNISSE = 15
# ... und so lange (s) nach dem letzten Abruf rechnet der Hör-Thread Pegel
# und Zwischentext noch aus. Liest niemand mit, kostet es nichts.
SPRACHE_LIVE_HALTEN_S = 3.0
# Mithören: so viele Blöcke (je 0,25 s) puffern, bevor welche wegfallen.
# Klein halten, sonst hört man mit Verzögerung.
SPRACHE_TON_PUFFER = 8

# Vosk-Modelle sind auf 16 kHz trainiert. Kann das Mikrofon das nicht,
# wird seine eigene Rate genommen und Vosk rechnet um.
SPRACHE_ABTASTRATE = 16000

# Wartezeit (s) vor dem nächsten Versuch, wenn das Mikrofon weg ist.
SPRACHE_NEUVERSUCH_S = 3.0

# Failsafe nach USB-Aussetzern. Nach einem Absturz meldete sich die C270
# schneller zurück als das USB-Mikrofon, und die Erkennung blieb dann auf
# der schlechteren C270 hängen (deshalb ist sie jetzt ganz ausgeschlossen).
# Läuft sie auf einem weniger bevorzugten Mikrofon, schaut sie so oft (s)
# nach, ob ein besseres wieder angemeldet ist, und wechselt ...
SPRACHE_MIKROFON_PRUEFEN_S = 5.0
# ... war es schon beim Öffnen da und PortAudio fand es trotzdem nicht,
# versucht sie es erst nach dieser Zeit (s) erneut (kein Dauerwechsel).
SPRACHE_MIKROFON_ERNEUT_S = 30.0
# Liefert das Mikrofon so lange (s) nur exakte Nullen, ist der Stream tot
# (das Gerät ist weg, ohne dass PortAudio einen Fehler meldet): neu öffnen.
SPRACHE_STUMM_S = 5.0
# Kommt so lange (s) gar kein Block mehr, hängt der Stream: Nach einem
# USB-Aussetzer blieb das Lesen ohne Fehler für immer stehen (Pegel
# eingefroren, Zustand "hoert", keine Befehle mehr). Dann neu öffnen.
SPRACHE_HAENGT_S = 3.0
# So viele Blöcke (je 0,25 s) puffert der Hör-Thread, wenn die Auswertung
# kurz nicht hinterherkommt (Piper rechnet); ältere fallen weg.
SPRACHE_BLOCK_PUFFER = 20
# So lange (s) darf das Schließen eines toten Streams dauern. Hängt auch
# das, wird er liegen gelassen, damit der Neuversuch nicht mit hängt.
SPRACHE_SCHLIESSEN_S = 2.0
# Kommt der Hör-Thread selbst so lange (s) nicht voran (z. B. hängt schon das
# Öffnen des Mikrofons in PortAudio), startet die Wache einen neuen. Muss
# länger sein als ein ganzer Neuversuch (Frist + Schließen + Warten).
SPRACHE_THREAD_HAENGT_S = 15.0
# Höchstens so oft hintereinander; danach Zustand "fehler" mit dem Hinweis,
# den Server neu zu starten (jeder hängende Thread bleibt liegen).
SPRACHE_THREAD_MAX_NEU = 3
