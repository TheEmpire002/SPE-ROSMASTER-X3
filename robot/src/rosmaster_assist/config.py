"""Zentrale Einstellungen.

Alle Werte, die man beim Ausprobieren verstellen will, stehen hier -- nicht
verstreut im Code. Wer am Fahrverhalten schraubt, muss nur diese Datei öffnen.
"""

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
MAX_SPEED_X = 5.0      # m/s  vorwärts / rückwärts
MAX_SPEED_Y = 5.0      # m/s  seitwärts (Mecanum)
MAX_SPEED_OMEGA = 5.0  # rad/s  drehen

# --- Sicherheit ---------------------------------------------------------------

# Kommt innerhalb dieser Zeit kein neuer Fahrbefehl, hält der Roboter an.
# Schützt vor WLAN-Abriss, geschlossenem Browser-Tab und App-Abstürzen.
# Zu kurz = ruckeliges Fahren, zu lang = Roboter fährt unkontrolliert weiter.
COMMAND_TIMEOUT_S = 0.5

# Takt, in dem der Watchdog prüft. Muss deutlich kleiner als das Timeout sein.
WATCHDOG_INTERVAL_S = 0.1

# --- Server --------------------------------------------------------------------

# 0.0.0.0 = aus dem ganzen WLAN erreichbar (nötig, damit das Handy rankommt).
HOST = "0.0.0.0"
PORT = 8000

# Takt, in dem der Roboter von sich aus Statusmeldungen an die App schickt.
STATUS_INTERVAL_S = 0.5

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

# Tempo der automatischen Heimfahrt. Bewusst gemächlich: der Roboter fährt
# hier ohne Hinderniserkennung, da will man nicht überrascht werden.
HOME_SPEED = 0.25        # m/s

# Ab dieser Entfernung zum Startpunkt gilt die Fahrt als beendet. Kleiner
# zu werden lohnt nicht -- die Koppelnavigation ist ungenauer als das.
HOME_TOLERANCE_M = 0.12

# Auf den letzten Zentimetern langsamer werden, sonst schießt er übers Ziel.
HOME_SLOWDOWN_M = 0.4

# Notbremse: nach dieser Zeit bricht die Heimfahrt ab. Schützt davor, dass
# der Roboter bei abgedrifteter Position endlos umherfährt.
HOME_TIMEOUT_S = 60.0

# Hält der Lidar etwas näher als das in Fahrtrichtung, wird angehalten.
# Der Roboter weicht nicht aus -- er bleibt stehen und meldet es.
HOME_STOP_DISTANCE_M = 0.45

# Halber Öffnungswinkel des überwachten Bereichs, in Grad. Zu schmal
# übersieht Tischbeine seitlich der Fahrlinie, zu breit hält wegen
# Wänden an, an denen man problemlos vorbeifährt.
HOME_CONE_DEG = 35.0

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
# 90° befohlen, 65° gedreht). Die Drehdauer wird damit verlängert, weil sich
# auf den IMU-Winkel der Bibliothek kein Verlass ist.
EXPLORE_TURN_FACTOR = 1.3

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
FOLGEN_KP_W = 1.5
FOLGEN_MAX_OMEGA = 0.8

# Innerhalb dieses Abstandsfehlers bleibt er stehen -- sonst ruckelt er
# bei jedem Atemzug der Person vor und zurück.
FOLGEN_TOTZONE_M = 0.10

# Erst geradeaus fahren, wenn die Person ungefähr vorn ist; sonst zuerst
# drehen (sonst fährt er schräg an ihr vorbei).
FOLGEN_MAX_WINKEL_FAHREN = 25.0

# So lange darf die Person aus dem Bild sein, bevor er sucht, und so lange
# sucht er (langsam drehen in die Richtung, in der sie verschwand).
FOLGEN_VERLOREN_S = 2.0
FOLGEN_SUCHEN_S = 6.0

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
