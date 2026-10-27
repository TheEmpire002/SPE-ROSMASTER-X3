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

# Welcher /dev/videoN-Index bei einer USB-Kamera zuerst probiert wird.
# Scheitert er, werden alle vorhandenen Geräte durchprobiert -- auf dem Pi
# ist /dev/video0 oft ein Encoder und nicht die Kamera.
CAMERA_INDEX = 0

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Bilder pro Sekunde im Stream. Höher kostet WLAN-Bandbreite und CPU; bei
# ruckelndem Bild über WLAN eher hier heruntergehen als die Auflösung senken.
CAMERA_FPS = 15

# JPEG-Qualität 1..100. Darunter wird das Bild matschig, darüber groß.
CAMERA_QUALITY = 70
