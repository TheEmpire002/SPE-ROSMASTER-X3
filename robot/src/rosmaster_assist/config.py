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

# Drehsinn der IMU. Die Software rechnet gegen den Uhrzeigersinn positiv;
# zählt die IMU andersherum, dreht sich jede Positionsschätzung nach einer
# Kurve in die falsche Richtung. 0 = automatisch erkennen (Vergleich mit den
# Radencodern während des Drehens), 1 oder -1 = fest vorgeben.
IMU_YAW_SIGN = 0

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

# Wo der Lidar relativ zum Drehpunkt des Roboters sitzt (Meter, x vorn,
# y links). Ein Versatz lässt den Lidar beim Drehen auf einem Kreis wandern
# -- bei 10 cm und 30° Drehung sind das 5 cm Fehler in der Ortung. Messen:
# Abstand Mitte Lidar-Kopf zur Mitte zwischen den vier Rädern.
LIDAR_MOUNT_X_M = 0.0
LIDAR_MOUNT_Y_M = 0.0

# --- Lidar-gestützte Ortung -----------------------------------------------------

# Unterwegs legt der Roboter eine Karte aus Aufnahmen an ("Schlüsselbilder"),
# jede mit der Position, an der sie entstand. Auf dem Rückweg wird gegen die
# nächstgelegenen abgeglichen -- so gibt es keine Reichweitengrenze mehr wie
# bei einer einzelnen Aufnahme vom Startpunkt.

# Neues Schlüsselbild, sobald der Roboter so weit vom nächsten entfernt ist.
KEYFRAME_ABSTAND_M = 0.6

# Ein Schlüsselbild wird nur aus einem sehr guten Abgleich übernommen --
# sonst wanderte der Fehler einer schlechten Schätzung fest in die Karte.
KEYFRAME_MAX_RESIDUAL_M = 0.06

# Obergrenze, damit der Speicher bei langen Fahrten nicht vollläuft.
KEYFRAME_MAX = 300

# Gegen Schlüsselbilder in diesem Umkreis um die geschätzte Position wird
# abgeglichen (die nächsten drei davon).
KEYFRAME_SUCHRADIUS_M = 3.0

# Mittlerer Restabstand nach dem Abgleich, bis zu dem das Ergebnis
# übernommen wird. Darüber gilt der Abgleich als misslungen. Lieber streng:
# ein falsch übernommener Abgleich verdreht alles, was danach kommt.
MATCH_MAX_RESIDUAL_M = 0.10

# Anteil der Messpunkte, die im Abgleich einen Partner finden müssen.
# Darunter passt die Aufnahme nicht wirklich zur Karte (anderer Raum,
# Menschen davor, falsches lokales Minimum).
MATCH_MIN_ANTEIL = 0.5

# Während schneller Drehungen nicht abgleichen: der Lidar braucht 0,18 s für
# eine Umdrehung, bei 1 rad/s ist die Aufnahme dabei um 10° verzerrt. In
# dieser Zeit trägt die IMU allein.
MATCH_MAX_OMEGA = 0.35  # rad/s

# Springt die Korrektur weiter als das, wird sie verworfen -- außer der
# Roboter hat die Spur verloren und sucht neu.
MATCH_MAX_SPRUNG_M = 0.8

# So lange ohne gelungenen Abgleich gilt die Spur als verloren. Dann wird
# mit mehreren Startwinkeln gesucht, weil die Odometrie inzwischen zu weit
# danebenliegen kann, als dass ein einzelner Versuch noch trifft.
MATCH_VERLOREN_S = 3.0

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

# Steht etwas im Weg, so lange warten, ob es verschwindet (Mensch läuft
# vorbei), bevor die Heimfahrt aufgibt.
HOME_HINDERNIS_WARTEN_S = 5.0

# --- Rückweg entlang der gefahrenen Strecke ---

# Abstand der Wegpunkte, die unterwegs aufgezeichnet werden.
RUECKWEG_PUNKTABSTAND_M = 0.25

# Kommt der Roboter auf der Hinfahrt wieder so nah an einen früheren
# Wegpunkt, wird die Schleife dazwischen gestrichen -- zurück geht es nicht
# alle Umwege noch einmal.
RUECKWEG_SCHLEIFE_M = 0.35

# Ein Wegpunkt gilt als erreicht in diesem Abstand.
RUECKWEG_ERREICHT_M = 0.2

# Abkürzen: ein näher am Start liegender Wegpunkt wird direkt angefahren,
# wenn er höchstens so weit weg ist und der Lidar freie Sicht meldet.
RUECKWEG_ABKUERZUNG_M = 3.0
# Halber Öffnungswinkel für die Sichtprüfung. Schmaler als HOME_CONE_DEG:
# hier geht es um "ist der Weg frei", nicht um "bremsen".
RUECKWEG_SICHT_DEG = 12.0

# --- Feinausrichtung am Ziel ---

# Nach Erreichen von HOME_TOLERANCE_M wird im Stand nachjustiert: anhalten,
# dem Lidar Zeit für ein paar ruhige Abgleiche geben, dann mit Schritttempo
# die Restabweichung wegfahren.
HOME_FEIN_TOLERANZ_M = 0.04
HOME_FEIN_TEMPO = 0.08       # m/s
HOME_FEIN_RUNDEN = 4

# Am Ende auf die Blickrichtung vom Startpunkt drehen. Ohne das steht er
# zwar richtig, schaut aber irgendwohin -- und die Kamera sieht den Marker
# nicht.
HOME_AUSRICHTEN = True
HOME_WINKEL_TOLERANZ_DEG = 2.0
HOME_DREH_TEMPO = 0.5        # rad/s, Höchstwert

# --- Andocken per Kamera-Marker (ArUco) ---

# Umschaltbar in der App. Aus = nur Lidar.
MARKER_ENABLED = False

# Marker aus dem Wörterbuch DICT_4X4_50. Druckvorlage: /marker/image
MARKER_ID = 0

# Gedruckte Kantenlänge des schwarzen Quadrats. Bestimmt nur den Maßstab:
# eine falsche Angabe macht das Andocken langsamer, nicht ungenauer, weil
# Referenz und aktuelle Messung mit demselben Wert rechnen.
MARKER_SIZE_M = 0.15

# Horizontaler Bildwinkel der Kamera, für die Umrechnung von Pixeln in
# Richtungen. Wie MARKER_SIZE_M: Referenz und Messung teilen den Fehler.
CAMERA_HFOV_DEG = 66.0

# Fertig, wenn Position und Winkel so nah an der Referenz sind.
MARKER_TOLERANZ_M = 0.02
MARKER_WINKEL_TOLERANZ_DEG = 1.5
MARKER_TEMPO = 0.08          # m/s, Höchstwert beim Andocken
MARKER_TIMEOUT_S = 25.0

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
