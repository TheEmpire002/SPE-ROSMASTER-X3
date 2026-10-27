"""Zentrale Einstellungen.

Alle Werte, die man beim Ausprobieren verstellen will, stehen hier -- nicht
verstreut im Code. Wer am Fahrverhalten schraubt, muss nur diese Datei öffnen.
"""

# --- Serielle Verbindung zum Expansion Board ---------------------------------

# Je nach Image heißt die Schnittstelle anders. Welche bei euch stimmt, zeigt
# `python scripts/check_hardware.py`. None = Rosmaster_Lib sucht selbst.
SERIAL_PORT: str | None = None

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

# --- Kamera ---------------------------------------------------------------------

CAMERA_ENABLED = True

# Welches Gerät bei mehreren Kameras (nur für USB-Kameras relevant).
CAMERA_INDEX = 0

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Bilder pro Sekunde im Stream. Höher kostet WLAN-Bandbreite und CPU; bei
# ruckelndem Bild über WLAN eher hier heruntergehen als die Auflösung senken.
CAMERA_FPS = 15

# JPEG-Qualität 1..100. Darunter wird das Bild matschig, darüber groß.
CAMERA_QUALITY = 70
