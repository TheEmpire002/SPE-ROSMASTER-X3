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
# Der X3 schafft rund 1 m/s. Wer langsamer üben will, dreht lieber den
# Tempo-Regler in der App herunter, statt diese Werte zu senken.
MAX_SPEED_X = 1.0      # m/s  vorwärts / rückwärts
MAX_SPEED_Y = 1.0      # m/s  seitwärts (Mecanum)
MAX_SPEED_OMEGA = 3.0  # rad/s  drehen

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
