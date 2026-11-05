"""Settings adjustable at runtime -- for the settings page.

The values themselves stay in ``config.py``, and the code keeps reading
them from there. This file only defines *which* of them the app may change,
within which limits and under which heading. Changes are made directly on
the ``config`` module -- because the code reads ``config.X`` anew on every
use, they usually take effect immediately.

Changes are stored in ``robot/settings.json`` so they survive a restart.
The file only contains what differs from the default; deleting it brings
back the values from ``config.py``. Deliberately a separate file instead of
rewriting ``config.py``: that is where the reasons live in the comments,
and those belong in Git, not in a runtime file.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from . import config

log = logging.getLogger(__name__)

# robot/src/rosmaster_assist/settings.py -> robot/settings.json
FILE = Path(__file__).resolve().parents[2] / "settings.json"


class SettingError(ValueError):
    """Invalid value -- the text goes to the app as is."""


@dataclass(frozen=True)
class Setting:
    key: str
    group: str
    label: str
    unit: str = ""
    min: float | None = None
    max: float | None = None
    step: float | None = None
    #: Choice instead of a slider.
    options: tuple | None = None
    help_text: str = ""
    #: Takes effect only after a server restart.
    restart: bool = False
    #: For values that do not map 1:1 to a config entry.
    read: Callable[[], Any] | None = None
    write: Callable[[Any], None] | None = None

    def value(self) -> Any:
        return self.read() if self.read else getattr(config, self.key)

    def apply(self, value: Any) -> None:
        if self.write:
            self.write(value)
        else:
            setattr(config, self.key, value)


def _read_resolution() -> str:
    return f"{config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}"


def _write_resolution(value: str) -> None:
    width, height = value.split("x")
    config.CAMERA_WIDTH, config.CAMERA_HEIGHT = int(width), int(height)


def _flip(role: str) -> Setting:
    """Mounting orientation of a camera -- lives in config.CAMERA_ROLES, not in a value of its own."""

    def read() -> str:
        return config.CAMERA_ROLES[role].get("flip") or "aus"

    def write(value: str) -> None:
        config.CAMERA_ROLES[role]["flip"] = None if value == "aus" else value

    return Setting(
        f"CAMERA_FLIP_{role.upper()}", "kamera",
        f"{config.CAMERA_ROLES[role].get('label', role)}: drehen/spiegeln",
        options=("aus", "horizontal", "vertikal", "180"),
        help_text="horizontal = links/rechts spiegeln, vertikal = oben/unten, 180 = auf dem Kopf. "
              "Gilt auch für die Personen- und Posenerkennung.",
        read=read, write=write,
    )


#: Order and headings of the blocks on the page.
GROUPS = [
    ("fahren", "Fahren & Motoren", "Obergrenzen und sanftes Anfahren für jeden Fahrbefehl, egal woher er kommt."),
    ("sicherheit", "Sicherheit", "Totmannschaltung und Hindernisstopp bei Handsteuerung."),
    ("kamera", "Kameras", "Gilt für alle Kameras gemeinsam."),
    ("personen", "Personenerkennung", "Sucht während des autonomen Fahrens im Bild der Personenkamera."),
    ("gesten", "Interaktion: Pose & Gesten", "Körperpose auf dem Hailo (Personenkamera), daraus die Handzeichen."),
    ("folgen", "Interaktion: Folgen, Herkommen, Weggehen", "Wie der Roboter auf die Handzeichen reagiert."),
    ("ton", "Ton & Lautstärke", "Stimme und Musik über den USB-Lautsprecher."),
    ("lidar", "Lidar", "RPLIDAR A1: Einbaulage und Messbereich."),
    ("ortung", "Ortung (Lidar-Abgleich)", "Korrigiert die Position durch Vergleich mit der Aufnahme am Startpunkt."),
    ("hindernisse", "Hinderniserkennung", "Benennt, was der Lidar sieht (Wand, Stuhl, Mensch …)."),
    ("autonom", "Autonomes Fahren", "Zufälliges Erkunden mit Ausweichen."),
    ("heimfahrt", "Heimfahrt", "Automatische Rückkehr zum Startpunkt."),
    ("diagnose", "Diagnose & Warnschwellen", "Ab wann die Diagnoseseite (/diagnose) warnt."),
    ("system", "System", "Server und Verbindung zum Board."),
]

SETTINGS: list[Setting] = [
    # --- Driving & motors
    Setting("MAX_SPEED_X", "fahren", "Höchsttempo vorwärts/rückwärts", "m/s", 0.05, 5.0, 0.05,
                help_text="Das Fahrgestell schafft rund 1 m/s, darüber kappt das Board selbst."),
    Setting("MAX_SPEED_Y", "fahren", "Höchsttempo seitwärts", "m/s", 0.05, 5.0, 0.05,
                help_text="Mecanum-Seitwärtsfahrt."),
    Setting("MAX_SPEED_OMEGA", "fahren", "Höchste Drehrate", "rad/s", 0.1, 5.0, 0.1,
                help_text="1 rad/s ≈ 57°/s."),
    # Smooth driving: the watchdog rereads RAMP_* on every tick, so this
    # takes effect immediately -- even in the middle of a drive.
    Setting("RAMP_ACTIVE", "fahren", "Sanft fahren (Anfahrrampe)",
                help_text="Fahrbefehle werden mit begrenzter Beschleunigung angefahren statt "
                      "sprunghaft. Schont den Akku und beugt USB-Aussetzern vor. Aus = jeder "
                      "Befehl wirkt sofort. Not-Stopp und Totmann halten immer sofort an."),
    Setting("RAMP_ACCEL_XY", "fahren", "Sanft fahren: Anfahren", "m/s²", 0.1, 5.0, 0.1,
                help_text="Kleiner = weicher, aber träger. 0,8 bringt ihn in knapp 0,4 s auf 0,3 m/s."),
    Setting("RAMP_BRAKE_XY", "fahren", "Sanft fahren: Bremsen", "m/s²", 0.5, 10.0, 0.1,
                help_text="Bremsen sollte schneller sein als Anfahren, sonst rollt er vor "
                      "Hindernissen weiter."),
    Setting("RAMP_ACCEL_OMEGA", "fahren", "Sanft fahren: Andrehen", "rad/s²", 0.5, 20.0, 0.5,
                help_text="Wie schnell eine Drehung anläuft. Drehen zieht am meisten Strom."),
    Setting("RAMP_BRAKE_OMEGA", "fahren", "Sanft fahren: Drehung stoppen", "rad/s²", 1.0, 30.0, 0.5),

    # --- Safety
    Setting("COMMAND_TIMEOUT_S", "sicherheit", "Totmannschaltung", "s", 0.2, 2.0, 0.05,
                help_text="Kommt so lange kein Fahrbefehl, hält der Roboter an. Zu kurz ruckelt, "
                      "zu lang fährt er nach einem WLAN-Abriss weiter."),
    Setting("COLLISION_ENABLED", "sicherheit", "Hindernisstopp beim Start an",
                help_text="Startzustand des Schalters in der App. Wird hier geändert, gilt es auch sofort."),
    Setting("COLLISION_DISTANCE_M", "sicherheit", "Hindernisstopp: Mindestabstand", "m",
                config.COLLISION_MIN_M, config.COLLISION_MAX_M, 0.05,
                help_text="Startwert des Reglers in der App. Wird hier geändert, gilt es auch sofort."),
    Setting("GUARD_MARGIN_M", "sicherheit", "Fahrschutz: Abstand beim selbstständigen Fahren",
                "m", 0.02, 0.3, 0.01,
                help_text="So viel Luft hält die Roboterkante beim Erkunden, Kartieren, bei der "
                      "Heimfahrt und beim Folgen mindestens. Gebremst wird vorher stufenlos. "
                      "Unter dem Rand der Bogenplanung (AUSWEICH_RAND_M, 5 cm) lassen, sonst "
                      "bremst er geplante Bögen ab."),

    # --- Cameras
    Setting("CAMERA_FPS", "kamera", "Bilder pro Sekunde", "fps", 1, config.CAMERA_MAX_FPS, 1,
                help_text=f"Die Kameras schaffen höchstens {config.CAMERA_MAX_FPS} fps, bei wenig Licht "
                      "weniger. Mehr kostet CPU und WLAN-Bandbreite. Was tatsächlich ankommt, "
                      "steht unten bei den Kameras."),
    Setting("CAMERA_RESOLUTION", "kamera", "Auflösung",
                options=("320x240", "640x480", "800x600", "1280x720"),
                help_text="Kann eine Kamera die Auflösung nicht, nimmt sie die nächstbeste. "
                      "Die Kameras werden dafür kurz neu geöffnet.",
                read=_read_resolution, write=_write_resolution),
    Setting("CAMERA_QUALITY", "kamera", "JPEG-Qualität", "%", 10, 95, 5,
                help_text="Höher = schärfer, aber größere Bilder übers WLAN."),
    *[_flip(role) for role in config.CAMERA_ROLES],
    Setting("CAMERA_ENABLED", "kamera", "Kameras eingeschaltet", restart=True),

    # --- Person detection
    Setting("PERSON_DETECTION_ENABLED", "personen", "Personenerkennung an"),
    Setting("PERSON_DETECT_HZ", "personen", "Erkennungen pro Sekunde", "Hz", 0.5, 5.0, 0.5,
                help_text="Ein Bild kostet auf dem Pi 0,1–0,2 s CPU. CPU-Lastsprünge lösen am X3 USB-Aussetzer aus."),
    Setting("PERSON_DETECT_WIDTH", "personen", "Bildbreite für die Erkennung", "px",
                options=(160, 240, 320, 480, 640),
                help_text="Kleiner = schneller, aber entfernte Personen werden übersehen."),
    Setting("PERSON_MIN_SCORE", "personen", "Mindestsicherheit", "", 0.0, 2.0, 0.1,
                help_text="Höher = weniger Fehlalarme, aber Personen von der Seite werden eher übersehen."),

    # --- Interaction: pose & gestures
    Setting("POSE_ENABLED", "gesten", "Posenerkennung (Hailo) an", restart=True),
    Setting("POSE_HZ", "gesten", "Posen pro Sekunde", "Hz", 2.0, 30.0, 1.0,
                help_text="Für Winken braucht es ~10, damit eine Auf-Ab-Bewegung mehrere Bilder umfasst. "
                      "Mehr als die Kamera liefert bringt nichts."),
    Setting("POSE_MIN_SCORE", "gesten", "Mindestsicherheit Person", "", 0.1, 0.95, 0.05),
    Setting("POSE_KP_MIN_CONF", "gesten", "Mindestsicherheit Körperpunkt", "", 0.1, 0.95, 0.05,
                help_text="Darunter gilt ein Punkt als verdeckt -- verdeckte Hände lösen keine Geste aus."),
    Setting("GESTURE_WINDOW_S", "gesten", "Rückblick für Bewegungen", "s", 0.5, 4.0, 0.1,
                help_text="So weit wird zurückgeschaut, um Winken und Wischen zu erkennen."),
    Setting("GESTURE_HOLD_S", "gesten", "„Folgen“ halten für", "s", 0.2, 3.0, 0.1,
                help_text="Ein Arm über dem Kopf. Zu kurz, und jedes Haare-Richten löst aus."),
    Setting("GESTURE_STOP_HOLD_S", "gesten", "„Stopp“ halten für", "s", 0.1, 2.0, 0.1,
                help_text="Beide Arme hoch. Bewusst kürzer, damit Stopp schnell wirkt."),
    Setting("GESTURE_SHARE", "gesten", "Anteil passender Bilder", "", 0.5, 1.0, 0.05,
                help_text="Nicht 100 %: einzelne Bilder mit verlorenem Handgelenk sind normal."),
    Setting("GESTURE_WAVE_REVERSALS", "gesten", "Winken: Richtungswechsel mindestens", "", 1, 6, 1),
    Setting("GESTURE_WAVE_AMPLITUDE", "gesten", "Winken: Mindesthub", "Schulterbr.", 0.1, 1.5, 0.05),
    Setting("GESTURE_SWIPE_DISTANCE", "gesten", "Wischen: Mindeststrecke", "Schulterbr.", 0.3, 3.0, 0.1),
    Setting("GESTURE_PAUSE_S", "gesten", "Pause nach einer Geste", "s", 0.5, 10.0, 0.5,
                help_text="Stopp wirkt auch in der Pause."),
    Setting("INTERACTION_JUMP_DEG", "gesten", "Personenwechsel ab Winkelsprung", "°", 5, 45, 1,
                help_text="Springt die verfolgte Person um mehr, ist es vermutlich eine andere."),

    # --- Interaction: follow, come here, go away
    Setting("FOLLOW_DISTANCE_M", "folgen", "Folgen: Wunschabstand", "m", 0.5, 3.0, 0.05),
    Setting("FOLLOW_MAX_SPEED", "folgen", "Folgen: Höchsttempo", "m/s", 0.05, 0.8, 0.01),
    Setting("FOLLOW_REVERSE_SPEED", "folgen", "Folgen: Tempo rückwärts", "m/s", 0.02, 0.3, 0.01,
                help_text="Hinten ist er weniger umsichtig unterwegs."),
    Setting("FOLLOW_MAX_OMEGA", "folgen", "Folgen: höchste Drehrate", "rad/s", 0.2, 2.0, 0.05),
    Setting("FOLLOW_KP_V", "folgen", "Regler Tempo", "(m/s)/m", 0.1, 2.0, 0.05,
                help_text="Tempo je Meter Abstandsfehler."),
    Setting("FOLLOW_KP_W", "folgen", "Regler Drehung", "(rad/s)/rad", 0.2, 4.0, 0.1,
                help_text="Drehrate je Radiant Winkelfehler."),
    Setting("FOLLOW_DEADBAND_M", "folgen", "Totzone", "m", 0.0, 0.5, 0.01,
                help_text="Innerhalb dieses Abstandsfehlers bleibt er stehen, statt bei jedem Atemzug zu ruckeln."),
    Setting("FOLLOW_MAX_ANGLE_DRIVE", "folgen", "Erst fahren, wenn Person innerhalb", "°", 5, 60, 1,
                help_text="Sonst zuerst drehen, statt schräg an ihr vorbeizufahren."),
    Setting("FOLLOW_LOST_S", "folgen", "Person verloren nach", "s", 0.5, 10.0, 0.5),
    Setting("FOLLOW_SEARCH_S", "folgen", "Suchen höchstens", "s", 0, 120, 1,
                help_text="0 = so lange, bis er wieder jemanden sieht."),
    Setting("COME_HERE_DISTANCE_M", "folgen", "Herkommen: bis auf", "m", 0.3, 2.0, 0.05),
    Setting("GO_AWAY_DISTANCE_M", "folgen", "Weggehen: Strecke", "m", 0.2, 3.0, 0.1),
    Setting("GO_AWAY_SPEED", "folgen", "Weggehen: Tempo", "m/s", 0.05, 0.4, 0.01),
    Setting("INTERACTION_BEHAVIOR_MAX_S", "folgen", "Herkommen/Weggehen höchstens", "s", 5, 120, 5),
    Setting("INTERACTION_STARTUP_S", "folgen", "Sanfter Start: aufwärmen", "s", 0, 15, 1,
                help_text="Beim Einschalten erst die Posenerkennung warmlaufen lassen, dann ansagen, "
                      "dann losfahren -- nicht alles in derselben Sekunde. Beugt USB-Aussetzern "
                      "vor. 0 = sofort."),

    # --- Sound
    Setting("VOLUME", "ton", "Gesamtlautstärke", "%", 0, 100, 5,
                help_text="Ein Regler für Stimme und Musik. Der Mixer des Lautsprechers bleibt auf "
                      "100 %, leiser wird in Software. Der Musik-Regler in der App ist ein Anteil "
                      "davon. Leiser zieht auch weniger Strom am USB."),

    Setting("TARGET_FEATURE_THRESHOLD", "folgen", "Ziel: Aussehen-Schwelle", "", 0.1, 0.9, 0.05,
                help_text="Kleiner = strenger. Verwechselt er Personen, verkleinern; verliert er dich ständig, vergrößern."),
    Setting("TARGET_GATE_DEG", "folgen", "Ziel: Suchfenster", "°", 5, 40, 1,
                help_text="So weit darf das Ziel von Bild zu Bild springen."),

    # --- Lidar
    Setting("LIDAR_ANGLE_OFFSET_DEG", "lidar", "Einbauwinkel", "°", -180, 180, 1,
                help_text="Hand vor den Roboter halten: Sie muss im Lidar-Bild oben erscheinen."),
    Setting("LIDAR_MIN_M", "lidar", "Mindestentfernung", "m", 0.05, 1.0, 0.01,
                help_text="Näheres wird verworfen, meist sind das Teile des Roboters."),
    Setting("LIDAR_MAX_M", "lidar", "Höchstentfernung", "m", 1.0, 12.0, 0.5,
                help_text="Weiteres wird als Rauschen verworfen."),
    Setting("LIDAR_ENABLED", "lidar", "Lidar eingeschaltet", restart=True),

    # --- Localization
    Setting("MATCH_HZ", "ortung", "Abgleiche pro Sekunde", "Hz", 0.5, 10.0, 0.5),
    Setting("MATCH_RADIUS_M", "ortung", "Abgleich nur im Umkreis", "m", 1.0, 10.0, 0.5,
                help_text="Weiter vom Startpunkt weg überlappt die Aufnahme zu wenig."),
    Setting("MATCH_MAX_RESIDUAL_M", "ortung", "Größte erlaubte Abweichung", "m", 0.05, 0.5, 0.01,
                help_text="Darüber gilt ein Abgleich als misslungen und wird verworfen."),
    Setting("MATCH_TRUST", "ortung", "Vertrauen in den Abgleich", "", 0.05, 1.0, 0.05,
                help_text="Wie stark eine Korrektur je Durchgang einfließt."),

    # --- Obstacle detection
    Setting("OBSTACLE_RANGE_M", "hindernisse", "Erkennungsumkreis", "m", 0.5, 6.0, 0.1),
    Setting("OBSTACLE_GAP_M", "hindernisse", "Objekt-Trennabstand", "m", 0.03, 0.3, 0.01,
                help_text="Springt der Abstand zweier Nachbarpunkte um mehr, beginnt ein neues Objekt."),
    Setting("OBSTACLE_LEG_MAX_M", "hindernisse", "Stuhl-/Tischbein bis", "m", 0.02, 0.15, 0.01),
    Setting("OBSTACLE_HUMAN_LEG_MAX_M", "hindernisse", "Menschenbein bis", "m", 0.1, 0.4, 0.01),
    Setting("OBSTACLE_WALL_MIN_M", "hindernisse", "Wand ab Länge", "m", 0.3, 2.0, 0.05),

    # --- Autonomous driving
    Setting("EXPLORE_SPEED", "autonom", "Reisetempo", "m/s", 0.05, 0.5, 0.01),
    Setting("EXPLORE_TURN_SPEED", "autonom", "Drehrate beim Wenden", "rad/s", 0.3, 2.0, 0.05,
                help_text="Unter 0,4 rad/s laufen die Motoren am Boden nicht zuverlässig an."),
    Setting("EXPLORE_STOP_M", "autonom", "Anhalten vor Hindernis", "m", 0.2, 1.0, 0.01,
                help_text="Gemessen ab Lidar-Mitte, der Roboter ragt ~0,15 m darüber hinaus."),
    Setting("EXPLORE_SLOW_M", "autonom", "Langsamer werden ab", "m", 0.3, 2.0, 0.05),
    Setting("EXPLORE_GO_M", "autonom", "Losfahren, wenn frei", "m", 0.3, 2.0, 0.05,
                help_text="Muss größer sein als „Anhalten vor Hindernis“, sonst pendelt er."),
    Setting("EXPLORE_HALF_WIDTH_M", "autonom", "Halbe Breite des Fahrschlauchs", "m", 0.12, 0.5, 0.01),
    Setting("EXPLORE_SIDE_M", "autonom", "Seitlich ausweichen ab", "m", 0.1, 0.8, 0.01),
    Setting("EXPLORE_NEAR_M", "autonom", "Stehen bleiben, wenn rundum näher als", "m", 0.1, 0.6, 0.01),
    Setting("EXPLORE_WANDER_MIN_S", "autonom", "Richtungswechsel frühestens nach", "s", 1, 60, 1),
    Setting("EXPLORE_WANDER_MAX_S", "autonom", "Richtungswechsel spätestens nach", "s", 1, 120, 1),
    Setting("EXPLORE_TURN_FACTOR", "autonom", "Drehausgleich (Schlupf)", "×", 1.0, 2.0, 0.05,
                help_text="Am Boden dreht er nur ~75 % des Befohlenen; die Drehdauer wird damit verlängert."),
    Setting("EXPLORE_PERSON_WAIT_S", "autonom", "Vor Mensch warten", "s", 0, 10, 0.5),
    Setting("EXPLORE_TIMEOUT_S", "autonom", "Höchstdauer (0 = unbegrenzt)", "s", 0, 3600, 30),

    # --- Homing
    Setting("HOME_SPEED", "heimfahrt", "Tempo", "m/s", 0.05, 0.6, 0.01,
                help_text="Ohne Ausweichen unterwegs, deshalb gemächlich."),
    Setting("HOME_TOLERANCE_M", "heimfahrt", "Angekommen bei", "m", 0.05, 0.5, 0.01),
    Setting("HOME_SLOWDOWN_M", "heimfahrt", "Abbremsen ab", "m", 0.1, 1.5, 0.05),
    Setting("HOME_STOP_DISTANCE_M", "heimfahrt", "Anhalten vor Hindernis", "m", 0.2, 1.5, 0.05),
    Setting("HOME_CONE_DEG", "heimfahrt", "Überwachter halber Winkel", "°", 10, 90, 1),
    Setting("HOME_TIMEOUT_S", "heimfahrt", "Abbruch nach", "s", 10, 600, 10,
                help_text="Gilt ab der nächsten Heimfahrt."),

    # --- Diagnostics & warning thresholds
    Setting("BATTERY_WARN_V", "diagnose", "Akku schwach unter", "V", 10.0, 12.6, 0.1,
                help_text="3S-Akku: voll 12,6 V. Darunter reicht der Anlaufstrom der Motoren, "
                      "um die USB-Geräte kurz abstürzen zu lassen."),
    Setting("BATTERY_CRITICAL_V", "diagnose", "Akku fast leer unter", "V", 9.5, 12.0, 0.1,
                help_text="Spätestens jetzt laden -- tiefer entladen schadet dem Akku."),
    Setting("VOLTAGE_DROP_V", "diagnose", "Spannungseinbruch ab", "V", 0.2, 2.0, 0.1,
                help_text="So weit unter dem Höchstwert der letzten Minute zählt als Einbruch."),
    Setting("TEMP_WARN_C", "diagnose", "Pi zu warm ab", "°C", 50, 85, 1,
                help_text="Der Pi 5 drosselt selbst ab 85 °C."),

    # --- System
    Setting("STATUS_INTERVAL_S", "system", "Statusmeldungen alle", "s", 0.1, 2.0, 0.1),
    Setting("TELEMETRY_ENABLED", "system", "Rückkanal vom Board (Akku, IMU)", restart=True,
                help_text="Abschalten, wenn ein anderes Programm am selben Port liest."),
]

_BY_KEY = {e.key: e for e in SETTINGS}

#: Setting names before the code was translated to English (2026-11-04).
#: A settings.json saved on the robot still uses them; load() maps them to
#: the new names so saved settings survive the update.
_OLD_NAMES = {
    "AKKU_KRITISCH_V": "BATTERY_CRITICAL_V",
    "AKKU_WARN_V": "BATTERY_WARN_V",
    "EXPLORE_NAH_M": "EXPLORE_NEAR_M",
    "FAHRSCHUTZ_RAND_M": "GUARD_MARGIN_M",
    "FOLGEN_ABSTAND_M": "FOLLOW_DISTANCE_M",
    "FOLGEN_KP_V": "FOLLOW_KP_V",
    "FOLGEN_KP_W": "FOLLOW_KP_W",
    "FOLGEN_MAX_OMEGA": "FOLLOW_MAX_OMEGA",
    "FOLGEN_MAX_SPEED": "FOLLOW_MAX_SPEED",
    "FOLGEN_MAX_WINKEL_FAHREN": "FOLLOW_MAX_ANGLE_DRIVE",
    "FOLGEN_RUECK_SPEED": "FOLLOW_REVERSE_SPEED",
    "FOLGEN_SUCHEN_S": "FOLLOW_SEARCH_S",
    "FOLGEN_TOTZONE_M": "FOLLOW_DEADBAND_M",
    "FOLGEN_VERLOREN_S": "FOLLOW_LOST_S",
    "GESTE_ANTEIL": "GESTURE_SHARE",
    "GESTE_FENSTER_S": "GESTURE_WINDOW_S",
    "GESTE_HALTEN_S": "GESTURE_HOLD_S",
    "GESTE_PAUSE_S": "GESTURE_PAUSE_S",
    "GESTE_STOPP_HALTEN_S": "GESTURE_STOP_HOLD_S",
    "GESTE_WINK_HUB": "GESTURE_WAVE_AMPLITUDE",
    "GESTE_WINK_WECHSEL": "GESTURE_WAVE_REVERSALS",
    "GESTE_WISCH_STRECKE": "GESTURE_SWIPE_DISTANCE",
    "HERKOMMEN_ABSTAND_M": "COME_HERE_DISTANCE_M",
    "INTERAKTION_ANLAUF_S": "INTERACTION_STARTUP_S",
    "INTERAKTION_SPRUNG_GRAD": "INTERACTION_JUMP_DEG",
    "INTERAKTION_VERHALTEN_MAX_S": "INTERACTION_BEHAVIOR_MAX_S",
    "LAUTSTAERKE": "VOLUME",
    "RAMPE_AKTIV": "RAMP_ACTIVE",
    "RAMPE_ANFAHREN_OMEGA": "RAMP_ACCEL_OMEGA",
    "RAMPE_ANFAHREN_XY": "RAMP_ACCEL_XY",
    "RAMPE_BREMSEN_OMEGA": "RAMP_BRAKE_OMEGA",
    "RAMPE_BREMSEN_XY": "RAMP_BRAKE_XY",
    "SPANNUNGSEINBRUCH_V": "VOLTAGE_DROP_V",
    "WEGGEHEN_SPEED": "GO_AWAY_SPEED",
    "WEGGEHEN_STRECKE_M": "GO_AWAY_DISTANCE_M",
    "ZIEL_GATE_GRAD": "TARGET_GATE_DEG",
    "ZIEL_MERKMAL_SCHWELLE": "TARGET_FEATURE_THRESHOLD",
}

#: Default values from config.py, captured before settings.json overwrites them.
_DEFAULTS = {e.key: e.value() for e in SETTINGS}

#: Pairs that must fit together: (smaller, larger, reason).
_RULES = [
    ("LIDAR_MIN_M", "LIDAR_MAX_M", "Mindestentfernung muss unter der Höchstentfernung liegen"),
    ("EXPLORE_STOP_M", "EXPLORE_GO_M", "„Losfahren, wenn frei“ muss größer sein als „Anhalten“ -- sonst pendelt er"),
    ("EXPLORE_STOP_M", "EXPLORE_SLOW_M", "„Langsamer werden ab“ muss größer sein als „Anhalten“"),
    ("EXPLORE_WANDER_MIN_S", "EXPLORE_WANDER_MAX_S", "frühester Richtungswechsel muss vor dem spätesten liegen"),
    ("HOME_TOLERANCE_M", "HOME_SLOWDOWN_M", "„Abbremsen ab“ muss größer sein als „Angekommen bei“"),
    ("OBSTACLE_LEG_MAX_M", "OBSTACLE_HUMAN_LEG_MAX_M", "Stuhlbein muss schmaler sein als Menschenbein"),
    # Older frames drop out of the look-back window -- a longer hold time could never be met.
    ("GESTURE_HOLD_S", "GESTURE_WINDOW_S", "„Folgen“ halten muss kürzer sein als der Rückblick"),
    ("GESTURE_STOP_HOLD_S", "GESTURE_WINDOW_S", "„Stopp“ halten muss kürzer sein als der Rückblick"),
    ("FOLLOW_DEADBAND_M", "FOLLOW_DISTANCE_M", "Totzone muss kleiner sein als der Wunschabstand"),
    ("BATTERY_CRITICAL_V", "BATTERY_WARN_V", "„Akku fast leer“ muss unter „Akku schwach“ liegen"),
]

_lock = threading.Lock()


def _check(e: Setting, value: Any) -> Any:
    """Converts to the right type and checks the limits."""
    default = _DEFAULTS[e.key]

    if e.options is not None:
        for option in e.options:
            if str(option) == str(value):
                return option
        raise SettingError(f"{e.label}: erlaubt sind {', '.join(map(str, e.options))}")

    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise SettingError(f"{e.label}: an oder aus erwartet")
        return value

    # bool is also a number in Python -- but here it is surely a mistake.
    if isinstance(value, bool):
        raise SettingError(f"{e.label}: Zahl erwartet")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise SettingError(f"{e.label}: Zahl erwartet") from None
    if not math.isfinite(number):
        raise SettingError(f"{e.label}: Zahl erwartet")
    if (e.min is not None and number < e.min) or (e.max is not None and number > e.max):
        raise SettingError(f"{e.label}: erlaubt sind {e.min} bis {e.max} {e.unit}".rstrip())
    return int(round(number)) if isinstance(default, int) else number


def _check_rules(values: dict[str, Any]) -> None:
    for smaller, larger, text in _RULES:
        if values[smaller] >= values[larger]:
            raise SettingError(text)


def _save() -> None:
    differing = {
        e.key: e.value() for e in SETTINGS if e.value() != _DEFAULTS[e.key]
    }
    # First into a neighbouring file, then rename: if power is lost while
    # writing (battery empty), the old file stays intact instead of half-written.
    tmp = FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(differing, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, FILE)


def load() -> None:
    """Apply saved changes. Call once at startup.

    Must run before the services are created: some read their values only
    at startup (e.g. the initial state of the obstacle stop).
    """
    if not FILE.exists():
        return
    try:
        saved = json.loads(FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        log.warning("Settings in %s unreadable, using defaults -- %s", FILE, exc)
        return

    with _lock:
        for key, value in saved.items():
            key = _OLD_NAMES.get(key, key)
            e = _BY_KEY.get(key)
            if e is None:
                log.warning("Setting %s no longer exists -- skipped", key)
                continue
            try:
                e.apply(_check(e, value))
            except SettingError as exc:
                log.warning("Saved setting discarded -- %s", exc)
        try:
            _check_rules({x.key: x.value() for x in SETTINGS})
        except SettingError as exc:
            # Better everything back to default than a robot that oscillates.
            log.warning("Saved settings contradictory (%s) -- using defaults", exc)
            for x in SETTINGS:
                x.apply(_DEFAULTS[x.key])
            return

    changed = [e.key for e in SETTINGS if e.value() != _DEFAULTS[e.key]]
    if changed:
        log.info("Settings from %s: %s", FILE.name, ", ".join(changed))


def entries() -> dict[str, Any]:
    """All settings by block, with value, default and limits."""
    groups = []
    for gid, title, text in GROUPS:
        rows = []
        for e in SETTINGS:
            if e.group != gid:
                continue
            default = _DEFAULTS[e.key]
            if e.options is not None:
                kind = "auswahl"
            elif isinstance(default, bool):
                kind = "bool"
            elif isinstance(default, int):
                kind = "int"
            else:
                kind = "float"
            rows.append({
                "key": e.key,
                "label": e.label,
                "typ": kind,
                "wert": e.value(),
                "standard": default,
                "min": e.min,
                "max": e.max,
                "schritt": e.step,
                "einheit": e.unit,
                "optionen": list(e.options) if e.options is not None else None,
                "hilfe": e.help_text,
                "neustart": e.restart,
            })
        groups.append({"id": gid, "titel": title, "text": text, "einstellungen": rows})
    return {"gruppen": groups}


def apply(new: dict[str, Any]) -> list[str]:
    """Applies several values -- all or none. Returns the changed ones.

    Check everything first, then set: otherwise an invalid value would
    leave half of them changed.
    """
    with _lock:
        checked = {}
        for key, value in new.items():
            e = _BY_KEY.get(key)
            if e is None:
                raise SettingError(f"Unbekannte Einstellung {key!r}")
            checked[key] = _check(e, value)

        _check_rules({e.key: checked.get(e.key, e.value()) for e in SETTINGS})

        changed = [k for k, v in checked.items() if _BY_KEY[k].value() != v]
        for key in changed:
            _BY_KEY[key].apply(checked[key])
        if changed:
            try:
                _save()
            except OSError as exc:
                # The value applies anyway -- just not beyond a restart.
                # No reason to reject the change.
                log.warning("Settings not saved (%s) -- valid until restart", exc)

    for key in changed:
        log.info("Setting %s = %r", key, checked[key])
    return changed


def restart_needed(keys: list[str]) -> list[str]:
    """Which of the settings take effect only after a server restart."""
    return [k for k in keys if k in _BY_KEY and _BY_KEY[k].restart]


def reset(keys: list[str] | None = None) -> list[str]:
    """Back to the values from config.py -- single ones or all."""
    keys = [e.key for e in SETTINGS] if keys is None else keys
    return apply({k: _DEFAULTS[k] for k in keys if k in _BY_KEY})
