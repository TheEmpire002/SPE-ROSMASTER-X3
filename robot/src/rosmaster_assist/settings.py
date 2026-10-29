"""Zur Laufzeit verstellbare Einstellungen -- für die Einstellungsseite.

Die Werte selbst bleiben in ``config.py``, und der Code liest sie weiter von
dort. Diese Datei legt nur fest, *welche* davon die App verstellen darf, in
welchen Grenzen und unter welcher Überschrift. Geändert wird direkt am
Modul ``config`` -- weil der Code ``config.X`` bei jeder Verwendung neu
liest, wirkt das meist sofort.

Änderungen landen in ``robot/settings.json``, damit sie einen Neustart
überstehen. Die Datei enthält nur, was vom Standard abweicht; wer sie
löscht, hat wieder die Werte aus ``config.py``. Bewusst eine eigene Datei
statt ``config.py`` umzuschreiben: dort stehen die Begründungen in den
Kommentaren, und die gehören in Git, nicht in eine Laufzeitdatei.
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
DATEI = Path(__file__).resolve().parents[2] / "settings.json"


class EinstellungsFehler(ValueError):
    """Ungültiger Wert -- der Text geht so an die App."""


@dataclass(frozen=True)
class Einstellung:
    key: str
    gruppe: str
    label: str
    einheit: str = ""
    min: float | None = None
    max: float | None = None
    schritt: float | None = None
    #: Auswahl statt Regler.
    optionen: tuple | None = None
    hilfe: str = ""
    #: Wirkt erst nach einem Neustart des Servers.
    neustart: bool = False
    #: Für Werte, die nicht 1:1 einem config-Eintrag entsprechen.
    lesen: Callable[[], Any] | None = None
    schreiben: Callable[[Any], None] | None = None

    def wert(self) -> Any:
        return self.lesen() if self.lesen else getattr(config, self.key)

    def setzen(self, wert: Any) -> None:
        if self.schreiben:
            self.schreiben(wert)
        else:
            setattr(config, self.key, wert)


def _aufloesung_lesen() -> str:
    return f"{config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}"


def _aufloesung_schreiben(wert: str) -> None:
    breite, hoehe = wert.split("x")
    config.CAMERA_WIDTH, config.CAMERA_HEIGHT = int(breite), int(hoehe)


#: Reihenfolge und Überschriften der Blöcke auf der Seite.
GRUPPEN = [
    ("fahren", "Fahren & Motoren", "Obergrenzen für jeden Fahrbefehl, egal woher er kommt."),
    ("sicherheit", "Sicherheit", "Totmannschaltung und Hindernisstopp bei Handsteuerung."),
    ("kamera", "Kameras", "Gilt für alle Kameras gemeinsam."),
    ("personen", "Personenerkennung", "Sucht während des autonomen Fahrens im Bild der Personenkamera."),
    ("lidar", "Lidar", "RPLIDAR A1: Einbaulage und Messbereich."),
    ("ortung", "Ortung (Lidar-Abgleich)", "Korrigiert die Position durch Vergleich mit der Aufnahme am Startpunkt."),
    ("hindernisse", "Hinderniserkennung", "Benennt, was der Lidar sieht (Wand, Stuhl, Mensch …)."),
    ("autonom", "Autonomes Fahren", "Zufälliges Erkunden mit Ausweichen."),
    ("heimfahrt", "Heimfahrt", "Automatische Rückkehr zum Startpunkt."),
    ("system", "System", "Server und Verbindung zum Board."),
]

EINSTELLUNGEN: list[Einstellung] = [
    # --- Fahren & Motoren
    Einstellung("MAX_SPEED_X", "fahren", "Höchsttempo vorwärts/rückwärts", "m/s", 0.05, 5.0, 0.05,
                hilfe="Das Fahrgestell schafft rund 1 m/s, darüber kappt das Board selbst."),
    Einstellung("MAX_SPEED_Y", "fahren", "Höchsttempo seitwärts", "m/s", 0.05, 5.0, 0.05,
                hilfe="Mecanum-Seitwärtsfahrt."),
    Einstellung("MAX_SPEED_OMEGA", "fahren", "Höchste Drehrate", "rad/s", 0.1, 5.0, 0.1,
                hilfe="1 rad/s ≈ 57°/s."),

    # --- Sicherheit
    Einstellung("COMMAND_TIMEOUT_S", "sicherheit", "Totmannschaltung", "s", 0.2, 2.0, 0.05,
                hilfe="Kommt so lange kein Fahrbefehl, hält der Roboter an. Zu kurz ruckelt, "
                      "zu lang fährt er nach einem WLAN-Abriss weiter."),
    Einstellung("COLLISION_ENABLED", "sicherheit", "Hindernisstopp beim Start an",
                hilfe="Startzustand des Schalters in der App. Wird hier geändert, gilt es auch sofort."),
    Einstellung("COLLISION_DISTANCE_M", "sicherheit", "Hindernisstopp: Mindestabstand", "m",
                config.COLLISION_MIN_M, config.COLLISION_MAX_M, 0.05,
                hilfe="Startwert des Reglers in der App. Wird hier geändert, gilt es auch sofort."),
    Einstellung("COLLISION_CONE_DEG", "sicherheit", "Hindernisstopp: halber Öffnungswinkel", "°", 10, 90, 1,
                hilfe="Zu schmal übersieht Tischbeine, zu breit hält vor jeder Wand."),

    # --- Kameras
    Einstellung("CAMERA_FPS", "kamera", "Bilder pro Sekunde", "fps", 1, config.CAMERA_MAX_FPS, 1,
                hilfe=f"Die Kameras schaffen höchstens {config.CAMERA_MAX_FPS} fps, bei wenig Licht "
                      "weniger. Mehr kostet CPU und WLAN-Bandbreite. Was tatsächlich ankommt, "
                      "steht unten bei den Kameras."),
    Einstellung("CAMERA_RESOLUTION", "kamera", "Auflösung",
                optionen=("320x240", "640x480", "800x600", "1280x720"),
                hilfe="Kann eine Kamera die Auflösung nicht, nimmt sie die nächstbeste. "
                      "Die Kameras werden dafür kurz neu geöffnet.",
                lesen=_aufloesung_lesen, schreiben=_aufloesung_schreiben),
    Einstellung("CAMERA_QUALITY", "kamera", "JPEG-Qualität", "%", 10, 95, 5,
                hilfe="Höher = schärfer, aber größere Bilder übers WLAN."),
    Einstellung("CAMERA_ENABLED", "kamera", "Kameras eingeschaltet", neustart=True),

    # --- Personenerkennung
    Einstellung("PERSON_DETECTION_ENABLED", "personen", "Personenerkennung an"),
    Einstellung("PERSON_DETECT_HZ", "personen", "Erkennungen pro Sekunde", "Hz", 0.5, 5.0, 0.5,
                hilfe="Ein Bild kostet auf dem Pi 0,1–0,2 s CPU. CPU-Lastsprünge lösen am X3 USB-Aussetzer aus."),
    Einstellung("PERSON_DETECT_WIDTH", "personen", "Bildbreite für die Erkennung", "px",
                optionen=(160, 240, 320, 480, 640),
                hilfe="Kleiner = schneller, aber entfernte Personen werden übersehen."),
    Einstellung("PERSON_MIN_SCORE", "personen", "Mindestsicherheit", "", 0.0, 2.0, 0.1,
                hilfe="Höher = weniger Fehlalarme, aber Personen von der Seite werden eher übersehen."),

    # --- Lidar
    Einstellung("LIDAR_ANGLE_OFFSET_DEG", "lidar", "Einbauwinkel", "°", -180, 180, 1,
                hilfe="Hand vor den Roboter halten: Sie muss im Lidar-Bild oben erscheinen."),
    Einstellung("LIDAR_MIN_M", "lidar", "Mindestentfernung", "m", 0.05, 1.0, 0.01,
                hilfe="Näheres wird verworfen, meist sind das Teile des Roboters."),
    Einstellung("LIDAR_MAX_M", "lidar", "Höchstentfernung", "m", 1.0, 12.0, 0.5,
                hilfe="Weiteres wird als Rauschen verworfen."),
    Einstellung("LIDAR_ENABLED", "lidar", "Lidar eingeschaltet", neustart=True),

    # --- Ortung
    Einstellung("MATCH_HZ", "ortung", "Abgleiche pro Sekunde", "Hz", 0.5, 10.0, 0.5),
    Einstellung("MATCH_RADIUS_M", "ortung", "Abgleich nur im Umkreis", "m", 1.0, 10.0, 0.5,
                hilfe="Weiter vom Startpunkt weg überlappt die Aufnahme zu wenig."),
    Einstellung("MATCH_MAX_RESIDUAL_M", "ortung", "Größte erlaubte Abweichung", "m", 0.05, 0.5, 0.01,
                hilfe="Darüber gilt ein Abgleich als misslungen und wird verworfen."),
    Einstellung("MATCH_TRUST", "ortung", "Vertrauen in den Abgleich", "", 0.05, 1.0, 0.05,
                hilfe="Wie stark eine Korrektur je Durchgang einfließt."),

    # --- Hinderniserkennung
    Einstellung("OBSTACLE_RANGE_M", "hindernisse", "Erkennungsumkreis", "m", 0.5, 6.0, 0.1),
    Einstellung("OBSTACLE_GAP_M", "hindernisse", "Objekt-Trennabstand", "m", 0.03, 0.3, 0.01,
                hilfe="Springt der Abstand zweier Nachbarpunkte um mehr, beginnt ein neues Objekt."),
    Einstellung("OBSTACLE_LEG_MAX_M", "hindernisse", "Stuhl-/Tischbein bis", "m", 0.02, 0.15, 0.01),
    Einstellung("OBSTACLE_HUMAN_LEG_MAX_M", "hindernisse", "Menschenbein bis", "m", 0.1, 0.4, 0.01),
    Einstellung("OBSTACLE_WALL_MIN_M", "hindernisse", "Wand ab Länge", "m", 0.3, 2.0, 0.05),

    # --- Autonomes Fahren
    Einstellung("EXPLORE_SPEED", "autonom", "Reisetempo", "m/s", 0.05, 0.5, 0.01),
    Einstellung("EXPLORE_TURN_SPEED", "autonom", "Drehrate beim Wenden", "rad/s", 0.3, 2.0, 0.05,
                hilfe="Unter 0,4 rad/s laufen die Motoren am Boden nicht zuverlässig an."),
    Einstellung("EXPLORE_STOP_M", "autonom", "Anhalten vor Hindernis", "m", 0.2, 1.0, 0.01,
                hilfe="Gemessen ab Lidar-Mitte, der Roboter ragt ~0,15 m darüber hinaus."),
    Einstellung("EXPLORE_SLOW_M", "autonom", "Langsamer werden ab", "m", 0.3, 2.0, 0.05),
    Einstellung("EXPLORE_GO_M", "autonom", "Losfahren, wenn frei", "m", 0.3, 2.0, 0.05,
                hilfe="Muss größer sein als „Anhalten vor Hindernis“, sonst pendelt er."),
    Einstellung("EXPLORE_HALF_WIDTH_M", "autonom", "Halbe Breite des Fahrschlauchs", "m", 0.12, 0.5, 0.01),
    Einstellung("EXPLORE_SIDE_M", "autonom", "Seitlich ausweichen ab", "m", 0.1, 0.8, 0.01),
    Einstellung("EXPLORE_NAH_M", "autonom", "Stehen bleiben, wenn rundum näher als", "m", 0.1, 0.6, 0.01),
    Einstellung("EXPLORE_WANDER_MIN_S", "autonom", "Richtungswechsel frühestens nach", "s", 1, 60, 1),
    Einstellung("EXPLORE_WANDER_MAX_S", "autonom", "Richtungswechsel spätestens nach", "s", 1, 120, 1),
    Einstellung("EXPLORE_TURN_FACTOR", "autonom", "Drehausgleich (Schlupf)", "×", 1.0, 2.0, 0.05,
                hilfe="Am Boden dreht er nur ~75 % des Befohlenen; die Drehdauer wird damit verlängert."),
    Einstellung("EXPLORE_PERSON_WAIT_S", "autonom", "Vor Mensch warten", "s", 0, 10, 0.5),
    Einstellung("EXPLORE_TIMEOUT_S", "autonom", "Höchstdauer (0 = unbegrenzt)", "s", 0, 3600, 30),

    # --- Heimfahrt
    Einstellung("HOME_SPEED", "heimfahrt", "Tempo", "m/s", 0.05, 0.6, 0.01,
                hilfe="Ohne Ausweichen unterwegs, deshalb gemächlich."),
    Einstellung("HOME_TOLERANCE_M", "heimfahrt", "Angekommen bei", "m", 0.05, 0.5, 0.01),
    Einstellung("HOME_SLOWDOWN_M", "heimfahrt", "Abbremsen ab", "m", 0.1, 1.5, 0.05),
    Einstellung("HOME_STOP_DISTANCE_M", "heimfahrt", "Anhalten vor Hindernis", "m", 0.2, 1.5, 0.05),
    Einstellung("HOME_CONE_DEG", "heimfahrt", "Überwachter halber Winkel", "°", 10, 90, 1),
    Einstellung("HOME_TIMEOUT_S", "heimfahrt", "Abbruch nach", "s", 10, 600, 10,
                hilfe="Gilt ab der nächsten Heimfahrt."),

    # --- System
    Einstellung("STATUS_INTERVAL_S", "system", "Statusmeldungen alle", "s", 0.1, 2.0, 0.1),
    Einstellung("TELEMETRY_ENABLED", "system", "Rückkanal vom Board (Akku, IMU)", neustart=True,
                hilfe="Abschalten, wenn ein anderes Programm am selben Port liest."),
]

_NACH_KEY = {e.key: e for e in EINSTELLUNGEN}

#: Standardwerte aus config.py, festgehalten bevor settings.json sie überschreibt.
_STANDARD = {e.key: e.wert() for e in EINSTELLUNGEN}

#: Paare, die zueinander passen müssen: (kleiner, größer, Begründung).
_REGELN = [
    ("LIDAR_MIN_M", "LIDAR_MAX_M", "Mindestentfernung muss unter der Höchstentfernung liegen"),
    ("EXPLORE_STOP_M", "EXPLORE_GO_M", "„Losfahren, wenn frei“ muss größer sein als „Anhalten“ -- sonst pendelt er"),
    ("EXPLORE_STOP_M", "EXPLORE_SLOW_M", "„Langsamer werden ab“ muss größer sein als „Anhalten“"),
    ("EXPLORE_WANDER_MIN_S", "EXPLORE_WANDER_MAX_S", "frühester Richtungswechsel muss vor dem spätesten liegen"),
    ("HOME_TOLERANCE_M", "HOME_SLOWDOWN_M", "„Abbremsen ab“ muss größer sein als „Angekommen bei“"),
    ("OBSTACLE_LEG_MAX_M", "OBSTACLE_HUMAN_LEG_MAX_M", "Stuhlbein muss schmaler sein als Menschenbein"),
]

_lock = threading.Lock()


def _pruefen(e: Einstellung, wert: Any) -> Any:
    """Wandelt in den richtigen Typ und prüft die Grenzen."""
    standard = _STANDARD[e.key]

    if e.optionen is not None:
        for option in e.optionen:
            if str(option) == str(wert):
                return option
        raise EinstellungsFehler(f"{e.label}: erlaubt sind {', '.join(map(str, e.optionen))}")

    if isinstance(standard, bool):
        if not isinstance(wert, bool):
            raise EinstellungsFehler(f"{e.label}: an oder aus erwartet")
        return wert

    # bool ist in Python auch eine Zahl -- hier aber sicher ein Versehen.
    if isinstance(wert, bool):
        raise EinstellungsFehler(f"{e.label}: Zahl erwartet")
    try:
        zahl = float(wert)
    except (TypeError, ValueError):
        raise EinstellungsFehler(f"{e.label}: Zahl erwartet") from None
    if not math.isfinite(zahl):
        raise EinstellungsFehler(f"{e.label}: Zahl erwartet")
    if (e.min is not None and zahl < e.min) or (e.max is not None and zahl > e.max):
        raise EinstellungsFehler(f"{e.label}: erlaubt sind {e.min} bis {e.max} {e.einheit}".rstrip())
    return int(round(zahl)) if isinstance(standard, int) else zahl


def _regeln_pruefen(werte: dict[str, Any]) -> None:
    for kleiner, groesser, text in _REGELN:
        if werte[kleiner] >= werte[groesser]:
            raise EinstellungsFehler(text)


def _speichern() -> None:
    abweichend = {
        e.key: e.wert() for e in EINSTELLUNGEN if e.wert() != _STANDARD[e.key]
    }
    # Erst in eine Nachbardatei, dann umbenennen: reißt beim Schreiben der
    # Strom ab (Akku leer), bleibt die alte Datei heil statt halb.
    tmp = DATEI.with_suffix(".tmp")
    tmp.write_text(json.dumps(abweichend, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, DATEI)


def laden() -> None:
    """Gespeicherte Änderungen übernehmen. Einmal beim Start aufrufen.

    Muss vor dem Anlegen der Dienste laufen: einige lesen ihre Werte nur
    beim Start (z. B. der Startzustand des Hindernisstopps).
    """
    if not DATEI.exists():
        return
    try:
        gespeichert = json.loads(DATEI.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        log.warning("Einstellungen in %s unlesbar, nehme Standardwerte -- %s", DATEI, exc)
        return

    with _lock:
        for key, wert in gespeichert.items():
            e = _NACH_KEY.get(key)
            if e is None:
                log.warning("Einstellung %s gibt es nicht mehr -- übersprungen", key)
                continue
            try:
                e.setzen(_pruefen(e, wert))
            except EinstellungsFehler as exc:
                log.warning("Gespeicherte Einstellung verworfen -- %s", exc)
        try:
            _regeln_pruefen({x.key: x.wert() for x in EINSTELLUNGEN})
        except EinstellungsFehler as exc:
            # Lieber alles zurück auf Standard als ein Roboter, der pendelt.
            log.warning("Gespeicherte Einstellungen widersprüchlich (%s) -- nehme Standardwerte", exc)
            for x in EINSTELLUNGEN:
                x.setzen(_STANDARD[x.key])
            return

    geaendert = [e.key for e in EINSTELLUNGEN if e.wert() != _STANDARD[e.key]]
    if geaendert:
        log.info("Einstellungen aus %s: %s", DATEI.name, ", ".join(geaendert))


def liste() -> dict[str, Any]:
    """Alle Einstellungen nach Blöcken, mit Wert, Standard und Grenzen."""
    gruppen = []
    for gid, titel, text in GRUPPEN:
        eintraege = []
        for e in EINSTELLUNGEN:
            if e.gruppe != gid:
                continue
            standard = _STANDARD[e.key]
            if e.optionen is not None:
                typ = "auswahl"
            elif isinstance(standard, bool):
                typ = "bool"
            elif isinstance(standard, int):
                typ = "int"
            else:
                typ = "float"
            eintraege.append({
                "key": e.key,
                "label": e.label,
                "typ": typ,
                "wert": e.wert(),
                "standard": standard,
                "min": e.min,
                "max": e.max,
                "schritt": e.schritt,
                "einheit": e.einheit,
                "optionen": list(e.optionen) if e.optionen is not None else None,
                "hilfe": e.hilfe,
                "neustart": e.neustart,
            })
        gruppen.append({"id": gid, "titel": titel, "text": text, "einstellungen": eintraege})
    return {"gruppen": gruppen}


def setzen(neu: dict[str, Any]) -> list[str]:
    """Übernimmt mehrere Werte -- alle oder keinen. Gibt die geänderten zurück.

    Erst alles prüfen, dann setzen: sonst bliebe bei einem ungültigen
    Wert die Hälfte geändert zurück.
    """
    with _lock:
        geprueft = {}
        for key, wert in neu.items():
            e = _NACH_KEY.get(key)
            if e is None:
                raise EinstellungsFehler(f"Unbekannte Einstellung {key!r}")
            geprueft[key] = _pruefen(e, wert)

        _regeln_pruefen({e.key: geprueft.get(e.key, e.wert()) for e in EINSTELLUNGEN})

        geaendert = [k for k, v in geprueft.items() if _NACH_KEY[k].wert() != v]
        for key in geaendert:
            _NACH_KEY[key].setzen(geprueft[key])
        if geaendert:
            try:
                _speichern()
            except OSError as exc:
                # Der Wert gilt trotzdem -- nur eben nicht über einen
                # Neustart hinaus. Kein Grund, die Änderung abzulehnen.
                log.warning("Einstellungen nicht gespeichert (%s) -- gelten bis zum Neustart", exc)

    for key in geaendert:
        log.info("Einstellung %s = %r", key, geprueft[key])
    return geaendert


def neustart_noetig(keys: list[str]) -> list[str]:
    """Welche der Einstellungen erst nach einem Neustart des Servers wirken."""
    return [k for k in keys if k in _NACH_KEY and _NACH_KEY[k].neustart]


def zuruecksetzen(keys: list[str] | None = None) -> list[str]:
    """Auf die Werte aus config.py zurück -- einzelne oder alle."""
    keys = [e.key for e in EINSTELLUNGEN] if keys is None else keys
    return setzen({k: _STANDARD[k] for k in keys if k in _NACH_KEY})
