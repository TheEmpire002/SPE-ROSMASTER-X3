"""Das Nachrichtenformat zwischen Roboter, Web-App und ESP32.

Diese Datei ist der Vertrag. Wer hier etwas ändert, muss die Web-App
(`webapp/app.js`) und die ESP-Firmware mitziehen. Deshalb: lieber einen neuen
Nachrichtentyp ergänzen als einen bestehenden umbauen.

Alle Nachrichten sind JSON-Objekte mit einem Feld `type`.

Von der App / vom ESP an den Roboter
------------------------------------
{"type": "drive",  "vx": 0.2, "vy": 0.0, "omega": 0.0}   Fahrbefehl, m/s bzw. rad/s
{"type": "stop"}                                          sofort anhalten
{"type": "beep",   "ms": 100}                             Buzzer
{"type": "box",    "action": "open" | "close"}            Transportbox
{"type": "vitals", "bpm": 72, "motion": true}             Messwerte vom ESP
{"type": "home"}                                          zum Startpunkt zurückfahren
{"type": "home_cancel"}                                   Heimfahrt abbrechen
{"type": "set_home"}                                      hier ist der neue Startpunkt
{"type": "ping"}                                          Verbindung am Leben halten

Vom Roboter an die App
----------------------
{"type": "limits", "max_vx":…, "max_vy":…, "max_omega":…}  einmalig beim Verbinden
{"type": "state",  "vx":…, "vy":…, "omega":…, "driver":…, "connected":…}
{"type": "vitals", "bpm":…, "motion":…}                   weitergereichte ESP-Daten
{"type": "alert",  "level": "info"|"warning"|"emergency", "text": "…"}
{"type": "error",  "text": "…"}                           ungültige Nachricht o.Ä.
{"type": "pong"}
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any


class ProtocolError(ValueError):
    """Eine eingehende Nachricht ist unbrauchbar."""


@dataclass(frozen=True)
class DriveCommand:
    """Ein Fahrbefehl in Roboter-Koordinaten.

    vx    vorwärts positiv, m/s
    vy    links positiv, m/s  (nur mit Mecanum-Rädern möglich)
    omega gegen den Uhrzeigersinn positiv, rad/s
    """

    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    @classmethod
    def from_message(cls, msg: dict[str, Any]) -> "DriveCommand":
        try:
            return cls(
                vx=float(msg.get("vx", 0.0)),
                vy=float(msg.get("vy", 0.0)),
                omega=float(msg.get("omega", 0.0)),
            )
        except (TypeError, ValueError) as exc:
            raise ProtocolError(f"Fahrbefehl enthält keine Zahlen: {msg!r}") from exc

    def is_stop(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.omega == 0.0


STOP = DriveCommand()


# --- Nachrichten an die App bauen ---------------------------------------------
# Kleine Hilfsfunktionen, damit die Feldnamen nur an einer Stelle stehen.


def limits_message(max_vx: float, max_vy: float, max_omega: float) -> dict[str, Any]:
    """Die geltenden Obergrenzen, direkt nach dem Verbinden.

    Damit muss die App sie nicht kennen: es gibt genau eine Quelle
    (``config.py``), und niemand kann vergessen, den Browser nachzuziehen.
    """
    return {
        "type": "limits",
        "max_vx": max_vx,
        "max_vy": max_vy,
        "max_omega": max_omega,
    }


def state_message(
    cmd: DriveCommand,
    driver: str,
    connected: bool,
    pose: tuple[float, float, float] | None = None,
    homing: bool = False,
    pose_gemessen: bool = False,
) -> dict[str, Any]:
    msg: dict[str, Any] = {
        "type": "state",
        "driver": driver,
        "connected": connected,
        "homing": homing,
        **asdict(cmd),
    }
    if pose is not None:
        x, y, winkel = pose
        msg["pose"] = {
            "x": round(x, 3),
            "y": round(y, 3),
            "winkel": round(winkel, 1),
            # Falsch heißt: nur aus den Fahrbefehlen gerechnet, also
            # deutlich ungenauer. Das gehört in die Anzeige.
            "gemessen": pose_gemessen,
        }
    return msg


def vitals_message(bpm: float | None, motion: bool | None) -> dict[str, Any]:
    return {"type": "vitals", "bpm": bpm, "motion": motion}


def alert_message(level: str, text: str) -> dict[str, Any]:
    if level not in ("info", "warning", "emergency"):
        raise ProtocolError(f"Unbekannte Alarmstufe: {level!r}")
    return {"type": "alert", "level": level, "text": text}


def error_message(text: str) -> dict[str, Any]:
    return {"type": "error", "text": text}
