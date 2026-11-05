"""The message format between robot, web app and ESP32.

This file is the contract. Whoever changes something here must update the
web app (`webapp/app.js`) and the ESP firmware as well. Therefore: rather add a
new message type than rework an existing one.

All messages are JSON objects with a field `type`.

From the app / the ESP to the robot
-----------------------------------
{"type": "drive",  "vx": 0.2, "vy": 0.0, "omega": 0.0}   drive command, m/s or rad/s
{"type": "stop"}                                          stop immediately
{"type": "beep",   "ms": 100}                             buzzer
{"type": "box",    "action": "open" | "close"}            transport box
{"type": "vitals", "bpm": 72, "motion": true}             readings from the ESP
{"type": "home"}                                          drive back to the start point
{"type": "home_cancel"}                                   cancel homing
{"type": "set_home"}                                      this is the new start point
{"type": "collision", "enabled": true, "distance": 0.35}  collision avoidance
{"type": "explore"}                                       drive around autonomously (test 1)
{"type": "explore_stop"}                                  end autonomous driving
{"type": "interaction_start"}                             interaction on: gestures control
                                                          (drives! only on the floor)
{"type": "interaction_stop"}                              interaction off, stop
{"type": "follow"}                                        follow the person in front of the camera
                                                          (also switches interaction on)
{"type": "follow", "ziel_id": 3}                          start with this person (only older
                                                          clients; id from /interaktion/pose)
{"type": "follow_stop"}                                   stop following, interaction stays on
{"type": "follow_pause"}                                  stop, target stays remembered
{"type": "follow_resume"}                                 continue after the pause
{"type": "record_save", "notiz": "..."}                   save the last minute (Recorder)
{"type": "say", "text": "Hallo", "vorrang": false}       speak through the loudspeaker
                                                          (vorrang: cancel what is playing)
{"type": "say_stop"}                                      fall silent immediately, clear the queue
{"type": "move", "richtung": "vor", "meter": 1.5}          drive exactly this far, "vor" | "zurueck" |
                                                          "links" | "rechts" (lidar measures, up to 5 m)
{"type": "turn", "grad": 90}                              turn on the spot, + = left (counter-
                                                          clockwise), - = right
{"type": "speech", "text": "robi folgen"}                 handle text as if it had been spoken
                                                          into the microphone (test, --mock)
{"type": "goto", "x": 2.5, "y": -1.0, "winkel": 90}       drive to a point on the map
                                                          (m in the map frame; winkel optional,
                                                          degrees, heading at the goal)
{"type": "goto_cancel"}                                   cancel the goal drive
{"type": "map_pose", "x": 1.0, "y": 0.5, "winkel": 0}     "I am standing here": search the pose
                                                          in the map there (winkel optional)
{"type": "map_load", "name": "wohnung"}                   load a saved map to drive
                                                          in it
{"type": "health_check", "x": 2.5, "y": -1.0}            healthbot: drive to the spot and check the
                                                          person's condition (x, y in m, map frame;
                                                          without x/y: check here).
                                                          Drives and nudges! Only on the floor.
{"type": "health_stop"}                                   cancel the healthbot check, stop
{"type": "health_wache", "an": true}                      healthbot watch: detect a lying person
                                                          on its own and then check
{"type": "ping"}                                          keep the connection alive

From the robot to the app
-------------------------
{"type": "limits", "max_vx":…, "max_vy":…, "max_omega":…}  once when connecting
{"type": "state",  "vx":…, "vy":…, "omega":…, "driver":…, "connected":…}
                   (with lidar additionally "autonomie": {"aktiv":…, "zustand":…,
                    "hindernisse": […], "personen": […], …};
                    with camera pose additionally "interaktion": {"aktiv":…,
                    "modus":…, "text":…, "person":…, "geste":…, …};
                    with voice additionally "stimme": {"verfuegbar":…,
                    "spricht":…, "text":…};
                    with microphone additionally "sprache": {"verfuegbar":…,
                    "geweckt":…, "gehoert":…, "befehl":…};
                    with a loaded map additionally "karte": {"name":…,
                    "live":…, "pose": {"x","y","winkel"}|null, "sicher":…,
                    "text":…, "fahrt": {"aktiv","art","ziel","rest_m","grund"}};
                    with healthbot additionally "healthbot": {"aktiv":…,
                    "zustand":…, "schritt":…, "alarm":…, "wache":…},
                    everything else under GET /healthbot/status)
{"type": "vitals", "bpm":…, "motion":…}                   forwarded ESP data
{"type": "alert",  "level": "info"|"warning"|"emergency", "text": "…"}
{"type": "error",  "text": "…"}                           invalid message or similar
{"type": "collision", "enabled":…, "distance":…, "verfuegbar":…, "blockiert":…}
{"type": "pong"}
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any


class ProtocolError(ValueError):
    """An incoming message is unusable."""


@dataclass(frozen=True)
class DriveCommand:
    """A drive command in robot coordinates.

    vx    forward positive, m/s
    vy    left positive, m/s  (only possible with mecanum wheels)
    omega counter-clockwise positive, rad/s
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


# --- Building messages for the app --------------------------------------------
# Small helpers so that the field names live in one place only.


def limits_message(max_vx: float, max_vy: float, max_omega: float) -> dict[str, Any]:
    """The current upper limits, sent right after connecting.

    That way the app does not need to know them: there is exactly one source
    (``config.py``), and nobody can forget to update the browser.
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
    pose_measured: bool = False,
    collision: dict[str, Any] | None = None,
    autonomy: dict[str, Any] | None = None,
    interaction: dict[str, Any] | None = None,
    voice: dict[str, Any] | None = None,
    speech: dict[str, Any] | None = None,
    occ_map: dict[str, Any] | None = None,
    healthbot: dict[str, Any] | None = None,
) -> dict[str, Any]:
    msg: dict[str, Any] = {
        "type": "state",
        "driver": driver,
        "connected": connected,
        "homing": homing,
        **asdict(cmd),
    }
    if collision is not None:
        msg["collision"] = collision
    if autonomy is not None:
        msg["autonomie"] = autonomy
    if interaction is not None:
        msg["interaktion"] = interaction
    if voice is not None:
        msg["stimme"] = voice
    if speech is not None:
        msg["sprache"] = speech
    if occ_map is not None:
        msg["karte"] = occ_map
    if healthbot is not None:
        msg["healthbot"] = healthbot
    if pose is not None:
        x, y, angle = pose
        msg["pose"] = {
            "x": round(x, 3),
            "y": round(y, 3),
            "winkel": round(angle, 1),
            # False means: computed only from the drive commands, so
            # considerably less accurate. That belongs in the display.
            "gemessen": pose_measured,
        }
    return msg


def collision_message(state: dict[str, Any]) -> dict[str, Any]:
    """Current setting of the collision avoidance."""
    return {"type": "collision", **state}


def vitals_message(bpm: float | None, motion: bool | None) -> dict[str, Any]:
    return {"type": "vitals", "bpm": bpm, "motion": motion}


def alert_message(level: str, text: str) -> dict[str, Any]:
    if level not in ("info", "warning", "emergency"):
        raise ProtocolError(f"Unbekannte Alarmstufe: {level!r}")
    return {"type": "alert", "level": level, "text": text}


def error_message(text: str) -> dict[str, Any]:
    return {"type": "error", "text": text}
