"""German user-facing texts and judgement words."""

import time

from . import config


def fmt_duration(seconds: float) -> str:
    s = int(round(seconds))
    if s < 60:
        return f"{s} s"
    if s < 3600:
        return f"{s // 60} min {s % 60} s"
    return f"{s // 3600} h {s % 3600 // 60} min"


def fmt_volts(v: float | None) -> str:
    if v is None:
        return "unbekannt"
    return f"{v:.2f}".replace(".", ",") + " V"


def fmt_num(value: float, decimals: int = 1) -> str:
    return f"{value:.{decimals}f}".replace(".", ",")


def fmt_clock(wall: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(wall))


def _judge(value, good_from, low_from):
    if value is None:
        return ("unbekannt", "unknown")
    if value >= good_from:
        return ("gut", "ok")
    if value >= low_from:
        return ("knapp", "warn")
    return ("zu niedrig", "bad")


def judge_ext5v(v):
    return _judge(v, config.EXT5V_GOOD_V, config.EXT5V_LOW_V)


def judge_battery(v):
    return _judge(v, config.BATTERY_GOOD_V, config.BATTERY_LOW_V)


def judge_temp(c):
    if c is None:
        return ("unbekannt", "unknown")
    if c < config.TEMP_WARM_C:
        return ("gut", "ok")
    if c < config.TEMP_HOT_C:
        return ("warm", "warn")
    return ("zu heiß", "bad")


def judge_cpu(pct):
    if pct is None:
        return ("unbekannt", "unknown")
    if pct < config.CPU_WARN_PCT:
        return ("gut", "ok")
    if pct < config.CPU_SPIKE_PCT:
        return ("hoch", "warn")
    return ("überlastet", "bad")


_SENTENCES = {
    "disconnect": "{d} hat sich abgemeldet",
    "new_device": "{d} wird erkannt",
    "connect": "{d} hat sich angemeldet",
    "overcurrent": "Überlast an den Pi-Buchsen – der gemeinsame Stromschalter hat abgeschaltet",
    "reset_failed": "Neustart des Anschlusses ist fehlgeschlagen (Leitung gestört)",
    "enum_failed": "{d} konnte sich nicht anmelden (Leitung gestört)",
    "protocol_error": "Übertragungsfehler auf der Leitung zu {d}",
    "driver_attached": "{d} ist bereit",
    "driver_detached": "{d} ist nicht mehr nutzbar",
    "undervoltage": "Der Pi meldet Unterspannung am Netzteil",
    "board_silent": "Das Board sendet keine Meldungen mehr, obwohl es noch angeschlossen ist",
}


def event_sentence(kind: str, device: str, details: dict) -> str:
    template = _SENTENCES.get(kind)
    if template is None:
        return device
    return template.format(d=device)


REASONS: dict[str, dict] = {
    "motor_start": {
        "title": "Motoranlauf",
        "action": "Motoren sanfter anfahren lassen (Rampe) und Board/Hub getrennt vom Motorstrom versorgen",
    },
    "reenum_loop": {
        "title": "Wiedereinschalt-Schleife",
        "action": "Roboter einmal ganz ausschalten; Tiefenkamera über eigenen, versorgten Hub anschließen",
    },
    "heat": {
        "title": "Hitze / Taktbremse",
        "action": "Lüfter prüfen und nicht fahren, während der Pi installiert oder kompiliert",
    },
    "undervoltage": {
        "title": "Unterspannung Pi-Netzteil",
        "action": "Pi-Netzteil und Kabel prüfen (offizielles 5-V-/5-A-Netzteil)",
    },
    "cpu_spike": {
        "title": "CPU-Lastspitze",
        "action": "Rechenlast reduzieren; nicht fahren, während apt oder ein Compiler läuft",
    },
    "board_mcu_silent": {
        "title": "Board-Mikrocontroller still",
        "action": "Akku und Board-Stromversorgung prüfen – der Mikrocontroller ist neu gestartet",
    },
    "idle_unclear": {
        "title": "Im Stand, Ursache unklar (Rückspeisung?)",
        "action": "Rückspeisung über den Board-Hub prüfen (VBUS trennen oder Hub ohne Rückspeisung)",
    },
}


PI_OVERCURRENT_NOTE = "Pi-Buchsen teilen sich einen 1,6-A-Schalter"
SAMPLING_NOTE = ("Spannung wird 1× pro Sekunde gemessen; sehr kurze Einbrüche sieht man nur an der "
                 "Taktbremse-Anzeige.")
DRIVER_WORDS = {
    "hub": "Hub-Treiber",
    "ch341": "Treiber für den Board-Chip",
    "cp210x": "Treiber für den Lidar-Chip",
}
PORT_STATE_WORDS = {"configured": "in Betrieb", "not attached": "nichts angeschlossen"}
CAPTURE_TEXTS = {
    "active": "Mithören aktiv",
    "idle": "kein Verkehr (Server aus)",
    "unavailable": "Mithören nicht verfügbar",
    "camera_on_bus": "Mithören abgeschaltet: Kamera am selben USB-Bus",
    "no_board": "Board nicht angeschlossen",
}


def fmt_short(seconds: float) -> str:
    """Duration for headlines: seconds below a minute, then whole minutes."""
    seconds = max(0.0, seconds)
    if seconds < 60:
        return f"{int(round(seconds))} s"
    minutes = int(round(seconds / 60))
    if minutes < 60:
        return f"{minutes} min"
    return f"{minutes // 60} h {minutes % 60} min"


def fmt_speed_link(mbps: int) -> str:
    if mbps >= 1000:
        return f"{fmt_num(mbps / 1000)} Gbit/s"
    return f"{mbps} Mbit/s"


def fmt_rate(bytes_per_s: float) -> str:
    if bytes_per_s >= 1000:
        return f"{fmt_num(bytes_per_s / 1000)} kB/s"
    return f"{int(round(bytes_per_s))} Byte/s"


MODULE_TITLES = {"pi": "Raspberry Pi 5", "board": "Expansion Board", "motor": "Motorsteuerung"}
SOCKETS_UNVERIFIED_NOTE = "Lage der Buchsen noch nicht geprüft"
