"""Die vier Zeichen der Zustandserkennung -- reine Rechnung, ohne Hardware.

Jedes Zeichen liefert ein Dict, das die Web-App direkt als Ampel zeigt::

    {"name": "liegend", "titel": "Liegende Haltung",
     "zustand": "auffaellig" | "ok" | "unbekannt",
     "wert": 12.0, "text": "Rumpf 12° gegen die Waagerechte -- liegt"}

"unbekannt" heißt: Dafür fehlen Messwerte (kein Mikrofon, keine
Keypoints, kein Lidar). Es zählt weder für noch gegen einen Notfall, damit
ein ausgefallener Sensor nicht allein Alarm auslöst oder ihn verhindert.
"""

from __future__ import annotations

import math
import re

import numpy as np

from .. import config
from ..sensors.pose import KP_HUEFTE_L, KP_HUEFTE_R, KP_SCHULTER_L, KP_SCHULTER_R

AUFFAELLIG = "auffaellig"
OK = "ok"
UNBEKANNT = "unbekannt"

#: Reihenfolge der Zeichen in Status und Web-App.
ZEICHEN = ("liegend", "regungslos", "antwort", "boden")
TITEL = {
    "liegend": "Liegende Haltung",
    "regungslos": "Regungslosigkeit",
    "antwort": "Keine Antwort",
    "boden": "Lage am Boden",
}

# Zwei Lidar-Punkte gehören zum selben Gegenstand, wenn sie näher als das
# beieinander liegen. Beine einer stehenden Person (≥ 15 cm Lücke) bleiben
# so getrennt, ein liegender Körper wird ein Stück.
_LUECKE_M = 0.12
# Zum Personenwinkel aus der Kamera kommt so viel Spielraum (Grad) dazu:
# Kamera und Lidar sitzen nicht an derselben Stelle.
_WINKEL_RAND_GRAD = 5.0


def zeichen(name: str, zustand: str, wert, text: str) -> dict:
    return {"name": name, "titel": TITEL[name], "zustand": zustand, "wert": wert, "text": text}


def leer() -> dict[str, dict]:
    """Alle Zeichen noch ungeprüft (Anfang eines Ablaufs)."""
    return {n: zeichen(n, UNBEKANNT, None, "noch nicht geprüft") for n in ZEICHEN}


def _kp_px(person: dict, nr: int) -> tuple[float, float] | None:
    """Keypoint in Pixeln des 640er Bildes, oder None bei zu wenig Sicherheit.

    In Pixeln, weil die Bildkoordinaten 0..1 in x und y verschieden lang
    sind -- ein Winkel daraus wäre verzerrt.
    """
    kp = person.get("kp") or []
    if nr >= len(kp) or kp[nr][2] < config.POSE_KP_MIN_CONF:
        return None
    return kp[nr][0] * config.CAMERA_WIDTH, kp[nr][1] * config.CAMERA_HEIGHT


def _mitte(a, b):
    if a is None and b is None:
        return None
    if a is None or b is None:
        return a or b
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2


def rumpfwinkel(person: dict) -> float | None:
    """Winkel (Grad, 0..90) der Schulter-Hüft-Achse gegen die Waagerechte."""
    schulter = _mitte(_kp_px(person, KP_SCHULTER_L), _kp_px(person, KP_SCHULTER_R))
    huefte = _mitte(_kp_px(person, KP_HUEFTE_L), _kp_px(person, KP_HUEFTE_R))
    if schulter is None or huefte is None:
        return None
    dx, dy = huefte[0] - schulter[0], huefte[1] - schulter[1]
    if math.hypot(dx, dy) < 5.0:
        return None  # Punkte fast aufeinander: Richtung ist Rauschen
    return math.degrees(math.atan2(abs(dy), abs(dx)))


def liegend(person: dict | None) -> dict:
    """Liegt die Person? Erst über den Rumpf, sonst über die Form der Box."""
    if person is None:
        return zeichen("liegend", UNBEKANNT, None, "keine Person im Bild")
    w = rumpfwinkel(person)
    if w is not None:
        if w < config.HEALTH_LIEGEND_GRAD:
            return zeichen("liegend", AUFFAELLIG, round(w, 1), f"Rumpf {w:.0f}° gegen die Waagerechte -- liegt")
        return zeichen("liegend", OK, round(w, 1), f"Rumpf {w:.0f}° gegen die Waagerechte -- aufrecht")
    # Rückfall: Box in Pixeln. Unsicherer (sitzt jemand breitbeinig, ist
    # die Box auch breit), daher nur, wenn der Rumpf fehlt.
    x1, y1, x2, y2 = person["box"]
    hoehe = max(1e-6, (y2 - y1) * config.CAMERA_HEIGHT)
    verh = (x2 - x1) * config.CAMERA_WIDTH / hoehe
    if verh > config.HEALTH_LIEGEND_SEITENVERHAELTNIS:
        return zeichen("liegend", AUFFAELLIG, round(verh, 2), f"Umriss {verh:.1f}× so breit wie hoch -- liegt vermutlich")
    return zeichen("liegend", OK, round(verh, 2), f"Umriss {verh:.1f}× so breit wie hoch -- eher aufrecht")


def bewegung_px(verlauf: list[dict]) -> float | None:
    """Wie stark sich die Person in ``verlauf`` (Personen-Dicts) bewegt hat.

    Je Keypoint die mittlere Abweichung (RMS, Pixel) von seiner mittleren
    Lage; Ergebnis ist der Mittelwert der drei beweglichsten. So zählt
    schon eine Hand, die sich rührt -- der Rest des Körpers darf ruhig
    liegen.
    """
    if len(verlauf) < 3:
        return None
    werte = []
    for nr in range(17):
        punkte = [p for p in (_kp_px(v, nr) for v in verlauf) if p is not None]
        # Nur Punkte, die in den meisten Bildern zu sehen sind: Ein Punkt,
        # der auftaucht und verschwindet, springt sonst zwischen Schätzungen.
        if len(punkte) < max(3, len(verlauf) // 2):
            continue
        a = np.asarray(punkte)
        werte.append(float(np.sqrt(((a - a.mean(axis=0)) ** 2).sum(axis=1).mean())))
    if not werte:
        return None
    werte.sort(reverse=True)
    return float(np.mean(werte[:3]))


def regungslos(verlauf: list[dict]) -> dict:
    b = bewegung_px(verlauf)
    if b is None:
        return zeichen("regungslos", UNBEKANNT, None, "zu wenige Bilder mit Keypoints")
    if b < config.HEALTH_REGUNGSLOS_PX:
        return zeichen("regungslos", AUFFAELLIG, round(b, 1), f"Bewegung {b:.1f} px -- regungslos")
    return zeichen("regungslos", OK, round(b, 1), f"Bewegung {b:.1f} px -- bewegt sich")


def antwort(gehoert: list[str] | None) -> dict:
    """``gehoert`` ist None, wenn kein Mikrofon da ist."""
    if gehoert is None:
        return zeichen("antwort", UNBEKANNT, None, "kein Mikrofon -- Antwort nicht prüfbar")
    if not gehoert:
        return zeichen("antwort", AUFFAELLIG, "", "keine Antwort")
    text = " / ".join(gehoert)
    return zeichen("antwort", OK, text, f"hat geantwortet: „{text}“")


def _enthaelt(woerter: list[str], ausdruck: str) -> int:
    """Stelle, an der der Ausdruck (ein oder mehrere Wörter) beginnt, sonst -1."""
    teil = ausdruck.lower().split()
    for i in range(len(woerter) - len(teil) + 1):
        if woerter[i : i + len(teil)] == teil:
            return i
    return -1


def antwort_deuten(texte: list[str]) -> str | None:
    """"hilfe", "nein" oder None (nichts Eindeutiges gehört).

    Hilfe geht vor: Wer "nein ... hilf mir" sagt, bekommt Hilfe. Nur
    "keine Hilfe" gilt als Nein, sonst löste genau dieser Satz Alarm aus.
    """
    woerter = re.findall(r"\w+", " ".join(texte).lower())
    hilfe = False
    for w in config.HEALTH_WOERTER_HILFE:
        i = _enthaelt(woerter, w)
        if i >= 0 and not (i > 0 and woerter[i - 1] in ("keine", "kein")):
            hilfe = True
    if hilfe:
        return "hilfe"
    if any(_enthaelt(woerter, w) >= 0 for w in config.HEALTH_WOERTER_NEIN) or "keine" in woerter:
        return "nein"
    return None


def lidar_stueck(punkte, winkel_grad: float, halbe_breite_grad: float):
    """Der nächste zusammenhängende Gegenstand in Richtung der Person.

    ``punkte`` Nx2 (x vorn, y links, Roboterrahmen). Liefert
    ``(abstand_m, laenge_m)`` oder None, wenn in der Richtung nichts ist.
    """
    if punkte is None or len(punkte) == 0:
        return None
    p = np.asarray(punkte, dtype=float)
    w = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    diff = (w - winkel_grad + 180.0) % 360.0 - 180.0
    fenster = p[np.abs(diff) <= halbe_breite_grad + _WINKEL_RAND_GRAD]
    if len(fenster) == 0:
        return None
    r = np.hypot(fenster[:, 0], fenster[:, 1])
    # Vom nächsten Punkt aus wachsen lassen. Gesucht wird in allen Punkten
    # (nicht nur im Fenster), denn ein liegender Körper reicht meist über
    # den Bildausschnitt hinaus.
    start = fenster[int(np.argmin(r))]
    drin = np.zeros(len(p), dtype=bool)
    drin[int(np.argmin(np.hypot(*(p - start).T)))] = True
    while True:
        neu = drin.copy()
        for q in p[drin]:
            neu |= np.hypot(*(p - q).T) < _LUECKE_M
        if neu.sum() == drin.sum():
            break
        drin = neu
    stueck = p[drin]
    laenge = float(np.max(np.hypot(*(stueck[:, None, :] - stueck[None, :, :]).transpose(2, 0, 1))))
    return float(np.min(r)), laenge


def boden(person: dict | None, punkte, fov_deg: float) -> dict:
    """Liegt die Person am Boden? Der Lidar misst knapp über dem Boden.

    Ein liegender Körper erscheint dort als ein langes Stück, eine
    stehende Person nur als zwei schmale Beine. Ohne Lidar zählt die Lage
    im Bild (liegend und unten im Bild), aber nur als schwaches Zeichen.
    """
    if person is None:
        return zeichen("boden", UNBEKANNT, None, "keine Person im Bild")
    x1, _, x2, y2 = person["box"]
    halbe = (x2 - x1) * fov_deg / 2
    if punkte is not None:
        stueck = lidar_stueck(punkte, person["winkel"], halbe)
        if stueck is not None:
            abstand, laenge = stueck
            if laenge >= config.HEALTH_BODEN_MIN_BREITE_M:
                return zeichen("boden", AUFFAELLIG, round(laenge, 2),
                               f"Lidar: {laenge:.2f} m langes Stück in {abstand:.2f} m knapp über dem Boden")
            return zeichen("boden", OK, round(laenge, 2),
                           f"Lidar: nur {laenge:.2f} m breit in {abstand:.2f} m -- eher Beine")
    winkel = rumpfwinkel(person)
    if winkel is not None and winkel < config.HEALTH_LIEGEND_GRAD and y2 > config.HEALTH_BODEN_BILD_UNTEN:
        return zeichen("boden", AUFFAELLIG, round(y2, 2), "nur Kamera: liegt unten im Bild")
    return zeichen("boden", UNBEKANNT, None, "kein Lidar-Echo in Richtung der Person")


def anzahl(checks: dict[str, dict], zustand: str = AUFFAELLIG) -> int:
    return sum(1 for c in checks.values() if c["zustand"] == zustand)


def gesamturteil(checks: dict[str, dict], reaktion: bool | None = None) -> dict:
    """Zusammenfassung für die Web-App: Stufe und ein Satz.

    ``reaktion`` ist das Ergebnis des Anstupsens (None: nicht gestupst).
    Notfall heißt: liegt bzw. am Boden, rührt sich nicht und antwortet
    nicht -- und hat auch aufs Anstupsen nicht reagiert, falls gestupst.
    """
    n = anzahl(checks)
    lage = checks["liegend"]["zustand"] == AUFFAELLIG or checks["boden"]["zustand"] == AUFFAELLIG
    still = checks["regungslos"]["zustand"] != OK
    stumm = checks["antwort"]["zustand"] != OK
    if reaktion is True or checks["antwort"]["zustand"] == OK:
        return {"stufe": "ansprechbar", "punkte": n, "text": "Person reagiert"}
    if lage and still and stumm and reaktion is not True:
        return {"stufe": "notfall", "punkte": n, "text": "liegt regungslos und reagiert nicht"}
    if n == 0:
        return {"stufe": "unauffaellig", "punkte": n, "text": "keine Auffälligkeit"}
    return {"stufe": "auffaellig", "punkte": n, "text": f"{n} von 4 Zeichen auffällig"}
