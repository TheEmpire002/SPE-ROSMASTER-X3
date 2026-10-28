"""Hindernisse im Lidar-Scan finden und grob benennen.

Der Lidar sieht nur eine Ebene ~20 cm über dem Boden. Was dort ankommt, hat
aber typische Formen:

- **Wand**: lange, gerade Punktreihe
- **Stuhl**: mehrere dünne Beine (2-5 cm) dicht beieinander
- **Tisch**: dünne Beine mit 0,6-1,8 m Abstand
- **Mensch**: zwei dickere Beine (8-25 cm) nebeneinander -- oder etwas,
  das die Kamera als Person erkannt hat
- alles andere heißt **Objekt**

Das ist eine Faustregel, keine Objekterkennung: Ein Papierkorb kann als
"Bein (Mensch?)" durchgehen. Für das Ausweichen ist die Benennung ohnehin
egal -- der Roboter weicht allem aus. Sie dient der Anzeige und dazu, vor
Menschen kurz zu warten, statt sofort abzudrehen.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, asdict

from .. import config

#: Diese Namen landen so in der App (state.autonomie.hindernisse[].art).
WAND = "Wand"
STUHL = "Stuhl"
TISCH = "Tisch"
MENSCH = "Mensch"
BEIN_MENSCH = "Bein (Mensch?)"
BEIN = "Bein (Stuhl/Tisch?)"
OBJEKT = "Objekt"


@dataclass
class Hindernis:
    art: str
    winkel: float     # Grad, 0 = vorn, gegen den Uhrzeigersinn
    abstand: float    # m, nächster Punkt ab Lidar-Mitte
    breite: float     # m
    x: float          # m, Mittelpunkt in Roboterkoordinaten (vorn)
    y: float          # m, (links)
    quelle: str = "Lidar"

    def als_dict(self) -> dict:
        d = asdict(self)
        for k in ("winkel",):
            d[k] = round(d[k], 1)
        for k in ("abstand", "breite", "x", "y"):
            d[k] = round(d[k], 2)
        return d


def _punkte(winkel: list[float], entfernungen: list[float]) -> list[tuple[float, float, float, float]]:
    """Nach Winkel sortiert: (winkel, r, x, y), nur im Erkennungsumkreis."""
    roh = sorted(
        (w, r) for w, r in zip(winkel, entfernungen) if 0.0 < r <= config.OBSTACLE_RANGE_M
    )
    return [
        (w, r, r * math.cos(math.radians(w)), r * math.sin(math.radians(w)))
        for w, r in roh
    ]


def _haufen(punkte: list) -> list[list]:
    """Benachbarte Punkte zu Objekten zusammenfassen.

    Ein neues Objekt beginnt, wo der Abstand zum Vorgänger springt. Die
    Schwelle wächst mit der Entfernung, weil dort schon der Winkelschritt
    allein die Punkte auseinanderzieht.
    """
    if not punkte:
        return []
    gruppen: list[list] = [[punkte[0]]]
    for vorher, p in zip(punkte, punkte[1:]):
        schwelle = config.OBSTACLE_GAP_M + 0.03 * p[1]
        luecke = math.hypot(p[2] - vorher[2], p[3] - vorher[3])
        if luecke <= schwelle and p[0] - vorher[0] <= 5.0:
            gruppen[-1].append(p)
        else:
            gruppen.append([p])

    # Rundum geschlossen: Ein Objekt genau hinter dem Roboter liegt am
    # Anfang und am Ende der Liste (-180°/+180°) -- das ist ein Objekt.
    if len(gruppen) > 1:
        erst, letzt = gruppen[0][0], gruppen[-1][-1]
        luecke = math.hypot(erst[2] - letzt[2], erst[3] - letzt[3])
        if luecke <= config.OBSTACLE_GAP_M + 0.03 * erst[1] and (erst[0] + 360.0 - letzt[0]) <= 5.0:
            gruppen[0] = gruppen.pop() + gruppen[0]
    return gruppen


def _abweichung(gruppe: list) -> tuple[float, int]:
    """Größter Abstand eines Punkts von der Linie erster-letzter Punkt."""
    ax, ay = gruppe[0][2], gruppe[0][3]
    bx, by = gruppe[-1][2], gruppe[-1][3]
    laenge = math.hypot(bx - ax, by - ay) or 1e-9
    groesste, index = 0.0, 0
    for i, p in enumerate(gruppe):
        d = abs((bx - ax) * (ay - p[3]) - (ax - p[2]) * (by - ay)) / laenge
        if d > groesste:
            groesste, index = d, i
    return groesste, index


def _zerlegen(gruppe: list, tiefe: int = 0) -> list[list]:
    """Ecken aufteilen: Zwei Wände im rechten Winkel sind ein Haufen, sollen
    aber als zwei gerade Stücke erkannt werden."""
    if len(gruppe) < 6 or tiefe >= 3:
        return [gruppe]
    breite = math.hypot(gruppe[-1][2] - gruppe[0][2], gruppe[-1][3] - gruppe[0][3])
    if breite < config.OBSTACLE_WALL_MIN_M:
        return [gruppe]
    abw, i = _abweichung(gruppe)
    if abw <= 0.05 or i in (0, len(gruppe) - 1):
        return [gruppe]
    return _zerlegen(gruppe[: i + 1], tiefe + 1) + _zerlegen(gruppe[i:], tiefe + 1)


def _hindernis(gruppe: list, art: str) -> Hindernis:
    x = sum(p[2] for p in gruppe) / len(gruppe)
    y = sum(p[3] for p in gruppe) / len(gruppe)
    breite = math.hypot(gruppe[-1][2] - gruppe[0][2], gruppe[-1][3] - gruppe[0][3])
    return Hindernis(
        art=art,
        winkel=math.degrees(math.atan2(y, x)),
        abstand=min(p[1] for p in gruppe),
        breite=breite,
        x=x,
        y=y,
    )


def _zusammenfassen(teile: list[Hindernis], art: str) -> Hindernis:
    """Mehrere Beine zu einem Möbelstück / Menschen vereinen."""
    x = sum(t.x for t in teile) / len(teile)
    y = sum(t.y for t in teile) / len(teile)
    breite = max(
        (math.hypot(a.x - b.x, a.y - b.y) for a in teile for b in teile), default=0.0
    )
    return Hindernis(
        art=art,
        winkel=math.degrees(math.atan2(y, x)),
        abstand=min(t.abstand for t in teile),
        breite=breite,
        x=x,
        y=y,
    )


def _gruppieren(teile: list[Hindernis], max_abstand: float) -> list[list[Hindernis]]:
    """Zusammenhängende Gruppen: jeder hat einen Nachbarn näher als max_abstand."""
    rest = list(teile)
    gruppen = []
    while rest:
        gruppe = [rest.pop()]
        i = 0
        while i < len(gruppe):
            nah = [t for t in rest if math.hypot(t.x - gruppe[i].x, t.y - gruppe[i].y) <= max_abstand]
            for t in nah:
                rest.remove(t)
            gruppe.extend(nah)
            i += 1
        gruppen.append(gruppe)
    return gruppen


def _im_bildwinkel(winkel: float, personen: list[tuple[float, float]]) -> bool:
    return any(lo - 5.0 <= winkel <= hi + 5.0 for lo, hi in personen)


def klassifizieren(
    winkel: list[float],
    entfernungen: list[float],
    personen_winkel: list[tuple[float, float]] | None = None,
) -> list[Hindernis]:
    """Hindernisse eines Scans, das nächste zuerst.

    ``personen_winkel``: Winkelbereiche (von, bis) in Grad, in denen die
    Kamera gerade eine Person sieht. Was der Lidar dort findet, heißt dann
    "Mensch", egal welche Form es hat.
    """
    personen_winkel = personen_winkel or []
    stuecke: list[Hindernis] = []
    for haufen in _haufen(_punkte(winkel, entfernungen)):
        # Einzelne Punkte sind meist Rauschen -- außer ganz nah, wo ein
        # Tischbein schon mit einem Strahl getroffen gefährlich ist.
        if len(haufen) < 2 and haufen[0][1] > 1.0:
            continue
        for teil in _zerlegen(haufen):
            breite = math.hypot(teil[-1][2] - teil[0][2], teil[-1][3] - teil[0][3])
            if breite >= config.OBSTACLE_WALL_MIN_M and _abweichung(teil)[0] <= 0.06:
                art = WAND
            elif breite <= config.OBSTACLE_LEG_MAX_M:
                art = BEIN
            elif breite <= config.OBSTACLE_HUMAN_LEG_MAX_M:
                art = BEIN_MENSCH
            else:
                art = OBJEKT
            stuecke.append(_hindernis(teil, art))

    ergebnis = [s for s in stuecke if s.art in (WAND, OBJEKT)]

    # Menschenbeine: zwei dicke Beine dicht nebeneinander.
    for gruppe in _gruppieren([s for s in stuecke if s.art == BEIN_MENSCH], 0.45):
        ergebnis.append(_zusammenfassen(gruppe, MENSCH) if len(gruppe) >= 2 else gruppe[0])

    # Dünne Beine: dicht beieinander ein Stuhl, weiter auseinander ein Tisch.
    duenne = [s for s in stuecke if s.art == BEIN]
    einzeln = []
    for gruppe in _gruppieren(duenne, 0.55):
        if len(gruppe) >= 2:
            ergebnis.append(_zusammenfassen(gruppe, STUHL))
        else:
            einzeln.extend(gruppe)
    for gruppe in _gruppieren(einzeln, 1.8):
        if len(gruppe) >= 2:
            ergebnis.append(_zusammenfassen(gruppe, TISCH))
        else:
            ergebnis.extend(gruppe)

    # Die Kamera sticht die Form: Sieht sie in dieser Richtung eine Person,
    # ist es eine -- auch wenn die Beine im Lidar wie ein Objekt aussehen.
    for h in ergebnis:
        if h.art != WAND and _im_bildwinkel(h.winkel, personen_winkel):
            h.art = MENSCH
            h.quelle = "Kamera+Lidar"

    ergebnis.sort(key=lambda h: h.abstand)
    return ergebnis


class Hinderniserkennung:
    """Merkt sich das Ergebnis je Scan.

    Server (Anzeige) und Erkunder (Fahren) fragen beide -- gerechnet wird
    trotzdem nur einmal pro Lidar-Umdrehung.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._scan_zeit: float | None = None
        self._personen: list[tuple[float, float]] = []
        self._ergebnis: list[Hindernis] = []

    def aktuell(self, scan, personen_winkel: list[tuple[float, float]] | None = None) -> list[Hindernis]:
        personen_winkel = personen_winkel or []
        with self._lock:
            if scan is None:
                return []
            if scan.zeit == self._scan_zeit and personen_winkel == self._personen:
                return self._ergebnis
        ergebnis = klassifizieren(scan.winkel, scan.entfernungen, personen_winkel)
        with self._lock:
            self._scan_zeit = scan.zeit
            self._personen = personen_winkel
            self._ergebnis = ergebnis
        return ergebnis
