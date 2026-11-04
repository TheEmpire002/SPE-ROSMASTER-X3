"""Einen geplanten Weg abfahren -- Takt für Takt ein Fahrbefehl.

Gemeinsam für die Fahrt zum Ziel (``homing.py``) und die Fahrt zur
nächsten Lücke beim Kartieren (``kartierfahrt.py``). Entscheidet nur die
Richtung; Tempo-Begrenzung und Bremsweg prüft wie immer der Fahrschutz im
Watchdog (``quelle="autonom"``).

- Gelenkt wird auf den nächsten Wegpunkt. Ist er erreicht
  (``KARTE_WEGPUNKT_M``), gilt der nächste.
- Steht etwas im Weg, das die Karte nicht kennt (Mensch, Stuhl), schiebt
  er schräg daran vorbei wie die Heimfahrt (``ausweichen.richtung_waehlen``).
  Geht auch das nicht, meldet er BLOCKIERT -- neu planen ist Sache des
  Aufrufers, der die Karte kennt.
- Er dreht sich dabei langsam in Fahrtrichtung. Mecanum könnte auch
  seitwärts, aber so schaut die Kamera dorthin, wo er hinfährt, und lange
  Seitwärtsfahrten rutschen mehr.
"""

from __future__ import annotations

import math

from .. import config
from ..protocol import DriveCommand
from . import ausweichen

FAEHRT = "faehrt"
ANGEKOMMEN = "angekommen"
BLOCKIERT = "blockiert"
ABGEKOMMEN = "abgekommen"


def _normalisieren(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def _abstand_zur_strecke(p, a, b) -> float:
    ax, ay = b[0] - a[0], b[1] - a[1]
    laenge2 = ax * ax + ay * ay
    if laenge2 < 1e-9:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / laenge2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


class Pfadfolger:
    """Folgt einer Liste von Wegpunkten im Kartenrahmen."""

    def __init__(self, tempo: float | None = None, toleranz: float | None = None) -> None:
        self.tempo = config.KARTE_FAHRT_TEMPO if tempo is None else tempo
        self.toleranz = config.KARTE_ZIEL_TOLERANZ_M if toleranz is None else toleranz
        self.punkte: list[tuple[float, float]] = []
        self.ziel_winkel: float | None = None
        self._i = 0
        self._von: tuple[float, float] | None = None
        self._seite = 0.0
        self.rest_m = 0.0

    def setzen(self, punkte, von=None, ziel_winkel: float | None = None) -> None:
        """Neuer Weg. ``von``: wo er beginnt (für die Abweichung vom ersten Stück)."""
        self.punkte = [tuple(p) for p in punkte]
        self.ziel_winkel = ziel_winkel
        self._i = 0
        self._von = tuple(von[:2]) if von is not None else None
        self._seite = 0.0
        self.rest_m = 0.0

    def _rest(self, pose) -> float:
        p = self.punkte
        rest = math.hypot(p[self._i][0] - pose[0], p[self._i][1] - pose[1])
        for a, b in zip(p[self._i:], p[self._i + 1:]):
            rest += math.hypot(b[0] - a[0], b[1] - a[1])
        return rest

    def schritt(self, pose, punkte) -> tuple[DriveCommand, str]:
        """Ein Takt. ``punkte``: Hindernisse im Roboterrahmen (Lidar samt Gedächtnis)."""
        halt = DriveCommand(0.0, 0.0, 0.0)
        if not self.punkte:
            return halt, ANGEKOMMEN
        x, y, th = pose
        p = self.punkte
        while self._i < len(p) - 1 and math.hypot(p[self._i][0] - x, p[self._i][1] - y) < config.KARTE_WEGPUNKT_M:
            self._von = p[self._i]
            self._i += 1
        self.rest_m = self._rest(pose)

        if self._von is not None and _abstand_zur_strecke(
            (x, y), self._von, p[self._i]
        ) > config.KARTE_ABWEICHUNG_M:
            return halt, ABGEKOMMEN

        ziel = p[-1]
        am_ziel = self._i == len(p) - 1 and math.hypot(ziel[0] - x, ziel[1] - y) <= self.toleranz
        if am_ziel:
            if self.ziel_winkel is None:
                return halt, ANGEKOMMEN
            fehler = _normalisieren(self.ziel_winkel - th)
            if abs(fehler) <= math.radians(config.KARTE_ZIEL_GRAD):
                return halt, ANGEKOMMEN
            return DriveCommand(0.0, 0.0, math.copysign(config.MAP_FAHRT_DREH_TEMPO, fehler)), FAEHRT

        tx, ty = p[self._i]
        richtung = _normalisieren(math.atan2(ty - y, tx - x) - th)
        tempo = self.tempo
        if self.rest_m < config.HOME_SLOWDOWN_M:
            # Nicht beliebig langsam: Sehr kleine Werte setzen die Motoren
            # am Boden nicht mehr um.
            tempo *= max(0.35, self.rest_m / config.HOME_SLOWDOWN_M)

        bis_punkt = math.hypot(tx - x, ty - y)
        weit = min(bis_punkt + config.AUSWEICH_VORN_M + config.AUSWEICH_RAND_M,
                   config.UMFAHREN_VORAUS_M + config.HOME_STOP_DISTANCE_M)
        wahl = None
        for noetig in (weit, min(weit, config.HOME_STOP_DISTANCE_M)):
            wahl = ausweichen.richtung_waehlen(
                punkte, math.degrees(richtung), noetig, config.EXPLORE_HALF_WIDTH_M, self._seite
            )
            if wahl is not None:
                break
        if wahl is None:
            return halt, BLOCKIERT
        neu = math.radians(wahl[0])
        versatz = _normalisieren(neu - richtung)
        self._seite = 0.0 if abs(versatz) < 1e-6 else math.copysign(1.0, versatz)

        omega = 0.0
        if self.rest_m > 0.5:
            omega = max(-config.KARTE_DREH_MAX, min(config.KARTE_DREH_MAX, 1.0 * richtung))
            if abs(omega) < 0.1:
                omega = 0.0  # kein Zittern um die Geradeausfahrt
        return DriveCommand(tempo * math.cos(neu), tempo * math.sin(neu), omega), FAEHRT
