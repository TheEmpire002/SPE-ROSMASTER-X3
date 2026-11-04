"""Fortschrittswache: dreht er die Räder, kommt aber nicht vom Fleck?

Hängt der Roboter an einem Stuhlfuß oder einer Kante, die weder Lidar noch
Kamera sehen, sieht für die Fahrlogik alles gut aus: vorn frei, Befehl
geht raus. Nur bewegt er sich nicht. Beim Kartieren stand er dann, bis
jemand kam -- und die Karte wuchs nicht weiter.

Die Wache vergleicht zwei Dinge über ein Fenster von ~2 s:

* **befohlen:** was der Watchdog wirklich an die Motoren gab (nach Rampe
  und Fahrschutz). Bremst der Fahrschutz ab, ist auch die Erwartung klein
  -- vor einer Wand stehen ist kein Festhängen.
* **gemessen:** wie weit sich die Lidar-Aufnahme von damals auf die von
  jetzt schieben lässt (ICP, ``scan_matching.py``). Das sieht auf die
  Wände, durchdrehende Räder sind ihm egal.

Kam laut Lidar weniger als ``FORTSCHRITT_ANTEIL`` der befohlenen Bewegung
an, hängt er fest. Geurteilt wird nur, wenn der Abgleich eindeutig ist:
Er wird zweimal gerechnet, einmal ab "nicht bewegt", einmal ab "wie
befohlen". Landen beide woanders, ist die Umgebung mehrdeutig (langer
Flur, kahle Wand) -- dann lieber kein Urteil als ein falsches.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import NamedTuple

from .. import config
from .scan_matching import match

log = logging.getLogger(__name__)

#: Punkte je Aufnahme für den Abgleich (jeder mit jedem, Zeit wächst quadratisch).
_MAX_PUNKTE = 200
#: So weit dürfen die beiden Abgleiche auseinanderliegen, damit er urteilt.
_EINIG_M = 0.05
_EINIG_RAD = math.radians(5.0)


class Festgefahren(NamedTuple):
    anlass: str
    richtung_grad: float | None  # wohin er wollte (Roboterrahmen), None beim Drehen


def _verknuepfen(a, b):
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def _umkehren(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), -at)


def _kuerzen(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def _ausduennen(punkte, hoechstens: int):
    if len(punkte) <= hoechstens:
        return punkte
    import numpy as np

    return punkte[(np.arange(hoechstens) * (len(punkte) / hoechstens)).astype(int)]


class Fortschrittswache:
    """In jedem Takt ``mitzaehlen``, dann ``pruefen``."""

    def __init__(self, watchdog) -> None:
        self._watchdog = watchdog
        self.urteile = 0
        self.zuruecksetzen()

    def zuruecksetzen(self) -> None:
        """Neu beobachten -- nach einem Urteil, Befreien, Hochheben."""
        self._befohlen = (0.0, 0.0, 0.0)
        self._letzte_zeit: float | None = None
        self._verlauf: deque = deque()  # (Zeit, Scan-Zeit, Punkte, befohlene Lage)
        self._letzter_eintrag = -1e9

    def mitzaehlen(self, jetzt: float) -> None:
        """Die gerade ausgegebene Bewegung aufsummieren."""
        dt = 0.0 if self._letzte_zeit is None else min(max(jetzt - self._letzte_zeit, 0.0), 0.3)
        self._letzte_zeit = jetzt
        c = self._watchdog.current_command
        self._befohlen = _verknuepfen(self._befohlen, (c.vx * dt, c.vy * dt, c.omega * dt))

    def pruefen(self, jetzt: float, scan) -> Festgefahren | None:
        """Urteil, wenn er laut Lidar nicht vorankommt; sonst None."""
        if not config.FORTSCHRITT_AKTIV or scan is None:
            return None
        if jetzt - self._letzter_eintrag < config.FORTSCHRITT_TAKT_S:
            return None
        if self._verlauf and scan.zeit == self._verlauf[-1][1]:
            return None
        self._letzter_eintrag = jetzt
        punkte = _ausduennen(scan.punkte(), _MAX_PUNKTE)
        self._verlauf.append((jetzt, scan.zeit, punkte, self._befohlen))
        # Vorn bleibt die jüngste Aufnahme, die mindestens ein Fenster alt ist.
        fenster = config.FORTSCHRITT_FENSTER_S
        while len(self._verlauf) > 2 and self._verlauf[1][0] <= jetzt - fenster:
            self._verlauf.popleft()
        t0, _, referenz, befohlen0 = self._verlauf[0]
        if jetzt - t0 < 0.9 * fenster:
            return None

        erwartet = _verknuepfen(_umkehren(befohlen0), self._befohlen)
        weg = math.hypot(erwartet[0], erwartet[1])
        dreh = abs(erwartet[2])
        min_dreh = math.radians(config.FORTSCHRITT_MIN_DREH_GRAD)
        if weg < config.FORTSCHRITT_MIN_WEG_M and dreh < min_dreh:
            return None

        ab_null = match(punkte, referenz, (0.0, 0.0, 0.0))
        ab_befehl = match(punkte, referenz, erwartet)
        if ab_null is None or ab_befehl is None or ab_null.residuum > config.MATCH_MAX_RESIDUAL_M:
            return None
        if (math.hypot(ab_null.x - ab_befehl.x, ab_null.y - ab_befehl.y) > _EINIG_M
                or abs(_kuerzen(ab_null.theta - ab_befehl.theta)) > _EINIG_RAD):
            return None  # mehrdeutig, siehe oben

        gemessen_weg = math.hypot(ab_null.x, ab_null.y)
        gemessen_dreh = abs(ab_null.theta)
        anteil = config.FORTSCHRITT_ANTEIL
        if weg >= config.FORTSCHRITT_MIN_WEG_M and gemessen_weg < anteil * weg:
            urteil = Festgefahren(
                f"kommt laut Lidar nicht voran ({gemessen_weg * 100:.0f} von {weg * 100:.0f} cm)",
                math.degrees(math.atan2(erwartet[1], erwartet[0])),
            )
        elif dreh >= min_dreh and gemessen_dreh < anteil * dreh:
            urteil = Festgefahren(
                f"dreht sich laut Lidar nicht ({math.degrees(gemessen_dreh):.0f}° von "
                f"{math.degrees(dreh):.0f}°)",
                None,
            )
        else:
            return None
        self.urteile += 1
        log.warning("Fortschrittswache: %s", urteil.anlass)
        self.zuruecksetzen()
        return urteil
