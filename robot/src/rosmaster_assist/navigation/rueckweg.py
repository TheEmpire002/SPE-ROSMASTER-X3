"""Die gefahrene Strecke merken, um auf ihr zurückzufinden.

Die Luftlinie zum Startpunkt ist oft versperrt: ist der Roboter durch eine
Tür gefahren, liegt dazwischen eine Wand. Der Weg, den er gekommen ist, war
dagegen nachweislich befahrbar. Also wird er unterwegs als Folge von
Wegpunkten aufgezeichnet und bei der Heimfahrt rückwärts abgefahren.

Zwei Vereinfachungen halten den Weg kurz:

- **Schleifen fallen weg.** Kommt der Roboter an einer Stelle vorbei, an der
  er schon war, wird alles dazwischen gestrichen. Zurück geht es nicht
  jeden Umweg noch einmal.
- **Abkürzen bei freier Sicht** übernimmt die Heimfahrt: liegt ein späterer
  Wegpunkt in Sichtweite, fährt sie direkt dorthin (siehe ``homing.py``).
"""

from __future__ import annotations

import math
import threading

from .. import config


class Rueckweg:
    """Wegpunkte vom Startpunkt bis zur aktuellen Position."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._punkte: list[tuple[float, float]] = [(0.0, 0.0)]

    def zuruecksetzen(self) -> None:
        """Neuer Startpunkt: der alte Weg gilt nicht mehr."""
        with self._lock:
            self._punkte = [(0.0, 0.0)]

    def aufzeichnen(self, x: float, y: float) -> None:
        """Aktuelle Position melden. Regelmäßig aufrufen, auch im Stand."""
        with self._lock:
            lx, ly = self._punkte[-1]
            if math.hypot(x - lx, y - ly) < config.RUECKWEG_PUNKTABSTAND_M:
                return

            # Schleife? Den ältesten Wegpunkt in der Nähe suchen -- die
            # letzten paar zählen nicht, die liegen naturgemäß nah.
            for i, (px, py) in enumerate(self._punkte[:-3]):
                if math.hypot(x - px, y - py) < config.RUECKWEG_SCHLEIFE_M:
                    del self._punkte[i + 1:]
                    break
            self._punkte.append((x, y))

    def punkte(self) -> list[tuple[float, float]]:
        """Kopie, vom Startpunkt (Index 0) bis zum letzten Wegpunkt."""
        with self._lock:
            return list(self._punkte)

    def kuerzen(self, bis_index: int) -> None:
        """Alles hinter ``bis_index`` streichen -- ist schon abgefahren."""
        with self._lock:
            del self._punkte[max(1, bis_index + 1):]

    def laenge_m(self) -> float:
        with self._lock:
            p = self._punkte
            return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:]))

    def __len__(self) -> int:
        with self._lock:
            return len(self._punkte)
