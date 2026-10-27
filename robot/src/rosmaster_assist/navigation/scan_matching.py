"""Zwei Lidar-Aufnahmen übereinanderlegen (ICP).

Die Idee: Beim Setzen des Startpunkts wird einmal aufgenommen, wie der Raum
von dort aussieht. Später wird die aktuelle Aufnahme so lange verschoben und
gedreht, bis sie möglichst genau auf die Referenz passt. Die dafür nötige
Verschiebung *ist* die Position des Roboters relativ zum Startpunkt.

Der große Unterschied zur Koppelnavigation: Dieses Verfahren summiert nichts
auf. Jede Messung ist für sich gültig, ein Fehler pflanzt sich nicht fort.
Durchdrehende Räder sind ihm egal -- es schaut auf die Wände.

Verfahren: Iterative Closest Point. Jeder Punkt wird dem nächstgelegenen
Referenzpunkt zugeordnet, daraus die beste Drehung samt Verschiebung
berechnet, und das Ganze einige Male wiederholt. Das braucht einen
brauchbaren Startwert -- den liefert die Koppelnavigation.
"""

from __future__ import annotations

import logging
import math

log = logging.getLogger(__name__)

#: Weiter entfernte Punktpaare gelten als nicht zusammengehörig. Zu groß
#: gewählt zieht Fremdes die Lösung schief, zu klein findet nichts.
MAX_PAAR_ABSTAND_M = 0.5

#: Unter so vielen Paaren ist das Ergebnis nicht aussagekräftig.
MIN_PAARE = 40


class MatchResult:
    """Ergebnis eines Abgleichs."""

    __slots__ = ("x", "y", "theta", "residuum", "paare")

    def __init__(self, x: float, y: float, theta: float, residuum: float, paare: int):
        self.x = x
        self.y = y
        self.theta = theta          # Bogenmaß
        self.residuum = residuum    # mittlerer Restabstand in Metern
        self.paare = paare

    def __repr__(self) -> str:
        return (
            f"MatchResult(x={self.x:.3f}, y={self.y:.3f}, "
            f"theta={math.degrees(self.theta):.1f}°, "
            f"residuum={self.residuum:.3f} m, paare={self.paare})"
        )


def match(
    aktuell,
    referenz,
    start: tuple[float, float, float],
    iterationen: int = 25,
) -> MatchResult | None:
    """Sucht die Lage, in der ``aktuell`` am besten auf ``referenz`` passt.

    ``aktuell``  Nx2-Array, Punkte im Roboterkoordinatensystem
    ``referenz`` Mx2-Array, Punkte im Koordinatensystem des Startpunkts
    ``start``    (x, y, Winkel im Bogenmaß) als erster Anhaltspunkt

    Gibt None zurück, wenn zu wenige Punkte zueinander finden -- etwa weil
    der Roboter inzwischen in einem anderen Raum steht.
    """
    import numpy as np

    if aktuell is None or referenz is None:
        return None
    if len(aktuell) < MIN_PAARE or len(referenz) < MIN_PAARE:
        return None

    A = np.asarray(aktuell, dtype=float)
    R = np.asarray(referenz, dtype=float)

    x, y, theta = start
    verschiebung = np.array([x, y], dtype=float)

    residuum = float("inf")
    paare = 0

    for _ in range(iterationen):
        # Aktuelle Schätzung anwenden.
        c, s = math.cos(theta), math.sin(theta)
        dreh = np.array([[c, -s], [s, c]])
        gedreht = A @ dreh.T + verschiebung

        # Für jeden Punkt den nächsten Referenzpunkt suchen. Bei einigen
        # hundert Punkten ist die vollständige Abstandsmatrix schneller als
        # jede Baumstruktur -- und kommt ohne Zusatzpaket aus.
        diff = gedreht[:, None, :] - R[None, :, :]
        abstaende = np.einsum("ijk,ijk->ij", diff, diff)  # quadriert, spart Wurzel
        naechster = np.argmin(abstaende, axis=1)
        min_quadrat = abstaende[np.arange(len(gedreht)), naechster]

        gueltig = min_quadrat < MAX_PAAR_ABSTAND_M**2
        paare = int(gueltig.sum())
        if paare < MIN_PAARE:
            return None

        P = gedreht[gueltig]
        Q = R[naechster[gueltig]]
        residuum = float(np.sqrt(min_quadrat[gueltig]).mean())

        # Beste Drehung samt Verschiebung für diese Zuordnung, geschlossen
        # lösbar (Kabsch in der Ebene).
        pm, qm = P.mean(axis=0), Q.mean(axis=0)
        H = (P - pm).T @ (Q - qm)
        dtheta = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])

        dc, ds = math.cos(dtheta), math.sin(dtheta)
        ddreh = np.array([[dc, -ds], [ds, dc]])
        dverschiebung = qm - ddreh @ pm

        # Die Korrektur auf die bisherige Schätzung draufrechnen.
        verschiebung = ddreh @ verschiebung + dverschiebung
        theta = _normalisieren(theta + dtheta)

        # Bewegt sich fast nichts mehr, bringt Weiterrechnen nichts.
        if abs(dtheta) < 1e-4 and float(np.hypot(*dverschiebung)) < 1e-4:
            break

    return MatchResult(float(verschiebung[0]), float(verschiebung[1]), theta, residuum, paare)


def _normalisieren(winkel: float) -> float:
    return math.atan2(math.sin(winkel), math.cos(winkel))
