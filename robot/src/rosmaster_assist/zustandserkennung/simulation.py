"""Nur im Mock: eine liegende Person in den gedachten Raum legen.

Kamera (Pose-Simulation) und Lidar (Zusatzhindernisse) sehen dieselbe
Person an derselben Stelle, abhängig davon, wo der Roboter *wirklich*
steht. So lässt sich der ganze Ablauf des Healthbots am Schreibtisch
durchspielen: hinfahren, suchen, beobachten, anstupsen.
"""

from __future__ import annotations

import math

#: Wo die Person liegt (Weltkoordinaten, m) -- frei im gedachten Raum,
#: weg von Wand und Säule.
STANDARD_ORT = (2.5, -1.0)
#: Der Körper als Kette sich überlappender Kreise quer zur Blickrichtung
#: vom Start aus: Kopf/Oberkörper, Hüfte, Beine. Etwa 1,4 m lang, so wie ein
#: liegender Mensch. Die Kreise überlappen, damit der Lidar wie bei einem
#: echten Körper ein durchgehendes Stück sieht und keine Einzelteile.
_KREISE = ((0.0, -0.55, 0.15), (0.0, -0.275, 0.17), (0.0, 0.0, 0.17),
           (0.0, 0.275, 0.15), (0.0, 0.55, 0.13))

SZENEN = ("liegend_regungslos", "liegend_bewegt")


class Simulation:
    def __init__(self, pose, wahre_pose, hindernisse_setzen=None) -> None:
        """``wahre_pose()`` liefert (x, y, theta im Bogenmaß) des Mock-Antriebs,
        ``hindernisse_setzen(kreise)`` kommt vom Mock-Lidar."""
        self._pose = pose
        self._wahre_pose = wahre_pose
        self._hindernisse_setzen = hindernisse_setzen
        self._szene: str | None = None
        self._ort = STANDARD_ORT

    def status(self) -> dict:
        return {"szene": self._szene, "x": self._ort[0], "y": self._ort[1]}

    def kreise(self) -> list[tuple[float, float, float]]:
        x, y = self._ort
        return [(x + dx, y + dy, r) for dx, dy, r in _KREISE]

    def setzen(self, szene: str | None, x: float | None = None, y: float | None = None) -> None:
        """Szene setzen (None = keine Person mehr)."""
        if szene is not None and szene not in SZENEN:
            raise ValueError(f"unbekannte Szene {szene!r} (erlaubt: {', '.join(SZENEN)})")
        if x is not None and y is not None:
            self._ort = (float(x), float(y))
        self._szene = szene
        if self._hindernisse_setzen is not None:
            self._hindernisse_setzen(self.kreise() if szene else [])
        self._pose.simulation_setzen(szene, self._lage if szene else None)

    def _lage(self) -> tuple[float, float] | None:
        """(Winkel in Grad, links positiv; Abstand zur Körperkante in m) vom
        Roboter aus -- das, was Kamera und Lidar von hier sähen."""
        rx, ry, theta = self._wahre_pose()
        x, y = self._ort
        winkel = math.degrees(math.atan2(y - ry, x - rx) - theta)
        winkel = (winkel + 180.0) % 360.0 - 180.0
        abstand = min(math.hypot(kx - rx, ky - ry) - r for kx, ky, r in self.kreise())
        return winkel, max(0.05, abstand)
