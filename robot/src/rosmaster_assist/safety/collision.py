"""Kollisionsvermeidung für die manuelle Steuerung.

Prüft vor jedem Fahrbefehl, ob in Fahrtrichtung etwas zu nah ist, und
unterdrückt in dem Fall die Bewegung.

Zwei Entwurfsentscheidungen, die das Verhalten prägen:

**Nur die Fahrtrichtung wird gesperrt, nicht alles.** Drehen bleibt immer
erlaubt, und sobald der Mensch in eine andere Richtung steuert, fährt der
Roboter wieder. Ein pauschaler Stopp würde ihn vor jedem Hindernis
festnageln -- man müsste ihn tragen, um ihn zu befreien.

**Serverseitig, nicht in der App.** Die Lidar-Daten liegen hier, und eine
Sperre, die sich über die Browser-Konsole umgehen lässt, wäre keine.

Standardmäßig **aus**: Wer von Hand steuert, sieht den Roboter meist und
will nicht bevormundet werden. Einschalten lässt sich das in der App.
"""

from __future__ import annotations

import logging
import math
import time

from .. import config
from ..protocol import DriveCommand

log = logging.getLogger(__name__)


class CollisionGuard:
    """Filtert Fahrbefehle anhand der Lidar-Daten."""

    #: Nicht häufiger als alle paar Sekunden melden -- sonst flutet die
    #: Meldung bei gedrückter Taste das Protokoll in der App.
    MELDE_ABSTAND_S = 2.0

    def __init__(self, localizer=None) -> None:
        self._localizer = localizer
        self.enabled = config.COLLISION_ENABLED
        self.distance = config.COLLISION_DISTANCE_M
        self._blockiert_seit: float | None = None
        self._zuletzt_gemeldet = 0.0

    @property
    def verfuegbar(self) -> bool:
        """Ob überhaupt ein Sensor da ist, der Hindernisse sehen könnte."""
        return self._localizer is not None and self._localizer.verfuegbar

    @property
    def blockiert(self) -> bool:
        return self._blockiert_seit is not None

    def einstellen(self, enabled: bool | None, distance: float | None) -> None:
        if enabled is not None:
            self.enabled = bool(enabled)
            if not self.enabled:
                self._blockiert_seit = None
        if distance is not None:
            # Begrenzen, damit niemand versehentlich einen Wert setzt, bei
            # dem der Roboter sich überhaupt nicht mehr bewegt.
            self.distance = max(
                config.COLLISION_MIN_M, min(config.COLLISION_MAX_M, float(distance))
            )
        log.info(
            "Kollisionsvermeidung: %s, Mindestabstand %.2f m",
            "an" if self.enabled else "aus",
            self.distance,
        )

    def zustand(self) -> dict:
        return {
            "enabled": self.enabled,
            "distance": round(self.distance, 2),
            "verfuegbar": self.verfuegbar,
            "blockiert": self.blockiert,
        }

    def pruefen(self, cmd: DriveCommand) -> tuple[DriveCommand, float | None]:
        """Gibt den erlaubten Befehl zurück, dazu den Hindernisabstand.

        Der zweite Wert ist None, wenn nichts im Weg war.
        """
        if not self.enabled or self._localizer is None:
            self._blockiert_seit = None
            return cmd, None

        # Reines Drehen auf der Stelle ist immer erlaubt: dabei nähert sich
        # der Roboter keinem Hindernis an.
        if abs(cmd.vx) < 1e-6 and abs(cmd.vy) < 1e-6:
            self._blockiert_seit = None
            return cmd, None

        richtung = math.degrees(math.atan2(cmd.vy, cmd.vx))
        abstand = self._localizer.hindernis_voraus(
            richtung, config.COLLISION_CONE_DEG
        )

        if abstand is None or abstand >= self.distance:
            self._blockiert_seit = None
            return cmd, None

        # Sperren -- aber nur die Fahrt, nicht die Drehung.
        if self._blockiert_seit is None:
            self._blockiert_seit = time.monotonic()
        return DriveCommand(0.0, 0.0, cmd.omega), abstand

    def melden(self) -> bool:
        """Ob jetzt eine Meldung an die App sinnvoll ist (entprellt)."""
        jetzt = time.monotonic()
        if jetzt - self._zuletzt_gemeldet < self.MELDE_ABSTAND_S:
            return False
        self._zuletzt_gemeldet = jetzt
        return True
