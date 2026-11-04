"""Kollisionsvermeidung für die manuelle Steuerung.

Die autonomen Modi prüft der Fahrschutz direkt im Watchdog
(``quelle="autonom"``, immer an). Bis 03.11. tat das hier ``AutonomSchutz``
mit einem Streifen in Fahrtrichtung; der kannte keine Drehung und keinen
Bogen und ist im Fahrschutz aufgegangen.

Prüft vor jedem Fahrbefehl über den Fahrschutz (``fahrschutz.py``), ob die
Grundfläche des Roboters etwas berühren würde, und bremst rechtzeitig --
stufenlos, nicht erst an einer harten Grenze. Früher war das ein Kegel ab
Lidar-Mitte mit hartem Stopp; der übersah Tischbeine neben den Rädern,
ließ den Roboter mit vollem Tempo heranrollen und kannte keine Drehung.

Entwurfsentscheidungen, die das Verhalten prägen:

**Nur gesperrt, was näher heranführt.** Wegfahren und Drehen, bei dem keine
Ecke anstößt, bleiben erlaubt; sobald der Mensch in eine andere Richtung
steuert, fährt der Roboter wieder. Ein pauschaler Stopp würde ihn vor jedem
Hindernis festnageln -- man müsste ihn tragen, um ihn zu befreien.

**Serverseitig, nicht in der App.** Die Lidar-Daten liegen hier, und eine
Sperre, die sich über die Browser-Konsole umgehen lässt, wäre keine.

Standardmäßig **aus**: Wer von Hand steuert, sieht den Roboter meist und
will nicht bevormundet werden. Einschalten lässt sich das in der App.
"""

from __future__ import annotations

import logging
import time

from .. import config
from ..protocol import STOP, DriveCommand
from .fahrschutz import KEIN_LIDAR

log = logging.getLogger(__name__)


class CollisionGuard:
    """Filtert Fahrbefehle anhand der Lidar-Daten."""

    #: Nicht häufiger als alle paar Sekunden melden -- sonst flutet die
    #: Meldung bei gedrückter Taste das Protokoll in der App.
    MELDE_ABSTAND_S = 2.0

    def __init__(self, localizer=None, fahrschutz=None) -> None:
        self._localizer = localizer
        self._fahrschutz = fahrschutz
        self.enabled = config.COLLISION_ENABLED
        self.distance = config.COLLISION_DISTANCE_M
        self._blockiert_seit: float | None = None
        self._zuletzt_gemeldet = 0.0

    @property
    def verfuegbar(self) -> bool:
        """Ob überhaupt ein Sensor da ist, der Hindernisse sehen könnte."""
        return self._fahrschutz is not None and (
            self._localizer is None or self._localizer.verfuegbar
        )

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
        if not self.enabled or self._fahrschutz is None:
            self._blockiert_seit = None
            return cmd, None

        # Der Regler in der App meint den Abstand ab Lidar-Mitte nach vorn.
        # Als Luft an der Roboterkante ist das rundum derselbe Rand.
        rand = max(0.02, self.distance - config.ROBOTER_VORN_M)
        pruefung = self._fahrschutz.pruefen(cmd, rand)

        if pruefung.grund == KEIN_LIDAR:
            # Ohne Lidar kann der Schutz nicht helfen. Wer von Hand steuert,
            # sieht den Roboter -- also nicht auch noch die Handsteuerung
            # lahmlegen. Selbstständige Fahrten halten dagegen an.
            self._blockiert_seit = None
            return cmd, None
        if pruefung.anteil > 0.0:
            # Frei oder nur gebremst: kein Grund für eine Meldung.
            self._blockiert_seit = None
            return pruefung.befehl, None

        # Die Fahrt ist gesperrt. Die Drehung allein darf bleiben, wenn
        # dabei keine Ecke anstößt -- mit knappem Rand, damit man sich auch
        # dicht vor einer Wand noch herausdrehen kann.
        rest = STOP
        if abs(cmd.omega) > 1e-6 and (abs(cmd.vx) > 1e-6 or abs(cmd.vy) > 1e-6):
            drehen = self._fahrschutz.pruefen(DriveCommand(0.0, 0.0, cmd.omega), min(rand, 0.03))
            if drehen.anteil > 0.0:
                rest = drehen.befehl
        if self._blockiert_seit is None:
            self._blockiert_seit = time.monotonic()
        return rest, (pruefung.frei_m or 0.0) + rand

    def melden(self) -> bool:
        """Ob jetzt eine Meldung an die App sinnvoll ist (entprellt)."""
        jetzt = time.monotonic()
        if jetzt - self._zuletzt_gemeldet < self.MELDE_ABSTAND_S:
            return False
        self._zuletzt_gemeldet = jetzt
        return True
