"""Kollisionsvermeidung für die manuelle Steuerung und die autonomen Modi.

Zwei Klassen: `CollisionGuard` für die Handsteuerung (abschaltbar, siehe
unten) und `AutonomSchutz` für Folgen, Erkunden und Heimfahrt (immer an,
siehe dort).

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


class AutonomSchutz:
    """Letzte Bremse für alle autonomen Fahrbefehle.

    Folgen, Erkunden und Heimfahrt prüfen Hindernisse jeweils selbst, aber
    jeder auf seine Art, mit eigenen Abständen und eigenem Scan-Alter. Diese
    Klasse sitzt im `SafetyWatchdog` und prüft *jeden* autonomen Befehl
    gleich: Kann der Roboter bei diesem Tempo vor dem nächsten Hindernis in
    Fahrtrichtung noch anhalten?

    Erlaubt ist das Tempo, bei dem gilt

        frei - Rand >= v * t + v² / (2 * a)

    mit `t` = Latenz + Alter des Scans (in dieser Zeit fährt der Roboter
    blind weiter) und `a` = angenommene Bremsverzögerung. Die Formel nach
    `v` aufgelöst ergibt die Obergrenze; ist der Befehl schneller, wird
    (vx, vy) anteilig verkleinert, die Richtung bleibt. Drehen bleibt
    unberührt -- auf der Stelle nähert sich der Roboter nichts an, und ein
    Modus muss sich aus einer Ecke herausdrehen können.

    Im Normalbetrieb greift die Schicht nicht ein: Die Modi halten selbst
    früher an (`EXPLORE_STOP_M`, `HOME_STOP_DISTANCE_M`) und fahren langsam
    an Hindernisse heran. Sie fängt das ab, was die Modi nicht sehen: ein
    Hindernis, das plötzlich auftaucht, einen Lidar, der keine frischen
    Daten mehr liefert, und Rechenfehler.

    Bewusst **kein** Stopp an der Rampe vorbei: Die Obergrenze sinkt
    stetig, je näher das Hindernis kommt, und die Bremsrampe des Watchdogs
    (`RAMPE_BREMSEN_XY`) ist schneller als die hier angenommene
    Verzögerung. Ein harter Stopp aus voller Fahrt erzeugt die Stromspitzen,
    die am X3 den USB-Hub zurücksetzen -- und damit den Lidar, auf den sich
    diese Schicht verlässt.
    """

    #: Eingriffe höchstens so oft protokollieren. Die Schicht läuft im
    #: 20-Hz-Takt; bei einem stehenden Hindernis käme sonst jede Meldung
    #: zwanzigmal pro Sekunde.
    MELDE_ABSTAND_S = 3.0

    def __init__(self, lidar) -> None:
        self._lidar = lidar
        self.eingriffe = 0
        self._zuletzt_gemeldet = -1e9
        # Punkte nur einmal pro Scan umrechnen: der Lidar liefert 7,5
        # Umdrehungen pro Sekunde, gefragt wird 20-mal.
        self._scan = None
        self._punkte = None

    def zustand(self) -> dict:
        return {
            "aktiv": bool(config.AUTONOM_SCHUTZ_AKTIV),
            "eingriffe": self.eingriffe,
        }

    def erlaubtes_tempo(self, frei: float, alter_s: float) -> float:
        """Höchsttempo (m/s), bei dem der Roboter vor `frei` noch steht."""
        netto = frei - config.AUTONOM_SCHUTZ_RAND_M
        if netto <= 0.0:
            return 0.0
        if math.isinf(netto):
            return float("inf")
        a = config.AUTONOM_SCHUTZ_BREMSEN
        t = config.AUTONOM_SCHUTZ_LATENZ_S + max(0.0, alter_s)
        # Positive Lösung von v*t + v²/(2a) = netto.
        return a * (-t + math.sqrt(t * t + 2.0 * netto / a))

    def pruefen(self, cmd: DriveCommand, jetzt: float | None = None) -> DriveCommand:
        """Gibt den erlaubten Befehl zurück (Drehung immer unverändert)."""
        tempo = math.hypot(cmd.vx, cmd.vy)
        if tempo < 1e-6:
            return cmd
        if jetzt is None:
            jetzt = time.monotonic()

        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or jetzt - scan.zeit > config.AUTONOM_SCHUTZ_SCAN_MAX_S:
            # Ohne frische Lidar-Daten weiß niemand, was vorn ist. Drehen
            # darf er weiter, fahren nicht.
            self._eingriff(jetzt, "keine frischen Lidar-Daten")
            return DriveCommand(0.0, 0.0, cmd.omega)

        if scan is not self._scan:
            self._scan = scan
            self._punkte = scan.punkte()

        # Später Import: explorer.py importiert den Watchdog, und der
        # importiert dieses Modul.
        from ..navigation.explorer import frei_in_richtung

        richtung = math.degrees(math.atan2(cmd.vy, cmd.vx))
        frei = frei_in_richtung(
            self._punkte, richtung, config.AUTONOM_SCHUTZ_HALBE_BREITE_M
        )
        # Der älteste Punkt des Scans ist eine ganze Umdrehung älter als
        # dessen Ende -- mit dem rechnen, nicht mit dem Mittel.
        alter = jetzt - (scan.zeit - scan.dauer)
        v_max = self.erlaubtes_tempo(frei, alter)
        if tempo <= v_max:
            return cmd

        faktor = v_max / tempo
        self._eingriff(
            jetzt,
            "Hindernis %.2f m in Richtung %.0f°, Tempo %.2f -> %.2f m/s"
            % (frei, richtung, tempo, v_max),
        )
        return DriveCommand(cmd.vx * faktor, cmd.vy * faktor, cmd.omega)

    def _eingriff(self, jetzt: float, grund: str) -> None:
        self.eingriffe += 1
        if jetzt - self._zuletzt_gemeldet >= self.MELDE_ABSTAND_S:
            self._zuletzt_gemeldet = jetzt
            log.warning("Autonomer Fahrbefehl gebremst: %s", grund)
