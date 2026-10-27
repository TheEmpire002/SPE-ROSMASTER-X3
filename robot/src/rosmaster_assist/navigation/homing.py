"""Automatische Rückfahrt zum Startpunkt.

Nutzt aus, dass der X3 Mecanum-Räder hat: er kann in jede Richtung fahren,
ohne sich vorher zu drehen. Die Heimfahrt ist damit denkbar einfach --
Richtung zum Startpunkt ausrechnen, in diese Richtung fahren, fertig. Das
Drehen ist bei der Koppelnavigation die größte Fehlerquelle, und genau das
entfällt hier.

**Der Roboter weicht nichts aus.** Es gibt noch keine Hinderniserkennung;
er fährt stur die Verbindungslinie. Deshalb das gemächliche Tempo und die
Zeitbegrenzung. Mit dem Lidar ließe sich das später ergänzen.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from .odometry import Odometry

log = logging.getLogger(__name__)


class Homing:
    """Steuert den Roboter zurück zum Startpunkt."""

    def __init__(self, watchdog: SafetyWatchdog, odometry: Odometry) -> None:
        self._watchdog = watchdog
        self._odometry = odometry
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        """Warum die letzte Heimfahrt endete -- für die Anzeige in der App."""
        return self._grund

    def start(self) -> bool:
        """Heimfahrt beginnen. False, wenn schon eine läuft."""
        if self.aktiv:
            return False
        self._grund = None
        self._task = asyncio.create_task(self._fahren())
        return True

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        """Sofort beenden. Auch aufzurufen, wenn der Mensch eingreift."""
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._grund = grund
            log.info("Heimfahrt %s", grund)
        self._task = None

    async def _fahren(self) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        frist = time.monotonic() + config.HOME_TIMEOUT_S

        start_x, start_y, _ = self._odometry.pose()
        entfernung = math.hypot(start_x, start_y)
        log.info("Heimfahrt gestartet -- %.2f m zu fahren", entfernung)

        if not self._odometry.misst_tatsaechlich:
            log.warning(
                "Keine Bewegungsmessung verfügbar -- die Position stammt nur "
                "aus den Fahrbefehlen und ist entsprechend ungenau"
            )

        try:
            while True:
                x, y, winkel_grad = self._odometry.pose()
                abstand = math.hypot(x, y)

                if abstand <= config.HOME_TOLERANCE_M:
                    self._watchdog.stop()
                    self._grund = f"angekommen ({abstand:.2f} m Restabweichung)"
                    log.info("Heimfahrt beendet -- %s", self._grund)
                    return

                if time.monotonic() > frist:
                    self._watchdog.stop()
                    self._grund = f"Zeit abgelaufen, noch {abstand:.2f} m entfernt"
                    log.warning("Heimfahrt abgebrochen -- %s", self._grund)
                    return

                self._watchdog.submit(self._schritt(x, y, winkel_grad, abstand))
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            # Immer anhalten, egal warum abgebrochen wurde.
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            log.exception("Heimfahrt abgebrochen")

    def _schritt(self, x: float, y: float, winkel_grad: float, abstand: float) -> DriveCommand:
        """Fahrbefehl für diesen Moment."""
        # Richtung zum Startpunkt, in Weltkoordinaten.
        dx, dy = -x, -y

        # In Roboterkoordinaten drehen: der Antrieb kennt nur "vorwärts"
        # und "seitwärts", nicht "Norden".
        theta = math.radians(winkel_grad)
        vorwaerts = dx * math.cos(theta) + dy * math.sin(theta)
        seitwaerts = -dx * math.sin(theta) + dy * math.cos(theta)

        # Auf Länge 1 bringen und mit dem Tempo malnehmen. Kurz vor dem Ziel
        # abbremsen, sonst schießt er darüber hinaus und pendelt.
        tempo = config.HOME_SPEED
        if abstand < config.HOME_SLOWDOWN_M:
            anteil = abstand / config.HOME_SLOWDOWN_M
            # Nicht beliebig langsam werden -- sonst bleibt er vor dem Ziel
            # stehen, weil die Motoren bei sehr kleinen Werten nicht anlaufen.
            tempo *= max(0.35, anteil)

        laenge = math.hypot(vorwaerts, seitwaerts) or 1.0
        return DriveCommand(
            vx=vorwaerts / laenge * tempo,
            vy=seitwaerts / laenge * tempo,
            omega=0.0,  # Drehen ist unnötig: Mecanum fährt in jede Richtung.
        )
