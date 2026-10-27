"""Mitrechnen, wo der Roboter steht.

Verfahren: Koppelnavigation. Es gibt keinen Sensor, der die Position
verrät -- sie wird laufend aus Geschwindigkeit und Blickrichtung
aufsummiert, ausgehend vom Startpunkt.

**Das driftet, und zwar unvermeidlich.** Jeder Messfehler und jedes
durchdrehende Rad geht dauerhaft in die Summe ein und wird nie wieder
korrigiert. Mecanum-Räder rutschen bauartbedingt viel, besonders beim
Drehen und auf glattem Boden. In einem Zimmer sind einige Dezimeter normal.

Für den Home-Button reicht das. Wer Zentimeter braucht, kommt um eine
äußere Referenz nicht herum -- Lidar und SLAM.

Koordinaten: x zeigt in die anfängliche Blickrichtung, y nach links,
Winkel gegen den Uhrzeigersinn. Startpunkt ist (0, 0) mit Winkel 0.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque

from .. import config
from ..drive.base import DriveBase
from ..protocol import DriveCommand

log = logging.getLogger(__name__)


class Odometry:
    """Schreibt die Position aus den Bewegungsdaten fort."""

    def __init__(self, driver: DriveBase) -> None:
        self._driver = driver
        self._lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Bogenmaß

        self._letzte_zeit: float | None = None
        # Rohwert der IMU beim Nullen -- der Winkel wird relativ dazu geführt,
        # weil die IMU ihren eigenen Nullpunkt hat.
        self._imu_referenz: float | None = None
        self._imu_versatz = 0.0
        # Drehsinn der IMU, siehe config.IMU_YAW_SIGN. Bis er feststeht,
        # wird mit der Drehrate gerechnet statt mit einem womöglich
        # spiegelverkehrten Winkel.
        self._imu_vorzeichen = config.IMU_YAW_SIGN or 1
        self._imu_entschieden = config.IMU_YAW_SIGN != 0
        self._imu_beleg = 0.0
        self._imu_letzt: float | None = None
        self._gemessen = False
        # Kurzer Rückblick für den Lidar-Abgleich: (Zeit, x, y, Winkel).
        # Zwei Sekunden reichen -- eine Lidar-Umdrehung dauert 0,2 s.
        self._verlauf: deque = deque(maxlen=120)

        # Stirbt der Empfangs-Thread der Bibliothek (etwa durch einen
        # Portkonflikt), liefert sie weiterhin Werte -- nämlich Nullen.
        # Das wäre schlimmer als gar keine Messung: die Position bliebe
        # stehen, und die Heimfahrt führte ins Leere, ohne zu warnen.
        self._stillstand_trotz_befehl = 0.0
        self._messung_verworfen = False

    # --- Abfrage -------------------------------------------------------------

    def pose(self) -> tuple[float, float, float]:
        """(x, y, Winkel in Grad)."""
        with self._lock:
            return self.x, self.y, math.degrees(self.theta)

    @property
    def misst_tatsaechlich(self) -> bool:
        """Ob echte Messwerte eingehen oder nur die Befehle gerechnet werden."""
        return self._gemessen

    def distanz_zum_start(self) -> float:
        with self._lock:
            return math.hypot(self.x, self.y)

    def pose_bei(self, zeitpunkt: float) -> tuple[float, float, float] | None:
        """Wo der Roboter zu einem früheren Zeitpunkt stand (x, y, Bogenmaß).

        Nötig für den Lidar-Abgleich: eine Aufnahme ist beim Auswerten
        bereits ein paar Zehntelsekunden alt. Ohne diesen Rückblick würde
        die Korrektur den Roboter auf eine Stelle ziehen, die er längst
        verlassen hat -- bei 0,3 m/s sind das schon mehrere Zentimeter.
        """
        with self._lock:
            if not self._verlauf:
                return None
            # Rückwärts suchen: der gesuchte Zeitpunkt liegt fast immer
            # am Ende des Verlaufs.
            bester = self._verlauf[0]
            for eintrag in reversed(self._verlauf):
                if eintrag[0] <= zeitpunkt:
                    bester = eintrag
                    break
            return bester[1], bester[2], bester[3]

    def korrigieren(self, x: float, y: float, theta: float, gewicht: float) -> None:
        """Position Richtung eines gemessenen Werts ziehen.

        Kommt vom Lidar-Abgleich. Bewusst nicht hart überschreiben: ein
        einzelner Fehlabgleich würde die Position sonst verreißen und die
        Heimfahrt in die Irre schicken. Über mehrere Durchgänge nähert sich
        die Schätzung dem gemessenen Wert an.
        """
        gewicht = max(0.0, min(1.0, gewicht))
        with self._lock:
            dx = (x - self.x) * gewicht
            dy = (y - self.y) * gewicht
            # Winkel nicht einfach mitteln: zwischen 179° und -179° liegt
            # ein Grad, nicht 358. Über die Differenz gehen.
            dtheta = _normalisieren(theta - self.theta) * gewicht

            self.x += dx
            self.y += dy
            self.theta = _normalisieren(self.theta + dtheta)

            # Den Verlauf mitverschieben. Sonst stünden dort Positionen aus
            # der Zeit vor der Korrektur, und die Latenzrechnung im
            # Lidar-Abgleich würde die Korrektur selbst für zurückgelegte
            # Strecke halten -- die Position schaukelt sich dann mit jedem
            # Durchgang weiter auf, statt sich einzupendeln.
            self._verlauf = deque(
                ((t, vx + dx, vy + dy, _normalisieren(vt + dtheta))
                 for t, vx, vy, vt in self._verlauf),
                maxlen=self._verlauf.maxlen,
            )

            # Nach einer Korrektur ist der IMU-Bezug nicht mehr stimmig --
            # er wird beim nächsten Schritt neu gesetzt.
            if self._imu_referenz is not None:
                self._imu_referenz = None

    # --- Nullen --------------------------------------------------------------

    def reset(self) -> None:
        """Hier ist jetzt der Startpunkt.

        Setzt auch den IMU-Bezug neu, damit der Winkel wieder bei null
        beginnt -- sonst zeigte der Roboter nach dem Nullen in eine
        Richtung, die es nie gab.
        """
        with self._lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self._imu_referenz = None
            self._imu_versatz = 0.0
            self._letzte_zeit = None
            # Der Messung eine neue Chance geben -- vielleicht wurde der
            # Portkonflikt inzwischen behoben.
            self._stillstand_trotz_befehl = 0.0
            self._messung_verworfen = False
            self._verlauf.clear()
        log.info("Startpunkt neu gesetzt")

    # --- Fortschreiben --------------------------------------------------------

    def update(self, befohlen: DriveCommand) -> None:
        """Einen Zeitschritt rechnen. Regelmäßig aufrufen."""
        jetzt = time.monotonic()
        if self._letzte_zeit is None:
            self._letzte_zeit = jetzt
            return

        dt = jetzt - self._letzte_zeit
        self._letzte_zeit = jetzt
        # Nach einer Pause (Server hing, Rechner im Ruhezustand) wäre der
        # Schritt riesig und würde die Position verreißen.
        if dt <= 0 or dt > 1.0:
            return

        gemessen = self._pruefen(self._driver.read_motion(), befohlen, dt)
        self._gemessen = gemessen is not None
        vx, vy, omega = gemessen if gemessen is not None else (
            befohlen.vx,
            befohlen.vy,
            befohlen.omega,
        )

        with self._lock:
            neuer_winkel = self._winkel_bestimmen(omega, dt, befohlen.omega)

            # In der Mitte des Zeitschritts auswerten: bei gleichzeitigem
            # Fahren und Drehen ist das deutlich genauer, als Anfangs- oder
            # Endwinkel zu nehmen.
            mittel = (self.theta + neuer_winkel) / 2.0
            self.x += (vx * math.cos(mittel) - vy * math.sin(mittel)) * dt
            self.y += (vx * math.sin(mittel) + vy * math.cos(mittel)) * dt
            self.theta = neuer_winkel
            self._verlauf.append((jetzt, self.x, self.y, self.theta))

    #: So lange darf gemeldeter Stillstand trotz Fahrbefehl andauern, bevor
    #: die Messung als unbrauchbar gilt. Großzügig, weil Motoren anlaufen
    #: müssen und der Roboter kurz blockieren kann.
    STILLSTAND_GRENZE_S = 3.0

    def _pruefen(
        self, gemessen: tuple[float, float, float] | None, befohlen: DriveCommand, dt: float
    ) -> tuple[float, float, float] | None:
        """Verwirft Messwerte, die offensichtlich nicht stimmen können.

        Meldet die Bibliothek beharrlich Stillstand, obwohl gefahren werden
        soll, ist ihr Empfangs-Thread vermutlich gestorben. Dann ist es
        besser, mit den Befehlen zu rechnen: ungenau, aber nicht falsch.
        """
        if gemessen is None:
            return None

        if self._messung_verworfen:
            return None

        bewegt_sich = any(abs(v) > 1e-6 for v in gemessen)
        soll_fahren = not befohlen.is_stop()

        if soll_fahren and not bewegt_sich:
            self._stillstand_trotz_befehl += dt
            if self._stillstand_trotz_befehl > self.STILLSTAND_GRENZE_S:
                self._messung_verworfen = True
                log.warning(
                    "Bewegungsmessung meldet seit %.0f s Stillstand trotz "
                    "Fahrbefehl -- vermutlich ist der Empfangs-Thread tot. "
                    "Es wird ab jetzt mit den Fahrbefehlen gerechnet "
                    "(ungenauer). Ursache: siehe docs/hardware.md, Portkonflikt.",
                    self._stillstand_trotz_befehl,
                )
                return None
        else:
            self._stillstand_trotz_befehl = 0.0

        return gemessen

    def _winkel_bestimmen(self, omega: float, dt: float, befohlen_omega: float = 0.0) -> float:
        """Blickrichtung aus der IMU, ersatzweise aus der Drehrate."""
        imu = self._driver.read_heading()
        if imu is not None:
            # Fürs Vorzeichen zählt der Befehl, wenn es einen gibt: der
            # ist sicher richtig herum, die Encoder melden beim Drehen
            # mitunter fast nichts.
            vergleich = befohlen_omega if abs(befohlen_omega) > 0.2 else omega
            self._imu_drehsinn_pruefen(imu, vergleich, dt)
        if imu is None or not self._imu_entschieden:
            # Aufsummieren: der Fehler wächst mit jeder Drehung.
            return _normalisieren(self.theta + omega * dt)
        imu *= self._imu_vorzeichen

        if self._imu_referenz is None:
            # Neuer Bezugspunkt. Der aktuelle Winkel wird als Versatz
            # gemerkt, sonst ginge eine Korrektur vom Lidar beim nächsten
            # Schritt wieder verloren -- die IMU würde ihren eigenen
            # Nullpunkt durchsetzen.
            self._imu_referenz = imu
            self._imu_versatz = self.theta
            return self.theta

        # Die Differenz bilden und nicht die Rohwerte vergleichen: sonst
        # springt der Winkel beim Übergang von +180° auf -180° um eine
        # halbe Umdrehung.
        seit_bezug = _normalisieren(math.radians(imu - self._imu_referenz))
        return _normalisieren(self._imu_versatz + seit_bezug)

    #: So viel Drehung (rad) muss übereinstimmend gemessen sein, bevor der
    #: Drehsinn als geklärt gilt. Rund 45°: genug, um Rauschen und Schlupf
    #: auszumitteln, wenig genug, dass es bei der ersten Kurve passiert.
    IMU_BELEG_GRENZE = 0.8

    def _imu_drehsinn_pruefen(self, imu: float, omega: float, dt: float) -> None:
        """Vergleicht die IMU-Drehung mit der Drehrate von Encodern/Befehl.

        Zählt die IMU andersherum als die Software, wäre jede Positions-
        schätzung nach einer Kurve gespiegelt -- genau das Fehlerbild "nach
        dem Drehen kommt er weit daneben raus". Die Encoder sind beim Drehen
        ungenau (Mecanum rutscht), haben aber verlässlich das richtige
        Vorzeichen.
        """
        letzt, self._imu_letzt = self._imu_letzt, imu
        if self._imu_entschieden or letzt is None or abs(omega) < 0.2:
            return
        d_imu = math.radians((imu - letzt + 180.0) % 360.0 - 180.0)
        if abs(d_imu) < 1e-3:
            return
        d_soll = abs(omega * dt)
        self._imu_beleg += d_soll if (d_imu > 0) == (omega > 0) else -d_soll

        if abs(self._imu_beleg) < self.IMU_BELEG_GRENZE:
            return
        self._imu_entschieden = True
        self._imu_referenz = None  # neu ansetzen, ab jetzt mit IMU
        if self._imu_beleg > 0:
            self._imu_vorzeichen = 1
            log.info("IMU-Drehsinn passt (IMU_YAW_SIGN = 1)")
        else:
            self._imu_vorzeichen = -1
            log.warning(
                "IMU zählt im Uhrzeigersinn -- wird umgerechnet. Fest "
                "eintragen in config.py: IMU_YAW_SIGN = -1"
            )


def _normalisieren(winkel: float) -> float:
    """Auf -pi..+pi bringen."""
    return math.atan2(math.sin(winkel), math.cos(winkel))
