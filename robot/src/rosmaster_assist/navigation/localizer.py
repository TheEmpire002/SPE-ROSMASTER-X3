"""Lidar-gestützte Ortung: korrigiert die Koppelnavigation.

Arbeitsteilung zwischen beiden Verfahren:

* **Koppelnavigation** läuft immer und überall. Sie ist sofort verfügbar und
  nie "verloren", driftet aber mit jedem Meter -- und nach jeder Drehung.
* **Lidar-Abgleich** vergleicht die aktuelle Umgebung mit einer Karte. Er
  summiert nichts auf, braucht aber etwas Wiedererkennbares in der Nähe.

**Die Karte.** Beim Setzen des Startpunkts wird die erste Aufnahme abgelegt.
Fährt der Roboter los, wird jede neue Aufnahme gegen die Karte abgeglichen;
passt sie sehr gut und ist er weit genug vom nächsten gespeicherten Punkt
entfernt, kommt sie als neues "Schlüsselbild" dazu -- mit der Position, an
der sie entstand. So wächst die Karte entlang der gefahrenen Strecke, und
auf dem Rückweg gibt es überall etwas, wogegen sich abgleichen lässt.

Früher gab es nur die eine Aufnahme vom Startpunkt. Jenseits von rund 4 m
oder hinter der nächsten Ecke war die Ortung damit blind, und der Roboter
kam mit der gesamten Drift der Koppelnavigation zurück.

Das ist bewusst kein vollständiges SLAM: Die Schlüsselbilder werden nicht
nachträglich gegeneinander ausgeglichen. Fehler, die sich auf dem Hinweg
einschleichen, bleiben in der Karte -- aber sie wachsen viel langsamer als
bei reiner Koppelnavigation, und am Startpunkt selbst ist die Karte exakt.

**Drei Schutzmaßnahmen gegen falsche Abgleiche**, denn ein falsch
übernommener Abgleich verdreht alles, was danach kommt:

- Während schneller Drehungen wird nicht abgeglichen. Die Aufnahme ist dann
  verzerrt, weil sich der Roboter während einer Lidar-Umdrehung mitdreht.
- Übernommen wird nur, wenn der Restabstand klein ist *und* der größte Teil
  der Punkte einen Partner findet.
- Weite Sprünge werden verworfen, außer die Spur ist verloren und es wird
  bewusst neu gesucht -- dann mit mehreren Startwinkeln.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .. import config
from ..protocol import DriveCommand
from ..sensors.lidar import LidarService
from .odometry import Odometry
from .scan_matching import match

log = logging.getLogger(__name__)

#: Raster zum Ausdünnen der Punktwolken. Dichter bringt dem Abgleich nichts,
#: kostet aber quadratisch Rechenzeit.
RASTER_SCAN_M = 0.03
RASTER_KARTE_M = 0.05

#: So lange nach dem Setzen des Startpunkts werden weitere Aufnahmen in das
#: erste Schlüsselbild eingemischt, solange der Roboter steht. Mehrere
#: Umdrehungen ergeben eine dichtere, weniger verrauschte Referenz.
HEIM_SAMMELN_S = 1.5

#: Startwinkel-Versatz bei der Neusuche nach verlorener Spur (Bogenmaß).
NEUSUCHE_WINKEL = (0.0, 0.26, -0.26, 0.52, -0.52)  # 0, ±15°, ±30°


class _Schluesselbild:
    """Eine gespeicherte Aufnahme samt der Position, an der sie entstand."""

    __slots__ = ("x", "y", "theta", "welt")

    def __init__(self, x: float, y: float, theta: float, welt) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.welt = welt  # Nx2, schon in Startpunkt-Koordinaten


class Localizer:
    """Hält die Karte und korrigiert damit die Position."""

    def __init__(self, odometry: Odometry, lidar: LidarService | None) -> None:
        self._odometry = odometry
        self._lidar = lidar
        self._lock = threading.Lock()

        self._karte: list[_Schluesselbild] = []
        self._heim_sammeln_bis = 0.0
        self._letzter_erfolg = 0.0
        self._voll_gemeldet = False

        self._letztes_residuum: float | None = None
        self._letzter_anteil: float | None = None
        self._korrekturen = 0
        self._verworfen = 0
        self._neusuchen = 0
        self._letzte_korrektur_m: float | None = None

    @property
    def hat_referenz(self) -> bool:
        return bool(self._karte)

    @property
    def verfuegbar(self) -> bool:
        return self._lidar is not None

    @property
    def spur_ok(self) -> bool:
        """Ob die Position kürzlich vom Lidar bestätigt wurde."""
        return (
            bool(self._karte)
            and time.monotonic() - self._letzter_erfolg < config.MATCH_VERLOREN_S
        )

    def status(self) -> dict:
        with self._lock:
            letzter = self._letzter_erfolg
            return {
                "lidar_vorhanden": self._lidar is not None,
                "referenz_aufgenommen": bool(self._karte),
                "referenzpunkte": len(self._karte[0].welt) if self._karte else 0,
                "schluesselbilder": len(self._karte),
                "spur": (
                    "keine Karte" if not self._karte
                    else "ok" if self.spur_ok else "verloren"
                ),
                "letzter_abgleich_vor_s": (
                    round(time.monotonic() - letzter, 1) if letzter else None
                ),
                "korrekturen": self._korrekturen,
                "verworfen": self._verworfen,
                "neusuchen": self._neusuchen,
                "letztes_residuum_m": _runden(self._letztes_residuum, 3),
                "letzter_anteil": _runden(self._letzter_anteil, 2),
                "letzte_korrektur_m": _runden(self._letzte_korrektur_m, 3),
            }

    # --- Referenz ---------------------------------------------------------------

    def referenz_aufnehmen(self) -> bool:
        """Neue Karte beginnen: hier ist der Startpunkt.

        Beim Setzen des Startpunkts aufzurufen. Die alte Karte gehört zum
        alten Startpunkt und wird verworfen.
        """
        if self._lidar is None:
            return False

        scan = self._lidar.latest()
        if scan is None or len(scan) < 60:
            log.warning(
                "Keine brauchbare Lidar-Aufnahme für den Startpunkt "
                "(%s Punkte) -- Ortung bleibt bei Koppelnavigation",
                0 if scan is None else len(scan),
            )
            return False

        punkte = self._punkte(scan)
        jetzt = time.monotonic()
        with self._lock:
            self._karte = [_Schluesselbild(0.0, 0.0, 0.0, punkte)]
            self._heim_sammeln_bis = jetzt + HEIM_SAMMELN_S
            self._letzter_erfolg = jetzt
            self._voll_gemeldet = False
            self._korrekturen = 0
            self._verworfen = 0
            self._neusuchen = 0
            self._letztes_residuum = None
            self._letzter_anteil = None
            self._letzte_korrektur_m = None

        log.info("Startpunkt eingemessen: %d Lidar-Punkte als Referenz", len(punkte))
        return True

    def referenz_verwerfen(self) -> None:
        with self._lock:
            self._karte = []

    # --- Laufender Abgleich -------------------------------------------------------

    def abgleichen(self, befehl: DriveCommand | None = None) -> bool:
        """Einen Abgleich versuchen. True, wenn korrigiert wurde.

        ``befehl`` ist, was der Roboter gerade fährt -- daran wird erkannt,
        ob die Aufnahme durch eine schnelle Drehung verzerrt ist.
        """
        if self._lidar is None or not self._karte:
            return False

        scan = self._lidar.latest()
        if scan is None:
            return False

        # Zu alte Aufnahmen nicht verwenden: der Roboter ist inzwischen
        # woanders, und eine veraltete Messung zöge die Position zurück.
        if time.monotonic() - scan.zeit > 1.5:
            return False

        if befehl is not None and abs(befehl.omega) > config.MATCH_MAX_OMEGA:
            return False

        punkte = self._punkte(scan)
        x, y, winkel_grad = self._odometry.pose()

        # Die Aufnahme zeigt, wo der Roboter *beim Messen* stand. Als
        # Startwert deshalb die damalige Position nehmen, nicht die
        # jetzige -- sonst beginnt die Suche schon schief.
        damals = self._odometry.pose_bei(scan.zeit)
        if damals is None:
            damals = (x, y, math.radians(winkel_grad))

        if self._heim_nachsammeln(punkte, damals, befehl):
            return False

        referenz = self._referenz_um(damals[0], damals[1])
        if referenz is None:
            return False  # nichts Bekanntes in der Nähe

        verloren = not self.spur_ok
        versaetze = NEUSUCHE_WINKEL if verloren else (0.0,)
        bestes = None
        for versatz in versaetze:
            start = (damals[0], damals[1], damals[2] + versatz)
            ergebnis = match(punkte, referenz, start)
            if ergebnis is not None and (
                bestes is None or ergebnis.residuum < bestes.residuum
            ):
                bestes = ergebnis

        with self._lock:
            if bestes is None:
                self._verworfen += 1
                return False
            anteil = bestes.paare / max(1, len(punkte))
            self._letztes_residuum = bestes.residuum
            self._letzter_anteil = anteil
            if (
                bestes.residuum > config.MATCH_MAX_RESIDUAL_M
                or anteil < config.MATCH_MIN_ANTEIL
            ):
                self._verworfen += 1
                return False

            sprung = math.hypot(bestes.x - damals[0], bestes.y - damals[1])
            weit = sprung > config.MATCH_MAX_SPRUNG_M
            if weit and not (verloren and bestes.residuum <= config.KEYFRAME_MAX_RESIDUAL_M):
                self._verworfen += 1
                return False

        # Seit der Aufnahme ist der Roboter weitergefahren. Diese Strecke
        # draufrechnen, sonst zieht die Korrektur ihn in die Vergangenheit
        # zurück -- bei 0,3 m/s und 0,3 s Verzug schon fast zehn Zentimeter.
        jetzt_x, jetzt_y, jetzt_grad = self._odometry.pose()
        jetzt_theta = math.radians(jetzt_grad)
        ziel_x = bestes.x + (jetzt_x - damals[0])
        ziel_y = bestes.y + (jetzt_y - damals[1])
        ziel_theta = bestes.theta + _winkeldifferenz(jetzt_theta, damals[2])

        # Nach verlorener Spur dem sehr guten Treffer fast ganz vertrauen:
        # die Odometrie liegt dann nachweislich daneben, sanftes Heranziehen
        # würde nur Zeit kosten, in der der Roboter falsch fährt.
        gewicht = 0.9 if verloren else config.MATCH_TRUST

        with self._lock:
            self._korrekturen += 1
            self._letzter_erfolg = time.monotonic()
            self._letzte_korrektur_m = math.hypot(ziel_x - jetzt_x, ziel_y - jetzt_y)
            if verloren:
                self._neusuchen += 1
        if verloren:
            log.info(
                "Spur wiedergefunden (Restabstand %.3f m, Korrektur %.2f m)",
                bestes.residuum,
                math.hypot(ziel_x - jetzt_x, ziel_y - jetzt_y),
            )

        self._odometry.korrigieren(ziel_x, ziel_y, ziel_theta, gewicht)

        if bestes.residuum <= config.KEYFRAME_MAX_RESIDUAL_M and _ruhig(befehl):
            self._schluesselbild_pruefen(bestes.x, bestes.y, bestes.theta, punkte)
        return True

    # --- Karte -------------------------------------------------------------------

    def _heim_nachsammeln(self, punkte, damals, befehl: DriveCommand | None) -> bool:
        """Kurz nach dem Setzen weitere Umdrehungen ins erste Bild mischen.

        True, wenn die Aufnahme dafür verwendet wurde.
        """
        if time.monotonic() > self._heim_sammeln_bis:
            return False
        steht = befehl is None or befehl.is_stop()
        if not steht or math.hypot(damals[0], damals[1]) > 0.02:
            self._heim_sammeln_bis = 0.0  # losgefahren: Sammeln beenden
            return False

        import numpy as np

        with self._lock:
            if not self._karte:
                return False
            heim = self._karte[0]
            heim.welt = _ausduennen(np.vstack((heim.welt, punkte)), RASTER_SCAN_M)
        return True

    def _referenz_um(self, x: float, y: float):
        """Die Punkte der nächstgelegenen Schlüsselbilder, zusammengelegt."""
        import numpy as np

        with self._lock:
            kandidaten = sorted(
                (math.hypot(k.x - x, k.y - y), i) for i, k in enumerate(self._karte)
            )
            gewaehlt = [
                self._karte[i].welt
                for abstand, i in kandidaten[:3]
                if abstand <= config.KEYFRAME_SUCHRADIUS_M
            ]
        if not gewaehlt:
            return None
        if len(gewaehlt) == 1:
            return gewaehlt[0]
        return _ausduennen(np.vstack(gewaehlt), RASTER_KARTE_M)

    def _schluesselbild_pruefen(self, x: float, y: float, theta: float, punkte) -> None:
        """Neues Schlüsselbild ablegen, wenn das nächste weit genug weg ist."""
        import numpy as np

        with self._lock:
            naechstes = min(math.hypot(k.x - x, k.y - y) for k in self._karte)
            if naechstes < config.KEYFRAME_ABSTAND_M:
                return
            if len(self._karte) >= config.KEYFRAME_MAX:
                if not self._voll_gemeldet:
                    log.warning(
                        "Karte voll (%d Schlüsselbilder) -- es kommt nichts "
                        "mehr dazu, die Ortung arbeitet mit dem Vorhandenen",
                        len(self._karte),
                    )
                    self._voll_gemeldet = True
                return

            c, s = math.cos(theta), math.sin(theta)
            welt = punkte @ np.array([[c, -s], [s, c]]).T + np.array([x, y])
            self._karte.append(_Schluesselbild(x, y, theta, welt))
            anzahl = len(self._karte)
        log.debug("Schlüsselbild %d bei (%.2f, %.2f)", anzahl, x, y)

    def _punkte(self, scan):
        """Aufnahme als ausgedünnte Punktwolke, bezogen auf den Drehpunkt."""
        import numpy as np

        punkte = scan.punkte()
        if config.LIDAR_MOUNT_X_M or config.LIDAR_MOUNT_Y_M:
            punkte = punkte + np.array([config.LIDAR_MOUNT_X_M, config.LIDAR_MOUNT_Y_M])
        return _ausduennen(punkte, RASTER_SCAN_M)

    # --- Hindernisse ---------------------------------------------------------------

    def hindernis_voraus(
        self, richtung_grad: float, oeffnung_grad: float | None = None
    ) -> float | None:
        """Abstand zum nächsten Hindernis in einer Richtung, falls messbar.

        ``richtung_grad`` im Roboterkoordinatensystem: 0 ist vorn. Für die
        Heimfahrt wird die Fahrtrichtung übergeben -- bei Mecanum ist das
        nicht zwangsläufig "vorn".
        """
        if self._lidar is None:
            return None
        scan = self._lidar.latest()
        if scan is None or time.monotonic() - scan.zeit > 2.0:
            return None

        if oeffnung_grad is None:
            oeffnung_grad = config.HOME_CONE_DEG
        abstand = scan.min_abstand(richtung_grad, oeffnung_grad)
        return None if abstand == float("inf") else abstand

    def sicht_frei(self, richtung_grad: float, entfernung: float) -> bool | None:
        """Ob der Weg in diese Richtung über ``entfernung`` hinaus frei ist.

        None, wenn es keine aktuelle Aufnahme gibt -- dann weiß man es nicht.
        """
        if self._lidar is None:
            return None
        scan = self._lidar.latest()
        if scan is None or time.monotonic() - scan.zeit > 2.0:
            return None
        frei = scan.min_abstand(richtung_grad, config.RUECKWEG_SICHT_DEG)
        # Etwas Reserve: der Roboter ist breiter als ein Lidar-Strahl.
        return frei > entfernung + 0.3


def _ausduennen(punkte, raster: float):
    """Höchstens ein Punkt je Rasterzelle, Reihenfolge bleibt erhalten."""
    import numpy as np

    if len(punkte) == 0:
        return punkte
    zellen = np.floor(punkte / raster).astype(np.int64)
    _, erste = np.unique(zellen, axis=0, return_index=True)
    return punkte[np.sort(erste)]


def _ruhig(befehl: DriveCommand | None) -> bool:
    """Ob die Aufnahme kaum verzerrt ist -- Voraussetzung für die Karte."""
    if befehl is None:
        return True
    return math.hypot(befehl.vx, befehl.vy) <= 0.35 and abs(befehl.omega) <= 0.2


def _runden(wert: float | None, stellen: int) -> float | None:
    return None if wert is None else round(wert, stellen)


def _winkeldifferenz(a: float, b: float) -> float:
    """a - b, auf -pi..+pi gebracht."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
