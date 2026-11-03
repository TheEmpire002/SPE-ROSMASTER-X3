"""Fahrschutz: Kollisionsprüfung über die Grundfläche des Roboters.

Die letzte Prüfung vor dem Antrieb, für jede Bewegung gleich: vorwärts,
seitwärts, schräg, im Bogen oder auf der Stelle. Für den Befehl wird
vorausgerechnet, wo die Grundfläche des Roboters in der nächsten Zeit
überall sein wird. Ragt ein Lidar-Punkt hinein, wird so weit gebremst, dass
er mit Reaktionszeit und Bremsweg noch vorher steht.

Warum nicht wie bisher ein Kegel in Fahrtrichtung:

* Ein Kegel ab Lidar-Mitte ist nah am Roboter schmal -- das Tischbein neben
  dem Vorderrad liegt außerhalb -- und weit weg breit, er bremst vor Wänden,
  an denen der Roboter vorbeikäme.
* Ein Kegel kennt keine Drehung. Beim Drehen schwenken die Ecken aus; neben
  einer Wand kann schon das anstoßen.
* Eine harte Grenze ("unter 35 cm halt") lässt ihn mit vollem Tempo
  heranfahren. Bei 0,22 s Befehlsverzögerung (Sensor-Check) und Bremsweg
  rollt er dann noch ein gutes Stück weiter. Hier bremst er stattdessen
  stufenlos, je näher, desto langsamer.

Wegfahren bleibt immer erlaubt: Ein Punkt, der schon im Rand liegt, sperrt
nur Bewegungen, die ihm *näher* kommen. Sonst säße der Roboter neben jedem
Hindernis fest und müsste getragen werden.
"""

from __future__ import annotations

import logging
import math
import time
from typing import NamedTuple

from .. import config
from ..protocol import STOP, DriveCommand

log = logging.getLogger(__name__)

#: Weiter entfernte Punkte spielen für die Vorausschau keine Rolle.
_REICHWEITE_M = 2.5
#: Höchstens so weit in die Zukunft rechnen.
_MAX_VORAUS_S = 4.0
#: Feinheit der Vorausschau: je Schritt höchstens so viel Weg bzw. Drehung.
_SCHRITT_M = 0.02
_SCHRITT_RAD = math.radians(2.0)


class Pruefung(NamedTuple):
    """Ergebnis einer Prüfung."""

    befehl: DriveCommand   # was gefahren werden darf
    anteil: float          # 1.0 = unverändert, 0.0 = gesperrt
    frei_m: float | None   # Weg bis zur Berührung (Rand eingerechnet), None = frei
    grund: str | None      # warum gebremst wurde, None = gar nicht


class Fahrschutz:
    """Begrenzt Fahrbefehle so, dass die Grundfläche nichts berührt."""

    def __init__(self, lidar) -> None:
        self._lidar = lidar

    def pruefen(self, cmd: DriveCommand, rand_m: float | None = None) -> Pruefung:
        """Den erlaubten Befehl berechnen.

        ``rand_m`` ist der Abstand, den die Roboterkante mindestens halten
        soll (Vorgabe ``FAHRSCHUTZ_RAND_M``).
        """
        if cmd.is_stop():
            return Pruefung(cmd, 1.0, None, None)
        rand = config.FAHRSCHUTZ_RAND_M if rand_m is None else rand_m

        scan = self._lidar.latest() if self._lidar is not None else None
        alter = time.monotonic() - scan.zeit if scan is not None else None
        if scan is None or alter > config.FAHRSCHUTZ_SCAN_MAX_S:
            # Blind wird nicht gefahren: Ein alter Scan zeigt eine Welt, in
            # der der Roboter längst woanders steht.
            return Pruefung(STOP, 0.0, None, "keine frischen Lidar-Daten")

        import numpy as np

        punkte = scan.punkte()
        if len(punkte):
            punkte = punkte[np.hypot(punkte[:, 0], punkte[:, 1]) < _REICHWEITE_M]
        # Punkte innerhalb der Grundfläche selbst kann es nicht geben -- das
        # sind Teile des Roboters oder Fehlmessungen. Mitgezählt würden sie
        # den Abstand dauerhaft auf null setzen und damit den Schutz
        # ausschalten (näher als null geht nicht).
        if len(punkte):
            punkte = punkte[_abstand_zur_flaeche(punkte[:, 0], punkte[:, 1]) > 0.01]
        if len(punkte) == 0:
            return Pruefung(cmd, 1.0, None, None)

        frei_s = self._freie_zeit(cmd, punkte, rand, alter)
        if frei_s is None:
            return Pruefung(cmd, 1.0, None, None)

        # Wie stark bremsen, damit er nach Reaktionszeit und Bremsweg
        # innerhalb des freien Stücks steht. Weg und Drehung getrennt, das
        # Strengere zählt. Alle Anteile gleich skaliert: so bleibt die Bahn
        # (etwa ein Bogen) dieselbe, nur langsamer.
        reaktion = config.FAHRSCHUTZ_REAKTION_S + alter
        v = math.hypot(cmd.vx, cmd.vy)
        w = abs(cmd.omega)
        anteil = 1.0
        if v > 1e-6:
            anteil = min(anteil, _anteil(v, v * frei_s, reaktion, config.FAHRSCHUTZ_BREMSEN))
        if w > 1e-6:
            anteil = min(anteil, _anteil(w, w * frei_s, reaktion, config.FAHRSCHUTZ_BREMSEN_DREH))
        frei_m = v * frei_s if v > 1e-6 else None

        if anteil >= 0.999:
            return Pruefung(cmd, 1.0, frei_m, None)
        if anteil < config.FAHRSCHUTZ_MIN_ANTEIL:
            # Sehr kleine Werte setzen die Motoren nicht mehr um; dann
            # lieber ehrlich stehen.
            return Pruefung(STOP, 0.0, frei_m, "Hindernis im Weg")
        return Pruefung(
            DriveCommand(cmd.vx * anteil, cmd.vy * anteil, cmd.omega * anteil),
            anteil, frei_m, "bremst vor Hindernis",
        )

    @staticmethod
    def _freie_zeit(cmd: DriveCommand, punkte, rand: float, alter: float) -> float | None:
        """Wie lange der Befehl gefahren werden kann, bis der Rand berührt wird.

        None, wenn innerhalb der Vorausschau nichts im Weg ist.
        """
        import numpy as np

        v = math.hypot(cmd.vx, cmd.vy)
        w = abs(cmd.omega)
        reaktion = config.FAHRSCHUTZ_REAKTION_S + alter
        # Vorausschauen über den Anhalteweg bei vollem Befehl, mit Reserve.
        dauer = 0.0
        if v > 1e-6:
            dauer = max(dauer, reaktion + v / config.FAHRSCHUTZ_BREMSEN)
        if w > 1e-6:
            dauer = max(dauer, reaktion + w / config.FAHRSCHUTZ_BREMSEN_DREH)
        dauer = min(dauer * 1.5 + 0.2, _MAX_VORAUS_S)
        schritte = int(min(200, max(4, math.ceil(max(v * dauer / _SCHRITT_M,
                                                      w * dauer / _SCHRITT_RAD)))))
        t = np.linspace(0.0, dauer, schritte + 1)

        # Lage des Roboters zu jedem Zeitpunkt bei gleichbleibendem Befehl
        # (Bogenfahrt, im Roboterrahmen von jetzt).
        theta = cmd.omega * t
        if abs(cmd.omega) < 1e-6:
            x, y = cmd.vx * t, cmd.vy * t
        else:
            s, c = np.sin(theta), np.cos(theta)
            x = (cmd.vx * s + cmd.vy * (c - 1.0)) / cmd.omega
            y = (cmd.vx * (1.0 - c) + cmd.vy * s) / cmd.omega

        # Alle Punkte in den Rahmen jeder künftigen Lage holen und den
        # Abstand zur Grundfläche messen (Zeilen: Zeitpunkte, Spalten: Punkte).
        dx = punkte[None, :, 0] - x[:, None]
        dy = punkte[None, :, 1] - y[:, None]
        cs, sn = np.cos(theta)[:, None], np.sin(theta)[:, None]
        px = cs * dx + sn * dy
        py = -sn * dx + cs * dy
        abstand = _abstand_zur_flaeche(px, py).min(axis=1)

        # Berührt ist, wenn der Rand unterschritten wird UND er näher kommt.
        # Bleibt der Abstand gleich oder wächst er, fährt er parallel oder
        # weg -- das ist erlaubt, auch dicht daneben.
        kritisch = (abstand[1:] < rand) & (abstand[1:] < abstand[:-1] - 1e-4)
        if not kritisch.any():
            return None
        return float(t[int(np.argmax(kritisch))])


def _abstand_zur_flaeche(x, y):
    """Abstand von Punkten (Roboterrahmen) zur rechteckigen Grundfläche, innen 0."""
    import numpy as np

    aussen_x = np.maximum(np.maximum(x - config.ROBOTER_VORN_M, -config.ROBOTER_HINTEN_M - x), 0.0)
    aussen_y = np.maximum(np.abs(y) - config.ROBOTER_HALBE_BREITE_M, 0.0)
    return np.hypot(aussen_x, aussen_y)


def _anteil(tempo: float, frei: float, reaktion: float, bremsen: float) -> float:
    """Größter Anteil k des Tempos, bei dem der Anhalteweg ins Freie passt.

    Anhalteweg bei Tempo k*v: k*v*Reaktionszeit + (k*v)^2 / (2*Verzögerung).
    """
    if frei <= 0.0:
        return 0.0
    a = tempo * tempo / (2.0 * bremsen)
    b = tempo * reaktion
    k = (-b + math.sqrt(b * b + 4.0 * a * frei)) / (2.0 * a)
    return max(0.0, min(1.0, k))
