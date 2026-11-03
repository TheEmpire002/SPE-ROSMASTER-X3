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

Auch "gesperrt" ist kein Stopp an der Rampe vorbei: Der Watchdog fährt das
Ergebnis mit seiner Bremsrampe an. Harte Stopps aus voller Fahrt erzeugen
die Stromspitzen, die am X3 den USB-Hub (und damit den Lidar) zurücksetzen.

Seit 03.11. ist das die einzige Schutzschicht für autonome Fahrt (im
Watchdog bei ``quelle="autonom"``). Der frühere ``AutonomSchutz`` prüfte
nur einen Streifen in Fahrtrichtung; übernommen sind von ihm das Alter des
ältesten Scan-Punkts und das Aufbereiten einmal pro Runde.
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

#: Grund, wenn kein frischer Scan da ist. Aufrufer unterscheiden daran
#: "blind" von "Hindernis".
KEIN_LIDAR = "keine frischen Lidar-Daten"


class Pruefung(NamedTuple):
    """Ergebnis einer Prüfung."""

    befehl: DriveCommand   # was gefahren werden darf
    anteil: float          # 1.0 = unverändert, 0.0 = gesperrt
    frei_m: float | None   # Weg bis zur Berührung (Rand eingerechnet), None = frei
    grund: str | None      # warum gebremst wurde, None = gar nicht


class Fahrschutz:
    """Begrenzt Fahrbefehle so, dass die Grundfläche nichts berührt."""

    #: Eingriffe höchstens so oft protokollieren (s).
    MELDE_ABSTAND_S = 3.0

    def __init__(self, lidar) -> None:
        self._lidar = lidar
        self.eingriffe = 0  # bei autonomer Fahrt, siehe eingriff_melden()
        self._zuletzt_gemeldet = -1e9
        self._scan = None
        self._scan_punkte = None

    def pruefen(self, cmd: DriveCommand, rand_m: float | None = None) -> Pruefung:
        """Den erlaubten Befehl berechnen.

        ``rand_m`` ist der Abstand, den die Roboterkante mindestens halten
        soll (Vorgabe ``FAHRSCHUTZ_RAND_M``).
        """
        if cmd.is_stop():
            return Pruefung(cmd, 1.0, None, None)
        rand = config.FAHRSCHUTZ_RAND_M if rand_m is None else rand_m

        punkte, alter = self._punkte()
        if punkte is None:
            # Blind wird nicht gefahren: Ein alter Scan zeigt eine Welt, in
            # der der Roboter längst woanders steht.
            return Pruefung(STOP, 0.0, None, KEIN_LIDAR)
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

    def drehen_frei(self, winkel_rad: float, rand_m: float | None = None) -> str | None:
        """Ob eine Drehung auf der Stelle um ``winkel_rad`` ganz durchgeht.

        Für Drehungen mit festem Ziel ("90 Grad nach links"): vorher prüfen,
        statt nach der Hälfte stehen zu bleiben. Geprüft wird die
        Grundfläche über den ganzen Winkel -- ein Kreis um den Lidar wäre
        vorn viel zu groß (dort ragt der Roboter nur 15 cm über) und sperrte
        schon bei Möbeln, an denen keine Ecke vorbeistreift.

        Gibt den Grund zurück, warum es nicht geht, sonst None.
        """
        import numpy as np

        rand = config.FAHRSCHUTZ_RAND_M if rand_m is None else rand_m
        punkte, _ = self._punkte()
        if punkte is None:
            return KEIN_LIDAR
        if len(punkte) == 0 or abs(winkel_rad) < 1e-6:
            return None
        # Nach einer ganzen Umdrehung wiederholt sich alles.
        bogen = min(abs(winkel_rad), 2.0 * math.pi)
        schritte = max(2, math.ceil(bogen / _SCHRITT_RAD))
        theta = math.copysign(1.0, winkel_rad) * np.linspace(0.0, bogen, schritte + 1)
        cs, sn = np.cos(theta)[:, None], np.sin(theta)[:, None]
        px = cs * punkte[None, :, 0] + sn * punkte[None, :, 1]
        py = -sn * punkte[None, :, 0] + cs * punkte[None, :, 1]
        abstand = _abstand_zur_flaeche(px, py).min(axis=1)
        # Gleiche Regel wie beim Fahren: nur Annähern unter den Rand zählt.
        kritisch = (abstand[1:] < rand) & (abstand[1:] < abstand[:-1] - 1e-4)
        if not kritisch.any():
            return None
        nach = math.degrees(abs(theta[int(np.argmax(kritisch)) + 1]))
        return f"eine Ecke stößt nach {nach:.0f}° an"

    def eingriff_melden(self, grund: str, jetzt: float) -> None:
        """Einen Eingriff bei autonomer Fahrt zählen und entprellt loggen.

        Ruft der Watchdog auf. Die Prüfung läuft im 20-Hz-Takt; vor einem
        stehenden Hindernis käme die Meldung sonst zwanzigmal pro Sekunde.
        """
        self.eingriffe += 1
        if jetzt - self._zuletzt_gemeldet >= self.MELDE_ABSTAND_S:
            self._zuletzt_gemeldet = jetzt
            log.warning("Autonomer Fahrbefehl gebremst: %s", grund)

    def zustand(self) -> dict:
        return {"aktiv": bool(config.AUTONOM_SCHUTZ_AKTIV), "eingriffe": self.eingriffe}

    def _punkte(self):
        """Lidar-Punkte im Nahbereich und Alter des Scans; (None, None) ohne frischen Scan."""
        scan = self._lidar.latest() if self._lidar is not None else None
        jetzt = time.monotonic()
        if scan is None or jetzt - scan.zeit > config.FAHRSCHUTZ_SCAN_MAX_S:
            return None, None
        # Der älteste Punkt der Runde ist eine ganze Umdrehung älter als ihr
        # Zeitstempel -- mit dem rechnen, nicht mit dem Mittel. So lange ist
        # der Roboter schon blind weitergefahren.
        alter = jetzt - (scan.zeit - getattr(scan, "dauer", 0.0))

        # Punkte nur einmal pro Runde aufbereiten: Der Lidar liefert 7,5
        # Runden pro Sekunde, gefragt wird 20-mal.
        if scan is self._scan:
            return self._scan_punkte, alter

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
        self._scan, self._scan_punkte = scan, punkte
        return punkte, alter

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
