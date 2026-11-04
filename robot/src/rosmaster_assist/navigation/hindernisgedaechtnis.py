"""Hindernisgedächtnis: was die Tiefenkamera gesehen hat, auch wenn sie es
nicht mehr sieht -- und wo der Roboter hängen geblieben ist.

Die Astra misst erst ab ~0,6 m. Ein Stuhlfuß taucht also in 1,5 m auf und
verschwindet bei 0,6 m wieder aus dem Bild -- genau bevor er wichtig wird.
Deshalb werden die Hindernispunkte in einem festen Rahmen gemerkt und mit
der eigenen Bewegung zurück in den Roboterrahmen gerechnet.

Abgeräumt wird ein Punkt, wenn die Kamera an seiner Stelle wieder freien
Boden sieht (Stuhl weggeschoben, Fehlmessung), wenn er zu alt ist oder zu
weit weg. Dazu kommen **Phantom-Hindernisse** von der Fortschrittswache:
Kam der Roboter an einer Stelle trotz Fahrbefehl nicht weiter, steckt dort
etwas, das niemand sieht. Diese Punkte räumt die Kamera nicht ab -- sie
sieht dort ja gerade nichts --, sie verfallen nur mit der Zeit.

Der Rahmen kommt aus ``lage_quelle``: beim Kartieren die Lidar-Odometrie,
sonst die Odometrie. Wechselt die Quelle oder springt die Lage (Startpunkt
neu gesetzt), passt das Gemerkte nicht mehr und wird verworfen.
"""

from __future__ import annotations

import logging
import math

import numpy as np

from .. import config

log = logging.getLogger(__name__)

#: Weiter als das springt die Lage zwischen zwei Takten nicht -- sonst wurde
#: sie neu gesetzt, und das Gemerkte gehört woandershin.
_MAX_SPRUNG_M = 0.5


def _in_welt(punkte, lage) -> np.ndarray:
    x, y, th = lage
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + punkte[:, 0] * c - punkte[:, 1] * s,
                            y + punkte[:, 0] * s + punkte[:, 1] * c))


def _in_roboter(punkte, lage) -> np.ndarray:
    x, y, th = lage
    c, s = math.cos(th), math.sin(th)
    dx, dy = punkte[:, 0] - x, punkte[:, 1] - y
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


class Hindernisgedaechtnis:
    """Tiefen- und Phantom-Hindernisse, im Roboterrahmen abrufbar."""

    def __init__(self, tiefe, lage_quelle) -> None:
        self._tiefe = tiefe
        self._lage_quelle = lage_quelle  # () -> (Name, (x, y, th)) oder None
        self._quelle: str | None = None
        self._lage: tuple | None = None
        self._bild_zeit: float | None = None
        self._welt = np.zeros((0, 2))
        self._zeit = np.zeros(0)
        self._phantom = np.zeros(0, dtype=bool)
        #: Wechselt bei jeder Änderung -- Fahrschutz und Erkunder rechnen
        #: dann neu, sonst nehmen sie das letzte Ergebnis.
        self.version = 0
        self._roboter_version = -1
        self._roboter = np.zeros((0, 2))

    def leeren(self) -> None:
        self._welt = np.zeros((0, 2))
        self._zeit = np.zeros(0)
        self._phantom = np.zeros(0, dtype=bool)
        self.version += 1

    @property
    def anzahl(self) -> int:
        return int(len(self._welt))

    @property
    def phantome(self) -> int:
        return int(self._phantom.sum())

    def aktualisieren(self, jetzt: float) -> None:
        """Lage nachführen, neues Tiefenbild übernehmen, Altes verwerfen.

        Darf beliebig oft aufgerufen werden; gerechnet wird nur bei Neuem.
        """
        quelle = self._lage_quelle()
        if quelle is None:
            return
        name, lage = quelle
        if name != self._quelle or (
            self._lage is not None
            and math.hypot(lage[0] - self._lage[0], lage[1] - self._lage[1]) > _MAX_SPRUNG_M
        ):
            if self.anzahl:
                log.info("Hindernisgedächtnis verworfen (Lage %s)",
                         "neu gesetzt" if name == self._quelle else f"jetzt aus {name}")
            self.leeren()
            self._quelle = name
        if lage != self._lage:
            self._lage = lage
            self.version += 1

        bild = self._tiefe.bild() if self._tiefe is not None else None
        if bild is not None and bild.zeit != self._bild_zeit:
            self._bild_zeit = bild.zeit
            self._einspeisen(bild, lage)

        if self.anzahl:
            alter = jetzt - self._zeit
            weit = np.hypot(self._welt[:, 0] - lage[0], self._welt[:, 1] - lage[1])
            behalten = (weit <= config.TIEFE_GEDAECHTNIS_M) & np.where(
                self._phantom, alter <= config.FORTSCHRITT_PHANTOM_S,
                alter <= config.TIEFE_GEDAECHTNIS_S)
            if not behalten.all():
                self._behalten(behalten)

    def _einspeisen(self, bild, lage) -> None:
        zelle = config.TIEFE_ZELLE_M
        zeit = bild.zeit
        # Abräumen: Kamerapunkte, an deren Stelle sie jetzt freien Boden sieht.
        if self.anzahl and len(bild.frei):
            frei = {(int(i), int(j)) for i, j in np.floor(bild.frei / zelle).astype(int)}
            hier = np.floor(_in_roboter(self._welt, lage) / zelle).astype(int)
            gesehen = np.array([(int(i), int(j)) in frei for i, j in hier])
            weg = gesehen & ~self._phantom
            if weg.any():
                self._behalten(~weg)
        if len(bild.hindernisse) == 0:
            return
        neu = _in_welt(np.asarray(bild.hindernisse, dtype=float), lage)
        # Dieselbe Weltzelle nicht doppelt: der alte Kamerapunkt weicht dem neuen.
        if self.anzahl:
            neue_zellen = {(int(i), int(j)) for i, j in np.floor(neu / zelle).astype(int)}
            alte_zellen = np.floor(self._welt / zelle).astype(int)
            doppelt = np.array([(int(i), int(j)) in neue_zellen for i, j in alte_zellen])
            doppelt &= ~self._phantom
            if doppelt.any():
                self._behalten(~doppelt)
        self._welt = np.concatenate((self._welt, neu))
        self._zeit = np.concatenate((self._zeit, np.full(len(neu), zeit)))
        self._phantom = np.concatenate((self._phantom, np.zeros(len(neu), dtype=bool)))
        self.version += 1

    def phantom(self, punkte_roboter, jetzt: float) -> None:
        """Gedachtes Hindernis an diesen Stellen (Roboterrahmen) eintragen."""
        if self._lage is None or len(punkte_roboter) == 0:
            return
        neu = _in_welt(np.asarray(punkte_roboter, dtype=float), self._lage)
        self._welt = np.concatenate((self._welt, neu))
        self._zeit = np.concatenate((self._zeit, np.full(len(neu), jetzt)))
        self._phantom = np.concatenate((self._phantom, np.ones(len(neu), dtype=bool)))
        self.version += 1

    def punkte(self) -> np.ndarray:
        """Alle gemerkten Punkte im aktuellen Roboterrahmen (Nx2)."""
        if self._lage is None or not self.anzahl:
            return np.zeros((0, 2))
        if self._roboter_version != self.version:
            self._roboter = _in_roboter(self._welt, self._lage)
            self._roboter_version = self.version
        return self._roboter

    def _behalten(self, maske) -> None:
        self._welt = self._welt[maske]
        self._zeit = self._zeit[maske]
        self._phantom = self._phantom[maske]
        self.version += 1

    def status(self) -> dict:
        return {"punkte": self.anzahl, "phantome": self.phantome, "rahmen": self._quelle}
