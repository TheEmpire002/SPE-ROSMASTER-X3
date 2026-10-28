"""Handzeichen aus der Körperpose erkennen.

Vier Gesten (Version 1, siehe docs/interaktion.md):

``FOLGEN``     ein Arm über dem Kopf, gehalten
``STOPP``      beide Hände über den Schultern, gehalten
``HERKOMMEN``  eine Hand winkt vor dem Körper auf und ab
``WEGGEHEN``   eine Hand wischt seitlich vom Körper weg

Alle Maße sind relativ zur **Schulterbreite** der Person gerechnet, nicht in
Bildpixeln: Ob jemand 1 m oder 3 m entfernt steht, ändert die Pixel, aber
nicht das Verhältnis von Hand zu Schultern. Ebenso wird alles auf die
Schultermitte bezogen -- geht die Person beim Winken seitlich, wandert die
Hand im Bild, relativ zum Körper aber nicht.

Gegen Fehlauslöser: Eine gehaltene Geste zählt erst, wenn sie über
``GESTE_HALTEN_S`` in den meisten Bildern zu sehen war; bewegte Gesten
brauchen mehrere Richtungswechsel bzw. eine lange Strecke. Nach jeder
erkannten Geste gibt es eine Pause, nur STOPP geht immer.

Das Modul kennt weder Kamera noch Hailo -- es bekommt Personen-Dicts aus
``sensors/pose.py`` und ist so ohne Hardware testbar
(``Marlon/interaktion/test_gesten.py``).
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

from .. import config

FOLGEN = "folgen"
STOPP = "stopp"
HERKOMMEN = "herkommen"
WEGGEHEN = "weggehen"

# Nummern der COCO-Körperpunkte (wie in sensors/pose.py; hier wiederholt,
# damit dieses Modul ohne numpy/Hailo importierbar bleibt).
_NASE = 0
_SCHULTER = (5, 6)
_HAND = (9, 10)
_HUEFTE = (11, 12)

#: Ohne erkannte Hüfte: so weit (in Schulterbreiten) unter den Schultern
#: liegt sie bei einem Erwachsenen ungefähr.
_HUEFTE_ERSATZ = 1.8

#: Ohne erkannte Nase: so weit über den Schultern liegt der Kopf ungefähr.
_KOPF_ERSATZ = 0.6

#: Über so viele Bilder wird die Schulterbreite geglättet (~1 s bei 10 Hz).
_BREITEN_ANZAHL = 9


@dataclass
class Bild:
    """Eine Pose, umgerechnet in Körpermaße (Einheit: Schulterbreite).

    u nach rechts im Bild, v nach unten, Nullpunkt in der Schultermitte.
    ``haende`` hat für linke/rechte Hand (u, v) oder None, wenn unsicher.
    """

    zeit: float
    haltung: str | None
    haende: tuple[tuple[float, float] | None, tuple[float, float] | None]
    kopf_v: float
    huefte_v: float


def _punkt_lesen(person: dict):
    """Liefert punkt(i) -> (x, y) in gleich langen Einheiten oder None."""
    kp = person.get("kp") or []
    if len(kp) < 17:
        return None
    grenze = config.POSE_KP_MIN_CONF
    # x mit dem Seitenverhältnis strecken, damit ein Schritt in x gleich
    # lang ist wie einer in y (die Koordinaten sind auf Breite bzw. Höhe
    # des Bildes bezogen).
    sv = float(person.get("seitenverhaeltnis") or 1.0)

    def punkt(i: int) -> tuple[float, float] | None:
        x, y, c = kp[i]
        return (x * sv, y) if c >= grenze else None

    return punkt


def schulterbreite(person: dict) -> float | None:
    """Abstand der Schultern in Bildeinheiten, None wenn unsicher."""
    punkt = _punkt_lesen(person)
    if punkt is None:
        return None
    sl, sr = punkt(_SCHULTER[0]), punkt(_SCHULTER[1])
    if sl is None or sr is None:
        return None
    breite = math.hypot(sl[0] - sr[0], sl[1] - sr[1])
    return breite if breite >= 1e-3 else None  # seitlich zur Kamera -- unbrauchbar


def koerpermass(person: dict, zeit: float, breite: float | None = None) -> Bild | None:
    """Personen-Dict -> Bild. None, wenn die Schultern nicht sicher sind.

    Ohne beide Schultern gibt es keinen Maßstab -- dann lieber gar nichts
    erkennen als mit falscher Größe. ``breite`` ersetzt die gemessene
    Schulterbreite durch einen geglätteten Wert (siehe Gestenerkennung).
    """
    gemessen = schulterbreite(person)
    if gemessen is None:
        return None
    breite = breite or gemessen
    punkt = _punkt_lesen(person)
    sl, sr = punkt(_SCHULTER[0]), punkt(_SCHULTER[1])
    mx, my = (sl[0] + sr[0]) / 2, (sl[1] + sr[1]) / 2

    def rel(p: tuple[float, float] | None) -> tuple[float, float] | None:
        return None if p is None else ((p[0] - mx) / breite, (p[1] - my) / breite)

    haende = (rel(punkt(_HAND[0])), rel(punkt(_HAND[1])))
    nase = rel(punkt(_NASE))
    kopf_v = nase[1] if nase is not None else -_KOPF_ERSATZ
    hueften = [h for h in (rel(punkt(_HUEFTE[0])), rel(punkt(_HUEFTE[1]))) if h is not None]
    huefte_v = sum(h[1] for h in hueften) / len(hueften) if hueften else _HUEFTE_ERSATZ

    return Bild(zeit, haltung(haende, kopf_v), haende, kopf_v, huefte_v)


def haltung(haende, kopf_v: float) -> str | None:
    """Gehaltene Geste in einem einzelnen Bild (STOPP, FOLGEN oder None)."""
    # v < 0 heißt: über der Schulterlinie (y zeigt nach unten).
    ueber_schulter = [h is not None and h[1] < 0.0 for h in haende]
    ueber_kopf = [h is not None and h[1] < kopf_v for h in haende]
    if all(ueber_schulter):
        return STOPP
    # Genau ein Arm über dem Kopf, der andere unten -- beide oben ist STOPP.
    if sum(ueber_kopf) == 1:
        return FOLGEN
    return None


def richtungswechsel(werte: list[float], hub: float) -> int:
    """Wie oft eine Folge die Richtung wechselt, Zittern unter ``hub`` nicht gezählt.

    Erst wenn sich der Wert um mindestens ``hub`` vom letzten Umkehrpunkt
    entfernt hat, gilt die neue Richtung -- so wird aus Rauschen der
    Körperpunkte kein Winken.
    """
    if not werte:
        return 0
    richtung = 0
    extrem = werte[0]
    wechsel = 0
    for w in werte[1:]:
        if richtung == 0:
            if abs(w - extrem) >= hub:
                richtung = 1 if w > extrem else -1
                extrem = w
        elif richtung > 0:
            if w > extrem:
                extrem = w
            elif extrem - w >= hub:
                richtung, extrem, wechsel = -1, w, wechsel + 1
        else:
            if w < extrem:
                extrem = w
            elif w - extrem >= hub:
                richtung, extrem, wechsel = 1, w, wechsel + 1
    return wechsel


def winkt(bilder: list[Bild], hand: int) -> bool:
    """HERKOMMEN: Hand vor dem Oberkörper, mehrmals auf und ab."""
    proben = [
        b.haende[hand] for b in bilder
        if b.haende[hand] is not None and b.kopf_v < b.haende[hand][1] < b.huefte_v
    ]
    # Die Hand muss fast das ganze Fenster über zu sehen sein, sonst
    # entstehen aus Lücken scheinbare Sprünge.
    if len(proben) < 0.6 * len(bilder) or len(proben) < 6:
        return False
    u = [p[0] for p in proben]
    v = [p[1] for p in proben]
    # Überwiegend senkrecht -- seitliches Hin und Her ist Hallo-Winken.
    if max(u) - min(u) > max(v) - min(v):
        return False
    return richtungswechsel(v, config.GESTE_WINK_HUB) >= config.GESTE_WINK_WECHSEL


def wischt(bilder: list[Bild], hand: int) -> bool:
    """WEGGEHEN: Hand bewegt sich weit seitlich vom Körper weg."""
    proben = [
        b.haende[hand] for b in bilder
        if b.haende[hand] is not None and b.haende[hand][1] < b.huefte_v
    ]
    if len(proben) < 4:
        return False
    # Größte Zunahme des seitlichen Abstands von früher nach später
    # (laufendes Minimum), dabei wenig Bewegung nach oben/unten -- sonst ist
    # es ein Arm, der seitlich gehoben wird.
    strecke = config.GESTE_WISCH_STRECKE
    kleinster = proben[0]
    for p in proben[1:]:
        if abs(p[0]) - abs(kleinster[0]) >= strecke:
            if abs(p[1] - kleinster[1]) < 0.6 * (abs(p[0]) - abs(kleinster[0])):
                return True
        if abs(p[0]) < abs(kleinster[0]):
            kleinster = p
    return False


class Gestenerkennung:
    """Merkt sich die letzten Posen einer Person und meldet bestätigte Gesten."""

    def __init__(self) -> None:
        self._bilder: deque[Bild] = deque()
        self._breiten: deque[float] = deque(maxlen=_BREITEN_ANZAHL)
        self._winkel: float | None = None
        self._letzte: str | None = None
        self._letzte_zeit = -1e9
        self.zuletzt: str | None = None  # für die Anzeige
        self.zuletzt_zeit: float | None = None

    def zuruecksetzen(self) -> None:
        self._bilder.clear()
        self._breiten.clear()
        self._winkel = None

    def update(self, person: dict | None, zeit: float) -> str | None:
        """Neue Pose der verfolgten Person. Gibt eine bestätigte Geste zurück.

        Nur einmal pro neuem Kamerabild aufrufen -- dasselbe Bild doppelt
        würde Haltedauern verfälschen.
        """
        if person is None:
            self.zuruecksetzen()
            return None
        winkel = person.get("winkel")
        if (
            self._winkel is not None and winkel is not None
            and abs(winkel - self._winkel) > config.INTERAKTION_SPRUNG_GRAD
        ):
            # Großer Sprung: vermutlich eine andere Person -- deren Arme
            # dürfen nicht mit den Armen der vorigen verrechnet werden.
            self._bilder.clear()
            self._breiten.clear()
        self._winkel = winkel

        # Die Schulterbreite ist der Maßstab für alles. Zittern die Schultern
        # von Bild zu Bild, würde jede Hand scheinbar mitwandern (eine ruhige
        # Hand 1,4 Breiten unter den Schultern "winkt" bei 15 % Zittern der
        # Breite um 0,4) -- daher der Median der letzten Bilder.
        gemessen = schulterbreite(person)
        if gemessen is not None:
            self._breiten.append(gemessen)
        breite = statistics.median(self._breiten) if self._breiten else None
        bild = koerpermass(person, zeit, breite)
        if bild is None:
            # Unvollständiges Bild zählt als "keine Geste", damit es die
            # Haltedauer unterbricht statt übersprungen zu werden.
            bild = Bild(zeit, None, (None, None), -_KOPF_ERSATZ, _HUEFTE_ERSATZ)
        self._bilder.append(bild)
        while self._bilder and zeit - self._bilder[0].zeit > config.GESTE_FENSTER_S:
            self._bilder.popleft()

        geste = self._erkennen(zeit)
        if geste is None:
            return None
        # Pause nach jeder Geste, damit eine Bewegung nicht mehrfach zählt.
        # STOPP geht immer -- außer es wurde gerade eben schon erkannt.
        in_pause = zeit - self._letzte_zeit < config.GESTE_PAUSE_S
        if in_pause and (geste != STOPP or self._letzte == STOPP):
            return None
        self._letzte, self._letzte_zeit = geste, zeit
        self.zuletzt, self.zuletzt_zeit = geste, zeit
        self._bilder.clear()
        return geste

    def _erkennen(self, zeit: float) -> str | None:
        bilder = list(self._bilder)
        # Sicherheit zuerst: STOPP wird vor allem anderen geprüft.
        if self._gehalten(bilder, zeit, STOPP, config.GESTE_STOPP_HALTEN_S):
            return STOPP
        if self._gehalten(bilder, zeit, FOLGEN, config.GESTE_HALTEN_S):
            return FOLGEN
        # Bewegte Gesten brauchen ein fast volles Fenster.
        if not bilder or zeit - bilder[0].zeit < 0.7 * config.GESTE_FENSTER_S:
            return None
        if any(winkt(bilder, h) for h in (0, 1)):
            return HERKOMMEN
        if any(wischt(bilder, h) for h in (0, 1)):
            return WEGGEHEN
        return None

    @staticmethod
    def _gehalten(bilder: list[Bild], zeit: float, geste: str, dauer: float) -> bool:
        teil = [b for b in bilder if zeit - b.zeit <= dauer]
        # Das älteste Bild muss weit genug zurückliegen, sonst ist die
        # Haltedauer noch gar nicht erreicht.
        if len(teil) < 3 or zeit - teil[0].zeit < 0.8 * dauer:
            return False
        treffer = sum(1 for b in teil if b.haltung == geste)
        return treffer >= config.GESTE_ANTEIL * len(teil)
