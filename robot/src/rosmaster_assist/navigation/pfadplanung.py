"""Wege durch die Karte: zu einem Ziel und zur nächsten unbekannten Stelle.

Geplant wird auf einem gröberen Raster als die Karte
(``KARTE_PLAN_AUFLOESUNG_M``), mit A* über acht Nachbarn. Befahrbar ist
eine Zelle, wenn sie frei ist und mindestens ``KARTE_SICHERHEIT_M`` von
der nächsten Wand entfernt -- dann passt der Roboter dort durch, wie er
auch steht. Unbekanntes ist nie befahrbar: Was die Karte nicht kennt, kann
eine Wand sein. Nähe zu Wänden kostet extra (bis
``KARTE_WUNSCH_ABSTAND_M``), so führen Wege mittig durch Türen.

"Lücken" (englisch frontier) sind freie Stellen, die ans Unbekannte
grenzen: offene Türen, Flure, Schatten hinter Möbeln. Fährt die
Kartierfahrt die nächste davon an, wächst die Karte dort -- so lange, bis
keine erreichbare mehr übrig ist. Gesucht wird mit Dijkstra vom Roboter aus;
die erste erreichte Lücke ist die mit dem kürzesten Weg.

Die Rechnung dauert am Pi Zehntelsekunden. Fahrschleifen mit 20 Hz rufen
sie deshalb über ``HintergrundPlaner`` auf und halten so lange an.
"""

from __future__ import annotations

import heapq
import logging
import math
from concurrent.futures import Future, ThreadPoolExecutor
from typing import NamedTuple

import numpy as np

from .. import config

log = logging.getLogger(__name__)

_WURZEL2 = math.sqrt(2.0)


class Weg(NamedTuple):
    punkte: list[tuple[float, float]]   # Wegpunkte im Kartenrahmen, ohne die Startlage
    laenge: float                       # Meter
    ziel: tuple[float, float]           # tatsächliches Ziel (ggf. verschoben)
    luecke: tuple[float, float] | None = None  # Mitte der angefahrenen Lücke


class Gitter:
    """Planungsraster aus einer Karte, mit befahrbaren Zellen und Kosten."""

    def __init__(self, karte, gesperrt=()) -> None:
        f = max(1, int(round(config.KARTE_PLAN_AUFLOESUNG_M / karte.aufloesung)))
        self.aufloesung = karte.aufloesung * f
        self.ursprung = karte.ursprung
        h, w = karte.hoehe, karte.breite
        self.hoehe, self.breite = -(-h // f), -(-w // f)
        ph, pw = self.hoehe * f - h, self.breite * f - w

        def bloecke(feld, rand):
            gepolstert = np.pad(feld, ((0, ph), (0, pw)), constant_values=rand)
            return gepolstert.reshape(self.hoehe, f, self.breite, f)

        self.belegt = bloecke(karte.belegt, False).any(axis=(1, 3))
        self.unbekannt = bloecke(karte.unbekannt, True).all(axis=(1, 3))
        self.frei = bloecke(karte.frei, False).any(axis=(1, 3)) & ~self.belegt
        abstand = bloecke(karte.abstand, 0.0).min(axis=(1, 3))

        # Vorübergehend versperrte Stellen (dort stand beim letzten Versuch
        # etwas, das die Karte nicht kennt): gelten wie eine Wand.
        if len(gesperrt):
            gy, gx = np.mgrid[0:self.hoehe, 0:self.breite]
            mx = self.ursprung[0] + (gx + 0.5) * self.aufloesung
            my = self.ursprung[1] + (gy + 0.5) * self.aufloesung
            for sx, sy in gesperrt:
                d = np.hypot(mx - sx, my - sy) - config.KARTE_SPERRE_RADIUS_M
                abstand = np.minimum(abstand, np.maximum(d, 0.0))
        self.abstand = abstand

        self.befahrbar = self.frei & (abstand >= config.KARTE_SICHERHEIT_M)
        spanne = max(config.KARTE_WUNSCH_ABSTAND_M - config.KARTE_SICHERHEIT_M, 1e-3)
        naehe = np.clip((config.KARTE_WUNSCH_ABSTAND_M - abstand) / spanne, 0.0, 1.0)
        self.kosten = 1.0 + config.KARTE_WANDKOSTEN * naehe

    def zelle(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.ursprung[0]) / self.aufloesung)),
                int(math.floor((y - self.ursprung[1]) / self.aufloesung)))

    def welt(self, ix: int, iy: int) -> tuple[float, float]:
        return (self.ursprung[0] + (ix + 0.5) * self.aufloesung,
                self.ursprung[1] + (iy + 0.5) * self.aufloesung)

    def innen(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.breite and 0 <= iy < self.hoehe

    # --- Suche ---------------------------------------------------------------------

    def naechste_befahrbare(self, ix: int, iy: int, weit_m: float) -> tuple[int, int] | None:
        """Nächste befahrbare Zelle (Breitensuche durch alles, was keine Wand ist)."""
        if self.innen(ix, iy) and self.befahrbar[iy, ix]:
            return ix, iy
        grenze = int(math.ceil(weit_m / self.aufloesung))
        gesehen = {(ix, iy)}
        rand = [(ix, iy)]
        for _ in range(grenze):
            neu = []
            for cx, cy in rand:
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if (nx, ny) in gesehen or not self.innen(nx, ny) or self.belegt[ny, nx]:
                        continue
                    if self.befahrbar[ny, nx]:
                        return nx, ny
                    gesehen.add((nx, ny))
                    neu.append((nx, ny))
            rand = neu
        return None

    def suchen(self, start: tuple[int, int], ziel: tuple[int, int] | None = None,
               ziele: np.ndarray | None = None) -> list[tuple[int, int]] | None:
        """A* zu ``ziel`` bzw. Dijkstra zur nächsten Zelle mit ``ziele[iy, ix]``."""
        w = self.breite
        befahrbar = self.befahrbar.ravel().tolist()
        kosten = self.kosten.ravel().tolist()
        zielmenge = ziele.ravel().tolist() if ziele is not None else None
        s = start[1] * w + start[0]
        z = ziel[1] * w + ziel[0] if ziel is not None else -1
        zx, zy = ziel if ziel is not None else (0, 0)

        def h(i: int) -> float:
            if ziel is None:
                return 0.0
            dx, dy = abs(i % w - zx), abs(i // w - zy)
            return (dx + dy) + (_WURZEL2 - 2.0) * min(dx, dy)

        g = {s: 0.0}
        vorher = {s: -1}
        offen = [(h(s), s)]
        fertig = set()
        nachbarn = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                    (1, 1, _WURZEL2), (1, -1, _WURZEL2), (-1, 1, _WURZEL2), (-1, -1, _WURZEL2))
        n = self.hoehe * w
        while offen:
            _, i = heapq.heappop(offen)
            if i in fertig:
                continue
            fertig.add(i)
            if i == z or (zielmenge is not None and zielmenge[i]):
                weg = []
                while i != -1:
                    weg.append((i % w, i // w))
                    i = vorher[i]
                return weg[::-1]
            ix, gi = i % w, g[i]
            for dx, dy, schritt in nachbarn:
                nx = ix + dx
                if nx < 0 or nx >= w:
                    continue
                j = i + dy * w + dx
                if j < 0 or j >= n or not befahrbar[j] or j in fertig:
                    continue
                if dx and dy and not (befahrbar[i + dx] and befahrbar[i + dy * w]):
                    continue  # nicht diagonal an einer Ecke vorbeischneiden
                neu = gi + schritt * 0.5 * (kosten[i] + kosten[j])
                if neu < g.get(j, float("inf")):
                    g[j] = neu
                    vorher[j] = i
                    heapq.heappush(offen, (neu + h(j), j))
        return None

    def glaetten(self, zellen: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """Überflüssige Zwischenpunkte weglassen.

        Abgekürzt wird nur, wo die gerade Linie mindestens so viel Abstand
        zur Wand hält wie der geplante Weg an seiner engsten Stelle dazwischen
        -- sonst ginge die Türmitte verloren, die die Kosten erzeugt haben.
        """
        if len(zellen) <= 2:
            return zellen
        abstand = [float(self.abstand[y, x]) for x, y in zellen]
        ergebnis = [zellen[0]]
        i = 0
        while i < len(zellen) - 1:
            j = len(zellen) - 1
            while j > i + 1:
                noetig = min(abstand[i:j + 1]) - 1e-6
                if self._linie_frei(zellen[i], zellen[j], noetig):
                    break
                j -= 1
            ergebnis.append(zellen[j])
            i = j
        return ergebnis

    def _linie_frei(self, a, b, noetig: float) -> bool:
        schritte = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2
        for k in range(schritte + 1):
            t = k / max(schritte, 1)
            x = int(round(a[0] + (b[0] - a[0]) * t))
            y = int(round(a[1] + (b[1] - a[1]) * t))
            if not self.befahrbar[y, x] or self.abstand[y, x] < noetig:
                return False
        return True

    def luecken(self) -> list[tuple[list[tuple[int, int]], tuple[float, float]]]:
        """Zusammenhängende Übergänge frei -> unbekannt, groß genug für einen Durchgang.

        Gibt je Lücke ihre Zellen und ihre Mitte (Welt) zurück.
        """
        unb = self.unbekannt
        grenze = np.zeros_like(unb)
        grenze[1:, :] |= unb[:-1, :]
        grenze[:-1, :] |= unb[1:, :]
        grenze[:, 1:] |= unb[:, :-1]
        grenze[:, :-1] |= unb[:, 1:]
        rand = self.frei & grenze
        ys, xs = np.nonzero(rand)
        offen = set(zip(xs.tolist(), ys.tolist()))
        min_zellen = max(1, int(math.ceil(config.MAP_LUECKE_MIN_M / self.aufloesung)))
        ergebnis = []
        while offen:
            erste = offen.pop()
            gruppe = [erste]
            stapel = [erste]
            while stapel:
                cx, cy = stapel.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (cx + dx, cy + dy)
                        if nb in offen:
                            offen.discard(nb)
                            gruppe.append(nb)
                            stapel.append(nb)
            if len(gruppe) >= min_zellen:
                mx = sum(p[0] for p in gruppe) / len(gruppe)
                my = sum(p[1] for p in gruppe) / len(gruppe)
                ergebnis.append((gruppe, self.welt(mx, my)))
        return ergebnis


def _weg_aus(gitter: Gitter, start_xy, zellen, ende=None) -> Weg:
    """Wegpunkte aus Zellen; ``ende``: genau dort enden statt in der Zellmitte."""
    zellen = gitter.glaetten(zellen)
    punkte = [gitter.welt(x, y) for x, y in zellen[1:]] or [gitter.welt(*zellen[0])]
    if ende is not None:
        punkte[-1] = (float(ende[0]), float(ende[1]))
    laenge = 0.0
    vorher = start_xy
    for p in punkte:
        laenge += math.hypot(p[0] - vorher[0], p[1] - vorher[1])
        vorher = p
    return Weg(punkte, laenge, punkte[-1])


def weg_planen(karte, start_xy, ziel_xy, gesperrt=()) -> Weg | None:
    """Weg vom Roboter zum Ziel; None, wenn keiner durch bekanntes Gebiet führt."""
    gitter = Gitter(karte, gesperrt)
    start = gitter.naechste_befahrbare(*gitter.zelle(*start_xy), config.KARTE_ZIEL_FANG_M)
    ziel = gitter.naechste_befahrbare(*gitter.zelle(*ziel_xy), config.KARTE_ZIEL_FANG_M)
    if start is None or ziel is None:
        return None
    zellen = gitter.suchen(start, ziel=ziel)
    if zellen is None:
        return None
    # Ist die Zielzelle selbst befahrbar (nicht verschoben), genau zum
    # verlangten Punkt -- die Zellmitte läge bis zu 7 cm daneben.
    genau = ziel_xy if ziel == gitter.zelle(*ziel_xy) else None
    return _weg_aus(gitter, start_xy, zellen, ende=genau)


def luecke_suchen(karte, start_xy, erledigt=(), gesperrt=()) -> Weg | None:
    """Weg zur nächsten erreichbaren Lücke; None, wenn die Karte fertig ist."""
    gitter = Gitter(karte, gesperrt)
    start = gitter.naechste_befahrbare(*gitter.zelle(*start_xy), config.KARTE_ZIEL_FANG_M)
    if start is None:
        return None
    nah = max(1, int(round(config.MAP_LUECKE_NAH_M / gitter.aufloesung)))
    ziele = np.zeros((gitter.hoehe, gitter.breite), dtype=bool)
    mitten: dict[tuple[int, int], tuple[float, float]] = {}
    for zellen, mitte in gitter.luecken():
        if any(math.hypot(mitte[0] - e[0], mitte[1] - e[1]) < config.MAP_LUECKE_ERLEDIGT_M
               for e in erledigt):
            continue
        for x, y in zellen:
            ziele[y, x] = True
            mitten[(x, y)] = mitte
    if not ziele.any():
        return None
    # Ziel ist nicht die Lücke selbst (die liegt am Rand des Bekannten und
    # oft zu nah an einer Wand), sondern jede befahrbare Stelle nahe daran.
    nahe = ziele.copy()
    for _ in range(nah):
        n = nahe.copy()
        n[1:, :] |= nahe[:-1, :]
        n[:-1, :] |= nahe[1:, :]
        n[:, 1:] |= nahe[:, :-1]
        n[:, :-1] |= nahe[:, 1:]
        nahe = n
    zellen = gitter.suchen(start, ziele=nahe & gitter.befahrbar)
    if zellen is None:
        return None
    weg = _weg_aus(gitter, start_xy, zellen)
    # Welche Lücke wird angefahren? Die mit der nächsten Zelle zum Ziel.
    zx, zy = zellen[-1]
    naechste = min(mitten, key=lambda p: (p[0] - zx) ** 2 + (p[1] - zy) ** 2)
    return weg._replace(luecke=mitten[naechste])


def luecken_anzeigen(karte) -> list[list[float]]:
    """Mitten aller Lücken, für die Seite /map."""
    return [[round(m[0], 2), round(m[1], 2)] for _, m in Gitter(karte).luecken()]


class HintergrundPlaner:
    """Rechnet Wege in einem eigenen Thread -- die Fahrschleife fragt nur nach."""

    def __init__(self) -> None:
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pfadplanung")

    def weg(self, karte, start_xy, ziel_xy, gesperrt=()) -> Future:
        return self._pool.submit(_sicher, weg_planen, karte, start_xy, ziel_xy, tuple(gesperrt))

    def luecke(self, karte, start_xy, erledigt=(), gesperrt=()) -> Future:
        return self._pool.submit(
            _sicher, luecke_suchen, karte, start_xy, tuple(erledigt), tuple(gesperrt)
        )


def _sicher(fn, *args):
    """Ein Fehler beim Planen heißt "kein Weg", nicht "Fahrschleife tot"."""
    try:
        return fn(*args)
    except Exception:
        log.exception("Pfadplanung fehlgeschlagen")
        return None
