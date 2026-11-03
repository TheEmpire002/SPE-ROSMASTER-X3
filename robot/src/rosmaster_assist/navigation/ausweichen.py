"""Lokales Ausweichen -- ohne Karte. Für den Erkunder (Schritt F2), für
Folgen/Herkommen und die Heimfahrt (Schritt F3).

Vier Bausteine, alle reine Rechnung auf Lidar-Runden und der vom Board
gemessenen Geschwindigkeit, damit sie sich ohne Roboter prüfen und an
aufgezeichneten Fahrten nachspielen lassen:

**Kurzgedächtnis.** Der Lidar sieht nicht alles in jeder Runde: dünne
Stuhlbeine tauchen nur manchmal auf, schwarze oder glänzende Flächen
schlucken einzelne Strahlen, und unter ``LIDAR_MIN_M`` (Totzone direkt
am Roboter) misst er gar nicht. Deshalb behält das Gedächtnis die Punkte
der letzten Sekunden und schreibt die eigene Lage aus der Board-
Geschwindigkeit fort. Nur kurz und nur nahe Punkte: Die fortgeschriebene
Lage driftet, und weiter entfernte Dinge sieht der Lidar ohnehin wieder.
Ohne echte Messung (Board liefert nichts, die Odometrie rechnet mit dem
Befehl) bleibt es leer -- ein blockierter Roboter würde sonst glauben, er
fahre, und Geisterpunkte vor sich herschieben.

**Lücken.** Türen und Durchgänge mittig anfahren statt knapp am Rahmen
vorbei: Aus den freien Richtungen werden zusammenhängende Bereiche
gebildet, gewählt wird deren Mitte oder eine Richtung mit Abstand zum
Rand.

**Bogenplanung.** Statt vor einem Hindernis anzuhalten und auf der Stelle
zu wenden, rechnet der Erkunder einige Bögen (Drehrate, seitliches
Schieben) eine gute Sekunde voraus und prüft mit dem Umriss des Roboters,
ob überall Luft bleibt. Unter den sicheren Bögen gewinnt der, der den
gewählten Kurs hält, Luft lässt und weit führt. Kein Zufall: gleiche
Lage, gleicher Befehl -- das macht das Nachspielen echter Fahrten
aussagekräftig.

**Hochheben.** Wird der Roboter hochgenommen oder getragen, ändert sich
das Lidar-Bild rundum auf einmal, ohne dass er selbst fährt. Dann hält er
an und vergisst, was er gelernt hat (Gedächtnis, Kurs), bis das Bild
wieder ruhig ist.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .. import config


def _winkel_normieren(a: float) -> float:
    """Auf -pi..pi."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _lage_relativ(von, nach) -> tuple[float, float, float]:
    """Lage ``nach`` im Rahmen von ``von`` (dx, dy, dth)."""
    dx, dy = nach[0] - von[0], nach[1] - von[1]
    c, s = math.cos(von[2]), math.sin(von[2])
    return dx * c + dy * s, -dx * s + dy * c, _winkel_normieren(nach[2] - von[2])


def _in_rahmen(punkte, lage) -> np.ndarray:
    """Punkte aus dem Rahmen einer Lage (x, y, th) in den Rahmen darüber."""
    x, y, th = lage
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((
        x + punkte[:, 0] * c - punkte[:, 1] * s,
        y + punkte[:, 0] * s + punkte[:, 1] * c,
    ))


def _aus_rahmen(punkte, lage) -> np.ndarray:
    """Umkehrung von ``_in_rahmen``: Punkte in den Rahmen der Lage."""
    x, y, th = lage
    c, s = math.cos(th), math.sin(th)
    dx, dy = punkte[:, 0] - x, punkte[:, 1] - y
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


# --- Kurzgedächtnis --------------------------------------------------------------


class Kurzgedaechtnis:
    """Lidar-Punkte der letzten Sekunden, in den aktuellen Rahmen gerechnet.

    ``fortschreiben(dt)`` in jedem Takt, ``punkte(scan)`` liefert die Punkte
    der Runde plus die gemerkten. Gerechnet wird je neuer Runde; dazwischen
    kommt dasselbe Ergebnis zurück -- genau wie ``scan.punkte()``, das der
    Erkunder vorher benutzte.
    """

    def __init__(self, odometry=None) -> None:
        self._odometry = odometry
        # Eigene Lage (x, y, th) in einem frei gewählten Rahmen. Läuft auch
        # beim Leeren weiter: Der gewählte Kurs bezieht sich auf sie.
        self.lage = (0.0, 0.0, 0.0)
        self.misst = False
        self._runden: list[tuple[float, np.ndarray]] = []  # (Zeit, Punkte im Lage-Rahmen)
        self._letzte_zeit: float | None = None
        self._ergebnis = np.zeros((0, 2))
        self.gemerkt = 0  # gemerkte Punkte im letzten Ergebnis

    @property
    def kurs(self) -> float | None:
        """Aktuelle Blickrichtung im eigenen Rahmen, None ohne Messung."""
        return self.lage[2] if self.misst else None

    def leeren(self) -> None:
        self._runden.clear()
        self._letzte_zeit = None
        self.gemerkt = 0

    def fortschreiben(self, dt: float) -> None:
        """Eigene Lage um einen Takt weiterrechnen."""
        odo = self._odometry
        self.misst = odo is not None and bool(getattr(odo, "misst_tatsaechlich", False))
        if not self.misst:
            self.leeren()
            return
        vx, vy, om = odo.geschwindigkeit()
        x, y, th = self.lage
        # Mit dem Winkel in der Mitte des Takts -- in Kurven genauer.
        mitte = th + om * dt / 2.0
        c, s = math.cos(mitte), math.sin(mitte)
        self.lage = (x + (vx * c - vy * s) * dt, y + (vx * s + vy * c) * dt,
                     _winkel_normieren(th + om * dt))

    def punkte(self, scan) -> np.ndarray:
        """Punkte der Runde und gemerkte Punkte im Roboterrahmen (Nx2)."""
        if not self.misst:
            self.gemerkt = 0
            self._ergebnis = scan.punkte()
            return self._ergebnis
        if scan.zeit == self._letzte_zeit:
            return self._ergebnis
        self._letzte_zeit = scan.zeit
        aktuell = scan.punkte()

        # Alte Runden verwerfen, die aktuelle merken -- nur, was nah genug
        # ist, um je wieder zu zählen.
        grenze = scan.zeit - config.AUSWEICH_GEDAECHTNIS_S
        self._runden = [(z, p) for z, p in self._runden if z >= grenze]
        alt = (np.concatenate([p for _, p in self._runden])
               if self._runden else np.zeros((0, 2)))
        if len(aktuell):
            nah = np.hypot(aktuell[:, 0], aktuell[:, 1]) < 2.0 * config.AUSWEICH_GEDAECHTNIS_M
            self._runden.append((scan.zeit, _in_rahmen(aktuell[nah], self.lage)))

        if len(alt) == 0:
            self.gemerkt = 0
            self._ergebnis = aktuell
            return aktuell
        alt = _aus_rahmen(alt, self.lage)
        abstand = np.hypot(alt[:, 0], alt[:, 1])
        behalten = abstand < config.AUSWEICH_GEDAECHTNIS_M

        # Freiräumen: Sieht der Lidar jetzt in dieser Richtung deutlich
        # weiter, ist das Ding weg (Mensch gegangen, Tür auf, Messfehler).
        # Kein Strahl in der Richtung oder Punkt in der Totzone: behalten --
        # genau dafür ist das Gedächtnis da.
        sicht = np.full(360, np.inf)
        if len(aktuell):
            fach = np.round(np.degrees(np.arctan2(aktuell[:, 1], aktuell[:, 0]))).astype(int) % 360
            np.minimum.at(sicht, fach, np.hypot(aktuell[:, 0], aktuell[:, 1]))
        fach_alt = np.round(np.degrees(np.arctan2(alt[:, 1], alt[:, 0]))).astype(int) % 360
        s = sicht[fach_alt]
        durchsehen = np.isfinite(s) & (s > abstand + config.AUSWEICH_FREIRAEUMEN_M)
        durchsehen &= abstand >= config.LIDAR_MIN_M
        behalten &= ~durchsehen

        self.gemerkt = int(behalten.sum())
        self._ergebnis = np.concatenate((aktuell, alt[behalten])) if self.gemerkt else aktuell
        return self._ergebnis


# --- Lücken ----------------------------------------------------------------------


def frei_je_richtung(punkte, richtungen_grad, halbe_breite: float) -> np.ndarray:
    """``frei_in_richtung`` für viele Richtungen auf einmal (unendlich = frei)."""
    a = np.radians(np.asarray(richtungen_grad, dtype=float))[:, None]
    if len(punkte) == 0:
        return np.full(a.shape[0], np.inf)
    c, s = np.cos(a), np.sin(a)
    vor = punkte[None, :, 0] * c + punkte[None, :, 1] * s
    quer = -punkte[None, :, 0] * s + punkte[None, :, 1] * c
    vor = np.where((vor > 0.0) & (np.abs(quer) < halbe_breite), vor, np.inf)
    return vor.min(axis=1)


def luecken(punkte, halbe_breite: float, schwelle: float, reichweite: float) -> list[tuple[float, float]]:
    """Gute Richtungen (Grad, frei in m) aus den freien Bereichen ringsum.

    Ein Bereich ist eine zusammenhängende Folge von Richtungen, in denen
    mindestens ``schwelle`` frei ist. Von jedem kommt die Mitte und jede
    10°-Richtung mit ``AUSWEICH_LUECKE_RAND_GRAD`` Abstand zu den Rändern
    in Frage. So wird eine Tür mittig angefahren, und in einem großen
    freien Bereich bleibt trotzdem Auswahl für den Zufall.
    """
    raster = config.AUSWEICH_LUECKE_RASTER_GRAD
    rand = config.AUSWEICH_LUECKE_RAND_GRAD
    richtungen = np.arange(-180, 180, raster)
    frei = np.minimum(frei_je_richtung(punkte, richtungen, halbe_breite), reichweite)
    gut = frei >= schwelle
    n = len(richtungen)

    def normiert(r: float) -> float:
        return (r + 180.0) % 360.0 - 180.0

    def frei_bei(r: float) -> float:
        return float(frei[int(round((normiert(r) + 180) / raster)) % n])

    if gut.all():
        return [(float(r), frei_bei(r)) for r in range(-180, 180, 10)]
    if not gut.any():
        return []

    # Bereiche ringsum suchen, beginnend hinter einer schlechten Richtung,
    # damit kein Bereich bei -180°/180° zerteilt wird.
    start = int(np.argmin(gut))
    bereiche = []  # (erster Index, Länge)
    laenge = 0
    for k in range(1, n + 1):
        i = (start + k) % n
        if gut[i]:
            laenge += 1
        elif laenge:
            bereiche.append(((i - laenge) % n, laenge))
            laenge = 0

    ergebnis: dict[float, float] = {}
    for erster, laenge in bereiche:
        von = float(richtungen[erster])
        bis = von + (laenge - 1) * raster  # darf über 180 hinausgehen
        mitte = normiert(round((von + bis) / 2.0 / raster) * raster)
        ergebnis[mitte] = frei_bei(mitte)
        r = math.ceil((von + rand) / 10.0) * 10
        while r <= bis - rand:
            ergebnis[normiert(r)] = frei_bei(r)
            r += 10
    return sorted(ergebnis.items())


def richtung_waehlen(punkte, ziel_grad: float, noetig: float, halbe_breite: float,
                     letzte_seite: float = 0.0) -> tuple[float, float] | None:
    """Freie Fahrtrichtung möglichst nah an ``ziel_grad``: (Grad, frei in m).

    Für Folgen und Heimfahrt: Dort gibt es ein Ziel (Person, Startpunkt),
    und der Mecanum-Antrieb kann schräg daran vorbeischieben, ohne den
    Blick abzuwenden. Frei heißt: ``noetig`` Meter im Fahrschlauch, mit
    ``AUSWEICH_RAND_M`` zusätzlich zur halben Breite.

    - Ist die Zielrichtung selbst frei, kommt sie unverändert zurück.
    - Sonst die nächste freie Richtung bis ``UMFAHREN_MAX_GRAD`` daneben.
      Die zuletzt gewählte Seite bekommt einen Vorsprung von
      ``UMFAHREN_SEITE_GRAD``, sonst pendelt er vor einem breiten
      Hindernis zwischen links und rechts.
    - Abstand zum Rand des freien Bereichs: mindestens
      ``UMFAHREN_RAND_GRAD``; ist der Bereich schmaler (Tür), seine Mitte.

    None, wenn nichts frei ist -- dann anhalten, nicht raten.
    """
    raster = config.UMFAHREN_RASTER_GRAD
    n = int(config.UMFAHREN_MAX_GRAD // raster)
    versatz = np.arange(-n, n + 1) * raster
    frei = frei_je_richtung(punkte, ziel_grad + versatz, halbe_breite + config.AUSWEICH_RAND_M)
    gut = frei >= noetig
    if gut[n]:
        return float(ziel_grad), float(frei[n])
    if not gut.any():
        return None

    kosten = np.abs(versatz).astype(float)
    if letzte_seite:
        kosten += np.where(np.sign(versatz) == -np.sign(letzte_seite), config.UMFAHREN_SEITE_GRAD, 0.0)
    i = int(np.argmin(np.where(gut, kosten, np.inf)))

    # Zusammenhängenden freien Bereich um i suchen und vom Rand wegrücken.
    a = i
    while a > 0 and gut[a - 1]:
        a -= 1
    b = i
    while b < len(gut) - 1 and gut[b + 1]:
        b += 1
    k = int(math.ceil(config.UMFAHREN_RAND_GRAD / raster))
    if b - a < 2 * k:
        i = (a + b) // 2
    else:
        i = min(max(i, a + k), b - k)
    return float(_winkel_normieren(math.radians(ziel_grad + versatz[i])) * 180.0 / math.pi), float(frei[i])


# --- Bogenplanung ----------------------------------------------------------------


def _umriss_abstand(lx, ly):
    """Abstand von Punkten (im Rahmen des Roboters) zu seinem Umriss."""
    ex = np.maximum(np.maximum(-config.AUSWEICH_HINTEN_M - lx, lx - config.AUSWEICH_VORN_M), 0.0)
    ey = np.maximum(np.abs(ly) - config.AUSWEICH_HALBE_BREITE_M, 0.0)
    return np.hypot(ex, ey)


def bogen_planen(punkte, vx_max: float, kurs_fehler: float, letztes_omega: float):
    """Besten sicheren Bogen wählen: ``((vx, vy, omega), Luft in m)`` oder None.

    ``kurs_fehler``: um wie viel (rad, links positiv) der gewählte Kurs von
    der aktuellen Blickrichtung abweicht; 0, wenn keiner bekannt ist -- dann
    zählt einfach, möglichst wenig zu lenken. None heißt: kein Bogen ist
    sicher, der Erkunder soll wenden.
    """
    vx_min = 0.35 * config.EXPLORE_SPEED
    stufen = sorted({max(vx_min, vx_max), max(vx_min, 0.7 * vx_max)}, reverse=True)
    kand = np.array([
        (v, vy, om)
        for v in stufen for vy in config.AUSWEICH_VYS for om in config.AUSWEICH_OMEGAS
    ])
    vx, vy, om = kand[:, 0:1], kand[:, 1:2], kand[:, 2:3]

    # Lagen entlang der Bögen (exakt für gleichbleibende Geschwindigkeiten).
    t = np.arange(0.0, config.AUSWEICH_HORIZONT_S + 1e-9, config.AUSWEICH_SCHRITT_S)[None, :]
    th = om * t
    gerade = np.abs(om) < 1e-6
    om_s = np.where(gerade, 1.0, om)
    sin_t, cos_t = np.sin(th), np.cos(th)
    px = np.where(gerade, vx * t, (vx * sin_t - vy * (1.0 - cos_t)) / om_s)
    py = np.where(gerade, vy * t, (vx * (1.0 - cos_t) + vy * sin_t) / om_s)

    # Nur Punkte, die je näher als LUFT_GUT kommen können -- der Rest
    # ändert an der (gekappten) Bewertung nichts.
    vy_max = max(map(abs, config.AUSWEICH_VYS)) or 1.0
    reich = (stufen[0] + vy_max) * config.AUSWEICH_HORIZONT_S
    umriss = math.hypot(max(config.AUSWEICH_VORN_M, config.AUSWEICH_HINTEN_M),
                        config.AUSWEICH_HALBE_BREITE_M)
    d = np.hypot(punkte[:, 0], punkte[:, 1]) if len(punkte) else np.zeros(0)
    nah = punkte[d < reich + umriss + config.AUSWEICH_LUFT_GUT_M] if len(punkte) else np.zeros((0, 2))

    if len(nah):
        # Form: Kandidat x Schritt x Punkt
        dx = nah[None, None, :, 0] - px[:, :, None]
        dy = nah[None, None, :, 1] - py[:, :, None]
        c, s = cos_t[:, :, None], sin_t[:, :, None]
        abstand = _umriss_abstand(dx * c + dy * s, -dx * s + dy * c).min(axis=2)
        luft = abstand.min(axis=1)
        jetzt = float(abstand[0, 0])  # Schritt 0 ist für alle Bögen gleich
    else:
        luft = np.full(len(kand), np.inf)
        jetzt = math.inf
    # Sicher, wenn überall genug Rand bleibt -- oder, wenn schon jetzt etwas
    # näher ist (Türrahmen, seitlich vorbei), es wenigstens nicht näher wird.
    # Sonst stünde er neben jedem Rahmen und wendete.
    sicher = (luft >= config.AUSWEICH_RAND_M) | (
        (jetzt < config.AUSWEICH_RAND_M) & (luft >= jetzt - 0.005))
    if not sicher.any():
        return None

    # Weite: wie weit es vom Ende des Bogens in dessen Richtung frei ist.
    ende_x, ende_y, ende_th = px[:, -1], py[:, -1], th[:, -1]
    weite_gut = config.AUSWEICH_WEITE_GUT_M
    fern = (punkte[d < reich + weite_gut + config.EXPLORE_HALF_WIDTH_M]
            if len(punkte) else np.zeros((0, 2)))
    if len(fern):
        dx = fern[None, :, 0] - ende_x[:, None]
        dy = fern[None, :, 1] - ende_y[:, None]
        c, s = np.cos(ende_th)[:, None], np.sin(ende_th)[:, None]
        vor = dx * c + dy * s
        quer = -dx * s + dy * c
        vor = np.where((vor > 0.0) & (np.abs(quer) < config.EXPLORE_HALF_WIDTH_M), vor, np.inf)
        weite = np.minimum(vor.min(axis=1), weite_gut)
    else:
        weite = np.full(len(kand), weite_gut)

    rest = np.abs((kurs_fehler - ende_th + np.pi) % (2.0 * np.pi) - np.pi)
    om_max = max(map(abs, config.AUSWEICH_OMEGAS)) or 1.0
    wert = (
        -config.AUSWEICH_GEWICHT_KURS * rest / np.pi
        + config.AUSWEICH_GEWICHT_LUFT * np.minimum(luft, config.AUSWEICH_LUFT_GUT_M) / config.AUSWEICH_LUFT_GUT_M
        + config.AUSWEICH_GEWICHT_TEMPO * kand[:, 0] / config.EXPLORE_SPEED
        + config.AUSWEICH_GEWICHT_WEITE * weite / weite_gut
        - config.AUSWEICH_GEWICHT_RUCK * np.abs(kand[:, 2] - letztes_omega) / (2.0 * om_max)
        - config.AUSWEICH_GEWICHT_SEITLICH * np.abs(kand[:, 1]) / vy_max
    )
    wert = np.where(sicher, wert, -np.inf)
    i = int(np.argmax(wert))  # bei Gleichstand der erste: kein Zufall
    return (float(kand[i, 0]), float(kand[i, 1]), float(kand[i, 2])), float(luft[i])


# --- Hochheben -------------------------------------------------------------------

_SEKTOREN = 36


def _sektoren(punkte) -> np.ndarray:
    """Median-Abstand je 10°-Sektor, NaN ohne Messung."""
    werte = np.full(_SEKTOREN, np.nan)
    if len(punkte) == 0:
        return werte
    w = np.degrees(np.arctan2(punkte[:, 1], punkte[:, 0])) % 360.0
    fach = (w // (360.0 / _SEKTOREN)).astype(int) % _SEKTOREN
    r = np.hypot(punkte[:, 0], punkte[:, 1])
    for k in range(_SEKTOREN):
        auswahl = r[fach == k]
        if len(auswahl):
            werte[k] = float(np.median(auswahl))
    return werte


class HochhebeErkennung:
    """Merkt, wenn der Roboter hochgehoben oder getragen wird.

    Vergleicht jede neue Runde mit einer früheren, die eigene Bewegung
    dazwischen herausgerechnet. Ein Mensch, der vorbeigeht, verändert wenige
    Sektoren; wird der Roboter gekippt oder versetzt, springt das Bild fast
    überall.

    Mit gemessener Lage vergleicht er mit der Runde von vor
    ``AUSWEICH_HOCH_FENSTER_S``: Wer ihn langsam trägt, verschiebt das Bild
    von Runde zu Runde nur wenig, über eine Sekunde aber deutlich -- und die
    Räder melden davon nichts. Ohne Messung bleibt nur die direkt vorige
    Runde, sonst sähe schon seine eigene Fahrt wie Tragen aus.

    Grenzen: Wird er waagerecht und gerade nach oben gehoben, bleiben die
    Wände gleich weit weg -- das sieht er nicht. Drehen sich die Räder dann
    in der Luft, greift die Festfahr-Erkennung (Board misst Fahrt, Lidar-
    Bild steht) und am Ende das Aufgeben.
    """

    def __init__(self) -> None:
        self.zuruecksetzen()

    def zuruecksetzen(self) -> None:
        self.hochgehoben = False
        self.anteil = 0.0
        self._verlauf: deque = deque()  # (Zeit, Punkte, Lage oder None)
        self._letzte_zeit: float | None = None
        self._unruhig = 0
        self._ruhig_seit: float | None = None

    def pruefen(self, scan, lage) -> bool:
        """Mit der aktuellen Runde prüfen. ``lage`` nur, wenn gemessen."""
        if scan.zeit == self._letzte_zeit:
            return self.hochgehoben
        self._letzte_zeit = scan.zeit
        punkte = scan.punkte()
        vorige = self._vergleichsrunde(scan.zeit, lage)
        self._verlauf.append((scan.zeit, punkte, lage))
        if vorige is None:
            return self.hochgehoben

        _zeit, alt_punkte, alt_lage = vorige
        if lage is not None and alt_lage is not None and len(alt_punkte):
            # Vorige Runde in den jetzigen Rahmen: eigene Fahrt ist kein Sprung.
            alt_punkte = _aus_rahmen(alt_punkte, _lage_relativ(alt_lage, lage))
        jetzt, alt = _sektoren(punkte), _sektoren(alt_punkte)
        j_ok, a_ok = ~np.isnan(jetzt), ~np.isnan(alt)
        beide = j_ok & a_ok
        verglichen = int((j_ok | a_ok).sum())
        if verglichen < 6:
            # Zu wenig zu sehen (freies Feld, Lidar verdeckt) -- kein Urteil.
            self.anteil = 0.0
        else:
            sprung = j_ok != a_ok
            sprung[beide] = np.abs(jetzt[beide] - alt[beide]) > config.AUSWEICH_HOCH_SPRUNG_M
            self.anteil = float(sprung.sum()) / verglichen

        if not self.hochgehoben:
            self._unruhig = self._unruhig + 1 if self.anteil >= config.AUSWEICH_HOCH_ANTEIL else 0
            if self._unruhig >= config.AUSWEICH_HOCH_RUNDEN:
                self.hochgehoben = True
                self._ruhig_seit = None
        elif self.anteil < config.AUSWEICH_HOCH_RUHIG_ANTEIL:
            if self._ruhig_seit is None:
                self._ruhig_seit = scan.zeit
            if scan.zeit - self._ruhig_seit >= config.AUSWEICH_HOCH_RUHE_S:
                self.hochgehoben = False
                self._unruhig = 0
        else:
            self._ruhig_seit = None
        return self.hochgehoben

    def _vergleichsrunde(self, zeit: float, lage):
        """Die Runde, mit der die neue verglichen wird (siehe Klassentext)."""
        while self._verlauf and zeit - self._verlauf[0][0] > config.AUSWEICH_HOCH_FENSTER_S:
            self._verlauf.popleft()
        if not self._verlauf:
            return None
        if lage is None:
            return self._verlauf[-1]
        # Die älteste Runde im Fenster mit Lage; fehlt sie dort (Messung
        # gerade erst wieder da), die vorige.
        for eintrag in self._verlauf:
            if eintrag[2] is not None:
                return eintrag
        return self._verlauf[-1]
