"""Wo steht der Roboter in der Karte?

Die Lidar-Odometrie (``scan_odometrie.py``) weiß, wie er sich seit ihrem
Start bewegt hat -- in ihrem eigenen Rahmen, mit langsamer Drift. Die Karte
hat einen anderen Rahmen (den von slam_toolbox bzw. der gespeicherten
Karte). Diese Klasse hält die **Korrektur** zwischen beiden:

    Lage in der Karte = Korrektur ∘ Lage laut Lidar-Odometrie

Mehrmals pro Sekunde wird die aktuelle Aufnahme auf die Karte gelegt
(``verfeinern``): Jeder Lidar-Punkt sollte auf einer Wand der Karte liegen.
Wie weit er davon weg ist, steht im vorausgerechneten Abstandsfeld der
Karte -- das macht den Abgleich schnell genug für den Pi, ohne Punkte
paarweise zu suchen wie beim ICP. Gelingt er, rückt die Korrektur ein
Stück in Richtung des Ergebnisses. So bleibt die Lage auch auf langen
Fahrten an der Karte, statt mit der Odometrie wegzudriften.

Beim Kartieren beginnen Karte und Lidar-Odometrie am selben Ort, die
Korrektur ist anfangs null. Wird eine gespeicherte Karte geladen, weiß
niemand, wo er steht: Dann wird im Umkreis eines Hinweises gesucht
(``suchen``) -- zuerst am Ausgangspunkt der Karte, denn dort steht er
meistens, oder dort, wo jemand auf der Karte hingetippt hat.

Bewusst nicht slam_toolbox im Lokalisierungsmodus: Dafür bräuchte es eine
zweite Brücke aus dem Container zurück, und das Fahren mit Karte hinge an
Docker und ROS2 Foxy. So geht es mit numpy allein (E12).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import NamedTuple

import numpy as np

from .. import config

log = logging.getLogger(__name__)

#: So viele Punkte je Aufnahme reichen für den Abgleich; mehr kosten nur Zeit.
_MAX_PUNKTE = 180
_SUCH_PUNKTE = 90
#: Punkte, die im unbekannten Teil der Karte landen, zählen beim Anteil
#: nicht mit -- beim Kartieren ist das der Raum, den er gerade erst sieht.
#: Bleiben weniger als so viele übrig, ist der Abgleich nichts wert.
_MIN_PUNKTE = 30
#: Höchstens so oft (s) eine Suche wiederholen, wenn sie nichts findet.
_SUCHE_PAUSE_S = 2.0


class Abgleich(NamedTuple):
    x: float
    y: float
    theta: float
    anteil: float      # getroffene Punkte / Punkte in bekanntem Gebiet
    residuum: float    # mittlerer Abstand der getroffenen Punkte (m)
    punkte: int        # Punkte in bekanntem Gebiet


def _normalisieren(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


def verketten(a, b) -> tuple[float, float, float]:
    """Pose ``b`` (relativ zu ``a``) an Pose ``a`` anhängen."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalisieren(a[2] + b[2])


def umkehren(a) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def bewerten(karte, punkte, pose) -> Abgleich:
    """Wie gut passt die Aufnahme bei dieser Lage auf die Karte?"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    qx = c * punkte[:, 0] - s * punkte[:, 1] + pose[0]
    qy = s * punkte[:, 0] + c * punkte[:, 1] + pose[1]
    bekannt = ~karte.unbekannt_bei(qx, qy)
    n = int(bekannt.sum())
    if n == 0:
        return Abgleich(pose[0], pose[1], pose[2], 0.0, float("inf"), 0)
    d = karte.abstand_bei(qx[bekannt], qy[bekannt])
    treffer = d < config.KARTE_TREFFER_M
    residuum = float(d[treffer].mean()) if treffer.any() else float("inf")
    return Abgleich(pose[0], pose[1], pose[2], float(treffer.mean()), residuum, n)


def verfeinern(karte, punkte, start, iterationen: int = 12) -> Abgleich:
    """Lage nahe ``start`` so verschieben, dass die Punkte auf Wänden liegen.

    Gauß-Newton auf dem Abstandsfeld: Jeder Punkt zieht in Richtung der
    nächsten Wand (Steigung des Feldes), und zwar umso weniger, je weiter
    er weg ist -- Punkte auf Menschen oder verrückten Stühlen sollen die
    Lage nicht verbiegen. Punkte weiter als ``KARTE_ABSTAND_MAX_M`` von
    jeder Wand ziehen gar nicht (dort ist das Feld flach).
    """
    x, y, th = start
    px, py = punkte[:, 0], punkte[:, 1]
    for k in range(iterationen):
        c, s = math.cos(th), math.sin(th)
        qx = c * px - s * py + x
        qy = s * px + c * py + y
        d, (gx, gy), innen = karte.abtasten(qx, qy)
        # Erst grob (weiter Fangbereich), dann nur noch nahe Punkte.
        tor = 0.5 if k < iterationen // 2 else 0.25
        nutz = innen & (d < tor)
        if int(nutz.sum()) < _MIN_PUNKTE:
            break
        w = np.where(d < 0.05, 1.0, 0.05 / np.maximum(d, 1e-6))[nutz]
        dth = gx * (-s * px - c * py) + gy * (c * px - s * py)
        J = np.stack((gx[nutz], gy[nutz], dth[nutz]), axis=1)
        H = (J * w[:, None]).T @ J + np.diag((1e-3, 1e-3, 1e-2))
        b = (J * w[:, None]).T @ d[nutz]
        try:
            schritt = -np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            break
        # Große Sprünge in kleinen Schritten: Das Feld ist nur nahe der
        # Wände aussagekräftig.
        laenge = math.hypot(schritt[0], schritt[1])
        if laenge > 0.1:
            schritt[:2] *= 0.1 / laenge
        schritt[2] = max(-0.08, min(0.08, schritt[2]))
        x += float(schritt[0])
        y += float(schritt[1])
        th = _normalisieren(th + float(schritt[2]))
        if laenge < 1e-4 and abs(schritt[2]) < 1e-4:
            break
    return bewerten(karte, punkte, (x, y, th))


def suchen(karte, punkte, mitte: tuple[float, float] | None, radius: float,
           winkel: float | None = None, min_anteil: float | None = None) -> Abgleich | None:
    """Lage im Umkreis von ``mitte`` suchen (ohne ``mitte``: in der ganzen Karte).

    Alle freien Stellen im Raster, alle Blickrichtungen (oder ±30° um
    ``winkel``), bewertet mit dem Abstandsfeld; die besten werden verfeinert.
    Nur ein klarer Treffer zählt -- lieber "weiß nicht" als eine falsche
    Lage, mit der er dann losfährt: Er muss ``min_anteil`` der Punkte
    treffen, und keine deutlich andere Lage darf fast genauso gut passen
    (ein rechteckiges Zimmer passt auch um 180° gedreht).
    """
    if min_anteil is None:
        min_anteil = config.KARTE_SUCH_MIN_ANTEIL
    if len(punkte) < _MIN_PUNKTE:
        return None
    pkt = punkte[:: max(1, math.ceil(len(punkte) / _SUCH_PUNKTE))]
    raster = config.KARTE_SUCH_RASTER_M if mitte is not None else 2 * config.KARTE_SUCH_RASTER_M
    schritt = max(1, int(round(raster / karte.aufloesung)))
    iy, ix = np.nonzero(karte.frei[::schritt, ::schritt])
    iy, ix = iy * schritt, ix * schritt
    kx, ky = karte.welt(ix, iy)
    gut = karte.abstand[iy, ix] >= 0.1
    if mitte is not None:
        gut &= np.hypot(kx - mitte[0], ky - mitte[1]) <= radius
    kx, ky = kx[gut], ky[gut]
    if len(kx) == 0:
        return None
    if winkel is None:
        winkel_liste = np.radians(np.arange(0.0, 360.0, config.KARTE_SUCH_GRAD))
    else:
        winkel_liste = winkel + np.radians(np.arange(-30.0, 30.1, config.KARTE_SUCH_GRAD))

    deckel = 0.3
    abstand = np.minimum(karte.abstand, deckel)
    # Unbekannte Zellen kosten wie "weit weg von jeder Wand".
    abstand = np.where(karte.unbekannt, deckel, abstand)
    besten: list[tuple[float, float, float, float]] = []
    for th in winkel_liste:
        c, s = math.cos(th), math.sin(th)
        rx = c * pkt[:, 0] - s * pkt[:, 1]
        ry = s * pkt[:, 0] + c * pkt[:, 1]
        # In Stücken, damit das Feld (Kandidaten x Punkte) klein bleibt.
        for a in range(0, len(kx), 4000):
            qx = kx[a:a + 4000, None] + rx[None, :]
            qy = ky[a:a + 4000, None] + ry[None, :]
            jx, jy = karte.zelle(qx, qy)
            innen = karte.innen(jx, jy)
            d = np.where(
                innen,
                abstand[np.clip(jy, 0, karte.hoehe - 1), np.clip(jx, 0, karte.breite - 1)],
                deckel,
            )
            wert = d.mean(axis=1)
            for i in np.argsort(wert)[:4]:
                besten.append((float(wert[i]), float(kx[a + i]), float(ky[a + i]), float(th)))
    besten.sort()
    kandidaten: list[Abgleich] = []
    for _, x, y, th in besten[:12]:
        a = verfeinern(karte, punkte, (x, y, th), iterationen=16)
        if a.punkte >= _MIN_PUNKTE:
            kandidaten.append(a)
    if not kandidaten:
        return None
    kandidaten.sort(key=lambda a: (a.anteil, -a.residuum), reverse=True)
    ergebnis = kandidaten[0]
    if ergebnis.anteil < min_anteil:
        return None
    for a in kandidaten[1:]:
        anders = (
            math.hypot(a.x - ergebnis.x, a.y - ergebnis.y) > config.KARTE_MEHRDEUTIG_M
            or abs(_normalisieren(a.theta - ergebnis.theta))
            > math.radians(config.KARTE_MEHRDEUTIG_GRAD)
        )
        if anders and a.anteil >= ergebnis.anteil - config.KARTE_MEHRDEUTIG_ANTEIL:
            log.info(
                "Suche mehrdeutig: (%.2f, %.2f, %.0f°) und (%.2f, %.2f, %.0f°) passen gleich gut",
                ergebnis.x, ergebnis.y, math.degrees(ergebnis.theta),
                a.x, a.y, math.degrees(a.theta),
            )
            return None
    return ergebnis


class Kartenortung:
    """Lage des Roboters in der aktuellen Karte, laufend nachgeführt."""

    def __init__(self, scan_odometrie) -> None:
        self._odo = scan_odometrie
        self._lock = threading.Lock()
        self._karte = None
        self._live = False
        self._start = (0.0, 0.0, 0.0)
        self._korrektur = (0.0, 0.0, 0.0)
        self._lauf: int | None = None        # Lauf der Odometrie, zu dem die Korrektur gehört
        self._letzte_nr: int | None = None
        self._erfolg_zeit: float | None = None
        self._letzte_pose: tuple[float, float, float] | None = None
        self._anteil: float | None = None
        self._residuum: float | None = None
        self._suche: tuple | None = None     # (mitte, radius, winkel, min_anteil) oder None
        self._suche_zuletzt = 0.0
        self._suche_text: str | None = None
        self._spur: deque = deque(maxlen=config.KARTE_SPUR_MAX)
        self._abgleiche = 0
        self._verworfen = 0

    # --- Karte -------------------------------------------------------------------

    @property
    def karte(self):
        return self._karte

    @property
    def start(self) -> tuple[float, float, float]:
        """Ausgangspunkt der Karte (dort begann die Kartierung) -- "zu Hause"."""
        return self._start

    def karte_setzen(self, karte, start=None, live: bool = False) -> None:
        """Neue Karte übernehmen.

        ``live``: Die Karte wächst gerade (Kartierung), ihr Rahmen ist der
        der Lidar-Odometrie, die mit der Kartierung neu begann (deshalb ruft
        der MappingManager vor dem Start ``entladen``). Die erste Karte
        einer Kartierung setzt die Korrektur auf null, jede weitere lässt
        sie, wie die Nachführung sie gefunden hat. Sonst ist es eine
        geladene Karte, und die Lage muss erst gesucht werden.
        """
        self._odo.start("ortung")
        with self._lock:
            neu = self._karte is None or live != self._live
            self._karte = karte
            self._live = live
            if start is not None:
                self._start = tuple(start)
            if live and neu:
                # Kartierung beginnt: Karte und Odometrie teilen den Ursprung.
                self._start = (0.0, 0.0, 0.0)
                self._korrektur = (0.0, 0.0, 0.0)
                self._lauf = self._odo.lauf
                self._erfolg_zeit = time.monotonic()
                self._suche = None
                self._spur.clear()
            elif not live:
                self._lauf = None
                self._erfolg_zeit = None
                self._spur.clear()
                # Zuerst mit der Blickrichtung vom Ende der Kartierung (dort steht
                # er meist noch so), findet er sich so nicht: alle Richtungen.
                self._suche = ((self._start[0], self._start[1]), config.KARTE_SUCH_RADIUS_M,
                               self._start[2], config.KARTE_SUCH_MIN_ANTEIL)
                self._suche_zuletzt = 0.0
                self._suche_text = "sucht die eigene Lage am Ausgangspunkt der Karte"

    @property
    def live(self) -> bool:
        """Die Karte stammt aus der laufenden (oder gerade beendeten) Kartierung."""
        return self._karte is not None and self._live

    def entladen(self) -> None:
        with self._lock:
            self._karte = None
            self._live = False
            self._lauf = None
            self._suche = None
            self._spur.clear()
        self._odo.stop("ortung")

    def start_setzen(self, start) -> None:
        """Neuer Ausgangspunkt ("zu Hause") in dieser Karte."""
        with self._lock:
            self._start = (float(start[0]), float(start[1]), float(start[2]))

    def lage_setzen(self, x: float, y: float, winkel: float | None = None) -> None:
        """Jemand zeigt auf der Karte, wo der Roboter steht: dort suchen."""
        with self._lock:
            radius = 0.6 if winkel is not None else 1.0
            self._suche = ((float(x), float(y)), radius, winkel, config.KARTE_HINWEIS_MIN_ANTEIL)
            self._suche_zuletzt = 0.0
            self._suche_text = "sucht die eigene Lage an der gezeigten Stelle"
            self._erfolg_zeit = None

    # --- Abfrage -----------------------------------------------------------------

    def pose(self) -> tuple[float, float, float] | None:
        """Lage in der Karte (x, y, Bogenmaß) oder None, solange unbekannt."""
        with self._lock:
            if self._karte is None or self._lauf is None or self._lauf != self._odo.lauf:
                return None
            korrektur = self._korrektur
        odo = self._odo.pose()
        if odo is None:
            return None
        return verketten(korrektur, odo)

    @property
    def sicher(self) -> bool:
        """Lage bekannt und zuletzt bestätigt -- dann darf nach Karte gefahren werden."""
        with self._lock:
            if self._karte is None or self._lauf is None or self._lauf != self._odo.lauf:
                return False
            if self._suche is not None:
                return False
            return (
                self._erfolg_zeit is not None
                and time.monotonic() - self._erfolg_zeit <= config.KARTE_SICHER_S
            )

    def spur(self) -> list[list[float]]:
        with self._lock:
            return [list(p) for p in self._spur]

    def zustand(self) -> dict:
        pose = self.pose()
        sicher = self.sicher
        with self._lock:
            karte = self._karte
            return {
                "karte": karte is not None,
                "name": getattr(karte, "name", None),
                "live": karte is not None and self._live,
                "pose": (
                    {"x": round(pose[0], 3), "y": round(pose[1], 3),
                     "winkel": round(math.degrees(pose[2]), 1)}
                    if pose is not None else None
                ),
                "sicher": sicher,
                "sucht": self._suche is not None,
                "text": self._text(pose, sicher),
                "anteil": round(self._anteil, 2) if self._anteil is not None else None,
                "residuum_m": (
                    round(self._residuum, 3)
                    if self._residuum is not None and math.isfinite(self._residuum) else None
                ),
                "start": [round(self._start[0], 3), round(self._start[1], 3),
                          round(math.degrees(self._start[2]), 1)],
                "abgleiche": self._abgleiche,
                "verworfen": self._verworfen,
            }

    def _text(self, pose, sicher: bool) -> str:
        if self._karte is None:
            return "keine Karte geladen"
        if self._suche is not None:
            return self._suche_text or "sucht die eigene Lage"
        if pose is None:
            return "Lage unbekannt -- auf der Karte zeigen, wo er steht"
        if not sicher:
            return "Lage unsicher -- die Aufnahme passt gerade nicht zur Karte"
        return "Lage in der Karte bekannt"

    # --- Nachführen (im eigenen Thread aufrufen, rechnet einige ms) ---------------

    def abgleichen(self) -> None:
        """Die neueste Aufnahme auf die Karte legen."""
        karte = self._karte
        if karte is None or not self._odo.laeuft:
            return
        messung = self._odo.messung()
        if messung is None or messung.nr == self._letzte_nr:
            return
        self._letzte_nr = messung.nr
        punkte = messung.scan.punkte()
        if len(punkte) < _MIN_PUNKTE:
            return
        punkte = np.asarray(punkte, dtype=float)
        punkte = punkte[np.hypot(punkte[:, 0], punkte[:, 1]) <= config.LIDAR_MAX_M]
        if len(punkte) > _MAX_PUNKTE:
            punkte = punkte[:: math.ceil(len(punkte) / _MAX_PUNKTE)]
        odo_pose = messung.pose

        with self._lock:
            suche = self._suche
            lauf_passt = self._lauf is not None and self._lauf == self._odo.lauf
            korrektur = self._korrektur
            erfolg = self._erfolg_zeit
            letzte = self._letzte_pose

        if suche is None and not lauf_passt:
            # Die Odometrie begann neu (Neustart): Der Roboter steht noch
            # da, wo er zuletzt war -- dort suchen.
            mitte = (letzte[0], letzte[1]) if letzte is not None else self._start[:2]
            suche = (mitte, config.KARTE_SUCH_RADIUS_M, None, config.KARTE_SUCH_MIN_ANTEIL)
            with self._lock:
                self._suche = suche
                self._suche_text = "sucht die eigene Lage nach einem Neustart der Odometrie"
        elif (
            suche is None and erfolg is not None
            and time.monotonic() - erfolg > 2 * config.KARTE_SICHER_S
            and letzte is not None
        ):
            # Lange nichts gepasst (hochgehoben, verschoben): im Umkreis der
            # letzten guten Lage neu suchen.
            suche = ((letzte[0], letzte[1]), config.KARTE_SUCH_RADIUS_M, None,
                     config.KARTE_SUCH_MIN_ANTEIL)
            with self._lock:
                self._suche = suche
                self._suche_text = "hat die Lage verloren und sucht sie wieder"

        if suche is not None:
            self._suchen(karte, punkte, odo_pose, suche)
            return

        vorhersage = verketten(korrektur, odo_pose)
        a = verfeinern(karte, punkte, vorhersage)
        sprung = math.hypot(a.x - vorhersage[0], a.y - vorhersage[1])
        dreh = abs(_normalisieren(a.theta - vorhersage[2]))
        gut = (
            a.punkte >= _MIN_PUNKTE
            and a.anteil >= config.KARTE_MIN_ANTEIL
            and sprung <= config.KARTE_MAX_SPRUNG_M
            and dreh <= math.radians(config.KARTE_MAX_SPRUNG_GRAD)
        )
        with self._lock:
            if self._karte is not karte or self._suche is not None:
                return  # während des Rechnens kam eine neue Karte oder ein Hinweis
            self._anteil, self._residuum = a.anteil, a.residuum
            if not gut:
                self._verworfen += 1
                return
            self._abgleiche += 1
            w = config.KARTE_VERTRAUEN
            ziel = (
                vorhersage[0] + w * (a.x - vorhersage[0]),
                vorhersage[1] + w * (a.y - vorhersage[1]),
                _normalisieren(vorhersage[2] + w * _normalisieren(a.theta - vorhersage[2])),
            )
            self._korrektur = verketten(ziel, umkehren(odo_pose))
            self._erfolg_zeit = time.monotonic()
            self._letzte_pose = ziel
            self._spur_ergaenzen(ziel)

    def _suchen(self, karte, punkte, odo_pose, suche) -> None:
        if time.monotonic() - self._suche_zuletzt < _SUCHE_PAUSE_S:
            return
        self._suche_zuletzt = time.monotonic()
        mitte, radius, winkel, min_anteil = suche
        a = suchen(karte, punkte, mitte, radius, winkel, min_anteil)
        with self._lock:
            if self._karte is not karte or self._suche != suche:
                return
            if a is None:
                if winkel is not None:
                    # Mit vorgegebener Blickrichtung nichts: alle Richtungen.
                    self._suche = (mitte, radius, None, min_anteil)
                    self._suche_zuletzt = 0.0
                    return
                self._suche_text = (
                    "findet sich nicht -- auf der Karte zeigen, wo er steht"
                )
                return
            self._korrektur = verketten((a.x, a.y, a.theta), umkehren(odo_pose))
            self._lauf = self._odo.lauf
            self._erfolg_zeit = time.monotonic()
            self._letzte_pose = (a.x, a.y, a.theta)
            self._anteil, self._residuum = a.anteil, a.residuum
            self._suche = None
            self._suche_text = None
            self._spur_ergaenzen((a.x, a.y, a.theta))
        log.info(
            "Lage in der Karte gefunden: x=%.2f y=%.2f %.0f° (%.0f %% Treffer)",
            a.x, a.y, math.degrees(a.theta), 100 * a.anteil,
        )

    def _spur_ergaenzen(self, pose) -> None:
        """Nur mit gehaltener Sperre aufrufen."""
        if not self._spur or math.hypot(
            pose[0] - self._spur[-1][0], pose[1] - self._spur[-1][1]
        ) >= config.KARTE_SPUR_M:
            self._spur.append((round(pose[0], 2), round(pose[1], 2)))
