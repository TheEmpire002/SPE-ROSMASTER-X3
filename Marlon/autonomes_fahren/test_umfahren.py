"""Prüft das Umfahren beim Folgen und bei der Heimfahrt (F3) -- ohne Roboter.

    python Marlon/autonomes_fahren/test_umfahren.py

Fälle:

* ``richtung_waehlen``: freier Weg, Ausweichen zur Seite, Türmitte, nichts
  frei, Seite halten;
* Folgen mit Stuhl zwischen Roboter und Person: schiebt vorbei, berührt
  nichts, kommt auf Abstand;
* Folgen durch eine Tür, die nicht mittig vor ihm liegt;
* Hochheben beim Folgen: hält an, danach BEREIT;
* Heimfahrt um ein Hindernis herum, Hochheben bei der Heimfahrt und
  Heimfahrt ohne freien Weg (Meldung nach der Wartezeit).

Die gedachte Welt kommt aus ``test_ausweichen.py`` (dort nachgebaut, weil
das Importieren der Datei ihre Tests laufen ließe).
"""

import asyncio
import math
import pathlib
import random
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaktion import verhalten as vh  # noqa: E402
from rosmaster_assist.navigation import ausweichen as aw  # noqa: E402
from rosmaster_assist.navigation import homing as hm  # noqa: E402
from rosmaster_assist.navigation.explorer import frei_in_richtung  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

random.seed(3)
fehler = 0
UNENDLICH = float("inf")
# Lidar-Mitte bis Umriss: Darunter hätte er etwas berührt.
ROBOTER_R = 0.15


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


# --- Gedachte Welt (wie test_ausweichen.py) -------------------------------------


class Welt:
    def __init__(self, raum=(-3.0, 5.0, -3.0, 3.0), waende=(), kreise=(), x=0.0, y=0.0, theta=0.0):
        self.raum = raum
        self.waende = list(waende)
        self.kreise = list(kreise)
        self.x, self.y, self.theta = x, y, theta
        self.v_ist = (0.0, 0.0, 0.0)
        self.gekippt = False

    def strahl(self, richtung):
        dx, dy = math.cos(richtung), math.sin(richtung)
        x0, x1, y0, y1 = self.raum
        best = UNENDLICH
        for grenze, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9:
                t = (grenze - start) / d
                if t > 0:
                    best = min(best, t)
        for (ax, ay), (bx, by) in self.waende:
            ex_, ey = bx - ax, by - ay
            nenner = dx * ey - dy * ex_
            if abs(nenner) < 1e-12:
                continue
            t = ((ax - self.x) * ey - (ay - self.y) * ex_) / nenner
            u = ((ax - self.x) * dy - (ay - self.y) * dx) / nenner
            if t > 0 and 0.0 <= u <= 1.0:
                best = min(best, t)
        for cx, cy, r in self.kreise:
            mx, my = self.x - cx, self.y - cy
            b = mx * dx + my * dy
            c = mx * mx + my * my - r * r
            disk = b * b - c
            if disk >= 0:
                t = -b - math.sqrt(disk)
                if t > 0:
                    best = min(best, t)
        return best

    def scan(self, zeit):
        winkel, entf = [], []
        boden = random.uniform(0.25, 1.5) if self.gekippt else None
        for i in range(360):
            r = self.strahl(self.theta + math.radians(i))
            if boden is not None and (i < 90 or i > 270):
                r = min(r, boden)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                winkel.append(float(i if i <= 180 else i - 360))
                entf.append(r)
        s = Scan(winkel, entf)
        s.zeit = zeit
        return s

    def bewegen(self, befehl, dt):
        if befehl is None or self.gekippt:
            self.v_ist = (0.0, 0.0, 0.0)
            return
        vx, vy, om = befehl.vx, befehl.vy, befehl.omega
        self.v_ist = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def abstand(self, ohne_kreise=()):
        x0, x1, y0, y1 = self.raum
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
        for (ax, ay), (bx, by) in self.waende:
            ex_, ey = bx - ax, by - ay
            u = max(0.0, min(1.0, ((self.x - ax) * ex_ + (self.y - ay) * ey) / (ex_ * ex_ + ey * ey)))
            d = min(d, math.hypot(self.x - ax - u * ex_, self.y - ay - u * ey))
        for i, (cx, cy, r) in enumerate(self.kreise):
            if i not in ohne_kreise:
                d = min(d, math.hypot(self.x - cx, self.y - cy) - r)
        return d


def kasten(x0, x1, y0, y1):
    """Vier Wandstücke um ein Rechteck (Stuhl, Kiste)."""
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


class OdometrieAttrappe:
    def __init__(self, welt):
        self.welt = welt
        misst_tatsaechlich = True

    misst_tatsaechlich = True

    def geschwindigkeit(self):
        return self.welt.v_ist


# --- richtung_waehlen ----------------------------------------------------------


def wand_punkte(a, b, schritt=0.02):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / schritt))
    return np.array([(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)])


leer = np.zeros((0, 2))
breite = config.EXPLORE_HALF_WIDTH_M
w = aw.richtung_waehlen(leer, 0.0, 1.0, breite)
pruefe("Richtung: freier Weg bleibt geradeaus", w is not None and w[0] == 0.0, w)

stuhl = wand_punkte((0.8, -0.15), (0.8, 0.25))
w = aw.richtung_waehlen(stuhl, 0.0, 1.2, breite)
pruefe("Richtung: Stuhl vorn -> zur Seite", w is not None and abs(w[0]) >= 15, w)
pruefe("Richtung: weicht zur freieren Seite (rechts) aus", w is not None and w[0] < 0, w)
if w is not None:
    pruefe("Richtung: gewählter Weg ist wirklich frei",
           frei_in_richtung(stuhl, w[0], breite + config.AUSWEICH_RAND_M) >= 1.2, w)

# Tür in 1 m, Lücke y -0.1..0.6: Mitte bei ~14°. Die Pfosten müssen außerhalb
# des Fahrschlauchs bleiben.
tuer = np.vstack([wand_punkte((1.0, -2.0), (1.0, -0.1)), wand_punkte((1.0, 0.6), (1.0, 2.0))])
w = aw.richtung_waehlen(tuer, 0.0, 2.0, breite)
pruefe("Richtung: Tür gefunden", w is not None and 5 <= w[0] <= 22, w)
if w is not None:
    a = math.radians(w[0])
    quer = [abs(-px * math.sin(a) + py * math.cos(a)) for px, py in ((1.0, -0.1), (1.0, 0.6))]
    pruefe("Richtung: beide Pfosten außerhalb des Fahrschlauchs", min(quer) > breite, [round(q, 2) for q in quer])

# Schräg an einer endlichen Wand vorbei ist 1 m durchaus frei -- also 2 m verlangen.
zu = np.vstack([wand_punkte((0.5, -2.0), (0.5, 2.0))])
pruefe("Richtung: Wand quer davor -> None", aw.richtung_waehlen(zu, 0.0, 2.0, breite) is None)

# Mittig vor einem symmetrischen Hindernis: die zuletzt gewählte Seite gewinnt.
mitte = wand_punkte((0.8, -0.2), (0.8, 0.2))
links = aw.richtung_waehlen(mitte, 0.0, 1.2, breite, letzte_seite=1.0)
rechts = aw.richtung_waehlen(mitte, 0.0, 1.2, breite, letzte_seite=-1.0)
pruefe("Richtung: Seite wird gehalten (kein Pendeln)",
       links is not None and rechts is not None and links[0] > 0 > rechts[0], (links, rechts))


# --- Folgen in der gedachten Welt ---------------------------------------------

FOV = config.CAMERA_FOV_DEG
SV = 4 / 3
ROT = [0.2 / 127] * 128
ROT[3] = 0.8


def person(winkel, zeit, abstand):
    """Person am Kamerawinkel ``winkel``; Boxbreite passend zum Abstand."""
    mx = 0.5 - winkel / FOV
    sx = math.degrees(math.atan2(0.22, abstand)) / FOV
    sy = 0.3
    e = sx * SV

    def bild(u, v):
        return [mx + u * sx, sy + v * e, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = bild(0.0, -0.7)
    kp[5], kp[6] = bild(0.5, 0.0), bild(-0.5, 0.0)
    kp[9], kp[10] = bild(0.6, 1.4), bild(-0.6, 1.4)
    kp[11], kp[12] = bild(0.35, 1.8), bild(-0.35, 1.8)
    return {"box": [mx - sx, sy - 0.1, mx + sx, sy + 0.6], "kp": kp, "score": 0.9, "winkel": winkel,
            "seitenverhaeltnis": SV, "merkmal": ROT, "zeit": zeit}


class Watchdog:
    def submit(self, cmd, quelle="hand"):
        pass

    def stop(self):
        pass


class Lidar:
    def __init__(self):
        self.letzter = None

    def latest(self):
        return self.letzter


class Pose:
    verfuegbar = True
    fov_deg = FOV

    def __init__(self):
        self.liste = []

    def personen(self):
        return self.liste

    def aktivieren(self, an):
        pass


def beine(px, py):
    """Eine Person für den Lidar: zwei Beine."""
    return [(px, py + 0.1, 0.06), (px, py - 0.1, 0.06)]


def folgen(welt, px, py, dauer, hz=10.0, ereignis=None):
    """Folgen simulieren. Gibt (Interaktion, Verlauf, kleinster Abstand) zurück.

    Die Person (Beine an px/py) sind die letzten beiden Kreise der Welt --
    sie zählen beim Mindestabstand nicht als Hindernis.
    """
    welt.kreise += beine(px, py)
    personen_idx = (len(welt.kreise) - 2, len(welt.kreise) - 1)
    po, li = Pose(), Lidar()
    ia = vh.Interaktion(Watchdog(), li, po, odometry=OdometrieAttrappe(welt))
    ia._modus, ia._modus_seit, ia._gesehen = vh.BEREIT, 0.0, 0.0
    verlauf = []
    kleinster = welt.abstand(personen_idx)
    gestartet = False
    for i in range(int(dauer * hz)):
        t = i / hz
        if ereignis:
            ereignis(t, welt)
        dx, dy = px - welt.x, py - welt.y
        d = math.hypot(dx, dy)
        winkel = math.degrees(aw._winkel_normieren(math.atan2(dy, dx) - welt.theta))
        po.liste = [person(winkel, t, d)] if abs(winkel) < FOV / 2 else []
        li.letzter = welt.scan(t)
        if not gestartet:
            ia._wechseln(vh.FOLGEN, "Test", t)
            gestartet = True
        befehl = ia._schritt(t)
        verlauf.append((t, ia._modus, befehl, ia._text, d))
        welt.bewegen(befehl, 1.0 / hz)
        kleinster = min(kleinster, welt.abstand(personen_idx))
    return ia, verlauf, kleinster


# Stuhl (0,3 x 0,3 m) halb im Weg, Person 2,6 m geradeaus dahinter.
welt = Welt(waende=kasten(0.9, 1.2, -0.35, -0.05))
ia, verlauf, kleinster = folgen(welt, 2.6, 0.0, 30.0)
schraeg = [v for v in verlauf if v[2] is not None and abs(v[2].vy) > 0.02]
texte = {v[3].split(" (")[0] for v in verlauf}
pruefe("Folgen/Stuhl: schiebt schräg vorbei (vy)", len(schraeg) > 5, len(schraeg))
pruefe("Folgen/Stuhl: meldet 'umfährt Hindernis'", "umfährt Hindernis" in texte, sorted(texte))
pruefe("Folgen/Stuhl: berührt nichts", kleinster > ROBOTER_R, f"{kleinster:.2f} m")
pruefe("Folgen/Stuhl: kommt auf Abstand (~1 m)", abs(verlauf[-1][4] - config.FOLGEN_ABSTAND_M) < 0.3,
       f"{verlauf[-1][4]:.2f} m, x={welt.x:.2f}")
pruefe("Folgen/Stuhl: folgt noch", ia._modus == vh.FOLGEN, ia._modus)

# Tür nicht mittig: Wand bei x = 1,5, Lücke y -0.1..0.6; Person dahinter.
welt = Welt(waende=[((1.5, -3.0), (1.5, -0.1)), ((1.5, 0.6), (1.5, 3.0))])
ia, verlauf, kleinster = folgen(welt, 3.2, 0.1, 30.0)
pruefe("Folgen/Tür: ist hindurch", welt.x > 1.6, f"x={welt.x:.2f}, y={welt.y:.2f}")
pruefe("Folgen/Tür: berührt keinen Pfosten", kleinster > ROBOTER_R, f"{kleinster:.2f} m")
pruefe("Folgen/Tür: kommt auf Abstand (~1 m)", abs(verlauf[-1][4] - config.FOLGEN_ABSTAND_M) < 0.3,
       f"{verlauf[-1][4]:.2f} m")


# Hochheben: nach 6 s 2 s lang gekippt, dann abgesetzt.
def kippen(t, welt):
    welt.gekippt = 6.0 <= t < 8.0


welt = Welt()
ia, verlauf, _ = folgen(welt, 3.0, 0.0, 14.0, ereignis=kippen)
gekippt = [v for v in verlauf if 6.3 <= v[0] < 8.0]
pruefe("Folgen/Hochheben: fährt vorher", any(v[2] is not None and v[2].vx > 0 for v in verlauf if v[0] < 6.0))
pruefe("Folgen/Hochheben: steht, solange er gehalten wird",
       all(v[2] is None or (v[2].vx == 0 and v[2].vy == 0 and v[2].omega == 0) for v in gekippt),
       [v[3] for v in gekippt][:3])
pruefe("Folgen/Hochheben: Text 'hochgehoben'", any(v[3].startswith("hochgehoben") for v in gekippt))
pruefe("Folgen/Hochheben: danach BEREIT (fährt nicht von selbst weiter)", ia._modus == vh.BEREIT,
       f"{ia._modus}, {ia._text}")


# --- Heimfahrt in der gedachten Welt -------------------------------------------


class HeimOdometrie(OdometrieAttrappe):
    """Pose relativ zum Startpunkt (Weltursprung), Winkel in Grad."""

    def pose(self):
        return self.welt.x, self.welt.y, math.degrees(self.welt.theta)


class HeimLocalizer:
    def __init__(self, welt, uhr):
        self.welt, self.uhr = welt, uhr

    def scan(self):
        return self.welt.scan(self.uhr[0])

    def hindernis_voraus(self, richtung):
        return frei_in_richtung(self.scan().punkte(), richtung, config.EXPLORE_HALF_WIDTH_M)


class HeimWatchdog:
    def __init__(self):
        self.befehl = None
        self.befehle = []

    def submit(self, cmd, quelle="hand"):
        self.befehl = cmd
        self.befehle.append(cmd)

    def stop(self):
        self.befehl = None


def heimfahrt(welt, ereignis=None):
    """``Homing._fahren`` mit simulierter Uhr. Gibt (Grund, Befehle, kleinster Abstand) zurück."""
    uhr = [0.0]
    wd = HeimWatchdog()
    kleinster = [welt.abstand()]

    async def schlafen(dt):
        welt.bewegen(wd.befehl, dt)
        uhr[0] += dt
        kleinster[0] = min(kleinster[0], welt.abstand())
        if ereignis:
            ereignis(uhr[0], welt)

    alt_asyncio, alt_time = hm.asyncio, hm.time
    hm.asyncio = types.SimpleNamespace(sleep=schlafen, CancelledError=asyncio.CancelledError,
                                       Task=asyncio.Task, create_task=asyncio.create_task)
    hm.time = types.SimpleNamespace(monotonic=lambda: uhr[0])
    try:
        h = hm.Homing(wd, HeimOdometrie(welt), HeimLocalizer(welt, uhr))
        asyncio.run(h._fahren())
    finally:
        hm.asyncio, hm.time = alt_asyncio, alt_time
    return h.letzter_grund, wd.befehle, kleinster[0]


# Kiste auf der Linie zwischen Roboter (3, 0) und Startpunkt.
welt = Welt(waende=kasten(1.3, 1.7, -0.25, 0.15), x=3.0, y=0.0, theta=0.0)
grund, befehle, kleinster = heimfahrt(welt)
pruefe("Heimfahrt/Kiste: angekommen", grund is not None and grund.startswith("angekommen"), grund)
pruefe("Heimfahrt/Kiste: weicht seitlich aus", any(abs(b.vy) > 0.05 for b in befehle))
pruefe("Heimfahrt/Kiste: berührt nichts", kleinster > ROBOTER_R, f"{kleinster:.2f} m")


def heben(t, welt):
    welt.gekippt = t >= 2.0


welt = Welt(x=3.0, y=0.0)
grund, befehle, _ = heimfahrt(welt, ereignis=heben)
pruefe("Heimfahrt/Hochheben: bricht ab", grund is not None and grund.startswith("hochgehoben"), grund)

# Sackgasse: vorn und an den Seiten Wände, nur nach hinten frei.
welt = Welt(waende=[((2.6, -0.6), (2.6, 0.6)), ((2.6, 0.45), (3.8, 0.45)), ((2.6, -0.45), (3.8, -0.45))],
            x=3.0, y=0.0)
grund, befehle, kleinster = heimfahrt(welt)
pruefe("Heimfahrt/Sackgasse: hält mit Meldung an", grund is not None and "noch" in grund
       and ("Hindernis" in grund or "kein Weg" in grund), grund)
pruefe("Heimfahrt/Sackgasse: berührt nichts", kleinster > ROBOTER_R, f"{kleinster:.2f} m")

print()
print("alle Fälle bestehen" if not fehler else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
