"""Prüft den Fahrschutz für autonome Fahrt und den Folgen-Abstand -- ohne Roboter.

    python Marlon/autonomes_fahren/test_schutz.py

Zwei Teile:

* ``Fahrschutz`` (safety/fahrschutz.py): Bremsen vor der Wand, veralteter
  Scan, Hindernis vorn/seitlich/neben dem Rad, Drehen neben der Wand,
  Einbindung im Watchdog nur für ``quelle="autonom"``. Bis 03.11. prüfte
  hier ``AutonomSchutz``, der ist im Fahrschutz aufgegangen;
* ``Interaktion._abstand_zu`` (interaktion/verhalten.py): Abstand zur Person
  aus ihrem Lidar-Cluster -- ein Möbelbein davor zählt nicht als Person.

Scans werden wie vom RPLIDAR A1 erzeugt (Strahlen treffen das nächste
Objekt); Strahlen ohne Echo fehlen, wie im Lidar-Dienst.
"""

import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.base import DriveBase  # noqa: E402
from rosmaster_assist.interaktion import verhalten as vh  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety import Fahrschutz, SafetyWatchdog  # noqa: E402
from rosmaster_assist.safety.fahrschutz import KEIN_LIDAR  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


# --- Künstliche Scans ----------------------------------------------------------


def strahl_kreis(a, cx, cy, r):
    dx, dy = math.cos(a), math.sin(a)
    b = dx * cx + dy * cy
    d = b * b - (cx * cx + cy * cy - r * r)
    if d < 0:
        return None
    t = b - math.sqrt(d)
    return t if t > 0 else None


def strahl_strecke(a, x1, y1, x2, y2):
    dx, dy = math.cos(a), math.sin(a)
    ex, ey = x2 - x1, y2 - y1
    nenner = dx * ey - dy * ex
    if abs(nenner) < 1e-12:
        return None
    t = (x1 * ey - y1 * ex) / nenner
    u = (x1 * dy - y1 * dx) / nenner
    return t if t > 0 and 0 <= u <= 1 else None


def scan(kreise=(), strecken=(), schritt=1.0, alter=0.0):
    winkel, entf = [], []
    w = 0.0
    while w < 360.0:
        a = math.radians(w)
        treffer = [strahl_kreis(a, *k) for k in kreise] + [strahl_strecke(a, *s) for s in strecken]
        treffer = [t for t in treffer if t is not None]
        if treffer:
            winkel.append(w)
            entf.append(min(treffer))
        w += schritt
    s = Scan(winkel, entf)
    s.zeit = time.monotonic() - alter
    return s


class Lidar:
    def __init__(self, s=None):
        self.s = s

    def latest(self):
        return self.s


def wand_vorn(x):
    return scan(strecken=[(x, -2.0, x, 2.0)])


# --- Teil 1: Fahrschutz pruefen() ----------------------------------------------
# Grundfläche: vorn ROBOTER_VORN_M (0,15), seitlich ROBOTER_HALBE_BREITE_M
# (0,125) ab Lidar-Mitte; die Kante hält FAHRSCHUTZ_RAND_M (0,04) Luft.

li = Lidar()
schutz = Fahrschutz(li)
STOP = DriveCommand(0.0, 0.0, 0.0)

li.s = None
p = schutz.pruefen(DriveCommand(0.2, 0.0, 0.5))
pruefe("ohne Scan: steht", p.befehl == STOP and p.grund == KEIN_LIDAR, p)

li.s = wand_vorn(3.0)
li.s.zeit -= config.FAHRSCHUTZ_SCAN_MAX_S + 0.2
p = schutz.pruefen(DriveCommand(0.2, 0.1, -0.3))
pruefe("veralteter Scan: steht", p.befehl == STOP and p.grund == KEIN_LIDAR, p)

li.s = scan(strecken=[(5.0, -5.0, 5.0, 5.0)])
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.2)).befehl
pruefe("weit frei: unverändert", c == DriveCommand(0.3, 0.0, 0.2), c)

li.s = wand_vorn(0.8)
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.0)).befehl
pruefe("Wand 0,8 m vorn: 0,3 m/s noch erlaubt", c == DriveCommand(0.3, 0.0, 0.0), c)

li.s = wand_vorn(0.3)
p = schutz.pruefen(DriveCommand(0.3, 0.0, 0.2))
c = p.befehl
pruefe("Wand 0,3 m vorn: gebremst, nicht gestoppt", 0.0 < c.vx < 0.3, p)
pruefe("Wand 0,3 m vorn: Bogen bleibt derselbe (alles gleich skaliert)",
       abs(c.omega / c.vx - 0.2 / 0.3) < 1e-9, c)

# Kurzes Brett 2 cm vor der Kante (innerhalb des Rands), seitlich ist frei.
li.s = scan(strecken=[(0.17, -0.3, 0.17, 0.3)])
c = schutz.pruefen(DriveCommand(0.2, 0.0, 0.0)).befehl
pruefe("Brett im Rand vorn: steht", c == STOP, c)
c = schutz.pruefen(DriveCommand(-0.1, 0.0, 0.0)).befehl
pruefe("Brett im Rand vorn: rückwärts weg erlaubt", c == DriveCommand(-0.1, 0.0, 0.0), c)
c = schutz.pruefen(DriveCommand(0.0, 0.15, 0.0)).befehl
pruefe("Brett im Rand vorn: seitlich daran entlang erlaubt", c == DriveCommand(0.0, 0.15, 0.0), c)

li.s = scan(strecken=[(-2.0, 0.3, 2.0, 0.3)])  # Wand links in 0,30 m
c = schutz.pruefen(DriveCommand(0.0, 0.25, 0.0)).befehl
pruefe("Wand links: seitlich nach links gebremst", 0.0 <= c.vy < 0.25, c)
c = schutz.pruefen(DriveCommand(0.25, 0.0, 0.0)).befehl
pruefe("Wand links: vorwärts daran entlang frei", c == DriveCommand(0.25, 0.0, 0.0), c)

li.s = wand_vorn(0.3)
c = schutz.pruefen(DriveCommand(0.3, 0.1, 0.0)).befehl
pruefe("schräg: Richtung bleibt", c.vx < 0.3 and abs(c.vy / c.vx - 1 / 3) < 1e-9, c)

# Drehen: Das konnte AutonomSchutz nicht -- er ließ jede Drehung durch.
li.s = scan(strecken=[(-2.0, 0.16, 2.0, 0.16)])  # Wand 3,5 cm neben der linken Seite
c = schutz.pruefen(DriveCommand(0.0, 0.0, 0.5)).befehl
pruefe("Wand dicht links: Drehen gesperrt (Ecke schwenkt aus)", c == STOP, c)
pruefe("Wand dicht links: drehen_frei sagt warum", schutz.drehen_frei(math.pi / 2) is not None,
       schutz.drehen_frei(math.pi / 2))
li.s = scan(strecken=[(1.0, -2.0, 1.0, 2.0), (-1.0, -2.0, -1.0, 2.0)])
c = schutz.pruefen(DriveCommand(0.0, 0.0, 0.5)).befehl
pruefe("viel Platz: Drehen unverändert", c == DriveCommand(0.0, 0.0, 0.5), c)
pruefe("viel Platz: drehen_frei", schutz.drehen_frei(2 * math.pi) is None)

# Tischbein vorn rechts neben dem Rad, 5 cm vor der Kante.
li.s = scan(kreise=[(0.22, -0.11, 0.02)], schritt=0.25)
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.0)).befehl
pruefe("Tischbein neben dem Vorderrad: gebremst", c.vx < 0.3, c)


# --- Teil 1: Einbindung im Watchdog -------------------------------------------


class Attrappe(DriveBase):
    def connect(self):
        pass

    def disconnect(self):
        pass

    def drive(self, cmd):
        pass

    def stop(self):
        pass

    @property
    def connected(self):
        return True

    def read_battery(self):
        return 12.0


wd = SafetyWatchdog(Attrappe())
li = Lidar(scan(strecken=[(0.17, -0.3, 0.17, 0.3)]))
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="autonom")
pruefe("Watchdog ohne Schutz: autonom wie bisher", wd._ziel.vx == 0.2, wd._ziel)

wd.fahrschutz = Fahrschutz(li)
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="autonom")
pruefe("Watchdog: autonom vor dem Brett gestoppt", wd._ziel == STOP, wd._ziel)
pruefe("Watchdog: Ergebnis für die Modi", wd.letzte_pruefung is not None
       and wd.letzte_pruefung.anteil == 0.0, wd.letzte_pruefung)
pruefe("Watchdog: Eingriff gezählt", wd.fahrschutz.zustand()["eingriffe"] == 1, wd.fahrschutz.zustand())
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="hand")
pruefe("Watchdog: Handsteuerung unberührt", wd._ziel.vx == 0.2, wd._ziel)

config.AUTONOM_SCHUTZ_AKTIV = False
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="autonom")
pruefe("Watchdog: abgeschaltet -> autonom unberührt", wd._ziel.vx == 0.2, wd._ziel)
pruefe("Watchdog: abgeschaltet -> kein altes Ergebnis", wd.letzte_pruefung is None)
config.AUTONOM_SCHUTZ_AKTIV = True

pruefe("Fahrschutz-Rand kleiner als Planer-Rand", config.FAHRSCHUTZ_RAND_M < config.AUSWEICH_RAND_M)


# --- Teil 2: Folgen-Abstand aus dem Lidar-Cluster ------------------------------


class Pose:
    verfuegbar = True
    fov_deg = config.CAMERA_FOV_DEG

    def personen(self):
        return []

    def aktivieren(self, an):
        pass


class Watchdog:
    def submit(self, cmd, quelle="hand"):
        pass

    def stop(self):
        pass


def interaktion():
    return vh.Interaktion(Watchdog(), Lidar(), Pose())


def box(mitte_grad, breite_grad):
    """Normierte Box (x0, y0, x1, y1) um einen Winkel (links positiv)."""
    fov = config.CAMERA_FOV_DEG
    x0 = 0.5 - (mitte_grad + breite_grad / 2) / fov
    x1 = 0.5 - (mitte_grad - breite_grad / 2) / fov
    return {"box": [x0, 0.1, x1, 0.9]}


def beine(x, y):
    """Zwei Menschenbeine (12 cm dick, 25 cm auseinander)."""
    return [(x, y + 0.125, 0.06), (x, y - 0.125, 0.06)]


# Person in 1,5 m geradeaus, davor ein dünnes Stuhlbein in 0,8 m.
s = scan(kreise=beine(1.5, 0.0) + [(0.8, 0.0, 0.0125)], schritt=0.5)
ia = interaktion()
d = ia._abstand_zu(box(0.0, 12.0), s, 10.0)
pruefe("Stuhlbein vor der Person: Abstand zur Person", d is not None and abs(d - 1.44) < 0.05,
       (round(d, 2), ia._abstand_quelle))
pruefe("Stuhlbein vor der Person: Quelle Mensch", ia._abstand_quelle == "Cluster: Mensch", ia._abstand_quelle)
pruefe("früher: Minimum wäre das Stuhlbein", s.min_abstand(0.0, 6.0) < 0.85, round(s.min_abstand(0.0, 6.0), 2))

# Zwei personenähnliche Objekte (z. B. Mantel, Sessel) im Bildbereich: eins
# in 1 m links vom Boxrand, eins in 2 m nah der Boxmitte.
s = scan(kreise=[(1.0, 0.25, 0.15), (2.0, -0.1, 0.15)], schritt=0.5)
b = box(0.0, 27.0)
ia = interaktion()
d = ia._abstand_zu(b, s, 10.0)
pruefe("zwei Objekte, kein guter Abstand: das nahe der Boxmitte", abs(d - 1.85) < 0.05,
       (round(d, 2), ia._abstand_quelle))
ia._abstand_gut, ia._abstand_gut_zeit = 1.0, 9.5
d = ia._abstand_zu(b, s, 10.0)
pruefe("zwei Objekte, zuletzt 1,0 m: das plausible", abs(d - 0.85) < 0.05, round(d, 2))
ia._abstand_gut, ia._abstand_gut_zeit = 2.0, 9.5
d = ia._abstand_zu(b, s, 10.0)
pruefe("zwei Objekte, zuletzt 2,0 m: das plausible", abs(d - 1.85) < 0.05, round(d, 2))
ia._abstand_gut, ia._abstand_gut_zeit = 1.0, 10.0 - config.FOLGEN_CLUSTER_GUT_S - 0.1
d = ia._abstand_zu(b, s, 10.0)
pruefe("zwei Objekte, guter Abstand veraltet: wieder nach Winkel", abs(d - 1.85) < 0.05, round(d, 2))

# Person vor der Wand: nur Wand im Bildbereich -> wie früher das Minimum.
s = wand_vorn(1.2)
ia = interaktion()
d = ia._abstand_zu(box(0.0, 12.0), s, 10.0)
pruefe("nur Wand: Rückfall auf Minimum", abs(d - 1.2) < 0.01 and ia._abstand_quelle == "Minimum",
       (round(d, 2), ia._abstand_quelle))

# Ohne Scan und ohne Ziel kein Abstand.
ia = interaktion()
pruefe("ohne Scan: kein Abstand", ia._abstand_zu(box(0.0, 12.0), None, 10.0) is None)
pruefe("ohne Ziel: kein Abstand", ia._abstand_zu(None, wand_vorn(1.0), 10.0) is None)

print()
print("alles bestanden" if fehler == 0 else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
