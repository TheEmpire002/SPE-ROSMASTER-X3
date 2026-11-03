"""Prüft die Bremsweg-Prüfung für autonome Fahrt und den Folgen-Abstand -- ohne Roboter.

    python Marlon/autonomes_fahren/test_schutz.py

Zwei Teile:

* ``AutonomSchutz`` (safety/collision.py): erlaubtes Tempo nach Bremsweg,
  veralteter Scan, Hindernis vorn/seitlich, Einbindung im Watchdog nur für
  ``quelle="autonom"``;
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
from rosmaster_assist.safety import AutonomSchutz, SafetyWatchdog  # noqa: E402
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


# --- Teil 1: erlaubtes Tempo ---------------------------------------------------

schutz = AutonomSchutz(Lidar())
rand = config.AUTONOM_SCHUTZ_RAND_M

pruefe("Tempo: am Rand 0", schutz.erlaubtes_tempo(rand, 0.0) == 0.0)
pruefe("Tempo: innerhalb des Rands 0", schutz.erlaubtes_tempo(rand - 0.1, 0.0) == 0.0)
pruefe("Tempo: frei unendlich", math.isinf(schutz.erlaubtes_tempo(float("inf"), 0.0)))

for frei, alter in [(0.5, 0.0), (1.0, 0.13), (0.4, 0.3)]:
    v = schutz.erlaubtes_tempo(frei, alter)
    t = config.AUTONOM_SCHUTZ_LATENZ_S + alter
    weg = v * t + v * v / (2 * config.AUTONOM_SCHUTZ_BREMSEN)
    pruefe(f"Tempo: Bremsweg passt genau ({frei} m, Alter {alter} s)",
           abs(weg - (frei - rand)) < 1e-9, f"v={v:.3f} weg={weg:.3f}")

pruefe("Tempo: weiter frei -> schneller",
       schutz.erlaubtes_tempo(0.6, 0.1) < schutz.erlaubtes_tempo(1.0, 0.1))
pruefe("Tempo: älterer Scan -> langsamer",
       schutz.erlaubtes_tempo(0.6, 0.4) < schutz.erlaubtes_tempo(0.6, 0.0))

# Die Modi selbst dürfen nicht ausgebremst werden (siehe config.py): Befreien
# mit EXPLORE_BEFREIEN_V bis EXPLORE_BEFREIEN_STOP_M, Erkunden bis EXPLORE_STOP_M.
pruefe("Tempo: Befreien bei 0,30 m und altem Scan nicht gebremst",
       schutz.erlaubtes_tempo(0.30, 0.13) >= 0.08, round(schutz.erlaubtes_tempo(0.30, 0.13), 3))
pruefe("Tempo: Erkunden bei 0,45 m nicht unter 0,3 m/s",
       schutz.erlaubtes_tempo(0.45, 0.13) >= 0.3, round(schutz.erlaubtes_tempo(0.45, 0.13), 3))


# --- Teil 1: pruefen() ---------------------------------------------------------

li = Lidar()
schutz = AutonomSchutz(li)
jetzt = time.monotonic

li.s = None
c = schutz.pruefen(DriveCommand(0.2, 0.0, 0.5), jetzt())
pruefe("ohne Scan: nicht fahren, drehen erlaubt", c == DriveCommand(0.0, 0.0, 0.5), c)

li.s = wand_vorn(3.0)
li.s.zeit -= config.AUTONOM_SCHUTZ_SCAN_MAX_S + 0.2
c = schutz.pruefen(DriveCommand(0.2, 0.1, -0.3), jetzt())
pruefe("veralteter Scan: nicht fahren, drehen erlaubt", c == DriveCommand(0.0, 0.0, -0.3), c)
c = schutz.pruefen(DriveCommand(0.0, 0.0, 0.7), jetzt())
pruefe("veralteter Scan: reine Drehung unverändert", c == DriveCommand(0.0, 0.0, 0.7), c)
pruefe("Eingriffe gezählt", schutz.zustand()["eingriffe"] == 2, schutz.zustand())

li.s = scan(strecken=[(5.0, -5.0, 5.0, 5.0)])
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.2), jetzt())
pruefe("weit frei: unverändert", c == DriveCommand(0.3, 0.0, 0.2), c)

li.s = wand_vorn(0.5)
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.2), jetzt())
pruefe("Wand 0,5 m vorn: 0,3 m/s noch erlaubt", c == DriveCommand(0.3, 0.0, 0.2), c)

li.s = wand_vorn(0.3)
c = schutz.pruefen(DriveCommand(0.3, 0.0, 0.2), jetzt())
erwartet = schutz.erlaubtes_tempo(0.3, jetzt() - (li.s.zeit - li.s.dauer))
pruefe("Wand 0,3 m vorn: gebremst, nicht gestoppt", 0.0 < c.vx < 0.3, c)
pruefe("Wand 0,3 m vorn: Tempo nach Bremsweg", abs(c.vx - erwartet) < 0.01, f"{c.vx:.3f} ~ {erwartet:.3f}")
pruefe("Wand 0,3 m vorn: Drehung bleibt", c.omega == 0.2, c)

# Kurzes Brett 0,21 m vorn (innerhalb des Rands), seitlich ist frei.
li.s = scan(strecken=[(0.21, -0.3, 0.21, 0.3)])
c = schutz.pruefen(DriveCommand(0.2, 0.0, 0.0), jetzt())
pruefe("Brett 0,21 m vorn: steht", c.vx == 0.0 and c.vy == 0.0, c)
c = schutz.pruefen(DriveCommand(-0.1, 0.0, 0.0), jetzt())
pruefe("Brett 0,21 m vorn: rückwärts weg erlaubt", c == DriveCommand(-0.1, 0.0, 0.0), c)
c = schutz.pruefen(DriveCommand(0.0, 0.15, 0.0), jetzt())
pruefe("Brett 0,21 m vorn: seitlich weg erlaubt", c == DriveCommand(0.0, 0.15, 0.0), c)

li.s = scan(strecken=[(-2.0, 0.3, 2.0, 0.3)])  # Wand links in 0,30 m
c = schutz.pruefen(DriveCommand(0.0, 0.25, 0.0), jetzt())
pruefe("Wand links: seitlich nach links gebremst", 0.0 <= c.vy < 0.25, c)
c = schutz.pruefen(DriveCommand(0.25, 0.0, 0.0), jetzt())
pruefe("Wand links: vorwärts daran entlang frei", c == DriveCommand(0.25, 0.0, 0.0), c)

li.s = wand_vorn(0.3)
c = schutz.pruefen(DriveCommand(0.3, 0.1, 0.0), jetzt())
pruefe("schräg: Richtung bleibt", abs(c.vy / c.vx - 1 / 3) < 1e-9 and c.vx < 0.3, c)


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
li = Lidar(wand_vorn(0.21))
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="autonom")
pruefe("Watchdog ohne Schutz: autonom wie bisher", wd._ziel.vx == 0.2, wd._ziel)

wd.autonom_schutz = AutonomSchutz(li)
wd.submit(DriveCommand(0.2, 0.0, 0.3), quelle="autonom")
pruefe("Watchdog: autonom vor der Wand gestoppt, dreht", wd._ziel == DriveCommand(0.0, 0.0, 0.3), wd._ziel)
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="hand")
pruefe("Watchdog: Handsteuerung unberührt", wd._ziel.vx == 0.2, wd._ziel)

config.AUTONOM_SCHUTZ_AKTIV = False
wd.submit(DriveCommand(0.2, 0.0, 0.0), quelle="autonom")
pruefe("Watchdog: abgeschaltet -> autonom unberührt", wd._ziel.vx == 0.2, wd._ziel)
config.AUTONOM_SCHUTZ_AKTIV = True


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
