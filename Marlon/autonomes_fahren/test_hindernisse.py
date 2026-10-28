"""Prüft die Hindernis-Benennung (robot/src/rosmaster_assist/navigation/hindernisse.py)
an einem gedachten Raum -- ohne Roboter, ohne Abhängigkeiten.

    python Marlon/autonomes_fahren/test_hindernisse.py

Der Scan wird wie vom RPLIDAR A1 erzeugt: 1° Auflösung, Strahlen treffen
das nächste Objekt. Kreise sind Beine, Strecken sind Wände.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.navigation import hindernisse as h  # noqa: E402


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


def scan(kreise=(), strecken=(), schritt=1.0):
    winkel, entf = [], []
    w = 0.0
    while w < 360.0:
        a = math.radians(w)
        treffer = [strahl_kreis(a, *k) for k in kreise] + [strahl_strecke(a, *s) for s in strecken]
        treffer = [t for t in treffer if t is not None]
        winkel.append(w)
        entf.append(min(treffer) if treffer else 0.0)  # 0 = kein Echo
        w += schritt
    return winkel, entf


def arten(ergebnis):
    return sorted(x.art for x in ergebnis)


fehler = 0


def pruefe(name, bedingung, ergebnis):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, [(x.art, round(x.abstand, 2), round(x.winkel)) for x in ergebnis])
    fehler += not bedingung


# Wand quer vor dem Roboter
e = h.klassifizieren(*scan(strecken=[(1.0, -1.5, 1.0, 1.5)]))
pruefe("Wand vorn", arten(e) == [h.WAND], e)

# Stuhl: vier Beine (2,5 cm Radius) im Quadrat von 40 cm, 1 m links vorn
stuhl = [(1.0 + dx, 0.6 + dy, 0.0125) for dx in (0, 0.4) for dy in (0, 0.4)]
e = h.klassifizieren(*scan(kreise=stuhl, schritt=0.5))
pruefe("Stuhl", h.STUHL in arten(e), e)

# Tisch: zwei Beine 1,2 m auseinander, rechts vorn
tisch = [(1.2, -0.4, 0.02), (1.2, -1.6, 0.02)]
e = h.klassifizieren(*scan(kreise=tisch, schritt=0.5))
pruefe("Tisch", arten(e) == [h.TISCH], e)

# Mensch: zwei Beine (12 cm Durchmesser) 25 cm auseinander, 1,2 m vorn
mensch = [(1.2, 0.125, 0.06), (1.2, -0.125, 0.06)]
e = h.klassifizieren(*scan(kreise=mensch))
pruefe("Mensch (Lidar)", arten(e) == [h.MENSCH], e)

# Kamera sticht die Form: ein Objekt, das die Kamera als Person sieht
kiste = [(1.0, 0.0, 0.2)]
e = h.klassifizieren(*scan(kreise=kiste), personen_winkel=[(-10.0, 10.0)])
pruefe("Mensch (Kamera)", arten(e) == [h.MENSCH] and e[0].quelle == "Kamera+Lidar", e)

# Ein Objekt genau vorn liegt im Scan halb am Anfang (0°) und halb am
# Ende (359°) -- es muss trotzdem als eines erkannt werden.
e = h.klassifizieren(*scan(kreise=[(1.0, 0.0, 0.15)]))
pruefe("Objekt über 0°-Grenze ist eins", len(e) == 1, e)

# Raum mit allem: nichts geht verloren, nächstes zuerst
e = h.klassifizieren(*scan(
    kreise=stuhl + tisch + mensch,
    strecken=[(2.5, -3, 2.5, 3), (-2.5, -3, -2.5, 3)],
    schritt=0.5,
))
pruefe("Raum: sortiert", [x.abstand for x in e] == sorted(x.abstand for x in e), e)
pruefe("Raum: alles da", {h.WAND, h.STUHL, h.MENSCH} <= set(arten(e)), e)

print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
