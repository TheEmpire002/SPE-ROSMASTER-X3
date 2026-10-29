"""Prüft Gestenerkennung, Pose-Auswertung und Verhalten -- ohne Roboter.

    python Marlon/interaktion/test_gesten.py

Getestet wird:

* jede Geste einzeln mit künstlichen Körperpunkten, dazu Fälle, die
  **nicht** auslösen dürfen (stillstehen, Hallo-Winken, zu kurz, verdeckt);
* der Ablauf der simulierten Person aus ``main.py --mock`` -- er muss alle
  vier Gesten in der richtigen Reihenfolge ergeben;
* das Auswerten der Hailo-Ausgaben (``dekodieren``) mit gebauten Tensoren;
* das Verhalten (Folgen, Stopp, Hindernis) mit Attrappen für Watchdog,
  Lidar und Pose.

Braucht nur numpy.
"""

import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

import numpy as np  # noqa: E402

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaktion import gesten as gs  # noqa: E402
from rosmaster_assist.interaktion import verhalten as vh  # noqa: E402
from rosmaster_assist.sensors import pose  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


# --- Künstliche Person ---------------------------------------------------------
#
# In Körpermaßen gedacht: Schulterbreite 1, Schultermitte bei (mx, sy).
# hand_l/hand_r als (u, v) relativ dazu (v nach unten), None = verdeckt.

SV = 4 / 3
BREITE = 0.12  # Schulterbreite als Bildanteil in x


def person(hand_l=(0.6, 1.4), hand_r=(-0.6, 1.4), mx=0.5, sy=0.3, schultern=True, rauschen=0.0):
    sx = BREITE  # in x-Bildkoordinaten
    sy_einheit = BREITE * SV  # dieselbe Länge in y-Bildkoordinaten

    def bild(u, v):
        r = random.uniform(-rauschen, rauschen)
        return [mx + (u + r) * sx, sy + (v + r) * sy_einheit, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = bild(0.0, -0.7)
    kp[5] = bild(0.5, 0.0) if schultern else [0, 0, 0.1]
    kp[6] = bild(-0.5, 0.0) if schultern else [0, 0, 0.1]
    kp[9] = bild(*hand_l) if hand_l else [0, 0, 0.1]
    kp[10] = bild(*hand_r) if hand_r else [0, 0, 0.1]
    kp[11] = bild(0.35, 1.8)
    kp[12] = bild(-0.35, 1.8)
    fov = config.CAMERA_FOV_DEG
    return {
        "box": [mx - sx, sy - 0.1, mx + sx, sy + 0.6],
        "kp": kp,
        "score": 0.9,
        "winkel": (0.5 - mx) * fov,
        "seitenverhaeltnis": SV,
    }


def abspielen(erzeuger, dauer, hz=10.0, start=0.0, erkennung=None):
    """Erzeuger(t) -> Person je Bild. Gibt [(t, geste)] zurück."""
    erkennung = erkennung or gs.Gestenerkennung()
    treffer = []
    for i in range(int(dauer * hz)):
        t = start + i / hz
        g = erkennung.update(erzeuger(t), t)
        if g:
            treffer.append((round(t, 1), g))
    return treffer


def nur(treffer):
    return [g for _, g in treffer]


random.seed(1)

# --- Einzelne Gesten -----------------------------------------------------------

t = abspielen(lambda t: person(hand_r=(-0.4, -1.4)), 2.0)
pruefe("Folgen: Arm über dem Kopf", nur(t) == [gs.FOLGEN], t)
pruefe("Folgen: erst nach der Haltezeit", t and t[0][0] >= 0.8 * config.GESTE_HALTEN_S, t)

t = abspielen(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.5)
pruefe("Stopp: beide Hände hoch", nur(t) == [gs.STOPP], t)
pruefe("Stopp: schneller als Folgen", t and t[0][0] < config.GESTE_HALTEN_S, t)


def winken(t):
    return person(hand_r=(-0.7, 0.5 + 0.4 * math.sin(2 * math.pi * 1.5 * t)))


t = abspielen(winken, 2.5)
pruefe("Herkommen: Hand winkt auf und ab", nur(t) == [gs.HERKOMMEN], t)


def wischen(t):
    return person(hand_r=(-(0.3 + 1.5 * ((t * 1.0) % 1.0)), 0.4))


t = abspielen(wischen, 2.0)
pruefe("Weggehen: Hand wischt nach außen", nur(t)[:1] == [gs.WEGGEHEN], t)

# Geht die Person beim Winken zur Seite, bleibt es Herkommen -- gemessen wird
# relativ zur Schultermitte.
t = abspielen(lambda t: person(
    hand_r=(-0.7, 0.5 + 0.4 * math.sin(2 * math.pi * 1.5 * t)), mx=0.3 + 0.1 * t), 2.5)
pruefe("Herkommen: auch beim Gehen", nur(t) == [gs.HERKOMMEN], t)

# Kleine Person (weit weg) -- gleiche Geste, andere Pixel
BREITE, alt = 0.04, BREITE
t = abspielen(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.5)
BREITE = alt
pruefe("Stopp: auch weit entfernt", nur(t) == [gs.STOPP], t)

# --- Was nicht auslösen darf ---------------------------------------------------

t = []
for seed in range(10):  # mehrere Zufallsfolgen, ein Glückstreffer reicht nicht
    random.seed(seed)
    t += abspielen(lambda t: person(rauschen=0.08), 6.0)
pruefe("Stillstehen mit Zittern: nichts", t == [], t)

t = abspielen(lambda t: person(hand_r=(-1.1 - 0.35 * math.sin(2 * math.pi * 2 * t), -0.3)), 4.0)
pruefe("Hallo-Winken (seitlich): nichts", t == [], t)

t = abspielen(lambda t: person(hand_r=(-0.4, -1.4)) if t < 0.5 else person(), 3.0)
pruefe("Arm nur kurz hoch: nichts", t == [], t)

t = abspielen(lambda t: person(hand_r=(-0.4, -1.4), schultern=False), 3.0)
pruefe("Schultern verdeckt: nichts", t == [], t)

t = abspielen(lambda t: person(hand_r=(-0.7, 0.5 + 0.1 * math.sin(2 * math.pi * 1.5 * t))), 3.0)
pruefe("Winken zu klein (Zittern): nichts", t == [], t)

t = abspielen(lambda t: person(hand_r=(-0.4, -1.4) if int(t * 10) % 2 else (-0.6, 1.4)), 3.0)
pruefe("Flackernde Erkennung: nichts", t == [], t)

# Personenwechsel: links hält eine den Arm hoch, rechts eine andere ohne
# Geste -- springt die Auswahl hin und her, darf nichts zusammenkommen.
t = abspielen(lambda t: person(hand_r=(-0.4, -1.4), mx=0.2) if int(t * 10) % 2 else person(mx=0.8), 3.0)
pruefe("Zwei Personen abwechselnd: nichts", t == [], t)

# --- Pause und Vorrang von Stopp ----------------------------------------------

e = gs.Gestenerkennung()
t = abspielen(lambda t: person(hand_r=(-0.4, -1.4)), 4.0, erkennung=e)
pruefe("Gehaltenes Folgen zählt nur einmal je Pause", nur(t).count(gs.FOLGEN) <= 2 and t[0][1] == gs.FOLGEN, t)

e = gs.Gestenerkennung()
abspielen(lambda t: person(hand_r=(-0.4, -1.4)), 1.2, erkennung=e)
t = abspielen(lambda t: person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), 1.0, start=1.2, erkennung=e)
pruefe("Stopp geht auch direkt nach Folgen", nur(t) == [gs.STOPP], t)

e = gs.Gestenerkennung()
abspielen(lambda t: person(hand_r=(-0.4, -1.4)), 1.2, erkennung=e)
t = abspielen(winken, 1.5, start=1.2, erkennung=e)
pruefe("Andere Geste in der Pause: nichts", t == [], t)

pruefe("Richtungswechsel zählen", gs.richtungswechsel([0, 1, 0, 1, 0], 0.5) == 3)
pruefe("Richtungswechsel: Zittern zählt nicht", gs.richtungswechsel([0, 0.1, 0, 0.1, 0], 0.5) == 0)

# --- Simulierte Person aus --mock ----------------------------------------------

dauer = sum(d for d, _ in pose._SIM_ABLAUF)
t = abspielen(lambda t: pose._simulierte_personen(t)[0], dauer)
reihenfolge = [g for i, g in enumerate(nur(t)) if i == 0 or nur(t)[i - 1] != g]
pruefe(
    "Mock-Person zeigt alle Gesten in Reihenfolge",
    reihenfolge == [gs.FOLGEN, gs.STOPP, gs.HERKOMMEN, gs.WEGGEHEN],
    t,
)

# --- dekodieren (Hailo-Ausgaben) -----------------------------------------------


def ausgaben(zellen):
    """Gebaute Modellausgaben: zellen = [(raster, gy, gx, score_logit, abst, kp_raw)]."""
    teile = []
    for raster in (80, 40, 20):
        box = np.zeros((raster, raster, 64), np.float32)
        score = np.full((raster, raster, 1), -10.0, np.float32)  # Logits
        kp = np.zeros((raster, raster, 51), np.float32)
        for r, gy, gx, s, abst, kp_raw in zellen:
            if r != raster:
                continue
            score[gy, gx, 0] = s
            for seite in range(4):  # spitze Verteilung bei "abst" Rasterschritten
                box[gy, gx, seite * 16 + abst] = 20.0
            kp[gy, gx] = kp_raw
        # Reihenfolge absichtlich durcheinander -- dekodieren sortiert nach Form.
        teile += [kp, box, score]
    return teile


kp_roh = np.zeros(51, np.float32)
kp_roh[0::3] = 0.25  # alle Punkte: (0.25*2 + gx) * schritt
kp_roh[2::3] = 5.0
erg = pose.dekodieren(np, ausgaben([(40, 10, 20, 4.0, 3, kp_roh)]), 640, 0.5)
pruefe("dekodieren: eine Person", len(erg) == 1, len(erg))
if erg:
    s, box, kp = erg[0]
    # Raster 40 bei 640 px -> Schritt 16; Zelle (20, 10) -> Mitte (328, 168);
    # 3 Schritte je Seite -> 48 px
    pruefe("dekodieren: Box", np.allclose(box, [280, 120, 376, 216], atol=0.5), box)
    pruefe("dekodieren: Punkt x", abs(kp[0, 0] - (0.5 + 20) * 16) < 0.5, kp[0])
    pruefe("dekodieren: Punkt y", abs(kp[0, 1] - 10 * 16) < 0.5, kp[0])
    pruefe("dekodieren: Sicherheit", 0.99 < kp[0, 2] <= 1.0 and 0.98 < s <= 1.0, (s, kp[0, 2]))

# Dieselbe Person in Nachbarzellen und zweitem Raster -> NMS lässt eine übrig;
# eine zweite Person weit weg bleibt.
erg = pose.dekodieren(np, ausgaben([
    (40, 10, 20, 4.0, 3, kp_roh), (40, 10, 21, 3.0, 3, kp_roh),
    (80, 21, 41, 2.0, 6, kp_roh), (20, 15, 3, 3.0, 2, kp_roh),
]), 640, 0.5)
pruefe("dekodieren: NMS fasst zusammen", len(erg) == 2, [round(e[0], 2) for e in erg])
erg = pose.dekodieren(np, ausgaben([(40, 10, 20, 0.0, 3, kp_roh)]), 640, 0.6)
pruefe("dekodieren: zu unsicher -> nichts", erg == [], len(erg))

# --- Verhalten mit Attrappen ---------------------------------------------------


class Watchdog:
    def __init__(self):
        self.befehle, self.stopps = [], 0

    def submit(self, cmd):
        self.befehle.append(cmd)

    def stop(self):
        self.stopps += 1


class Lidar:
    def __init__(self, objekte):
        self.objekte = objekte  # [(winkel_von, winkel_bis, abstand)]

    def latest(self):
        winkel = [float(w) for w in range(360)]
        r = [5.0] * 360
        for von, bis, d in self.objekte:
            for w in range(360):
                diff = (w - von) % 360
                if diff <= (bis - von) % 360:
                    r[w] = min(r[w], d)
        return Scan(winkel, r)


class Pose:
    verfuegbar = True

    def __init__(self):
        self.liste, self.zeit = [], 0.0

    def personen(self):
        return self.liste

    def aktivieren(self, an):
        pass


def interaktion(objekte, modus):
    wd, li, po = Watchdog(), Lidar(objekte), Pose()
    ia = vh.Interaktion(wd, li, po)
    ia._modus, ia._modus_seit, ia._gesehen = modus, 0.0, 0.0
    return ia, wd, li, po


def mit_zeit(p, z):
    return dict(p, zeit=z)


# Person genau vorn in 2 m (Beine im Lidar) -> vorwärts, kaum drehen
ia, wd, li, po = interaktion([(-5, 5, 2.0)], vh.FOLGEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: Person 2 m vorn -> vorwärts", b and b.vx > 0.1 and abs(b.omega) < 0.05, b)

# Zu nah (0.6 m) -> zurück
ia, wd, li, po = interaktion([(-5, 5, 0.6)], vh.FOLGEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: Person zu nah -> rückwärts", b and b.vx < 0, b)

# Im Sollabstand -> stehen
ia, wd, li, po = interaktion([(-5, 5, config.FOLGEN_ABSTAND_M)], vh.FOLGEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: im Sollabstand -> steht", b and b.vx == 0, b)

# Person weit links -> nur drehen (links = positiv)
ia, wd, li, po = interaktion([(30, 40, 2.0)], vh.FOLGEN)
po.liste = [mit_zeit(person(mx=0.0), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: Person links -> dreht links, fährt nicht", b and b.vx == 0 and b.omega > 0, b)

# Hindernis im Fahrschlauch vor der Person -> nicht hineinfahren
ia, wd, li, po = interaktion([(-5, 5, 2.0), (-8, 8, 0.45)], vh.FOLGEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: Hindernis davor -> kaum Fahrt", b and b.vx <= 0.06, b)

# Etwas ganz nah -> gar nicht bewegen
ia, wd, li, po = interaktion([(-5, 5, 2.0), (80, 90, 0.15)], vh.FOLGEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Folgen: etwas sehr nah -> steht", b and b.vx == 0 and b.omega == 0, b)

# Person weg -> erst warten, dann suchen
ia, wd, li, po = interaktion([], vh.FOLGEN)
ia._winkel = 10.0
b = ia._schritt(1.0)
pruefe("Folgen: Person kurz weg -> hält", b and b.vx == 0 and b.omega == 0 and ia._modus == vh.FOLGEN, b)
ia._schritt(1.0 + config.FOLGEN_VERLOREN_S + 0.1)
b = ia._schritt(1.2 + config.FOLGEN_VERLOREN_S)
pruefe("Folgen: Person länger weg -> sucht", ia._modus == vh.SUCHEN and b and b.omega > 0, (ia._modus, b))
ia._schritt(1.3 + config.FOLGEN_VERLOREN_S + config.FOLGEN_SUCHEN_S)
pruefe("Suchen erfolglos -> bereit und angehalten", ia._modus == vh.BEREIT and wd.stopps >= 1, ia._modus)

# Stopp-Geste beim Folgen -> sofort bereit, Watchdog angehalten, keine Befehle mehr
ia, wd, li, po = interaktion([(-5, 5, 2.0)], vh.FOLGEN)
for i in range(15):
    z = 0.1 * i
    po.liste = [mit_zeit(person(hand_l=(0.7, -0.6), hand_r=(-0.7, -0.6)), z)]
    b = ia._schritt(z)
pruefe("Stopp-Geste -> bereit", ia._modus == vh.BEREIT and wd.stopps >= 1, ia._modus)
pruefe("Bereit -> schickt keine Fahrbefehle", b is None, b)
pruefe("Stopp-Geste -> Meldung", any("Stopp" in m for m in ia.meldungen_abholen()))

# Folgen-Geste aus BEREIT
ia, wd, li, po = interaktion([(-5, 5, 2.0)], vh.BEREIT)
for i in range(15):
    z = 0.1 * i
    po.liste = [mit_zeit(person(hand_r=(-0.4, -1.4)), z)]
    ia._schritt(z)
pruefe("Folgen-Geste -> folgt", ia._modus == vh.FOLGEN, ia._modus)

# Herkommen: angekommen -> bereit
ia, wd, li, po = interaktion([(-5, 5, config.HERKOMMEN_ABSTAND_M)], vh.HERKOMMEN)
po.liste = [mit_zeit(person(), 0.1)]
ia._schritt(0.1)
pruefe("Herkommen: angekommen -> bereit", ia._modus == vh.BEREIT, ia._modus)

# Weggehen: hinten frei -> rückwärts; hinten Wand -> stehen
ia, wd, li, po = interaktion([(-5, 5, 0.8)], vh.WEGGEHEN)
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Weggehen: hinten frei -> rückwärts", b and b.vx < 0, b)
ia, wd, li, po = interaktion([(-5, 5, 0.8), (170, 190, 0.3)], vh.WEGGEHEN)
po.liste = [mit_zeit(person(), 0.1)]
ia._schritt(0.1)
pruefe("Weggehen: hinten Wand -> bereit", ia._modus == vh.BEREIT, ia._modus)
ia, wd, li, po = interaktion([(-5, 5, 0.8)], vh.WEGGEHEN)
po.liste = [mit_zeit(person(), 0.1)]
ia._schritt(0.1 + config.WEGGEHEN_STRECKE_M / config.WEGGEHEN_SPEED + 0.1)
pruefe("Weggehen: Strecke geschafft -> bereit", ia._modus == vh.BEREIT, ia._modus)

# Ohne Lidar-Daten -> steht
ia, wd, li, po = interaktion([(-5, 5, 2.0)], vh.FOLGEN)
li.latest = lambda: None
po.liste = [mit_zeit(person(), 0.1)]
b = ia._schritt(0.1)
pruefe("Ohne Lidar -> steht", b and b.vx == 0 and b.omega == 0, b)

print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
