"""Prüft das lokale Ausweichen des Erkunders (Schritt F2) -- ohne Roboter.

    python Marlon/autonomes_fahren/test_ausweichen.py

Erst die Bausteine aus ``navigation/ausweichen.py`` einzeln (Lücken, Bögen,
Kurzgedächtnis, Hochheben), dann der ganze Erkunder in einer gedachten Welt
mit Wänden, Türen und runden Hindernissen: Er soll durch Türen und Flure
finden, Hindernisse im Bogen umfahren statt davor zu wenden, Hochheben
erkennen und dabei keinen Fehlalarm geben.

Die Welt rechnet wie ``test_erkunder.py`` Lidar-Strahlen nach (7,5 Hz) und
schreibt die Pose aus den Fahrbefehlen fort; hinzu kommen Wandstücke.
"""

import math
import pathlib
import random
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.navigation import ausweichen as aw  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import hindernisse as hd  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0
TAKT = 1.0 / config.ODOMETRY_HZ
RUNDE = 1.0 / 7.5
#: Ungefährer Radius des Roboters um die Lidar-Mitte (Rumpf ~0,30 m breit).
ROBOTER_R = 0.15
UNENDLICH = float("inf")


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


# --- Gedachte Welt ---------------------------------------------------------------


class Welt:
    """Rechteckiger Raum mit Wandstücken, runden Hindernissen und einem Roboter."""

    def __init__(self, raum=(-3.0, 3.0, -3.0, 3.0), waende=(), kreise=(), x=0.0, y=0.0, theta=0.0):
        self.raum = raum
        self.waende = list(waende)  # ((x1, y1), (x2, y2))
        self.kreise = list(kreise)  # (x, y, r)
        self.x, self.y, self.theta = x, y, theta
        self.v_ist = (0.0, 0.0, 0.0)  # im Roboterrahmen, wie das Board sie meldet
        self.gekippt = False  # hochgehoben: Lidar-Ebene trifft Boden und Decke
        self.getragen = None  # (vx, vy, omega) in Weltkoordinaten, ohne Raddrehung

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
        # Wer ihn hält, wackelt: Die Neigung und damit der Abstand, in dem die
        # Lidar-Ebene den Boden trifft, ändert sich von Runde zu Runde.
        boden = random.uniform(0.25, 1.5) if self.gekippt else None
        for i in range(360):
            r = self.strahl(self.theta + math.radians(i))
            if boden is not None and (i < 90 or i > 270):
                # Schräg gehalten: Die vordere Hälfte trifft den Boden.
                r = min(r, boden)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                winkel.append(float(i if i <= 180 else i - 360))
                entf.append(r)
        s = Scan(winkel, entf)
        s.zeit = zeit
        return s

    def bewegen(self, befehl, dt):
        if self.getragen is not None:
            # Getragen: Er bewegt sich, die Räder melden aber nichts davon.
            vx, vy, om = self.getragen
            self.x += vx * dt
            self.y += vy * dt
            self.theta += om * dt
            self.v_ist = (0.0, 0.0, 0.0)
            return
        if befehl is None:
            self.v_ist = (0.0, 0.0, 0.0)
            return
        vx, vy, om = befehl.vx, befehl.vy, befehl.omega
        self.v_ist = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def abstand(self):
        """Kleinster Abstand der Lidar-Mitte zu Wand, Wandstück oder Hindernis."""
        x0, x1, y0, y1 = self.raum
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
        for (ax, ay), (bx, by) in self.waende:
            ex_, ey = bx - ax, by - ay
            u = max(0.0, min(1.0, ((self.x - ax) * ex_ + (self.y - ay) * ey) / (ex_ * ex_ + ey * ey)))
            d = min(d, math.hypot(self.x - ax - u * ex_, self.y - ay - u * ey))
        for cx, cy, r in self.kreise:
            d = min(d, math.hypot(self.x - cx, self.y - cy) - r)
        return d


class LidarAttrappe:
    def __init__(self):
        self.letzter = None

    def latest(self):
        return self.letzter


class OdometrieAttrappe:
    def __init__(self, welt, misst=True):
        self.welt = welt
        self.misst = misst

    @property
    def misst_tatsaechlich(self):
        return self.misst

    def geschwindigkeit(self):
        return self.welt.v_ist


class WatchdogAttrappe:
    # Die Fortschrittswache liest den zuletzt angenommenen Befehl mit.
    current_command = DriveCommand()

    def submit(self, befehl, quelle=""):
        self.current_command = befehl

    def stop(self):
        pass


def aufbauen(welt, misst=True):
    lidar = LidarAttrappe()
    e = ex.Erkunder(WatchdogAttrappe(), lidar, hd.Hinderniserkennung(), None,
                    odometry=OdometrieAttrappe(welt, misst))
    return e, lidar


def laufen(welt, e, lidar, dauer, t0=0.0, beobachter=None, ohne_wechsel=False):
    """Simuliert ``dauer`` Sekunden. Gibt (Zustände, kleinster Abstand, Ende) zurück."""
    jetzt = t0
    naechste_runde = t0
    zustaende = []
    kleinster = welt.abstand()
    if t0 == 0.0:
        lidar.letzter = welt.scan(0.0)
        e._beginnen(0.0)
    while jetzt < t0 + dauer:
        if ohne_wechsel:
            e._naechster_wechsel = UNENDLICH
        if jetzt >= naechste_runde:
            lidar.letzter = welt.scan(jetzt)
            naechste_runde += RUNDE
        befehl = e._schritt(jetzt)
        zustaende.append((jetzt, e._zustand, befehl))
        if beobachter:
            beobachter(jetzt, e, befehl)
        if befehl is None:
            return zustaende, kleinster, jetzt
        welt.bewegen(befehl, TAKT)
        if welt.getragen is None:
            kleinster = min(kleinster, welt.abstand())
        jetzt += TAKT
    return zustaende, kleinster, None


def wand_punkte(a, b, schritt=0.02):
    (ax, ay), (bx, by) = a, b
    n = max(2, int(math.hypot(bx - ax, by - ay) / schritt))
    return np.column_stack([np.linspace(ax, bx, n), np.linspace(ay, by, n)])


def kreis_punkte(cx, cy, r, n=60):
    w = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return np.column_stack([cx + r * np.cos(w), cy + r * np.sin(w)])


def scan_aus(winkel_entf, zeit):
    s = Scan([w for w, _ in winkel_entf], [r for _, r in winkel_entf])
    s.zeit = zeit
    return s


# --- 1. Lücken: die Mitte einer Tür, nicht der Rahmen ---------------------------
# Kasten um den Roboter, vorn in 1,0 m eine 0,8 m breite Tür bei y = 0,5;
# die Wände sind näher als die Schwelle, frei ist es nur durch die Tür.
kasten = np.vstack([
    wand_punkte((1.0, -1.0), (1.0, 0.1)),
    wand_punkte((1.0, 0.9), (1.0, 1.0)),
    wand_punkte((-1.0, -1.0), (-1.0, 1.0)),
    wand_punkte((-1.0, 1.0), (1.0, 1.0)),
    wand_punkte((-1.0, -1.0), (1.0, -1.0)),
])
breite = config.EXPLORE_HALF_WIDTH_M
schwelle = config.EXPLORE_GO_M + 0.5
gute = aw.luecken(kasten, breite, schwelle, config.OBSTACLE_RANGE_M)
tuer = math.degrees(math.atan2(0.5, 1.0))
pruefe("Lücken: Tür gefunden", any(abs(r - tuer) <= 4 for r, _ in gute),
       f"Tür bei {tuer:.0f}°, gefunden {[r for r, _ in gute]}")
# Ecken sind weiter weg als die Wände und dürfen frei sein; gerade auf eine
# Wand zu oder knapp am Türrahmen vorbei darf keine Richtung zeigen.
pruefe("Lücken: nicht auf die Wand zu",
       not any(min(abs(r - w) for w in (0, 90, -90, 180, -180)) < 15 for r, _ in gute), str(gute))
# Durch die Tür: Beide Rahmenpfosten liegen quer zur Richtung außerhalb des
# Fahrschlauchs (der hat schon Rand zum Roboter) -- die Mitte des freien Bereichs muss
# nicht genau die geometrische Türmitte sein, aber sicher hindurchführen.
def quer_abstand(r, punkt):
    w = math.radians(r)
    return abs(-math.sin(w) * punkt[0] + math.cos(w) * punkt[1])
tuer_richtungen = [r for r, _ in gute if abs(r - tuer) < 20]
pruefe("Lücken: nicht knapp am Türrahmen",
       tuer_richtungen and all(min(quer_abstand(r, (1.0, 0.1)), quer_abstand(r, (1.0, 0.9))) >= breite
                               for r in tuer_richtungen), str(gute))
# Probe: Jede gewählte Richtung ist im Fahrschlauch wirklich frei.
frei = aw.frei_je_richtung(kasten, [r for r, _ in gute], breite)
pruefe("Lücken: gewählte Richtungen frei", bool(np.all(frei >= schwelle)), str(np.round(frei, 2)))
pruefe("Lücken: alles frei -> Richtungen rundherum",
       len(aw.luecken(np.zeros((0, 2)), breite, schwelle, config.OBSTACLE_RANGE_M)) >= 30)
eng = np.vstack([kreis_punkte(0.0, 0.0, 0.5, 200)])
pruefe("Lücken: rundum zu -> keine", aw.luecken(eng, breite, schwelle, config.OBSTACLE_RANGE_M) == [])

# --- 2. Bögen -------------------------------------------------------------------
leer = np.zeros((0, 2))
wahl = aw.bogen_planen(leer, config.EXPLORE_SPEED, 0.0, 0.0)
pruefe("Bogen: frei -> geradeaus mit vollem Tempo",
       wahl is not None and wahl[0][1] == 0.0 and wahl[0][2] == 0.0
       and abs(wahl[0][0] - config.EXPLORE_SPEED) < 1e-9, str(wahl))
wahl = aw.bogen_planen(leer, config.EXPLORE_SPEED, 0.6, 0.0)
pruefe("Bogen: Kurs liegt links -> lenkt links", wahl is not None and wahl[0][2] > 0, str(wahl))
wahl = aw.bogen_planen(kreis_punkte(0.7, 0.05, 0.15), config.EXPLORE_SPEED * 0.5, 0.0, 0.0)
pruefe("Bogen: Hindernis voraus -> fährt vorbei statt drauf",
       wahl is not None and wahl[0][0] > 0 and (wahl[0][2] != 0.0 or wahl[0][1] != 0.0), str(wahl))
pruefe("Bogen: weicht zur freien Seite aus (Hindernis leicht links -> rechts)",
       wahl is not None and (wahl[0][2] < 0 or wahl[0][1] < 0), str(wahl))
zelle = np.vstack([wand_punkte((0.22, -0.3), (0.22, 0.3)),
                   wand_punkte((-0.3, 0.2), (0.22, 0.2)),
                   wand_punkte((-0.3, -0.2), (0.22, -0.2))])
pruefe("Bogen: Sackgasse direkt vorn -> keiner (wenden)",
       aw.bogen_planen(zelle, config.EXPLORE_SPEED * 0.35, 0.0, 0.0) is None)

# --- 3. Kurzgedächtnis ----------------------------------------------------------


class Fahrt:
    """Odometrie, die eine feste Geschwindigkeit meldet."""

    def __init__(self, v=(0.0, 0.0, 0.0), misst=True):
        self.v = v
        self.misst_tatsaechlich = misst

    def geschwindigkeit(self):
        return self.v


fahrt = Fahrt((0.2, 0.0, 0.0))
g = aw.Kurzgedaechtnis(fahrt)
g.fortschreiben(0.05)
# Kiste vorn in 0,40 m, dann fährt er 0,30 m näher heran: Bei 0,10 m liegt
# sie unter LIDAR_MIN_M und fehlt in der Runde -- das Gedächtnis hat sie noch.
g.punkte(scan_aus([(0.0, 0.40), (90.0, 2.0), (-90.0, 2.0)], 0.0))
for _ in range(30):
    g.fortschreiben(0.05)
p = g.punkte(scan_aus([(90.0, 2.0), (-90.0, 2.0)], 1.5))
nah = p[np.hypot(p[:, 0], p[:, 1]) < config.LIDAR_MIN_M]
pruefe("Gedächtnis: Kiste im toten Winkel bleibt", len(nah) == 1 and abs(nah[0, 0] - 0.10) < 0.02, str(p))
pruefe("Gedächtnis: Kurs gemessen", g.kurs is not None and abs(g.kurs) < 1e-9, str(g.kurs))

g = aw.Kurzgedaechtnis(Fahrt((0.0, 0.0, 0.0)))
g.fortschreiben(0.05)
g.punkte(scan_aus([(0.0, 0.60), (90.0, 2.0)], 0.0))
g.fortschreiben(0.05)
p = g.punkte(scan_aus([(0.0, 2.50), (90.0, 2.0)], 0.2))
pruefe("Gedächtnis: weggegangenes Ding wird ausgeräumt",
       not np.any(np.hypot(p[:, 0] - 0.6, p[:, 1]) < 0.05), str(p))

g = aw.Kurzgedaechtnis(Fahrt((0.0, 0.0, 0.0)))
g.fortschreiben(0.05)
g.punkte(scan_aus([(0.0, 0.60)], 0.0))
for _ in range(int(config.AUSWEICH_GEDAECHTNIS_S / 0.05) + 5):
    g.fortschreiben(0.05)
p = g.punkte(scan_aus([(90.0, 2.0)], config.AUSWEICH_GEDAECHTNIS_S + 0.5))
pruefe("Gedächtnis: vergisst nach AUSWEICH_GEDAECHTNIS_S", len(p) == 1, str(p))

g = aw.Kurzgedaechtnis(Fahrt(misst=False))
g.fortschreiben(0.05)
g.punkte(scan_aus([(0.0, 0.60)], 0.0))
p = g.punkte(scan_aus([(90.0, 2.0)], 0.2))
pruefe("Gedächtnis: ohne Messung nur die aktuelle Runde", len(p) == 1 and g.kurs is None, str(p))

# --- 4. Hochheben (Bausteine) ---------------------------------------------------
random.seed(3)
welt = Welt(raum=(-2.0, 3.0, -1.5, 2.0), kreise=[(1.0, 0.8, 0.3)])
h = aw.HochhebeErkennung()
alarm = False
for i in range(60):
    welt.theta = 0.08 * i  # dreht sich auf der Stelle, Lage gemessen
    alarm |= h.pruefen(welt.scan(i * RUNDE), (0.0, 0.0, welt.theta))
pruefe("Hochheben: Drehen mit Messung kein Alarm", not alarm, f"Anteil zuletzt {h.anteil:.2f}")
h = aw.HochhebeErkennung()
alarm = False
for i in range(60):
    welt.theta = 0.08 * i
    alarm |= h.pruefen(welt.scan(i * RUNDE), None)
pruefe("Hochheben: Drehen ohne Messung kein Alarm", not alarm, f"Anteil zuletzt {h.anteil:.2f}")
h = aw.HochhebeErkennung()
h.pruefen(welt.scan(9.9), None)  # eine ruhige Runde vorher
welt.gekippt = True
erkannt = [h.pruefen(welt.scan(10 + i * RUNDE), None) for i in range(4)]
pruefe("Hochheben: gekippt erkannt", erkannt[-1], str(erkannt))
welt.gekippt = False
ruhig = [h.pruefen(welt.scan(11 + i * RUNDE), None) for i in range(int(config.AUSWEICH_HOCH_RUHE_S / RUNDE) + 3)]
pruefe("Hochheben: wieder ruhig -> vorbei", ruhig[0] and not ruhig[-1], f"{sum(ruhig)} Runden noch hoch")

# --- 5. Erkunder: Hindernis im Bogen umfahren, ohne zu wenden --------------------
for misst in (True, False):
    name = "mit Drehrate" if misst else "ohne Drehrate"
    random.seed(5)
    welt = Welt(raum=(-1.0, 6.0, -2.0, 2.0), kreise=[(1.4, 0.05, 0.20)])
    e, lidar = aufbauen(welt, misst)
    vorbei = [None]

    def beobachter(jetzt, e, befehl):
        if vorbei[0] is None and welt.x > 2.2:
            vorbei[0] = (jetzt, welt.theta)

    z, kleinster, ende = laufen(welt, e, lidar, 20.0, beobachter=beobachter, ohne_wechsel=True)
    zust = {s for t, s, _ in z if vorbei[0] is None or t <= vorbei[0][0]}
    pruefe(f"Umfahren ({name}): kommt vorbei", vorbei[0] is not None, f"x={welt.x:.2f}")
    pruefe(f"Umfahren ({name}): ohne Wenden", ex.DREHT not in zust and ex.BEFREIT not in zust, str(zust))
    pruefe(f"Umfahren ({name}): keine Berührung", kleinster > ROBOTER_R, f"kleinster Abstand {kleinster:.2f} m")
    if misst:
        # Mit Kurs lenkt er nach dem Hindernis zurück auf die alte Richtung.
        pruefe("Umfahren (mit Drehrate): danach wieder auf Kurs",
               abs(math.degrees(welt.theta)) < 15, f"Blick {math.degrees(welt.theta):.0f}° bei x={welt.x:.2f}")

# --- 6. Erkunder: Flur ----------------------------------------------------------
random.seed(6)
welt = Welt(raum=(-1.0, 7.0, -0.38, 0.38), x=0.0, y=0.08, theta=math.radians(8))
e, lidar = aufbauen(welt)
z, kleinster, ende = laufen(welt, e, lidar, 40.0, ohne_wechsel=True,
                            beobachter=lambda j, e, b: None)
zust = {s for _, s, _ in z}
weit = max(0.0, welt.x)
pruefe("Flur: fährt durch", ende is None and welt.x > 5.0 or ex.DREHT in zust, f"x={welt.x:.2f}, {zust}")
pruefe("Flur: keine Berührung", kleinster > ROBOTER_R, f"kleinster Abstand {kleinster:.2f} m")

# --- 7. Erkunder: fährt mittig durch eine Tür -------------------------------------
# Zwei Räume, dazwischen eine Wand bei x = 0 mit 0,7 m breiter Tür. Er steht
# schräg und versetzt davor; durch muss er aus eigener Kraft finden.
for nr, (y0, th0) in enumerate(((0.25, -0.15), (-0.2, 0.2), (0.0, 0.0))):
    random.seed(70 + nr)
    welt = Welt(raum=(-2.5, 2.5, -1.5, 1.5),
                waende=[((0.0, -1.5), (0.0, -0.35)), ((0.0, 0.35), (0.0, 1.5))],
                x=-1.2, y=y0, theta=th0)
    e, lidar = aufbauen(welt)
    in_tuer = []

    def beobachter(jetzt, e, befehl):
        if abs(welt.x) < 0.05:
            in_tuer.append(welt.y)

    z, kleinster, ende = laufen(welt, e, lidar, 15.0, beobachter=beobachter, ohne_wechsel=True)
    zust = {s for _, s, _ in z}
    pruefe(f"Tür {nr}: fährt hindurch", welt.x > 0.5, f"x={welt.x:.2f}, {zust}")
    pruefe(f"Tür {nr}: keine Berührung", kleinster > ROBOTER_R, f"kleinster Abstand {kleinster:.2f} m")
    if in_tuer:
        pruefe(f"Tür {nr}: mittig", max(abs(y) for y in in_tuer) < 0.35 - ROBOTER_R,
               f"Versatz in der Tür bis {max(abs(y) for y in in_tuer):.2f} m")

# Ohne Vorgabe, aus der Ecke: Er soll irgendwann hindurchfinden, ohne
# anzustoßen und ohne aufzugeben (Zufall; mit festem Seed wiederholbar).
durch = 0
for seed in range(4):
    random.seed(170 + seed)
    welt = Welt(raum=(-2.5, 2.5, -1.5, 1.5),
                waende=[((0.0, -1.5), (0.0, -0.35)), ((0.0, 0.35), (0.0, 1.5))],
                x=-1.8, y=random.uniform(-0.8, 0.8), theta=random.uniform(-3.0, 3.0))
    e, lidar = aufbauen(welt)
    drueben = [False]

    def beobachter(jetzt, e, befehl):
        drueben[0] |= welt.x > 0.4

    z, kleinster, ende = laufen(welt, e, lidar, 120.0, beobachter=beobachter)
    durch += drueben[0]
    pruefe(f"Tür frei (Lauf {seed}): keine Berührung, bleibt aktiv",
           kleinster > ROBOTER_R and ende is None, f"kleinster Abstand {kleinster:.2f} m, {e.letzter_grund}")
pruefe("Tür frei: findet mindestens einmal hindurch", durch >= 1, f"{durch} von 4")

# --- 8. Erkunder: hochgehoben und woanders abgesetzt -------------------------------
random.seed(8)
welt = Welt(raum=(-2.0, 2.0, -2.0, 2.0), kreise=[(1.0, 0.8, 0.3)])
e, lidar = aufbauen(welt)
log = []


def tragen(jetzt, e, befehl):
    log.append((jetzt, e._zustand, befehl))
    if abs(jetzt - 5.0) < TAKT / 2:
        welt.gekippt = True
        welt.getragen = (0.3, 0.2, 0.8)
    if abs(jetzt - 8.0) < TAKT / 2:
        # Im Nachbarraum abgesetzt.
        welt.gekippt = False
        welt.getragen = None
        welt.raum = (-1.0, 4.0, -1.0, 1.2)
        welt.kreise = [(2.5, -0.3, 0.25)]
        welt.x, welt.y, welt.theta = 1.0, 0.0, 2.0


z, kleinster, ende = laufen(welt, e, lidar, 16.0, beobachter=tragen)
hoch = [t for t, s, _ in log if s == ex.HOCHGEHOBEN]
pruefe("Hochheben: erkannt", bool(hoch) and hoch[0] < 5.6, f"ab {hoch[0]:.2f} s" if hoch else "nie")
pruefe("Hochheben: steht, solange hochgehoben",
       all(b is not None and b.vx == 0 and b.vy == 0 and b.omega == 0 for t, s, b in log if s == ex.HOCHGEHOBEN))
nachher = [(t, s) for t, s, _ in log if t > 8.0]
weiter = next((t for t, s in nachher if s != ex.HOCHGEHOBEN), None)
pruefe("Hochheben: fährt nach dem Absetzen wieder",
       weiter is not None and weiter < 8.0 + config.AUSWEICH_HOCH_RUHE_S + 1.0 and ende is None,
       f"weiter ab {weiter}" if weiter else e.letzter_grund)
pruefe("Hochheben: kein Befreien/Festfahren durchs Tragen",
       ex.BEFREIT not in {s for t, s in nachher[:40]} and ende is None)

# --- 9. Kein Fehlalarm im Zimmer --------------------------------------------------
for misst in (True, False):
    name = "mit Drehrate" if misst else "ohne Drehrate"
    random.seed(9)
    welt = Welt(raum=(-2.0, 2.0, -1.5, 1.5),
                kreise=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
                x=-0.2)
    e, lidar = aufbauen(welt, misst)
    z, kleinster, ende = laufen(welt, e, lidar, 90.0)
    zust = {s for _, s, _ in z}
    pruefe(f"Zimmer ({name}): nie hochgehoben", ex.HOCHGEHOBEN not in zust, str(zust))
    pruefe(f"Zimmer ({name}): keine Berührung", kleinster > ROBOTER_R, f"kleinster Abstand {kleinster:.2f} m")
    pruefe(f"Zimmer ({name}): fährt die ganze Zeit", ende is None, e.letzter_grund)

print()
print("alle Fälle bestehen" if fehler == 0 else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
