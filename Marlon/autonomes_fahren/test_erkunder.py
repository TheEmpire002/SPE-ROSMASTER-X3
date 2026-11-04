"""Prüft den Erkunder in einer kleinen Simulation -- ohne Roboter.

    python Marlon/autonomes_fahren/test_erkunder.py

Der Test ruft ``Erkunder._schritt`` direkt mit einer gedachten Uhr auf. Eine
Welt aus Wänden und runden Hindernissen liefert Lidar-Runden (Strahlen
nachgerechnet, 7,5 Hz), die Pose wird aus den Fahrbefehlen fortgeschrieben.
Damit lassen sich Fälle nachstellen, die am Boden schwer zu wiederholen
sind: etwas rückt ganz nah heran, Räder drehen durch, er sitzt fest.

Hintergrund: Vor dem 03.11. blieb er bei Nah-Punkten für immer stehen,
drehte nur nach Zeit und gab nach sechs erfolglosen Drehungen auf.
"""

import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import hindernisse as hd  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0
TAKT = 1.0 / config.ODOMETRY_HZ
RUNDE = 1.0 / 7.5
#: Ungefährer Radius des Roboters um die Lidar-Mitte (Rumpf ~0,30 m breit).
ROBOTER_R = 0.15


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


class Welt:
    """Rechteckiger Raum mit runden Hindernissen und ein Roboter darin."""

    def __init__(self, raum=(-3.0, 3.0, -3.0, 3.0), kreise=(), x=0.0, y=0.0, theta=0.0):
        self.raum = raum
        self.kreise = list(kreise)  # (x, y, r)
        self.x, self.y, self.theta = x, y, theta
        self.blockiert = False  # Räder drehen durch / er hängt fest
        self.dreh_anteil = 1.0  # wie viel der befohlenen Drehung ankommt
        self.v_ist = (0.0, 0.0, 0.0)  # im Roboterrahmen

    def strahl(self, richtung):
        dx, dy = math.cos(richtung), math.sin(richtung)
        x0, x1, y0, y1 = self.raum
        best = float("inf")
        for grenze, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9:
                t = (grenze - start) / d
                if t > 0:
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
        for i in range(360):
            r = self.strahl(self.theta + math.radians(i))
            # Wie der echte Lidar: unter LIDAR_MIN_M kommt nichts an.
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                winkel.append(float(i if i <= 180 else i - 360))
                entf.append(r)
        s = Scan(winkel, entf)
        s.zeit = zeit
        return s

    def bewegen(self, befehl, dt):
        if befehl is None or self.blockiert:
            self.v_ist = (0.0, 0.0, 0.0)
            return
        vx, vy, om = befehl.vx, befehl.vy, befehl.omega * self.dreh_anteil
        self.v_ist = (vx, vy, om)
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.theta += om * dt

    def abstand(self):
        """Kleinster Abstand der Lidar-Mitte zu Wand oder Hindernis."""
        x0, x1, y0, y1 = self.raum
        d = min(self.x - x0, x1 - self.x, self.y - y0, y1 - self.y)
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


def aufbauen(welt, misst=True, mit_odometrie=True):
    lidar = LidarAttrappe()
    odo = OdometrieAttrappe(welt, misst) if mit_odometrie else None
    e = ex.Erkunder(WatchdogAttrappe(), lidar, hd.Hinderniserkennung(), None, odometry=odo)
    return e, lidar


def laufen(welt, e, lidar, dauer, t0=0.0, beobachter=None):
    """Simuliert ``dauer`` Sekunden. Gibt (Zustände, kleinster Abstand, Ende) zurück."""
    jetzt = t0
    naechste_runde = t0
    zustaende = []
    kleinster = welt.abstand()
    if t0 == 0.0:
        lidar.letzter = welt.scan(0.0)
        e._beginnen(0.0)
    while jetzt < t0 + dauer:
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
        kleinster = min(kleinster, welt.abstand())
        jetzt += TAKT
    return zustaende, kleinster, None


def namen(zustaende):
    return [z for _, z, _ in zustaende]


# --- 1. Etwas rückt ganz nah heran: erst warten, dann freischieben -------------
random.seed(1)
# Hindernis rechts neben dem Roboter, 0,18 m bis zur Lidar-Mitte.
welt = Welt(kreise=[(0.0, -0.48, 0.30)])
e, lidar = aufbauen(welt)
z, kleinster, ende = laufen(welt, e, lidar, 8.0)
n = namen(z)
erst_befreit = next((t for t, s, _ in z if s == ex.BEFREIT), None)
pruefe("Nah: wartet zuerst", n[1] == ex.WARTET, n[:3])
pruefe(
    "Nah: schiebt sich nach der Wartezeit frei",
    erst_befreit is not None and abs(erst_befreit - config.EXPLORE_NAH_WARTEN_S) < 0.2,
    erst_befreit,
)
befehle_frei = [b for _, s, b in z if s == ex.BEFREIT and b is not None]
pruefe("Nah: schiebt ohne Drehung", all(b.omega == 0.0 for b in befehle_frei))
pruefe("Nah: schiebt weg vom Hindernis (nach links)", befehle_frei and befehle_frei[0].vy > 0, befehle_frei[:1])
pruefe("Nah: steht nicht mehr dauerhaft", n[-1] != ex.WARTET and ende is None, n[-1])
pruefe("Nah: danach mit Abstand", welt.abstand() > config.EXPLORE_NAH_M, f"{welt.abstand():.2f} m")

# Ein Mensch, der nur kurz herantritt, löst kein Befreien aus.
random.seed(1)
welt = Welt(kreise=[(0.0, -0.48, 0.30)])
e, lidar = aufbauen(welt)
z, _, _ = laufen(welt, e, lidar, 1.5)
welt.kreise.clear()
z2, _, _ = laufen(welt, e, lidar, 2.0, t0=1.5)
pruefe("Nah kurz: kein Befreien", ex.BEFREIT not in namen(z + z2))
pruefe("Nah kurz: fährt danach weiter", z2[-1][1] == ex.FAEHRT, z2[-1][1])

# --- 2. Festgefahren: Board meldet Stillstand -----------------------------------
random.seed(2)
welt = Welt(raum=(-3.0, 2.0, -3.0, 3.0))  # Wand 2 m voraus
welt.blockiert = True
e, lidar = aufbauen(welt)
z, _, ende = laufen(welt, e, lidar, 3.0)
erst_befreit = next(((t, b) for t, s, b in z if s == ex.BEFREIT), None)
pruefe(
    "Fest (Board): erkannt nach EXPLORE_FEST_S",
    erst_befreit is not None and erst_befreit[0] < config.EXPLORE_FEST_S + 0.5,
    erst_befreit and round(erst_befreit[0], 2),
)
pruefe(
    "Fest (Board): befreit sich nicht nach vorn",
    erst_befreit is not None and abs(e._befrei_richtung) >= 90,
    e._befrei_richtung,
)
pruefe("Fest (Board): Erkennung vor dem Odometrie-Verwerfen (3 s)", config.EXPLORE_FEST_S < 3.0)

# Bleibt er blockiert, gibt er nach EXPLORE_BEFREIEN_MAX Versuchen auf.
z, _, ende = laufen(welt, e, lidar, 40.0, t0=3.0)
pruefe("Fest (Board): gibt irgendwann auf", ende is not None, e.letzter_grund)
pruefe("Fest (Board): Grund nennt festgefahren", "festgefahren" in (e.letzter_grund or ""), e.letzter_grund)
pruefe("Fest (Board): Zahl der Befreiungen", e._befreit_gesamt == config.EXPLORE_BEFREIEN_MAX, e._befreit_gesamt)

# --- 3. Festgefahren ohne Board-Messung: nur der Lidar merkt es ----------------
random.seed(3)
welt = Welt(raum=(-3.0, 2.0, -3.0, 3.0))
welt.blockiert = True
e, lidar = aufbauen(welt, mit_odometrie=False)
z, _, _ = laufen(welt, e, lidar, 3.0)
erst_befreit = next((t for t, s, _ in z if s == ex.BEFREIT), None)
pruefe(
    "Fest (Lidar): Abstand schrumpft nicht -> befreien",
    erst_befreit is not None and erst_befreit < config.EXPLORE_FEST_S + 0.5,
    erst_befreit,
)

# Fährt er wirklich, löst die Erkennung nicht aus (auch ohne Board).
random.seed(3)
welt = Welt(raum=(-3.0, 2.0, -3.0, 3.0), x=-2.5)
e, lidar = aufbauen(welt, mit_odometrie=False)
z, _, _ = laufen(welt, e, lidar, 6.0)
pruefe("Fest (Lidar): kein Fehlalarm bei freier Fahrt", ex.BEFREIT not in namen(z), namen(z)[-1])

# Zimmer ohne Zufallswechsel: Er fährt oft schräg auf Wände zu und lenkt
# daran entlang. Der Abstand nach vorn bleibt dabei fast gleich -- früher
# hielt die Erkennung das für Festfahren (4-5 Fehlalarme in 2 Minuten).
alt_min, alt_max = config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S
config.EXPLORE_WANDER_MIN_S = config.EXPLORE_WANDER_MAX_S = 1e9
for misst in (True, False):
    random.seed(3)
    welt = Welt(
        raum=(-2.5, 2.5, -2.0, 2.0),
        kreise=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
        x=-0.2,
    )
    welt.dreh_anteil = 0.8
    e, lidar = aufbauen(welt, misst=misst)
    laufen(welt, e, lidar, 120.0)
    pruefe(
        f"Fest: kein Fehlalarm an Wänden ({'mit' if misst else 'ohne'} Board-Messung)",
        e._befreit_gesamt == 0,
        f"{e._befreit_gesamt}x befreit, {e._ausweichen}x ausgewichen",
    )
config.EXPLORE_WANDER_MIN_S, config.EXPLORE_WANDER_MAX_S = alt_min, alt_max

# Großer Raum wie im MockLidar, Start in der Mitte: Die Wände in
# Fahrtrichtung sind weiter weg als OBSTACLE_RANGE_M, die seitlichen ändern
# sich nicht. Zählten nur Strahlen in Reichweite, meldete er nach 1,5 s
# Festfahren (so im Mock aufgefallen).
random.seed(3)
welt = Welt(raum=(-3.0, 5.0, -3.0, 3.0), kreise=[(2.0, 1.2, 0.35)])
e, lidar = aufbauen(welt, mit_odometrie=False)
laufen(welt, e, lidar, 4.0)
pruefe("Fest: kein Fehlalarm im großen Raum", e._befreit_gesamt == 0, f"{e._befreit_gesamt}x befreit")

# --- 4. Drehen nach gemessenem Winkel -----------------------------------------
def drehung_messen(misst, mit_odometrie=True, dreh_anteil=0.6, seed=4):
    """Wand knapp voraus -> er dreht. Gibt (Ziel, tatsächlich gedreht) zurück."""
    random.seed(seed)
    welt = Welt(raum=(-3.0, 0.35, -3.0, 3.0))
    welt.dreh_anteil = dreh_anteil
    e, lidar = aufbauen(welt, misst=misst, mit_odometrie=mit_odometrie)
    ergebnis = {}

    def beobachter(jetzt, e, befehl):
        if e._zustand == ex.DREHT and "ziel" not in ergebnis:
            ergebnis["ziel"] = e._dreh_richtung
            ergebnis["start"] = welt.theta
        if "ziel" in ergebnis and e._dreh_fertig is not None and "ende" not in ergebnis:
            ergebnis["ende"] = welt.theta
            ergebnis["fertig"] = e._dreh_fertig
        if "fertig" in ergebnis and befehl is not None and befehl.vx > 0 and "los" not in ergebnis:
            ergebnis["los"] = jetzt
            ergebnis["scan_beginn"] = lidar.letzter.zeit - lidar.letzter.dauer

    laufen(welt, e, lidar, 12.0, beobachter=beobachter)
    gedreht = math.degrees(ergebnis.get("ende", 0.0) - ergebnis.get("start", 0.0))
    return ergebnis, gedreht


erg, gedreht = drehung_messen(misst=True)
fehler_grad = abs(gedreht - erg["ziel"])
pruefe(
    "Winkel: trotz 60 % Drehwirkung am Ziel",
    fehler_grad <= config.EXPLORE_DREH_VORHALT_GRAD + 3,
    f"Ziel {erg['ziel']:+.0f}°, gedreht {gedreht:+.0f}°",
)
erg_zeit, gedreht_zeit = drehung_messen(misst=False)
pruefe(
    "Winkel: ohne Messung nach Zeit (Rückfall, ungenau)",
    abs(gedreht_zeit) < abs(erg_zeit["ziel"]) * 0.8,
    f"Ziel {erg_zeit['ziel']:+.0f}°, gedreht {gedreht_zeit:+.0f}°",
)
erg_ohne, gedreht_ohne = drehung_messen(misst=True, mit_odometrie=False)
pruefe("Winkel: ohne Odometrie dreht er trotzdem", abs(gedreht_ohne) > 20, f"{gedreht_ohne:+.0f}°")

# Dreht er gar nicht (eingeklemmt), endet die Drehung über die Zeitgrenze.
random.seed(4)
welt = Welt(raum=(-3.0, 0.35, -3.0, 3.0))
welt.dreh_anteil = 0.0
e, lidar = aufbauen(welt)
erstes_ende = []
laufen(welt, e, lidar, 15.0, beobachter=lambda t, e, b: (
    erstes_ende.append((t, e._dreh_richtung)) if e._dreh_fertig is not None and not erstes_ende else None))
grenze = (
    abs(math.radians(erstes_ende[0][1])) / config.EXPLORE_TURN_SPEED * config.EXPLORE_DREH_ZEIT_FAKTOR
    if erstes_ende else None
)
pruefe(
    "Winkel: Zeitgrenze, wenn er sich nicht dreht",
    bool(erstes_ende) and abs(erstes_ende[0][0] - grenze) < 0.1,
    erstes_ende and f"Ende nach {erstes_ende[0][0]:.2f} s, Grenze {grenze:.2f} s",
)

# --- 5. Nach der Drehung erst eine frische Lidar-Runde -------------------------
pruefe(
    "Frische Runde: fährt erst mit einer Runde nach der Drehung",
    "los" in erg and erg["scan_beginn"] >= erg["fertig"] + config.EXPLORE_NACHLAUF_S - 1e-9,
    {k: round(v, 2) for k, v in erg.items() if isinstance(v, float)},
)

# --- 6. Ringsum zu: kein Platz zum Befreien -> gibt nach N Versuchen auf -------
random.seed(6)
welt = Welt(raum=(-0.20, 0.20, -0.20, 0.20))
e, lidar = aufbauen(welt)
z, _, ende = laufen(welt, e, lidar, 30.0)
pruefe("Eingeklemmt: bewegt sich nicht", all(b is None or (b.vx, b.vy, b.omega) == (0, 0, 0) for _, _, b in z))
pruefe("Eingeklemmt: endet mit Grund", ende is not None and "festgefahren" in (e.letzter_grund or ""), e.letzter_grund)

# --- 7. Freies Erkunden im Zimmer: keine Kollision, kein Aufgeben --------------
for misst, name in ((True, "mit Drehrate"), (False, "nach Zeit")):
    random.seed(7)
    welt = Welt(
        raum=(-2.5, 2.5, -2.0, 2.0),
        kreise=[(1.0, 0.8, 0.30), (-1.0, -0.7, 0.25), (0.8, -1.2, 0.20), (-1.2, 1.0, 0.35)],
        x=-0.2, y=0.0,
    )
    welt.dreh_anteil = 0.8
    e, lidar = aufbauen(welt, misst=misst)
    strecke = [0.0]
    letzte = [welt.x, welt.y]

    def mitzaehlen(jetzt, e, befehl):
        strecke[0] += math.hypot(welt.x - letzte[0], welt.y - letzte[1])
        letzte[:] = [welt.x, welt.y]

    z, kleinster, ende = laufen(welt, e, lidar, 120.0, beobachter=mitzaehlen)
    pruefe(f"Zimmer ({name}): fährt die ganze Zeit", ende is None, e.letzter_grund)
    pruefe(f"Zimmer ({name}): keine Berührung", kleinster > ROBOTER_R, f"kleinster Abstand {kleinster:.2f} m")
    pruefe(f"Zimmer ({name}): kommt herum", strecke[0] > 8.0, f"{strecke[0]:.1f} m, {e._ausweichen}x ausgewichen")

print()
print("alle Fälle bestehen" if fehler == 0 else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
