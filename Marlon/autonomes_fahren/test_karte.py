"""Prüft das Fahren mit Karte -- ohne Roboter und ohne ROS.

    python Marlon/autonomes_fahren/test_karte.py

Die gedachte Wohnung ist die der Simulation mit Zwischenwand
(``MockLidar``, ``MOCK_ZWISCHENWAND``): zwei Zimmer, Tür bei x = 1,0 m,
y = 1,4 bis 2,3 m, eine Säule im rechten Zimmer.

Fälle:

* Karte: Abstandsfeld, Speichern als PGM + YAML und wieder Laden;
* Ortung: verschobene Lage wird zurückgezogen, Suche findet die Lage im
  Umkreis, eine falsche Gegend wird nicht "gefunden";
* Planung: Weg ins Nachbarzimmer führt durch die Tür und hält Abstand;
  geschlossene Tür heißt kein Weg; Ziel in der Wand wird verschoben;
* Lücken: Ist das Nachbarzimmer unbekannt, führt der Weg zur Tür; ist alles
  bekannt, gibt es keine Lücke mehr;
* Pfadfolger: fährt in der gedachten Welt ins Nachbarzimmer, berührt nichts;
* Kartierfahrt: Ist das Zimmer fertig, endet sie nicht, sondern fährt zur
  Lücke; versperrte Lücken werden übersprungen; am Ende "Karte vollständig".
"""

import math
import pathlib
import sys
import tempfile
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402

config.MOCK_ZWISCHENWAND = True

from rosmaster_assist.mapping.karte import Karte  # noqa: E402
from rosmaster_assist.mapping.map_service import MapService  # noqa: E402
from rosmaster_assist.navigation import kartenortung as ko  # noqa: E402
from rosmaster_assist.navigation import kartierfahrt as kf  # noqa: E402
from rosmaster_assist.navigation import pfadfolger as pf  # noqa: E402
from rosmaster_assist.navigation import pfadplanung as pp  # noqa: E402
from rosmaster_assist.sensors.lidar import MockLidar  # noqa: E402

fehler = 0


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


# --- Gedachte Wohnung ---------------------------------------------------------------

LIDAR = MockLidar()
X_MIN, X_MAX, Y_MIN, Y_MAX = MockLidar.RAUM
TUER = (1.4, 2.3)


def _strecke_abstand(x, y, strecke):
    x1, y1, x2, y2 = strecke
    ex, ey = x2 - x1, y2 - y1
    t = np.clip(((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
    return np.hypot(x - x1 - t * ex, y - y1 - t * ey)


def welt_abstand(x, y, tuer_offen=True):
    """Abstand eines Punktes zur nächsten Wand (oder zum nächsten Möbel) der Wohnung.

    Dieselben Strecken wie im simulierten Lidar (``MockLidar.ZWISCHENWAND``).
    """
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    d = np.minimum.reduce([x - X_MIN, X_MAX - x, y - Y_MIN, Y_MAX - y])
    strecken = list(MockLidar.ZWISCHENWAND)
    if not tuer_offen:
        strecken.append((1.0, TUER[0], 1.0, TUER[1]))
    for s in strecken:
        d = np.minimum(d, _strecke_abstand(x, y, s))
    cx, cy, r = MockLidar.SAEULE
    return np.minimum(d, np.hypot(x - cx, y - cy) - r)


def wohnung(aufloesung=0.05, tuer_offen=True, nur_links=False):
    """Karte der Wohnung; ``nur_links``: das rechte Zimmer ist unbekannt."""
    ursprung = (X_MIN - 0.5, Y_MIN - 0.5)
    breite = int((X_MAX - X_MIN + 1.0) / aufloesung)
    hoehe = int((Y_MAX - Y_MIN + 1.0) / aufloesung)
    iy, ix = np.mgrid[0:hoehe, 0:breite]
    x = ursprung[0] + (ix + 0.5) * aufloesung
    y = ursprung[1] + (iy + 0.5) * aufloesung
    innen = (x > X_MIN) & (x < X_MAX) & (y > Y_MIN) & (y < Y_MAX)
    d = welt_abstand(x, y, tuer_offen)
    belegung = np.full((hoehe, breite), -1, dtype=np.int8)
    belegung[innen & (d > aufloesung)] = 0
    belegung[(np.abs(d) <= aufloesung) & (x > X_MIN - aufloesung) & (x < X_MAX + aufloesung)
             & (y > Y_MIN - aufloesung) & (y < Y_MAX + aufloesung)] = 100
    if nur_links:
        # Rechtes Zimmer unbekannt, nur ein Stück hinter der Tür gesehen.
        rechts = x > 1.0 + aufloesung
        hinter_tuer = (x < 1.6) & (y > TUER[0]) & (y < TUER[1])
        belegung[rechts & ~hinter_tuer] = -1
    return Karte(belegung, aufloesung, ursprung)


def scan_bei(x, y, theta, schritt_grad=1.0):
    """Was der simulierte Lidar bei dieser Lage sähe, im Roboterrahmen."""
    pkt = []
    for i in range(int(360 / schritt_grad)):
        a = math.radians(i * schritt_grad)
        r = LIDAR._strahl(x, y, a + theta)
        if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
            pkt.append((r * math.cos(a), r * math.sin(a)))
    return np.asarray(pkt)


# --- Karte ---------------------------------------------------------------------------

karte = wohnung()
d_mitte = karte.abstand_bei(np.array([-1.0]), np.array([0.0]))[0]
pruefe("Abstandsfeld: Zimmermitte weit weg (gekappt)", d_mitte >= config.KARTE_ABSTAND_MAX_M - 0.01,
       f"{d_mitte:.2f} m")
d_wand = karte.abstand_bei(np.array([-2.7]), np.array([0.0]))[0]
pruefe("Abstandsfeld: 0,3 m vor der Wand", abs(d_wand - 0.3) < 0.08, f"{d_wand:.2f} m")

with tempfile.TemporaryDirectory() as ordner:
    dienst = MapService(ordner)
    info = dienst.save_map(karte.als_payload(), "test")
    geladen = Karte.laden(info["yaml"])
    gleich = (
        np.array_equal(geladen.belegt, karte.belegt)
        and np.array_equal(geladen.frei, karte.frei)
        and np.array_equal(geladen.unbekannt, karte.unbekannt)
    )
    pruefe("Speichern und Laden: dieselbe Karte", gleich)
    pruefe("Speichern und Laden: Ursprung und Auflösung",
           np.allclose(geladen.ursprung, karte.ursprung, atol=1e-6)
           and abs(geladen.aufloesung - karte.aufloesung) < 1e-9)

# --- Ortung ----------------------------------------------------------------------------

wahr = (-1.2, -0.8, math.radians(30.0))
punkte = scan_bei(*wahr)
start = (wahr[0] + 0.2, wahr[1] - 0.15, wahr[2] + math.radians(8.0))
a = ko.verfeinern(karte, punkte, start)
pruefe("Ortung: verschobene Lage wird zurückgezogen",
       math.hypot(a.x - wahr[0], a.y - wahr[1]) < 0.05
       and abs(math.degrees(ko._normalisieren(a.theta - wahr[2]))) < 2.0,
       f"({a.x:.2f}, {a.y:.2f}, {math.degrees(a.theta):.1f}°), {100 * a.anteil:.0f} % Treffer")

a = ko.suchen(karte, punkte, (wahr[0] + 0.8, wahr[1] + 0.6), 1.5)
pruefe("Ortung: Suche im Umkreis findet die Lage",
       a is not None and math.hypot(a.x - wahr[0], a.y - wahr[1]) < 0.1
       and abs(math.degrees(ko._normalisieren(a.theta - wahr[2]))) < 3.0,
       "nichts" if a is None else f"({a.x:.2f}, {a.y:.2f}, {math.degrees(a.theta):.1f}°)")

wahr_r = (3.2, -1.5, math.radians(-100.0))
a = ko.suchen(karte, scan_bei(*wahr_r), None, 0.0)
pruefe("Ortung: Suche ohne Hinweis in der ganzen Karte",
       a is not None and math.hypot(a.x - wahr_r[0], a.y - wahr_r[1]) < 0.15,
       "nichts" if a is None else f"({a.x:.2f}, {a.y:.2f})")

# Aufnahme aus dem rechten Zimmer (an der Säule), gesucht im linken: darf
# nicht passen. (Ohne Säule wären beide Zimmer gleich -- zwei gleiche
# Rechtecke kann keine Ortung unterscheiden, dann hilft nur ein Hinweis.)
a = ko.suchen(karte, scan_bei(2.6, 1.2, 1.0), (-1.0, 0.5), 1.5)
pruefe("Ortung: falsche Gegend wird nicht 'gefunden'", a is None,
       "" if a is None else f"angeblich ({a.x:.2f}, {a.y:.2f}), {100 * a.anteil:.0f} %")

# Mitten im (fast) symmetrischen linken Zimmer passt auch die Gegenrichtung:
# ohne Richtungshinweis lieber nachfragen, mit Hinweis eindeutig.
mitte_zimmer = (-1.0, 0.0, 0.0)
a = ko.suchen(karte, scan_bei(*mitte_zimmer), (-1.0, 0.0), 1.5)
pruefe("Ortung: mehrdeutig (Zimmermitte, ohne Richtung) -- keine falsche Lage",
       a is None or abs(math.degrees(ko._normalisieren(a.theta))) < 5.0,
       "nachfragen" if a is None else f"{math.degrees(a.theta):.0f}°")
a = ko.suchen(karte, scan_bei(*mitte_zimmer), (-1.0, 0.0), 1.5, winkel=math.radians(10.0))
pruefe("Ortung: mit Richtungshinweis eindeutig",
       a is not None and abs(math.degrees(ko._normalisieren(a.theta))) < 3.0,
       "nichts" if a is None else f"{math.degrees(a.theta):.1f}°")


class OdoAttrappe:
    """Lidar-Odometrie mit Drift: meldet die wahre Lage, etwas verdreht."""

    def __init__(self):
        self.lauf = 1
        self.laeuft = True
        self.nr = 0
        self.wahr = (-1.0, 0.0, 0.0)
        self.drift = (0.0, 0.0, 0.0)

    def start(self, nutzer="x"):
        pass

    def stop(self, nutzer="x"):
        pass

    def _odo(self):
        return ko.verketten(ko.umkehren(self.drift), self.wahr)

    def pose(self):
        return self._odo()

    def messung(self):
        scan = types.SimpleNamespace(punkte=lambda: scan_bei(*self.wahr))
        return types.SimpleNamespace(nr=self.nr, scan=scan, pose=self._odo())


odo = OdoAttrappe()
ortung = ko.Kartenortung(odo)
ortung.karte_setzen(karte, start=(-1.0, 0.0, 0.0), live=False)
for _ in range(3):
    odo.nr += 1
    ortung._suche_zuletzt = 0.0
    ortung.abgleichen()
p = ortung.pose()
pruefe("Kartenortung: geladene Karte, Lage am Ausgangspunkt gefunden",
       ortung.sicher and p is not None and math.hypot(p[0] + 1.0, p[1]) < 0.08,
       ortung.zustand()["text"])

# Driften lassen: Die Odometrie verdreht sich langsam, die Ortung hält dagegen.
for k in range(40):
    odo.wahr = (-1.0 + 0.04 * k, 0.03 * k, math.radians(2.0 * k))
    odo.drift = (0.004 * k, -0.003 * k, math.radians(0.3 * k))
    odo.nr += 1
    ortung.abgleichen()
p = ortung.pose()
pruefe("Kartenortung: Drift der Odometrie wird ausgeglichen",
       p is not None and math.hypot(p[0] - odo.wahr[0], p[1] - odo.wahr[1]) < 0.08
       and abs(math.degrees(ko._normalisieren(p[2] - odo.wahr[2]))) < 2.5,
       f"{math.hypot(p[0] - odo.wahr[0], p[1] - odo.wahr[1]):.3f} m daneben" if p else "keine Lage")

# --- Planung ---------------------------------------------------------------------------


def durch_tuer(weg):
    """Kreuzt der Weg x = 1,0 innerhalb der Türöffnung?"""
    pkt = [(-1.5, -1.5)] + list(weg.punkte)
    for (x1, y1), (x2, y2) in zip(pkt, pkt[1:]):
        if (x1 - 1.0) * (x2 - 1.0) < 0:
            t = (1.0 - x1) / (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not TUER[0] + 0.1 < y < TUER[1] - 0.1:
                return False
    return True


def kleinster_abstand(weg, start):
    pkt = [start] + list(weg.punkte)
    xs, ys = [], []
    for (x1, y1), (x2, y2) in zip(pkt, pkt[1:]):
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.02))
        xs += list(np.linspace(x1, x2, n))
        ys += list(np.linspace(y1, y2, n))
    return float(welt_abstand(np.array(xs), np.array(ys)).min())


weg = pp.weg_planen(karte, (-1.5, -1.5), (3.5, -1.5))
pruefe("Planung: Weg ins Nachbarzimmer gefunden", weg is not None,
       "" if weg is None else f"{weg.laenge:.1f} m, {len(weg.punkte)} Wegpunkte")
if weg is not None:
    pruefe("Planung: Weg führt durch die Tür", durch_tuer(weg))
    abstand = kleinster_abstand(weg, (-1.5, -1.5))
    pruefe("Planung: Weg hält Abstand zu Wänden", abstand >= config.KARTE_SICHERHEIT_M - 0.06,
           f"engste Stelle {abstand:.2f} m")

zu = wohnung(tuer_offen=False)
pruefe("Planung: geschlossene Tür -- kein Weg", pp.weg_planen(zu, (-1.5, -1.5), (3.5, -1.5)) is None)

weg = pp.weg_planen(karte, (-1.5, -1.5), (-2.98, 0.0))
pruefe("Planung: Ziel in der Wand wird auf befahrbare Stelle verschoben",
       weg is not None and welt_abstand(np.array([weg.ziel[0]]), np.array([weg.ziel[1]]))[0]
       >= config.KARTE_SICHERHEIT_M - 0.06,
       "" if weg is None else f"Ziel ({weg.ziel[0]:.2f}, {weg.ziel[1]:.2f})")

# --- Lücken ----------------------------------------------------------------------------

halb = wohnung(nur_links=True)
weg = pp.luecke_suchen(halb, (-1.5, -1.5))
pruefe("Lücken: rechtes Zimmer unbekannt -- Weg zur Tür",
       weg is not None and weg.luecke is not None and abs(weg.luecke[0] - 1.4) < 0.6
       and TUER[0] - 0.3 < weg.luecke[1] < TUER[1] + 0.3,
       "keine" if weg is None else f"Lücke bei ({weg.luecke[0]:.2f}, {weg.luecke[1]:.2f})")
if weg is not None:
    weg2 = pp.luecke_suchen(halb, (-1.5, -1.5), erledigt=[weg.luecke])
    pruefe("Lücken: angefahrene Lücke zählt nicht mehr", weg2 is None or
           math.hypot(weg2.luecke[0] - weg.luecke[0], weg2.luecke[1] - weg.luecke[1])
           >= config.MAP_LUECKE_ERLEDIGT_M)
pruefe("Lücken: ganze Wohnung bekannt -- keine Lücke", pp.luecke_suchen(karte, (-1.5, -1.5)) is None)

# --- Pfadfolger in der gedachten Welt --------------------------------------------------


def fahren(folger, lage, schritte=4000, dt=0.05):
    """Einfache Kinematik ohne Schlupf; gibt (Zustand, Lage, engster Abstand) zurück."""
    x, y, th = lage
    engste = float("inf")
    zustand = pf.FAEHRT
    for _ in range(schritte):
        befehl, zustand = folger.schritt((x, y, th), scan_bei(x, y, th, 2.0))
        if zustand != pf.FAEHRT:
            break
        mitte = th + befehl.omega * dt / 2
        x += (befehl.vx * math.cos(mitte) - befehl.vy * math.sin(mitte)) * dt
        y += (befehl.vx * math.sin(mitte) + befehl.vy * math.cos(mitte)) * dt
        th += befehl.omega * dt
        engste = min(engste, float(welt_abstand(np.array([x]), np.array([y]))[0]))
    return zustand, (x, y, th), engste


weg = pp.weg_planen(karte, (-1.5, -1.5), (3.5, -1.5))
folger = pf.Pfadfolger()
folger.setzen(weg.punkte, von=(-1.5, -1.5), ziel_winkel=math.radians(90.0))
zustand, lage, engste = fahren(folger, (-1.5, -1.5, 0.0))
pruefe("Pfadfolger: kommt im Nachbarzimmer an", zustand == pf.ANGEKOMMEN
       and math.hypot(lage[0] - 3.5, lage[1] + 1.5) <= config.KARTE_ZIEL_TOLERANZ_M + 0.02,
       f"{zustand}, ({lage[0]:.2f}, {lage[1]:.2f})")
pruefe("Pfadfolger: richtet sich am Ziel aus",
       abs(math.degrees(ko._normalisieren(lage[2] - math.radians(90.0)))) <= config.KARTE_ZIEL_GRAD + 1,
       f"{math.degrees(lage[2]):.0f}°")
pruefe("Pfadfolger: berührt nichts", engste > config.ROBOTER_HALBE_BREITE_M + 0.03,
       f"engste Stelle {engste:.2f} m")

# --- Kartierfahrt: weiter ins Unbekannte ------------------------------------------------


class Sofort:
    """Planer ohne Thread: Ergebnis steht gleich bereit (wie ein fertiges Future)."""

    def __init__(self, sperre_luecke=False):
        self.sperre_luecke = sperre_luecke
        self.auftraege = []

    def _fertig(self, wert):
        return types.SimpleNamespace(done=lambda: True, result=lambda: wert)

    def luecke(self, karte, start, erledigt=(), gesperrt=()):
        self.auftraege.append("luecke")
        return self._fertig(pp.luecke_suchen(karte, start, erledigt))

    def weg(self, karte, start, ziel, gesperrt=()):
        self.auftraege.append("weg")
        return self._fertig(pp.weg_planen(karte, start, ziel))


class OrtungAttrappe:
    def __init__(self, karte, lage):
        self.karte = karte
        self.lage = lage
        self.start = (-1.5, -1.5, 0.0)
        self.sicher = True

    def pose(self):
        return self.lage


class ScanOdoAttrappe:
    def pose(self):
        return (0.0, 0.0, 0.0)

    def alter(self):
        return 0.1


ortung_a = OrtungAttrappe(halb, (-2.0, -2.0, 0.0))
fahrt = kf.Kartierfahrt(ScanOdoAttrappe(), ortung=ortung_a, planer=Sofort())
fahrt.beginnen(0.0)
fahrt._raum_fertig(0.0, "gegenüberliegende Wand erreicht")
pruefe("Kartierfahrt: Zimmer fertig -- sie endet nicht", fahrt.grund is None
       and fahrt._schritt_name == kf.LUECKE_SUCHEN, fahrt.anzeige())

scan = types.SimpleNamespace()
x, y, th = ortung_a.lage
for t in range(3000):
    ortung_a.lage = (x, y, th)
    befehl = fahrt.schritt(t * 0.05, scan, scan_bei(x, y, th, 2.0))
    if befehl is None:
        break
    x += (befehl.vx * math.cos(th) - befehl.vy * math.sin(th)) * 0.05
    y += (befehl.vx * math.sin(th) + befehl.vy * math.cos(th)) * 0.05
    th += befehl.omega * 0.05
    if fahrt._luecken_angefahren >= 1:
        break
pruefe("Kartierfahrt: fährt zur Lücke an der Tür", fahrt._luecken_angefahren >= 1
       and x > 0.0 and TUER[0] - 0.5 < y < TUER[1] + 0.5,
       f"{fahrt.anzeige()} bei ({x:.2f}, {y:.2f})")

# Ist danach alles bekannt, kommt "Karte vollständig" -- nach der Rückfahrt.
ortung_a.karte = karte
for t in range(3000):
    ortung_a.lage = (x, y, th)
    befehl = fahrt.schritt(100 + t * 0.05, scan, scan_bei(x, y, th, 2.0))
    if befehl is None:
        break
    x += (befehl.vx * math.cos(th) - befehl.vy * math.sin(th)) * 0.05
    y += (befehl.vx * math.sin(th) + befehl.vy * math.cos(th)) * 0.05
    th += befehl.omega * 0.05
pruefe("Kartierfahrt: am Ende 'Karte vollständig' am Ausgangspunkt",
       fahrt.grund is not None and "vollständig" in fahrt.grund
       and math.hypot(x - ortung_a.start[0], y - ortung_a.start[1]) < 0.4,
       f"{fahrt.grund} bei ({x:.2f}, {y:.2f})")

# Hindernis während der Fahrt zur Lücke: Lücke aufgeben, nicht die Fahrt.
ortung_b = OrtungAttrappe(halb, (-2.0, -2.0, 0.0))
fahrt = kf.Kartierfahrt(ScanOdoAttrappe(), ortung=ortung_b, planer=Sofort())
fahrt.beginnen(0.0)
fahrt._raum_fertig(0.0, "Test")
fahrt.schritt(0.0, scan, scan_bei(-2.0, -2.0, 0.0))   # sucht
fahrt.schritt(0.05, scan, scan_bei(-2.0, -2.0, 0.0))  # Weg erhalten
weiter = fahrt.hindernis_melden("Fahrschutz: Test")
pruefe("Kartierfahrt: festgefahren -- Lücke aufgeben, weiterfahren",
       weiter and fahrt.grund is None and fahrt._schritt_name == kf.LUECKE_SUCHEN
       and len(fahrt._erledigt) == 1, fahrt.anzeige())
for _ in range(config.MAP_LUECKE_MAX_FEHLER + 1):
    fahrt._ziel_luecke = (0.0, 0.0)
    fahrt._schritt_name = kf.ZUR_LUECKE
    fahrt.hindernis_melden("Test")
pruefe("Kartierfahrt: nach zu vielen Fehlschlägen endet sie", fahrt.grund is not None,
       str(fahrt.grund))

# Ohne Karte (kein SLAM) wie früher: Ende nach dem Zimmer.
fahrt = kf.Kartierfahrt(ScanOdoAttrappe())
fahrt.beginnen(0.0)
fahrt._raum_fertig(0.0, "gegenüberliegende Wand erreicht")
pruefe("Kartierfahrt ohne Karte: endet nach dem Zimmer wie bisher",
       fahrt.grund == "gegenüberliegende Wand erreicht")

print()
print("Alles in Ordnung." if fehler == 0 else f"{fehler} Fehler.")
sys.exit(1 if fehler else 0)
