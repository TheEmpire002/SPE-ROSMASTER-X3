"""Prüft, wem er beim Folgen folgt (Tracker, ZielWahl, Verhalten) -- ohne Roboter.

    python Marlon/interaktion/test_ziel.py

Synthetische Szenen mit mehreren Personen:

* Kreuzen zweier Personen (mit kurzer Verdeckung am Kreuzungspunkt);
* Eigendrehung des Roboters: die Personen wandern durchs Bild, Tracks bleiben;
* kurze Verdeckung des Ziels, während eine andere Person sichtbar ist;
* das Ziel geht aus dem Bild und kommt zurück -> er folgt weiter
  (der Grund, warum Personen seit 04.11. nicht mehr unterschieden werden);
* Folgen-Start bei zwei Personen -> die größere;
* die Folgen-Geste macht den Gestikulierenden zum Ziel, nicht den Größten;
* ein Passant kann das Folgen nicht umlenken, aber per Geste anhalten;
* schneller wiederfinden (F4): früher suchen, wenn die Person am Bildrand
  verschwand, und zu einer Person drehen, die der Lidar sieht.

Braucht nur numpy (über die Paketimporte).
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.interaktion import verhalten as vh  # noqa: E402
from rosmaster_assist.interaktion import ziel as zl  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


# --- Künstliche Personen -------------------------------------------------------

FOV = config.CAMERA_FOV_DEG
SV = 4 / 3
BREITE = 0.12


def merkmal(farbe):
    """Histogramm (16x8) mit fast allem Gewicht in einem Fach: eine "Kleiderfarbe"."""
    h = [0.2 / 127] * 128
    h[farbe] = 0.8
    return h


ROT, BLAU, GRUEN = merkmal(3), merkmal(60), merkmal(100)


def person(winkel, farbe=ROT, zeit=0.0, groesse=1.0, arm=None):
    """Person am Kamerawinkel ``winkel`` (links positiv).

    arm: None = Arme unten, "folgen" = rechter Arm über dem Kopf,
    "stopp" = beide Hände hoch.
    """
    mx = 0.5 - winkel / FOV
    sx = BREITE * groesse
    sy = 0.3
    e = sx * SV
    hand_l, hand_r = (0.6, 1.4), (-0.6, 1.4)
    if arm == "folgen":
        hand_r = (-0.4, -1.4)
    elif arm == "stopp":
        hand_l, hand_r = (0.7, -0.6), (-0.7, -0.6)

    def bild(u, v):
        return [mx + u * sx, sy + v * e, 0.9]

    kp = [[mx, sy, 0.1] for _ in range(17)]
    kp[0] = bild(0.0, -0.7)
    kp[5], kp[6] = bild(0.5, 0.0), bild(-0.5, 0.0)
    kp[9], kp[10] = bild(*hand_l), bild(*hand_r)
    kp[11], kp[12] = bild(0.35, 1.8), bild(-0.35, 1.8)
    return {
        "box": [mx - sx, sy - 0.1 * groesse, mx + sx, sy + 0.6 * groesse],
        "kp": kp,
        "score": 0.9,
        "winkel": winkel,
        "seitenverhaeltnis": SV,
        "merkmal": farbe,
        "zeit": zeit,
    }


# --- Tracker: Kreuzen ----------------------------------------------------------

# A läuft von +20° nach -20°, B umgekehrt, 10 Bilder/s, 2 s. Am
# Kreuzungspunkt sieht die Kamera nur die vordere Person (B).
tr = zl.PersonenTracker()
id_a = id_b = None
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    liste = [person(wb, BLAU, t)] if abs(wa - wb) < 5 else [person(wa, ROT, t), person(wb, BLAU, t)]
    tr.aktualisieren(liste, t)
    if i == 0:
        id_a = next(x.id for x in tr.tracks if x.merkmal == ROT)
        id_b = next(x.id for x in tr.tracks if x.merkmal == BLAU)
ta, tb = tr.track(id_a), tr.track(id_b)
pruefe("Kreuzen: Track A lebt noch", ta is not None and ta.gemessen, [x.id for x in tr.tracks])
pruefe("Kreuzen: A ist rechts angekommen", ta is not None and ta.person["winkel"] < -15,
       ta and ta.person["winkel"])
pruefe("Kreuzen: A trägt weiter Rot", ta is not None and ta.person["merkmal"] is ROT)
pruefe("Kreuzen: B ist links angekommen und Blau",
       tb is not None and tb.person["winkel"] > 15 and tb.person["merkmal"] is BLAU)

# --- Tracker: Eigendrehung -------------------------------------------------------

# Roboter dreht mit 1 rad/s links -> beide (stehenden) Personen wandern mit
# ~57°/s nach rechts. Abstand der beiden nur 20°, 5 Bilder/s.
omega = 1.0
tr = zl.PersonenTracker()
ids = None
stabil = True
for i in range(6):
    t = i * 0.2
    versatz = -57.3 * omega * t
    liste = [person(25 + versatz, ROT, t), person(5 + versatz, BLAU, t)]
    tr.aktualisieren(liste, t, omega=omega)
    zuordnung = {x.id: x.person["merkmal"] is ROT for x in tr.tracks if x.gemessen}
    if ids is None:
        ids = zuordnung
    stabil &= zuordnung == ids
pruefe("Eigendrehung: IDs bleiben trotz Drehung", stabil and len(tr.tracks) == 2,
       [(x.id, round(x.w)) for x in tr.tracks])

pruefe("Eigendrehung: Personen gelten als stehend (v ~ 0)",
       all(abs(x.v) < 5 for x in tr.tracks), [round(x.v) for x in tr.tracks])

# Gegenprobe: ohne Drehrate hält der Filter die Drehung für Bewegung der
# Personen -- nach einem Halt des Roboters lägen die Vorhersagen daneben.
tr = zl.PersonenTracker()
for i in range(6):
    t = i * 0.2
    versatz = -57.3 * omega * t
    tr.aktualisieren([person(25 + versatz, ROT, t), person(5 + versatz, BLAU, t)], t, omega=0.0)
pruefe("Eigendrehung: ohne Drehrate falsche Personengeschwindigkeit (Gegenprobe)",
       all(x.v < -20 for x in tr.tracks), [round(x.v) for x in tr.tracks])


# --- ZielWahl ------------------------------------------------------------------


def lauf(tr, zw, szenen, start=0.0, hz=10.0):
    """szenen: Liste von Personenlisten je Bild. Gibt die zugeordneten Tracks zurück."""
    ergebnis = []
    for i, liste in enumerate(szenen):
        t = start + i / hz
        for p in liste:
            p["zeit"] = t
        tr.aktualisieren(liste, t)
        ergebnis.append(zw.zuordnen(tr.tracks, t, t))
    return ergebnis


tr, zw = zl.PersonenTracker(), zl.ZielWahl()
zw.erfassen(None, 0.0)
lauf(tr, zw, [[person(0, ROT)] for _ in range(5)])
pruefe("Start: eine Person -> folgt", zw.zustand == zl.FOLGT, zw.als_dict())
# B kommt dazu (rechts), dann ist A kurz (0,5 s) verdeckt, B bleibt sichtbar.
lauf(tr, zw, [[person(0, ROT), person(-25, BLAU)] for _ in range(5)], start=0.5)
weg = lauf(tr, zw, [[person(-25, BLAU)] for _ in range(5)], start=1.0)
pruefe("Kurz verdeckt: B wird nicht Ziel", all(x is None for x in weg), [x and x.id for x in weg])
pruefe("Kurz verdeckt: Zustand kurz weg", zw.zustand == zl.KURZ_WEG, zw.zustand)
zurueck = lauf(tr, zw, [[person(3, ROT), person(-25, BLAU)] for _ in range(3)], start=1.5)
pruefe("Kurz verdeckt: wieder Rot", zurueck[0] is not None and zurueck[0].person["merkmal"] is ROT
       and zw.zustand == zl.FOLGT, zw.als_dict())

# Der eigentliche Fall: Rot geht aus dem Bild (Track verfällt) und kommt
# wieder -- mit neuer Track-Nummer, aber er folgt ihr sofort weiter.
tr, zw = zl.PersonenTracker(), zl.ZielWahl()
zw.erfassen(None, 0.0)
lauf(tr, zw, [[person(10, ROT)] for _ in range(5)])
alte_id = zw.id
lauf(tr, zw, [[] for _ in range(30)], start=0.5)
pruefe("Weg und zurück: alter Track verfallen", tr.track(alte_id) is None)
zurueck = lauf(tr, zw, [[person(-5, ROT)]], start=3.5)
pruefe("Weg und zurück: sofort wieder Ziel", zurueck[0] is not None and zw.zustand == zl.FOLGT,
       zw.als_dict())
pruefe("Weg und zurück: neue Track-Nummer, egal", zw.id != alte_id, (alte_id, zw.id))

# Lange weg, eine andere Person im Bild: nach ZIEL_WECHSEL_S folgt er ihr.
tr, zw = zl.PersonenTracker(), zl.ZielWahl()
zw.erfassen(None, 0.0)
lauf(tr, zw, [[person(0, ROT, groesse=1.3), person(-25, BLAU)] for _ in range(5)])
pruefe("Start bei zwei: die größere (Rot)", tr.track(zw.id).person["merkmal"] is ROT, zw.als_dict())
res = lauf(tr, zw, [[person(-25, BLAU)] for _ in range(20)], start=0.5)
n = int(round(config.ZIEL_WECHSEL_S * 10))
pruefe("Lange weg: erst nach ZIEL_WECHSEL_S die andere Person",
       all(x is None for x in res[:n - 1]) and res[-1] is not None and res[-1].person["merkmal"] is BLAU,
       [x and x.id for x in res])

# Kreuzen: Am Kreuzungspunkt ist Rot kurz verdeckt -- er bleibt bei Rot.
tr, zw = zl.PersonenTracker(), zl.ZielWahl()
zw.erfassen(None, 0.0)
lauf(tr, zw, [[person(20, ROT)] for _ in range(5)])
szenen = []
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    szenen.append([person(wb, BLAU)] if abs(wa - wb) < 5 else [person(wa, ROT), person(wb, BLAU)])
res = lauf(tr, zw, szenen, start=0.5)
pruefe("Kreuzen: Ziel ist nie die blaue Person",
       all(x is None or x.person["merkmal"] is ROT for x in res))
pruefe("Kreuzen: am Ende folgt er wieder Rot", res[-1] is not None and res[-1].person["merkmal"] is ROT,
       zw.als_dict())


# --- Verhalten mit Attrappen ---------------------------------------------------


class Watchdog:
    def __init__(self):
        self.befehle, self.stopps = [], 0

    def submit(self, cmd, quelle="hand"):
        self.befehle.append(cmd)

    def stop(self):
        self.stopps += 1


class Lidar:
    """Beine jeder Person als Objekt in 2 m an ihrem Winkel."""

    def __init__(self, pose):
        self.pose = pose

    def latest(self):
        winkel = [float(w) for w in range(360)]
        r = [5.0] * 360
        for p in self.pose.liste:
            for d in range(-4, 5):
                r[int(round(p["winkel"] + d)) % 360] = 2.0
        return Scan(winkel, r)


class Pose:
    verfuegbar = True
    fov_deg = FOV

    def __init__(self):
        self.liste = []

    def personen(self):
        return self.liste

    def aktivieren(self, an):
        pass


class Odometrie:
    def __init__(self):
        self.omega = 0.0

    def geschwindigkeit(self):
        return (0.0, 0.0, self.omega)


def interaktion():
    po = Pose()
    wd, li = Watchdog(), Lidar(po)
    ia = vh.Interaktion(wd, li, po, odometry=Odometrie())
    ia._modus, ia._modus_seit, ia._gesehen = vh.BEREIT, 0.0, 0.0
    return ia, wd, po


def spielen(ia, po, erzeuger, von, bis, hz=10.0):
    """erzeuger(t) -> Personenliste. Gibt [(t, modus, befehl)] zurück."""
    verlauf = []
    n = int(round((bis - von) * hz))
    for i in range(n):
        t = von + i / hz
        po.liste = [dict(p, zeit=t) for p in erzeuger(t)]
        verlauf.append((t, ia._modus, ia._schritt(t)))
    return verlauf


def folgt_rot(ia, po):
    """Start per Knopf mit genau einer Person (Rot vorn) bis FOLGT."""
    ia._wechseln(vh.FOLGEN, "Test", 0.0)
    spielen(ia, po, lambda t: [person(0, ROT)], 0.0, 1.5)


# App-Start (Knopf) bei zwei Personen -> folgt der größeren
ia, wd, po = interaktion()
zwei_gross = lambda t: [person(10, ROT), person(-15, BLAU, groesse=1.3)]
ia._wechseln(vh.FOLGEN, "Test", 0.0)
v = spielen(ia, po, zwei_gross, 0.0, 1.5)
ziel = ia._ziel_waehlen(1.5)
pruefe("App-Start bei zwei Personen: folgt", ia._modus == vh.FOLGEN, ia._modus)
pruefe("App-Start bei zwei Personen: der größeren (Blau)", ziel is not None and ziel["merkmal"] is BLAU,
       ziel and ziel["winkel"])
pruefe("App-Start bei zwei Personen: dreht zu ihr (rechts)", v[-1][2].omega < 0, v[-1][2])

# Der eigentliche Fall: Rot geht aus dem Bild, er sucht, Rot kommt zurück
ia, wd, po = interaktion()
folgt_rot(ia, po)
ende = 1.5 + config.FOLGEN_VERLOREN_S + 0.5
spielen(ia, po, lambda t: [], 1.5, ende)
pruefe("Suche: Rot weg -> sucht", ia._modus == vh.SUCHEN, ia._modus)
ia.meldungen_abholen()
spielen(ia, po, lambda t: [person(15, ROT)], ende, ende + 0.5)
pruefe("Suche: Rot zurück -> folgt wieder", ia._modus == vh.FOLGEN, ia._modus)
pruefe("Suche: meldet 'wiedergefunden'", any("wiedergefunden" in m for m in ia.meldungen_abholen()))

# Während der Suche taucht jemand auf -- unterschieden wird nicht, er folgt.
ia, wd, po = interaktion()
folgt_rot(ia, po)
spielen(ia, po, lambda t: [], 1.5, ende)
spielen(ia, po, lambda t: [person(-10, BLAU)], ende, ende + 0.5)
pruefe("Suche: wer auftaucht, dem folgt er", ia._modus == vh.FOLGEN, ia._modus)

# Niemand kommt zurück: Mit Zeitgrenze (Standard 0 = endlos suchen) hält er
# danach an, der Grund steht in der App.
suchen_s, config.FOLGEN_SUCHEN_S = config.FOLGEN_SUCHEN_S, 3.0
ia, wd, po = interaktion()
folgt_rot(ia, po)
spielen(ia, po, lambda t: [], 1.5, ende)
v = spielen(ia, po, lambda t: [], ende, ende + config.FOLGEN_SUCHEN_S + 0.5)
pruefe("Suche: fährt dabei nie vorwärts", all(b is None or b.vx <= 0 for _, _, b in v))
pruefe("Suche: am Ende angehalten, kein Ziel",
       ia._modus == vh.BEREIT and ia._zielwahl.zustand == zl.IDLE and wd.stopps >= 1,
       ia.zustand()["ziel"])
pruefe("Suche: Grund 'nicht wiedergefunden'", "wiedergefunden" in ia.zustand()["ziel"]["grund"],
       ia.zustand()["ziel"])
config.FOLGEN_SUCHEN_S = suchen_s

# Folgen-Geste: der Gestikulierende wird Ziel, nicht die größere Person
ia, wd, po = interaktion()
spielen(ia, po, lambda t: [person(15, BLAU, groesse=1.3), person(-15, ROT, arm="folgen")], 0.0, 1.5)
ziel = ia._ziel_waehlen(1.5)
pruefe("Geste: Folgen ausgelöst", ia._modus == vh.FOLGEN, ia._modus)
pruefe("Geste: Ziel ist der Gestikulierende (Rot)", ziel is not None and ziel["merkmal"] is ROT,
       ziel and ziel["winkel"])

# Passant gestikuliert beim Folgen: Herkommen-Wechsel nicht, Stopp schon
ia, wd, po = interaktion()
folgt_rot(ia, po)
spielen(ia, po, lambda t: [person(0, ROT), person(-20, BLAU, arm="folgen")], 1.5, 3.0)
pruefe("Passant: Geste ändert das Ziel nicht",
       ia._modus == vh.FOLGEN and ia._ziel_waehlen(3.0)["merkmal"] is ROT,
       ia.zustand()["ziel"])
spielen(ia, po, lambda t: [person(0, ROT), person(-20, BLAU, arm="stopp")], 3.0, 4.5)
pruefe("Passant: Stopp-Geste hält an", ia._modus == vh.BEREIT and wd.stopps >= 1, ia._modus)

# Ausschalten löscht alles
ia, wd, po = interaktion()
folgt_rot(ia, po)
ia._ziel_vergessen()
pruefe("Vergessen: kein Ziel, keine Tracks", ia._zielwahl.zustand == zl.IDLE and not ia._tracker.tracks)

# /interaktion/pose: Zustand enthält das Ziel-Feld (Schnittstelle unverändert)
z = ia.zustand()
pruefe("Zustand: Feld ziel mit zustand/id/sicherheit/grund",
       set(z["ziel"]) == {"zustand", "id", "sicherheit", "grund"}, z["ziel"])


# --- Bedienung aus der App -----------------------------------------------------


class LaeuftNoch:
    """Steht für die laufende Interaktions-Schleife (aktiv == True)."""

    def done(self):
        return False


def track_id(ia, farbe):
    return next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is farbe)


zwei = lambda t: [person(10, ROT), person(-15, BLAU)]

# Knopf "Folge mir": ohne Nummer im Text
ia, wd, po = interaktion()
ia._task = LaeuftNoch()
spielen(ia, po, zwei, 0.0, 0.3)
pruefe("Knopf: angenommen", ia.folgen() is True)
pruefe("Knopf: Text ohne Personennummer", ia.zustand()["text"] == "folgt (per App)", ia.zustand()["text"])

# Ältere Clients schicken noch ziel_id: Die Person gilt dann als Start-Ziel.
ia._wechseln(vh.BEREIT, "Test", 0.3)
blau_id = track_id(ia, BLAU)
pruefe("ziel_id (alt): angenommen", ia.folgen(blau_id) is True)
v = spielen(ia, po, zwei, 0.3, 1.8)
pruefe("ziel_id (alt): folgt Blau", ia._modus == vh.FOLGEN and ia._zielwahl.id == blau_id, ia.zustand()["ziel"])
pruefe("ziel_id (alt): dreht zu Blau (rechts)", v[-1][2].omega < 0, v[-1][2])
pruefe("ziel_id unbekannt: trotzdem angenommen", ia.folgen(999) is True)

# Pause: steht, Ziel bleibt gemerkt und verfolgt
pruefe("Pause: angenommen", ia.pausieren() is True)
pruefe("Pause: Zustand meldet pausiert", ia.zustand()["pausiert"] is True)
# Länger als Verloren + Suchen: Ohne Pause wäre er längst in der Suche.
t0 = 1.8 + config.FOLGEN_VERLOREN_S + config.FOLGEN_SUCHEN_S + 1.0
v = spielen(ia, po, zwei, 1.8, t0)
pruefe("Pause: fährt nicht", all(b is None or (b.vx == 0 and b.vy == 0 and b.omega == 0) for _, _, b in v),
       [b for _, _, b in v if b is not None][:2])
pruefe("Pause: Modus und Ziel bleiben", ia._modus == vh.FOLGEN and ia._zielwahl.id == blau_id,
       (ia._modus, ia.zustand()["ziel"]))
pruefe("Pause: doppelt pausieren geht nicht", ia.pausieren() is False)

# Weiter: fährt wieder zum selben Ziel
pruefe("Weiter: angenommen", ia.fortsetzen() is True)
v = spielen(ia, po, zwei, t0, t0 + 1.0)
pruefe("Weiter: folgt wieder Blau", ia._modus == vh.FOLGEN and ia._zielwahl.id == blau_id
       and not ia.zustand()["pausiert"], ia.zustand()["ziel"])
pruefe("Weiter: dreht wieder zu Blau", v[-1][2].omega < 0, v[-1][2])
pruefe("Weiter: ohne Pause abgelehnt", ia.fortsetzen() is False)

# Pause, dann Stopp-Geste: Pause endet mit dem Modus
ia.pausieren()
spielen(ia, po, lambda t: [person(10, ROT), person(-15, BLAU, arm="stopp")], t0 + 1.0, t0 + 2.5)
pruefe("Pause + Stopp: bereit, nicht mehr pausiert",
       ia._modus == vh.BEREIT and not ia.zustand()["pausiert"], (ia._modus, ia.zustand()["pausiert"]))
pruefe("Pause im Stand: abgelehnt", ia.pausieren() is False)

# Pose für die App: id und ziel je Person, kein Merkmal
ia, wd, po = interaktion()
ia._task = LaeuftNoch()
spielen(ia, po, zwei, 0.0, 0.3)
blau_id = track_id(ia, BLAU)
ia.folgen(blau_id)
spielen(ia, po, zwei, 0.3, 1.0)
liste = ia.personen_mit_id(po.personen())
pruefe("Pose-IDs: kein Merkmal", all("merkmal" not in p for p in liste))
pruefe("Pose-IDs: jede Person hat eine ID", all(isinstance(p["id"], int) for p in liste), liste)
pruefe("Pose-IDs: genau Blau ist Ziel",
       [p["id"] for p in liste if p["ziel"]] == [blau_id], [(p["id"], p["ziel"]) for p in liste])
pruefe("Pose-IDs: Reihenfolge unverändert", [p["winkel"] for p in liste] == [10, -15],
       [p["winkel"] for p in liste])
fremd = ia.personen_mit_id([person(0, ROT)])
pruefe("Pose-IDs: neues Bild vor dem Tracker -> id None", fremd[0]["id"] is None and not fremd[0]["ziel"])

# Loslassen löscht auch den Grund der letzten Suche
ia._wechseln(vh.BEREIT, "Test", 1.0)
ia._zielwahl.freigeben("Person nicht wiedergefunden")
ia.folgen_stop()
pruefe("Loslassen: kein Grund mehr", ia._zielwahl.zustand == zl.IDLE and not ia._zielwahl.grund,
       ia.zustand()["ziel"])

# --- Schneller wiederfinden (F4) ------------------------------------------------

# Am Bildrand verschwunden -> früher suchen als in der Bildmitte
rand = FOV / 2 - config.ZIEL_RAND_GRAD + 3
for name, winkel, erwartet in (("Rand", rand, vh.SUCHEN), ("Mitte", 0, vh.FOLGEN)):
    ia, wd, po = interaktion()
    ia._wechseln(vh.FOLGEN, "Test", 0.0)
    spielen(ia, po, lambda t, w=winkel: [person(w, ROT)], 0.0, 1.5)
    weg = 1.5 + config.FOLGEN_VERLOREN_RAND_S + 0.3
    spielen(ia, po, lambda t: [], 1.5, weg)
    pruefe(f"Bildrand: zuletzt {name} -> nach {weg - 1.5:.1f} s {erwartet}", ia._modus == erwartet,
           (ia._modus, ia._zielwahl.zustand))


def om(b):
    return 0.0 if b is None else b.omega


class Mensch:
    def __init__(self, winkel):
        self.art, self.winkel, self.abstand = vh.hd.MENSCH, winkel, 2.0


def in_suche(ia, po):
    """Folgt Rot (leicht links), dann verschwindet Rot -> Suche; gibt die Zeit zurück."""
    ia._wechseln(vh.FOLGEN, "Test", 0.0)
    spielen(ia, po, lambda t: [person(5, ROT)], 0.0, 1.5)
    ende = 1.5 + config.FOLGEN_VERLOREN_S + 0.5
    spielen(ia, po, lambda t: [], 1.5, ende)
    return ende


# Lidar sieht hinten rechts eine Person -> dreht nach rechts, obwohl Rot
# zuletzt links war; steht sie dann im Bild, hält er kurz still.
ia, wd, po = interaktion()
ende = in_suche(ia, po)
pruefe("Lidar-Hinweis: sucht", ia._modus == vh.SUCHEN, ia._modus)
richtung = [-120.0]
ia._hinderniserkennung.aktuell = lambda scan, *a: [Mensch(richtung[0])]
v = spielen(ia, po, lambda t: [], ende, ende + 0.5)
pruefe("Lidar-Hinweis: dreht zur Person (rechts)", all(om(b) < 0 for _, _, b in v), [om(b) for _, _, b in v][:3])
richtung[0] = -5.0
v = spielen(ia, po, lambda t: [], ende + 0.5, ende + 0.5 + config.ZIEL_HINWEIS_S - 0.2)
pruefe("Lidar-Hinweis: im Bild -> schaut still hin", all(om(b) == 0 for _, _, b in v), [om(b) for _, _, b in v][:3])
t1 = ende + 0.5 + config.ZIEL_HINWEIS_S
v = spielen(ia, po, lambda t: [], t1, t1 + config.FOLGEN_SUCHEN_DREH_S + config.FOLGEN_SUCHEN_PAUSE_S)
pruefe("Lidar-Hinweis: niemand erkannt -> wieder fester Suchtakt", any(om(b) != 0 for _, _, b in v))

# Kein Lidar-Mensch -> fester Takt zur letzten Seite (links) wie bisher
ia, wd, po = interaktion()
ende = in_suche(ia, po)
ia._hinderniserkennung.aktuell = lambda scan, *a: []
v = spielen(ia, po, lambda t: [], ende, ende + 0.5)
pruefe("Ohne Lidar-Hinweis: dreht zur letzten Seite (links)", om(v[0][2]) > 0, v[0][2])

print()
print("alles bestanden" if not fehler else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
