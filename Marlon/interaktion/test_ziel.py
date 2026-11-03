"""Prüft den Ziel-Lock beim Folgen (Tracker, ZielSperre, Verhalten) -- ohne Roboter.

    python Marlon/interaktion/test_ziel.py

Synthetische Szenen mit mehreren Personen:

* Kreuzen zweier Personen (mit kurzer Verdeckung am Kreuzungspunkt);
* Eigendrehung des Roboters: die Personen wandern durchs Bild, IDs bleiben;
* kurze Verdeckung des Ziels, während eine andere Person sichtbar ist;
* zweite Person tritt während der Suche ein -> sicher anhalten, nicht folgen;
* zwei gleich gekleidete Personen nach Verlust -> sicher anhalten;
* Folgen-Start ohne Auswahl bei zwei Personen -> abgelehnt;
* die Folgen-Geste macht den Gestikulierenden zum Ziel, nicht den Größten;
* ein Passant kann das Folgen nicht umlenken, aber per Geste anhalten.

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


# --- ZielSperre: Verdeckung ------------------------------------------------------


def lauf(tr, sp, szenen, start=0.0, hz=10.0):
    """szenen: Liste von Personenlisten je Bild. Gibt die zugeordneten Tracks zurück."""
    ergebnis = []
    for i, liste in enumerate(szenen):
        t = start + i / hz
        for p in liste:
            p["zeit"] = t
        tr.aktualisieren(liste, t)
        ergebnis.append(sp.zuordnen(tr.tracks, t, t))
    return ergebnis


tr, sp = zl.PersonenTracker(), zl.ZielSperre()
sp.erfassen(None, 0.0)
lauf(tr, sp, [[person(0, ROT)] for _ in range(15)])
pruefe("Erfassen: eine Person -> folgt", sp.zustand == zl.FOLGT, sp.als_dict())
ziel_id = sp.id
# B kommt dazu (rechts), dann ist A 1 s verdeckt, B bleibt sichtbar.
lauf(tr, sp, [[person(0, ROT), person(-25, BLAU)] for _ in range(5)], start=1.5)
weg = lauf(tr, sp, [[person(-25, BLAU)] for _ in range(10)], start=2.0)
pruefe("Verdeckung: B wird nie Ziel", all(x is None for x in weg), [x and x.id for x in weg])
pruefe("Verdeckung: Zustand kurz weg", sp.zustand == zl.KURZ_WEG, sp.zustand)
zurueck = lauf(tr, sp, [[person(3, ROT), person(-25, BLAU)] for _ in range(3)], start=3.0)
pruefe("Verdeckung kurz: derselbe Track, sofort wieder Ziel",
       zurueck[0] is not None and sp.id == ziel_id and sp.zustand == zl.FOLGT, sp.als_dict())

# Lange Verdeckung (2 s): Der Track ist verworfen, Rot kommt als neuer Track
# zurück und muss sich erst über mehrere Bilder bestätigen.
lauf(tr, sp, [[person(-25, BLAU)] for _ in range(20)], start=3.3)
pruefe("Verdeckung lang: alter Track verworfen", tr.track(ziel_id) is None)
zurueck = lauf(tr, sp, [[person(5, ROT), person(-25, BLAU)] for _ in range(8)], start=5.3)
gefunden = [x for x in zurueck if x is not None]
pruefe("Verdeckung lang: Rot wird wiedererkannt", gefunden and gefunden[-1].person["merkmal"] is ROT
       and sp.zustand == zl.FOLGT and sp.id != ziel_id, sp.als_dict())
pruefe("Verdeckung lang: erst nach Bestätigung",
       all(x is None for x in zurueck[:config.ZIEL_BESTAETIGEN_BILDER - 1]), [x and x.id for x in zurueck])

# Kreuzen mit Sperre: dem richtigen bleiben
tr, sp = zl.PersonenTracker(), zl.ZielSperre()
sp.erfassen(None, 0.0)
lauf(tr, sp, [[person(20, ROT)] for _ in range(15)])
szenen = []
for i in range(21):
    t = i * 0.1
    wa, wb = 20.0 - 20.0 * t, -20.0 + 20.0 * t
    szenen.append([person(wb, BLAU)] if abs(wa - wb) < 5 else [person(wa, ROT), person(wb, BLAU)])
res = lauf(tr, sp, szenen, start=1.5)
pruefe("Kreuzen: Ziel ist nie die blaue Person",
       all(x is None or x.person["merkmal"] is ROT for x in res))
pruefe("Kreuzen: am Ende folgt er wieder Rot", res[-1] is not None and res[-1].person["merkmal"] is ROT,
       sp.als_dict())


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


# App-Start (Knopf, ohne Antippen) bei zwei Personen -> abgelehnt
ia, wd, po = interaktion()
po.liste = [person(10, ROT), person(-15, BLAU)]
ia._wechseln(vh.FOLGEN, "Test", 0.0)
v = spielen(ia, po, lambda t: [person(10, ROT), person(-15, BLAU)], 0.0, 0.5)
pruefe("App-Start bei zwei Personen: zurück auf bereit", ia._modus == vh.BEREIT, ia._modus)
pruefe("App-Start bei zwei Personen: fährt nicht",
       all(b is None or (b.vx == 0 and b.omega == 0) for _, _, b in v), [b for _, _, b in v][:3])
pruefe("App-Start bei zwei Personen: Grund für die App",
       "antippen" in ia.zustand()["ziel"]["grund"], ia.zustand()["ziel"])
pruefe("App-Start bei zwei Personen: Meldung",
       any("antippen" in m for m in ia.meldungen_abholen()))

# App-Start mit Auswahl (ID) bei zwei Personen -> folgt genau dieser
ia, wd, po = interaktion()
spielen(ia, po, lambda t: [person(10, ROT), person(-15, BLAU)], 0.0, 0.3)
blau_id = next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is BLAU)
ia._auswahl_id = blau_id
ia._wechseln(vh.FOLGEN, "Test", 0.3)
v = spielen(ia, po, lambda t: [person(10, ROT), person(-15, BLAU)], 0.3, 1.8)
pruefe("App-Start mit Auswahl: folgt", ia._modus == vh.FOLGEN and ia._sperre.id == blau_id,
       ia.zustand()["ziel"])
pruefe("App-Start mit Auswahl: dreht zur gewählten Person (rechts)", v[-1][2].omega < 0, v[-1][2])

# Zweite Person tritt während der Suche ein -> sicher anhalten, nicht folgen
ia, wd, po = interaktion()
folgt_rot(ia, po)
pruefe("Suche: vorher folgt er Rot", ia._sperre.zustand == zl.FOLGT, ia.zustand()["ziel"])
ende = 1.5 + config.FOLGEN_VERLOREN_S + 0.5
spielen(ia, po, lambda t: [], 1.5, ende)
pruefe("Suche: Rot weg -> sucht", ia._modus == vh.SUCHEN, ia._modus)
v = spielen(ia, po, lambda t: [person(-10, BLAU)], ende, ende + config.FOLGEN_SUCHEN_S + 0.5)
pruefe("Suche: folgt Blau nie", all(m in (vh.SUCHEN, vh.BEREIT) for _, m, _ in v),
       sorted({m for _, m, _ in v}))
pruefe("Suche: fährt dabei nie vorwärts", all(b is None or b.vx <= 0 for _, _, b in v))
pruefe("Suche: am Ende sicher angehalten",
       ia._modus == vh.BEREIT and ia._sperre.zustand == zl.SICHER_HALT and wd.stopps >= 1,
       ia.zustand()["ziel"])

# Rot kommt während der Suche zurück, Blau steht daneben -> folgt Rot
ia, wd, po = interaktion()
folgt_rot(ia, po)
ende = 1.5 + config.FOLGEN_VERLOREN_S + 0.5
spielen(ia, po, lambda t: [], 1.5, ende)
spielen(ia, po, lambda t: [person(-12, BLAU), person(15, ROT)], ende, ende + 1.0)
ziel = ia._ziel_waehlen(ende + 1.0)
pruefe("Suche: Rot zurück -> folgt wieder Rot",
       ia._modus == vh.FOLGEN and ziel is not None and ziel["merkmal"] is ROT, (ia._modus, ziel and ziel["winkel"]))

# Zwei gleich gekleidete Personen nach Verlust -> nicht raten
ia, wd, po = interaktion()
folgt_rot(ia, po)
ende = 1.5 + config.FOLGEN_VERLOREN_S + 0.5
spielen(ia, po, lambda t: [], 1.5, ende)
v = spielen(ia, po, lambda t: [person(-15, ROT), person(15, ROT)], ende, ende + 1.5)
pruefe("Gleiche Kleidung: folgt keinem", all(m != vh.FOLGEN for _, m, _ in v[1:]),
       sorted({m for _, m, _ in v}))
pruefe("Gleiche Kleidung: sicher angehalten",
       ia._modus == vh.BEREIT and ia._sperre.zustand == zl.SICHER_HALT, ia.zustand()["ziel"])
pruefe("Gleiche Kleidung: Grund 'nicht eindeutig'", "eindeutig" in ia.zustand()["ziel"]["grund"],
       ia.zustand()["ziel"])

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

# Ausschalten löscht alles Gelernte
ia, wd, po = interaktion()
folgt_rot(ia, po)
ia._ziel_vergessen()
pruefe("Vergessen: kein Ziel, kein Merkmal, keine Tracks",
       ia._sperre.zustand == zl.IDLE and ia._sperre.referenz is None and not ia._tracker.tracks)

# /interaktion/pose: Zustand enthält das Ziel-Feld
z = ia.zustand()
pruefe("Zustand: Feld ziel mit zustand/id/sicherheit/grund",
       set(z["ziel"]) == {"zustand", "id", "sicherheit", "grund"}, z["ziel"])


# --- Bedienung aus der App (Schritt C) -----------------------------------------


class LaeuftNoch:
    """Steht für die laufende Interaktions-Schleife (aktiv == True)."""

    def done(self):
        return False


def track_id(ia, farbe):
    return next(t.id for t in ia._tracker.tracks if t.person["merkmal"] is farbe)


zwei = lambda t: [person(10, ROT), person(-15, BLAU)]

# Antippen: follow mit ziel_id wählt genau diese Person
ia, wd, po = interaktion()
ia._task = LaeuftNoch()
spielen(ia, po, zwei, 0.0, 0.3)
blau_id = track_id(ia, BLAU)
pruefe("Antippen: angenommen", ia.folgen(blau_id) is True)
pruefe("Antippen: Text nennt die Person", f"Person {blau_id}" in ia.zustand()["text"], ia.zustand()["text"])
v = spielen(ia, po, zwei, 0.3, 1.8)
pruefe("Antippen: folgt Blau", ia._modus == vh.FOLGEN and ia._sperre.id == blau_id, ia.zustand()["ziel"])
pruefe("Antippen: dreht zu Blau (rechts)", v[-1][2].omega < 0, v[-1][2])

# Umwählen während des Folgens: Rot antippen ersetzt Blau
rot_id = track_id(ia, ROT)
pruefe("Umwählen: angenommen", ia.folgen(rot_id) is True)
v = spielen(ia, po, zwei, 1.8, 3.3)
pruefe("Umwählen: folgt jetzt Rot", ia._modus == vh.FOLGEN and ia._sperre.id == rot_id, ia.zustand()["ziel"])
pruefe("Umwählen: dreht zu Rot (links)", v[-1][2].omega > 0, v[-1][2])

# Unbekannte ID: abgelehnt, Ziel bleibt
ia.meldungen_abholen()
pruefe("Unbekannte ID: abgelehnt", ia.folgen(999) is False)
pruefe("Unbekannte ID: Ziel bleibt Rot", ia._sperre.id == rot_id, ia.zustand()["ziel"])
pruefe("Unbekannte ID: Meldung", any("nicht mehr im Bild" in m for m in ia.meldungen_abholen()))

# Pause: steht, Ziel bleibt gesperrt und verfolgt
pruefe("Pause: angenommen", ia.pausieren() is True)
pruefe("Pause: Zustand meldet pausiert", ia.zustand()["pausiert"] is True)
# Länger als Verloren + Suchen: Ohne Pause wäre er längst in der Suche.
t0 = 3.3 + config.FOLGEN_VERLOREN_S + config.FOLGEN_SUCHEN_S + 1.0
v = spielen(ia, po, zwei, 3.3, t0)
pruefe("Pause: fährt nicht", all(b is None or (b.vx == 0 and b.vy == 0 and b.omega == 0) for _, _, b in v),
       [b for _, _, b in v if b is not None][:2])
pruefe("Pause: Modus und Ziel bleiben", ia._modus == vh.FOLGEN and ia._sperre.id == rot_id,
       (ia._modus, ia.zustand()["ziel"]))
pruefe("Pause: doppelt pausieren geht nicht", ia.pausieren() is False)

# Weiter: fährt wieder zum selben Ziel
pruefe("Weiter: angenommen", ia.fortsetzen() is True)
v = spielen(ia, po, zwei, t0, t0 + 1.0)
pruefe("Weiter: folgt wieder Rot", ia._modus == vh.FOLGEN and ia._sperre.id == rot_id
       and not ia.zustand()["pausiert"], ia.zustand()["ziel"])
pruefe("Weiter: dreht wieder zu Rot", v[-1][2].omega > 0, v[-1][2])
pruefe("Weiter: ohne Pause abgelehnt", ia.fortsetzen() is False)

# Pause, dann Stopp-Geste: Pause endet mit dem Modus
ia.pausieren()
spielen(ia, po, lambda t: [person(10, ROT, arm="stopp"), person(-15, BLAU)], t0 + 1.0, t0 + 2.5)
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

# Loslassen quittiert ein sicheres Anhalten
ia._sperre.sicher_anhalten("Test", 1.0)
ia._modus = vh.BEREIT
ia.folgen_stop()
pruefe("Loslassen: aus safe_stop wird idle", ia._sperre.zustand == zl.IDLE, ia.zustand()["ziel"])

print()
print("alles bestanden" if not fehler else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
