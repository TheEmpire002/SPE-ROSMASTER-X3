"""Prüft den Fahrtenschreiber (aufzeichnung.py) -- ohne Roboter.

    python Marlon/autonomes_fahren/test_aufzeichnung.py

Geprüft wird: Der Puffer hält nur die letzte Zeit, doppelte Scans und
unveränderte Zustände landen nicht darin, Speichern und Laden ergeben
dasselbe, alte Dateien werden aufgeräumt, nur gültige Dateinamen werden
ausgeliefert, und der Watchdog meldet jeden Befehl vor und nach den Grenzen.

Wichtigster Fall zum Schluss: Eine simulierte Erkunder-Fahrt wird
aufgezeichnet, gespeichert, geladen und einem frischen Erkunder vorgelegt.
Bei gleichem Zufall muss er Takt für Takt dieselben Befehle geben -- sonst
taugt das Nachspielen echter Fahrten nicht zum Prüfen neuer Ausweichlogik.
"""

import math
import pathlib
import random
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import aufzeichnung as az  # noqa: E402
from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.mock_driver import MockDriver  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import hindernisse as hd  # noqa: E402
from rosmaster_assist.protocol import STOP, DriveCommand  # noqa: E402
from rosmaster_assist.safety.watchdog import SafetyWatchdog  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

fehler = 0
TAKT = 1.0 / config.ODOMETRY_HZ
RUNDE = 1.0 / 7.5


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


def scan_bei(zeit, n=360, r=1.5):
    s = Scan([float(i - 180) for i in range(n)], [r + 0.001 * i for i in range(n)])
    s.zeit = zeit
    return s


tmp = pathlib.Path(tempfile.mkdtemp(prefix="aufzeichnung_test_"))

# --- 1. Ringpuffer, doppelte Scans, Zustände ------------------------------------
fs = az.Fahrtenschreiber(puffer_s=1.0, ordner=tmp / "a")
for i in range(31):
    fs.befehl("hand", STOP, STOP, jetzt=i * 0.1)
e = fs.ereignisse()
pruefe("Puffer: nur die letzte Sekunde", e[0]["t"] >= 2.0 - 1e-9 and e[-1]["t"] == 3.0,
       f"{e[0]['t']:.1f}–{e[-1]['t']:.1f} s")

fs = az.Fahrtenschreiber(puffer_s=60.0, ordner=tmp / "a")
s = scan_bei(5.0)
pruefe("Scan: neuer wird aufgenommen", fs.scan(s, jetzt=5.0))
pruefe("Scan: derselbe nicht noch einmal", not fs.scan(s, jetzt=5.05))
pruefe("Scan: None wird übergangen", not fs.scan(None))
pruefe("Zustand: Änderung aufgenommen", fs.zustand(jetzt=5.1, erkunder="fährt"))
pruefe("Zustand: gleicher nicht noch einmal", not fs.zustand(jetzt=5.2, erkunder="fährt"))
pruefe("Zustand: neue Änderung aufgenommen", fs.zustand(jetzt=5.3, erkunder=None))
st = fs.status()
pruefe("Status zählt Ereignisse und Scans", st["ereignisse"] == 3 and st["scans"] == 1, st)

# --- 2. Speichern und Laden -----------------------------------------------------
fs.kopf = {"antrieb": "test"}
fs.odo((0.1, 0.0, 0.2), (1.0, 2.0, 30.0), DriveCommand(0.1, 0.0, 0.2), True, jetzt=5.4)
pfad = fs.speichern("Tür im Flur")
pruefe("Speichern: gültiger Dateiname", az.gueltiger_name(pfad.name), pfad.name)
pruefe("Speichern: keine .tmp-Datei übrig", not list(pfad.parent.glob("*.tmp")))
kopf, ev = az.laden(pfad)
pruefe("Laden: Kopf vollständig",
       kopf["version"] == az.VERSION and kopf["notiz"] == "Tür im Flur"
       and kopf["antrieb"] == "test" and "AUTONOM_MAX_SPEED" in kopf["config"], kopf.get("notiz"))
pruefe("Laden: alle Ereignisse", len(ev) == 4, len(ev))
pruefe("Laden: Zeiten ab erstem Ereignis", ev[0]["t"] == 0.0 and ev[0]["zeit"] == 0.0,
       (ev[0]["t"], ev[0].get("zeit")))
s2 = az.scan_aus(ev[0])
abw_w = max(abs(a - b) for a, b in zip(s.winkel, s2.winkel))
abw_r = max(abs(a - b) for a, b in zip(s.entfernungen, s2.entfernungen))
pruefe("Laden: Scan bis auf Rundung gleich", abw_w <= 0.05 and abw_r <= 0.0005,
       f"{abw_w:.3f}°, {abw_r * 1000:.2f} mm")
pruefe("Laden: Odometrie erhalten", ev[3]["v"] == [0.1, 0.0, 0.2] and ev[3]["misst"] is True)

# --- 3. Aufräumen und Dateinamen ------------------------------------------------
fs = az.Fahrtenschreiber(puffer_s=60.0, ordner=tmp / "b", max_dateien=3)
fs.stop(jetzt=1.0)
gespeichert = [fs.speichern() for _ in range(5)]
liste = fs.liste()
pruefe("Aufräumen: nur die neuesten 3", len(liste) == 3, [d["name"] for d in liste])
pruefe("Aufräumen: letzte Datei bleibt", gespeichert[-1].exists())
fs2 = az.Fahrtenschreiber(puffer_s=60.0, ordner=tmp / "b2")
fs2.stop(jetzt=1.0)
a, b = fs2.speichern(), fs2.speichern()
pruefe("Gleiche Sekunde: eigener Name, nichts überschrieben", a != b and a.exists() and b.exists(),
       (a.name, b.name))
pruefe("Name: gültig", az.gueltiger_name("fahrt_20261103_141500.jsonl.gz"))
pruefe("Name: mit Zähler gültig", az.gueltiger_name("fahrt_20261103_141500_2.jsonl.gz"))
for falsch in ("../config.py", "..\\fahrt_20261103_141500.jsonl.gz", "fahrt_x.jsonl.gz",
               "fahrt_20261103_141500.jsonl.gz.tmp", ""):
    pruefe(f"Name: abgelehnt {falsch!r}", fs.datei(falsch) is None)
pruefe("Name: gültig, aber nicht vorhanden", fs.datei("fahrt_20000101_000000.jsonl.gz") is None)
pruefe("Name: vorhanden", fs.datei(gespeichert[-1].name) == gespeichert[-1])

# --- 4. Watchdog meldet Befehle -------------------------------------------------
wd = SafetyWatchdog(MockDriver())
fs = az.Fahrtenschreiber(puffer_s=60.0, ordner=tmp / "c")
wd.aufzeichnung = fs
wd.submit(DriveCommand(5.0, 0.0, 0.0), quelle="hand")
wd.stop()
ev = fs.ereignisse()
pruefe("Watchdog: Befehl aufgezeichnet", len(ev) == 2 and ev[0]["art"] == "befehl", ev[:1])
pruefe("Watchdog: soll unverändert, safe begrenzt",
       ev[0]["soll"][0] == 5.0 and ev[0]["safe"][0] <= config.MAX_SPEED_X, ev[0])
pruefe("Watchdog: Halt aufgezeichnet", ev[1]["art"] == "stop")


class KaputterSchreiber:
    def befehl(self, *a):
        raise RuntimeError("Absicht")

    def stop(self):
        raise RuntimeError("Absicht")


wd.aufzeichnung = KaputterSchreiber()
try:
    wd.submit(DriveCommand(0.1, 0.0, 0.0), quelle="hand")
    wd.stop()
    ok = True
except Exception as exc:  # noqa: BLE001
    ok = repr(exc)
pruefe("Watchdog: kaputter Schreiber stört das Fahren nicht", ok is True, ok)

# --- 5. Wiedergabe und Abschnitte -----------------------------------------------
w = az.Wiedergabe([
    {"t": 0.1, "art": "scan", "zeit": 0.1, "w": [0], "r": [1000]},
    {"t": 0.2, "art": "odo", "v": [0.2, 0.0, 0.0], "pose": [0, 0, 0], "ist": [0, 0, 0], "misst": True},
])
pruefe("Wiedergabe: vor dem ersten Scan nichts", w.latest() is None and not w.misst_tatsaechlich)
w.bis(0.15)
pruefe("Wiedergabe: Scan ab seiner Zeit", w.latest() is not None and w.geschwindigkeit() == (0.0, 0.0, 0.0))
w.bis(0.2)
pruefe("Wiedergabe: Odometrie ab ihrer Zeit", w.geschwindigkeit() == (0.2, 0.0, 0.0) and w.misst_tatsaechlich)
try:
    w.bis(0.1)
    rueck = False
except ValueError:
    rueck = True
pruefe("Wiedergabe: nur vorwärts", rueck)
ab = az.abschnitte([
    {"t": 0.0, "art": "zustand", "erkunder": None},
    {"t": 1.0, "art": "zustand", "erkunder": "fährt"},
    {"t": 3.0, "art": "zustand", "erkunder": None},
    {"t": 4.0, "art": "zustand", "erkunder": "dreht"},
    {"t": 5.0, "art": "odo"},
])
pruefe("Abschnitte: Erkunder-Zeiträume", ab == [(1.0, 3.0), (4.0, 5.0)], ab)


# --- 6. Erkunder-Fahrt aufzeichnen und nachspielen ------------------------------
class Welt:
    """Rechteckiger Raum mit runden Hindernissen (wie in test_erkunder.py)."""

    def __init__(self, raum, kreise):
        self.raum, self.kreise = raum, kreise
        self.x = self.y = self.theta = 0.0
        self.v_ist = (0.0, 0.0, 0.0)

    def strahl(self, richtung):
        dx, dy = math.cos(richtung), math.sin(richtung)
        x0, x1, y0, y1 = self.raum
        best = float("inf")
        for grenze, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9 and (grenze - start) / d > 0:
                best = min(best, (grenze - start) / d)
        for cx, cy, r in self.kreise:
            mx, my = self.x - cx, self.y - cy
            b = mx * dx + my * dy
            disk = b * b - (mx * mx + my * my - r * r)
            if disk >= 0 and -b - math.sqrt(disk) > 0:
                best = min(best, -b - math.sqrt(disk))
        return best

    def scan(self, zeit):
        winkel, entf = [], []
        for i in range(360):
            r = self.strahl(self.theta + math.radians(i))
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                winkel.append(float(i if i <= 180 else i - 360))
                entf.append(r)
        # Gerundet wie im Fahrtenschreiber: Der Original-Erkunder soll genau
        # das sehen, was die Datei später wiedergibt.
        w, r = az.scan_runden(Scan(winkel, entf))
        return az.scan_aus({"w": w, "r": r, "zeit": zeit, "dauer": round(RUNDE, 4)})

    def bewegen(self, befehl, dt):
        if befehl is None:
            self.v_ist = (0.0, 0.0, 0.0)
            return
        self.v_ist = tuple(round(v, 4) for v in (befehl.vx, befehl.vy, befehl.omega))
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (befehl.vx * c - befehl.vy * s) * dt
        self.y += (befehl.vx * s + befehl.vy * c) * dt
        self.theta += befehl.omega * dt


class LidarAttrappe:
    letzter = None

    def latest(self):
        return self.letzter


class OdometrieAttrappe:
    misst_tatsaechlich = True

    def __init__(self, welt):
        self.welt = welt

    def geschwindigkeit(self):
        return self.welt.v_ist


class OhneAntrieb:
    # Die Fortschrittswache liest den zuletzt angenommenen Befehl mit.
    current_command = DriveCommand()

    def submit(self, befehl, quelle=""):
        self.current_command = befehl
        return befehl

    def stop(self):
        pass


random.seed(7)
welt = Welt((-1.5, 2.5, -1.2, 1.4), [(1.0, 0.3, 0.25), (-0.6, -0.7, 0.2)])
lidar = LidarAttrappe()
erk = ex.Erkunder(OhneAntrieb(), lidar, hd.Hinderniserkennung(), None, odometry=OdometrieAttrappe(welt))
fs = az.Fahrtenschreiber(puffer_s=1000.0, ordner=tmp / "d")
erk._beginnen(0.0)
original = []
naechste_runde = 0.0
for k in range(int(30.0 / TAKT)):
    # Gerundete Uhr: In der Datei stehen Zeiten auf 0,1 ms.
    t = round(k * TAKT, 4)
    if t >= naechste_runde:
        lidar.letzter = welt.scan(t)
        fs.scan(lidar.letzter, jetzt=t)
        naechste_runde += RUNDE
    fs.odo(welt.v_ist, (welt.x, welt.y, math.degrees(welt.theta)), STOP, True, jetzt=t)
    fs.zustand(jetzt=t, erkunder=erk._zustand)
    befehl = erk._schritt(t)
    original.append(None if befehl is None else [round(v, 4) for v in (befehl.vx, befehl.vy, befehl.omega)])
    if befehl is None:
        break
    fs.befehl("autonom", befehl, befehl, jetzt=t)
    welt.bewegen(befehl, TAKT)
zustaende = {e["erkunder"] for e in fs.ereignisse() if e["art"] == "zustand"}
pruefe("Simulation: Erkunder fährt und weicht aus", len(zustaende) >= 2, zustaende)

_, ev = az.laden(fs.speichern("Simulation"))


def bauen(wiedergabe):
    return ex.Erkunder(OhneAntrieb(), wiedergabe, hd.Hinderniserkennung(), None, odometry=wiedergabe)


random.seed(7)
takte = az.erkunder_nachspielen(ev, bauen)
gleich = sum(1 for t in takte if t["aufgezeichnet"] == t["nachgespielt"])
pruefe("Nachspielen: jeder Takt kommt dran", len(takte) == len([b for b in original if b is not None]),
       f"{len(takte)} von {len(original)}")
pruefe("Nachspielen: Takt für Takt gleiche Befehle", gleich == len(takte), f"{gleich}/{len(takte)}")
erster_anders = next((t for t in takte if t["aufgezeichnet"] != t["nachgespielt"]), None)
if erster_anders:
    print("       erster Unterschied:", erster_anders)

print()
print("Alle Prüfungen bestanden." if fehler == 0 else f"{fehler} Prüfung(en) fehlgeschlagen.")
sys.exit(1 if fehler else 0)
