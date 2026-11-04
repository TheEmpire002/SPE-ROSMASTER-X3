"""Prüft die Testläufe der Seite /tests (testlauf.py), ohne Roboter.

    python Marlon/diagnose/test_testlauf.py

Geprüft mit Attrappen für Pose, Kamera, Lidar und Mikrofon:

* ein kurzer Sichttest legt Ordner, ergebnis.json und Bilder an, die
  Zusammenfassung stimmt mit den Attrappen überein;
* ein Mikrofontest schreibt ton.wav, sammelt die erkannten Sätze und sperrt
  währenddessen die Befehle;
* Notiz, ZIP, Liste, Schutz gegen fremde Pfade (../);
* Hochladen ohne Token: klare Meldung, nichts passiert;
* Hochladen in ein lokales Repo (file://): erster Commit ohne Eltern, zweiter
  mit; HEAD und Index des Repos bleiben unberührt; das Token steht nie in
  der Befehlszeile.
"""

import base64
import io
import json
import pathlib
import queue
import subprocess
import sys
import tempfile
import time
import wave
import zipfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config, testlauf as tl  # noqa: E402
from rosmaster_assist.sensors import pose as ps  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


# Kurz halten: Der echte Vorlauf (Zeit zum Hinlegen) ist für einen Test zu lang.
config.TESTS_VORLAUF_S = 0.6
config.TESTS_NACHLAUF_S = 0.2
config.TESTS_STIMME_WARTEN_S = 0.5

# Kopf oben, Hüfte und Füße fast auf gleicher Höhe daneben: eine liegende Person.
LIEGEND_KP = [[0.2, 0.6, 0.9]] * 5 + [[0.3, 0.6, 0.9], [0.3, 0.62, 0.9]] + [[0.4, 0.6, 0.9]] * 4 \
    + [[0.55, 0.62, 0.9], [0.55, 0.64, 0.9]] + [[0.7, 0.62, 0.9]] * 4


class Pose:
    """Attrappe: eine liegende Person, merkt sich, wer sie eingeschaltet hat."""

    verfuegbar = True
    fov_deg = 45.0

    def __init__(self):
        self.nutzer = set()
        self.person = ps._person([0.15, 0.5, 0.75, 0.7], LIEGEND_KP, 0.9, 4 / 3, 45.0)

    def aktivieren(self, an, nutzer):
        (self.nutzer.add if an else self.nutzer.discard)(nutzer)

    def personen(self):
        return [self.person]

    def status(self):
        return {"fps": 10.0, "bilder": 5, "bild_alter_s": 0.05, "veraltete_bilder": 0, "aktiv": True}


class Kamera:
    def latest_mit_zeit(self):
        return (b"\xff\xd8kein echtes jpeg", "image/jpeg"), time.monotonic()

    def status(self):
        return {"fps_ist": 10.0, "fps_soll": 10}


class Hog:
    """Attrappe mit Nutzerzählung wie die echte Personenerkennung."""

    def __init__(self):
        self.nutzer = set()

    @property
    def an(self):
        return bool(self.nutzer)

    def status(self):
        return {"aktiv": self.an}

    def aktivieren(self, an, wer="erkunder"):
        (self.nutzer.add if an else self.nutzer.discard)(wer)

    def boxen(self):
        return [{"x": 0.2, "y": 0.1, "w": 0.3, "h": 0.8}]


class Sprache:
    """Attrappe: liefert Ton in die Ton-Schlange und einen erkannten Befehl."""

    verfuegbar = True
    ton_rate = 16000

    def __init__(self):
        self.abos = []
        self.start = time.time()
        self.satz_zeit = None  # wie echt: ein erkannter Satz behält seine Zeit

    def ton_abonnieren(self):
        q = queue.Queue()
        q.put(b"\x01\x00" * 1600)
        self.abos.append(q)
        return q

    def ton_abmelden(self, q):
        self.abos.remove(q)

    def live(self):
        self.satz_zeit = self.satz_zeit or time.time()
        return {"pegel": 0.3, "teil": "robi", "taub": False, "zustand": "hört",
                "ereignisse": [
                    {"zeit": self.start - 100, "text": "alt", "ergebnis": "befehl", "befehl": "stop"},
                    {"zeit": self.satz_zeit, "text": "robi stopp", "ergebnis": "befehl", "befehl": "stop",
                     "konfidenz": 0.9},
                ]}


tmp = pathlib.Path(tempfile.mkdtemp(prefix="testlauf_"))
pose, kamera, hog = Pose(), Kamera(), Hog()
lauf = tl.Testlauf(pose=pose, personen=hog, kamera=kamera, kameras={"personen": kamera},
                   stimme=None, sprache=None, mock=True, ordner=tmp / "laeufe")

# --- Katalog ---------------------------------------------------------------------
kat = lauf.katalog()
ids = [t["id"] for t in kat["tests"]]
pruefe("Katalog hat alle drei Kategorien",
       {t["kategorie"] for t in kat["tests"]} == set(kat["kategorien"]), sorted(kat["kategorien"]))
pruefe("Test-IDs passen zum Ordnernamen", all(tl._NAME.match(f"2026-01-01_00-00-00_{i}") for i in ids))
pruefe("unbekannter Test wird abgelehnt", lauf.start("gibts_nicht") is not None)
pruefe("Mikrofontest ohne Spracherkennung abgelehnt", "Sprache" in (lauf.start("mikro_pegel") or ""))

# --- Sichttest (kurz, direkt im Thread des Tests) --------------------------------
test = {"id": "personen_probe", "titel": "Probe", "kategorie": "personen", "art": "personen",
        "anleitung": "nur Test", "phasen": [
            tl._phase("liegt", "Hinlegen", "Hinlegen", 0.8, erwartet="liegt"),
            tl._phase("leer", "Weggehen", "Weggehen", 0.6, erwartet="niemand"),
        ]}
lauf._lauf(test)
laeufe = lauf.liste()
pruefe("ein Lauf in der Liste", len(laeufe) == 1, [e["name"] for e in laeufe])
name = laeufe[0]["name"]
d = lauf.ordner / name
daten = json.loads((d / "ergebnis.json").read_text(encoding="utf-8"))
pruefe("Ordner heißt nach Datum, Uhrzeit und Test", tl._NAME.match(name) and name.endswith("_personen_probe"), name)
pruefe("beide Phasen gespeichert", [p["name"] for p in daten["phasen"]] == ["liegt", "leer"])
pruefe("nicht abgebrochen", daten["abgebrochen"] is False)
pruefe("Proben gesammelt", len(daten["proben"]) >= 5, len(daten["proben"]))
z_liegt, z_leer = (p["zusammenfassung"] for p in daten["phasen"])
pruefe("liegende Person erkannt", z_liegt["liegend_anteil"] == 1.0, z_liegt)
pruefe("liegend: 100 % richtig", z_liegt["richtig"] == 1.0, z_liegt["kurz"])
pruefe("leerer Raum: Fehltreffer zählen als falsch", z_leer["richtig"] == 0.0, z_leer["kurz"])
pruefe("HOG mitgemessen", z_liegt["hog_mit_person"] == 1.0)
pruefe("Bilder gespeichert", 1 <= len(daten["bilder"]) <= config.TESTS_MAX_BILDER
       and all((d / b["datei"]).is_file() for b in daten["bilder"]), len(daten["bilder"]))
pruefe("Fehltreffer im leeren Raum fotografiert", any(b["grund"] == "fehltreffer" for b in daten["bilder"]))
pruefe("Skelett zum Bild gespeichert", all(b["personen"] and b["personen"][0]["kp"] for b in daten["bilder"]))
pruefe("Einstellungen gespeichert", "HEALTH_LIEGEND_GRAD" in daten["einstellungen"])
pruefe("Pose danach wieder freigegeben", pose.nutzer == set(), pose.nutzer)
pruefe("HOG danach wieder aus", hog.an is False)
pruefe("Status danach nicht aktiv", lauf.status()["aktiv"] is False)

# HOG war schon an (z. B. Erkunder): bleibt danach an
hog.aktivieren(True, "erkunder")
test_kurz = {**test, "id": "personen_kurz", "phasen": [tl._phase("a", "A", "A", 0.3, erwartet="eine_person")]}
time.sleep(1.0)  # neuer Ordnername (Sekunden)
lauf._lauf(test_kurz)
pruefe("fremd eingeschalteter HOG bleibt an", hog.an is True, hog.nutzer)
hog.aktivieren(False, "erkunder")


class HogErkunderStartet(Hog):
    """Der Erkunder schaltet HOG erst ein, während der Test schon läuft."""

    def boxen(self):
        self.aktivieren(True, "erkunder")
        return super().boxen()


hog2 = HogErkunderStartet()
lauf2 = tl.Testlauf(pose=Pose(), personen=hog2, kamera=kamera, kameras={"personen": kamera},
                    stimme=None, sprache=None, mock=True, ordner=tmp / "laeufe2")
lauf2._lauf(test_kurz)
pruefe("während des Tests gestarteter Erkunder behält HOG", hog2.nutzer == {"erkunder"}, hog2.nutzer)

# Die echte Personenerkennung zählt ihre Nutzer genauso
from rosmaster_assist.sensors import personen as pe  # noqa: E402

echt = pe.Personenerkennung(None)
echt.aktivieren(True)  # Erkunder (Standardname, wie in explorer.py)
echt.aktivieren(True, "tests")
echt.aktivieren(False, "tests")
pruefe("Personenerkennung: Ende des Tests lässt Erkunder rechnen", echt._aktiv.is_set())
echt.aktivieren(False)
pruefe("Personenerkennung: ohne Nutzer aus", not echt._aktiv.is_set())

# --- Notiz, ZIP, Pfadschutz ---------------------------------------------------------
pruefe("Notiz gespeichert", lauf.notiz(name, "Licht von hinten") is None)
pruefe("Notiz in der Liste", next(e for e in lauf.liste() if e["name"] == name)["notiz"] == "Licht von hinten")
pruefe("Notiz zu fremdem Pfad abgelehnt", lauf.notiz("../../etc", "x") is not None)
roh = lauf.zip_bauen(name)
namen_zip = zipfile.ZipFile(io.BytesIO(roh)).namelist() if roh else []
pruefe("ZIP enthält ergebnis.json und Bilder", f"{name}/ergebnis.json" in namen_zip
       and any("/bilder/" in n for n in namen_zip), len(namen_zip))
pruefe("ZIP zu fremdem Pfad: nichts", lauf.zip_bauen("..") is None and lauf.zip_bauen("../laeufe") is None)

# --- Mikrofontest ---------------------------------------------------------------------
sprache = Sprache()
mikro = tl.Testlauf(sprache=sprache, mock=True, ordner=tmp / "mikro")
test_m = {"id": "mikro_probe", "titel": "Probe", "kategorie": "mikrofon", "art": "mikro",
          "anleitung": "nur Test", "phasen": [
              tl._phase("stopp", "Stopp sagen", "Sag: Robi stopp", 0.5, erwartet="stop", satz="Robi stopp")]}
pruefe("Mikrofontest startet", mikro.start("mikro_pegel") is None)
time.sleep(0.2)
pruefe("während des Mikrofontests Befehle gesperrt", mikro.befehle_sperren is True)
mikro.stop()
mikro._thread.join(timeout=5)
pruefe("Abbruch beendet den Lauf", not mikro.aktiv and mikro.befehle_sperren is False)
abgebr = mikro.liste()
pruefe("abgebrochener Lauf ist markiert", abgebr and abgebr[0]["abgebrochen"] is True)
time.sleep(1.0)
mikro._stop.clear()
mikro._lauf(test_m)
m = next(e for e in mikro.liste() if e["test"] == "mikro_probe")
dm = mikro.ordner / m["name"]
dm_daten = json.loads((dm / "ergebnis.json").read_text(encoding="utf-8"))
with wave.open(str(dm / "ton.wav")) as w:
    pruefe("ton.wav geschrieben", w.getframerate() == 16000 and w.getnframes() > 0, w.getnframes())
pruefe("Ton-Abo wieder abgemeldet", sprache.abos == [])
pruefe("nur Sätze seit Laufbeginn", [e["text"] for e in dm_daten["ereignisse"]] == ["robi stopp"],
       dm_daten["ereignisse"])
z = dm_daten["phasen"][0]["zusammenfassung"]
pruefe("Befehl erkannt", z["treffer"] is True, z["kurz"])
pruefe("Pegel gemessen", z["pegel"] and z["pegel"]["median"] == 0.3)

# --- Hochladen ohne Token -------------------------------------------------------------
config.TESTS_GIT_TOKEN_DATEI = str(tmp / "kein_token")
r = lauf.hochladen()
pruefe("ohne Token: Fehler mit Hinweis auf ZIP", not r["ok"] and "ZIP" in r["fehler"], r["fehler"])
pruefe("ohne Token: nichts als hochgeladen markiert", not any(e["hochgeladen"] for e in lauf.liste()))
pruefe("unbekannter Lauf beim Hochladen abgelehnt", not lauf.hochladen(["../x"])["ok"])
pruefe("Status meldet fehlendes Token", lauf.status()["token"] is False)


# --- Hochladen in ein lokales Repo ---------------------------------------------------
def git(repo, *args):
    return subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=True).stdout.strip()


repo = tmp / "repo"
ziel = tmp / "ziel.git"
repo.mkdir()
git(repo, "init", "-q")
git(repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "Start")
git(tmp, "init", "-q", "--bare", str(ziel))
kopf_vorher = git(repo, "rev-parse", "HEAD")
(repo / "datei.txt").write_text("in Arbeit\n")  # ungespeicherte Arbeit darf nicht verschwinden
git(repo, "add", "datei.txt")
index_vorher = git(repo, "diff", "--cached", "--name-only")

token_datei = tmp / "github_token"
token_datei.write_text("geheim123\n")
config.TESTS_GIT_TOKEN_DATEI = str(token_datei)
config.TESTS_GIT_URL = ziel.as_uri()
tl.git_wurzel = lambda _ordner: repo  # nicht ins echte Projekt-Repo schreiben
pruefe("Status meldet Token", lauf.status()["token"] is True)

r = lauf.hochladen([name])
pruefe("erstes Hochladen klappt", r["ok"] and r["hochgeladen"] == [name], r)
baum = git(ziel, "ls-tree", "-r", "--name-only", "testdaten").splitlines()
pruefe("liegt unter tests/<name>/", f"tests/{name}/ergebnis.json" in baum, baum[:3])
pruefe("Bilder mit hochgeladen", any(p.startswith(f"tests/{name}/bilder/") for p in baum))
pruefe("hochgeladen.json bleibt lokal", not any(p.endswith("hochgeladen.json") for p in baum))
pruefe("erster Commit ohne Eltern", git(ziel, "rev-list", "--count", "testdaten") == "1")
pruefe("Lauf als hochgeladen markiert", next(e for e in lauf.liste() if e["name"] == name)["hochgeladen"])

r2 = lauf.hochladen()  # alle neuen: nur der zweite Lauf
pruefe("„alle neuen“ lädt nur den Rest", r2["ok"] and len(r2["hochgeladen"]) == 1 and name not in r2["hochgeladen"],
       r2["hochgeladen"])
baum2 = git(ziel, "ls-tree", "-r", "--name-only", "testdaten").splitlines()
pruefe("zweiter Commit behält den ersten Lauf", f"tests/{name}/ergebnis.json" in baum2
       and any(r2["hochgeladen"][0] in p for p in baum2))
pruefe("zweiter Commit hat Eltern", git(ziel, "rev-list", "--count", "testdaten") == "2")
pruefe("nichts Neues: ok ohne Commit", lauf.hochladen() == {"ok": True, "hochgeladen": [], "fehler": None,
                                                            "commit": None})
lauf.notiz(name, "nachgetragen")
pruefe("Notiz macht den Lauf wieder neu",
       not next(e for e in lauf.liste() if e["name"] == name)["hochgeladen"])

pruefe("HEAD des Repos unverändert", git(repo, "rev-parse", "HEAD") == kopf_vorher)
pruefe("Index des Repos unverändert", git(repo, "diff", "--cached", "--name-only") == index_vorher)
pruefe("Arbeitsdatei noch da", (repo / "datei.txt").read_text() == "in Arbeit\n")
pruefe("Repo bleibt auf seinem Branch", git(repo, "symbolic-ref", "-q", "HEAD") in ("refs/heads/master",
                                                                                     "refs/heads/main"))

# --- Token-Schutz ---------------------------------------------------------------------
auth = tl._auth("https://github.com/a/b.git", "geheim123")
pruefe("Token nur als Kopfzeile in der Umgebung", auth.get("GIT_CONFIG_KEY_0") == "http.extraHeader"
       and base64.b64encode(b"x-access-token:geheim123").decode() in auth["GIT_CONFIG_VALUE_0"])
pruefe("kein Token an file:// oder http://", tl._auth("http://x/y.git", "geheim123") == {})
pruefe("ssh-Adresse wird zu https", tl.https_url("git@github.com:a/b.git") == "https://github.com/a/b.git")
pruefe("https-Adresse bleibt", tl.https_url("https://github.com/a/b.git") == "https://github.com/a/b.git")
try:
    tl._git(repo, ["push", "https://nicht-da.invalid/x.git", "HEAD:refs/heads/x"],
            env=tl._auth("https://nicht-da.invalid/x.git", "geheim123"))
    pruefe("Push an falsche Adresse scheitert", False)
except tl.GitFehler as e:
    pruefe("Fehlermeldung enthält das Token nicht", "geheim123" not in str(e), str(e))

print()
print("alles bestanden" if not fehler else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
