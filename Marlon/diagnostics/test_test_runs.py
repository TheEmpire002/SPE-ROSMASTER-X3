"""Tests the test runs of the /tests page (robot/src/rosmaster_assist/test_runs.py) without a robot.

    python Marlon/diagnostics/test_test_runs.py

Checked with dummies for pose, camera, lidar and microphone:

* a short sight test creates folder, ergebnis.json and images, the summary
  matches the dummies, and meta (code commit, settings, definition) is stored
* a microphone test writes ton.wav, collects recognized sentences and blocks
  commands meanwhile
* note, zip, list, guard against foreign paths (../)
* health records are stored as their own runs (typ "health")
* upload without a token: clear message, nothing happens
* upload into a local bare repo (file://): first commit without parent,
  later ones with; index.json and README.md in the branch; HEAD and index
  of the repo stay untouched; the token never appears in an error
* background auto upload, skipped without a token, errors recorded per run
  and retried
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

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist import test_runs as tl  # noqa: E402
from rosmaster_assist.sensors import pose as ps  # noqa: E402

failures = 0


def check(name, condition, result=""):
    global failures
    print(("ok  " if condition else "FAIL"), name, result)
    failures += not condition


# keep it short: the real lead time (time to lie down) is too long for a test
config.TESTS_LEAD_S = 0.6
config.TESTS_TAIL_S = 0.2
config.TESTS_VOICE_WAIT_S = 0.5
# earlier sections must not upload on their own; switched on further down
config.TESTS_GIT_AUTO_UPLOAD = False

tmp = pathlib.Path(tempfile.mkdtemp(prefix="test_runs_"))
config.TESTS_GIT_TOKEN_FILE = str(tmp / "no_token")
config.TESTS_GIT_URL = None

# head at the top, hips and feet at almost the same height next to it: a lying person
LYING_KP = [[0.2, 0.6, 0.9]] * 5 + [[0.3, 0.6, 0.9], [0.3, 0.62, 0.9]] + [[0.4, 0.6, 0.9]] * 4 \
    + [[0.55, 0.62, 0.9], [0.55, 0.64, 0.9]] + [[0.7, 0.62, 0.9]] * 4


class Pose:
    """Dummy: one lying person, remembers who switched it on."""

    available = True
    fov_deg = 45.0

    def __init__(self):
        self.users = set()
        self.person = ps._person([0.15, 0.5, 0.75, 0.7], LYING_KP, 0.9, 4 / 3, 45.0)

    def activate(self, an, who):
        (self.users.add if an else self.users.discard)(who)

    def people(self):
        return [self.person]

    def status(self):
        return {"fps": 10.0, "bilder": 5, "bild_alter_s": 0.05, "veraltete_bilder": 0, "aktiv": True}


class Camera:
    def latest_with_time(self):
        return (b"\xff\xd8not a real jpeg", "image/jpeg"), time.monotonic()

    def status(self):
        return {"fps_ist": 10.0, "fps_soll": 10}


class Hog:
    """Dummy with user counting like the real person detection."""

    def __init__(self):
        self.users = set()

    @property
    def on(self):
        return bool(self.users)

    def status(self):
        return {"aktiv": self.on}

    def activate(self, an, who="explorer"):
        (self.users.add if an else self.users.discard)(who)

    def boxes(self):
        return [{"x": 0.2, "y": 0.1, "w": 0.3, "h": 0.8}]


class Speech:
    """Dummy: delivers audio into the sound queue and one recognized command."""

    available = True
    sound_rate = 16000

    def __init__(self):
        self.subscriptions = []
        self.start = time.time()
        self.sentence_time = None  # like the real one: a recognized sentence keeps its time

    def sound_subscribe(self):
        q = queue.Queue()
        q.put(b"\x01\x00" * 1600)
        self.subscriptions.append(q)
        return q

    def sound_unsubscribe(self, q):
        self.subscriptions.remove(q)

    def live(self):
        self.sentence_time = self.sentence_time or time.time()
        return {"pegel": 0.3, "teil": "robi", "taub": False, "zustand": "hört",
                "ereignisse": [
                    {"zeit": self.start - 100, "text": "alt", "ergebnis": "befehl", "befehl": "stop"},
                    {"zeit": self.sentence_time, "text": "robi stopp", "ergebnis": "befehl", "befehl": "stop",
                     "konfidenz": 0.9},
                ]}


def find(runs, name):
    return next(e for e in runs.list_runs() if e["name"] == name)


def wait_for_auto_upload(runs, timeout=30.0):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        t = runs._auto_thread
        if t is None or not t.is_alive():
            return True
        time.sleep(0.1)
    return False


pose, camera, hog = Pose(), Camera(), Hog()
runs = tl.TestRuns(pose=pose, people=hog, camera=camera, cameras={"personen": camera},
                   voice=None, speech=None, mock=True, folder=tmp / "runs")

# --- catalog ---------------------------------------------------------------------
cat = runs.catalog()
ids = [t["id"] for t in cat["tests"]]
check("catalog has all three categories",
      {t["kategorie"] for t in cat["tests"]} == set(cat["kategorien"]), sorted(cat["kategorien"]))
check("test ids fit the folder name", all(tl._NAME.match(f"2026-01-01_00-00-00_{i}") for i in ids))
check("unknown test rejected", runs.start("does_not_exist") is not None)
check("microphone test without speech rejected", "Sprach" in (runs.start("mikro_pegel") or ""))
check("default folder of a mock run gets _mock",
      tl.TestRuns(mock=True).folder.name == pathlib.Path(config.TESTS_FOLDER).name + "_mock")

# --- sight test (short, directly in the test's thread) ---------------------------
test = {"id": "personen_probe", "titel": "Probe", "kategorie": "personen", "art": "personen",
        "anleitung": "nur Test", "phasen": [
            tl._phase("liegt", "Hinlegen", "Hinlegen", 0.8, erwartet="liegt"),
            tl._phase("leer", "Weggehen", "Weggehen", 0.6, erwartet="niemand"),
        ]}
runs._run(test)
listed = runs.list_runs()
check("one run in the list", len(listed) == 1, [e["name"] for e in listed])
name = listed[0]["name"]
d = runs.folder / name
data = json.loads((d / "ergebnis.json").read_text(encoding="utf-8"))
check("folder named after date, time and test", bool(tl._NAME.match(name)) and name.endswith("_personen_probe"),
      name)
check("both phases stored", [p["name"] for p in data["phasen"]] == ["liegt", "leer"])
check("not aborted", data["abgebrochen"] is False)
check("samples collected", len(data["proben"]) >= 5, len(data["proben"]))
z_lying, z_empty = (p["zusammenfassung"] for p in data["phasen"])
check("lying person detected", z_lying["liegend_anteil"] == 1.0, z_lying)
check("lying: 100 % correct", z_lying["richtig"] == 1.0, z_lying["kurz"])
check("empty room: false hits count as wrong", z_empty["richtig"] == 0.0, z_empty["kurz"])
check("hog measured too", z_lying["hog_mit_person"] == 1.0)
check("images stored", 1 <= len(data["bilder"]) <= config.TESTS_MAX_IMAGES
      and all((d / b["datei"]).is_file() for b in data["bilder"]), len(data["bilder"]))
check("false hit in the empty room photographed", any(b["grund"] == "fehltreffer" for b in data["bilder"]))
check("skeleton stored with the image", all(b["personen"] and b["personen"][0]["kp"] for b in data["bilder"]))
check("settings stored", "HEALTH_LYING_DEG" in data["einstellungen"])
check("pose released afterwards", pose.users == set(), pose.users)
check("hog off afterwards", hog.on is False)
check("status not active afterwards", runs.status()["aktiv"] is False)

# --- meta: the run describes itself -----------------------------------------------
meta = data.get("meta") or {}
check("typ is test", data.get("typ") == "test")
check("meta has begin and end", bool(meta.get("beginn")) and bool(meta.get("ende")), meta.get("ende"))
commit = meta.get("code_commit")
check("meta has the code commit", commit is None or (len(commit) == 40 and int(commit, 16) >= 0), commit)
check("meta has the code commit (repo found)", isinstance(commit, str), commit)
check("meta has the full settings", meta.get("einstellungen", {}).get("TESTS_RATE_HZ") == config.TESTS_RATE_HZ)
check("meta never contains the token file setting", not any("TOKEN" in k for k in meta.get("einstellungen", {})))
check("meta has the test definition", (meta.get("definition") or {}).get("id") == "personen_probe")
check("verdict in ergebnis.json and list", data.get("urteil") == listed[0]["urteil"]
      and "% richtig" in (data.get("urteil") or ""), data.get("urteil"))

# hog was already on (e.g. explorer): stays on afterwards
hog.activate(True, "explorer")
test_short = {**test, "id": "personen_kurz", "phasen": [tl._phase("a", "A", "A", 0.3, erwartet="eine_person")]}
runs._run(test_short)
check("hog switched on by someone else stays on", hog.on is True, hog.users)
hog.activate(False, "explorer")


class HogExplorerStarts(Hog):
    """The explorer switches hog on only while the test is already running."""

    def boxes(self):
        self.activate(True, "explorer")
        return super().boxes()


hog2 = HogExplorerStarts()
runs2 = tl.TestRuns(pose=Pose(), people=hog2, camera=camera, cameras={"personen": camera},
                    mock=True, folder=tmp / "runs2")
runs2._run(test_short)
check("explorer started during the test keeps hog", hog2.users == {"explorer"}, hog2.users)

# the real person detection counts its users the same way
from rosmaster_assist.sensors import people as pe  # noqa: E402

real = pe.PersonDetection(None)
real.activate(True)  # explorer (default name, as in explorer.py)
real.activate(True, "tests")
real.activate(False, "tests")
check("person detection: end of the test keeps the explorer running", real._active.is_set())
real.activate(False)
check("person detection: off without users", not real._active.is_set())

# --- note, zip, path guard -----------------------------------------------------------
check("note stored", runs.set_note(name, "Licht von hinten") is None)
check("note in the list", find(runs, name)["notiz"] == "Licht von hinten")
check("note for a foreign path rejected", runs.set_note("../../etc", "x") is not None)
raw = runs.zip_bytes(name)
zip_names = zipfile.ZipFile(io.BytesIO(raw)).namelist() if raw else []
check("zip contains ergebnis.json and images", f"{name}/ergebnis.json" in zip_names
      and any("/bilder/" in n for n in zip_names), len(zip_names))
check("zip for a foreign path: nothing", runs.zip_bytes("..") is None and runs.zip_bytes("../runs") is None)

# --- health record as its own run ---------------------------------------------------
record = {"kind": "health_check", "started_at": time.time() - 30, "outcome": "ok",
          "verdict": {"stufe": "unauffaellig", "punkte": 0, "text": "keine Auffälligkeit"},
          "reason": "keine Auffälligkeit", "checks": [{"name": "liegend", "zustand": "ok"}]}
health = runs.save_health_record(record)
check("health record gets a run name", bool(tl._NAME.match(health)) and health.endswith("_health"), health)
entry = find(runs, health)
check("health record listed with typ health", entry.get("typ") == "health", entry.get("typ"))
check("health verdict from the record", "keine Auffälligkeit" in (entry.get("urteil") or ""), entry.get("urteil"))
hdata = json.loads((runs.folder / health / "ergebnis.json").read_text(encoding="utf-8"))
check("health record stored unchanged", hdata["aufzeichnung"]["checks"] == record["checks"])
check("health record has meta", bool(hdata["meta"]["beginn"]) and hdata["meta"]["definition"] is None)
check("health record can get a note", runs.set_note(health, "Test im Flur") is None
      and find(runs, health)["notiz"] == "Test im Flur")
raw = runs.zip_bytes(health)
check("health record can be zipped", raw is not None
      and f"{health}/ergebnis.json" in zipfile.ZipFile(io.BytesIO(raw)).namelist())
cancelled = runs.save_health_record({"outcome": "cancelled", "reason": "Person weg", "started_at": time.time()})
check("cancelled health check marked", find(runs, cancelled)["abgebrochen"] is True
      and "abgebrochen" in find(runs, cancelled)["urteil"], find(runs, cancelled)["urteil"])
check("two records in the same second get distinct names", cancelled != health)

# --- microphone test -------------------------------------------------------------
speech = Speech()
micro = tl.TestRuns(speech=speech, mock=True, folder=tmp / "micro")
test_m = {"id": "mikro_probe", "titel": "Probe", "kategorie": "mikrofon", "art": "mikro",
          "anleitung": "nur Test", "phasen": [
              tl._phase("stopp", "Stopp sagen", "Sag: Robi stopp", 0.5, erwartet="stop", satz="Robi stopp")]}
check("microphone test starts", micro.start("mikro_pegel") is None)
time.sleep(0.2)
check("commands blocked during the microphone test", micro.block_commands is True)
micro.stop()
micro._thread.join(timeout=5)
check("stopping ends the run", not micro.active and micro.block_commands is False)
aborted = micro.list_runs()
check("aborted run is marked", bool(aborted) and aborted[0]["abgebrochen"] is True
      and aborted[0]["urteil"].startswith("abgebrochen"), aborted[0].get("urteil") if aborted else None)
micro._stop.clear()
micro._run(test_m)
m = next(e for e in micro.list_runs() if e["test"] == "mikro_probe")
dm = micro.folder / m["name"]
dm_data = json.loads((dm / "ergebnis.json").read_text(encoding="utf-8"))
with wave.open(str(dm / "ton.wav")) as w:
    check("ton.wav written", w.getframerate() == 16000 and w.getnframes() > 0, w.getnframes())
check("sound subscription released", speech.subscriptions == [])
check("only sentences since the run start", [e["text"] for e in dm_data["ereignisse"]] == ["robi stopp"],
      dm_data["ereignisse"])
z = dm_data["phasen"][0]["zusammenfassung"]
check("command recognized", z["treffer"] is True, z["kurz"])
check("level measured", bool(z["pegel"]) and z["pegel"]["median"] == 0.3)
check("microphone verdict", m["urteil"] == "1/1 Befehle erkannt", m["urteil"])

# --- upload without a token ------------------------------------------------------
config.TESTS_GIT_TOKEN_FILE = str(tmp / "no_token")
r = runs.upload()
check("without token: error pointing to zip", not r["ok"] and "ZIP" in r["fehler"], r["fehler"])
check("without token: nothing marked as uploaded", not any(e["hochgeladen"] for e in runs.list_runs()))
check("unknown run rejected on upload", not runs.upload(["../x"])["ok"])
st = runs.status()
check("status reports missing token", st["token"] is False)
check("status shows a german hint without token", "Token" in (st["upload"]["hinweis"] or ""),
      st["upload"]["hinweis"])
check("status counts pending runs", st["upload"]["ausstehend"] == 4, st["upload"]["ausstehend"])


# --- upload into a local repo ------------------------------------------------------
def git(repo, *args):
    return subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=True).stdout.strip()


repo = tmp / "repo"
target = tmp / "target.git"
repo.mkdir()
git(repo, "init", "-q")
git(repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "--allow-empty", "-m", "Start")
git(tmp, "init", "-q", "--bare", str(target))
head_before = git(repo, "rev-parse", "HEAD")
(repo / "file.txt").write_text("work in progress\n")  # unsaved work must not disappear
git(repo, "add", "file.txt")
index_before = git(repo, "diff", "--cached", "--name-only")

token_file = tmp / "github_token"
token_file.write_text("secret123\n")
config.TESTS_GIT_TOKEN_FILE = str(token_file)
config.TESTS_GIT_URL = target.as_uri()
tl.git_root = lambda _folder: repo  # do not write into the real project repo
check("status reports token", runs.status()["token"] is True)
check("mock: status says no auto upload", "Simulation" in (runs.status()["upload"]["hinweis"] or ""))

r = runs.upload([name])
check("first upload works and takes all pending runs", r["ok"] and name in r["hochgeladen"]
      and health in r["hochgeladen"] and len(r["hochgeladen"]) == 4, r)
tree = git(target, "ls-tree", "-r", "--name-only", "testdaten").splitlines()
check("stored under tests/<name>/", f"tests/{name}/ergebnis.json" in tree, tree[:3])
check("images uploaded too", any(p.startswith(f"tests/{name}/bilder/") for p in tree))
check("health record uploaded", f"tests/{health}/ergebnis.json" in tree)
check("local bookkeeping stays local", not any(p.endswith(("hochgeladen.json", "upload_fehler.json"))
                                               for p in tree))
check("first commit without parent", git(target, "rev-list", "--count", "testdaten") == "1")
e = find(runs, name)
check("run marked as uploaded with commit and time", e["hochgeladen"] and e["upload_commit"] == r["commit"]
      and bool(e["upload_zeit"]), e)
check("index.json and README.md in the branch", "index.json" in tree and "README.md" in tree)
index = json.loads(git(target, "show", "testdaten:index.json"))
by_name = {x["name"]: x for x in index["laeufe"]}
check("index lists every run", set(by_name) == {x["name"] for x in runs.list_runs()}, sorted(by_name))
check("index entry has date, kind, test and verdict", by_name[health]["typ"] == "health"
      and by_name[name]["test"] == "personen_probe" and "% richtig" in by_name[name]["urteil"]
      and bool(by_name[name]["datum"]), by_name[name])
readme = git(target, "show", "testdaten:README.md")
check("readme has a row per run", all(n in readme for n in by_name) and "Gesundheitscheck" in readme)

check("nothing new: ok without commit", runs.upload() == {"ok": True, "hochgeladen": [], "fehler": None,
                                                          "commit": None})
runs.set_note(name, "nachgetragen")
check("note makes the run pending again", not find(runs, name)["hochgeladen"])
r2 = runs.upload()
check("re-upload of the noted run", r2["ok"] and r2["hochgeladen"] == [name], r2)
check("second commit has a parent", git(target, "rev-list", "--count", "testdaten") == "2")
index2 = json.loads(git(target, "show", "testdaten:index.json"))
check("index keeps earlier runs", len(index2["laeufe"]) == len(index["laeufe"]))
check("note in the branch", json.loads(git(target, "show", f"testdaten:tests/{name}/ergebnis.json"))["notiz"]
      == "nachgetragen")

# runs uploaded from another folder (another computer) end up in the same index
r3 = runs2.upload()
index3 = json.loads(git(target, "show", "testdaten:index.json"))
check("upload from a second store merges into the index", r3["ok"]
      and len(index3["laeufe"]) == len(index["laeufe"]) + 1, r3)

check("repo HEAD unchanged", git(repo, "rev-parse", "HEAD") == head_before)
check("repo index unchanged", git(repo, "diff", "--cached", "--name-only") == index_before)
check("work file still there", (repo / "file.txt").read_text() == "work in progress\n")
check("repo stays on its branch", git(repo, "symbolic-ref", "-q", "HEAD") in ("refs/heads/master",
                                                                             "refs/heads/main"))

# --- auto upload in the background ------------------------------------------------
config.TESTS_GIT_AUTO_UPLOAD = True
auto = tl.TestRuns(mock=False, folder=tmp / "auto")
st = auto.status()["upload"]
check("auto upload on with token", st["auto"] is True and st["hinweis"] is None, st)
started = time.monotonic()
h1 = auto.save_health_record({"outcome": "ok", "verdict": {"stufe": "ansprechbar", "text": "Person reagiert"}})
check("saving does not wait for the upload", time.monotonic() - started < 2.0)
check("auto upload finishes", wait_for_auto_upload(auto))
e = find(auto, h1)
check("auto upload marks the run", e["hochgeladen"] and bool(e["upload_commit"]), e)
check("auto upload in the branch", f"tests/{h1}/ergebnis.json"
      in git(target, "ls-tree", "-r", "--name-only", "testdaten").splitlines())
check("status shows the last upload", (auto.status()["upload"]["letzter"] or {}).get("commit")
      == e["upload_commit"])

# without a token: skipped, only a hint
config.TESTS_GIT_TOKEN_FILE = str(tmp / "no_token")
h2 = auto.save_health_record({"outcome": "ok"})
check("no token: no upload thread", auto._auto_thread is None or not auto._auto_thread.is_alive())
check("no token: run stays local", not find(auto, h2)["hochgeladen"] and find(auto, h2)["upload_fehler"] is None)
check("no token: german hint in status", "Token" in (auto.status()["upload"]["hinweis"] or ""))

# offline / wrong address: error stored per run, retried later
config.TESTS_GIT_TOKEN_FILE = str(token_file)
config.TESTS_GIT_URL = (tmp / "does_not_exist.git").as_uri()
h3 = auto.save_health_record({"outcome": "ok"})
check("failed auto upload finishes", wait_for_auto_upload(auto))
e3 = find(auto, h3)
check("upload error recorded per run", not e3["hochgeladen"] and bool(e3["upload_fehler"]), e3["upload_fehler"])
check("earlier pending run also records the error", bool(find(auto, h2)["upload_fehler"]))
check("status shows the upload error", bool(auto.status()["upload"]["fehler"]))
check("upload error never contains the token", "secret123" not in (e3["upload_fehler"] or ""))
config.TESTS_GIT_URL = target.as_uri()
h4 = auto.save_health_record({"outcome": "ok"})  # next save retries the pending runs
check("retry finishes", wait_for_auto_upload(auto))
check("retry uploads all pending runs", all(find(auto, n)["hochgeladen"] for n in (h2, h3, h4)))
check("retry clears the error", find(auto, h3)["upload_fehler"] is None and auto.status()["upload"]["fehler"] is None)
config.TESTS_GIT_AUTO_UPLOAD = False

# --- token protection ---------------------------------------------------------------
auth = tl._auth("https://github.com/a/b.git", "secret123")
check("token only as a header in the environment", auth.get("GIT_CONFIG_KEY_0") == "http.extraHeader"
      and base64.b64encode(b"x-access-token:secret123").decode() in auth["GIT_CONFIG_VALUE_0"])
check("no token for file:// or http://", tl._auth("http://x/y.git", "secret123") == {}
      and tl._auth(target.as_uri(), "secret123") == {})
check("ssh address becomes https", tl.https_url("git@github.com:a/b.git") == "https://github.com/a/b.git")
check("https address stays", tl.https_url("https://github.com/a/b.git") == "https://github.com/a/b.git")
try:
    tl._git(repo, ["push", "https://nicht-da.invalid/x.git", "HEAD:refs/heads/x"],
            env=tl._auth("https://nicht-da.invalid/x.git", "secret123"))
    check("push to a wrong address fails", False)
except tl.GitError as e:
    check("error message does not contain the token", "secret123" not in str(e), str(e))

runs.shutdown()
auto.shutdown()
print()
print("all passed" if not failures else f"{failures} failed")
sys.exit(1 if failures else 0)
