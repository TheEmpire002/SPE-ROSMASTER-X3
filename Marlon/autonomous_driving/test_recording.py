"""Checks the recorder (recording.py) -- without robot.

    python Marlon/autonomous_driving/test_recording.py

Checked: the buffer only keeps the most recent time, duplicate scans and
unchanged states do not end up in it, saving and loading give the same
result, old files are cleaned up, only valid file names are served, and the
watchdog reports every command before and after the limits.

Most important case at the end: a simulated explorer drive is recorded,
saved, loaded and fed to a fresh explorer. With the same randomness it must
give the same commands tick by tick -- otherwise replaying real drives is
useless for checking new avoidance logic.
"""

import math
import pathlib
import random
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import recording as az  # noqa: E402
from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.mock_driver import MockDriver  # noqa: E402
from rosmaster_assist.navigation import explorer as ex  # noqa: E402
from rosmaster_assist.navigation import obstacles as hd  # noqa: E402
from rosmaster_assist.protocol import STOP, DriveCommand  # noqa: E402
from rosmaster_assist.safety.watchdog import SafetyWatchdog  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402

error = 0
TICK = 1.0 / config.ODOMETRY_HZ
SCAN_S = 1.0 / 7.5


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FEHLER"), name, info)
    error += not condition


def scan_at(timestamp, n=360, r=1.5):
    s = Scan([float(i - 180) for i in range(n)], [r + 0.001 * i for i in range(n)])
    s.timestamp = timestamp
    return s


tmp = pathlib.Path(tempfile.mkdtemp(prefix="aufzeichnung_test_"))

# --- 1. Ring buffer, duplicate scans, states ------------------------------------
rec = az.Recorder(buffer_s=1.0, folder=tmp / "a")
for i in range(31):
    rec.command("hand", STOP, STOP, now=i * 0.1)
e = rec.events()
check("buffer: only the last second", e[0]["t"] >= 2.0 - 1e-9 and e[-1]["t"] == 3.0,
       f"{e[0]['t']:.1f}–{e[-1]['t']:.1f} s")

rec = az.Recorder(buffer_s=60.0, folder=tmp / "a")
s = scan_at(5.0)
check("scan: a new one is recorded", rec.scan(s, now=5.0))
check("scan: the same one not again", not rec.scan(s, now=5.05))
check("scan: None is skipped", not rec.scan(None))
check("state: change recorded", rec.state(now=5.1, **{"erkunder": "fährt"}))
check("state: the same one not again", not rec.state(now=5.2, **{"erkunder": "fährt"}))
check("state: new change recorded", rec.state(now=5.3, **{"erkunder": None}))
st = rec.status()
check("status counts events and scans", st["ereignisse"] == 3 and st["scans"] == 1, st)

# --- 2. Saving and loading ------------------------------------------------------
rec.header = {"antrieb": "test"}
rec.odo((0.1, 0.0, 0.2), (1.0, 2.0, 30.0), DriveCommand(0.1, 0.0, 0.2), True, now=5.4)
path = rec.save("Tür im Flur")
check("save: valid file name", az.valid_name(path.name), path.name)
check("save: no .tmp file left over", not list(path.parent.glob("*.tmp")))
header, ev = az.load(path)
check("load: header complete",
       header["version"] == az.VERSION and header["notiz"] == "Tür im Flur"
       and header["antrieb"] == "test" and "AUTONOMOUS_MAX_SPEED" in header["config"], header.get("notiz"))
check("load: all events", len(ev) == 4, len(ev))
check("load: times from the first event", ev[0]["t"] == 0.0 and ev[0]["zeit"] == 0.0,
       (ev[0]["t"], ev[0].get("zeit")))
s2 = az.scan_from(ev[0])
dev_a = max(abs(a - b) for a, b in zip(s.angle, s2.angle))
dev_r = max(abs(a - b) for a, b in zip(s.ranges, s2.ranges))
check("load: scan equal except for rounding", dev_a <= 0.05 and dev_r <= 0.0005,
       f"{dev_a:.3f}°, {dev_r * 1000:.2f} mm")
check("load: odometry preserved", ev[3]["v"] == [0.1, 0.0, 0.2] and ev[3]["misst"] is True)

# --- 3. Cleanup and file names --------------------------------------------------
rec = az.Recorder(buffer_s=60.0, folder=tmp / "b", max_files=3)
rec.stop(now=1.0)
saved = [rec.save() for _ in range(5)]
entries = rec.entries()
check("cleanup: only the newest 3", len(entries) == 3, [d["name"] for d in entries])
check("cleanup: last file stays", saved[-1].exists())
rec2 = az.Recorder(buffer_s=60.0, folder=tmp / "b2")
rec2.stop(now=1.0)
a, b = rec2.save(), rec2.save()
check("same second: own name, nothing overwritten", a != b and a.exists() and b.exists(),
       (a.name, b.name))
check("name: valid", az.valid_name("fahrt_20261103_141500.jsonl.gz"))
check("name: valid with counter", az.valid_name("fahrt_20261103_141500_2.jsonl.gz"))
for wrong in ("../config.py", "..\\fahrt_20261103_141500.jsonl.gz", "fahrt_x.jsonl.gz",
               "fahrt_20261103_141500.jsonl.gz.tmp", ""):
    check(f"name: rejected {wrong!r}", rec.file(wrong) is None)
check("name: valid, but not present", rec.file("fahrt_20000101_000000.jsonl.gz") is None)
check("name: present", rec.file(saved[-1].name) == saved[-1])

# --- 4. Watchdog reports commands -----------------------------------------------
wd = SafetyWatchdog(MockDriver())
rec = az.Recorder(buffer_s=60.0, folder=tmp / "c")
wd.recording = rec
wd.submit(DriveCommand(5.0, 0.0, 0.0), source="hand")
wd.stop()
ev = rec.events()
check("watchdog: command recorded", len(ev) == 2 and ev[0]["art"] == "befehl", ev[:1])
check("watchdog: soll unchanged, safe limited",
       ev[0]["soll"][0] == 5.0 and ev[0]["safe"][0] <= config.MAX_SPEED_X, ev[0])
check("watchdog: stop recorded", ev[1]["art"] == "stop")


class BrokenWriter:
    def command(self, *a):
        raise RuntimeError("on purpose")

    def stop(self):
        raise RuntimeError("on purpose")


wd.recording = BrokenWriter()
try:
    wd.submit(DriveCommand(0.1, 0.0, 0.0), source="hand")
    wd.stop()
    ok = True
except Exception as exc:  # noqa: BLE001
    ok = repr(exc)
check("watchdog: a broken writer does not disturb driving", ok is True, ok)

# --- 5. Playback and segments ---------------------------------------------------
w = az.Playback([
    {"t": 0.1, "art": "scan", "zeit": 0.1, "w": [0], "r": [1000]},
    {"t": 0.2, "art": "odo", "v": [0.2, 0.0, 0.0], "pose": [0, 0, 0], "ist": [0, 0, 0], "misst": True},
])
check("playback: nothing before the first scan", w.latest() is None and not w.actually_measuring)
w.until(0.15)
check("playback: scan from its time on", w.latest() is not None and w.velocity() == (0.0, 0.0, 0.0))
w.until(0.2)
check("playback: odometry from its time on", w.velocity() == (0.2, 0.0, 0.0) and w.actually_measuring)
try:
    w.until(0.1)
    back = False
except ValueError:
    back = True
check("playback: forward only", back)
ab = az.segments([
    {"t": 0.0, "art": "zustand", "erkunder": None},
    {"t": 1.0, "art": "zustand", "erkunder": "fährt"},
    {"t": 3.0, "art": "zustand", "erkunder": None},
    {"t": 4.0, "art": "zustand", "erkunder": "dreht"},
    {"t": 5.0, "art": "odo"},
])
check("segments: explorer time spans", ab == [(1.0, 3.0), (4.0, 5.0)], ab)


# --- 6. Record and replay an explorer drive -------------------------------------
class World:
    """Rectangular room with round obstacles (as in test_explorer.py)."""

    def __init__(self, room, circles):
        self.room, self.circles = room, circles
        self.x = self.y = self.theta = 0.0
        self.v_actual = (0.0, 0.0, 0.0)

    def ray(self, direction):
        dx, dy = math.cos(direction), math.sin(direction)
        x0, x1, y0, y1 = self.room
        best = float("inf")
        for limit, d, start in ((x0, dx, self.x), (x1, dx, self.x), (y0, dy, self.y), (y1, dy, self.y)):
            if abs(d) > 1e-9 and (limit - start) / d > 0:
                best = min(best, (limit - start) / d)
        for cx, cy, r in self.circles:
            mx, my = self.x - cx, self.y - cy
            b = mx * dx + my * dy
            disk = b * b - (mx * mx + my * my - r * r)
            if disk >= 0 and -b - math.sqrt(disk) > 0:
                best = min(best, -b - math.sqrt(disk))
        return best

    def scan(self, timestamp):
        angle, dist = [], []
        for i in range(360):
            r = self.ray(self.theta + math.radians(i))
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                angle.append(float(i if i <= 180 else i - 360))
                dist.append(r)
        # Rounded as in the recorder: the original explorer should see exactly
        # what the file plays back later.
        w, r = az.round_scan(Scan(angle, dist))
        return az.scan_from({"w": w, "r": r, "zeit": timestamp, "dauer": round(SCAN_S, 4)})

    def move(self, command, dt):
        if command is None:
            self.v_actual = (0.0, 0.0, 0.0)
            return
        self.v_actual = tuple(round(v, 4) for v in (command.vx, command.vy, command.omega))
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += (command.vx * c - command.vy * s) * dt
        self.y += (command.vx * s + command.vy * c) * dt
        self.theta += command.omega * dt


class FakeLidar:
    last = None

    def latest(self):
        return self.last


class FakeOdometry:
    actually_measuring = True

    def __init__(self, world):
        self.world = world

    def velocity(self):
        return self.world.v_actual


class NoDriver:
    # The progress watch also reads the last accepted command.
    current_command = DriveCommand()

    def submit(self, command, source=""):
        self.current_command = command
        return command

    def stop(self):
        pass


random.seed(7)
world = World((-1.5, 2.5, -1.2, 1.4), [(1.0, 0.3, 0.25), (-0.6, -0.7, 0.2)])
lidar = FakeLidar()
expl = ex.Explorer(NoDriver(), lidar, hd.ObstacleDetection(), None, odometry=FakeOdometry(world))
rec = az.Recorder(buffer_s=1000.0, folder=tmp / "d")
expl._begin(0.0)
original = []
next_scan = 0.0
for k in range(int(30.0 / TICK)):
    # Rounded clock: the file stores times to 0.1 ms.
    t = round(k * TICK, 4)
    if t >= next_scan:
        lidar.last = world.scan(t)
        rec.scan(lidar.last, now=t)
        next_scan += SCAN_S
    rec.odo(world.v_actual, (world.x, world.y, math.degrees(world.theta)), STOP, True, now=t)
    rec.state(now=t, **{"erkunder": expl._state})
    command = expl._step(t)
    original.append(None if command is None else [round(v, 4) for v in (command.vx, command.vy, command.omega)])
    if command is None:
        break
    rec.command("autonom", command, command, now=t)
    world.move(command, TICK)
states = {e["erkunder"] for e in rec.events() if e["art"] == "zustand"}
check("simulation: explorer drives and avoids", len(states) >= 2, states)

_, ev = az.load(rec.save("Simulation"))


def build(playback):
    return ex.Explorer(NoDriver(), playback, hd.ObstacleDetection(), None, odometry=playback)


random.seed(7)
ticks = az.replay_explorer(ev, build)
same = sum(1 for t in ticks if t["aufgezeichnet"] == t["nachgespielt"])
check("replay: every tick gets its turn", len(ticks) == len([b for b in original if b is not None]),
       f"{len(ticks)} of {len(original)}")
check("replay: same commands tick by tick", same == len(ticks), f"{same}/{len(ticks)}")
first_diff = next((t for t in ticks if t["aufgezeichnet"] != t["nachgespielt"]), None)
if first_diff:
    print("       first difference:", first_diff)

print()
print("All checks passed." if error == 0 else f"{error} check(s) failed.")
sys.exit(1 if error else 0)
