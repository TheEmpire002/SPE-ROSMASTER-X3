"""Checks the grid of always-on mapping -- no robot, no wiring.

    python Marlon/autonomous_driving/test_live_map.py

Cases:

* a few scans in the apartment give walls where the walls are;
* the grid grows in every direction, also negative, and world positions stay;
* log-odds stay within their limits;
* established walls survive a whole session of "free" rays (at worst unknown);
* free space only up to LIVE_MAP_FREE_RANGE_M, rays without echo change nothing;
* ``step``: written only after LIVE_GATE_LAG_N more passes, never while
  frozen, a freeze drops the waiting scans, standing still writes once;
* founding a map needs still, good scans;
* ``OccupancyMap.derived`` equals the full distance field;
* ``nearest_free``;
* ``LiveStore``: round trip, extras, backups, a broken npz falls back to
  the pgm; the search stages after a start.
"""

import math
import pathlib
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402

config.MOCK_APARTMENT = True
config.LIVE_MAP_ENABLED = True

import apartment_sim as sim  # noqa: E402
from rosmaster_assist.api.map_control import startup_stages  # noqa: E402
from rosmaster_assist.mapping.live_mapper import LiveMapper  # noqa: E402
from rosmaster_assist.mapping.live_store import LiveStore  # noqa: E402
from rosmaster_assist.mapping.map_service import MapService  # noqa: E402
from rosmaster_assist.mapping.occ_map import OccupancyMap  # noqa: E402
from rosmaster_assist.navigation import integrate_gate as ig  # noqa: E402
from rosmaster_assist.navigation import map_localization as ml  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FAIL  "), name, info)
    error += not condition


def lo_at(mapper, x, y):
    lo, origin, res = mapper.arrays()
    ix = int(math.floor((x - origin[0]) / res))
    iy = int(math.floor((y - origin[1]) / res))
    if not (0 <= iy < lo.shape[0] and 0 <= ix < lo.shape[1]):
        return 0.0   # outside the grid: never touched
    return float(lo[iy, ix])


# --- walls where the walls are -------------------------------------------------------

mapper = LiveMapper()
for pose in [(-1.0, 0.0, 0.0), (-0.5, 0.3, 0.5), (-1.5, -0.3, -0.4), (-1.0, 0.5, 1.2)]:
    mapper.integrate(sim.scan_points(*pose), pose)
snap = mapper.snapshot()
ys, xs = np.nonzero(snap.occupied)
wx, wy = snap.world(xs, ys)
d = sim.world_distance(wx, wy)
share = float(np.mean(d <= 0.1)) if len(d) else 0.0
check("walls: occupied cells lie on real walls", len(d) > 50 and share >= 0.95,
      f"{len(d)} cells, {share:.1%} within 10 cm")
check("free space around the robot", snap.is_free(-1.0, 0.0) and snap.is_free(-0.7, 0.2))
check("snapshot carries the frame id", snap.frame_id == mapper.frame_id and snap.frame_id)
check("snapshot without changes is the same object", mapper.snapshot() is snap)

# --- growth --------------------------------------------------------------------------

mapper = LiveMapper()
pose = (0.0, 0.0, 0.0)
wall = np.array([[1.0, y] for y in np.linspace(-0.5, 0.5, 21)])
for _ in range(3):
    mapper.integrate(wall, pose)
before = lo_at(mapper, 1.0, 0.0)
origin_before = mapper.arrays()[1]
far = np.array([[-x, 0.0] for x in np.linspace(5.5, 5.9, 5)])
for p in [(-6.0, -6.0, 0.0), (6.0, 6.0, 0.0)]:
    mapper.integrate(far, p)
after = lo_at(mapper, 1.0, 0.0)
lo, origin, res = mapper.arrays()
check("grows to negative coordinates", origin[0] < origin_before[0] and origin[1] < origin_before[1],
      f"{origin_before} -> {origin}")
check("growth keeps world positions", abs(before - after) < 1e-6 and before > 1.6,
      f"{before:.2f} / {after:.2f}")
check("origin stays on the cell lattice",
      abs(origin[0] / res - round(origin[0] / res)) < 1e-6)

big = LiveMapper()
big.integrate(wall, pose)
big.integrate(far, (config.LIVE_MAP_MAX_SIZE_M + 5.0, 0.0, 0.0))
check("beyond LIVE_MAP_MAX_SIZE_M it does not grow", big.too_big and
      max(big.size_m()) <= config.LIVE_MAP_MAX_SIZE_M)

# --- limits, established cells ---------------------------------------------------------

mapper = LiveMapper()
for _ in range(40):
    mapper.integrate(wall, pose)
lo = mapper.arrays()[0]
check("log-odds stay within LO_MIN..LO_MAX",
      lo.max() <= config.LIVE_MAP_LO_MAX + 1e-6 and lo.min() >= config.LIVE_MAP_LO_MIN - 1e-6,
      f"{lo.min():.2f}..{lo.max():.2f}")

lo, origin, res = mapper.arrays()
session = LiveMapper()
session.load_array(lo, origin, res, "abc")
behind = np.array([[2.0, y] for y in np.linspace(-0.5, 0.5, 41)])   # the wall at 1 m is "gone"
for _ in range(50):
    session.integrate(behind, pose)
w = lo_at(session, 1.0, 0.0)
check("an established wall stays at least LO_MAX - SESSION_CHANGE_MAX",
      w >= config.LIVE_MAP_LO_MAX - config.LIVE_MAP_SESSION_CHANGE_MAX - 1e-6, f"{w:.2f}")
check("... so it is never free in this session", w > config.LIVE_MAP_FREE_LO)

# --- ranges -------------------------------------------------------------------------------

mapper = LiveMapper()
mapper.integrate(np.array([[5.5, 0.0]]), pose)
check("free only up to LIVE_MAP_FREE_RANGE_M",
      lo_at(mapper, 2.0, 0.0) < 0 and lo_at(mapper, config.LIVE_MAP_FREE_RANGE_M + 0.3, 0.0) == 0)
check("the hit within LIVE_MAP_HIT_RANGE_M counts", lo_at(mapper, 5.5, 0.0) > 0)
mapper.integrate(np.array([[0.0, 7.0]]), pose)
check("a hit beyond LIVE_MAP_HIT_RANGE_M is not occupied", lo_at(mapper, 0.0, 7.0) == 0)
check("a ray without echo changes nothing", lo_at(mapper, -2.0, 0.0) == 0)

# --- step behind the gate ------------------------------------------------------------------


class Track:
    """Tracking results of a robot driving along x with a correct pose."""

    def __init__(self, dx=0.05):
        self.t = 100.0
        self.x = -1.5
        self.dx = dx
        self.nr = 0

    def step(self, **over):
        self.t += 0.25
        self.x += self.dx
        self.nr += 1
        pose = (self.x, 0.0, 0.0)
        pts = sim.scan_points(*pose, step_deg=2.0)
        values = dict(
            nr=self.nr, timestamp=self.t, run=1, scan=sim.make_scan(pts, self.t, pose), points=pts,
            odo_pose=pose, prediction=pose,
            match=ml.Match(pose[0], pose[1], pose[2], 0.9, 0.02, 150),
            accepted=True, kind="track", jump_m=0.01, turn_rad=0.002, known_share=0.8,
            constraint=0.3, map_gen=1, gap=False, odo_good=True, odo_streak=10,
        )
        values.update(over)
        return ml.TrackResult(**values)


def live(dx=0.05):
    m = LiveMapper()
    m.integrate(sim.scan_points(-1.5, 0.0, 0.0), (-1.5, 0.0, 0.0))
    m.scans_written = 0
    m.gate.integrating_now(1)
    return m, Track(dx)


m, tr = live()
now = tr.t
written = []
for _ in range(config.LIVE_GATE_LAG_N + 2):
    t = tr.step()
    written.append(m.step(t, now=t.timestamp + 0.05))
check("the first scans wait for LIVE_GATE_LAG_N more passes",
      not any(written[:config.LIVE_GATE_LAG_N]) and written[config.LIVE_GATE_LAG_N],
      str(written))

m, tr = live()
for _ in range(config.LIVE_GATE_LAG_N):
    t = tr.step()
    m.step(t, now=t.timestamp + 0.05)
before = m.arrays()[0].copy()
t = tr.step(kind="search_ok")
m.step(t, now=t.timestamp + 0.05)
for _ in range(5):
    t = tr.step()
    m.step(t, now=t.timestamp + 0.05)
check("a freeze drops the waiting scans, frozen writes nothing",
      m.gate.state != ig.INTEGRATING and np.array_equal(before, m.arrays()[0]) and m.scans_written == 0,
      m.gate.reason)

m, tr = live(dx=0.0)
for _ in range(12):
    t = tr.step()
    m.step(t, now=t.timestamp + 0.05)
check("standing still writes a single scan", m.scans_written == 1, str(m.scans_written))

m = LiveMapper()
m.gate.state = ig.FROZEN
check("no step at all while frozen without a map", not m.step(None))

# --- founding a map ---------------------------------------------------------------------


class Measurement:
    def __init__(self, nr, pose, good=True, gap=False):
        self.nr, self.pose, self.good, self.gap = nr, pose, good, gap
        self.scan = sim.make_scan(sim.scan_points(*pose, step_deg=2.0), float(nr), pose)


m = LiveMapper()
m.start_empty()
founded = [m.bootstrap(Measurement(k, (-1.0, 0.0, 0.0))) for k in range(1, 4)]
check("three still scans found the map", founded == [False, False, True] and m.has_map, str(founded))

m = LiveMapper()
m.start_empty()
m.bootstrap(Measurement(1, (-1.0, 0.0, 0.0)))
m.bootstrap(Measurement(2, (-0.9, 0.0, 0.0)))
m.bootstrap(Measurement(3, (-0.9, 0.0, 0.0), good=False))
done = m.bootstrap(Measurement(4, (-0.9, 0.0, 0.0)))
check("moving or a bad scan starts the founding again", not done and not m.has_map)

# --- incremental distance field -------------------------------------------------------------

rng = np.random.default_rng(3)
room = sim.apartment()
worst = 0.0
worst_g = 0.0
for _ in range(12):
    occ = room.occupancy.copy()
    h, w = occ.shape
    x0, y0 = int(rng.integers(0, w - 10)), int(rng.integers(0, h - 10))
    x1, y1 = x0 + int(rng.integers(1, 30)), y0 + int(rng.integers(1, 30))
    x1, y1 = min(x1, w), min(y1, h)
    patch = rng.choice(np.array([-1, 0, 100], dtype=np.int8), size=(y1 - y0, x1 - x0),
                       p=[0.3, 0.5, 0.2])
    occ[y0:y1, x0:x1] = patch
    fast = OccupancyMap.derived(occ, room.resolution, room.origin, room, (x0, y0, x1, y1))
    full = OccupancyMap(occ, room.resolution, room.origin)
    worst = max(worst, float(np.abs(fast.distance - full.distance).max()))
    gx, gy = fast.gradient
    fx, fy = full.gradient
    worst_g = max(worst_g, float(np.abs(gx - fx).max()), float(np.abs(gy - fy).max()))
check("derived: distance field equals the full computation", worst < 1e-6, f"{worst:.2e}")
check("derived: gradient equals the full computation", worst_g < 1e-6, f"{worst_g:.2e}")

# --- nearest_free -----------------------------------------------------------------------------

p = room.nearest_free(-1.0, 0.0, 0.5)
check("nearest_free in free space is right there", p is not None and math.hypot(p[0] + 1.0, p[1]) < 0.05)
p = room.nearest_free(sim.X_MAX + 0.3, 0.0, 0.2)
check("nearest_free outside the map within a small reach: none", p is None)
p = room.nearest_free(sim.X_MAX + 0.1, 0.0, 0.6)
check("nearest_free outside but near: a cell inside", p is not None and p[0] < sim.X_MAX)

# --- saving and loading ------------------------------------------------------------------

with tempfile.TemporaryDirectory() as folder:
    store = LiveStore(MapService(storage_dir=folder))
    check("without a file the main map does not exist", store.load("hauptkarte") is None
          and store.active_name() == config.LIVE_MAP_NAME)
    mapper = LiveMapper()
    for pose in [(-1.0, 0.0, 0.0), (-0.5, 0.3, 0.5)]:
        mapper.integrate(sim.scan_points(*pose), pose)
    snap = mapper.snapshot()
    store.save("hauptkarte", mapper.arrays(), snap, mapper.frame_id,
               {"start": [-1.0, 0.0, 0.0], "last_pose": [-0.5, 0.3, 0.5]})
    saved = store.load("hauptkarte")
    lo = mapper.arrays()[0]
    check("npz round trip: grid, origin, frame id",
          saved is not None and "lo" in saved and saved["lo"].shape == lo.shape
          and float(np.abs(saved["lo"] - lo).max()) < 0.01
          and saved["origin"] == mapper.arrays()[1] and saved["frame_id"] == mapper.frame_id)
    check("extras come back", saved["extra"].get("last_pose") == [-0.5, 0.3, 0.5]
          and saved["extra"].get("start") == [-1.0, 0.0, 0.0])
    check("the map list shows it (pgm/yaml)",
          any(m["name"] == "hauptkarte" for m in MapService(storage_dir=folder).list_maps()))
    again = LiveMapper()
    again.load_array(saved["lo"], saved["origin"], saved["resolution"], saved["frame_id"])
    snap2 = again.snapshot()
    check("loaded map equals the saved one", np.array_equal(snap2.occupancy, snap.occupancy))

    store.save_extra("hauptkarte", {"last_pose": [1.0, 2.0, 0.3]})
    check("save_extra keeps the other extras",
          store.load("hauptkarte")["extra"].get("start") == [-1.0, 0.0, 0.0])

    for _ in range(config.LIVE_MAP_BACKUPS + 2):
        store.backup("hauptkarte")
        time.sleep(1.05)   # one stamp per second
    stamps = {b.name.split(".")[1] for b in (pathlib.Path(folder) / "backups").iterdir()}
    check("only LIVE_MAP_BACKUPS backups are kept", len(stamps) == config.LIVE_MAP_BACKUPS,
          str(sorted(stamps)))

    (pathlib.Path(folder) / "hauptkarte.npz").write_bytes(b"kaputt")
    saved = store.load("hauptkarte")
    check("a broken npz falls back to the pgm",
          saved is not None and "occ_map" in saved
          and np.array_equal(saved["occ_map"].occupancy, snap.occupancy))
    check("... and is kept as .broken", any(pathlib.Path(folder).glob("hauptkarte.*.broken")))

    store.set_active("keller")
    check("the active map is remembered", LiveStore(MapService(storage_dir=folder)).active_name()
          == "keller")

stages = startup_stages({"start": [0.0, 0.0, 0.0], "last_pose": [1.0, 2.0, 0.5]})
check("after a start: last pose, around it, home, whole map",
      [s.center for s in stages] == [(1.0, 2.0), (1.0, 2.0), (0.0, 0.0), None]
      and stages[0].radius == config.LIVE_START_SEARCH_NEAR_M and stages[1].angle is None)
check("without extras only the whole map", len(startup_stages({})) == 1)

print()
print("All good." if error == 0 else f"{error} failed.")
sys.exit(1 if error else 0)
