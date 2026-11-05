"""Always-on mapping end to end in the imagined apartment, without real time.

    python Marlon/autonomous_driving/test_live_sim.py

The real chain runs: lidar odometry (``ScanOdometry.feed``), localization,
integrate gate, mapper -- on a simulated clock, with wheels that read turns a
quarter too much and slip on travel, and 1 cm lidar noise.

Cases:

* founding at standstill and a drive through both rooms: the walls of the
  map lie on the real walls, the pose stays within 10 cm / 3°;
* kidnapped, lidar dropout, lifted and carried, an empty corridor: the map
  stays bit-identical until the pose is confirmed again, and afterwards it
  is still right (no second wall);
* a saved map (npz, and a pgm without npz) is loaded again: the robot finds
  itself at its last pose, confirms and maps on in the same frame.
"""

import math
import pathlib
import sys
import tempfile
import time as _time
from collections import deque

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
from rosmaster_assist.navigation import integrate_gate as ig  # noqa: E402
from rosmaster_assist.navigation import map_localization as ml  # noqa: E402
from rosmaster_assist.navigation import scan_odometry as so  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FAIL  "), name, info)
    error += not condition


class Clock:
    """Simulated time for the modules that ask the clock themselves."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def perf_counter(self):
        return _time.perf_counter()

    def sleep(self, _s):
        pass


clock = Clock()
ml.time = clock
so.time = clock


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Wheels:
    """Wheel odometry with the X3's errors: turns a quarter too much, travel slips."""

    actually_measuring = True

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self._history = deque(maxlen=400)

    def move(self, dx, dy, dth, t):
        dx, dy, dth = 0.97 * dx, 0.97 * dy, 1.25 * dth
        c, s = math.cos(self.theta), math.sin(self.theta)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy
        self.theta += dth
        self._history.append((t, self.x, self.y, self.theta))

    def pose(self):
        return self.x, self.y, math.degrees(self.theta)

    def pose_at(self, t):
        best = None
        for entry in reversed(self._history):
            if entry[0] <= t:
                best = entry
                break
        if best is None:
            return (self.x, self.y, self.theta)
        return best[1], best[2], best[3]


class FedOdometry(so.ScanOdometry):
    """Lidar odometry without its thread: the scans come through ``feed``."""

    running = property(lambda self: True)


class Sim:
    """Robot, lidar, wheels and the always-on mapping chain."""

    def __init__(self, start, scan_fn=None, seed=1):
        self.true = start
        self.start = start            # the odometry frame begins here
        self.scan_fn = scan_fn or (lambda pose: sim.scan_points(*pose, step_deg=1.0, noise=0.01,
                                                                rng=self.rng))
        self.rng = np.random.default_rng(seed)
        self.wheels = Wheels()
        self.odo = FedOdometry(None, self.wheels)
        self.odo.run = 1
        self.loc = ml.MapLocalization(self.odo)
        self.mapper = LiveMapper(wheel_pose_at=self.wheels.pose_at)
        self.snap_t = 0.0
        self.t = clock.t
        self.steps = []               # (gate state, map changed) per step

    def to_map(self, pose):
        """True pose -> map frame (the map frame is the odometry frame at start)."""
        return compose(inverse(self.start), pose)

    def found(self, scans=6):
        self.mapper.start_empty()
        for _ in range(scans):
            self.step()
            if self.mapper.has_map:
                return True
        return False

    def step(self, v=0.0, omega=0.0, dt=0.18, carry=None, blind=False):
        # the real motion
        if carry is not None:
            self.true = (self.true[0] + carry[0], self.true[1] + carry[1], self.true[2])
        else:
            c, s = math.cos(self.true[2]), math.sin(self.true[2])
            self.true = (self.true[0] + c * v * dt, self.true[1] + s * v * dt,
                         math.atan2(math.sin(self.true[2] + omega * dt),
                                    math.cos(self.true[2] + omega * dt)))
            self.wheels.move(v * dt, 0.0, omega * dt, self.t + dt)
        self.t += dt
        clock.t = self.t
        if blind:
            return
        pts = self.scan_fn(self.true)
        self.odo.feed(sim.make_scan(pts, self.t, self.true))
        before = self.mapper.scans_written
        if not self.mapper.has_map:
            if self.mapper.bootstrap(self.odo.measurement()):
                m = self.odo.measurement()
                self.loc.set_map(self.mapper.snapshot(), start=m.pose, source="dauer",
                                 correction=(0.0, 0.0, 0.0))
                self.mapper.gate.integrating_now(self.loc.map_gen)
            return
        track = self.loc.match_scan()
        self.mapper.step(track, now=self.t)
        if self.t - self.snap_t >= config.LIVE_MAP_SNAPSHOT_S:
            self.snap_t = self.t
            snap = self.mapper.snapshot()
            if snap is not None:
                self.loc.update_map(snap)
        self.steps.append((self.mapper.gate.state, self.mapper.scans_written != before))

    def drive_to(self, x, y, max_steps=400):
        for _ in range(max_steps):
            dx, dy = x - self.true[0], y - self.true[1]
            if math.hypot(dx, dy) < 0.08:
                return True
            err = math.atan2(math.sin(math.atan2(dy, dx) - self.true[2]),
                             math.cos(math.atan2(dy, dx) - self.true[2]))
            omega = max(-0.3, min(0.3, 2.0 * err))
            v = 0.2 if abs(err) < 0.2 else 0.0
            self.step(v, omega)
        return False

    def wait(self, n):
        for _ in range(n):
            self.step()

    def accuracy(self):
        """Share of occupied map cells within 10 cm of a real wall."""
        snap = self.mapper.snapshot() or self.loc.occ_map
        ys, xs = np.nonzero(snap.occupied)
        mx, my = snap.world(xs, ys)
        c, s = math.cos(self.start[2]), math.sin(self.start[2])
        wx = self.start[0] + c * mx - s * my
        wy = self.start[1] + s * mx + c * my
        d = sim.world_distance(wx, wy)
        return float(np.mean(d <= 0.10)) if len(d) else 0.0, len(d)

    def pose_error(self):
        p = self.loc.pose()
        if p is None:
            return math.inf, math.inf
        t = self.to_map(self.true)
        return (math.hypot(p[0] - t[0], p[1] - t[1]),
                abs(math.degrees(math.atan2(math.sin(p[2] - t[2]), math.cos(p[2] - t[2])))))

    def lo(self):
        return self.mapper.arrays()[0].copy()


def unchanged_until_confirmed(s, action, settle=150):
    """Run ``action``; the map must not change until the gate writes again."""
    s.steps.clear()
    reference = s.lo()
    action()
    froze = False
    changed_early = False
    for _ in range(settle):
        state = s.mapper.gate.state
        if state != ig.INTEGRATING:
            froze = True
        elif froze:
            break
        if froze and s.mapper.gate.state != ig.INTEGRATING:
            changed_early |= not np.array_equal(reference, s.lo())
        s.step()
    for state, changed in s.steps:
        if state != ig.INTEGRATING and changed:
            changed_early = True
    return froze, changed_early, reference


ROUTE = [(-1.5, -1.5), (0.3, -1.5), (0.3, 1.85), (1.6, 1.85), (3.2, 1.9), (3.6, 0.0),
         (3.0, -1.4), (3.4, 1.9), (1.6, 1.85), (0.0, 1.0), (-1.5, 0.3)]
START = (-1.0, 0.0, 0.0)


def drive_route(s, route):
    reached = True
    for x, y in route:
        reached &= s.drive_to(x, y)
    return reached


# --- 1. founding and a drive through both rooms ------------------------------------------

s = Sim(START)
check("a map is founded at standstill", s.found())
check("... and is written right away", s.mapper.gate.state == ig.INTEGRATING, s.mapper.gate.reason)
reached = drive_route(s, ROUTE)
share, cells = s.accuracy()
err_m, err_deg = s.pose_error()
frozen = s.mapper.gate.frozen
check("the route was driven", reached)
check("the map grew into the second room", s.mapper.size_m()[0] >= 6.0 and cells > 300,
      f"{s.mapper.size_m()}, {cells} wall cells, {s.mapper.scans_written} scans written")
check("walls of the map lie on real walls (>= 95 % within 10 cm)", share >= 0.95,
      f"{share:.1%} of {cells}")
check("pose at the end within 10 cm / 3°", err_m < 0.10 and err_deg < 3.0,
      f"{err_m:.3f} m, {err_deg:.1f}°")
check("a clean drive hardly freezes", frozen <= 2,
      f"{frozen}x, last reason: {s.mapper.gate.reason}")
written = sum(1 for st, c in s.steps if c)
integrating = sum(1 for st, c in s.steps if st == ig.INTEGRATING)
print(f"      {len(s.steps)} steps, {integrating} integrating, {written} with writes")

with tempfile.TemporaryDirectory() as folder:
    store = LiveStore(MapService(storage_dir=folder))
    store.save("hauptkarte", s.mapper.arrays(), s.mapper.snapshot(), s.mapper.frame_id,
               {"start": list(s.loc.start), "last_pose": list(s.loc.pose())})
    end_true = s.true
    saved_lo = s.lo()

    # --- 6. load the saved map again and map on ----------------------------------------
    for kind in ("npz", "pgm"):
        if kind == "pgm":
            (pathlib.Path(folder) / "hauptkarte.npz").unlink()
        saved = store.load("hauptkarte")
        t = Sim(START, seed=7)
        t.true = end_true
        if "lo" in saved:
            t.mapper.load_array(saved["lo"], saved["origin"], saved["resolution"], saved["frame_id"])
        else:
            t.mapper.import_occupancy(saved["occ_map"], saved["frame_id"])
        t.loc.set_map(t.mapper.snapshot(), start=tuple(saved["extra"]["start"]), source="dauer",
                      stages=startup_stages(saved["extra"]))
        # true pose -> map frame: the map keeps the frame of the first drive
        t.to_map = s.to_map
        before_lo = t.lo()
        found = False
        for _ in range(60):
            t.step()
            if t.loc.confident:
                found = True
                break
        err_m, err_deg = t.pose_error()
        check(f"loaded ({kind}): finds itself at the last pose", found and err_m < 0.10,
              f"{err_m:.3f} m, {err_deg:.1f}°, {t.loc.state()['text']}")
        check(f"loaded ({kind}): nothing written before confirming",
              all(not c for st, c in t.steps if st != ig.INTEGRATING))
        t.drive_to(3.6, 0.0)
        t.drive_to(4.3, -1.5)
        t.drive_to(3.4, 1.9)
        check(f"loaded ({kind}): confirms and maps on", t.mapper.scans_written > 5
              and not np.array_equal(before_lo, t.lo()),
              f"{t.mapper.scans_written} scans, gate {t.mapper.gate.state}: {t.mapper.gate.reason}")
        snap = t.mapper.snapshot()
        ys, xs = np.nonzero(snap.occupied)
        mx, my = snap.world(xs, ys)
        c0, s0 = math.cos(START[2]), math.sin(START[2])
        d = sim.world_distance(START[0] + c0 * mx - s0 * my, START[1] + s0 * mx + c0 * my)
        acc = float(np.mean(d <= 0.10))
        check(f"loaded ({kind}): still the same frame, walls right", acc >= 0.95,
              f"{acc:.1%} of {len(d)}")

# --- 2. kidnapped ------------------------------------------------------------------------------

s = Sim(START, seed=2)
s.found()
drive_route(s, ROUTE[:3])
froze, changed, _ = unchanged_until_confirmed(
    s, lambda: setattr(s, "true", (s.true[0] + 0.7, s.true[1] - 0.4, s.true[2] + 0.6)))
check("kidnapped: the map freezes", froze, s.mapper.gate.reason)
check("kidnapped: not a single cell changes until confirmed again", not changed)
drive_route(s, ROUTE[3:7])
share, cells = s.accuracy()
check("kidnapped: afterwards the walls are still right", share >= 0.95, f"{share:.1%} of {cells}")

# --- 3. lidar dropout ----------------------------------------------------------------------------

s = Sim(START, seed=3)
s.found()
drive_route(s, ROUTE[:2])


def dropout():
    for _ in range(12):           # about 2 s without scans while driving on
        s.step(0.2, 0.0, blind=True)


froze, changed, _ = unchanged_until_confirmed(s, dropout)
check("lidar dropout: the map freezes", froze, s.mapper.gate.reason)
check("lidar dropout: nothing written until confirmed again", not changed)
drive_route(s, ROUTE[2:5])
share, cells = s.accuracy()
check("lidar dropout: afterwards the walls are still right", share >= 0.95,
      f"{share:.1%} of {cells}")

# --- 4. lifted and carried -------------------------------------------------------------------------

s = Sim(START, seed=4)
s.found()
drive_route(s, ROUTE[:2])


def carried():
    for _ in range(15):           # 0.6 m in about 3 s, the wheels stand still
        s.step(carry=(0.04, 0.0))


froze, changed, _ = unchanged_until_confirmed(s, carried)
check("carried: the map freezes", froze, s.mapper.gate.reason)
check("carried: nothing written until confirmed again", not changed)
drive_route(s, ROUTE[2:5])
share, cells = s.accuracy()
check("carried: afterwards the walls are still right", share >= 0.95, f"{share:.1%} of {cells}")

# --- 5. empty corridor --------------------------------------------------------------------------------

corridor_scan = sim.corridor_scan(0.0)
c = Sim((2.0, 0.0, 0.0), scan_fn=lambda pose: corridor_scan, seed=5)
c.found()
founded = c.lo()
for _ in range(60):
    c.step(0.2, 0.0)
check("corridor: the map freezes", c.mapper.gate.state != ig.INTEGRATING, c.mapper.gate.reason)
check("corridor: the map stays exactly as founded", np.array_equal(founded, c.lo()))

print()
print("All good." if error == 0 else f"{error} failed.")
sys.exit(1 if error else 0)
