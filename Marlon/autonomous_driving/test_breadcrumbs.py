"""Checks the way back: breadcrumbs, planning along the trail, the fallback.

    python Marlon/autonomous_driving/test_breadcrumbs.py

Cases:

* a crumb every BREADCRUMB_M, a lost pose breaks the trail, a crumb on the
  same spot can still become strict;
* anchors: the trail "since the mission" starts where the anchor was set;
* the corridor fills in only between neighbouring strict crumbs of one piece;
* the reversed path is the newest piece backwards, simplified;
* saving and loading keeps trail and anchors, a restart breaks the trail,
  broken json gives an empty trail;
* planning: a gap too narrow for the normal wall clearance is passable along
  the trail -- but never through a wall;
* homing's fallback starts at the crumb nearest to the robot;
* two mapped areas without a known link: a guessed path through the unknown,
  a known link is a sure path.
"""

import math
import pathlib
import sys
import types

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.mapping.occ_map import OccupancyMap  # noqa: E402
from rosmaster_assist.navigation import path_planning as pp  # noqa: E402
from rosmaster_assist.navigation.breadcrumbs import Breadcrumbs  # noqa: E402
from rosmaster_assist.navigation.homing import Homing  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("ok    " if condition else "FAIL  ") + name + (f"  ({info})" if info and not condition else ""))
    if not condition:
        error += 1


# --- recording ---------------------------------------------------------------------------------

b = Breadcrumbs()
for i in range(16):
    b.add((i * 0.07, 0.0, 0.0), strict=True)         # 1.05 m in 7 cm steps
check("a crumb every BREADCRUMB_M", len(b.near()) == 6, str(len(b.near())))
check("the trail is 1.05 m long", abs(b.length_m() - 1.05) < 1e-6, f"{b.length_m():.3f}")

b.add((1.1, 0.0, 0.0), strict=False)
check("no crumb within BREADCRUMB_M", len(b.near()) == 6)

b.gap()
b.add((1.1, 0.5, 0.0), strict=True)                   # pose came back somewhere else
check("a gap starts a new piece", b.display()[-1][3] == 1)
check("the jump over the gap does not count as trail", abs(b.length_m() - 1.05) < 1e-6,
      f"{b.length_m():.3f}")

c = Breadcrumbs()
c.add((0.0, 0.0, 0.0), strict=False)
c.add((0.05, 0.0, 0.0), strict=True)
check("the same spot can become strict later", c.display()[-1][2] is True)

# --- anchors ---------------------------------------------------------------------------------

m = Breadcrumbs()
for i in range(5):
    m.add((i * 0.25, 0.0, 0.0), strict=True)
m.set_anchor("mission", (1.0, 0.0, 0.5))
for i in range(5, 10):
    m.add((i * 0.25, 0.0, 0.0), strict=True)
check("anchor pose is kept", m.anchor("mission") == (1.0, 0.0, 0.5))
check("the trail since the mission starts there",
      abs(m.length_m("mission") - 1.0) < 1e-6, f"{m.length_m('mission'):.3f}")
check("an unknown anchor gives None", m.anchor("person") is None)

# --- corridor -------------------------------------------------------------------------------

k = Breadcrumbs()
k.add((0.0, 0.0, 0.0), True)
k.add((0.25, 0.0, 0.0), True)
k.add((0.5, 0.0, 0.0), False)                        # a weaker piece in between
k.add((0.75, 0.0, 0.0), True)
k.add((1.0, 0.0, 0.0), True)
corridor = k.corridor()
xs = corridor[:, 0]
check("the corridor is filled in between strict neighbours",
      np.any((xs > 0.05) & (xs < 0.2)) and np.any((xs > 0.8) & (xs < 0.95)))
check("... but not over a weaker piece", not np.any((xs > 0.3) & (xs < 0.7)), str(xs))
check("only strict crumbs: an empty trail gives an empty corridor",
      Breadcrumbs().corridor().shape == (0, 2))

# --- reversed path --------------------------------------------------------------------------

r = Breadcrumbs()
for i in range(6):
    r.add((i * 0.2, 0.0, 0.0), True)                 # straight piece
r.gap()
for i in range(6):
    r.add((2.0, i * 0.2, 0.0), True)                 # newest piece, straight up
path = r.reversed_path()
check("the reversed path is the newest piece backwards, simplified",
      len(path) == 2 and path[0] == (2.0, 1.0) and path[-1] == (2.0, 0.0), str(path))

l_trail = Breadcrumbs()
for i in range(6):
    l_trail.add((i * 0.2, 0.0, 0.0), True)
for i in range(1, 6):
    l_trail.add((1.0, i * 0.2, 0.0), True)
corner = l_trail.reversed_path()
check("a corner stays in the simplified path", len(corner) == 3
      and math.hypot(corner[1][0] - 1.0, corner[1][1]) < 1e-6, str(corner))

# --- saving -----------------------------------------------------------------------------------

saved = m.to_json()
n = Breadcrumbs()
n.from_json(saved)
check("trail and anchors survive saving", n.display() == m.display()
      and n.anchor("mission") == m.anchor("mission"))
check("the trail since the mission survives saving",
      abs(n.length_m("mission") - m.length_m("mission")) < 1e-6)
n.add((5.0, 5.0, 0.0), True)
check("a restart breaks the trail", n.display()[-1][3] == m.display()[-1][3] + 1)
broken = Breadcrumbs()
broken.from_json({"spur": [[1, "x"]], "anker": {"mission": {}}})
check("broken json gives an empty trail", len(broken.near()) == 0 and not broken.anchors)

# --- into another frame (slam map adopted) --------------------------------------------------

f = Breadcrumbs()
f.add((1.0, 0.0, 0.0), True)
f.set_anchor("mission", (1.0, 0.0, 0.0))
f.transform((0.5, 0.0, math.pi / 2))
crumb = f.display()[-1]
anchor = f.anchor("mission")
check("transform moves crumbs and anchors alike",
      abs(crumb[0] - 0.5) < 1e-6 and abs(crumb[1] - 1.0) < 1e-6
      and abs(anchor[0] - 0.5) < 1e-6 and abs(anchor[1] - 1.0) < 1e-6
      and abs(anchor[2] - math.pi / 2) < 1e-6, f"{crumb} {anchor}")

# --- planning along the trail ---------------------------------------------------------------

res = 0.05
occupancy = np.zeros((60, 100), dtype=np.int8)       # 5 m x 3 m, all free
occupancy[0, :] = occupancy[-1, :] = 100
occupancy[:, 0] = occupancy[:, -1] = 100
occupancy[:, 50] = 100                               # a wall at x = 2.5 m ...
occupancy[28:37, 50] = 0                             # ... with a 0.45 m gap
occ_map = OccupancyMap(occupancy, res, (0.0, 0.0))
start, goal = (1.0, 1.5), (4.0, 1.5)
gap_y = 32.5 * res
narrow = pp.plan_path(occ_map, start, goal)
check("without the trail the gap is too narrow for a sure path -- only a guessed one",
      narrow is not None and narrow.guessed)
trail = [(x, gap_y) for x in np.arange(1.0, 4.01, 0.05)]
along = pp.plan_path(occ_map, start, goal, corridor=trail)
check("along the trail the robot passes the gap, sure", along is not None and not along.guessed)
if along is not None:
    near_gap = [p for p in along.points if abs(p[0] - 2.5) < 0.15]
    check("... through the gap, not elsewhere",
          all(28 * res < p[1] < 37 * res for p in near_gap), str(near_gap))

walled = occupancy.copy()
walled[28:37, 50] = 100                              # gap closed
check("a trail never leads through a wall, nor does a guess",
      pp.plan_path(OccupancyMap(walled, res, (0.0, 0.0)), start, goal, corridor=trail) is None)

# --- two mapped areas, the unknown in between ------------------------------------------------

two = np.full((60, 140), -1, dtype=np.int8)          # 7 m x 3 m, all unknown
two[10:50, 5:45] = 0                                 # room A mapped
two[10:50, 95:135] = 0                               # room B mapped
two[9, 5:45] = two[50, 5:45] = 100                   # their long walls
two[9, 95:135] = two[50, 95:135] = 100
areas = OccupancyMap(two, res, (0.0, 0.0))
a, b = (1.0, 1.5), (5.7, 1.5)
config.NAV_GUESS_PATHS = False
check("two areas without a known link: no sure path", pp.plan_path(areas, a, b) is None)
config.NAV_GUESS_PATHS = True
guess = pp.plan_path(areas, a, b)
check("... but the likely path through the unknown is guessed", guess is not None and guess.guessed)
if guess is not None:
    check("... straight across, not around", guess.length < 5.5, f"{guess.length:.2f} m")
sure = two.copy()
sure[25:35, 45:95] = 0                               # a known corridor between them
detour = pp.plan_path(OccupancyMap(sure, res, (0.0, 0.0)), a, b)
check("a known link is taken as a sure path", detour is not None and not detour.guessed)

# --- homing's fallback ----------------------------------------------------------------------

fake = types.SimpleNamespace(
    _fallback=[(3.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
    _localization=types.SimpleNamespace(pose=lambda: (2.1, 0.1, 0.0)),
)
fallback = Homing._fallback_path(fake)
check("the fallback starts at the crumb nearest to the robot",
      fallback is not None and fallback.points[0] == (2.0, 0.0) and fallback.target == (0.0, 0.0))
check("... and its length is measured from the robot",
      fallback is not None and abs(fallback.length - (math.hypot(0.1, 0.1) + 2.0)) < 1e-6)
fake._fallback = []
check("without trail no fallback", Homing._fallback_path(fake) is None)

print()
print("All good." if error == 0 else f"{error} failed.")
sys.exit(1 if error else 0)
