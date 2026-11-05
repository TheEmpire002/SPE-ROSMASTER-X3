"""Checks the integrate gate of always-on mapping -- pure logic, no robot.

    python Marlon/autonomous_driving/test_live_gate.py

Cases:

* every hard criterion alone freezes the map at once, every soft one only
  skips the scan, turning only skips without doubting the pose;
* soft failures that last too long freeze as well, a corridor too;
* back from frozen only after enough strict matches over enough time; a
  soft failure in between starts the count again, turning does not;
* after a lidar dropout nothing is written for a while;
* pause and resume never go straight back to writing;
* a match that contradicts the odometry (similar spot elsewhere) freezes;
* ``constraint``: a room with corners pins the pose, a corridor does not.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402

config.MOCK_APARTMENT = True

import apartment_sim as sim  # noqa: E402
from rosmaster_assist.navigation import integrate_gate as ig  # noqa: E402
from rosmaster_assist.navigation import map_localization as ml  # noqa: E402

error = 0


def check(name, condition, info=""):
    global error
    print(("OK    " if condition else "FAIL  "), name, info)
    error += not condition


class Drive:
    """Inputs of a robot driving straight along x with a correct pose."""

    def __init__(self):
        self.t = 0.0
        self.x = 0.0

    def step(self, **over):
        self.t += 0.25
        self.x += 0.05
        pose = (self.x, 0.0, 0.0)
        values = dict(
            now=self.t, kind="track", accepted=True, share=0.9, residual=0.02, known=150,
            known_share=0.8, constraint=0.3, jump_m=0.01, turn_deg=0.3, omega=0.0, speed=0.2,
            odo_good=True, odo_streak=10, gap=False, age_s=0.1, lifted=False, map_gen=1,
            match_pose=pose, odo_pose=pose,
        )
        values.update(over)
        return ig.GateInputs(**values)


def integrating_gate():
    gate = ig.IntegrateGate()
    gate.integrating_now(1)
    return gate


# --- single criteria -------------------------------------------------------------

d = Drive()
gate = integrating_gate()
check("a perfect scan passes while integrating", gate.update(d.step()) and gate.state == ig.INTEGRATING)

hard_cases = {
    "pose searched anew": dict(kind="search_ok"),
    "still searching": dict(kind="search", accepted=False),
    "match rejected": dict(accepted=False),
    "lidar gap": dict(gap=True),
    "lifted": dict(lifted=True),
    "share below the hard limit": dict(share=0.4),
    "big jump": dict(jump_m=0.2),
    "big turn": dict(turn_deg=7.0),
    "map replaced": dict(map_gen=2),
}
for name, over in hard_cases.items():
    d = Drive()
    gate = integrating_gate()
    gate.update(d.step())
    passed = gate.update(d.step(**over))
    check(f"hard: {name} freezes at once", not passed and gate.state == ig.FROZEN, gate.reason)

soft_cases = {
    "too few points": dict(kind="skip", accepted=False),
    "few hits": dict(share=0.6),
    "residual": dict(residual=0.12),
    "little overlap": dict(known=40),
    "little overlap share": dict(known_share=0.2),
    "corridor": dict(constraint=0.05),
    "small jump": dict(jump_m=0.1),
    "old scan": dict(age_s=0.8),
    "shaky odometry": dict(odo_streak=1),
}
for name, over in soft_cases.items():
    d = Drive()
    gate = integrating_gate()
    gate.update(d.step())
    passed = gate.update(d.step(**over))
    again = gate.update(d.step())
    check(f"soft: {name} skips but does not freeze",
          not passed and again and gate.state == ig.INTEGRATING, gate.reason)

d = Drive()
gate = integrating_gate()
gate.update(d.step(wheel_speed=0.2))
passed = gate.update(d.step(wheel_speed=0.0))
check("carried (the room moves, the wheels do not) freezes at once",
      not passed and gate.state == ig.FROZEN, gate.reason)

d = Drive()
gate = integrating_gate()
gate.update(d.step(wheel_speed=0.0, carry_speed=0.02))
gate.update(d.step(wheel_speed=0.0, carry_speed=0.02))
check("... but not on scan-to-scan jitter at standstill (over the window it does not move)",
      gate.state == ig.INTEGRATING, gate.reason)

d = Drive()
gate = integrating_gate()
passed = gate.update(d.step(wheel_speed=0.0, omega=0.6))
check("... but not while turning on the spot (false travel of the lidar odometry)",
      gate.state == ig.INTEGRATING, gate.reason)

d = Drive()
gate = integrating_gate()
passed = gate.update(d.step(omega=0.6))
check("turning fast skips the scan, stays integrating",
      not passed and gate.state == ig.INTEGRATING, gate.reason)

# --- soft too long, corridor too long ----------------------------------------------

d = Drive()
gate = integrating_gate()
for _ in range(int(config.LIVE_GATE_SOFT_MAX_S / 0.25) + 3):
    gate.update(d.step(share=0.6))
check("soft failures longer than LIVE_GATE_SOFT_MAX_S freeze", gate.state == ig.FROZEN, gate.reason)

d = Drive()
gate = integrating_gate()
for _ in range(int(config.LIVE_GATE_DEGENERATE_S / 0.25) + 3):
    gate.update(d.step(constraint=0.05))
check("a corridor for longer than LIVE_GATE_DEGENERATE_S freezes", gate.state == ig.FROZEN, gate.reason)

# --- confirming -----------------------------------------------------------------------

d = Drive()
gate = ig.IntegrateGate()
check("a new gate starts frozen", gate.state == ig.FROZEN)
for _ in range(config.LIVE_GATE_CONFIRM_N - 1):
    gate.update(d.step())
check("one confirmation short: still not writing", gate.state == ig.CONFIRMING, gate.reason)
for _ in range(20):
    if gate.update(d.step()):
        break
check("enough confirmations over enough time: writing again", gate.state == ig.INTEGRATING)

d = Drive()
gate = ig.IntegrateGate()
for _ in range(config.LIVE_GATE_CONFIRM_N + 2):
    d.t -= 0.2   # 0.05 s apart: N matches in far less than LIVE_GATE_CONFIRM_MIN_S
    gate.update(d.step())
check("enough confirmations but too quickly: still confirming", gate.state == ig.CONFIRMING,
      gate.reason)

d = Drive()
gate = ig.IntegrateGate()
for _ in range(3):
    gate.update(d.step())
before = gate.status()["bestaetigt"]
gate.update(d.step(share=0.6))
status = gate.status()
check("a soft failure while confirming does not count, but keeps the count",
      status["bestaetigt"] == before, str(status))
for _ in range(30):
    if gate.update(d.step()):
        break
check("... and strict matches after it confirm again", gate.state == ig.INTEGRATING, gate.reason)

d = Drive()
gate = ig.IntegrateGate()
for _ in range(6):
    gate.update(d.step())
for _ in range(config.LIVE_GATE_CONFIRM_N - 2):
    gate.update(d.step())
before = gate.status()["bestaetigt"]
gate.update(d.step(omega=0.6))
status = gate.status()
check("turning while confirming keeps the count", status["bestaetigt"] == before, str(status))

d = Drive()
gate = ig.IntegrateGate()
for _ in range(30):
    gate.update(d.step(share=0.72))
check("confirming needs LIVE_GATE_CONFIRM_SHARE, not just the minimum",
      gate.state == ig.CONFIRMING, gate.reason)

# --- lidar dropout, pause --------------------------------------------------------------

d = Drive()
gate = integrating_gate()
gate.update(d.step(gap=True))
held = [gate.update(d.step()) for _ in range(int(config.LIVE_GATE_RECONNECT_HOLD_S / 0.25) - 2)]
check("after a lidar gap nothing passes for LIVE_GATE_RECONNECT_HOLD_S",
      not any(held) and gate.state == ig.FROZEN, gate.reason)

d = Drive()
gate = integrating_gate()
gate.pause("nutzer")
check("paused: nothing passes", not gate.update(d.step()) and gate.state == ig.PAUSED)
gate.pause("slam")
gate.resume("nutzer")
check("still paused while another reason holds", gate.state == ig.PAUSED)
gate.resume("slam")
check("resumed: frozen first, not straight to writing", gate.state == ig.FROZEN, gate.reason)

# --- consistency with the odometry ------------------------------------------------------

d = Drive()
gate = integrating_gate()
for _ in range(5):
    gate.update(d.step())
x = d.x + 0.05
passed = gate.update(d.step(match_pose=(x + 0.3, 0.4, 0.0), jump_m=0.0))
check("a match 0.5 m off the odometry track freezes although its jump looks small",
      not passed and gate.state == ig.FROZEN, gate.reason)

d = Drive()
gate = integrating_gate()
ok = True
for k in range(8):
    x = d.x + 0.05
    # slow drift of 1 cm per step against the odometry is normal
    ok &= gate.update(d.step(match_pose=(x, 0.01 * k, 0.0)))
check("slow drift within the allowance keeps writing", ok and gate.state == ig.INTEGRATING,
      gate.reason)

d = Drive()
gate = integrating_gate()
for _ in range(5):
    gate.update(d.step())
x = d.x + 0.05
passed = gate.update(d.step(match_pose=(x + 0.12, 0.0, 0.0), jump_m=0.0))
again = gate.update(d.step())
check("a match a little off the odometry (12 cm) is only skipped, the map keeps writing",
      not passed and again and gate.state == ig.INTEGRATING, gate.reason)

d = Drive()
gate = integrating_gate()
for _ in range(5):
    gate.update(d.step())
for k in range(int(config.LIVE_GATE_SOFT_MAX_S / 0.25) + 2):
    x = d.x + 0.05
    gate.update(d.step(match_pose=(x + (0.12 if k % 2 else -0.12), 0.0, 0.0), jump_m=0.0))
# a single pass in between never reaches the map: it waits for LIVE_GATE_LAG_N more
check("... but matches jumping back and forth freeze the map",
      gate.frozen >= 1 and gate.state != ig.INTEGRATING, gate.reason)

# --- constraint ---------------------------------------------------------------------------

room = sim.apartment()
pts = sim.scan_points(-1.0, 0.0, 0.0)
c_room = ml.constraint(room, np.asarray(pts), (-1.0, 0.0, 0.0))
hall = sim.corridor()
c_hall = ml.constraint(hall, sim.corridor_scan(5.0), (5.0, 0.0, 0.0))
check("constraint: the room pins the pose down", c_room > 0.2, f"{c_room:.3f}")
check("constraint: the corridor does not", c_hall < 0.05, f"{c_hall:.3f}")

print()
print("All good." if error == 0 else f"{error} failed.")
sys.exit(1 if error else 0)
