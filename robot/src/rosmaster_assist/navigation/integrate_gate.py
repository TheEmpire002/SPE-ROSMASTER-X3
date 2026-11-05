"""May this scan change the map? The integrate gate of always-on mapping (E14).

The map grows on every drive -- but a scan written at a wrong pose puts a
wall where none is, and the next match then believes that wall. So the
rule is strict: a scan only reaches the map if the localization is sure of
it, by every measure there is, and has been for a while.

Each tracking step (``map_localization.TrackResult``) is checked against a
list of criteria. They come in three kinds:

- **hard**: something is wrong with the pose (the match jumps, contradicts
  the odometry, the lidar was gone, the robot was lifted, the pose was
  searched anew). The map freezes at once, and it only thaws after
  ``LIVE_GATE_CONFIRM_N`` strict matches over ``LIVE_GATE_CONFIRM_MIN_S``.
- **soft**: this scan is not good enough (few hits, little overlap, a
  corridor without cross walls). It is skipped; if that lasts longer than
  ``LIVE_GATE_SOFT_MAX_S`` (or the corridor longer than
  ``LIVE_GATE_DEGENERATE_S``), it freezes as well.
- **motion**: turning or driving too fast -- the A1 distorts scans then. The
  scan is skipped, nothing else happens.

Pure logic without threads or hardware, so every rule can be tested on its
own (``Marlon/autonomous_driving/test_live_gate.py``).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from .. import config

# kinds of failure
HARD = "hart"
SOFT = "weich"
MOTION = "bewegung"

# states (also the protocol values of state.karte.kartieren.zustand)
START = "start"              # empty map, waiting to found it
INTEGRATING = "integriert"   # writes into the map
CONFIRMING = "bestaetigt"    # pose good again, counting confirmations
FROZEN = "eingefroren"       # nothing is written until confirmed
PAUSED = "pausiert"          # switched off by the user or during slam
OFF = "aus"                  # no lidar or disabled in config


@dataclass
class GateInputs:
    """Everything the gate looks at for one tracking step."""

    now: float
    kind: str                       # TrackResult.kind
    accepted: bool
    share: float = 0.0
    residual: float = math.inf
    known: int = 0                  # points in established cells
    known_share: float = 0.0
    constraint: float = 0.0
    jump_m: float = 0.0
    turn_deg: float = 0.0
    omega: float = 0.0              # rad/s
    speed: float = 0.0              # m/s
    odo_good: bool = True
    odo_streak: int = 0
    gap: bool = False
    age_s: float | None = None
    lifted: bool = False
    map_gen: int = 0
    match_pose: tuple | None = None
    odo_pose: tuple | None = None


def check(inp: GateInputs) -> list[tuple[str, str]]:
    """The criteria that depend on this step alone: (kind, reason) per failure."""
    issues: list[tuple[str, str]] = []
    if inp.kind == "search_ok":
        issues.append((HARD, "Lage gerade neu gefunden"))
    elif inp.kind == "search":
        issues.append((HARD, "sucht die eigene Lage"))
    elif inp.kind == "skip":
        issues.append((SOFT, "zu wenige Lidar-Punkte"))
    elif not inp.accepted:
        issues.append((HARD, "Aufnahme passt nicht zur Karte"))
    if inp.gap:
        issues.append((HARD, "Lidar war kurz weg"))
    if inp.lifted:
        issues.append((HARD, "hochgehoben oder verschoben"))
    if not inp.odo_good or inp.odo_streak < config.LIVE_GATE_ODO_GOOD_N:
        # soft: the scan is written at the map match, not at the odometry
        # pose -- but a shaky odometry makes the prediction shaky
        issues.append((SOFT, "Lidar-Odometrie unsicher"))
    if inp.kind != "track":
        return issues

    if inp.share < config.LIVE_GATE_HARD_SHARE:
        issues.append((HARD, f"Lage unsicher ({100 * inp.share:.0f} % Treffer)"))
    elif inp.share < config.LIVE_GATE_MIN_SHARE:
        issues.append((SOFT, f"zu wenige Treffer ({100 * inp.share:.0f} %)"))
    if not inp.residual <= config.LIVE_GATE_MAX_RESIDUAL_M:
        issues.append((SOFT, "Aufnahme liegt ungenau auf der Karte"))
    if inp.known < config.LIVE_GATE_MIN_KNOWN or inp.known_share < config.LIVE_GATE_MIN_KNOWN_SHARE:
        issues.append((SOFT, "zu wenig bekannte Karte im Blick"))
    if inp.constraint < config.LIVE_GATE_MIN_CONSTRAINT:
        issues.append((SOFT, "Gang ohne Querwände"))
    jump_hard = (inp.jump_m > 2 * config.LIVE_GATE_MAX_JUMP_M
                 or inp.turn_deg > 2 * config.LIVE_GATE_MAX_JUMP_DEG)
    jump_soft = (inp.jump_m > config.LIVE_GATE_MAX_JUMP_M
                 or inp.turn_deg > config.LIVE_GATE_MAX_JUMP_DEG)
    if jump_hard:
        issues.append((HARD, f"Lage springt ({100 * inp.jump_m:.0f} cm, {inp.turn_deg:.0f}°)"))
    elif jump_soft:
        issues.append((SOFT, "Lage wackelt"))
    if inp.age_s is not None and inp.age_s > config.LIVE_GATE_MAX_AGE_S:
        issues.append((SOFT, "Aufnahme zu alt"))
    if abs(inp.omega) > config.LIVE_GATE_MAX_OMEGA:
        issues.append((MOTION, "dreht zu schnell"))
    if inp.speed > config.LIVE_GATE_MAX_SPEED:
        issues.append((MOTION, "fährt zu schnell"))
    return issues


def _relative(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, _normalize(b[2] - a[2])


def _compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _normalize(a[2] + b[2])


def _normalize(w: float) -> float:
    return math.atan2(math.sin(w), math.cos(w))


class IntegrateGate:
    """State machine with hysteresis around ``check``."""

    def __init__(self) -> None:
        self.state = FROZEN
        self.reason: str | None = "wartet auf eine sichere Lage"
        self._k = 0
        self._confirm_since: float | None = None
        self._soft_since: float | None = None
        self._degenerate_since: float | None = None
        self._hold_until = 0.0
        self._map_gen: int | None = None
        self._paused_by: set[str] = set()
        # (odo_pose, match_pose) of recent passes, for the consistency check
        self._window: deque = deque(maxlen=max(2, config.LIVE_GATE_WINDOW_N))
        self.passed = 0
        self.frozen = 0

    # --- control -------------------------------------------------------------

    def pause(self, by: str) -> None:
        self._paused_by.add(by)
        self.state = PAUSED
        self.reason = "pausiert"
        self._reset()

    def resume(self, by: str) -> None:
        self._paused_by.discard(by)
        if self._paused_by or self.state != PAUSED:
            return
        # never straight back to writing: the pose has to prove itself first
        self.state = FROZEN
        self.reason = "fortgesetzt -- bestätigt erst die Lage"

    @property
    def paused_by(self) -> set[str]:
        return set(self._paused_by)

    def integrating_now(self, map_gen: int) -> None:
        """A new map was just founded at a known pose (bootstrap, adoption)."""
        self._reset()
        self._map_gen = map_gen
        if not self._paused_by:
            self.state = INTEGRATING
            self.reason = None

    def freeze(self, reason: str) -> None:
        if self.state in (PAUSED, OFF):
            return
        if self.state != FROZEN:
            self.frozen += 1
        self.state = FROZEN
        self.reason = reason
        self._reset()

    def _reset(self) -> None:
        self._k = 0
        self._confirm_since = None
        self._soft_since = None
        self._degenerate_since = None
        self._window.clear()

    # --- one step ------------------------------------------------------------

    def update(self, inp: GateInputs) -> bool:
        """True: this scan passed and the map is being written."""
        if self.state in (PAUSED, OFF):
            return False
        issues = check(inp)
        if self._map_gen is not None and inp.map_gen != self._map_gen:
            issues.append((HARD, "Karte gewechselt"))
        self._map_gen = inp.map_gen
        if inp.gap:
            self._hold_until = inp.now + config.LIVE_GATE_RECONNECT_HOLD_S
        elif inp.now < self._hold_until:
            issues.append((HARD, "Lidar war kurz weg"))

        if inp.kind == "track" and inp.constraint < config.LIVE_GATE_MIN_CONSTRAINT:
            self._degenerate_since = self._degenerate_since or inp.now
            if inp.now - self._degenerate_since > config.LIVE_GATE_DEGENERATE_S:
                issues.append((HARD, "Gang ohne Querwände -- Lage nicht eindeutig"))
        else:
            self._degenerate_since = None

        if not any(k == HARD for k, _ in issues) and inp.accepted and inp.kind == "track":
            consistent = self._consistent(inp)
            if consistent is not None:
                issues.append((HARD, consistent))

        hard = [r for k, r in issues if k == HARD]
        soft = [r for k, r in issues if k == SOFT]
        motion = [r for k, r in issues if k == MOTION]
        if hard:
            self.freeze(hard[0])
            return False
        if soft:
            self._soft_since = self._soft_since or inp.now
            if inp.now - self._soft_since > config.LIVE_GATE_SOFT_MAX_S:
                self.freeze(f"länger unsicher: {soft[0]}")
                return False
            if self.state == CONFIRMING:
                self._k = 0
                self._confirm_since = None
            self.reason = soft[0]
            return False
        self._soft_since = None
        if motion:
            # a skipped scan, no reason to doubt the pose
            self.reason = motion[0]
            return False

        # a strict pass
        self._window.append((inp.odo_pose, inp.match_pose))
        if self.state in (FROZEN, START):
            self.state = CONFIRMING
            self._k = 0
            self._confirm_since = inp.now
        if self.state == CONFIRMING:
            if inp.share < config.LIVE_GATE_CONFIRM_SHARE:
                self._k = 0
                self._confirm_since = inp.now
                self.reason = f"bestätigt die Lage (zu wenige Treffer, {100 * inp.share:.0f} %)"
                return False
            self._k += 1
            if (self._k >= config.LIVE_GATE_CONFIRM_N
                    and inp.now - self._confirm_since >= config.LIVE_GATE_CONFIRM_MIN_S):
                self.state = INTEGRATING
                self.reason = None
            else:
                self.reason = f"bestätigt die Lage ({self._k}/{config.LIVE_GATE_CONFIRM_N})"
                return False
        self.reason = None
        self.passed += 1
        return True

    def _consistent(self, inp: GateInputs) -> str | None:
        """Does this match agree with the earlier ones, given the odometry in between?

        The oldest match of the window plus the odometry since then predicts
        where this one should be; drift is allowed per metre driven. A wrong
        match (a similar spot in the next room) agrees with nothing.
        """
        if inp.match_pose is None or inp.odo_pose is None or not self._window:
            return None
        odo0, match0 = self._window[0]
        if odo0 is None or match0 is None:
            return None
        rel = _relative(odo0, inp.odo_pose)
        expected = _compose(match0, rel)
        travel = math.hypot(rel[0], rel[1])
        err = math.hypot(inp.match_pose[0] - expected[0], inp.match_pose[1] - expected[1])
        err_deg = abs(math.degrees(_normalize(inp.match_pose[2] - expected[2])))
        allowed = config.LIVE_GATE_WINDOW_M + config.LIVE_GATE_DRIFT_M_PER_M * travel
        allowed_deg = config.LIVE_GATE_WINDOW_DEG + config.LIVE_GATE_DRIFT_DEG_PER_M * travel
        if err > allowed or err_deg > allowed_deg:
            return f"Abgleich widerspricht der Odometrie ({100 * err:.0f} cm, {err_deg:.0f}°)"
        return None

    def status(self) -> dict:
        return {
            "zustand": self.state,
            "grund": self.reason,
            "bestaetigt": self._k if self.state == CONFIRMING else None,
            "von": config.LIVE_GATE_CONFIRM_N,
            "durchgelassen": self.passed,
            "eingefroren_mal": self.frozen,
        }
