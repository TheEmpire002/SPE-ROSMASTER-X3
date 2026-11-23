"""Live view of the condition detection for the page ``/zustand``.

Shows what the robot sees right now, without starting a check: which
detector delivers the person boxes (Hailo or HOG), the people with their
keypoints, the lying sign of the tracked person per frame and as a vote
over the last seconds, its movement, the watch and the healthbot, plus the
last pose model comparison (``scripts/bench_pose.py``).

The pose detection only computes while someone uses it. The page keeps it
running with a lease: every request renews it, and without requests for
``CONDITION_VIEW_LEASE_S`` the server switches it off again.
"""

from __future__ import annotations

import json
import logging
import math
import time
from collections import deque
from pathlib import Path

from .. import config
from ..sensors.pose import SKELETON
from . import assessment as am

log = logging.getLogger(__name__)

# user name at the pose detection, see PoseDetection.activate
POSE_USER = "zustand-seite"
# a benchmark upload is small; anything larger is not from bench_pose.py
_MAX_MODELS = 8
_MAX_FIELDS = 20


class ConditionView:
    def __init__(self, pose, people=None, healthbot=None, benchmark_file: Path | None = None) -> None:
        self._pose = pose
        self._people = people
        self._healthbot = healthbot
        self._benchmark_file = Path(benchmark_file or config.CONDITION_BENCHMARK_FILE)
        self._leased = False
        self._last_request = 0.0
        self._tracked: dict | None = None
        self._tracked_at = 0.0
        self._history: deque[tuple[float, dict]] = deque()

    # ------------------------------------------------------------------ lease

    @property
    def leased(self) -> bool:
        return self._leased

    def touch(self, now: float | None = None) -> None:
        """A request from the page: keep the pose detection running."""
        self._last_request = time.monotonic() if now is None else now
        if not self._leased and self._pose is not None:
            self._pose.activate(True, POSE_USER)
            self._leased = True

    def release_if_idle(self, now: float | None = None) -> bool:
        """Switch the pose detection off when the page stopped asking."""
        now = time.monotonic() if now is None else now
        if self._leased and now - self._last_request >= config.CONDITION_VIEW_LEASE_S:
            self.release()
            return True
        return False

    def release(self) -> None:
        if self._leased and self._pose is not None:
            self._pose.activate(False, POSE_USER)
        self._leased = False
        self._tracked = None
        self._history.clear()

    # ------------------------------------------------------------------ status

    def status(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        seen = self._pose.people() if self._pose is not None else []
        tracked = self._track(seen, now)
        history = [p for _, p in self._history]
        single = am.lying(tracked)
        return {
            "seite_aktiv": self._leased,
            "erkennung": self._detection(),
            "skelett": SKELETON,
            "personen": [self._person(p, p is tracked) for p in seen],
            "zustand": {
                "liegend": single,
                "liegend_ueber_zeit": am.lying_over(history, tracked) if history else single,
                "bewegung": am.motionless(history),
                "rumpfwinkel": _rounded(am.torso_angle(tracked)) if tracked else None,
                "bilder": len(history),
                "zeitraum_s": config.CONDITION_VIEW_HISTORY_S,
                "schwellen": {
                    "HEALTH_LYING_DEG": config.HEALTH_LYING_DEG,
                    "HEALTH_LYING_ASPECT": config.HEALTH_LYING_ASPECT,
                    "HEALTH_FORESHORTEN_RATIO": config.HEALTH_FORESHORTEN_RATIO,
                    "HEALTH_LYING_SHARE": config.HEALTH_LYING_SHARE,
                    "HEALTH_MOTIONLESS_PX": config.HEALTH_MOTIONLESS_PX,
                    "POSE_KP_MIN_CONF": config.POSE_KP_MIN_CONF,
                },
            },
            "wache": self._healthbot.watch_status() if self._healthbot is not None else None,
            "healthbot": self._healthbot_summary(),
            "modellvergleich": self.benchmark(),
        }

    def _track(self, seen: list[dict], now: float) -> dict | None:
        """The person to judge: kept from frame to frame like in the healthbot."""
        if self._tracked is not None and now - self._tracked_at > config.HEALTH_AUTO_GAP_S:
            # lost for longer than a missed frame: whoever comes next is someone new
            self._tracked = None
            self._history.clear()
        tracked = am.pick(seen, self._tracked)
        if tracked is None:
            return None
        if self._tracked is None or tracked.get("zeit") != self._tracked.get("zeit"):
            self._history.append((now, tracked))  # the page polls faster than new frames arrive
        self._tracked, self._tracked_at = tracked, now
        while self._history and now - self._history[0][0] > config.CONDITION_VIEW_HISTORY_S:
            self._history.popleft()
        return tracked

    def _detection(self) -> dict:
        return {
            "personenboxen": self._people.status() if self._people is not None else None,
            "pose": self._pose.status() if self._pose is not None else None,
            "modell": Path(config.POSE_HEF).name,
        }

    @staticmethod
    def _person(p: dict, tracked: bool) -> dict:
        return {
            "box": p["box"],
            "kp": p["kp"],
            "score": _rounded(p.get("score"), 2),
            "winkel": _rounded(p.get("winkel")),
            "verfolgt": tracked,
            "liegend": am.lying(p),
        }

    def _healthbot_summary(self) -> dict | None:
        if self._healthbot is None:
            return None
        s = self._healthbot.status()
        return {k: s[k] for k in ("aktiv", "zustand", "schritt", "alarm", "urteil")}

    # ------------------------------------------------------------------ model comparison

    def benchmark(self) -> dict | None:
        """The last stored pose model comparison, or None."""
        try:
            return json.loads(self._benchmark_file.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            log.warning("Pose model comparison unreadable (%s): %s", self._benchmark_file, exc)
            return None

    def save_benchmark(self, data) -> str | None:
        """Store a comparison from bench_pose.py. Returns an error text or None."""
        models = data.get("modelle") if isinstance(data, dict) else None
        if not isinstance(models, list) or not models or len(models) > _MAX_MODELS:
            return "erwartet {\"modelle\": [...]} mit 1 bis 8 Einträgen"
        clean = []
        for m in models:
            if not isinstance(m, dict) or not isinstance(m.get("modell"), str) or len(m) > _MAX_FIELDS:
                return "jedes Modell braucht ein Feld \"modell\" und höchstens 20 Werte"
            if not all(isinstance(v, (str, int, float, type(None))) and not isinstance(v, bool) for v in m.values()):
                return "nur Zahlen und Texte als Werte"
            clean.append({str(k)[:40]: (v[:200] if isinstance(v, str) else v) for k, v in m.items()})
        record = {"zeit": time.time(), "bild": str(data.get("bild", ""))[:200], "modelle": clean}
        try:
            self._benchmark_file.parent.mkdir(parents=True, exist_ok=True)
            self._benchmark_file.write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8")
        except OSError as exc:
            return f"speichern fehlgeschlagen: {exc}"
        return None


def _rounded(value, digits: int = 1):
    return round(value, digits) if isinstance(value, (int, float)) and math.isfinite(value) else None
