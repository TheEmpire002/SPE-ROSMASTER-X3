"""Checks the live view of the condition detection (page /zustand) -- without the robot.

    python Marlon/health/test_condition_view.py

Cases:

* lease: the first request switches the pose detection on, it stays on while
  requests keep coming and goes off after ``CONDITION_VIEW_LEASE_S`` without;
* status: people with keypoints, the tracked person marked, lying per frame
  and as a vote over the last seconds, frames counted once even when the page
  asks faster than new frames arrive, history dropped after a gap;
* detector, watch and healthbot summary are passed through;
* model comparison: stored and read back, bad uploads refused with a text.
"""

import pathlib
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.health import assessment as am  # noqa: E402
from rosmaster_assist.health.live import POSE_USER, ConditionView  # noqa: E402
from rosmaster_assist.sensors import pose as pose_module  # noqa: E402

error = 0


def check(name: str, condition: bool, info: str = "") -> None:
    global error
    if condition:
        print(f"  ok    {name}")
    else:
        error += 1
        print(f"  FEHLER {name} {info}")


def standing(t: float, mx: float = 0.5) -> dict:
    kp, box = pose_module._sim_body(mx, 0.2, 0.6, 0.03)
    return pose_module._person(box, [[x, y, 0.8] for x, y in kp], 0.8, 4 / 3, 60, timestamp=t)


def lying(t: float, my: float = 0.7) -> dict:
    # the standing body turned by 90 degrees around its middle
    kp, _ = pose_module._sim_body(0.5, 0.2, 0.6, 0.03)
    turned = [[0.5 + (y - 0.5), my + (x - 0.5), 0.8] for x, y in kp]
    xs = [p[0] for p in turned]
    ys = [p[1] for p in turned]
    return pose_module._person([min(xs), min(ys), max(xs), max(ys)], turned, 0.8, 4 / 3, 60, timestamp=t)


class FakePose:
    def __init__(self) -> None:
        self.users: set[str] = set()
        self.seen: list[dict] = []

    def activate(self, on: bool, who: str = "x") -> None:
        (self.users.add if on else self.users.discard)(who)

    def people(self) -> list[dict]:
        return list(self.seen)

    def status(self) -> dict:
        return {"verfuegbar": True, "aktiv": bool(self.users), "fps": 9.5, "kamera": "personen"}


class FakePeople:
    def status(self) -> dict:
        return {"verfuegbar": True, "aktiv": True, "verfahren": "hailo"}


class FakeHealthbot:
    def watch_status(self) -> dict:
        return {"an": True, "liegt_seit_s": 1.2, "noetig_s": 3.0}

    def status(self) -> dict:
        return {"aktiv": False, "zustand": "bereit", "schritt": "", "alarm": False, "urteil": None,
                "timeline": [], "checks": []}


def test_lease(tmp: pathlib.Path) -> None:
    print("lease")
    pose = FakePose()
    view = ConditionView(pose, benchmark_file=tmp / "b.json")
    lease = config.CONDITION_VIEW_LEASE_S
    check("off before the first request", not pose.users and not view.leased)
    view.touch(100.0)
    check("first request switches the pose on", pose.users == {POSE_USER})
    view.touch(100.0 + lease * 0.8)
    check("still on while asked", not view.release_if_idle(100.0 + lease * 1.2) and pose.users == {POSE_USER})
    check("off after the lease without requests", view.release_if_idle(100.0 + lease * 1.9) and not pose.users)
    view.touch(200.0)
    check("on again with the next request", pose.users == {POSE_USER})
    view.release()
    check("release on shutdown", not pose.users and not view.leased)
    view.release()
    check("second release harmless", not pose.users)


def test_status(tmp: pathlib.Path) -> None:
    print("status")
    pose = FakePose()
    view = ConditionView(pose, FakePeople(), FakeHealthbot(), benchmark_file=tmp / "b.json")

    s = view.status(0.0)
    check("no person: unknown", s["zustand"]["liegend"]["zustand"] == am.UNKNOWN and s["personen"] == [])
    check("skeleton lines sent", s["skelett"] == pose_module.SKELETON)
    check("detector passed through", s["erkennung"]["personenboxen"]["verfahren"] == "hailo"
          and s["erkennung"]["modell"] == pathlib.Path(config.POSE_HEF).name)
    check("watch passed through", s["wache"]["an"] is True and s["wache"]["liegt_seit_s"] == 1.2)
    check("healthbot summary", s["healthbot"]["zustand"] == "bereit" and "timeline" not in s["healthbot"])
    check("no comparison yet", s["modellvergleich"] is None)

    # standing person, several frames; the page asks twice per frame
    t = 10.0
    for i in range(4):
        pose.seen = [standing(1000.0 + i)]
        s = view.status(t)
        s = view.status(t + 0.05)
        t += 0.1
    check("standing: ok", s["zustand"]["liegend"]["zustand"] == am.OK, s["zustand"]["liegend"])
    check("each frame counted once", s["zustand"]["bilder"] == 4, s["zustand"]["bilder"])
    check("tracked person marked", [p["verfolgt"] for p in s["personen"]] == [True])
    check("keypoints sent", len(s["personen"][0]["kp"]) == 17)
    check("motion judged over the frames", s["zustand"]["bewegung"]["zustand"] != am.UNKNOWN,
          s["zustand"]["bewegung"])

    # a second, smaller person next to them: the tracked one stays
    pose.seen = [standing(1004.0), standing(1004.0, mx=0.15)]
    pose.seen[1]["box"] = [0.1, 0.5, 0.2, 0.7]
    s = view.status(t)
    t += 0.1
    check("two people, one tracked", sum(p["verfolgt"] for p in s["personen"]) == 1 and len(s["personen"]) == 2)

    # the person lies down: single frame flips at once, the vote follows
    pose.seen = [lying(1005.0)]
    s = view.status(t)
    t += 0.1
    check("lying frame flagged at once", s["zustand"]["liegend"]["zustand"] == am.FLAGGED, s["zustand"]["liegend"])
    for i in range(10):
        pose.seen = [lying(1006.0 + i)]
        s = view.status(t)
        t += 0.1
    check("vote over time flagged", s["zustand"]["liegend_ueber_zeit"]["zustand"] == am.FLAGGED,
          s["zustand"]["liegend_ueber_zeit"])
    check("person in the list marked lying", s["personen"][0]["liegend"]["zustand"] == am.FLAGGED)
    check("history limited to the window",
          s["zustand"]["bilder"] <= int(config.CONDITION_VIEW_HISTORY_S / 0.1) + 1, s["zustand"]["bilder"])

    # gone for longer than the gap: history starts again
    pose.seen = []
    view.status(t)
    t += config.HEALTH_AUTO_GAP_S + 0.5
    pose.seen = [standing(2000.0)]
    s = view.status(t)
    check("history reset after a gap", s["zustand"]["bilder"] == 1, s["zustand"]["bilder"])


def test_benchmark(tmp: pathlib.Path) -> None:
    print("model comparison")
    path = tmp / "sub" / "bench.json"
    view = ConditionView(FakePose(), benchmark_file=path)
    good = {"bild": "test.jpg", "modelle": [
        {"modell": "yolov8s_pose_h10.hef", "hailo_ms": 12.3, "total_ms": 20.1, "torso_conf_mean": 0.81},
        {"modell": "yolov8m_pose_h10.hef", "fehler": "RuntimeError: kaputt"},
    ]}
    check("good upload stored", view.save_benchmark(good) is None and path.exists())
    b = view.benchmark()
    check("read back", b is not None and b["bild"] == "test.jpg" and len(b["modelle"]) == 2
          and b["modelle"][0]["hailo_ms"] == 12.3 and b["zeit"] > 0, b)
    for name, bad in [
        ("not a dict", [1, 2]),
        ("no models", {"modelle": []}),
        ("model without name", {"modelle": [{"hailo_ms": 1}]}),
        ("nested value", {"modelle": [{"modell": "x", "y": {"z": 1}}]}),
        ("boolean value", {"modelle": [{"modell": "x", "y": True}]}),
        ("too many models", {"modelle": [{"modell": str(i)} for i in range(9)]}),
    ]:
        text = view.save_benchmark(bad)
        check(f"refused: {name}", isinstance(text, str) and bool(text), str(text))
    check("refused upload keeps the old one", view.benchmark()["bild"] == "test.jpg")
    path.write_text("{kaputt", encoding="utf-8")
    check("broken file reads as none", view.benchmark() is None)


def main() -> None:
    with tempfile.TemporaryDirectory() as d:
        tmp = pathlib.Path(d)
        test_lease(tmp)
        test_status(tmp)
        test_benchmark(tmp)
    print()
    print("all cases pass" if error == 0 else f"{error} error(s)")
    sys.exit(1 if error else 0)


if __name__ == "__main__":
    main()
