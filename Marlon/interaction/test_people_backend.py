"""Checks where the person boxes come from: Hailo pose boxes or HOG.

    python Marlon/interaction/test_people_backend.py

Since 23.11.2026 the person detection (explorer, test runs) takes its boxes
from the pose detection on the Hailo when it works, and only falls back to
OpenCV HOG on the CPU otherwise (``PERSON_DETECT_BACKEND``). Checked
without Hailo, camera or OpenCV (dummies):

* with a healthy Hailo the boxes are the pose boxes, HOG stays idle
* activating/deactivating switches the pose detection on and off
* a Hailo error, a simulated pose or "hog" in the config falls back to HOG
* a user asking for HOG (test runs) keeps HOG running next to the Hailo
* the status names the method ("verfahren")
"""

import logging
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.sensors import people as pp  # noqa: E402

failures = 0


def check(name, condition, result=""):
    global failures
    print(("ok  " if condition else "FAIL"), name, result)
    failures += not condition


class Camera:
    name = "personen"
    fov_deg = 60.0

    def latest(self):
        return None


class Pose:
    """Dummy pose detection with one person."""

    def __init__(self) -> None:
        self.available = True
        self.simulated = False
        self.healthy = True
        self.users: set[str] = set()
        self.person = {"box": [0.2, 0.1, 0.4, 0.9], "score": 0.87, "kp": []}

    def activate(self, on: bool, who: str) -> None:
        (self.users.add if on else self.users.discard)(who)

    def people(self):
        return [self.person] if self.users else []

    def status(self):
        return {"letzte_erkennung_vor_s": 0.2}


def make(pose):
    d = pp.PersonDetection(Camera(), pose=pose)
    d._hog = object()  # stands in for opencv's hog detector, not available here
    d._error = None
    return d


def main() -> None:
    logging.disable(logging.WARNING)  # "opencv missing" is expected here
    old = (config.PERSON_DETECTION_ENABLED, config.PERSON_DETECT_BACKEND)
    config.PERSON_DETECTION_ENABLED = True
    config.PERSON_DETECT_BACKEND = "auto"
    try:
        pose = Pose()
        d = make(pose)
        check("backend hailo with a healthy pose detection", d.backend == "hailo", str(d.backend))
        check("nothing before activation", d.boxes() == [])
        d.activate(True, "explorer")
        check("activation switches the pose detection on", pp._POSE_USER in pose.users, str(pose.users))
        boxes = d.boxes()
        check("boxes are the pose boxes",
              boxes == [{"x1": 0.2, "y1": 0.1, "x2": 0.4, "y2": 0.9, "score": 0.87}], str(boxes))
        ranges = d.angle_ranges()
        check("angle range from the pose box",
              len(ranges) == 1 and all(abs(a - b) < 1e-6 for a, b in zip(ranges[0], (6.0, 18.0))), str(ranges))
        check("hog idle while the hailo works", not d._hog_needed())
        st = d.status()
        check("status names the method", st["verfahren"] == "hailo" and st["aktiv"] and st["fehler"] is None, str(st))
        check("status age from the pose detection", st["letzte_erkennung_vor_s"] is not None, str(st))

        d.activate(True, "test", hog=True)
        check("hog runs for a user that asks for it", d._hog_needed())
        d.activate(False, "test")
        check("hog idle again after that user", not d._hog_needed())

        pose.healthy = False
        check("hailo error falls back to hog", d.backend == "hog" and d._hog_needed(), str(d.backend))
        check("hog boxes empty until hog has run", d.boxes() == [])
        pose.healthy = True
        pose.simulated = True
        check("simulated pose falls back to hog", d.backend == "hog", str(d.backend))
        pose.simulated = False
        config.PERSON_DETECT_BACKEND = "hog"
        check("config hog forces hog", d.backend == "hog", str(d.backend))
        config.PERSON_DETECT_BACKEND = "auto"

        d.activate(False, "explorer")
        check("deactivation switches the pose detection off", pp._POSE_USER not in pose.users, str(pose.users))
        check("no boxes after deactivation", d.boxes() == [])

        config.PERSON_DETECTION_ENABLED = False
        d.activate(True, "explorer")
        check("switched off in the settings: no boxes", d.boxes() == [] and not d.available)
        config.PERSON_DETECTION_ENABLED = True

        plain = pp.PersonDetection(Camera(), pose=None)
        plain._hog = None
        check("without pose and hog nothing is available", plain.backend is None and not plain.available)
        d.stop()
        check("stop releases the pose detection", pp._POSE_USER not in pose.users, str(pose.users))
    finally:
        config.PERSON_DETECTION_ENABLED, config.PERSON_DETECT_BACKEND = old
    print()
    print("All good." if failures == 0 else f"{failures} failure(s)")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
