"""Checks that the pose detection does not report old camera images as fresh.

    python Marlon/interaction/test_pose_frame_age.py

Background (test protocol 04.11.2026): one camera stuttered badly and the
robot detected people who were no longer there. When the camera hung, the
pose detection used to deliver the same image again and again as a new
detection.

Checked without Hailo and without a camera (dummy):

* the same image is not computed twice (no new detection)
* an image that is too old gives "nobody seen" and is counted
* the status shows the frame age and the number of stale frames
* without new images the people disappear after a short time
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.sensors import pose as ps  # noqa: E402

failures = 0


def check(name, condition, result=""):
    global failures
    print(("ok  " if condition else "FAIL"), name, result)
    failures += not condition


class Camera:
    """Dummy: delivers a fixed image with an adjustable capture time."""

    name = "personen"
    fov_deg = 45.0

    def __init__(self):
        self.captured_at = time.monotonic()

    def latest_with_time(self):
        return (b"not a real jpeg", "image/jpeg"), self.captured_at


class Decoder:
    """Dummy for cv2/numpy: counts how often an image was decoded.

    Returns no image (like cv2 with broken data), so that _detect() then
    ends without Hailo. That way the test works the same with and without
    OpenCV.
    """

    IMREAD_COLOR = 1
    uint8 = "uint8"

    def __init__(self):
        self.calls = 0

    def frombuffer(self, data, dtype=None):
        return data

    def imdecode(self, data, flag):
        self.calls += 1
        return None


camera = Camera()
decoder = Decoder()
# mock=False as on the robot; without hailo it stays "not available", but
# _detect() can still be called
pose = ps.PoseDetection(camera, mock=False)
pose._cv2 = pose._np = decoder

# image too old: nobody seen, counted, not computed
camera.captured_at = time.monotonic() - (config.POSE_MAX_FRAME_AGE_S + 1.0)
result = pose._detect(None)
check("old image gives nobody", result == [], repr(result))
check("old image is not computed", decoder.calls == 0, decoder.calls)
st = pose.status()
check("status counts the stale image", st["veraltete_bilder"] == 1, st["veraltete_bilder"])
check("status shows the frame age", st["bild_alter_s"] is not None
      and st["bild_alter_s"] > config.POSE_MAX_FRAME_AGE_S, st["bild_alter_s"])

# the same image again: no result at all (nothing new)
result = pose._detect(None)
check("same image gives no new result", result is None, repr(result))
check("same image is not computed again", decoder.calls == 0, decoder.calls)
check("duplicate image does not count as stale", pose.status()["veraltete_bilder"] == 1)

# a fresh image is computed, but only once
camera.captured_at = time.monotonic()
pose._detect(None)
check("fresh image is computed", decoder.calls == 1, decoder.calls)
pose._detect(None)
check("fresh image is computed only once", decoder.calls == 1, decoder.calls)

# people disappear when no new detections arrive: a remembered person must
# not survive a hanging camera
person = ps._person([0.4, 0.2, 0.6, 0.9], [[0.5, 0.5, 0.9]] * 17, 0.9, 4 / 3, 45.0)
pose._remember([person], time.monotonic())
pose._active.set()
check("freshly remembered person is visible", len(pose.people()) == 1)
with pose._lock:
    pose._time -= ps._MAX_AGE_S + 0.1  # pretend nothing new arrived since
check("without new images the person is gone soon", pose.people() == [])

print()
print("all passed" if not failures else f"{failures} failed")
sys.exit(1 if failures else 0)
