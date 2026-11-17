"""Tests the usbdiag bounded log files.

    python Marlon/diagnostics/test_usbdiag_store.py
"""

import json
import pathlib
import shutil
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import kmsg, store  # noqa: E402


def test_events_rotate_and_stay_bounded():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", file_bytes=2000, file_count=3, keep_incidents=5, keep_boots=2)
        for i in range(200):
            st.append_event(kmsg.Event(i, 1e9 + i, "disconnect", "1-1.4", {}, "x" * 50))
        files = sorted(p.name for p in pathlib.Path(d).glob("events.jsonl*"))
        assert files == ["events.jsonl", "events.jsonl.1", "events.jsonl.2"]
        assert all(p.stat().st_size <= 2000 for p in pathlib.Path(d).glob("events.jsonl*"))
        st.close()


def test_seen_event_after_restart():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        st.append_event(kmsg.Event(42.0, 1e9, "disconnect", "1-1.4", {}, "r"))
        st.close()
        again = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        assert again.seen_event("boot-a", 42.0, "r") and not again.seen_event("boot-a", 43.0, "s")
        assert not again.seen_event("boot-b", 1.0, "r")
        again.close()


def test_seen_event_skips_partial_line_and_uses_backup():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        st.append_event(kmsg.Event(7.0, 1e9, "disconnect", None, {}, "r"))
        st.close()
        base = pathlib.Path(d)
        (base / "events.jsonl").rename(base / "events.jsonl.1")
        (base / "events.jsonl").write_text('{"boot": "boot-a", "mono": 9')
        again = store.Store(d, "boot-a", 10_000, 3, 5, 2)
        assert again.seen_event("boot-a", 7.0, "") and not again.seen_event("boot-a", 8.0, "")
        again.close()


def test_append_survives_vanishing_folder():
    d = tempfile.mkdtemp()
    st = store.Store(d, "boot-a", 10_000, 3, 5, 2)
    shutil.rmtree(d)
    st.append_event(kmsg.Event(1.0, 1e9, "disconnect", None, {}, "r"))
    assert (pathlib.Path(d) / "events.jsonl").exists()
    st.close()
    shutil.rmtree(d)


def test_incident_pruning_and_loading():
    with tempfile.TemporaryDirectory() as d:
        st = store.Store(d, "boot-a", 10_000, 3, keep_incidents=3, keep_boots=2)
        for i in range(5):
            st.write_incident({"id": f"2026-11-17_11-0{i}-00", "boot": "boot-a", "reasons": []})
        names = sorted(p.name for p in (pathlib.Path(d) / "incidents").iterdir())
        assert names == ["2026-11-17_11-02-00.json", "2026-11-17_11-03-00.json", "2026-11-17_11-04-00.json"]
        assert [i["id"] for i in st.load_incidents("boot-a")][0] == "2026-11-17_11-04-00"
        (pathlib.Path(d) / "incidents" / "2026-11-17_11-03-00.json").write_text("{broken")
        assert [i["id"] for i in st.load_incidents("boot-a")] == ["2026-11-17_11-04-00", "2026-11-17_11-02-00"]
        assert st.load_incidents("boot-b") == []
        st.close()


def test_summary_keeps_boots_and_is_valid_json():
    with tempfile.TemporaryDirectory() as d:
        for b in ["b1", "b2", "b3"]:
            s = store.Store(d, b, 10_000, 3, 5, keep_boots=2)
            s.write_summary({"incidents": 1})
            s.close()
        data = json.loads((pathlib.Path(d) / "summary.json").read_text())
        assert list(data["boots"]) == ["b2", "b3"]
        (pathlib.Path(d) / "summary.json").write_text("{corrupt")
        s = store.Store(d, "b4", 10_000, 3, 5, 2)
        s.write_summary({"incidents": 0})
        s.close()
        assert list(json.loads((pathlib.Path(d) / "summary.json").read_text())["boots"]) == ["b4"]


def test_read_boot_id_falls_back():
    assert store.read_boot_id("/nonexistent/boot_id") == "mock"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print("bad ", name, repr(exc))
    sys.exit(1 if failed else 0)
