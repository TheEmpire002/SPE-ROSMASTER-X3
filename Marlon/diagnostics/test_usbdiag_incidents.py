"""Tests the usbdiag incident tracker and crash reason rules.

    python Marlon/diagnostics/test_usbdiag_incidents.py
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import incidents, kmsg, sampler, usbmon  # noqa: E402


def ev(t, kind, port=None, **details):
    return kmsg.Event(mono=t, wall=1_700_000_000 + t, kind=kind, port=port, details=details, raw="")


def sample(t, ext5v=5.06, temp=55.0, cpu=20.0, **thr):
    return sampler.Sample(t, 0, ext5v, {}, thr, temp, 2000.0, cpu, 0.5)


def motion(vz):
    return usbmon.Command(0x12, "MOTION", {"vx": 0.0, "vy": 0.0, "vz": vz, "car_type": 1})


def started_incident(at):
    return incidents.IncidentTracker(10, 120).on_event(ev(at, "overcurrent", "1-1"), {"1-1"})


def test_burst_is_one_incident():
    tr = incidents.IncidentTracker(quiet_s=10, max_s=120)
    present = {"1-1", "1-1.2", "1-1.4"}
    started = tr.on_event(ev(100, "overcurrent", "1-1"), present)
    assert started is not None
    for i in range(30):
        assert tr.on_event(ev(100.1 + i * 0.05, "overcurrent", "1-1"), present) is None
    tr.on_event(ev(101, "disconnect", "1-1.4"), present - {"1-1.4"})
    assert tr.tick(105, present - {"1-1.4"}) is None
    tr.on_event(ev(103, "connect", "1-1.4"), present)
    done = tr.tick(113.5, present)
    assert done is started and done.recovered and "1-1.4" in done.ports
    # ends with the reconnect, the quiet wait after it is not part of the incident
    assert done.end_mono == 103, done.end_mono
    assert tr.current is None


def test_protocol_errors_alone_do_not_start():
    tr = incidents.IncidentTracker(10, 120)
    assert tr.on_event(ev(5, "protocol_error", "3-2", error=-71), set()) is None


def test_timeout_ends_unrecovered():
    tr = incidents.IncidentTracker(10, 120)
    tr.on_event(ev(0, "disconnect", "1-1.4"), {"1-1.4"})
    done = tr.tick(121, set())
    assert done and not done.recovered and done.missing == {"1-1.4"}
    assert done.end_mono == 120, done.end_mono


def test_backlog_incident_is_not_stretched_to_the_replay():
    # a restart replays old kernel records long after they happened
    tr = incidents.IncidentTracker(10, 120)
    tr.on_event(ev(100, "disconnect", "1-1.4"), {"1-1.4"})
    tr.on_event(ev(105.7, "connect", "1-1.4"), {"1-1.4"})
    done = tr.tick(5000, {"1-1.4"})
    assert done.recovered and done.end_mono == 105.7, done.end_mono
    tr.on_event(ev(200, "disconnect", "1-1.4"), set())
    done = tr.tick(5000, set())
    assert not done.recovered and done.end_mono == 320, done.end_mono


def test_classify_motor_start():
    inc = started_incident(at=50.0)   # helper: tracker + one overcurrent at t=50
    ctx = incidents.Context([sample(49)], [(40.0, motion(0.0)), (49.2, motion(0.6))],
                            [(48.0, 11.9), (49.6, 11.4)], [], True, {})
    top = incidents.classify(inc, ctx)[0]
    assert top.key == "motor_start" and any("0,8 s vorher" in e for e in top.evidence)
    assert any("Akku fiel um 0,5 V" in e for e in top.evidence)
    assert any("Drehung 0,6 rad/s" in e for e in top.evidence)
    assert top.score <= 1.0


def test_classify_loop_and_idle():
    tr = incidents.IncidentTracker(10, 120)
    inc = tr.on_event(ev(10, "overcurrent", "1-1"), {"3-1.1"})
    tr.on_event(ev(12, "connect", "3-1.1", vid="2bc5", pid="060f"), {"3-1.1"})
    tr.on_event(ev(12.4, "overcurrent", "3-1"), {"3-1.1"})
    for t in (13, 14, 15):
        tr.on_event(ev(t, "reset_failed", "3-1.2", error=-71), {"3-1.1"})
    ctx = incidents.Context([sample(9)], [], [], [], False, {"3-1.1": "Tiefenkamera"})
    keys = [r.key for r in incidents.classify(inc, ctx)]
    assert keys[0] == "reenum_loop" and "idle_unclear" in keys
    loop = incidents.classify(inc, ctx)[0]
    assert any("Tiefenkamera meldete sich an" in e for e in loop.evidence)
    assert any("3× Neustart" in e for e in loop.evidence)
    assert all("-71" not in e for r in incidents.classify(inc, ctx) for e in r.evidence)
    idle = [r for r in incidents.classify(inc, ctx) if r.key == "idle_unclear"][0]
    assert any("30 s davor" in e for e in idle.evidence) and any("Server lief nicht" in e for e in idle.evidence)


def test_classify_heat_undervoltage_cpu():
    inc = started_incident(at=50.0)
    ctx = incidents.Context([sample(48, ext5v=4.70, temp=82.0, cpu=95.0)], [], [], ["dpkg"], True, {})
    reasons = {r.key: r for r in incidents.classify(inc, ctx)}
    assert {"heat", "undervoltage", "cpu_spike"} <= set(reasons) and "idle_unclear" not in reasons
    assert "CPU-Temperatur 82 °C" in reasons["heat"].evidence
    assert "5-V-Schiene 4,70 V" in reasons["undervoltage"].evidence
    assert "dpkg lief gerade" in reasons["cpu_spike"].evidence


def test_board_silent_and_empty_context():
    tr = incidents.IncidentTracker(10, 120)
    inc = tr.on_event(ev(5, "board_silent", "1-1.4"), {"1-1.4"})
    keys = [r.key for r in incidents.classify(inc, incidents.Context([], [], [], [], True, {}))]
    assert keys == ["board_mcu_silent"]


def test_late_event_does_not_move_quiet_timer_back():
    tr = incidents.IncidentTracker(10, 120)
    inc = tr.on_event(ev(100, "overcurrent", "1-1"), {"1-1"})
    tr.on_event(ev(50, "disconnect", "1-1"), {"1-1"})
    assert inc.last_trigger_mono == 100


def test_idle_evidence_only_without_drive_commands():
    inc = started_incident(at=50.0)
    quiet = incidents.Context([sample(49)], [], [], [], True, {})
    idle = [r for r in incidents.classify(inc, quiet) if r.key == "idle_unclear"][0]
    assert any("30 s davor" in e for e in idle.evidence)
    driving = incidents.Context([sample(49)], [(30.0, motion(0.3)), (40.0, motion(0.3))], [], [], True, {})
    idle = [r for r in incidents.classify(inc, driving) if r.key == "idle_unclear"][0]
    assert not any("davor" in e for e in idle.evidence)


def _storm(tr, with_connect):
    present = {"1-1", "1-1.4"}
    tr.on_event(ev(100, "overcurrent", "1-1"), present)
    tr.on_event(ev(100.5, "disconnect", "1-1.4"), present - {"1-1.4"})
    if with_connect:
        tr.on_event(ev(103, "connect", "1-1.4"), present)
    return present


def test_new_trigger_after_quiet_gap_closes_the_open_incident():
    tr = incidents.IncidentTracker(10, 120)
    present = _storm(tr, True)
    started = tr.on_event(ev(200, "overcurrent", "1-1"), present)
    assert started is not None and len(tr.closed) == 1
    old = tr.closed[0]
    assert old.start_mono == 100 and old.end_mono == 103 and old.recovered is True
    assert tr.current.start_mono == 200
    # without the reconnect the port stays missing
    tr = incidents.IncidentTracker(10, 120)
    present = _storm(tr, False)
    tr.on_event(ev(200, "overcurrent", "1-1"), present)
    old = tr.closed[0]
    assert old.recovered is False and "1-1.4" in old.missing, (old.recovered, old.missing)
    # a trigger inside the quiet period joins the open incident
    tr = incidents.IncidentTracker(10, 120)
    present = _storm(tr, True)
    assert tr.on_event(ev(108, "overcurrent", "1-1"), present) is None
    assert not tr.closed and tr.current.start_mono == 100


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
