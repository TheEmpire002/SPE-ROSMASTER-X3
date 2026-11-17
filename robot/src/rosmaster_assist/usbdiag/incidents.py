"""Groups USB events into incidents and explains the likely cause."""

import time
from dataclasses import dataclass, field

from . import config, texts
from .kmsg import Event
from .sampler import Sample
from .usbmon import Command

TRIGGER_KINDS = {"disconnect", "overcurrent", "reset_failed", "enum_failed", "board_silent", "undervoltage"}


@dataclass
class Reason:
    key: str
    score: float
    evidence: list[str]


@dataclass
class Incident:
    id: str
    start_mono: float
    start_wall: float
    last_trigger_mono: float
    end_mono: float | None = None
    events: list[Event] = field(default_factory=list)
    ports: set[str] = field(default_factory=set)
    missing: set[str] = field(default_factory=set)
    recovered: bool = False
    reasons: list[Reason] = field(default_factory=list)
    baseline: set[str] = field(default_factory=set)


class IncidentTracker:
    def __init__(self, quiet_s: float, max_s: float):
        self.quiet_s = quiet_s
        self.max_s = max_s
        self.current: Incident | None = None

    @staticmethod
    def _update_missing(inc: Incident, present: set[str]):
        inc.missing |= inc.baseline - present
        inc.missing -= present

    def on_event(self, ev: Event, present: set[str]) -> Incident | None:
        inc = self.current
        started = None
        if inc is None:
            if ev.kind not in TRIGGER_KINDS:
                return None
            # a disconnect proves the port had a device
            baseline = set(present)
            if ev.kind == "disconnect" and ev.port:
                baseline.add(ev.port)
            inc = Incident(
                id=time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(ev.wall)),
                start_mono=ev.mono,
                start_wall=ev.wall,
                last_trigger_mono=ev.mono,
                baseline=baseline,
            )
            self.current = started = inc
        inc.events.append(ev)
        if ev.port:
            inc.ports.add(ev.port)
        if ev.kind in TRIGGER_KINDS:
            # a late or backlog event must not move the quiet timer back
            inc.last_trigger_mono = max(inc.last_trigger_mono, ev.mono)
        self._update_missing(inc, present)
        if ev.kind == "disconnect" and ev.port:
            inc.missing.add(ev.port)
        elif ev.kind == "connect" and ev.port:
            inc.missing.discard(ev.port)
        return started

    def tick(self, now_mono: float, present: set[str]) -> Incident | None:
        inc = self.current
        if inc is None:
            return None
        self._update_missing(inc, present)
        if inc.baseline <= present and now_mono - inc.last_trigger_mono >= self.quiet_s:
            inc.recovered = True
        elif now_mono - inc.start_mono >= self.max_s:
            inc.recovered = False
        else:
            return None
        inc.end_mono = now_mono
        self.current = None
        return inc


@dataclass
class Context:
    samples: list[Sample]
    commands: list[tuple[float, Command]]
    battery: list[tuple[float, float]]
    busy: list[str]
    server_running: bool
    names: dict[str, str]  # port -> German device name


DRIVE_COMMANDS = ("MOTOR", "CAR_RUN", "MOTION")


def _name(ctx: Context, port: str | None) -> str:
    return ctx.names.get(port, config.UNKNOWN_DEVICE_NAME)


def _speed(cmd: Command | None) -> tuple[float, str]:
    """Magnitude of a command (m/s, rad/s or fraction) and its German wording."""
    if cmd is None:
        return 0.0, ""
    p = cmd.params
    if cmd.name == "MOTION":
        axes = [("vz", abs(p.get("vz", 0.0))), ("vx", abs(p.get("vx", 0.0))), ("vy", abs(p.get("vy", 0.0)))]
        axis, mag = max(axes, key=lambda a: a[1])
        unit = "rad/s" if axis == "vz" else "m/s"
        word = {"vz": "Drehung", "vx": "vorwärts/rückwärts", "vy": "seitwärts"}[axis]
        return mag, f"{word} {texts.fmt_num(mag)} {unit}"
    if cmd.name == "MOTOR":
        mag = max((abs(s) for s in p.get("speeds", [])), default=0) / config.MOTOR_SPEED_SCALE
        return mag, f"Motorleistung {round(mag * 100)} %"
    return 0.0, ""


def _motor_start(inc: Incident, ctx: Context) -> Reason | None:
    start = inc.start_mono
    found = None
    prev = None
    for t, cmd in sorted(ctx.commands, key=lambda c: c[0]):
        if start - config.MOTOR_WINDOW_S <= t <= start:
            if cmd.name == "CAR_RUN":
                # car_run is not decoded, so only its first occurrence counts
                started = prev is None or prev.name != "CAR_RUN"
                text = "Fahrmodus gestartet"
            elif cmd.name in ("MOTOR", "MOTION"):
                mag, text = _speed(cmd)
                started = abs(mag - _speed(prev)[0]) >= config.MOTOR_DELTA
            else:
                started = False
            if started:
                found = (t, text)
        prev = cmd
    if found is None:
        return None
    t, text = found
    evidence = [f"Fahrbefehl ({text}) {texts.fmt_num(start - t)} s vorher"]
    score = config.REASON_SCORES["motor_start"]
    window = [v for bt, v in sorted(ctx.battery) if start - config.MOTOR_WINDOW_S <= bt <= start]
    drop = max((window[i] - window[j] for i in range(len(window)) for j in range(i + 1, len(window))),
               default=0.0)
    if drop >= config.BATTERY_DROP_V:
        evidence.append(f"Akku fiel um {texts.fmt_num(drop)} V")
        score = min(1.0, score + config.MOTOR_BATTERY_BONUS)
    return Reason("motor_start", score, evidence)


def _reenum_loop(inc: Incident, ctx: Context) -> Reason | None:
    evidence = []
    last_connect: Event | None = None
    for ev in inc.events:
        if ev.kind == "connect":
            last_connect = ev
        elif ev.kind in ("overcurrent", "disconnect") and last_connect is not None:
            gap = ev.mono - last_connect.mono
            if 0 <= gap <= config.LOOP_REENUM_WINDOW_S:
                what = "neue Überlast" if ev.kind == "overcurrent" else "wieder abgemeldet"
                evidence.append(
                    f"{_name(ctx, last_connect.port)} meldete sich an, {texts.fmt_num(gap)} s später {what}")
                break
            last_connect = None
    fails = sum(1 for ev in inc.events if ev.kind == "reset_failed")
    if fails >= config.LOOP_RESET_FAILS:
        evidence.append(f"{fails}× Neustart eines Anschlusses fehlgeschlagen")
    if not evidence:
        return None
    return Reason("reenum_loop", config.REASON_SCORES["reenum_loop"], evidence)


def _snapshot(inc: Incident, ctx: Context) -> list[Sample]:
    end = inc.end_mono if inc.end_mono is not None else inc.events[-1].mono
    return [s for s in ctx.samples if inc.start_mono - config.SNAPSHOT_BEFORE_S <= s.mono <= end]


def _heat(snap: list[Sample]) -> Reason | None:
    evidence = []
    temps = [s.temp_c for s in snap if s.temp_c is not None]
    if temps and max(temps) >= config.TEMP_HOT_C:
        evidence.append(f"CPU-Temperatur {round(max(temps))} °C")
    if any(s.throttled.get("soft_temp_now") for s in snap):
        evidence.append("Der Pi hat wegen Hitze den Takt gedrosselt")
    elif any(s.throttled.get("throttled_now") for s in snap):
        evidence.append("Der Pi hat den Takt gedrosselt")
    if not evidence:
        return None
    return Reason("heat", config.REASON_SCORES["heat"], evidence)


def _undervoltage(inc: Incident, snap: list[Sample]) -> Reason | None:
    evidence = []
    volts = [s.ext5v for s in snap if s.ext5v is not None]
    if volts and min(volts) < config.EXT5V_LOW_V:
        evidence.append(f"5-V-Schiene {texts.fmt_volts(min(volts))}")
    if any(s.throttled.get("under_voltage_now") for s in snap):
        evidence.append("Der Pi meldete Unterspannung")
    if any(ev.kind == "undervoltage" for ev in inc.events):
        evidence.append(texts.event_sentence("undervoltage", "", {}))
    if not evidence:
        return None
    return Reason("undervoltage", config.REASON_SCORES["undervoltage"], evidence)


def _cpu_spike(inc: Incident, ctx: Context) -> Reason | None:
    evidence = []
    start = inc.start_mono
    cpus = [s.cpu_pct for s in ctx.samples
            if s.cpu_pct is not None and start - config.CPU_WINDOW_S <= s.mono <= start]
    if cpus and max(cpus) >= config.CPU_SPIKE_PCT:
        evidence.append(f"CPU-Auslastung {round(max(cpus))} %")
    if ctx.busy:
        evidence.append(f"{', '.join(ctx.busy)} lief gerade")
    if not evidence:
        return None
    return Reason("cpu_spike", config.REASON_SCORES["cpu_spike"], evidence)


def _board_silent(inc: Incident, ctx: Context) -> Reason | None:
    for ev in inc.events:
        if ev.kind == "board_silent":
            return Reason("board_mcu_silent", config.REASON_SCORES["board_mcu_silent"],
                          [texts.event_sentence("board_silent", _name(ctx, ev.port), ev.details)])
    return None


def classify(inc: Incident, ctx: Context) -> list[Reason]:
    snap = _snapshot(inc, ctx) if inc.events else []
    causes = [r for r in (_motor_start(inc, ctx), _heat(snap), _undervoltage(inc, snap),
                          _cpu_spike(inc, ctx), _board_silent(inc, ctx)) if r]
    reasons = list(causes)
    loop = _reenum_loop(inc, ctx)
    if loop:
        reasons.append(loop)
    if not causes:
        evidence = []
        before = inc.start_mono - config.SNAPSHOT_BEFORE_S
        if not any(before <= t <= inc.start_mono and cmd.name in DRIVE_COMMANDS for t, cmd in ctx.commands):
            evidence.append(f"keine Fahrbefehle in den {texts.fmt_num(config.SNAPSHOT_BEFORE_S, 0)} s davor")
        if not ctx.server_running:
            evidence.append("Server lief nicht")
        reasons.append(Reason("idle_unclear", config.REASON_SCORES["idle_unclear"], evidence))
    return sorted(reasons, key=lambda r: -r.score)
