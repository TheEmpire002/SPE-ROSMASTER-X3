"""Live state of the USB diagnosis and the German page model built from it."""

import threading
from collections import deque
from dataclasses import asdict, replace

from . import config, texts
from .incidents import DRIVE_COMMANDS, Context, Incident, IncidentTracker, _speed, classify
from .kmsg import Event
from .layout import Tile, build_layout, tile_for_port
from .sampler import Sample
from .topology import DeviceInfo, Topology, device_name, device_role
from .usbmon import FrameDecoder, Urb, decode_commands, decode_report

_LEVEL_ORDER = {"empty": 0, "ok": 1, "warn": 2, "bad": 3}


def _clamp(seconds: float) -> float:
    return max(0.0, seconds)


class DiagState:
    """Everything the page shows.

    The store is only ever called from one thread (the daemon loop that calls apply_* and tick),
    because Store.append_event has no lock. Only page() may run on another thread: it takes
    self.lock itself, so a caller of page() must not already hold it. The apply_* methods and
    tick do not lock; the daemon holds self.lock around them.
    """

    def __init__(self, store, boot_wall: float, daemon_start_mono: float):
        self.store = store  # may be None
        self.boot_wall = boot_wall
        self.start_mono = daemon_start_mono
        self.lock = threading.Lock()

        self.topo = Topology()
        self.layout = build_layout(self.topo, None)
        self.present: set[str] = set()
        self.port_names: dict[str, str] = {}
        self.last_device: dict[str, str] = {}
        self.missing_since: dict[str, float] = {}
        self.tty_port: dict[str, str] = {}
        self.tile_events: dict[str, deque] = {}
        self.disconnected: set[str] = set()
        self.tile_reconnects: dict[str, list[float]] = {}
        self.proto_errors: dict[str, list[float]] = {}
        self.reconnects_boot = 0
        self.reconnects_daemon = 0
        self.urb_errors: dict[int, int] = {}
        self.traffic: dict[int, deque] = {}

        self.samples: deque = deque(maxlen=max(1, int(config.SAMPLE_RING_S / config.SAMPLE_INTERVAL_S)))
        self.sample: Sample | None = None
        self.commands: deque = deque()
        self.battery: deque = deque()
        self.reports: deque = deque()
        self.last_report: float | None = None
        self.last_report_vals: dict = {}
        self.last_command_mono: float | None = None
        self.report_ref: float | None = None
        self.silent_raised = False

        self.tracker = IncidentTracker(config.INCIDENT_QUIET_S, config.INCIDENT_MAX_S)
        self.finished: list[dict] = []
        if self.store is not None:
            try:
                self.finished = list(self.store.load_incidents(self.store.boot_id))
            except (OSError, ValueError, TypeError, AttributeError):
                self.finished = []

        self.capture = "unavailable"
        self.last_urb: float | None = None
        self.busy: list[str] = []
        self.server_running = True
        self.last_summary = daemon_start_mono
        self.decoders: dict[int, FrameDecoder] = {}

    # topology helpers

    def _board(self) -> DeviceInfo | None:
        return next((d for d in self.topo.devices.values() if (d.vid, d.pid) == config.BOARD_ID), None)

    def _direct(self, tile: Tile) -> DeviceInfo | None:
        return next((self.topo.devices[lane] for lane in tile.lanes if lane in self.topo.devices), None)

    def _tile(self, tile_id: str) -> Tile | None:
        return next((t for t in self.layout.tiles if t.id == tile_id), None)

    def _camera_on_board_bus(self) -> bool:
        board = self._board()
        if board is None:
            return False
        return any(d.busnum == board.busnum and (device_role(d) == "camera"
                                                 or d.speed_mbps >= config.CAPTURE_MAX_SPEED_MBPS)
                   for d in self.topo.devices.values())

    # inputs

    def apply_topology(self, topo: Topology, now: float) -> None:
        self.topo = topo
        self.layout = build_layout(topo, self.layout)
        self.present = set(topo.devices)
        for path, dev in topo.devices.items():
            self.port_names[path] = device_name(dev)
            if dev.tty:
                self.tty_port[dev.tty] = path
        for tile in self.layout.tiles:
            if tile.kind == "motor":
                continue
            dev = self._direct(tile)
            if dev is not None:
                self.last_device[tile.id] = device_name(dev)
                self.missing_since.pop(tile.id, None)
            elif tile.id in self.last_device:
                self.missing_since.setdefault(tile.id, now)
        if self._board() is None:
            self.report_ref = None
            self.last_report = None
            self.silent_raised = False
            self.decoders.clear()
        elif self.report_ref is None:
            self.report_ref = now

    def apply_event(self, ev: Event, backlog: bool = False) -> None:
        port = ev.port
        if ev.kind == "driver_detached" and port is None:
            port = self.tty_port.get(ev.details.get("tty"))
            ev = replace(ev, port=port)
        if ev.kind == "connect" and port:
            known = config.KNOWN_DEVICES.get((ev.details.get("vid"), ev.details.get("pid")))
            self.port_names[port] = known[0] if known else config.UNKNOWN_DEVICE_NAME
        if ev.kind == "driver_attached" and port and ev.details.get("tty"):
            self.tty_port[ev.details["tty"]] = port
        tile = tile_for_port(self.layout, port) if port else None
        text = texts.event_sentence(ev.kind, self.port_names.get(port, config.UNKNOWN_DEVICE_NAME), ev.details)

        if self.store is not None and not self.store.seen_event(self.store.boot_id, ev.mono, ev.raw):
            self.store.append_event(ev)
        if tile and ev.kind != "new_device":
            ring = self.tile_events.setdefault(tile, deque(maxlen=config.PORT_EVENT_RING))
            ring.append((ev.wall, text, ev.raw))
        if tile and ev.kind == "disconnect":
            self.disconnected.add(tile)
        elif tile and ev.kind == "connect" and tile in self.disconnected:
            self.disconnected.discard(tile)
            self.tile_reconnects.setdefault(tile, []).append(ev.mono)
            self.reconnects_boot += 1
            if not backlog:
                self.reconnects_daemon += 1
        elif tile and ev.kind == "protocol_error":
            self.proto_errors.setdefault(tile, []).append(ev.mono)
        self.tracker.on_event(ev, set(self.present))

    def apply_urbs(self, urbs: list[Urb], now: float) -> None:
        target = self._target()
        if target is None:
            return
        bus, devnums = target
        board = self._board()
        for urb in urbs:
            if urb.bus != bus or urb.dev not in devnums:
                continue
            ring = self.traffic.setdefault(urb.dev, deque())
            ring.append((now, max(urb.length, len(urb.data))))
            if urb.event == "C" and urb.status is not None and urb.status < 0:
                self.urb_errors[urb.dev] = self.urb_errors.get(urb.dev, 0) + 1
            if board is None or urb.dev != board.devnum:
                continue
            self.last_urb = now
            if urb.xfer.endswith("o") and urb.event == "S":
                for cmd in decode_commands(urb.data):
                    self.commands.append((now, cmd))
                    self.last_command_mono = now
            elif urb.xfer.endswith("i") and urb.event == "C" and urb.data:
                decoder = self.decoders.setdefault(urb.dev, FrameDecoder())
                for frame in decoder.feed(urb.data):
                    self._on_report(frame, now)
        self._trim(now)

    def _on_report(self, frame, now: float) -> None:
        vals = decode_report(frame)
        if frame.type != 0x0A or "battery_v" not in vals:
            return
        self.battery.append((now, vals["battery_v"]))
        self.last_report_vals = vals
        self.reports.append(now)
        self.last_report = now
        self.silent_raised = False

    def _trim(self, now: float) -> None:
        horizon = now - config.SAMPLE_RING_S
        while self.commands and self.commands[0][0] < horizon:
            self.commands.popleft()
        while self.battery and self.battery[0][0] < horizon:
            self.battery.popleft()
        while self.reports and self.reports[0] < horizon:
            self.reports.popleft()
        for ring in self.traffic.values():
            while ring and ring[0][0] < horizon:
                ring.popleft()

    def apply_sample(self, s: Sample) -> None:
        self.sample = s
        self.samples.append(s)

    def apply_processes(self, p: dict) -> None:
        self.busy = list(p.get("busy", []))
        self.server_running = bool(p.get("server_running", True))

    def set_capture(self, status: str) -> None:
        self.capture = status

    def _target(self) -> tuple[int, set[int]] | None:
        board = self._board()
        if board is None or self._camera_on_board_bus():
            return None
        devnums = {board.devnum} | {d.devnum for d in self.topo.devices.values()
                                    if d.busnum == board.busnum and device_role(d) == "lidar"}
        return board.busnum, devnums

    def capture_target(self) -> tuple[int, set[int]] | None:
        """Bus and device numbers to listen to, or None (and the reason in the capture status)."""
        if self._board() is None:
            return None
        if self._camera_on_board_bus():
            self.set_capture("camera_on_bus")
            return None
        if self.capture == "camera_on_bus":
            self.set_capture("unavailable")  # the camera left, the daemon may try again
        return self._target()

    # periodic work

    def tick(self, now: float) -> None:
        self._check_board_silent(now)
        inc = self.tracker.tick(now, set(self.present))
        if inc is not None:
            self._finish(inc)
        if now - self.last_summary >= config.SUMMARY_INTERVAL_S:
            self.last_summary = now
            self._write_summary(now)

    def _check_board_silent(self, now: float) -> None:
        if self._board() is None or self.capture != "on" or self.silent_raised:
            return
        if self.last_command_mono is None or now - self.last_command_mono > config.BOARD_SILENT_S:
            return
        refs = [t for t in (self.last_report, self.report_ref) if t is not None]
        if not refs or now - max(refs) < config.BOARD_SILENT_S:
            return
        self.silent_raised = True
        self.apply_event(Event(now, self.boot_wall + now, "board_silent", self.layout.board_port, {}, ""))

    def _write_summary(self, now: float) -> None:
        if self.store is None:
            return
        self.store.write_summary({
            "boot_wall": self.boot_wall,
            "uptime_s": round(_clamp(now)),
            "incidents": len(self.finished) + (1 if self.tracker.current else 0),
            "reconnects": self.reconnects_boot,
            "reconnects_daemon": self.reconnects_daemon,
        })

    def _context(self) -> Context:
        return Context(list(self.samples), list(self.commands), list(self.battery), list(self.busy),
                       self.server_running, dict(self.port_names))

    def _entry(self, inc: Incident, end_mono: float, recovered: bool) -> dict:
        reasons = classify(inc, self._context())
        inc.reasons = reasons
        labels = {t.id: t.label for t in self.layout.tiles}
        tiles = sorted({t for p in inc.ports if (t := tile_for_port(self.layout, p))})
        where = ", ".join(labels[t] for t in tiles if t in labels) or texts.MODULE_TITLES["pi"]
        sentences = []
        for ev in inc.events:
            name = self.port_names.get(ev.port, config.UNKNOWN_DEVICE_NAME)
            sentences.append({"time": texts.fmt_clock(ev.wall), "text": texts.event_sentence(ev.kind, name, ev.details)})
        return {
            "id": inc.id,
            "time": texts.fmt_clock(inc.start_wall),
            "where": where,
            "reason": texts.REASONS[reasons[0].key]["title"] if reasons else "",
            "duration": texts.fmt_duration(_clamp(end_mono - inc.start_mono)),
            "recovered": recovered,
            "reasons": [{"key": r.key, "title": texts.REASONS[r.key]["title"], "evidence": list(r.evidence),
                         "action": texts.REASONS[r.key]["action"]} for r in reasons],
            "events": sentences,
            "boot": self.store.boot_id if self.store is not None else "",
            "start_mono": inc.start_mono,
            "end_mono": end_mono,
            "start_wall": inc.start_wall,
            "tiles": tiles,
            "snapshot": [asdict(s) for s in self.samples if inc.start_mono - config.SNAPSHOT_BEFORE_S <= s.mono],
            "technical": [ev.raw for ev in inc.events if ev.raw],
        }

    def _finish(self, inc: Incident) -> None:
        end = inc.end_mono if inc.end_mono is not None else inc.last_trigger_mono
        entry = self._entry(inc, end, inc.recovered)
        if self.store is not None:
            self.store.write_incident(entry)
        self.finished.insert(0, entry)

    # page model

    def page(self, now: float) -> dict:
        with self.lock:
            return self._page(now)

    def _all_incidents(self, now: float) -> list[dict]:
        cur = self.tracker.current
        live = [self._entry(cur, now, False)] if cur else []
        return live + self.finished

    def _page(self, now: float) -> dict:
        incidents = self._all_incidents(now)
        levels = {}
        lines = {}
        open_tiles = self._open_tiles()
        for tile in self.layout.tiles:
            if tile.kind != "motor":
                levels[tile.id], lines[tile.id] = self._tile_level(tile, now, open_tiles)
        mcu = next((t for t in self.layout.tiles if t.kind == "mcu"), None)
        motor_block = self._motor(now, incidents)

        modules = []
        for module in ("pi", "board", "motor"):
            tiles = []
            for tile in self.layout.tiles:
                if tile.module != module:
                    continue
                if tile.kind == "motor":
                    level = levels.get(mcu.id, "empty") if mcu else "empty"
                    line, device = motor_block["state"], texts.MODULE_TITLES["motor"]
                else:
                    level, line = levels[tile.id], lines[tile.id]
                    device = self.last_device.get(tile.id, config.NO_DEVICE_NAME)
                tiles.append({"id": tile.id, "label": tile.label, "kind": tile.kind, "level": level,
                              "device": device, "line": line})
            note = texts.SOCKETS_UNVERIFIED_NOTE if module == "pi" and not config.SOCKETS_VERIFIED else ""
            modules.append({"id": module, "title": texts.MODULE_TITLES[module], "note": note, "tiles": tiles})

        details = {t.id: self._detail(t, now, incidents, motor_block) for t in self.layout.tiles}
        return {
            "verdict": self._verdict(now, incidents, levels),
            "uptime": texts.fmt_short(now),
            "running": texts.fmt_short(now - self.start_mono),
            "incident_count": len(incidents),
            "vitals": self._vitals(),
            "sampling_note": texts.SAMPLING_NOTE,
            "modules": modules,
            "links": self._links(),
            "details": details,
            "incidents": [{k: e.get(k) for k in ("id", "time", "where", "reason", "duration", "recovered",
                                                 "reasons", "events")}
                          for e in incidents[:config.INCIDENTS_ON_PAGE]],
            "motor": motor_block,
        }

    def _open_tiles(self) -> set[str]:
        cur = self.tracker.current
        if cur is None:
            return set()
        return {t for p in cur.ports if (t := tile_for_port(self.layout, p))}

    def _recent(self, monos: list[float], now: float) -> list[float]:
        return [t for t in monos if _clamp(now - t) <= config.RECENT_RECONNECT_S]

    def _tile_level(self, tile: Tile, now: float, open_tiles: set[str]) -> tuple[str, str]:
        dev = self._direct(tile)
        if dev is None and tile.id in self.last_device:
            since = texts.fmt_clock(self.boot_wall + self.missing_since.get(tile.id, now))
            return "bad", f"fehlt seit {since}"
        if dev is None:
            return "empty", "nichts angeschlossen"
        if tile.id in open_tiles:
            return "bad", "Vorfall läuft"
        if tile.kind == "mcu" and self._camera_on_board_bus():
            return "warn", texts.CAPTURE_TEXTS["camera_on_bus"]
        reconnects = self._recent(self.tile_reconnects.get(tile.id, []), now)
        if reconnects:
            age = texts.fmt_short(now - max(reconnects))
            return "warn", f"{len(reconnects)}× neu verbunden, zuletzt vor {age}"
        errors = self._recent(self.proto_errors.get(tile.id, []), now)
        if errors:
            return "warn", f"{len(errors)}× Übertragungsfehler, zuletzt vor {texts.fmt_short(now - max(errors))}"
        return "ok", "keine Auffälligkeiten"

    def _links(self) -> list[dict]:
        lay = self.layout
        if lay.board_socket is None or lay.board_port is None:
            return []
        level = "ok" if lay.board_present else "bad"
        mcu = next((t.id for t in lay.tiles if t.kind == "mcu"), "")
        return [{"from": lay.board_socket, "to": "board", "level": level},
                {"from": mcu, "to": "motor", "level": level}]

    def _vitals(self) -> list[dict]:
        s = self.sample
        volts = s.ext5v if s else None
        temp = s.temp_c if s else None
        cpu = s.cpu_pct if s else None
        batt = self.battery[-1][1] if self.battery else None
        capped = bool(s and any(s.throttled.get(k) for k in ("freq_capped", "throttled", "soft_temp")))
        j5, l5 = texts.judge_ext5v(volts)
        jb, lb = texts.judge_battery(batt)
        jt, lt = texts.judge_temp(temp)
        jc, lc = texts.judge_cpu(cpu)
        return [
            {"label": "5V Pi", "value": texts.fmt_volts(volts), "judgement": j5, "level": l5},
            {"label": "Akku", "value": "unbekannt" if batt is None else f"{texts.fmt_num(batt)} V",
             "judgement": jb, "level": lb},
            {"label": "Temperatur", "value": "unbekannt" if temp is None else f"{texts.fmt_num(temp, 0)} °C",
             "judgement": jt, "level": lt},
            {"label": "CPU", "value": "unbekannt" if cpu is None else f"{texts.fmt_num(cpu, 0)} %",
             "judgement": jc, "level": lc},
            {"label": "Taktbremse", "value": "aktiv" if capped else "aus",
             "judgement": "gedrosselt" if capped else "gut", "level": "warn" if capped else "ok"},
        ]

    def _verdict(self, now: float, incidents: list[dict], levels: dict) -> dict:
        missing = [self.last_device[t] for t, since in sorted(self.missing_since.items(), key=lambda kv: kv[1])
                   if t in self.last_device and levels.get(t) == "bad"]
        if missing:
            names = ", ".join(dict.fromkeys(missing))
            return {"level": "bad", "text": f"Gerade USB-Absturz: {names} fehlen"}
        cur = self.tracker.current
        if cur is not None:
            where = incidents[0].get("where", "") if incidents else ""
            return {"level": "bad", "text": "Gerade USB-Vorfall" + (f": {where}" if where else "")}
        recent = [e for e in incidents if _clamp(now - e.get("start_mono", now)) <= config.VERDICT_WINDOW_S]
        if recent:
            first = min(recent, key=lambda e: e.get("start_wall", 0.0))
            tally: dict[str, int] = {}
            for e in recent:
                if e.get("reason"):
                    tally[e["reason"]] = tally.get(e["reason"], 0) + 1
            top = max(tally, key=tally.get) if tally else "unklar"
            noun = "USB-Absturz" if len(recent) == 1 else "USB-Abstürze"
            since = texts.fmt_clock(first.get("start_wall", self.boot_wall))
            return {"level": "warn", "text": f"Seit {since} {len(recent)} {noun}, wahrscheinlich {top}"}
        ends = [e["end_mono"] for e in incidents if isinstance(e.get("end_mono"), (int, float))]
        since_s = now - max(ends) if ends else now - self.start_mono
        return {"level": "ok", "text": f"Alles stabil seit {texts.fmt_short(since_s)}"}

    # motor block

    def _latest_drive(self):
        for t, cmd in reversed(self.commands):
            if cmd.name in DRIVE_COMMANDS:
                return t, cmd
        return None

    @staticmethod
    def _command_text(cmd) -> str:
        if cmd.name == "MOTION":
            p = cmd.params
            return (f"vorwärts/rückwärts {texts.fmt_num(p.get('vx', 0.0))} m/s · "
                    f"seitwärts {texts.fmt_num(p.get('vy', 0.0))} m/s · "
                    f"Drehung {texts.fmt_num(p.get('vz', 0.0))} rad/s")
        if cmd.name == "CAR_RUN":
            return "Fahrmodus gestartet"
        return _speed(cmd)[1] or "kein Fahrbefehl"

    def _capture_text(self, now: float) -> str:
        if self._board() is None:
            return texts.CAPTURE_TEXTS["no_board"]
        if self._camera_on_board_bus():
            return texts.CAPTURE_TEXTS["camera_on_bus"]
        if self.capture == "on":
            live = self.last_urb is not None and _clamp(now - self.last_urb) < config.CAPTURE_IDLE_S
            return texts.CAPTURE_TEXTS["active" if live else "idle"]
        return texts.CAPTURE_TEXTS.get(self.capture, texts.CAPTURE_TEXTS["unavailable"])

    def _motor(self, now: float, incidents: list[dict]) -> dict:
        drive = self._latest_drive()
        running = bool(drive and _clamp(now - drive[0]) <= config.MOTOR_WINDOW_S
                       and drive[1].name in ("MOTOR", "MOTION") and self._drive_active(drive[1]))
        mcu = next((t for t in self.layout.tiles if t.kind == "mcu"), None)
        if self._board() is None:
            if mcu is not None and mcu.id in self.missing_since:
                state = f"fehlt seit {texts.fmt_clock(self.boot_wall + self.missing_since[mcu.id])}"
            else:
                state = texts.CAPTURE_TEXTS["no_board"]
        elif self._camera_on_board_bus():
            state = texts.CAPTURE_TEXTS["camera_on_bus"]
        elif running:
            state = "Motoren laufen"
        elif self.capture == "on":
            state = "Motoren stehen"
        else:
            state = "Zustand unbekannt"
        batt = self.battery[-1][1] if self.battery else None
        judge, _level = texts.judge_battery(batt)
        v = self.last_report_vals
        reported = ("keine Meldung" if not v else
                    f"vorwärts/rückwärts {texts.fmt_num(v.get('vx', 0.0))} m/s · "
                    f"seitwärts {texts.fmt_num(v.get('vy', 0.0))} m/s · "
                    f"Drehung {texts.fmt_num(v.get('vz', 0.0))} rad/s")
        last_cmd = ("noch keiner" if self.last_command_mono is None
                    else f"vor {texts.fmt_short(now - self.last_command_mono)}")
        with_motor = [e for e in incidents if any(r["key"] == "motor_start" for r in e.get("reasons", []))]
        near = (f"{len(with_motor)} von {len(incidents)} Vorfällen kurz nach Motorstart" if incidents
                else "keine Vorfälle")
        return {
            "state": state,
            "battery": "unbekannt" if batt is None else f"{texts.fmt_num(batt)} V – {judge}",
            "battery_curve": [round(b, 1) for _t, b in list(self.battery)[-config.BATTERY_CURVE_POINTS:]],
            "commanded": self._command_text(drive[1]) if drive else "kein Fahrbefehl",
            "reported": reported,
            "last_command": last_cmd,
            "near_motor_start": near,
            "capture": self._capture_text(now),
        }

    @staticmethod
    def _drive_active(cmd) -> bool:
        return _speed(cmd)[0] > 0

    # details

    def _history(self, tile_id: str, now: float, incidents: list[dict]) -> list[int]:
        hist = [0] * config.HISTORY_BUCKETS
        for e in incidents:
            if tile_id not in e.get("tiles", []):
                continue
            age = _clamp(now - e.get("start_mono", now))
            idx = config.HISTORY_BUCKETS - 1 - int(age / config.HISTORY_BUCKET_S)
            if 0 <= idx < config.HISTORY_BUCKETS:
                hist[idx] += 1
        return hist

    def _rate(self, ring, now: float) -> float:
        total = sum(n for t, n in ring if 0 <= now - t <= config.TRAFFIC_WINDOW_S)
        return total / config.TRAFFIC_WINDOW_S

    def _detail(self, tile: Tile, now: float, incidents: list[dict], motor: dict) -> dict:
        tile_id = tile.id
        mcu = next((t for t in self.layout.tiles if t.kind == "mcu"), None)
        source = mcu.id if tile.kind == "motor" and mcu else tile_id
        touching = next((e for e in incidents if source in e.get("tiles", [])), None)
        reasons = [{"title": r["title"], "evidence": r["evidence"], "action": r["action"]}
                   for r in (touching or {}).get("reasons", [])]
        events = [{"time": texts.fmt_clock(w), "text": t}
                  for w, t, _raw in reversed(self.tile_events.get(source, ()))][:config.TILE_TECHNICAL_LINES]
        technical = [raw for _w, _t, raw in self.tile_events.get(source, ()) if raw][-config.TILE_TECHNICAL_LINES:]
        dev = self._direct(tile) if tile.kind != "motor" else None
        if dev is not None:
            technical.append(f"path={dev.path} id={dev.vid}:{dev.pid} dev={dev.devnum} bus={dev.busnum} "
                             f"driver={dev.driver} tty={dev.tty}")
        if tile.kind == "motor":
            rows = [{"label": "Zustand", "value": motor["state"]},
                    {"label": "Befohlen", "value": motor["commanded"]},
                    {"label": "Gemeldet", "value": motor["reported"]},
                    {"label": "Akku", "value": motor["battery"]},
                    {"label": "Letzter Fahrbefehl", "value": motor["last_command"]},
                    {"label": "Vorfälle", "value": motor["near_motor_start"]}]
        elif tile.kind == "mcu":
            rows = self._mcu_rows(tile, now, motor)
        else:
            rows = self._port_rows(tile, now, dev)
        title = tile.label if tile.module != "pi" else f"{texts.MODULE_TITLES['pi']} · {tile.label}"
        return {"title": title, "rows": rows, "history": self._history(source, now, incidents),
                "events": events, "reasons": reasons, "technical": technical}

    def _mcu_rows(self, tile: Tile, now: float, motor: dict) -> list[dict]:
        dev = self._direct(tile)
        if dev is None:
            state = (f"fehlt seit {texts.fmt_clock(self.boot_wall + self.missing_since[tile.id])}"
                     if tile.id in self.missing_since else "nichts angeschlossen")
        else:
            state = "angeschlossen"
        if dev is not None and self._camera_on_board_bus():
            reports = texts.CAPTURE_TEXTS["camera_on_bus"]
        elif self.capture == "on" and dev is not None:
            hz = len([t for t in self.reports if 0 <= now - t <= config.TRAFFIC_WINDOW_S]) / config.TRAFFIC_WINDOW_S
            reports = f"{texts.fmt_num(hz)} pro Sekunde – " + ("gut" if hz > 0 else "keine Meldungen")
        else:
            reports = "unbekannt (kein Mithören)"
        return [{"label": "Zustand", "value": state},
                {"label": "Meldungen vom Board", "value": reports},
                {"label": "Akku", "value": motor["battery"]},
                {"label": "Letzter Fahrbefehl", "value": motor["last_command"]}]

    def _port_rows(self, tile: Tile, now: float, dev: DeviceInfo | None) -> list[dict]:
        rows = []
        if dev is None:
            known = self.last_device.get(tile.id)
            rows.append({"label": "Angeschlossen", "value": f"{known} (fehlt)" if known else "nichts"})
        else:
            value = (f"{device_name(dev)} · {texts.fmt_speed_link(dev.speed_mbps)} · "
                     f"zieht bis zu {dev.max_power_ma} mA")
            if dev.driver in texts.DRIVER_WORDS:
                value += f" · {texts.DRIVER_WORDS[dev.driver]}"
            rows.append({"label": "Angeschlossen", "value": value})
            behind = [device_name(d) for p, d in sorted(self.topo.devices.items())
                      if any(p.startswith(lane + ".") for lane in tile.lanes)]
            rows.append({"label": "Dahinter", "value": ", ".join(behind) if behind else "nichts"})
        port = next((self.topo.ports[lane] for lane in tile.lanes if lane in self.topo.ports), None)
        if port is not None:
            rows.append({"label": "Zustand", "value": texts.PORT_STATE_WORDS.get(port.state, "unbekannt")})
            over = f"{port.over_current}× seit Start" if port.over_current else "keine"
            if tile.module == "pi":
                over += f" · {texts.PI_OVERCURRENT_NOTE}"
            rows.append({"label": "Überlast", "value": over})
        if dev is not None and dev.devnum in self.traffic:
            rows.append({"label": "Verkehr", "value": texts.fmt_rate(self._rate(self.traffic[dev.devnum], now))})
        if tile.kind == "socket" and self.sample is not None:
            word, _level = texts.judge_ext5v(self.sample.ext5v)
            rows.append({"label": "Spannung", "value": f"{texts.fmt_volts(self.sample.ext5v)} – {word}"})
        return rows
