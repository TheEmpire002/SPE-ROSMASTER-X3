"""Bounded log files: usb.log, events.jsonl, incident files and the boot summary."""

import json
import logging
import logging.handlers
import os
import time

from . import config
from .kmsg import Event

try:
    import pwd
except ImportError:  # windows
    pwd = None


def read_boot_id(path: str = config.BOOT_ID_FILE) -> str:
    try:
        with open(path, encoding="utf-8") as f:
            return f.read().strip() or config.MOCK_BOOT_ID
    except OSError:
        return config.MOCK_BOOT_ID


def _owner_ids() -> tuple[int, int] | None:
    # only root can chown, and only when the target user exists
    if pwd is None or not hasattr(os, "geteuid") or os.geteuid() != 0:
        return None
    try:
        entry = pwd.getpwnam(config.LOG_OWNER)
    except KeyError:
        return None
    return entry.pw_uid, entry.pw_gid


def _fix_perms(path: str, mode: int) -> None:
    try:
        os.chmod(path, mode)
        ids = _owner_ids()
        if ids:
            os.chown(path, *ids)
    except OSError:
        pass  # perms are cosmetic, never fail a write over them


class _PermHandler(logging.handlers.RotatingFileHandler):
    """Rotating handler whose files get the log mode and owner, also after a rollover."""

    def _open(self):
        stream = super()._open()
        _fix_perms(self.baseFilename, config.LOG_FILE_MODE)
        return stream


class Store:
    def __init__(self, folder: str, boot_id: str, file_bytes: int, file_count: int,
                 keep_incidents: int, keep_boots: int):
        self.folder = folder
        self.boot_id = boot_id
        self.file_bytes = file_bytes
        self.file_count = file_count
        self.keep_incidents = keep_incidents
        self.keep_boots = keep_boots
        self._events = os.path.join(folder, "events.jsonl")
        self._incidents = os.path.join(folder, "incidents")
        self._summary = os.path.join(folder, "summary.json")
        self._ensure_dir(folder)
        self.logger = self._make_logger()
        self._max_mono = self._read_max_mono()

    # helpers

    def _ensure_dir(self, path: str) -> bool:
        try:
            os.makedirs(path, exist_ok=True)
            _fix_perms(path, config.LOG_DIR_MODE)
            return True
        except OSError:
            return False

    def _make_logger(self) -> logging.Logger:
        # one logger per folder so several stores in one process do not share handlers
        logger = logging.getLogger(f"usbdiag.store.{os.path.abspath(self.folder)}")
        logger.setLevel(logging.INFO)
        logger.propagate = False
        for h in list(logger.handlers):
            logger.removeHandler(h)
            h.close()
        path = os.path.join(self.folder, "usb.log")
        try:
            handler = _PermHandler(
                path, maxBytes=self.file_bytes, backupCount=max(self.file_count - 1, 0),
                encoding="utf-8", delay=True)
        except OSError:
            return logger
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        logger.addHandler(handler)
        return logger

    def close(self) -> None:
        for h in list(self.logger.handlers):
            self.logger.removeHandler(h)
            h.close()

    # events.jsonl

    def _rotate_events(self) -> None:
        keep = max(self.file_count - 1, 0)
        try:
            if keep == 0:
                os.remove(self._events)
                return
            for i in range(keep, 0, -1):
                src = self._events if i == 1 else f"{self._events}.{i - 1}"
                if os.path.exists(src):
                    os.replace(src, f"{self._events}.{i}")
        except OSError as exc:
            self.logger.warning("event log rotation failed: %s", exc)

    def append_event(self, ev: Event) -> None:
        try:
            line = self._event_line(ev)
        except (TypeError, ValueError) as exc:
            self.logger.warning("event not serialisable: %s", exc)
            return
        self._write_event(line.encode("utf-8"))

    def _event_line(self, ev: Event) -> str:
        return json.dumps({
            "boot": self.boot_id, "mono": ev.mono, "wall": ev.wall, "kind": ev.kind,
            "port": ev.port, "device": ev.details.get("device", ""),
            "details": ev.details, "raw": ev.raw,
        }, ensure_ascii=False, default=str) + "\n"

    def _write_event(self, data: bytes) -> None:
        try:
            try:
                size = os.path.getsize(self._events)
            except OSError:
                size = 0
            # rotate before the write so no file ever exceeds file_bytes
            if size > 0 and size + len(data) > self.file_bytes:
                self._rotate_events()
            self._append(data)
        except OSError as exc:
            self.logger.warning("event write failed: %s", exc)

    def _append(self, data: bytes) -> None:
        flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, "O_BINARY", 0)
        try:
            fd = os.open(self._events, flags, config.LOG_FILE_MODE)
        except FileNotFoundError:
            self._ensure_dir(self.folder)  # folder vanished
            fd = os.open(self._events, flags, config.LOG_FILE_MODE)
        try:
            os.write(fd, data)
        finally:
            os.close(fd)
        _fix_perms(self._events, config.LOG_FILE_MODE)

    def _tail_max_mono(self, path: str) -> float | None:
        try:
            with open(path, "rb") as f:
                f.seek(0, os.SEEK_END)
                start = max(f.tell() - config.STORE_TAIL_BYTES, 0)
                f.seek(start)
                chunk = f.read()
        except OSError:
            return None
        lines = chunk.split(b"\n")
        if start > 0:
            lines = lines[1:]  # first line is cut off
        best = None
        for raw in lines:
            try:
                rec = json.loads(raw)
                if rec["boot"] != self.boot_id:
                    continue
                mono = float(rec["mono"])
            except (ValueError, KeyError, TypeError):
                continue  # partial or malformed line
            if best is None or mono > best:
                best = mono
        return best

    def _read_max_mono(self) -> float | None:
        best = self._tail_max_mono(self._events)
        if best is None:
            best = self._tail_max_mono(f"{self._events}.1")
        return best

    def seen_event(self, boot_id: str, mono: float, raw: str) -> bool:
        if boot_id != self.boot_id or self._max_mono is None:
            return False
        return mono <= self._max_mono

    # incidents

    def _atomic_write(self, path: str, data: dict) -> None:
        tmp = path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1, default=str)
            _fix_perms(tmp, config.LOG_FILE_MODE)
            os.replace(tmp, path)
        except BaseException:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise

    def write_incident(self, data: dict) -> str:
        name = str(data.get("id") or time.strftime("%Y-%m-%d_%H-%M-%S"))
        path = os.path.join(self._incidents, name + ".json")
        try:
            if not self._ensure_dir(self._incidents):
                raise OSError(f"cannot create {self._incidents}")
            self._atomic_write(path, data)
            self._prune_incidents()
        except (OSError, TypeError, ValueError) as exc:
            self.logger.error("incident write failed: %s", exc)
            return ""
        return path

    def _incident_names(self) -> list[str]:
        try:
            return sorted(n for n in os.listdir(self._incidents) if n.endswith(".json"))
        except OSError:
            return []

    def _prune_incidents(self) -> None:
        names = self._incident_names()
        for name in names[:max(len(names) - self.keep_incidents, 0)]:
            try:
                os.remove(os.path.join(self._incidents, name))
            except OSError:
                pass

    def load_incidents(self, boot_id: str) -> list[dict]:
        out = []
        for name in reversed(self._incident_names()):
            try:
                with open(os.path.join(self._incidents, name), encoding="utf-8") as f:
                    data = json.load(f)
                if data["boot"] == boot_id:
                    out.append(data)
            except (OSError, ValueError, KeyError, TypeError):
                continue  # unreadable or corrupt file
        return out

    # summary

    def write_summary(self, data: dict) -> None:
        try:
            boots = {}
            try:
                with open(self._summary, encoding="utf-8") as f:
                    loaded = json.load(f)["boots"]
                if isinstance(loaded, dict):
                    boots = loaded
            except (OSError, ValueError, KeyError, TypeError):
                pass  # missing or corrupt, start fresh
            boots.pop(self.boot_id, None)  # rewriting moves the boot to the end
            boots[self.boot_id] = data
            while len(boots) > self.keep_boots:
                del boots[next(iter(boots))]
            if not self._ensure_dir(self.folder):
                raise OSError(f"cannot create {self.folder}")
            self._atomic_write(self._summary, {"boots": boots})
        except (OSError, TypeError, ValueError) as exc:
            self.logger.error("summary write failed: %s", exc)
