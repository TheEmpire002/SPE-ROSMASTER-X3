"""Guided test runs for the /tests page: collect data, store it, upload it.

Why: microphone, lying and person detection should improve on the robot,
but the evaluation happens at a desk (without SSH to the Pi). A test run
guides the person through a few phases with announcements ("stand 2 m
away", "lie down across", "say: Robi, stopp"), records what the sensors
report and stores everything in one self-describing folder named after
date and time::

    ~/.rosmaster/tests/2026-11-04_14-03-12_liegend_haltungen/
        ergebnis.json   phases, samples (~5 per second), summary, plus
                        "meta": code commit, settings, timestamps, test definition
        bilder/*.jpg    a few camera images, the skeleton is in the JSON
        ton.wav         microphone tests only

Health check records from the healthbot are stored the same way as their
own run (``<date>_<time>_health``, ``typ`` "health") so they can get a note,
be zipped and uploaded like a test run.

Uploading pushes the folders to ``tests/<folder>/`` on the ``testdaten``
branch of the GitHub repo, together with a top-level ``index.json`` and
``README.md`` listing every run. It uses plain git plumbing (hash-object,
commit-tree, push) and never touches the working tree or HEAD of the Pi.
After every saved run a background upload starts on its own if a token
exists and ``config.TESTS_GIT_AUTO_UPLOAD`` is set; failed uploads are
retried with the next run or upload call.

A test run never drives, with one exception: ``mikro_fahrt`` spins the
wheels while the robot stands on its stand, to measure how much the motor
noise hurts speech recognition. It only starts after the stand has been
confirmed (``podest``), drives through ``SafetyWatchdog.submit()`` (dead
man's switch: the motor thread renews the command several times a second)
and stops on every stop path (button, app input, connection loss, end of
the test). During a microphone test the server does not execute recognized
speech commands (``block_commands``); otherwise the robot would start
moving on "Robi, folge mir".
"""

from __future__ import annotations

import asyncio
import base64
import io
import json
import logging
import math
import os
import queue
import re
import shutil
import socket
import statistics
import subprocess
import tempfile
import threading
import time
import wave
import zipfile
from datetime import datetime
from pathlib import Path

from . import config
from .health import assessment
from .protocol import DriveCommand

log = logging.getLogger(__name__)

# folder names: date_time_test; everything else is rejected so that
# ?name=... cannot leave the folder ("../")
_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_[a-z0-9_]+$")

# user name under which a test run switches pose and hog detection on
_POSE_USER = "tests"

# local bookkeeping inside a run folder; never zipped or uploaded
_UPLOADED_FILE = "hochgeladen.json"
_UPLOAD_ERROR_FILE = "upload_fehler.json"
_LOCAL_FILES = {_UPLOADED_FILE, _UPLOAD_ERROR_FILE}

# run kinds ("typ" in ergebnis.json and the list)
KIND_TEST = "test"
KIND_HEALTH = "health"

_NO_TOKEN_HINT = "kein GitHub-Token: Läufe bleiben lokal (als ZIP herunterladen oder Token einrichten)"


def _phase(name, titel, anweisung, dauer_s, **extra) -> dict:
    return {"name": name, "titel": titel, "anweisung": anweisung, "dauer_s": dauer_s, **extra}


def _motor_phases() -> list[dict]:
    """Phases of the motor-noise test: per wheel condition silence, then two commands.

    Forward and turning only: on the stand the direction does not matter for
    the noise, and on the floor there is no room behind the robot.
    """
    conditions = (
        ("aus", "Räder stehen", None),
        ("langsam", "Räder langsam", {"vx": config.TESTS_MOTOR_SLOW_MS, "vy": 0.0, "omega": 0.0}),
        ("schnell", "Räder schnell", {"vx": config.TESTS_MOTOR_FAST_MS, "vy": 0.0, "omega": 0.0}),
        ("drehen", "Räder drehen (Drehung)", {"vx": 0.0, "vy": 0.0, "omega": config.TESTS_MOTOR_OMEGA}),
    )
    phases = []
    for cond, title, drive in conditions:
        phases.append(_phase(f"{cond}_stille", f"{title}: Stille", "Bitte still sein, nur zuhören.", 5,
                             erwartet="nichts", bedingung=cond, fahrt=drive))
        for command, sentence in (("stopp", "stopp"), ("folgen", "folge mir")):
            phases.append(_phase(f"{cond}_{command}", f"{title}: „Robi, {sentence}“",
                                 f"Sag jetzt: Robi, {sentence}.", 7, erwartet=command,
                                 satz=f"Robi, {sentence}", bedingung=cond, fahrt=drive))
    return phases


# ``art`` decides what is recorded: "mikro" (level, audio, heard sentences),
# "pose" (skeletons, lying and floor signs, lidar), "personen" (like pose,
# plus hog detection), "kamera" (frame rate, frame age).
# ``erwartet`` in a phase is what would be correct; the summary counts how
# often it was
CATALOG: list[dict] = [
    {
        "id": "mikro_pegel",
        "kategorie": "mikrofon",
        "titel": "Mikrofon: Pegel und Störgeräusche",
        "kurz": "Wie laut ist es still, wie laut beim Sprechen, und hört er Befehle, wo keine sind?",
        "art": "mikro",
        "anleitung": [
            "In normalem Abstand (1-2 m) vor den Roboter stellen.",
            "Musik und Fernseher erst ausmachen; der Test sagt, wann Geräusche erlaubt sind.",
            "In der Sprech-Phase ganz normal erzählen, aber ohne 'Robi' zu sagen.",
            "Aufgezeichnet werden Pegel, alles Gehörte und der Ton als WAV.",
        ],
        "phasen": [
            _phase("stille", "Stille", "Bitte ganz still sein.", 10, erwartet="nichts"),
            _phase("sprechen", "Normal sprechen", "Erzähl etwas, aber sag nicht Robi.", 12, erwartet="nichts"),
            _phase("geraeusche", "Geräusche", "Jetzt Geräusche machen: Musik, klatschen, Stühle rücken.", 12,
                   erwartet="nichts"),
        ],
    },
    {
        "id": "mikro_befehle",
        "kategorie": "mikrofon",
        "titel": "Mikrofon: Sprachbefehle nah und fern",
        "kurz": "Versteht er die Befehle sicher? Mit Konfidenz je Wort, nah (1 m) und fern (3 m).",
        "art": "mikro",
        "anleitung": [
            "Erste Runde in etwa 1 m Abstand, zweite Runde in etwa 3 m.",
            "Nach jeder Ansage den angezeigten Satz einmal deutlich sagen, mit 'Robi' davor.",
            "Der Roboter führt die Befehle während des Tests nicht aus und fährt nicht.",
            "Aufgezeichnet werden Erkanntes, Verworfenes, Konfidenz je Wort und der Ton.",
        ],
        "phasen": [
            _phase(f"{round_}_{command}", f"{title}: „Robi, {sentence}“", f"Sag jetzt: Robi, {sentence}.", 7,
                   erwartet=command, satz=f"Robi, {sentence}")
            for round_, title in (("nah", "1 m"), ("fern", "3 m"))
            for command, sentence in (
                ("stopp", "stopp"), ("folgen", "folge mir"), ("herkommen", "komm her"),
                ("nach_hause", "nach Hause"), ("musik_aus", "Musik aus"),
            )
        ],
    },
    {
        "id": "mikro_fahrt",
        "kategorie": "mikrofon",
        "titel": "Mikrofon: Befehle bei drehenden Rädern (Podest)",
        "kurz": "Übertönen die Motoren die Befehle? Stille und zwei Befehle bei stehenden, "
                "langsamen, schnellen und drehenden Rädern.",
        "art": "mikro",
        "faehrt": True,
        "anleitung": [
            "Nur auf dem Podest! Die Räder müssen frei in der Luft drehen können.",
            "In etwa 1 m Abstand vor den Roboter stellen.",
            "Nach jeder Ansage den angezeigten Satz einmal deutlich sagen, mit 'Robi' davor.",
            "Der Roboter führt die Befehle nicht aus; die Räder drehen nur nach Plan.",
            "Stopp-Knopf auf dieser Seite oder Not-Stopp in der App beendet den Test sofort.",
            "Aufgezeichnet werden Erkanntes, Verworfenes, Konfidenz je Wort, Pegel und der Ton.",
        ],
        "phasen": _motor_phases(),
    },
    {
        "id": "liegend_haltungen",
        "kategorie": "liegend",
        "titel": "Liegend: Haltungen",
        "kurz": "Erkennt er Stehen und Sitzen als aufrecht und verschiedene Arten zu liegen als liegend?",
        "art": "pose",
        "anleitung": [
            "Etwa 2 m vor dem Roboter, mitten im Kamerabild. Platz zum Hinlegen freimachen (Decke).",
            "Den Ansagen folgen; jede Haltung ruhig halten, bis die nächste Ansage kommt.",
            "Quer = Körper von links nach rechts im Bild. Längs = Füße zum Roboter.",
            "Aufgezeichnet werden Skelette, Rumpfwinkel, Liegend- und Boden-Zeichen, Lidar und einige Bilder.",
        ],
        "phasen": [
            _phase("stehen", "Stehen", "Stell dich in zwei Metern Abstand hin.", 10, erwartet="aufrecht"),
            _phase("sitzen_stuhl", "Auf Stuhl sitzen", "Setz dich auf einen Stuhl.", 10, erwartet="aufrecht"),
            _phase("sitzen_boden", "Auf dem Boden sitzen", "Setz dich auf den Boden.", 10, erwartet="aufrecht"),
            _phase("liegen_quer", "Liegen quer", "Leg dich quer vor den Roboter, auf den Rücken.", 15,
                   erwartet="liegt"),
            _phase("liegen_laengs", "Liegen längs", "Leg dich längs hin, die Füße zum Roboter.", 15,
                   erwartet="liegt"),
            _phase("liegen_seite", "Liegen auf der Seite", "Leg dich auf die Seite, Beine angezogen.", 15,
                   erwartet="liegt"),
            _phase("liegen_bewegen", "Liegen und bewegen", "Bleib liegen und beweg Arme und Beine.", 12,
                   erwartet="liegt"),
        ],
    },
    {
        "id": "liegend_abstand",
        "kategorie": "liegend",
        "titel": "Liegend: Abstand",
        "kurz": "Ab welchem Abstand sieht er eine liegende Person noch (Kamera sitzt tief)?",
        "art": "pose",
        "anleitung": [
            "Abstände vorher auf dem Boden markieren: 1 m, 2 m, 3 m vor dem Roboter.",
            "Jeweils quer auf den Rücken legen, Körpermitte vor dem Roboter.",
        ],
        "phasen": [
            _phase("liegen_1m", "Liegen in 1 m", "Leg dich quer hin, einen Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
            _phase("liegen_2m", "Liegen in 2 m", "Leg dich quer hin, zwei Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
            _phase("liegen_3m", "Liegen in 3 m", "Leg dich quer hin, drei Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
        ],
    },
    {
        "id": "liegend_zwei_personen",
        "kategorie": "liegend",
        "titel": "Liegend: zwei Personen",
        "kurz": "Eine steht, eine liegt. Wen nimmt er? (Er nimmt die größte Person zuerst.)",
        "art": "pose",
        "anleitung": [
            "Braucht zwei Personen. Beide etwa 2 m vor dem Roboter, nebeneinander.",
            "Erst stehen beide, dann legt sich eine hin, dann tauschen.",
        ],
        "phasen": [
            _phase("beide_stehen", "Beide stehen", "Beide stellen sich nebeneinander hin.", 10, erwartet="aufrecht"),
            _phase("links_liegt", "Links liegt", "Die linke Person legt sich hin, die rechte bleibt stehen.", 15,
                   erwartet="einer_liegt"),
            _phase("rechts_liegt", "Rechts liegt", "Jetzt tauschen: rechts liegt, links steht.", 15,
                   erwartet="einer_liegt"),
        ],
    },
    {
        "id": "personen_leerer_raum",
        "kategorie": "personen",
        "titel": "Personen: leerer Raum",
        "kurz": "Sieht er Personen, wo keine sind? Jeder Fehltreffer wird mit Bild gespeichert.",
        "art": "personen",
        "anleitung": [
            "Niemand darf im Kamerabild sein, auch nicht teilweise.",
            "Für die zweite Phase den Roboter von Hand in eine andere Richtung drehen (Möbel, Fenster, Bilder).",
        ],
        "phasen": [
            _phase("blick_a", "Leerer Raum, Blick A", "Bitte aus dem Bild gehen.", 25, erwartet="niemand"),
            _phase("blick_b", "Leerer Raum, Blick B",
                   "Dreh den Roboter in eine andere Richtung und geh aus dem Bild.", 25, erwartet="niemand"),
        ],
    },
    {
        "id": "personen_abstand",
        "kategorie": "personen",
        "titel": "Personen: Abstand und Haltung",
        "kurz": "Wie zuverlässig findet er eine Person in 1-4 m, seitlich, halb verdeckt und sitzend?",
        "art": "personen",
        "anleitung": [
            "Abstände auf dem Boden markieren: 1, 2, 3 und 4 m vor dem Roboter.",
            "Bei 'halb verdeckt' hinter einen Stuhl oder Tisch stellen.",
        ],
        "phasen": [
            _phase("abstand_1m", "1 m", "Stell dich einen Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_2m", "2 m", "Stell dich zwei Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_3m", "3 m", "Stell dich drei Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_4m", "4 m", "Stell dich vier Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("seitlich", "Seitlich", "Stell dich in zwei Metern seitlich hin, Schulter zum Roboter.", 8,
                   erwartet="eine_person"),
            _phase("rand", "Am Bildrand", "Stell dich in zwei Metern an den linken Bildrand.", 8,
                   erwartet="eine_person"),
            _phase("verdeckt", "Halb verdeckt", "Stell dich halb hinter einen Stuhl oder Tisch.", 8,
                   erwartet="eine_person"),
            _phase("sitzend", "Sitzend", "Setz dich in zwei Metern auf einen Stuhl.", 8, erwartet="eine_person"),
        ],
    },
    {
        "id": "kamera_takt",
        "kategorie": "personen",
        "titel": "Kamera: Takt und Bildalter",
        "kurz": "Ruckelt die Kamera? Bildrate, Bildalter und veraltete Bilder aller Kameras.",
        "art": "kamera",
        "anleitung": [
            "Erst ruhig vor dem Roboter stehen, dann vor der Kamera hin und her gehen.",
            "Ruckeln im Videobild der App bitte notieren (Notizfeld nach dem Test).",
        ],
        "phasen": [
            _phase("ruhig", "Ruhig", "Bleib ruhig vor dem Roboter stehen.", 15, erwartet="eine_person"),
            _phase("bewegung", "Bewegung", "Geh jetzt vor der Kamera hin und her.", 15, erwartet="eine_person"),
        ],
    },
]

CATEGORIES = {
    "mikrofon": "Mikrofon",
    "liegend": "Liegend erkennen",
    "personen": "Personenerkennung",
}


def _r(value, digits: int = 3):
    """Round for the JSON (keeps the files small); None stays None."""
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return round(float(value), digits)


def _distribution(values: list[float]) -> dict | None:
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {
        "min": _r(min(values)), "median": _r(statistics.median(values)),
        "max": _r(max(values)), "mittel": _r(statistics.fmean(values)), "anzahl": len(values),
    }


def _json_default(value):
    # numpy scalars and arrays from a health record; anything else as text
    for attr in ("item", "tolist"):
        if hasattr(value, attr):
            try:
                return getattr(value, attr)()
            except (TypeError, ValueError):
                pass
    if isinstance(value, (set, tuple)):
        return list(value)
    if isinstance(value, Path):
        return str(value)
    return repr(value)[:200]


class TestRuns:
    """Runs one test after the other (own thread) and manages the run folders."""

    __test__ = False  # not a pytest class despite the name

    def __init__(
        self, pose=None, people=None, camera=None, cameras=None, lidar=None,
        voice=None, speech=None, mock: bool = False, folder: Path | None = None,
        watchdog=None,
    ) -> None:
        self._pose = pose
        self._people = people
        self._camera = camera  # the camera the pose detection looks through
        self._cameras = dict(cameras or {})
        self._lidar = lidar
        self._voice = voice
        self._speech = speech
        self._mock = mock
        if folder is None:
            folder = Path(config.TESTS_FOLDER)
            # keep simulated runs apart so they are never uploaded as real measurements
            if mock:
                folder = folder.with_name(folder.name + "_mock")
        self.folder = Path(folder)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state: dict = {"aktiv": False}
        self._last: str | None = None
        self._message: str | None = None
        self._folder_lock = threading.Lock()  # unique folder names within one second
        # upload: one at a time; a request during an upload makes the worker run again
        self._upload_lock = threading.Lock()
        self._auto_lock = threading.Lock()
        self._auto_wanted = False
        self._auto_thread: threading.Thread | None = None
        self._upload_last: dict | None = None
        self._upload_error: str | None = None
        self._code_info: dict | None = None
        # wheels for mikro_fahrt: the watchdog lives in the event loop, so
        # every call is handed over to it (``_on_loop``)
        self._watchdog = watchdog
        self._loop: asyncio.AbstractEventLoop | None = None
        self._motor_lock = threading.Lock()
        self._motor_target: DriveCommand | None = None

    # --- queries ----------------------------------------------------------------

    @staticmethod
    def catalog() -> dict:
        return {"kategorien": CATEGORIES, "tests": CATALOG}

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def block_commands(self) -> bool:
        """True during a microphone test: do not execute recognized commands."""
        with self._lock:
            return bool(self._state.get("aktiv")) and self._state.get("art") == "mikro"

    @property
    def drives(self) -> bool:
        """True while a test that spins the wheels runs."""
        with self._lock:
            return bool(self._state.get("aktiv")) and bool(self._state.get("faehrt"))

    def status(self) -> dict:
        with self._lock:
            state = dict(self._state)
        if state.get("aktiv") and state.get("ende"):
            state["rest_s"] = _r(max(0.0, state.pop("ende") - time.monotonic()), 1)
        else:
            state.pop("ende", None)
        state["letzter"] = self._last
        state["meldung"] = self._message
        state["ordner"] = str(self.folder)
        state["simulation"] = self._mock
        token = self._token() is not None
        state["token"] = token
        state["upload"] = self._upload_status(token)
        return state

    def _upload_status(self, token: bool) -> dict:
        auto = bool(config.TESTS_GIT_AUTO_UPLOAD) and not self._mock
        if not token:
            hint = _NO_TOKEN_HINT
        elif self._mock:
            hint = "Simulation: Läufe werden nicht automatisch hochgeladen"
        elif not config.TESTS_GIT_AUTO_UPLOAD:
            hint = "automatisches Hochladen ist aus"
        elif self._upload_error:
            hint = f"Hochladen misslungen, neuer Versuch beim nächsten Lauf: {self._upload_error}"
        else:
            hint = None
        return {
            "auto": auto and token,
            "laeuft": self._upload_lock.locked(),
            "ausstehend": len(self._pending_names()),
            "letzter": self._upload_last,
            "fehler": self._upload_error,
            "hinweis": hint,
        }

    def list_runs(self) -> list[dict]:
        """All stored runs, newest first (reads every folder: call in a thread)."""
        if not self.folder.is_dir():
            return []
        runs = []
        for d in sorted(self.folder.iterdir(), reverse=True):
            if not d.is_dir() or not _NAME.match(d.name):
                continue
            entry: dict = {"name": d.name}
            entry.update(self._upload_info(d))
            try:
                data = json.loads((d / "ergebnis.json").read_text(encoding="utf-8"))
                entry.update({
                    "typ": data.get("typ", KIND_TEST), "test": data.get("test"), "titel": data.get("titel"),
                    "kategorie": data.get("kategorie"), "beginn": data.get("beginn"),
                    "abgebrochen": data.get("abgebrochen"), "notiz": data.get("notiz", ""),
                    "urteil": data.get("urteil", verdict(data)),
                    "phasen": [
                        {"titel": p.get("titel"), "kurz": (p.get("zusammenfassung") or {}).get("kurz")}
                        for p in data.get("phasen", [])
                    ],
                })
            except (OSError, ValueError):
                entry["fehler"] = "ergebnis.json fehlt oder ist kaputt"
            entry["bytes"] = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
            runs.append(entry)
        return runs

    @staticmethod
    def _upload_info(d: Path) -> dict:
        info = {"hochgeladen": False, "upload_commit": None, "upload_zeit": None, "upload_fehler": None}
        try:
            done = json.loads((d / _UPLOADED_FILE).read_text(encoding="utf-8"))
            info.update(hochgeladen=True, upload_commit=done.get("commit"), upload_zeit=done.get("zeit"))
        except (OSError, ValueError):
            pass
        try:
            info["upload_fehler"] = json.loads((d / _UPLOAD_ERROR_FILE).read_text(encoding="utf-8")).get("fehler")
        except (OSError, ValueError):
            pass
        return info

    # --- control ----------------------------------------------------------------

    def start(self, test_id: str, podest: bool = False) -> str | None:
        """Start a test. Returns a (German) reason on failure, otherwise None.

        ``podest``: the user confirmed that the robot stands on its stand;
        tests that spin the wheels refuse without it.
        """
        test = next((t for t in CATALOG if t["id"] == test_id), None)
        if test is None:
            return f"unbekannter Test: {test_id!r}"
        if self.active:
            return "es läuft schon ein Test"
        missing = self._missing(test)
        if missing:
            return missing
        if test.get("faehrt") and not podest:
            return "Nur auf dem Podest: bitte bestätigen, dass die Räder frei drehen."
        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:  # started outside the server (tests, scripts)
            self._loop = None
        self._stop.clear()
        self._message = None
        with self._lock:
            self._state = {"aktiv": True, "test": test["id"], "titel": test["titel"], "art": test["art"],
                           "faehrt": bool(test.get("faehrt"))}
        self._thread = threading.Thread(target=self._run_safe, args=(test,), name="test-run", daemon=True)
        self._thread.start()
        return None

    def stop(self, reason: str | None = None) -> None:
        """Cancel the running test; ``reason`` only for the log (map_control passes one)."""
        if reason and self.active:
            log.info("Test run stopped: %s", reason)
        self._stop.set()
        if self._voice is not None:
            self._voice.mute()
        if self._motor_set(None):
            self._on_loop(self._watchdog.stop)

    # --- wheels (mikro_fahrt) ---------------------------------------------------

    def _on_loop(self, fn, *args) -> None:
        """Run ``fn`` in the event loop of the watchdog (it has no lock)."""
        loop = self._loop
        try:
            on_loop = asyncio.get_running_loop() is loop
        except RuntimeError:
            on_loop = False
        if loop is None or on_loop:
            fn(*args)
            return
        try:
            loop.call_soon_threadsafe(fn, *args)
        except RuntimeError:
            # loop closed: the server is shutting down and the watchdog has stopped the robot
            pass

    def _motor_set(self, target: DriveCommand | None) -> bool:
        """Set what the wheels should do. True if they were turning before."""
        with self._motor_lock:
            before = self._motor_target
            self._motor_target = target
        return before is not None and self._watchdog is not None

    def _motor_submit(self) -> None:
        # checked again here: a stop may have come in since the call was queued
        with self._motor_lock:
            target = self._motor_target
        if target is None or self._stop.is_set():
            return
        # "hand": the drive guard would block the autonomous path with a person
        # in front; the speeds come from config and the stand is confirmed
        self._watchdog.submit(target, source="hand")

    def _motor_loop(self, end: threading.Event) -> None:
        """Renews the drive command so the dead man's switch keeps the wheels going."""
        period = 1.0 / max(1.0, config.TESTS_MOTOR_KEEPALIVE_HZ)
        while not end.is_set() and not self._stop.is_set():
            with self._motor_lock:
                target = self._motor_target
            if target is not None:
                self._on_loop(self._motor_submit)
            end.wait(period)

    def shutdown(self) -> None:
        """When the server shuts down."""
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        # an upload in flight is a daemon thread; git has its own timeout
        if self._auto_thread is not None:
            self._auto_thread.join(timeout=1.0)

    def set_note(self, name: str, text: str) -> str | None:
        """Store a note on a run (what stood out). German reason or None."""
        d = self._folder_of(name)
        if d is None:
            return "unbekannter Lauf"
        file = d / "ergebnis.json"
        try:
            data = json.loads(file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "ergebnis.json fehlt oder ist kaputt"
        data["notiz"] = str(text)[: config.TESTS_NOTE_MAX_CHARS]
        self._write_json(file, data)
        # changed means upload again; the new state replaces the old one in the branch
        (d / _UPLOADED_FILE).unlink(missing_ok=True)
        self._auto_upload()
        return None

    def delete(self, names: list[str] | None) -> dict:
        """Delete stored runs on the robot (blocking, call in a thread).

        ``names`` None = all runs. Copies already on the testdaten branch
        stay there. Result: ``{"ok": bool, "geloescht": [...], "fehler": str|None}``.
        """
        if self.active:
            # the running test still writes into its folder
            return {"ok": False, "geloescht": [], "fehler": "Erst den laufenden Test beenden."}
        if names is None:
            folders = ([d for d in self.folder.iterdir() if d.is_dir() and _NAME.match(d.name)]
                       if self.folder.is_dir() else [])
        else:
            folders = [self._folder_of(n) for n in names]
            if any(f is None for f in folders):
                return {"ok": False, "geloescht": [], "fehler": "unbekannter Lauf in der Liste"}
        # git reads the folders while uploading
        if not self._upload_lock.acquire(blocking=False):
            return {"ok": False, "geloescht": [], "fehler": "es wird gerade hochgeladen, gleich nochmal versuchen"}
        try:
            done, failed = [], []
            for f in folders:
                try:
                    shutil.rmtree(f)
                    done.append(f.name)
                except OSError as e:
                    log.warning("Deleting test run %s failed: %s", f.name, e)
                    failed.append(f.name)
        finally:
            self._upload_lock.release()
        if self._last in done:
            self._last = None
        if not self._pending_names():
            # nothing left to retry, an old upload error would only confuse
            self._upload_error = None
        if done:
            log.info("Deleted %d test runs", len(done))
        error = f"nicht gelöscht: {', '.join(failed)}" if failed else None
        return {"ok": not failed, "geloescht": sorted(done), "fehler": error}

    def save_health_record(self, record: dict) -> str:
        """Store a healthbot record as its own run (typ "health"); returns the folder name."""
        started = record.get("started_at") if isinstance(record, dict) else None
        # folder and date from the start of the check, not from the moment it is saved
        begin = datetime.fromtimestamp(started) if isinstance(started, (int, float)) else datetime.now()
        target = self._new_folder(begin, KIND_HEALTH)
        verdict_text = _health_verdict(record)
        data = {
            "typ": KIND_HEALTH, "test": KIND_HEALTH, "titel": "Gesundheitscheck", "kategorie": KIND_HEALTH,
            "art": KIND_HEALTH, "beginn": begin.isoformat(timespec="seconds"), "simulation": self._mock,
            "einstellungen": self._settings(KIND_HEALTH),
            "abgebrochen": isinstance(record, dict) and record.get("outcome") == "cancelled",
            "phasen": [], "notiz": "", "urteil": verdict_text, "aufzeichnung": record,
            "meta": self._meta(begin, datetime.now(), None),
        }
        self._write_json(target / "ergebnis.json", data)
        log.info("Health record saved: %s", target)
        self._auto_upload()
        return target.name

    def _missing(self, test: dict) -> str | None:
        kind = test["art"]
        if test.get("faehrt") and self._watchdog is None:
            return "kein Antrieb"
        if kind == "mikro":
            if self._speech is None or not self._speech.available:
                return "Spracherkennung nicht verfügbar (siehe /sprache/status)"
        elif kind in ("pose", "personen"):
            if self._pose is None or not self._pose.available:
                return "Posenerkennung nicht verfügbar (siehe /diagnose)"
        if kind in ("personen", "kamera") and self._camera is None:
            return "keine Personenkamera"
        return None

    def _set(self, **fields) -> None:
        with self._lock:
            self._state.update(fields)

    # --- run --------------------------------------------------------------------

    def _run_safe(self, test: dict) -> None:
        try:
            self._run(test)
        except Exception as e:  # a failing test must not disturb the server
            log.exception("Test run %s failed", test["id"])
            self._message = f"Test abgebrochen: {e}"
        finally:
            with self._lock:
                self._state = {"aktiv": False}

    def _say(self, text: str) -> None:
        if self._voice is not None:
            self._voice.say(text)

    def _wait_for_voice(self) -> None:
        """Wait until the announcement is over: the ear is deaf while the robot
        speaks, and the spoken sentence would otherwise end up in the microphone."""
        if self._voice is None:
            return
        until = time.monotonic() + config.TESTS_VOICE_WAIT_S
        time.sleep(0.3)  # until the sentence has been taken from the queue
        while time.monotonic() < until and self._voice.busy and not self._stop.is_set():
            time.sleep(0.1)

    def _new_folder(self, begin: datetime, suffix: str) -> Path:
        with self._folder_lock:
            base = f"{begin:%Y-%m-%d_%H-%M-%S}_{suffix}"
            name, n = base, 2
            # a health record and a run can land in the same second
            while (self.folder / name).exists():
                name, n = f"{base}_{n}", n + 1
            target = self.folder / name
            target.mkdir(parents=True)
            return target

    def _run(self, test: dict) -> None:
        begin = datetime.now()
        target = self._new_folder(begin, test["id"])
        name = target.name
        (target / "bilder").mkdir(exist_ok=True)
        kind = test["art"]
        sight = kind in ("pose", "personen", "kamera")
        data = {
            "typ": KIND_TEST, "test": test["id"], "titel": test["titel"], "kategorie": test["kategorie"],
            "art": kind, "beginn": begin.isoformat(timespec="seconds"), "simulation": self._mock,
            "einstellungen": self._settings(kind), "anleitung": test["anleitung"],
            "phasen": [], "proben": [], "status": [], "bilder": [], "ereignisse": [], "notiz": "",
        }
        start_mono, start_unix = time.monotonic(), time.time()
        subscription = wav = sound_thread = motor_thread = None
        sound_end = threading.Event()
        motor_end = threading.Event()
        if test.get("faehrt") and self._watchdog is not None:
            motor_thread = threading.Thread(target=self._motor_loop, args=(motor_end,),
                                            name="test-run-motor", daemon=True)
            motor_thread.start()
        events: dict = {}
        if self._pose is not None and sight:
            self._pose.activate(True, _POSE_USER)
        if kind == "personen" and self._people is not None:
            # own user name: at the end only the test run signs off, a
            # concurrently running explorer keeps its detection
            self._people.activate(True, _POSE_USER)
        if kind == "mikro" and self._speech is not None and self._speech.sound_rate:
            rate = self._speech.sound_rate
            subscription = self._speech.sound_subscribe()
            wav = wave.open(str(target / "ton.wav"), "wb")
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(rate)
            # own thread: nothing is sampled during announcements, the audio
            # queue (2 s) would overflow and pieces would be missing
            sound_thread = threading.Thread(target=self._write_sound, args=(subscription, wav, sound_end),
                                            name="test-run-sound", daemon=True)
            sound_thread.start()
        data["ton"] = "ton.wav" if wav is not None else None
        images_per_phase = max(1, config.TESTS_MAX_IMAGES // max(1, len(test["phasen"])))
        try:
            self._say(f"Test {test['titel']} beginnt.")
            for nr, phase in enumerate(test["phasen"]):
                if self._stop.is_set():
                    break
                self._set(phase=nr, phasen=len(test["phasen"]), phase_titel=phase["titel"],
                          anweisung=phase["anweisung"], satz=phase.get("satz"), vorlauf=True,
                          ende=time.monotonic() + config.TESTS_LEAD_S, live=None,
                          bedingung=phase.get("bedingung"))
                if motor_thread is not None:
                    # set already in the lead time: the ramp has settled when the measurement starts
                    drive = phase.get("fahrt")
                    if self._motor_set(DriveCommand(**drive) if drive else None) and not drive:
                        self._on_loop(self._watchdog.stop)
                self._say(phase["anweisung"])
                if kind == "mikro":
                    self._wait_for_voice()
                    self._set(ende=time.monotonic() + config.TESTS_LEAD_S / 2)
                    self._stop.wait(config.TESTS_LEAD_S / 2)
                else:
                    self._stop.wait(config.TESTS_LEAD_S)
                if self._stop.is_set():
                    break
                phase_start = time.monotonic()
                phase_end = phase_start + phase["dauer_s"]
                self._set(vorlauf=False, ende=phase_end)
                entry = dict(phase)
                entry["von_s"] = _r(phase_start - start_mono, 2)
                image_times = [phase_start + phase["dauer_s"] * (i + 0.5) / images_per_phase
                               for i in range(images_per_phase)]
                next_status = 0.0
                while not self._stop.is_set():
                    now = time.monotonic()
                    if now >= phase_end:
                        break
                    sample = {"t": _r(now - start_mono, 2), "phase": phase["name"]}
                    if motor_thread is not None:
                        sample["motor"] = phase.get("bedingung")
                    if kind == "mikro":
                        sample.update(self._sample_microphone(start_unix, events))
                    if sight:
                        sample.update(self._sample_sight(kind))
                    data["proben"].append(sample)
                    self._set(live=self._live(kind, sample))
                    if sight and now >= next_status:
                        next_status = now + 1.0
                        data["status"].append({"t": sample["t"], "phase": phase["name"], **self._sight_status()})
                    # images evenly over the phase; in the empty room also
                    # every false hit (as long as there is room)
                    false_hit = phase.get("erwartet") == "niemand" and bool(
                        sample.get("personen") or sample.get("hog"))
                    due = bool(image_times) and now >= image_times[0]
                    if sight and len(data["bilder"]) < config.TESTS_MAX_IMAGES and (due or false_hit):
                        if due:
                            image_times.pop(0)
                        image = self._image(target, phase["name"], len(data["bilder"]), sample,
                                            "fehltreffer" if false_hit else "phase")
                        if image is not None:
                            data["bilder"].append(image)
                    self._stop.wait(1.0 / config.TESTS_RATE_HZ)
                entry["bis_s"] = _r(time.monotonic() - start_mono, 2)
                data["phasen"].append(entry)
            if motor_thread is not None and self._motor_set(None):
                # the wheels are done with the last phase, no need to spin while listening on
                self._on_loop(self._watchdog.stop)
            if kind == "mikro" and self._speech is not None and not self._stop.is_set():
                # a sentence from the end of a phase is only recognized after
                # the speaking pause; keep listening briefly or it is missing
                self._stop.wait(config.TESTS_TAIL_S)
                self._sample_microphone(start_unix, events)
        finally:
            if motor_thread is not None:
                if self._motor_set(None):
                    self._on_loop(self._watchdog.stop)
                motor_end.set()
                motor_thread.join(timeout=2.0)
            if self._pose is not None and sight:
                self._pose.activate(False, _POSE_USER)
            if kind == "personen" and self._people is not None:
                self._people.activate(False, _POSE_USER)
            if sound_thread is not None:
                sound_end.set()
                sound_thread.join(timeout=2.0)
            if subscription is not None:
                self._speech.sound_unsubscribe(subscription)
            if wav is not None:
                wav.close()
        # times relative to the run start (t), like the samples
        data["ereignisse"] = sorted(events.values(), key=lambda e: e["t"])
        data["abgebrochen"] = self._stop.is_set()
        data["dauer_s"] = _r(time.monotonic() - start_mono, 1)
        for p in data["phasen"]:
            p["zusammenfassung"] = summarize(kind, p, data)
        data["urteil"] = verdict(data)
        data["meta"] = self._meta(begin, datetime.now(), test)
        self._write_json(target / "ergebnis.json", data)
        self._last = name
        self._message = f"{'Abgebrochen' if data['abgebrochen'] else 'Fertig'}: {name}"
        log.info("Test run saved: %s", target)
        aborted = data["abgebrochen"]
        self._auto_upload()
        if not aborted:
            self._say("Test fertig. Danke!")

    def _meta(self, begin: datetime, end: datetime, test: dict | None) -> dict:
        """What is needed to understand the run later without the robot."""
        code = self._code()
        return {
            "beginn": begin.isoformat(timespec="seconds"), "ende": end.isoformat(timespec="seconds"),
            "dauer_s": _r((end - begin).total_seconds(), 1),
            "code_commit": code.get("commit"), "code_branch": code.get("branch"),
            "code_geaendert": code.get("dirty"), "rechner": socket.gethostname(),
            "simulation": self._mock, "einstellungen": self._config_snapshot(), "definition": test,
        }

    def _code(self) -> dict:
        # code only changes with a server restart, so ask git once
        if self._code_info is None:
            info: dict = {}
            try:
                repo = git_root(Path(__file__).resolve().parent)
                info["commit"] = _git(repo, ["rev-parse", "HEAD"]).strip() or None
                info["branch"] = _git(repo, ["rev-parse", "--abbrev-ref", "HEAD"]).strip() or None
                info["dirty"] = bool(_git(repo, ["status", "--porcelain", "--untracked-files=no"]).strip())
            except GitError as e:
                log.debug("Code version unknown: %s", e)
            self._code_info = info
        return self._code_info

    @staticmethod
    def _config_snapshot() -> dict:
        """All scalar settings, so old runs can be compared with new thresholds."""
        snapshot = {}
        for name in dir(config):
            if not name.isupper() or name.startswith("_") or "TOKEN" in name:
                continue
            value = getattr(config, name)
            if isinstance(value, (bool, int, float, str, type(None))):
                snapshot[name] = value
            elif isinstance(value, Path):
                snapshot[name] = str(value)
            elif isinstance(value, (list, tuple)) and all(isinstance(v, (bool, int, float, str)) for v in value):
                snapshot[name] = list(value)
        return snapshot

    @staticmethod
    def _settings(kind: str) -> dict:
        """The thresholds this run is evaluated with: without them old runs
        cannot be compared with new ones."""
        names = ["CAMERA_FPS", "PERSON_CAMERA", "POSE_MAX_FRAME_AGE_S"]
        if kind == "mikro":
            names += ["SPEECH_MIN_CONFIDENCE", "SPEECH_MIN_CONFIDENCE_MUSIC", "SPEECH_MAX_UNK_SHARE",
                      "SPEECH_MICROPHONE", "SPEECH_WAKE_WINDOW_S", "TESTS_MOTOR_SLOW_MS",
                      "TESTS_MOTOR_FAST_MS", "TESTS_MOTOR_OMEGA"]
        else:
            names += ["HEALTH_LYING_DEG", "HEALTH_LYING_ASPECT", "HEALTH_FLOOR_MIN_WIDTH_M",
                      "HEALTH_FLOOR_IMAGE_BOTTOM", "CAMERA_WIDTH", "CAMERA_HEIGHT"]
        values = {n: getattr(config, n, None) for n in names}
        return {n: v for n, v in values.items() if isinstance(v, (int, float, str, bool, type(None)))}

    def _sample_microphone(self, start_unix: float, events: dict) -> dict:
        live = self._speech.live()  # also keeps the level measurement running
        # live() only knows the last sentences, so collect them continuously
        for e in live.get("ereignisse", []):
            if e.get("zeit", 0) >= start_unix:
                events[(e["zeit"], e.get("text"))] = {**e, "t": _r(e["zeit"] - start_unix, 2)}
        self._set(gehoert=max(events.values(), key=lambda e: e["t"], default=None))
        return {"pegel": live.get("pegel"), "teil": live.get("teil") or "", "taub": live.get("taub"),
                "zustand": live.get("zustand")}

    def _points(self):
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or time.monotonic() - scan.timestamp > config.TESTS_SCAN_MAX_AGE_S:
            return None
        return scan.points()

    def _sample_sight(self, kind: str) -> dict:
        sample: dict = {}
        if self._pose is not None:
            people = self._pose.people()
            points = self._points() if kind != "kamera" else None
            fov = self._pose.fov_deg
            sample["personen"] = [self._person(p, points, fov) for p in people]
            sample["lidar"] = points is not None
        if kind == "personen" and self._people is not None:
            sample["hog"] = [{k: _r(v) for k, v in b.items()} for b in self._people.boxes()]
        return sample

    @staticmethod
    def _person(p: dict, points, fov: float) -> dict:
        """One person as the healthbot would assess it, plus raw data."""
        lying = assessment.lying(p)
        floor = assessment.floor(p, points, fov)
        return {
            "box": [_r(v) for v in p["box"]], "score": _r(p.get("score")), "winkel": _r(p.get("winkel"), 1),
            "rumpfwinkel": _r(assessment.torso_angle(p), 1),
            "liegend": lying["zustand"], "liegend_wert": lying["wert"],
            "boden": floor["zustand"], "boden_wert": floor["wert"], "boden_text": floor["text"],
            "kp": [[_r(x), _r(y), _r(c, 2)] for x, y, c in p.get("kp", [])],
        }

    def _sight_status(self) -> dict:
        st: dict = {}
        if self._pose is not None:
            ps = self._pose.status()
            st["pose"] = {k: ps.get(k) for k in ("fps", "bilder", "letzte_erkennung_vor_s", "bild_alter_s",
                                                  "veraltete_bilder", "aktiv", "fehler")}
        st["kameras"] = {}
        for name, cam in self._cameras.items():
            cs = cam.status()
            st["kameras"][name] = {k: cs.get(k) for k in ("fps_ist", "fps_soll", "letztes_bild_vor_s",
                                                          "bilder_gesamt", "letzter_fehler", "neu_geoeffnet")}
        return st

    @staticmethod
    def _live(kind: str, sample: dict) -> dict:
        """Short version for the page: what is being measured right now."""
        if kind == "mikro":
            return {"pegel": sample.get("pegel"), "teil": sample.get("teil"), "motor": sample.get("motor")}
        people = sample.get("personen") or []
        live = {"personen": len(people), "liegend": [p["liegend"] for p in people],
                "rumpfwinkel": [p["rumpfwinkel"] for p in people]}
        if "hog" in sample:
            live["hog"] = len(sample["hog"])
        return live

    def _image(self, target: Path, phase: str, nr: int, sample: dict, reason: str) -> dict | None:
        if self._camera is None:
            return None
        frame, captured = self._camera.latest_with_time()
        if not frame:
            return None
        raw, mime = frame
        ext = "png" if "png" in (mime or "") else "jpg"
        file = f"bilder/{nr:02d}_{phase}.{ext}"
        (target / file).write_bytes(raw)
        age = time.monotonic() - captured if captured else None
        # the skeleton for the image lives here (same people as in the
        # sample) so it can be drawn over the image during evaluation
        return {"datei": file, "t": sample["t"], "phase": phase, "grund": reason, "bild_alter_s": _r(age, 2),
                "personen": sample.get("personen", []), "hog": sample.get("hog")}

    @staticmethod
    def _write_sound(subscription: queue.Queue, wav, end: threading.Event) -> None:
        limit = int(config.TESTS_WAV_MAX_S * wav.getframerate())
        frames = 0
        while not end.is_set() or not subscription.empty():
            try:
                raw = subscription.get(timeout=0.2)
            except queue.Empty:
                continue
            if frames < limit:
                wav.writeframes(raw)
                frames += len(raw) // 2

    @staticmethod
    def _write_json(file: Path, data: dict) -> None:
        tmp = file.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1, default=_json_default), encoding="utf-8")
        tmp.replace(file)

    def _folder_of(self, name: str) -> Path | None:
        if not isinstance(name, str) or not _NAME.match(name):
            return None
        d = self.folder / name
        return d if d.is_dir() else None

    # --- zip and upload ---------------------------------------------------------

    def zip_bytes(self, name: str) -> bytes | None:
        """The run folder as a ZIP (in memory), None for unknown names."""
        d = self._folder_of(name)
        if d is None:
            return None
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(d.rglob("*")):
                if f.is_file() and f.name not in _LOCAL_FILES and f.suffix != ".tmp":
                    z.write(f, f"{name}/{f.relative_to(d).as_posix()}")
        return buffer.getvalue()

    @staticmethod
    def _token() -> str | None:
        try:
            token = Path(config.TESTS_GIT_TOKEN_FILE).read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return token or None

    def _pending_names(self) -> list[str]:
        """Finished runs not uploaded yet (a run still being recorded has no ergebnis.json)."""
        if not self.folder.is_dir():
            return []
        return sorted(
            d.name for d in self.folder.iterdir()
            if d.is_dir() and _NAME.match(d.name) and (d / "ergebnis.json").is_file()
            and not (d / _UPLOADED_FILE).exists()
        )

    def upload(self, names: list[str] | None = None) -> dict:
        """Push runs to the testdaten branch (blocking, call in a thread).

        ``names`` None = all that are not uploaded yet. Runs still pending
        from an earlier failed upload are always included. Result:
        ``{"ok": bool, "hochgeladen": [...], "fehler": str|None, "commit": sha}``.
        """
        pending = self._pending_names()
        if names is None:
            names = pending
        else:
            names = list(dict.fromkeys([*names, *pending]))
        folders = [self._folder_of(n) for n in names]
        if any(f is None for f in folders):
            return {"ok": False, "hochgeladen": [], "fehler": "unbekannter Lauf in der Liste", "commit": None}
        if not folders:
            return {"ok": True, "hochgeladen": [], "fehler": None, "commit": None}
        token = self._token()
        if token is None:
            return {"ok": False, "hochgeladen": [], "commit": None, "fehler": (
                f"kein GitHub-Token in {config.TESTS_GIT_TOKEN_FILE} (Anleitung auf der Seite); "
                "solange die Läufe als ZIP herunterladen")}
        if not self._upload_lock.acquire(blocking=False):
            return {"ok": False, "hochgeladen": [], "fehler": "es wird schon hochgeladen", "commit": None}
        try:
            # a note written during the upload must not be marked as uploaded
            stamps = {f: self._stamp(f) for f in folders}
            try:
                repo = git_root(Path(__file__).resolve().parent)
                url = config.TESTS_GIT_URL or https_url(_git(repo, ["remote", "get-url", "origin"]).strip())
                commit = upload_git(repo, url, token, config.TESTS_GIT_BRANCH, folders)
            except (GitError, OSError) as e:
                error = str(e)
                log.warning("Uploading test runs failed: %s", error)
                self._upload_error = error
                now = datetime.now().isoformat(timespec="seconds")
                for f in folders:
                    self._write_json(f / _UPLOAD_ERROR_FILE, {"zeit": now, "fehler": error})
                return {"ok": False, "hochgeladen": [], "fehler": error, "commit": None}
            now = datetime.now().isoformat(timespec="seconds")
            done = []
            for f in folders:
                (f / _UPLOAD_ERROR_FILE).unlink(missing_ok=True)
                if self._stamp(f) != stamps[f]:
                    continue  # changed meanwhile: stays pending for the next upload
                self._write_json(f / _UPLOADED_FILE, {"zeit": now, "commit": commit,
                                                      "branch": config.TESTS_GIT_BRANCH})
                done.append(f.name)
            self._upload_error = None
            self._upload_last = {"zeit": now, "commit": commit, "anzahl": len(done)}
            log.info("%d test runs uploaded (%s)", len(done), commit[:8])
            return {"ok": True, "hochgeladen": done, "fehler": None, "commit": commit}
        finally:
            self._upload_lock.release()

    @staticmethod
    def _stamp(folder: Path) -> int | None:
        try:
            return (folder / "ergebnis.json").stat().st_mtime_ns
        except OSError:
            return None

    def _auto_upload(self) -> None:
        """Upload pending runs in the background; never blocks the caller."""
        if not config.TESTS_GIT_AUTO_UPLOAD or self._mock or self._token() is None:
            return  # the hint in status() says why
        with self._auto_lock:
            self._auto_wanted = True
            if self._auto_thread is not None and self._auto_thread.is_alive():
                return  # the running worker picks the request up before it ends
            self._auto_thread = threading.Thread(target=self._auto_worker, name="test-runs-upload", daemon=True)
            self._auto_thread.start()

    def _auto_worker(self) -> None:
        while True:
            with self._auto_lock:
                if not self._auto_wanted:
                    self._auto_thread = None
                    return
                self._auto_wanted = False
            try:
                result = self.upload()
                if result["fehler"] == "es wird schon hochgeladen":
                    # a manual upload is running; try again once it is done
                    with self._upload_lock:
                        pass
                    with self._auto_lock:
                        self._auto_wanted = True
            except Exception:  # offline, full disk, anything: the robot keeps working
                log.exception("Background upload of test runs failed")


# --- verdicts and summaries ---------------------------------------------------------


def _health_verdict(record: dict) -> str | None:
    """Short German verdict of a healthbot record for the list and the index."""
    if not isinstance(record, dict):
        return None
    # healthbot records (procedure.Healthbot._record) end in "outcome"; only
    # finished checks have a verdict worth showing
    outcome = record.get("outcome")
    if outcome in ("cancelled", "error", "no_person"):
        labels = {"cancelled": "abgebrochen", "error": "Fehler", "no_person": "keine Person"}
        reason = record.get("reason")
        return f"{labels[outcome]}: {reason}" if reason else labels[outcome]
    for key in ("verdict", "urteil", "gesamt", "ergebnis"):
        value = record.get(key)
        if isinstance(value, str) and value:
            return value
        if isinstance(value, dict):
            level, text = value.get("stufe"), value.get("text")
            if level and text:
                return f"{level}: {text}"
            if level or text:
                return str(level or text)
    return None


def verdict(data: dict) -> str | None:
    """One German line per run: what came out (list on /tests, index on GitHub)."""
    if data.get("typ") == KIND_HEALTH:
        return data.get("urteil") or _health_verdict(data.get("aufzeichnung") or {})
    summaries = [p.get("zusammenfassung") or {} for p in data.get("phasen", [])]
    parts = []
    hits = [z["treffer"] for z in summaries if "treffer" in z]
    if hits:
        parts.append(f"{sum(hits)}/{len(hits)} Befehle erkannt")
    false_commands = [z["fehlbefehle"] for z in summaries if "fehlbefehle" in z]
    if false_commands:
        parts.append(f"{sum(false_commands)} Fehlbefehle")
    correct = [z["richtig"] for z in summaries if z.get("richtig") is not None]
    if correct:
        parts.append(f"{statistics.fmean(correct) * 100:.0f} % richtig")
    # motor-noise test: hits per wheel condition, that is the actual finding
    per_condition: dict[str, list[bool]] = {}
    for p in data.get("phasen", []):
        z = p.get("zusammenfassung") or {}
        if p.get("bedingung") and "treffer" in z:
            per_condition.setdefault(p["bedingung"], []).append(bool(z["treffer"]))
    if per_condition:
        parts.append(", ".join(f"{c} {sum(h)}/{len(h)}" for c, h in per_condition.items()))
    text = ", ".join(parts) or "keine Daten"
    return f"abgebrochen, {text}" if data.get("abgebrochen") else text


def summarize(kind: str, phase: dict, data: dict) -> dict:
    """The essentials of a phase in a few numbers, ``kurz`` as text for the list."""
    samples = [p for p in data["proben"] if p["phase"] == phase["name"]]
    expected = phase.get("erwartet")
    z: dict = {"proben": len(samples)}
    if kind == "mikro":
        z["pegel"] = _distribution([p.get("pegel") for p in samples])
        start, end = phase.get("von_s") or 0.0, phase.get("bis_s") or 0.0
        heard = [e for e in data.get("ereignisse", []) if start - 0.5 <= e.get("t", -1) <= end + 1.0]
        z["gehoert"] = [{k: e.get(k) for k in ("text", "ergebnis", "befehl", "grund", "konfidenz")}
                        for e in heard]
        commands = [e.get("befehl") for e in heard if e.get("ergebnis") == "befehl"]
        if expected == "nichts":
            z["fehlbefehle"] = len(commands)
            z["kurz"] = f"{len(heard)} Sätze gehört, {len(commands)} Befehle (sollten 0 sein)"
        else:
            z["treffer"] = expected in commands
            rejected = [e for e in heard if e.get("ergebnis") == "verworfen"]
            z["kurz"] = ("erkannt" if z["treffer"] else
                         f"nicht erkannt ({len(rejected)} verworfen, {len(heard)} Sätze)")
        return z
    counts = [len(p.get("personen") or []) for p in samples]
    z["mit_person"] = _r(sum(1 for a in counts if a) / len(counts), 2) if counts else None
    z["personen_max"] = max(counts, default=0)
    first = [p["personen"][0] for p in samples if p.get("personen")]
    z["rumpfwinkel"] = _distribution([p["rumpfwinkel"] for p in first])
    z["liegend_anteil"] = (_r(sum(p["liegend"] == assessment.FLAGGED for p in first) / len(first), 2)
                           if first else None)
    z["boden_anteil"] = (_r(sum(p["boden"] == assessment.FLAGGED for p in first) / len(first), 2)
                         if first else None)
    if kind == "personen":
        hog = [len(p.get("hog") or []) for p in samples]
        z["hog_mit_person"] = _r(sum(1 for a in hog if a) / len(hog), 2) if hog else None
    if kind == "kamera" or data.get("status"):
        st = [s for s in data.get("status", []) if s["phase"] == phase["name"]]
        z["pose_fps"] = _distribution([(s.get("pose") or {}).get("fps") for s in st])
        z["bild_alter_s"] = _distribution([(s.get("pose") or {}).get("bild_alter_s") for s in st])
    # "richtig": share of samples whose result matches the phase
    if expected == "niemand":
        z["richtig"] = _r(1.0 - (z["mit_person"] or 0.0), 2) if counts else None
    elif expected == "eine_person":
        z["richtig"] = z["mit_person"]
    elif expected in ("aufrecht", "liegt"):
        target = assessment.FLAGGED if expected == "liegt" else assessment.OK
        z["richtig"] = _r(sum(p["liegend"] == target for p in first) / len(samples), 2) if samples else None
    elif expected == "einer_liegt":
        hits = sum(1 for p in samples
                   if any(q["liegend"] == assessment.FLAGGED for q in p.get("personen") or []))
        z["richtig"] = _r(hits / len(samples), 2) if samples else None
        z["liegende_zuerst"] = (_r(sum(p["liegend"] == assessment.FLAGGED for p in first) / len(samples), 2)
                                if samples else None)
    parts = []
    if z.get("richtig") is not None:
        parts.append(f"{z['richtig'] * 100:.0f} % richtig")
    if z["mit_person"] is not None:
        parts.append(f"Person in {z['mit_person'] * 100:.0f} % der Proben")
    if z["rumpfwinkel"]:
        parts.append(f"Rumpf median {z['rumpfwinkel']['median']:.0f}°")
    z["kurz"] = ", ".join(parts) or "keine Daten"
    return z


# --- git without a working tree ------------------------------------------------------


class GitError(RuntimeError):
    pass


def _git(repo: Path, args: list[str], env: dict | None = None, stdin: str | None = None) -> str:
    environment = {**os.environ, "GIT_TERMINAL_PROMPT": "0", **(env or {})}
    try:
        # bytes instead of text=True: on windows stdin would get crlf and
        # update-index silently ignores such paths (empty commit)
        r = subprocess.run(
            ["git", *args], cwd=repo, env=environment, capture_output=True,
            input=stdin.encode("utf-8") if stdin is not None else None,
            timeout=config.TESTS_GIT_TIMEOUT_S,
        )
    except FileNotFoundError as e:
        raise GitError("git ist nicht installiert") from e
    except subprocess.TimeoutExpired as e:
        raise GitError(f"git {args[0]} antwortet nicht (Netz?)") from e
    out = r.stdout.decode("utf-8", "replace")
    if r.returncode != 0:
        lines = (r.stderr.decode("utf-8", "replace") or out).strip().splitlines()
        # git puts the cause in the fatal line, the last line is often only a hint
        fatal = [x for x in lines if x.startswith(("fatal:", "error:"))]
        reason = (fatal or lines or [str(r.returncode)])[0 if fatal else -1]
        raise GitError(f"git {args[0]}: {reason}")
    return out


def git_root(folder: Path) -> Path:
    return Path(_git(folder, ["rev-parse", "--show-toplevel"]).strip())


def https_url(url: str) -> str:
    """git@github.com:a/b.git -> https://github.com/a/b.git (the token only travels over https)."""
    m = re.match(r"^(?:ssh://)?git@([^:/]+)[:/](.+)$", url)
    return f"https://{m.group(1)}/{m.group(2)}" if m else url


def _auth(url: str, token: str) -> dict:
    """Token as an http header via the environment: so it is neither on the
    command line (ps) nor in an error message or the git config."""
    if not token or not url.startswith("https://"):
        return {}
    basic = base64.b64encode(f"x-access-token:{token}".encode()).decode()
    return {"GIT_CONFIG_COUNT": "1", "GIT_CONFIG_KEY_0": "http.extraHeader",
            "GIT_CONFIG_VALUE_0": f"Authorization: Basic {basic}"}


def _index_entry(folder: Path) -> dict:
    """One line of index.json, from the run's ergebnis.json."""
    try:
        data = json.loads((folder / "ergebnis.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        data = {}
    meta = data.get("meta") or {}
    return {
        "name": folder.name, "datum": data.get("beginn") or _date_from_name(folder.name),
        "typ": data.get("typ", KIND_TEST), "test": data.get("test"), "titel": data.get("titel"),
        "urteil": data.get("urteil", verdict(data) if data else None),
        "abgebrochen": data.get("abgebrochen"), "simulation": data.get("simulation"),
        "notiz": bool(data.get("notiz")), "code_commit": meta.get("code_commit"),
        "pfad": f"tests/{folder.name}/",
    }


def _date_from_name(name: str) -> str | None:
    try:
        return datetime.strptime(name[:19], "%Y-%m-%d_%H-%M-%S").isoformat(timespec="seconds")
    except ValueError:
        return None


def _old_index(repo: Path, parent: str | None) -> dict[str, dict]:
    """Index entries already in the branch; rebuilt from the folder list if index.json is missing."""
    if parent is None:
        return {}
    try:
        entries = json.loads(_git(repo, ["show", f"{parent}:index.json"])).get("laeufe", [])
        return {e["name"]: e for e in entries if isinstance(e, dict) and e.get("name")}
    except (GitError, ValueError, AttributeError):
        pass
    try:
        listing = _git(repo, ["ls-tree", "--name-only", parent, "tests/"]).split()
    except GitError:
        return {}
    names = [Path(p).name for p in listing if _NAME.match(Path(p).name)]
    return {n: {"name": n, "datum": _date_from_name(n), "typ": None, "test": None, "titel": None,
                "urteil": None, "pfad": f"tests/{n}/"} for n in names}


def _readme(entries: list[dict], branch: str) -> str:
    """README.md of the data branch: a table GitHub renders, newest first."""
    def cell(value) -> str:
        text = "" if value is None else str(value)
        return text.replace("|", "/").replace("\n", " ")

    lines = [
        "# Testdaten ROSMASTER X3",
        "",
        f"Automatisch erzeugt vom Roboter (Branch `{branch}`). Jeder Lauf liegt in `tests/<name>/`:",
        "`ergebnis.json` (Messwerte, Zusammenfassung, Code-Stand, Einstellungen), Bilder, Ton.",
        "Maschinenlesbar: [`index.json`](index.json).",
        "",
        f"{len(entries)} Läufe.",
        "",
        "| Datum | Art | Test | Urteil | Lauf |",
        "|---|---|---|---|---|",
    ]
    for e in entries:
        kind = "Gesundheitscheck" if e.get("typ") == KIND_HEALTH else "Test"
        title = e.get("titel") or e.get("test") or ""
        lines.append(f"| {cell(e.get('datum'))} | {kind} | {cell(title)} | {cell(e.get('urteil'))} "
                     f"| [{e['name']}]({e.get('pfad') or 'tests/' + e['name'] + '/'}) |")
    return "\n".join(lines) + "\n"


def upload_git(repo: Path, url: str, token: str, branch: str, folders: list[Path]) -> str:
    """Push folders as ``tests/<name>/`` in one commit on ``branch``, plus index.json and README.md.

    A separate index (temp file) and commit-tree instead of checkout: the Pi
    stays on its branch, its working tree untouched. If the branch was moved
    elsewhere in the meantime, it is rebuilt once.
    """
    auth = _auth(url, token)
    for attempt in range(2):
        existing = _git(repo, ["ls-remote", url, f"refs/heads/{branch}"], env=auth).strip()
        parent = None
        if existing:
            _git(repo, ["fetch", "--no-tags", url, f"+refs/heads/{branch}:refs/rosmaster/{branch}"], env=auth)
            parent = _git(repo, ["rev-parse", f"refs/rosmaster/{branch}"]).strip()
        index = _old_index(repo, parent)
        for f in folders:
            index[f.name] = _index_entry(f)
        entries = sorted(index.values(), key=lambda e: e["name"], reverse=True)
        index_json = json.dumps({"branch": branch, "erzeugt": datetime.now().isoformat(timespec="seconds"),
                                 "anzahl": len(entries), "laeufe": entries}, ensure_ascii=False, indent=1)
        with tempfile.TemporaryDirectory() as tmp:
            idx = {"GIT_INDEX_FILE": str(Path(tmp) / "index")}
            _git(repo, ["read-tree", parent] if parent else ["read-tree", "--empty"], env=idx)
            files = [(o, f) for o in folders for f in sorted(o.rglob("*"))
                     if f.is_file() and f.name not in _LOCAL_FILES and f.suffix != ".tmp"]
            shas = _git(repo, ["hash-object", "-w", "--stdin-paths"],
                        stdin="\n".join(str(f) for _, f in files) + "\n").split() if files else []
            lines = [f"100644 {sha}\ttests/{o.name}/{f.relative_to(o).as_posix()}"
                     for (o, f), sha in zip(files, shas)]
            for path, content in (("index.json", index_json), ("README.md", _readme(entries, branch))):
                sha = _git(repo, ["hash-object", "-w", "--stdin"], stdin=content).strip()
                lines.append(f"100644 {sha}\t{path}")
            _git(repo, ["update-index", "--add", "--index-info"], env=idx, stdin="\n".join(lines) + "\n")
            tree = _git(repo, ["write-tree"], env=idx).strip()
        who = {"GIT_AUTHOR_NAME": config.TESTS_GIT_AUTHOR, "GIT_AUTHOR_EMAIL": config.TESTS_GIT_EMAIL,
               "GIT_COMMITTER_NAME": config.TESTS_GIT_AUTHOR, "GIT_COMMITTER_EMAIL": config.TESTS_GIT_EMAIL}
        message = "Testdaten: " + ", ".join(o.name for o in folders)
        commit = _git(repo, ["commit-tree", tree, *(["-p", parent] if parent else []), "-m", message],
                      env=who).strip()
        try:
            _git(repo, ["push", url, f"{commit}:refs/heads/{branch}"], env=auth)
            return commit
        except GitError as e:
            if attempt == 0 and ("rejected" in str(e) or "fetch first" in str(e) or "non-fast-forward" in str(e)):
                log.info("Branch %s moved meanwhile, rebuilding on top of it", branch)
                continue
            raise
    raise GitError("Hochladen nach zweitem Versuch abgelehnt")
