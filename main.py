#!/usr/bin/env python3
"""Starts the helper robot with a single command.

    python main.py

The script takes care of the setup:

1. creates the virtual environment under ``robot/.venv`` if needed
2. installs missing dependencies
3. detects whether real hardware is connected and picks the drive
4. starts the server and opens the web app in the browser

It runs with the *system-wide* Python installation and switches to the
virtual environment for the actual server. That is why it must not import
any packages itself that only get installed there.

Useful options:

    python main.py --mock          force simulated hardware
    python main.py --real          force real hardware (fails without Rosmaster_Lib)
    python main.py --no-browser    do not open the browser
    python main.py --setup-only    only set up, do not start
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import webbrowser
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ROBOT_DIR = ROOT / "robot"
VENV_DIR = ROBOT_DIR / ".venv"

MIN_PYTHON = (3, 11)

# Speech commands: must match SPEECH_MODEL in config.py. Duplicated here
# because main.py must not import anything from the virtual environment.
VOSK_MODEL = "vosk-model-small-de-0.15"
VOSK_URL = f"https://alphacephei.com/vosk/models/{VOSK_MODEL}.zip"


# --- Output --------------------------------------------------------------------

def enable_utf8_output() -> None:
    """Makes sure umlauts stay readable on the Windows console.

    The German Windows console uses cp1252 by default and raises a
    UnicodeEncodeError for characters outside of it -- the program would
    die on a text output.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass  # Redirected output or similar -- then just skip it.


def step(text: str) -> None:
    print(f"\n  {text}")


def info(text: str) -> None:
    print(f"    {text}")


def fail(text: str) -> None:
    # Flush stdout first, otherwise the error message appears on the
    # console before the lines that actually come before it.
    sys.stdout.flush()
    print(f"\n  Fehler: {text}", file=sys.stderr)
    sys.stderr.flush()


# --- Virtual environment --------------------------------------------------------

def venv_python() -> Path:
    """Path to the Python in the virtual environment."""
    if platform.system() == "Windows":
        return VENV_DIR / "Scripts" / "python.exe"
    return VENV_DIR / "bin" / "python"


def ensure_venv() -> Path:
    """Creates the virtual environment if it is missing."""
    py = venv_python()
    if py.exists():
        return py

    step("Lege virtuelle Umgebung an …")
    # --system-site-packages matters on the robot: Yahboom's Rosmaster_Lib
    # is installed system-wide there and would not be visible in an
    # isolated environment.
    subprocess.run(
        [sys.executable, "-m", "venv", "--system-site-packages", str(VENV_DIR)],
        check=True,
    )
    info(f"angelegt unter {VENV_DIR}")
    return py


def has_module(py: Path, module: str) -> bool:
    """Checks whether a module is importable in the virtual environment."""
    result = subprocess.run(
        [str(py), "-c", f"import {module}"],
        capture_output=True,
    )
    return result.returncode == 0


def ensure_dependencies(py: Path) -> None:
    """Installs the package if it is not set up yet."""
    # Check every package individually, not just the first: when a new
    # dependency is added (most recently rplidar), an existing setup must
    # install it on the next start -- otherwise it is silently missing.
    modules = ("rosmaster_assist", "fastapi", "numpy", "rplidar")
    if all(has_module(py, m) for m in modules):
        info("Abhängigkeiten vorhanden")
        return

    step("Installiere Abhängigkeiten (dauert beim ersten Mal etwas) …")
    subprocess.run(
        [str(py), "-m", "pip", "install", "--quiet", "--editable", str(ROBOT_DIR)],
        check=True,
    )
    info("fertig")


def ensure_speech(py: Path) -> None:
    """Sets up the speech commands (packages and Vosk model) as far as possible.

    Deliberately without aborting: if something is missing or there is no
    internet, the server starts anyway, just without speech. /sprache/status
    shows the reason.
    """
    step("Sprachbefehle")
    # Only check "is installed", do not import: without libportaudio2 the
    # import fails, a repeated pip install does not help then and would
    # cost time on every start.
    missing = [m for m in ("vosk", "sounddevice")
               if not has_module(py, f"importlib.util as u; assert u.find_spec('{m}')")]
    if missing:
        info(f"Installiere {', '.join(missing)} …")
        result = subprocess.run(
            [str(py), "-m", "pip", "install", "--quiet", "--editable", f"{ROBOT_DIR}[sprache]"],
        )
        if result.returncode != 0:
            info("Installation fehlgeschlagen (Internet?) -- Server startet ohne Sprache.")
            return
    if not has_module(py, "sounddevice"):
        info("PortAudio fehlt -- einmalig: sudo apt install libportaudio2")
        return

    model = Path.home() / VOSK_MODEL
    if not model.is_dir():
        info(f"Lade Vosk-Modell (~45 MB) nach {model} …")
        try:
            # Download into a temporary file first, so that an aborted
            # download does not leave half a model behind that would count
            # as present on the next start.
            with tempfile.TemporaryDirectory() as tmp:
                zip_path = Path(tmp) / "modell.zip"
                with urllib.request.urlopen(VOSK_URL, timeout=30) as response,                         open(zip_path, "wb") as target:
                    while block := response.read(1 << 20):
                        target.write(block)
                with zipfile.ZipFile(zip_path) as z:
                    z.extractall(tmp)
                # move instead of rename: /tmp on the Pi is often a separate
                # file system (tmpfs), rename would not work then.
                shutil.move(str(Path(tmp, VOSK_MODEL)), str(model))
        except Exception as exc:  # no network, disk full, ...
            info(f"Download fehlgeschlagen ({exc}) -- Server startet ohne Sprache.")
            return
    info("bereit")


# --- Hardware detection -----------------------------------------------------------

# Asks the library itself instead of looking for device files.
#
# Searching for /dev/ttyUSB* & co. is the wrong approach here: depending on
# the image and connection type the device has a different name, and
# Rosmaster_Lib finds it on its own. If ``Rosmaster()`` can be opened, the
# hardware is there -- that is the only answer one can rely on.
HARDWARE_PROBE = """
import sys
try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("RESULT:no-lib"); sys.exit(0)
try:
    Rosmaster()
except Exception as exc:
    print("RESULT:no-port:" + str(exc).replace("\\n", " ")[:100]); sys.exit(0)
print("RESULT:ok")
"""


def detect_hardware(py: Path) -> tuple[bool, str]:
    """Decides whether real hardware can be used.

    Returns (hardware_available, reason).
    """
    if platform.system() == "Windows":
        return False, "Windows — der Roboter läuft unter Linux"

    try:
        proc = subprocess.run(
            [str(py), "-c", HARDWARE_PROBE],
            capture_output=True,
            text=True,
            timeout=20,
        )
    except subprocess.TimeoutExpired:
        return False, "Hardware-Prüfung hängt — Expansion Board reagiert nicht"

    # The library prints something itself on startup, so pick out the
    # result line specifically instead of taking the whole output.
    result = ""
    for line in proc.stdout.splitlines():
        if line.startswith("RESULT:"):
            result = line[len("RESULT:"):]

    if result == "ok":
        return True, "Rosmaster_Lib hat das Board gefunden"
    if result == "no-lib":
        return False, "Rosmaster_Lib nicht installiert"
    if result.startswith("no-port:"):
        return False, f"Board nicht erreichbar ({result[len('no-port:'):].strip()})"

    return False, "Hardware-Prüfung ohne verwertbare Antwort"


# --- Browser ------------------------------------------------------------------------

def open_browser_later(url: str, delay: float = 2.0) -> None:
    """Opens the browser as soon as the server is probably ready."""

    def run() -> None:
        time.sleep(delay)
        try:
            webbrowser.open(url)
        except Exception:
            pass  # No browser available (e.g. over SSH) -- not a problem.

    threading.Thread(target=run, daemon=True).start()


def port_in_use(port: int) -> bool:
    """Whether something is already listening on the port.

    Without this check uvicorn fails with a hard-to-read socket error
    message -- typically because an old server is still running.
    """
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def has_display() -> bool:
    """Whether a graphical desktop is available at all."""
    if platform.system() == "Windows":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


# --- Main flow ------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Startet den ROSMASTER-X3-Hilfsroboter",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--mock", action="store_true", help="Hardware simulieren")
    mode.add_argument("--real", action="store_true", help="Hardware erzwingen")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--no-browser", action="store_true", help="Browser nicht öffnen")
    p.add_argument("--setup-only", action="store_true", help="nur einrichten")
    p.add_argument("--debug", action="store_true", help="ausführliche Logausgabe")
    return p.parse_args()


def _home_under_sudo() -> None:
    """Started with ``sudo``: still point ``~`` to the actual user.

    Under sudo HOME=/root. Then Piper (~/piper), the Vosk model and
    ~/.rosmaster are missing -- on 03.11. the robot therefore spoke with
    espeak again instead of Piper. The server inherits the environment, so
    doing it here is enough.
    """
    user = os.environ.get("SUDO_USER")
    if not user or not hasattr(os, "geteuid") or os.geteuid() != 0:
        return
    import pwd
    try:
        home = pwd.getpwnam(user).pw_dir
    except KeyError:
        return
    os.environ["HOME"] = home
    info(f"Mit sudo gestartet -- nutze trotzdem {home} (Piper, Vosk, Diagnose).")
    info("sudo ist nicht nötig: python3 main.py reicht.")


def main() -> int:
    enable_utf8_output()
    args = parse_args()

    print("\n  ROSMASTER X3 — Hilfsroboter")
    print("  " + "─" * 40)
    _home_under_sudo()

    if sys.version_info < MIN_PYTHON:
        fail(
            f"Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]} oder neuer nötig, "
            f"gefunden: {platform.python_version()}"
        )
        return 1

    if not ROBOT_DIR.is_dir():
        fail(f"Ordner 'robot' nicht gefunden neben {Path(__file__).name}")
        return 1

    try:
        py = ensure_venv()
        ensure_dependencies(py)
    except subprocess.CalledProcessError as exc:
        fail(f"Einrichtung fehlgeschlagen ({exc})")
        return 1

    # --- Choose the drive ---
    available, reason = detect_hardware(py)

    # If Rosmaster_Lib is installed, we are running on the robot -- even if
    # the board is currently missing. On 29.10. /dev/myserial kept vanishing
    # for seconds during a USB over-current; whether the server started real
    # or simulated was then a matter of luck. The driver reconnects on its
    # own as soon as the board is back.
    on_robot = available or reason.startswith(("Board nicht erreichbar", "Hardware-Prüfung hängt"))

    if args.real and not on_robot:
        fail(f"Hardware angefordert, aber nicht verfügbar: {reason}")
        info("Zur Diagnose: python scripts/check_hardware.py")
        return 1

    use_mock = args.mock or not on_robot

    step("Antrieb")
    if use_mock:
        info("Simulation " + ("(erzwungen mit --mock)" if args.mock else f"({reason})"))
        info("Es bewegt sich nichts.")
    else:
        if available:
            info(f"Echte Hardware ({reason})")
        else:
            info(f"Echte Hardware -- Achtung: {reason}.")
            info("Der Server verbindet sich, sobald das Board wieder da ist "
                 "(USB-Überlast? siehe /diagnose).")
        info("Der Roboter kann sich bewegen. Freie Fläche sicherstellen!")
        # Only on the robot: in mock mode speech commands are simulated,
        # so neither a microphone nor a model is needed there.
        ensure_speech(py)

    if args.setup_only:
        step("Einrichtung abgeschlossen (--setup-only)")
        return 0

    # --- Start the server ---
    if port_in_use(args.port):
        fail(f"Auf Port {args.port} läuft bereits etwas.")
        info("Vermutlich ein älterer Server. Entweder dort Strg+C drücken,")
        info(f"oder hier einen anderen Port wählen:  --port {args.port + 1}")
        return 1

    url = f"http://localhost:{args.port}"
    step(f"Starte Server auf {url}")
    info("Beenden mit Strg+C")
    print()

    if not args.no_browser and has_display():
        open_browser_later(url)

    cmd = [str(py), "-m", "rosmaster_assist.main",
           "--host", args.host, "--port", str(args.port)]
    if use_mock:
        cmd.append("--mock")
    if args.debug:
        cmd.append("--debug")

    # The server should print its umlauts readably too.
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}

    try:
        code = subprocess.run(cmd, env=env).returncode
    except KeyboardInterrupt:
        # The server process gets the same Ctrl+C and shuts down cleanly on
        # its own -- including stopping the motors.
        print("\n  Beendet.\n")
        return 0
    _explain_exit(code)
    return code


# How the server ended -- so that "it is just gone" gets a cause.
# On Linux negative codes mean: terminated by this signal.
_EXIT_TEXTS = {
    -9: "vom System hart beendet (SIGKILL) -- meist Speicher voll oder `kill -9`.",
    -11: "Speicherzugriffsfehler (Segfault) in einer C-Bibliothek, Details in "
         "~/.rosmaster/diagnose/absturz.txt.",
    -6: "abgebrochen (SIGABRT) in einer C-Bibliothek.",
    -1: "Terminal/SSH geschlossen (SIGHUP). Tipp: den Server in `tmux` starten.",
    -15: "von außen beendet (SIGTERM).",
}


def _explain_exit(code: int) -> None:
    if code in (0, 130, -2):
        return
    text = _EXIT_TEXTS.get(code, "mit einem Fehler beendet, siehe Log oben.")
    print(f"\n  Server beendet (Code {code}): {text}")
    print("  Mehr auf der Seite /diagnose unter „Abstürze & Hänger“.\n")


if __name__ == "__main__":
    sys.exit(main())
