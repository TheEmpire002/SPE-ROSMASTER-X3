#!/usr/bin/env python3
"""Startet den Hilfsroboter mit einem einzigen Befehl.

    python main.py

Das Skript nimmt einem die Einrichtung ab:

1. legt bei Bedarf die virtuelle Umgebung unter ``robot/.venv`` an
2. installiert fehlende Abhängigkeiten
3. erkennt, ob echte Hardware angeschlossen ist, und wählt den Antrieb
4. startet den Server und öffnet die Web-App im Browser

Es läuft mit der *system-weiten* Python-Installation und wechselt für den
eigentlichen Server in die virtuelle Umgebung. Deshalb darf es selbst keine
Pakete importieren, die erst dort installiert werden.

Nützliche Optionen:

    python main.py --mock          Hardware erzwungen simulieren
    python main.py --real          Hardware erzwingen (Fehler ohne Rosmaster_Lib)
    python main.py --no-browser    Browser nicht öffnen
    python main.py --setup-only    nur einrichten, nicht starten
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

# Sprachbefehle: muss zu SPRACHE_MODELL in config.py passen. Hier doppelt,
# weil main.py nichts aus der virtuellen Umgebung importieren darf.
VOSK_MODELL = "vosk-model-small-de-0.15"
VOSK_URL = f"https://alphacephei.com/vosk/models/{VOSK_MODELL}.zip"


# --- Ausgabe -------------------------------------------------------------------

def enable_utf8_output() -> None:
    """Sorgt dafür, dass Umlaute auf der Windows-Konsole lesbar bleiben.

    Die deutsche Windows-Konsole nutzt standardmäßig cp1252 und wirft bei
    Zeichen außerhalb davon einen UnicodeEncodeError -- das Programm würde
    an einer Textausgabe sterben.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError):
            pass  # Umgeleitete Ausgabe o.Ä. -- dann eben nicht.


def step(text: str) -> None:
    print(f"\n  {text}")


def info(text: str) -> None:
    print(f"    {text}")


def fail(text: str) -> None:
    # stdout zuerst leeren, sonst erscheint die Fehlermeldung auf der
    # Konsole vor den Zeilen, die eigentlich davor kommen.
    sys.stdout.flush()
    print(f"\n  Fehler: {text}", file=sys.stderr)
    sys.stderr.flush()


# --- Virtuelle Umgebung ---------------------------------------------------------

def venv_python() -> Path:
    """Pfad zum Python in der virtuellen Umgebung."""
    if platform.system() == "Windows":
        return VENV_DIR / "Scripts" / "python.exe"
    return VENV_DIR / "bin" / "python"


def ensure_venv() -> Path:
    """Legt die virtuelle Umgebung an, falls sie fehlt."""
    py = venv_python()
    if py.exists():
        return py

    step("Lege virtuelle Umgebung an …")
    # --system-site-packages ist auf dem Roboter wichtig: Yahboons
    # Rosmaster_Lib ist dort system-weit installiert und wäre in einer
    # abgeschotteten Umgebung nicht sichtbar.
    subprocess.run(
        [sys.executable, "-m", "venv", "--system-site-packages", str(VENV_DIR)],
        check=True,
    )
    info(f"angelegt unter {VENV_DIR}")
    return py


def has_module(py: Path, module: str) -> bool:
    """Prüft, ob ein Modul in der virtuellen Umgebung importierbar ist."""
    result = subprocess.run(
        [str(py), "-c", f"import {module}"],
        capture_output=True,
    )
    return result.returncode == 0


def ensure_dependencies(py: Path) -> None:
    """Installiert das Paket, falls es noch nicht eingerichtet ist."""
    # Jedes Paket einzeln prüfen, nicht nur das erste: kommt eine neue
    # Abhängigkeit dazu (zuletzt rplidar), muss ein bestehendes Setup sie
    # beim nächsten Start nachinstallieren -- sonst fehlt sie stillschweigend.
    module = ("rosmaster_assist", "fastapi", "numpy", "rplidar")
    if all(has_module(py, m) for m in module):
        info("Abhängigkeiten vorhanden")
        return

    step("Installiere Abhängigkeiten (dauert beim ersten Mal etwas) …")
    subprocess.run(
        [str(py), "-m", "pip", "install", "--quiet", "--editable", str(ROBOT_DIR)],
        check=True,
    )
    info("fertig")


def ensure_sprache(py: Path) -> None:
    """Richtet die Sprachbefehle ein (Pakete und Vosk-Modell), soweit möglich.

    Bewusst ohne Abbruch: Fehlt etwas oder ist kein Internet da, startet
    der Server trotzdem, nur ohne Sprache. /sprache/status zeigt den Grund.
    """
    step("Sprachbefehle")
    # Nur auf "ist installiert" prüfen, nicht importieren: Ohne
    # libportaudio2 schlägt der Import fehl, ein erneutes pip install
    # hilft dann nicht und kostete bei jedem Start Zeit.
    fehlt = [m for m in ("vosk", "sounddevice")
             if not has_module(py, f"importlib.util as u; assert u.find_spec('{m}')")]
    if fehlt:
        info(f"Installiere {', '.join(fehlt)} …")
        result = subprocess.run(
            [str(py), "-m", "pip", "install", "--quiet", "--editable", f"{ROBOT_DIR}[sprache]"],
        )
        if result.returncode != 0:
            info("Installation fehlgeschlagen (Internet?) -- Server startet ohne Sprache.")
            return
    if not has_module(py, "sounddevice"):
        info("PortAudio fehlt -- einmalig: sudo apt install libportaudio2")
        return

    modell = Path.home() / VOSK_MODELL
    if not modell.is_dir():
        info(f"Lade Vosk-Modell (~45 MB) nach {modell} …")
        try:
            # Erst in eine temporäre Datei, damit ein abgebrochener Download
            # kein halbes Modell hinterlässt, das beim nächsten Start als
            # vorhanden gälte.
            with tempfile.TemporaryDirectory() as tmp:
                zip_pfad = Path(tmp) / "modell.zip"
                with urllib.request.urlopen(VOSK_URL, timeout=30) as antwort,                         open(zip_pfad, "wb") as ziel:
                    while block := antwort.read(1 << 20):
                        ziel.write(block)
                with zipfile.ZipFile(zip_pfad) as z:
                    z.extractall(tmp)
                # move statt rename: /tmp ist am Pi oft ein eigenes
                # Dateisystem (tmpfs), rename ginge dann nicht.
                shutil.move(str(Path(tmp, VOSK_MODELL)), str(modell))
        except Exception as exc:  # kein Netz, voller Speicher, ...
            info(f"Download fehlgeschlagen ({exc}) -- Server startet ohne Sprache.")
            return
    info("bereit")


# --- Hardware-Erkennung -----------------------------------------------------------

# Fragt die Bibliothek selbst, statt nach Gerätedateien zu suchen.
#
# Ein Suchen nach /dev/ttyUSB* & Co. ist hier der falsche Weg: je nach Image
# und Anschlussart heißt das Gerät anders, und Rosmaster_Lib findet es ohne
# Zutun. Wenn ``Rosmaster()`` sich öffnen lässt, ist die Hardware da -- das
# ist die einzige Auskunft, auf die man sich verlassen kann.
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
    """Entscheidet, ob echte Hardware verwendet werden kann.

    Gibt (hardware_verfügbar, Begründung) zurück.
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

    # Die Bibliothek gibt beim Start selbst etwas aus, deshalb die
    # Ergebniszeile gezielt heraussuchen statt die Ausgabe zu nehmen.
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
    """Öffnet den Browser, sobald der Server vermutlich bereit ist."""

    def run() -> None:
        time.sleep(delay)
        try:
            webbrowser.open(url)
        except Exception:
            pass  # Kein Browser vorhanden (z.B. über SSH) -- nicht schlimm.

    threading.Thread(target=run, daemon=True).start()


def port_in_use(port: int) -> bool:
    """Ob auf dem Port schon etwas lauscht.

    Ohne diese Prüfung scheitert uvicorn mit einer schwer lesbaren
    Socket-Fehlermeldung -- typischerweise, weil noch ein alter Server läuft.
    """
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def has_display() -> bool:
    """Ob überhaupt eine grafische Oberfläche da ist."""
    if platform.system() == "Windows":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


# --- Hauptablauf ----------------------------------------------------------------------

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


def main() -> int:
    enable_utf8_output()
    args = parse_args()

    print("\n  ROSMASTER X3 — Hilfsroboter")
    print("  " + "─" * 40)

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

    # --- Antrieb wählen ---
    available, reason = detect_hardware(py)

    # Ist die Rosmaster_Lib installiert, laufen wir auf dem Roboter -- auch
    # wenn das Board gerade fehlt. Am 29.10. verschwand /dev/myserial bei
    # einer USB-Überlast immer wieder für Sekunden; ob der Server echt oder
    # simuliert startete, war dann Glückssache. Der Treiber verbindet sich
    # selbst neu, sobald das Board wieder da ist.
    am_roboter = available or reason.startswith(("Board nicht erreichbar", "Hardware-Prüfung hängt"))

    if args.real and not am_roboter:
        fail(f"Hardware angefordert, aber nicht verfügbar: {reason}")
        info("Zur Diagnose: python scripts/check_hardware.py")
        return 1

    use_mock = args.mock or not am_roboter

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
        # Nur am Roboter: Im Mock werden Sprachbefehle simuliert, dort
        # braucht es weder Mikrofon noch Modell.
        ensure_sprache(py)

    if args.setup_only:
        step("Einrichtung abgeschlossen (--setup-only)")
        return 0

    # --- Server starten ---
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

    # Auch der Server soll seine Umlaute lesbar ausgeben.
    env = {**os.environ, "PYTHONIOENCODING": "utf-8"}

    try:
        code = subprocess.run(cmd, env=env).returncode
    except KeyboardInterrupt:
        # Der Serverprozess bekommt dasselbe Strg+C und fährt selbst sauber
        # herunter -- inklusive Anhalten der Motoren.
        print("\n  Beendet.\n")
        return 0
    _ende_erklaeren(code)
    return code


# Wie der Server endete -- damit "er ist einfach weg" eine Ursache bekommt.
# Negative Codes heißen unter Linux: durch dieses Signal beendet.
_ENDE_TEXTE = {
    -9: "vom System hart beendet (SIGKILL) -- meist Speicher voll oder `kill -9`.",
    -11: "Speicherzugriffsfehler (Segfault) in einer C-Bibliothek, Details in "
         "~/.rosmaster/diagnose/absturz.txt.",
    -6: "abgebrochen (SIGABRT) in einer C-Bibliothek.",
    -1: "Terminal/SSH geschlossen (SIGHUP). Tipp: den Server in `tmux` starten.",
    -15: "von außen beendet (SIGTERM).",
}


def _ende_erklaeren(code: int) -> None:
    if code in (0, 130, -2):
        return
    text = _ENDE_TEXTE.get(code, "mit einem Fehler beendet, siehe Log oben.")
    print(f"\n  Server beendet (Code {code}): {text}")
    print("  Mehr auf der Seite /diagnose unter „Abstürze & Hänger“.\n")


if __name__ == "__main__":
    sys.exit(main())
