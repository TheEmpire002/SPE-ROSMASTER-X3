"""Absturzerfassung -- hält fest, was einen Absturz überlebt.

Am 29.10. hieß es "der Roboter stürzt random ab", auch am Netzteil und am
anderen Board. Aus der Ferne ließ sich nicht sagen, *was* da abstürzt:

- der Server (Python-Fehler, Speicher voll, Segfault im Hailo-Treiber,
  SSH-Fenster zu),
- der ganze Pi (Strom weg, Reset, Kernel hängt),
- oder nur ein Hänger, bei dem der Server noch läuft, aber nicht reagiert.

Dieses Modul schreibt deshalb in ``config.DIAGNOSE_ORDNER`` mit:

- ``laeufe.json``: jeder Serverstart mit Boot-ID, letztem Lebenszeichen und
  wie er endete. Fehlt das Ende, ist der Lauf abgestürzt; die Boot-ID
  verrät, ob der Pi dabei weiterlief.
- ``server.log``: das Server-Log als Datei, damit es nach einem Absturz
  noch da ist (die Terminalausgabe ist dann weg).
- ``absturz.txt``: Stack-Dump von ``faulthandler`` bei einem Segfault.
- ``blockiert.txt``: woran die Ereignisschleife hing, wenn sie stand.

Alles hier ist "best effort": Fehlt etwas (Windows, kein journalctl,
Ordner nicht beschreibbar), geht der Rest trotzdem.
"""

from __future__ import annotations

import json
import logging
import logging.handlers
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from . import config

log = logging.getLogger(__name__)


def boot_id() -> str | None:
    """Kennung des aktuellen Pi-Starts. Ändert sich bei jedem Neustart."""
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip() or None
    except OSError:
        return None


def journal_dauerhaft() -> bool:
    """Ob das Journal Neustarts überlebt (sonst ist das Kernel-Log davor weg).

    Es reicht nicht, dass /var/log/journal existiert: Am Pi lag dort nur ein
    Ordner einer alten machine-id, das aktuelle Journal lief flüchtig im RAM.
    """
    try:
        mid = Path("/etc/machine-id").read_text().strip()
    except OSError:
        return False
    return bool(mid) and (Path("/var/log/journal") / mid).is_dir()


def letzte_zeilen(datei: Path, n: int) -> list[str]:
    try:
        with open(datei, encoding="utf-8", errors="replace") as f:
            return [z.rstrip("\n") for z in f.readlines()[-n:]]
    except OSError:
        return []


def journal(*argumente: str, n: int = 40) -> list[str]:
    """Kernel-Log über journalctl. Leere Liste, wenn es nicht geht."""
    if shutil.which("journalctl") is None:
        return []
    try:
        out = subprocess.run(
            ["journalctl", "-k", "--no-pager", "-o", "short-iso", "-n", str(n), *argumente],
            capture_output=True, text=True, timeout=5,
        )
    except (subprocess.SubprocessError, OSError):
        return []
    return [z for z in out.stdout.splitlines() if z and not z.startswith("-- ")]


def speicher() -> dict[str, Any]:
    """Speicher, Threads und offene Dateien des Servers (nur Linux)."""
    werte: dict[str, Any] = {}
    try:
        for zeile in Path("/proc/self/status").read_text().splitlines():
            if zeile.startswith("VmRSS:"):
                werte["server_mb"] = round(int(zeile.split()[1]) / 1024)
            elif zeile.startswith("Threads:"):
                werte["threads"] = int(zeile.split()[1])
        for zeile in Path("/proc/meminfo").read_text().splitlines():
            if zeile.startswith("MemAvailable:"):
                werte["frei_mb"] = round(int(zeile.split()[1]) / 1024)
            elif zeile.startswith("MemTotal:"):
                werte["gesamt_mb"] = round(int(zeile.split()[1]) / 1024)
            elif zeile.startswith("SwapFree:"):
                werte["swap_frei_mb"] = round(int(zeile.split()[1]) / 1024)
        werte["dateien"] = len(os.listdir("/proc/self/fd"))
    except (OSError, ValueError, IndexError):
        pass
    return werte


def logdatei_einrichten(datei: Path) -> None:
    """Hängt eine rotierende Logdatei an den Wurzel-Logger -- nur einmal.

    Am Wurzel-Logger, damit auch uvicorn und Bibliotheken drinstehen.
    """
    wurzel = logging.getLogger()
    if any(getattr(h, "_rosmaster_datei", False) for h in wurzel.handlers):
        return
    h = logging.handlers.RotatingFileHandler(
        datei, maxBytes=config.DIAGNOSE_LOGDATEI_BYTES, backupCount=2, encoding="utf-8",
    )
    h._rosmaster_datei = True  # type: ignore[attr-defined]
    h.setLevel(logging.INFO)
    h.setFormatter(logging.Formatter(
        "%(asctime)s  %(levelname)-7s %(name)-28s %(message)s", "%Y-%m-%d %H:%M:%S",
    ))
    wurzel.addHandler(h)
    # Ohne basicConfig (z. B. in Tests) stünde der Wurzel-Logger auf WARNING.
    if wurzel.level > logging.INFO:
        wurzel.setLevel(logging.INFO)


class Laufprotokoll:
    """Die letzten Serverläufe in einer kleinen JSON-Datei.

    Jeder Eintrag: pid, boot, start, zuletzt (letztes Lebenszeichen), ende
    (Grund) -- ``ende`` bleibt leer, wenn der Prozess nicht sauber endete.
    """

    def __init__(self, datei: Path) -> None:
        self.datei = datei
        self._laeufe: list[dict] = []
        self._meiner: dict | None = None

    def _laden(self) -> list[dict]:
        try:
            daten = json.loads(self.datei.read_text(encoding="utf-8"))
            return [d for d in daten if isinstance(d, dict)] if isinstance(daten, list) else []
        except (OSError, ValueError):
            return []

    def _schreiben(self) -> None:
        # Erst in eine Nebendatei, dann umbenennen: Fällt mitten im Schreiben
        # der Strom aus, bleibt die alte Datei heil.
        tmp = self.datei.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(self._laeufe, indent=1), encoding="utf-8")
            os.replace(tmp, self.datei)
        except OSError:
            log.debug("Laufprotokoll nicht schreibbar", exc_info=True)

    def beginnen(self) -> dict | None:
        """Trägt diesen Lauf ein. Gibt den vorigen Lauf zurück (oder None)."""
        self._laeufe = self._laden()
        vorher = self._laeufe[-1] if self._laeufe else None
        jetzt = time.time()
        self._meiner = {
            "pid": os.getpid(), "boot": boot_id(), "start": jetzt,
            "zuletzt": jetzt, "ende": None,
        }
        self._laeufe = (self._laeufe + [self._meiner])[-config.DIAGNOSE_LAEUFE:]
        self._schreiben()
        return dict(vorher) if vorher else None

    def lebenszeichen(self) -> None:
        if self._meiner is not None and self._meiner["ende"] is None:
            self._meiner["zuletzt"] = time.time()
            self._schreiben()

    def beenden(self, grund: str) -> None:
        # Der erste Grund zählt: Nach SIGHUP folgt das normale Herunterfahren,
        # das soll "SIGHUP" nicht mit "beendet" überschreiben.
        if self._meiner is not None and self._meiner["ende"] is None:
            self._meiner["ende"] = grund
            self._meiner["zuletzt"] = time.time()
            self._schreiben()

    def liste(self) -> list[dict]:
        return [dict(d) for d in self._laeufe][::-1]


def einordnen(vorher: dict, boot_jetzt: str | None, kernel: list[str],
              segfault: bool) -> dict[str, Any]:
    """Macht aus dem vorigen, unsauber beendeten Lauf eine Erklärung."""
    zeit = time.strftime("%d.%m. %H:%M:%S", time.localtime(vorher.get("zuletzt") or 0))
    text_kernel = "\n".join(kernel).lower()
    if vorher.get("boot") and boot_jetzt and vorher["boot"] != boot_jetzt:
        return {
            "art": "pi_neustart",
            "titel": "Der ganze Pi war aus oder ist neu gestartet",
            "was": f"Der letzte Serverlauf hörte um {zeit} mitten im Betrieb auf, und "
                   "seitdem ist der Pi neu gestartet. Der Server selbst ist also nicht "
                   "abgestürzt -- der Pi hatte keinen Strom, wurde zurückgesetzt oder "
                   "der Kernel hing. Das ist ein Hardware- bzw. Stromproblem, kein "
                   "Fehler im Python-Code.",
            "tun": [
                "Wurde er absichtlich aus- oder abgesteckt? Dann ist alles gut.",
                "Netzteil: Reicht der Strom (Strombegrenzung/CC-Anzeige am Labornetzteil)? "
                "Beim Anlaufen von Motoren, Lidar und Hailo gibt es Spitzen.",
                "Kabel und Stecker der 5-V-Versorgung zum Pi prüfen (Wackelkontakt).",
                "Kernel-Log vom Lauf davor unten ansehen (nur bei dauerhaftem Journal).",
                "`journalctl --list-boots` zeigt die letzten Starts (nur bei dauerhaftem Journal).",
            ],
        }
    if segfault or "segfault" in text_kernel:
        return {
            "art": "segfault",
            "titel": "Der Server ist hart abgestürzt (Speicherzugriffsfehler)",
            "was": f"Der Server endete um {zeit} durch einen Segfault -- ein Fehler in "
                   "einer C-Bibliothek (Hailo, OpenCV, Kamera-/Lidar-Treiber), nicht im "
                   "Python-Code. Der Stack-Dump unten zeigt, in welchem Teil es passierte.",
            "tun": ["Stack-Dump unten ansehen: Welche Datei steht ganz oben?",
                    "Bericht kopieren und ans Team geben."],
        }
    if "out of memory" in text_kernel or "killed process" in text_kernel or "oom" in text_kernel:
        return {
            "art": "speicher",
            "titel": "Der Kernel hat den Server wegen Speichermangel beendet",
            "was": f"Um {zeit} war der Arbeitsspeicher voll, und der Kernel hat den "
                   "größten Prozess beendet -- den Server.",
            "tun": ["Speicherverlauf unter „Abstürze & Hänger“ beobachten: wächst der Server?",
                    "Andere große Programme auf dem Pi beenden."],
        }
    return {
        "art": "prozess_weg",
        "titel": "Der Server wurde ohne Herunterfahren beendet",
        "was": f"Der letzte Serverlauf hörte um {zeit} auf, der Pi lief dabei weiter. "
               "Kein Python-Fehler (der stünde im Log), kein Segfault, kein "
               "Speichermangel im Kernel-Log. Bleibt: hart beendet (`kill -9`, "
               "`pkill`, ein zweiter Start), oder das Terminal/SSH ging zu, als "
               "es diese Erfassung noch nicht gab.",
        "tun": ["Server in `tmux` starten, dann überlebt er ein geschlossenes SSH-Fenster.",
                "Letzte Log-Zeilen unten ansehen: Was passierte direkt davor?"],
    }
