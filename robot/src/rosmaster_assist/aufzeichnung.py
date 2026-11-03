"""Fahrtenschreiber -- zeichnet echte Fahrten auf, damit man sie nachspielen kann.

Am Roboter lässt sich schlecht nachvollziehen, warum er an einer Tür hängen
blieb oder einen Stuhl gestreift hat: Bis man nachsieht, ist die Lage
vorbei, und nachstellen lässt sie sich selten genau. Deshalb hält dieser
Schreiber laufend die letzten ``AUFZEICHNUNG_PUFFER_S`` Sekunden im
Speicher:

- jede Lidar-Runde (Winkel und Entfernungen, gerundet),
- jeden Fahrbefehl am Watchdog: verlangt (``soll``) und nach Tempogrenze
  und Bremsweg-Prüfung (``safe``), mit Quelle,
- jeden Not-Halt (``stop()``),
- im Odometrie-Takt die Board-Geschwindigkeit, Pose und den gefahrenen
  Befehl (nach Rampe),
- Zustandswechsel von Erkunder, Interaktion und Heimfahrt.

Erst ein Druck auf "Fahrt speichern" in der App schreibt den Puffer auf die
Karte (gzip, eine JSON-Zeile je Ereignis). Dauernd mitschreiben würde die
SD-Karte belasten, und interessant sind ohnehin nur die Sekunden vor dem
Moment, in dem man denkt: "Das war falsch."

Ausgewertet wird am eigenen Rechner mit ``scripts/abspielen.py``: Es
zeigt, was passiert ist, zeichnet die Fahrt als SVG und kann die
aufgezeichneten Scans einem neuen Erkunder vorlegen -- so lässt sich eine
geänderte Ausweichlogik an echten Fahrten prüfen, ohne den Roboter zu
bewegen.

Der Schreiber darf das Fahren nie stören: Der Watchdog ruft ihn in einem
``try`` auf, und alle Methoden sind kurz und threadsicher.
"""

from __future__ import annotations

import gzip
import io
import json
import logging
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable

from . import config
from .sensors.lidar import Scan

log = logging.getLogger(__name__)

#: Dateiformat. Bei inkompatiblen Änderungen erhöhen, damit das Abspielen
#: alte Dateien erkennt statt sie falsch zu deuten.
VERSION = 1

#: Nur so benannte Dateien werden ausgeliefert -- der Name kommt aus der
#: URL, und ".." oder ein Pfad darf nicht an andere Dateien heranführen.
_DATEINAME = re.compile(r"^fahrt_\d{8}_\d{6}(_\d{1,3})?\.jsonl\.gz$")

#: Diese Einstellungen landen im Dateikopf: Wer eine Fahrt später
#: nachspielt, muss wissen, mit welchen Grenzen der Roboter unterwegs war.
_KOPF_EINSTELLUNGEN = (
    "AUTONOM_", "EXPLORE_", "LIDAR_", "RAMPE_", "ODOMETRY_", "OBSTACLE_",
    "FOLGEN_", "MAX_SPEED_",
)


def gueltiger_name(name: str) -> bool:
    """Ob ``name`` eine Aufzeichnungsdatei bezeichnet (und nichts anderes)."""
    return bool(_DATEINAME.match(name))


def _r(wert: float, stellen: int = 4) -> float:
    return round(float(wert), stellen)


def _befehl_liste(cmd) -> list[float]:
    return [_r(cmd.vx), _r(cmd.vy), _r(cmd.omega)]


def scan_runden(scan: Scan) -> tuple[list[int], list[int]]:
    """Winkel in Zehntelgrad, Entfernungen in Millimetern.

    Genauer misst der RPLIDAR A1 ohnehin nicht, und ganze Zahlen brauchen
    im Speicher und in der Datei nur einen Bruchteil des Platzes. Tests,
    die Aufnahme und Wiedergabe vergleichen, runden ihre Scans genauso.
    """
    return (
        [int(round(w * 10.0)) for w in scan.winkel],
        [int(round(r * 1000.0)) for r in scan.entfernungen],
    )


def scan_aus(ereignis: dict) -> Scan:
    """Aus einem Scan-Ereignis wieder einen ``Scan`` machen."""
    scan = Scan([w / 10.0 for w in ereignis["w"]], [r / 1000.0 for r in ereignis["r"]])
    scan.zeit = ereignis["zeit"]
    scan.dauer = ereignis.get("dauer", Scan.DAUER_STANDARD_S)
    return scan


class Fahrtenschreiber:
    """Ringpuffer der letzten Sekunden einer Fahrt, auf Wunsch gespeichert."""

    def __init__(
        self,
        puffer_s: float | None = None,
        ordner: Path | None = None,
        max_dateien: int | None = None,
        uhr: Callable[[], float] = time.monotonic,
    ) -> None:
        self._puffer_s = config.AUFZEICHNUNG_PUFFER_S if puffer_s is None else puffer_s
        self._ordner = Path(config.AUFZEICHNUNG_ORDNER if ordner is None else ordner)
        self._max_dateien = (
            config.AUFZEICHNUNG_MAX_DATEIEN if max_dateien is None else max_dateien
        )
        self._uhr = uhr
        self._lock = threading.Lock()
        self._puffer: deque[dict] = deque()
        self._letzter_scan: float | None = None
        self._letzter_zustand: dict | None = None
        self._gespeichert = 0
        self._letzte_datei: str | None = None
        #: Steht im Dateikopf (Antrieb, Simulation, ...); setzt der Server.
        self.kopf: dict[str, Any] = {}

    # --- Aufnehmen ----------------------------------------------------------

    def _anhaengen(self, ereignis: dict) -> None:
        with self._lock:
            self._puffer.append(ereignis)
            # Nach Alter kürzen, nicht nach Anzahl: Scans, Befehle und
            # Odometrie kommen in verschiedenen Takten, und gefragt ist
            # "die letzte Minute", nicht "die letzten 3000 Zeilen".
            grenze = ereignis["t"] - self._puffer_s
            while self._puffer and self._puffer[0]["t"] < grenze:
                self._puffer.popleft()

    def befehl(self, quelle: str, soll, safe, jetzt: float | None = None) -> None:
        """Fahrbefehl am Watchdog: verlangt und nach den Grenzen."""
        self._anhaengen({
            "t": self._uhr() if jetzt is None else jetzt,
            "art": "befehl",
            "quelle": quelle,
            "soll": _befehl_liste(soll),
            "safe": _befehl_liste(safe),
        })

    def stop(self, jetzt: float | None = None) -> None:
        """Harter Halt am Watchdog (ohne Rampe)."""
        self._anhaengen({"t": self._uhr() if jetzt is None else jetzt, "art": "stop"})

    def scan(self, scan: Scan | None, jetzt: float | None = None) -> bool:
        """Lidar-Runde aufnehmen, falls neu. True, wenn aufgenommen.

        Die Schleife fragt öfter, als der Lidar liefert (20 Hz gegen
        7,5 Hz); dieselbe Runde zweimal zu speichern, verfälschte beim
        Abspielen die Scan-Rate.
        """
        if scan is None:
            return False
        with self._lock:
            if self._letzter_scan == scan.zeit:
                return False
            self._letzter_scan = scan.zeit
        w, r = scan_runden(scan)
        self._anhaengen({
            "t": self._uhr() if jetzt is None else jetzt,
            "art": "scan",
            "zeit": scan.zeit,
            "dauer": _r(scan.dauer),
            "w": w,
            "r": r,
        })
        return True

    def odo(
        self,
        v: Iterable[float],
        pose: Iterable[float],
        ist,
        misst: bool,
        jetzt: float | None = None,
    ) -> None:
        """Eigenbewegung: Board-Geschwindigkeit, Pose, gefahrener Befehl."""
        self._anhaengen({
            "t": self._uhr() if jetzt is None else jetzt,
            "art": "odo",
            "v": [_r(x) for x in v],
            "pose": [_r(x) for x in pose],
            "ist": _befehl_liste(ist),
            "misst": bool(misst),
        })

    def zustand(self, jetzt: float | None = None, **felder: Any) -> bool:
        """Zustände der Module; aufgenommen wird nur eine Änderung."""
        with self._lock:
            if felder == self._letzter_zustand:
                return False
            self._letzter_zustand = dict(felder)
        self._anhaengen({"t": self._uhr() if jetzt is None else jetzt, "art": "zustand", **felder})
        return True

    # --- Abfragen -----------------------------------------------------------

    def ereignisse(self) -> list[dict]:
        """Kopie des Puffers (älteste zuerst)."""
        with self._lock:
            return list(self._puffer)

    def status(self) -> dict:
        with self._lock:
            anzahl = len(self._puffer)
            scans = sum(1 for e in self._puffer if e["art"] == "scan")
            dauer = self._puffer[-1]["t"] - self._puffer[0]["t"] if anzahl > 1 else 0.0
        return {
            "aktiv": True,
            "puffer_s": self._puffer_s,
            "ereignisse": anzahl,
            "scans": scans,
            "dauer_s": round(dauer, 1),
            "ordner": str(self._ordner),
            "gespeichert": self._gespeichert,
            "letzte_datei": self._letzte_datei,
        }

    # --- Speichern ----------------------------------------------------------

    def speichern(self, notiz: str = "") -> Path:
        """Puffer als ``fahrt_<Datum>_<Zeit>.jsonl.gz`` schreiben.

        Läuft im Server per ``asyncio.to_thread``: gzip und Schreiben auf
        die SD-Karte dauern einige hundert Millisekunden, die Fahrschleife
        darf so lange nicht warten.
        """
        ereignisse = self.ereignisse()
        t0 = ereignisse[0]["t"] if ereignisse else self._uhr()
        jetzt = datetime.now()
        kopf = {
            "art": "kopf",
            "version": VERSION,
            "erstellt": jetzt.isoformat(timespec="seconds"),
            "notiz": str(notiz)[:200],
            "dauer_s": round(ereignisse[-1]["t"] - t0, 2) if ereignisse else 0.0,
            **self.kopf,
            "config": _einstellungen(),
        }
        self._ordner.mkdir(parents=True, exist_ok=True)
        pfad = self._freier_name(jetzt)
        # Zeiten relativ zum ersten Ereignis: Die monotone Uhr des Pi zählt
        # ab dem Einschalten und sagt in der Datei nichts aus.
        puffer = io.StringIO()
        puffer.write(json.dumps(kopf, ensure_ascii=False) + "\n")
        for e in ereignisse:
            e = dict(e, t=round(e["t"] - t0, 4))
            if "zeit" in e:
                e["zeit"] = round(e["zeit"] - t0, 4)
            puffer.write(json.dumps(e, ensure_ascii=False, separators=(",", ":")) + "\n")
        # Erst unter anderem Namen schreiben, dann umbenennen: Fällt der
        # Strom beim Schreiben aus, bleibt keine halbe Datei mit gültigem
        # Namen liegen.
        tmp = pfad.with_name(pfad.name + ".tmp")
        with gzip.open(tmp, "wt", encoding="utf-8") as f:
            f.write(puffer.getvalue())
        tmp.replace(pfad)
        self._aufraeumen()
        with self._lock:
            self._gespeichert += 1
            self._letzte_datei = pfad.name
        log.info("Fahrt gespeichert: %s (%d Ereignisse)", pfad.name, len(ereignisse))
        return pfad

    def _freier_name(self, jetzt: datetime) -> Path:
        basis = f"fahrt_{jetzt:%Y%m%d_%H%M%S}"
        pfad = self._ordner / f"{basis}.jsonl.gz"
        n = 1
        while pfad.exists():  # zweimal in derselben Sekunde gespeichert
            pfad = self._ordner / f"{basis}_{n}.jsonl.gz"
            n += 1
        return pfad

    def _dateien(self) -> list[Path]:
        if not self._ordner.is_dir():
            return []
        return sorted(
            (p for p in self._ordner.iterdir() if gueltiger_name(p.name)),
            key=lambda p: p.stat().st_mtime,
        )

    def _aufraeumen(self) -> None:
        """Nur die neuesten ``max_dateien`` behalten -- die SD-Karte ist klein."""
        dateien = self._dateien()
        for alt in dateien[: max(0, len(dateien) - self._max_dateien)]:
            try:
                alt.unlink()
            except OSError:
                log.warning("Alte Aufzeichnung %s nicht löschbar", alt.name)

    def liste(self) -> list[dict]:
        """Gespeicherte Fahrten, neueste zuerst."""
        return [
            {"name": p.name, "bytes": p.stat().st_size,
             "zeit": datetime.fromtimestamp(p.stat().st_mtime).isoformat(timespec="seconds")}
            for p in reversed(self._dateien())
        ]

    def datei(self, name: str) -> Path | None:
        """Pfad einer gespeicherten Fahrt, None bei ungültigem Namen."""
        if not gueltiger_name(name):
            return None
        pfad = self._ordner / name
        return pfad if pfad.is_file() else None


def _einstellungen() -> dict[str, Any]:
    werte = {}
    for name in dir(config):
        if name.startswith(_KOPF_EINSTELLUNGEN):
            wert = getattr(config, name)
            if isinstance(wert, (bool, int, float, str)):
                werte[name] = wert
    return werte


# --- Lesen und Abspielen -------------------------------------------------------


def laden(pfad: str | Path) -> tuple[dict, list[dict]]:
    """Datei lesen: (Kopf, Ereignisse)."""
    kopf: dict = {}
    ereignisse: list[dict] = []
    with gzip.open(pfad, "rt", encoding="utf-8") as f:
        for zeile in f:
            zeile = zeile.strip()
            if not zeile:
                continue
            e = json.loads(zeile)
            if e.get("art") == "kopf":
                kopf = e
            else:
                ereignisse.append(e)
    if kopf.get("version", VERSION) > VERSION:
        raise ValueError(f"Aufzeichnung hat Version {kopf['version']}, unterstützt bis {VERSION}")
    return kopf, ereignisse


class Wiedergabe:
    """Spielt aufgezeichnete Sensordaten zeitlich passend wieder ab.

    Stellt dieselben Methoden bereit, die Erkunder und Co. von Lidar und
    Odometrie erwarten (``latest()``, ``geschwindigkeit()``,
    ``misst_tatsaechlich``). Mit ``bis(t)`` stellt man die Uhr: Es gilt
    dann die letzte Lidar-Runde, die bis ``t`` fertig war, und der letzte
    Odometrie-Wert bis ``t``. So sieht der nachgespielte Erkunder genau
    das, was der echte zu diesem Zeitpunkt sehen konnte.
    """

    def __init__(self, ereignisse: list[dict]) -> None:
        self._scans = [e for e in ereignisse if e["art"] == "scan"]
        self._scans.sort(key=lambda e: e["zeit"])
        self._odo = [e for e in ereignisse if e["art"] == "odo"]
        self._i_scan = -1
        self._i_odo = -1
        self._scan: Scan | None = None
        self._t = float("-inf")

    def bis(self, t: float) -> None:
        """Uhr vorstellen (nur vorwärts)."""
        if t < self._t:
            raise ValueError("Wiedergabe läuft nur vorwärts")
        self._t = t
        alt = self._i_scan
        while self._i_scan + 1 < len(self._scans) and self._scans[self._i_scan + 1]["zeit"] <= t:
            self._i_scan += 1
        if self._i_scan != alt:
            self._scan = scan_aus(self._scans[self._i_scan])
        while self._i_odo + 1 < len(self._odo) and self._odo[self._i_odo + 1]["t"] <= t:
            self._i_odo += 1

    # wie LidarService
    def latest(self) -> Scan | None:
        return self._scan

    # wie Odometry
    @property
    def misst_tatsaechlich(self) -> bool:
        return self._i_odo >= 0 and self._odo[self._i_odo]["misst"]

    def geschwindigkeit(self) -> tuple[float, float, float]:
        if self._i_odo < 0:
            return 0.0, 0.0, 0.0
        vx, vy, w = self._odo[self._i_odo]["v"]
        return vx, vy, w

    def pose(self) -> tuple[float, float, float]:
        if self._i_odo < 0:
            return 0.0, 0.0, 0.0
        x, y, grad = self._odo[self._i_odo]["pose"]
        return x, y, grad


def abschnitte(ereignisse: list[dict], feld: str = "erkunder") -> list[tuple[float, float]]:
    """Zeiträume (von, bis), in denen ``feld`` im Zustand gesetzt war.

    Für ``"erkunder"`` also: wann der Erkunder lief. Ein Abschnitt, der
    am Ende der Aufzeichnung noch offen ist, endet mit dem letzten Ereignis.
    """
    ergebnis = []
    von = None
    for e in ereignisse:
        if e["art"] != "zustand":
            continue
        an = e.get(feld) is not None
        if an and von is None:
            von = e["t"]
        elif not an and von is not None:
            ergebnis.append((von, e["t"]))
            von = None
    if von is not None and ereignisse:
        ergebnis.append((von, ereignisse[-1]["t"]))
    return ergebnis


def erkunder_nachspielen(
    ereignisse: list[dict],
    bauen: Callable[[Wiedergabe], Any],
) -> list[dict]:
    """Aufgezeichnete Erkunder-Fahrten einem (neuen) Erkunder vorlegen.

    ``bauen(wiedergabe)`` liefert einen Erkunder, der ``wiedergabe`` als
    Lidar und Odometrie benutzt. Für jeden Abschnitt, in dem der Erkunder
    lief, beginnt eine frische Fahrt; zu jedem aufgezeichneten autonomen
    Fahrbefehl rechnet er einen Takt mit derselben Uhrzeit. Ergebnis je
    Takt: ``{"t", "aufgezeichnet", "nachgespielt"}`` (je ``[vx, vy, omega]``,
    ``nachgespielt`` None, wenn er aufgibt).

    Ruft bewusst ``_beginnen``/``_schritt`` auf (wie die Tests): So läuft
    keine Fahrschleife, und die Uhr bestimmt allein die Aufzeichnung.
    Gleiche Befehle sind nur zu erwarten, wenn der Erkunder unverändert ist
    und ``random`` gleich gesetzt wurde -- am Roboter wählt er Richtungen
    und Wanderzeiten zufällig.
    """
    ergebnis = []
    for von, bis_t in abschnitte(ereignisse, "erkunder"):
        wiedergabe = Wiedergabe(ereignisse)
        erkunder = bauen(wiedergabe)
        erkunder._beginnen(von)
        for e in ereignisse:
            if e["art"] != "befehl" or e.get("quelle") != "autonom":
                continue
            if not von <= e["t"] <= bis_t:
                continue
            wiedergabe.bis(e["t"])
            befehl = erkunder._schritt(e["t"])
            ergebnis.append({
                "t": e["t"],
                "aufgezeichnet": e["soll"],
                "nachgespielt": None if befehl is None else _befehl_liste(befehl),
            })
            if befehl is None:
                break
    return ergebnis
