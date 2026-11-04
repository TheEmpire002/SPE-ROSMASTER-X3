"""Die Karte als Python-Objekt: Belegung, Abstand zur nächsten Wand, Laden.

slam_toolbox liefert die Karte als ROS-OccupancyGrid (über
``ros_bridge.py``), gespeichert wird sie als PGM + YAML
(``map_service.py``). Ortung und Pfadplanung brauchen sie aber als
Zahlenfeld, und zwar oft: Die Ortung fragt mehrmals pro Sekunde "wie weit
ist dieser Punkt von der nächsten Wand?". Dafür wird das Abstandsfeld
einmal je Karte vorausgerechnet.

Koordinaten wie in ROS: Zelle ``[iy, ix]`` liegt bei
``ursprung + (ix + 0.5, iy + 0.5) * aufloesung`` im Kartenrahmen. Der
Kartenrahmen ist der von slam_toolbox (``map``); er beginnt dort, wo die
Kartierung begann, x nach vorn, y nach links (wie die Lidar-Odometrie beim
Start).
"""

from __future__ import annotations

import base64
import json
import math
import re
import time
from pathlib import Path

import numpy as np

from .. import config

#: Belegungswerte wie in nav_msgs/OccupancyGrid.
UNBEKANNT = -1
#: Ab hier gilt eine Zelle als Wand, bis hier als frei (Schwellen wie
#: map_server: occupied_thresh 0.65, free_thresh 0.196).
BELEGT_AB = 65
FREI_BIS = 19


class Karte:
    """Unveränderliche Momentaufnahme einer Belegungskarte."""

    def __init__(
        self,
        belegung: np.ndarray,
        aufloesung: float,
        ursprung: tuple[float, float],
        name: str | None = None,
        zeit: float | None = None,
    ) -> None:
        self.belegung = np.asarray(belegung, dtype=np.int8)
        self.aufloesung = float(aufloesung)
        self.ursprung = (float(ursprung[0]), float(ursprung[1]))
        self.name = name
        self.zeit = time.time() if zeit is None else zeit
        self.hoehe, self.breite = self.belegung.shape
        b = self.belegung.astype(np.int16)
        self.belegt = b >= BELEGT_AB
        self.frei = (b >= 0) & (b <= FREI_BIS)
        self.unbekannt = b < 0
        self._abstand: np.ndarray | None = None
        self._gradient: tuple[np.ndarray, np.ndarray] | None = None

    # --- Erzeugen ------------------------------------------------------------------

    @classmethod
    def aus_payload(cls, payload: dict) -> "Karte":
        """Aus der Nachricht der ROS-Brücke (oder der Simulation)."""
        info = payload["info"]
        breite, hoehe = int(info["width"]), int(info["height"])
        roh = base64.b64decode(payload["data_b64"])
        if len(roh) != breite * hoehe:
            raise ValueError(f"Karte unvollständig: {len(roh)} statt {breite * hoehe} Zellen")
        belegung = np.frombuffer(roh, dtype=np.int8).reshape(hoehe, breite)
        pos = info["origin"]["position"]
        return cls(
            belegung, float(info["resolution"]), (float(pos["x"]), float(pos["y"])),
            zeit=payload.get("received_at"),
        )

    @classmethod
    def laden(cls, yaml_pfad: str | Path) -> "Karte":
        """Eine mit ``MapService.save_map`` gespeicherte Karte (PGM + YAML).

        Bewusst ohne YAML-Paket: Die Datei hat nur die wenigen Zeilen, die
        ``map_service.py`` selbst schreibt.
        """
        yaml_pfad = Path(yaml_pfad)
        werte: dict[str, str] = {}
        for zeile in yaml_pfad.read_text(encoding="utf-8").splitlines():
            if ":" in zeile:
                k, v = zeile.split(":", 1)
                werte[k.strip()] = v.strip()
        aufloesung = float(werte["resolution"])
        ursprung = [float(z) for z in re.findall(r"[-+0-9.eE]+", werte["origin"])]
        bild = yaml_pfad.parent / werte.get("image", yaml_pfad.with_suffix(".pgm").name)
        grau = _pgm_lesen(bild)
        # PGM beginnt oben, die Karte unten (ROS); Grauwerte wie map_server
        # im Modus "trinary": dunkel = Wand, hell = frei, dazwischen unbekannt.
        grau = grau[::-1]
        belegung = np.full(grau.shape, UNBEKANNT, dtype=np.int8)
        anteil = (255.0 - grau) / 255.0
        belegung[anteil >= 0.65] = 100
        belegung[anteil <= 0.196] = 0
        return cls(belegung, aufloesung, (ursprung[0], ursprung[1]), name=yaml_pfad.stem,
                   zeit=yaml_pfad.stat().st_mtime)

    def als_payload(self) -> dict:
        """Im Format der ROS-Brücke -- für die Seite /map."""
        return {
            "topic": "/map",
            "frame_id": "map",
            "info": {
                "resolution": self.aufloesung,
                "width": self.breite,
                "height": self.hoehe,
                "origin": {
                    "position": {"x": self.ursprung[0], "y": self.ursprung[1], "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data_b64": base64.b64encode(self.belegung.tobytes()).decode("ascii"),
            "received_at": self.zeit,
            "name": self.name,
        }

    # --- Umrechnen -----------------------------------------------------------------

    def zelle(self, x, y):
        """Weltkoordinaten (m) -> Zellindizes (ix, iy), auch für Arrays."""
        ix = np.floor((np.asarray(x) - self.ursprung[0]) / self.aufloesung).astype(int)
        iy = np.floor((np.asarray(y) - self.ursprung[1]) / self.aufloesung).astype(int)
        return ix, iy

    def welt(self, ix, iy):
        """Zellmitte (m) zu Zellindizes."""
        return (
            self.ursprung[0] + (np.asarray(ix) + 0.5) * self.aufloesung,
            self.ursprung[1] + (np.asarray(iy) + 0.5) * self.aufloesung,
        )

    def innen(self, ix, iy):
        return (ix >= 0) & (iy >= 0) & (ix < self.breite) & (iy < self.hoehe)

    def ist_frei(self, x: float, y: float) -> bool:
        ix, iy = self.zelle(x, y)
        return bool(self.innen(ix, iy) and self.frei[iy, ix])

    # --- Abstand zur nächsten Wand ---------------------------------------------------

    @property
    def abstand(self) -> np.ndarray:
        """Abstand jeder Zelle zur nächsten Wand (m), gekappt bei KARTE_ABSTAND_MAX_M."""
        if self._abstand is None:
            self._abstand = abstandsfeld(self.belegt, self.aufloesung, config.KARTE_ABSTAND_MAX_M)
        return self._abstand

    @property
    def gradient(self) -> tuple[np.ndarray, np.ndarray]:
        """Steigung des Abstandsfelds je Meter (d/dx, d/dy) -- für die Ortung."""
        if self._gradient is None:
            gy, gx = np.gradient(self.abstand, self.aufloesung)
            self._gradient = (gx, gy)
        return self._gradient

    def abstand_bei(self, x, y):
        """Abstand zur Wand an Weltpunkten (bilinear), außerhalb das Maximum."""
        d, _, _ = self.abtasten(np.asarray(x, dtype=float), np.asarray(y, dtype=float),
                                 mit_gradient=False)
        return d

    def abtasten(self, x: np.ndarray, y: np.ndarray, mit_gradient: bool = True):
        """Abstand (und Steigung) an beliebigen Punkten, zwischen Zellmitten gemittelt."""
        fx = (x - self.ursprung[0]) / self.aufloesung - 0.5
        fy = (y - self.ursprung[1]) / self.aufloesung - 0.5
        x0 = np.floor(fx).astype(int)
        y0 = np.floor(fy).astype(int)
        ax, ay = fx - x0, fy - y0
        innen = (x0 >= 0) & (y0 >= 0) & (x0 < self.breite - 1) & (y0 < self.hoehe - 1)
        x0c = np.clip(x0, 0, self.breite - 2)
        y0c = np.clip(y0, 0, self.hoehe - 2)

        def mischen(feld):
            return (
                feld[y0c, x0c] * (1 - ax) * (1 - ay)
                + feld[y0c, x0c + 1] * ax * (1 - ay)
                + feld[y0c + 1, x0c] * (1 - ax) * ay
                + feld[y0c + 1, x0c + 1] * ax * ay
            )

        d = np.where(innen, mischen(self.abstand), config.KARTE_ABSTAND_MAX_M)
        if not mit_gradient:
            return d, None, innen
        gx, gy = self.gradient
        return d, (np.where(innen, mischen(gx), 0.0), np.where(innen, mischen(gy), 0.0)), innen

    def unbekannt_bei(self, x, y):
        """True für Punkte in unbekannten Zellen oder außerhalb der Karte."""
        ix, iy = self.zelle(x, y)
        innen = self.innen(ix, iy)
        ixc = np.clip(ix, 0, self.breite - 1)
        iyc = np.clip(iy, 0, self.hoehe - 1)
        return ~innen | self.unbekannt[iyc, ixc]


def abstandsfeld(belegt: np.ndarray, aufloesung: float, maximum: float) -> np.ndarray:
    """Abstand zur nächsten belegten Zelle (m), gekappt bei ``maximum``.

    Chamfer-Abstand mit 8 Nachbarn, nur so viele Runden, wie es für
    ``maximum`` braucht -- jede Runde ist eine Handvoll ganzer Feld-
    operationen in numpy. Genau auf etwa 8 %, für Ortung und Wegkosten mehr
    als genug, und ohne scipy.
    """
    d = np.where(belegt, 0.0, np.inf)
    runden = int(math.ceil(maximum / aufloesung)) + 1
    diag = aufloesung * math.sqrt(2.0)
    for _ in range(runden):
        neu = d.copy()
        neu[1:, :] = np.minimum(neu[1:, :], d[:-1, :] + aufloesung)
        neu[:-1, :] = np.minimum(neu[:-1, :], d[1:, :] + aufloesung)
        neu[:, 1:] = np.minimum(neu[:, 1:], d[:, :-1] + aufloesung)
        neu[:, :-1] = np.minimum(neu[:, :-1], d[:, 1:] + aufloesung)
        neu[1:, 1:] = np.minimum(neu[1:, 1:], d[:-1, :-1] + diag)
        neu[:-1, :-1] = np.minimum(neu[:-1, :-1], d[1:, 1:] + diag)
        neu[1:, :-1] = np.minimum(neu[1:, :-1], d[:-1, 1:] + diag)
        neu[:-1, 1:] = np.minimum(neu[:-1, 1:], d[1:, :-1] + diag)
        if np.array_equal(neu, d):
            break
        d = neu
    return np.minimum(d, maximum)


def _pgm_lesen(pfad: Path) -> np.ndarray:
    """Binäres PGM (P5, 8 Bit) als Grauwert-Feld, Zeile 0 oben."""
    daten = pfad.read_bytes()
    teile: list[bytes] = []
    pos = 0
    # Kopf: "P5", Breite, Höhe, Maximalwert -- Kommentare (#) überspringen.
    while len(teile) < 4:
        while daten[pos:pos + 1].isspace():
            pos += 1
        if daten[pos:pos + 1] == b"#":
            pos = daten.index(b"\n", pos) + 1
            continue
        ende = pos
        while not daten[ende:ende + 1].isspace():
            ende += 1
        teile.append(daten[pos:ende])
        pos = ende
    if teile[0] != b"P5":
        raise ValueError(f"{pfad.name}: kein binäres PGM")
    breite, hoehe = int(teile[1]), int(teile[2])
    pos += 1  # genau ein Leerzeichen nach dem Maximalwert
    pixel = np.frombuffer(daten, dtype=np.uint8, count=breite * hoehe, offset=pos)
    return pixel.reshape(hoehe, breite).astype(float)


# --- Zusatzdaten zur gespeicherten Karte --------------------------------------------


def zusatz_pfad(yaml_pfad: str | Path) -> Path:
    return Path(yaml_pfad).with_suffix(".json")


def zusatz_lesen(yaml_pfad: str | Path) -> dict:
    """Startpunkt und benannte Orte einer gespeicherten Karte (falls vorhanden)."""
    pfad = zusatz_pfad(yaml_pfad)
    try:
        return json.loads(pfad.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def zusatz_schreiben(yaml_pfad: str | Path, daten: dict) -> None:
    pfad = zusatz_pfad(yaml_pfad)
    tmp = pfad.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(daten, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(pfad)
