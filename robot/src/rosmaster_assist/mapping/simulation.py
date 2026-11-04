"""Kartierung in der Simulation (--mock): die Karte entsteht in Python.

Am Schreibtisch gibt es weder Docker noch slam_toolbox. Damit die Seite
/map, die Kartierfahrt und alles, was die Karte nutzt (Ortung, Fahrt zum
Ziel), trotzdem laufen, baut diese Klasse aus dem simulierten Lidar und der
Lidar-Odometrie eine einfache Belegungskarte: Jeder Strahl macht die Zellen
bis zum Treffer freier und die Zelle am Treffer belegter (Log-Odds).

Eingetragen wird mit der **wahren** Lage, die nur die Simulation kennt
(``Scan.wahre_pose``), umgerechnet in den Rahmen, in dem die Kartierung
begann. So verhält sich die Karte wie die von slam_toolbox mit
Schleifenschluss: Sie ist richtig, während die Lidar-Odometrie langsam
driftet -- und genau diese Drift muss die Ortung in der Karte dann
ausgleichen. Mit der Lage der Lidar-Odometrie eingetragen, verschmierte die
Karte im ersten Versuch nach 7 Minuten um rund 30° (04.11.).

Bewusst nur für die Simulation (E8). Die Karte kommt im selben Format wie
von der ROS-Brücke, also merkt der Rest nichts vom Unterschied.
"""

from __future__ import annotations

import base64
import logging
import math
import threading
import time

import numpy as np

from .. import config
from ..navigation.kartenortung import umkehren, verketten

log = logging.getLogger(__name__)

#: Größe des Rasters (m) um den Ursprung; die Karte wird beim Senden auf den
#: bekannten Teil zugeschnitten.
_GROESSE_M = 30.0
_FREI = -0.4
_BELEGT = 0.9
_GRENZE = 4.0


class SimKartierung:
    """Belegungskarte aus Scans und Lidar-Odometrie, im eigenen Thread."""

    def __init__(self, scan_odometrie) -> None:
        self._odo = scan_odometrie
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._karte: dict | None = None
        self._scans = 0
        self._started_at: float | None = None
        self._zuruecksetzen()

    def _zuruecksetzen(self) -> None:
        self.aufloesung = config.MAP_SIM_AUFLOESUNG_M
        n = int(_GROESSE_M / self.aufloesung)
        self._n = n
        self._ursprung = -_GROESSE_M / 2.0
        self._lo = np.zeros((n, n), dtype=np.float32)
        self._gesehen = np.zeros((n, n), dtype=bool)
        self._karte = None
        self._scans = 0
        # Wahre Lage -> Kartenrahmen, festgelegt bei der ersten Aufnahme.
        self._wahr_zu_karte: tuple[float, float, float] | None = None

    @property
    def laeuft(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.laeuft:
            return
        with self._lock:
            self._zuruecksetzen()
        self._stop.clear()
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._schleife, name="sim-kartierung", daemon=True)
        self._thread.start()
        log.info("Kartierung in der Simulation gestartet (ohne Docker/slam_toolbox)")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def get_latest_map(self) -> dict | None:
        with self._lock:
            return dict(self._karte) if self._karte is not None else None

    def status(self) -> dict:
        with self._lock:
            return {
                "running": self.laeuft,
                "map_available": self._karte is not None,
                "last_map_at": self._karte["received_at"] if self._karte else None,
                "scans": self._scans,
            }

    def _schleife(self) -> None:
        letzte_nr = None
        takt = 1.0 / max(config.MAP_SIM_HZ, 0.1)
        naechste_karte = 0.0
        while not self._stop.is_set():
            messung = self._odo.messung()
            if messung is not None and messung.nr != letzte_nr:
                letzte_nr = messung.nr
                try:
                    self._eintragen(messung.scan.punkte(), self._lage(messung))
                    if time.monotonic() >= naechste_karte:
                        naechste_karte = time.monotonic() + takt
                        self._veroeffentlichen()
                except Exception:
                    log.exception("Simulierte Kartierung: Aufnahme nicht verarbeitet")
            self._stop.wait(0.05)

    def _lage(self, messung) -> tuple[float, float, float]:
        """Lage für den Eintrag: die wahre, im Rahmen der Kartierung.

        Der Kartenrahmen ist -- wie bei slam_toolbox -- der der Lidar-
        Odometrie zu Beginn. Bei der ersten Aufnahme stimmen beide noch
        überein; daraus ergibt sich die feste Umrechnung von wahr nach Karte.
        Ohne wahre Lage (kein Mock-Lidar) bleibt nur die Odometrie.
        """
        wahr = getattr(messung.scan, "wahre_pose", None)
        if wahr is None:
            return messung.pose
        if self._wahr_zu_karte is None:
            self._wahr_zu_karte = verketten(messung.pose, umkehren(wahr))
        return verketten(self._wahr_zu_karte, wahr)

    def _eintragen(self, punkte, pose) -> None:
        pkt = np.asarray(punkte, dtype=float).reshape(-1, 2)
        if len(pkt) == 0:
            return
        pkt = pkt[:: max(1, len(pkt) // 240)]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = c * pkt[:, 0] - s * pkt[:, 1] + pose[0]
        wy = s * pkt[:, 0] + c * pkt[:, 1] + pose[1]
        r = np.hypot(wx - pose[0], wy - pose[1])
        schritte = int(np.ceil(r.max() / self.aufloesung))
        t = np.arange(schritte) * self.aufloesung
        # Freie Zellen entlang jedes Strahls bis kurz vor dem Treffer.
        anteil = t[None, :] / np.maximum(r[:, None], 1e-6)
        gueltig = t[None, :] < (r[:, None] - self.aufloesung)
        fx = pose[0] + anteil * (wx - pose[0])[:, None]
        fy = pose[1] + anteil * (wy - pose[1])[:, None]
        frei = self._indizes(fx[gueltig], fy[gueltig])
        belegt = self._indizes(wx, wy)
        with self._lock:
            flach = self._lo.ravel()
            gesehen = self._gesehen.ravel()
            frei = np.setdiff1d(frei, belegt)
            flach[frei] += _FREI
            flach[belegt] += _BELEGT
            np.clip(flach, -_GRENZE, _GRENZE, out=flach)
            gesehen[frei] = True
            gesehen[belegt] = True
            self._scans += 1

    def _indizes(self, x, y) -> np.ndarray:
        ix = np.floor((x - self._ursprung) / self.aufloesung).astype(int)
        iy = np.floor((y - self._ursprung) / self.aufloesung).astype(int)
        innen = (ix >= 0) & (iy >= 0) & (ix < self._n) & (iy < self._n)
        return np.unique(iy[innen] * self._n + ix[innen])

    def _veroeffentlichen(self) -> None:
        with self._lock:
            lo = self._lo.copy()
            gesehen = self._gesehen.copy()
        ys, xs = np.nonzero(gesehen)
        if len(xs) == 0:
            return
        rand = 10
        x0, x1 = max(0, xs.min() - rand), min(self._n, xs.max() + rand + 1)
        y0, y1 = max(0, ys.min() - rand), min(self._n, ys.max() + rand + 1)
        lo, gesehen = lo[y0:y1, x0:x1], gesehen[y0:y1, x0:x1]
        belegung = np.full(lo.shape, -1, dtype=np.int8)
        belegung[gesehen & (lo < 0.0)] = 0
        belegung[gesehen & (lo > 0.6)] = 100
        karte = {
            "topic": "/map",
            "frame_id": "map",
            "info": {
                "resolution": self.aufloesung,
                "width": int(belegung.shape[1]),
                "height": int(belegung.shape[0]),
                "origin": {
                    "position": {"x": self._ursprung + x0 * self.aufloesung,
                                 "y": self._ursprung + y0 * self.aufloesung, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "data_b64": base64.b64encode(belegung.tobytes()).decode("ascii"),
            "received_at": time.time(),
        }
        with self._lock:
            self._karte = karte
