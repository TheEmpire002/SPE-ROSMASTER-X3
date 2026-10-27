"""RPLIDAR A1 auslesen.

Liefert Rundum-Abstandsmessungen: pro Umdrehung einige hundert Punkte aus
Winkel und Entfernung. Daraus wird die Ortung gespeist (siehe
``navigation/scan_matching.py``) und die Hinderniskontrolle während der
Heimfahrt.

Wie bei Antrieb und Kamera gibt es einen Simulations-Zwilling: ``MockLidar``
tastet einen gedachten Raum ab und erlaubt damit, Ortung und Heimfahrt ohne
Hardware zu entwickeln und zu prüfen.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from abc import ABC, abstractmethod

from .. import config

log = logging.getLogger(__name__)


class Scan:
    """Eine Umdrehung: Winkel in Grad, Entfernungen in Metern.

    Die Punkte liegen im Roboterkoordinatensystem: 0° ist vorn, der Winkel
    wächst gegen den Uhrzeigersinn -- dieselbe Konvention wie beim Antrieb,
    damit man nicht ständig umrechnen muss.
    """

    __slots__ = ("winkel", "entfernungen", "zeit")

    def __init__(self, winkel: list[float], entfernungen: list[float]) -> None:
        self.winkel = winkel
        self.entfernungen = entfernungen
        self.zeit = time.monotonic()

    def __len__(self) -> int:
        return len(self.winkel)

    def punkte(self):
        """Als Nx2-Array kartesischer Punkte (x vorwärts, y links)."""
        import numpy as np

        a = np.radians(np.asarray(self.winkel, dtype=float))
        r = np.asarray(self.entfernungen, dtype=float)
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    def min_abstand(self, richtung_grad: float, oeffnung_grad: float) -> float:
        """Kleinste Entfernung in einem Kreisausschnitt.

        Für die Hinderniskontrolle: "was ist das Nächste in Fahrtrichtung".
        """
        kleinster = float("inf")
        for w, r in zip(self.winkel, self.entfernungen):
            diff = abs((w - richtung_grad + 180.0) % 360.0 - 180.0)
            if diff <= oeffnung_grad and r < kleinster:
                kleinster = r
        return kleinster


class LidarBase(ABC):
    name = "base"
    label = "Lidar"

    def __init__(self) -> None:
        self._opened = False

    @property
    def opened(self) -> bool:
        return self._opened

    def open(self) -> None:
        if self._opened:
            return
        try:
            self._open()
        except Exception:
            try:
                self._close()
            except Exception:
                log.debug("Aufräumen nach Fehlschlag misslungen", exc_info=True)
            raise
        self._opened = True

    def close(self) -> None:
        if not self._opened:
            return
        try:
            self._close()
        finally:
            # Auch wenn das Schließen scheitert (Gerät schon weg): sonst
            # hielte open() den Lidar für offen und täte beim Neuversuch
            # nichts.
            self._opened = False

    @abstractmethod
    def _open(self) -> None: ...

    @abstractmethod
    def _close(self) -> None: ...

    @abstractmethod
    def read(self) -> Scan | None:
        """Nächste vollständige Umdrehung. Darf blockieren."""


# --- Echte Hardware ---------------------------------------------------------------


class RPLidar(LidarBase):
    name = "rplidar"
    label = "RPLIDAR A1"

    def __init__(self, port: str | None = None) -> None:
        super().__init__()
        self._port = port or config.LIDAR_PORT
        self._lidar = None
        self._scans = None

    def _open(self) -> None:
        # Eigenes Paket, nicht Teil der Standardinstallation.
        from rplidar import RPLidar as Treiber

        self._lidar = Treiber(self._port, baudrate=config.LIDAR_BAUDRATE, timeout=3)

        info = self._lidar.get_info()
        zustand = self._lidar.get_health()
        log.info("RPLIDAR verbunden: %s, Zustand %s", info, zustand[0])
        if zustand[0] == "Error":
            raise RuntimeError(f"Lidar meldet Fehler: {zustand}")

        # iter_scans liefert fertige Umdrehungen statt Einzelpunkte.
        # min_len niedrig halten: bei 5,5 Umdrehungen/s sind ~180 Punkte
        # pro Runde normal, und ein zu hoher Wert lässt es nie anlaufen.
        self._scans = self._lidar.iter_scans(min_len=60)

    def _close(self) -> None:
        if self._lidar is None:
            return
        try:
            self._lidar.stop()
            self._lidar.stop_motor()
            self._lidar.disconnect()
        except Exception:
            log.exception("Lidar ließ sich nicht sauber schließen")
        finally:
            self._lidar = None
            self._scans = None

    def read(self) -> Scan | None:
        if self._scans is None:
            return None

        roh = next(self._scans)  # Liste aus (Güte, Winkel in Grad, Abstand in mm)
        winkel, entfernungen = [], []
        for _guete, grad, mm in roh:
            meter = mm / 1000.0
            if not (config.LIDAR_MIN_M <= meter <= config.LIDAR_MAX_M):
                continue
            # Der Treiber zählt im Uhrzeigersinn, unser Antrieb dagegen.
            # Ohne diese Drehung wäre die ganze Karte gespiegelt.
            winkel.append((-grad) % 360.0)
            entfernungen.append(meter)

        return Scan(winkel, entfernungen) if winkel else None


# --- Simulation ---------------------------------------------------------------------


class MockLidar(LidarBase):
    """Tastet einen gedachten rechteckigen Raum ab.

    Fragt beim Antrieb nach, wo der Roboter *wirklich* steht, und berechnet
    daraus, was ein Lidar sähe. Damit lässt sich die Ortung am Schreibtisch
    prüfen -- einschließlich der Frage, ob sie einen Odometriefehler
    tatsächlich korrigiert.
    """

    name = "mock"
    label = "Simulierter Lidar"

    #: Raum in Metern, Startpunkt des Roboters liegt bei (0, 0).
    RAUM = (-3.0, 5.0, -3.0, 3.0)  # x_min, x_max, y_min, y_max
    #: Ein Möbelstück, damit der Raum nicht symmetrisch ist -- sonst wäre
    #: die Ausrichtung mehrdeutig und der Abgleich könnte jede Drehung finden.
    SAEULE = (2.0, 1.2, 0.35)  # x, y, Radius
    SCHRITT_GRAD = 1.0

    def __init__(self, pose_quelle=None) -> None:
        super().__init__()
        self._pose_quelle = pose_quelle

    def _open(self) -> None:
        log.info("Lidar: Simulation (keine echte Hardware)")

    def _close(self) -> None:
        pass

    def read(self) -> Scan | None:
        if not self.opened:
            return None
        time.sleep(0.18)  # ~5,5 Umdrehungen/s wie beim echten A1

        x, y, theta = (0.0, 0.0, 0.0)
        if self._pose_quelle is not None:
            x, y, theta = self._pose_quelle()

        winkel, entfernungen = [], []
        schritte = int(360 / self.SCHRITT_GRAD)
        for i in range(schritte):
            a_robot = i * self.SCHRITT_GRAD
            a_welt = math.radians(a_robot) + theta
            r = self._strahl(x, y, a_welt)
            if config.LIDAR_MIN_M <= r <= config.LIDAR_MAX_M:
                winkel.append(a_robot)
                entfernungen.append(r)
        return Scan(winkel, entfernungen)

    def _strahl(self, x: float, y: float, richtung: float) -> float:
        """Entfernung bis zur nächsten Wand oder zur Säule."""
        dx, dy = math.cos(richtung), math.sin(richtung)
        x_min, x_max, y_min, y_max = self.RAUM

        treffer = float("inf")
        for grenze, d, start in ((x_min, dx, x), (x_max, dx, x), (y_min, dy, y), (y_max, dy, y)):
            if abs(d) < 1e-9:
                continue
            t = (grenze - start) / d
            if t <= 0:
                continue
            px, py = x + dx * t, y + dy * t
            # Nur zählen, wenn der Schnittpunkt auf der Wand liegt und nicht
            # auf deren gedachter Verlängerung.
            if x_min - 1e-6 <= px <= x_max + 1e-6 and y_min - 1e-6 <= py <= y_max + 1e-6:
                treffer = min(treffer, t)

        t_saeule = self._saeule_treffen(x, y, dx, dy)
        if t_saeule is not None:
            treffer = min(treffer, t_saeule)
        return treffer

    def _saeule_treffen(self, x, y, dx, dy) -> float | None:
        cx, cy, radius = self.SAEULE
        mx, my = x - cx, y - cy
        b = 2 * (mx * dx + my * dy)
        c = mx * mx + my * my - radius * radius
        diskriminante = b * b - 4 * c
        if diskriminante < 0:
            return None
        wurzel = math.sqrt(diskriminante)
        for t in sorted(((-b - wurzel) / 2, (-b + wurzel) / 2)):
            if t > 0:
                return t
        return None


# --- Auswahl und Hintergrundbetrieb -----------------------------------------------------


def create_lidar(use_mock: bool = False, pose_quelle=None) -> LidarBase:
    """Nimmt den echten Lidar, sonst die Simulation."""
    if not use_mock:
        lidar = RPLidar()
        try:
            lidar.open()
            return lidar
        except Exception as exc:
            log.warning("Lidar: %s nicht nutzbar -- %s", lidar.label, exc)
            try:
                lidar.close()
            except Exception:
                pass
        log.warning("Kein Lidar -- Ortung läuft ohne, Hindernisse bleiben unerkannt")

    ersatz = MockLidar(pose_quelle)
    ersatz.open()
    return ersatz


class LidarService:
    """Liest fortlaufend im Hintergrund und hält die letzte Umdrehung bereit."""

    def __init__(self, lidar: LidarBase) -> None:
        self._lidar = lidar
        self._scan: Scan | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._runden = 0
        self._letzter_fehler: str | None = None
        self._neuverbindungen = 0

    @property
    def label(self) -> str:
        return self._lidar.label

    @property
    def ist_simulation(self) -> bool:
        return self._lidar.name == "mock"

    def latest(self) -> Scan | None:
        with self._lock:
            return self._scan

    def status(self) -> dict:
        with self._lock:
            scan = self._scan
            runden = self._runden
            fehler = self._letzter_fehler
        return {
            "lidar": self._lidar.label,
            "geoeffnet": self._lidar.opened,
            "thread_laeuft": self._thread is not None and self._thread.is_alive(),
            "umdrehungen": runden,
            "punkte_letzte_runde": len(scan) if scan else 0,
            "letzte_runde_vor_s": round(time.monotonic() - scan.zeit, 1) if scan else None,
            "letzter_fehler": fehler,
            "neu_verbunden": self._neuverbindungen,
        }

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            self._lidar.open()
        except Exception as exc:
            log.exception("Lidar lässt sich nicht öffnen")
            self._letzter_fehler = str(exc)
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # Großzügig warten: read() blockiert bis zur nächsten Umdrehung.
            self._thread.join(timeout=3.0)
            self._thread = None
        self._lidar.close()

    def _loop(self) -> None:
        fehler = 0
        while not self._stop.is_set():
            try:
                scan = self._lidar.read()
                if scan is not None:
                    with self._lock:
                        self._scan = scan
                        self._runden += 1
                        self._letzter_fehler = None
                    fehler = 0
            except Exception as exc:
                fehler += 1
                with self._lock:
                    self._letzter_fehler = f"{type(exc).__name__}: {exc}"
                if fehler <= 3:
                    log.exception("Lidar liefert nichts (%d. Versuch)", fehler)
                # Kurz warten, sonst dreht die Schleife bei dauerhaftem
                # Fehler mit voller Last.
                self._stop.wait(0.5)
                # Weiter am selben Gerät zu lesen ist danach zwecklos: der
                # Datenstrom des Treibers ist nach der ersten Ausnahme
                # erschöpft, und nach einem USB-Aussetzer gibt es das
                # geöffnete Gerät gar nicht mehr.
                if fehler >= 3:
                    self._neu_oeffnen()
                    fehler = 0

    def _neu_oeffnen(self) -> None:
        """Schließen und erneut öffnen, bis es klappt oder beendet wird."""
        log.warning("Lidar antwortet nicht mehr -- öffne ihn neu")
        try:
            self._lidar.close()
        except Exception:
            log.debug("Lidar schließen misslungen", exc_info=True)

        wartezeit = config.RECONNECT_MIN_S
        while not self._stop.wait(wartezeit):
            try:
                self._lidar.open()
            except Exception as exc:
                with self._lock:
                    self._letzter_fehler = f"Neu öffnen: {type(exc).__name__}: {exc}"
                log.warning(
                    "Lidar noch nicht erreichbar (%s) -- nächster Versuch in %.0f s",
                    exc,
                    min(wartezeit * 2, config.RECONNECT_MAX_S),
                )
                wartezeit = min(wartezeit * 2, config.RECONNECT_MAX_S)
                continue
            self._neuverbindungen += 1
            log.warning("Lidar wieder verbunden")
            return
