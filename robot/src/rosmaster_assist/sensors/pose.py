"""Körperpose im Kamerabild (YOLOv8-Pose auf dem Hailo-10H).

Für Gesten reicht eine Box um die Person nicht -- man muss wissen, wo
Schultern und Handgelenke sind. Das Modell ``yolov8s_pose_h10.hef`` liefert
pro Person 17 Körperpunkte (COCO-Reihenfolge, siehe ``KP_*``).

Gerechnet wird auf dem AI HAT+ 2, nicht auf der CPU: Die CPU des Pi bleibt
frei, und CPU-Lastsprünge lösen am X3 USB-Resets aus (Sensor-Check
28.10.2026). Die Auswertung der Modellausgaben (Boxen, Punkte, NMS) ist
dagegen wenig Arbeit und läuft hier in numpy.

Das Muster ist dasselbe wie bei ``personen.py``: eigener Thread, rechnet nur,
solange jemand ``aktivieren(True)`` gesagt hat, und meldet über ``status()``,
warum es gerade nicht geht. Fällt der Hailo aus, wird er nach einer Pause
neu geöffnet -- der Rest des Servers merkt davon nur, dass kurz niemand
erkannt wird.

Mit ``mock=True`` (``main.py --mock``) entsteht eine simulierte Person, die
nacheinander alle Gesten zeigt. So lässt sich die ganze Kette bis zum
Fahren ohne Roboter ausprobieren.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time

from .. import config
from .camera import CameraService

log = logging.getLogger(__name__)

# COCO-Körperpunkte. "links" ist die linke Seite der Person -- schaut sie in
# die Kamera, liegt sie im Bild rechts.
KP_NASE = 0
KP_SCHULTER_L, KP_SCHULTER_R = 5, 6
KP_ELLBOGEN_L, KP_ELLBOGEN_R = 7, 8
KP_HAND_L, KP_HAND_R = 9, 10
KP_HUEFTE_L, KP_HUEFTE_R = 11, 12

#: Linien des Skeletts, zum Zeichnen in der App (Paare von Punktnummern).
SKELETT = [
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16), (0, 5), (0, 6),
]

#: Ältere Ergebnisse zählen nicht mehr -- die Person kann längst weg sein.
_MAX_ALTER_S = 1.0

#: Pause, bevor ein ausgefallener Hailo neu geöffnet wird.
_NEU_OEFFNEN_S = 2.0

#: Überlappen sich zwei Boxen stärker, ist es dieselbe Person (NMS).
_NMS_IOU = 0.7


class Posenerkennung:
    def __init__(self, camera: CameraService | None, mock: bool = False) -> None:
        self._camera = camera
        self._mock = mock
        self._lock = threading.Lock()
        self._aktiv = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._personen: list[dict] = []
        self._zeit = 0.0
        self._fehler: str | None = None
        self._bilder = 0
        self._fps = 0.0
        self._neu_geoeffnet = 0
        self._np = None
        self._cv2 = None
        self._verfuegbar = False

        if not config.POSE_ENABLED:
            self._fehler = "abgeschaltet (config.POSE_ENABLED)"
            return
        try:
            import numpy as np

            self._np = np
        except ImportError:
            self._fehler = "numpy fehlt"
            return
        if mock:
            self._verfuegbar = True
            return
        if camera is None:
            self._fehler = "keine Kamera"
            return
        try:
            import cv2
            import hailo_platform  # noqa: F401 -- nur prüfen, ob es da ist

            self._cv2 = cv2
        except Exception as exc:  # ImportError oder kaputte Installation
            self._fehler = f"nicht verfügbar ({exc.__class__.__name__}: {exc})"
            log.warning("Posenerkennung %s", self._fehler)
            return
        if not os.path.exists(config.POSE_HEF):
            self._fehler = f"Modell fehlt: {config.POSE_HEF}"
            log.warning("Posenerkennung: %s", self._fehler)
            return
        self._verfuegbar = True

    # --- Bedienung (wie Personenerkennung) ----------------------------------------

    @property
    def verfuegbar(self) -> bool:
        return self._verfuegbar

    @property
    def simuliert(self) -> bool:
        return self._mock

    def start(self) -> None:
        if not self.verfuegbar or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="pose", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._aktiv.set()  # aus dem Warten wecken
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def aktivieren(self, an: bool) -> None:
        """Nur rechnen, solange die Interaktion läuft."""
        if an:
            self._aktiv.set()
        else:
            self._aktiv.clear()
            with self._lock:
                self._personen = []

    @property
    def aktiv(self) -> bool:
        return self._aktiv.is_set() and self.verfuegbar

    def personen(self) -> list[dict]:
        """Aktuell gesehene Personen, die größte (= nächste) zuerst.

        Jede Person: ``box`` [x1, y1, x2, y2] und ``kp`` (17 × [x, y, sicherheit])
        relativ zur Bildgröße (0..1, y nach unten), ``score``, ``winkel``
        (Grad, links positiv wie beim Lidar), ``seitenverhaeltnis`` (Breite/Höhe
        des Bildes, damit Abstände in x und y vergleichbar werden) und ``zeit``.
        """
        with self._lock:
            if time.monotonic() - self._zeit > _MAX_ALTER_S:
                return []
            return list(self._personen)

    def winkelbereiche(self) -> list[tuple[float, float]]:
        """Richtungen (von, bis) in Grad -- dieselbe Form wie bei ``Personenerkennung``."""
        fov = config.CAMERA_FOV_DEG
        return [((0.5 - p["box"][2]) * fov, (0.5 - p["box"][0]) * fov) for p in self.personen()]

    def status(self) -> dict:
        with self._lock:
            return {
                "verfuegbar": self.verfuegbar,
                "simuliert": self._mock,
                "aktiv": self.aktiv,
                "fehler": self._fehler,
                "fps": round(self._fps, 1),
                "bilder": self._bilder,
                "neu_geoeffnet": self._neu_geoeffnet,
                "letzte_erkennung_vor_s": round(time.monotonic() - self._zeit, 1) if self._zeit else None,
            }

    # --- Rechenschleife -------------------------------------------------------------

    def _loop(self) -> None:
        takt = 1.0 / max(0.5, config.POSE_HZ)
        hailo: _Hailo | None = None
        try:
            while not self._stop.is_set():
                if not self._aktiv.wait(timeout=1.0):
                    # Nicht gebraucht: Hailo freigeben, damit andere
                    # Programme (hailo-apps) ihn nutzen können.
                    if hailo is not None:
                        hailo.schliessen()
                        hailo = None
                    continue
                if self._stop.is_set():
                    # stop() weckt uns über _aktiv auf. Ohne diese Prüfung
                    # würde der Thread beim Beenden noch den Hailo öffnen --
                    # am 29.10. fiel genau dabei die USB-Versorgung kurz aus.
                    break
                beginn = time.monotonic()
                try:
                    if self._mock:
                        personen = _simulierte_personen(beginn)
                    else:
                        if hailo is None:
                            hailo = _Hailo(self._np)
                            log.info("Posenerkennung: Hailo bereit (%s)", config.POSE_HEF)
                        personen = self._erkennen(hailo)
                    if personen is not None:
                        self._merken(personen, beginn)
                except Exception as exc:
                    with self._lock:
                        self._fehler = f"{exc.__class__.__name__}: {exc}"
                    if hailo is not None or not self._mock:
                        # Hailo weg (USB/PCIe-Aussetzer, anderes Programm
                        # hält ihn) -- schließen und nach einer Pause neu.
                        log.warning("Posenerkennung: %s -- öffne den Hailo neu", self._fehler)
                        if hailo is not None:
                            hailo.schliessen()
                            hailo = None
                        self._neu_geoeffnet += 1
                        self._stop.wait(_NEU_OEFFNEN_S)
                        continue
                rest = takt - (time.monotonic() - beginn)
                if rest > 0:
                    self._stop.wait(rest)
        finally:
            if hailo is not None:
                hailo.schliessen()

    def _merken(self, personen: list[dict], beginn: float) -> None:
        jetzt = time.monotonic()
        with self._lock:
            if self._zeit:
                # Geglättete Bildrate für den Status
                momentan = 1.0 / max(1e-3, jetzt - self._zeit)
                self._fps = 0.8 * self._fps + 0.2 * momentan if self._fps else momentan
            self._personen = personen
            self._zeit = jetzt
            self._bilder += 1
            self._fehler = None

    def _erkennen(self, hailo: "_Hailo") -> list[dict] | None:
        frame = self._camera.latest() if self._camera is not None else None
        if frame is None:
            return None
        cv2, np = self._cv2, self._np
        bild = cv2.imdecode(np.frombuffer(frame[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if bild is None:
            return None
        h, w = bild.shape[:2]
        groesse = hailo.groesse

        # Letterbox wie beim Training: Seitenverhältnis behalten, Rand grau.
        # Verzerrt man stattdessen, werden Arme kürzer oder länger.
        skala = min(groesse / w, groesse / h)
        nw, nh = int(round(w * skala)), int(round(h * skala))
        dx, dy = (groesse - nw) // 2, (groesse - nh) // 2
        eingang = np.full((groesse, groesse, 3), 114, dtype=np.uint8)
        klein = cv2.resize(bild, (nw, nh), interpolation=cv2.INTER_CUBIC)
        eingang[dy:dy + nh, dx:dx + nw] = cv2.cvtColor(klein, cv2.COLOR_BGR2RGB)

        treffer = dekodieren(np, hailo.rechnen(eingang), groesse, config.POSE_MIN_SCORE)

        personen = []
        for score, box, kp in treffer:
            # Zurück ins Originalbild, dann auf 0..1
            bx = ((box[[0, 2]] - dx) / skala / w).clip(0, 1)
            by = ((box[[1, 3]] - dy) / skala / h).clip(0, 1)
            kx = ((kp[:, 0] - dx) / skala / w).clip(0, 1)
            ky = ((kp[:, 1] - dy) / skala / h).clip(0, 1)
            personen.append(_person(
                [bx[0], by[0], bx[1], by[1]],
                [[float(x), float(y), float(c)] for x, y, c in zip(kx, ky, kp[:, 2])],
                float(score), w / h,
            ))
        personen.sort(key=lambda p: -(p["box"][3] - p["box"][1]))
        return personen


def _person(box, kp, score: float, seitenverhaeltnis: float) -> dict:
    mitte = (box[0] + box[2]) / 2
    return {
        "box": [round(float(v), 3) for v in box],
        "kp": [[round(x, 3), round(y, 3), round(c, 2)] for x, y, c in kp],
        "score": round(score, 2),
        "winkel": round((0.5 - mitte) * config.CAMERA_FOV_DEG, 1),
        "seitenverhaeltnis": round(seitenverhaeltnis, 3),
        "zeit": time.monotonic(),
    }


# --- Hailo -----------------------------------------------------------------------


class _Hailo:
    """Ein geöffnetes Modell auf dem Hailo. Nur aus einem Thread benutzen.

    Die Ausgaben werden als FLOAT32 angefordert: Dann rechnet HailoRT die
    Quantisierung selbst zurück, und die Auswertung muss keine Skalen und
    Nullpunkte je Ausgabe kennen.
    """

    def __init__(self, np) -> None:
        from hailo_platform import HEF, FormatType, HailoSchedulingAlgorithm, VDevice

        self._np = np
        params = VDevice.create_params()
        # "SHARED" + Round-Robin: andere Programme (z. B. hailo-apps) dürfen
        # den Chip gleichzeitig nutzen, statt dass einer ihn blockiert.
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "SHARED"
        self._vdev = VDevice(params)
        self._ctx = None
        try:
            modell = self._vdev.create_infer_model(config.POSE_HEF)
            modell.set_batch_size(1)
            namen = [i.name for i in HEF(config.POSE_HEF).get_output_vstream_infos()]
            for name in namen:
                modell.output(name).set_format_type(FormatType.FLOAT32)
            self.groesse = int(modell.input().shape[0])
            self._ctx = modell.configure()
            self._konf = self._ctx.__enter__()
            self._namen = namen
            self._bindings = self._konf.create_bindings(output_buffers={
                name: np.empty(modell.output(name).shape, dtype=np.float32) for name in namen
            })
        except Exception:
            self.schliessen()
            raise

    def rechnen(self, eingang) -> list:
        self._bindings.input().set_buffer(self._np.ascontiguousarray(eingang))
        self._konf.run([self._bindings], 2000)
        return [self._bindings.output(name).get_buffer() for name in self._namen]

    def schliessen(self) -> None:
        try:
            if self._ctx is not None:
                self._ctx.__exit__(None, None, None)
                self._ctx = None
        except Exception:
            log.debug("Hailo-Modell schließen misslungen", exc_info=True)
        try:
            self._vdev.release()
        except Exception:
            log.debug("Hailo freigeben misslungen", exc_info=True)


def dekodieren(np, ausgaben: list, groesse: int, min_score: float) -> list:
    """Rohausgaben von YOLOv8-Pose -> [(score, box[4], kp[17, 3])] in Eingangspixeln.

    Das Modell hat je Raster (80, 40, 20 Zellen bei 640 px) drei Ausgaben,
    erkannt an der Kanalzahl: 64 (Box als Verteilung über 16 Abstände je
    Seite, "DFL"), 1 (Personen-Wahrscheinlichkeit) und 51 (17 Punkte × x, y,
    Sicherheit). Die Reihenfolge der Ausgaben im HEF ist nicht garantiert,
    deshalb wird nach Form sortiert statt nach Name.
    """
    je_raster: dict[int, dict[int, object]] = {}
    for a in ausgaben:
        a = np.asarray(a, dtype=np.float32)
        kanaele = a.shape[-1]
        a = a.reshape(-1, kanaele)
        raster = int(round(math.sqrt(a.shape[0])))
        je_raster.setdefault(raster, {})[kanaele] = a

    scores, boxen, punkte = [], [], []
    for raster, teile in je_raster.items():
        if not {64, 1, 51} <= teile.keys():
            continue
        schritt = groesse / raster
        s = teile[1][:, 0]
        if s.min() < 0.0 or s.max() > 1.0:  # Logits statt Wahrscheinlichkeiten
            s = 1.0 / (1.0 + np.exp(-s))
        auswahl = s >= min_score
        if not auswahl.any():
            continue
        gy, gx = np.meshgrid(np.arange(raster), np.arange(raster), indexing="ij")
        mitte = np.column_stack(((gx.ravel() + 0.5) * schritt, (gy.ravel() + 0.5) * schritt))[auswahl]

        # Box: je Seite eine Verteilung über 16 Abstände -> Erwartungswert
        verteilung = teile[64][auswahl].reshape(-1, 4, 16)
        verteilung = np.exp(verteilung - verteilung.max(axis=2, keepdims=True))
        verteilung /= verteilung.sum(axis=2, keepdims=True)
        abst = (verteilung @ np.arange(16, dtype=np.float32)) * schritt  # l, o, r, u
        box = np.column_stack((mitte - abst[:, :2], mitte + abst[:, 2:]))

        kp = teile[51][auswahl].reshape(-1, 17, 3).copy()
        kp[:, :, :2] = kp[:, :, :2] * 2.0 * schritt + (mitte[:, None, :] - 0.5 * schritt)
        kp[:, :, 2] = 1.0 / (1.0 + np.exp(-kp[:, :, 2]))

        scores.append(s[auswahl])
        boxen.append(box)
        punkte.append(kp)

    if not scores:
        return []
    scores = np.concatenate(scores)
    boxen = np.concatenate(boxen)
    punkte = np.concatenate(punkte)
    return [(float(scores[i]), boxen[i], punkte[i]) for i in _nms(np, boxen, scores, _NMS_IOU)]


def _nms(np, boxen, scores, iou_grenze: float) -> list[int]:
    """Doppelte Treffer derselben Person entfernen (bester bleibt)."""
    reihenfolge = list(np.argsort(-scores))
    flaeche = (boxen[:, 2] - boxen[:, 0]).clip(0) * (boxen[:, 3] - boxen[:, 1]).clip(0)
    behalten = []
    while reihenfolge:
        i = reihenfolge.pop(0)
        behalten.append(int(i))
        if not reihenfolge:
            break
        rest = np.array(reihenfolge)
        x1 = np.maximum(boxen[i, 0], boxen[rest, 0])
        y1 = np.maximum(boxen[i, 1], boxen[rest, 1])
        x2 = np.minimum(boxen[i, 2], boxen[rest, 2])
        y2 = np.minimum(boxen[i, 3], boxen[rest, 3])
        schnitt = (x2 - x1).clip(0) * (y2 - y1).clip(0)
        iou = schnitt / np.maximum(1e-6, flaeche[i] + flaeche[rest] - schnitt)
        reihenfolge = [int(j) for j, u in zip(rest, iou) if u < iou_grenze]
    return behalten


# --- Simulation (--mock) ------------------------------------------------------------

#: Ablauf der simulierten Person: (Dauer in s, Geste). Ein Durchlauf dauert
#: 40 s, jede Geste kommt einmal vor, dazwischen steht sie ruhig.
_SIM_ABLAUF = [
    (6.0, None), (3.0, "folgen"), (5.0, None), (2.0, "stopp"), (5.0, None),
    (3.0, "herkommen"), (5.0, None), (3.0, "weggehen"), (8.0, None),
]


def _simulierte_personen(t: float) -> list[dict]:
    """Eine Person, die langsam hin und her geht und Gesten zeigt."""
    dauer = sum(d for d, _ in _SIM_ABLAUF)
    rest = t % dauer
    geste, seit = None, 0.0
    for d, g in _SIM_ABLAUF:
        if rest < d:
            geste, seit = g, rest
            break
        rest -= d

    # Bildkoordinaten 0..1; das Testbild ist 4:3. "e" ist eine halbe
    # Schulterbreite in x, in y entsprechend umgerechnet.
    seitenverhaeltnis = config.CAMERA_WIDTH / config.CAMERA_HEIGHT
    mx = 0.5 + 0.15 * math.sin(t / 6.0)
    oben, hoehe = 0.12, 0.8
    e = 0.06
    ey = e * seitenverhaeltnis

    def y(anteil: float) -> float:
        return oben + anteil * hoehe

    kp = [[mx, y(0.07)], [mx + e * 0.3, y(0.05)], [mx - e * 0.3, y(0.05)],
          [mx + e * 0.6, y(0.06)], [mx - e * 0.6, y(0.06)],
          [mx + e, y(0.2)], [mx - e, y(0.2)],             # Schultern L, R
          [mx + e * 1.2, y(0.35)], [mx - e * 1.2, y(0.35)],  # Ellbogen
          [mx + e * 1.1, y(0.5)], [mx - e * 1.1, y(0.5)],    # Handgelenke
          [mx + e * 0.7, y(0.52)], [mx - e * 0.7, y(0.52)],  # Hüften
          [mx + e * 0.7, y(0.74)], [mx - e * 0.7, y(0.74)],
          [mx + e * 0.7, y(0.96)], [mx - e * 0.7, y(0.96)]]

    schulter_y = y(0.2)
    if geste == "folgen":  # rechter Arm gestreckt über den Kopf
        kp[KP_ELLBOGEN_R] = [mx - e * 1.2, schulter_y - ey * 1.2]
        kp[KP_HAND_R] = [mx - e * 1.1, y(0.07) - ey * 1.5]
    elif geste == "stopp":  # beide Arme hoch
        kp[KP_ELLBOGEN_L] = [mx + e * 1.4, schulter_y - ey * 1.0]
        kp[KP_ELLBOGEN_R] = [mx - e * 1.4, schulter_y - ey * 1.0]
        kp[KP_HAND_L] = [mx + e * 1.4, schulter_y - ey * 2.2]
        kp[KP_HAND_R] = [mx - e * 1.4, schulter_y - ey * 2.2]
    elif geste == "herkommen":  # Hand winkt vor der Brust auf und ab
        kp[KP_ELLBOGEN_R] = [mx - e * 1.6, y(0.33)]
        hub = math.sin(seit * 2 * math.pi * 1.5) * ey * 0.7
        kp[KP_HAND_R] = [mx - e * 1.3, schulter_y + ey * 0.3 + hub]
    elif geste == "weggehen":  # Hand wischt immer wieder nach außen
        kp[KP_ELLBOGEN_R] = [mx - e * 1.5, y(0.33)]
        anteil = (seit * 1.2) % 1.0  # Sägezahn: raus, schnell zurück
        kp[KP_HAND_R] = [mx - e * (0.5 + 3.0 * anteil), schulter_y + ey * 0.8]

    box = [mx - e * 2.5, oben, mx + e * 2.5, oben + hoehe]
    return [_person(box, [[x, yy, 0.9] for x, yy in kp], 0.9, seitenverhaeltnis)]
