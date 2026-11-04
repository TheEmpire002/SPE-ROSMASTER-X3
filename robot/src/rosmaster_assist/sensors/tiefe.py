"""Tiefenkamera (Orbbec Astra): Hindernisse unter und über der Lidar-Ebene.

Der Lidar misst nur in seiner Ebene, rund 17 cm über dem Boden. Was tiefer
liegt -- Stuhlfüße, Kufen, Kabelkanäle, Bettgestell --, sieht er nicht; der
Roboter fährt auf und bleibt hängen. Die Tiefenkamera schaut nach vorn und
misst für jeden Bildpunkt die Entfernung. Daraus wird hier:

1. eine Punktwolke im Roboterrahmen (Einbauhöhe und -neigung aus config),
2. der **Boden**: eine Ebene durch die Punkte nahe der erwarteten Bodenhöhe.
   Sie gleicht Einbaufehler und Nicken des Roboters aus. Ohne sie würde ein
   Grad falsche Neigung in 1,5 m schon 2,6 cm "Hindernis" erzeugen.
3. **Hindernispunkte**: alles zwischen ``TIEFE_MIN_HOEHE_M`` und
   ``TIEFE_MAX_HOEHE_M`` über dem Boden, auf ein Raster gebracht. Eine
   Zelle zählt erst ab ``TIEFE_MIN_PUNKTE_ZELLE`` Punkten -- Ausreißer im
   Tiefenbild sollen den Roboter nicht anhalten.
4. **Freie Zellen**: wo die Kamera Boden sieht und nichts darauf. Damit
   räumt das Hindernisgedächtnis Punkte wieder ab.

Ergebnis sind 2D-Punkte wie vom Lidar (x vorn, y links). So können
Fahrschutz, Erkunder und Kartierfahrt sie ohne Sonderfälle mitbenutzen.

Angesprochen wird die Kamera über Orbbecs OpenNI2-Build per ctypes -- ohne
ROS und ohne Zusatzpaket. Das PyPI-Paket ``openni`` läuft unter Python 3.13
nicht mehr (ctypes-Enums), und die paar Aufrufe sind schnell selbst
geschrieben.

Die Kamera läuft nur, solange jemand sie braucht (``aktivieren``): Der
Projektor zieht Strom, und am X3 heißt Strom am USB-Hub Aussetzer
(docs/hardware.md). Fällt sie aus, verbindet der Dienst sich selbst neu.
"""

from __future__ import annotations

import ctypes as C
import logging
import math
import os
import threading
import time
from typing import NamedTuple

from .. import config

log = logging.getLogger(__name__)

# --- OpenNI2 (C-Schnittstelle, OniCAPI.h) -------------------------------------------

_ONI_API_VERSION = 2002          # 2.2
_SENSOR_TIEFE = 3
_FORMAT_TIEFE_1MM = 100
_PROP_HFOV = 1
_PROP_VFOV = 2
_PROP_VIDEOMODUS = 3
_PROP_SPIEGELN = 7


class _Modus(C.Structure):
    _fields_ = [("format", C.c_int), ("breite", C.c_int), ("hoehe", C.c_int), ("fps", C.c_int)]


class _Bild(C.Structure):
    _fields_ = [
        ("groesse", C.c_int), ("daten", C.c_void_p), ("sensor", C.c_int),
        ("zeitstempel", C.c_uint64), ("nummer", C.c_int),
        ("breite", C.c_int), ("hoehe", C.c_int), ("modus", _Modus),
        ("zuschnitt", C.c_int), ("zuschnitt_x", C.c_int), ("zuschnitt_y", C.c_int),
        ("zeilenbreite", C.c_int),
    ]


class _SensorInfo(C.Structure):
    _fields_ = [("sensor", C.c_int), ("anzahl", C.c_int), ("modi", C.POINTER(_Modus))]


class _OpenNI:
    """Das Nötigste aus OpenNI2: ein Gerät, ein Tiefenstrom."""

    _lib = None  # einmal pro Prozess laden und initialisieren

    def __init__(self, pfad: str) -> None:
        if _OpenNI._lib is None:
            lib = C.CDLL(os.path.join(pfad, "libOpenNI2.so"))
            lib.oniGetExtendedError.restype = C.c_char_p
            lib.oniStreamGetSensorInfo.restype = C.POINTER(_SensorInfo)
            self._pruefen(lib, lib.oniInitialize(_ONI_API_VERSION), "initialisieren")
            _OpenNI._lib = lib
        self._l = _OpenNI._lib
        self._geraet = C.c_void_p()
        self._strom = C.c_void_p()
        self.breite = self.hoehe = 0
        self.hfov = self.vfov = 0.0

    @staticmethod
    def _pruefen(lib, status: int, was: str) -> None:
        if status != 0:
            fehler = lib.oniGetExtendedError() or b""
            raise OSError(f"OpenNI2 {was}: Status {status} {fehler.decode(errors='replace').strip()}")

    def oeffnen(self, breite: int, hoehe: int) -> None:
        l = self._l
        self._pruefen(l, l.oniDeviceOpen(None, C.byref(self._geraet)), "Gerät öffnen")
        self._pruefen(l, l.oniDeviceCreateStream(self._geraet, _SENSOR_TIEFE, C.byref(self._strom)),
                      "Tiefenstrom anlegen")
        info = l.oniStreamGetSensorInfo(self._strom).contents
        modi = [info.modi[i] for i in range(info.anzahl)]
        passend = [m for m in modi if m.format == _FORMAT_TIEFE_1MM
                   and m.breite == breite and m.hoehe == hoehe]
        if passend:
            modus = _Modus(passend[0].format, passend[0].breite, passend[0].hoehe, passend[0].fps)
            self._pruefen(l, l.oniStreamSetProperty(self._strom, _PROP_VIDEOMODUS, C.byref(modus),
                                                    C.sizeof(modus)), "Videomodus setzen")
        # Nicht spiegeln: links im Bild soll links vom Roboter sein.
        aus = C.c_int(0)
        l.oniStreamSetProperty(self._strom, _PROP_SPIEGELN, C.byref(aus), C.sizeof(aus))
        wert, groesse = C.c_float(), C.c_int(C.sizeof(C.c_float))
        self._pruefen(l, l.oniStreamGetProperty(self._strom, _PROP_HFOV, C.byref(wert), C.byref(groesse)),
                      "Bildwinkel lesen")
        self.hfov = float(wert.value)
        self._pruefen(l, l.oniStreamGetProperty(self._strom, _PROP_VFOV, C.byref(wert), C.byref(groesse)),
                      "Bildwinkel lesen")
        self.vfov = float(wert.value)
        self._pruefen(l, l.oniStreamStart(self._strom), "Tiefenstrom starten")

    def lesen(self, frist_ms: int = 500):
        """Nächstes Tiefenbild als uint16-Array (mm, 0 = ungültig), None bei Frist."""
        import numpy as np

        l = self._l
        index = C.c_int()
        stroeme = (C.c_void_p * 1)(self._strom)
        if l.oniWaitForAnyStream(stroeme, 1, C.byref(index), frist_ms) != 0:
            return None
        zeiger = C.POINTER(_Bild)()
        self._pruefen(l, l.oniStreamReadFrame(self._strom, C.byref(zeiger)), "Bild lesen")
        try:
            b = zeiger.contents
            daten = np.ctypeslib.as_array(C.cast(b.daten, C.POINTER(C.c_uint16)),
                                          shape=(b.hoehe, b.zeilenbreite // 2))
            bild = daten[:, :b.breite].copy()
        finally:
            l.oniFrameRelease(zeiger)
        self.breite, self.hoehe = bild.shape[1], bild.shape[0]
        return bild

    def schliessen(self) -> None:
        l = self._l
        if self._strom:
            l.oniStreamStop(self._strom)
            l.oniStreamDestroy(self._strom)
            self._strom = C.c_void_p()
        if self._geraet:
            l.oniDeviceClose(self._geraet)
            self._geraet = C.c_void_p()


# --- Auswertung ----------------------------------------------------------------------


class TiefenBild(NamedTuple):
    """Ein ausgewertetes Tiefenbild, alles im Roboterrahmen (x vorn, y links)."""

    zeit: float
    hindernisse: object          # Nx2: Mitten der Rasterzellen mit Hindernis
    frei: object                 # Mx2: Mitten der Zellen, in denen nur Boden ist
    boden: tuple | None          # (a, b, c) der Ebene z = a*x + b*y + c, None = geschätzt
    # Für die Anzeige (/tiefe/stream): Rohbild in mm und welche Bildpunkte
    # als Hindernis zählten (jeder zweite Punkt, also halbe Auflösung).
    roh: object = None
    maske: object = None


def auswerten(tiefe_mm, hfov: float, vfov: float, boden_vorher=None):
    """Tiefenbild -> (Hindernispunkte, freie Zellen, Bodenebene, neu erkannt?,
    Hindernis-Maske).

    Reine Rechnung, ohne Kamera prüfbar. ``boden_vorher`` ist die zuletzt
    erkannte Ebene; findet sich diesmal keine, gilt sie weiter (dann ist der
    vierte Wert False). Die Maske hat die halbe Bildauflösung und markiert
    jeden Bildpunkt, der als Hindernis zählte -- nur für die Anzeige.
    """
    import numpy as np

    schritt = 2  # jeder zweite Bildpunkt reicht für 5-cm-Zellen
    t = tiefe_mm[::schritt, ::schritt].astype(np.float32) / 1000.0
    h_px, b_px = tiefe_mm.shape
    v, u = np.mgrid[0:h_px:schritt, 0:b_px:schritt]
    gueltig = t > 0.0
    z_c = t[gueltig]
    fx = (b_px / 2.0) / math.tan(hfov / 2.0)
    fy = (h_px / 2.0) / math.tan(vfov / 2.0)
    x_c = (u[gueltig] - b_px / 2.0) * z_c / fx   # rechts
    y_c = (v[gueltig] - h_px / 2.0) * z_c / fy   # unten

    # Kamera -> Roboterrahmen: vorn = Blickrichtung, links = -rechts, oben = -unten,
    # dann um die Neigung nach unten drehen und an den Einbauort schieben.
    p = math.radians(config.TIEFE_KAMERA_NEIGUNG_GRAD)
    xr, zr = z_c, -y_c
    x = xr * math.cos(p) + zr * math.sin(p) + config.TIEFE_KAMERA_VOR_M
    y = -x_c
    z = -xr * math.sin(p) + zr * math.cos(p) + config.TIEFE_KAMERA_HOEHE_M

    imbereich = (x >= config.TIEFE_MIN_M) & (x <= config.TIEFE_MAX_M + 0.5)
    x, y, z = x[imbereich], y[imbereich], z[imbereich]
    # Wo im (ausgedünnten) Bild jeder verbliebene Punkt liegt -- für die Maske.
    stelle = np.flatnonzero(gueltig)[imbereich]

    # Boden: Ebene durch die Punkte nahe der erwarteten Bodenhöhe, zweimal
    # mit Ausreißern weniger. Nur übernehmen, wenn sie plausibel liegt.
    boden = None
    kandidat = np.abs(z) < 0.08
    for _ in range(2):
        if kandidat.sum() < 200:
            break
        A = np.column_stack((x[kandidat], y[kandidat], np.ones(int(kandidat.sum()))))
        loesung, *_ = np.linalg.lstsq(A, z[kandidat], rcond=None)
        rest = z - (loesung[0] * x + loesung[1] * y + loesung[2])
        kandidat = kandidat & (np.abs(rest) < 0.025)
        boden = loesung
    if boden is not None and (abs(boden[0]) > 0.15 or abs(boden[1]) > 0.15 or abs(boden[2]) > 0.08
                              or kandidat.sum() < 200):
        boden = None
    neu = boden is not None
    if boden is None:
        boden = boden_vorher
    a, b, c = boden if boden is not None else (0.0, 0.0, 0.0)
    hoehe = z - (a * x + b * y + c)

    zelle = config.TIEFE_ZELLE_M
    min_hoehe, reichweite = config.TIEFE_MIN_HOEHE_M, config.TIEFE_MAX_M
    if boden is None:
        # Boden nie gesehen (dunkler, glänzender Boden; Roboter hochgestellt):
        # dann zählen nur die Einbauwerte, und deren Fehler wächst mit der
        # Entfernung. Lieber ein flaches Hindernis übersehen als den Boden
        # selbst zum Hindernis machen.
        min_hoehe += config.TIEFE_OHNE_BODEN_ZUSCHLAG_M
        reichweite = min(reichweite, config.TIEFE_OHNE_BODEN_MAX_M)
    nah = x <= reichweite

    def zellen(maske):
        """Mitten der Zellen mit genug Punkten aus ``maske``."""
        if not maske.any():
            return np.zeros((0, 2)), set()
        ix = np.floor(x[maske] / zelle).astype(np.int32)
        iy = np.floor(y[maske] / zelle).astype(np.int32)
        schluessel, anzahl = np.unique(np.column_stack((ix, iy)), axis=0, return_counts=True)
        voll = schluessel[anzahl >= config.TIEFE_MIN_PUNKTE_ZELLE]
        return (voll + 0.5) * zelle, {(int(i), int(j)) for i, j in voll}

    hindernis = nah & (hoehe >= min_hoehe) & (hoehe <= config.TIEFE_MAX_HOEHE_M)
    hindernisse, belegt = zellen(hindernis)
    maske = np.zeros(t.shape, dtype=bool)
    maske.flat[stelle[hindernis]] = True
    frei, frei_zellen = zellen(nah & (np.abs(hoehe) < 0.02))
    if len(frei) and belegt:
        ix = np.floor(frei[:, 0] / zelle).astype(int)
        iy = np.floor(frei[:, 1] / zelle).astype(int)
        nicht_belegt = np.array([(i, j) not in belegt for i, j in zip(ix, iy)])
        frei = frei[nicht_belegt]
    boden = tuple(float(w) for w in boden) if boden is not None else None
    return hindernisse, frei, boden, neu, maske


# --- Dienst --------------------------------------------------------------------------


class TiefenKamera:
    """Liest die Kamera im Hintergrund, solange jemand sie braucht."""

    def __init__(self) -> None:
        self._pfad = os.path.expanduser(config.TIEFE_OPENNI_PFAD)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wecken = threading.Event()
        self._thread: threading.Thread | None = None
        self._wer: set[str] = set()
        self._bild: TiefenBild | None = None
        self._boden = None
        self._boden_zeit = 0.0
        self._verbunden = False
        self._fehler: str | None = None
        self._bilder = 0
        self._fps = 0.0
        self._hfov = self._vfov = 0.0

    @property
    def verfuegbar(self) -> bool:
        """Ob die Bibliothek da ist -- ohne sie gibt es keine Tiefe."""
        return config.TIEFE_AKTIV and os.path.exists(os.path.join(self._pfad, "libOpenNI2.so"))

    @property
    def hfov(self) -> float:
        """Horizontaler Bildwinkel (Bogenmaß), 0 solange nie verbunden."""
        return self._hfov

    def start(self) -> None:
        if self._thread is not None or not self.verfuegbar:
            return
        self._thread = threading.Thread(target=self._schleife, name="tiefe", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wecken.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def aktivieren(self, wer: str, an: bool) -> None:
        """Kamera für ``wer`` an- oder abmelden. Sie läuft, solange einer sie will."""
        with self._lock:
            if an:
                self._wer.add(wer)
            else:
                self._wer.discard(wer)
                if not self._wer:
                    self._bild = None
        self._wecken.set()

    def bild(self) -> TiefenBild | None:
        """Letztes ausgewertetes Bild, None wenn zu alt oder keins."""
        with self._lock:
            b = self._bild
        if b is None or time.monotonic() - b.zeit > config.TIEFE_MAX_ALTER_S:
            return None
        return b

    def status(self) -> dict:
        b = self.bild()
        with self._lock:
            wer = sorted(self._wer)
        boden = b.boden if b is not None else None
        # Nächster Hindernispunkt (x vorn, y links) -- zum Prüfen, ob links und
        # rechts stimmen: Hand links vor die Kamera, dann muss y positiv sein.
        naechstes = None
        if b is not None and len(b.hindernisse):
            import numpy as np

            i = int(np.argmin(np.hypot(b.hindernisse[:, 0], b.hindernisse[:, 1])))
            naechstes = [round(float(b.hindernisse[i, 0]), 2), round(float(b.hindernisse[i, 1]), 2)]
        return {
            "verfuegbar": self.verfuegbar,
            "verbunden": self._verbunden,
            "aktiv_fuer": wer,
            "bilder_pro_s": round(self._fps, 1),
            "hindernisse": int(len(b.hindernisse)) if b is not None else None,
            "frei_zellen": int(len(b.frei)) if b is not None else None,
            "naechstes": naechstes,
            # Erkannte Einbaulage: Kamerahöhe über dem Boden und Neigung.
            "boden_erkannt": boden is not None,
            "kamera_hoehe_m": round(config.TIEFE_KAMERA_HOEHE_M - boden[2], 3) if boden else None,
            "neigung_grad": round(config.TIEFE_KAMERA_NEIGUNG_GRAD + math.degrees(math.atan(boden[0])), 1)
            if boden else None,
            "bildwinkel_grad": round(math.degrees(self._hfov), 1) if self._hfov else None,
            "fehler": self._fehler,
        }

    def _gebraucht(self) -> bool:
        with self._lock:
            return bool(self._wer)

    def _schleife(self) -> None:
        kamera: _OpenNI | None = None
        letzte_auswertung = 0.0
        zaehler_start, zaehler = time.monotonic(), 0
        while not self._stop.is_set():
            if not self._gebraucht():
                if kamera is not None:
                    # Abschalten, nicht nur nicht lesen: Der Projektor zieht Strom.
                    self._schliessen(kamera)
                    kamera = None
                    log.info("Tiefenkamera aus (niemand braucht sie)")
                self._wecken.wait(1.0)
                self._wecken.clear()
                continue
            try:
                if kamera is None:
                    kamera = _OpenNI(self._pfad)
                    kamera.oeffnen(config.TIEFE_BREITE, config.TIEFE_HOEHE_PX)
                    self._hfov, self._vfov = kamera.hfov, kamera.vfov
                    self._verbunden = True
                    self._fehler = None
                    log.info("Tiefenkamera an (Bildwinkel %.0f° x %.0f°)",
                             math.degrees(kamera.hfov), math.degrees(kamera.vfov))
                tiefe = kamera.lesen()
                if tiefe is None:
                    raise OSError("liefert keine Bilder")
                jetzt = time.monotonic()
                # Die Kamera liefert 30 Bilder/s; ausgewertet wird seltener.
                # Lesen muss trotzdem jedes, sonst stauen sie sich.
                if jetzt - letzte_auswertung < 1.0 / max(0.5, config.TIEFE_HZ):
                    continue
                letzte_auswertung = jetzt
                boden_vorher = self._boden if jetzt - self._boden_zeit < 10.0 else None
                hindernisse, frei, boden, neu, maske = auswerten(
                    tiefe, kamera.hfov, kamera.vfov, boden_vorher)
                if neu:
                    self._boden, self._boden_zeit = boden, jetzt
                with self._lock:
                    if self._wer:
                        self._bild = TiefenBild(jetzt, hindernisse, frei, boden, tiefe, maske)
                zaehler += 1
                if jetzt - zaehler_start >= 2.0:
                    self._fps = zaehler / (jetzt - zaehler_start)
                    zaehler_start, zaehler = jetzt, 0
            except Exception as exc:
                # USB-Aussetzer, Kamera abgezogen, Bibliothek fehlt: schließen,
                # warten, neu versuchen. Ein Ausfall darf nie den Server stören.
                if self._fehler != str(exc):
                    log.warning("Tiefenkamera: %s -- neuer Versuch in %.0f s", exc,
                                config.TIEFE_NEUVERSUCH_S)
                self._fehler = str(exc)
                self._verbunden = False
                with self._lock:
                    self._bild = None
                if kamera is not None:
                    self._schliessen(kamera)
                    kamera = None
                self._stop.wait(config.TIEFE_NEUVERSUCH_S)
        if kamera is not None:
            self._schliessen(kamera)

    def _schliessen(self, kamera: _OpenNI) -> None:
        try:
            kamera.schliessen()
        except Exception:
            log.debug("Tiefenkamera schließen fehlgeschlagen", exc_info=True)
        self._verbunden = False


def create_tiefe(use_mock: bool = False) -> TiefenKamera | None:
    """Auf dem Roboter die Astra, in der Simulation keine Tiefe.

    Bewusst keine erfundene Tiefe in der Simulation: Sie würde Hindernisse
    zeigen, die es nicht gibt. Alles, was Tiefe nutzt, kommt ohne sie aus.
    """
    if use_mock or not config.TIEFE_AKTIV:
        return None
    kamera = TiefenKamera()
    if not kamera.verfuegbar:
        log.warning("Tiefenkamera: keine OpenNI2-Bibliothek in %s -- Tiefe aus "
                    "(scripts/orbbec_einrichten.sh)", kamera._pfad)
        return None
    return kamera
