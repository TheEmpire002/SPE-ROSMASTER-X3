"""Kamerabild für die Web-App.

Drei Varianten derselben Schnittstelle, in dieser Reihenfolge versucht:

1. ``Picamera2``  -- Kamera am CSI-Anschluss des Pi (Standard beim Pi 5)
2. ``OpenCV``     -- USB-Kamera
3. ``Mock``       -- erzeugtes Testbild, damit die App auch ohne Kamera
                     entwickelt werden kann

Ausgeliefert wird als MJPEG: eine endlose Folge einzelner Bilder in einer
HTTP-Antwort. Das kann jeder Browser ohne JavaScript und ohne zusätzliche
Bibliothek direkt in einem ``<img>`` anzeigen -- deutlich weniger Aufwand
als WebRTC, und für ein Kamerabild im eigenen WLAN völlig ausreichend.
"""

from __future__ import annotations

import io
import logging
import math
import struct
import threading
import time
import zlib
from abc import ABC, abstractmethod

from .. import config

log = logging.getLogger(__name__)

#: Ein Einzelbild: die Rohdaten und ihr MIME-Typ.
Frame = tuple[bytes, str]


class CameraBase(ABC):
    """Eine Kamera, die Einzelbilder liefern kann.

    ``open()`` muss mehrfach aufrufbar sein, ohne die Kamera ein zweites Mal
    zu belegen: ein doppelt geöffnetes Gerät lässt sich bei den meisten
    Treibern nicht mehr öffnen, und der Fehler sieht dann so aus, als wäre
    gar keine Kamera da.
    """

    name: str = "base"
    #: Klartextname für Logmeldungen.
    label: str = "Kamera"

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
            # Hinter einem Fehlschlag kann ein halb geöffnetes Gerät
            # zurückbleiben -- etwa wenn picamera2 die Kamera schon belegt
            # hat und erst der Encoder scheitert. Ohne Aufräumen wäre sie
            # für den nächsten Versuch dauerhaft blockiert.
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
            # hielte open() die Kamera für offen und täte beim Neuversuch
            # nichts.
            self._opened = False

    @abstractmethod
    def _open(self) -> None: ...

    @abstractmethod
    def _close(self) -> None: ...

    @abstractmethod
    def read(self) -> Frame | None:
        """Ein Bild holen, oder None wenn gerade keines verfügbar ist."""


# --- Kamera am CSI-Anschluss ------------------------------------------------------


class _JpegSink(io.BufferedIOBase):
    """Nimmt die fertig kodierten Bilder von picamera2 entgegen.

    Der Encoder ruft ``write()`` für jedes Bild auf. Gespeichert wird immer
    nur das neueste -- wer zu langsam abholt, überspringt Bilder, statt dass
    sich ein Rückstau bildet.
    """

    def __init__(self) -> None:
        self.frame: bytes | None = None
        self._lock = threading.Lock()

    def write(self, buf) -> int:  # type: ignore[override]
        data = bytes(buf)
        with self._lock:
            self.frame = data
        return len(data)

    def latest(self) -> bytes | None:
        with self._lock:
            return self.frame


class Picamera2Camera(CameraBase):
    name = "picamera2"
    #: Taucht in Logmeldungen auf -- deutsch, damit man danach suchen kann.
    label = "CSI-Kamera"

    def __init__(self) -> None:
        super().__init__()
        self._cam = None
        self._sink: _JpegSink | None = None

    def _open(self) -> None:
        # Alles nur auf dem Pi vorhanden.
        from picamera2 import Picamera2
        from picamera2.encoders import JpegEncoder
        from picamera2.outputs import FileOutput

        self._cam = Picamera2()
        self._cam.configure(
            self._cam.create_video_configuration(
                main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)}
            )
        )

        # Bilder werden laufend kodiert und hereingereicht, statt dass wir
        # jedes einzeln anfordern. Einzelabrufe (capture_file) sind für einen
        # Dauerstrom zu langsam und bleiben dabei gern stehen.
        self._sink = _JpegSink()
        self._cam.start_recording(
            JpegEncoder(q=config.CAMERA_QUALITY), FileOutput(self._sink)
        )

        # Auf das erste Bild warten: der Sensor braucht einen Moment für
        # Belichtung und Weißabgleich. Ohne das wäre das erste Bild schwarz
        # -- und ein Fehlschlag hier fiele erst später auf.
        frist = time.monotonic() + 5.0
        while self._sink.latest() is None:
            if time.monotonic() > frist:
                raise RuntimeError("Kamera liefert kein Bild (5 s gewartet)")
            time.sleep(0.1)

        log.info("Kamera (CSI) bereit")

    def _close(self) -> None:
        if self._cam is None:
            return
        try:
            self._cam.stop_recording()
            # stop_recording() beendet nur den Bildstrom -- die Kamera bleibt
            # dem Prozess zugeteilt. Ohne close() scheitert jedes spätere
            # Öffnen mit "Device or resource busy".
            self._cam.close()
        except Exception:
            log.exception("Kamera ließ sich nicht sauber schließen")
        finally:
            self._cam = None
            self._sink = None

    def read(self) -> Frame | None:
        if self._sink is None:
            return None
        data = self._sink.latest()
        return (data, "image/jpeg") if data else None


# --- USB-Kamera -------------------------------------------------------------------


#: Geräte, die unter /dev/video* auftauchen, aber keine Kameras sind.
#: Auf dem Pi 5 sind das die Bildverarbeitungs- und Codec-Einheiten --
#: sie lassen sich öffnen und blockieren dann beim Lesen.
KEINE_KAMERA = ("pispbe", "hevc", "codec", "isp", "cfe", "rpivid")


def _geraetename(idx: int) -> str:
    """Was der Kernel zu /dev/videoN sagt, z.B. 'USB 2.0 Camera'."""
    try:
        with open(f"/sys/class/video4linux/video{idx}/name") as f:
            return f.read().strip()
    except OSError:
        return ""


class OpenCVCamera(CameraBase):
    name = "usb"
    label = "USB-Kamera"

    def __init__(self, index: int | None = None) -> None:
        super().__init__()
        self._index = config.CAMERA_INDEX if index is None else index
        self._cap = None

    def _kandidaten(self) -> list[int]:
        """Welche Geräte-Indizes probiert werden.

        Auf dem Raspberry Pi ist ``/dev/video0`` nicht zwangsläufig die
        Kamera, deshalb wird nicht darauf beharrt. Umgekehrt dürfen auch
        nicht blind alle Geräte probiert werden: ein Pi 5 bringt rund zwanzig
        ``/dev/video*`` mit, von denen die meisten Codec- und
        Bildverarbeitungs-Einheiten sind.
        """
        import glob
        import re

        indizes = [self._index] if self._index is not None else []
        for pfad in sorted(glob.glob("/dev/video*")):
            treffer = re.search(r"(\d+)$", pfad)
            if not treffer:
                continue
            idx = int(treffer.group(1))
            if idx in indizes:
                continue
            bezeichnung = _geraetename(idx).lower()
            if any(wort in bezeichnung for wort in KEINE_KAMERA):
                continue
            indizes.append(idx)

        # Die Kamera mit passendem Namen nach vorn, ohne die übrige
        # Reihenfolge zu ändern. Ist sie nicht da, gilt der Index wie bisher.
        if config.CAMERA_NAME:
            gesucht = config.CAMERA_NAME.lower()
            indizes.sort(key=lambda i: gesucht not in _geraetename(i).lower())
        return indizes or [0]

    def _open(self) -> None:
        import cv2

        versucht: list[str] = []
        for idx in self._kandidaten():
            cap, api = self._oeffnen(cv2, idx)
            if cap is None:
                name = _geraetename(idx)
                versucht.append(f"video{idx}{f' ({name})' if name else ''}: öffnet nicht")
                continue

            gewaehlt = self._einstellung_finden(cv2, cap, idx, versucht)
            if gewaehlt is None:
                cap.release()
                continue

            self._cap = cap
            self._index = idx
            log.info(
                "Kamera (USB, /dev/video%d%s) bereit -- %s, %s",
                idx,
                f", {_geraetename(idx)}" if _geraetename(idx) else "",
                api,
                gewaehlt,
            )
            return

        if not versucht:
            raise RuntimeError("keine /dev/video*-Geräte vorhanden")
        raise RuntimeError("keine brauchbare Kamera unter " + "; ".join(versucht))

    @staticmethod
    def _oeffnen(cv2, idx: int):
        """Öffnet das Gerät, bevorzugt über V4L2.

        Ohne ausdrückliche Angabe sucht sich OpenCV auf dem Pi gern einen
        GStreamer-Pfad, der an einer schlichten USB-Kamera scheitert -- und
        zwar wortlos mit ``isOpened() == False``.
        """
        for name, api in (("V4L2", cv2.CAP_V4L2), ("Standard", cv2.CAP_ANY)):
            cap = cv2.VideoCapture(idx, api)
            if cap.isOpened():
                return cap, name
            cap.release()
        return None, ""

    def _einstellung_finden(self, cv2, cap, idx: int, versucht: list[str]) -> str | None:
        """Probiert Format und Auflösung durch, bis ein Bild kommt.

        Es gibt keine Einstellung, die alle Kameras mögen: MJPG rettet die
        einen (sonst reicht die USB-Bandbreite nicht) und bricht die anderen,
        die es gar nicht können. Deshalb der Reihe nach von der besten zur
        anspruchslosesten Variante.
        """
        w, h = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
        varianten = [
            ("MJPG", w, h),
            (None, w, h),       # Format unangetastet lassen
            ("YUYV", w, h),
            (None, 640, 480),
            (None, 320, 240),
        ]

        for fourcc, breite, hoehe in varianten:
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, breite)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, hoehe)

            frame = self._warmlaufen(cap)
            if frame is not None:
                ist_h, ist_w = frame.shape[:2]
                return f"{fourcc or 'Standardformat'} {ist_w}x{ist_h}"

            versucht.append(f"video{idx} {fourcc or 'Standardformat'} {breite}x{hoehe}: kein Bild")

        return None

    @staticmethod
    def _warmlaufen(cap, sekunden: float = 1.5):
        """Liest wiederholt, bis ein Bild kommt oder die Zeit abläuft.

        Ein einzelner Leseversuch genügt nicht: viele USB-Kameras liefern
        direkt nach dem Öffnen noch nichts und brauchen ein paar hundert
        Millisekunden. Ein sofortiges Aufgeben sieht dann aus, als wäre die
        Kamera defekt.
        """
        frist = time.monotonic() + sekunden
        while time.monotonic() < frist:
            ok, frame = cap.read()
            if ok and frame is not None and getattr(frame, "size", 0) > 0:
                return frame
            time.sleep(0.05)
        return None

    def _close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def read(self) -> Frame | None:
        if self._cap is None:
            return None
        import cv2

        ok, frame = self._cap.read()
        if not ok:
            return None
        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), config.CAMERA_QUALITY]
        )
        if not ok:
            return None
        return buf.tobytes(), "image/jpeg"


# --- Testbild ohne Kamera -----------------------------------------------------------


def _png(width: int, height: int, rows: list[bytearray]) -> bytes:
    """Baut ein PNG aus RGB-Zeilen.

    Selbst gebaut, weil es dafür keine Bibliothek braucht: zlib reicht, und
    der Mock soll auf jedem Rechner laufen -- auch ohne OpenCV oder Pillow.
    """

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8 bit, RGB
    # Jede Zeile bekommt ein führendes Filter-Byte (0 = kein Filter).
    raw = b"".join(b"\x00" + bytes(row) for row in rows)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


class MockCamera(CameraBase):
    """Erzeugt ein bewegtes Testbild -- kein Standbild.

    Die Bewegung ist Absicht: an einem stehenden Bild sieht man nicht, ob
    der Stream noch läuft oder längst eingefroren ist.
    """

    name = "mock"
    label = "Testbild"

    WIDTH = 320
    HEIGHT = 240

    def __init__(self) -> None:
        super().__init__()
        self._frame = 0

    def _open(self) -> None:
        log.info("Kamera: Testbild (keine echte Kamera)")

    def _close(self) -> None:
        pass

    def read(self) -> Frame | None:
        if not self.opened:
            return None

        self._frame += 1
        # Ein heller Balken wandert waagerecht durchs Bild.
        pos = int((math.sin(self._frame / 12.0) * 0.5 + 0.5) * (self.WIDTH - 1))

        rows: list[bytearray] = []
        for y in range(self.HEIGHT):
            row = bytearray(self.WIDTH * 3)
            for x in range(self.WIDTH):
                i = x * 3
                if abs(x - pos) < 14:
                    row[i], row[i + 1], row[i + 2] = 0x4D, 0xA3, 0xFF  # blau
                else:
                    grau = 0x22 + (y * 24 // self.HEIGHT)
                    row[i] = row[i + 1] = row[i + 2] = grau
            rows.append(row)

        return _png(self.WIDTH, self.HEIGHT, rows), "image/png"


# --- Auswahl und Hintergrundbetrieb ---------------------------------------------------


def create_camera(use_mock: bool = False) -> CameraBase:
    """Nimmt die erste Kamera, die sich öffnen lässt -- und lässt sie offen.

    Wichtig: die geöffnete Instanz wird zurückgegeben, nicht eine frische.
    Wer zum Prüfen öffnet und danach eine zweite Instanz anlegt, bekommt
    "Device or resource busy" -- entweder weil der Treiber das Gerät noch
    nicht freigegeben hat, oder weil das Schließen es gar nicht freigibt
    (so bei picamera2: ``stop()`` reicht nicht, es braucht ``close()``).
    """
    if use_mock:
        kamera: CameraBase = MockCamera()
        kamera.open()
        return kamera

    for kandidat in (Picamera2Camera, OpenCVCamera):
        kamera = kandidat()
        try:
            kamera.open()
            return kamera
        except Exception as exc:
            log.warning("Kamera: %s nicht nutzbar -- %s", kamera.label, exc)
            # Auch nach einem Fehlschlag aufräumen: ein halb geöffnetes
            # Gerät würde den nächsten Kandidaten blockieren.
            try:
                kamera.close()
            except Exception:
                pass

    log.warning("Keine Kamera gefunden -- es wird ein Testbild angezeigt")
    log.warning("Ursache finden: python scripts/check_camera.py")
    ersatz = MockCamera()
    ersatz.open()
    return ersatz


class CameraService:
    """Hält ein aktuelles Bild bereit, das sich mehrere Clients teilen.

    Ohne diese Schicht würde jeder Browser-Tab die Kamera erneut öffnen --
    was bei den meisten Geräten schlicht fehlschlägt. Stattdessen holt ein
    Hintergrund-Thread Bilder, alle Streams greifen auf das jeweils letzte
    zu. Nebenbei kostet ein zweiter Zuschauer dadurch fast nichts.
    """

    def __init__(self, camera: CameraBase) -> None:
        self._camera = camera
        self._latest: Frame | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        # Für /camera/status -- damit sich "es kommt kein Bild" beantworten
        # lässt, ohne im Log zu suchen.
        self._bilder = 0
        self._letztes_bild = 0.0
        self._letzter_fehler: str | None = None
        self._neu_geoeffnet = 0

    def status(self) -> dict:
        """Kurzbericht, warum ein Bild kommt oder eben nicht."""
        with self._lock:
            frame = self._latest
            bilder = self._bilder
            letztes = self._letztes_bild
            fehler = self._letzter_fehler

        laeuft = self._thread is not None and self._thread.is_alive()
        return {
            "kamera": self._camera.label,
            "geoeffnet": self._camera.opened,
            "thread_laeuft": laeuft,
            "bilder_gesamt": bilder,
            "letztes_bild_vor_s": (
                round(time.monotonic() - letztes, 1) if letztes else None
            ),
            "bildgroesse_bytes": len(frame[0]) if frame else None,
            "format": frame[1] if frame else None,
            "letzter_fehler": fehler,
            "neu_geoeffnet": self._neu_geoeffnet,
        }

    @property
    def name(self) -> str:
        return self._camera.name

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            # Normalerweise schon offen (create_camera lässt sie offen);
            # open() ist absichtlich wiederholbar und tut dann nichts.
            self._camera.open()
        except Exception as exc:
            log.exception("Kamera lässt sich nicht öffnen -- kein Bild")
            with self._lock:
                self._letzter_fehler = f"Öffnen fehlgeschlagen: {exc}"
            return

        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._camera.close()

    def latest(self) -> Frame | None:
        with self._lock:
            return self._latest

    def _neu_oeffnen(self) -> None:
        log.warning("Kamera liefert nichts mehr -- öffne sie neu")
        try:
            self._camera.close()
        except Exception:
            log.debug("Kamera schließen misslungen", exc_info=True)
        try:
            self._camera.open()
        except Exception as exc:
            with self._lock:
                self._letzter_fehler = f"Neu öffnen: {type(exc).__name__}: {exc}"
            log.warning("Kamera lässt sich noch nicht wieder öffnen -- %s", exc)
            return
        self._neu_geoeffnet += 1
        log.warning("Kamera wieder geöffnet")

    def _loop(self) -> None:
        interval = 1.0 / max(1, config.CAMERA_FPS)
        fehler = 0
        letzte_aenderung = time.monotonic()
        eingefroren_gemeldet = False
        reopen_wartezeit = config.CAMERA_REOPEN_S

        while not self._stop.is_set():
            begin = time.monotonic()
            try:
                frame = self._camera.read()
                if frame is None:
                    with self._lock:
                        if self._bilder == 0:
                            self._letzter_fehler = "Kamera liefert None (kein Bild)"
                else:
                    with self._lock:
                        neu = frame != self._latest
                        # Nur bei echter Änderung ersetzen: der Stream
                        # erkennt am Objekt, ob er ein Bild schon
                        # verschickt hat. Blindes Zuweisen würde jedes
                        # Standbild erneut durchs WLAN schicken.
                        if neu:
                            self._latest = frame
                            self._bilder += 1
                            self._letztes_bild = begin
                            self._letzter_fehler = None
                    fehler = 0
                    if neu:
                        letzte_aenderung = begin
                        reopen_wartezeit = config.CAMERA_REOPEN_S
                        if eingefroren_gemeldet:
                            log.info("Kamera liefert wieder Bilder")
                            eingefroren_gemeldet = False
            except Exception as exc:
                fehler += 1
                with self._lock:
                    self._letzter_fehler = f"{type(exc).__name__}: {exc}"
                # Einzelne Aussetzer sind normal. Die ersten paar trotzdem
                # melden -- sonst sucht man bei einem stehenden Bild lange.
                if fehler <= 3 or fehler % 100 == 0:
                    log.exception("Kamera liefert kein Bild (%d. Versuch)", fehler)

            # Ein stehendes Bild sieht im Browser aus wie ein laufender
            # Stream. Ohne diese Meldung merkt man den Stillstand nicht.
            if not eingefroren_gemeldet and begin - letzte_aenderung > 5.0:
                log.warning(
                    "Kamera seit %.0f s ohne neues Bild -- Stream steht",
                    begin - letzte_aenderung,
                )
                eingefroren_gemeldet = True

            # Steht der Strom länger, ist das Gerät meist weg -- etwa nach
            # einem Aussetzer am USB-Hub. Dann hilft nur neu öffnen; am
            # alten Gerät zu lesen liefert nie wieder etwas.
            if begin - letzte_aenderung > reopen_wartezeit:
                self._neu_oeffnen()
                letzte_aenderung = time.monotonic()
                reopen_wartezeit = min(reopen_wartezeit * 2, config.CAMERA_REOPEN_MAX_S)

            rest = interval - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)
