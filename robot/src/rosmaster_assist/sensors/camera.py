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

    def __init__(self) -> None:
        self._opened = False

    @property
    def opened(self) -> bool:
        return self._opened

    def open(self) -> None:
        if self._opened:
            return
        self._open()
        self._opened = True

    def close(self) -> None:
        if not self._opened:
            return
        self._close()
        self._opened = False

    @abstractmethod
    def _open(self) -> None: ...

    @abstractmethod
    def _close(self) -> None: ...

    @abstractmethod
    def read(self) -> Frame | None:
        """Ein Bild holen, oder None wenn gerade keines verfügbar ist."""


# --- Kamera am CSI-Anschluss ------------------------------------------------------


class Picamera2Camera(CameraBase):
    name = "picamera2"

    def __init__(self) -> None:
        super().__init__()
        self._cam = None

    def _open(self) -> None:
        from picamera2 import Picamera2  # nur auf dem Pi vorhanden

        self._cam = Picamera2()
        self._cam.configure(
            self._cam.create_video_configuration(
                main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)}
            )
        )
        self._cam.start()
        # Der Sensor braucht einen Moment für Belichtung und Weißabgleich,
        # sonst sind die ersten Bilder schwarz.
        time.sleep(1.0)
        log.info("Kamera (CSI) bereit")

    def _close(self) -> None:
        if self._cam is None:
            return
        try:
            self._cam.stop()
            # stop() beendet nur den Bildstrom -- die Kamera bleibt dem
            # Prozess zugeteilt. Ohne close() scheitert jedes spätere
            # Öffnen mit "Device or resource busy".
            self._cam.close()
        except Exception:
            log.exception("Kamera ließ sich nicht sauber schließen")
        finally:
            self._cam = None

    def read(self) -> Frame | None:
        if self._cam is None:
            return None
        import io

        buf = io.BytesIO()
        self._cam.capture_file(buf, format="jpeg")
        return buf.getvalue(), "image/jpeg"


# --- USB-Kamera -------------------------------------------------------------------


class OpenCVCamera(CameraBase):
    name = "usb"

    def __init__(self, index: int | None = None) -> None:
        super().__init__()
        self._index = config.CAMERA_INDEX if index is None else index
        self._cap = None

    def _open(self) -> None:
        import cv2

        self._cap = cv2.VideoCapture(self._index)
        if not self._cap.isOpened():
            self._cap = None
            raise RuntimeError(f"USB-Kamera {self._index} lässt sich nicht öffnen")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        log.info("Kamera (USB, Index %d) bereit", self._index)

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
            log.info("%s nicht nutzbar: %s", kandidat.__name__, exc)
            # Auch nach einem Fehlschlag aufräumen: ein halb geöffnetes
            # Gerät würde den nächsten Kandidaten blockieren.
            try:
                kamera.close()
            except Exception:
                pass

    log.warning("Keine Kamera gefunden -- es wird ein Testbild angezeigt")
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
        except Exception:
            log.exception("Kamera lässt sich nicht öffnen -- kein Bild")
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

    def _loop(self) -> None:
        interval = 1.0 / max(1, config.CAMERA_FPS)
        fehler = 0

        while not self._stop.is_set():
            begin = time.monotonic()
            try:
                frame = self._camera.read()
                if frame is not None:
                    with self._lock:
                        self._latest = frame
                    fehler = 0
            except Exception:
                fehler += 1
                # Einzelne Aussetzer sind normal; erst bei einer Serie
                # lohnt eine Meldung, sonst flutet es das Log.
                if fehler in (1, 50):
                    log.exception("Kamera liefert kein Bild (%d. Versuch)", fehler)

            rest = interval - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)
