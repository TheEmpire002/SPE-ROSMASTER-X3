"""Kamerabilder für die Web-App und die Bildauswertung.

Es laufen alle Kameras, die gefunden werden, jede mit eigenem
Hintergrund-Thread. Drei Varianten derselben Schnittstelle:

1. ``Picamera2``  -- Kamera am CSI-Anschluss des Pi
2. ``OpenCV``     -- USB-Kamera
3. ``Mock``       -- erzeugtes Testbild, damit die App auch ohne Kamera
                     entwickelt werden kann

Welche USB-Kamera wofür da ist (Personenerkennung, Orientierung), steht in
``config.CAMERA_ROLES``. Zugeordnet wird über die USB-Kennung, nicht über
``/dev/videoN``: die Nummern vertauschen sich nach einem Neustart oder
Neu-Einstecken.

Ausgeliefert wird als MJPEG: eine endlose Folge einzelner Bilder in einer
HTTP-Antwort. Das kann jeder Browser ohne JavaScript und ohne zusätzliche
Bibliothek direkt in einem ``<img>`` anzeigen -- deutlich weniger Aufwand
als WebRTC, und für ein Kamerabild im eigenen WLAN völlig ausreichend.
"""

from __future__ import annotations

import io
import logging
import math
import os
import struct
import threading
import time
import zlib
from abc import ABC, abstractmethod
from dataclasses import dataclass

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
    #: Horizontaler Bildwinkel in Grad -- ordnet Bildpositionen dem Lidar zu.
    fov_deg: float = config.CAMERA_FOV_DEG

    def __init__(
        self,
        name: str | None = None,
        label: str | None = None,
        fov_deg: float | None = None,
    ) -> None:
        self._opened = False
        # Je Instanz überschreibbar: bei mehreren Kameras braucht jede einen
        # eigenen Namen, sonst sind Endpunkte und Logmeldungen nicht
        # auseinanderzuhalten.
        if name is not None:
            self.name = name
        if label is not None:
            self.label = label
        if fov_deg is not None:
            self.fov_deg = fov_deg

    @property
    def geraet(self) -> str | None:
        """Welches Gerät gerade benutzt wird -- für /camera/status."""
        return None

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

    def __init__(self, nummer: int = 0, **kwargs) -> None:
        super().__init__(**kwargs)
        self._nummer = nummer
        self._cam = None
        self._sink: _JpegSink | None = None

    @property
    def geraet(self) -> str | None:
        return f"CSI {self._nummer}"

    def _open(self) -> None:
        # Alles nur auf dem Pi vorhanden.
        from picamera2 import Picamera2
        from picamera2.encoders import JpegEncoder
        from picamera2.outputs import FileOutput

        self._cam = Picamera2(self._nummer)
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

        log.info("Kamera '%s' (CSI %d) bereit", self.name, self._nummer)

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


def _lesen(pfad: str) -> str:
    try:
        with open(pfad) as f:
            return f.read().strip()
    except OSError:
        return ""


def _geraetename(idx: int) -> str:
    """Was der Kernel zu /dev/videoN sagt, z.B. 'USB 2.0 Camera'."""
    return _lesen(f"/sys/class/video4linux/video{idx}/name")


@dataclass(frozen=True)
class UsbGeraet:
    """Eine angeschlossene USB-Kamera, wie sie der Kernel gerade sieht."""

    index: int       # /dev/videoN -- ändert sich nach Neu-Einstecken
    usb_id: str      # "046d:0825", wie in lsusb
    anschluss: str   # USB-Port, z.B. "1-1.1.2" -- fest, solange sie dort steckt
    name: str        # z.B. "Logi C270 HD WebCam"


def usb_kameras() -> list[UsbGeraet]:
    """Alle USB-Kameras, die gerade Bilder liefern können.

    Ein Pi 5 bringt rund zwanzig ``/dev/video*`` mit, von denen die meisten
    Codec- und Bildverarbeitungs-Einheiten sind -- die werden aussortiert,
    ebenso alles ohne USB-Kennung.
    """
    import glob
    import re

    gefunden: list[UsbGeraet] = []
    for pfad in glob.glob("/sys/class/video4linux/video*"):
        treffer = re.search(r"video(\d+)$", pfad)
        if not treffer:
            continue
        idx = int(treffer.group(1))
        name = _geraetename(idx)
        if any(wort in name.lower() for wort in KEINE_KAMERA):
            continue
        # UVC legt je Kamera zwei Knoten an: index 0 liefert Bilder, index 1
        # nur Metadaten. Der zweite lässt sich öffnen, liefert aber nie ein
        # Bild -- und sähe sonst aus wie eine zweite, defekte Kamera.
        if _lesen(f"{pfad}/index") not in ("", "0"):
            continue
        # "device" zeigt auf die USB-Schnittstelle (z.B. 1-1.1.2:1.0);
        # Hersteller und Produkt stehen eine Ebene höher beim Gerät selbst.
        usb = os.path.dirname(os.path.realpath(f"{pfad}/device"))
        hersteller = _lesen(f"{usb}/idVendor")
        if not hersteller:
            continue
        produkt = _lesen(f"{usb}/idProduct")
        gefunden.append(
            UsbGeraet(idx, f"{hersteller}:{produkt}".lower(), os.path.basename(usb), name)
        )
    return sorted(gefunden, key=lambda g: g.index)


class OpenCVCamera(CameraBase):
    """USB-Kamera, gefunden über USB-Kennung und/oder Anschluss.

    Ohne beides nimmt sie die erste USB-Kamera, die sich öffnen lässt.
    """

    name = "usb"
    label = "USB-Kamera"

    def __init__(
        self, usb_id: str | None = None, anschluss: str | None = None, **kwargs
    ) -> None:
        super().__init__(**kwargs)
        self._usb_id = usb_id.lower() if usb_id else None
        self._anschluss = anschluss
        self._index: int | None = None
        self._cap = None

    @property
    def geraet(self) -> str | None:
        return f"/dev/video{self._index}" if self._index is not None else None

    def passt(self, geraet: UsbGeraet) -> bool:
        if self._usb_id and not geraet.usb_id.startswith(self._usb_id):
            return False
        if self._anschluss and geraet.anschluss != self._anschluss:
            return False
        return True

    def _gesucht(self) -> str:
        teile = []
        if self._usb_id:
            teile.append(f"USB {self._usb_id}")
        if self._anschluss:
            teile.append(f"Anschluss {self._anschluss}")
        return ", ".join(teile) or "beliebige USB-Kamera"

    def _open(self) -> None:
        import cv2

        # Bei jedem Öffnen neu suchen: nach einem USB-Aussetzer bekommt die
        # Kamera oft eine andere /dev/videoN-Nummer.
        kandidaten = [g for g in usb_kameras() if self.passt(g)]
        if not kandidaten:
            raise RuntimeError(f"nicht angeschlossen ({self._gesucht()})")

        versucht: list[str] = []
        for geraet in kandidaten:
            idx = geraet.index
            cap, api = self._oeffnen(cv2, idx)
            if cap is None:
                versucht.append(f"video{idx} ({geraet.name}): öffnet nicht")
                continue

            gewaehlt = self._einstellung_finden(cv2, cap, idx, versucht)
            if gewaehlt is None:
                cap.release()
                continue

            self._cap = cap
            self._index = idx
            log.info(
                "Kamera '%s' (USB %s, /dev/video%d, %s) bereit -- %s, %s",
                self.name,
                geraet.usb_id,
                idx,
                geraet.name,
                api,
                gewaehlt,
            )
            return

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
            # Die Kamera läuft immer mit voller Rate; gedrosselt wird beim
            # Abholen (CAMERA_FPS). So wirkt eine geänderte Bildrate sofort,
            # ohne die Kamera neu zu öffnen.
            cap.set(cv2.CAP_PROP_FPS, config.CAMERA_MAX_FPS)
            # Nur ein Bild puffern: sonst liefert read() bei gedrosseltem
            # Abholen das älteste von mehreren, und das Bild hinkt hinterher.
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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

    def __init__(self, farbe: tuple[int, int, int] = (0x4D, 0xA3, 0xFF), **kwargs) -> None:
        super().__init__(**kwargs)
        # Bei mehreren Testbildern eine eigene Farbe je Kamera -- sonst sieht
        # man in der App nicht, ob das Umschalten überhaupt etwas tut.
        self._farbe = farbe
        self._frame = 0

    def _open(self) -> None:
        log.info("Kamera '%s': Testbild (keine echte Kamera)", self.name)

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
                    row[i], row[i + 1], row[i + 2] = self._farbe
                else:
                    grau = 0x22 + (y * 24 // self.HEIGHT)
                    row[i] = row[i + 1] = row[i + 2] = grau
            rows.append(row)

        return _png(self.WIDTH, self.HEIGHT, rows), "image/png"


# --- Auswahl und Hintergrundbetrieb ---------------------------------------------------


#: Balkenfarben der Testbilder, eine je Kamera.
_TESTFARBEN = [(0x4D, 0xA3, 0xFF), (0x3F, 0xD0, 0x7A), (0xF5, 0xA5, 0x24)]


def _csi_nummern() -> list[int]:
    """Nummern der Kameras am CSI-Anschluss, leer ohne picamera2.

    Nur echte CSI-Kameras: libcamera listet auf dem Pi auch USB-Kameras
    auf. Würde picamera2 eine davon belegen, bekäme OpenCV sie nicht mehr
    -- "Device or resource busy", und die Rolle bliebe ohne Bild.
    """
    try:
        from picamera2 import Picamera2

        infos = Picamera2.global_camera_info()
    except Exception:
        return []
    return [
        info.get("Num", n)
        for n, info in enumerate(infos)
        if "usb" not in str(info.get("Id", "")).lower()
    ]


def create_cameras(use_mock: bool = False) -> dict[str, CameraBase]:
    """Alle Kameras, nach Namen. Geöffnet wird erst in ``CameraService``.

    Die Rollen aus ``config.CAMERA_ROLES`` sind immer dabei, auch wenn ihre
    Kamera gerade fehlt: sie wird dann im Hintergrund weiter gesucht. Ein
    USB-Aussetzer beim Start soll die Personenerkennung nicht bis zum
    nächsten Neustart abschalten.
    """
    rollen = config.CAMERA_ROLES

    if use_mock:
        # Je Rolle ein eigenes Testbild, damit sich die App mit mehreren
        # Kameras auch ohne Roboter entwickeln lässt.
        namen = list(rollen) or ["kamera"]
        return {
            name: MockCamera(
                farbe=_TESTFARBEN[i % len(_TESTFARBEN)],
                name=name,
                label=f"Testbild ({rollen[name]['label']})" if name in rollen else "Testbild",
                fov_deg=rollen.get(name, {}).get("fov_deg"),
            )
            for i, name in enumerate(namen)
        }

    kameras: dict[str, CameraBase] = {}
    gefunden = usb_kameras()
    vergeben: set[str] = set()

    for name, rolle in rollen.items():
        kamera = OpenCVCamera(
            usb_id=rolle["usb_id"],
            name=name,
            label=rolle.get("label"),
            fov_deg=rolle.get("fov_deg"),
        )
        kameras[name] = kamera
        treffer = next(
            (g for g in gefunden if kamera.passt(g) and g.anschluss not in vergeben), None
        )
        if treffer is None:
            log.warning(
                "Kamera '%s' (%s, USB %s) nicht angeschlossen -- wird weiter gesucht",
                name, kamera.label, rolle["usb_id"],
            )
        else:
            vergeben.add(treffer.anschluss)

    # Alles ohne Rolle läuft trotzdem mit -- fest an seinem USB-Port, damit
    # es nach einem Aussetzer nicht einer Rolle ihre Kamera wegschnappt.
    for nr, geraet in enumerate((g for g in gefunden if g.anschluss not in vergeben), 1):
        kameras[f"usb{nr}"] = OpenCVCamera(
            anschluss=geraet.anschluss, name=f"usb{nr}", label=geraet.name
        )

    for nr in _csi_nummern():
        name = "csi" if nr == 0 else f"csi{nr}"
        kameras[name] = Picamera2Camera(nr, name=name)

    if not kameras:
        log.warning("Keine Kamera gefunden -- es wird ein Testbild angezeigt")
        log.warning("Ursache finden: python scripts/check_camera.py")
        kameras["kamera"] = MockCamera(name="kamera")

    log.info("Kameras: %s", ", ".join(f"{n} ({k.label})" for n, k in kameras.items()))
    return kameras


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
        # Tatsächlich erreichte Bildrate, geglättet. Zeigt, was Kamera und
        # Pi wirklich schaffen -- CAMERA_FPS ist nur der Wunsch.
        self._fps_ist = 0.0
        self._neu_oeffnen_angefordert = threading.Event()

    def neu_oeffnen(self) -> None:
        """Kamera beim nächsten Durchlauf neu öffnen, z. B. für eine neue Auflösung.

        Nur anfordern statt selbst öffnen: das Öffnen dauert und gehört in
        den Kamera-Thread, nicht in den Aufrufer (meist die Ereignisschleife
        des Servers).
        """
        self._neu_oeffnen_angefordert.set()

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
            "name": self._camera.name,
            "geraet": self._camera.geraet,
            "fov_deg": self._camera.fov_deg,
            "fps_ist": round(self._fps_ist, 1) if laeuft and self._camera.opened else 0.0,
            "fps_soll": config.CAMERA_FPS,
        }

    @property
    def name(self) -> str:
        return self._camera.name

    @property
    def label(self) -> str:
        return self._camera.label

    @property
    def fov_deg(self) -> float:
        return self._camera.fov_deg

    def start(self) -> None:
        if self._thread is not None:
            return
        try:
            # open() ist absichtlich wiederholbar und tut nichts, wenn die
            # Kamera schon offen ist.
            self._camera.open()
        except Exception as exc:
            log.warning("Kamera '%s' lässt sich nicht öffnen -- %s", self.name, exc)
            with self._lock:
                self._letzter_fehler = f"Öffnen fehlgeschlagen: {exc}"
            # Der Thread startet trotzdem: er öffnet die Kamera in größer
            # werdenden Abständen neu. So kommt eine Kamera, die beim Start
            # fehlte oder belegt war, ohne Neustart des Servers dazu.

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name=f"kamera-{self.name}", daemon=True
        )
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

    def _neu_oeffnen(self, grund: str = "liefert nichts") -> None:
        log.warning("Kamera '%s' %s -- öffne sie neu", self.name, grund)
        try:
            self._camera.close()
        except Exception:
            log.debug("Kamera schließen misslungen", exc_info=True)
        try:
            self._camera.open()
        except Exception as exc:
            with self._lock:
                self._letzter_fehler = f"Neu öffnen: {type(exc).__name__}: {exc}"
            log.warning("Kamera '%s' lässt sich noch nicht öffnen -- %s", self.name, exc)
            return
        self._neu_geoeffnet += 1
        log.warning("Kamera '%s' geöffnet", self.name)

    def _loop(self) -> None:
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
                        # Bei einer geschlossenen Kamera steht der Grund
                        # (z.B. "nicht angeschlossen") schon da -- nicht
                        # mit einem nichtssagenden "None" überschreiben.
                        if self._bilder == 0 and self._camera.opened:
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
                        abstand = begin - letzte_aenderung
                        if 0 < abstand < 2.0:
                            self._fps_ist += 0.1 * (1.0 / abstand - self._fps_ist)
                        letzte_aenderung = begin
                        reopen_wartezeit = config.CAMERA_REOPEN_S
                        if eingefroren_gemeldet:
                            log.info("Kamera '%s' liefert wieder Bilder", self.name)
                            eingefroren_gemeldet = False
            except Exception as exc:
                fehler += 1
                with self._lock:
                    self._letzter_fehler = f"{type(exc).__name__}: {exc}"
                # Einzelne Aussetzer sind normal. Die ersten paar trotzdem
                # melden -- sonst sucht man bei einem stehenden Bild lange.
                if fehler <= 3 or fehler % 100 == 0:
                    log.exception(
                        "Kamera '%s' liefert kein Bild (%d. Versuch)", self.name, fehler
                    )

            # Ein stehendes Bild sieht im Browser aus wie ein laufender
            # Stream. Ohne diese Meldung merkt man den Stillstand nicht.
            # Eine gar nicht geöffnete Kamera hat ihre eigene Meldung.
            if (
                not eingefroren_gemeldet
                and self._camera.opened
                and begin - letzte_aenderung > 5.0
            ):
                log.warning(
                    "Kamera '%s' seit %.0f s ohne neues Bild -- Stream steht",
                    self.name,
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
            elif self._neu_oeffnen_angefordert.is_set():
                self._neu_oeffnen_angefordert.clear()
                self._neu_oeffnen("hat neue Einstellungen")
                letzte_aenderung = time.monotonic()

            # Jedes Mal neu lesen: die Bildrate ist zur Laufzeit verstellbar.
            interval = 1.0 / max(1, config.CAMERA_FPS)
            rest = interval - (time.monotonic() - begin)
            if rest > 0:
                self._stop.wait(rest)
