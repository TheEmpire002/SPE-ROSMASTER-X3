"""Diagnose -- sammelt alles, was am X3 schiefgehen kann, an einer Stelle.

Bisher musste man bei jedem Aussetzer per SSH ins Kernel-Log, in
``vcgencmd`` und in die Serverausgabe schauen und die Zeitpunkte selbst
zusammenpuzzeln. Dieses Modul tut das laufend im Hintergrund und macht
daraus *Probleme* in Alltagssprache: was ist passiert, was bedeutet es,
was soll man tun. Die Seite ``/diagnose`` zeigt sie an.

Zwei Arten von Problemen:

- **Zustände** bestehen, solange die Bedingung gilt (Akku schwach, Lidar
  weg, Pi zu warm).
- **Vorfälle** sind einmalige Ereignisse (USB-Überlast, Abmeldung). Sie
  bleiben ``config.PROBLEM_FENSTER_S`` lang sichtbar.

Wer ein Problem gesehen hat, quittiert es. Tritt es danach erneut auf,
erscheint es wieder -- so geht nichts unter, ohne dass bekannte Dinge
ständig rot leuchten.

Alles hier ist "best effort": fehlt eine Quelle (Windows, kein
``vcgencmd``, Kernel-Log nicht lesbar, Simulation), steht dort eben
"unbekannt" -- abstürzen darf die Diagnose nie, sie läuft ja gerade dann,
wenn sonst etwas kaputt ist.
"""

from __future__ import annotations

import glob
import logging
import os
import shutil
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from . import config

log = logging.getLogger(__name__)

#: Rangfolge für Sortierung und Gesamtzustand.
_RANG = {"kritisch": 0, "warnung": 1, "hinweis": 2}

#: Prozesse, die auf dem Pi große Last erzeugen. Am 28.10. hing ein
#: ``dpkg``/``dkms``-Build, heizte den Pi auf 83 °C und löste eine
#: USB-Überlast nach der anderen aus. /proc/<pid>/comm ist auf 15 Zeichen
#: gekürzt, daher "unattended-upgr".
_BAU_PROZESSE = {"cc1", "cc1plus", "dkms", "dpkg", "apt", "apt-get", "make", "unattended-upgr"}

#: Bits von ``vcgencmd get_throttled`` (0-3 jetzt, 16-19 seit dem Start).
_DROSSEL_BITS = {
    0: "Unterspannung",
    1: "Takt begrenzt",
    2: "gedrosselt",
    3: "Temperaturgrenze",
}

_USB_TEXTE = [
    "Akku laden -- unter etwa 11,5 V passiert das deutlich häufiger.",
    "Orbbec-Kamera abziehen, wenn sie nicht gebraucht wird; sie zieht am meisten Strom.",
    "Dauerhafte Lösung: USB-Hub mit eigener Stromversorgung bzw. eigener 5-V-Wandler "
    "für den Pi (siehe docs/hardware.md).",
]


# --- Log-Puffer -------------------------------------------------------------------


class LogPuffer(logging.Handler):
    """Hält die letzten Log-Zeilen des Servers für die Diagnoseseite.

    Ein normaler Handler am Logger "rosmaster_assist": die Konsole bekommt
    weiterhin alles, die Seite zusätzlich. Jede Zeile hat eine fortlaufende
    Nummer, damit der Browser nur Neues abholt.
    """

    def __init__(self, groesse: int) -> None:
        super().__init__(level=logging.INFO)
        self._zeilen: deque[dict] = deque(maxlen=groesse)
        self._naechste = 1
        self._fehler = 0
        self._letzter_fehler: dict | None = None
        self._puffer_lock = threading.Lock()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = record.getMessage()
            if record.exc_info and record.exc_info[1] is not None:
                exc = record.exc_info[1]
                text += f" -- {type(exc).__name__}: {exc}"
            with self._puffer_lock:
                zeile = {
                    "id": self._naechste,
                    "zeit": record.created,
                    "stufe": record.levelname,
                    "quelle": record.name.removeprefix("rosmaster_assist."),
                    "text": text,
                }
                self._naechste += 1
                self._zeilen.append(zeile)
                if record.levelno >= logging.ERROR:
                    self._fehler += 1
                    self._letzter_fehler = zeile
        except Exception:
            # Ein Fehler beim Protokollieren darf nie den Aufrufer treffen.
            self.handleError(record)

    def zeilen(self, nach: int = 0) -> tuple[list[dict], int]:
        """Zeilen nach Nummer ``nach`` und die Nummer der letzten Zeile.

        Die letzte Nummer kommt immer vom Puffer, nie vom Aufrufer: ist sie
        kleiner als das, was der Browser schon hat, wurde der Server neu
        gestartet und der Browser fängt von vorn an.
        """
        with self._puffer_lock:
            return [z for z in self._zeilen if z["id"] > nach], self._naechste - 1

    def fehler(self) -> tuple[int, dict | None]:
        with self._puffer_lock:
            return self._fehler, self._letzter_fehler


# --- Probleme ---------------------------------------------------------------------


@dataclass
class Problem:
    id: str
    schwere: str
    titel: str
    was: str
    tun: list[str]
    seit: float
    zuletzt: float
    anzahl: int = 1
    #: Zustand, der gerade noch besteht (bei Vorfällen nie).
    besteht: bool = False
    #: True bei Bedingungen (Akku, Lidar ...), False bei einmaligen Vorfällen.
    zustand: bool = False
    #: Zeitpunkt des letzten *neuen* Auftretens -- Grundlage fürs Quittieren.
    vorfall: float = 0.0
    quittiert: float | None = None
    extra: dict = field(default_factory=dict)

    def als_dict(self, jetzt: float) -> dict:
        return {
            "id": self.id,
            "schwere": self.schwere,
            "titel": self.titel,
            "was": self.was,
            "tun": self.tun,
            "seit": self.seit,
            "zuletzt": self.zuletzt,
            "anzahl": self.anzahl,
            "besteht": self.besteht,
            "zustand": self.zustand,
            "quittiert": self.ist_quittiert(),
            "vor_s": round(jetzt - self.zuletzt, 1),
        }

    def ist_quittiert(self) -> bool:
        return self.quittiert is not None and self.vorfall <= self.quittiert


# --- Systemwerte (alle liefern None, wenn nicht verfügbar) ----------------------------


def _lies(pfad: str) -> str | None:
    try:
        with open(pfad, encoding="ascii", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return None


def usb_ueberlast_zaehler() -> int | None:
    """Summe der Überlast-Zähler aller USB-Ports seit dem Booten.

    Liest ohne root-Rechte. Am Pi 5 hängen alle vier Buchsen an einem
    gemeinsamen 1,6-A-Schalter -- jede Überlast setzt alles zurück.
    """
    dateien = glob.glob("/sys/bus/usb/devices/*-0:1.0/usb*-port*/over_current_count")
    if not dateien:
        return None
    summe = 0
    for pfad in dateien:
        wert = _lies(pfad)
        if wert and wert.isdigit():
            summe += int(wert)
    return summe


def cpu_temperatur() -> float | None:
    wert = _lies("/sys/class/thermal/thermal_zone0/temp")
    try:
        return round(int(wert) / 1000.0, 1) if wert else None
    except ValueError:
        return None


def cpu_max_mhz() -> int | None:
    wert = _lies("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq")
    try:
        return int(wert) // 1000 if wert else None
    except ValueError:
        return None


def cpu_ist_mhz() -> int | None:
    wert = _lies("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    try:
        return int(wert) // 1000 if wert else None
    except ValueError:
        return None


def pi_laufzeit_s() -> float | None:
    wert = _lies("/proc/uptime")
    try:
        return float(wert.split()[0]) if wert else None
    except (ValueError, IndexError):
        return None


def drosselung() -> int | None:
    """Rohwert von ``vcgencmd get_throttled`` oder None."""
    if shutil.which("vcgencmd") is None:
        return None
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2
        )
        return int(out.stdout.strip().split("=", 1)[-1], 16)
    except (subprocess.SubprocessError, OSError, ValueError):
        return None


def drossel_text(roh: int | None) -> dict | None:
    if roh is None:
        return None
    return {
        "roh": hex(roh),
        "jetzt": [t for b, t in _DROSSEL_BITS.items() if roh & (1 << b)],
        "seit_start": [t for b, t in _DROSSEL_BITS.items() if roh & (1 << (b + 16))],
    }


def bau_prozesse() -> list[str]:
    """Namen laufender Build-/Installationsprozesse (leer, wenn keine)."""
    gefunden = set()
    for pfad in glob.glob("/proc/[0-9]*/comm"):
        name = _lies(pfad)
        if name in _BAU_PROZESSE:
            gefunden.add(name)
    return sorted(gefunden)


def akku_prozent(volt: float | None) -> int | None:
    """Grobe Füllstandsschätzung für den 3S-Akku (10,5 V leer, 12,6 V voll).

    Nur eine Hausnummer: die Spannung sackt unter Last ab und erholt sich
    im Stand -- deshalb zeigt die Seite die Volt immer mit an.
    """
    if volt is None:
        return None
    return int(max(0.0, min(1.0, (volt - 10.5) / (12.6 - 10.5))) * 100)


# --- Diagnose ---------------------------------------------------------------------


class Diagnose:
    """Hintergrund-Thread, der Messwerte, Ereignisse und Probleme pflegt."""

    def __init__(
        self,
        driver,
        watchdog,
        lidar=None,
        kameras: dict | None = None,
        pose=None,
        log_puffer: LogPuffer | None = None,
        mock: bool = False,
    ) -> None:
        self._driver = driver
        self._watchdog = watchdog
        self._lidar = lidar
        self._kameras = kameras or {}
        self._pose = pose
        self._log = log_puffer
        self._mock = mock

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._gestartet = time.time()

        self._probleme: dict[str, Problem] = {}
        self._ereignisse: deque[dict] = deque(maxlen=config.DIAGNOSE_EREIGNISSE)
        self._ereignis_nr = 1
        self._akku: deque[tuple[float, float]] = deque(maxlen=config.DIAGNOSE_AKKU_PUNKTE)
        self._mess: dict[str, Any] = {}

        # Vergleichswerte vom letzten Durchlauf, um Änderungen zu erkennen.
        self._oc_vorher: int | None = None
        self._oc_start: int | None = None
        self._verbunden_vorher: bool | None = None
        self._lidar_neu_vorher: int | None = None
        self._kamera_neu_vorher: dict[str, int] = {}
        self._fehler_vorher = 0
        self._drossel_vorher = 0
        self._zuletzt_gefahren = 0.0
        self._zuletzt_gedreht = 0.0
        self._einbruch_zuletzt = 0.0
        self._kernel_seit: float | None = None
        self._kernel_quelle = "unbekannt"
        self._langsam_zuletzt = 0.0
        self._start_mono = time.monotonic()

    # --- Lebenszyklus -------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._start_mono = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="diagnose", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # Der langsame Durchlauf wartet höchstens auf journalctl (Timeout 3 s).
            self._thread.join(timeout=4.0)
            self._thread = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._schnell()
                if time.monotonic() - self._langsam_zuletzt >= config.DIAGNOSE_LANGSAM_S:
                    self._langsam_zuletzt = time.monotonic()
                    self._langsam()
            except Exception:
                # Nur Warnung, nicht ERROR: sonst meldete die Diagnose ihren
                # eigenen Absturz als "Fehler im Log" -- bei jedem Durchlauf.
                log.warning("Diagnose-Durchlauf fehlgeschlagen", exc_info=True)
            self._stop.wait(config.DIAGNOSE_INTERVAL_S)

    # --- Probleme pflegen ---------------------------------------------------------

    def _vorfall(
        self, pid: str, schwere: str, titel: str, was: str, tun: list[str],
        zeit: float | None = None, anzahl: int = 1,
    ) -> None:
        """Einmaliges Ereignis -- zählt hoch und macht das Problem wieder sichtbar."""
        zeit = zeit or time.time()
        with self._lock:
            p = self._probleme.get(pid)
            if p is None or zeit - p.zuletzt > config.PROBLEM_FENSTER_S:
                self._probleme[pid] = Problem(
                    pid, schwere, titel, was, tun, seit=zeit, zuletzt=zeit,
                    anzahl=anzahl, vorfall=zeit, quittiert=p.quittiert if p else None,
                )
                return
            p.schwere, p.titel, p.was, p.tun = schwere, titel, was, tun
            p.zuletzt = max(p.zuletzt, zeit)
            p.vorfall = max(p.vorfall, zeit)
            p.anzahl += anzahl

    def _zustand(
        self, pid: str, aktiv: bool, schwere: str = "warnung", titel: str = "",
        was: str = "", tun: list[str] | None = None,
    ) -> None:
        """Bedingung, die bestehen kann oder nicht. Nur der Beginn zählt als neu."""
        jetzt = time.time()
        with self._lock:
            p = self._probleme.get(pid)
            if not aktiv:
                if p is not None:
                    p.besteht = False
                return
            if p is None or (not p.besteht and jetzt - p.zuletzt > config.PROBLEM_FENSTER_S):
                self._probleme[pid] = Problem(
                    pid, schwere, titel, was, tun or [], seit=jetzt, zuletzt=jetzt,
                    besteht=True, zustand=True, vorfall=jetzt,
                    quittiert=p.quittiert if p else None,
                )
                return
            if not p.besteht:
                p.anzahl += 1
                p.vorfall = jetzt
            p.besteht = True
            p.zuletzt = jetzt
            p.schwere, p.titel, p.was = schwere, titel, was
            p.tun = tun or []

    def _ereignis(self, art: str, schwere: str, text: str, zeit: float | None = None) -> None:
        with self._lock:
            self._ereignisse.append({
                "id": self._ereignis_nr,
                "zeit": zeit or time.time(),
                "art": art,
                "schwere": schwere,
                "text": text,
            })
            self._ereignis_nr += 1
        # Auch ins Log: wer nur die Konsole sieht, soll es ebenfalls merken.
        if schwere != "info":
            log.warning("Diagnose: %s", text)

    def quittieren(self, pid: str | None = None) -> int:
        """Markiert ein Problem (oder alle bei ``None``) als gesehen."""
        jetzt = time.time()
        n = 0
        with self._lock:
            for p in self._probleme.values():
                if pid is None or p.id == pid:
                    p.quittiert = jetzt
                    n += 1
        return n

    # --- Schnelle Messungen (jede Sekunde) -------------------------------------------

    def _schnell(self) -> None:
        jetzt = time.time()
        mono = time.monotonic()

        cmd = self._watchdog.current_command
        if abs(cmd.vx) > 0.01 or abs(cmd.vy) > 0.01 or abs(cmd.omega) > 0.01:
            self._zuletzt_gefahren = mono
        if abs(cmd.omega) > 0.05:
            self._zuletzt_gedreht = mono

        self._akku_pruefen(jetzt)
        self._usb_pruefen(mono)
        self._antrieb_pruefen()
        self._lidar_pruefen()
        self._kameras_pruefen()
        self._pose_pruefen()
        self._log_pruefen()

    def _akku_pruefen(self, jetzt: float) -> None:
        try:
            volt = self._driver.read_battery()
        except Exception:
            volt = None
        self._mess["akku_v"] = volt
        self._mess["akku_prozent"] = akku_prozent(volt)
        if volt is None:
            self._zustand("akku_kritisch", False)
            self._zustand("akku_schwach", False)
            return

        with self._lock:
            self._akku.append((jetzt, volt))
            letzte_minute = [v for t, v in self._akku if jetzt - t <= 60.0]
        hoch = max(letzte_minute)

        kritisch = volt < config.AKKU_KRITISCH_V
        self._zustand(
            "akku_kritisch", kritisch, "kritisch", f"Akku fast leer ({volt:.1f} V)",
            "Die Spannung liegt unter der kritischen Schwelle. Das Board und die "
            "USB-Geräte melden sich jetzt ständig ab, und der Akku nimmt beim "
            "Weiterentladen Schaden.",
            ["Roboter anhalten und ausschalten.", "Akku laden (voll = 12,6 V).",
             "Zum Testen am Schreibtisch das Netzteil statt des Akkus verwenden."],
        )
        self._zustand(
            "akku_schwach", not kritisch and volt < config.AKKU_WARN_V, "warnung",
            f"Akku schwach ({volt:.1f} V)",
            "Mit schwachem Akku bricht beim Anfahren der Motoren die Spannung ein. "
            "Dann setzen sich Board, Lidar und Kamera zurück -- der Roboter bleibt "
            "stehen oder fährt ruckelig.",
            ["Bald laden.", "Bis dahin langsam fahren und abrupte Drehungen vermeiden."],
        )

        # Einbruch gegenüber dem Höchstwert der letzten Minute. Höchstens
        # alle 10 s melden: ein Einbruch dauert oft mehrere Messungen.
        if hoch - volt >= config.SPANNUNGSEINBRUCH_V and jetzt - self._einbruch_zuletzt > 10.0:
            self._einbruch_zuletzt = jetzt
            text = f"Spannungseinbruch: {hoch:.1f} V → {volt:.1f} V"
            if time.monotonic() - self._zuletzt_gefahren < 3.0:
                text += " (beim Fahren)"
            self._ereignis("spannung", "warnung", text)
            self._vorfall(
                "spannungseinbruch", "warnung", "Spannungseinbruch am Akku",
                f"Die Akkuspannung ist kurz um {hoch - volt:.1f} V gefallen. Meist "
                "laufen dabei die Motoren an; bei schwachem oder kaltem Akku fällt "
                "die Spannung stärker ab.",
                ["Akku laden bzw. wechseln.",
                 "Prüfen, ob gleichzeitig USB-Geräte ausgefallen sind (Zeitleiste)."],
            )

    def _usb_pruefen(self, mono: float) -> None:
        oc = usb_ueberlast_zaehler()
        self._mess["usb_ueberlast_gesamt"] = oc
        if oc is None:
            return
        if self._oc_start is None:
            self._oc_start = oc
            self._oc_vorher = oc
            if oc > 0:
                # Schon vor dem Serverstart passiert, z. B. beim Booten.
                self._ereignis("usb", "hinweis", f"Seit dem Booten bereits {oc} USB-Überlast(en)")
        self._mess["usb_ueberlast_seit_serverstart"] = oc - self._oc_start
        neu = oc - (self._oc_vorher or 0)
        self._oc_vorher = oc
        if neu <= 0:
            return

        gedreht = mono - self._zuletzt_gedreht < 5.0
        gefahren = mono - self._zuletzt_gefahren < 5.0
        if gedreht:
            anlass = " kurz nach einer Drehung"
        elif gefahren:
            anlass = " beim Fahren"
        else:
            anlass = " im Stand"
        self._ereignis("usb", "warnung", f"USB-Überlast (+{neu}){anlass}")

        was = (
            "Der Pi 5 versorgt alle vier USB-Buchsen über einen gemeinsamen "
            "1,6-A-Schalter. Zieht ein Gerät zu viel Strom oder bricht die "
            "5-V-Versorgung ein, schaltet er kurz ab -- Board, Lidar und Kamera "
            "melden sich dann ab und wieder an."
        )
        tun = list(_USB_TEXTE)
        if gedreht or gefahren:
            was += (
                " Diesmal kurz nach einer Fahrbewegung: typisch für den Anlaufstrom "
                "der Motoren, der die Versorgung kurz in die Knie zwingt."
            )
            tun.insert(1, "Sanfter anfahren (geringeres Höchsttempo in den Einstellungen).")
        else:
            tun.insert(0, "Prüfen, ob im Hintergrund etwas Rechenintensives läuft "
                          "(Installation, Build) -- siehe Messwerte.")
        self._vorfall("usb_ueberlast", "warnung", "USB-Überlast: Buchsen kurz abgeschaltet",
                      was, tun, anzahl=neu)

    def _im_anlauf(self) -> bool:
        """Kurz nach dem Start liefern Sensoren noch nichts -- kein Fehlalarm."""
        return time.monotonic() - self._start_mono < config.DIAGNOSE_ANLAUF_S

    def _antrieb_pruefen(self) -> None:
        verbunden = bool(self._driver.connected)
        if self._verbunden_vorher is not None and verbunden != self._verbunden_vorher:
            if verbunden:
                self._ereignis("antrieb", "info", "Antrieb wieder verbunden")
            else:
                self._ereignis("antrieb", "kritisch", "Verbindung zum Antrieb verloren")
        self._verbunden_vorher = verbunden
        self._zustand(
            "antrieb", not verbunden, "kritisch", "Keine Verbindung zum Antrieb",
            "Das Expansion Board (Motoren, Akku-Messung, IMU) antwortet nicht. "
            "Der Server versucht laufend, sich neu zu verbinden; solange fährt "
            "der Roboter nicht.",
            ["Ist der Roboter eingeschaltet (Schalter am Akku)?",
             "Akku prüfen -- häufigste Ursache.",
             "USB-Kabel zwischen Pi und Board kontrollieren.",
             "Läuft noch ein anderes Programm am Board (ROS2-Knoten)? "
             "Siehe AGENTS.md „Am Roboter“."],
        )

    def _lidar_pruefen(self) -> None:
        if self._lidar is None:
            return
        try:
            st = self._lidar.status()
        except Exception:
            return
        alt = st.get("letzte_runde_vor_s")
        liefert = bool(st.get("geoeffnet")) and alt is not None and alt < 3.0
        self._mess["lidar"] = st
        self._zustand(
            "lidar", not liefert and not self._im_anlauf(), "warnung", "Lidar liefert keine Daten",
            "Ohne Lidar gibt es keinen Hindernisstopp, kein autonomes Fahren und "
            "keine Ortung. Der Server versucht, ihn neu zu öffnen."
            + (f" Letzter Fehler: {st['letzter_fehler']}" if st.get("letzter_fehler") else ""),
            ["Dreht sich der Lidar-Kopf? Wenn nicht: Stromversorgung/USB prüfen.",
             "Läuft noch ein sllidar_node? `sudo pkill -f sllidar_node`.",
             "Kabel am Lidar-Adapter kontrollieren."],
        )
        neu = int(st.get("neu_verbunden") or 0)
        if self._lidar_neu_vorher is not None and neu > self._lidar_neu_vorher:
            self._ereignis("lidar", "warnung", "Lidar hat sich neu verbunden")
            self._vorfall(
                "lidar_neu", "hinweis", "Lidar musste neu verbunden werden",
                "Der Lidar war kurz weg und wurde automatisch neu geöffnet. Das "
                "passiert fast immer zusammen mit einer USB-Überlast.",
                ["Zeitleiste ansehen: kam gleichzeitig eine USB-Überlast?",
                 "Wenn ja: siehe Problem „USB-Überlast“."],
                anzahl=neu - self._lidar_neu_vorher,
            )
        self._lidar_neu_vorher = neu

    def _kameras_pruefen(self) -> None:
        kameras = {}
        for name, kamera in self._kameras.items():
            try:
                st = kamera.status()
            except Exception:
                continue
            kameras[name] = st
            alt = st.get("letztes_bild_vor_s")
            liefert = bool(st.get("geoeffnet")) and alt is not None and alt < 5.0
            self._zustand(
                f"kamera_{name}", not liefert and not self._im_anlauf(), "warnung",
                f"Kamera „{st.get('kamera') or name}“ liefert kein Bild",
                "Die Kamera ist nicht geöffnet oder schickt keine Bilder. Der Server "
                "versucht es weiter."
                + (f" Letzter Fehler: {st['letzter_fehler']}" if st.get("letzter_fehler") else ""),
                ["USB-Stecker der Kamera prüfen.",
                 "Wurde die Kamera absichtlich abgezogen (z. B. Orbbec zum Stromsparen)? "
                 "Dann kann diese Meldung quittiert werden.",
                 "Gerätepfad in den Einstellungen bzw. config.py (CAMERA_ROLES) prüfen."],
            )
            neu = int(st.get("neu_geoeffnet") or 0)
            vorher = self._kamera_neu_vorher.get(name)
            if vorher is not None and neu > vorher:
                self._ereignis("kamera", "warnung", f"Kamera „{name}“ neu geöffnet")
                self._vorfall(
                    f"kamera_neu_{name}", "hinweis", f"Kamera „{name}“ musste neu geöffnet werden",
                    "Die Kamera war kurz weg (meist USB-Rücksetzer) und läuft wieder.",
                    ["Zeitleiste ansehen: kam gleichzeitig eine USB-Überlast?"],
                    anzahl=neu - vorher,
                )
            self._kamera_neu_vorher[name] = neu
        self._mess["kameras"] = kameras

    def _pose_pruefen(self) -> None:
        if self._pose is None:
            return
        try:
            st = self._pose.status()
        except Exception:
            return
        self._mess["pose"] = st
        fehler = st.get("fehler")
        self._zustand(
            "pose", bool(fehler) and bool(st.get("aktiv")), "hinweis",
            "Körperposen-Erkennung (Hailo) hat ein Problem",
            f"Die Gestenerkennung läuft nicht: {fehler}",
            ["Läuft ein anderes Programm auf dem Hailo (hailo-apps)? Beenden.",
             "`ls /dev/hailo0` -- fehlt das Gerät, den Pi neu starten."],
        )

    def _log_pruefen(self) -> None:
        if self._log is None:
            return
        anzahl, letzter = self._log.fehler()
        neu = anzahl - self._fehler_vorher
        self._fehler_vorher = anzahl
        if neu > 0 and letzter is not None:
            self._vorfall(
                "log_fehler", "hinweis", "Fehler im Server-Log",
                f"Der Server hat einen Fehler protokolliert ({letzter['quelle']}): "
                f"{letzter['text']}",
                ["Im Server-Log unten nach Zeilen der Stufe ERROR filtern.",
                 "Wiederholt sich der Fehler: Bericht kopieren und ans Team geben."],
                zeit=letzter["zeit"], anzahl=neu,
            )

    # --- Langsame Messungen (alle paar Sekunden) -----------------------------------

    def _langsam(self) -> None:
        temp = cpu_temperatur()
        mhz = cpu_max_mhz()
        roh = drosselung()
        bauen = bau_prozesse()
        try:
            last = round(os.getloadavg()[0], 2)
        except (AttributeError, OSError):
            last = None
        self._mess.update({
            "temp_c": temp,
            "cpu_max_mhz": mhz,
            "cpu_ist_mhz": cpu_ist_mhz(),
            "last_1min": last,
            "kerne": os.cpu_count(),
            "drosselung": drossel_text(roh),
            "bau_prozesse": bauen,
            "pi_laufzeit_s": pi_laufzeit_s(),
        })

        if temp is not None:
            self._zustand(
                "temperatur", temp >= config.TEMP_WARN_C, "warnung",
                f"Pi wird heiß ({temp:.0f} °C)",
                "Hohe Temperatur kommt fast immer von Dauerlast. Ab 85 °C drosselt "
                "der Pi; davor steigt schon der Strombedarf -- am X3 ein bekannter "
                "Auslöser für USB-Überlast.",
                ["Nachsehen, was rechnet (Messwerte: Installation/Build?).",
                 "Lüfter des Pi prüfen.", "Nicht benötigte Programme beenden."],
            )
        if mhz is not None:
            self._zustand(
                "takt", mhz > config.CPU_MAX_MHZ, "warnung",
                f"CPU-Takt {mhz} MHz (empfohlen ≤ {config.CPU_MAX_MHZ})",
                "Bei 2,4 GHz lösten Lastsprünge am X3 USB-Rücksetzer aus. In "
                "/boot/firmware/config.txt sollte `arm_freq=2000` stehen; das wirkt "
                "erst nach einem Neustart.",
                ["`grep arm_freq /boot/firmware/config.txt` prüfen.",
                 "Sofort (bis zum Neustart): `echo 2000000 | sudo tee "
                 "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq`.",
                 "Details: docs/hardware.md."],
            )
        self._zustand(
            "bauen", bool(bauen), "warnung",
            "Im Hintergrund läuft eine Installation",
            f"Gerade laufen: {', '.join(bauen)}. Solche Builds lasten die CPU "
            "minutenlang aus. Am 28.10. hing ein Treiber-Build endlos, heizte den "
            "Pi auf 83 °C und löste eine USB-Überlast nach der anderen aus.",
            ["Nicht fahren, bis die Installation fertig ist.",
             "Hängt sie (läuft > 10 min)? `ps aux | grep -E 'dpkg|apt|dkms'` und ggf. abbrechen."],
        )
        self._drossel_pruefen(roh)
        self._kernel_log_lesen()

    def _drossel_pruefen(self, roh: int | None) -> None:
        if roh is None:
            return
        self._zustand(
            "unterspannung", bool(roh & 0x1), "kritisch", "Pi meldet Unterspannung",
            "Die 5-V-Versorgung des Pi ist gerade zu niedrig. Dann drosselt er und "
            "schaltet USB-Geräte ab -- alles, was dranhängt, wird unzuverlässig.",
            ["Akku laden bzw. Netzteil prüfen.",
             "5-V-Wandler (Board → Pi) und Kabel kontrollieren.",
             "Stromhungrige USB-Geräte (Orbbec) abziehen."],
        )
        self._zustand(
            "temperaturgrenze", bool(roh & 0x8), "warnung",
            "Pi drosselt wegen Temperatur",
            "Der Pi hat seine Temperaturgrenze erreicht und rechnet langsamer.",
            ["Last reduzieren, Lüfter prüfen."],
        )
        neu = roh & ~self._drossel_vorher & 0xF0000
        self._drossel_vorher |= roh
        if neu & 0x10000:
            self._ereignis("pi", "warnung", "Der Pi hatte seit dem Start Unterspannung")
            self._vorfall(
                "unterspannung_frueher", "hinweis", "Seit dem Start gab es Unterspannung",
                "Irgendwann seit dem Einschalten war die 5-V-Versorgung zu niedrig "
                "(z. B. beim Anfahren mit schwachem Akku). Wann genau, verrät der "
                "Pi nicht -- die Zeitleiste hilft beim Eingrenzen.",
                ["Akku laden.", "Tritt es erneut auf, siehe „Pi meldet Unterspannung“."],
            )
        if neu & 0x80000:
            self._ereignis("pi", "warnung", "Der Pi war seit dem Start an der Temperaturgrenze")

    def _kernel_log_lesen(self) -> None:
        """USB-Abmeldungen und Unterspannung aus dem Kernel-Log.

        journalctl statt dmesg: es liefert echte Uhrzeiten und lässt sich
        ab einem Zeitpunkt abfragen. Dazu muss der Benutzer in der Gruppe
        ``systemd-journal`` oder ``adm`` sein -- sonst sieht er nur eigene
        Meldungen und es kommt der Hinweis unten.
        """
        if self._mock or shutil.which("journalctl") is None:
            self._kernel_quelle = "nicht vorhanden"
            return
        seit = self._kernel_seit or (time.time() - config.DIAGNOSE_KERNEL_RUECKBLICK_S)
        try:
            out = subprocess.run(
                ["journalctl", "-k", "-o", "short-unix", "--no-pager",
                 f"--since=@{seit:.6f}"],
                capture_output=True, text=True, timeout=3,
            )
        except (subprocess.SubprocessError, OSError):
            self._kernel_quelle = "Fehler beim Lesen"
            return

        meldung = out.stderr + out.stdout[:400]
        if "not seeing messages" in meldung or "No journal files" in meldung \
                or "insufficient permissions" in meldung.lower():
            if self._kernel_quelle != "keine Berechtigung":
                self._ereignis("kernel", "hinweis", "Kernel-Log nicht lesbar (Berechtigung)")
            self._kernel_quelle = "keine Berechtigung"
            self._zustand(
                "kernel_log", True, "hinweis", "Kernel-Log nicht lesbar",
                "Ohne Kernel-Log sieht die Diagnose keine USB-Abmeldungen, nur die "
                "Überlast-Zähler. Dem Benutzer fehlt die Leseberechtigung.",
                ["`sudo usermod -aG systemd-journal pi`, danach neu anmelden und "
                 "den Server neu starten."],
            )
            return
        self._kernel_quelle = "ok"
        self._zustand("kernel_log", False)

        for zeile in out.stdout.splitlines():
            teile = zeile.split(None, 3)
            if len(teile) < 4:
                continue
            try:
                zeit = float(teile[0])
            except ValueError:
                continue
            if self._kernel_seit is not None and zeit <= self._kernel_seit:
                continue  # --since ist inklusiv
            self._kernel_seit = max(self._kernel_seit or 0.0, zeit)
            text = teile[3].removeprefix("kernel: ")
            self._kernel_zeile(zeit, text)
        if self._kernel_seit is None:
            self._kernel_seit = time.time() - 1.0

    def _kernel_zeile(self, zeit: float, text: str) -> None:
        if "USB disconnect" in text:
            # "usb 3-1.2: USB disconnect, device number 7" -> "usb 3-1.2"
            port = text.split(":", 1)[0]
            self._ereignis("usb", "warnung", f"USB-Gerät abgemeldet ({port})", zeit)
            self._vorfall(
                "usb_abmeldung", "warnung", "USB-Geräte haben sich abgemeldet",
                "Laut Kernel ist mindestens ein USB-Gerät kurz verschwunden. Die "
                "Software verbindet sich selbst neu; fuhr der Roboter gerade, hat "
                "er angehalten.",
                ["Zeitleiste ansehen: gleichzeitig eine USB-Überlast? Dann gilt deren Lösung.",
                 "Kabel und Stecker am USB-Hub kontrollieren (Wackelkontakt).",
                 *_USB_TEXTE[:1]],
                zeit=zeit,
            )
        elif "ndervoltage" in text or "under-voltage" in text:
            self._ereignis("pi", "kritisch", "Kernel meldet Unterspannung", zeit)
        elif "over-current" in text and "change" not in text:
            # Der Zähler erfasst die Überlast schon -- hier nur fürs Protokoll
            # der Zeitleiste, damit man die Portnummer sieht.
            self._ereignis("usb", "info", f"Kernel: {text[:120]}", zeit)

    # --- Abfragen -------------------------------------------------------------------

    def status(self) -> dict:
        jetzt = time.time()
        with self._lock:
            # Erledigte Vorfälle nach Ablauf des Fensters vergessen.
            for pid in [
                pid for pid, p in self._probleme.items()
                if not p.besteht and jetzt - p.zuletzt > config.PROBLEM_FENSTER_S
            ]:
                del self._probleme[pid]
            probleme = sorted(
                (p.als_dict(jetzt) for p in self._probleme.values()),
                key=lambda p: (p["quittiert"], _RANG[p["schwere"]], -p["zuletzt"]),
            )
            ereignisse = list(self._ereignisse)[-150:][::-1]
            akku = list(self._akku)
            mess = dict(self._mess)

        offen = [p for p in probleme if not p["quittiert"]]
        if any(p["schwere"] == "kritisch" for p in offen):
            gesamt = ("kritisch", "Es gibt ein ernstes Problem -- bitte zuerst ansehen.")
        elif any(p["schwere"] == "warnung" for p in offen):
            gesamt = ("warnung", "Läuft, aber etwas stimmt nicht.")
        else:
            gesamt = ("ok", "Alles in Ordnung." if not offen else "Alles in Ordnung, nur Hinweise.")

        # Für die Kurve reichen ~300 Punkte -- 1800 Punkte alle 3 s an den
        # Browser zu schicken, wäre nur unnötiger WLAN-Verkehr.
        schritt = max(1, len(akku) // 300)
        verlauf = [[round(t, 1), v] for t, v in akku[::schritt]]

        return {
            "zeit": jetzt,
            "simulation": self._mock,
            "gesamt": {"stufe": gesamt[0], "text": gesamt[1]},
            "probleme": probleme,
            "messwerte": {
                **{k: v for k, v in mess.items() if k not in ("lidar", "kameras", "pose")},
                "server_laufzeit_s": round(jetzt - self._gestartet),
            },
            "geraete": self._geraete(mess),
            "ereignisse": ereignisse,
            "akku_verlauf": verlauf,
            "schwellen": {
                "akku_warn_v": config.AKKU_WARN_V,
                "akku_kritisch_v": config.AKKU_KRITISCH_V,
                "temp_warn_c": config.TEMP_WARN_C,
                "cpu_max_mhz": config.CPU_MAX_MHZ,
            },
            "quellen": {
                "kernel_log": self._kernel_quelle,
                "usb_zaehler": mess.get("usb_ueberlast_gesamt") is not None,
                "vcgencmd": mess.get("drosselung") is not None,
                "akku": mess.get("akku_v") is not None,
            },
        }

    def _geraete(self, mess: dict) -> list[dict]:
        """Ein Eintrag je Gerät mit Ampel -- für Menschen, nicht für Programme."""
        geraete = []
        verbunden = bool(self._driver.connected)
        geraete.append({
            "name": "Antrieb (Expansion Board)",
            "ok": verbunden,
            "text": ("Simulation" if self._driver.name == "mock" else "verbunden")
            if verbunden else "nicht verbunden",
            "neu": None,
        })
        lidar = mess.get("lidar")
        if lidar is not None:
            alt = lidar.get("letzte_runde_vor_s")
            ok = bool(lidar.get("geoeffnet")) and alt is not None and alt < 3.0
            geraete.append({
                "name": f"Lidar ({lidar.get('lidar')})",
                "ok": ok,
                "text": f"{lidar.get('umdrehungen', 0)} Umdrehungen" if ok
                else (lidar.get("letzter_fehler") or "keine Daten"),
                "neu": lidar.get("neu_verbunden"),
            })
        for name, st in (mess.get("kameras") or {}).items():
            alt = st.get("letztes_bild_vor_s")
            ok = bool(st.get("geoeffnet")) and alt is not None and alt < 5.0
            geraete.append({
                "name": f"Kamera „{name}“ ({st.get('kamera')})",
                "ok": ok,
                "text": f"{st.get('fps_ist', 0)} fps" if ok
                else (st.get("letzter_fehler") or "kein Bild"),
                "neu": st.get("neu_geoeffnet"),
            })
        pose = mess.get("pose")
        if pose is not None:
            if pose.get("fehler"):
                ok, text = False, str(pose["fehler"])
            elif pose.get("aktiv"):
                ok, text = True, f"{pose.get('fps', 0)} Bilder/s" + (" (simuliert)" if pose.get("simuliert") else "")
            else:
                # Ruht, solange die Interaktion aus ist -- kein Problem.
                ok, text = None, "ruht (Interaktion aus)"
            geraete.append({"name": "Hailo-Posenerkennung", "ok": ok, "text": text,
                            "neu": pose.get("neu_geoeffnet")})
        return geraete
