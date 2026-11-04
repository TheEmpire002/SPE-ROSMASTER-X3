"""Geführte Testläufe für die Seite /tests: Daten sammeln, speichern, hochladen.

Wozu: Mikrofon, Liegend-Erkennung und Personenerkennung sollen am Roboter
besser werden, aber ausgewertet wird am Schreibtisch (ohne SSH auf den Pi).
Ein Testlauf führt die Person mit Ansagen durch einige Phasen ("stell dich
in 2 m Abstand hin", "leg dich quer hin", "sag: Robi, stopp"), schreibt
dabei mit, was die Sensoren melden, und legt alles in einen Ordner mit
Datum und Uhrzeit:

    ~/.rosmaster/tests/2026-11-04_14-03-12_liegend_haltungen/
        ergebnis.json   Phasen, Messwerte (~5x pro Sekunde), Zusammenfassung
        bilder/*.jpg    wenige Kamerabilder, das Skelett steht im JSON
        ton.wav         nur bei Mikrofontests

"Hochladen" schiebt die Ordner nach ``tests/<ordner>/`` auf den Branch
``testdaten`` im GitHub-Repo. Das geht über Git-Bordmittel (hash-object,
commit-tree, push), ohne den Arbeitsbaum oder HEAD des Pi anzufassen.

Ein Testlauf fährt nie. Während eines Mikrofontests führt der Server
erkannte Sprachbefehle nicht aus (``befehle_sperren``), sonst führe der
Roboter bei "Robi, folge mir" los.
"""

from __future__ import annotations

import base64
import io
import json
import logging
import math
import os
import queue
import re
import shutil
import statistics
import subprocess
import tempfile
import threading
import time
import wave
import zipfile
from datetime import datetime
from pathlib import Path

from . import config
from .zustandserkennung import bewertung as bw

log = logging.getLogger(__name__)

# Ordnernamen: Datum_Uhrzeit_test. Alles andere wird abgelehnt, damit über
# ?name=... niemand aus dem Ordner herauskommt ("../").
_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_[a-z0-9_]+$")

# Name, unter dem der Testlauf die Posenerkennung einschaltet.
_POSE_NUTZER = "tests"


def _phase(name, titel, anweisung, dauer_s, **mehr) -> dict:
    return {"name": name, "titel": titel, "anweisung": anweisung, "dauer_s": dauer_s, **mehr}


# ``art`` legt fest, was mitgeschrieben wird: "mikro" (Pegel, Ton, gehörte
# Sätze), "pose" (Skelette, Liegend- und Boden-Zeichen, Lidar), "personen"
# (wie pose, zusätzlich die HOG-Erkennung), "kamera" (Bildtakt, Bildalter).
# ``erwartet`` in einer Phase ist das, was richtig wäre; die Zusammenfassung
# zählt, wie oft es stimmte.
KATALOG: list[dict] = [
    {
        "id": "mikro_pegel",
        "kategorie": "mikrofon",
        "titel": "Mikrofon: Pegel und Störgeräusche",
        "kurz": "Wie laut ist es still, wie laut beim Sprechen, und hört er Befehle, wo keine sind?",
        "art": "mikro",
        "anleitung": [
            "In normalem Abstand (1-2 m) vor den Roboter stellen.",
            "Musik und Fernseher erst ausmachen; der Test sagt, wann Geräusche erlaubt sind.",
            "In der Sprech-Phase ganz normal erzählen, aber ohne 'Robi' zu sagen.",
            "Aufgezeichnet werden Pegel, alles Gehörte und der Ton als WAV.",
        ],
        "phasen": [
            _phase("stille", "Stille", "Bitte ganz still sein.", 10, erwartet="nichts"),
            _phase("sprechen", "Normal sprechen", "Erzähl etwas, aber sag nicht Robi.", 12, erwartet="nichts"),
            _phase("geraeusche", "Geräusche", "Jetzt Geräusche machen: Musik, klatschen, Stühle rücken.", 12,
                   erwartet="nichts"),
        ],
    },
    {
        "id": "mikro_befehle",
        "kategorie": "mikrofon",
        "titel": "Mikrofon: Sprachbefehle nah und fern",
        "kurz": "Versteht er die Befehle sicher? Mit Konfidenz je Wort, nah (1 m) und fern (3 m).",
        "art": "mikro",
        "anleitung": [
            "Erste Runde in etwa 1 m Abstand, zweite Runde in etwa 3 m.",
            "Nach jeder Ansage den angezeigten Satz einmal deutlich sagen, mit 'Robi' davor.",
            "Der Roboter führt die Befehle während des Tests nicht aus und fährt nicht.",
            "Aufgezeichnet werden Erkanntes, Verworfenes, Konfidenz je Wort und der Ton.",
        ],
        "phasen": [
            _phase(f"{runde}_{befehl}", f"{titel}: „Robi, {satz}“", f"Sag jetzt: Robi, {satz}.", 7,
                   erwartet=befehl, satz=f"Robi, {satz}")
            for runde, titel in (("nah", "1 m"), ("fern", "3 m"))
            for befehl, satz in (
                ("stopp", "stopp"), ("folgen", "folge mir"), ("herkommen", "komm her"),
                ("nach_hause", "nach Hause"), ("musik_aus", "Musik aus"),
            )
        ],
    },
    {
        "id": "liegend_haltungen",
        "kategorie": "liegend",
        "titel": "Liegend: Haltungen",
        "kurz": "Erkennt er Stehen und Sitzen als aufrecht und verschiedene Arten zu liegen als liegend?",
        "art": "pose",
        "anleitung": [
            "Etwa 2 m vor dem Roboter, mitten im Kamerabild. Platz zum Hinlegen freimachen (Decke).",
            "Den Ansagen folgen; jede Haltung ruhig halten, bis die nächste Ansage kommt.",
            "Quer = Körper von links nach rechts im Bild. Längs = Füße zum Roboter.",
            "Aufgezeichnet werden Skelette, Rumpfwinkel, Liegend- und Boden-Zeichen, Lidar und einige Bilder.",
        ],
        "phasen": [
            _phase("stehen", "Stehen", "Stell dich in zwei Metern Abstand hin.", 10, erwartet="aufrecht"),
            _phase("sitzen_stuhl", "Auf Stuhl sitzen", "Setz dich auf einen Stuhl.", 10, erwartet="aufrecht"),
            _phase("sitzen_boden", "Auf dem Boden sitzen", "Setz dich auf den Boden.", 10, erwartet="aufrecht"),
            _phase("liegen_quer", "Liegen quer", "Leg dich quer vor den Roboter, auf den Rücken.", 15,
                   erwartet="liegt"),
            _phase("liegen_laengs", "Liegen längs", "Leg dich längs hin, die Füße zum Roboter.", 15,
                   erwartet="liegt"),
            _phase("liegen_seite", "Liegen auf der Seite", "Leg dich auf die Seite, Beine angezogen.", 15,
                   erwartet="liegt"),
            _phase("liegen_bewegen", "Liegen und bewegen", "Bleib liegen und beweg Arme und Beine.", 12,
                   erwartet="liegt"),
        ],
    },
    {
        "id": "liegend_abstand",
        "kategorie": "liegend",
        "titel": "Liegend: Abstand",
        "kurz": "Ab welchem Abstand sieht er eine liegende Person noch (Kamera sitzt tief)?",
        "art": "pose",
        "anleitung": [
            "Abstände vorher auf dem Boden markieren: 1 m, 2 m, 3 m vor dem Roboter.",
            "Jeweils quer auf den Rücken legen, Körpermitte vor dem Roboter.",
        ],
        "phasen": [
            _phase("liegen_1m", "Liegen in 1 m", "Leg dich quer hin, einen Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
            _phase("liegen_2m", "Liegen in 2 m", "Leg dich quer hin, zwei Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
            _phase("liegen_3m", "Liegen in 3 m", "Leg dich quer hin, drei Meter vor dem Roboter.", 12,
                   erwartet="liegt"),
        ],
    },
    {
        "id": "liegend_zwei_personen",
        "kategorie": "liegend",
        "titel": "Liegend: zwei Personen",
        "kurz": "Eine steht, eine liegt. Wen nimmt er? (Er nimmt die größte Person zuerst.)",
        "art": "pose",
        "anleitung": [
            "Braucht zwei Personen. Beide etwa 2 m vor dem Roboter, nebeneinander.",
            "Erst stehen beide, dann legt sich eine hin, dann tauschen.",
        ],
        "phasen": [
            _phase("beide_stehen", "Beide stehen", "Beide stellen sich nebeneinander hin.", 10, erwartet="aufrecht"),
            _phase("links_liegt", "Links liegt", "Die linke Person legt sich hin, die rechte bleibt stehen.", 15,
                   erwartet="einer_liegt"),
            _phase("rechts_liegt", "Rechts liegt", "Jetzt tauschen: rechts liegt, links steht.", 15,
                   erwartet="einer_liegt"),
        ],
    },
    {
        "id": "personen_leerer_raum",
        "kategorie": "personen",
        "titel": "Personen: leerer Raum",
        "kurz": "Sieht er Personen, wo keine sind? Jeder Fehltreffer wird mit Bild gespeichert.",
        "art": "personen",
        "anleitung": [
            "Niemand darf im Kamerabild sein, auch nicht teilweise.",
            "Für die zweite Phase den Roboter von Hand in eine andere Richtung drehen (Möbel, Fenster, Bilder).",
        ],
        "phasen": [
            _phase("blick_a", "Leerer Raum, Blick A", "Bitte aus dem Bild gehen.", 25, erwartet="niemand"),
            _phase("blick_b", "Leerer Raum, Blick B",
                   "Dreh den Roboter in eine andere Richtung und geh aus dem Bild.", 25, erwartet="niemand"),
        ],
    },
    {
        "id": "personen_abstand",
        "kategorie": "personen",
        "titel": "Personen: Abstand und Haltung",
        "kurz": "Wie zuverlässig findet er eine Person in 1-4 m, seitlich, halb verdeckt und sitzend?",
        "art": "personen",
        "anleitung": [
            "Abstände auf dem Boden markieren: 1, 2, 3 und 4 m vor dem Roboter.",
            "Bei 'halb verdeckt' hinter einen Stuhl oder Tisch stellen.",
        ],
        "phasen": [
            _phase("abstand_1m", "1 m", "Stell dich einen Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_2m", "2 m", "Stell dich zwei Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_3m", "3 m", "Stell dich drei Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("abstand_4m", "4 m", "Stell dich vier Meter vor den Roboter.", 8, erwartet="eine_person"),
            _phase("seitlich", "Seitlich", "Stell dich in zwei Metern seitlich hin, Schulter zum Roboter.", 8,
                   erwartet="eine_person"),
            _phase("rand", "Am Bildrand", "Stell dich in zwei Metern an den linken Bildrand.", 8,
                   erwartet="eine_person"),
            _phase("verdeckt", "Halb verdeckt", "Stell dich halb hinter einen Stuhl oder Tisch.", 8,
                   erwartet="eine_person"),
            _phase("sitzend", "Sitzend", "Setz dich in zwei Metern auf einen Stuhl.", 8, erwartet="eine_person"),
        ],
    },
    {
        "id": "kamera_takt",
        "kategorie": "personen",
        "titel": "Kamera: Takt und Bildalter",
        "kurz": "Ruckelt die Kamera? Bildrate, Bildalter und veraltete Bilder aller Kameras.",
        "art": "kamera",
        "anleitung": [
            "Erst ruhig vor dem Roboter stehen, dann vor der Kamera hin und her gehen.",
            "Ruckeln im Videobild der App bitte notieren (Notizfeld nach dem Test).",
        ],
        "phasen": [
            _phase("ruhig", "Ruhig", "Bleib ruhig vor dem Roboter stehen.", 15, erwartet="eine_person"),
            _phase("bewegung", "Bewegung", "Geh jetzt vor der Kamera hin und her.", 15, erwartet="eine_person"),
        ],
    },
]

KATEGORIEN = {
    "mikrofon": "Mikrofon",
    "liegend": "Liegend erkennen",
    "personen": "Personenerkennung",
}


def _r(wert, stellen: int = 3):
    """Runden für das JSON (hält die Dateien klein); None bleibt None."""
    if wert is None or not isinstance(wert, (int, float)) or not math.isfinite(wert):
        return None
    return round(float(wert), stellen)


def _verteilung(werte: list[float]) -> dict | None:
    werte = [w for w in werte if w is not None]
    if not werte:
        return None
    return {
        "min": _r(min(werte)), "median": _r(statistics.median(werte)),
        "max": _r(max(werte)), "mittel": _r(statistics.fmean(werte)), "anzahl": len(werte),
    }


class Testlauf:
    """Führt einen Test nach dem anderen aus (eigener Thread) und verwaltet die Ordner."""

    def __init__(
        self, pose=None, personen=None, kamera=None, kameras=None, lidar=None,
        stimme=None, sprache=None, mock: bool = False, ordner: Path | None = None,
    ) -> None:
        self._pose = pose
        self._personen = personen
        self._kamera = kamera  # die Kamera, durch die die Pose schaut
        self._kameras = dict(kameras or {})
        self._lidar = lidar
        self._stimme = stimme
        self._sprache = sprache
        self._mock = mock
        if ordner is None:
            ordner = Path(config.TESTS_ORDNER)
            # Simulierte Läufe getrennt halten, damit sie nie als echte
            # Messung hochgeladen werden.
            if mock:
                ordner = ordner.with_name(ordner.name + "_mock")
        self.ordner = Path(ordner)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._hochladen_lock = threading.Lock()
        self._zustand: dict = {"aktiv": False}
        self._letzter: str | None = None
        self._meldung: str | None = None

    # --- Abfragen ---------------------------------------------------------------

    @staticmethod
    def katalog() -> dict:
        return {"kategorien": KATEGORIEN, "tests": KATALOG}

    @property
    def aktiv(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def befehle_sperren(self) -> bool:
        """True während eines Mikrofontests: erkannte Befehle nicht ausführen."""
        with self._lock:
            return bool(self._zustand.get("aktiv")) and self._zustand.get("art") == "mikro"

    def status(self) -> dict:
        with self._lock:
            zustand = dict(self._zustand)
        if zustand.get("aktiv") and zustand.get("ende"):
            zustand["rest_s"] = _r(max(0.0, zustand.pop("ende") - time.monotonic()), 1)
        else:
            zustand.pop("ende", None)
        zustand["letzter"] = self._letzter
        zustand["meldung"] = self._meldung
        zustand["ordner"] = str(self.ordner)
        zustand["simulation"] = self._mock
        zustand["token"] = self._token() is not None
        return zustand

    def liste(self) -> list[dict]:
        """Alle gespeicherten Läufe, neueste zuerst."""
        if not self.ordner.is_dir():
            return []
        laeufe = []
        for d in sorted(self.ordner.iterdir(), reverse=True):
            if not d.is_dir() or not _NAME.match(d.name):
                continue
            eintrag = {"name": d.name, "hochgeladen": (d / "hochgeladen.json").exists()}
            try:
                daten = json.loads((d / "ergebnis.json").read_text(encoding="utf-8"))
                eintrag.update({
                    "test": daten.get("test"), "titel": daten.get("titel"),
                    "kategorie": daten.get("kategorie"), "beginn": daten.get("beginn"),
                    "abgebrochen": daten.get("abgebrochen"), "notiz": daten.get("notiz", ""),
                    "phasen": [
                        {"titel": p.get("titel"), "kurz": p.get("zusammenfassung", {}).get("kurz")}
                        for p in daten.get("phasen", [])
                    ],
                })
            except (OSError, ValueError):
                eintrag["fehler"] = "ergebnis.json fehlt oder ist kaputt"
            eintrag["bytes"] = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
            laeufe.append(eintrag)
        return laeufe

    # --- Steuern ----------------------------------------------------------------

    def start(self, test_id: str) -> str | None:
        """Test starten. Gibt einen Fehlertext zurück, sonst None."""
        test = next((t for t in KATALOG if t["id"] == test_id), None)
        if test is None:
            return f"unbekannter Test: {test_id!r}"
        if self.aktiv:
            return "es läuft schon ein Test"
        fehlt = self._fehlt(test)
        if fehlt:
            return fehlt
        self._stop.clear()
        self._meldung = None
        with self._lock:
            self._zustand = {"aktiv": True, "test": test["id"], "titel": test["titel"], "art": test["art"]}
        self._thread = threading.Thread(target=self._lauf_sicher, args=(test,), name="testlauf", daemon=True)
        self._thread.start()
        return None

    def stop(self) -> None:
        self._stop.set()
        if self._stimme is not None:
            self._stimme.verstummen()

    def beenden(self) -> None:
        """Beim Herunterfahren des Servers."""
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def notiz(self, name: str, text: str) -> str | None:
        """Notiz zum Lauf speichern (was auffiel). Fehlertext oder None."""
        d = self._ordner_von(name)
        if d is None:
            return "unbekannter Lauf"
        datei = d / "ergebnis.json"
        try:
            daten = json.loads(datei.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "ergebnis.json fehlt oder ist kaputt"
        daten["notiz"] = str(text)[: config.TESTS_NOTIZ_MAX_ZEICHEN]
        self._json_schreiben(datei, daten)
        # Geändert = noch einmal hochladen; der neue Stand ersetzt den alten.
        (d / "hochgeladen.json").unlink(missing_ok=True)
        return None

    def _fehlt(self, test: dict) -> str | None:
        art = test["art"]
        if art == "mikro":
            if self._sprache is None or not self._sprache.verfuegbar:
                return "Spracherkennung nicht verfügbar (siehe /sprache/status)"
        elif art in ("pose", "personen"):
            if self._pose is None or not self._pose.verfuegbar:
                return "Posenerkennung nicht verfügbar (siehe /diagnose)"
        if art in ("personen", "kamera") and self._kamera is None:
            return "keine Personenkamera"
        return None

    def _setzen(self, **felder) -> None:
        with self._lock:
            self._zustand.update(felder)

    # --- Ablauf -----------------------------------------------------------------

    def _lauf_sicher(self, test: dict) -> None:
        try:
            self._lauf(test)
        except Exception as e:  # ein Fehler im Test darf den Server nicht stören
            log.exception("Testlauf %s fehlgeschlagen", test["id"])
            self._meldung = f"Test abgebrochen: {e}"
        finally:
            with self._lock:
                self._zustand = {"aktiv": False}

    def _sagen(self, text: str) -> None:
        if self._stimme is not None:
            self._stimme.sagen(text)

    def _stimme_abwarten(self) -> None:
        """Warten, bis die Ansage fertig ist: Solange er spricht, hört das Ohr
        weg, und der gesprochene Satz käme sonst im Mikrofon mit an."""
        if self._stimme is None:
            return
        bis = time.monotonic() + config.TESTS_STIMME_WARTEN_S
        time.sleep(0.3)  # bis der Satz aus der Warteschlange geholt ist
        while time.monotonic() < bis and self._stimme.beschaeftigt and not self._stop.is_set():
            time.sleep(0.1)

    def _lauf(self, test: dict) -> None:
        beginn = datetime.now()
        name = f"{beginn:%Y-%m-%d_%H-%M-%S}_{test['id']}"
        ziel = self.ordner / name
        (ziel / "bilder").mkdir(parents=True, exist_ok=True)
        art = test["art"]
        sicht = art in ("pose", "personen", "kamera")
        daten = {
            "test": test["id"], "titel": test["titel"], "kategorie": test["kategorie"], "art": art,
            "beginn": beginn.isoformat(timespec="seconds"), "simulation": self._mock,
            "einstellungen": self._einstellungen(art), "anleitung": test["anleitung"],
            "phasen": [], "proben": [], "status": [], "bilder": [], "ereignisse": [], "notiz": "",
        }
        start_mono, start_unix = time.monotonic(), time.time()
        abo = wav = ton_thread = None
        ton_ende = threading.Event()
        ereignisse: dict = {}
        if self._pose is not None and art in ("pose", "personen", "kamera"):
            self._pose.aktivieren(True, _POSE_NUTZER)
        if art == "personen" and self._personen is not None:
            # Eigener Nutzername: Am Ende wird nur der Testlauf abgemeldet,
            # ein gleichzeitig laufender Erkunder behält seine Erkennung.
            self._personen.aktivieren(True, _POSE_NUTZER)
        if art == "mikro" and self._sprache is not None and self._sprache.ton_rate:
            abo = self._sprache.ton_abonnieren()
            wav = wave.open(str(ziel / "ton.wav"), "wb")
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(self._sprache.ton_rate)
            # Eigener Thread: Während der Ansagen wird nicht abgetastet, die
            # Ton-Schlange (2 s) liefe sonst über und es fehlten Stücke.
            ton_thread = threading.Thread(target=self._ton_schreiben, args=(abo, wav, ton_ende),
                                          name="testlauf-ton", daemon=True)
            ton_thread.start()
        daten["ton"] = "ton.wav" if wav is not None else None
        bilder_je_phase = max(1, config.TESTS_MAX_BILDER // max(1, len(test["phasen"])))
        try:
            self._sagen(f"Test {test['titel']} beginnt.")
            for nr, phase in enumerate(test["phasen"]):
                if self._stop.is_set():
                    break
                self._setzen(phase=nr, phasen=len(test["phasen"]), phase_titel=phase["titel"],
                             anweisung=phase["anweisung"], satz=phase.get("satz"), vorlauf=True,
                             ende=time.monotonic() + config.TESTS_VORLAUF_S, live=None)
                self._sagen(phase["anweisung"])
                if art == "mikro":
                    self._stimme_abwarten()
                    self._setzen(ende=time.monotonic() + config.TESTS_VORLAUF_S / 2)
                    self._stop.wait(config.TESTS_VORLAUF_S / 2)
                else:
                    self._stop.wait(config.TESTS_VORLAUF_S)
                if self._stop.is_set():
                    break
                anfang = time.monotonic()
                ende = anfang + phase["dauer_s"]
                self._setzen(vorlauf=False, ende=ende)
                eintrag = {k: v for k, v in phase.items()}
                eintrag["von_s"] = _r(anfang - start_mono, 2)
                bild_zeiten = [anfang + phase["dauer_s"] * (i + 0.5) / bilder_je_phase
                               for i in range(bilder_je_phase)]
                naechster_status = 0.0
                while not self._stop.is_set():
                    jetzt = time.monotonic()
                    if jetzt >= ende:
                        break
                    probe = {"t": _r(jetzt - start_mono, 2), "phase": phase["name"]}
                    if art == "mikro":
                        probe.update(self._probe_mikro(start_unix, ereignisse))
                    if sicht:
                        probe.update(self._probe_sicht(art))
                    daten["proben"].append(probe)
                    self._setzen(live=self._live(art, probe))
                    if sicht and jetzt >= naechster_status:
                        naechster_status = jetzt + 1.0
                        daten["status"].append({"t": probe["t"], "phase": phase["name"], **self._status_sicht()})
                    # Bilder: gleichmäßig über die Phase, beim leeren Raum
                    # zusätzlich jeder Fehltreffer (solange Platz ist).
                    fehltreffer = phase.get("erwartet") == "niemand" and (probe.get("personen") or probe.get("hog"))
                    if sicht and len(daten["bilder"]) < config.TESTS_MAX_BILDER and (
                        (bild_zeiten and jetzt >= bild_zeiten[0]) or fehltreffer
                    ):
                        if bild_zeiten and jetzt >= bild_zeiten[0]:
                            bild_zeiten.pop(0)
                        bild = self._bild(ziel, phase["name"], len(daten["bilder"]), probe,
                                          "fehltreffer" if fehltreffer else "phase")
                        if bild is not None:
                            daten["bilder"].append(bild)
                    self._stop.wait(1.0 / config.TESTS_TAKT_HZ)
                eintrag["bis_s"] = _r(time.monotonic() - start_mono, 2)
                daten["phasen"].append(eintrag)
            if art == "mikro" and self._sprache is not None and not self._stop.is_set():
                # Ein Satz vom Phasenende wird erst nach der Sprechpause
                # erkannt; kurz nachhören, sonst fehlt er.
                self._stop.wait(config.TESTS_NACHLAUF_S)
                self._probe_mikro(start_unix, ereignisse)
        finally:
            if self._pose is not None:
                self._pose.aktivieren(False, _POSE_NUTZER)
            if art == "personen" and self._personen is not None:
                self._personen.aktivieren(False, _POSE_NUTZER)
            if ton_thread is not None:
                ton_ende.set()
                ton_thread.join(timeout=2.0)
            if abo is not None:
                self._sprache.ton_abmelden(abo)
            if wav is not None:
                wav.close()
        # Zeiten relativ zum Laufbeginn (t), wie bei den Proben.
        daten["ereignisse"] = sorted(ereignisse.values(), key=lambda e: e["t"])
        daten["abgebrochen"] = self._stop.is_set()
        daten["dauer_s"] = _r(time.monotonic() - start_mono, 1)
        for p in daten["phasen"]:
            p["zusammenfassung"] = zusammenfassen(art, p, daten)
        self._json_schreiben(ziel / "ergebnis.json", daten)
        self._letzter = name
        self._meldung = f"{'Abgebrochen' if daten['abgebrochen'] else 'Fertig'}: {name}"
        log.info("Testlauf gespeichert: %s", ziel)
        if not daten["abgebrochen"]:
            self._sagen("Test fertig. Danke!")

    def _einstellungen(self, art: str) -> dict:
        """Die Schwellen, mit denen gerade bewertet wird: Ohne sie lassen
        sich alte Läufe nicht mit neuen vergleichen."""
        namen = ["CAMERA_FPS", "PERSON_CAMERA", "POSE_MAX_BILDALTER_S"]
        if art == "mikro":
            namen += ["SPRACHE_MIN_KONFIDENZ", "SPRACHE_MIN_KONFIDENZ_MUSIK", "SPRACHE_MAX_UNK_ANTEIL",
                      "SPRACHE_MIKROFON", "SPRACHE_WECK_FENSTER_S"]
        else:
            namen += ["HEALTH_LIEGEND_GRAD", "HEALTH_LIEGEND_SEITENVERHAELTNIS", "HEALTH_BODEN_MIN_BREITE_M",
                      "HEALTH_BODEN_BILD_UNTEN", "CAMERA_WIDTH", "CAMERA_HEIGHT"]
        return {n: getattr(config, n, None) for n in namen if isinstance(getattr(config, n, None),
                                                                           (int, float, str, bool, type(None)))}

    def _probe_mikro(self, start_unix: float, ereignisse: dict) -> dict:
        live = self._sprache.live()  # hält nebenbei die Pegelmessung an
        # live() kennt nur die letzten Sätze; deshalb laufend einsammeln.
        for e in live.get("ereignisse", []):
            if e.get("zeit", 0) >= start_unix:
                ereignisse[(e["zeit"], e.get("text"))] = {**e, "t": _r(e["zeit"] - start_unix, 2)}
        self._setzen(gehoert=max(ereignisse.values(), key=lambda e: e["t"], default=None))
        return {"pegel": live.get("pegel"), "teil": live.get("teil") or "", "taub": live.get("taub"),
                "zustand": live.get("zustand")}

    def _punkte(self):
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or time.monotonic() - scan.zeit > config.TESTS_SCAN_MAX_ALTER_S:
            return None
        return scan.punkte()

    def _probe_sicht(self, art: str) -> dict:
        probe: dict = {}
        if self._pose is not None:
            personen = self._pose.personen()
            punkte = self._punkte() if art != "kamera" else None
            fov = self._pose.fov_deg
            probe["personen"] = [self._person(p, punkte, fov) for p in personen]
            probe["lidar"] = punkte is not None
        if art == "personen" and self._personen is not None:
            probe["hog"] = [{k: _r(v) for k, v in b.items()} for b in self._personen.boxen()]
        return probe

    @staticmethod
    def _person(p: dict, punkte, fov: float) -> dict:
        """Eine Person so, wie der Healthbot sie bewerten würde, plus Rohdaten."""
        liegt = bw.liegend(p)
        boden = bw.boden(p, punkte, fov)
        return {
            "box": [_r(v) for v in p["box"]], "score": _r(p.get("score")), "winkel": _r(p.get("winkel"), 1),
            "rumpfwinkel": _r(bw.rumpfwinkel(p), 1),
            "liegend": liegt["zustand"], "liegend_wert": liegt["wert"],
            "boden": boden["zustand"], "boden_wert": boden["wert"], "boden_text": boden["text"],
            "kp": [[_r(x), _r(y), _r(c, 2)] for x, y, c in p.get("kp", [])],
        }

    def _status_sicht(self) -> dict:
        st: dict = {}
        if self._pose is not None:
            ps = self._pose.status()
            st["pose"] = {k: ps.get(k) for k in ("fps", "bilder", "letzte_erkennung_vor_s", "bild_alter_s",
                                                  "veraltete_bilder", "aktiv", "fehler")}
        st["kameras"] = {}
        for name, kam in self._kameras.items():
            ks = kam.status()
            st["kameras"][name] = {k: ks.get(k) for k in ("fps_ist", "fps_soll", "letztes_bild_vor_s",
                                                           "bilder_gesamt", "letzter_fehler", "neu_geoeffnet")}
        return st

    @staticmethod
    def _live(art: str, probe: dict) -> dict:
        """Kurz für die Seite: was gerade gemessen wird."""
        if art == "mikro":
            return {"pegel": probe.get("pegel"), "teil": probe.get("teil")}
        personen = probe.get("personen") or []
        live = {"personen": len(personen), "liegend": [p["liegend"] for p in personen],
                "rumpfwinkel": [p["rumpfwinkel"] for p in personen]}
        if "hog" in probe:
            live["hog"] = len(probe["hog"])
        return live

    def _bild(self, ziel: Path, phase: str, nr: int, probe: dict, grund: str) -> dict | None:
        if self._kamera is None:
            return None
        frame, aufnahme = self._kamera.latest_mit_zeit()
        if not frame:
            return None
        daten, mime = frame
        endung = "png" if "png" in (mime or "") else "jpg"
        datei = f"bilder/{nr:02d}_{phase}.{endung}"
        (ziel / datei).write_bytes(daten)
        alter = time.monotonic() - aufnahme if aufnahme else None
        # Das Skelett zum Bild steht hier (dieselben Personen wie in der
        # Probe), damit man es beim Auswerten darüberzeichnen kann.
        return {"datei": datei, "t": probe["t"], "phase": phase, "grund": grund, "bild_alter_s": _r(alter, 2),
                "personen": probe.get("personen", []), "hog": probe.get("hog")}

    @staticmethod
    def _ton_schreiben(abo: queue.Queue, wav, ende: threading.Event) -> None:
        grenze = int(config.TESTS_WAV_MAX_S * wav.getframerate())
        frames = 0
        while not ende.is_set() or not abo.empty():
            try:
                roh = abo.get(timeout=0.2)
            except queue.Empty:
                continue
            if frames < grenze:
                wav.writeframes(roh)
                frames += len(roh) // 2

    @staticmethod
    def _json_schreiben(datei: Path, daten: dict) -> None:
        tmp = datei.with_suffix(".tmp")
        tmp.write_text(json.dumps(daten, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(datei)

    def _ordner_von(self, name: str) -> Path | None:
        if not isinstance(name, str) or not _NAME.match(name):
            return None
        d = self.ordner / name
        return d if d.is_dir() else None

    # --- ZIP und Hochladen ---------------------------------------------------------

    def zip_bauen(self, name: str) -> bytes | None:
        d = self._ordner_von(name)
        if d is None:
            return None
        puffer = io.BytesIO()
        with zipfile.ZipFile(puffer, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(d.rglob("*")):
                if f.is_file() and f.name != "hochgeladen.json":
                    z.write(f, f"{name}/{f.relative_to(d).as_posix()}")
        return puffer.getvalue()

    @staticmethod
    def _token() -> str | None:
        try:
            token = Path(config.TESTS_GIT_TOKEN_DATEI).read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return token or None

    def hochladen(self, namen: list[str] | None = None) -> dict:
        """Läufe auf den Branch testdaten schieben (blockiert, im Thread aufrufen).

        ``namen`` None = alle, die noch nicht hochgeladen sind. Ergebnis:
        ``{"ok": bool, "hochgeladen": [...], "fehler": str|None, "commit": sha}``.
        """
        if namen is None:
            namen = [e["name"] for e in self.liste() if not e["hochgeladen"]]
        ordner = [self._ordner_von(n) for n in namen]
        if any(o is None for o in ordner):
            return {"ok": False, "hochgeladen": [], "fehler": "unbekannter Lauf in der Liste"}
        if not ordner:
            return {"ok": True, "hochgeladen": [], "fehler": None, "commit": None}
        token = self._token()
        if token is None:
            return {"ok": False, "hochgeladen": [], "fehler": (
                f"kein GitHub-Token in {config.TESTS_GIT_TOKEN_DATEI} (Anleitung auf der Seite); "
                "solange die Läufe als ZIP herunterladen")}
        if not self._hochladen_lock.acquire(blocking=False):
            return {"ok": False, "hochgeladen": [], "fehler": "es wird schon hochgeladen"}
        try:
            repo = git_wurzel(Path(__file__).resolve().parent)
            url = config.TESTS_GIT_URL or https_url(_git(repo, ["remote", "get-url", "origin"]).strip())
            commit = hochladen_git(repo, url, token, config.TESTS_GIT_BRANCH, ordner)
        except GitFehler as e:
            log.warning("Hochladen der Testläufe misslungen: %s", e)
            return {"ok": False, "hochgeladen": [], "fehler": str(e)}
        finally:
            self._hochladen_lock.release()
        for o in ordner:
            self._json_schreiben(o / "hochgeladen.json", {
                "zeit": datetime.now().isoformat(timespec="seconds"), "commit": commit,
                "branch": config.TESTS_GIT_BRANCH})
        log.info("%d Testläufe hochgeladen (%s)", len(ordner), commit[:8])
        return {"ok": True, "hochgeladen": [o.name for o in ordner], "fehler": None, "commit": commit}


# --- Zusammenfassung je Phase ------------------------------------------------------


def zusammenfassen(art: str, phase: dict, daten: dict) -> dict:
    """Das Wichtigste einer Phase in wenigen Zahlen, ``kurz`` als Text für die Liste."""
    proben = [p for p in daten["proben"] if p["phase"] == phase["name"]]
    erwartet = phase.get("erwartet")
    z: dict = {"proben": len(proben)}
    if art == "mikro":
        z["pegel"] = _verteilung([p.get("pegel") for p in proben])
        von, bis = phase.get("von_s") or 0.0, phase.get("bis_s") or 0.0
        gehoert = [e for e in daten.get("ereignisse", []) if von - 0.5 <= e.get("t", -1) <= bis + 1.0]
        z["gehoert"] = [{k: e.get(k) for k in ("text", "ergebnis", "befehl", "grund", "konfidenz")}
                        for e in gehoert]
        befehle = [e.get("befehl") for e in gehoert if e.get("ergebnis") == "befehl"]
        if erwartet == "nichts":
            z["fehlbefehle"] = len(befehle)
            z["kurz"] = f"{len(gehoert)} Sätze gehört, {len(befehle)} Befehle (sollten 0 sein)"
        else:
            z["treffer"] = erwartet in befehle
            verworfen = [e for e in gehoert if e.get("ergebnis") == "verworfen"]
            z["kurz"] = ("erkannt" if z["treffer"] else
                         f"nicht erkannt ({len(verworfen)} verworfen, {len(gehoert)} Sätze)")
        return z
    anzahl = [len(p.get("personen") or []) for p in proben]
    z["mit_person"] = _r(sum(1 for a in anzahl if a) / len(anzahl), 2) if anzahl else None
    z["personen_max"] = max(anzahl, default=0)
    erste = [p["personen"][0] for p in proben if p.get("personen")]
    z["rumpfwinkel"] = _verteilung([p["rumpfwinkel"] for p in erste])
    z["liegend_anteil"] = (_r(sum(p["liegend"] == bw.AUFFAELLIG for p in erste) / len(erste), 2)
                           if erste else None)
    z["boden_anteil"] = (_r(sum(p["boden"] == bw.AUFFAELLIG for p in erste) / len(erste), 2)
                         if erste else None)
    if art == "personen":
        hog = [len(p.get("hog") or []) for p in proben]
        z["hog_mit_person"] = _r(sum(1 for a in hog if a) / len(hog), 2) if hog else None
    if art == "kamera" or daten.get("status"):
        st = [s for s in daten.get("status", []) if s["phase"] == phase["name"]]
        z["pose_fps"] = _verteilung([(s.get("pose") or {}).get("fps") for s in st])
        z["bild_alter_s"] = _verteilung([(s.get("pose") or {}).get("bild_alter_s") for s in st])
    # "richtig": Anteil der Proben, in denen das Ergebnis zur Phase passt.
    if erwartet == "niemand":
        z["richtig"] = _r(1.0 - (z["mit_person"] or 0.0), 2) if anzahl else None
    elif erwartet == "eine_person":
        z["richtig"] = z["mit_person"]
    elif erwartet in ("aufrecht", "liegt"):
        soll = bw.AUFFAELLIG if erwartet == "liegt" else bw.OK
        z["richtig"] = _r(sum(p["liegend"] == soll for p in erste) / len(proben), 2) if proben else None
    elif erwartet == "einer_liegt":
        treffer = sum(1 for p in proben if any(q["liegend"] == bw.AUFFAELLIG for q in p.get("personen") or []))
        z["richtig"] = _r(treffer / len(proben), 2) if proben else None
        z["liegende_zuerst"] = (_r(sum(p["liegend"] == bw.AUFFAELLIG for p in erste) / len(proben), 2)
                                if proben else None)
    teile = []
    if z.get("richtig") is not None:
        teile.append(f"{z['richtig'] * 100:.0f} % richtig")
    if z["mit_person"] is not None:
        teile.append(f"Person in {z['mit_person'] * 100:.0f} % der Proben")
    if z["rumpfwinkel"]:
        teile.append(f"Rumpf median {z['rumpfwinkel']['median']:.0f}°")
    z["kurz"] = ", ".join(teile) or "keine Daten"
    return z


# --- Git, ohne Arbeitsbaum ------------------------------------------------------------


class GitFehler(RuntimeError):
    pass


def _git(repo: Path, args: list[str], env: dict | None = None, eingabe: str | None = None) -> str:
    umgebung = {**os.environ, "GIT_TERMINAL_PROMPT": "0", **(env or {})}
    try:
        # Bytes statt text=True: Unter Windows würde stdin sonst CRLF bekommen,
        # und update-index ignoriert solche Pfade still (leerer Commit).
        r = subprocess.run(
            ["git", *args], cwd=repo, env=umgebung, capture_output=True,
            input=eingabe.encode("utf-8") if eingabe is not None else None,
            timeout=config.TESTS_GIT_TIMEOUT_S,
        )
    except FileNotFoundError as e:
        raise GitFehler("git ist nicht installiert") from e
    except subprocess.TimeoutExpired as e:
        raise GitFehler(f"git {args[0]} antwortet nicht (Netz?)") from e
    aus = r.stdout.decode("utf-8", "replace")
    if r.returncode != 0:
        meldung = (r.stderr.decode("utf-8", "replace") or aus).strip().splitlines()
        raise GitFehler(f"git {args[0]}: {meldung[-1] if meldung else r.returncode}")
    return aus


def git_wurzel(ordner: Path) -> Path:
    return Path(_git(ordner, ["rev-parse", "--show-toplevel"]).strip())


def https_url(url: str) -> str:
    """git@github.com:a/b.git -> https://github.com/a/b.git (das Token geht nur über https)."""
    m = re.match(r"^(?:ssh://)?git@([^:/]+)[:/](.+)$", url)
    return f"https://{m.group(1)}/{m.group(2)}" if m else url


def _auth(url: str, token: str) -> dict:
    """Token als HTTP-Kopfzeile über die Umgebung: So steht es weder in der
    Befehlszeile (ps) noch in einer Fehlermeldung oder der Git-Konfiguration."""
    if not token or not url.startswith("https://"):
        return {}
    basis = base64.b64encode(f"x-access-token:{token}".encode()).decode()
    return {"GIT_CONFIG_COUNT": "1", "GIT_CONFIG_KEY_0": "http.extraHeader",
            "GIT_CONFIG_VALUE_0": f"Authorization: Basic {basis}"}


def hochladen_git(repo: Path, url: str, token: str, branch: str, ordner: list[Path]) -> str:
    """Ordner als ``tests/<name>/`` in einem Commit auf ``branch`` schieben.

    Ein eigener Index (Temp-Datei) und commit-tree statt checkout: Der Pi
    bleibt auf seinem Branch, sein Arbeitsbaum unberührt. Wurde der Branch
    zwischendurch woanders weitergeschoben, wird einmal neu aufgesetzt.
    """
    auth = _auth(url, token)
    for versuch in range(2):
        vorhanden = _git(repo, ["ls-remote", url, f"refs/heads/{branch}"], env=auth).strip()
        eltern = None
        if vorhanden:
            _git(repo, ["fetch", "--no-tags", url,
                        f"+refs/heads/{branch}:refs/rosmaster/{branch}"], env=auth)
            eltern = _git(repo, ["rev-parse", f"refs/rosmaster/{branch}"]).strip()
        with tempfile.TemporaryDirectory() as tmp:
            index = {"GIT_INDEX_FILE": str(Path(tmp) / "index")}
            _git(repo, ["read-tree", eltern] if eltern else ["read-tree", "--empty"], env=index)
            dateien = [(o, f) for o in ordner for f in sorted(o.rglob("*"))
                       if f.is_file() and f.name != "hochgeladen.json"]
            shas = _git(repo, ["hash-object", "-w", "--stdin-paths"],
                        eingabe="\n".join(str(f) for _, f in dateien) + "\n").split()
            zeilen = [f"100644 {sha}\ttests/{o.name}/{f.relative_to(o).as_posix()}"
                      for (o, f), sha in zip(dateien, shas)]
            _git(repo, ["update-index", "--add", "--index-info"], env=index, eingabe="\n".join(zeilen) + "\n")
            baum = _git(repo, ["write-tree"], env=index).strip()
        wer = {"GIT_AUTHOR_NAME": config.TESTS_GIT_AUTOR, "GIT_AUTHOR_EMAIL": config.TESTS_GIT_EMAIL,
               "GIT_COMMITTER_NAME": config.TESTS_GIT_AUTOR, "GIT_COMMITTER_EMAIL": config.TESTS_GIT_EMAIL}
        nachricht = "Testdaten: " + ", ".join(o.name for o in ordner)
        commit = _git(repo, ["commit-tree", baum, *(["-p", eltern] if eltern else []), "-m", nachricht],
                      env=wer).strip()
        try:
            _git(repo, ["push", url, f"{commit}:refs/heads/{branch}"], env=auth)
            return commit
        except GitFehler as e:
            if versuch == 0 and ("rejected" in str(e) or "fetch first" in str(e) or "non-fast-forward" in str(e)):
                log.info("Branch %s wurde inzwischen weitergeschoben, setze neu auf", branch)
                continue
            raise
    raise GitFehler("Hochladen nach zweitem Versuch abgelehnt")
