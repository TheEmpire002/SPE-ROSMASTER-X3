"""Prüft den Healthbot (Zustandserkennung) -- ohne Roboter.

    python Marlon/zustandserkennung/test_zustand.py

Eine gedachte Welt ersetzt Antrieb, Lidar, Kamera, Heimfahrt, Lautsprecher
und Mikrofon. Die Person liegt dort, wo auch die Mock-Simulation sie
hinlegt (``simulation.STANDARD_ORT``), als drei Kreise für den Lidar und
als liegendes Skelett für die Kamera.

Fälle:

* Bewertung: ``antwort_deuten`` und ``liegend`` (stehend/liegend);
* liegt regungslos, antwortet nicht -> zweimal anstupsen, keine Reaktion
  -> Alarm, Ansage, heim, Musik; dabei nie tiefer als
  ``HEALTH_STUPSEN_TIEFE_M`` in die Person;
* antwortet "ja", auf die Nachfrage "nein alles gut" -> ansprechbar, heim;
* bewegt sich, antwortet nicht -> anstupsen -> Reaktion -> "hilfe" -> Alarm;
* keine Person -> Warnung, heim, keine Musik;
* Anstupsen abgeschaltet -> Alarm ohne Stups;
* Person steht und antwortet nicht -> unauffällig, kein Stups;
* Abbrechen mitten im Ablauf -> hält an, Heimfahrt abgebrochen;
* Wache: liegende Person selbst erkannt -> Prüfung startet;
* Person liegt unter der Lidar-Ebene, nur der Fahrschutz (Tiefe,
  Gedächtnis) sieht sie -> fährt trotzdem heran und stupst (Testprotokoll).
"""

import asyncio
import json
import math
import os
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety.fahrschutz import Pruefung  # noqa: E402
from rosmaster_assist.sensors import pose as pose_modul  # noqa: E402
from rosmaster_assist.sensors.lidar import Scan  # noqa: E402
from rosmaster_assist.zustandserkennung import ablauf as ab  # noqa: E402
from rosmaster_assist.zustandserkennung import bewertung as bw  # noqa: E402
from rosmaster_assist.zustandserkennung import simulation as sim  # noqa: E402

fehler = 0


def pruefe(name: str, bedingung: bool, info: str = "") -> None:
    global fehler
    if bedingung:
        print(f"  ok    {name}")
    else:
        fehler += 1
        print(f"  FEHLER {name} {info}")


# ---------------------------------------------------------------- Welt

class Welt:
    """Roboter (x, y, theta) und Person; integriert bei jedem Zugriff."""

    RAUM = (-3.0, 5.0, -3.0, 3.0)

    def __init__(self, person: str | None = "liegend", bewegt: bool = False, lidar_sieht: bool = True) -> None:
        self.x = self.y = self.th = 0.0
        self.vx = self.omega = 0.0
        self.letzter_befehl = time.monotonic()
        self.zuletzt = time.monotonic()
        self.person = person
        self.bewegt = bewegt
        # False: Die Person liegt flacher als die Lidar-Ebene. Der Scan sieht
        # über sie hinweg, nur der Fahrschutz (Tiefe, Gedächtnis) sieht sie.
        self.lidar_sieht = lidar_sieht
        ox, oy = sim.STANDARD_ORT
        if person == "liegend":
            self.kreise = [(ox + dx, oy + dy, r) for dx, dy, r in sim._KREISE]
        elif person == "stehend":
            self.kreise = [(ox, oy, 0.15)]  # Beine zusammen, von oben
        else:
            self.kreise = []

    def aktualisieren(self) -> None:
        jetzt = time.monotonic()
        dt = jetzt - self.zuletzt
        self.zuletzt = jetzt
        self.th += self.omega * dt
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        if jetzt - self.letzter_befehl > 0.5:  # Totmannschaltung
            self.vx = self.omega = 0.0

    def setzen(self, x: float, y: float, th: float) -> None:
        self.aktualisieren()
        self.x, self.y, self.th = x, y, th
        self.vx = self.omega = 0.0

    def luft_vorn(self) -> float:
        """Kleinster Abstand Lidar -> Körperkante vor dem Roboter (±35°)."""
        self.aktualisieren()
        kleinster = math.inf
        for cx, cy, r in self.kreise:
            dx, dy = cx - self.x, cy - self.y
            w = math.degrees(math.atan2(dy, dx) - self.th)
            w = (w + 180.0) % 360.0 - 180.0
            if abs(w) <= 35.0:
                kleinster = min(kleinster, math.hypot(dx, dy) - r)
        return kleinster

    def scan(self) -> Scan:
        self.aktualisieren()
        winkel = np.arange(0.0, 360.0, 1.0)
        a = self.th + np.radians(winkel)
        ux, uy = np.cos(a), np.sin(a)
        x0, x1, y0, y1 = self.RAUM
        with np.errstate(divide="ignore", invalid="ignore"):
            tx = np.where(ux > 0, (x1 - self.x) / ux, (x0 - self.x) / ux)
            ty = np.where(uy > 0, (y1 - self.y) / uy, (y0 - self.y) / uy)
        r = np.minimum(np.abs(tx), np.abs(ty))
        for cx, cy, rad in (self.kreise if self.lidar_sieht else []):
            # Strahl x + t*u trifft Kreis: t^2 - 2 t (u.d) + |d|^2 - rad^2 = 0
            dx, dy = cx - self.x, cy - self.y
            b = ux * dx + uy * dy
            c = dx * dx + dy * dy - rad * rad
            disk = b * b - c
            t = np.where(disk >= 0, b - np.sqrt(np.maximum(disk, 0.0)), np.inf)
            t = np.where(t > 0, t, np.inf)
            r = np.minimum(r, t)
        return Scan(list(winkel), list(r))


class Watchdog:
    def __init__(self, welt: Welt) -> None:
        self.welt = welt
        self.letzte_pruefung = None
        self.stopps = 0
        self.kleinste_luft = math.inf
        self.befehle = 0

    def submit(self, cmd: DriveCommand, quelle: str = "app", rand_m: float | None = None) -> DriveCommand:
        w = self.welt
        luft = w.luft_vorn()
        rand = config.FAHRSCHUTZ_RAND_M if rand_m is None else rand_m
        vx = cmd.vx
        frei = luft - config.ROBOTER_VORN_M - rand if vx > 0 and math.isfinite(luft) else None
        grund = None
        if vx > 0 and frei is not None and frei < 0.0:
            vx, grund = 0.0, "Hindernis im Weg"
        befehl = DriveCommand(vx=vx, vy=cmd.vy, omega=cmd.omega)
        # Wie der echte Fahrschutz: Weg bis zur Berührung, Rand eingerechnet.
        self.letzte_pruefung = Pruefung(befehl, 1.0 if grund is None else 0.0,
                                        None if frei is None else max(0.0, frei), grund)
        w.vx, w.omega = vx, cmd.omega
        w.letzter_befehl = time.monotonic()
        self.kleinste_luft = min(self.kleinste_luft, luft)
        self.befehle += 1
        return befehl

    def stop(self) -> None:
        self.welt.aktualisieren()
        self.welt.vx = self.welt.omega = 0.0
        self.stopps += 1


class Odometrie:
    def __init__(self, welt: Welt) -> None:
        self.welt = welt

    def pose(self):
        self.welt.aktualisieren()
        return self.welt.x, self.welt.y, math.degrees(self.welt.th)

    def geschwindigkeit(self):
        self.welt.aktualisieren()
        return self.welt.vx, 0.0, self.welt.omega


class Lidar:
    def __init__(self, welt: Welt) -> None:
        self.welt = welt

    def latest(self):
        return self.welt.scan()


class Pose:
    verfuegbar = True

    def __init__(self, welt: Welt) -> None:
        self.welt = welt
        self.fov_deg = config.CAMERA_FOV_DEG
        self.aktiviert: list[tuple[bool, str]] = []

    def aktivieren(self, an: bool, wer: str) -> None:
        self.aktiviert.append((an, wer))

    def personen(self) -> list[dict]:
        w = self.welt
        w.aktualisieren()
        if w.person is None:
            return []
        ox, oy = sim.STANDARD_ORT
        winkel = math.degrees(math.atan2(oy - w.y, ox - w.x) - w.th)
        winkel = (winkel + 180.0) % 360.0 - 180.0
        if abs(winkel) > self.fov_deg / 2:
            return []
        mx = 0.5 - winkel / self.fov_deg
        if w.person == "liegend":
            abstand = max(0.05, min(math.hypot(cx - w.x, cy - w.y) - r for cx, cy, r in w.kreise))
            return [pose_modul._sim_liegend(mx, abstand, w.bewegt, time.monotonic(), self.fov_deg)]
        kp, box = pose_modul._sim_koerper(mx, 0.2, 0.6, 0.03)
        return [pose_modul._person(box, [[x, y, 0.8] for x, y in kp], 0.8,
                                   config.CAMERA_WIDTH / config.CAMERA_HEIGHT, self.fov_deg)]


class Heimfahrt:
    """Fährt nicht, sondern setzt den Roboter nach kurzer Zeit ans Ziel.

    Andere Ziele als den Start erreicht sie wie die echte mit Toleranz:
    0,9 m davor, mit Blick zur Stelle.
    """

    def __init__(self, welt: Welt) -> None:
        self.welt = welt
        self.ziele: list[tuple[float, float, str]] = []
        self.abgebrochen: list[str] = []
        self._ziel = None
        self._ende = 0.0
        self.letzter_grund = None

    def start_zu(self, x, y, toleranz=None, name="Startpunkt", blockiert_reicht_m=0.0) -> bool:
        if self.aktiv:
            return False
        self.ziele.append((x, y, name))
        self._ziel = (x, y)
        self._ende = time.monotonic() + 0.1
        self.letzter_grund = None
        return True

    @property
    def aktiv(self) -> bool:
        if self._ziel is None:
            return False
        if time.monotonic() < self._ende:
            return True
        x, y = self._ziel
        w = self.welt
        if (x, y) == (0.0, 0.0):
            w.setzen(0.0, 0.0, 0.0)
        else:
            w.aktualisieren()
            d = math.hypot(x - w.x, y - w.y)
            ux, uy = (x - w.x) / d, (y - w.y) / d
            w.setzen(x - 0.9 * ux, y - 0.9 * uy, math.atan2(uy, ux))
        self._ziel = None
        self.letzter_grund = "angekommen"
        return False

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        self.abgebrochen.append(grund)
        self._ziel = None
        self.letzter_grund = grund


class Sprache:
    verfuegbar = True

    def __init__(self, antworten: dict[str, list[str]]) -> None:
        self.antworten = antworten
        self.eintraege: list[tuple[float, str]] = []

    def gehoert_seit(self, seit: float) -> list[str]:
        jetzt = time.time()
        return [t for z, t in self.eintraege if seit <= z <= jetzt]


class Stimme:
    verfuegbar = True
    spricht = False

    def __init__(self, sprache: Sprache) -> None:
        self.sprache = sprache
        self.gesagt: list[str] = []

    def sagen(self, text: str, vorrang: bool = False) -> None:
        self.gesagt.append(text)
        # Die Antwort kommt kurz nach der Frage -- nach dem Beginn des Zuhörens.
        for t in self.sprache.antworten.get(text, []):
            self.sprache.eintraege.append((time.time() + 0.45, t))


class Mukke:
    def __init__(self) -> None:
        self.gespielt: list[tuple[str, str]] = []
        self.laeuft = False
        self.stopps = 0

    def datei_abspielen(self, pfad: str, titel: str) -> None:
        self.gespielt.append((pfad, titel))
        self.laeuft = True

    def stoppen(self) -> None:
        self.laeuft = False
        self.stopps += 1


class Aufbau:
    def __init__(self, person: str | None = "liegend", bewegt: bool = False, antworten=None,
                 lidar_sieht: bool = True) -> None:
        self.welt = Welt(person, bewegt, lidar_sieht)
        self.watchdog = Watchdog(self.welt)
        self.homing = Heimfahrt(self.welt)
        self.pose = Pose(self.welt)
        self.sprache = Sprache(antworten or {})
        self.stimme = Stimme(self.sprache)
        self.mukke = Mukke()
        self.meldungen: list[dict] = []

        async def melden(nachricht):
            self.meldungen.append(nachricht)

        self.hb = ab.Healthbot(
            self.watchdog, Odometrie(self.welt), self.homing, lidar=Lidar(self.welt),
            pose=self.pose, stimme=self.stimme, sprache=self.sprache, mukke=self.mukke,
            melden=melden,
        )

    def stufen(self) -> list[str]:
        return [m["level"] for m in self.meldungen if m.get("type") == "alert"]

    async def bis_fertig(self, frist_s: float = 60.0) -> None:
        ende = time.monotonic() + frist_s
        while self.hb.aktiv and time.monotonic() < ende:
            await asyncio.sleep(0.05)


def check(hb, name: str) -> str:
    return next(c["zustand"] for c in hb.status()["checks"] if c["name"] == name)


# ---------------------------------------------------------------- Fälle

def test_bewertung() -> None:
    print("Bewertung")
    pruefe("„keine Hilfe“ ist nein", bw.antwort_deuten(["nein danke keine hilfe"]) == "nein")
    pruefe("„nein hilf mir“ ist Hilfe", bw.antwort_deuten(["nein", "hilf mir"]) == "hilfe")
    pruefe("„ja“ ist Hilfe", bw.antwort_deuten(["ja"]) == "hilfe")
    pruefe("Unklares ist None", bw.antwort_deuten(["der hund bellt"]) is None)
    kp, box = pose_modul._sim_koerper(0.5, 0.2, 0.6, 0.03)
    stehend = pose_modul._person(box, [[x, y, 0.8] for x, y in kp], 0.8, 4 / 3, 60)
    liegend = pose_modul._sim_liegend(0.5, 1.0, False, 0.0, 60)
    pruefe("stehend ist nicht liegend", bw.liegend(stehend)["zustand"] == bw.OK, str(bw.liegend(stehend)))
    pruefe("liegend ist liegend", bw.liegend(liegend)["zustand"] == bw.AUFFAELLIG, str(bw.liegend(liegend)))
    pruefe("keine Antwort ohne Mikrofon ist unbekannt", bw.antwort(None)["zustand"] == bw.UNBEKANNT)


async def fall_regungslos() -> None:
    print("Liegt regungslos, antwortet nicht")
    a = Aufbau("liegend")
    pruefe("startet", a.hb.starten(*sim.STANDARD_ORT) is None)
    pruefe("zweiter Start abgelehnt", a.hb.starten() == "läuft schon")
    await a.bis_fertig()
    s = a.hb.status()
    pruefe("Zustand hilfe", s["zustand"] == ab.HILFE, s["zustand"] + " / " + s["schritt"])
    pruefe("Alarm gesetzt", s["alarm"])
    pruefe("Notfall-Meldung", "emergency" in a.stufen(), str(a.stufen()))
    pruefe("Urteil notfall", (s["urteil"] or {}).get("stufe") == "notfall", str(s["urteil"]))
    pruefe("zweimal gestupst", len(s["stupser"]) == config.HEALTH_STUPSEN_ANZAHL, str(s["stupser"]))
    pruefe("Stupser enden an Person oder Weggrenze",
           all(st["grund"] in ("Kontakt", ab.FAHRSCHUTZ_HAELT, "Weggrenze") for st in s["stupser"]),
           str(s["stupser"]))
    pruefe("keine Reaktion bemerkt", not any(st["reaktion"] for st in s["stupser"]))
    grenze = config.ROBOTER_VORN_M - config.HEALTH_STUPSEN_TIEFE_M - 0.01
    pruefe("nie tiefer als die Stupstiefe in die Person", a.watchdog.kleinste_luft >= grenze,
           f"{a.watchdog.kleinste_luft:.3f} < {grenze:.3f}")
    pruefe("kam überhaupt nah heran", a.watchdog.kleinste_luft <= config.ROBOTER_VORN_M + 0.02,
           f"{a.watchdog.kleinste_luft:.3f}")
    pruefe("vier Zeichen geprüft", all(c["zustand"] != bw.UNBEKANNT for c in s["checks"]), str(s["checks"]))
    pruefe("alle vier auffällig", all(c["zustand"] == bw.AUFFAELLIG for c in s["checks"]),
           str([(c["name"], c["zustand"]) for c in s["checks"]]))
    pruefe("Hilfe angesagt", config.HEALTH_ANSAGE_HILFE in a.stimme.gesagt)
    pruefe("Musik gespielt", len(a.mukke.gespielt) == 1, str(a.mukke.gespielt))
    pruefe("Musikdatei gibt es", bool(a.mukke.gespielt) and os.path.isfile(a.mukke.gespielt[0][0]),
           str(a.mukke.gespielt))
    pruefe("erst zur Stelle, dann heim", [z[:2] for z in a.homing.ziele] == [sim.STANDARD_ORT, (0.0, 0.0)],
           str(a.homing.ziele))
    pruefe("steht am Start", math.hypot(a.welt.x, a.welt.y) < 0.01)
    pruefe("steht still", a.welt.vx == 0.0 and a.welt.omega == 0.0)
    pruefe("Pose wieder freigegeben", a.pose.aktiviert[-1] == (False, "healthbot"), str(a.pose.aktiviert))
    pruefe("Status ist JSON", bool(json.dumps(s)))
    reihenfolge = [e["zustand"] for e in s["timeline"]]
    pruefe("Ablauf in der richtigen Reihenfolge",
           [z for i, z in enumerate(reihenfolge) if i == 0 or reihenfolge[i - 1] != z]
           == [ab.ANFAHRT, ab.GESAMTBILD, ab.ANSPRECHEN, ab.ANSTUPSEN, ab.HILFE], str(reihenfolge))


async def fall_ansprechbar() -> None:
    print("Antwortet, braucht keine Hilfe")
    a = Aufbau("liegend", antworten={config.HEALTH_FRAGE: ["ja"], config.HEALTH_FRAGE_HILFE: ["nein alles gut"]})
    a.hb.starten(*sim.STANDARD_ORT)
    await a.bis_fertig()
    s = a.hb.status()
    pruefe("Zustand ansprechbar", s["zustand"] == ab.ANSPRECHBAR, s["zustand"] + " / " + s["schritt"])
    pruefe("Antwort ok", check(a.hb, "antwort") == bw.OK)
    pruefe("nicht gestupst", not s["stupser"])
    pruefe("kein Alarm", not s["alarm"] and "emergency" not in a.stufen())
    pruefe("nachgefragt", config.HEALTH_FRAGE_HILFE in a.stimme.gesagt)
    pruefe("keine Musik", not a.mukke.gespielt)
    pruefe("heim", a.homing.ziele[-1][:2] == (0.0, 0.0))


async def fall_bewegt() -> None:
    print("Bewegt sich, antwortet erst nach dem Stups")
    a = Aufbau("liegend", bewegt=True, antworten={config.HEALTH_FRAGE_HILFE: ["hilfe"]})
    a.hb.starten(*sim.STANDARD_ORT)
    await a.bis_fertig()
    s = a.hb.status()
    pruefe("bewegt sich erkannt", check(a.hb, "regungslos") == bw.OK)
    pruefe("einmal gestupst, Reaktion", len(s["stupser"]) == 1 and s["stupser"][0]["reaktion"], str(s["stupser"]))
    pruefe("nachgefragt", config.HEALTH_FRAGE_HILFE in a.stimme.gesagt)
    pruefe("Zustand hilfe", s["zustand"] == ab.HILFE, s["zustand"] + " / " + s["schritt"])
    pruefe("Grund: bittet um Hilfe", (s["urteil"] or {}).get("text") == "bittet um Hilfe", str(s["urteil"]))
    pruefe("Musik gespielt", len(a.mukke.gespielt) == 1)


async def fall_keine_person() -> None:
    print("Keine Person da")
    a = Aufbau(None)
    a.hb.starten()
    await a.bis_fertig()
    s = a.hb.status()
    pruefe("Zustand keine_person", s["zustand"] == ab.KEINE_PERSON, s["zustand"] + " / " + s["schritt"])
    pruefe("Warnung", "warning" in a.stufen(), str(a.stufen()))
    pruefe("hat sich gedreht", a.watchdog.befehle > 5)
    pruefe("heim", a.homing.ziele == [(0.0, 0.0, "Startpunkt")], str(a.homing.ziele))
    pruefe("keine Musik", not a.mukke.gespielt)


async def fall_ohne_stupsen() -> None:
    print("Anstupsen abgeschaltet")
    alt = config.HEALTH_STUPSEN_AKTIV
    config.HEALTH_STUPSEN_AKTIV = False
    try:
        a = Aufbau("liegend")
        a.hb.starten(*sim.STANDARD_ORT)
        await a.bis_fertig()
        s = a.hb.status()
        pruefe("Zustand hilfe", s["zustand"] == ab.HILFE, s["zustand"])
        pruefe("kein Stups", not s["stupser"])
        pruefe("Status meldet es", s["stupsen_aktiv"] is False)
        pruefe("nicht nah herangefahren", a.watchdog.kleinste_luft > 0.5, f"{a.watchdog.kleinste_luft:.2f}")
    finally:
        config.HEALTH_STUPSEN_AKTIV = alt


async def fall_stehend() -> None:
    print("Steht und antwortet nicht")
    a = Aufbau("stehend")
    a.hb.starten(*sim.STANDARD_ORT)
    await a.bis_fertig()
    s = a.hb.status()
    pruefe("Zustand unauffällig", s["zustand"] == ab.UNAUFFAELLIG, s["zustand"] + " / " + s["schritt"])
    pruefe("liegend ok", check(a.hb, "liegend") == bw.OK)
    pruefe("Boden ok", check(a.hb, "boden") == bw.OK, str(s["checks"][3]))
    pruefe("kein Stups", not s["stupser"])
    pruefe("kein Alarm", not s["alarm"])


async def fall_abbrechen() -> None:
    print("Abbrechen")
    a = Aufbau("liegend")
    a.hb.starten(*sim.STANDARD_ORT)
    await asyncio.sleep(0.6)
    stopps = a.watchdog.stopps
    a.hb.abbrechen("Not-Stopp")
    await asyncio.sleep(0.2)
    s = a.hb.status()
    pruefe("nicht mehr aktiv", not s["aktiv"])
    pruefe("Zustand abgebrochen", s["zustand"] == ab.ABGEBROCHEN, s["zustand"])
    pruefe("angehalten", a.watchdog.stopps > stopps and a.welt.vx == 0.0 and a.welt.omega == 0.0)
    pruefe("Heimfahrt abgebrochen", a.homing.abgebrochen == ["Not-Stopp"], str(a.homing.abgebrochen))
    pruefe("Pose wieder freigegeben", a.pose.aktiviert[-1] == (False, "healthbot"))


async def fall_alarm_beenden() -> None:
    # Testprotokoll: Nach dem Stopp lief die Alarm-Musik weiter, und der
    # Roboter blieb im Zustand hilfe. Nach _hilfe ist der Ablauf schon
    # vorbei -- abbrechen() allein tat dann nichts.
    print("Stopp beendet den Alarm")
    a = Aufbau("liegend")
    a.hb.starten(*sim.STANDARD_ORT)
    a.hb._task.cancel()
    await asyncio.sleep(0.05)
    await a.hb._hilfe("Test")
    s = a.hb.status()
    pruefe("Alarm läuft, Ablauf vorbei", s["alarm"] and not s["aktiv"] and a.mukke.laeuft, str(s["alarm"]))
    await a.hb.alarm_beenden("Not-Stopp")
    s = a.hb.status()
    pruefe("Musik aus", not a.mukke.laeuft and a.mukke.stopps == 1, str(a.mukke.stopps))
    pruefe("Alarm aus", not s["alarm"])
    pruefe("Zustand bereit", s["zustand"] == ab.BEREIT, s["zustand"] + " / " + s["schritt"])
    pruefe("Zeitleiste vermerkt es", "Alarm beendet" in s["timeline"][-1]["text"], s["timeline"][-1]["text"])
    await a.hb.alarm_beenden("Not-Stopp")
    pruefe("zweiter Stopp stoppt nichts mehr", a.mukke.stopps == 1, str(a.mukke.stopps))
    pruefe("danach wieder startbar", a.hb.starten(*sim.STANDARD_ORT) is None)
    await asyncio.sleep(0.3)
    stopps = a.watchdog.stopps
    await a.hb.alarm_beenden("Not-Stopp")
    s = a.hb.status()
    pruefe("laufende Prüfung abgebrochen", not s["aktiv"] and s["zustand"] == ab.ABGEBROCHEN, s["zustand"])
    pruefe("dabei angehalten", a.watchdog.stopps > stopps)
    pruefe("fremde Musik nicht angefasst", a.mukke.stopps == 1, str(a.mukke.stopps))


async def fall_unter_lidar() -> None:
    print("Liegt unter der Lidar-Ebene, nur der Fahrschutz sieht sie")
    a = Aufbau("liegend", lidar_sieht=False)
    pruefe("startet", a.hb.starten(*sim.STANDARD_ORT) is None)
    await a.bis_fertig(90.0)
    s = a.hb.status()
    texte = " | ".join(e["text"] for e in s["timeline"])
    pruefe("nicht am Heranfahren gescheitert", "nicht nah genug" not in texte, texte)
    pruefe("zweimal gestupst", len(s["stupser"]) == config.HEALTH_STUPSEN_ANZAHL, str(s["stupser"]))
    pruefe("Stupser melden den Fahrschutz als Ende",
           all(st["grund"] == ab.FAHRSCHUTZ_HAELT for st in s["stupser"]), str(s["stupser"]))
    pruefe("kam nah heran", a.watchdog.kleinste_luft <= config.ROBOTER_VORN_M + 0.02,
           f"{a.watchdog.kleinste_luft:.3f}")
    grenze = config.ROBOTER_VORN_M - config.HEALTH_STUPSEN_TIEFE_M - 0.01
    pruefe("nie tiefer als die Stupstiefe in die Person", a.watchdog.kleinste_luft >= grenze,
           f"{a.watchdog.kleinste_luft:.3f} < {grenze:.3f}")
    pruefe("Stupsweg begrenzt, obwohl der Lidar weit Luft sieht",
           all(st["weg_m"] <= config.HEALTH_STUPSEN_TIEFE_M + 0.12 for st in s["stupser"]), str(s["stupser"]))
    pruefe("Zeitleiste nennt den Fahrschutz", "Fahrschutz" in texte, texte)
    pruefe("Zustand hilfe", s["zustand"] == ab.HILFE, s["zustand"])


async def fall_wache() -> None:
    print("Wache erkennt liegende Person")
    a = Aufbau("liegend")
    ox, oy = sim.STANDARD_ORT
    a.welt.setzen(ox - 1.2, oy, 0.0)  # Person 1,2 m voraus
    pruefe("Wache an", a.hb.wache_setzen(True) is None and a.hb.wache)
    ende = time.monotonic() + config.HEALTH_AUTO_S + 3.0
    while not a.hb.aktiv and time.monotonic() < ende:
        await asyncio.sleep(0.05)
    s = a.hb.status()
    pruefe("Prüfung gestartet", s["aktiv"])
    pruefe("Quelle selbst erkannt", (s["ziel"] or {}).get("quelle") == "selbst erkannt", str(s["ziel"]))
    if s["ziel"]:
        pruefe("Ziel an der Person", math.hypot(s["ziel"]["x"] - ox, s["ziel"]["y"] - oy) < 0.5, str(s["ziel"]))
    await asyncio.sleep(0.3)
    pruefe("Wache danach aus", not a.hb.wache)
    a.hb.abbrechen("Test")
    await asyncio.sleep(0.1)
    pruefe("Wache-Pose freigegeben", (False, "healthbot-wache") in a.pose.aktiviert)


async def fall_ohne_kamera() -> None:
    print("Ohne Kamera")
    a = Aufbau("liegend")
    a.pose.verfuegbar = False
    pruefe("Start abgelehnt", a.hb.starten() is not None)
    pruefe("Wache abgelehnt", a.hb.wache_setzen(True) is not None)


async def alle() -> None:
    await fall_ohne_kamera()
    await fall_regungslos()
    await fall_ansprechbar()
    await fall_bewegt()
    await fall_keine_person()
    await fall_ohne_stupsen()
    await fall_stehend()
    await fall_abbrechen()
    await fall_alarm_beenden()
    await fall_unter_lidar()
    await fall_wache()


def main() -> None:
    # Wartezeiten kürzen, damit der Test schnell bleibt; die Logik bleibt gleich.
    alt = {n: getattr(config, n) for n in ("HEALTH_BEOBACHTEN_S", "HEALTH_ANTWORT_S", "HEALTH_SUCHEN_S", "HEALTH_AUTO_S")}
    alt_reaktion = ab._REAKTION_S
    config.HEALTH_BEOBACHTEN_S = 1.0
    config.HEALTH_ANTWORT_S = 0.6
    config.HEALTH_SUCHEN_S = 3.0
    config.HEALTH_AUTO_S = 0.5
    ab._REAKTION_S = 0.8
    try:
        test_bewertung()
        asyncio.run(alle())
    finally:
        for n, v in alt.items():
            setattr(config, n, v)
        ab._REAKTION_S = alt_reaktion
    print()
    print("alle Fälle bestehen" if fehler == 0 else f"{fehler} Fehler")
    sys.exit(1 if fehler else 0)


if __name__ == "__main__":
    main()
