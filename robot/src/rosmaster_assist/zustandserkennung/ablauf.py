"""Ablauf des Healthbots: hinfahren, Gesamtbild, ansprechen, anstupsen, Hilfe.

Zustände (``status()["zustand"]``)::

    bereit -> anfahrt -> gesamtbild -> ansprechen -+-> nachfrage --> hilfe | ansprechbar
                                                   +-> anstupsen -+
                                                   +-> unauffaellig (heim)

Jeder Schritt landet mit Uhrzeit und Begründung in ``timeline``; die vier
Zeichen stehen als Ampel in ``checks``. Die Web-App zeigt beides, damit
nachvollziehbar bleibt, *warum* der Roboter Alarm schlägt.

Fahren nur über ``SafetyWatchdog.submit(quelle="autonom")``: Tempolimit,
Totmannschaltung und Fahrschutz gelten auch hier. Beim Anstupsen wird der
Rand des Fahrschutzes auf ``HEALTH_STUPSEN_RAND_M`` gesenkt -- sonst hielte
der Roboter immer einige Zentimeter vor der Person und berührte sie nie.
"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import time

from .. import config
from ..protocol import DriveCommand, alert_message
from . import bewertung as bw

log = logging.getLogger(__name__)

BEREIT = "bereit"
ANFAHRT = "anfahrt"
GESAMTBILD = "gesamtbild"
ANSPRECHEN = "ansprechen"
ANSTUPSEN = "anstupsen"
NACHFRAGE = "nachfrage"
HILFE = "hilfe"
ANSPRECHBAR = "ansprechbar"
UNAUFFAELLIG = "unauffaellig"
KEINE_PERSON = "keine_person"
ABGEBROCHEN = "abgebrochen"

_TAKT_S = 0.1  # weit unter der Totmannzeit (0,5 s) des Watchdogs
_SCAN_MAX_ALTER_S = 1.0
_DREHEN_MAX_S = 10.0
_HERAN_MAX_S = 15.0
_ZURUECK_MAX_S = 4.0
_REAKTION_S = 2.5  # nach jedem Stups so lange auf Bewegung oder Worte achten
_SPRECHEN_MAX_S = 15.0
_MUSIK_PFAD = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")


class Healthbot:
    def __init__(self, watchdog, odometry, homing, lidar=None, pose=None,
                 stimme=None, sprache=None, mukke=None, melden=None) -> None:
        """``homing`` ist eine eigene ``Homing``-Instanz (nicht die der
        Heimfahrt), damit deren Anzeige nicht von Healthbot-Fahrten
        überschrieben wird. ``melden(nachricht)`` ist ein async Callback,
        der eine Nachricht an alle Clients schickt."""
        self._watchdog = watchdog
        self._odometry = odometry
        self._homing = homing
        self._lidar = lidar
        self._pose = pose
        self._stimme = stimme
        self._sprache = sprache
        self._mukke = mukke
        self._melden = melden
        self._task: asyncio.Task | None = None
        self._wache_task: asyncio.Task | None = None
        self._wache_seit: float | None = None
        self._zustand = BEREIT
        self._schritt = "wartet auf Meldung"
        self._timeline: list[dict] = []
        self._checks = bw.leer()
        self._urteil: dict | None = None
        self._ziel: dict | None = None
        self._alarm = False
        self._person: dict | None = None
        self._stupser: list[dict] = []

    # ------------------------------------------------------------------ Status

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def wache(self) -> bool:
        return self._wache_task is not None and not self._wache_task.done()

    def status(self) -> dict:
        p = self._person
        return {
            "aktiv": self.aktiv,
            "zustand": self._zustand,
            "schritt": self._schritt,
            "timeline": list(self._timeline),
            "checks": [dict(self._checks[n]) for n in bw.ZEICHEN],
            "urteil": self._urteil,
            "ziel": self._ziel,
            "alarm": self._alarm,
            "stupser": list(self._stupser),
            "wache": self.wache,
            "stupsen_aktiv": config.HEALTH_STUPSEN_AKTIV,
            "person": None if p is None else {"box": p["box"], "kp": p["kp"], "winkel": p["winkel"]},
        }

    def _notiz(self, text: str, zustand: str | None = None) -> None:
        if zustand is not None:
            self._zustand = zustand
        self._schritt = text
        self._timeline.append({"zeit": time.time(), "zustand": self._zustand, "text": text})
        log.info("HEALTHBOT [%s] %s", self._zustand, text)

    def _check(self, c: dict) -> None:
        self._checks[c["name"]] = c
        self._notiz(f"{c['titel']}: {c['text']}")

    async def _senden(self, stufe: str, text: str) -> None:
        if self._melden is None:
            return
        try:
            await self._melden(alert_message(stufe, text))
        except Exception:  # eine kaputte Verbindung darf den Ablauf nicht stoppen
            log.exception("HEALTHBOT: Meldung nicht gesendet")

    # --------------------------------------------------------------- Bedienung

    def starten(self, x: float | None = None, y: float | None = None, quelle: str = "eingabe") -> str | None:
        """Prüfung beginnen. Ohne Ziel: Person hier in der Nähe prüfen.

        Gibt None zurück oder den Grund, warum es nicht geht.
        """
        if not config.HEALTH_AKTIV:
            return "abgeschaltet (config.HEALTH_AKTIV)"
        if self.aktiv:
            return "läuft schon"
        if self._pose is None or not getattr(self._pose, "verfuegbar", False):
            return "keine Pose-Erkennung -- ohne Kamera kein Gesamtbild"
        self._timeline = []
        self._checks = bw.leer()
        self._urteil = None
        self._alarm = False
        self._person = None
        self._stupser = []
        self._ziel = None if x is None or y is None else {"x": round(float(x), 2), "y": round(float(y), 2), "quelle": quelle}
        self._task = asyncio.create_task(self._ablauf())
        return None

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        """Sofort beenden und anhalten. Auch aufrufen, wenn der Mensch eingreift."""
        if not self.aktiv:
            return
        self._task.cancel()
        self._homing.abbrechen(grund)
        self._watchdog.stop()
        self._notiz(f"abgebrochen: {grund}", ABGEBROCHEN)

    def wache_setzen(self, an: bool) -> str | None:
        """Selbst nach liegenden Personen Ausschau halten (fährt nicht los,
        solange keine gesehen wird). Gibt None oder den Grund zurück."""
        if not an:
            if self.wache:
                self._wache_task.cancel()
            self._wache_task = None
            if self._pose is not None:
                self._pose.aktivieren(False, "healthbot-wache")
            return None
        if not config.HEALTH_AUTO_ERKENNEN:
            return "abgeschaltet (config.HEALTH_AUTO_ERKENNEN)"
        if self._pose is None or not getattr(self._pose, "verfuegbar", False):
            return "keine Pose-Erkennung"
        if not self.wache:
            self._wache_task = asyncio.create_task(self._wache())
        return None

    def beenden(self) -> None:
        """Beim Herunterfahren des Servers."""
        self.wache_setzen(False)
        self.abbrechen("Server beendet")

    # ------------------------------------------------------------ Hilfsmittel

    def _fahren(self, vx: float = 0.0, omega: float = 0.0, rand_m: float | None = None) -> DriveCommand:
        return self._watchdog.submit(DriveCommand(vx=vx, omega=omega), quelle="autonom", rand_m=rand_m)

    def _punkte(self):
        """Aktueller Lidar-Scan als Punkte, oder None (kein/alter Scan)."""
        scan = self._lidar.latest() if self._lidar is not None else None
        if scan is None or time.monotonic() - scan.zeit > _SCAN_MAX_ALTER_S:
            return None
        return scan.punkte()

    def _gesehen(self) -> dict | None:
        personen = self._pose.personen() if self._pose is not None else []
        return personen[0] if personen else None

    def _abstand(self, person: dict | None) -> float | None:
        """Lidar-Abstand zur Person (nächster Punkt ihres Stücks), sonst
        das Nächste genau voraus."""
        punkte = self._punkte()
        if punkte is None:
            return None
        if person is not None:
            halbe = (person["box"][2] - person["box"][0]) * self._pose.fov_deg / 2
            stueck = bw.lidar_stueck(punkte, person["winkel"], halbe)
            if stueck is not None:
                return stueck[0]
        scan = self._lidar.latest()
        d = scan.min_abstand(0.0, 15.0) if scan is not None else math.inf
        return d if math.isfinite(d) else None

    def _strecke(self, start: tuple[float, float, float]) -> float:
        x, y, _ = self._odometry.pose()
        return math.hypot(x - start[0], y - start[1])

    async def _sprechen(self, text: str, vorrang: bool = False) -> None:
        """Sagen und warten, bis es gesagt ist -- sonst hört das Mikrofon
        beim Warten auf die Antwort den Roboter selbst."""
        s = self._stimme
        if s is None or not getattr(s, "verfuegbar", False):
            self._notiz(f"(kein Lautsprecher) „{text}“")
            return
        s.sagen(text, vorrang=vorrang)
        self._notiz(f"sagt: „{text}“")
        frist = time.monotonic() + _SPRECHEN_MAX_S
        await asyncio.sleep(0.3)  # bis die Ausgabe wirklich beginnt
        while s.spricht and time.monotonic() < frist:
            await asyncio.sleep(_TAKT_S)

    async def _zuhoeren(self, dauer: float) -> list[str] | None:
        """Was in ``dauer`` Sekunden gehört wird; None ohne Mikrofon."""
        s = self._sprache
        if s is None or not getattr(s, "verfuegbar", False):
            return None
        seit = time.time()
        await asyncio.sleep(dauer)
        return s.gehoert_seit(seit)

    async def _beobachten(self, dauer: float) -> list[dict]:
        """Bilder der Person über ``dauer`` Sekunden sammeln. Der Roboter
        steht dabei: Eigenbewegung sähe sonst wie Bewegung der Person aus."""
        self._watchdog.stop()
        verlauf: list[dict] = []
        zuletzt = None
        ende = time.monotonic() + dauer
        while time.monotonic() < ende:
            p = self._gesehen()
            if p is not None and p.get("zeit") != zuletzt:
                zuletzt = p.get("zeit")
                verlauf.append(p)
                self._person = p
            await asyncio.sleep(_TAKT_S)
        return verlauf

    async def _drehen_um(self, grad: float) -> None:
        """Um ``grad`` drehen (links positiv), nach Odometrie mit Zeitgrenze."""
        if abs(grad) < 10.0:
            return
        _, _, start = self._odometry.pose()
        omega = math.copysign(config.HEALTH_SUCHEN_OMEGA * 1.5, grad)
        frist = time.monotonic() + min(_DREHEN_MAX_S, abs(math.radians(grad) / omega) * 1.5 + 1.0)
        while time.monotonic() < frist:
            _, _, jetzt = self._odometry.pose()
            gedreht = (jetzt - start + 180.0) % 360.0 - 180.0
            if abs(gedreht) >= abs(grad) - 5.0:
                break
            self._fahren(omega=omega)
            await asyncio.sleep(_TAKT_S)
        self._watchdog.stop()

    async def _person_suchen(self) -> dict | None:
        """Drehen, bis eine Person mittig im Bild ist. None nach einer Runde."""
        frist = time.monotonic() + config.HEALTH_SUCHEN_S
        while time.monotonic() < frist:
            p = self._gesehen()
            if p is not None:
                self._person = p
                if abs(p["winkel"]) <= config.HEALTH_MITTIG_GRAD:
                    self._watchdog.stop()
                    return p
                # Proportional einschwenken; begrenzt, damit sie nicht aus dem Bild rutscht.
                omega = max(-config.HEALTH_SUCHEN_OMEGA, min(config.HEALTH_SUCHEN_OMEGA, math.radians(p["winkel"]) * 1.5))
                self._fahren(omega=omega)
            else:
                self._fahren(omega=config.HEALTH_SUCHEN_OMEGA)
            await asyncio.sleep(_TAKT_S)
        self._watchdog.stop()
        return None

    async def _heim(self) -> None:
        self._notiz("fährt zum Startpunkt zurück")
        if not self._homing.start_zu(0.0, 0.0, name="Startpunkt"):
            self._notiz("Heimfahrt startet nicht (läuft schon eine?)")
            return
        while self._homing.aktiv:
            await asyncio.sleep(0.2)
        self._notiz(f"Heimfahrt: {self._homing.letzter_grund or 'beendet'}")

    # ------------------------------------------------------------------ Ablauf

    async def _ablauf(self) -> None:
        self._pose.aktivieren(True, "healthbot")
        try:
            await self._ablauf_innen()
        except asyncio.CancelledError:
            raise
        except Exception as e:  # nie einen fahrenden Roboter zurücklassen
            log.exception("HEALTHBOT: Fehler im Ablauf")
            self._watchdog.stop()
            self._notiz(f"Fehler: {e}", ABGEBROCHEN)
            await self._senden("warning", f"Healthbot: Fehler ({e}) -- bitte selbst nachsehen")
        finally:
            self._watchdog.stop()
            self._pose.aktivieren(False, "healthbot")

    async def _ablauf_innen(self) -> None:
        # 1. Anfahrt
        if self._ziel is not None:
            self._notiz(f"fährt zur gemeldeten Stelle ({self._ziel['x']:.2f}, {self._ziel['y']:.2f})", ANFAHRT)
            await self._senden("info", "Healthbot: fahre zur gemeldeten Stelle")
            self._homing.start_zu(
                self._ziel["x"], self._ziel["y"], toleranz=config.HEALTH_ZIEL_TOLERANZ_M,
                name="gemeldete Stelle", blockiert_reicht_m=config.HEALTH_BLOCKIERT_REICHT_M,
            )
            while self._homing.aktiv:
                await asyncio.sleep(0.2)
            self._notiz(f"Anfahrt: {self._homing.letzter_grund or 'beendet'}")
            # Zur Stelle drehen: Die Person liegt dort, nicht in Fahrtrichtung
            # des letzten Ausweichens.
            x, y, grad = self._odometry.pose()
            if math.hypot(self._ziel["x"] - x, self._ziel["y"] - y) > 0.2:
                peilung = math.degrees(math.atan2(self._ziel["y"] - y, self._ziel["x"] - x))
                await self._drehen_um((peilung - grad + 180.0) % 360.0 - 180.0)

        # 2. Gesamtbild
        self._notiz("sucht die Person", GESAMTBILD)
        person = await self._person_suchen()
        if person is None:
            self._notiz("keine Person gefunden", KEINE_PERSON)
            self._urteil = {"stufe": "unbekannt", "punkte": 0, "text": "keine Person gefunden"}
            await self._senden("warning", "Healthbot: Keine Person gefunden – bitte selbst nachsehen")
            await self._heim()
            return
        self._notiz(f"Person gefunden ({person['winkel']:+.0f}°), beobachtet {config.HEALTH_BEOBACHTEN_S:.0f} s")
        verlauf = await self._beobachten(config.HEALTH_BEOBACHTEN_S)
        person = verlauf[-1] if verlauf else self._gesehen()
        self._check(bw.liegend(person))
        assert self._pose is not None  # starten() verlangt die Pose-Erkennung
        self._check(bw.boden(person, self._punkte(), self._pose.fov_deg))
        self._check(bw.regungslos(verlauf))

        # 3. Ansprechen
        self._notiz("spricht die Person an", ANSPRECHEN)
        await self._sprechen(config.HEALTH_FRAGE)
        gehoert = await self._zuhoeren(config.HEALTH_ANTWORT_S)
        self._check(bw.antwort(gehoert))
        self._urteil = bw.gesamturteil(self._checks)

        if gehoert:
            await self._nachfragen()
            return
        lage = any(self._checks[n]["zustand"] == bw.AUFFAELLIG for n in ("liegend", "boden"))
        if not lage:
            # Wer steht oder sitzt, wird nicht angestupst -- auch wenn er
            # nicht antwortet (Kopfhörer, kein Interesse, Mikrofon verhört).
            self._notiz("liegt nicht -- Person scheint wohlauf", UNAUFFAELLIG)
            await self._senden("info", "Healthbot: keine Auffälligkeit")
            await self._heim()
            return

        # 4. Anstupsen
        if config.HEALTH_STUPSEN_AKTIV:
            reaktion = await self._anstupsen()
            self._urteil = bw.gesamturteil(self._checks, reaktion)
            if reaktion:
                await self._nachfragen()
                return
        else:
            self._notiz("Anstupsen abgeschaltet (config.HEALTH_STUPSEN_AKTIV)")
        await self._hilfe()

    async def _nachfragen(self) -> None:
        """Die Person reagiert -- aber braucht sie Hilfe?"""
        self._notiz("Person reagiert, fragt nach", NACHFRAGE)
        await self._sprechen(config.HEALTH_FRAGE_HILFE)
        gehoert = await self._zuhoeren(config.HEALTH_ANTWORT_S)
        deutung = bw.antwort_deuten(gehoert or [])
        if gehoert:
            self._notiz(f"hört: „{' / '.join(gehoert)}“ -> {deutung or 'unklar'}")
        if deutung == "nein":
            self._notiz("Person ansprechbar", ANSPRECHBAR)
            self._urteil = {"stufe": "ansprechbar", "punkte": bw.anzahl(self._checks), "text": "Person ansprechbar, braucht keine Hilfe"}
            await self._senden("info", "Healthbot: Person ansprechbar")
            await self._sprechen(config.HEALTH_ANSAGE_OK)
            await self._heim()
            return
        # Ja, Hilfe -- oder keine/unklare Antwort: Im Zweifel Hilfe holen.
        # Ein Fehlalarm kostet einen Blick, ein verpasster Notfall mehr.
        grund = "bittet um Hilfe" if deutung == "hilfe" else "keine klare Antwort auf die Nachfrage"
        self._urteil = {"stufe": "notfall", "punkte": bw.anzahl(self._checks), "text": grund}
        await self._hilfe(grund)

    async def _anstupsen(self) -> bool:
        """Bis zu HEALTH_STUPSEN_ANZAHL Mal anstupsen. True bei Reaktion."""
        self._notiz("fährt heran, um anzustupsen", ANSTUPSEN)
        if not await self._heranfahren():
            self._notiz("kommt nicht nah genug heran -- kein Stupsen möglich")
            return False
        for nr in range(1, config.HEALTH_STUPSEN_ANZAHL + 1):
            seit = time.time()
            ergebnis = await self._stups(nr)
            await self._zurueck()
            verlauf = await self._beobachten(_REAKTION_S)
            bewegt = bw.regungslos(verlauf)
            gehoert = self._sprache.gehoert_seit(seit) if self._sprache is not None and getattr(self._sprache, "verfuegbar", False) else []
            reaktion = bewegt["zustand"] == bw.OK or bool(gehoert)
            ergebnis.update({"reaktion": reaktion, "bewegung": bewegt["wert"], "gehoert": gehoert})
            self._stupser.append(ergebnis)
            if reaktion:
                was = f"„{' / '.join(gehoert)}“" if gehoert else f"Bewegung {bewegt['wert']} px"
                self._notiz(f"Stups {nr}: Reaktion ({was})")
                self._check(bw.regungslos(verlauf))
                if gehoert:
                    self._check(bw.antwort(gehoert))
                return True
            self._notiz(f"Stups {nr}: keine Reaktion")
        return False

    async def _heranfahren(self) -> bool:
        """Langsam bis HEALTH_STUPSEN_START_M (Lidar) an die Person, dabei
        mittig halten. Der normale Fahrschutz bleibt an."""
        frist = time.monotonic() + _HERAN_MAX_S
        while time.monotonic() < frist:
            p = self._gesehen()
            if p is not None:
                self._person = p
            d = self._abstand(p)
            if d is None:
                self._watchdog.stop()
                self._notiz("kein Lidar-Abstand zur Person")
                return False
            if d <= config.HEALTH_STUPSEN_START_M:
                self._watchdog.stop()
                return True
            grenze = config.HEALTH_HERAN_OMEGA_MAX
            omega = 0.0 if p is None else max(-grenze, min(grenze, math.radians(p["winkel"])))
            gefahren = self._fahren(vx=config.HEALTH_HERAN_SPEED, omega=omega)
            reicht = config.HEALTH_STUPSEN_START_M + config.HEALTH_HERAN_REICHT_M
            if gefahren.vx <= 0.0 and self._watchdog.letzte_pruefung is not None and d <= reicht:
                # Fahrschutz hält knapp vor dem Ziel: Das reicht zum Stupsen.
                self._watchdog.stop()
                return True
            await asyncio.sleep(_TAKT_S)
        self._watchdog.stop()
        return False

    async def _stups(self, nr: int) -> dict:
        """Ein Stups: langsam vor, bis Kontakt oder Weggrenze.

        Weggrenze = Luft bis zur Person + HEALTH_STUPSEN_TIEFE_M, gemessen
        mit der Odometrie -- so drückt er höchstens ~10 cm weiter, als der
        Lidar Luft sieht. Kontakt = er steht, obwohl er fahren soll (Räder
        drehen nicht weiter, oder der Fahrschutz hält).
        """
        d = self._abstand(self._gesehen())
        luft = max(0.0, (d if d is not None else config.ROBOTER_VORN_M) - config.ROBOTER_VORN_M)
        grenze = luft + config.HEALTH_STUPSEN_TIEFE_M
        self._notiz(f"Stups {nr}: {luft:.2f} m Luft, höchstens {grenze:.2f} m vor")
        start = self._odometry.pose()
        beginn = time.monotonic()
        steht_seit: float | None = None
        grund = "Zeitgrenze"
        while time.monotonic() - beginn < config.HEALTH_STUPSEN_MAX_S:
            if self._strecke(start) >= grenze:
                grund = "Weggrenze"
                break
            gefahren = self._fahren(vx=config.HEALTH_STUPSEN_SPEED, rand_m=config.HEALTH_STUPSEN_RAND_M)
            vx_ist = self._odometry.geschwindigkeit()[0]
            # Erst nach dem Anfahren (Rampe) auf Stillstand achten.
            steht = time.monotonic() - beginn > 0.6 and (gefahren.vx <= 0.0 or vx_ist < config.HEALTH_STUPSEN_STEHT_MS)
            if steht:
                steht_seit = steht_seit or time.monotonic()
                if time.monotonic() - steht_seit >= config.HEALTH_STUPSEN_KONTAKT_S:
                    grund = "Kontakt"
                    break
            else:
                steht_seit = None
            await asyncio.sleep(_TAKT_S)
        self._watchdog.stop()
        weg = self._strecke(start)
        self._notiz(f"Stups {nr}: {grund} nach {weg:.2f} m")
        return {"nr": nr, "grund": grund, "weg_m": round(weg, 3), "zeit": time.time()}

    async def _zurueck(self) -> None:
        """Sofort zurück, damit die Person Platz hat (Heck mit Fahrschutz)."""
        start = self._odometry.pose()
        frist = time.monotonic() + _ZURUECK_MAX_S
        while time.monotonic() < frist and self._strecke(start) < config.HEALTH_STUPSEN_ZURUECK_M:
            self._fahren(vx=-config.HEALTH_HERAN_SPEED)
            await asyncio.sleep(_TAKT_S)
        self._watchdog.stop()

    async def _hilfe(self, grund: str = "liegt regungslos und reagiert nicht") -> None:
        """Hilfe holen: Alarm in der App, Ansage, heimfahren, Musik."""
        self._alarm = True
        self._notiz(f"HILFE: {grund}", HILFE)
        if self._urteil is None or self._urteil.get("stufe") != "notfall":
            self._urteil = {"stufe": "notfall", "punkte": bw.anzahl(self._checks), "text": grund}
        ort = ""
        if self._ziel is not None:
            ort = f" bei ({self._ziel['x']:.1f}, {self._ziel['y']:.1f})"
        await self._senden("emergency", f"Healthbot: Person{ort} braucht Hilfe – {grund}")
        await self._sprechen(config.HEALTH_ANSAGE_HILFE, vorrang=True)
        await self._heim()
        # Musik auch, wenn die Heimfahrt nicht ganz geklappt hat: Sie soll
        # Menschen herbeirufen, egal wo der Roboter steht.
        await self._musik()
        self._schritt = "Hilfe gerufen"

    async def _musik(self) -> None:
        if self._mukke is None:
            self._notiz("keine Musik (Lautsprecher fehlt)")
            return
        from ..audio.mukke import MukkeFehler

        pfad = os.path.join(_MUSIK_PFAD, config.HEALTH_MUSIK_DATEI)
        try:
            await asyncio.to_thread(self._mukke.datei_abspielen, pfad, config.HEALTH_MUSIK_TITEL)
            self._notiz(f"spielt „{config.HEALTH_MUSIK_TITEL}“")
        except MukkeFehler as e:
            self._notiz(f"Musik geht nicht: {e}")

    # ------------------------------------------------------------------- Wache

    async def _wache(self) -> None:
        """Nach liegenden Personen Ausschau halten. Wer HEALTH_AUTO_S lang
        liegend gesehen wird, wird geprüft; danach ist die Wache aus, sonst
        löste dieselbe Person sofort die nächste Prüfung aus."""
        # wache_setzen() lässt die Wache nur mit Pose-Erkennung an.
        assert self._pose is not None
        self._pose.aktivieren(True, "healthbot-wache")
        self._wache_seit = None
        try:
            while True:
                await asyncio.sleep(0.2)
                if self.aktiv:
                    self._wache_seit = None
                    continue
                p = self._gesehen()
                if p is None or bw.liegend(p)["zustand"] != bw.AUFFAELLIG:
                    self._wache_seit = None
                    continue
                self._wache_seit = self._wache_seit or time.monotonic()
                if time.monotonic() - self._wache_seit < config.HEALTH_AUTO_S:
                    continue
                d = self._abstand(p)
                x, y, grad = self._odometry.pose()
                if d is None:
                    ziel = None  # ohne Abstand: von hier aus prüfen
                else:
                    w = math.radians(grad + p["winkel"])
                    ziel = (x + d * math.cos(w), y + d * math.sin(w))
                fehler = self.starten(*(ziel or (None, None)), quelle="selbst erkannt")
                if fehler is None:
                    self._notiz("liegende Person selbst erkannt -- Wache aus")
                    await self._senden("warning", "Healthbot: liegende Person erkannt, prüfe")
                    return
                self._wache_seit = None
        finally:
            self._pose.aktivieren(False, "healthbot-wache")
