"""Genaue Fahrbefehle: "x Meter vor/zurück/links/rechts" und "x Grad drehen".

Der Roboter soll dabei wirklich gerade fahren und sich wirklich um den
Winkel drehen. Fahren nach Zeit (Tempo × Dauer) taugt dafür nicht: Die
Mecanum-Räder rutschen, beim Drehen besonders, und der Fahrbefehl allein
überschätzt die Bewegung um 10-30 % (Sensor-Check 28.10.). Der Gyro hat
einen großen Bias und das falsche Vorzeichen.

Deshalb misst das Lidar. Beim Start wird eine Aufnahme gemacht, und jede
weitere wird darauf gelegt (ICP, siehe scan_matching.py). Daraus folgt,
wie weit er sich gegenüber dem Start wirklich bewegt und gedreht hat --
Rutschen ist dem Lidar egal, es schaut auf die Wände. Räder und Fahrbefehl
liefern nur die Schätzung für die Zeit zwischen zwei Aufnahmen (7,7 pro
Sekunde) und den Startwert für den Abgleich.

Beim Drehen wechselt die Bezugsaufnahme alle ~25°, sonst überschnitte sich
nach einer halben Drehung zu wenig. Die Winkel werden dabei aufaddiert,
nicht auf ±180° gekürzt -- so geht auch "360 Grad".

Gerade fahren: Der Regler hält nicht nur den Restweg, sondern auch die
seitliche Abweichung und die Blickrichtung bei null. Der X3 kann seitwärts
fahren, also wird eine Abweichung direkt ausgeglichen statt mit Lenken.

Am Ende wird im Stand nachgemessen: Während der Fahrt ist eine Umdrehung
des Lidars verzerrt (er bewegt sich ja währenddessen). Erst eine Aufnahme
im Stillstand sagt genau, wo er steht; liegt er daneben, bessert er nach.

Die Kamera wird dafür nicht gebraucht: Sie sieht nur nach vorn und misst
keine Entfernungen. Das Lidar sieht ringsum und misst auf Zentimeter.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections import deque

from .. import config
from ..protocol import DriveCommand
from ..safety import SafetyWatchdog
from .explorer import frei_in_richtung
from .scan_matching import match

log = logging.getLogger(__name__)

VOR = "vor"
ZURUECK = "zurueck"
LINKS = "links"
RECHTS = "rechts"
DREHEN = "drehen"

#: Fahrtrichtung im Roboter-Koordinatensystem (0° = vorn, 90° = links).
RICHTUNGEN = {VOR: 0.0, ZURUECK: 180.0, LINKS: 90.0, RECHTS: -90.0}

_RICHTUNG_TEXT = {VOR: "vor", ZURUECK: "zurück", LINKS: "nach links", RECHTS: "nach rechts"}


def beschreiben(art: str, wert: float) -> str:
    """'1,5 m vor', '90° nach links' -- für Meldungen und Ansagen."""
    if art == DREHEN:
        seite = "nach links" if wert >= 0 else "nach rechts"
        return f"{abs(wert):g}° {seite}".replace(".", ",")
    return f"{wert:g} m {_RICHTUNG_TEXT[art]}".replace(".", ",")


# --- Lage in der Ebene: (x, y, Winkel), Winkel NICHT gekürzt --------------------------

def _verknuepfen(a, b):
    """Erst Lage a, dann von dort aus b (a ∘ b)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, at + bt)


def _umkehren(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-(c * ax + s * ay), -(-s * ax + c * ay), -at)


def _kuerzen(winkel: float) -> float:
    return math.atan2(math.sin(winkel), math.cos(winkel))


def _ausduennen(punkte, hoechstens: int):
    """Gleichmäßig auf höchstens so viele Punkte -- der Abgleich rechnet
    mit allen Paaren, die Zeit wächst mit dem Quadrat."""
    if len(punkte) <= hoechstens:
        return punkte
    import numpy as np

    auswahl = (np.arange(hoechstens) * (len(punkte) / hoechstens)).astype(int)
    return punkte[auswahl]


class Manoever:
    """Führt einen genauen Fahrbefehl aus. Bedienung wie ``Homing``."""

    def __init__(self, watchdog: SafetyWatchdog, driver, lidar) -> None:
        self._watchdog = watchdog
        self._driver = driver
        self._lidar = lidar
        self._task: asyncio.Task[None] | None = None
        self._grund: str | None = None
        self._art: str | None = None
        self._wert = 0.0
        self._text = ""
        self.erfolgreich = False
        self.ansage: str | None = None  # was er sagt, wenn es nicht geklappt hat
        self._dreh_seit: float | None = None  # seit wann (aus dem Stand) gedreht wird
        self.beendet = 0  # zählt jedes Ende von selbst (für die Meldung im Server)

        # Lidar-Ortung während des Befehls, Bezug ist die Lage beim Start.
        self._pose = (0.0, 0.0, 0.0)
        self._verlauf: deque = deque(maxlen=200)  # (Zeit, Lage) für den Rückblick
        self._letzte_zeit = 0.0
        self._bezug = None                  # Punkte der Bezugsaufnahme
        self._bezug_pose = (0.0, 0.0, 0.0)  # wo sie entstand
        self._letzter_scan = 0.0
        self._umlauf = 0.13                 # Dauer einer Lidar-Umdrehung (s), wird gemessen
        self._abgleiche = 0
        self._fehlschlaege = 0

    # --- Bedienung -------------------------------------------------------------

    @property
    def verfuegbar(self) -> bool:
        return self._lidar is not None

    @property
    def aktiv(self) -> bool:
        return self._task is not None and not self._task.done()

    @property
    def letzter_grund(self) -> str | None:
        return self._grund

    def start(self, art: str, wert: float) -> str | None:
        """Befehl beginnen. Gibt einen Fehlertext zurück, wenn er nicht geht."""
        if not self.verfuegbar:
            return "Genaue Fahrbefehle gehen nur mit Lidar"
        try:
            wert = float(wert)
        except (TypeError, ValueError):
            return "Zahl erwartet"
        if not math.isfinite(wert):
            return "Zahl erwartet"
        if art == DREHEN:
            if not 0 < abs(wert) <= config.MANOEVER_MAX_GRAD:
                return f"Drehen: 1 bis {config.MANOEVER_MAX_GRAD:g} Grad"
        elif art in RICHTUNGEN:
            if not 0 < wert <= config.MANOEVER_MAX_M:
                return f"Fahren: bis {config.MANOEVER_MAX_M:g} m"
        else:
            return f"unbekannte Richtung {art!r}"
        self.abbrechen("neuer Befehl")
        self._art, self._wert = art, wert
        self._grund = None
        self.erfolgreich = False
        self.ansage = None
        self._text = f"{beschreiben(art, wert)}: startet"
        self._task = asyncio.create_task(self._fahren(art, wert))
        return None

    def abbrechen(self, grund: str = "abgebrochen") -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
            self._grund = grund
            log.info("Fahrbefehl %s", grund)
        self._task = None

    def zustand(self) -> dict:
        x, y, th = self._pose
        return {
            "aktiv": self.aktiv,
            "art": self._art,
            "wert": self._wert,
            "text": self._text,
            "grund": self._grund,
            "erfolgreich": self.erfolgreich,
            # Bisher geschafft, gegenüber dem Start (Lidar).
            "gefahren_m": round(math.hypot(x, y), 3),
            "gedreht_grad": round(math.degrees(th), 1),
            "abgleiche": self._abgleiche,
            "fehlschlaege": self._fehlschlaege,
        }

    # --- Ablauf ------------------------------------------------------------------

    async def _fahren(self, art: str, wert: float) -> None:
        takt = 1.0 / max(1, config.ODOMETRY_HZ)
        if art == DREHEN:
            ziel = (0.0, 0.0, math.radians(wert))
            dauer = abs(math.radians(wert)) / config.MANOEVER_DREH
        else:
            r = math.radians(RICHTUNGEN[art])
            ziel = (wert * math.cos(r), wert * math.sin(r), 0.0)
            dauer = wert / config.MANOEVER_SPEED
        # Großzügig: Anfahren, Nachbessern, kurzes Warten auf Aufnahmen.
        frist = time.monotonic() + 3.0 * dauer + 10.0
        beschreibung = beschreiben(art, wert)

        try:
            # Was vorher lief, steht jetzt; dann eine Aufnahme im Stand.
            self._watchdog.stop()
            scan = await self._aufnahme_nach(time.monotonic() + config.MANOEVER_RUHE_S)
            if scan is None:
                return self._ende("keine frische Lidar-Aufnahme",
                                  "Mein Lidar sieht gerade nichts, ich fahre lieber nicht.")
            if art == DREHEN and scan.min_abstand(0.0, 180.0) < config.MANOEVER_DREH_FREI_M:
                return self._ende("zum Drehen zu wenig Platz", "Zum Drehen ist es mir hier zu eng.")
            self._pose = (0.0, 0.0, 0.0)
            self._bezug = _ausduennen(scan.punkte(), config.MANOEVER_MAX_PUNKTE)
            self._bezug_pose = self._pose
            self._letzter_scan = scan.zeit
            self._letzte_zeit = time.monotonic()
            self._verlauf.clear()
            self._verlauf.append((scan.zeit, self._pose))
            self._abgleiche = self._fehlschlaege = 0
            erkannt = time.monotonic()
            nachgebessert = 0
            log.info("Fahrbefehl %s gestartet", beschreibung)

            while True:
                self._vorhersagen()
                scan = self._lidar.latest()
                if scan is not None and scan.zeit > self._letzter_scan:
                    if await asyncio.to_thread(self._abgleichen, scan):
                        erkannt = time.monotonic()
                if time.monotonic() - erkannt > config.MANOEVER_BLIND_S:
                    return self._ende("Lidar erkennt die Umgebung nicht wieder -- angehalten",
                                      "Ich habe die Orientierung verloren und angehalten.")

                ex, ey, eth = self._fehler(ziel)
                if self._erreicht(art, ex, ey, eth, config.MANOEVER_ZIEL_ANTEIL):
                    self._watchdog.stop()
                    scan = await self._aufnahme_nach(time.monotonic() + config.MANOEVER_RUHE_S)
                    if scan is not None and await asyncio.to_thread(self._abgleichen, scan):
                        ex, ey, eth = self._fehler(ziel)
                    # Im Stand gemessen streng im Ziel: fertig. Sonst
                    # nachbessern; erst wenn das aufgebraucht ist, reicht die
                    # normale Toleranz.
                    genau = self._erreicht(art, ex, ey, eth, config.MANOEVER_ZIEL_ANTEIL)
                    aufgebraucht = nachgebessert >= config.MANOEVER_NACHBESSERN
                    if genau or (aufgebraucht and self._erreicht(art, ex, ey, eth)):
                        self.erfolgreich = True
                        return self._ende(f"{beschreibung} erledigt ({self._rest(art, ex, ey, eth)})")
                    nachgebessert += 1
                    if nachgebessert > config.MANOEVER_NACHBESSERN:
                        return self._ende(f"{beschreibung} fast erledigt ({self._rest(art, ex, ey, eth)})",
                                          self._daneben(art, ex, ey, eth))
                    log.info("Fahrbefehl: im Stand gemessen noch %s -- bessere nach",
                             self._rest(art, ex, ey, eth))
                    erkannt = time.monotonic()
                    continue

                if time.monotonic() > frist:
                    return self._ende(f"Zeit abgelaufen, noch {self._rest(art, ex, ey, eth)}",
                                      "Das hat zu lange gedauert, ich habe aufgehört.")

                befehl = self._befehl(art, ex, ey, eth)
                hindernis = self._hindernis(art, befehl)
                if hindernis is not None:
                    return self._ende(hindernis, "Mir ist beim Drehen etwas zu nah, ich habe angehalten."
                                      if art == DREHEN else "Da ist etwas im Weg, ich habe angehalten.")
                self._text = f"{beschreibung}: noch {self._rest(art, ex, ey, eth)}"
                # Selbstständige Fahrt: engere Tempogrenze und die zentrale
                # Bremsweg-Prüfung (AutonomSchutz) wie bei Folgen und Erkunden.
                self._watchdog.submit(befehl, quelle="autonom")
                await asyncio.sleep(takt)

        except asyncio.CancelledError:
            self._watchdog.stop()
            raise
        except Exception:
            self._watchdog.stop()
            self._grund = "Fehler während der Fahrt"
            self.beendet += 1
            log.exception("Fahrbefehl abgebrochen")

    def _ende(self, grund: str, ansage: str | None = None) -> None:
        """Befehl beenden. ``ansage``: was er laut sagt, wenn es nicht geklappt
        hat -- per Sprache gestartet schaut keiner auf die App, und ein
        bloßes "hat nicht geklappt" lässt raten, ob man etwas wegräumen muss."""
        self._watchdog.stop()
        self._grund = grund
        self._text = grund
        self.ansage = ansage
        self.beendet += 1
        (log.info if self.erfolgreich else log.warning)("Fahrbefehl: %s", grund)

    @staticmethod
    def _daneben(art: str, ex: float, ey: float, eth: float) -> str:
        if art == DREHEN:
            return f"Fast geschafft, ich liege {abs(math.degrees(eth)):.0f} Grad daneben."
        return f"Fast geschafft, ich liege {math.hypot(ex, ey) * 100:.0f} Zentimeter daneben."

    async def _aufnahme_nach(self, zeitpunkt: float, frist_s: float = 2.0):
        """Erste Lidar-Aufnahme, die ganz nach ``zeitpunkt`` entstand."""
        ende = time.monotonic() + frist_s
        while time.monotonic() < ende:
            self._vorhersagen()
            scan = self._lidar.latest()
            # Eine ganze Umdrehung nach dem Zeitpunkt: vorher begonnene
            # Aufnahmen enthalten noch Bewegung.
            if scan is not None and scan.zeit - self._umlauf >= zeitpunkt:
                return scan
            await asyncio.sleep(0.02)
        return None

    # --- Ortung ------------------------------------------------------------------

    def _geschwindigkeit(self) -> tuple[float, float, float]:
        """Wie er sich gerade bewegt (Roboter-Koordinaten).

        Vorwärts/seitwärts aus den Rädern (stimmen gut), Drehung aus dem
        Fahrbefehl -- der Gyro ist unbrauchbar, die Rad-Drehrate wegen
        Rutschen ebenso. Das Lidar korrigiert beides bei jeder Aufnahme.
        """
        befohlen = self._watchdog.current_command
        gemessen = None
        try:
            gemessen = self._driver.read_motion()
        except Exception:
            pass
        if gemessen is None:
            return befohlen.vx, befohlen.vy, befohlen.omega
        return gemessen[0], gemessen[1], befohlen.omega

    def _vorhersagen(self) -> None:
        jetzt = time.monotonic()
        dt = min(max(jetzt - self._letzte_zeit, 0.0), 0.2)
        self._letzte_zeit = jetzt
        if dt <= 0:
            return
        vx, vy, w = self._geschwindigkeit()
        # Aus dem Stand drehen die Motoren erst nach einer Weile wirklich
        # (siehe MANOEVER_ANLAUF_S); bis dahin nicht mitzählen. Unterschätzen
        # ist harmlos -- die nächste Lidar-Aufnahme korrigiert --, Überschätzen
        # ließ Nachbesserungen ins Leere laufen.
        if abs(w) < 1e-3:
            self._dreh_seit = None
        else:
            if self._dreh_seit is None:
                self._dreh_seit = jetzt
            if jetzt - self._dreh_seit < config.MANOEVER_ANLAUF_S:
                w = 0.0
        # In der Mitte des Schritts drehen: genauer bei Fahrt mit Drehung.
        halb = (0.0, 0.0, w * dt / 2.0)
        self._pose = _verknuepfen(_verknuepfen(_verknuepfen(self._pose, halb),
                                               (vx * dt, vy * dt, 0.0)), halb)
        self._verlauf.append((jetzt, self._pose))

    def _pose_bei(self, zeitpunkt: float):
        for t, pose in reversed(self._verlauf):
            if t <= zeitpunkt:
                return pose
        return self._verlauf[0][1] if self._verlauf else self._pose

    def _abgleichen(self, scan) -> bool:
        """Aufnahme auf die Bezugsaufnahme legen und die Lage korrigieren."""
        abstand = scan.zeit - self._letzter_scan
        if 0.05 < abstand < 0.4:
            self._umlauf = 0.8 * self._umlauf + 0.2 * abstand
        self._letzter_scan = scan.zeit
        punkte = _ausduennen(scan.punkte(), config.MANOEVER_MAX_PUNKTE)
        # Eine Aufnahme dauert eine Umdrehung; im Mittel zeigt sie die Lage
        # eine halbe Umdrehung vor ihrem Zeitstempel.
        damals = self._pose_bei(scan.zeit - self._umlauf / 2.0)
        schaetzung = _verknuepfen(_umkehren(self._bezug_pose), damals)
        ergebnis = match(punkte, self._bezug, schaetzung)
        if ergebnis is None or ergebnis.residuum > config.MATCH_MAX_RESIDUAL_M:
            self._fehlschlaege += 1
            return False
        self._abgleiche += 1
        # ICP kürzt den Winkel auf ±180°; neben die Schätzung legen, damit
        # die aufaddierte Drehung erhalten bleibt.
        winkel = schaetzung[2] + _kuerzen(ergebnis.theta - schaetzung[2])
        relativ = (ergebnis.x, ergebnis.y, winkel)
        gemessen = _verknuepfen(self._bezug_pose, relativ)
        # Korrektur K mit K ∘ damals = gemessen; gilt auch für alles danach.
        k = _verknuepfen(gemessen, _umkehren(damals))
        self._pose = _verknuepfen(k, self._pose)
        self._verlauf = deque(((t, _verknuepfen(k, p)) for t, p in self._verlauf),
                              maxlen=self._verlauf.maxlen)
        # Bezug nur wechseln, wenn der Abgleich nachlässt (siehe config).
        if (ergebnis.residuum > config.MANOEVER_NEUER_BEZUG_RESIDUUM_M
                or ergebnis.paare < config.MANOEVER_NEUER_BEZUG_PAARE * len(punkte)):
            log.debug("Fahrbefehl: neue Bezugsaufnahme (Rest %.3f m, %d Paare)",
                      ergebnis.residuum, ergebnis.paare)
            self._bezug, self._bezug_pose = punkte, gemessen
        return True

    # --- Regeln ------------------------------------------------------------------

    def _fehler(self, ziel) -> tuple[float, float, float]:
        """Restweg (vor, links) im Roboter-Koordinatensystem und Restwinkel."""
        x, y, th = self._pose
        dx, dy = ziel[0] - x, ziel[1] - y
        c, s = math.cos(th), math.sin(th)
        return c * dx + s * dy, -s * dx + c * dy, ziel[2] - th

    @staticmethod
    def _erreicht(art: str, ex: float, ey: float, eth: float, anteil: float = 1.0) -> bool:
        """Im Ziel? ``anteil`` < 1: strenger (im Fahren, siehe MANOEVER_ZIEL_ANTEIL)."""
        winkel_ok = abs(math.degrees(eth)) < config.MANOEVER_TOLERANZ_GRAD * anteil
        if art == DREHEN:
            # Beim Drehen auf der Stelle zählt nur der Winkel; die paar
            # Zentimeter, die Mecanum dabei wandert, sind nicht gefragt.
            return winkel_ok
        return winkel_ok and math.hypot(ex, ey) < config.MANOEVER_TOLERANZ_M * anteil

    @staticmethod
    def _rest(art: str, ex: float, ey: float, eth: float) -> str:
        grad = f"{math.degrees(eth):+.1f}°"
        if art == DREHEN:
            return grad
        return f"{math.hypot(ex, ey) * 100:.0f} cm, {grad}"

    @staticmethod
    def _befehl(art: str, ex: float, ey: float, eth: float) -> DriveCommand:
        # Bis in den strengen Bereich hinein fahren, siehe _erreicht().
        anteil = config.MANOEVER_ZIEL_ANTEIL
        toleranz_w = math.radians(config.MANOEVER_TOLERANZ_GRAD) * anteil
        omega = max(-config.MANOEVER_DREH, min(config.MANOEVER_DREH, config.MANOEVER_KP_W * eth))
        if art == DREHEN:
            if abs(eth) >= toleranz_w and abs(omega) < config.MANOEVER_MIN_DREH:
                omega = math.copysign(config.MANOEVER_MIN_DREH, eth)
            return DriveCommand(0.0, 0.0, omega)

        # Fahren: auf das Ziel zu, seitliche Abweichung gleich mit (Mecanum).
        rest = math.hypot(ex, ey)
        tempo = min(config.MANOEVER_SPEED, config.MANOEVER_KP_V * rest)
        if rest >= config.MANOEVER_TOLERANZ_M * anteil:
            tempo = max(tempo, config.MANOEVER_MIN_SPEED)
            vx, vy = ex / rest * tempo, ey / rest * tempo
        else:
            vx = vy = 0.0
            # Nur noch die Richtung falsch: im Stand nachdrehen.
            if abs(eth) >= toleranz_w and abs(omega) < config.MANOEVER_MIN_DREH:
                omega = math.copysign(config.MANOEVER_MIN_DREH, eth)
        return DriveCommand(vx, vy, omega)

    def _hindernis(self, art: str, befehl: DriveCommand) -> str | None:
        """Grund zum Anhalten, oder None."""
        scan = self._lidar.latest()
        if scan is None:
            return None  # MANOEVER_BLIND_S greift
        if art == DREHEN:
            nah = scan.min_abstand(0.0, 180.0)
            if nah < config.MANOEVER_DREH_FREI_M:
                return f"beim Drehen etwas in {nah:.2f} m -- angehalten"
            return None
        if abs(befehl.vx) < 1e-6 and abs(befehl.vy) < 1e-6:
            return None
        richtung = math.degrees(math.atan2(befehl.vy, befehl.vx))
        frei = frei_in_richtung(scan.punkte(), richtung, config.EXPLORE_HALF_WIDTH_M)
        if frei - config.EXPLORE_STOP_M < 0.02:
            return f"Hindernis in {frei:.2f} m -- angehalten"
        return None
