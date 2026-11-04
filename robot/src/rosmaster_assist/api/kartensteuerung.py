"""Alles, was die Seite /map und die App mit der Karte tun können.

Drei Aufgaben, die nur der Server erledigen kann, weil nur er alle
Beteiligten kennt (Erkunder, Heimfahrt, Interaktion, Fahrbefehle, Antrieb):

1. **Kartierfahrt starten und am Laufen halten.** Sie ist ein Modus des
   Erkunders; Not-Stopp, Hand und Antriebsausfall gelten unverändert. Neu
   (04.11.2026): Sie endet nicht mehr, sobald eine Seite kurz zugeht oder
   der Antrieb per USB-Reset kurz weg ist -- sie wartet und fährt weiter.
   Ohne jeden Zuschauer (keine App, keine /map-Seite) hält sie nach
   ``MAP_ZUSCHAUER_WARTEN_S`` aber wie bisher an.
2. **Karte nutzen.** Gespeicherte Karte laden, zeigen, wo der Roboter
   steht, zu einem angetippten Punkt fahren (``Homing.start(ziel=...)``).
3. **Takt.** Neue Karten aus der Kartierung an die Ortung geben und die
   Lage nachführen (rechnet in einem Thread, nicht in der Ereignisschleife).
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any

from .. import config
from ..mapping.karte import Karte
from ..navigation.explorer import KARTE
from ..navigation.pfadplanung import luecken_anzeigen
from ..protocol import alert_message

log = logging.getLogger(__name__)


class Kartensteuerung:
    """Kartierfahrt, Ortung und Fahrt zum Ziel -- für /map und die App."""

    def __init__(self, *, mapping_manager, ortung, erkunder, homing, interaktion, manoever,
                 watchdog, driver, hub) -> None:
        self._manager = mapping_manager
        self._ortung = ortung
        self._erkunder = erkunder
        self._homing = homing
        self._interaktion = interaktion
        self._manoever = manoever
        self._watchdog = watchdog
        self._driver = driver
        self._hub = hub
        #: Offene /map-Seiten (zählt der Router).
        self.seiten = 0
        self._zuschauer_task: asyncio.Task | None = None
        self._unterbrochen: float | None = None   # Antrieb weg während der Kartierfahrt
        self._luecken: list[list[float]] = []
        self._luecken_zeit = 0.0

    # --- 1. Kartierfahrt (Schnittstelle wie bisher für mapping/web.py) ----------------

    @property
    def aktiv(self) -> bool:
        return self._erkunder.modus == KARTE

    def zustand(self) -> dict[str, Any]:
        z = self._erkunder.zustand()
        aktiv = self.aktiv
        k = z["kartierfahrt"] or {}
        return {
            "aktiv": aktiv,
            "text": z["text"] if aktiv else None,
            "dauer_s": z["dauer_s"] if aktiv else None,
            "schritt": k.get("schritt") if aktiv else None,
            "bahnen": k.get("bahnen") if aktiv else None,
            "luecken": k.get("luecken") if aktiv else None,
            # Wartet nach einem Antriebsausfall aufs Weiterfahren.
            "unterbrochen": self._unterbrochen is not None,
            # Warum die letzte Kartierfahrt endete (nicht die letzte
            # Zufallsfahrt).
            "grund": (
                self._erkunder.letzter_grund
                if not aktiv and self._erkunder.letzter_modus == KARTE
                else None
            ),
        }

    def _andere_abbrechen(self, grund: str) -> None:
        """Nur ein Programm lenkt."""
        self._homing.abbrechen(grund)
        self._erkunder.abbrechen(grund)
        self._interaktion.abbrechen(grund)
        self._manoever.abbrechen(grund)

    def starten(self, fortsetzen: bool = False) -> str | None:
        if self.aktiv:
            return None
        if not self._manager.aktiv:
            return ("Erst das Mapping starten: Die Kartierfahrt lenkt mit der "
                    "Lidar-Odometrie, und die läuft nur beim Kartieren.")
        if not self._erkunder.verfuegbar:
            return "Die Kartierfahrt geht nur mit Lidar."
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb."
        self._andere_abbrechen("Kartierfahrt gestartet")
        self._unterbrochen = None
        if not self._erkunder.start(KARTE, fortsetzen=fortsetzen):
            return "Die Kartierfahrt ließ sich nicht starten."
        self._melden("info", "Kartierfahrt fortgesetzt" if fortsetzen else "Kartierfahrt gestartet")
        return None

    def stoppen(self, grund: str) -> None:
        self._unterbrochen = None
        if self.aktiv:
            self._erkunder.abbrechen(grund)
            self._watchdog.stop()

    # Zuschauer: /map-Seiten und App-Verbindungen.

    def seite_auf(self) -> None:
        self.seiten += 1

    def seite_zu(self) -> None:
        self.seiten = max(0, self.seiten - 1)
        self.zuschauer_pruefen("keine Seite mehr offen")

    def niemand_schaut(self) -> bool:
        return self.seiten <= 0 and self._hub.count <= 0

    def zuschauer_pruefen(self, grund: str) -> None:
        """Schaut niemand mehr zu, nach einer Frist anhalten (nicht sofort).

        Ein Handy, dessen Bildschirm ausgeht, trennt die Verbindung; kommt
        es binnen ``MAP_ZUSCHAUER_WARTEN_S`` zurück, fährt er einfach weiter.
        """
        if not self.niemand_schaut():
            return
        if not (self.aktiv or self._homing.aktiv or self._unterbrochen is not None):
            return
        if self._zuschauer_task is not None and not self._zuschauer_task.done():
            return
        log.warning("Niemand schaut mehr zu -- halte in %.0f s an, wenn keiner zurückkommt",
                    config.MAP_ZUSCHAUER_WARTEN_S)
        self._zuschauer_task = asyncio.get_running_loop().create_task(self._spaeter_anhalten(grund))

    async def _spaeter_anhalten(self, grund: str) -> None:
        await asyncio.sleep(config.MAP_ZUSCHAUER_WARTEN_S)
        if not self.niemand_schaut():
            return
        log.warning("Kein Zuschauer zurückgekommen -- halte an (%s)", grund)
        self.stoppen(grund)
        self._homing.abbrechen(grund)
        self._watchdog.stop()

    # Antriebsausfall (USB-Reset) während der Kartierfahrt.

    def antrieb_weg(self) -> None:
        if self.aktiv:
            self._unterbrochen = time.monotonic()
            log.warning("Antrieb weg während der Kartierfahrt -- fahre nach dem Neuverbinden weiter")

    def antrieb_wieder(self) -> None:
        if self._unterbrochen is None:
            return
        weg_seit = time.monotonic() - self._unterbrochen
        self._unterbrochen = None
        if weg_seit > config.MAP_FAHRT_WIEDER_S:
            self._melden("warning", f"Kartierfahrt nicht fortgesetzt: Antrieb war {weg_seit:.0f} s weg")
            return
        if self.niemand_schaut():
            return
        fehler = self.starten(fortsetzen=True)
        if fehler:
            self._melden("warning", f"Kartierfahrt nicht fortgesetzt: {fehler}")

    def _melden(self, stufe: str, text: str) -> None:
        try:
            asyncio.get_running_loop().create_task(self._hub.broadcast(alert_message(stufe, text)))
        except RuntimeError:
            log.info(text)  # außerhalb der Ereignisschleife (Tests)

    # --- 2. Karte nutzen ----------------------------------------------------------------

    def ziel(self, x, y, winkel_grad=None) -> str | None:
        """Zu einem Punkt auf der Karte fahren. Fehlertext oder None."""
        try:
            x, y = float(x), float(y)
            winkel = math.radians(float(winkel_grad)) if winkel_grad is not None else None
        except (TypeError, ValueError):
            return "x und y müssen Zahlen sein (Meter im Kartenrahmen)"
        if not all(math.isfinite(v) for v in (x, y)):
            return "x und y müssen endlich sein"
        if self._ortung.karte is None:
            return "Keine Karte geladen"
        if not self._ortung.sicher:
            return "Lage in der Karte unbekannt -- erst auf der Karte zeigen, wo er steht"
        if not self._driver.connected:
            return "Keine Verbindung zum Antrieb"
        self._andere_abbrechen("neues Ziel")
        if not self._homing.start(ziel=(x, y), winkel=winkel):
            return self._homing.letzter_grund or "Fahrt zum Ziel ließ sich nicht starten"
        self._melden("info", f"Fahrt zum Ziel ({x:.2f}, {y:.2f}) gestartet")
        return None

    def ziel_stop(self, grund: str = "vom Nutzer abgebrochen") -> None:
        self._homing.abbrechen(grund)
        self._watchdog.stop()

    def lage_setzen(self, x, y, winkel_grad=None) -> str | None:
        try:
            x, y = float(x), float(y)
            winkel = math.radians(float(winkel_grad)) if winkel_grad is not None else None
        except (TypeError, ValueError):
            return "x und y müssen Zahlen sein"
        if self._ortung.karte is None:
            return "Keine Karte geladen"
        self._homing.abbrechen("Lage wird neu gesucht")
        self._ortung.lage_setzen(x, y, winkel)
        return None

    def laden(self, name: str) -> dict[str, Any]:
        self._andere_abbrechen("Karte wird geladen")
        return self._manager.load_map(name)

    def start_hier(self) -> bool:
        """"Startpunkt hier setzen" gilt mit Karte auch für die Heimfahrt nach Karte."""
        pose = self._ortung.pose()
        if pose is None or not self._ortung.sicher:
            return False
        self._manager.start_merken(pose)
        return True

    def anzeige(self) -> dict[str, Any]:
        """Für die Seite /map: Lage, Spur, laufende Fahrt, Lücken."""
        k = (self._erkunder.zustand().get("kartierfahrt") or {}) if self.aktiv else {}
        return {
            "ortung": self._ortung.zustand(),
            "spur": self._ortung.spur(),
            "fahrt": self._homing.zustand(),
            "kartierfahrt": {"weg": k.get("weg") or [], "ziel": k.get("ziel")},
            "luecken": self._luecken,
        }

    def zustand_kurz(self) -> dict[str, Any] | None:
        """Für state.karte in der App -- ohne Spur und Wege, die sind groß."""
        o = self._ortung.zustand()
        if not o["karte"]:
            return None
        f = self._homing.zustand()
        return {
            "name": o["name"],
            "live": o["live"],
            "pose": o["pose"],
            "sicher": o["sicher"],
            "text": o["text"],
            "fahrt": {k: f[k] for k in ("aktiv", "art", "ziel", "rest_m", "grund")},
        }

    # --- 3. Takt -----------------------------------------------------------------------

    async def takt_schleife(self, fehler_melden) -> None:
        """Karte übernehmen und Lage nachführen; läuft, solange der Server läuft."""
        takt = 1.0 / max(config.KARTE_ORTUNG_HZ, 0.5)
        letzte_karte = None
        while True:
            await asyncio.sleep(takt)
            try:
                if self._manager.aktiv:
                    payload = self._manager.kartierte_karte()
                    stempel = payload.get("received_at") if payload else None
                    if payload is not None and stempel != letzte_karte:
                        letzte_karte = stempel
                        karte = await asyncio.to_thread(_karte_bauen, payload)
                        self._ortung.karte_setzen(karte, live=True)
                karte = self._ortung.karte
                if karte is not None:
                    await asyncio.to_thread(self._ortung.abgleichen)
                    if self.seiten > 0 and time.monotonic() - self._luecken_zeit > 5.0:
                        self._luecken_zeit = time.monotonic()
                        self._luecken = await asyncio.to_thread(luecken_anzeigen, karte)
                else:
                    self._luecken = []
            except Exception:
                fehler_melden("Ortung in der Karte")


def _karte_bauen(payload) -> Karte:
    """Karte samt Abstandsfeld -- das Teure gleich hier im Thread rechnen."""
    karte = Karte.aus_payload(payload)
    _ = karte.gradient
    return karte
