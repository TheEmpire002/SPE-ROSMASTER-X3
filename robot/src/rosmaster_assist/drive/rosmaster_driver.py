"""Antrieb über Yahboons `Rosmaster_Lib` (echte Hardware).

Dieses Modul wird nur auf dem Roboter importiert. Auf Entwicklungsrechnern
nimmt `create_driver` stattdessen den Mock.

**Wiederverbinden.** Bricht die Versorgung am USB-Hub kurz ein, meldet der
Kernel alle Geräte ab und zwei Sekunden später unter neuen Nummern wieder
an. Rosmaster_Lib bemerkt davon nichts: ihr Lese-Thread stirbt mit
"device reports readiness to read but returned no data", und jeder weitere
Befehl geht an eine Datei, die es nicht mehr gibt. ``ueberwachen()``
erkennt das und baut die Verbindung neu auf.
"""

from __future__ import annotations

import logging
import os
import threading
import time

from .base import DriveBase
from .. import config
from ..protocol import DriveCommand
from ..serial_ports import wer_haelt

log = logging.getLogger(__name__)


class RosmasterDriver(DriveBase):
    name = "rosmaster"

    def __init__(self, port: str | None = None) -> None:
        # Die Vorgabe bleibt getrennt vom tatsächlich benutzten Port: nach
        # einem USB-Aussetzer muss neu gesucht werden, die Nummern tauschen.
        self._port_vorgabe = port if port is not None else config.SERIAL_PORT
        self._port: str | None = None
        self._bot = None
        self._last: DriveCommand | None = None
        self._lock = threading.Lock()
        # Woran sich eine tote Verbindung erkennen lässt -- siehe _lebt().
        self._geraet: int | None = None
        self._empfaenger: list[threading.Thread] = []
        self._naechster_versuch = 0.0
        self._wartezeit = config.RECONNECT_MIN_S

    def _port_bestimmen(self) -> str | None:
        """Sucht das Expansion Board unter mehreren seriellen Geräten.

        Am ROSMASTER hängen typischerweise zwei USB-Seriell-Adapter: das
        Board (CH340) und der RPLIDAR (CP210x). Beide erscheinen als
        ``/dev/ttyUSB*``, und wer welche Nummer bekommt, hängt von der
        Einschaltreihenfolge ab. Greift Rosmaster_Lib den Lidar, liest sie
        dessen Datenstrom als Board-Antworten -- das endet in
        "device reports readiness to read but returned no data".

        Der Lidar lässt sich sicher ausschließen, weil eine udev-Regel ihn
        unter ``/dev/rplidar`` festnagelt. Bleibt danach genau ein Gerät
        übrig, ist das das Board.
        """
        import glob

        if self._port_vorgabe:
            return self._port_vorgabe

        # Dieselbe Yahboom-udev-Regel nagelt das Board (CH340, 1a86:7523)
        # unter /dev/myserial fest. Gibt es den Pfad, ist er die sicherste
        # Wahl -- er hängt an Hersteller- und Produktnummer, nicht an der
        # Reihenfolge, in der sich die Geräte anmelden.
        if os.path.exists("/dev/myserial"):
            return "/dev/myserial"

        geraete = sorted(glob.glob("/dev/serial/by-id/*"))
        if not geraete:
            return None  # nichts zu entscheiden, Bibliothek soll suchen

        # Den Lidar immer ausschließen, auch wenn er das einzige Gerät ist:
        # Am 28.10. meldete er sich nach einem USB-Aussetzer eine Sekunde vor
        # dem Board wieder an, /dev/rplidar gab es noch nicht, und der
        # Treiber verband sich mit dem Lidar. In dem Moment verrät ihn nur
        # sein Name (CP210x von Silicon Labs).
        lidar = os.path.realpath("/dev/rplidar") if os.path.exists("/dev/rplidar") else None
        kandidaten = [
            g for g in geraete
            if os.path.realpath(g) != lidar and (lidar is not None or "CP210" not in g)
        ]

        if not kandidaten:
            # Nur der Lidar ist da, das Board meldet sich gerade noch an.
            # None lässt Rosmaster_Lib /dev/myserial öffnen; das schlägt fehl,
            # und ueberwachen() versucht es gleich noch einmal.
            log.info("Nur der Lidar ist angemeldet -- warte auf das Board")
            return None

        if len(kandidaten) == 1:
            log.info("Expansion Board erkannt: %s", kandidaten[0])
            log.info("(Lidar ausgeschlossen über /dev/rplidar)")
            return kandidaten[0]

        log.warning(
            "%d serielle Geräte, keines eindeutig zuzuordnen -- "
            "Rosmaster_Lib sucht sich eines aus:",
            len(geraete),
        )
        for g in geraete:
            log.warning("    %s", g)
        log.warning("Das richtige in config.py bei SERIAL_PORT eintragen.")
        return None

    def connect(self) -> None:
        try:
            import Rosmaster_Lib  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                "Rosmaster_Lib ist nicht installiert. Auf dem Entwicklungsrechner "
                "stattdessen mit --mock starten. Auf dem Roboter: siehe "
                "docs/hardware.md."
            ) from exc

        if not config.TELEMETRY_ENABLED:
            log.warning(
                "Rückkanal abgeschaltet (TELEMETRY_ENABLED=False) -- "
                "keine Akku- und IMU-Werte"
            )
        self._oeffnen()

    def _oeffnen(self) -> None:
        """Verbindung aufbauen. Wirft, wenn das Board nicht erreichbar ist."""
        from Rosmaster_Lib import Rosmaster

        self._port = self._port_bestimmen()
        # Rosmaster() ohne Argument sucht die Schnittstelle selbst.
        bot = Rosmaster(com=self._port) if self._port else Rosmaster()

        # Als Erstes anhalten. Nach einem Aussetzer war nur der USB-Wandler
        # weg, der Prozessor auf dem Board lief am Akku weiter -- und führt
        # womöglich noch den letzten Fahrbefehl aus.
        bot.set_car_motion(0.0, 0.0, 0.0)

        # Startet den Hintergrund-Thread, der Rückmeldungen (IMU, Akku,
        # Encoder) vom Board einliest. Ohne diesen Aufruf bleiben alle
        # Lesefunktionen der Bibliothek leer -- Fahren geht trotzdem.
        # Welcher Thread das ist, verrät die Bibliothek nicht; also
        # nachsehen, welcher neu dazukam.
        empfaenger: list[threading.Thread] = []
        if config.TELEMETRY_ENABLED:
            vorher = set(threading.enumerate())
            bot.create_receive_threading()
            empfaenger = [t for t in threading.enumerate() if t not in vorher]

        with self._lock:
            self._bot = bot
            self._geraet = _geraet_von(bot)
            self._empfaenger = empfaenger
            self._last = None
        self._wartezeit = config.RECONNECT_MIN_S

        log.info(
            "ROSMASTER verbunden (Port: %s)",
            getattr(getattr(bot, "ser", None), "port", None) or self._port or "automatisch",
        )
        # Schon beim Verbinden nachsehen: ein zweites Programm am Port fällt
        # sonst erst auf, wenn der Lese-Thread Minuten später stirbt.
        self._andere_melden()

    def _andere_melden(self) -> bool:
        """Loggt andere Prozesse am Board-Port. True, wenn welche da sind."""
        pfad = self._port or "/dev/myserial"
        andere = wer_haelt(pfad)
        if andere:
            log.error("Ein anderes Programm hat den Board-Port %s offen:", pfad)
            for zeile in andere:
                log.error("    %s", zeile)
            log.error("Das stört den Empfang -- beenden mit: kill <PID>")
        return bool(andere)

    def _verloren(self, grund: str) -> None:
        """Verbindung als tot markieren; ``ueberwachen()`` baut sie neu auf."""
        with self._lock:
            bot, self._bot = self._bot, None
            self._empfaenger = []
        if bot is None:
            return  # schon gemeldet
        log.error("Verbindung zum Board verloren (%s) -- verbinde neu", grund)
        self._andere_melden()
        # Den alten Port schließen, sonst bleibt er belegt -- und kommt das
        # Board unter derselben Nummer wieder, ließe er sich nicht öffnen.
        try:
            bot.ser.close()
        except Exception:
            log.debug("Alten Port schließen misslungen", exc_info=True)
        self._naechster_versuch = time.monotonic() + self._wartezeit

    def _lebt(self) -> bool:
        """Ob die bestehende Verbindung noch zum Board führt.

        Zwei Anzeichen für eine tote Verbindung, die ohne Schreibversuch
        auskommen:

        - Der Lese-Thread der Bibliothek ist gestorben. Genau das ist der
          Traceback, den man bei einem Aussetzer im Terminal sieht.
        - Der Gerätepfad zeigt inzwischen auf ein anderes Gerät als das
          geöffnete. Nach dem Neuanmelden ist ``/dev/myserial`` etwa von
          ``ttyUSB1`` auf ``ttyUSB0`` umgezogen.
        """
        with self._lock:
            bot, geraet, empfaenger = self._bot, self._geraet, self._empfaenger
        if bot is None:
            return False
        if empfaenger and not any(t.is_alive() for t in empfaenger):
            return False
        ser = getattr(bot, "ser", None)
        if ser is not None and not getattr(ser, "is_open", True):
            return False
        if geraet is not None:
            try:
                if os.stat(ser.port).st_rdev != geraet:
                    return False
            except OSError:
                return False  # Pfad gerade verschwunden
        return True

    def ueberwachen(self) -> None:
        if self._bot is not None:
            if not self._lebt():
                self._verloren("USB-Gerät neu angemeldet oder Lese-Thread beendet")
            return

        if time.monotonic() < self._naechster_versuch:
            return
        try:
            self._oeffnen()
            log.warning("Board wieder verbunden")
        except Exception as exc:
            # Häufig: das Gerät ist noch nicht wieder angemeldet. Mit
            # wachsendem Abstand weiterprobieren, statt das Log zu fluten.
            log.warning(
                "Board noch nicht erreichbar (%s) -- nächster Versuch in %.0f s",
                exc,
                self._wartezeit,
            )
            self._andere_melden()
            self._naechster_versuch = time.monotonic() + self._wartezeit
            self._wartezeit = min(self._wartezeit * 2, config.RECONNECT_MAX_S)

    def disconnect(self) -> None:
        if self._bot is None:
            return
        self.stop()
        # Rosmaster_Lib hat kein close(); der Serial-Port wird beim Beenden
        # des Prozesses freigegeben. Wichtig ist nur, dass der Roboter steht.
        self._bot = None
        log.info("ROSMASTER getrennt")

    def drive(self, cmd: DriveCommand) -> None:
        bot = self._bot
        if bot is None:
            # Während des Neuverbindens: Befehle verwerfen statt werfen. Ein
            # Fehler hier würde die Verbindung zur App mit abreißen.
            return
        try:
            # Mecanum: set_car_motion(vx, vy, vz) erwartet m/s, m/s, rad/s.
            bot.set_car_motion(cmd.vx, cmd.vy, cmd.omega)
        except Exception as exc:
            self._verloren(f"Schreiben fehlgeschlagen: {exc}")
            return

        # Nur bei Änderung loggen, sonst 10 Zeilen pro Sekunde. Dass hier
        # überhaupt geloggt wird, ist Absicht: sonst sieht ein stiller
        # Roboter im echten Betrieb genauso aus wie einer in der
        # Simulation, und man sucht den Fehler an der falschen Stelle.
        if cmd != self._last:
            log.info("FAHRE  vx=%+.2f  vy=%+.2f  omega=%+.2f", cmd.vx, cmd.vy, cmd.omega)
            self._last = cmd

    def stop(self) -> None:
        # Darf nie werfen -- stop() ist der Notausgang und wird auch aus
        # Fehlerpfaden und beim Herunterfahren aufgerufen.
        bot = self._bot
        if bot is None:
            return
        try:
            bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception as exc:
            log.exception("Anhalten fehlgeschlagen -- Hardware prüfen!")
            self._verloren(f"Anhalten fehlgeschlagen: {exc}")

    def beep(self, ms: int = 100) -> None:
        bot = self._bot
        if bot is None:
            return
        try:
            bot.set_beep(ms)
        except Exception:
            log.exception("Buzzer fehlgeschlagen")

    def read_motion(self) -> tuple[float, float, float] | None:
        # Braucht den Empfangs-Thread; ohne ihn liefert die Bibliothek Nullen
        # oder -- nach einem Aussetzer -- die letzten Werte vor dem Abriss.
        # Beides wäre schlimmer als keine Messung.
        bot = self._bot
        if bot is None or not config.TELEMETRY_ENABLED or not self._lebt():
            return None
        try:
            vx, vy, omega = bot.get_motion_data()
            return float(vx), float(vy), float(omega)
        except Exception:
            return None

    def read_heading(self) -> float | None:
        bot = self._bot
        if bot is None or not config.TELEMETRY_ENABLED or not self._lebt():
            return None
        try:
            _roll, _pitch, yaw = bot.get_imu_attitude_data()
            return float(yaw)
        except Exception:
            return None

    @property
    def connected(self) -> bool:
        return self._bot is not None


def _geraet_von(bot) -> int | None:
    """Kennung des tatsächlich geöffneten Geräts (Haupt-/Nebennummer).

    Bleibt gültig, auch wenn das Gerät längst abgemeldet ist -- genau das
    macht sie zum Vergleich mit dem, worauf der Pfad jetzt zeigt, brauchbar.
    """
    try:
        return os.fstat(bot.ser.fileno()).st_rdev
    except Exception:
        return None
