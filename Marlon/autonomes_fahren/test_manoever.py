"""Prüft die genauen Fahrbefehle ("1 m vor", "90° drehen") in der Simulation.

    python Marlon/autonomes_fahren/test_manoever.py

Simulierter Raum und simulierter Lidar wie beim Server mit --mock, dazu ein
Antrieb, der rutscht: Er fährt 5 % weniger weit und dreht 20 % weniger als
befohlen, die "Räder" melden noch einmal 12 % weniger (MockDriver). Wer nach
Fahrbefehl oder Rädern zählte, läge also deutlich daneben -- geprüft wird,
ob das Lidar ihn trotzdem genau ans Ziel bringt.

Dauert gut eine Minute (es wird in Echtzeit gefahren).
"""

import asyncio
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.drive.mock_driver import MockDriver  # noqa: E402
from rosmaster_assist.navigation import manoever as mv  # noqa: E402
from rosmaster_assist.safety import Fahrschutz, SafetyWatchdog  # noqa: E402
from rosmaster_assist.sensors.lidar import LidarService, MockLidar  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


class RutschenderAntrieb(MockDriver):
    """Die "wahre" Bewegung bleibt hinter dem Befehl zurück (Mecanum rutscht)."""

    VOR, DREH = 0.95, 0.80

    def _fortschreiben(self) -> None:
        jetzt = time.monotonic()
        if self._wahr_zeit is None:
            self._wahr_zeit = jetzt
            return
        dt = jetzt - self._wahr_zeit
        self._wahr_zeit = jetzt
        if dt <= 0 or dt > 1.0:
            return
        c = self._last
        w = c.omega * self.DREH
        mittel = self._wahr_theta + w * dt / 2.0
        vx, vy = c.vx * self.VOR, c.vy * self.VOR
        self._wahr_x += (vx * math.cos(mittel) - vy * math.sin(mittel)) * dt
        self._wahr_y += (vx * math.sin(mittel) + vy * math.cos(mittel)) * dt
        self._wahr_theta += w * dt


def relativ(start, ende):
    """Wo ``ende`` von ``start`` aus gesehen liegt (x vor, y links, Winkel)."""
    dx, dy = ende[0] - start[0], ende[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, ende[2] - start[2]


async def ausfuehren(m, antrieb, art, wert):
    vorher = antrieb.wahre_pose()
    t0 = time.monotonic()
    pruefe(f"{art} {wert}: startet", m.start(art, wert) is None)
    while m.aktiv:
        await asyncio.sleep(0.1)
    return relativ(vorher, antrieb.wahre_pose()), time.monotonic() - t0


async def main():
    antrieb = RutschenderAntrieb()
    antrieb.connect()
    lidar = LidarService(MockLidar(antrieb.wahre_pose))
    lidar.start()
    watchdog = SafetyWatchdog(antrieb)
    watchdog.fahrschutz = Fahrschutz(lidar)  # wie im Server
    watchdog.start()
    m = mv.Manoever(watchdog, antrieb, lidar)
    for _ in range(50):
        if lidar.latest() is not None:
            break
        await asyncio.sleep(0.1)

    # Ungültiges wird abgelehnt, ohne zu fahren.
    pruefe("0 m abgelehnt", m.start(mv.VOR, 0) is not None)
    pruefe("1000° abgelehnt", m.start(mv.DREHEN, 1000) is not None)
    pruefe("unbekannte Richtung abgelehnt", m.start("hoch", 1) is not None)

    for art, wert, soll in [
        (mv.VOR, 1.0, (1.0, 0.0, 0.0)),
        (mv.DREHEN, 90, (0.0, 0.0, 90.0)),
        (mv.DREHEN, -180, (0.0, 0.0, -180.0)),
        (mv.RECHTS, 0.5, (0.0, -0.5, 0.0)),
        (mv.ZURUECK, 0.3, (-0.3, 0.0, 0.0)),
        (mv.DREHEN, 360, (0.0, 0.0, 360.0)),
    ]:
        (x, y, th), dauer = await ausfuehren(m, antrieb, art, wert)
        grad = math.degrees(th)
        abweichung_m = math.hypot(x - soll[0], y - soll[1])
        abweichung_grad = grad - soll[2]
        text = (f"wirklich x={x:+.3f} y={y:+.3f} {grad:+.1f}° in {dauer:.1f} s "
                f"-- {m.letzter_grund}")
        pruefe(f"{mv.beschreiben(art, wert)}: genau (Winkel ±3°)", abs(abweichung_grad) < 3.0, text)
        if art != mv.DREHEN:
            pruefe(f"{mv.beschreiben(art, wert)}: genau (Weg ±4 cm, gerade)", abweichung_m < 0.04,
                   f"{abweichung_m * 100:.1f} cm daneben")
        pruefe(f"{mv.beschreiben(art, wert)}: meldet Erfolg", m.erfolgreich, m.letzter_grund)

    # 5 m in irgendeine Richtung gehen im 8 x 6 m großen Raum nirgends ganz:
    # Er muss vor der Wand anhalten und darf es nicht als Erfolg melden.
    (x, y, th), _ = await ausfuehren(m, antrieb, mv.RECHTS, 5.0)
    pruefe("vor der Wand angehalten, nicht erfolgreich",
           not m.erfolgreich and "Hindernis" in (m.letzter_grund or ""), m.letzter_grund)
    wx, wy, _ = antrieb.wahre_pose()
    x_min, x_max, y_min, y_max = MockLidar.RAUM
    rand = min(wx - x_min, x_max - wx, wy - y_min, y_max - wy)
    pruefe("Abstand zur Wand gewahrt (> 0,3 m)", rand > 0.3, f"{rand:.2f} m bei ({wx:.2f}, {wy:.2f})")

    # Abbrechen hält sofort an.
    m.start(mv.VOR, 2.0)
    await asyncio.sleep(1.5)
    m.abbrechen("Test")
    await asyncio.sleep(0.2)
    pruefe("abbrechen hält an", not m.aktiv and watchdog.current_command.is_stop(),
           watchdog.current_command)

    await watchdog.shutdown()
    lidar.stop()


asyncio.run(main())
print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
