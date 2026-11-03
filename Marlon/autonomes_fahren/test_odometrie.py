"""Prüft Blickrichtung der Odometrie und Tempogrenzen des Watchdogs -- ohne Roboter.

    python Marlon/autonomes_fahren/test_odometrie.py

Hintergrund: Die IMU-Lage des X3 liefert dauerhaft 0 (Sensor-Check 28.10.).
Früher nahm die Odometrie diesen Wert als Blickrichtung, theta blieb beim
Drehen stehen. Jetzt muss theta der gemessenen Drehrate folgen.
"""

import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.drive.base import DriveBase  # noqa: E402
from rosmaster_assist.navigation.odometry import Odometry  # noqa: E402
from rosmaster_assist.protocol import DriveCommand  # noqa: E402
from rosmaster_assist.safety.watchdog import SafetyWatchdog  # noqa: E402

fehler = 0


def pruefe(name, bedingung, info=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, info)
    fehler += not bedingung


class Attrappe(DriveBase):
    """Treiber wie am X3: Board-Geschwindigkeit stimmt, IMU-Winkel immer 0."""

    def __init__(self):
        self.bewegung = (0.0, 0.0, 0.0)
        self.akku = 12.0
        self.gefahren = []

    def connect(self):
        pass

    def disconnect(self):
        pass

    def drive(self, cmd):
        self.gefahren.append(cmd)

    def stop(self):
        self.gefahren.append(DriveCommand())

    def read_motion(self):
        return self.bewegung

    def read_heading(self):
        return 0.0

    def read_battery(self):
        return self.akku

    @property
    def connected(self):
        return True


def fahren(odo, befehl, sekunden, dt=0.05):
    """Zeitschritte simulieren, ohne zu warten: die letzte Zeit zurückdrehen."""
    odo.update(befehl)  # erster Aufruf setzt nur die Zeit
    for _ in range(int(round(sekunden / dt))):
        odo._letzte_zeit = time.monotonic() - dt
        odo.update(befehl)


# 1) Standard: IMU wird gar nicht gefragt, theta folgt der Drehrate
treiber = Attrappe()
odo = Odometry(treiber)
treiber.bewegung = (0.0, 0.0, 0.5)
fahren(odo, DriveCommand(omega=0.5), math.pi / 2 / 0.5)
_, _, theta = odo.pose()
pruefe("theta folgt Drehrate (90° Drehung)", abs(theta - 90) < 3, f"{theta:.1f}°")  # pose() liefert Grad
pruefe("Drehrate abfragbar", abs(odo.geschwindigkeit()[2] - 0.5) < 1e-9, odo.geschwindigkeit())

# 2) Danach geradeaus: Fahrt geht nach links (theta = 90°)
treiber.bewegung = (0.2, 0.0, 0.0)
fahren(odo, DriveCommand(vx=0.2), 2.0)
x, y, _ = odo.pose()
pruefe("Geradeaus nach Drehung geht in +y", abs(x) < 0.05 and abs(y - 0.4) < 0.05, f"x={x:.2f} y={y:.2f}")

# 3) Mit eingeschaltetem IMU-Winkel: starre IMU wird erkannt und verworfen
config.ODOM_IMU_WINKEL = True
try:
    treiber = Attrappe()
    odo = Odometry(treiber)
    treiber.bewegung = (0.0, 0.0, 0.8)
    fahren(odo, DriveCommand(omega=0.8), 3.0)
    _, _, theta = odo.pose()
    pruefe("starre IMU verworfen", odo.imu_verworfen)
    # Bis zum Verwerfen (ODOM_IMU_PRUEF_S) stand theta, danach summiert
    erwartet = 0.8 * (3.0 - config.ODOM_IMU_PRUEF_S)
    pruefe(
        "theta dreht nach dem Verwerfen weiter",
        abs(theta) > math.degrees(erwartet) * 0.7,
        f"{theta:.1f}° (mind. ~{math.degrees(erwartet) * 0.7:.0f}°)",
    )
finally:
    config.ODOM_IMU_WINKEL = False

# 4) Watchdog: autonome Befehle enger begrenzt als Handsteuerung
treiber = Attrappe()
wd = SafetyWatchdog(treiber)
wd.submit(DriveCommand(vx=2.0, omega=5.0), quelle="hand")
pruefe("Hand: MAX_SPEED_X greift", abs(wd._ziel.vx - config.MAX_SPEED_X) < 1e-9, wd._ziel)
wd.submit(DriveCommand(vx=2.0, vy=-2.0, omega=5.0), quelle="autonom")
pruefe(
    "Autonom: AUTONOM_MAX_* greift",
    abs(wd._ziel.vx - config.AUTONOM_MAX_SPEED) < 1e-9
    and abs(wd._ziel.vy + config.AUTONOM_MAX_SPEED) < 1e-9
    and abs(wd._ziel.omega - config.AUTONOM_MAX_OMEGA) < 1e-9,
    wd._ziel,
)

# 5) Schwacher Akku: autonomes Tempo gedrosselt
treiber = Attrappe()
treiber.akku = config.AKKU_WARN_V - 0.5
wd = SafetyWatchdog(treiber)
wd.submit(DriveCommand(vx=2.0), quelle="autonom")
soll = config.AUTONOM_MAX_SPEED * config.AUTONOM_AKKU_FAKTOR
pruefe("Akku schwach: Tempo gedrosselt", abs(wd._ziel.vx - soll) < 1e-9, wd._ziel)

print("\nalles bestanden" if not fehler else f"\n{fehler} Fehler")
sys.exit(1 if fehler else 0)
