"""Die Schnittstelle, die jeder Antrieb erfüllen muss.

Der restliche Code kennt nur diese Klasse, nie einen konkreten Treiber.
Dadurch lässt sich die Hardware austauschen -- gegen den Mock zum Entwickeln,
oder später gegen eine ROS2-Anbindung -- ohne dass Server oder Watchdog
angefasst werden müssen.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..protocol import DriveCommand


class DriveBase(ABC):
    """Ein Antrieb, der Fahrbefehle ausführen kann."""

    #: Kurzname für Logausgaben und Statusmeldungen an die App.
    name: str = "base"

    @abstractmethod
    def connect(self) -> None:
        """Verbindung zur Hardware aufbauen. Muss vor `drive` laufen."""

    @abstractmethod
    def disconnect(self) -> None:
        """Verbindung sauber schließen. Hält den Roboter vorher an."""

    @abstractmethod
    def drive(self, cmd: DriveCommand) -> None:
        """Fahrbefehl ausführen.

        Wird oft aufgerufen (mehrmals pro Sekunde) und muss schnell
        zurückkehren -- also nicht blockieren.
        """

    @abstractmethod
    def stop(self) -> None:
        """Sofort anhalten.

        Muss auch dann funktionieren, wenn vorher etwas schiefgelaufen ist,
        und darf keine Ausnahme werfen.
        """

    def beep(self, ms: int = 100) -> None:
        """Signalton. Optional -- Standard ist: passiert nichts."""

    def ueberwachen(self) -> None:
        """Verbindung prüfen und nach einem Abriss neu aufbauen.

        Wird regelmäßig aus einem Hintergrund-Thread aufgerufen und darf
        dabei kurz blockieren. Ob die Verbindung steht, meldet danach
        ``connected``. Standard: nichts zu tun.
        """

    def read_motion(self) -> tuple[float, float, float] | None:
        """Tatsächlich gefahrene Geschwindigkeit (vx, vy, omega), falls messbar.

        Für die Positionsverfolgung deutlich besser als der befohlene Wert:
        rutschende Räder, Steigungen und Anfahrverzögerung schlagen sich hier
        nieder. ``None`` bedeutet "nicht messbar" -- dann wird ersatzweise mit
        dem Befehl gerechnet.
        """
        return None

    def read_heading(self) -> float | None:
        """Blickrichtung in Grad aus der IMU, falls vorhanden.

        Ein gemessener Winkel ist der aufsummierten Drehrate weit überlegen:
        beim Aufsummieren wächst der Fehler mit jeder Drehung, hier nicht.
        """
        return None

    def read_battery(self) -> float | None:
        """Akkuspannung in Volt, falls messbar.

        Für die Diagnoseseite: ein schwacher Akku ist am X3 die häufigste
        Ursache für Abmeldungen des Boards. ``None`` heißt "nicht messbar".
        """
        return None

    @property
    @abstractmethod
    def connected(self) -> bool:
        """Ob die Verbindung zur Hardware steht."""

    # Ermöglicht `with create_driver(...) as drv:` -- der Roboter hält auch
    # dann an, wenn das Programm mit einer Ausnahme endet.
    def __enter__(self) -> "DriveBase":
        self.connect()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.disconnect()
