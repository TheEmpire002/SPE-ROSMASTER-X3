"""Sensorik: Lidar, Kamera, Vitaldaten.

Noch leer. Geplant:

- ``lidar.py``   Slamtec RPLIDAR A1 auslesen (USB-Seriell, 115200 Baud),
                 Grundlage für Hinderniserkennung und später SLAM.
                 Achtung: fester Gerätepfad über ``/dev/serial/by-id/``, sonst
                 kann der Port mit dem Expansion Board vertauscht werden --
                 siehe docs/hardware.md.
- ``camera.py``  Bildstream für die App, optional Personenerkennung über
                 den AI HAT+
- ``vitals.py``  Auswertung der ESP-Daten (Bewegungslosigkeit erkennen)

Dabei demselben Muster folgen wie ``drive/``: eine abstrakte Basisklasse plus
ein Mock, damit auch ohne Hardware entwickelt werden kann.
"""
