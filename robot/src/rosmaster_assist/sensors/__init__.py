"""Sensorik: Lidar, Kamera, Vitaldaten.

Noch leer. Geplant:

- ``lidar.py``   Abstandsdaten einlesen, Grundlage für Hinderniserkennung
                 und später SLAM
- ``camera.py``  Bildstream für die App, optional Personenerkennung über
                 den AI HAT+
- ``vitals.py``  Auswertung der ESP-Daten (Bewegungslosigkeit erkennen)

Dabei demselben Muster folgen wie ``drive/``: eine abstrakte Basisklasse plus
ein Mock, damit auch ohne Hardware entwickelt werden kann.
"""
