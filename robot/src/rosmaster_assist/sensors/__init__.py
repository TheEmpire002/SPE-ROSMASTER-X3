"""Sensors: lidar, camera, vital data.

Still empty. Planned:

- ``lidar.py``   Read the Slamtec RPLIDAR A1 (USB serial, 115200 baud),
                 basis for obstacle detection and later SLAM.
                 Caution: use a fixed device path via ``/dev/serial/by-id/``,
                 otherwise the port can get swapped with the expansion board --
                 see docs/hardware.md.
- ``camera.py``  Image stream for the app, optionally person detection via
                 the AI HAT+
- ``vitals.py``  Evaluate the ESP data (detect lack of movement)

Follow the same pattern as ``drive/``: an abstract base class plus a mock,
so development is possible without hardware too.
"""
