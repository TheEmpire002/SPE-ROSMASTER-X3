"""Health check (healthbot): drive to a person who is probably not well,
assess their state and get help if needed.

* ``assessment`` -- the four signs (lying, motionless, no answer, on the
  floor) as pure computation, testable without hardware;
* ``procedure`` -- the procedure as a state machine (``Healthbot``);
* ``simulation`` -- mock only: a lying person for camera and lidar.

Page in the web app: ``/healthbot``. Interface: ``docs/api.md``.
"""

from .procedure import Healthbot
from .simulation import SCENES, Simulation

__all__ = ["Healthbot", "SCENES", "Simulation"]
