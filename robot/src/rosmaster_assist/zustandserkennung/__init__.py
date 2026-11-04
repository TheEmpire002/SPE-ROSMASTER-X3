"""Zustandserkennung (Healthbot): zu einer Person fahren, der es vermutlich
nicht gut geht, ihren Zustand einschätzen und bei Bedarf Hilfe holen.

* ``bewertung`` -- die vier Zeichen (liegend, regungslos, keine Antwort,
  am Boden) als reine Rechnung, ohne Hardware testbar;
* ``ablauf`` -- der Ablauf als Zustandsautomat (``Healthbot``).

Seite in der Web-App: ``/healthbot``. Schnittstelle: ``docs/api.md``.
"""

from .ablauf import Healthbot

__all__ = ["Healthbot"]
