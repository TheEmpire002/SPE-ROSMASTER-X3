"""Prüft, dass die Posenerkennung alte Kamerabilder nicht als frisch meldet.

    python Marlon/interaktion/test_pose_bildalter.py

Hintergrund (Testprotokoll 04.11.2026): Eine Kamera ruckelte stark, und der
Roboter erkannte Personen, die nicht (mehr) da waren. Hängt die Kamera,
lieferte die Pose vorher dasselbe Bild immer wieder als neue Erkennung.

Geprüft ohne Hailo und ohne Kamera (Attrappe):

* dasselbe Bild wird nicht zweimal gerechnet (keine neue Erkennung);
* ein zu altes Bild ergibt "niemand gesehen" und wird gezählt;
* der Status zeigt Bildalter und Zahl der veralteten Bilder;
* ohne neue Bilder verschwinden die Personen nach kurzer Zeit.
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist.sensors import pose as ps  # noqa: E402

fehler = 0


def pruefe(name, bedingung, ergebnis=""):
    global fehler
    print(("OK    " if bedingung else "FEHLER"), name, ergebnis)
    fehler += not bedingung


class Kamera:
    """Attrappe: liefert ein festes Bild mit einstellbarer Aufnahmezeit."""

    name = "personen"
    fov_deg = 45.0

    def __init__(self):
        self.aufnahme = time.monotonic()

    def latest_mit_zeit(self):
        return (b"kein echtes jpeg", "image/jpeg"), self.aufnahme


class Dekodierer:
    """Attrappe für cv2/numpy: zählt, wie oft ein Bild dekodiert wurde.

    Liefert kein Bild (wie cv2 bei kaputten Daten), damit _erkennen() danach
    ohne Hailo endet. So prüft der Test dasselbe mit und ohne OpenCV.
    """

    IMREAD_COLOR = 1
    uint8 = "uint8"

    def __init__(self):
        self.aufrufe = 0

    def frombuffer(self, daten, dtype=None):
        return daten

    def imdecode(self, daten, flag):
        self.aufrufe += 1
        return None


kamera = Kamera()
dekodierer = Dekodierer()
# mock=False wie am Roboter; ohne Hailo bleibt sie "nicht verfügbar", aber
# _erkennen() ist trotzdem aufrufbar.
pose = ps.Posenerkennung(kamera, mock=False)
pose._cv2 = pose._np = dekodierer

# Zu altes Bild: niemand gesehen, gezählt, nicht gerechnet
kamera.aufnahme = time.monotonic() - (config.POSE_MAX_BILDALTER_S + 1.0)
ergebnis = pose._erkennen(None)
pruefe("altes Bild ergibt niemanden", ergebnis == [], repr(ergebnis))
pruefe("altes Bild wird nicht gerechnet", dekodierer.aufrufe == 0, dekodierer.aufrufe)
st = pose.status()
pruefe("Status zählt das veraltete Bild", st["veraltete_bilder"] == 1, st["veraltete_bilder"])
pruefe("Status zeigt das Bildalter", st["bild_alter_s"] is not None
       and st["bild_alter_s"] > config.POSE_MAX_BILDALTER_S, st["bild_alter_s"])

# Dasselbe Bild noch einmal: gar kein Ergebnis (nichts Neues)
ergebnis = pose._erkennen(None)
pruefe("dasselbe Bild liefert kein neues Ergebnis", ergebnis is None, repr(ergebnis))
pruefe("dasselbe Bild wird nicht noch einmal gerechnet", dekodierer.aufrufe == 0, dekodierer.aufrufe)
pruefe("doppeltes Bild zählt nicht als veraltet", pose.status()["veraltete_bilder"] == 1)

# Ein frisches Bild wird dagegen gerechnet, aber nur einmal
kamera.aufnahme = time.monotonic()
pose._erkennen(None)
pruefe("frisches Bild wird gerechnet", dekodierer.aufrufe == 1, dekodierer.aufrufe)
pose._erkennen(None)
pruefe("frisches Bild wird nur einmal gerechnet", dekodierer.aufrufe == 1, dekodierer.aufrufe)

# Personen verschwinden, wenn keine neuen Erkennungen kommen: Eine einmal
# gemerkte Person darf eine hängende Kamera nicht überleben.
person = ps._person([0.4, 0.2, 0.6, 0.9], [[0.5, 0.5, 0.9]] * 17, 0.9, 4 / 3, 45.0)
pose._merken([person], time.monotonic())
pose._aktiv.set()
pruefe("frisch gemerkte Person ist sichtbar", len(pose.personen()) == 1)
with pose._lock:
    pose._zeit -= ps._MAX_ALTER_S + 0.1  # so tun, als käme seither nichts Neues
pruefe("ohne neue Bilder ist sie bald weg", pose.personen() == [])

print()
print("alles bestanden" if not fehler else f"{fehler} Fehler")
sys.exit(1 if fehler else 0)
