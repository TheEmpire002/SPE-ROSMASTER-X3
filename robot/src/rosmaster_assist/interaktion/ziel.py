"""Wem folgt er? Personen über Bilder verfolgen, ohne sie zu unterscheiden.

Zwei Bausteine, beide ohne Hardware testbar:

- ``PersonenTracker`` ordnet die Personen jedes Bildes einander zu. Je
  Person sagt ein kleiner Kalman-Filter den Kamerawinkel voraus und rechnet
  dabei die Eigendrehung des Roboters heraus (Board-Drehrate, nicht
  ``theta``: der Winkel kam lange von der starren IMU). Die Track-Nummern
  sind nur intern: Sie halten Gesten zweier Personen auseinander und das
  Folgen ruhig, wenn jemand vorbeiläuft.
- ``ZielWahl`` sagt, welchem Track er folgt. Ist die Person weg und kommt
  wieder, folgt er ihr weiter, egal welche Nummer sie jetzt hat.

Bis 04.11. wurde das Ziel zusätzlich am Aussehen wiedererkannt (Ziel-Sperre)
und Personen hießen in der App "Person 1, 2, ...". Wer aus dem Bild ging
und zurückkam, galt oft als fremd, und er blieb stehen. Deshalb raus.

Alles bleibt im RAM und wird beim Ausschalten gelöscht.
"""

from __future__ import annotations

import math

from .. import config

# Zustände der Zielwahl (Werte gehen so an die App, siehe docs/api.md).
IDLE = "idle"
ERFASSEN = "acquiring_target"
FOLGT = "following"
KURZ_WEG = "target_temporarily_lost"
SUCHT = "searching_for_target"

# Kalman-Modell des Kamerawinkels (Grad). Keine Einstellungen, sondern
# Eigenschaften der Messung: Die Pose-Erkennung trifft die Boxmitte auf etwa
# 1,5° genau; ein gehender Mensch ändert seinen Winkel selten schneller als
# 60°/s (auf 1 m Abstand ~1 m/s quer).
_MESS_STD = 1.5
_BESCHL_STD = 60.0
_V_START_STD = 30.0
_V_MAX = 60.0
# Größte Zeitspanne für eine Vorhersage: Nach einer langen Pause (Pose
# stand) soll die Unsicherheit nicht ins Unendliche wachsen.
_DT_MAX = 0.5
# Wie stark Boxhöhe und Track-Merkmal je Messung nachgeführt werden.
_HOEHE_GLAETTUNG = 0.3
_MERKMAL_GLAETTUNG = 0.3


def bhattacharyya(p, q) -> float | None:
    """Abstand zweier normierter Histogramme: 0 = gleich, 1 = nichts gemein.

    None, wenn eines fehlt -- dann lässt sich über das Aussehen nichts sagen
    (z. B. Rumpf nicht im Bild), und das darf nicht als „passt“ zählen.
    """
    if not p or not q or len(p) != len(q):
        return None
    summe = sum(math.sqrt(max(a, 0.0) * max(b, 0.0)) for a, b in zip(p, q))
    return math.sqrt(max(0.0, 1.0 - summe))


def _hoehe(person: dict) -> float:
    box = person["box"]
    return max(box[3] - box[1], 1e-3)


def _mischen(alt: list | None, neu: list | None, anteil: float) -> list | None:
    if neu is None:
        return alt
    if alt is None or len(alt) != len(neu):
        return list(neu)
    return [a + anteil * (b - a) for a, b in zip(alt, neu)]


class Track:
    """Eine Person über mehrere Bilder: Winkel, Winkelgeschwindigkeit, Aussehen."""

    def __init__(self, track_id: int, person: dict, zeit: float) -> None:
        self.id = track_id
        self.w = float(person["winkel"])
        self.v = 0.0
        # Kovarianz als (P00, P01, P10, P11) -- 2x2 lohnt kein numpy.
        self.p = (_MESS_STD ** 2, 0.0, 0.0, _V_START_STD ** 2)
        self.hoehe = _hoehe(person)
        self.merkmal = _mischen(None, person.get("merkmal"), 1.0)
        self.person = person
        self.zeit = zeit
        self.gemessen = True

    def vorhersagen(self, dt: float, omega: float) -> None:
        """Konstante Winkelgeschwindigkeit; Eigendrehung herausrechnen.

        Dreht der Roboter mit omega gegen den Uhrzeigersinn, wandert jede
        Person im Bild um omega*dt nach rechts (kleinerer Winkel).
        """
        if dt <= 0:
            return
        self.w += self.v * dt - math.degrees(omega) * dt
        p00, p01, p10, p11 = self.p
        q = _BESCHL_STD ** 2
        self.p = (
            p00 + dt * (p01 + p10) + dt * dt * p11 + q * dt ** 4 / 4,
            p01 + dt * p11 + q * dt ** 3 / 2,
            p10 + dt * p11 + q * dt ** 3 / 2,
            p11 + q * dt * dt,
        )

    def gate(self) -> float:
        """Zuordnungsfenster: mindestens ZIEL_GATE_GRAD, wächst mit der Unsicherheit."""
        g = config.ZIEL_GATE_GRAD
        return min(max(g, 3.0 * math.sqrt(self.p[0] + _MESS_STD ** 2)), 3.0 * g)

    def messen(self, person: dict, zeit: float) -> None:
        p00, p01, p10, p11 = self.p
        s = p00 + _MESS_STD ** 2
        k0, k1 = p00 / s, p10 / s
        abweichung = float(person["winkel"]) - self.w
        self.w += k0 * abweichung
        self.v = max(-_V_MAX, min(_V_MAX, self.v + k1 * abweichung))
        self.p = ((1 - k0) * p00, (1 - k0) * p01, p10 - k1 * p00, p11 - k1 * p01)
        self.hoehe += _HOEHE_GLAETTUNG * (_hoehe(person) - self.hoehe)
        self.merkmal = _mischen(self.merkmal, person.get("merkmal"), _MERKMAL_GLAETTUNG)
        self.person = person
        self.zeit = zeit
        self.gemessen = True


class PersonenTracker:
    """Ordnet die Personen jedes Bildes den bekannten Tracks zu."""

    def __init__(self) -> None:
        self.leeren()

    def leeren(self) -> None:
        self.tracks: list[Track] = []
        self._naechste_id = 1
        self._zeit: float | None = None

    def track(self, track_id) -> Track | None:
        return next((t for t in self.tracks if t.id == track_id), None)

    def aktualisieren(self, personen: list[dict], zeit: float, omega: float = 0.0) -> list[Track]:
        """Ein Bild verarbeiten. ``omega``: gemessene Drehrate des Roboters (rad/s)."""
        dt = 0.0 if self._zeit is None else min(max(zeit - self._zeit, 0.0), _DT_MAX)
        self._zeit = zeit if self._zeit is None else max(self._zeit, zeit)
        for t in self.tracks:
            t.vorhersagen(dt, omega)
            t.gemessen = False

        # Gierige Zuordnung nach Kosten: bei zwei, drei Personen genügt das,
        # die ungarische Methode brächte nur eine Abhängigkeit mehr.
        paare = []
        for i, t in enumerate(self.tracks):
            gate = t.gate()
            for j, p in enumerate(personen):
                abweichung = abs(float(p["winkel"]) - t.w)
                if abweichung > gate:
                    continue
                h = _hoehe(p)
                verhaeltnis = min(h, t.hoehe) / max(h, t.hoehe)
                if verhaeltnis < config.ZIEL_HOEHE_VERHAELTNIS:
                    continue
                d = bhattacharyya(t.merkmal, p.get("merkmal"))
                kosten = abweichung / gate + 0.5 * abs(math.log(verhaeltnis)) + (d or 0.0)
                paare.append((kosten, i, j))
        paare.sort()
        vergeben_t, vergeben_p = set(), set()
        for _, i, j in paare:
            if i in vergeben_t or j in vergeben_p:
                continue
            vergeben_t.add(i)
            vergeben_p.add(j)
            self.tracks[i].messen(personen[j], zeit)

        for j, p in enumerate(personen):
            if j not in vergeben_p:
                self.tracks.append(Track(self._naechste_id, p, zeit))
                self._naechste_id += 1

        self.tracks = [t for t in self.tracks if zeit - t.zeit <= config.ZIEL_TRACK_HALTEN_S]
        return self.tracks


class ZielWahl:
    """Wem er folgt: der Person im Bild -- ohne Personen auseinanderzuhalten.

    Er bleibt beim selben Track, solange der gemessen wird; sonst spränge er
    zwischen zwei Personen im selben Bild hin und her. Ist der Track weg,
    nimmt er nach ``ZIEL_WECHSEL_S`` die Person, die dem letzten Winkel am
    nächsten steht -- auch wenn der Tracker ihr eine neue Nummer gegeben
    hat. Genau das ist gewollt: Wer kurz aus dem Bild geht und zurückkommt,
    ist für den Tracker ein neuer Track, aber dieselbe Person.

    Bis 04.11. stand hier eine Ziel-Sperre, die das Ziel am Aussehen
    wiedererkennen musste. Klappte das nicht (Licht, Drehung), galt die
    zurückkehrende Person als fremd und er blieb stehen.
    """

    def __init__(self) -> None:
        self.freigeben()

    def freigeben(self, grund: str = "") -> None:
        """Nicht mehr folgen."""
        self.zustand = IDLE
        self.id = None
        self.grund = grund
        self.sicherheit = 0.0
        self._winkel: float | None = None
        self._weg_seit: float | None = None

    def erfassen(self, track_id, jetzt: float) -> None:
        """Folgen beginnen: dem Track, der die Geste machte, oder (None) der größten Person."""
        self.freigeben()
        self.zustand = ERFASSEN
        self.id = track_id

    def suchen(self, jetzt: float) -> None:
        """Person verloren: die nächste, die auftaucht, ist es."""
        if self.zustand != IDLE:
            self.zustand = SUCHT

    def als_dict(self) -> dict:
        return {
            "zustand": self.zustand,
            "id": self.id,
            "sicherheit": round(self.sicherheit, 2),
            "grund": self.grund,
        }

    def zuordnen(self, tracks: list[Track], bild_zeit, jetzt: float) -> Track | None:
        """Den Track, dem er folgt -- nur, wenn er im letzten Bild gemessen wurde."""
        if self.zustand == IDLE:
            return None
        gemessen = [t for t in tracks if t.gemessen]
        ziel = next((t for t in gemessen if t.id == self.id), None)
        if ziel is None and gemessen:
            # Kurz verdeckt (jemand läuft vorbei)? Erst nach ZIEL_WECHSEL_S
            # eine andere Person nehmen. Beim Start und in der Suche sofort.
            weg_lange = self._weg_seit is not None and jetzt - self._weg_seit >= config.ZIEL_WECHSEL_S
            if self.id is None or self.zustand in (ERFASSEN, SUCHT) or weg_lange:
                ziel = self._naechste(gemessen)
        if ziel is None:
            if self._weg_seit is None:
                self._weg_seit = jetzt
            if self.zustand == FOLGT:
                self.zustand = KURZ_WEG
            return None
        self._weg_seit = None
        self.id = ziel.id
        self._winkel = ziel.w
        self.zustand = FOLGT
        self.sicherheit = 1.0
        return ziel

    def _naechste(self, gemessen: list[Track]) -> Track:
        """Dort, wo er zuletzt jemanden sah; ohne Anhaltspunkt die größte Person."""
        if self._winkel is None:
            return max(gemessen, key=lambda t: t.hoehe)
        return min(gemessen, key=lambda t: (abs(t.w - self._winkel), -t.hoehe))
