"""Ziel-Lock beim Folgen: Personen über Bilder verfolgen, Ziel wiedererkennen.

Früher wählte das Folgen in jedem Bild die Person, deren Winkel dem letzten
am nächsten lag -- und nach einer Suche einfach die größte. Kreuzte jemand
oder kam das Ziel nach einer Verdeckung nicht als erstes zurück, folgte der
Roboter der falschen Person. Hier stecken zwei Bausteine, beide ohne
Hardware testbar:

- ``PersonenTracker`` gibt jeder Person eine ID, die über Bilder hinweg
  bleibt. Je Track sagt ein kleiner Kalman-Filter den Kamerawinkel voraus
  und rechnet dabei die Eigendrehung des Roboters heraus (Board-Drehrate,
  nicht ``theta``: der Winkel kam lange von der starren IMU).
- ``ZielSperre`` merkt sich, welcher Track das Ziel ist, und das Aussehen
  des Oberkörpers (HSV-Histogramm aus ``sensors/pose.py``). Nach einer
  Verdeckung gilt nur wieder als Ziel, wer über mehrere Bilder deutlich
  besser passt als alle anderen. Im Zweifel hält er an, statt zu raten.

Alles bleibt im RAM und wird beim Loslassen gelöscht: keine Bilder, nur ein
Histogramm mit 128 Zahlen je Person.
"""

from __future__ import annotations

import math

from .. import config

# Zustände des Ziel-Locks (Werte gehen so an die App, siehe docs/api.md).
IDLE = "idle"
ERFASSEN = "acquiring_target"
FOLGT = "following"
KURZ_WEG = "target_temporarily_lost"
SUCHT = "searching_for_target"
SICHER_HALT = "safe_stop"

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


class ZielSperre:
    """Welcher Track ist das Ziel? Zustandsautomat, siehe docs/interaktion.md."""

    def __init__(self) -> None:
        # Aussehen der Person, die zuletzt das Folgen ausgelöst hat bzw.
        # verfolgt wurde. Überdauert Freigabe und sicheres Anhalten, damit er
        # beim nächsten "Folgen" wieder dieselbe Person nimmt statt
        # abzulehnen, wenn mehrere im Bild stehen. Nur im RAM, gelöscht bei
        # vergessen() und nach ZIEL_MERKEN_S.
        self.gemerkt: list | None = None
        self._gemerkt_zeit = 0.0
        self.freigeben()

    # --- Übergänge von außen --------------------------------------------------

    def vergessen(self) -> None:
        """Loslassen durch den Menschen: auch die gemerkte Person löschen."""
        self.gemerkt = None
        self.freigeben()

    def freigeben(self, grund: str = "") -> None:
        """Ziel loslassen und das Gelernte verwerfen (die gemerkte Person bleibt)."""
        self.zustand = IDLE
        self.id = None
        self.referenz: list | None = None
        self.grund = grund
        self.sicherheit = 0.0
        self.mehrdeutig = False
        self._nie_gesperrt = True
        self._proben = 0
        self._seit = 0.0
        self._fehl = 0
        self._letztes_bild = None
        self._unklar_bilder = 0
        self._kandidat_neu()

    def erfassen(self, track_id, jetzt: float, merkmal: list | None = None) -> None:
        """Neues Ziel aufnehmen: eine bestimmte ID oder (None) die einzige Person.

        ``merkmal`` ist das Aussehen der gewählten Person in diesem Moment
        (Antippen/Geste). Damit kennt er sie sofort -- verliert der Tracker
        sie in der Lernphase kurz, findet er genau sie wieder, statt neu zu
        wählen.
        """
        self.freigeben()
        self.zustand = ERFASSEN
        self.id = track_id
        self._nie_gesperrt = track_id is None
        self._seit = jetzt
        if track_id is not None and merkmal is not None:
            self.referenz = list(merkmal)
            self._proben = 1
            self._merken(jetzt)

    def suchen(self, jetzt: float) -> None:
        if self.zustand in (IDLE, SICHER_HALT):
            return
        self.zustand = SUCHT
        self._seit = jetzt
        self._kandidat_neu()

    def sicher_anhalten(self, grund: str, jetzt: float) -> None:
        """Stehen bleiben, bis jemand neu wählt. Das Gelernte wird gelöscht."""
        self.freigeben(grund)
        self.zustand = SICHER_HALT
        self._seit = jetzt

    def als_dict(self) -> dict:
        return {
            "zustand": self.zustand,
            "id": self.id,
            "sicherheit": round(self.sicherheit, 2),
            "grund": self.grund,
        }

    @property
    def prueft_kandidat(self) -> bool:
        """Beim Suchen passt gerade jemand und wird bestätigt -- dann stillhalten,
        sonst dreht er die Person wieder aus dem Bild."""
        return self._kandidat is not None and self.zustand in (KURZ_WEG, SUCHT)

    # --- je Takt ----------------------------------------------------------------

    def zuordnen(self, tracks: list[Track], bild_zeit, jetzt: float) -> Track | None:
        """Den Ziel-Track liefern -- nur, wenn er in diesem Bild gemessen wurde.

        Wird im Regeltakt aufgerufen; gezählt (Bestätigungen, Fehlbilder) wird
        nur bei einem neuen Bild (``bild_zeit`` anders als zuletzt).
        """
        self.mehrdeutig = False
        if self.zustand in (IDLE, SICHER_HALT):
            return None
        neu = bild_zeit is not None and bild_zeit != self._letztes_bild
        self._letztes_bild = bild_zeit
        gemessen = [t for t in tracks if t.gemessen]

        if self.zustand == ERFASSEN and self.id is None:
            # Ohne Antippen nur, wenn die Wahl eindeutig ist -- oder wenn er
            # sich die Person vom letzten Folgen gemerkt hat und sie sicher
            # unter den Sichtbaren wiedererkennt.
            if len(gemessen) > 1:
                if not self._gemerkt_gueltig(jetzt):
                    self.mehrdeutig = True
                    return None
                bester = self._gemerkt_waehlen(gemessen, neu)
                if bester is None:
                    return None
                self.id = bester.id
                self.referenz = list(self.gemerkt)
                self._proben = 1
            elif not gemessen:
                return None
            else:
                self.id = gemessen[0].id
            self._nie_gesperrt = False

        ziel = next((t for t in tracks if t.id == self.id), None) if self.id is not None else None
        if ziel is not None and ziel.gemessen:
            return self._ziel_gemessen(ziel, gemessen, neu, jetzt)

        # Ziel ohne Messung (verdeckt, aus dem Bild, Track verworfen)
        if self.zustand == ERFASSEN:
            if ziel is not None:
                return None
            if self.referenz is None:
                # Noch nichts gelernt: wie ein frischer Start.
                self.id = None
                self._nie_gesperrt = True
                return None
        if self.zustand in (ERFASSEN, FOLGT):
            self.zustand = KURZ_WEG
            self._seit = jetzt
        return self._wiederfinden(gemessen, neu, jetzt)

    # --- intern -----------------------------------------------------------------

    def _kandidat_neu(self) -> None:
        self._kandidat = None
        self._kandidat_bilder = 0
        self._knapp_bilder = 0

    def _merken(self, jetzt: float) -> None:
        self.gemerkt = list(self.referenz) if self.referenz is not None else None
        self._gemerkt_zeit = jetzt

    def _gemerkt_gueltig(self, jetzt: float) -> bool:
        return self.gemerkt is not None and jetzt - self._gemerkt_zeit <= config.ZIEL_MERKEN_S

    def _gemerkt_waehlen(self, gemessen: list[Track], neu: bool) -> Track | None:
        """Die gemerkte Person unter mehreren Sichtbaren bestätigen."""
        treffer = self._vergleichen(self.gemerkt, gemessen, neu)
        if treffer is None:
            # Niemand passt (andere Kleidung, anderes Licht): nach kurzer
            # Zeit wie bisher ablehnen, damit jemand im Bild antippt.
            if neu:
                self._unklar_bilder += 1
            if self._unklar_bilder >= config.ZIEL_BESTAETIGEN_BILDER:
                self.mehrdeutig = True
            return None
        self._unklar_bilder = 0
        if not self._bestaetigen(*treffer, neu):
            return None
        self._kandidat_neu()
        return treffer[0]

    def _vergleichen(self, referenz: list | None, gemessen: list[Track], neu: bool) -> tuple[Track, bool] | None:
        """Bester Kandidat zum Merkmal und ob er klar passt -- oder None.

        Klar heißt: deutlich unter der Schwelle und weit vor dem
        Zweitbesten. Dann reichen weniger Bestätigungsbilder.
        """
        bewertet = sorted(
            (d, t.id, t)
            for t in gemessen
            if (d := bhattacharyya(referenz, t.person.get("merkmal"))) is not None
        )
        if not bewertet or bewertet[0][0] > config.ZIEL_MERKMAL_SCHWELLE:
            self._kandidat_neu()
            return None
        if len(bewertet) > 1 and bewertet[1][0] - bewertet[0][0] < config.ZIEL_MERKMAL_MARGE:
            # Zwei passen fast gleich gut (gleiche Kleidung?): nicht raten.
            if neu:
                self._knapp_bilder += 1
            if self._knapp_bilder >= config.ZIEL_BESTAETIGEN_BILDER:
                self.mehrdeutig = True
            return None
        self._knapp_bilder = 0
        d0 = bewertet[0][0]
        self.sicherheit = 1.0 - d0
        klar = d0 < config.ZIEL_MERKMAL_SCHWELLE / 2 and (
            len(bewertet) == 1 or bewertet[1][0] - d0 >= 2 * config.ZIEL_MERKMAL_MARGE
        )
        return bewertet[0][2], klar

    def _bestaetigen(self, bester: Track, klar: bool, neu: bool) -> bool:
        """Zählt Folgebilder für denselben Kandidaten; True, wenn genug."""
        if self._kandidat != bester.id:
            self._kandidat = bester.id
            self._kandidat_bilder = 1 if neu else 0
        elif neu:
            self._kandidat_bilder += 1
        noetig = config.ZIEL_BESTAETIGEN_SCHNELL_BILDER if klar else config.ZIEL_BESTAETIGEN_BILDER
        return self._kandidat_bilder >= noetig

    def _noch_lernen(self) -> bool:
        # Ging das Ziel schon in der Lernphase verloren, ist das Aussehen
        # noch dünn belegt -- nach dem Wiederfinden weiterlernen statt
        # gleich als bestätigt zu gelten.
        return self.referenz is None or self._proben < config.ZIEL_BESTAETIGEN_BILDER

    def _ziel_gemessen(self, ziel: Track, gemessen: list[Track], neu: bool, jetzt: float) -> Track | None:
        merkmal = ziel.person.get("merkmal")
        d = bhattacharyya(self.referenz, merkmal)
        if d is not None and d > config.ZIEL_MERKMAL_SCHWELLE and self.zustand != ERFASSEN:
            # Der Tracker hat vermutlich eine andere Person auf die ID
            # gelegt (z. B. beim Kreuzen). Nicht folgen; hält es an, wird die
            # ID aufgegeben und neu gesucht.
            if neu:
                self._fehl += 1
            if self._fehl >= config.ZIEL_BESTAETIGEN_BILDER:
                self.id = None
                self._fehl = 0
                if self.zustand == FOLGT:
                    self.zustand = KURZ_WEG
                    self._seit = jetzt
            return None
        self._fehl = 0
        self._kandidat_neu()
        # Steht jemand dicht daneben, könnte das Histogramm Teile der anderen
        # Person enthalten -- dann nichts lernen.
        eindeutig = not any(
            abs(t.w - ziel.w) < 2 * config.ZIEL_GATE_GRAD for t in gemessen if t is not ziel
        )
        if self.zustand == ERFASSEN:
            if neu and eindeutig and merkmal is not None:
                self._proben += 1
                self.referenz = _mischen(self.referenz, merkmal, 1.0 / self._proben)
            if jetzt - self._seit >= config.ZIEL_ERFASSEN_S and (
                self._proben >= config.ZIEL_BESTAETIGEN_BILDER or merkmal is None
            ):
                self.zustand = FOLGT
        elif self.zustand in (KURZ_WEG, SUCHT):
            self.zustand = ERFASSEN if self._noch_lernen() else FOLGT
        elif neu and eindeutig and d is not None and d < config.ZIEL_MERKMAL_SCHWELLE / 2:
            self.referenz = _mischen(self.referenz, merkmal, config.ZIEL_MERKMAL_NACHFUEHREN)
        self.sicherheit = 1.0 - d if d is not None else 0.5
        if self.referenz is not None:
            self._merken(jetzt)
        return ziel

    def _wiederfinden(self, gemessen: list[Track], neu: bool, jetzt: float) -> Track | None:
        """Unter den sichtbaren Personen das Ziel wiedererkennen -- nur sicher."""
        if not gemessen:
            self._kandidat_neu()
            return None

        if self.referenz is None:
            # Ohne gelerntes Aussehen nie jemand anderen übernehmen. Einzige
            # Ausnahme: Es war noch nie jemand gesperrt (Start ohne Person im
            # Bild) und genau eine Person taucht stabil auf.
            if not (self.zustand == SUCHT and self._nie_gesperrt):
                return None
            if len(gemessen) > 1:
                self.mehrdeutig = True
                return None
            bester, klar = gemessen[0], False
        else:
            treffer = self._vergleichen(self.referenz, gemessen, neu)
            if treffer is None:
                return None
            bester, klar = treffer

        if not self._bestaetigen(bester, klar, neu):
            return None

        self.id = bester.id
        self._fehl = 0
        self._kandidat_neu()
        if self.referenz is None:
            # Erste Sperre überhaupt: jetzt das Aussehen lernen.
            self._nie_gesperrt = False
            self.zustand = ERFASSEN
            self._seit = jetzt
        else:
            self.zustand = ERFASSEN if self._noch_lernen() else FOLGT
        return bester
