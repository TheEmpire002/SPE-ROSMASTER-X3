"""Prüft die Problem-Regeln der Diagnose (robot/src/rosmaster_assist/diagnose.py)
ohne Roboter: Akku, Quittieren, Vorfälle, Kernel-Log, Log-Puffer.

    python Marlon/diagnose/test_diagnose.py

Statt des Hintergrund-Threads werden die Prüfungen direkt aufgerufen;
Treiber und Watchdog sind Attrappen mit genau den Feldern, die die
Diagnose liest.
"""

import logging
import pathlib
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist import config  # noqa: E402
from rosmaster_assist import diagnose as d  # noqa: E402


class Treiber:
    name = "mock"
    connected = True

    def __init__(self) -> None:
        self.volt = 12.4

    def read_battery(self):
        return self.volt


def neue_diagnose():
    watchdog = SimpleNamespace(current_command=SimpleNamespace(vx=0.0, vy=0.0, omega=0.0))
    treiber = Treiber()
    return d.Diagnose(treiber, watchdog, mock=True), treiber


def problem(diag, pid):
    return next((p for p in diag.status()["probleme"] if p["id"] == pid), None)


def test_akku_voll_ist_ok():
    diag, _ = neue_diagnose()
    diag._akku_pruefen(time.time())
    s = diag.status()
    assert s["gesamt"]["stufe"] == "ok", s["gesamt"]
    assert s["probleme"] == []


def test_akku_kritisch_und_quittieren():
    diag, treiber = neue_diagnose()
    treiber.volt = config.AKKU_KRITISCH_V - 0.2
    diag._akku_pruefen(time.time())
    p = problem(diag, "akku_kritisch")
    assert p and p["besteht"] and p["zustand"] and not p["quittiert"]
    assert diag.status()["gesamt"]["stufe"] == "kritisch"
    # Gleichzeitig "schwach" wäre doppelt gemoppelt.
    assert problem(diag, "akku_schwach") is None

    assert diag.quittieren("akku_kritisch") == 1
    assert diag.status()["gesamt"]["stufe"] == "ok"

    # Besteht weiter -> bleibt quittiert, kein erneutes Rot bei jeder Messung.
    diag._akku_pruefen(time.time())
    assert problem(diag, "akku_kritisch")["quittiert"]

    # Erholt sich und bricht erneut ein -> neues Auftreten, wieder offen.
    treiber.volt = 12.4
    diag._akku_pruefen(time.time())
    assert not problem(diag, "akku_kritisch")["besteht"]
    time.sleep(0.01)
    treiber.volt = config.AKKU_KRITISCH_V - 0.2
    diag._akku_pruefen(time.time())
    p = problem(diag, "akku_kritisch")
    assert not p["quittiert"] and p["anzahl"] == 2, p


def test_spannungseinbruch():
    diag, treiber = neue_diagnose()
    jetzt = time.time()
    diag._akku_pruefen(jetzt)
    treiber.volt = 12.4 - config.SPANNUNGSEINBRUCH_V - 0.1
    diag._akku_pruefen(jetzt + 1)
    assert problem(diag, "spannungseinbruch") is not None
    arten = [e["art"] for e in diag.status()["ereignisse"]]
    assert "spannung" in arten


def test_vorfall_zaehlt_und_verfaellt():
    diag, _ = neue_diagnose()
    jetzt = time.time()
    for i in range(3):
        diag._vorfall("usb_ueberlast", "warnung", "Titel", "was", ["tun"], zeit=jetzt + i)
    p = problem(diag, "usb_ueberlast")
    assert p["anzahl"] == 3 and not p["besteht"] and not p["zustand"]

    # Außerhalb des Fensters vergessen.
    diag._probleme["usb_ueberlast"].zuletzt = jetzt - config.PROBLEM_FENSTER_S - 1
    assert problem(diag, "usb_ueberlast") is None


def test_quittierter_vorfall_kommt_wieder():
    diag, _ = neue_diagnose()
    jetzt = time.time()
    diag._vorfall("x", "warnung", "T", "w", [], zeit=jetzt)
    diag.quittieren()
    assert problem(diag, "x")["quittiert"]
    diag._vorfall("x", "warnung", "T", "w", [], zeit=jetzt + 5)
    assert not problem(diag, "x")["quittiert"]


def test_kernel_usb_abmeldung():
    diag, _ = neue_diagnose()
    diag._kernel_zeile(time.time(), "usb 3-1.2: USB disconnect, device number 7")
    assert problem(diag, "usb_abmeldung") is not None
    texte = [e["text"] for e in diag.status()["ereignisse"]]
    assert any("usb 3-1.2" in t for t in texte), texte


def test_unterspannung_aus_vcgencmd():
    diag, _ = neue_diagnose()
    diag._drossel_pruefen(0x50005)  # jetzt Unterspannung + seit dem Start
    assert problem(diag, "unterspannung")["schwere"] == "kritisch"
    assert problem(diag, "unterspannung_frueher") is not None
    diag._drossel_pruefen(0x50000)  # vorbei, nur noch "seit dem Start"
    assert not problem(diag, "unterspannung")["besteht"]


def test_log_puffer_letzte_id():
    puffer = d.LogPuffer(3)
    lg = logging.getLogger("rosmaster_assist.test")
    lg.addHandler(puffer)
    lg.setLevel(logging.INFO)
    try:
        for i in range(5):
            lg.warning("Zeile %d", i)
        lg.error("kaputt")
    finally:
        lg.removeHandler(puffer)
    zeilen, letzte = puffer.zeilen()
    assert letzte == 6 and [z["id"] for z in zeilen] == [4, 5, 6]
    # Browser ist "weiter" als der Server (Neustart) -> echte letzte Nummer.
    zeilen, letzte = puffer.zeilen(99999)
    assert zeilen == [] and letzte == 6
    assert puffer.fehler()[0] == 1


def test_akku_prozent():
    assert d.akku_prozent(None) is None
    assert d.akku_prozent(12.6) == 100
    assert d.akku_prozent(9.0) == 0


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    fehler = 0
    for name, f in tests:
        try:
            f()
            print(f"ok      {name}")
        except AssertionError as e:
            fehler += 1
            print(f"FEHLER  {name}: {e}")
    print(f"\n{len(tests) - fehler}/{len(tests)} bestanden")
    sys.exit(1 if fehler else 0)
