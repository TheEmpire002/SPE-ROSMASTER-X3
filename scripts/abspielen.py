#!/usr/bin/env python3
"""Spielt eine gespeicherte Fahrt am eigenen Rechner nach -- ohne Roboter.

Eine Fahrt entsteht mit "Fahrt speichern" in der Web-App (Fahrtenschreiber,
siehe ``rosmaster_assist/aufzeichnung.py``). Holen lässt sie sich über
``http://<roboter>:8000/aufzeichnung/liste`` und
``/aufzeichnung/datei/<name>``.

    python scripts/abspielen.py fahrt_20261103_141500.jsonl.gz
    python scripts/abspielen.py fahrt.jsonl.gz --svg fahrt.svg
    python scripts/abspielen.py fahrt.jsonl.gz --erkunder

Ohne Schalter gibt es eine Zusammenfassung: Dauer, Scan-Rate, Befehle je
Quelle, wie oft der Bremsweg-Schutz eingegriffen hat, harte Halts und
Zustandswechsel mit Uhrzeit.

``--svg`` zeichnet den Weg (Pose der Odometrie) und die Lidar-Punkte
einiger Scans in diesen Rahmen -- gut, um Türen und Engstellen zu sehen.
Achtung: Die Pose stammt aus der Odometrie und driftet; wurde der Roboter
hochgehoben, springt sie nicht mit.

``--erkunder`` legt die aufgezeichneten Scans und Geschwindigkeiten dem
aktuellen Erkunder vor und vergleicht seine Befehle mit den damaligen. So
prüft man eine geänderte Ausweichlogik an echten Fahrten. Volle
Übereinstimmung ist nicht zu erwarten: Am Roboter wählt der Erkunder
zufällig, und die Odometrie wird nur im 20-Hz-Takt mitgeschrieben.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
from collections import Counter

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "robot" / "src"))

from rosmaster_assist import aufzeichnung as az  # noqa: E402

#: Ab dieser Abweichung (m/s bzw. rad/s) gilt ein Befehl als verändert.
#: Kleiner als jede Stufe, die der Erkunder bewusst wählt.
TOLERANZ = 0.02
#: So viele Scans landen höchstens im SVG -- mehr wird unlesbar.
SVG_SCANS = 12


def _geaendert(a, b) -> bool:
    return any(abs(x - y) > TOLERANZ for x, y in zip(a, b))


def zusammenfassung(kopf: dict, ereignisse: list[dict]) -> None:
    print(f"Aufgenommen: {kopf.get('erstellt', '?')}  Antrieb: {kopf.get('antrieb', '?')}"
          f"  Simulation: {kopf.get('mock', '?')}")
    if kopf.get("notiz"):
        print(f"Notiz: {kopf['notiz']}")
    if not ereignisse:
        print("Keine Ereignisse.")
        return
    dauer = ereignisse[-1]["t"] - ereignisse[0]["t"]
    arten = Counter(e["art"] for e in ereignisse)
    scans = arten.get("scan", 0)
    print(f"Dauer: {dauer:.1f} s, {len(ereignisse)} Ereignisse")
    print(f"Lidar: {scans} Runden ({scans / dauer:.1f} Hz)" if dauer > 0 else f"Lidar: {scans} Runden")

    # Lücken im Lidar zeigen USB-Aussetzer oder einen hängenden Dienst.
    zeiten = [e["zeit"] for e in ereignisse if e["art"] == "scan"]
    luecken = [(a, b) for a, b in zip(zeiten, zeiten[1:]) if b - a > 0.5]
    for a, b in luecken:
        print(f"  Lücke im Lidar: {a:.1f}–{b:.1f} s ({b - a:.1f} s)")

    befehle = [e for e in ereignisse if e["art"] == "befehl"]
    quellen = Counter(e["quelle"] for e in befehle)
    print("Fahrbefehle: " + (", ".join(f"{q} {n}" for q, n in quellen.items()) or "keine"))
    begrenzt = [e for e in befehle if _geaendert(e["soll"], e["safe"])]
    if begrenzt:
        print(f"  davon begrenzt (Tempo/Bremsweg): {len(begrenzt)}")
        for e in begrenzt[:5]:
            print(f"    {e['t']:6.2f} s {e['quelle']}: {e['soll']} -> {e['safe']}")
    halts = [e["t"] for e in ereignisse if e["art"] == "stop"]
    if halts:
        print(f"Harte Halts: {len(halts)} (bei " + ", ".join(f"{t:.1f}" for t in halts[:10]) + " s)")

    print("Zustände:")
    for e in ereignisse:
        if e["art"] == "zustand":
            felder = {k: v for k, v in e.items() if k not in ("t", "art") and v not in (None, False)}
            print(f"  {e['t']:6.2f} s  " + (", ".join(f"{k}={v}" for k, v in felder.items()) or "alles aus"))


def svg(ereignisse: list[dict], pfad: pathlib.Path) -> None:
    posen = [e["pose"] for e in ereignisse if e["art"] == "odo"]
    if not posen:
        sys.exit("Keine Odometrie in der Aufzeichnung.")
    scans = [e for e in ereignisse if e["art"] == "scan"]
    schritt = max(1, len(scans) // SVG_SCANS)
    punkte = []
    wiedergabe = az.Wiedergabe(ereignisse)
    for e in scans[::schritt]:
        wiedergabe.bis(e["zeit"])
        x, y, grad = wiedergabe.pose()
        th = math.radians(grad)
        for w, r in zip(e["w"], e["r"]):
            a = th + math.radians(w / 10.0)
            punkte.append((x + r / 1000.0 * math.cos(a), y + r / 1000.0 * math.sin(a)))
    alle = [(p[0], p[1]) for p in posen] + punkte
    xs, ys = [p[0] for p in alle], [p[1] for p in alle]
    rand = 0.3
    x0, x1, y0, y1 = min(xs) - rand, max(xs) + rand, min(ys) - rand, max(ys) + rand
    m = 600 / max(x1 - x0, y1 - y0)  # Pixel je Meter

    # SVG zählt y nach unten, der Roboter nach links -- deshalb gespiegelt.
    def px(x, y):
        return f"{(x - x0) * m:.1f}", f"{(y1 - y) * m:.1f}"

    def kreis(x, y, r, farbe):
        cx, cy = px(x, y)
        return f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{farbe}"/>'

    teile = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{(x1 - x0) * m:.0f}" '
             f'height="{(y1 - y0) * m:.0f}" style="background:#111">']
    teile += [kreis(x, y, 1.2, "#4da3ff") for x, y in punkte]
    teile.append('<polyline fill="none" stroke="#ffcc33" stroke-width="2" points="'
                 + " ".join(",".join(px(p[0], p[1])) for p in posen) + '"/>')
    teile.append(kreis(posen[0][0], posen[0][1], 5, "#3c3"))
    teile.append(kreis(posen[-1][0], posen[-1][1], 5, "#e33"))
    teile.append("</svg>")
    pfad.write_text("\n".join(teile), encoding="utf-8")
    print(f"SVG geschrieben: {pfad} (grün Start, rot Ende, blau Lidar)")


def erkunder_vergleich(ereignisse: list[dict]) -> None:
    from rosmaster_assist.navigation import explorer as ex
    from rosmaster_assist.navigation import hindernisse as hd

    class OhneAntrieb:
        """Der nachgespielte Erkunder darf nichts ausgeben."""

        def submit(self, befehl, quelle=""):
            return befehl

        def stop(self):
            pass

    def bauen(wiedergabe):
        return ex.Erkunder(OhneAntrieb(), wiedergabe, hd.Hinderniserkennung(), None,
                           odometry=wiedergabe)

    takte = az.erkunder_nachspielen(ereignisse, bauen)
    if not takte:
        print("Kein Erkunder-Abschnitt in der Aufzeichnung.")
        return
    anders = [t for t in takte if t["nachgespielt"] is None
              or _geaendert(t["aufgezeichnet"], t["nachgespielt"])]
    gleich = len(takte) - len(anders)
    print(f"Erkunder nachgespielt: {len(takte)} Takte, {gleich} gleich "
          f"({100 * gleich / len(takte):.0f} %), {len(anders)} anders")
    for t in anders[:10]:
        print(f"  {t['t']:6.2f} s  damals {t['aufgezeichnet']}  jetzt {t['nachgespielt']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Gespeicherte Fahrt nachspielen")
    parser.add_argument("datei", type=pathlib.Path)
    parser.add_argument("--svg", type=pathlib.Path, help="Weg und Lidar als SVG zeichnen")
    parser.add_argument("--erkunder", action="store_true",
                        help="Scans dem aktuellen Erkunder vorlegen und vergleichen")
    args = parser.parse_args()

    kopf, ereignisse = az.laden(args.datei)
    zusammenfassung(kopf, ereignisse)
    if args.svg:
        svg(ereignisse, args.svg)
    if args.erkunder:
        erkunder_vergleich(ereignisse)


if __name__ == "__main__":
    main()
