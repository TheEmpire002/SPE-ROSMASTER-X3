#!/usr/bin/env python3
"""Startet alle Testskripte gleichzeitig und meldet nur, was schiefgeht.

    python scripts/alle_tests.py            # alle Tests
    python scripts/alle_tests.py ziel gesten   # nur Dateien, deren Name das enthält

Die Skripte laufen jeweils in einem eigenen Prozess. Nacheinander brauchten
sie über zehn Minuten, weil einige Simulationen lange rechnen. Parallel
bestimmt nur noch das langsamste die Dauer.

Ausgegeben wird je Skript eine Zeile. Die volle Ausgabe kommt nur bei
Fehlschlägen, damit man (und ein KI-Assistent) nicht seitenweise
bestandene Prüfungen lesen muss.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

WURZEL = Path(__file__).resolve().parent.parent
MUSTER = [
    "Marlon/autonomes_fahren/test_*.py",
    "Marlon/diagnose/test_*.py",
    "Marlon/interaktion/test_*.py",
]
# Ein hängender Test soll den Lauf nicht ewig blockieren.
ZEITLIMIT_S = 900


def finden(filter_: list[str]) -> list[Path]:
    dateien = sorted({p for m in MUSTER for p in WURZEL.glob(m)})
    if filter_:
        dateien = [p for p in dateien if any(f in p.name for f in filter_)]
    return dateien


def ausfuehren(datei: Path) -> tuple[Path, int, str, float]:
    start = time.monotonic()
    # UTF-8 erzwingen: Unter Windows bricht print() sonst an Umlauten ab.
    umgebung = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        lauf = subprocess.run(
            [sys.executable, str(datei)], cwd=WURZEL, env=umgebung,
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=ZEITLIMIT_S,
        )
        code, ausgabe = lauf.returncode, lauf.stdout + lauf.stderr
    except subprocess.TimeoutExpired:
        code, ausgabe = -1, f"Zeitlimit {ZEITLIMIT_S} s überschritten"
    return datei, code, ausgabe, time.monotonic() - start


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("filter", nargs="*", help="nur Testdateien, deren Name das enthält")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    args = parser.parse_args()

    dateien = finden(args.filter)
    if not dateien:
        print("Keine Testdateien gefunden.")
        return 1

    start = time.monotonic()
    fehlgeschlagen = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for zukunft in as_completed([pool.submit(ausfuehren, d) for d in dateien]):
            datei, code, ausgabe, dauer = zukunft.result()
            name = datei.relative_to(WURZEL).as_posix()
            # Manche Skripte enden mit Exit 0, zählen aber FEHLER-Zeilen.
            fehler = sum(1 for z in ausgabe.splitlines() if "FEHLER" in z)
            letzte = (ausgabe.strip().splitlines() or [""])[-1]
            ok = code == 0 and fehler == 0
            print(f"{'ok    ' if ok else 'FEHLER'} {name} ({dauer:.0f} s) :: {letzte}", flush=True)
            if not ok:
                fehlgeschlagen.append((name, ausgabe))

    for name, ausgabe in fehlgeschlagen:
        print(f"\n===== {name} =====\n{ausgabe.strip()}")
    print(f"\n{len(dateien) - len(fehlgeschlagen)}/{len(dateien)} bestanden "
          f"in {time.monotonic() - start:.0f} s")
    return 1 if fehlgeschlagen else 0


if __name__ == "__main__":
    sys.exit(main())
