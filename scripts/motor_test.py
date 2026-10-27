#!/usr/bin/env python3
"""Findet heraus, warum sich die Motoren nicht drehen.

Läuft direkt auf dem Roboter, ohne Server und ohne App:

    robot/.venv/bin/python scripts/motor_test.py

RÄDER AUFBOCKEN. Das Skript lässt sie absichtlich drehen.

Getestet wird der Reihe nach:

1. Akkuspannung      -- Motoren brauchen Akkustrom, die Logik nicht
2. set_motor         -- direkte Radansteuerung (bekannt funktionierend)
3. set_car_motion    -- was unser Treiber benutzt
4. set_car_type      -- ob dem Board erst der Fahrzeugtyp fehlt

Nach dem Lauf steht in der Zusammenfassung, welcher Weg funktioniert hat.
"""

from __future__ import annotations

import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    sys.exit("Rosmaster_Lib nicht gefunden -- läuft das hier wirklich auf dem Pi?")

DAUER = 2.0  # Sekunden pro Versuch
results: dict[str, str] = {}


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def beobachtung(frage: str) -> bool:
    """Fragt, ob sich etwas gedreht hat. Das kann nur ein Mensch sehen."""
    antwort = input(f"    {frage} [j/n] ").strip().lower()
    return antwort.startswith("j")


def main() -> int:
    print(__doc__.split("Getestet")[0])
    if not beobachtung("Sind die Räder aufgebockt und drehen frei?"):
        print("\n  Abgebrochen. Erst aufbocken.\n")
        return 1

    bot = Rosmaster()
    bot.create_receive_threading()
    time.sleep(0.5)

    # --- 1. Stromversorgung ---
    headline("1. Akku")
    try:
        volt = bot.get_battery_voltage()
        print(f"    Akkuspannung: {volt} V")
        if volt < 1.0:
            print("    -> Keine Messung. Ist der Motorstrom-Schalter an?")
            print("       Der Pi läuft auch ohne Akku über USB -- die Motoren nicht.")
        elif volt < 10.0:
            print("    -> Sehr niedrig. Bei leerem Akku drehen die Motoren nicht.")
        results["Akku"] = f"{volt} V"
    except Exception as exc:
        print(f"    Nicht auslesbar: {exc}")
        results["Akku"] = "unbekannt"

    # --- 2. set_motor: der bekannt funktionierende Weg ---
    headline("2. set_motor(100, 100, 100, -100)")
    print(f"    Läuft {DAUER:.0f} Sekunden …")
    bot.set_motor(100, 100, 100, -100)
    time.sleep(DAUER)
    bot.set_motor(0, 0, 0, 0)
    results["set_motor"] = "ja" if beobachtung("Haben sich Räder gedreht?") else "NEIN"

    # --- 3. set_car_motion: das, was unser Treiber benutzt ---
    headline("3. set_car_motion -- wie im Treiber")
    for vx in (0.16, 0.4, 1.0):
        print(f"    vx = {vx} m/s …")
        bot.set_car_motion(vx, 0.0, 0.0)
        time.sleep(DAUER)
        bot.set_car_motion(0.0, 0.0, 0.0)
        if beobachtung(f"Gedreht bei vx={vx}?"):
            results["set_car_motion"] = f"ja, ab {vx} m/s"
            break
    else:
        results["set_car_motion"] = "NEIN, bei keiner Geschwindigkeit"

    # --- 4. Fahrzeugtyp setzen und erneut versuchen ---
    # Das Board rechnet die Mecanum-Kinematik selbst. Kennt es den
    # Fahrzeugtyp nicht, kann set_car_motion wirkungslos bleiben --
    # set_motor umgeht diese Rechnung und funktioniert trotzdem.
    headline("4. set_car_type(X3), dann nochmal set_car_motion")
    try:
        bot.set_car_type(1)  # 1 = X3
        time.sleep(0.5)
        bot.set_car_motion(0.4, 0.0, 0.0)
        time.sleep(DAUER)
        bot.set_car_motion(0.0, 0.0, 0.0)
        results["nach set_car_type"] = (
            "ja" if beobachtung("Jetzt gedreht?") else "NEIN"
        )
    except Exception as exc:
        print(f"    Fehlgeschlagen: {exc}")
        results["nach set_car_type"] = f"Fehler: {exc}"

    # --- Zusammenfassung ---
    bot.set_motor(0, 0, 0, 0)
    headline("Ergebnis")
    for name, value in results.items():
        print(f"    {name:<20} {value}")
    print("\n  Diese Ausgabe bitte zurückmelden.\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n  Abgebrochen.\n")
        sys.exit(1)
