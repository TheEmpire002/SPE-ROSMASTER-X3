#!/usr/bin/env python3
"""Finds out why the motors do not turn.

Runs directly on the robot, without server and without app:

    robot/.venv/bin/python scripts/motor_test.py

JACK UP THE WHEELS. The script deliberately makes them turn.

Tested in order:

1. Battery voltage   -- the motors need battery power, the logic does not
2. set_motor         -- direct wheel control (known to work)
3. set_car_motion    -- what our driver uses
4. set_car_type      -- whether the board is missing the vehicle type first

After the run, the summary shows which path worked.
"""

from __future__ import annotations

import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    sys.exit("Rosmaster_Lib not found -- is this really running on the Pi?")

DURATION = 2.0  # seconds per attempt
results: dict[str, str] = {}


def headline(text: str) -> None:
    print(f"\n{text}\n{'-' * len(text)}")


def observation(question: str) -> bool:
    """Asks whether something turned. Only a human can see that."""
    response = input(f"    {question} [j=yes/n] ").strip().lower()
    return response.startswith("j")


def main() -> int:
    print(__doc__.split("Tested in order")[0])
    if not observation("Are the wheels jacked up and turning freely?"):
        print("\n  Aborted. Jack up the wheels first.\n")
        return 1

    bot = Rosmaster()
    bot.create_receive_threading()
    time.sleep(0.5)

    # --- 1. Power supply ---
    headline("1. Battery")
    try:
        volt = bot.get_battery_voltage()
        print(f"    Battery voltage: {volt} V")
        if volt < 1.0:
            print("    -> No reading. Is the motor power switch on?")
            print("       The Pi also runs on USB without the battery -- the motors do not.")
        elif volt < 10.0:
            print("    -> Very low. With an empty battery the motors do not turn.")
        results["Battery"] = f"{volt} V"
    except Exception as exc:
        print(f"    Cannot be read: {exc}")
        results["Battery"] = "unknown"

    # --- 2. set_motor: the path known to work ---
    headline("2. set_motor(100, 100, 100, -100)")
    print(f"    Running for {DURATION:.0f} seconds …")
    bot.set_motor(100, 100, 100, -100)
    time.sleep(DURATION)
    bot.set_motor(0, 0, 0, 0)
    results["set_motor"] = "yes" if observation("Did the wheels turn?") else "NO"

    # --- 3. set_car_motion: what our driver uses ---
    headline("3. set_car_motion -- as in the driver")
    for vx in (0.16, 0.4, 1.0):
        print(f"    vx = {vx} m/s …")
        bot.set_car_motion(vx, 0.0, 0.0)
        time.sleep(DURATION)
        bot.set_car_motion(0.0, 0.0, 0.0)
        if observation(f"Turned at vx={vx}?"):
            results["set_car_motion"] = f"yes, from {vx} m/s"
            break
    else:
        results["set_car_motion"] = "NO, at no speed"

    # --- 4. Set the vehicle type and try again ---
    # The board computes the mecanum kinematics itself. If it does not know
    # the vehicle type, set_car_motion can have no effect -- set_motor
    # bypasses this computation and works anyway.
    headline("4. set_car_type(X3), then set_car_motion again")
    try:
        bot.set_car_type(1)  # 1 = X3
        time.sleep(0.5)
        bot.set_car_motion(0.4, 0.0, 0.0)
        time.sleep(DURATION)
        bot.set_car_motion(0.0, 0.0, 0.0)
        results["after set_car_type"] = (
            "yes" if observation("Turned now?") else "NO"
        )
    except Exception as exc:
        print(f"    Failed: {exc}")
        results["after set_car_type"] = f"Error: {exc}"

    # --- Summary ---
    bot.set_motor(0, 0, 0, 0)
    headline("Result")
    for name, value in results.items():
        print(f"    {name:<20} {value}")
    print("\n  Please report this output back.\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n  Aborted.\n")
        sys.exit(1)
