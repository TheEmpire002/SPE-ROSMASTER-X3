# Sensor-Check Phase A: direkt ueber Rosmaster_Lib (ohne ROS), Raeder in der Luft.
# Aufruf auf dem Pi:  python3 sensorcheck_a.py  -> schreibt phase_a.txt / phase_a.json
import json
import statistics as st
import sys
import time

from Rosmaster_Lib import Rosmaster

LOG = []
DATA = {}


def log(msg):
    line = time.strftime("%H:%M:%S ") + msg
    print(line, flush=True)
    LOG.append(line)


def stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {"mittel": round(st.mean(vals), 4),
            "std": round(st.pstdev(vals), 4),
            "min": round(min(vals), 4), "max": round(max(vals), 4)}


bot = Rosmaster(car_type=1, com="/dev/myserial")
bot.create_receive_threading()
time.sleep(1.0)


def sample(dur, hz=20):
    """Alle Sensoren fuer dur Sekunden mitschreiben."""
    rows = []
    t_end = time.time() + dur
    while time.time() < t_end:
        try:
            rows.append({
                "t": time.time(),
                "acc": bot.get_accelerometer_data(),
                "gyro": bot.get_gyroscope_data(),
                "mag": bot.get_magnetometer_data(),
                "att": bot.get_imu_attitude_data(),
                "vel": bot.get_motion_data(),
                "enc": bot.get_motor_encoder(),
                "volt": bot.get_battery_voltage(),
            })
        except Exception as e:          # serielle Verbindung weg?
            log(f"FEHLER beim Lesen: {e!r}")
        time.sleep(1.0 / hz)
    return rows


def summary(rows):
    out = {}
    for key, n in (("acc", 3), ("gyro", 3), ("mag", 3), ("att", 3), ("vel", 3)):
        out[key] = [stats([r[key][i] for r in rows]) for i in range(n)]
    out["volt"] = stats([r["volt"] for r in rows])
    if rows:
        e0, e1 = rows[0]["enc"], rows[-1]["enc"]
        out["enc_delta"] = [b - a for a, b in zip(e0, e1)]
    out["samples"] = len(rows)
    return out


def stuck(rows):
    """Haengen die Werte (USB weg)? -> alle Encoder/IMU-Werte unveraendert."""
    return len({(tuple(r["enc"]), tuple(r["acc"])) for r in rows}) <= 1


try:
    # --- 1. Stammdaten ---
    DATA["version"] = bot.get_version()
    DATA["car_type"] = bot.get_car_type_from_machine()
    DATA["pid"] = bot.get_motion_pid()
    log(f"Firmware {DATA['version']}, car_type {DATA['car_type']}, PID {DATA['pid']}")

    # --- 2. Ruhe ---
    log("Ruhe 10 s ...")
    rows = sample(10)
    DATA["ruhe"] = summary(rows)
    DATA["ruhe"]["haengt"] = stuck(rows)
    r = DATA["ruhe"]
    log(f"  Akku {r['volt']}")
    log(f"  acc {r['acc']}")
    log(f"  gyro {r['gyro']}")
    log(f"  mag {r['mag']}")
    log(f"  att (roll,pitch,yaw) {r['att']}")
    log(f"  enc_delta {r['enc_delta']}  haengt={r['haengt']}")

    # --- 3. Jedes Rad einzeln ---
    DATA["einzeln"] = {}
    for m in range(4):
        for sp in (30, -30):
            sp_list = [0, 0, 0, 0]
            sp_list[m] = sp
            log(f"Motor {m + 1} mit {sp} ...")
            bot.set_motor(*sp_list)
            rows = sample(2.0)
            bot.set_motor(0, 0, 0, 0)
            s = summary(rows)
            DATA["einzeln"][f"M{m + 1}_{sp}"] = s
            log(f"  enc_delta {s['enc_delta']}  Akku min {s['volt'] and s['volt']['min']}"
                f"  vel {[x and x['mittel'] for x in s['vel']]}")
            time.sleep(0.8)

    # --- 4. Fahrbefehle (Mecanum-Kinematik) ---
    DATA["fahrt"] = {}
    for name, cmd in (("vor", (0.2, 0, 0)), ("zurueck", (-0.2, 0, 0)),
                      ("links", (0, 0.2, 0)), ("rechts", (0, -0.2, 0)),
                      ("drehen_links", (0, 0, 1.0)), ("drehen_rechts", (0, 0, -1.0))):
        log(f"set_car_motion {name} {cmd} ...")
        bot.set_car_motion(*cmd)
        rows = sample(2.0)
        bot.set_car_motion(0, 0, 0)
        s = summary(rows)
        DATA["fahrt"][name] = s
        log(f"  enc_delta {s['enc_delta']}  vel {[x and x['mittel'] for x in s['vel']]}"
            f"  Akku min {s['volt'] and s['volt']['min']}")
        time.sleep(0.8)

    # --- 5. Lasttest: alle Raeder, steigende Leistung ---
    DATA["last"] = {}
    for sp in (40, 70, 100):
        log(f"Last: alle Motoren {sp} fuer 3 s ...")
        bot.set_motor(sp, sp, sp, sp)
        rows = sample(3.0)
        bot.set_motor(0, 0, 0, 0)
        s = summary(rows)
        s["haengt"] = stuck(rows)
        DATA["last"][str(sp)] = s
        log(f"  Akku {s['volt']}  enc_delta {s['enc_delta']}  haengt={s['haengt']}")
        time.sleep(2.0)
    rows = sample(3.0)
    DATA["nach_last"] = summary(rows)
    log(f"Akku nach Last (Erholung) {DATA['nach_last']['volt']}")

    # --- 6. Buzzer + LEDs ---
    log("Buzzer 100 ms, LEDs rot/gruen/blau")
    bot.set_beep(100)
    for rgb in ((255, 0, 0), (0, 255, 0), (0, 0, 255)):
        bot.set_colorful_lamps(0xFF, *rgb)
        time.sleep(0.6)
    bot.set_colorful_lamps(0xFF, 0, 0, 0)
    log("fertig")
except KeyboardInterrupt:
    log("abgebrochen")
finally:
    bot.set_motor(0, 0, 0, 0)
    bot.set_car_motion(0, 0, 0)
    with open("phase_a.json", "w") as f:
        json.dump(DATA, f, indent=1)
    with open("phase_a.txt", "w") as f:
        f.write("\n".join(LOG) + "\n")
    sys.exit(0)
