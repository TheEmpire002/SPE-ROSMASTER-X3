from Rosmaster_Lib import Rosmaster
import time

bot = Rosmaster()

try:
    print("Servo auf 90 Grad")
    bot.set_pwm_servo(1, 90)
    time.sleep(2)

    for angle in [60, 90, 120, 90]:
        print(f"Servo: {angle} Grad")
        bot.set_pwm_servo(1, angle)
        time.sleep(2)

finally:
    bot.set_pwm_servo(1, 90)
    print("Test beendet")
