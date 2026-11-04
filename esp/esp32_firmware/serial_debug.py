import serial
import time

PORT = "/dev/cu.usbserial-10"
BAUD = 115200

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    time.sleep(2)

    print("Empfange Rohdaten...\n")

    for _ in range(30):
        data = ser.read(1000)

        if data:
            print(repr(data[:200]))
        else:
            print("Keine Daten empfangen")

