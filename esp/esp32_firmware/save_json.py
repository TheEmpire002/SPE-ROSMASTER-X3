
import serial
import json
from pathlib import Path

port = "/dev/cu.usbserial-10"
output = Path.home() / "Desktop" / "empty_01_jsonl"

with serial.Serial(port, 115200, timeout=1) as esp:
    with output.open("a", encoding="utf-8") as file:

        print("CSI-JSON-Empfang gestartet")

        try:
            while True:
                line = esp.readline().decode(
                    "utf-8", errors="replace"
                ).strip()

                try:
                    measurement = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if not isinstance(measurement, dict):
                    continue

                if "csi_data" not in measurement:
                    continue

                file.write(
                    json.dumps(measurement) + "\n"
                )
                file.flush()

                print(
                    "CSI gespeichert:",
                    measurement["length"],
                    "Bytes"
                )

        except KeyboardInterrupt:
            print("Aufzeichnung beendet")

