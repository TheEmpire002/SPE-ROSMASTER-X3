
import serial
import struct
import time
import argparse
from pathlib import Path
from datetime import datetime

# ==========================================
# KONFIGURATION
# ==========================================

PORT = "/dev/cu.usbserial-10"
BAUD = 460800

MAGIC = b"CS"
MAX_CSI = 768

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "data"


# ==========================================
# DATEN VOLLSTÄNDIG LESEN
# ==========================================

def read_exact(ser, n):
    data = bytearray()

    while len(data) < n:
        chunk = ser.read(n - len(data))

        if not chunk:
            raise TimeoutError("UART-Timeout")

        data.extend(chunk)

    return bytes(data)


# ==========================================
# ARGUMENTE
# ==========================================

parser = argparse.ArgumentParser(
    description="ESP32 CSI Binary Receiver"
)

parser.add_argument(
    "--port",
    default=PORT,
    help="Serieller Port"
)

parser.add_argument(
    "--baud",
    type=int,
    default=BAUD,
    help="UART-Baudrate"
)

parser.add_argument(
    "--duration",
    type=float,
    default=30,
    help="Aufzeichnungsdauer in Sekunden (0 = unbegrenzt)"
)

parser.add_argument(
    "--name",
    default=None,
    help="Name der Ausgabedatei"
)

args = parser.parse_args()


# ==========================================
# DATEI VORBEREITEN
# ==========================================

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

filename = args.name or f"csi_{timestamp}"
filename = Path(filename).name

if filename.endswith(".bin"):
    filename = filename[:-4]

output_file = OUTPUT_DIR / f"{filename}.bin"


# ==========================================
# CSI EMPFANGEN
# ==========================================
count = 0
total_bytes = 0
invalid_headers = 0
last_rssi = None
last_length = None

print("\nESP32 CSI Binary Receiver")
print(f"Port: {args.port}")
print(f"Baudrate: {args.baud}")
print(f"Zieldatei: {output_file}")
print(f"Dauer: {args.duration} Sekunden")
print("\nEmpfange binaere CSI-Daten...\n")

try:
    with serial.Serial(
        args.port,
        args.baud,
        timeout=0.1
    ) as ser, open(output_file, "wb") as file:

        start = time.monotonic()

        last_rate_time = start
        last_rate_count = 0

        while True:
            now = time.monotonic()

            # Aufzeichnungsdauer prüfen
            if (
                args.duration > 0
                and now - start >= args.duration
            ):
                break

            try:
                # Paketbeginn suchen
                first = ser.read(1)

                if first != MAGIC[:1]:
                    continue

                second = ser.read(1)

                if second != MAGIC[1:]:
                    continue

                # Header lesen (7 weitere Bytes)
                header = read_exact(ser, 7)

                length, timestamp_us, rssi = struct.unpack(
                    "<HIb", header
                )

                # Ungültige Länge verwerfen
                if length == 0 or length > MAX_CSI:
                    invalid_headers += 1
                    continue

                # CSI-Rohdaten einlesen
                raw = read_exact(ser, length)

                # Vollständiges Paket speichern
                file.write(MAGIC + header + raw)

                count += 1
                total_bytes += 9 + length

                last_rssi = rssi
                last_length = length

            except TimeoutError:
                pass

            # ==================================
            # RATEN JEDE SEKUNDE BERECHNEN
            # ==================================

            now = time.monotonic()
            elapsed = now - start
            interval = now - last_rate_time

            if interval >= 1.0:

                packets_in_interval = (
                    count - last_rate_count
                )

                current_rate = (
                    packets_in_interval / interval
                )

                average_rate = (
                    count / elapsed
                    if elapsed > 0 else 0
                )

                print(
                    f"Pakete: {count:6d} | "
                    f"Durchschnitt: {average_rate:6.1f}/s | "
                    f"Aktuell: {current_rate:6.1f}/s | "
                    f"RSSI: {last_rssi} | "
                    f"CSI-Laenge: {last_length}"
                )

                last_rate_time = now
                last_rate_count = count

except KeyboardInterrupt:
    print("\nAufzeichnung manuell beendet.")

except serial.SerialException as e:
    print(f"\nSerieller Fehler: {e}")

finally:
    # ======================================
    # ABSCHLUSSSTATISTIK
    # ======================================

    if "start" in locals():
        elapsed = time.monotonic() - start
    else:
        elapsed = 0

    average = count / elapsed if elapsed > 0 else 0

    print("\n================================")
    print("       CSI AUFZEICHNUNG")
    print("================================")
    print(f"Gespeicherte Pakete: {count}")
    print(f"Messdauer: {elapsed:.1f} s")
    print(f"Durchschnitt: {average:.1f} Pakete/s")
    print(f"Gespeicherte Daten: {total_bytes} Bytes")
    print(f"Ungueltige Header: {invalid_headers}")
    print(f"Datei: {output_file}")
    print("================================")

