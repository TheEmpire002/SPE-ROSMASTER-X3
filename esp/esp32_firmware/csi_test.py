#!/usr/bin/env python3
"""ESP32 Wi-Fi CSI acquisition CLI.

Uses the existing ESP-IDF Python environment automatically.
Writes only one JSONL output file; original CSI fields are preserved.
Trimming is done using the Mac's monotonic receipt times, not packet indexes.
"""

import argparse
import os
import json
import math
from pathlib import Path
import sys
import tempfile
import time

# Relaunch with the known ESP-IDF Python environment, regardless of shell Python.
PREFERRED_PYTHON = Path.home() / ".espressif/python_env/idf6.1_py3.13_env/bin/python3"
if Path(sys.executable).resolve() != PREFERRED_PYTHON.resolve():
    if not PREFERRED_PYTHON.is_file():
        print(f"Fehler: Python-Umgebung nicht gefunden: {PREFERRED_PYTHON}", file=sys.stderr)
        sys.exit(1)
    os.execv(str(PREFERRED_PYTHON), [str(PREFERRED_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Fehler: pyserial fehlt in dieser Python-Umgebung.", file=sys.stderr)
    print("Installation: python3 -m pip install pyserial", file=sys.stderr)
    sys.exit(1)

USB_BRIDGE_VIDS = {
    0x10C4,  # Silicon Labs CP210x
    0x1A86,  # WCH CH340/CH9102
    0x0403,  # FTDI
    0x303A,  # Espressif USB JTAG/Serial
    0x067B,  # Prolific
}
PORT_PATTERNS = ("usbserial", "usbmodem", "slab_usbtouart", "wchusbserial", "tty.usb")


def is_csi_measurement(obj):
    """Accept only JSON objects with usable CSI samples."""
    return (
        isinstance(obj, dict)
        and isinstance(obj.get("csi_data"), list)
        and bool(obj["csi_data"])
        and all(isinstance(x, int) and not isinstance(x, bool) for x in obj["csi_data"])
        and ("length" not in obj or obj["length"] == len(obj["csi_data"]))
    )


def available_ports():
    return [p for p in list_ports.comports() if p.device.startswith("/dev/cu.")]


def is_usb_serial(p):
    text = " ".join(str(x or "") for x in (p.device, p.description, p.manufacturer)).lower()
    return p.vid in USB_BRIDGE_VIDS or any(pattern in text for pattern in PORT_PATTERNS)


def print_ports(ports):
    if not ports:
        print("Keine /dev/cu.*-Anschlüsse gefunden.")
        return
    print("Verfügbare serielle Anschlüsse:")
    for p in ports:
        marker = "[USB]" if is_usb_serial(p) else "     "
        identity = f"VID:PID={p.vid:04X}:{p.pid:04X}" if p.vid is not None and p.pid is not None else ""
        print(f"  {marker} {p.device}  {p.description}  {identity}")


def check_candidate_for_csi(device, baud, seconds=7):
    """Probe ambiguous ports; return True only if real CSI JSON is observed.

    On ESP32 dev boards opening the serial port may reboot the board. Wait up
    to seconds for the firmware to reconnect to Wi-Fi and print JSON again.
    """
    try:
        with serial.Serial(device, baudrate=baud, timeout=0.3) as conn:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                raw = conn.readline()
                if not raw:
                    continue
                try:
                    obj = json.loads(raw.decode("utf-8", errors="replace"))
                except (json.JSONDecodeError, UnicodeError):
                    continue
                if is_csi_measurement(obj):
                    return True
    except (OSError, serial.SerialException) as exc:
        print(f"  {device}: nicht lesbar ({exc})")
    return False


def choose_port(requested, baud, ports):
    if requested:
        return requested
    candidates = [p for p in ports if is_usb_serial(p)]
    if not candidates:
        print_ports(ports)
        raise RuntimeError("Kein ESP32-kompatibler USB-Serial-Port gefunden. USB-Kabel prüfen.")
    if len(candidates) == 1:
        return candidates[0].device

    print("Mehrere USB-Serial-Ports erkannt. Suche Empfänger mit CSI-JSON-Ausgabe ...")
    matches = []
    for p in candidates:
        print(f"  Prüfe {p.device} ...", flush=True)
        if check_candidate_for_csi(p.device, baud):
            matches.append(p.device)
            print("    CSI-Daten erkannt.")
    if len(matches) == 1:
        return matches[0]

    print_ports(ports)
    if not matches:
        raise RuntimeError("Auf keinem Port CSI-JSON erkannt. Mit --port den Empfänger explizit angeben.")
    raise RuntimeError(
        "Mehrere Ports liefern CSI-Daten: " + ", ".join(matches)
        + ". Bitte den richtigen Empfänger mit --port auswählen."
    )


def finite_nonnegative(s):
    try:
        v = float(s)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Muss eine Zahl sein.") from exc
    if not math.isfinite(v) or v < 0:
        raise argparse.ArgumentTypeError("Muss eine endliche Zahl >= 0 sein.")
    return v


def positive_duration(s):
    v = finite_nonnegative(s)
    if v <= 0:
        raise argparse.ArgumentTypeError("Muss > 0 sein.")
    return v


def cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="ESP32-CSI-Aufzeichnung mit Startverzögerung und zeitlichem Zuschnitt."
    )
    parser.add_argument("--name", help="Zieldatei, z. B. empty_02 oder occupied_01.jsonl")
    parser.add_argument("--sleep", type=finite_nonnegative, default=0,
                        help="Sekunden vor Aufzeichnungsbeginn warten (Standard: 0)")
    parser.add_argument("--duration", type=positive_duration,
                        help="Aufzeichnungsdauer in Sekunden; ohne Option beenden mit Ctrl+C")
    parser.add_argument("--trim-start", type=finite_nonnegative, default=0,
                        help="Sekunden vom Anfang der Messreihe abschneiden (Standard: 0)")
    parser.add_argument("--trim-end", type=finite_nonnegative, default=0,
                        help="Sekunden vom Ende der Messreihe abschneiden (Standard: 0)")
    parser.add_argument("--out-dir", type=Path, default=Path.home() / "Documents/SPE/SPE-ROSMASTER-X3/esp/data",
                        help="Zielordner (Standard: ~/Documents/SPE/SPE-ROSMASTER-X3/esp/data)")
    parser.add_argument("--port", help="Seriellen Port manuell setzen, falls Auto-Erkennung scheitert")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baudrate des ESP32 (Standard: 115200)")
    parser.add_argument("--list-ports", action="store_true", help="Ports anzeigen und beenden")
    args = parser.parse_args(argv)
    if args.list_ports:
        return args
    if not args.name:
        parser.error("--name ist erforderlich (oder --list-ports verwenden).")
    if not args.name.strip() or Path(args.name).name != args.name or args.name in (".", ".."):
        parser.error("--name muss ein einfacher Dateiname ohne Verzeichnispfad sein.")
    if args.name.startswith("."):
        parser.error("--name darf nicht mit einem Punkt beginnen.")
    if Path(args.name).suffix not in ("", ".jsonl"):
        parser.error("Nur Namen ohne Endung oder mit .jsonl sind erlaubt.")
    if args.baud <= 0:
        parser.error("--baud muss positiv sein.")
    if args.duration is not None and args.trim_start + args.trim_end >= args.duration:
        parser.error("--trim-start + --trim-end muss kleiner als --duration sein.")
    return args


def formatted_time(seconds):
    return f"{seconds:.1f} s"


def countdown(seconds):
    if seconds <= 0:
        return
    print(f"Start in {seconds:g} Sekunden. Ctrl+C zum Abbrechen.", flush=True)
    deadline = time.monotonic() + seconds
    last_shown = None
    while True:
        remaining = max(0.0, deadline - time.monotonic())
        if remaining <= 0:
            break
        # Keep output concise even for a long sleep.
        display = math.ceil(remaining)
        if display != last_shown and (seconds <= 15 or display % 5 == 0):
            print(f"  {display} ...", flush=True)
            last_shown = display
        time.sleep(min(0.2, remaining))


def capture_to_temp(conn, tmp, duration):
    """Write receipt_time<TAB>original_JSON per valid packet to a temporary stream."""
    started = time.monotonic()
    first = last = None
    valid = 0
    invalid = 0
    last_status = started
    problem = None
    print("Aufzeichnung läuft. " + ("Automatisches Ende nach " + formatted_time(duration) + "." if duration else "Beenden mit Ctrl+C."), flush=True)
    try:
        while duration is None or time.monotonic() - started < duration:
            raw = conn.readline()
            if not raw:
                continue
            receipt = time.monotonic()
            try:
                entry = json.loads(raw.decode("utf-8", errors="replace"))
            except (json.JSONDecodeError, UnicodeError):
                invalid += 1
                continue
            if not is_csi_measurement(entry):
                invalid += 1
                continue
            if first is None:
                first = receipt
            last = receipt
            tmp.write(f"{receipt:.9f}\t{json.dumps(entry, ensure_ascii=False, separators=(',', ':'))}\n")
            valid += 1
            if receipt - last_status >= 5:
                print(f"  {receipt - started:.0f} s | {valid} CSI-Pakete erfasst", flush=True)
                last_status = receipt
    except KeyboardInterrupt:
        print("\nAufzeichnung per Ctrl+C beendet.")
    except (serial.SerialException, OSError) as exc:
        problem = f"Verbindung zum ESP32 verloren: {exc}"
        print(f"WARNUNG: {problem}", file=sys.stderr)
    return {
        "capture_seconds": time.monotonic() - started,
        "first_receipt": first,
        "last_receipt": last,
        "valid": valid,
        "ignored_lines": invalid,
        "problem": problem,
    }


def trim_temp_to_jsonl(tmp, outfile, first, last, cut_head, cut_tail):
    """Remove first/last N seconds according to host receipt clock; no packet mixing."""
    keep_from = first + cut_head
    keep_until = last - cut_tail
    if keep_until < keep_from:
        raise RuntimeError("Zuschnitt ist länger als die vorhandene Messreihe. Keine Datei angelegt.")
    tmp.seek(0)
    saved = 0
    # Exclusive-create prevents mixing trials even in a race with another process.
    with outfile.open("x", encoding="utf-8") as out:
        for row in tmp:
            receipt_str, payload = row.split("\t", 1)
            receipt = float(receipt_str)
            if keep_from <= receipt <= keep_until:
                out.write(payload if payload.endswith("\n") else payload + "\n")
                saved += 1
    if not saved:
        outfile.unlink(missing_ok=True)
        raise RuntimeError("Nach dem Zuschnitt bleiben keine CSI-Pakete übrig. Keine Datei angelegt.")
    return saved


def run(argv=None):
    args = cli_args(argv)
    ports = available_ports()
    if args.list_ports:
        print_ports(ports)
        return 0
    name = args.name if args.name.endswith(".jsonl") else args.name + ".jsonl"
    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    outfile = out_dir / name
    for p in (outfile,):
        if p.exists():
            raise RuntimeError(f"Zieldatei existiert bereits: {p}\nBitte --name ändern. Es wird nichts überschrieben.")

    device = choose_port(args.port, args.baud, ports)
    print(f"Empfänger: {device} | Baudrate: {args.baud}")
    print(f"Zieldatei: {outfile}")
    print(f"Zuschnitt: Anfang {args.trim_start:g} s, Ende {args.trim_end:g} s")

    # Open before start delay: ESP32 boards may reset when the serial port opens.
    with serial.Serial(device, baudrate=args.baud, timeout=0.25) as conn:
        countdown(args.sleep)
        # Discard data from the countdown, avoiding stale buffered CSI packets.
        conn.reset_input_buffer()
        with tempfile.TemporaryFile(mode="w+t", encoding="utf-8") as tmp:
            stats = capture_to_temp(conn, tmp, args.duration)
            if stats["valid"] == 0:
                raise RuntimeError("Kein gültiges CSI-JSON empfangen. Firmware, Port und Baudrate prüfen.")
            saved = trim_temp_to_jsonl(
                tmp, outfile, stats["first_receipt"], stats["last_receipt"],
                args.trim_start, args.trim_end,
            )

    print(f"\nFertig: {saved} von {stats['valid']} CSI-Paketen gespeichert.")
    print(f"JSONL: {outfile}")
    return 1 if stats["problem"] else 0


if __name__ == "__main__":
    try:
        sys.exit(run())
    except KeyboardInterrupt:
        print("\nAbgebrochen, bevor die Aufzeichnung gestartet wurde.", file=sys.stderr)
        sys.exit(130)
    except (OSError, RuntimeError, serial.SerialException) as exc:
        print(f"FEHLER: {exc}", file=sys.stderr)
        sys.exit(1)

