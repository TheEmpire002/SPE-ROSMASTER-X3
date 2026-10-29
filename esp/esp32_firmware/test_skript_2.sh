#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSI_TEST="$SCRIPT_DIR/csi_test.py"

PYTHON="$HOME/.espressif/python_env/idf6.1_py3.13_env/bin/python3"
SOUND="/System/Library/Sounds/Ping.aiff"

# Erste Rundennummer
START_NUMBER=7

# Speicherort
OUT_DIR="$HOME/Documents/SPE/SPE-ROSMASTER-X3/esp/data"

# Prüfen, ob benötigte Dateien vorhanden sind
if [[ ! -f "$CSI_TEST" || ! -x "$PYTHON" ]]; then
    echo "Fehler: csi_test.py oder Python-Umgebung fehlt."
    exit 1
fi

# Prüfen, ob Zieldateien schon existieren
for ((runde=0; runde<5; runde++)); do
    printf -v nummer "%02d" "$((START_NUMBER + runde))"

    for y in 0 1 2; do
        for x in 0 1 2; do
            name="occupied-x_${x}-y_${y}-${nummer}"

            if [[ -e "$OUT_DIR/$name.jsonl" ]]; then
                echo "Datei existiert bereits: $name.jsonl"
                exit 1
            fi
        done
    done
done

zaehler=0

for ((runde=0; runde<5; runde++)); do

    printf -v nummer "%02d" "$((START_NUMBER + runde))"

    for y in 0 1 2; do
        for x in 0 1 2; do

            if (( zaehler == 0 )); then
                sleep_time=10
		afplay "$SOUND"
            else
                echo "Bitte Position wechseln: x=$x, y=$y"
                afplay "$SOUND"
                sleep_time=5
            fi

            name="occupied-x_${x}-y_${y}-${nummer}"

            echo "=================================="
            echo "Messung $((zaehler + 1))/90"
            echo "Runde $((runde + 1))/10"
            echo "Position: x=$x, y=$y"
            echo "Datei: $name.jsonl"
            echo "=================================="

            "$PYTHON" "$CSI_TEST" \
                --name "$name" \
                --out-dir "$OUT_DIR" \
                --sleep "$sleep_time" \
                --duration 12 \
                --trim-start 1 \
                --trim-end 1

            zaehler=$((zaehler + 1))

        done
    done
done

echo "Alle 45 Messungen abgeschlossen!"
afplay /System/Library/Sounds/Hero.aiff

