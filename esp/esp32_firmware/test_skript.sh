#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSI_TEST="$SCRIPT_DIR/csi_test.py"

TEST_NUMBER="01"
SOUND="/System/Library/Sounds/Ping.aiff"

trap 'echo "Testreihe abgebrochen."; exit 130' INT TERM

completed=0

for y in 0 1 2; do
    for x in 0 1 2; do

        # Feld (0,0) wurde bereits getestet
        if [[ "$x" -eq 0 && "$y" -eq 0 ]]; then
            continue
        fi

        name="occupied-x_${x}-y_${y}-${TEST_NUMBER}"

        # Zwischen zwei Tests ein Signalton
        if (( completed >= 0 )); then
            afplay "$SOUND"
        fi

        echo ""
        echo "================================="
        echo "Test $((completed + 1))/8"
        echo "Position: x=$x, y=$y"
        echo "Dateiname: $name"
        echo "================================="

        "$CSI_TEST" \
            --sleep 10 \
            --duration 80 \
            --trim-start 10 \
            --trim-end 10 \
            --name "$name"

        ((completed += 1))
    done
done

echo "Alle 8 Tests abgeschlossen!"

