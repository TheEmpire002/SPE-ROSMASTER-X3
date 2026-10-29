# CSI-Testpipeline für ESP32 (macOS)

## Installation

1. `csi-test` und `csi_test.py` in den ESP32-Projektordner kopieren, z. B. `~/Documents/SPE/SPE-ROSMASTER-X3/esp/esp32_firmware/`.
2. In diesem Ordner einmal `chmod +x csi-test` ausführen.
3. Es wird automatisch die bereits installierte ESP-IDF-Python-3.13-Umgebung unter `~/.espressif/python_env/idf6.1_py3.13_env/bin/python3` verwendet. `source export.sh` ist für die reine Datenaufzeichnung **nicht** nötig.
4. Den ESP32-Empfänger per USB anschließen, die Firmware muss pro Zeile ein vollständiges CSI-JSON-Objekt senden. Einen eventuell laufenden `idf.py monitor` zuvor schließen.

## Beispiele

```bash
./csi-test --name empty_02 --sleep 5 --duration 120 --trim-start 5 --trim-end 5 --label empty
./csi-test --name occupied_02 --sleep 10 --trim-start 3 --trim-end 5 --label occupied
./csi-test --list-ports
./csi-test --name test_01 --port /dev/cu.usbserial-10 --baud 115200
./csi-test --help
```

`--duration` ist optional. Ohne Dauer läuft die Aufnahme bis **Ctrl+C**; anschließend werden die ersten und letzten N Sekunden abgeschnitten und die Datei geschrieben.

Die finale Datei liegt standardmäßig auf dem **Desktop**, z. B. `~/Desktop/empty_02.jsonl`; zusätzlich gibt es `~/Desktop/empty_02.meta.json` mit Label, Laufzeit, Zuschnitt, verwendeten Ports und Zählwerten. Alternativ `--out-dir ~/Documents/CSI`.

Der zeitliche Zuschnitt orientiert sich an der vom **Mac empfangenen Zeit** des ersten bzw. letzten gültigen CSI-Pakets, nicht an Paketanzahl. Das ursprüngliche Feld `timestamp_us` bleibt unverändert, sodass die ESP32-Zeitstempel weiterhin für die Signalanalyse verfügbar sind.

Die Software **überschreibt keine vorhandenen Messdateien** und hängt auch keine neuen Messungen an alte Dateien an. Feste Portnamen sind nicht erforderlich. Wenn mehrere USB-Ports angeschlossen sind, wird vorsichtig nach einer gültigen CSI-JSON-Ausgabe gesucht. Sollte dies nicht eindeutig sein, bitte `--list-ports` und `--port` nutzen.

## Optional: von überall starten

In `~/.zshrc` einen Alias ergänzen:

```bash
echo 'alias csi-test="$HOME/Documents/SPE/SPE-ROSMASTER-X3/esp/esp32_firmware/csi-test"' >> ~/.zshrc
source ~/.zshrc
```

Danach geht `csi-test --name empty_02 --sleep 5 --duration 120 --trim-start 5 --trim-end 5 --label empty` von überall.
