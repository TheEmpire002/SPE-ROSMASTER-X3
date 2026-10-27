# Hardware

## Bestand

| Komponente      | Modell                | Anschluss        | Status      |
|-----------------|-----------------------|------------------|-------------|
| Rechner         | Raspberry Pi 5 + AI HAT+ | —             | vorhanden   |
| Fahrgestell     | Yahboom ROSMASTER X3  | USB-Serial       | vorhanden   |
| Expansion Board | Yahboom (STM32)       | USB-Serial       | vorhanden   |
| Lidar           | *(Modell eintragen)*  | USB              | vorhanden, nicht eingebunden |
| Kamera          | *(Modell eintragen)*  | USB / CSI        | vorhanden, nicht eingebunden |
| ESP32           | *(Modell eintragen)*  | WLAN             | geplant     |
| Pulssensor      | *(Modell eintragen)*  | ESP32            | geplant     |
| Box-Motor       | *(Modell eintragen)*  | Servo am X3      | geplant     |

> Bitte beim Einbauen die Modellbezeichnungen hier eintragen — ohne die lässt
> sich später nicht nachvollziehen, welcher Treiber gebraucht wird.

## Antrieb

Der X3 hat **Mecanum-Räder**. Er kann dadurch seitwärts fahren, ohne sich zu
drehen. Die Fahrbefehle haben deshalb drei Werte statt zwei:

| Wert    | Bedeutung                    | Einheit |
|---------|------------------------------|---------|
| `vx`    | vorwärts (+) / rückwärts (−) | m/s     |
| `vy`    | links (+) / rechts (−)       | m/s     |
| `omega` | drehen gegen (+) / im (−) Uhrzeigersinn | rad/s |

Grenzwerte und Standard-Tempo stehen in `robot/src/rosmaster_assist/config.py`.

## Rosmaster_Lib

Yahboons Python-Treiber spricht über USB-Serial mit dem Expansion Board.

```python
from Rosmaster_Lib import Rosmaster

bot = Rosmaster()                     # Standard: /dev/myserial bzw. /dev/ttyUSB0
bot.create_receive_threading()        # startet den Empfangs-Thread
bot.set_car_motion(vx, vy, omega)     # fahren
bot.set_beep(100)                     # Buzzer, Dauer in ms
bot.set_colorful_lamps(0xFF, r, g, b) # LED-Streifen
```

Falls die Bibliothek fehlt, liegt sie im Yahboom-Lieferumfang bzw. im
offiziellen Yahboom-GitHub-Repository zum ROSMASTER.

**Serielle Schnittstelle.** Der Gerätename unterscheidet sich je nach Image
(`/dev/myserial`, `/dev/ttyUSB0`, `/dev/ttyAMA0`). Welcher bei euch stimmt,
zeigt `python scripts/check_hardware.py`. Eintragen in `config.py` unter
`SERIAL_PORT`.

**Rechte.** Wenn der Zugriff an „Permission denied" scheitert:

```bash
sudo usermod -aG dialout $USER   # danach neu anmelden
```

## AI HAT+

Der AI HAT+ enthält einen Hailo-Beschleuniger. Er ist für die spätere
Kamera-Auswertung interessant (Personenerkennung, Sturzerkennung) und wird im
aktuellen Grundgerüst noch nicht genutzt. Ob die Personenerkennung über die
Kamera oder über den ESP läuft, ist noch offen — der AI HAT+ wäre das deutlich
zuverlässigere Verfahren.
