# Roboter-API

Alles, was man braucht, um eine eigene Oberfläche, einen Autonomie-Baustein
oder ein Skript an den Roboter anzubinden, ohne den Server-Code zu lesen.

Der Roboter bietet zwei Wege an:

| Weg | Adresse | Wofür |
|---|---|---|
| **WebSocket** | `ws://<IP-des-Pi>:8000/ws` | Fahren, Heimfahrt, Einstellungen; laufender Zustand vom Roboter |
| **HTTP** | `http://<IP-des-Pi>:8000/...` | Kamerabild, Lidar-Aufnahme, Diagnose |

Die IP zeigt `hostname -I` auf dem Pi. Port 8000 ist der Standard
(`--port` beim Start, `PORT` in `config.py`).

Eine Anmeldung gibt es nicht: wer im selben WLAN ist, kann den Roboter
steuern. HTTP-Endpunkte sind per CORS für alle Herkünfte freigegeben, eine
Browser-App muss also nicht vom Roboter ausgeliefert werden.

---

## Regeln, die man kennen muss

Diese fünf Punkte bestimmen das Verhalten. Wer sie übersieht, wundert sich,
warum der Roboter stehen bleibt.

1. **Fahrbefehle laufend wiederholen.** Kommt `COMMAND_TIMEOUT_S` (0,5 s)
   lang kein `drive`, hält der Roboter von selbst an. Das ist die
   Totmannschaltung für WLAN-Abriss und abgestürzte Apps. Die Web-App
   sendet mit **10 Hz**, und das ist auch die Empfehlung. Ein einzelnes
   `drive` bewegt den Roboter also nur für einen Moment.
2. **Letzter Client weg heißt Stopp.** Trennt sich die letzte
   WebSocket-Verbindung, hält der Roboter an und bricht eine laufende
   Heimfahrt ab.
3. **Der Server begrenzt, nicht der Client.** Jeder Befehl wird auf die
   Werte aus der `limits`-Nachricht gekappt. Physikalisch schafft das
   Fahrgestell ohnehin nur rund 1 m/s.
4. **Die Kollisionsvermeidung kann eingreifen.** Ist sie eingeschaltet und
   steht etwas zu nah in Fahrtrichtung, werden `vx`/`vy` auf 0 gesetzt. Die
   Drehung bleibt erlaubt. Was der Roboter tatsächlich ausführt, steht in
   der `state`-Nachricht, nicht unbedingt das, was man geschickt hat.
5. **Alle Clients sind gleichberechtigt.** Es gibt keine Steuerhoheit. Fahren
   zwei Clients gleichzeitig, gilt der jeweils letzte Befehl. Zustand und
   Meldungen gehen an alle.

---

## Konventionen

Überall dieselben, für Fahrbefehle, Position und Lidar:

| Größe | Einheit | Richtung |
|---|---|---|
| `vx` | m/s | vorwärts positiv |
| `vy` | m/s | **links** positiv (Mecanum: seitwärts fahren ohne Drehen) |
| `omega` | rad/s | gegen den Uhrzeigersinn positiv (= nach links drehen) |
| Lidar-Winkel | Grad, 0–360 | 0° = vorn (Kamera), 90° = links, 180° = hinten, 270° = rechts |
| Lidar-Entfernung | Meter | nur 0,15–8 m, alles andere ist schon verworfen |
| Position `x`, `y` | Meter | bezogen auf den Startpunkt: `x` in die Blickrichtung beim Start, `y` links davon |
| Position `winkel` | **Grad** | gegen den Uhrzeigersinn, 0 = Blickrichtung beim Start |

Der Startpunkt ist dort, wo der Server gestartet oder zuletzt `set_home`
geschickt wurde.

---

## Schnellstart

### Python: einen Meter vorwärts, dann Lidar abfragen

```python
import asyncio, json, urllib.request
import websockets  # pip install websockets

ROBOTER = "192.168.1.50:8000"   # IP des Pi eintragen

async def main():
    async with websockets.connect(f"ws://{ROBOTER}/ws") as ws:
        limits = json.loads(await ws.recv())          # immer die erste Nachricht
        print("Grenzen:", limits)

        # 0,2 m/s für 5 s = 1 m. Mit 10 Hz wiederholen, sonst greift der Watchdog.
        for _ in range(50):
            await ws.send(json.dumps({"type": "drive", "vx": 0.2, "vy": 0, "omega": 0}))
            await asyncio.sleep(0.1)
        await ws.send(json.dumps({"type": "stop"}))

asyncio.run(main())

scan = json.load(urllib.request.urlopen(f"http://{ROBOTER}/lidar/scan"))
vorn = [m for grad, m in scan["punkte"] if grad < 15 or grad > 345]
print("Nächstes Hindernis vorn:", min(vorn) if vorn else "nichts", "m")
```

### Browser (JavaScript)

```js
const ROBOTER = "192.168.1.50:8000";
const ws = new WebSocket(`ws://${ROBOTER}/ws`);

ws.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  if (msg.type === "state") console.log("Position", msg.pose);
  if (msg.type === "alert") console.log(msg.level, msg.text);
};

// Fahren: im Takt senden, solange gefahren werden soll.
let fahrt = { vx: 0, vy: 0, omega: 0 };
setInterval(() => {
  if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "drive", ...fahrt }));
}, 100);

// Kamerabild: einfach als <img> einbinden.
// <img src="http://192.168.1.50:8000/camera/stream">
```

### Kommandozeile

```bash
curl http://<IP>:8000/lidar/scan
curl http://<IP>:8000/lidar/status
curl -o bild.jpg http://<IP>:8000/camera/snapshot
```

Eine automatisch erzeugte Übersicht aller HTTP-Endpunkte (ohne WebSocket)
liefert FastAPI unter `http://<IP>:8000/docs`.

---

## WebSocket `/ws`

Alle Nachrichten sind JSON-Objekte mit einem Feld `type`. Der verbindliche
Vertrag steht in
[`robot/src/rosmaster_assist/protocol.py`](../robot/src/rosmaster_assist/protocol.py),
die Verarbeitung in `_handle()` in
[`api/server.py`](../robot/src/rosmaster_assist/api/server.py).

### Ablauf beim Verbinden

Direkt nach dem Verbinden schickt der Roboter von sich aus, in dieser
Reihenfolge:

1. `limits`
2. `collision` (aktuelle Einstellung)
3. danach alle 0,5 s `state`, dazu bei Bedarf `alert`, `vitals` und `collision`

### An den Roboter

| `type` | Felder | Wirkung | Antwort |
|---|---|---|---|
| `drive` | `vx`, `vy`, `omega` (Zahlen, fehlende = 0) | Fahren. Siehe Regeln 1, 3, 4. Bricht eine laufende Heimfahrt ab, sobald ein Wert ≠ 0 ist; Nullbefehle werden während der Heimfahrt ignoriert. | keine; bei ungültigen Werten `error` |
| `stop` | – | Sofort anhalten, Heimfahrt abbrechen. | keine |
| `home` | – | Selbstständig zum Startpunkt zurückfahren (siehe unten). | `alert` an alle |
| `home_cancel` | – | Heimfahrt abbrechen und anhalten. | `alert` an alle |
| `set_home` | – | Aktuelle Stelle wird neuer Startpunkt (0, 0, 0°), Lidar-Referenz wird neu aufgenommen. | `alert` an alle |
| `collision` | `enabled` (bool, optional), `distance` (m, optional) | Kollisionsvermeidung ein/aus, Mindestabstand. `distance` wird auf 0,15–1,5 m begrenzt. | `collision` an alle |
| `beep` | `ms` (int, Standard 100) | Buzzer. | keine |
| `vitals` | `bpm` (Zahl/null), `motion` (bool/null) | Messwerte vom ESP32, werden an alle weitergereicht. | `vitals` an alle |
| `box` | `action`: `"open"` / `"close"` | Noch nicht gebaut. | `alert` „Box ist noch nicht angebaut“ |
| `ping` | – | Verbindung prüfen. | `pong` |

Ein unbekannter `type` liefert `{"type": "error", "text": "Unbekannter Nachrichtentyp: …"}`.

### Vom Roboter

**`limits`**: einmal beim Verbinden.

```json
{"type": "limits", "max_vx": 5.0, "max_vy": 5.0, "max_omega": 5.0}
```

**`state`**: alle 0,5 s an alle Clients.

```json
{
  "type": "state",
  "driver": "rosmaster",
  "connected": true,
  "homing": false,
  "vx": 0.2, "vy": 0.0, "omega": 0.0,
  "pose": {"x": 1.234, "y": -0.05, "winkel": 12.5, "gemessen": true},
  "collision": {"enabled": false, "distance": 0.35, "verfuegbar": true, "blockiert": false}
}
```

| Feld | Bedeutung |
|---|---|
| `driver` | `"rosmaster"` = echte Motoren, `"mock"` = Simulation |
| `connected` | Verbindung zum Motorboard steht. `false` nach einem USB-Aussetzer; der Server verbindet selbst neu. |
| `homing` | Heimfahrt läuft |
| `vx`, `vy`, `omega` | was der Roboter **gerade ausführt**, nach Begrenzung und Kollisionsvermeidung |
| `pose` | geschätzte Position, siehe Konventionen. `gemessen: false` heißt: nur aus den Befehlen gerechnet, deutlich ungenauer. |
| `collision.verfuegbar` | ob ein Lidar da ist, der Hindernisse sehen kann |
| `collision.blockiert` | die Kollisionsvermeidung hält gerade eine Fahrt zurück |

**`collision`**: beim Verbinden und nach jeder Änderung, Felder wie
`state.collision`.

**`alert`**: Meldungen für den Menschen.

```json
{"type": "alert", "level": "warning", "text": "Hindernis in 0.31 m -- Fahrt gesperrt"}
```

`level` ist `"info"`, `"warning"` oder `"emergency"`. `emergency` ist für
Notfälle vorgesehen, wird derzeit aber noch nirgends ausgelöst. Der Text ist
für die Anzeige gedacht, nicht zum Auswerten. Wer auf ein Ereignis reagieren
will, liest besser die Felder in `state` (etwa `homing` wechselt auf
`false`, wenn die Heimfahrt endet).

**`vitals`**: weitergereichte ESP-Daten: `{"type": "vitals", "bpm": 72, "motion": true}`

**`error`**: Antwort auf eine unbrauchbare Nachricht: `{"type": "error", "text": "…"}`

**`pong`**: Antwort auf `ping`.

### Heimfahrt im Detail

`home` fährt auf direktem Weg zum Startpunkt. Dank Mecanum-Rädern muss sich
der Roboter dafür nicht drehen. Die Werte stehen in `config.py`:

- Tempo `HOME_SPEED` (0,25 m/s), langsamer ab 0,4 m vor dem Ziel
- angekommen bei weniger als `HOME_TOLERANCE_M` (0,12 m) Abstand
- bricht nach `HOME_TIMEOUT_S` (60 s) ab
- **weicht nicht aus**: steht etwas näher als 0,45 m in Fahrtrichtung, hält
  er an und meldet das
- endet mit einem `alert` „Heimfahrt: …“ und `state.homing = false`

---

## HTTP

### Kamera

| Endpunkt | Liefert |
|---|---|
| `GET /camera/stream` | MJPEG-Strom (`multipart/x-mixed-replace`). Direkt in `<img src>` verwendbar, bis zu `CAMERA_FPS` (15) Bilder/s. |
| `GET /camera/snapshot` | ein Einzelbild (meist `image/jpeg`). `503`, solange noch keines da ist. Gut für Bildverarbeitung: einfach im gewünschten Takt abholen. |
| `GET /camera/status` | Diagnose, siehe unten |

Auflösung standardmäßig 640×480. Wird die Kamera abgeschaltet
(`CAMERA_ENABLED = False`), antworten `stream` und `snapshot` mit `503`. Im
Simulationsmodus kommt ein bewegtes Testbild als `image/png`.

`/camera/status`:

```json
{
  "kamera": "USB-Kamera", "geoeffnet": true, "thread_laeuft": true,
  "bilder_gesamt": 5321, "letztes_bild_vor_s": 0.1,
  "bildgroesse_bytes": 41230, "format": "image/jpeg",
  "letzter_fehler": null, "neu_geoeffnet": 0
}
```

### Lidar

**`GET /lidar/scan`**: die letzte vollständige Umdrehung.

```json
{"punkte": [[0.0, 1.52], [1.1, 1.53], [358.9, 1.51]], "alter_s": 0.12}
```

- `punkte`: Liste von `[winkel_grad, entfernung_m]`, Konventionen siehe
  oben. Pro Umdrehung einige hundert Punkte, der Lidar dreht rund 5,5-mal
  pro Sekunde. Winkel auf 0,1°, Entfernung auf 1 cm gerundet.
- `alter_s`: wie alt die Aufnahme ist.
- Ist keine aktuelle Aufnahme da (älter als 3 s oder Lidar gestört), kommt
  stattdessen `{"punkte": [], "hinweis": "Lidar: <Grund>"}`.

Umrechnen in x/y (Meter, Roboter in der Mitte, x vorn, y links):

```python
import math
xy = [(m * math.cos(math.radians(g)), m * math.sin(math.radians(g))) for g, m in scan["punkte"]]
```

**`GET /lidar/status`**: Diagnose und Zustand der Ortung.

```json
{
  "lidar": "RPLIDAR A1", "geoeffnet": true, "thread_laeuft": true,
  "umdrehungen": 1204, "punkte_letzte_runde": 312, "letzte_runde_vor_s": 0.1,
  "letzter_fehler": null, "neu_verbunden": 0,
  "ortung": {
    "lidar_vorhanden": true, "referenz_aufgenommen": true, "referenzpunkte": 318,
    "korrekturen": 85, "verworfen": 3,
    "letztes_residuum_m": 0.031, "letzte_korrektur_m": 0.004
  }
}
```

`letztes_residuum_m` sagt, wie gut die aktuelle Aufnahme auf die Referenz
vom Startpunkt passt. Unter 0,05 ist gut.

### Sonstiges

| Endpunkt | Liefert |
|---|---|
| `GET /` | die mitgelieferte Web-App |
| `GET /static/...` | deren Dateien (`webapp/`) |
| `GET /docs` | automatisch erzeugte HTTP-Übersicht (FastAPI) |

---

## Entwickeln ohne Roboter

```bash
python main.py --mock
```

startet denselben Server auf dem eigenen Rechner unter `http://localhost:8000`.
Alle Endpunkte und Nachrichten verhalten sich wie am Roboter:

- **Antrieb:** simuliert, mit absichtlichem Radschlupf. Fahrbefehle
  erscheinen im Log.
- **Lidar:** tastet einen gedachten Raum ab (8 × 6 m mit einer Säule), der
  sich passend zur Fahrt bewegt. Ortung, Heimfahrt und Kollisionsvermeidung
  lassen sich damit vollständig ausprobieren.
- **Kamera:** bewegtes Testbild.

`driver: "mock"` in der `state`-Nachricht zeigt, dass man in der Simulation
ist.

---

## Erweitern

Wer in einem eigenen Branch neue Funktionen anbaut:

**Neuer WebSocket-Befehl**
1. In `protocol.py` im Kopfkommentar eintragen (das ist der Vertrag).
2. In `_handle()` in `api/server.py` einen Zweig `if kind == "…":` ergänzen.
3. Diese Datei (`docs/api.md`) nachziehen.

**Neuer HTTP-Endpunkt**: in `create_app()` in `api/server.py`, neben den
bestehenden `@app.get(...)`. Er erscheint dann automatisch unter `/docs`.

**Bestehende Nachrichten nicht umbauen.** Web-App und ESP verlassen sich
auf die Feldnamen. Lieber ein neues Feld oder einen neuen `type` ergänzen;
zusätzliche Felder stören keinen bestehenden Client.

**Fahren immer über den Watchdog.** Wer selbst Fahrbefehle erzeugt (etwa ein
Autonomie-Baustein im Server), ruft `watchdog.submit(DriveCommand(...))` auf,
nie den Treiber direkt. Nur so greifen Begrenzung und Totmannschaltung.
Vorbild ist `navigation/homing.py`.

**Wo was liegt** (`robot/src/rosmaster_assist/`):

| Datei | Inhalt |
|---|---|
| `protocol.py` | Nachrichtenformat, `DriveCommand` |
| `api/server.py` | alle Endpunkte, WebSocket-Verarbeitung, Hintergrundschleifen |
| `config.py` | alle Grenzwerte und Einstellungen an einer Stelle |
| `safety/watchdog.py` | Begrenzung und Totmannschaltung |
| `safety/collision.py` | Kollisionsvermeidung bei manueller Fahrt |
| `navigation/odometry.py` | Positionsschätzung |
| `navigation/localizer.py` | Lidar-Abgleich gegen die Startpunkt-Aufnahme |
| `navigation/homing.py` | Heimfahrt |
| `sensors/lidar.py` | Lidar auslesen (echt und simuliert) |
| `sensors/camera.py` | Kamera (CSI, USB, Testbild) |
| `drive/` | Motortreiber (echt und simuliert) |
