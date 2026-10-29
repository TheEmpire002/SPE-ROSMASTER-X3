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
| `stop` | – | Sofort anhalten, Heimfahrt und autonomes Fahren abbrechen. | keine |
| `explore` | – | Autonomes Fahren starten (siehe unten). Braucht den Lidar; bricht eine Heimfahrt ab. | `alert` an alle; ohne Lidar `alert` (warning) |
| `explore_stop` | – | Autonomes Fahren beenden und anhalten. | `alert` an alle |
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
  "collision": {"enabled": false, "distance": 0.35, "verfuegbar": true, "blockiert": false},
  "autonomie": {"aktiv": false, "verfuegbar": true, "zustand": null, "...": "siehe unten"}
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
| `autonomie` | autonomes Fahren und erkannte Hindernisse, siehe „Autonomes Fahren im Detail“ |

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

### Autonomes Fahren im Detail

`explore` startet Test 1: Der Roboter fährt zufällig durch den Raum und
weicht allem aus, was der Lidar sieht. Werte in `config.py` (`EXPLORE_*`):

- geradeaus mit `EXPLORE_SPEED` (0,2 m/s), langsamer ab 0,9 m vor einem
  Hindernis; schiebt sich seitlich von nahen Wänden weg
- frei ist ein 40 cm breiter Korridor (`EXPLORE_HALF_WIDTH_M`), nicht ein
  Kegel -- so passt er nicht in Lücken, die schmaler sind als er selbst
- unter `EXPLORE_STOP_M` (0,4 m) vorn: anhalten und in die freieste
  Richtung drehen. Gedreht wird nach Zeit (`EXPLORE_TURN_FACTOR`), weil
  der Roboter seine Blickrichtung nicht misst.
- alle 4–10 s biegt er zufällig ab, damit er nicht immer dieselbe Bahn fährt
- steht ein **Mensch** bis 1 m vor ihm, wartet er `EXPLORE_PERSON_WAIT_S`
  (2 s), bevor er ausweicht
- liefert der Lidar länger als 3 s nichts, endet die Fahrt
- jede Fahreingabe ≠ 0, `stop`, `home`, `set_home` oder das Trennen des
  letzten Clients beendet sie. Nullbefehle der App werden ignoriert.
- endet mit `alert` „Autonomes Fahren beendet: …“ und `autonomie.aktiv = false`

Die Hindernisse werden auch ohne laufende Fahrt erkannt (bis 3 m) und grob
benannt: `Wand`, `Stuhl` (dünne Beine dicht beieinander), `Tisch` (dünne
Beine 0,6–1,8 m auseinander), `Mensch` (zwei dicke Beine oder Kamera-Treffer),
`Bein (Mensch?)`, `Bein (Stuhl/Tisch?)`, `Objekt`. Das ist eine Faustregel
auf einer Lidar-Ebene, keine Objekterkennung.

Ist OpenCV installiert, sucht die Personenkamera (Logitech C270, siehe
[Kamera](#kamera)) während der Fahrt nach Personen (HOG-Detektor, 2 Bilder/s). Lidar-Hindernisse in deren Richtung heißen dann
`Mensch` mit `quelle: "Kamera+Lidar"`. Ohne OpenCV läuft alles andere weiter.

`state.autonomie`:

```json
{
  "aktiv": true,
  "verfuegbar": true,
  "zustand": "faehrt",
  "text": "frei: 1.8 m",
  "frei_vorn": 1.83,
  "dauer_s": 42,
  "ausweichen": 5,
  "menschen": 1,
  "grund": null,
  "hindernisse": [
    {"art": "Stuhl", "winkel": 31.5, "abstand": 0.84, "breite": 0.41,
     "x": 0.71, "y": 0.44, "quelle": "Lidar"}
  ],
  "personen": [{"x1": 0.41, "y1": 0.1, "x2": 0.62, "y2": 0.95, "score": 1.2}],
  "personenerkennung": {"verfuegbar": true, "aktiv": true, "fehler": null,
                        "kamera": "personen", "letzte_erkennung_vor_s": 0.3}
}
```

| Feld | Bedeutung |
|---|---|
| `verfuegbar` | Lidar vorhanden, `explore` möglich |
| `zustand` | `faehrt`, `dreht`, `wartet`, `wartet auf Mensch`, `wartet auf Lidar`; `null` wenn aus |
| `text` | Erklärung für die Anzeige |
| `frei_vorn` | freie Strecke im Korridor vor dem Roboter (m) |
| `ausweichen`, `menschen` | Zähler dieser Fahrt |
| `grund` | warum die letzte Fahrt endete |
| `hindernisse` | die 12 nächsten, nächstes zuerst. `winkel` in Grad wie beim Lidar, `x`/`y` Mitte in Roboterkoordinaten (m) |
| `personen` | Personenrahmen im Bild der Personenkamera, relativ zur Bildgröße (0..1) |
| `personenerkennung` | ob die Kamera-Erkennung geht; `fehler` z. B. „opencv fehlt“; `kamera`: auf welchem Kamerabild die Rahmen liegen (Name wie in `/cameras`) |

---

## HTTP

### Kamera

Der Roboter betreibt alle Kameras, die er findet, jede mit eigenem Bild.
Jede hat einen Namen:

| Name | Kamera | Wofür |
|---|---|---|
| `orientierung` | Orbbec, Farbbild | Orientierung, Fahren. **Standardkamera.** |
| `personen` | Logitech C270 | Personenerkennung |
| `usb1`, `usb2`, … | weitere USB-Kameras | nichts Festes |
| `csi` | Kamera am Flachbandkabel, falls vorhanden | nichts Festes |

`orientierung` und `personen` sind immer aufgeführt, auch wenn die Kamera
gerade fehlt (dann `geoeffnet: false`). Der Roboter sucht sie im Hintergrund
weiter. Zugeordnet wird über die USB-Kennung (`config.CAMERA_ROLES`).

| Endpunkt | Liefert |
|---|---|
| `GET /cameras` | alle Kameras mit Zustand und Adressen, siehe unten |
| `GET /camera/<name>/stream` | MJPEG-Strom (`multipart/x-mixed-replace`) dieser Kamera. Direkt in `<img src>` verwendbar, bis zu `CAMERA_FPS` Bilder/s (Standard 15, höchstens 30, auf der Einstellungsseite verstellbar). |
| `GET /camera/<name>/snapshot` | ein Einzelbild (meist `image/jpeg`). `503`, solange noch keines da ist. Gut für Bildverarbeitung: einfach im gewünschten Takt abholen. |
| `GET /camera/<name>/status` | Diagnose dieser Kamera, siehe unten |
| `GET /camera/stream`, `/camera/snapshot`, `/camera/status` | dasselbe für die Standardkamera (`config.CAMERA_DEFAULT`) |

Ein unbekannter Name gibt `404`. Auflösung standardmäßig 640×480. Wird die
Kamera abgeschaltet (`CAMERA_ENABLED = False`), antworten `stream` und
`snapshot` mit `503`. Im Simulationsmodus liefert jede Kamera ein bewegtes
Testbild als `image/png`, mit eigener Balkenfarbe.

`/cameras`:

```json
{
  "standard": "orientierung",
  "personen": "personen",
  "kameras": [
    {"name": "orientierung", "kamera": "Orbbec (Farbbild)", "geoeffnet": true,
     "stream": "/camera/orientierung/stream",
     "snapshot": "/camera/orientierung/snapshot", "...": "wie /camera/status"},
    {"name": "personen", "kamera": "Logitech C270", "geoeffnet": false,
     "letzter_fehler": "Öffnen fehlgeschlagen: nicht angeschlossen (USB 046d)",
     "...": "..."}
  ]
}
```

`personen` ist der Name der Kamera, auf deren Bild die Personenrahmen aus
`state.autonomie.personen` liegen.

`/camera/status` bzw. `/camera/<name>/status`:

```json
{
  "kamera": "Orbbec (Farbbild)", "geoeffnet": true, "thread_laeuft": true,
  "bilder_gesamt": 5321, "letztes_bild_vor_s": 0.1,
  "bildgroesse_bytes": 41230, "format": "image/jpeg",
  "letzter_fehler": null, "neu_geoeffnet": 0,
  "name": "orientierung", "geraet": "/dev/video0", "fov_deg": 66.0,
  "fps_ist": 14.8, "fps_soll": 15
}
```

`geraet` ist das gerade benutzte Gerät. Es kann sich nach einem
USB-Aussetzer ändern. `fov_deg` ist der horizontale Bildwinkel in Grad. `fps_soll` ist die
eingestellte Bildrate (`CAMERA_FPS`), `fps_ist` die tatsächlich erreichte.
Die Kameras schaffen höchstens 30 fps, die Logitech bei wenig Licht nur
etwa 15.

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

### Autonomes Fahren

| Endpunkt | Liefert |
|---|---|
| `GET /autonomie/status` | dasselbe wie `state.autonomie`, zum Nachsehen ohne WebSocket |

### Einstellungen

Die Einstellungen aus `config.py`, die sich zur Laufzeit verstellen lassen
(Tempo, Kamera-Bildrate, Lidar, Autonomie …). In der Web-App öffnet das Zahnrad oben rechts sie als Fenster; als
eigene Seite gibt es sie unter `/einstellungen`.

| Endpunkt | Liefert |
|---|---|
| `GET /settings` | alle Einstellungen nach Blöcken, mit Wert, Standard und Grenzen |
| `POST /settings` | Body `{"CAMERA_FPS": 20, "EXPLORE_SPEED": 0.15}`: setzt die Werte. Alle oder keiner: Ist einer ungültig, kommt `400` mit `{"fehler": "…"}`, und nichts wird geändert. |
| `POST /settings/reset` | Body `{"keys": ["CAMERA_FPS"]}` für einzelne Werte, `{}` für alle: zurück auf die Werte aus `config.py` |

`GET /settings`:

```json
{
  "gruppen": [
    {"id": "kamera", "titel": "Kameras", "text": "…", "einstellungen": [
      {"key": "CAMERA_FPS", "label": "Bilder pro Sekunde", "typ": "int",
       "wert": 15, "standard": 15, "min": 1, "max": 30, "schritt": 1,
       "einheit": "fps", "optionen": null, "hilfe": "…", "neustart": false}
    ]}
  ]
}
```

- `typ`: `bool`, `int`, `float` oder `auswahl` (dann stehen die erlaubten
  Werte in `optionen`).
- `neustart: true`: wirkt erst nach einem Neustart des Servers.
- Die Antwort auf beide `POST` ist dieselbe Liste, dazu `geaendert` (welche
  Werte sich wirklich geändert haben) und `neustart_noetig`.

Änderungen gelten sofort und werden in `robot/settings.json` gespeichert.
Die Datei enthält nur Abweichungen vom Standard. Wer sie löscht, hat wieder
die Werte aus `config.py`. Ändern sich die Tempo-Grenzen, schickt der
Roboter allen Clients ein neues `limits`. Jede Änderung meldet er außerdem
als `alert` (`info`).

Ändern geht nur von einer Seite, die vom Roboter selbst geladen wurde: CORS
erlaubt fremden Seiten nur `GET`.

### Sonstiges

| Endpunkt | Liefert |
|---|---|
| `GET /` | die mitgelieferte Web-App |
| `GET /einstellungen` | die Einstellungsseite der Web-App |
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
- **Kamera:** je Kamera (`orientierung`, `personen`) ein bewegtes Testbild.

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
