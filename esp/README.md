# ESP32 — Personentracking und Vitaldaten

Noch nicht implementiert. Dieser Ordner beschreibt, was der ESP tun soll und
wie er sich anbindet, damit jemand direkt loslegen kann.

## Aufgabe

Der ESP32 beobachtet eine Person im Raum und meldet:

- **Bewegung** — ist jemand aktiv? Längere Bewegungslosigkeit kann auf einen
  Sturz hindeuten.
- **Puls** — Richtwert, kein Messgerät.

## Anbindung

Der ESP verbindet sich per WLAN auf denselben WebSocket wie die App:

```
ws://<IP-des-Pi>:8000/ws
```

und schickt im Sekundentakt:

```json
{"type": "vitals", "bpm": 72, "motion": true}
```

Der Roboter reicht das unverändert an alle verbundenen Apps weiter. Mehr
passiert vorerst nicht — die Bewertung kommt später und bewusst serverseitig,
nicht auf dem ESP.

Das vollständige Protokoll steht in [`docs/api.md`](../docs/api.md), der
verbindliche Vertrag in
[`robot/src/rosmaster_assist/protocol.py`](../robot/src/rosmaster_assist/protocol.py).

## Zum Testen ohne ESP

Mit laufendem Server lassen sich Messwerte von Hand einspeisen — nützlich, um
die Anzeige in der App zu prüfen:

```python
# pip install websockets
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        await ws.send(json.dumps({"type": "vitals", "bpm": 72, "motion": True}))
        await asyncio.sleep(1)

asyncio.run(main())
```

## Offene Fragen

- Welcher Pulssensor? (z.B. MAX30102 über I²C)
- Bewegungserkennung über PIR-Sensor oder über die Kamera mit dem AI HAT+?
  Die Kamera wäre deutlich zuverlässiger, aber aufwendiger.
- Trägt die Person den ESP am Körper, oder hängt er im Raum? Davon hängt ab,
  ob Pulsmessung überhaupt möglich ist.
