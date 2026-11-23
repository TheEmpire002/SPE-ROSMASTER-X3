# ESP32 — person tracking and vital signs

Not implemented yet. This folder describes what the ESP is supposed to do and
how it connects, so that someone can get started right away.

## Task

The ESP32 observes a person in the room and reports:

- **Motion** — is someone active? A longer period without motion can indicate
  a fall.
- **Pulse** — a rough guide value, not a measuring instrument.

## Connection

The ESP connects over Wi-Fi to the same WebSocket as the app:

```
ws://<IP-of-the-Pi>:8000/ws
```

and sends once per second:

```json
{"type": "vitals", "bpm": 72, "motion": true}
```

The robot forwards this unchanged to all connected apps. Nothing more happens
for now — the evaluation comes later and deliberately on the server side, not
on the ESP.

The full protocol is in [`docs/api.md`](../docs/api.md), the binding contract
in
[`robot/src/rosmaster_assist/protocol.py`](../robot/src/rosmaster_assist/protocol.py).

## Testing without an ESP

With the server running, readings can be fed in by hand — useful for checking
the display in the app:

```python
# pip install websockets
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://localhost:8000/ws") as ws:
        await ws.send(json.dumps({"type": "vitals", "bpm": 72, "motion": True}))
        await asyncio.sleep(1)

asyncio.run(main())
```

## Open questions

- Which pulse sensor? (e.g. MAX30102 via I²C)
- Motion detection via a PIR sensor or via the camera with the AI HAT+?
  The camera would be much more reliable, but more work.
- Does the person wear the ESP on the body, or does it hang in the room? This
  decides whether pulse measurement is possible at all.
