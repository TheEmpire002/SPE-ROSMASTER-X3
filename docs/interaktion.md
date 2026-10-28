# Interaktion: Gesten, Sprache, „Geist“

> **Stand:** Konzept vom 2026-10-28, freigegeben. Die Umsetzung läuft in
> Stufen (siehe unten), und jede Stufe trägt hier ein, was fertig ist. Den
> Projektverlauf dazu beschreibt `Marlon/PROJEKTDOKU.md`.

Der Roboter soll Menschen wahrnehmen, auf Handzeichen und Zurufe reagieren
und dabei wirken, als hätte er ein eigenes Innenleben. Alles läuft **im
bestehenden FastAPI-Server**. Es gibt keinen zweiten Prozess an Board, Lidar
oder Kamera.

## Überblick

```
 Wahrnehmung          Deutung           Geist                    Verhalten        Sicherheit
 ───────────          ───────           ─────                    ─────────        ──────────
 C270 ─► Hailo-Pose ─► gesten.py ──┐                             ┌► Folgen    ┐
 Mikro ─► Vosk ──────► sprache.py ─┼─► geist.py                 ─┼► Herkommen ├─► SafetyWatchdog.submit()
 Lidar ────────────────────────────┘   Aufmerksamkeit, Stimmung, ├► Weggehen  │   + Lidar-Stopp
                                       Gedächtnis, Absicht       └► Stopp     ┘
                                            │
                                            └─► denken.py  (Claude-API / lokal / Vorlagen)
                                                 → Gedanken und Antworten: Text in der App + Stimme (Piper)
```

## Leitgedanken

**1. Reflexe sind lokal und deterministisch.**
Gesten und Sprachbefehle werden ohne Sprachmodell erkannt und lösen feste
Verhalten aus. „Stopp“ muss auch ohne Internet sofort wirken.

**2. Das Sprachmodell steuert nie direkt.**
Es formuliert Gedanken und Antworten. Es darf höchstens eine Absicht aus
einer festen Liste vorschlagen (`folgen`, `stopp`, `herkommen`, `weggehen`,
`umschauen`), und der Geist prüft sie gegen den aktuellen Zustand.
Fahrbefehle entstehen ausschließlich in den Verhaltensklassen. Begründung
siehe E6 in [`entscheidungen.md`](entscheidungen.md).

**3. Gleiche Sicherheit wie überall.**
Jedes Verhalten fährt über `SafetyWatchdog.submit()`. Der Lidar-Stopp
(`EXPLORE_STOP_M`) gilt immer, auch beim Folgen. Not-Stopp, Handsteuerung,
Heimfahrt und Verbindungsverlust brechen jedes Verhalten ab, genau wie
beim Erkunder.

**4. Bewegt wird nur am Boden.**
Der Lidar sieht keine Tischkanten. Der Interaktionsmodus ist deshalb
standardmäßig **aus** und muss in der App bewusst eingeschaltet werden.

**5. Fehlt etwas, läuft der Rest weiter.**
Ohne Hailo gibt es keine Gesten, ohne Mikrofon oder Vosk keine Sprache, ohne
Lautsprecher nur Text, ohne Internet antworten Vorlagen statt Claude. Kein fehlendes Teil darf den Server
anhalten.

## Module (geplant in `robot/src/rosmaster_assist/`)

| Modul | Verantwortung |
|---|---|
| `sensors/pose.py` | Körperpose auf dem Hailo-10H (`yolov8s_pose_h10.hef`, 17 Punkte nach COCO). Läuft als Thread wie `sensors/personen.py` mit ~10–15 Hz. Liefert Personen mit Box, Keypoints und Winkel zur Bildmitte. Im Mock gibt es simulierte Personen. |
| `interaktion/gesten.py` | Reine Funktionen über eine kurze Keypoint-Historie (~1,5 s). Ohne Hardware testbar. |
| `interaktion/sprache.py` | USB-Mikrofon über Vosk (offline, deutsch). Erkennt eine feste Befehlsgrammatik plus freien Text nach dem Weckwort. |
| `interaktion/stimme.py` | Sprachausgabe über Piper auf dem USB-Lautsprecher, in einem eigenen Thread mit Warteschlange. |
| `interaktion/verhalten.py` | `Folgen`, `Herkommen`, `Weggehen` im Muster von `navigation/explorer.py` (`start`, `abbrechen`, `aktiv`, `letzter_grund`, `zustand`). |
| `interaktion/geist.py` | Innerer Zustand, Gedächtnis, Absichtswahl, Ausdrucksbewegungen, Gedankenstrom. Takt ~2 Hz. |
| `interaktion/denken.py` | Hybrider Sprachkern mit derselben Schnittstelle für alle Stufen: Claude-API, später lokales LLM auf dem Hailo, Vorlagen. |

## Gesten (Version 1)

Die Gesten werden aus den Körperpunkten bestimmt (Schultern, Ellbogen,
Handgelenke, Nase). Bildkoordinaten: y wächst nach unten.

| Geste | Erkennung | Wirkung |
|---|---|---|
| **Folgen** | Ein Handgelenk über dem Kopf, ≥ 1 s gehalten | Verhalten *Folgen* startet |
| **Stopp** | Beide Handgelenke über den Schultern, *oder* ein Arm waagerecht nach vorn (Handgelenk ≈ Schulterhöhe) | Jedes Verhalten endet sofort |
| **Herkommen** | Ein Handgelenk bewegt sich mehrmals auf und ab (Winken zum Körper) | Auf die Person zu bis 0,7 m |
| **Weggehen** | Ein Handgelenk wischt seitlich vom Körper weg | ~1 m zurück, dann stehen |

Eine Geste zählt erst, wenn sie über mehrere aufeinanderfolgende Bilder
stabil erkannt wurde (`GESTE_*` in `config.py`). Ein einzelnes Fehlbild löst
also nichts aus. **Stopp** hat Vorrang vor allem anderen.

Weil die C270 nur leicht nach oben geneigt ist, sieht sie eine stehende
Person erst ab etwa 1,5–2 m Abstand vollständig. Die Gesten brauchen
mindestens Schultern und Handgelenke im Bild.

## Verhalten

- **Folgen:**
  - Ziel ist die Person, die die Geste gemacht hat. Die Richtung kommt aus der Kamera, der Abstand aus dem Lidar in genau dieser Richtung.
  - Sollabstand `FOLGEN_ABSTAND_M` (1,0 m), P-Regler auf `vx` und `omega`, begrenzt durch `FOLGEN_MAX_SPEED`.
  - Ist die Person länger als 2 s weg, hält er an und dreht sich kurz suchend.
- **Herkommen:** fährt auf die Person zu bis 0,7 m und bleibt stehen.
- **Weggehen:** weicht ~1 m zurück und bleibt stehen.
- Es ist immer nur **ein** Verhalten aktiv. Erkunder, Heimfahrt und Interaktion brechen sich gegenseitig ab.

## Sprache

- **Eingang:** das USB-Mikrofon (ALSA „Microphone“, `sounddevice`; Ersatz: Mikrofon der C270), erkannt mit **Vosk** und dem kleinen deutschen Modell auf der CPU.
- **Befehle mit fester Grammatik** sind schnell und robust: „folge mir“, „stopp“, „komm her“, „geh weg“, „hallo“.
- **Freier Text** geht nur nach dem Weckwort an `denken.py`. Der Name ist noch offen.
- **Ausgabe:** über den USB-Lautsprecher mit **Piper** (offline, deutsche Stimme). Antworten und Gedanken erscheinen zusätzlich als Text in der App. Ohne Lautsprecher oder Piper bleibt es beim Text.
- Während er spricht, ignoriert die Erkennung das Mikrofon, damit er sich nicht selbst zuhört.

## Geist: Wie der Roboter „bewusst“ wirkt

Der Roboter hat kein Bewusstsein. Er soll aber nachvollziehbar so wirken.
Dafür sorgen diese vier Bausteine:

- **Innerer Zustand:**
  - Energie (vorerst aus der Laufzeit, später aus dem Akku),
  - Neugier (steigt, wenn nichts passiert),
  - Stimmung (Lob hebt sie, Ignorieren senkt sie),
  - Aufmerksamkeit (welche Person er gerade anschaut).
- **Gedächtnis:**
  - Kurzzeit: die letzten ~50 Ereignisse („Person erschienen“, „Geste Folgen“, „Hindernis“).
  - Langzeit: eine kleine Datei `~/.rosmaster/erinnerung.json`.
- **Absicht:** Aus Gesten, Sprache und Zustand wählt der Geist, was als Nächstes passiert, und startet bzw. stoppt die Verhalten.
- **Ausdruck:**
  - Kleine Bewegungen: zur Person drehen, kurz „wackeln“ bei Freude, zurückweichen bei Erschrecken. Sie laufen über den Watchdog und nur, wenn kein Verhalten fährt.
  - Die Stimme: Er spricht Antworten und ab und zu einen Gedanken laut aus.
  - Ein **Gedankenstrom** in der App macht das Innenleben sichtbar.

`denken.py` bekommt den Zustand und die letzten Ereignisse und liefert JSON:
`{"gedanke": "...", "antwort": "...", "absicht": "folgen" | null}`.

Gedanken entstehen nur bei Ereignissen oder alle ~20 s. Das hält die
API-Kosten klein.

Der API-Schlüssel kommt aus der Umgebungsvariable `ANTHROPIC_API_KEY` und
**nie** ins Repo.

## Schnittstelle (geplant, wird bei Umsetzung in `docs/api.md` übernommen)

- **Befehle:**
  - `interaction_start` / `interaction_stop`: Modus an/aus.
  - `follow` / `follow_stop`: Folgen direkt aus der App.
  - `chat` mit `{text}`: Text an den Roboter.
- **Zustand:** `state.interaktion` mit den Feldern `modus`, `verhalten`, `person{winkel, abstand}`, `geste`, `stimmung`, `aufmerksamkeit`, `gedanke`, `letzte_antwort`.
- **Neue Nachricht:** `type: "gedanke"` für den Gedankenstrom.
- **Endpunkte:**
  - `GET /interaktion/status`
  - `GET /interaktion/pose` mit den Keypoints zum Einblenden des Skeletts.

## Stufen

| Stufe | Inhalt | Stand |
|---|---|---|
| 1 | Doku (diese Datei, Projektdoku, Entscheidungen E5/E6) | erledigt |
| 2 | Wahrnehmung: `pose.py`, `gesten.py`, Skelett in der App, **ohne Fahren** | offen |
| 3 | Verhalten: Folgen/Stopp, Herkommen/Weggehen, **nur am Boden** | offen |
| 4 | Sprache: Vosk-Befehle, Stimme (Piper), Chat in der App | offen |
| 5 | Geist und Denken: Zustand, Gedankenstrom, Claude/Vorlagen, Ausdruck | offen |
| 6 | Später: lokales LLM auf dem Hailo, Gesichter, Orbbec-Tiefe | – |

## Testen

- Die Gestenlogik wird ohne Hardware geprüft, mit synthetischen Keypoint-Folgen pro Geste plus Negativfällen: `python Marlon/interaktion/test_gesten.py`. Das Muster ist dasselbe wie in `test_hindernisse.py`.
- `python main.py --mock` liefert simulierte Personen und Gesten.
- Am Pi:
  - `/interaktion/status` zeigt, ob der Hailo verfügbar ist, und die Pose-Rate.
  - Dabei auf CPU-Last und `dmesg` achten, damit keine USB-Resets auftreten (siehe [`hardware.md`](hardware.md)).
