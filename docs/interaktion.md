# Interaktion: Gesten, Sprache, „Geist“

> **Stand 2026-10-28:** Stufen 1–3 umgesetzt (Pose auf dem Hailo, Gesten,
> Folgen/Herkommen/Weggehen/Stopp). Aus Stufe 4 ist die **Sprachausgabe**
> fertig (`stimme.py`, USB-Lautsprecher). Spracherkennung und Geist sind
> noch Konzept.
> Den Projektverlauf dazu beschreibt `Marlon/PROJEKTDOKU.md`.

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

## Module (in `robot/src/rosmaster_assist/`)

| Modul | Verantwortung |
|---|---|
| `sensors/pose.py` | **fertig.** Körperpose auf dem Hailo-10H (`POSE_HEF`, 17 Punkte nach COCO). Thread wie `sensors/personen.py`, `POSE_HZ` (10 Hz). Das Hailo-Gerät wird nur geöffnet, solange die Interaktion läuft, und nach Fehlern neu geöffnet. Liefert Personen mit Box, Keypoints und Winkel zur Bildmitte. Im Mock spielt eine simulierte Person die Gesten der Reihe nach vor. |
| `interaktion/gesten.py` | **fertig.** `Gestenerkennung` über eine kurze Keypoint-Historie (`GESTE_FENSTER_S`, 1,5 s). Ohne Hardware testbar. |
| `interaktion/sprache.py` | *geplant.* USB-Mikrofon über Vosk (offline, deutsch). Erkennt eine feste Befehlsgrammatik plus freien Text nach dem Weckwort. |
| `interaktion/stimme.py` | **fertig.** Sprachausgabe auf dem USB-Lautsprecher (ALSA `UACDemoV10`): Piper → `aplay`, Ersatz espeak-ng, sonst nur Text. Eigener Thread mit kleiner Warteschlange (`STIMME_*` in `config.py`). `spricht` sagt der künftigen Spracherkennung, wann sie weghören muss. Im Mock wird nur geloggt. |
| `interaktion/verhalten.py` | **fertig.** Klasse `Interaktion` mit den Modi `bereit`, `folgen`, `herkommen`, `weggehen`, `sucht`; Bedienung wie `navigation/explorer.py` (`start`, `folgen`, `folgen_stop`, `abbrechen`, `aktiv`, `letzter_grund`, `zustand`, `meldungen_abholen`). |
| `interaktion/geist.py` | *geplant.* Innerer Zustand, Gedächtnis, Absichtswahl, Ausdrucksbewegungen, Gedankenstrom. Takt ~2 Hz. |
| `interaktion/denken.py` | *geplant.* Hybrider Sprachkern mit derselben Schnittstelle für alle Stufen: Claude-API, später lokales LLM auf dem Hailo, Vorlagen. |

## Gesten (Version 1)

Die Gesten werden aus den Körperpunkten bestimmt (Schultern, Ellbogen,
Handgelenke, Nase). Bildkoordinaten: y wächst nach unten. Alle Maße sind in
**Schulterbreiten** gerechnet und auf die Schultermitte bezogen, damit der
Abstand der Person keine Rolle spielt. Die Schulterbreite ist der Median
über das Fenster: Dreht sich die Person beim Winken kurz seitlich, wird
die Breite in einem Bild winzig, und ohne Median würde jede kleine
Handbewegung riesig wirken.

| Geste | Erkennung | Wirkung |
|---|---|---|
| **Folgen** | Ein Handgelenk über dem Kopf, ≥ 1 s gehalten | Verhalten *Folgen* startet |
| **Stopp** | Beide Handgelenke über den Schultern, ≥ 0,4 s gehalten. (Der „Arm waagerecht nach vorn“ aus dem Konzept ist in 2D von hängenden Armen kaum zu unterscheiden und entfällt.) | Jedes Verhalten endet sofort |
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
- Sicherheit wie beim Erkunder: vorwärts nur bei freiem Fahrschlauch (`frei_in_richtung`), rückwärts nur bei freiem Raum hinten, ohne frische Lidar-Daten oder bei etwas sehr Nahem steht er. Im Modus `bereit` schickt er keine Befehle.
- Springt die Kamera auf eine andere Person (Winkel ändert sich um mehr als `INTERAKTION_SPRUNG_GRAD`), wechselt er nicht auf die neue, sondern behandelt die bisherige als kurz verloren.
- `herkommen` und `weggehen` enden spätestens nach `INTERAKTION_VERHALTEN_MAX_S`.
- Alle Werte: `FOLGEN_*`, `HERKOMMEN_*`, `WEGGEHEN_*` in `config.py`.

## Sprache

- **Eingang:** das USB-Mikrofon (ALSA „Microphone“, `sounddevice`; Ersatz: Mikrofon der C270), erkannt mit **Vosk** und dem kleinen deutschen Modell auf der CPU.
- **Befehle mit fester Grammatik** sind schnell und robust: „folge mir“, „stopp“, „komm her“, „geh weg“, „hallo“.
- **Freier Text** geht nur nach dem Weckwort an `denken.py`. Der Name ist noch offen.
- **Ausgabe (umgesetzt):** über den USB-Lautsprecher mit **Piper** (offline, Stimme `de_DE-thorsten-medium`). Fehlt Piper, spricht espeak-ng, fehlt auch das, bleibt es beim Log. Der zuletzt gesagte Satz steht in `state.stimme` und in der App. Einrichtung: [`hardware.md`](hardware.md).
  - Er sagt kurz, was er tut: „Ich folge dir.“, „Ich komme.“, „Da bin ich.“, „Wo bist du?“, „Okay, ich halte an.“ und beim Not-Stopp „Halt.“. Die Sätze stehen in `verhalten.py` (`_GESTE_SAGEN` und die Aufrufe von `_wechseln(..., sagen=…)`).
  - Beim Start sagt er „Ich bin bereit.“. So hört man sofort, ob der Lautsprecher geht.
  - Aus der App: Feld „Sagen“ im Interaktionsbereich (Befehl `say`).
  - Gleiche Sätze innerhalb von `STIMME_WIEDERHOLUNG_S` werden verschluckt. Die Schlange hält höchstens `STIMME_MAX_WARTESCHLANGE` Sätze, das Älteste fällt weg. Stopp hat Vorrang und schneidet den laufenden Satz ab.
- Während er spricht (`Stimme.spricht`, plus `STIMME_NACHLAUF_S`), soll die Erkennung das Mikrofon ignorieren, damit er sich nicht selbst zuhört.

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

## Schnittstelle

Umgesetzt und in [`api.md`](api.md) beschrieben („Interaktion im Detail“):

- Befehle `interaction_start` / `interaction_stop`, `follow` / `follow_stop`.
- `state.interaktion` mit `aktiv`, `verfuegbar`, `modus`, `text`, `person{winkel, abstand}`, `personen`, `geste`, `grund`.
- `GET /interaktion/status` und `GET /interaktion/pose` (Skelett zum Einblenden; die Web-App zeichnet es über das Kamerabild).
- Stimme: Befehle `say {text, vorrang}` und `say_stop`, `state.stimme` mit `verfuegbar`, `spricht`, `text`, Diagnose unter `GET /stimme/status`.

Für Stufe 4/5 geplant (nur ergänzen): Befehl `chat {text}`, Felder `stimmung`, `aufmerksamkeit`, `gedanke`, `letzte_antwort`, Nachricht `type: "gedanke"`.

## Stufen

| Stufe | Inhalt | Stand |
|---|---|---|
| 1 | Doku (diese Datei, Projektdoku, Entscheidungen E5/E6) | erledigt |
| 2 | Wahrnehmung: `pose.py`, `gesten.py`, Skelett in der App, **ohne Fahren** | erledigt (Mock + Tests), Hailo am Pi zu prüfen |
| 3 | Verhalten: Folgen/Stopp, Herkommen/Weggehen, **nur am Boden** | erledigt (Mock), Fahrtest am Boden offen |
| 4 | Sprache: Vosk-Befehle, Stimme (Piper), Chat in der App | Stimme erledigt (Mock + Tests), am Pi zu prüfen; Vosk und Chat offen |
| 5 | Geist und Denken: Zustand, Gedankenstrom, Claude/Vorlagen, Ausdruck | offen |
| 6 | Später: lokales LLM auf dem Hailo, Gesichter, Orbbec-Tiefe | – |

## Testen

- Die Gestenlogik wird ohne Hardware geprüft, mit synthetischen Keypoint-Folgen pro Geste plus Negativfällen: `python Marlon/interaktion/test_gesten.py`. Das Muster ist dasselbe wie in `test_hindernisse.py`.
- Die Sprachausgabe (Warteschlange, Vorrang, Ausweichen auf Text, Sätze des Verhaltens) prüft `python Marlon/interaktion/test_stimme.py`, ebenfalls ohne Lautsprecher.
- `python main.py --mock` liefert eine simulierte Person, die nacheinander Herkommen, Weggehen, Folgen und Stopp vormacht. In der App „Interaktion an“ drücken und die Meldungen ansehen.
- Am Pi:
  - `/interaktion/status` zeigt, ob der Hailo verfügbar ist, und die Pose-Rate.
  - Dabei auf CPU-Last und `dmesg` achten, damit keine USB-Resets auftreten (siehe [`hardware.md`](hardware.md)).
