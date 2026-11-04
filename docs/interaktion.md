# Interaktion: Gesten, Sprache, „Geist“

> **Stand 2026-10-29:** Stufen 1–3 umgesetzt (Pose auf dem Hailo, Gesten,
> Folgen/Herkommen/Weggehen/Stopp). Aus Stufe 4 sind **Sprachausgabe**
> (`stimme.py`, USB-Lautsprecher) und **Sprachbefehle** mit dem Weckwort
> „Robi“ (`sprache.py`, Vosk) fertig. Freier Text und Geist sind noch
> Konzept.
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
| `sensors/pose.py` | **fertig.** Körperpose auf dem Hailo-10H (`POSE_HEF`, 17 Punkte nach COCO). Thread wie `sensors/personen.py`, `POSE_HZ` (10 Hz). Das Hailo-Gerät wird beim Serverstart geöffnet und warm gehalten (`POSE_WARM_HALTEN`, siehe „Antrieb weg beim Start der Gesten“), nach Fehlern neu geöffnet. Liefert Personen mit Box, Keypoints und Winkel zur Bildmitte. Im Mock spielt eine simulierte Person die Gesten der Reihe nach vor. |
| `interaktion/gesten.py` | **fertig.** `Gestenerkennung` über eine kurze Keypoint-Historie (`GESTE_FENSTER_S`, 1,5 s). Ohne Hardware testbar. |
| `interaktion/sprache.py` | **fertig.** `Spracherkennung`: USB-Mikrofon (`sounddevice`) → Vosk (offline, kleines deutsches Modell) mit fester Grammatik. Eigener Thread, verbindet sich nach Ausfällen neu. Befehle nur mit Weckwort „Robi“, Stopp auch ohne. Der Server holt die Befehle ab (`befehle_abholen`) und führt sie im Event-Loop aus. Freier Text nach dem Weckwort ist noch offen. |
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

**Entprellt (29.10.):** Beim Folgen wechselte der Roboter manchmal in ein
anderes Verhalten, ohne dass jemand ein Zeichen gegeben hatte. Wer läuft,
schwingt die Arme, und das sah nach Herkommen oder Weggehen aus. Deshalb
gilt für **Herkommen** und **Weggehen** zusätzlich:

- Die Person muss stillstehen: Schultermitte und Schulterbreite dürfen sich
  über das Fenster kaum ändern (`GESTE_RUHE_WEG`, `GESTE_RUHE_GROESSE`, in
  Schulterbreiten). Wer auf den Roboter zu- oder an ihm vorbeiläuft, zählt
  nicht.
- Die Person muss zur Kamera schauen (Nase sichtbar). Von hinten gibt es
  keine Geste.
- Der Roboter selbst muss annähernd stehen (`GESTE_ROBOTER_RUHIG_V`,
  `GESTE_ROBOTER_RUHIG_W`, gelesen aus `watchdog.current_command`). Fährt er,
  bewegt sich das ganze Bild, und Armbewegungen sind nicht zu trennen.
- Dieselbe Geste muss `GESTE_BESTAETIGEN`-mal hintereinander erkannt werden.

**Stopp** und **Folgen** sind davon ausgenommen. Stopp muss immer sofort
wirken, und Folgen (Arm über dem Kopf, gehalten) wird beim Laufen nicht
zufällig ausgelöst.

Weil die C270 nur leicht nach oben geneigt ist, sieht sie eine stehende
Person erst ab etwa 1,5–2 m Abstand vollständig. Die Gesten brauchen
mindestens Schultern und Handgelenke im Bild.

## Verhalten

- **Folgen:**
  - Ziel ist die Person, die die Geste gemacht hat. Die Richtung kommt aus der Kamera, der Abstand aus dem Lidar: aus dem Lidar-Cluster im Winkelbereich der Box (siehe „Abstand zur Person“).
  - Sollabstand `FOLGEN_ABSTAND_M` (1,0 m), P-Regler auf `vx` und `omega`, begrenzt durch `FOLGEN_MAX_SPEED`.
  - Ist die Person länger als 2 s weg, sagt er „Wo bist du?“ und sucht: Er dreht sich zu der Seite, auf der sie verschwand, je 1,2 s drehen und 0,8 s stillhalten (`FOLGEN_SUCHEN_DREH_S`, `FOLGEN_SUCHEN_PAUSE_S`). Im Drehen verwischt das Kamerabild, im Stand erkennt die Pose scharf. Seit dem 03.11. sucht er so lange im Kreis, bis er wieder jemanden sieht (`FOLGEN_SUCHEN_S` = 0), dann folgt er dieser Person. Vorher gab er nach 6 s auf. Beenden mit Stopp, Stopp-Geste oder „Robi, gute Nacht“.
- **Herkommen:** fährt auf die Person zu bis 0,7 m und bleibt stehen.
- **Weggehen:** weicht ~1 m zurück und bleibt stehen.
- Es ist immer nur **ein** Verhalten aktiv. Erkunder, Heimfahrt und Interaktion brechen sich gegenseitig ab.
- Sicherheit wie beim Erkunder: vorwärts nur bei freiem Fahrschlauch (`frei_in_richtung`), rückwärts nur bei freiem Raum hinten, ohne frische Lidar-Daten oder bei etwas sehr Nahem steht er. Im Modus `bereit` schickt er keine Befehle.
- Welcher Person er folgt, entscheidet der **Ziel-Lock** (siehe unten). Eine andere Person wird nie von selbst übernommen.
- `herkommen` und `weggehen` enden spätestens nach `INTERAKTION_VERHALTEN_MAX_S`.
- **Ohne Antrieb** (Board weg, USB-Reset) hält er an und geht nach `bereit`, der Interaktionsmodus bleibt aber an. Fahrende Modi werden abgelehnt, solange `driver.connected` falsch ist; ist das Board wieder da, geht es mit der nächsten Geste oder dem nächsten Sprachbefehl weiter.
- Alle Werte: `FOLGEN_*`, `HERKOMMEN_*`, `WEGGEHEN_*` in `config.py`.

### Abstand zur Person (03.11.)

Früher galt der kleinste Lidar-Wert im Winkelbereich der Box als Abstand.
Stand ein Stuhl oder Tisch vor der Person, las der Roboter dessen Abstand,
hielt an und „hing“, während die Person weiterging.

Jetzt werden die Cluster aus `navigation/hindernisse.py` im Bereich der Box
(plus `FOLGEN_CLUSTER_RAND_GRAD`, weil Kamera und Lidar nicht zeitgleich
messen) betrachtet, Wände ausgenommen. Gewählt wird in dieser Reihenfolge:

1. zwei Menschenbeine („Mensch“),
2. passt zum letzten guten Abstand (Sprung unter `FOLGEN_CLUSTER_SPRUNG_M`,
   solange dieser jünger als `FOLGEN_CLUSTER_GUT_S` ist),
3. andere Objekte vor Möbelbeinen (Stuhl, Tisch, Bein),
4. Winkel am nächsten an der Boxmitte.

Gibt es keinen Cluster (Person zu weit, steht an der Wand), gilt wie früher
das Minimum. Die Sicherheit hängt nicht daran: Die Vorwärtsfahrt ist immer
durch den freien Fahrschlauch begrenzt, und der `AutonomSchutz` im Watchdog
bremst nach Bremsweg (siehe `docs/architektur.md`).

### Sanft fahren (29.10.)

Beim Folgen fuhr der Roboter ruckartig, und gleichzeitig traten USB-Resets
auf. Ursache waren harte Sprünge im Fahrbefehl: Person kurz nicht erkannt →
sofort 0, wieder da → sofort volle Fahrt; dazu ein zitternder Kamerawinkel,
der `omega` hin und her riss. Jede dieser Stufen erzeugt eine Stromspitze.

| Maßnahme | Wirkung | Einstellung |
|---|---|---|
| Anfahrrampe im Watchdog | kein Befehl springt mehr auf die Motoren, gilt für alles; in der App unter Einstellungen → „Sanft fahren“ abschaltbar | `RAMPE_*` |
| Winkel glätten | gleitender Mittelwert über die Bilder, kleine Ruckler fallen weg | `FOLGEN_WINKEL_GLAETTUNG` |
| Totzone | unter wenigen Grad wird nicht nachgedreht | `FOLGEN_WINKEL_TOTZONE` |
| weicher Übergang Drehen/Fahren | zwischen `FOLGEN_VOLL_WINKEL` und `FOLGEN_MAX_WINKEL_FAHREN` wird das Vorwärtstempo allmählich kleiner, statt hart auf 0 zu fallen | `FOLGEN_VOLL_WINKEL` |
| sanftere Drehung | kleinerer P-Anteil auf `omega` | `FOLGEN_KP_W` |
| Ausrollen | ist die Person kurz weg, läuft der letzte Befehl über `FOLGEN_HALTEN_S` linear aus, mit Prüfung des freien Raums | `FOLGEN_HALTEN_S` |
| Lidar-Abstand halten | fehlt der Lidar-Abstand in einem Takt, gilt kurz der letzte gute Wert | `FOLGEN_HALTEN_S` |
| Hysterese „nah“ | der Zustand „Person sehr nah“ endet erst nach `FOLGEN_NAH_HALTEN_S`, damit er nicht im Takt hin und her schaltet | `FOLGEN_NAH_HALTEN_S` |

Not-Stopp, Stopp-Geste und das Ende eines Verhaltens rufen `watchdog.stop()`
und stehen sofort, ohne Rampe. Bei einem Hindernis oder fehlenden
Lidar-Daten schickt das Verhalten einen Haltbefehl; der läuft über die
Bremsrampe (`RAMPE_BREMSEN_XY` = 2 m/s², von 0,3 m/s also in 0,15 s auf
etwa 2 cm). Die Bremsrampe ist bewusst deutlich steiler als die Anfahrrampe.

### Antrieb weg beim Start der Gesten (29.10.)

Beim Einschalten der Gestensteuerung kam oft „Keine Verbindung zum
Antrieb“, und die Interaktion war sofort wieder aus. Zwei Ursachen:

1. **Lastspitze:** Erst beim Einschalten wurde der Hailo geöffnet und das
   Netz geladen. Das zieht kurz viel Strom, und am wackligen USB-Hub meldet
   sich dann das Board ab (siehe [`hardware.md`](hardware.md), USB-Resets).
   **Lösung:** Der Hailo wird schon beim Serverstart geöffnet, solange der
   Roboter steht, und bleibt offen (`POSE_WARM_HALTEN = True`). Gerechnet
   wird weiterhin nur, während die Interaktion an ist.
2. **Zu harte Reaktion:** Ging der Antrieb verloren, beendete der Server den
   ganzen Interaktionsmodus. **Lösung:** `Interaktion.antrieb_verloren()`
   hält nur an (Modus `bereit`). Die Interaktion bekommt `driver.connected`
   als Rückfrage und lehnt Fahrmodi ab, solange das Board fehlt. Der Treiber
   verbindet sich selbst neu, danach geht es ohne Neustart weiter.

### Ziel-Lock (03.11.)

Früher sprang der Roboter beim Folgen auf eine zweite Person über. Die
Zuordnung lief nur über den Kamerawinkel, nach jeder Suche nahm er die
größte Person im Bild. Jetzt gilt: **Im Zweifel stehen bleiben.**

Code: `interaktion/ziel.py` (reine Logik, ohne Hardware testbar), eingebunden
in `interaktion/verhalten.py`.

- **`PersonenTracker`** vergibt Track-IDs über die Bilder hinweg. Je Track
  ein Kalman-Filter auf Winkel und Winkelgeschwindigkeit. Die Vorhersage
  zieht die **Eigendrehung** des Roboters ab (gemessene Drehrate aus der
  Odometrie, sonst der Fahrbefehl), sonst sähe jede Drehung wie eine
  laufende Person aus. Zuordnung gierig nach Kosten aus Winkelabstand
  (Gate `ZIEL_GATE_GRAD`, wächst mit der Zeit ohne Messung), Boxhöhe und
  Aussehen. Ein Track ohne Messung lebt `ZIEL_TRACK_HALTEN_S` weiter.
- **Aussehen:** `sensors/pose.py` berechnet je Person ein HSV-Histogramm
  (16 × 8) des Rumpfs zwischen Schultern und Hüften (`merkmal`). Verglichen
  wird per Bhattacharyya-Abstand.
- **`ZielSperre`** hält das Ziel: Track-ID, Referenz-Merkmal (Mittel aus der
  Erfassung, danach nur bei eindeutiger Szene langsam nachgeführt,
  `ZIEL_MERKMAL_NACHFUEHREN`) und den Zustand:

| Zustand (`ziel.zustand`) | Bedeutung | Roboter |
|---|---|---|
| `idle` | kein Ziel | keine Befehle |
| `acquiring_target` | Ziel wird erfasst, Merkmal wird gelernt | fährt schon, sobald das Ziel eindeutig ist |
| `following` | Ziel bestätigt | folgt nur dem Ziel-Track |
| `target_temporarily_lost` | Ziel-Track ohne Messung | rollt aus, steht |
| `searching_for_target` | länger verloren (`FOLGEN_VERLOREN_S`) | dreht langsam zur letzten Seite |
| `safe_stop` | nicht wiedergefunden oder nicht eindeutig | steht, `grund` sagt warum |

- **Erste Auswahl:** die Person, die die Folgen-Geste macht (Gesten werden
  je Track erkannt), oder die in der App gewählte. Startet man ohne Auswahl
  bei mehreren Personen, lehnt er ab: „mehrere Personen – bitte im Bild
  antippen“.
- **Wiederfinden:** Während einer kurzen Verdeckung gehört derselbe Track
  sofort wieder dem Ziel. Ein neuer Track muss zum Merkmal passen
  (`ZIEL_MERKMAL_SCHWELLE`), deutlich besser als jeder andere Kandidat
  (`ZIEL_MERKMAL_MARGE`), und das in `ZIEL_BESTAETIGEN_BILDER` Bildern
  hintereinander.
- **Sicher anhalten:** Nach `FOLGEN_SUCHEN_S` ohne bestätigten Kandidaten
  oder wenn zwei Personen gleich gut passen, geht er nach `safe_stop`.
  Daraus kommt er nur durch eine neue Auswahl. Gleich gekleidete Personen
  kann das Farbmerkmal nicht trennen; dann hält er bewusst an, statt zu
  raten.
- **Gesten anderer Personen:** Beim Folgen zählen nur die Gesten des Ziels.
  Ausnahme ist **Stopp**: Das darf jeder.
- **Datenschutz:** Merkmal und Track-IDs liegen nur im RAM. Sie werden bei
  Freigabe, Ende der Interaktion und Serverende gelöscht. Es werden keine
  Bilder gespeichert und keine dauerhafte Personen-ID vergeben.
  `/interaktion/pose` gibt das Merkmal nicht heraus.

Abweichungen vom Plan:

- Die Zeiten heißen weiter `FOLGEN_VERLOREN_S` und `FOLGEN_SUCHEN_S`.
  Eigene `ZIEL_*`-Namen hätten nur doppelte Einstellungen gebracht.
- In `acquiring_target` fährt er schon los, sobald das Ziel eindeutig ist.
  Die Sicherheit kommt von der Eindeutigkeit, nicht von einer Wartezeit im
  Stand; das Merkmal wird währenddessen weiter gelernt.

Test: `Marlon/interaktion/test_ziel.py`. Er prüft Kreuzen mit Verdeckung,
Eigendrehung, kurze und lange Verdeckung, zweite Person während der Suche,
gleiche Kleidung, App-Start bei zwei Personen, Geste wählt Gestikulierenden
und Stopp von Passanten.

### Bedienung in der Web-App (03.11.)

Gesteuert wird über die Web-App; Gesten und Sprache bleiben Nebenwege.

- **Person antippen:** Solange die Interaktion läuft, sind die Personen im
  Bild der Personenkamera antippbar. Ein Tipp schickt `follow {ziel_id}` mit
  der Track-ID aus `/interaktion/pose`. Das geht auch während des Folgens:
  Dann wechselt das Ziel. Ein eigener Knopf „Neu wählen“ wäre damit doppelt
  und entfällt. Liegen Boxen übereinander, gilt die kleinere, meist die
  weiter hinten stehende, die man gezielt gemeint hat.
- **Anzeige:** Das Ziel ist grün, alle anderen grau, über jedem Kopf steht
  „Person N“. Die Zeile „Ziel“ zeigt den Zustand (z. B. „folgt Person 3 ·
  82 %“, „sucht Person 3 …“, „sicher angehalten – bitte neu wählen“).
- **Pause / Weiter:** `follow_pause` hält an, Tracker und Sperre laufen
  weiter, damit nach `follow_resume` noch dieselbe Person das Ziel ist.
  Während der Pause greifen keine Zeitgrenzen (Verloren, Suchen); die
  Stopp-Geste beendet die Pause wie jeden Modus.
- **Loslassen:** `follow_stop` gibt das Ziel frei und quittiert auch ein
  `safe_stop`.
- Tippt man eine Person an, die der Tracker gerade verloren hat, lehnt er
  ab („bitte neu antippen“), statt jemand anderen zu nehmen.

Test: ebenfalls `test_ziel.py` (Antippen, Umwählen, unbekannte ID, Pause
über die Suchzeit hinaus, Weiter, Stopp in der Pause, IDs in der Pose,
Loslassen aus `safe_stop`).

## Sprache

- **Eingang:** das USB-Mikrofon (FongLun „USB Microphone“ bzw. C-Media, `SPRACHE_MIKROFON_NAMEN`; die Mikrofone in C270 und Orbbec sind über `SPRACHE_MIKROFON_NIE` ausgeschlossen, das der C270 versteht zu schlecht), erkannt mit **Vosk** und dem kleinen deutschen Modell (`SPRACHE_MODELL`) auf der CPU. Fehlen Vosk, Modell oder Mikrofon, ist die Erkennung aus (`/sprache/status` sagt warum), der Server läuft weiter.
- **Feste Grammatik:** Vosk darf nur die Wörter der Befehle und des Weckworts erkennen, der Rest wird `[unk]`. Das macht die Erkennung schnell und robust gegen Gespräche im Raum. Wörter, die das Modell nicht kennt (z. B. „robi“), fallen aus der Grammatik heraus; deshalb gibt es Varianten (`SPRACHE_WECKWORT_VARIANTEN`: „robbie“, „ruby“ …).
- **Weckwort „Robi“:** Ein Befehl gilt nur, wenn „Robi“ im selben Satz vorkommt („Robi, folge mir“, „folgen Robi“) oder höchstens `SPRACHE_WECK_FENSTER_S` (5 s) vorher allein gesagt wurde. Ein Befehl verbraucht das Fenster.
- **Stopp geht immer:** „Stopp“, „Halt“, „Bleib stehen“ wirken auch ohne Weckwort (`SPRACHE_STOPP_OHNE_WECKWORT`) und gewinnen gegen jeden Fahrbefehl im selben Satz.
- **Nicht sich selbst zuhören:** Solange die Stimme spricht (`Stimme.spricht` plus `STIMME_NACHLAUF_S`), wird das Mikrofon verworfen.
- **Kehrseite der Grammatik:** Vosk presst jedes Geräusch in einen erlaubten Satz. Musik und Gespräche wurden so zu „robbie hallo“ und schalteten die Interaktion ein (03.11.). Deshalb gilt ein Satz nur ganz, wenn jedes Wort mindestens `SPRACHE_MIN_KONFIDENZ` hat und höchstens `SPRACHE_MAX_UNK_ANTEIL` der Wörter `[unk]` sind; sonst nur Stopp. Ebenso bei Wortsalat: Ein gesprochener Befehl kommt als genau ein Grammatik-Satz, aus Geräuschen hängt Vosk dagegen mehrere aneinander („robin hallo hause zu weg ruby“ → weggehen, „ruby komm da fahr mir la ruby leise box“ → herkommen, beide am 03.11. im Log). Kommt das Weckwort doppelt oder stehen mehr als `SPRACHE_MAX_ZUSATZWOERTER` (0) Wörter neben Weckwort und Wendung, gilt nur Stopp. Läuft MUKKE, braucht jeder Befehl außer Stopp das Weckwort und mindestens `SPRACHE_MIN_KONFIDENZ_MUSIK`. Zuerst galten bei Musik nur Stopp und „Musik aus“, dann war der Roboter nach „Robi, Beatbox“ bis zum Liedende taub. Ein so verworfenes Weckwort öffnet kein Fenster, ebenso wenig ein Weckwort im Wortsalat („robin la“). Verworfenes steht im Log und in `/sprache/status`.
- **„Robi“ allein wird beantwortet:** Nach `SPRACHE_WECK_ANTWORT_NACH_S` (1 s) sagt er „Ja?“ (`SPRACHE_WECK_ANTWORT`), außer der Befehl kam schon. Die Pause verhindert, dass das „Ja?“ jemandem ins Wort fällt, der „Robi … folge mir“ mit kurzem Absatz sagt, denn während er spricht, hört er nicht. Aus demselben Grund läuft das Weckfenster erst ab, wenn er ausgeredet hat.
- **Live-Feed in der App:** Zwei Schalter im Bereich „Interaktion“. *Mitlesen* zeigt Pegel, den Zwischentext von Vosk und die letzten Sätze mit Urteil (Befehl, verworfen samt Grund, Weckwort, kein Befehl), über `GET /sprache/live`. *Mithören* spielt den Mikrofon-Ton im Browser ab (WebSocket `/sprache/ton`, Wiedergabe mit der Web-Audio-API). Beides kostet am Roboter nur etwas, solange es eingeschaltet ist. Gedacht für die Fehlersuche: Man hört, was beim Roboter ankommt, und sieht, warum ein Satz (nicht) als Befehl galt.
- **Durchsage („Über den Roboter sprechen“):** Dritter Schalter. Was man ins Mikrofon von Laptop oder Handy spricht, kommt live aus dem Lautsprecher des Roboters (WebSocket `/sprache/durchsage`, 16 kHz mono, Verzögerung etwa eine halbe Sekunde). Die Durchsage steht in derselben Warteschlange wie die Sätze der Stimme: Musik pausiert, die Spracherkennung hört weg (sonst verstünde sie die Durchsage als Befehl), ein Stopp bricht ab, und die Lautstärke ist dieselbe (`LAUTSTAERKE`). Nach 2 s Stille gibt sie den Lautsprecher frei.
  - **Nur über HTTPS:** Browser geben das Mikrofon nur auf sicheren Seiten frei. Unter `http://10.42.0.62:8000` zeigt die App deshalb einen Link auf `https://10.42.0.62:8443`. Dieselbe App läuft dort mit einem Zertifikat, das der Server beim ersten Start selbst erzeugt. Der Browser warnt einmal („nicht sicher“), die Warnung bestätigen.
  - Gleichzeitig Mithören einschalten führt zu Echo: Das Roboter-Mikrofon hört den eigenen Lautsprecher.
- **Failsafe nach USB-Aussetzern:** Das Mikrofon fällt bei einem Over-Current-Sturm mit ab. Bei einer Fehlermeldung sucht die Erkennung nach `SPRACHE_NEUVERSUCH_S` (3 s) neu, das gab es schon. Neu (03.11.) sind drei Fälle ohne Fehlermeldung:
  - *Stream hängt:* Nach einem Aussetzer kam `strom.read()` nie wieder zurück. Der Pegel war eingefroren, der Zustand blieb „hoert“, Befehle kamen keine mehr, bis zum Neustart des Servers. Gelesen wird jetzt per Callback in eine Warteschlange mit Frist: Kommt `SPRACHE_HAENGT_S` (3 s) lang kein Block, öffnet sie neu. Hängt auch das Schließen des toten Streams, bleibt er liegen (`SPRACHE_SCHLIESSEN_S`), statt den Neuversuch zu blockieren.
  - *Stream tot:* `SPRACHE_STUMM_S` (5 s) lang nur exakte Nullen → neu öffnen. Ein echtes Mikrofon rauscht immer etwas.
  - *Falsches Mikrofon:* Die C270 meldete sich nach einem Aussetzer oft vor dem USB-Mikrofon zurück, und die Erkennung lief dann auf ihr. Seit dem 03.11. wird sie gar nicht mehr genommen; ist das USB-Mikrofon weg, ist die Erkennung aus, bis es wieder da ist. Läuft die Erkennung auf einem weniger bevorzugten Eintrag aus `SPRACHE_MIKROFON_NAMEN`, schaut sie weiterhin alle `SPRACHE_MIKROFON_PRUEFEN_S` in `/proc/asound/cards` nach und wechselt zurück, sobald das bessere wieder da ist.
  - *Hör-Thread steckt fest:* Bleibt schon das Öffnen des Mikrofons in PortAudio hängen, helfen die Fristen oben nicht mehr. Eine Wache startet dann nach `SPRACHE_THREAD_HAENGT_S` (15 s) ohne Fortschritt einen neuen Hör-Thread, höchstens `SPRACHE_THREAD_MAX_NEU` (3) Mal hintereinander. Danach steht in `/sprache/status`: „Server neu starten“.
  - *Mithören in der App:* Ohne Ton schickt der Server jede Sekunde ein Lebenszeichen. Kommt 5 s gar nichts (Pi hängt, WLAN weg, Server neu gestartet), verbindet die App von selbst neu, so lange, bis der Roboter wieder antwortet. Das Mitlesen fragt mit 2 s Frist, damit ein hängender Abruf es nicht anhält.
  - Gemessen am 03.11.: Nach „Robi, folge mir“ fiel das Mikrofon beim Sprechen ab („Stream hängt“ um 09:22:51), 3 s später hörte die Erkennung wieder. Der Auslöser ist die Versorgung: Spricht er, zieht der Lautsprecher Strom, und das Mikrofon meldet sich teils acht Mal in acht Sekunden ab und an.
- **Ausführung:** Der Server holt die Befehle alle 0,1 s ab und nutzt dieselben Wege wie die App (`_handle`). Wird ein Befehl abgelehnt (z. B. Kamera fehlt), kommt die Warnung in der App, und er sagt „Das geht gerade nicht.“.

| Sagen (Beispiele) | Befehl | Wirkung |
|---|---|---|
| „Robi, folgen“, „Robi, folge mir“, „Robi, komm mit“ | `folgen` | wie `follow`: Interaktion an, er folgt der nächsten Person |
| „Robi, komm her“, „Robi, komm zu mir“ | `herkommen` | Interaktion an (falls aus), fährt auf die Person zu |
| „Robi, geh weg“, „Robi, zurück“ | `weggehen` | Interaktion an (falls aus), weicht zurück |
| „Stopp“, „Halt“, „Bleib stehen“ (ohne Robi) | `stopp` | Interaktion an: anhalten, weiter zuschauen („Okay, ich halte an.“); sonst Not-Stopp wie `stop` |
| „Robi, fahr nach Hause“, „Robi, heim“ | `nach_hause` | wie `home` |
| „Robi, erkunde“, „Robi, schau dich um“ | `erkunden` | wie `explore` |
| „Robi, Musik aus“, „Robi, Ruhe“ | `musik_aus` | MUKKE stoppen |
| „La da da“, „La la la“ (ohne Robi) | `ladada` | spielt das Lied aus `SPRACHE_LADADA_URL` über MUKKE; läuft es schon, passiert nichts |
| „Robi, Beatbox“ | `beatbox` | spielt das Lied aus `SPRACHE_BEATBOX_URL` über MUKKE; läuft es schon, passiert nichts |
| „Hallo Robi“, „Robi, wach auf“ | `start` | Interaktion an (ohne Fahren); ist sie schon an: „Ja? Ich bin da.“ |
| „Robi, gute Nacht“, „Robi, Feierabend“ | `aus` | Heimfahrt/Erkunder abbrechen, Interaktion aus, „Gute Nacht.“ |
| „Robi, fahr zwei Meter vor“, „Robi, einen halben Meter nach links“, „Robi, fahr fünfzig Zentimeter zurück“ | `manoever` | genau so weit fahren, das Lidar misst (wie `move`, siehe [`api.md`](api.md)) |
| „Robi, dreh dich um neunzig Grad nach rechts“, „Robi, dreh dich um“ (180°), „Robi, dreh dich nach links“ (90°) | `manoever` | genau um den Winkel drehen (wie `turn`); ohne Seite links herum |

Genaue Fahrbefehle tragen eine Zahl und stehen deshalb nicht in `BEFEHLE`,
sondern werden von `manoever_erkennen()` gelesen. Der Satz muss ganz passen
(„fahr zwei Meter vor bitte“ gilt nicht), damit Wortsalat nicht fährt. Vosk
erkennt nur die ausgeschriebenen Zahlen aus `_manoever_saetze()` (1–5 Meter,
halber und anderthalb Meter, 20/50 Zentimeter; 30–360 Grad). Getippt
(Feld „Hören“, `speech`) geht jede Zahl, auch „fahr 1,5 m vor“. „Robi, fahr
zwei Meter zurück“ ist ein Fahrbefehl, „Robi, fahr zurück“ bleibt Weggehen.
Klappt ein Fahrbefehl nicht, sagt er den Grund („Zum Drehen ist es mir hier
zu eng“, „Da ist etwas im Weg, ich habe angehalten“, „Fast geschafft, ich
liege 2 Grad daneben“). Drehen braucht ringsum 30 cm Platz
(`MANOEVER_DREH_FREI_M`).

Die Wendungen stehen in `sprache.py` (`BEFEHLE`); die Reihenfolge dort ist
die Rangfolge. „Musik stopp“ steht deshalb vor „Stopp“.

**Ohne Mikrofon ausprobieren:** In der App das Feld „Hören“ (Befehl
`speech {text}`) nimmt Text, als wäre er ins Mikrofon gesprochen worden. Das
geht auch in `--mock`.

**Fahren per Sprache braucht keine offene App.** Das ist gewollt (wie ein
Zuruf). Die Totmannschaltung des Watchdogs gilt für Handsteuerung; die
Verhalten fahren mit eigenem Takt und Lidar-Stopp, wie beim Erkunder.

**Mikrofon und Lautsprecher:** MUKKE (mpv) und Stimme (aplay) nutzen
denselben Lautsprecher, und das ALSA-Gerät lässt sich nur einmal öffnen.
Spricht der Roboter, beendet MUKKE mpv kurz und spielt nach dem letzten Satz
an derselben Stelle weiter (Pause allein gibt das Gerät nicht frei). Das
Mikrofon hört die Musik mit; „Robi, Musik aus“ hilft.

- **Freier Text** geht später nur nach dem Weckwort an `denken.py`.
- **Ausgabe (umgesetzt):** über den USB-Lautsprecher mit **Piper** (offline, weibliche Stimme `de_DE-kerstin-low`, nur auf einem CPU-Kern). Fehlt Piper, spricht espeak-ng, fehlt auch das, bleibt es beim Log. Der zuletzt gesagte Satz steht in `state.stimme` und in der App. Einrichtung: [`hardware.md`](hardware.md).
  - Er sagt kurz, was er tut: „Ich folge dir.“, „Ich komme.“, „Da bin ich.“, „Wo bist du?“, „Okay, ich halte an.“ und beim Not-Stopp „Halt.“. Die Sätze stehen in `verhalten.py` (`_GESTE_SAGEN` und die Aufrufe von `_wechseln(..., sagen=…)`).
  - **Sanfter Start:** Beim Einschalten läuft erst die Posenerkennung `INTERAKTION_ANLAUF_S` (5 s) warm, still und im Stand. Dann kommt der Startsatz („Jetzt folge ich dir.“), und gefahren wird erst, wenn er zu Ende ist (`Stimme.beschaeftigt`). Wünsche in dieser Zeit (Sprache, App) merkt sich die Interaktion (`_start_modus`) und führt sie danach aus. Grund: Am 03.11. starteten Pose, Ansage und Motoren in derselben Sekunde, die USB-Versorgung brach zusammen, und der Pi startete neu.
  - Beim Start sagt er „Ich bin bereit.“. So hört man sofort, ob der Lautsprecher geht.
  - Aus der App: Feld „Sagen“ im Interaktionsbereich (Befehl `say`).
  - Gleiche Sätze innerhalb von `STIMME_WIEDERHOLUNG_S` werden verschluckt. Die Schlange hält höchstens `STIMME_MAX_WARTESCHLANGE` Sätze, das Älteste fällt weg. Stopp hat Vorrang und schneidet den laufenden Satz ab.

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

- Befehle `interaction_start` / `interaction_stop`, `follow {ziel_id?}` / `follow_pause` / `follow_resume` / `follow_stop`.
- `state.interaktion` mit `aktiv`, `verfuegbar`, `modus`, `text`, `person{winkel, abstand}`, `personen`, `geste`, `grund`, `pausiert`, `ziel{zustand, id, sicherheit, grund}`.
- `GET /interaktion/status` und `GET /interaktion/pose` (Skelett zum Einblenden; die Web-App zeichnet es über das Kamerabild).
- Stimme: Befehle `say {text, vorrang}` und `say_stop`, `state.stimme` mit `verfuegbar`, `spricht`, `text`, Diagnose unter `GET /stimme/status`.
- Sprache: Befehl `speech {text}`, `state.sprache` mit `verfuegbar`, `geweckt`, `gehoert`, `befehl`, Diagnose unter `GET /sprache/status`.

Für Stufe 4/5 geplant (nur ergänzen): Befehl `chat {text}`, Felder `stimmung`, `aufmerksamkeit`, `gedanke`, `letzte_antwort`, Nachricht `type: "gedanke"`.

## Stufen

| Stufe | Inhalt | Stand |
|---|---|---|
| 1 | Doku (diese Datei, Projektdoku, Entscheidungen E5/E6) | erledigt |
| 2 | Wahrnehmung: `pose.py`, `gesten.py`, Skelett in der App, **ohne Fahren** | erledigt (Mock + Tests), Hailo am Pi zu prüfen |
| 3 | Verhalten: Folgen/Stopp, Herkommen/Weggehen, **nur am Boden** | erledigt (Mock), Fahrtest am Boden offen |
| 4 | Sprache: Vosk-Befehle, Stimme (Piper), Chat in der App | Stimme und Sprachbefehle („Robi …“) erledigt (Mock + Tests), am Pi zu prüfen; Chat offen |
| 5 | Geist und Denken: Zustand, Gedankenstrom, Claude/Vorlagen, Ausdruck | offen |
| 6 | Später: lokales LLM auf dem Hailo, Gesichter, Orbbec-Tiefe | – |

## Testen

- Die Gestenlogik wird ohne Hardware geprüft, mit synthetischen Keypoint-Folgen pro Geste plus Negativfällen: `python Marlon/interaktion/test_gesten.py`. Das Muster ist dasselbe wie in `test_hindernisse.py`.
- Die Sprachausgabe (Warteschlange, Vorrang, Ausweichen auf Text, Sätze des Verhaltens) prüft `python Marlon/interaktion/test_stimme.py`, ebenfalls ohne Lautsprecher.
- Die Sprachbefehle (Weckwort, Weckfenster, Rangfolge, Grammatik gegen den Wortschatz des Modells, kein Absturz ohne Vosk) prüft `python Marlon/interaktion/test_sprache.py`, ohne Mikrofon und ohne Vosk.
- `python main.py --mock` liefert eine simulierte Person, die nacheinander Herkommen, Weggehen, Folgen und Stopp vormacht. In der App „Interaktion an“ drücken und die Meldungen ansehen.
- Am Pi:
  - `/interaktion/status` zeigt, ob der Hailo verfügbar ist, und die Pose-Rate.
  - Dabei auf CPU-Last und `dmesg` achten, damit keine USB-Resets auftreten (siehe [`hardware.md`](hardware.md)).
