# Mapping-Weboberfläche integrieren

Diese Ergänzung überschreibt keine bestehende `api.py`.

## 1. Dateien kopieren

Den Inhalt dieses Pakets in das Projektverzeichnis entpacken. Es ergänzt:

- `robot/rosmaster_assist/mapping/web.py`
- `robot/rosmaster_assist/templates/map.html`

Die vier bereits gelieferten Dateien im Ordner `mapping/` bleiben bestehen.

## 2. In `api.py` die Funktion `create_app(...)` erweitern

In `robot/rosmaster_assist/api.py` die folgenden Imports ergänzen (oben bei den anderen Imports oder innerhalb von `create_app`):

```python
from .mapping.manager import MappingManager
from .mapping.web import create_mapping_router
```

Innerhalb der Funktion `create_app(driver, use_mock_camera=...)`:

1. Suche die Stelle, an der die FastAPI-Instanz erstellt wird, z. B. `app = FastAPI(...)`.
2. Direkt danach – und unbedingt vor `return app` – diesen Block einsetzen:

```python
    mapping_manager = MappingManager()
    app.state.mapping_manager = mapping_manager
    app.include_router(create_mapping_router(mapping_manager))

    def _stop_mapping_on_shutdown() -> None:
        try:
            mapping_manager.stop()
        except Exception:
            # Beispielsweise, wenn Docker beim Herunterfahren bereits beendet ist.
            logging.getLogger(__name__).exception(
                "Mapping konnte beim Herunterfahren nicht sauber gestoppt werden"
            )

    app.add_event_handler("shutdown", _stop_mapping_on_shutdown)
```

Die Einrückung muss zum Funktionsrumpf von `create_app` passen. `logging` muss in `api.py` importiert sein; falls nicht, oben `import logging` ergänzen.

**Die zu erweiternde Funktion heißt `create_app()` in `api.py`.** Nicht `main()` in `main.py` ändern. Der Router registriert `/map`, die API-Endpunkte und den WebSocket.

## 3. URLs

- `GET /map` – Mapping-Oberfläche
- `GET /api/mapping/status` – Status
- `POST /api/mapping/start` – SLAM starten
- `POST /api/mapping/stop` – SLAM stoppen
- `GET /api/mapping/map` – letzte Karte als JSON
- `POST /api/mapping/save` – letzte Karte als PGM + YAML speichern
- `GET /api/mapping/maps` – gespeicherte Karten auflisten
- `WS /api/mapping/ws` – Status und Live-Kartenupdates

## 4. Konfiguration

Der Manager nutzt standardmäßig:

- Docker-Container: `x3`
- ROS-Setup: `/opt/ros/foxy/setup.bash`
- SLAM-Paket: `slam_toolbox`
- Launch-Datei: `online_async_launch.py`
- Kartentopic: `/map`

Diese Werte lassen sich vor dem Start der Web-App über Umgebungsvariablen überschreiben:

```bash
export ROSMAP_CONTAINER=x3
export ROSMAP_SLAM_PACKAGE=slam_toolbox
export ROSMAP_SLAM_LAUNCH=online_async_launch.py
export ROSMAP_TOPIC=/map
```

Wichtig: Der Standard-Launch funktioniert nur, wenn im Container `slam_toolbox` vorhanden ist und ROS 2 die benötigten Sensordaten/TF-Frames bereitstellt (typisch `/scan` sowie `odom` → `base_link`). Das muss auf dem echten Roboter getestet werden.
