# Integrating the mapping web interface

> **As of 03.11.2026: already built in** (`api/server.py`). This guide is
> only kept for the record. Important: `/scan` and TF are now provided by our
> own server via `scan_feed.py`, see E8 in `docs/decisions.md` and the
> section "Mapping (SLAM)" in `docs/api.md`. `sllidar_node` and
> `yahboomcar_bringup` must **not** run for this.

This addition does not overwrite an existing `api.py`.

## 1. Copy the files

Unpack the contents of this package into the project directory. It adds:

- `robot/rosmaster_assist/mapping/web.py`
- `robot/rosmaster_assist/templates/map.html`

The four files already delivered in the `mapping/` folder stay in place.

## 2. Extend the function `create_app(...)` in `api.py`

In `robot/rosmaster_assist/api.py`, add the following imports (at the top with the other imports or inside `create_app`):

```python
from .mapping.manager import MappingManager
from .mapping.web import create_mapping_router
```

Inside the function `create_app(driver, use_mock_camera=...)`:

1. Find the place where the FastAPI instance is created, e.g. `app = FastAPI(...)`.
2. Directly after it – and definitely before `return app` – insert this block:

```python
    mapping_manager = MappingManager()
    app.state.mapping_manager = mapping_manager
    app.include_router(create_mapping_router(mapping_manager))

    def _stop_mapping_on_shutdown() -> None:
        try:
            mapping_manager.stop()
        except Exception:
            # For example when Docker has already been shut down.
            logging.getLogger(__name__).exception(
                "Could not stop mapping cleanly during shutdown"
            )

    app.add_event_handler("shutdown", _stop_mapping_on_shutdown)
```

The indentation must match the function body of `create_app`. `logging` must be imported in `api.py`; if it is not, add `import logging` at the top.

**The function to extend is `create_app()` in `api.py`.** Do not change `main()` in `main.py`. The router registers `/map`, the API endpoints and the WebSocket.

## 3. URLs

- `GET /map` – mapping interface
- `GET /api/mapping/status` – status
- `POST /api/mapping/start` – start SLAM
- `POST /api/mapping/stop` – stop SLAM
- `GET /api/mapping/map` – latest map as JSON
- `POST /api/mapping/save` – save the latest map as PGM + YAML
- `GET /api/mapping/maps` – list saved maps
- `WS /api/mapping/ws` – status and live map updates

## 4. Configuration

By default the manager uses:

- Docker container: `x3`
- ROS setup: `/opt/ros/foxy/setup.bash`
- SLAM package: `slam_toolbox`
- Launch file: `online_async_launch.py`
- Map topic: `/map`

These values can be overridden with environment variables before starting the web app:

```bash
export ROSMAP_CONTAINER=x3
export ROSMAP_SLAM_PACKAGE=slam_toolbox
export ROSMAP_SLAM_LAUNCH=online_async_launch.py
export ROSMAP_TOPIC=/map
```

Important: the default launch only works if `slam_toolbox` is present in the container and ROS 2 provides the required sensor data/TF frames (typically `/scan` and `odom` → `base_link`). This has to be tested on the real robot.
