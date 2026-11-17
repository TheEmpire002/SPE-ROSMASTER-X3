"""Tile layout of the page, derived from the topology."""

from dataclasses import dataclass, field

from . import config
from .topology import Topology


@dataclass
class Tile:
    id: str
    module: str  # "pi" | "board" | "motor"
    label: str
    lanes: list[str]  # port paths
    kind: str  # "socket" | "hubport" | "mcu" | "motor"


@dataclass
class Layout:
    tiles: list[Tile] = field(default_factory=list)
    board_hub: str | None = None
    board_port: str | None = None
    board_socket: str | None = None
    board_present: bool = False


def _socket_tiles() -> list[Tile]:
    return [Tile(s["id"], "pi", s["label"], list(s["lanes"]), "socket") for s in config.PI_SOCKETS]


def build_layout(topo: Topology, previous: Layout | None) -> Layout:
    tiles = _socket_tiles()
    board = next((d for d in topo.devices.values() if (d.vid, d.pid) == config.BOARD_ID), None)
    hub = board.path.rpartition(".")[0] if board else ""
    if not hub:
        if previous is None or previous.board_port is None:
            return Layout(tiles=tiles)
        # board left sysfs: keep its modules so the page can show it missing
        tiles += [t for t in previous.tiles if t.module != "pi"]
        return Layout(tiles, previous.board_hub, previous.board_port, previous.board_socket, False)

    top = board.path.split(".")[0]
    socket = next((s["id"] for s in config.PI_SOCKETS if top in s["lanes"]), None)
    for port in sorted((p for p in topo.ports.values() if p.hub == hub), key=lambda p: p.number):
        lanes = [port.path] + ([port.peer] if port.peer else [])
        if port.path == board.path:
            label, kind = f"Port {port.number} · {config.BOARD_MCU_SUFFIX}", "mcu"
        else:
            label, kind = f"Port {port.number}", "hubport"
        tiles.append(Tile(f"board-p{port.number}", "board", label, lanes, kind))
    tiles.append(Tile("motor", "motor", config.MOTOR_TILE_LABEL, [board.path], "motor"))
    return Layout(tiles, hub, board.path, socket, True)


def tile_for_port(layout: Layout, port: str) -> str | None:
    lane_tile: dict[str, str] = {}
    for tile in layout.tiles:
        if tile.kind == "motor":
            continue  # shares the board lane, the hub port tile owns it
        for lane in tile.lanes:
            lane_tile.setdefault(lane, tile.id)
    path = port
    while path:
        if path in lane_tile:
            return lane_tile[path]
        path = path.rpartition(".")[0]
    return None
