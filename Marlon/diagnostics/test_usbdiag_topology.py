"""Tests the usbdiag sysfs topology scan and the automatic module layout.

    python Marlon/diagnostics/test_usbdiag_topology.py
"""

import json
import os
import pathlib
import shutil
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "robot" / "src"))

from rosmaster_assist.usbdiag import layout, mock, topology  # noqa: E402

FIXTURES = pathlib.Path(__file__).resolve().parents[2] / "robot/src/rosmaster_assist/usbdiag/fixtures"


def load_tree() -> dict:
    return json.loads((FIXTURES / "topology.json").read_text())


def make_tree(tmp):
    mock.write_fake_sysfs(load_tree(), tmp)
    return tmp


def rename_prefix(tree, old, new):
    def fix(name):
        # device paths, port dir names and peers all start with the hub path
        return new + name[len(old):] if name.startswith(old) else name

    ports = {}
    for name, info in tree["ports"].items():
        info = dict(info)
        if info.get("peer"):
            info["peer"] = fix(info["peer"])
        ports["usb3-port2" if name == "usb1-port1" else fix(name)] = info
    return {"devices": {fix(p): d for p, d in tree["devices"].items()}, "ports": ports}


def test_scan_real_pi_fixture():
    with tempfile.TemporaryDirectory() as d:
        topo = topology.scan(str(make_tree(pathlib.Path(d))))
    assert topo.ports["1-1"].over_current == 54 and topo.ports["1-1"].peer == "2-1"
    assert topo.ports["1-1.4"].device.vid == "1a86" and topo.ports["1-1.4"].device.tty == "ttyUSB1"
    assert topo.ports["1-1.4"].device.driver == "ch341" and topo.ports["1-1.4"].device.max_power_ma == 104
    assert topo.ports["1-1.4"].device.devnum == 114 and topo.ports["1-1.4"].device.busnum == 1
    assert topo.ports["1-1.2"].device.devnum == 113
    assert topo.ports["1-1.1"].device is None and topo.ports["1-1.1"].state == "not attached"
    assert topo.ports["3-1.1"].device.product == "ORBBEC Depth Sensor"
    assert topo.ports["1-1.2"].peer == "2-1.2" and topo.ports["1-1"].hub == "usb1"


def test_scan_survives_vanishing_files():
    with tempfile.TemporaryDirectory() as d:
        root = make_tree(pathlib.Path(d))
        (root / "1-1.4" / "idVendor").unlink()
        shutil.rmtree(root / "3-1.2")
        topo = topology.scan(str(root))
        assert "1-1.2" in topo.devices
        assert topo.ports["3-1.2"].device is None
        assert topology.scan(str(root / "missing")).ports == {}


def test_scan_survives_dir_vanishing_between_listdir_and_read():
    real_listdir = os.listdir
    with tempfile.TemporaryDirectory() as d:
        root = make_tree(pathlib.Path(d))

        def racy(path):
            names = real_listdir(path)
            if pathlib.Path(path) == root:
                # gone right after the root listing, before any attribute read
                shutil.rmtree(root / "1-1.2")
                shutil.rmtree(root / "1-1.2!1.0")
                shutil.rmtree(root / "1-1!1.0")
            return names

        os.listdir = racy
        try:
            topo = topology.scan(str(root))
        finally:
            os.listdir = real_listdir
    assert topo.devices["1-1.2"].vid == "" and topo.devices["1-1.2"].tty is None
    assert topo.devices["1-1.4"].vid == "1a86" and "3-1.1" in topo.devices


def test_layout_detects_board_and_modules():
    with tempfile.TemporaryDirectory() as d:
        lay = layout.build_layout(topology.scan(str(make_tree(pathlib.Path(d)))), None)
    assert (lay.board_hub, lay.board_port, lay.board_socket, lay.board_present) == ("1-1", "1-1.4", "pi-usb3-a", True)
    ids = [t.id for t in lay.tiles]
    assert ids[:4] == ["pi-usb3-a", "pi-usb3-b", "pi-usb2-a", "pi-usb2-b"]
    assert {"board-p1", "board-p2", "board-p3", "board-p4", "motor"} <= set(ids)
    assert next(t for t in lay.tiles if t.id == "board-p4").kind == "mcu"
    assert next(t for t in lay.tiles if t.id == "board-p4").label == "Port 4 · Board-MCU"
    assert next(t for t in lay.tiles if t.id == "board-p2").lanes == ["1-1.2", "2-1.2"]
    assert layout.tile_for_port(lay, "3-1.1") == "pi-usb3-b"
    assert layout.tile_for_port(lay, "1-1.4") == "board-p4"
    assert layout.tile_for_port(lay, "1-1") == "pi-usb3-a"


def test_layout_keeps_board_while_it_is_gone():
    with tempfile.TemporaryDirectory() as d:
        root = make_tree(pathlib.Path(d))
        first = layout.build_layout(topology.scan(str(root)), None)
        for p in ["1-1", "1-1.2", "1-1.4", "2-1"]:
            shutil.rmtree(root / p)
        gone = layout.build_layout(topology.scan(str(root)), first)
        again = layout.build_layout(topology.scan(str(root)), gone)
    assert gone.board_present is False and gone.board_hub == "1-1"
    assert gone.board_port == "1-1.4" and gone.board_socket == "pi-usb3-a"
    assert "board-p4" in [t.id for t in gone.tiles] and "motor" in [t.id for t in gone.tiles]
    assert again.board_present is False and [t.id for t in again.tiles] == [t.id for t in gone.tiles]


def test_layout_without_board_has_only_sockets():
    lay = layout.build_layout(topology.Topology(), None)
    assert [t.kind for t in lay.tiles] == ["socket"] * 4 and lay.board_present is False


def test_layout_follows_board_to_other_socket():
    tree = rename_prefix(load_tree(), "1-1", "3-2")  # moves the board hub to black socket B
    with tempfile.TemporaryDirectory() as d:
        mock.write_fake_sysfs(tree, pathlib.Path(d))
        lay = layout.build_layout(topology.scan(d), None)
    assert lay.board_socket == "pi-usb2-b" and lay.board_port == "3-2.4"


def test_device_names():
    dev = topology.DeviceInfo("1-1.2", "10c4", "ea60", "CP2102", 12, 100, 5, 1, "cp210x", "ttyUSB0")
    assert topology.device_name(dev) == "Lidar" and topology.device_role(dev) == "lidar"
    assert topology.device_name(None) == "nichts"
    odd = topology.DeviceInfo("3-2", "dead", "beef", "Stick  ", 480, 100, 7, 3, None, None)
    assert topology.device_name(odd) == "„Stick“" and topology.device_role(odd) == "other"
    blank = topology.DeviceInfo("3-2", "dead", "beef", " ", 480, 100, 7, 3, None, None)
    assert topology.device_name(blank) == "Unbekanntes USB-Gerät"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("ok  ", name)
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print("bad ", name, repr(exc))
    sys.exit(1 if failed else 0)
