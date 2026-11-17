"""Fake sysfs for tests.

Links are plain files holding the target path, and interface dirs use "!" instead
of ":" (windows forbids it in names).
"""

import pathlib
import re

_PORT_RE = re.compile(r"(\S+)-port(\d+)")
_ROOT_RE = re.compile(r"usb(\d+)")


def _write(path: pathlib.Path, text) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(str(text), encoding="utf-8")


def write_fake_sysfs(tree: dict, root: pathlib.Path) -> None:
    root = pathlib.Path(root)
    root.mkdir(parents=True, exist_ok=True)
    for path, attrs in tree.get("devices", {}).items():
        (root / path).mkdir(parents=True, exist_ok=True)
        for key, value in attrs.items():
            if key != "interfaces":
                _write(root / path / key, value)
        for iface, info in attrs.get("interfaces", {}).items():
            idir = root / f"{path}!{iface}"
            idir.mkdir(parents=True, exist_ok=True)
            if info.get("driver"):
                _write(idir / "driver", f"../../../bus/usb/drivers/{info['driver']}")
            if info.get("tty"):
                (idir / info["tty"]).mkdir(exist_ok=True)
    for name, info in tree.get("ports", {}).items():
        owner = _PORT_RE.fullmatch(name).group(1)
        root_m = _ROOT_RE.fullmatch(owner)
        if root_m:
            pdir = root / owner / f"{root_m.group(1)}-0!1.0" / name
        else:
            pdir = root / f"{owner}!1.0" / name
        pdir.mkdir(parents=True, exist_ok=True)
        _write(pdir / "state", info.get("state", ""))
        _write(pdir / "over_current_count", info.get("over_current_count", 0))
        if info.get("peer"):
            _write(pdir / "peer", f"../../../{info['peer']}")
        if info.get("device"):
            _write(pdir / "device", "../../../child")
