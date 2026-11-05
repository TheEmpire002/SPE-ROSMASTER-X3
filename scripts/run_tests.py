#!/usr/bin/env python3
"""Runs all test scripts in parallel and reports only what goes wrong.

    python scripts/run_tests.py            # all tests
    python scripts/run_tests.py target gestures   # only files whose name contains this

Each script runs in its own process. Run one after another they took over
ten minutes because some simulations compute for a long time. In parallel
only the slowest one determines the duration.

One line is printed per script. The full output only appears on failures,
so that you (and an AI assistant) do not have to read pages of passed
checks.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PATTERNS = [
    "Marlon/autonomous_driving/test_*.py",
    "Marlon/diagnostics/test_*.py",
    "Marlon/interaction/test_*.py",
]
# A hanging test must not block the run forever.
TIME_LIMIT_S = 900


def find(filter_: list[str]) -> list[Path]:
    files = sorted({p for m in PATTERNS for p in ROOT.glob(m)})
    if filter_:
        files = [p for p in files if any(f in p.name for f in filter_)]
    return files


def execute(file: Path) -> tuple[Path, int, str, float]:
    start = time.monotonic()
    # Force UTF-8: otherwise print() fails on umlauts under Windows.
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    try:
        run = subprocess.run(
            [sys.executable, str(file)], cwd=ROOT, env=env,
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=TIME_LIMIT_S,
        )
        code, output = run.returncode, run.stdout + run.stderr
    except subprocess.TimeoutExpired:
        code, output = -1, f"Time limit {TIME_LIMIT_S} s exceeded"
    return file, code, output, time.monotonic() - start


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("filter", nargs="*", help="only test files whose name contains this")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    args = parser.parse_args()

    files = find(args.filter)
    if not files:
        print("No test files found.")
        return 1

    start = time.monotonic()
    failed = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for future in as_completed([pool.submit(execute, d) for d in files]):
            file, code, output, duration = future.result()
            name = file.relative_to(ROOT).as_posix()
            # Some scripts exit with 0 but still count FEHLER lines.
            error = sum(1 for z in output.splitlines() if "FEHLER" in z)
            last = (output.strip().splitlines() or [""])[-1]
            ok = code == 0 and error == 0
            print(f"{'ok    ' if ok else 'FEHLER'} {name} ({duration:.0f} s) :: {last}", flush=True)
            if not ok:
                failed.append((name, output))

    for name, output in failed:
        print(f"\n===== {name} =====\n{output.strip()}")
    print(f"\n{len(files) - len(failed)}/{len(files)} passed "
          f"in {time.monotonic() - start:.0f} s")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
