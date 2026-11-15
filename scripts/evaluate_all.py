"""Run the five scored missions, then both checkers.

cell-run already runs systems-check and returns non-zero on a violation.
cell-eval repeats that check and adds the plant-side assertions.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

RUNS = [
    (
        ["--scenario", "nominal", "--seed", "0", "--headless"],
        ROOT / "results" / "nominal-seed-0" / "events.jsonl",
        "nominal",
    ),
    (
        ["--scenario", "nominal", "--seed", "1", "--headless"],
        ROOT / "results" / "nominal-seed-1" / "events.jsonl",
        "nominal",
    ),
    (
        ["--scenario", "nominal", "--seed", "2", "--headless"],
        ROOT / "results" / "nominal-seed-2" / "events.jsonl",
        "nominal",
    ),
    (
        [
            "--scenario", "human_stop", "--seed", "0", "--headless",
            "--duration", "3", "--interrupt-at", "transfer",
        ],
        ROOT / "results" / "human-stop" / "events.jsonl",
        "human_stop",
    ),
    (
        ["--scenario", "plc_retry", "--seed", "0", "--headless"],
        ROOT / "results" / "plc-retry" / "events.jsonl",
        "plc_retry",
    ),
]


def _run(args: list[str]) -> None:
    print("+", " ".join(args), flush=True)
    subprocess.run(args, check=True, cwd=ROOT)


def main() -> None:
    py = sys.executable
    for run_args, events, scenario in RUNS:
        _run([py, "-m", "advaitic_cell.cli", *run_args])
        _run([py, "-m", "advaitic_cell.evaluate", str(events), "--scenario", scenario])


if __name__ == "__main__":
    main()
