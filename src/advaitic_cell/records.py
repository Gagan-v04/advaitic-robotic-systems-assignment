"""JSONL evidence plus the side files the checker does not read."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def clean(value):
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


class EventLog:
    def __init__(self) -> None:
        self.events: list[dict] = []

    def add(self, event: dict) -> None:
        self.events.append(clean(event))

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = [json.dumps(event, separators=(",", ":")) for event in self.events]
        path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(clean(payload), indent=2), encoding="utf-8")
