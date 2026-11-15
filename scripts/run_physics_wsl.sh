#!/usr/bin/env bash
# Physics-only cell run on Ubuntu-24.04 WSL (OSMesa). No Docker.
set -euo pipefail
WORK="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:?out dir}"
SCENARIO="${2:-human_stop}"
SEED="${3:-0}"
mkdir -p "$OUT"
export PYTHONPATH="${WORK}/src"
export PYTHONUNBUFFERED=1
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa
cd "$WORK"
exec /opt/ars-venv/bin/python -m advaitic_cell.cli \
  --scenario "$SCENARIO" --seed "$SEED" --headless --no-record --out "$OUT"
