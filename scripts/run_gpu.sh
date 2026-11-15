#!/usr/bin/env bash
# Ubuntu-24.04 WSL2 runtime for this cell. Not Docker Desktop.
#
# Recording / video (RTX EGL d3d12):
#   powershell -File scripts/run_gpu.ps1
#   powershell -File scripts/run_gpu.ps1 cell-run --scenario human_stop --seed 0 --headless --video results/demo/full-mission.mp4
#   bash scripts/run_gpu.sh cell-run --scenario human_stop --seed 0 --headless --video results/demo/full-mission.mp4
#
# Physics-only (OSMesa, no video; faster systems-check):
#   powershell -File scripts/run_gpu.ps1 physics pytest -q
#   powershell -File scripts/run_gpu.ps1 physics ruff check .
#   powershell -File scripts/run_gpu.ps1 physics cell-run --scenario human_stop --seed 0 --headless --no-record --out results/demo/check-hstop-wsl
#   bash scripts/run_gpu.sh physics cell-run --scenario nominal --seed 0 --headless --no-record --out results/demo/check-nominal-wsl
#
# Renderer: MUJOCO_GL=egl + Mesa d3d12 -> GL_RENDERER="D3D12 (NVIDIA GeForce RTX 5050 Laptop GPU)"
# Proven 2026-11-11: 1137 frames / 18 s = 63 fps at 1920x1080; nvidia-smi GPU-Util 25-29%
# and memory.used 1621 -> 1834 MiB during the loop. No live GUI/desktop required.
#
# Why not libEGL_nvidia (cloud path): WSL has /dev/dxg only. Extracted NVIDIA 590.48
# userspace (libEGL_nvidia.so.0 + 10_nvidia.json) fails eglInitialize / PLATFORM_DEVICE.
# NVIDIA's CUDA-on-WSL guide forbids installing a Linux nvidia kernel driver.
# Host Windows mujoco.dll is WDAC-blocked; never rely on Windows Python for physics.

set -euo pipefail
WORK="$(cd "$(dirname "$0")/.." && pwd)"
PHYSICS=0
if [ "${1:-}" = "physics" ]; then
  PHYSICS=1
  shift
fi

export PYTHONPATH="${WORK}/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export PATH="/opt/ars-venv/bin:/usr/lib/wsl/lib:${PATH}"
export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$WORK"

if [ "$PHYSICS" -eq 1 ] || [ "${MUJOCO_GL:-osmesa}" = "osmesa" ]; then
  export MUJOCO_GL=osmesa
  unset PYOPENGL_PLATFORM || true
  unset GALLIUM_DRIVER || true
  unset MESA_D3D12_DEFAULT_ADAPTER_NAME || true
  unset __EGL_VENDOR_LIBRARY_FILENAMES || true
  echo "==== High-Speed Render + RTX 5050 NVENC Video ===="
  echo "WORK=$WORK MUJOCO_GL=$MUJOCO_GL"
  /usr/lib/wsl/lib/nvidia-smi --query-gpu=name,driver_version,utilization.gpu,memory.used,memory.total --format=csv || true
else
  export MUJOCO_GL=egl
  export PYOPENGL_PLATFORM=egl
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
  echo "==== GPU launch env ===="
  echo "WORK=$WORK MUJOCO_GL=$MUJOCO_GL GALLIUM_DRIVER=$GALLIUM_DRIVER"
  /usr/lib/wsl/lib/nvidia-smi --query-gpu=name,driver_version,utilization.gpu,memory.used,memory.total --format=csv || true
fi

if [ $# -eq 0 ]; then
  if [ "$PHYSICS" -eq 1 ]; then
    echo "usage: $0 physics <pytest|ruff|cell-run|systems-check|...> [args]" >&2
    exit 2
  fi
  exec /opt/ars-venv/bin/python "$WORK/scripts/probe_gpu_render.py"
fi

cmd="$1"
shift
case "$cmd" in
  cell-run)
    exec /opt/ars-venv/bin/python -m advaitic_cell.cli "$@"
    ;;
  cell-eval)
    exec /opt/ars-venv/bin/python -m advaitic_cell.evaluate "$@"
    ;;
  systems-check)
    exec /opt/ars-venv/bin/python -m advaitic_systems.evidence "$@"
    ;;
  systems-timing)
    exec /opt/ars-venv/bin/python -m advaitic_systems.timing "$@"
    ;;
  pytest)
    exec /opt/ars-venv/bin/pytest "$@"
    ;;
  ruff)
    exec /opt/ars-venv/bin/ruff "$@"
    ;;
  *)
    exec "$cmd" "$@"
    ;;
esac
