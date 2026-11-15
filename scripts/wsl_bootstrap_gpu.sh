#!/usr/bin/env bash
# Bootstrap Ubuntu-24.04 WSL for headless MuJoCo EGL on the Windows NVIDIA driver.
# Does NOT install a Linux NVIDIA kernel driver (forbidden on WSL).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
echo "=== distro ==="
cat /etc/os-release | head -5
uname -a
echo "=== wsl nvidia stub ==="
ls -la /usr/lib/wsl/lib | head
/usr/lib/wsl/lib/nvidia-smi --query-gpu=name,driver_version,memory.total,utilization.gpu,memory.used --format=csv || true
echo "=== devices ==="
ls -la /dev/dxg /dev/nvidia* /dev/dri 2>/dev/null || true
echo "=== apt packages ==="
apt-get update -qq
apt-get install -y --no-install-recommends \
  python3 python3-pip python3-venv \
  libegl1 libgles2 libglvnd0 libopengl0 libgl1 \
  libgl1-mesa-dri mesa-utils mesa-utils-bin \
  libosmesa6 ffmpeg ca-certificates xz-utils \
  pkg-config
echo "=== extract NVIDIA 590.48 EGL userspace (no kernel install) ==="
NVIDIA_RUN="/mnt/c/Users/Gagan/WSL/cache/NVIDIA-Linux-x86_64-590.48.01-no-compat32.run"
DEST=/opt/nvidia-egl
rm -rf /tmp/nvidia-extract
mkdir -p /tmp/nvidia-extract "$DEST"
chmod +x "$NVIDIA_RUN"
sh "$NVIDIA_RUN" --extract-only --target /tmp/nvidia-extract
cd /tmp/nvidia-extract
for f in libEGL_nvidia.so* libGLX_nvidia.so* libnvidia-eglcore.so* libnvidia-glcore.so* \
         libnvidia-glsi.so* libnvidia-tls.so* libnvidia-rtcore.so* libnvidia-gpucomp.so* \
         10_nvidia.json; do
  if compgen -G "$f" > /dev/null; then
    cp -a $f "$DEST/"
  fi
done
cd "$DEST"
# soname links
[ -e libEGL_nvidia.so.0 ] || ln -sf libEGL_nvidia.so.590.48.01 libEGL_nvidia.so.0 2>/dev/null || \
  ln -sf "$(ls libEGL_nvidia.so.* | head -1)" libEGL_nvidia.so.0
[ -e libGLX_nvidia.so.0 ] || ln -sf "$(ls libGLX_nvidia.so.* 2>/dev/null | head -1)" libGLX_nvidia.so.0 || true
mkdir -p /usr/share/glvnd/egl_vendor.d
if [ -f 10_nvidia.json ]; then
  cp 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
else
  printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
    > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
echo "=== extracted ==="
ls -la "$DEST" | head -40
echo "=== ICD ==="
cat /usr/share/glvnd/egl_vendor.d/10_nvidia.json
echo "=== python venv + mujoco ==="
python3 -m venv /opt/ars-venv
/opt/ars-venv/bin/pip install -U pip
/opt/ars-venv/bin/pip install "mujoco==3.8.1" "numpy>=1.26,<3" pillow pyopengl
echo "bootstrap ok"
