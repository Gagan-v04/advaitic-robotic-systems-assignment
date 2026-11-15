# Linux fallback image only. Local cell work on this Windows host runs in
# Ubuntu-24.04 WSL2 (scripts/run_gpu.ps1 / scripts/run_gpu.sh), not Docker Desktop.
# The unsigned host mujoco.dll is WDAC-blocked; do not use Windows Python for physics.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 python3-pip \
        libosmesa6 libgl1 libglib2.0-0 \
        libegl1 libgles2 libglvnd0 libopengl0 \
        libgl1-mesa-dri libglx-mesa0 mesa-utils \
        ffmpeg xvfb ca-certificates \
    && ln -sf /usr/bin/python3 /usr/bin/python \
    && rm -rf /var/lib/apt/lists/*
COPY wheelhouse /wheelhouse
RUN pip3 install --break-system-packages --no-index --find-links /wheelhouse \
    "mujoco==3.8.1" "numpy>=1.26,<3" "pytest>=8" "pillow>=10" "ruff>=0.12"
WORKDIR /work
# Leave MUJOCO_GL unset in the image. Recording sets glfw+Xvfb or egl at
# runtime; baking PYOPENGL_PLATFORM=egl breaks OSMesa and GLFW imports.
