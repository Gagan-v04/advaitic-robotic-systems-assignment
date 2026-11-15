"""EGL setup for MuJoCo.

On a real Linux NVIDIA box (cloud), leave GALLIUM_DRIVER alone so libEGL_nvidia
and PLATFORM_DEVICE work. On WSL2 there is no NVIDIA kernel module and no
libEGL_nvidia that can eglInitialize; the Windows driver only stubs CUDA/DX12
at /usr/lib/wsl/lib. Headless GPU pixels there go through Mesa d3d12, which
reports GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5050 Laptop GPU) and shows
GPU-Util / memory in nvidia-smi during mj_render.
"""

from __future__ import annotations

import os


def _is_wsl() -> bool:
    return os.path.exists("/dev/dxg") or os.path.exists("/usr/lib/wsl/lib/libdxcore.so")


def prepare_egl() -> None:
    if os.environ.get("MUJOCO_GL", "egl").lower() != "egl":
        return
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    if _is_wsl():
        os.environ.setdefault("GALLIUM_DRIVER", "d3d12")
        os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
        wsl_lib = "/usr/lib/wsl/lib"
        current = os.environ.get("LD_LIBRARY_PATH", "")
        if wsl_lib not in current.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{wsl_lib}:{current}" if current else wsl_lib
        mesa_icd = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
        if os.path.isfile(mesa_icd):
            os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", mesa_icd)
    try:
        import mujoco.egl as egl_mod
        from mujoco.egl import egl_ext as EGL
    except Exception:
        return
    original = egl_mod.create_initialized_egl_device_display

    def _display():
        display = original()
        if display != EGL.EGL_NO_DISPLAY:
            print("EGL PLATFORM_DEVICE display", flush=True)
            return display
        fallback = EGL.eglGetDisplay(EGL.EGL_DEFAULT_DISPLAY)
        print(f"EGL DEFAULT_DISPLAY={fallback}", flush=True)
        if fallback == EGL.EGL_NO_DISPLAY:
            return fallback
        try:
            initialized = EGL.eglInitialize(fallback, None, None)
        except Exception as exc:
            print(f"EGL initialize failed: {exc}", flush=True)
            return EGL.EGL_NO_DISPLAY
        if initialized == EGL.EGL_TRUE:
            print("EGL DEFAULT_DISPLAY fallback (Mesa/WSL D3D12)", flush=True)
            return fallback
        print("EGL initialize returned false", flush=True)
        return EGL.EGL_NO_DISPLAY

    egl_mod.create_initialized_egl_device_display = _display
