"""Print PC inference readiness without downloading model files or opening a camera."""
from __future__ import annotations

import importlib
import platform
import shutil
import subprocess
import sys


def main() -> int:
    print("=== Presentation Escort / PC GPU preflight ===")
    print(f"Python: {sys.version.split()[0]} ({sys.executable})")
    print(f"OS: {platform.platform()}")
    print(f"Architecture: {platform.machine()}")

    for name in ("numpy", "cv2", "torch", "ultralytics"):
        try:
            module = importlib.import_module(name)
            version = getattr(module, "__version__", "installed")
            print(f"[OK] {name}: {version}")
        except Exception as exc:
            print(f"[NG] {name}: {type(exc).__name__}: {exc}")

    has_cuda = False
    try:
        torch = importlib.import_module("torch")
        has_cuda = bool(torch.cuda.is_available())
        print(f"CUDA available: {has_cuda}")
        print(f"PyTorch CUDA runtime: {torch.version.cuda}")
        if has_cuda:
            print(f"GPU: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        print(f"CUDA check unavailable: {exc}")

    if shutil.which("nvidia-smi"):
        process = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=False,
            timeout=8,
        )
        if process.returncode == 0:
            print("NVIDIA driver: " + process.stdout.strip())
        else:
            print("nvidia-smi failed: " + process.stderr.strip())
    else:
        print("nvidia-smi: not found (normal for non-NVIDIA GPUs)")

    print("\nIMPORTANT: A working PC GPU model is NOT confirmed by this check alone.")
    if not has_cuda:
        print("NVIDIA CUDA is unavailable. Use '--device cpu' for a fallback test,")
        print("or install the matching CUDA-enabled PyTorch build for your hardware.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
