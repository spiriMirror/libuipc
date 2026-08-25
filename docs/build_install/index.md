# Build & Install

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

## Specific Build Instructions

- Libuipc:
    - [Windows](./windows.md)
    - [Linux](./linux.md)
    - [Docker](./docker.md)
    - [XMake](./xmake.md)
- Libuipc Documentation:
    - [Build Document](./build_docs.md)

## PyPI 

> **Note:** The PyPI wheel is currently under development and may have problems. Use with caution.

Limitations:

- Only supports Windows/Ubuntu22.04
- The prebuilt wheels target the **CUDA 12.8 runtime** and dynamically load
  `cublas64_12.dll` on Windows (the corresponding CUDA 12 libraries on
  Linux). A CUDA 13-only toolkit is not a substitute: driver compatibility
  does not provide a differently versioned cuBLAS runtime. Install CUDA 12.8
  side-by-side or build libuipc from source against CUDA 13.
- Source builds work with newer toolkits too (CUDA 13.x is used in daily
  development), and newer CUDA code is kept compatible with older toolkits
  where feasible.
- The release pipeline supports CPython 3.10-3.14. Release 0.0.26 contains only
  3.10-3.13 wheels; use the next release or build from source on Python 3.14.
- New wheels contain native CUDA code for compute capabilities 7.5, 8.0, 8.6,
  and 8.9, plus compute-8.9 PTX for forward JIT on newer GPUs. Older wheels
  compiled only for 8.9 and can fail with `no kernel image is available` on an
  Ampere GPU even when the CUDA runtime is installed correctly.

```bash
pip install pyuipc
```

`import uipc` verifies only the Python extension and core libraries. Verify
the actual simulation backend as well:

```python
from tempfile import TemporaryDirectory
from uipc.core import Engine

with TemporaryDirectory(prefix="uipc-wheel-check-") as workspace:
    engine = Engine("cuda", workspace)

print("CUDA backend loaded")
```

On Windows, `Could not load ... uipc_backend_cuda.dll` together with a missing
`cublas64_12.dll` means the CUDA 12.8 runtime is not discoverable on `PATH`.
Installing only CUDA 13 does not satisfy that DLL dependency.

The next release and current source tree provide a compatibility doctor. The
first command performs non-invasive Python, native ABI, backend-library, CUDA
runtime, driver, and GPU-architecture checks. The second also creates a CUDA
engine and therefore initializes the GPU:

```bash
python -m uipc doctor
python -m uipc doctor --probe-cuda
python -m uipc doctor --json
```

The human report distinguishes three commonly conflated failures:

- no wheel for the active Python ABI (for example Python 3.14 with 0.0.26);
- no CUDA 12 cuBLAS runtime (`cublas64_12.dll` / `libcublas.so.12`);
- no SASS/PTX image compatible with the GPU compute capability.

## Development Build

- [Development in UV](./dev_in_uv.md)
