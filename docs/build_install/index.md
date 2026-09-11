# Build & Install

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

## Specific Build Instructions

- Libuipc:
    - [Windows](./windows.md)
    - [Linux](./linux.md)
    - [Docker](./docker.md)
    - [XMake](./xmake.md)
    - [Blender Extension](./blender.md)
- Libuipc Documentation:
    - [Build Document](./build_docs.md)

## PyPI 

> **Note:** The PyPI wheel is currently under development and may have problems. Use with caution.

Limitations:

- Only supports Windows/Ubuntu22.04
- Wheels built from the current source use CUDA 12.8 at build time but carry
  the runtime code they need. Users do not need a local CUDA Toolkit; they need
  a compatible NVIDIA driver. For GPUs served by packaged SASS, the floor is
  >=525.60.13 on Linux or >=528.33 on Windows. A CUDA 13.x driver is backward
  compatible with the CUDA 12.x application binary; see
  NVIDIA's [minor-version compatibility table](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html).
  These are the CUDA 12.x minor-compatibility floors for packaged SASS.
  Hardware that uses the forward-JIT CUDA 12.8 PTX image instead requires
  driver >=570.124.06 on Linux or >=572.61 on Windows.
- The immutable 0.0.27 wheel predates that change and still dynamically loads
  `cublas64_12.dll` on Windows (and the corresponding CUDA 12 library on
  Linux). Install CUDA 12.8 side-by-side for 0.0.27, build from current source,
  or use the next wheel release.
- Source builds work with newer toolkits too (CUDA 13.x is used in daily
  development), and newer CUDA code is kept compatible with older toolkits
  where feasible.
- The release pipeline and release 0.0.27 support CPython 3.10-3.14.
- New wheels contain native CUDA code for compute capabilities 7.5, 8.0, 8.6,
  8.9, and 12.0, plus compute-8.9 PTX for forward JIT on other newer GPUs.
  `python -m uipc doctor` reports which code path applies and checks its driver
  floor. Older wheels compiled only for 8.9 and can fail with
  `no kernel image is available` on an Ampere GPU even when the CUDA runtime is
  installed correctly.

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

For the immutable 0.0.27 Windows wheel, `Could not load ...
uipc_backend_cuda.dll` together with a missing `cublas64_12.dll` means the CUDA
12.8 runtime is not discoverable on `PATH`. Current-source and future wheels do
not have that dependency; use `python -m uipc doctor --probe-cuda` to distinguish
an old wheel from a driver or GPU-code-image failure.

Release 0.0.27 and the current source tree provide a compatibility doctor. The
first command performs non-invasive Python, native ABI, backend-library, CUDA
runtime, driver, and GPU-architecture checks. The second also creates a CUDA
engine and therefore initializes the GPU:

```bash
python -m uipc doctor
python -m uipc doctor --probe-cuda
python -m uipc doctor --json
```

The human report distinguishes commonly conflated failures:

- no wheel for the active Python ABI;
- an old wheel with a missing CUDA 12 cuBLAS runtime;
- an NVIDIA driver below the packaged compatibility floor;
- no SASS/PTX image compatible with the GPU compute capability.

## Development Build

- [Development in UV](./dev_in_uv.md)
