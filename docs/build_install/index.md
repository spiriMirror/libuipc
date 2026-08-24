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
- Only supports Python 3.10-3.13

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

## Development Build

- [Development in UV](./dev_in_uv.md)
