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
- The prebuilt wheels target CUDA 12.8; building from source works with newer toolkits too (13.x is used in daily development), and newer CUDA code is kept compatible back to CUDA 11.x where feasible (e.g. the CUDA-graph helper in `cuda_tool/graph.h` selects its instantiation API by `CUDART_VERSION`)
- Only supports Python 3.10-3.13

```bash
pip install pyuipc
```

## Development Build

- [Development in UV](./dev_in_uv.md)