# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LibUIPC is a cross-platform C++20 library implementing Unified Incremental Potential Contact for GPU-accelerated physics simulation. It simulates rigid bodies, soft bodies, cloth, and threads with penetration-free frictional contact. Both C++ and Python APIs are provided.

## Build Commands

### Prerequisites
- CMake >= 3.26
- Python >= 3.11
- CUDA >= 12.4
- Vcpkg with `CMAKE_TOOLCHAIN_FILE` environment variable set

### Configure and Build
```bash
# Using presets (recommended)
cmake --preset release
cmake --build --preset release -j8

# Or manually
mkdir build && cd build
cmake -S .. -DUIPC_BUILD_PYBIND=ON
cmake --build . --config Release -j8
```

### Key CMake Options
- `UIPC_BUILD_PYBIND` - Build Python bindings (OFF by default)
- `UIPC_BUILD_TESTS` - Build test suite (ON by default)
- `UIPC_BUILD_EXAMPLES` - Build examples (ON by default)
- `UIPC_WITH_CUDA_BACKEND` - Enable CUDA backend (auto, disabled on macOS)

### Run Tests
Tests are Catch2 executables built to `build/Release/bin/`:
```bash
./build/Release/bin/uipc_test_<name>
```

### Install Python Package
```bash
cd build/python
pip install .
python ../python/uipc_info.py  # verify installation
```

## Architecture

### Three-Tier Design
1. **Engine** - Simulation algorithm running on a backend (`"cuda"` or `"none"`)
2. **World** - Manages simulation lifecycle (`init()`, `advance()`, `retrieve()`)
3. **Scene** - Data structure containing simulation state (Objects, Geometries, Constitutions, Contacts, Animator)

### Reporter-Manager-Receiver (RMR) Pattern
The codebase uses Data-Oriented Programming with an ECS-inspired RMR pattern for cache-friendly data flow between components. See `docs/development/index.md`.

### Source Layout
- `src/core/` - Main simulation engine, compiled into `libuipc_core` shared library
- `src/geometry/` - Geometry processing (SimplicialComplex, BVH, distance, intersection)
- `src/constitution/` - Material models (AffineBody, NeoHookean, springs, constraints)
- `src/backends/` - Backend implementations loaded as dynamic modules
  - `cuda/` - GPU backend with CUDA kernels
  - `none/` - CPU reference implementation
- `src/pybind/` - Python bindings via pybind11
- `src/io/` - File I/O (obj, gltf, serialization)

### Key Classes
- **SimplicialComplex** - Core geometry type (vertices, edges, triangles, tetrahedra)
- **Constitution** - Material models applied via `apply_to(mesh, properties)`
- **Contact Model** - Pairwise contact parameters stored in tabular form

### Backend Architecture
Backends are MODULE libraries dynamically loaded at runtime. They implement a visitor pattern for scene traversal and provide device-specific optimizations.

## Testing Structure

Tests are organized under `apps/tests/`:
- `geometry/` - Geometry processing tests
- `core/` - Engine, scene, world tests
- `common/` - Utility tests
- `backends/cuda/` - CUDA backend tests
- `sim_case/` - Full simulation scenarios

Test executables are named `uipc_test_<name>` via the `uipc_add_test()` CMake function.

## Python API Structure

Modules mirror C++ namespaces:
- `uipc.core` - Engine, World, Scene
- `uipc.geometry` - Geometry operations
- `uipc.constitution` - Material models
- `uipc.unit` - Physical units (GPa, MPa, etc.)
