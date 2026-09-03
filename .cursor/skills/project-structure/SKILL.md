---
name: project-structure
description: Overview of the main directories and important files in the repository. Use this to understand the project layout and find where code lives.
---

# Project Structure Guide

Overview of the main directories and important files in the repository.

## Top-Level Directories

- `apps/` - C++ apps, benchmarks, examples, and tests (see `apps/CMakeLists.txt`).
- `assets/` - Scene assets and simulation data.
- `cmake/` - Shared CMake utilities and macros.
- `docs/` - Documentation, specs, and site assets.
- `include/` - Public C++ headers.
- `output/` - Generated outputs from simulations. (the contents is ignored by git)
- `python/` - Python package, tests, and metadata.
- `libuipc-samples/` - Tracked git submodule containing Python examples, benchmarks, and sample assets.
- `scripts/` - Utility scripts and notebooks.
- `src/` - Core C++ implementation.
- `xmake/` - XMake rules and build helpers.

## Subfolder Purposes

### `apps/`

- `app/` - Shared app utilities and test helpers.
- `benchmarks/` - Benchmark executables.
- `examples/` - Example applications (each in its own subfolder).
- `tests/` - C++ test suites grouped by feature area.

### `assets/`

- `scenes/` - Scene README and references.
- `sim_data/` - Mesh assets (`linemesh/`, `tetmesh/`, `trimesh/`).

### `docs/`

- `build_install/` - Build and install guides.
- `development/` - Developer docs and diagrams.
- `specification/` - API/spec references and constitutions.
- `tutorial/` - Tutorial content and media.
- `overrides/` - MkDocs HTML overrides.
- `stylesheets/` - Docs CSS.
- `javascripts/` - Docs JS.
- `media/` - Images for docs pages.
- `misc/` - Miscellaneous documentation.

### `include/`

- `uipc/` - Public headers grouped by module (backend, core, geometry, io, etc.). The removed C++ GUI has no public header tree.

### `python/`

- `src/uipc/` - Pure python code to enhance the C++ pybind11 bindings.
- `tests/` - Python test suite.

### `scripts/`

- `optional_import/` - Optional dependency loaders.
- `symbol_calculation/` - Symbolic derivation notebooks and helpers.
- `SymEigen/` - SymEigen utilities, notebooks, and tests.

### `src/`

- `backends/` - Backend implementations (`common/`, `cuda/`, `none/`).
  - `common/` - Common utilities for all backends.
  - `cuda/` - CUDA backend with CUDA kernels.
  - `none/` - An empty backend that does nothing, as a template for creating a new backend or checking the basic functionality.
- `geometry/metis/` - Private C++ METIS implementation and required
  GKlib-derived support code, built as `uipc_metis` for mesh partitioning.
- `constitution/` - Physical constitution implementations.
- `core/` - Core engine systems and modules.
- `geometry/` - Geometry algorithms and data types.
- `io/` - Serialization and I/O.
- `pybind/` - C++/Python bindings.
- `sanity_check/` - Internal scene sanity checks to ensure the correctness of scene before the simulation.
- `usd/` - USD integration.
- `vdb/` - VDB integration.

The supported visualization frontend is the pure-Python polyscope layer at
`python/src/uipc/gui.py`. CMake is the primary build; XMake currently has known
option/module drift documented in `agent_docs/07-build-test-workflow.md` and
`agent_docs/12-secondary-modules-samples-and-docs.md`.
