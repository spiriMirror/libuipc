# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Read `agent_docs/rule.md` before making changes, then use
`agent_docs/README.md` as the maintained architecture index. Accepted design
boundaries live in `agent_docs/adr/`, durable performance evidence in
`agent_docs/performance/`, and chronological history in `agent_docs/handoff.md`.

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
# Using presets (available presets: ci-release, ci-build-wheel)
cmake --preset ci-release
cmake --build --preset ci-release -j8

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
- `UIPC_WITH_CUDA_LEGACY_COLLISION` - Build alternate legacy broad-phase filters (ON by default)

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
Backends are shared libraries loaded dynamically at runtime. Core validates a
versioned `uipc_query_module` handshake before entering their C++ interface.
They implement a visitor pattern for scene traversal and provide device-specific
optimizations.

## Testing Structure

Tests are organized under `apps/tests/`:
- `geometry/` - Geometry processing tests
- `core/` - Engine, scene, world tests
- `common/` - Utility tests
- `backends/cuda/` - CUDA backend tests
- `sim_case/` - Full simulation scenarios

Test executables are named `uipc_test_<name>` via the `uipc_add_test()` CMake function.
Keep the 95-case `uipc.sim_case` single-process aggregate for global-state
pollution coverage. Use `scripts/run_sim_case_isolated.py` for fresh-process
manifests/shards and `scripts/run_benchmark.py` for registered performance
scenes.

## Python API Structure

Modules mirror C++ namespaces:
- `uipc.core` - Engine, World, Scene
- `uipc.geometry` - Geometry operations
- `uipc.constitution` - Material models
- `uipc.unit` - Physical units (GPa, MPa, etc.)

## Cursor Rules and Skills

This repo also has Cursor-format rules and skills that apply to Claude Code work:

- **Always-on rules** — read `.cursor/rules/*.mdc` at the start of any coding task and follow them. Treat the prose as authoritative. Ignore Cursor-specific syntax: `mdc:<path>` links are just relative file paths, and the frontmatter (`globs`, `alwaysApply`, etc.) is for Cursor's auto-attach logic — not instructions for you. Currently:
  - `.cursor/rules/cpp-format.mdc` — C++ formatting rules (clang-format-aligned style). Apply to every `*.h *.hpp *.cpp *.inl *.cu *.cuh` edit.
  - `.cursor/rules/self-improvement.mdc` — when you notice repeated patterns or rule gaps, propose updates to the rules.

- **Cursor skills** — `.cursor/skills/<name>/SKILL.md`. Each has YAML frontmatter with `name`, `description`, and optional `disable-model-invocation: true` (which means the user must explicitly invoke it; do NOT auto-trigger).

  **How to use them:** when the user's request matches a skill — by name (e.g. "run the `commit` skill", "/format"), by intent matching the description, or by explicit ask — read the full `.cursor/skills/<name>/SKILL.md` with the Read tool and follow its steps. Skills may reference other skills via `[name](./other.md)` links — load those too on demand.

  **Available skills (index):**

  *Auto-triggerable (read whenever the description matches the task):*
  - `cmake-workflow` — Build and test libuipc via CMake (configure, RelWithDebInfo, Catch2). Trigger on build/test asks.
  - `commit-convention` — Conventional commit message format and rules for this project.
  - `cursor-rules` — How to add or edit Cursor rules in this project.
  - `document` — Documentation style guide and rules.
  - `gpu-optimization` — GPU profiling/optimization workflow (`uipc.profile`, `uipc.profile.nsight`, Nsight Compute CLI). Trigger when profiling/optimizing CUDA kernels.
  - `project-structure` — Overview of main directories and important files. Trigger when needing repo layout.
  - `repository-setup` — Setting up remotes when working with forks.
  - `review-pr` — End-to-end PR review (checkout, summarize, domain-aware AI review covering physics, backend, C++ style, GPU, pybind). Trigger on PR-review asks.
  - `simulation-dev` — Simulation dev best practices: correctness, stability, debuggability, index safety, NaN/Inf, diagnostics. Trigger when modifying solvers/constraints/GPU kernels.
  - `xmake-workflow` — Build/test via XMake (alternate build system).

  *Explicit-invocation only (`disable-model-invocation: true` — only when the user explicitly asks):*
  - `commit` — Create a conventional commit locally (no push without explicit ask).
  - `fix-issue` — Fix a GitHub issue with proper branch / testing / PR flow.
  - `fix-pr` — Fix a PR based on review feedback.
  - `format` — Run `clang-format` on C++ files changed vs. `main`.
  - `github-pr` — Create a structured pull request.
  - `push-tag` — Push a version tag and optionally create a GitHub release.
  - `run-tests` — Run C++ and Python tests.
