# libuipc

[![PyPI version](https://img.shields.io/pypi/v/pyuipc?color=blue)](https://pypi.org/project/pyuipc/)
[![Downloads](https://static.pepy.tech/badge/pyuipc/month)](https://pepy.tech/projects/pyuipc)
[![Documentation](https://img.shields.io/badge/docs-spirimirror.github.io%2Flibuipc--doc-green)](https://spirimirror.github.io/libuipc-doc/)
[![Issues](https://img.shields.io/github/issues/spiriMirror/libuipc)](https://github.com/spiriMirror/libuipc/issues)
[![License](https://img.shields.io/badge/license-Apache%202.0-red)](LICENSE)

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact — both C++ and Python APIs.

Website ➡️ [spirimirror.github.io/libuipc-web](https://spirimirror.github.io/libuipc-web/) ・ Samples ➡️ [github.com/spiriMirror/libuipc-samples](https://github.com/spiriMirror/libuipc-samples/)

![teaser](docs/media/teaser.png)

**Libuipc** is a GPU-accelerated simulation library built around a unified **Incremental Potential Contact** framework. It couples rigid bodies (affine body dynamics), soft bodies (FEM), cloth, and threads in one scene with accurate, **penetration-free frictional contact**, and is designed to be naturally **differentiable** for ML, inverse dynamics, and robotics workflows. The whole pipeline — collision detection, contact assembly, and a fused-PCG linear solver with CUDA-graph replay and an optional MAS preconditioner — runs on the GPU.

We are **actively** developing Libuipc. Feedback and contributions are welcome!

## Table of Contents

- [libuipc](#libuipc)
  - [Table of Contents](#table-of-contents)
  - [What is libuipc?](#what-is-libuipc)
    - [Why libuipc](#why-libuipc)
  - [Catalogue](#catalogue)
    - [Stiff-GIPC Benchmark Suite (88–93)](#stiff-gipc-benchmark-suite-8893)
    - [Robotics](#robotics)
    - [FEM \& Cloth](#fem--cloth)
    - [Rigid Bodies, Joints \& Coupling](#rigid-bodies-joints--coupling)
  - [Installation](#installation)
    - [Python (PyPI)](#python-pypi)
    - [From Source](#from-source)
  - [Quick Start (Python)](#quick-start-python)
  - [Contributing \& Support](#contributing--support)
  - [News](#news)
  - [License \& Acknowledgments](#license--acknowledgments)
  - [Citation](#citation)

## What is libuipc?

libuipc is organized in three layers: a friendly scene API on top, a reusable simulation core in the middle, and a fully-parallel CUDA backend underneath.

- **Simulation Interface** — Python and C++ APIs for building scenes: geometry IO (OBJ/MSH/URDF/glTF), constitutions (SNK elasticity, Baraff-Witkin cloth, shell bending, ABD affine bodies, joint systems), contact tabulars, animation scripting, and a Polyscope-based GUI.
- **Simulation Core** — scene/world/engine abstractions with the incremental-potential Newton solver: BDF1 time integration, line search with continuous collision detection, friction with scene-adaptive tolerances, and a scene-adaptive kappa corridor.
- **CUDA Backend** — fused-PCG linear solver with CUDA-graph block replay, MAS (Multi-Level Additive Schwarz) FEM preconditioner with internal auto-partitioning, stackless-BVH collision detection, and per-constitution GPU kernels — all running without CPU round-trips in the solver loop.

### Why libuipc

- **Easy & Powerful**: an intuitive, unified way to create and drive vivid simulation scenes; objects and constraints compose freely.
- **Fast & Robust**: fully GPU-parallel, with Stiff-GIPC-grade numerics (SNK1 constitution, analytic SPD projections) and a contact model that stays penetration-free under stiff, frictional, coupled scenarios.
- **High Flexibility**: Python and C++ APIs, Linux and Windows, PyPI wheels and source builds.
- **Fully Differentiable**: differentiable simulation APIs for backward optimization (Diff-Sim, coming soon).

## Catalogue

Runnable examples live in [libuipc-samples/examples/](https://github.com/spiriMirror/libuipc-samples/tree/main/examples) — each is a self-contained `main.py` with a GUI (run/stop) and most support `--headless [N]` benchmarking. Highlights:

### Stiff-GIPC Benchmark Suite (88–93)

- [88: set_case2](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/88_stiff_gipc_benchmark)
- [89: set_case7](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/89_mas_bunny)
- [90: set_case1](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/90_abd_fem_cube_stack)
- [91: set_case4](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/91_pinned_cloth)
- [92: set_case5](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/92_twisting_bar)
- [93: set_case6](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/93_cube_wall_cloth)

### Robotics

- [87: robot hand (URDF links + soft constraints + ABD cube)](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/87_robot_hand)

### FEM & Cloth

- [3: periodically pressed tetrahedron](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/3_periodically_pressed_tetrahedron)
- [11: bunny + cloth](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/11_bunny_cloth)
- [23: Kirchhoff rod bending](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/23_kirchoff_rod_bending)
- [24: sewing pattern](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/24_sewing_pattern)
- [34: cloth stack](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/34_cloth_stack)
- [36: vertex stitch family](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/36_vertex_stitch_family)

### Rigid Bodies, Joints & Coupling

- [2: walking cube](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/2_walking_cube)
- [6: wrecking balls](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/6_wrecking_balls)
- [10: ramp sliding](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/10_ramp_sliding)
- [12: soft transform constraint](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/12_soft_transform_constraint)
- [17: revolute joint](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/17_affine_body_revolute_joint)
- [18: pendulum joint](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/18_pendulum_joint)


## Installation

### Python (PyPI)

```bash
pip install pyuipc
```

Prebuilt wheels: **Windows / Linux, Python 3.10–3.13, CUDA 12.8 runtime**.
The Windows wheel dynamically loads `cublas64_12.dll`; a CUDA 13-only install
does not provide that versioned runtime. Use CUDA 12.8 side-by-side, or build
from source for CUDA 13.

### From Source

**Prerequisites**: CMake ≥ 3.26, Python ≥ 3.11, CUDA ≥ 12.4, and [vcpkg](https://github.com/microsoft/vcpkg) (XMake ≥ 3.0.5 if you prefer that build path).

```bash
# 1) vcpkg (once): clone + bootstrap, then point CMake at the toolchain
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.bat   # Linux: ./bootstrap-vcpkg.sh
# set CMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake

# 2) clone (samples come along as a submodule)
git clone https://github.com/spiriMirror/libuipc.git --recurse-submodules
cd libuipc && mkdir build && cd build

# 3) configure + build (NVCC is slow — use all cores)
cmake -S .. -DUIPC_BUILD_PYBIND=1
cmake --build . --config Release -j8   # Linux: add -DCMAKE_BUILD_TYPE=Release
```

With `UIPC_BUILD_PYBIND=ON`, the Python binding is built **and installed** into the active Python environment. If the install step fails, finish it manually with `cd build/python && pip install .`; to target a specific venv, pass `-DUIPC_PYTHON_EXECUTABLE_PATH=<python.exe>`.

On Linux, a conda environment is recommended: `conda env create -f conda/env.yaml && conda activate uipc_env`, then `conda env config vars set CMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake` before the build commands above.

**XMake alternative**: `xmake f -c && xmake build -j8` (Python binding: `xmake f --pybind=true --python_system=true --python_version=3.11.x -c`).

**Check the install**: `cd python && python uipc_info.py`.

Full guide (incl. Docker and CUDA/driver compatibility notes): [Build & Install](https://spirimirror.github.io/libuipc-doc/build_install/). For the samples' uv-based workflow, see the [libuipc-samples README](https://github.com/spiriMirror/libuipc-samples#readme).

## Quick Start (Python)

```python
from uipc.core import Engine, World, Scene
from uipc.geometry import SimplicialComplexIO, ground, label_surface, label_triangle_orient, flip_inward_triangles
from uipc.constitution import StableNeoHookean, ElasticModuli

engine = Engine("cuda", "output")
world  = World(engine)
scene  = Scene(Scene.default_config())
scene.contact_tabular().default_model(0.2, 1e8)

mesh = SimplicialComplexIO().read("bunny.msh")
label_surface(mesh)
label_triangle_orient(mesh)
mesh = flip_inward_triangles(mesh)
StableNeoHookean().apply_to(mesh, ElasticModuli.youngs_poisson(1e6, 0.49), mass_density=1e3)

scene.objects().create("bunny").geometries().create(mesh)
scene.objects().create("ground").geometries().create(ground(-1.0))

world.init(scene)
for _ in range(100):
    world.advance()
    world.retrieve()
```

Optional: enable the MAS preconditioner for stiff FEM scenes with one config line — `config["linear_system"]["fem_preconditioner"] = "mas"`.

## Contributing & Support

- **Bugs & features**: file an [Issue](https://github.com/spiriMirror/libuipc/issues); questions and ideas go to [Discussions](https://github.com/spiriMirror/libuipc/discussions).
- **Pull requests** are welcome — CI runs the full simulation suite, please keep it green.
- **For maintainers**: `.cursor/` holds AI-assisted build/test/commit/PR commands and style rules; `agent_docs/` documents the architecture.

## News

**2026-8-24**: We preliminarily optimized the CUDA backend, improving the framework's simulation performance across the board — and **Libuipc v1.0 will be ready soon**!

**2026-3-29**: Thanks [Genesis AI](https://genesis-embodied-ai.github.io/) for contributing [AL-IPC](https://simulation-intelligence.github.io/barrier-free/) (Augmented Lagrangian IPC) integration, a Minimal Coordinate Articulated Joint System (ExternalArticulationConstraint) for affine bodies, and the CIBuildWheel cross-platform PyPI packaging pipeline.

**2026-2-13**: [Cursor](https://www.cursor.com/) AI-assisted development support — see `.cursor/` for built-in commands and rules.

**2026-2-7**: `pip install pyuipc` on PyPI (Win/Linux, Python 3.10–3.13, CUDA 12.8).

**2025-11-01**: The prototype implementation of Libuipc was open-sourced ([Stiff-GIPC](https://github.com/KemengHuang/Stiff-GIPC)) as our performance benchmark reference.

**2025-5-23**: [StiffGIPC](https://dl.acm.org/doi/10.1145/3735126) presented at SIGGRAPH 2025.

**2024-11-25**: Libuipc v0.9.0 (Alpha) published — feedback welcome via [Issues](https://github.com/spiriMirror/libuipc/issues) and [PRs](https://github.com/spiriMirror/libuipc/pulls)!

## License & Acknowledgments

libuipc is released under the **Apache License 2.0** (see [LICENSE](LICENSE)).

This project is led by [Kemeng Huang](https://kemenghuang.github.io/) and Professor [Taku Komura](https://www.cs.hku.hk/~taku) (The University of Hong Kong), in collaboration with Professor [Minchen Li](https://www.cs.cmu.edu/~minchenl/) (Carnegie Mellon University), with comprehensive support from TransGP. It builds upon the GPU IPC project, originally developed and continuously optimized by Kemeng Huang. We also thank the many open-source contributors, with special thanks to Xinyu Lu, who initially worked with us at TransGP and helped a lot in this project.

## Citation

If you use **Libuipc** in your project, please cite our works:

```bibtex
@article{stiffgipc2025,
      author = {Huang, Kemeng and Lu, Xinyu and Lin, Huancheng and Komura, Taku and Li, Minchen},
      title = {StiffGIPC: Advancing GPU IPC for Stiff Affine-Deformable Simulation},
      year = {2025},
      publisher = {Association for Computing Machinery},
      volume = {44},
      number = {3},
      issn = {0730-0301},
      doi = {10.1145/3735126},
      journal = {ACM Trans. Graph.},
      month = may,
      articleno = {31},
      numpages = {20}
}
```

```bibtex
@article{gipc2024,
      author = {Huang, Kemeng and Chitalu, Floyd M. and Lin, Huancheng and Komura, Taku},
      title = {GIPC: Fast and Stable Gauss-Newton Optimization of IPC Barrier Energy},
      year = {2024},
      publisher = {Association for Computing Machinery},
      volume = {43},
      number = {2},
      issn = {0730-0301},
      doi = {10.1145/3643028},
      journal = {ACM Trans. Graph.},
      month = {mar},
      articleno = {23},
      numpages = {18}
}
```
