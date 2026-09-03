# 01 — Project Overview

## Positioning

**libuipc** (**Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact) is a cross-platform modern C++20 library that unifies simulation of rigid bodies, soft bodies, cloth, threads, and their coupling on the GPU, guaranteeing **penetration-free, accurate frictional contact**, and is designed to support differentiable simulation (Diff-Sim planned). It provides dual C++ and Python (`pyuipc`) APIs, supporting Windows/Linux.

Academic origins:
- GIPC 2024 (ACM TOG 43(2)): Gauss-Newton optimization of the IPC barrier energy
- StiffGIPC 2025 (ACM TOG 44(3), Siggraph): stiff affine-deformable coupling GPU IPC
- Prototype open-source implementation: [Stiff-GIPC](https://github.com/KemengHuang/Stiff-GIPC) (performance benchmark)

Recent important changes (README News):
- 2026-03: Genesis AI contributed AL-IPC (Augmented Lagrangian IPC) contact, ExternalArticulationConstraint (minimal-coordinate joint systems), cibuildwheel CI packaging
- 2026-02: PyPI release `pip install pyuipc` (Win/Linux, Python 3.10–3.13, CUDA 12.8)

## Top-Level Directories

| Directory | Contents |
|---|---|
| `src/` | Core implementation: `core/` (engine, compiled as `libuipc_core`), `geometry/` (geometry algorithms plus the private C++ METIS implementation under `geometry/metis/`), `constitution/` (constitutions), `backends/` (`common/`+`cuda/`+`none/`), `io/`, `pybind/`, `sanity_check/`, `usd/`, `vdb/` |
| `include/uipc/` | Public headers, organized by module: `core/ geometry/ constitution/ io/ backend/ builtin/ common/ diff_sim/ usd/ vdb/`; umbrella headers `uipc.h / core.h / geometry.h / io.h` |
| `apps/` | `tests/` (Catch2 tests), `examples/` (3 C++ examples), `benchmarks/`, `app/` (test utility library) |
| `python/` | Python package source `python/src/uipc/`, tests, examples, and a standalone development `pyproject.toml` |
| `docs/` | mkdocs documentation source: `tutorial/ specification/ development/ build_install/ media/`, etc. |
| `ports/` | vcpkg **overlay ports** (currently `tinygltf/`: pins the regenerated v2.9.6 tarball SHA512 that upstream vcpkg has stale; referenced via `overlay-ports` in the generated `vcpkg-configuration.json`) |
| `scripts/` | Build helpers (`gen_vcpkg_json.py`, `after_build_pyuipc.py`, `build_docs.py`), symbolic derivation notebooks, and the tracked `SymEigen/` submodule |
| `benchmarks/` | Versioned four-scene end-to-end benchmark manifest and reproducibility contract; implementations/assets remain in `libuipc-samples/` |
| `assets/` | Simulation mesh assets (`sim_data/{linemesh,tetmesh,trimesh}`) |
| `libuipc-samples/` | Tracked git submodule containing the Python sample library (52 current example directories plus benchmarks/assets) |
| `output/` | Simulation output directory (gitignored) |
| `.cursor/` | AI development configuration: `rules/` (C++ style, self-improvement) + `skills/` (17 workflow SKILL.md files) |
| `agent_docs/` | Agent guide, including durable `adr/` decisions, `performance/` evidence, and chronological `handoff.md` history |
| `CMakeLists.txt` / `CMakePresets.json` / `pyproject.toml` | Build entry points |

> **Local reference checkouts in this working copy (untracked, do NOT commit into this repo)**: `Stiff-GIPC/` (performance-alignment reference) and `references/` (third-party reference code). `libuipc-samples/` is different: it is a tracked submodule, so update its gitlink deliberately rather than treating its files as part of the root repository.

## Three Core Concepts

```
Engine (algorithm + device, e.g. "cuda")
  └── World (simulation lifecycle: init → advance → retrieve; frame() queries the current frame)
        └── Scene (snapshot at a moment in time, holding all information needed to drive the physics)
```

The 5 major components of a Scene (see `docs/tutorial/concepts.md` for details):

1. **Objects**: real-world entities (e.g. "a T-shirt"), `scene.objects().create("name")`; an Object can hold multiple geometries (`object.geometries().create(mesh)` returns a GeometrySlot) and supports multiple instances.
2. **Geometries**: geometry data held by the scene (the core is `SimplicialComplex`).
3. **Constitution Tabular**: collection of constitution models. `constitution.apply_to(mesh, params...)` writes parameters into geometry attributes, identified by a **UID** (official: 0~2³²-1; user-defined: 2³²~2⁶⁴-1; written to the `constitution_uid` attribute in meta).
4. **Contact Tabular**: contact model table. `contact_tabular.create("wood")` creates a ContactElement; `insert(ce1, ce2, friction, resistance)` builds a pairwise model $C_{ij}=(\kappa,\mu,f)$; undefined pairs fall back to `default_model` (id=0). There is also the Subscene Tabular (subscene enable matrix, default identity).
5. **Animator**: animation script. `animator.insert(obj, update_fn)` is a per-frame callback; `info.geo_slots()` retrieves the bound geometries; must be combined with Constraints (`SoftTransformConstraint` drives an affine body's `aim_transform`, `SoftPositionConstraint` drives vertices' `aim_position`), and the geometry must first set `is_constrained=1`.

## Typical Simulation Flow (Hello Libuipc)

```cpp
Engine engine{"cuda", workspace};
World world{engine};
Scene scene{Scene::default_config()};
auto& ct = scene.contact_tabular();
ct.default_model(0.5, 1.0_GPa);            // friction 0.5, contact stiffness 1 GPa

auto mesh = tetmesh(Vs, Ts);               // build a tetrahedral mesh
abd.apply_to(mesh, 100.0_MPa);             // AffineBodyConstitution, kappa=100MPa
label_surface(mesh);                       // required for contact: label the surface
label_triangle_orient(mesh);               // label triangle orientation
mesh.instances().find<IndexT>(builtin::is_fixed)->view()[0] = 0; // movable

auto obj = scene.objects().create("tet");
obj->geometries().create(mesh);
world.init(scene);
world.advance();
world.retrieve();
scene_io.write_surface("out.obj", scene);  // export the surface
```

Python follows the same overall lifecycle (`from uipc.core import Engine, World, Scene`;
the native extension also promotes these names to `uipc`). Public constitution
coverage is checked against `src/pybind/pyuipc/constitution/` in CI; for other
modules, check their pybind sources before promising exact parity. The rotating and
linear motor classes are currently bound.

## Key Design Trade-offs

- **Data-Oriented**: geometry/physics data is stored in attribute collections (AttributeCollection); systems pass views to each other via RMR, avoiding deep OOP inheritance, which favors GPU cache-friendly access.
- **Pluggable backends**: all algorithms live in the backend; core only handles data structures and lifecycle; supporting new hardware only requires implementing a new backend module.
- **Unit literals**: `uipc::unit` provides literals such as `1.0_GPa`, `100.0_MPa`, `0.05_m/1.0_s`, so physical parameters carry their dimensions.
- **Dual build systems**: CMake (primary, used by CI/wheels) + XMake (alternative); `xmake.lua` is maintained in parallel alongside source directories.
- **Narrow umbrella headers**: `uipc/uipc.h` includes core, geometry, and basic IO,
  not every constitution, URDF/AttributeIO, optional module, or advanced geometry
  facility. See doc 10 before relying on transitive includes.
