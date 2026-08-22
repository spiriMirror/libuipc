# 06 — Python API and Packaging

## pybind structure (`src/pybind/pyuipc/`)

`PYBIND11_MODULE(pyuipc, m)` in the entry `module.cpp`:

- Submodules: `unit`, `geometry`, `constitution`, `diff_sim`, `core`, `backend`, `builtin`, `usd` (USD only when `UIPC_WITH_USD_SUPPORT`).
- C++ namespaces `pyuipc::xxx` map one-to-one to Python submodules `pyuipc.xxx`; each subdirectory has its own `module.cpp` that binds classes one by one via the `PyXxx{m}` constructor.
- **Early exposure**: the main `module.cpp` first binds the core data structures (`PyFeature`, `PyBufferView`, `PyAttributeSlot`, `PyGeometry`, `PySimplicialComplex`, the various Slots, `PyParameterCollection`), then calls each submodule's `PyModule` (geometry utilities/IO depend on core types).
- Top-level aliases: `Engine`, `World`, `Scene`, `SceneIO`, `Animation` are promoted to the `pyuipc` top level; also registers `init`, `default_config`, `config`, `uipc::Exception`, `__version__`.
- Build: `pybind11_add_module(pyuipc)` links `uipc::uipc` + `uipc::backends`, and POST_BUILD runs `scripts/after_build_pyuipc.py` (copies the package, copies dependency libraries, generates `.pyi` via pybind11_stubgen).

## Python package layout (`python/src/uipc/`)

- `__init__.py`: on import, first calls `pyuipc.init(config)` (injecting `module_dir` pointing to `_native`), then `from ._native.pyuipc import *`.
- `core.py / geometry.py / constitution.py / backend.py / builtin.py / diff_sim.py / unit.py`: each is a single line `from uipc._native.pyuipc.xxx import *` (pure forwarding of the native module).
- Pure-Python enhancement layer:
  - `gui.py` (polyscope visualization)
  - `profile/` (benchmark timing + `nsight.py` Nsight Compute wrapper)
  - `cli/` (`benchmark`, `mesh_doctor`, `uid_info`)
  - `adapter/torch`, `adapter/warp` (ML framework adapters)
  - `assets/` (downloads scenes from HuggingFace `MuGdxy/uipc-assets`; each asset has a `scene.py` with `build_scene(scene)`)
  - `stats.py`, `dev/`
- Examples: `python/examples/` (4 demos: shell plastic crease ×2, prismatic/revolute joint limit GUI).

## Packaging pipeline

**Official release (root `pyproject.toml`, scikit-build-core)**:
- Version is generated dynamically by `setuptools_scm` (release-branch-semver).
- `wheel.packages = ["python/src/uipc"]`; CMake defines `UIPC_BUILD_PYBIND/WHEEL=ON`, `UIPC_CUDA_ARCHITECTURES=89`, and disables tests/examples/benchmarks.
- When `if(DEFINED SKBUILD)`, `UIPC_INSTALL_DIR = uipc/_native`: the pyuipc extension + vcpkg runtime DLLs are all installed into `_native/` inside the package; `.pyi` stubs are installed to the package root.
- cibuildwheel: on linux, auditwheel excludes all CUDA libraries (relies on system CUDA 12.8).

**Development mode (`python/pyproject.toml` + `python/setup.py`)**:
- During the CMake build, `after_build_pyuipc.py` copies `python/src/` + pyproject to `<build>/python/`, copies the extension and shared libraries into `src/uipc/_native/`, generates stubs, and in non-wheel mode runs `pip install` directly.
- `setup.py`'s `BuildPyCommand` collects dll/so from `build/vcpkg_installed/<triplet>/{bin,lib}` and `build/<config>/bin`.
- uv editable development: `uv run --no-sync pytest python/tests` (`--no-sync` avoids rebuilding every time).

## Python tests

`python/tests/`, pytest. Prerequisite: pyuipc installed (CMake build or `uv pip install -e .`).

## Key points for extending bindings

- When adding a new C++ public API, consider syncing the pybind side: add a `PyXxx` binding file in the corresponding submodule directory and register it in that directory's `module.cpp`; keep the namespace mapping consistent.
- Do not break the import chain in `__init__.py` (`pyuipc` → `init()` → `config["module_dir"]`).
- New binding surfaces need tests added in `python/tests/`.
