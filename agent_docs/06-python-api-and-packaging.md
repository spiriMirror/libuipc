# 06 — Python API and Packaging

## pybind structure (`src/pybind/pyuipc/`)

`PYBIND11_MODULE(pyuipc, m)` in the entry `module.cpp`:

- Submodules: `unit`, `geometry`, `constitution`, `diff_sim`, `core`, `backend`, `builtin`, `usd`. The `usd` module object is created unconditionally, but USD classes are registered only when `UIPC_WITH_USD_SUPPORT` is enabled.
- C++ namespaces `pyuipc::xxx` map one-to-one to Python submodules `pyuipc.xxx`; each subdirectory has its own `module.cpp` that binds classes one by one via the `PyXxx{m}` constructor.
- **Early exposure**: the main `module.cpp` first binds the core data structures (`PyFeature`, `PyBufferView`, `PyAttributeSlot`, `PyGeometry`, `PySimplicialComplex`, the various Slots, `PyParameterCollection`), then calls each submodule's `PyModule` (geometry utilities/IO depend on core types).
- Top-level aliases: `Engine`, `World`, `Scene`, `SceneIO`, `Animation` are promoted to the `pyuipc` top level; also registers `init`, `default_config`, `config`, `build_info`, `uipc::Exception`, `__version__`. `build_info()` reports the compiled Python ABI, build type, CUDA-backend flag, CUDA architectures, and toolkit version for runtime diagnosis.
- Build: `pybind11_add_module(pyuipc)` links `uipc::uipc` + `uipc::backends`, and POST_BUILD runs `scripts/after_build_pyuipc.py` (copies the package, copies dependency libraries, generates `.pyi` via pybind11_stubgen).

## Python package layout (`python/src/uipc/`)

- `__init__.py`: on import, first calls `pyuipc.init(config)` (injecting `module_dir` pointing to `_native`), then `from ._native.pyuipc import *`.
- `core.py / geometry.py / constitution.py / backend.py / builtin.py / diff_sim.py / unit.py`: each is a single line `from uipc._native.pyuipc.xxx import *` (pure forwarding of the native module).
- Pure-Python enhancement layer:
  - `gui.py` (polyscope visualization)
  - `profile/` (phase-correct benchmark timing, deterministic performance
    baseline/check gates, and the `nsight.py` Nsight Compute wrapper)
  - `cli/` (`benchmark`, queryable `config-schema`, compatibility `doctor`,
    `mesh_doctor`, `uid_info`);
  `python -m uipc <command>` and the `uipc` console script dispatch these tools
  - `adapter/torch`, `adapter/warp` (ML framework adapters)
  - `assets/` (downloads scenes from HuggingFace `MuGdxy/uipc-assets`; each asset has a `scene.py` with `build_scene(scene)`)
  - `stats.py`, `dev/`
- Examples: `python/examples/` (4 demos: shell plastic crease ×2, prismatic/revolute joint limit GUI).

The base wheel installs `numpy`, `matplotlib`, `polyscope`, and
`huggingface_hub`, covering the statistics/reporting and asset layers. The
torch/warp adapters still require their own optional frameworks.

## Packaging pipeline

**Official release (root `pyproject.toml`, scikit-build-core)**:
- Version is generated dynamically by `setuptools_scm` (release-branch-semver).
- `python/src/uipc/compatibility.json` is the canonical release support policy.
  `scripts/check_release_policy.py` verifies both pyprojects, classifiers, the
  workflow ABI/toolkit matrix, and the CMake wheel architecture list against it.
- `wheel.packages = ["python/src/uipc"]`; CMake defines `UIPC_BUILD_PYBIND/WHEEL=ON`, targets `75-real;80-real;86-real;89-real;89-virtual`, and disables tests/examples/benchmarks. This gives Turing/Ampere/Ada native code plus a forward-compatible PTX path instead of the 0.0.26 wheel's Ada-only target.
- When `if(DEFINED SKBUILD)`, `UIPC_INSTALL_DIR = uipc/_native`: the pyuipc extension + vcpkg runtime DLLs are all installed into `_native/` inside the package; `.pyi` stubs are installed to the package root.
- cibuildwheel: on linux, auditwheel excludes all CUDA libraries (relies on system CUDA 12.8).
- The Windows wheel also relies on the system CUDA 12 runtime. In 0.0.26,
  `dumpbin /DEPENDENTS uipc_backend_cuda.dll` shows a direct
  `cublas64_12.dll` import. CUDA 13 installs only `cublas64_13.dll`, so a
  CUDA 13-only machine can import `uipc` but cannot construct
  `Engine("cuda", ...)`. The prebuilt-wheel compatibility statement must say
  CUDA **12.8 runtime**, not "12.6+". Source builds can target CUDA 13.

**Development mode (`python/pyproject.toml` + `python/setup.py`)**:
- During the CMake build, `after_build_pyuipc.py` copies `python/src/` + pyproject to `<build>/python/`, copies the extension and shared libraries into `src/uipc/_native/`, generates stubs, and in non-wheel mode runs `pip install` directly.
- `setup.py`'s `BuildPyCommand` collects dll/so from `build/vcpkg_installed/<triplet>/{bin,lib}` and `build/<config>/bin`.
- uv editable development: `uv run --no-sync pytest python/tests` (`--no-sync` avoids rebuilding every time).

## Python tests

`python/tests/`, pytest: 19 `test_*.py` files and 75 top-level test functions at
the audited revision. Prerequisite: pyuipc installed (CMake build or
`uv pip install -e .`).

The release matrix covers CPython 3.10-3.14 on Windows and manylinux. The
immutable 0.0.26 release stops at 3.13; Python 3.14 support begins with the next
wheel release.

Pytest excludes `example` and `cuda` by default, so the portable unit suite can
run on wheel builders without a display or NVIDIA device. Select GPU coverage
explicitly with `pytest -m "cuda and not example" python/tests`; interactive
examples remain opt-in with `-m example`. Tests that replace `uipc` modules must
restore `sys.modules` before returning so collection order cannot hide the real
package. Each cibuildwheel environment runs both the metadata/backend smoke test
and the portable pytest suite against the installed wheel.

Release verification must go beyond `import uipc`: importing loads the pybind
extension and core DLLs, while the backend is loaded lazily. At minimum create
`Engine("cuda", temporary_workspace)` from a clean environment; preferably
advance one asset-free frame. Otherwise a missing CUDA-major runtime dependency
escapes the smoke test.

## Key points for extending bindings

- When adding a new C++ public API, consider syncing the pybind side: add a `PyXxx` binding file in the corresponding submodule directory and register it in that directory's `module.cpp`; keep the namespace mapping consistent.
- Do not break the import chain in `__init__.py` (`pyuipc` → `init()` → `config["module_dir"]`).
- New binding surfaces need tests added in `python/tests/`.
- Audit exports rather than assuming C++/Python parity. `RotatingMotor` and
  `LinearMotor` are registered by
  `constitution/soft_transform_constraint.cpp`; an earlier hand-maintained audit
  incorrectly called them missing, which is why generated drift checks are
  preferable to prose inventories.

## Packaging/helper invariants

- Root and development metadata both include `matplotlib`, require
  `pytest>=9.0.3` for the dev extra, and describe the prebuilt-wheel CUDA 12.8
  runtime requirement consistently.
- `python -m uipc doctor [--probe-cuda] [--json]` separates Python ABI, native
  extension ABI, CUDA runtime-library, backend-load, driver, and GPU-code-image
  failures. It consumes both packaged `compatibility.json` and native
  `build_info()` instead of guessing from the package version.
- The Warp adapter falls back to the dtype's element size when a one-dimensional
  array reports no stride; a focused optional-Warp test covers this path.
- Python exposes `Scene.Objects.created_count()` as the exclusive object-ID upper
  bound. `assets.strip_constitutions` uses it so deleted/sparse object IDs do not
  leave higher-ID geometry partially processed.

## Frontend-visible changes (2026-08-24)

- `uipc.geometry.mesh_partition` was REMOVED from the python API. The MAS
  preconditioner is now a scene-level switch:
  `config["linear_system"]["fem_preconditioner"] = "mas"` (default
  `"diag"`), which auto-partitions every FEM geometry internally (fixed
  cluster size 16). Old scripts calling `mesh_partition(...)` raise
  ImportError — migrate them to the config switch.
- `newton/min_iter` semantics narrowed: it is now a pure hard floor with
  default `0` (no forced minimum). The semi-implicit beta accumulation
  start moved to `config["newton"]["semi_implicit"]["K_min"]` (default 1).
  Old scenes that set `min_iter` for the Stiff-GIPC Kmin role should set
  `K_min` instead.
