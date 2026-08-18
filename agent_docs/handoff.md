# Handoff — Current State of the Repo

> Written 2026-08-18 (second pass). Supersedes the earlier handoff from the
> same day: **all loose ends listed there are now resolved and verified.**
> Verify against the working tree before assuming anything beyond this file.

## TL;DR

- Branch `refactor-main`. The muda dependency is fully detached from the
  build: vendored copy at `src/backends/cuda/cuda_tool/muda/` is the only
  muda the CMake build uses; the `external/muda` submodule is gone.
- Working tree is clean; everything is committed. Nothing here is risky.

## Commits added in this pass (on top of `74a5df62`)

```
9018bec7 refactor(cuda): trim unused ext subtrees from vendored muda
921bf14f build: remove unused external/muda submodule
0b9f88d3 refactor(cuda): drop CUDA graph includes from vendored muda umbrella header
```

1. `0b9f88d3` — the previously uncommitted `cuda_tool/muda/muda.h` edit
   (removing `#include <muda/graph.h>` / `#include <muda/compute_graph.h>`)
   was **verified, not reverted**: muda's launch internals
   (`launch/details/*.inl`) include the compute_graph headers directly, so
   the umbrella includes were dead weight. Full sim_case rebuild passed and
   regression cases `0_abd_gravity` / `14_fem_3d_ground_contact` /
   `37_abd_revolute_joint` all passed on CUDA (178/432/311 assertions).
2. `921bf14f` — `external/muda` submodule removed (gitlink + `.gitmodules`
   entry + `.git/modules` cache). Note: `external/CMakeLists.txt` still had
   an `add_subdirectory(muda)` block guarded by `UIPC_WITH_CUDA_BACKEND`
   (the earlier handoff's "build no longer references it" was inaccurate);
   that block was deleted in the same commit, otherwise reconfigure would
   re-clone or fail. Reconfigure + rebuild verified.
3. `9018bec7` — vendored muda trimmed: `ext/{spatial_hash,spgrid,field}`
   (+ umbrella headers, 41 files) deleted. Zero references in sources and in
   build depfiles; verified by recompiling representative TUs
   (`global_linear_system`, `ipc_simplex_normal_contact`,
   `stable_neo_hookean_3d`, `advance_ipc`) and relinking.

## State of the muda vendoring

- Vendored tree: `src/backends/cuda/cuda_tool/muda/` (288 files after the
  trim). It equals the old `external/muda/src/muda` working tree (which had
  carried a local CUDA-13 fix in 4 `buffer/graph_*_view.h` files — that fix
  lives on in the vendored copy) plus the committed `UIPC_EIGEN_ARG_SHIM`
  guard in `ext/eigen/eigen_cxx20.h` and the umbrella-header trim above.
- The earlier handoff said "~460 files"; the actual count was 329 pre-trim.
- xmake caveat: `src/backends/cuda/xmake.lua` still does
  `add_requires("muda 09f8a0be…", {system=false})`, i.e. xmake fetches its
  own muda copy from the package registry. It does not use the vendored
  tree and was untouched by the submodule removal. Aligning xmake with the
  vendored copy is open.
- Self-written `cuda_tool/*.h` primitives (namespace
  `uipc::backend::cuda_tool`) remain reference/future material: business
  code still runs on vendored muda via `cuda_tool/muda_compat.h`. The
  `muda::` → `cuda_tool::` migration decision is still open.

## Environment notes (unchanged)

- Build: `output/build.bat` (calls VS 2022 Build Tools `vcvars64.bat`, then
  `cmake --build build --config Release --target sim_case -j8`). nvcc needs
  the MSVC environment — plain shells fail with "Cannot find compiler
  'cl.exe'".
- Configure: `cmake -S . -B build --preset ci-release
  -DUIPC_BUILD_BENCHMARKS=OFF -DUIPC_BUILD_EXAMPLES=OFF` (must pass
  `-B build`).
- Catch2: pass test names one run at a time; multiple quoted names in one
  invocation matched nothing in this environment.
- Test binary: `build/Release/bin/uipc_test_sim_case.exe`.
