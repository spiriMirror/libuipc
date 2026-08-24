# 10 — Code Navigation and Extension Map

This document answers two questions that the topic guides do not answer directly:

1. Where does a public API call travel through the repository?
2. Which files must change together when a capability is extended?

The map was checked against `2b954973` on 2026-08-24. Counts are an orientation aid,
not an invariant: there are 237 public headers under `include/uipc/`, 1,013 source
files under `src/`, 141 C++ test translation units, and 17 Python test files.

## Public Include Surface

`include/uipc/uipc.h` is intentionally small. It includes only:

- `uipc/core.h`: initialization, Engine/World/Scene, snapshots, scene tables,
  geometry slots, common types/units, and built-in attribute names;
- `uipc/geometry.h`: the core geometry types, atlas/commit types, and the utilities
  exported by `uipc/geometry/utils.h`;
- `uipc/io.h`: `SimplicialComplexIO`, `SpreadSheetIO`, and `SceneIO`.

Do not assume that including `uipc/uipc.h` exposes every public facility. These
families normally need an explicit header:

- constitutions and constraints: `uipc/constitution/<model>.h`;
- URDF: `uipc/io/urdf_io.h`;
- single-attribute IO: `uipc/io/attribute_io.h`;
- optional USD/VDB APIs: `uipc/usd/*`, `uipc/vdb/*`;
- advanced geometry helpers that are not in `geometry/utils.h`, including the
  BVH/octree, distance/intersection, graph coloring, affine-body mass helpers,
  and some area/volume utilities.

The geometry factory functions are declared in
`include/uipc/geometry/utils/factory.h`, not `include/uipc/geometry/factory.h`.

## API-to-Implementation Trace

| User-facing capability | Public/API layer | Core or CPU implementation | Runtime/backend consumer | Python binding | Good evidence |
|---|---|---|---|---|---|
| Process initialization | `include/uipc/common/uipc.h` | `src/core/common/uipc.cpp` | Backend loader reads `config()["module_dir"]` | `python/src/uipc/__init__.py` supplies `_native` automatically | `apps/tests/core/engine.cpp` |
| Engine and backend loading | `include/uipc/core/engine.h` | `src/core/core/engine.cpp`, `src/core/core/internal/engine.cpp` | `src/backends/{cuda,none}/entrance.cpp` implements the three-symbol ABI | `src/pybind/pyuipc/core/engine.cpp` | `apps/tests/core/engine.cpp` |
| World lifecycle | `include/uipc/core/world.h` | `src/core/core/world.cpp`, `src/core/core/internal/world.cpp` | `IEngine::do_init/do_advance/do_sync/do_retrieve` | `src/pybind/pyuipc/core/world.cpp` | nearly every `apps/tests/sim_case/*.cpp` |
| Scene and default config | `include/uipc/core/scene.h` | `src/core/core/scene*.cpp`, `src/core/core/internal/scene.cpp` | Managers read attributes during `on_init_scene` and rebuild events | `src/pybind/pyuipc/core/scene.cpp` | `apps/tests/core/scene.cpp`; `docs/specification/scene_config.md` |
| Objects and geometry slots | `include/uipc/core/object.h`, `include/uipc/geometry/geometry_slot.h` | `src/core/core/object*.cpp`, `src/core/geometry/geometry_collection.cpp` | Global geometry managers consume current/rest slots | `src/pybind/pyuipc/core/object.cpp`, `src/pybind/pyuipc/geometry/geometry_slot.cpp` | `apps/tests/core/object.cpp` |
| Attribute storage/views | `include/uipc/geometry/attribute*.h` | `src/core/geometry/attribute*.cpp` | Backend reporters copy attributes into device buffers | early bindings in `src/pybind/pyuipc/module.cpp` | `apps/tests/core/attribute_collection.cpp` |
| SimplicialComplex and algorithms | `include/uipc/geometry/simplicial_complex.h`, `utils/*.h` | `src/geometry/*.cpp` plus focused `affine_body/`, `bvh/`, `graph_coloring/`, and `implicit_geometries/` directories | Constitution/contact systems claim labeled attributes | `src/pybind/pyuipc/geometry/` | `apps/tests/geometry/`; samples `16_geometry_operation`, `27_compute_mesh_d_hat` |
| Constitutions/constraints | `include/uipc/constitution/*.h` | `src/constitution/*.cpp` writes UID and parameter attributes | `src/backends/cuda/{finite_element,affine_body,inter_primitive_effect_system}/` | `src/pybind/pyuipc/constitution/` | focused sim cases and `docs/specification/constitutions/` |
| Contact table | `include/uipc/core/contact_tabular.h` | `src/core/core/contact_tabular.cpp` | `src/backends/cuda/contact_system/`, collision/dytopo systems | `src/pybind/pyuipc/core/contact_tabular.cpp` | `apps/tests/core/contact_model.cpp`; samples `10_ramp_sliding`, `14_load_scene` |
| Subscene table | `include/uipc/core/subscene_tabular.h` | `src/core/core/subscene_tabular.cpp` | Collision filtering reads `subscene_element_id` | `src/pybind/pyuipc/core/subscene_tabular.cpp` | sample `29_subscene` |
| Animator | `include/uipc/core/animator.h` | `src/core/core/animator.cpp` | `src/backends/cuda/animator/` invokes callbacks per substep | `src/pybind/pyuipc/core/animator.cpp` | sample `3_periodically_pressed_tetrahedron` |
| Full/incremental scene IO | `include/uipc/io/scene_io.h`, `include/uipc/core/scene_snapshot.h` | `src/io/scene_io.cpp`, `src/core/core/scene_snapshot.cpp`, serializers in `src/core/` | No physics system owns the wire format | `src/pybind/pyuipc/core/{scene_io,scene_snapshot}.cpp` | `apps/tests/core/scene_io.cpp`; samples `14_load_scene`, `15_scene_commit` |
| Sanity checks | `include/uipc/core/sanity_checker.h` | `src/sanity_check/` is a dynamically loaded checker module | CUDA checkers may replace CPU checkers by ID | `src/pybind/pyuipc/core/sanity_checker.cpp` | `apps/tests/sanity_check/`, sample `35_mesh_check` |
| Backend features | `include/uipc/core/feature.h` and focused core/DiffSim feature headers | Feature handles live in the core collection | Implementations/overriders live beside the relevant CUDA subsystem | `src/pybind/pyuipc/backend/` | samples `20_contact_system_feature`, `31_state_accessor_feature`, `40_distance_diagnoser` |
| Python package helpers | `python/src/uipc/` | pure Python | Native backend remains lazily loaded | n/a | `python/tests/`, `libuipc-samples/` |

## Runtime Trace of One Frame

For the CUDA backend, the useful call chain is:

```text
World::advance
  -> internal::World::advance
    -> internal::Engine::advance
      -> backend::IEngine::advance
        -> cuda::SimEngine::do_advance
          -> rebuild actions and solve pending scene mutations
          -> update external attributes / animation / forces
          -> predict degrees of freedom
          -> IPC or AL-IPC advance pipeline
          -> update state and frame

World::retrieve
  -> sync first when needed
  -> cuda::SimEngine::do_retrieve
  -> registered write-scene actions copy current device state to Scene
```

The IPC orchestration is split across
`src/backends/cuda/engine/sim_engine_do_advance.cu` and
`src/backends/cuda/engine/advance_ipc.cu`. AL-IPC has a separate path in
`advance_al.cu`; do not infer parity merely because both paths satisfy the same
`World` API.

## Extension Recipes

### Add or change a scene config key

1. Register its type and default in `src/core/core/scene_default_config.cpp`.
2. Read it in the owning manager/system; search for neighboring keys rather than
   duplicating JSON parsing.
3. Define accepted values, units, range, and fallback behavior in
   `docs/specification/scene_config.md`.
4. Add a strict-config test (unknown keys intentionally throw) and a behavior test.
5. Keep C++ and Python examples synchronized if users set the key directly.

An entry existing only in the default schema is not an implemented behavior. The
current audit found `newton/use_adaptive_tol` registered but no consumer.

### Add a built-in attribute

1. Add the name through the X-macro list in
   `include/uipc/builtin/details/attribute_name.h`.
2. Create it at the geometry/constitution boundary with the correct collection
   (`meta`, `instances`, vertices, edges, triangles, or tetrahedra), type, size,
   and default.
3. Read it through the named built-in on the backend; avoid a second literal.
4. Bind/export the name in `src/pybind/pyuipc/builtin/` if it is user-facing.
5. Test creation, serialization, cloning, and backend consumption.

The surface flag is named `builtin::is_surf` / `uipc.builtin.is_surf`; there is
no `is_surface` built-in.

### Add a geometry algorithm

1. Put the public declaration in `include/uipc/geometry/utils/` when it belongs
   to the normal utility surface; otherwise use the focused geometry subdirectory.
2. Add the CPU implementation under `src/geometry/` and its build entries to both
   CMake and XMake where the existing glob/rule does not already cover it.
3. Add focused Catch2 coverage under `apps/tests/geometry/`.
4. Add a binding file and register it from `src/pybind/pyuipc/geometry/module.cpp`
   if Python users need it.
5. Decide explicitly whether `include/uipc/geometry/utils.h` should export it.

### Add a constitution or constraint

1. Choose a stable UID and add public registration metadata through
   `include/uipc/builtin/constitution_uid_collection.h` and the auto-register
   machinery. Do not reuse a UID unless the classes intentionally describe one
   backend family.
2. Implement `apply_to` in `src/constitution/`: validate user input, write the
   UID and every required parameter/state attribute, and define rest-state rules.
3. Implement and `REGISTER_SIM_SYSTEM` the CUDA reporter/receiver/energy system;
   make its dependencies explicit.
4. Add the pybind class and register it in the constitution module. Check the
   actual module export—public C++ does not imply Python parity.
5. Add a mathematical specification plus a small unit test and an end-to-end
   sim case. Update UID documentation manually until the generator limitation
   described in doc 04 is fixed.

### Add or replace a SimSystem

1. Derive from the appropriate common/CUDA system type and implement `do_build()`.
2. Use `require<T>` only for hard dependencies; use `find<T>` for optional
   capabilities. Invalid systems cause their strong dependents to be removed.
3. Register callbacks only while building, and place them at the event that matches
   the data lifetime (`on_init_scene`, `on_rebuild_scene`, or `on_write_scene`).
4. Add `REGISTER_SIM_SYSTEM(T)` in exactly one translation unit.
5. Verify the system appears in `engine.to_json()` and exercise missing-dependency
   behavior as well as the happy path.

### Extend a public C++ API into Python

1. Find the owning binding module under `src/pybind/pyuipc/<module>/`.
2. Add `PyXxx{module}` and invoke it from that directory's `module.cpp`.
3. Update the forwarding layer only when pure-Python behavior is needed; native
   submodules are already re-exported by `python/src/uipc/*.py`.
4. Regenerate/check stubs through the normal pybind build and add pytest coverage.
5. Test from the built/installed package, not only from the repository checkout.

### Change serialization

Treat the full SceneIO schema, incremental commit schema, clone behavior, and
Python JSON conversion as separate surfaces. Add round-trip tests for:

- config and object metadata;
- current and rest geometries, including attribute removal;
- contact and subscene table topology;
- geometry creation and destruction with non-contiguous IDs;
- a snapshot taken at a supported synchronization point.

Read doc 11 before relying on the incremental path; it is currently append/update
oriented and does not faithfully represent every Scene mutation.

## Verification Map

| Change | Minimum focused verification | Broader gate when risk warrants it |
|---|---|---|
| core lifecycle/config | `uipc_test_core`, strict config test | representative sim cases on both `ipc` and `al-ipc` |
| geometry/attributes/IO | owning `uipc_test_geometry` or `uipc_test_core` filter | serialization round trip + Python binding test |
| constitution/constraint | focused algebra/apply test + one sim case | isolated 95-case suite, then full single-process suite |
| CUDA system/kernel | owning CUDA test + smallest sim case | `scripts/run_sim_case_isolated.py`, full suite, sanitizer/profiler as appropriate |
| Python binding/helper | focused `pytest python/tests/...` | clean-wheel smoke that constructs `Engine("cuda", ...)` |
| docs/API pages | local link check + full MkDoxy build | GitHub docs workflow artifact/site smoke |
| build metadata/dependency | configure both CMake and XMake paths | Windows/Linux CI and wheel matrix if packaging is affected |

## Fast Search Patterns

Use these before broad browsing:

```bash
rg "REGISTER_SIM_SYSTEM" src/backends
rg "default_config|create_attribute" src/core/core/scene_default_config.cpp
rg "constitution_uid|UIDInfo" include/uipc src/constitution src/backends/cuda
rg "Py[A-Z].*\{" src/pybind/pyuipc
rg "TEST_CASE|SECTION" apps/tests
rg "on_init_scene|on_rebuild_scene|on_write_scene" src/backends/cuda
```

On PowerShell, prefer `rg -g '*.cpp' pattern apps/tests` over a shell-expanded
`apps/tests/*.cpp` glob.
