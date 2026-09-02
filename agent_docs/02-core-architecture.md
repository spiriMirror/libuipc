# 02 — Core Architecture (core module and backend plugin mechanism)

## Three-Layer Structure

```
core::Engine / World / Scene          ← public handle layer (holds only S<internal::X>, pure forwarding)
  → core::internal::Engine / World / Scene   ← lifecycle and logic layer
    → backend::IEngine (virtual interface)   ← implemented by backend dynamic libraries
```

- Public layer: `include/uipc/core/{engine,world,scene}.h`, implemented in `src/core/core/{engine,world,scene}.cpp`
- internal layer: `include/uipc/core/internal/*.h` + `src/core/core/internal/*.cpp`
- Backend interface: `include/uipc/core/i_engine.h`

## Backend Module Loading (Plugin ABI)

`Engine::Impl::load_module(backend_name)` in `src/core/core/internal/engine.cpp`:

1. Uses `dylib` to load the `uipc_backend_<name>` dynamic library from `uipc::config()["module_dir"]` (static cache `m_cache` + mutex; a module with the same name is loaded only once).
2. Calls **`uipc_query_module(UIPCBackendModuleInfo*)`** before any backend code. The loader requires the current backend ABI, the same libuipc major/minor version, and an identity matching the requested backend name. Missing query support is reported as an old/non-libuipc module instead of failing later at an arbitrary virtual call.
3. Calls **`uipc_init_module(UIPCModuleInitInfo*)`** (defined in `include/uipc/backend/module_init_info.h`): passes the host's `std::pmr::get_default_resource()` to the backend and calls `set_default_resource`, ensuring consistent pmr container allocators across DLLs.
4. Calls **`uipc_create_engine(EngineCreateInfo*)`** to create the `IEngine*`; **`uipc_destroy_engine`** serves as the deleter. `EngineCreateInfo{ workspace, Json config }` (`include/uipc/backend/engine_create_info.h`).
5. The four exported symbols are declared in `src/backends/common/module.h`. The common module source implements query/init; each backend's `entrance.cpp` implements create/destroy (cuda: `new cuda::SimEngine(info)`; none: `new none::SimEngine(info)`). CMake and XMake both build the same runtime-loadable shared-library form in test and packaged configurations.

Custom engines can also be injected: the `Engine(S<IEngine> overrider)` constructor (used by pyuipc and similar scenarios) does not load any module.

Other details:
- A C++ process must call `uipc::init` with an existing `module_dir` before constructing a named backend Engine. The default process config leaves this path empty. The Python package performs this step automatically and points it at `uipc/_native`.
- The normal named-backend constructor converts the workspace to an absolute path and calls `create_directory` when it is absent (only the final directory level, not recursive parent creation). The explicit `IEngine` overrider constructor does not normalize or create the workspace.
- `advance()` clears the sync flag; `retrieve()` automatically calls `sync()`
  first if not yet synced and marks the wrapper synchronized. Repeated
  `retrieve()` calls therefore do not repeat the backend synchronization.
- `Engine::default_config()` can be overridden by the environment variable `UIPC_ENGINE_DEFAULT_CONFIG` (pointing to a JSON file); the active default is `gpu/device=0`. It also still carries the stale key `extras/gui/enable=true` even though the C++ GUI has been removed; no current backend consumes it.
- Every backend call is wrapped in `LogPatternGuard{backend_name}` to switch the log prefix.

## IEngine Template Method

`include/uipc/core/i_engine.h`: public non-virtual `init/advance/sync/retrieve/to_json/dump/recover/frame/status/features/insert_sanity_checkers` → protected pure virtual `do_init/do_advance/do_sync/do_retrieve/get_frame/get_status/get_features`; `do_to_json/do_dump/do_recover/do_insert_sanity_checkers` have empty default implementations.

## World Lifecycle (`src/core/core/internal/world.cpp`)

Holds `S<internal::Scene>`, `W<internal::Engine>` (weak, to prevent circular references), `S<SanityChecker>`, `m_valid`.

**`World::init(Scene&)` in four steps**:
1. record the scene (return immediately if already initialized);
2. when `sanity_check/enable` in the config is true, create a `SanityChecker` to run the checks; on failure set `m_valid=false` and skip init;
3. `m_scene->init(*this)`;
4. `engine->init(*this)`; afterwards check `engine->status().has_error()`; on error the world becomes invalid.

`advance/sync/retrieve/dump`: each step first checks `m_valid`, and after the call checks `EngineStatusCollection::has_error()`; on error the world is **permanently invalidated** and an error is logged.
`recover(aim_frame)`: after success, if `diff_sim.parameters().size() > 0` then `parameters().broadcast()`.

## internal::Scene (`src/core/core/internal/scene.cpp`)

- `init(world)`: records the world raw pointer; `m_constitution_tabular.init(*this)` scans all geometry metas to collect UIDs; when `diff_sim/enable`, calls `m_diff_sim.init(*this)`; sets `m_started=true`.
- **Pending mechanism**: after the world has started, geometry additions/removals do not take effect immediately; they go into `m_pending_create/m_pending_destroy` (`GeometryCollection`). The CUDA backend calls `begin_pending()` during initialization and `solve_pending()` in each rebuild phase. The `none` backend does neither, so post-init additions remain pending there. `Scene::Objects::destroy` chooses between direct destroy and pending destroy based on `is_started()/is_pending()`.
- `dt()` is read from the config attribute `"dt"`.
- Calling non-const `Scene::diff_sim()` sets `diff_sim/enable=1`; merely requesting the mutable handle changes Scene configuration.
- `update_from(const SceneSnapshotCommit&)` applies config, objects, contact and
  subscene tables, independent current/rest geometry commits, removals, exact
  sparse IDs, and allocator next-ID state. The commit remains relative to its
  compatible full baseline rather than a standalone Scene file; see doc 11.

## Scene Default Configuration (`src/core/core/scene_default_config.cpp`)

Configuration is stored in an `AttributeCollection` (resize(1)). Key entries:
- `dt=0.01`, `gravity`, `cfl/enable`
- `integrator/type="bdf1"`
- `newton/*` (max_iter, velocity_tol use unit literals such as `0.05_m/1.0_s`; on the standard IPC path, `semi_implicit/{enable,beta_tol,K_min}` = Stiff-GIPC-style early exit, enabled by default with `beta_tol=1e-3` and `K_min=6`: from iteration `K_min` on, β←(1-α)β per iteration, exit when β≤beta_tol. AL-IPC keeps its distinct cumulative-safe-path termination and does not consume these three keys. `newton/min_iter` is a pure hard floor, default 0 = no forced minimum — normal convergence may exit at any iteration)
- `linear_system/*`: `solver="fused_pcg"`; `fem_preconditioner` = `"diag"` (default) | `"mas"` (MAS, auto-partitions all FEM geometries internally, fixed cluster size 16); `use_cuda_graph` (0/1/2, PCG graph replay); `check_interval` (host convergence check cadence)
- `line_search/*`
- `contact/enable`, `contact/d_hat`, `contact/constitution="ipc"` (or `"al-ipc"` and its tuning parameters, including conditioning-aware `mu_scale_mode="diag_norm"` by default); `contact/d_hat_relative` (when >0, d_hat = relative value × scene diagonal, the Stiff-GIPC convention), `newton/velocity_tol_relative` (when >0, exit threshold = relative value × diagonal × dt), `contact/eps_velocity_relative` (when >0, friction C1 smoothing threshold = relative value × diagonal × dt) — scene-adaptive parameters, default 0 (disabled); `contact/adaptive/{min_kappa,max_kappa,init_kappa}` bound the effective contact stiffness
- `collision_detection/*` (`method`: `info_stackless_bvh` default; the V0, stackless, and linear-BVH selectors are schema-visible only when the matching legacy CUDA component was built), `sanity_check/*`, `diff_sim/enable`

**Config contract**: a single `make_scene_config_contract()` declaration creates
both the typed `AttributeCollection` defaults and the metadata for all 48 keys.
`Scene::default_config()` and `Scene::config_schema()` consume that same result,
so a default cannot be added or changed independently of its public schema.
`Scene::config_schema()` returns defaults, JSON/storage types, units, hard
bounds/enums, lifecycle, status, descriptions, and source consumers.
`from_config_json` rejects unknown keys and
validates constraints during construction; `World::init(Scene&)` validates the
mutable attribute collection again. Adding, destroying, or changing a config
attribute through `Scene::config()` therefore cannot silently bypass the
contract. `newton/use_adaptive_tol` remains visible for compatibility but is
reserved at exactly `0` until a real consumer exists. Contact/Subscene per-model
`config` arguments accept only `{}` while no extension keys are implemented.

**Default kappa policy**: the scene-adaptive corridor (Stiff-GIPC `suggestKappa`/`upperBoundKappa` ported with the $/dt^2$ conversion, computed at `GlobalContactManager::init` from scene diagonal, mean mass and `d_hat`) rules all clamping; `contact/adaptive/{min,max}_kappa` are only the fallback when the corridor is not computable. If `default_model(...)` is never called, the effective default stiffness is the corridor lower bound; a user-set value is clamped into the corridor with a range-printing warning; negative kappa (adaptive-kappa opt-in marker) is never clamped. The evaluation scale is configurable via `contact/adaptive/kappa_eval_scale` (default `1e-16`, Stiff's value; the $/dt^2$ conversion makes it dt-independent).

See `docs/specification/scene_config.md` for the full key table.

## RMR (Reporter-Manager-Receiver) and the SimSystem Hierarchy

Defined in `src/backends/common/`; class hierarchy:

```
core::IEngine
 └── backend::SimEngine            (common/sim_engine.h)
      ├── cuda::SimEngine          (cuda/sim_engine.h + engine/*.cu)
      └── none::SimEngine          (none/)

backend::ISimSystem                 (common/i_sim_system.h)
 └── backend::SimSystem            (common/sim_system.h)
      └── cuda::SimSystem          (cuda/sim_system.h)
           ├── Manager (GlobalVertexManager, GlobalContactManager, GlobalLinearSystem...)
           ├── Reporter / Subsystem (Animator, LineSearchReporter, DiagLinearSubsystem...)
           └── Receiver (DyTopoEffectReceiver, ContactReceiver...)
```

Key points:
- **`ISimSystem`**: `name/is_valid/is_engine_aware/is_building`, strong/weak dependencies (`strong_dependencies/weak_dependencies`), `to_json`; dump/recover lifecycle `dump/try_recover/apply_recover/clear_recover` mapped to `do_*` virtual functions; `BaseInfo{frame, workspace, config}`.
- **`backend::SimSystem`**: holds a `SimEngine&`; `find<T>(QueryOptions)/require<T>` look up other systems in the `SimSystemCollection`; on failure `require` throws `SimSystemException` → this system is judged invalid during build.
- **`cuda::SimSystem`**: adds event registration `on_init_scene/on_rebuild_scene/on_write_scene` (may only be called inside `do_build()`; the actions are stored in the engine's `SimActionCollection`); `check_state(SimEngineState)` asserts the pipeline stage.
- **`SimSystemCollection`** (`common/sim_system_collection.cpp`): exact lookup is keyed by collision-safe `std::type_index`, while a separate registration-order vector controls all iteration, JSON, build, and invalidation order. `build_systems()` first calls `set_building(true)` on all systems → builds them one by one (internally calling `do_build()`) → caught exceptions mark a system invalid → `cleanup_invalid_systems()` **cascades the deletion of systems whose strong dependencies have become invalid** (weak dependencies are unaffected). The remaining strong-dependency graph must be acyclic; a cycle aborts initialization with its complete `A -> B -> A` path.
- **Override mechanism**: `QueryOptions{exact}`; `exact=false` allows returning a derived-type system — i.e. replacing base-class functionality with a derived class. Compatible lookup follows deterministic registration order and skips implementations already invalidated by a selector or missing dependency. There are also `*FeatureOverrider` classes (e.g. `affine_body_state_accessor_feature.cu`) that override features via `engine.features()`.
- **Auto-registration**: the `REGISTER_SIM_SYSTEM(T)` macro (at the end of `common/sim_system.h`) registers a named creator at static-init time via `SimSystemAutoRegister`; `SimEngine::build_systems()` sorts creators by their complete demangled type name before instantiation. The creator's constructor parameter type determines which engine it belongs to (cuda systems take `cuda::SimEngine&` as constructor parameter, verified via dynamic_cast). Systems must not use source/link/static-initialization order as an implicit priority.
- **SimSystemSlot<T> / SimSystemSlotCollection<T>** (`common/sim_system_slot.h`): lazily-bound slots; `register_sim_system(T&)` hooks a system into the manager's slot; `lazy_init()` confirms the binding on first access. Managers use SlotCollection to gather all reporters/subsystems.

## Peripheral Subsystems (within core)

- **SanityCheck** (`src/sanity_check/`): scene validation before init (built-in checkers such as surface distance/intersection, half-plane distance); tests in `apps/tests/sanity_check/`.
- **DiffSim** (`include/uipc/diff_sim/`): parameter set `ParameterCollection`; `broadcast()` is triggered after recover; GPU side in `src/backends/cuda/diff_sim/`.
- **SceneSnapshot/Commit**: incremental Scene change representation used by
  `SceneIO::commit/update`. It is lossless relative to the exact compatible
  baseline, including subscene changes, removals, sparse IDs, and allocator
  state. It is not a standalone archive or a schema-negotiating network
  protocol (doc 11).

## Logging, Errors, and Infrastructure (`include/uipc/common/`)

- `Logger` (spdlog wrapper), `Timer` (scoped timing, nested into a tree, outputs `timer_frames.json`), `Json` (nlohmann), `unit.h` (literals such as `1.0_GPa`), `UIPC_ASSERT` / `UIPC_ASSERT_THROW` (fast-fail convention: the former for internal invariants, the latter for user input), `Exception`, smart pointer aliases `S<T>=shared_ptr`, `U<T>=unique_ptr`, `W<T>=weak_ptr`.
