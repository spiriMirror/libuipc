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
2. Calls the exported symbol **`uipc_init_module(UIPCModuleInitInfo*)`** (defined in `include/uipc/backend/module_init_info.h`): passes the host's `std::pmr::get_default_resource()` to the backend and calls `set_default_resource`, ensuring consistent pmr container allocators across DLLs.
3. Calls **`uipc_create_engine(EngineCreateInfo*)`** to create the `IEngine*`; **`uipc_destroy_engine`** serves as the deleter. `EngineCreateInfo{ workspace, Json config }` (`include/uipc/backend/engine_create_info.h`).
4. The three exported symbols are declared in `src/backends/common/module.h`; each backend implements them in `entrance.cpp` (cuda: `new cuda::SimEngine(info)`; none: `new none::SimEngine(info)`).

Custom engines can also be injected: the `Engine(S<IEngine> overrider)` constructor (used by pyuipc and similar scenarios) does not load any module.

Other details:
- The workspace is converted to an absolute path at construction time and created if it does not exist.
- `advance()` clears the sync flag; `retrieve()` automatically calls `sync()` first if not yet synced.
- `Engine::default_config()` can be overridden by the environment variable `UIPC_ENGINE_DEFAULT_CONFIG` (pointing to a JSON file); defaults are `gpu/device=0` and gui enable.
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
- **Pending mechanism**: after the world has started, geometry additions/removals do not take effect immediately; they go into `m_pending_create/m_pending_destroy` (`GeometryCollection`) and are settled together by `solve_pending()`. `Scene::Objects::destroy` chooses between direct destroy and pending_destroy based on `is_started()/is_pending()`.
- `dt()` is read from the config attribute `"dt"`.
- `update_from(const SceneSnapshotCommit&)`: applies an incremental commit to update config/objects/contact_tabular/geometries/rest_geometries.

## Scene Default Configuration (`src/core/core/scene_default_config.cpp`)

Configuration is stored in an `AttributeCollection` (resize(1)). Key entries:
- `dt=0.01`, `gravity`, `cfl/enable`
- `integrator/type="bdf1"`
- `newton/*` (max_iter, velocity_tol use unit literals such as `0.05_m/1.0_s`; `semi_implicit/{enable,beta_tol,K_min}` = Stiff-GIPC-style early exit: from iteration `K_min` on, β←(1-α)β per iteration, exit when β≤beta_tol. `newton/min_iter` is a pure hard floor, default 0 = no forced minimum — normal convergence may exit at any iteration)
- `linear_system/*`: `solver="fused_pcg"`; `fem_preconditioner` = `"diag"` (default) | `"mas"` (MAS, auto-partitions all FEM geometries internally, fixed cluster size 16); `use_cuda_graph` (0/1/2, PCG graph replay); `check_interval` (host convergence check cadence)
- `line_search/*`
- `contact/enable`, `contact/d_hat`, `contact/constitution="ipc"` (or `"al-ipc"` and its tuning parameters); `contact/d_hat_relative` (when >0, d_hat = relative value × scene diagonal, the Stiff-GIPC convention), `newton/velocity_tol_relative` (when >0, exit threshold = relative value × diagonal × dt), `contact/eps_velocity_relative` (when >0, friction C1 smoothing threshold = relative value × diagonal × dt) — scene-adaptive parameters, default 0 (disabled); `contact/adaptive/{min_kappa,max_kappa,init_kappa}` bound the effective contact stiffness
- `collision_detection/*` (`method`: `info_stackless_bvh` default, `info_stackless_bvh_v0`), `sanity_check/*`, `diff_sim/enable`

**Config strictness**: `from_config_json` recursively validates the user json against the registered defaults and **throws a guided error on unregistered keys** (they were silently dropped before). Two consequences: (1) adding a new config key REQUIRES registering its default in `scene_default_config.cpp`; (2) old scenes with typo/dead keys now fail loudly — fix the key, don't bypass the check.

**Default kappa policy**: if `default_model(...)` is never called, the effective default contact stiffness is `contact/adaptive/min_kappa` (default 1e8); a user-set value is clamped into [min_kappa, max_kappa] with a range-printing warning; negative kappa (adaptive-kappa opt-in marker) is never clamped. See handoff "Default kappa policy".

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
- **`SimSystemCollection`** (`common/sim_system_collection.cpp`): keyed by `typeid(s).hash_code()`; `build_systems()` first calls `set_building(true)` on all systems → builds them one by one (internally calling `do_build()`) → caught exceptions mark a system invalid → `cleanup_invalid_systems()` **cascades the deletion of systems whose strong dependencies have become invalid** (weak dependencies are unaffected).
- **Override mechanism**: `QueryOptions{exact}`; `exact=false` allows returning a derived-type system — i.e. replacing base-class functionality with a derived class. There are also `*FeatureOverrider` classes (e.g. `affine_body_state_accessor_feature.cu`) that override features via `engine.features()`.
- **Auto-registration**: the `REGISTER_SIM_SYSTEM(T)` macro (at the end of `common/sim_system.h`) registers a creator at static-init time via `SimSystemAutoRegister`; `SimEngine::build_systems()` iterates over the creators to instantiate the systems. The creator's constructor parameter type determines which engine it belongs to (cuda systems take `cuda::SimEngine&` as constructor parameter, verified via dynamic_cast).
- **SimSystemSlot<T> / SimSystemSlotCollection<T>** (`common/sim_system_slot.h`): lazily-bound slots; `register_sim_system(T&)` hooks a system into the manager's slot; `lazy_init()` confirms the binding on first access. Managers use SlotCollection to gather all reporters/subsystems.

## Peripheral Subsystems (within core)

- **SanityCheck** (`src/sanity_check/`): scene validation before init (built-in checkers such as surface distance/intersection, half-plane distance); tests in `apps/tests/sanity_check/`.
- **DiffSim** (`include/uipc/diff_sim/`): parameter set `ParameterCollection`; `broadcast()` is triggered after recover; GPU side in `src/backends/cuda/diff_sim/`.
- **SceneSnapshot/Commit**: incremental scene serialization (`SceneIO`'s `commit/update`), used for GUI/network synchronization.

## Logging, Errors, and Infrastructure (`include/uipc/common/`)

- `Logger` (spdlog wrapper), `Timer` (scoped timing, nested into a tree, outputs `timer_frames.json`), `Json` (nlohmann), `unit.h` (literals such as `1.0_GPa`), `UIPC_ASSERT` / `UIPC_ASSERT_THROW` (fast-fail convention: the former for internal invariants, the latter for user input), `Exception`, smart pointer aliases `S<T>=shared_ptr`, `U<T>=unique_ptr`, `W<T>=weak_ptr`.
