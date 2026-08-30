# Scene Configuration Reference

`Scene::default_config()` / `Scene.default_config()` provides the values for a
simulation scene. `Scene::config_schema()` / `Scene.config_schema()` is the
machine-readable contract for every registered key: defaults, storage and JSON
types, units, hard constraints, lifecycle, implementation status, descriptions,
and source-level consumers. The tables below present the same contract for
humans. A single typed declaration creates both the runtime defaults and schema
metadata, preventing the two views from drifting as keys evolve. It lives in
[`scene_default_config.cpp`](https://github.com/spiriMirror/libuipc/blob/main/src/core/core/scene_default_config.cpp),
while effective-value rules and selector values were checked against the CUDA
backend that consumes them.

!!! important

    Scene configuration is strict: unknown keys, wrong storage types, non-finite
    values, unsupported selectors, and values outside the documented hard
    constraints are rejected. Validation runs when constructing a `Scene` and
    again in `World::init(scene)`, after any mutable `scene.config()` edits.

## Querying the contract

The schema is available without reading backend source and can be consumed by
tools, agents, editors, and configuration generators.

=== "C++"

    ```cpp
    auto schema = uipc::core::Scene::config_schema();
    auto dt = schema["entries"]["dt"];
    std::cout << dt.dump(2) << '\n';
    ```

=== "Python"

    ```python
    from uipc import Scene

    schema = Scene.config_schema()
    print(schema["entries"]["dt"])
    ```

The installed command-line interface can dump either the complete schema or a
single slash-separated key:

```bash
python -m uipc config-schema
python -m uipc config-schema contact/d_hat
```

## Creating and editing a configuration

The usual path is to copy the defaults, change only what the scene needs, and
then construct the scene.

=== "C++"

    ```cpp
    auto config = uipc::core::Scene::default_config();
    config["dt"] = 0.01;
    config["gravity"] = uipc::Vector3{0.0, -9.8, 0.0};
    config["contact"]["friction"]["enable"] = true;

    uipc::core::Scene scene{config};
    ```

=== "Python"

    ```python
    from uipc.core import Scene

    config = Scene.default_config()
    config["dt"] = 0.01
    config["gravity"] = [[0.0], [-9.8], [0.0]]
    config["contact"]["friction"]["enable"] = True

    scene = Scene(config)
    ```

A scene also exposes its registered configuration as attributes. This is
useful when a scene has already been created or loaded. Make these changes
**before** `world.init(scene)`: several backend systems cache configuration
values during initialization, so `scene.config()` is not a general hot-reload
interface. Call `scene.validate_config()` for an earlier explicit check;
`world.init(scene)` calls it automatically.

=== "C++"

    ```cpp
    auto config = scene.config();
    auto dt = config.find<uipc::Float>("dt");
    uipc::geometry::view(*dt)[0] = 0.02;
    ```

=== "Python"

    ```python
    from uipc import view

    dt = scene.config().find("dt")
    view(dt)[:] = 0.02
    ```

Do not use `scene.config().create(...)` as an application metadata store.
Unregistered values have no backend consumer; attach custom attributes to the
appropriate geometry instead.

## Types and units

- `flag` is stored as an integer (`0` or `1`); Python `False`/`True` is
  accepted.
- Length, time, velocity, acceleration, pressure, and density use SI units:
  m, s, m/s, m/s², Pa, and kg/m³.
- A vector is a three-component column vector. Python JSON therefore displays
  gravity as `[[x], [y], [z]]`.
- A selector must match one of the documented strings exactly. An unsupported
  selector normally leaves a required backend system unavailable and makes
  world initialization fail.

## Time integration

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `dt` | float, s | `0.01` | finite, `> 0` | Time represented by one call to `world.advance()`. |
| `gravity` | Vector3, m/s² | `[0, -9.8, 0]` | finite components | Global acceleration applied to dynamic bodies and FEM vertices. |
| `integrator/type` | string | `"bdf1"` | `"bdf1"`, `"bdf2"` | Backward differentiation formula used for time integration. Start with BDF1; BDF2 changes numerical damping and transient response. |
| `cfl/enable` | flag | `0` | `0`, `1` | Enables the contact-system CFL step filter. This is an advanced/experimental high-speed-contact control; leave it off unless a scene has been diagnosed to need it. |

## Newton solve

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `newton/max_iter` | integer | `1024` | `>= 1` | Maximum nonlinear iterations in one frame. Reaching it warns, or throws in strict mode. |
| `newton/min_iter` | integer | `0` | `0 <= value <= max_iter` | Hard floor before ordinary Newton convergence may terminate. `0` disables the floor. |
| `newton/use_adaptive_tol` | reserved integer | `0` | exactly `0` | Reserved for compatibility. Because no adaptive-tolerance consumer exists, setting it to `1` is rejected rather than silently doing nothing. |
| `newton/velocity_tol` | float, m/s | `0.05` | `> 0` when used | Absolute velocity tolerance. The displacement test is `max_axis_displacement <= velocity_tol * dt`. |
| `newton/velocity_tol_relative` | float | `0.0` | `> 0` enables; `<= 0` disables | Scene-relative override. Effective velocity tolerance becomes `value * rest_scene_bbox_diagonal`. |
| `newton/ccd_tol` | float | `1.0` | normally `(0, 1]` | Newton convergence additionally requires the latest CCD step fraction to be at least this value. |
| `newton/transrate_tol` | float, 1/s | `0.1` | `>= 0` | ABD transform-rate tolerance. The per-step threshold is `transrate_tol * dt`; irrelevant when no affine bodies exist. |
| `newton/semi_implicit/enable` | flag | `0` | `0`, `1` | Enables the semi-implicit beta termination criterion. |
| `newton/semi_implicit/beta_tol` | float | `1e-3` | normally `[0, 1]` | Semi-implicit early-exit threshold for accumulated beta. |
| `newton/semi_implicit/K_min` | integer | `1` | `>= 0` | Iteration at which beta accumulation starts. It is **not** a minimum Newton-iteration count. |

See [Newton and Linear Solvers](scene_configs/newton.md) for the exact
termination logic and tuning guidance.

## Linear system

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `linear_system/tol_rate` | float | `1e-3` | normally `(0, 1)` | Relative PCG residual tolerance. Smaller is more accurate and usually more expensive. |
| `linear_system/solver` | string | `"fused_pcg"` | `"fused_pcg"`, `"linear_pcg"` | Global iterative solver. `fused_pcg` is the optimized default; `linear_pcg` supports detailed PCG vector dumps. |
| `linear_system/fem_preconditioner` | string | `"diag"` | `"diag"`, `"mas"` | FEM local preconditioner. MAS auto-partitions every non-empty FEM geometry into fixed-size clusters and is intended for stiff/ill-conditioned FEM scenes. |
| `linear_system/use_cuda_graph` | integer mode | `1` | `0`, `1`, `2` | Fused-PCG launch mode: `0` plain launches; `1` host-checked block replay; `2` full-GPU while-loop graph. Mode 2 requires CUDA 12.4+ and falls back when unsupported. Non-IPC pipelines currently force graphs off. |
| `linear_system/check_interval` | integer | `5` | `>= 1` | Number of fused-PCG iterations between host convergence checks in modes 0/1. Larger values reduce checks but make exit granularity coarser. |

## Line search

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `line_search/max_iter` | integer | `8` | `>= 1` | Maximum backtracking iterations per Newton step. |
| `line_search/report_energy` | flag | `0` | `0`, `1` | Logs the energy contribution of every line-search reporter. Useful for diagnosis, noisy for normal runs. |

## Contact and friction

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `contact/enable` | flag | `1` | `0`, `1` | Builds contact detection and response systems. Disable for a deliberately contact-free scene. |
| `contact/d_hat` | float, m | `0.01` | `> 0` when used | Absolute IPC activation distance. |
| `contact/d_hat_relative` | float | `0.0` | `> 0` enables; `<= 0` disables | Overrides `d_hat` with `value * rest_scene_bbox_diagonal`. |
| `contact/friction/enable` | flag | `1` | `0`, `1` | Enables frictional contact terms. Normal non-penetration remains active when disabled. |
| `contact/eps_velocity` | float, m/s | `0.01` | `> 0` when used | Absolute friction transition velocity. |
| `contact/eps_velocity_relative` | float | `0.0` | `> 0` enables; `<= 0` disables | Overrides `eps_velocity` with `value * rest_scene_bbox_diagonal`. |
| `contact/constitution` | string | `"ipc"` | `"ipc"`, `"al-ipc"` | Selects the standard IPC or augmented-Lagrangian IPC pipeline. |

These global keys do not define pairwise material behavior. Friction and
contact resistance for geometry pairs are set through `ContactTabular`; see
[Contact and Collision](scene_configs/contact.md).

## AL-IPC parameters

The following keys are read only by the `"al-ipc"` pipeline. They are
algorithm parameters rather than material contact resistance.

| Key | Type | Default | Valid domain | Meaning |
| --- | --- | --- | --- | --- |
| `contact/al-ipc/mu_scale_fem` | float | `5e7` | `> 0` | Scales FEM augmented-Lagrangian penalty estimates. |
| `contact/al-ipc/mu_scale_abd` | float | `1e5` | `> 0` | Scales ABD penalty estimates; the estimator multiplies body mass, this value, and `dt²`. |
| `contact/al-ipc/toi_threshold` | float | `0.1` | normally `(0, 1]` | TOI threshold used by active-set handling. |
| `contact/al-ipc/alpha_lower_bound` | float | `1e-6` | normally `(0, 1]` | Lower bound for AL step length. |
| `contact/al-ipc/decay_factor` | float | `0.3` | normally `(0, 1)` | Penalty/constraint decay factor. |

## Adaptive contact resistance

| Key | Type | Default | Valid domain | Meaning |
| --- | --- | --- | --- | --- |
| `contact/adaptive/min_kappa` | float, Pa | `1e8` (100 MPa) | `> 0` | Lower fallback bound and the effective default resistance when the user never calls `default_model(...)`. |
| `contact/adaptive/init_kappa` | float, Pa | `1e9` (1 GPa) | `> 0` | Initial resistance used by the adaptive-kappa strategy. |
| `contact/adaptive/max_kappa` | float, Pa | `1e11` (100 GPa) | `> 0` | Upper fallback bound. Keep `min <= init <= max`. |
| `contact/adaptive/kappa_eval_scale` | float | `1e-16` | `> 0` | Evaluation scale for the scene-adaptive kappa corridor. This is an expert parameter; keep the default unless reproducing a calibrated method. |

If a user explicitly sets a non-negative default contact resistance, the
backend clamps it into `[min_kappa, max_kappa]` and reports the range. A
negative resistance is the explicit opt-in marker for adaptive kappa and is
not clamped. When a scene-derived corridor is computable, it takes precedence
over the configured fallback bounds.

## Collision detection, validation, differentiation, and diagnostics

| Key | Type | Default | Valid domain / choices | Meaning |
| --- | --- | --- | --- | --- |
| `collision_detection/method` | string | `"info_stackless_bvh"` | `"info_stackless_bvh"`; optionally `"info_stackless_bvh_v0"`, `"stackless_bvh"`, `"linear_bvh"` | Broad-phase trajectory filter. Keep the default unless benchmarking or diagnosing the broad phase. |
| `sanity_check/enable` | flag | `1` | `0`, `1` | Runs pre-initialization intersection and distance checks. A failed check makes the world invalid. |
| `sanity_check/mode` | string | `"normal"` | `"normal"`, `"quiet"` | `normal` also writes diagnostic geometry when a check fails; `quiet` reports the failure without exporting that geometry. |
| `diff_sim/enable` | flag | `0` | `0`, `1` | Initializes differentiable-simulation state. Calling non-const `scene.diff_sim()` sets this flag automatically; do so before world initialization. |
| `extras/debug/dump_surface` | flag | `0` | `0`, `1` | Dumps intermediate surface state during the nonlinear solve. Produces substantial output. |
| `extras/debug/dump_linear_system` | flag | `0` | `0`, `1` | Dumps assembled global linear systems for diagnosis. |
| `extras/debug/dump_linear_pcg` | flag | `0` | `0`, `1` | Dumps PCG vectors for `linear_pcg`. `fused_pcg` warns and ignores this option. |
| `extras/debug/dump_mas_matrices` | flag | `0` | `0`, `1` | Dumps MAS matrices when the MAS FEM preconditioner is active. |
| `extras/strict_mode/enable` | flag | `0` | `0`, `1` | Converts nonlinear/line-search limit warnings into engine errors. Recommended for automated validation, not exploratory tuning. |

The three alternate collision selectors are compiled only when
`UIPC_WITH_CUDA_LEGACY_COLLISION=ON` (CMake, the default) or
`cuda_legacy_collision=true` (XMake). Builds with the option disabled omit the
filter implementations and remove their names from the schema enum, so scene
construction rejects a serialized or manually edited unavailable selector.
The machine-readable entry exposes this state in `conditionalValues`.

## Effective-value precedence

Three pairs of absolute/relative controls follow the same pattern. Let `L` be
the diagonal of the rest-scene bounding box:

| Effective quantity | Rule |
| --- | --- |
| Newton velocity tolerance | `velocity_tol_relative > 0 ? velocity_tol_relative * L : velocity_tol` |
| Contact activation distance | `d_hat_relative > 0 ? d_hat_relative * L : d_hat` |
| Friction transition velocity | `eps_velocity_relative > 0 ? eps_velocity_relative * L : eps_velocity` |

Relative controls are useful when the same scene recipe is run at different
scales. Absolute controls are easier to reason about when mesh units are known
and stable. Do not enable both with the expectation that they are added: a
positive relative value overrides the absolute value.

## Source map

For audits and future documentation updates, the main implementation points
are:

- schema, defaults, and unknown-key rejection:
  [`src/core/core/scene_default_config.cpp`](https://github.com/spiriMirror/libuipc/blob/main/src/core/core/scene_default_config.cpp)
- scene construction and mutable config attributes:
  [`src/core/core/scene.cpp`](https://github.com/spiriMirror/libuipc/blob/main/src/core/core/scene.cpp)
- backend initialization and cached values:
  [`src/backends/cuda/engine/sim_engine_do_init.cu`](https://github.com/spiriMirror/libuipc/blob/main/src/backends/cuda/engine/sim_engine_do_init.cu)
- relative tolerances:
  [`max_translation_checker.cu`](https://github.com/spiriMirror/libuipc/blob/main/src/backends/cuda/newton_tolerance/max_translation_checker.cu)
  and
  [`global_contact_manager.cu`](https://github.com/spiriMirror/libuipc/blob/main/src/backends/cuda/contact_system/global_contact_manager.cu)
- solver modes:
  [`linear_fused_pcg.cu`](https://github.com/spiriMirror/libuipc/blob/main/src/backends/cuda/linear_system/linear_fused_pcg.cu)
