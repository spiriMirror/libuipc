# Scene Config

`Scene.default_config()` returns the full default configuration. Override any fields before passing it to the `Scene` constructor:

```python
config = Scene.default_config()
config['dt'] = 0.005
config['gravity'] = [[0.0], [0.0], [-9.8]]
config['contact']['d_hat'] = 0.005
scene = Scene(config)
```

## Global

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `dt` | Float | `0.01` | | Timestep in seconds |
| `gravity` | Vector3 | `[0, -9.8, 0]` | | Gravity acceleration ($m/s^2$) |

## CFL

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `cfl/enable` | Int | `0` | | Enable CFL-based step limiting |

## Integrator

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `integrator/type` | String | `"bdf1"` | `"bdf1"` `"bdf2"` | Time integrator |

## Newton Solver

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `newton/max_iter` | Int | `1024` | | Maximum Newton iterations per step |
| `newton/min_iter` | Int | `1` | | Minimum Newton iterations before early exit |
| `newton/use_adaptive_tol` | Int | `0` | | Enable adaptive convergence tolerance |
| `newton/velocity_tol` | Float | `0.05` | | Velocity tolerance ($m/s$); absolute displacement tolerance = `velocity_tol * dt` |
| `newton/ccd_tol` | Float | `1.0` | | CCD alpha convergence threshold; Newton continues while `ccd_alpha < ccd_tol` |
| `newton/transrate_tol` | Float | `0.1` | | Rotation-rate tolerance ($1/s$); absolute rotation tolerance = `transrate_tol * dt` |
| `newton/semi_implicit/enable` | Int | `0` | | Enable semi-implicit beta schedule |
| `newton/semi_implicit/beta_tol` | Float | `1e-3` | | Terminate semi-implicit when `beta <= beta_tol` |

See [Newton Solver Details](scene_configs/newton.md) for the convergence criteria and semi-implicit mode.

## Linear System

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `linear_system/tol_rate` | Float | `1e-3` | | Relative tolerance for PCG solver |
| `linear_system/solver` | String | `"fused_pcg"` | `"fused_pcg"` `"linear_pcg"` | Linear solver; `linear_pcg` is ~ 30% slower |

## Line Search

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `line_search/max_iter` | Int | `8` | | Maximum line search iterations |
| `line_search/report_energy` | Int | `0` | | Log energy at each line search step |

## Contact

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `contact/enable` | Int | `1` | | Enable contact handling |
| `contact/d_hat` | Float | `0.01` | | Barrier activation distance ($m$) |
| `contact/friction/enable` | Int | `1` | | Enable frictional contact |
| `contact/eps_velocity` | Float | `0.01` | | Friction regularization velocity ($m/s$) |
| `contact/constitution` | String | `"ipc"` | `"ipc"` `"al-ipc"` | Contact pipeline |

### IPC Adaptive Stiffness

Only takes effect when `contact/constitution = "ipc"` **and** the contact tabular contains a resistance set to `-1.0` (i.e. `adaptive(-1.0)`), which triggers the adaptive $\kappa$ mechanism.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `contact/adaptive/min_kappa` | Float | `1e8` | Minimum adaptive contact stiffness ($Pa$, 100 MPa) |
| `contact/adaptive/init_kappa` | Float | `1e9` | Initial adaptive contact stiffness ($Pa$, 1 GPa) |
| `contact/adaptive/max_kappa` | Float | `1e11` | Maximum adaptive contact stiffness ($Pa$, 100 GPa) |

### AL-IPC

Only takes effect when `contact/constitution = "al-ipc"`.

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `contact/al-ipc/mu_scale_fem` | Float | `5e7` | Penalty scale for FEM |
| `contact/al-ipc/mu_scale_abd` | Float | `1e5` | Penalty scale for affine bodies |
| `contact/al-ipc/toi_threshold` | Float | `0.1` | TOI threshold for active set |
| `contact/al-ipc/alpha_lower_bound` | Float | `1e-6` | Minimum step size |
| `contact/al-ipc/decay_factor` | Float | `0.3` | Active-set decay factor |

See [Contact Details](scene_configs/contact.md) for IPC vs AL-IPC pipeline selection and parameter tuning.

## Collision Detection

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `collision_detection/method` | String | `"info_stackless_bvh"` | `"info_stackless_bvh"` `"stackless_bvh"` `"linear_bvh"` | `info_stackless_bvh`: general-purpose; `stackless_bvh`: better when few collisions need culling; `linear_bvh`: slow, only for benchmarking |

## Sanity Check

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `sanity_check/backend` | String | `"cpu"` | `"none"` `"cpu"` `"cuda"` | Sanity check backend: `none` disables, `cpu` runs CPU checks, `cuda` runs CUDA-accelerated checks |
| `sanity_check/mode` | String | `"normal"` | `"normal"` `"quiet"` | `normal` writes diagnostic meshes on failure; `quiet` skips file output |

## Differentiable Simulation

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `diff_sim/enable` | Int | `0` | | Enable differentiable simulation |

## Extras

| Key | Type | Default | Option | Description |
|-----|------|---------|--------|-------------|
| `extras/debug/dump_surface` | Int | `0` | | Dump surface meshes during Newton solve |
| `extras/debug/dump_linear_system` | Int | `0` | | Dump linear system (matrix + RHS) |
| `extras/debug/dump_linear_pcg` | Int | `0` | | Dump per-PCG-iteration data (only for `linear_pcg` solver) |
| `extras/debug/dump_mas_matrices` | Int | `0` | | Dump MAS preconditioner matrices |
| `extras/strict_mode/enable` | Int | `0` | | Promote line-search / Newton max-iter warnings to exceptions |
