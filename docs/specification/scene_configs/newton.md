# Newton and Linear Solvers

This page explains how the nonlinear and linear stopping criteria fit
together. See [Scene Configuration Reference](../scene_config.md) for the
complete key table and defaults.

The ordinary and IPC semi-implicit criteria below describe the standard `ipc`
pipeline. `al-ipc` shares the tolerance check inside its fixed-active-set inner
solve and consumes `semi_implicit/enable` plus `K_min`, but has a separate
safe-path termination tolerance described below.

## One frame, from outside to inside

For each `world.advance()`, the CUDA backend performs a nonlinear solve. A
Newton iteration assembles the global system, solves it with PCG, limits the
step for contact, performs backtracking line search, and then asks every active
tolerance checker whether it converged.

Ordinary termination requires all applicable checks to pass:

1. animation targets have been reached;
2. the maximum per-axis displacement is below the effective velocity
   tolerance multiplied by `dt`;
3. ABD transform-rate tolerance passes when affine bodies are present;
4. the last CCD step fraction is at least `newton/ccd_tol`; and
5. `newton/min_iter` has been reached.

The maximum-displacement threshold is

$$
\Delta x_{\mathrm{tol}} = \Delta t
\begin{cases}
r_v L, & r_v > 0,\\
v_{\mathrm{abs}}, & r_v \le 0,
\end{cases}
$$

where $r_v$ is `newton/velocity_tol_relative`, $L$ is the rest-scene bounding
box diagonal, and $v_{\mathrm{abs}}$ is `newton/velocity_tol`.

## `min_iter` and semi-implicit `K_min` are different

These two parameters are easy to confuse:

- `newton/min_iter` is a hard floor on ordinary Newton termination.
- `newton/semi_implicit/K_min` only controls when the beta accumulator starts
  updating in semi-implicit mode.

Semi-implicit termination is enabled by default with `K_min = 6` and
`beta_tol = 1e-3`. Set `newton/semi_implicit/enable = 0` to disable this
additional early-exit criterion.

With semi-implicit mode enabled, iterations at or after `K_min` update

$$
\beta \leftarrow (1 - \alpha)\beta,
$$

where $\alpha$ is the accepted line-search step. `beta <= beta_tol` is an
additional early-exit condition. It does not force the solver to run until
`K_min`; use `newton/min_iter` if a true iteration floor is required.

## AL-IPC termination

AL-IPC retains a possibly penetrating optimization iterate while advancing a
separate collision-free state by CCD. Let `w` be the remaining weight of the
initial collision-free state. It starts at one and, with semi-implicit mode
enabled, stays at one for the first `K_min - 1` completed outer updates. From
the `K_min`-th update onward,

$$
w \leftarrow (1 - \alpha_{\mathrm{CCD}})w.
$$

AL-IPC terminates when `w <= contact/al-ipc/toi_threshold` and animation plus
`newton/min_iter` conditions pass. Therefore its default `K_min = 6` prevents
the one-step termination that otherwise changes a stiff scene's transient
trajectory. `beta_tol` remains IPC-only. If semi-implicit mode is disabled, or
if `K_min < 1`, AL-IPC uses an effective `K_min` of one.

An AL CCD fraction is counted only when it is finite, clamped into `[0, 1]`,
and greater than `contact/al-ipc/alpha_lower_bound`. A rejected tiny step does
not create false termination progress.

AL line search evaluates the final configured backtracking trial as well as
the earlier ones. If every trial fails, it restores the exact recorded start
point and ends that frame as non-converged; strict mode throws. This recovery
prevents an energy-increasing trial from turning the next CCD query and active
set into invalid data.

## Linear solve

`linear_system/tol_rate` compares the preconditioned residual against its
initial value. Smaller values request a tighter solve, but an unnecessarily
tight PCG solve may not improve the outer Newton result.

The supported solvers are:

| Solver | Use it when |
| --- | --- |
| `fused_pcg` | Normal simulation. It is the optimized default and supports CUDA graph replay. |
| `linear_pcg` | Diagnosing PCG internals or using `extras/debug/dump_linear_pcg`. It is slower. |

The FEM local preconditioner is independent of the global solver selector:

| Preconditioner | Behavior |
| --- | --- |
| `diag` | 3x3 block-Jacobi. Low setup cost and a good default for small/moderately conditioned FEM scenes. |
| `mas` | Multi-level Additive Schwarz. Every non-`Empty` FEM geometry is auto-partitioned internally with cluster size 16; no per-mesh opt-in tag is needed. Use for stiff, large, or poorly conditioned FEM/cloth scenes after measuring. |

MAS affects FEM degrees of freedom only. ABD and other subsystems keep their
own preconditioners, so mixed ABD/FEM scenes can enable MAS safely for the FEM
portion.

## CUDA graph modes

`linear_system/use_cuda_graph` is a mode, despite its historical name:

| Mode | Execution |
| --- | --- |
| `0` | Plain per-iteration kernel launches. |
| `1` | Replays blocks of `check_interval` iterations, then checks convergence on the host. This is the measured default. |
| `2` | Uses a CUDA conditional while-loop graph (CUDA 12.4+). It removes the CPU from the loop but is not necessarily faster. It falls back when unavailable. |

Graph replay is currently gated to the standard `ipc` pipeline. Selecting
`al-ipc` forces the plain path. MAS may also fall back to plain launches if a
captured operation is unsupported.

## A conservative tuning order

When a frame is slow or fails to converge, change one layer at a time:

1. Check mesh validity, units, material parameters, initial intersections, and
   `dt`. Solver tolerances cannot repair an invalid scene.
2. Inspect whether the limit is Newton (`newton/max_iter`), line search, or
   PCG. Enable reporting only long enough to identify the layer.
3. For a stiff FEM-dominated scene, measure `fem_preconditioner = "mas"`.
4. Tighten `linear_system/tol_rate` only if an inaccurate inner solve is
   blocking Newton progress.
5. Adjust Newton tolerances in the scene's units, or use the relative controls
   for scale-independent recipes.
6. Increase iteration limits only after understanding why the current limit is
   reached.

For CI and regression scenes, enable `extras/strict_mode/enable` so an exhausted
Newton or line-search budget becomes a failure instead of a warning.

## Example: a scale-relative stiff-FEM profile

This profile mirrors the controls used by the stiff FEM and cloth samples. It
is a starting point, not a universal preset.

=== "C++"

    ```cpp
    auto config = uipc::core::Scene::default_config();
    config["dt"] = 0.01;
    config["newton"]["velocity_tol_relative"] = 1e-2;
    config["contact"]["d_hat_relative"] = 1e-3;
    config["contact"]["eps_velocity_relative"] = 1e-2;
    config["linear_system"]["tol_rate"] = 1e-4;
    config["linear_system"]["fem_preconditioner"] = "mas";

    uipc::core::Scene scene{config};
    ```

=== "Python"

    ```python
    config = Scene.default_config()
    config["dt"] = 0.01
    config["newton"]["velocity_tol_relative"] = 1e-2
    config["contact"]["d_hat_relative"] = 1e-3
    config["contact"]["eps_velocity_relative"] = 1e-2
    config["linear_system"]["tol_rate"] = 1e-4
    config["linear_system"]["fem_preconditioner"] = "mas"

    scene = Scene(config)
    ```

Keep `newton/use_adaptive_tol` at its default. It remains registered for schema
compatibility, but because no CUDA consumer exists, the validator rejects any
value other than `0` instead of accepting a silent no-op.
