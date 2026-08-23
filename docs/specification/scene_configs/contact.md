# Contact Config Details

## Barrier Distance (`d_hat`)

`d_hat` (in meters) is the distance at which the IPC barrier potential activates. Two surfaces closer than `d_hat` experience a repulsive force that grows to infinity at zero distance, preventing interpenetration.

- Larger `d_hat` creates a wider "safety cushion" — more robust but slightly less tight contacts.
- Smaller `d_hat` gives tighter contacts but requires more Newton iterations to resolve near-contact configurations.
- A good starting point is the same order as the smallest feature size in your mesh (default `0.01` = 1 cm).

## Friction (`eps_velocity`)

`eps_velocity` is the velocity threshold ($m/s$) for friction regularization. Below this velocity, static friction transitions smoothly to dynamic friction via a $C^1$ mollifier. Smaller values give sharper stick-slip transitions but can slow convergence.

## IPC vs AL-IPC (`constitution`)

| Value | Pipeline | When to use |
|-------|----------|-------------|
| `"ipc"` | Standard IPC with log-barrier | General-purpose. Robust and well-tested. |
| `"al-ipc"` | Augmented-Lagrangian IPC | Designed for real-time applications; generally more efficient than standard IPC. |

When `constitution = "ipc"`, all `al-ipc/*` parameters are ignored.

## AL-IPC Parameters

Original paper: [Robust and Efficient Penetration-Free Elastodynamics without Barriers](https://arxiv.org/pdf/2512.12151)

These only take effect when `contact/constitution = "al-ipc"`.

| Key | Default | Role |
|-----|---------|------|
| `mu_scale_fem` | `5e7` | Initial penalty stiffness scale for FEM bodies |
| `mu_scale_abd` | `1e5` | Initial penalty stiffness scale for affine bodies |
| `toi_threshold` | `0.1` | Time-of-impact fraction below which a contact pair enters the active set |
| `alpha_lower_bound` | `1e-6` | Minimum allowed step size in the AL line search |
| `decay_factor` | `0.3` | Shrink factor for the active-set envelope each AL outer iteration |

## Adaptive Contact Stiffness (`adaptive/*`)

Reference repository: [Stiff-GIPC](https://github.com/KemengHuang/Stiff-GIPC)

The GIPC adaptive strategy adjusts contact stiffness $\kappa$ each Newton step to balance convergence speed and contact accuracy.

At init the engine computes a **scene-adaptive $\kappa$ corridor** (a port of Stiff-GIPC's `suggestKappa`/`upperBoundKappa`, with the $dt^2$ conversion — see the [scene config reference](../scene_config.md#ipc-adaptive-stiffness) for the formula): $[\kappa_\text{suggest},\, 100 \times \kappa_\text{suggest}]$ from the scene diagonal, mean mass and $d_\text{hat}$. **The corridor rules**: the default-model resolution, per-model clamps and the adaptive projection are all clamped into it. The config values below are only the fallback when the corridor is not computable.

| Key | Default | Role |
|-----|---------|------|
| `min_kappa` | 100 MPa (`1e8`) | Fallback lower bound (used only when the corridor is not computable) |
| `init_kappa` | 1 GPa (`1e9`) | Starting value at the first Newton step |
| `max_kappa` | 100 GPa (`1e11`) | Fallback upper bound (used only when the corridor is not computable) |
| `kappa_eval_scale` | `1e-16` | Evaluation-point scale of the corridor (Stiff's raw-$\kappa$ value; the $/dt^2$ conversion makes it valid for any `dt`) |

If contacts are too soft (visible penetration), increase `init_kappa`. If the solver struggles to converge, check the computed corridor in the log (`Adaptive kappa corridor: [...]`) — it is adapted to the scene's mass and size.
