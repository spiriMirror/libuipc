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

These only take effect when `contact/constitution = "al-ipc"`.

| Key | Default | Role |
|-----|---------|------|
| `mu_scale_fem` | `5e7` | Initial penalty stiffness scale for FEM bodies |
| `mu_scale_abd` | `1e5` | Initial penalty stiffness scale for affine bodies |
| `toi_threshold` | `0.1` | Time-of-impact fraction below which a contact pair enters the active set |
| `alpha_lower_bound` | `1e-6` | Minimum allowed step size in the AL line search |
| `decay_factor` | `0.3` | Shrink factor for the active-set envelope each AL outer iteration |

## Adaptive Contact Stiffness (`adaptive/*`)

The GIPC adaptive strategy adjusts contact stiffness $\kappa$ each Newton step to balance convergence speed and contact accuracy.

| Key | Default | Role |
|-----|---------|------|
| `min_kappa` | 100 MPa (`1e8`) | Lower bound — prevents $\kappa$ from dropping too low |
| `init_kappa` | 1 GPa (`1e9`) | Starting value at the first Newton step |
| `max_kappa` | 100 GPa (`1e11`) | Upper bound — caps stiffness to avoid ill-conditioning |

If contacts are too soft (visible penetration), increase `init_kappa` or `min_kappa`. If the solver struggles to converge, try lowering `max_kappa`.
