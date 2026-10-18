# External Force for Finite Element

## #671 FiniteElementExternalForce

The **Finite Element External Force** applies external 3D forces to individual vertices of finite element meshes through a constraint mechanism, enabling per-vertex force control for effects such as wind, applied loads, or user-driven interactions.

The external force is applied as acceleration per vertex:

$$
\mathbf{a}_{ext,i} = \frac{\mathbf{F}_{ext,i}}{m_i}
$$

where:

- $\mathbf{F}_{ext,i} \in \mathbb{R}^{3}$ is the external force vector at vertex $i$
- $m_i \in \mathbb{R}$ is the lumped mass at vertex $i$
- $\mathbf{a}_{ext,i} \in \mathbb{R}^{3}$ is the resulting acceleration at vertex $i$

The energy function for the **Finite Element External Force** will be incorporated into the FEM Kinetic Term as:

$$
E = \frac{1}{2} \sum_i m_i \left\|\mathbf{x}_i - \tilde{\mathbf{x}}_i\right\|^2
$$

where $\tilde{\mathbf{x}}_i$ is the predicted position updated each time step by the velocity, gravity, and external acceleration. The exact formula depends on the time integration scheme used.

Typically, when `bdf1` (implicit Euler) is used:

$$
\tilde{\mathbf{x}}_i = \mathbf{x}_i^t + \Delta t \cdot \mathbf{v}_i^t + \Delta t^2 \cdot (\mathbf{a}_{ext,i} + \mathbf{g}),
$$

where $\mathbf{x}_i^t$ is the position of vertex $i$ at time $t$, $\mathbf{v}_i^t$ is the velocity at time $t$, $\Delta t$ is the time step size, and $\mathbf{g}$ is the gravitational acceleration.

When `bdf2` is used, the external acceleration $\mathbf{a}_{ext,i}$ is combined with gravity before being passed to the BDF2 predictor:

$$
\mathbf{g}_{\text{eff},i} = \mathbf{g} + \mathbf{a}_{ext,i}
$$

Users are allowed to modify the external force $\mathbf{F}_{ext,i}$ per vertex using the `uipc` Animator System interface during the simulation.

## Activation and animation

`apply_to(sc, force, enable=true)` in C++, or `apply_to(sc, force, enable=True)`
in Python, sets the force uniformly and **enables it by default** on all
vertices. Pass `false` / `False` to store a force for later activation.
Each call resets the force and `external_force/is_constrained` on all current
vertices; edit their attributes directly for per-vertex selection.

The Animator can change `external_force` and `external_force/is_constrained`
before each step: `1` enables a force and `0` disables it. The flag persists
until changed; disabling every force clears the backend's active force buffers.
The dedicated flag is independent of `is_constrained`, which controls
`SoftPositionConstraint`. Applying or toggling a force does not activate that
soft constraint, regardless of which constitution was applied first.

### Migration from the shared flag

Code that calls `apply_to` and then toggles `vertices.is_constrained` to control
external forces must instead toggle `vertices["external_force/is_constrained"]`.
For delayed activation, pass `enable=false` / `enable=False` or clear the new
mask before initialization. `apply_to` no longer creates or modifies the shared
`is_constrained` attribute.

Older serialized geometries lacking the dedicated attribute retain their
original behavior: the backend reads their shared `is_constrained` flag. If both
attributes exist, **only the dedicated force flag** controls the force, including
when it is zero. Reapplying the constitution upgrades a geometry to the new
layout and resets its force mask from `enable`; it does not copy a legacy mask.

## Attributes

On `vertices`:

- `external_force`: $\mathbf{F}_{ext,i} \in \mathbb{R}^3$ per-vertex external force vector
- `external_force/is_constrained`: integer enable flag ($1$ = enabled, $0$ = disabled), set from `enable` (default `true`) by `apply_to`
