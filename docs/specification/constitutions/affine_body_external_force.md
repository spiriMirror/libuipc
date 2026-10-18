# External Force for Affine Body

## #666 AffineBodyExternalForce

The **Affine Body External Force** applies external 12D generalized forces to affine body instances through a constraint mechanism, supporting both translational forces and affine (rotational) forces.

The external force is applied as acceleration:

$$
\mathbf{a}_{ext} = \mathbf{M}^{-1} \mathbf{F}_{ext}
$$

where:

- $\mathbf{F}_{ext} \in \mathbb{R}^{12}$ is the generalized force vector
- $\mathbf{M} \in \mathbb{R}^{12 \times 12}$ is the affine body mass matrix
- $\mathbf{a}_{ext} \in \mathbb{R}^{12}$ is the resulting acceleration

The force vector is structured as:

$$
\mathbf{F}_{ext} = \begin{bmatrix} \mathbf{f} \\ \mathbf{f}_a \end{bmatrix} \in \mathbb{R}^{12}
$$

where:

- $\mathbf{f} \in \mathbb{R}^3$ is the translational force
- $\mathbf{f}_a \in \mathbb{R}^{9}$ is the affine force (flattened 3x3 matrix)

The energy function for the **Affine Body External Force** will be incorporated into the Affine Body Kinetic Term as:

$$
E = \frac{1}{2} \left(\mathbf{q} - \tilde{\mathbf{q}}\right)^T \mathbf{M} \left(\mathbf{q} - \tilde{\mathbf{q}}\right) 
$$

where $\tilde{\mathbf{q}}$ is updated each time step by the velocity and acceleration, the formula depends on the time integration scheme used.

Typically, when `bdf1` (implicit euler) is used:

$$
\tilde{\mathbf{q}} = \mathbf{q}^t + \Delta t \cdot \mathbf{v}^t + \Delta t^2 \cdot (\mathbf{a}_{ext} + \mathbf{g}),
$$

where $\mathbf{q}^t$, is the previous state vector at time $t$, $\mathbf{v}^t$ is the previous velocity vector at time $t$, $\Delta t$ is the time step size, and $\mathbf{g}$ is the gravitational acceleration.

Users are allowed to modify the external force $\mathbf{F}_{ext}$ using the `uipc` Animator System interface during the simulation.

## Activation and animation

`apply_to(sc, force, enable=true)` in C++, or `apply_to(sc, force, enable=True)`
in Python, sets the force uniformly and **enables it by default** on all
instances. Pass `false` / `False` to store a force for later activation.
Each call resets the force and `external_force/is_constrained` on all current
instances; edit their attributes directly for per-instance selection.

The Animator can change `external_force` and `external_force/is_constrained`
before each step: `1` enables a force and `0` disables it. The flag persists
until changed; disabling every force clears the backend's active force buffers.
The dedicated flag is independent of `is_constrained`, which controls
`SoftTransformConstraint`. Applying or toggling a force does not activate that
soft constraint, regardless of which constitution was applied first.

### Migration from the shared flag

Code that calls `apply_to` and then toggles `instances.is_constrained` to control
external forces must instead toggle `instances["external_force/is_constrained"]`.
For delayed activation, pass `enable=false` / `enable=False` or clear the new
mask before initialization. `apply_to` no longer creates or modifies the shared
`is_constrained` attribute.

Older serialized geometries lacking the dedicated attribute retain their
original behavior: the backend reads their shared `is_constrained` flag. If both
attributes exist, **only the dedicated force flag** controls the force, including
when it is zero. Reapplying the constitution upgrades a geometry to the new
layout and resets its force mask from `enable`; it does not copy a legacy mask.

## Attributes

On `instances`:

- `external_force`: $\mathbf{F}_{ext}$ in the formulae above
- `external_force/is_constrained`: integer enable flag ($1$ = enabled, $0$ = disabled), set from `enable` (default `true`) by `apply_to`
