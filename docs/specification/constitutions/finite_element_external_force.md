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

## Attributes

On `vertices`:

- `external_force`: $\mathbf{F}_{ext,i} \in \mathbb{R}^3$ per-vertex external force vector
- `is_constrained`: flag indicating whether the vertex is subject to external force ($1$ = enabled, $0$ = disabled)
