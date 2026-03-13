# Affine Body Driving Prismatic Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #21 AffineBodyDrivingPrismaticJoint

The **Affine Body Driving Prismatic Joint** is a constraint constitution that drives a [Prismatic Joint](./affine_body_prismatic_joint.md) (UID=20) to a target distance along its sliding axis. It must be applied to a geometry that already has an `AffineBodyPrismaticJoint` constitution.

The driving joint supports two operating modes:

- **Active mode** (`is_passive = 0`): the joint drives toward a user-specified `aim_distance`.
- **Passive mode** (`is_passive = 1`): the joint resists external forces by treating the current distance as the target, effectively locking the joint in place.

The constraint can also be toggled on and off at runtime via the `is_constrained` flag.

## Energy

We assume 2 affine body indices $i$ and $j$, each with their own state vector $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The joint axis is defined by the tangent direction $\hat{\mathbf{t}}$ and center positions $\mathbf{c}_i$, $\mathbf{c}_j$ as in the [Prismatic Joint](./affine_body_prismatic_joint.md).

The target distance $\tilde d$ is determined by:

$$
\tilde{d} =
\begin{cases}
d_{\text{aim}} + d_{\text{init}}, & \text{is\_passive} = 0 \\
d_{\text{current}} + d_{\text{init}}, & \text{is\_passive} = 1
\end{cases}
$$

where $d_{\text{init}}$ is the initial distance offset captured at the start of the simulation.

The energy function is:

$$
E = \frac{K}{2} \left( \hat{\mathbf{t}}_i \cdot (\mathbf{c}_j - \mathbf{c}_i) - \tilde{d} \right)^2
  + \frac{K}{2} \left( \hat{\mathbf{t}}_j \cdot (\mathbf{c}_i - \mathbf{c}_j) - \tilde{d} \right)^2,
$$

where $K = \gamma (m_i + m_j)$, $\gamma$ is the **user defined** `strength_ratio` parameter, and $m_i$, $m_j$ are the masses of the two affine bodies.

The two symmetric terms ensure that the constraint is evaluated from both bodies' perspectives.

When `is_constrained = 0`, the energy is zero and the driving effect is disabled.

## State Update

At the end of each time step, the time integrator updates the `distance` attribute to reflect the current joint displacement:

$$
d_{\text{current}} = d(\mathbf{q}) - d_{\text{init}}
$$

where $d(\mathbf{q})$ is the signed displacement along the joint axis extracted from the current affine body states.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyPrismaticJoint](./affine_body_prismatic_joint.md) (UID=20) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on `edges`:

- `driving/strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
- `is_constrained`: enables (`1`) or disables (`0`) the driving effect
- `is_passive`: passive mode (`1`) locks at the current distance; active mode (`0`) drives to `aim_distance`
- `aim_distance`: $d_{\text{aim}}$, the target distance in active mode
- `distance`: $d_{\text{current}}$, the current displacement (updated by the backend each time step)
- `init_distance`: $d_{\text{init}}$, the initial distance offset
