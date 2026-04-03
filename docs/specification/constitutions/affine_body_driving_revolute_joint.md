# Affine Body Driving Revolute Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #19 AffineBodyDrivingRevoluteJoint

The **Affine Body Driving Revolute Joint** is a constraint constitution that drives a [Revolute Joint](./affine_body_revolute_joint.md) (UID=18) to a target angle. It must be applied to a geometry that already has an `AffineBodyRevoluteJoint` constitution.

The driving joint supports two operating modes:

- **Active mode** (`is_passive = 0`): the joint drives toward a user-specified `aim_angle`.
- **Passive mode** (`is_passive = 1`): the joint resists external forces by treating the current angle as the target, effectively locking the joint in place.

The constraint can also be toggled on and off at runtime via the `is_constrained` flag.

## Energy

We assume 2 affine body indices $i$ and $j$, each with their own state vector $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The current relative rotation angle $\theta$ between the two bodies about the joint axis is extracted from the affine body states using the joint axis and normal rest frames, as defined in the [Revolute Joint](./affine_body_revolute_joint.md):

$$
\cos\theta = \frac{\hat{\mathbf{n}}_i \cdot \hat{\mathbf{n}}_j + \hat{\mathbf{b}}_i \cdot \hat{\mathbf{b}}_j}{2}, \quad
\sin\theta = \frac{\hat{\mathbf{n}}_i \cdot \hat{\mathbf{b}}_j - \hat{\mathbf{b}}_i \cdot \hat{\mathbf{n}}_j}{2}.
$$

where $\hat{\mathbf{n}}$ and $\hat{\mathbf{b}}$ are the normal and binormal directions in each body's frame.

The target angle $\tilde\theta$ is determined by:

$$
\tilde\theta =
\begin{cases}
\theta_{\text{aim}} + \theta_{\text{init}}, & \text{is\_passive} = 0 \\
\theta_{\text{current}} + \theta_{\text{init}}, & \text{is\_passive} = 1
\end{cases}
$$

where $\theta_{\text{init}}$ is the initial angle offset captured at the start of the simulation.

The energy function is:

$$
E = \frac{K}{2} \sin^2(\theta - \tilde\theta) = \frac{K}{2} \left(\sin\theta \cos\tilde\theta - \cos\theta \sin\tilde\theta\right)^2,
$$

where $K = \gamma (m_i + m_j)$, $\gamma$ is the **user defined** `strength_ratio` parameter, and $m_i$, $m_j$ are the masses of the two affine bodies.

When `is_constrained = 0`, the energy is zero and the driving effect is disabled.

## State Update

At the end of each time step, the time integrator updates the `angle` attribute to reflect the current joint angle:

$$
\theta_{\text{current}} = \text{map}_{[-\pi,\pi]}(\theta - \theta_{\text{init}})
$$

where $\text{map}_{[-\pi,\pi]}$ maps the angle into $(-\pi, \pi]$.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) (UID=18) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). The edge carries the same linking fields as [Affine Body Revolute Joint](./affine_body_revolute_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, base `strength_ratio` (if present), and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

Driving-specific attributes on **edges**:

- `driving/strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
- `is_constrained`: enables (`1`) or disables (`0`) the driving effect
- `is_passive`: passive mode (`1`) locks at the current angle; active mode (`0`) drives to `aim_angle`
- `aim_angle`: $\theta_{\text{aim}}$, the target angle in active mode
- `angle`: $\theta_{\text{current}}$, the current relative angle (updated by the backend each time step)
- `init_angle`: $\theta_{\text{init}}$, the initial angle offset
