# Affine Body Revolute Joint External Force

## #668 AffineBodyRevoluteJointExternalForce

The **Affine Body Revolute Joint External Force** applies a scalar external torque around the axis of a [Revolute Joint](./affine_body_revolute_joint.md). The torque drives rotational motion between the two connected affine bodies about the shared joint axis.

## Torque Application

Given a scalar torque $\tau$ and the joint axis direction $\hat{\mathbf{e}}$, the external torque is converted into a translational force at each body's center of mass. The torque produces equal-and-opposite tangential forces on the two bodies:

$$
\mathbf{F}_i = \begin{bmatrix} -\tau \, \dfrac{\hat{\mathbf{e}}_i \times \mathbf{r}_i}{\|\mathbf{r}_i\|^2} \\[6pt] \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12}, \quad
\mathbf{F}_j = \begin{bmatrix} +\tau \, \dfrac{\hat{\mathbf{e}}_j \times \mathbf{r}_j}{\|\mathbf{r}_j\|^2} \\[6pt] \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12},
$$

where:

- $\tau$ is the scalar torque magnitude (per edge)
- $\hat{\mathbf{e}}_k$ is the normalized joint axis direction in body $k$'s current frame
- $\mathbf{r}_k$ is the lever arm vector from the joint axis to body $k$'s center of mass, perpendicular to the axis
- $\mathbf{0}_9$ denotes the nine-dimensional zero vector (no affine force component)

### Axis Direction

The joint axis is defined by two points $\mathbf{x}^0$ and $\mathbf{x}^1$ on each body, following the [Revolute Joint](./affine_body_revolute_joint.md) convention. The axis direction in world space is:

$$
\hat{\mathbf{e}}_k = \frac{\mathring{\mathbf{J}}^{\hat{e}}_k \mathbf{q}_k}{\|\mathring{\mathbf{J}}^{\hat{e}}_k \mathbf{q}_k\|}, \quad k \in \{i, j\},
$$

where $\mathring{\mathbf{J}}^{\hat{e}}_k$ maps the rest-space axis direction through the current affine transform.

### Lever Arm

The lever arm $\mathbf{r}_k$ is the perpendicular distance from the joint axis to the center of mass of body $k$:

$$
\mathbf{d}_k = -\mathring{\mathbf{J}}^{\mathbf{x}^0_k} \mathbf{q}_k, \quad
\mathbf{r}_k = \mathbf{d}_k - (\mathbf{d}_k \cdot \hat{\mathbf{e}}_k) \hat{\mathbf{e}}_k,
$$

where $\mathbf{d}_k$ is the vector from the axis point to the center of mass in world space.

When $\|\mathbf{r}_k\|^2 < \epsilon$ (the center of mass lies on the axis), the force contribution for that body is zero.

### Sign Convention

A positive $\tau$ applies torque to body $j$ in the $+\hat{\mathbf{e}}$ direction (counterclockwise when viewed along $+\hat{\mathbf{e}}$, right-hand rule) and an equal-and-opposite reaction torque to body $i$.

## Energy Integration

The external forces are incorporated into each affine body's kinetic energy term through the predicted position $\tilde{\mathbf{q}}$, following the same mechanism as [AffineBodyExternalForce](./affine_body_external_force.md):

$$
E = \frac{1}{2} \left(\mathbf{q} - \tilde{\mathbf{q}}\right)^T \mathbf{M} \left(\mathbf{q} - \tilde{\mathbf{q}}\right),
$$

where $\tilde{\mathbf{q}}$ is updated each time step to include the acceleration from the external force:

$$
\mathbf{a}_{ext} = \mathbf{M}^{-1} \mathbf{F}_{ext}.
$$

## State Update

At the end of each time step, the time integrator computes the current joint angle:

$$
\theta_{\text{current}} = \theta(\mathbf{q}) - \theta_{\text{init}},
$$

where $\theta(\mathbf{q})$ is the revolute angle extracted from the current affine body states, and $\theta_{\text{init}}$ is the initial angle offset captured at the start of the simulation. The angle is mapped to the range $(-\pi, \pi]$. This value is written back to the `angle` attribute for user inspection via the Animator.

## Runtime Control

The torque can be updated at each frame through the Animator system:

- Set `external_torque/is_constrained` to `1` to enable the torque, or `0` to disable it.
- Modify the `external_torque` attribute to change the torque magnitude.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyRevoluteJoint](./affine_body_revolute_joint.md) (UID=18) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on **edges** (one edge per joint). The edge carries the same linking fields as [Affine Body Revolute Joint](./affine_body_revolute_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

External-force attributes on **edges**:

- `external_torque`: $\tau$ in the formulae above, scalar torque around the joint axis (one per edge)
- `external_torque/is_constrained`: enables (`1`) or disables (`0`) the external torque
- `angle`: $\theta_{\text{current}}$, the current joint angle in radians (updated by the backend each time step)
