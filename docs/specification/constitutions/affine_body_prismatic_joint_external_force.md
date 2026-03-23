# Affine Body Prismatic Joint External Force

## #667 AffineBodyPrismaticJointExternalForce

The **Affine Body Prismatic Joint External Force** applies a scalar external force along the axis of a [Prismatic Joint](./affine_body_prismatic_joint.md). The force drives translational motion between the two connected affine bodies along the joint's tangent direction $\hat{\mathbf{t}}$.

## Force Application

Given a scalar force $f$ and the joint tangent direction $\hat{\mathbf{t}}$, the external force is applied as a 12D generalized force to each connected affine body:

$$
\mathbf{F}_i = \begin{bmatrix} +f \, \hat{\mathbf{t}}_i \\ \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12}, \quad
\mathbf{F}_j = \begin{bmatrix} -f \, \hat{\mathbf{t}}_j \\ \mathbf{0}_9 \end{bmatrix} \in \mathbb{R}^{12},
$$

where:

- $f$ is the scalar force magnitude (per edge)
- $\hat{\mathbf{t}}_i = \mathring{\mathbf{J}}^{\hat{t}}_i \mathbf{q}_i$ is the joint tangent direction in body $i$'s current frame
- $\hat{\mathbf{t}}_j = \mathring{\mathbf{J}}^{\hat{t}}_j \mathbf{q}_j$ is the joint tangent direction in body $j$'s current frame
- $\mathbf{0}_9$ denotes the nine-dimensional zero vector (no affine force component)

The tangent directions $\hat{\mathbf{t}}_i$ and $\hat{\mathbf{t}}_j$ are extracted from the current affine body states using the rest-space tangent coordinates, following the same conventions as the [Prismatic Joint](./affine_body_prismatic_joint.md).

A positive $f$ pushes body $i$ along $+\hat{\mathbf{t}}$ and body $j$ along $-\hat{\mathbf{t}}$, effectively driving the two bodies apart along the joint axis.

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

At the end of each time step, the time integrator computes the current displacement along the joint axis:

$$
d_{\text{current}} = d(\mathbf{q}) - d_{\text{init}},
$$

where $d(\mathbf{q})$ is the signed displacement along the joint axis computed from the current affine body states, and $d_{\text{init}}$ is the initial distance offset captured at the start of the simulation. This value is written back to the `distance` attribute for user inspection via the Animator.

## Runtime Control

The force can be updated at each frame through the Animator system:

- Set `external_force/is_constrained` to `1` to enable the force, or `0` to disable it.
- Modify the `external_force` attribute to change the force magnitude.

## Requirement

This constitution must be applied to a geometry that already has an [AffineBodyPrismaticJoint](./affine_body_prismatic_joint.md) (UID=20) constitution.

## Attributes

On the joint geometry (1D simplicial complex), on `edges`:

- `external_force`: $f$ in the formulae above, scalar force along the joint axis (one per edge)
- `external_force/is_constrained`: enables (`1`) or disables (`0`) the external force
- `distance`: $d_{\text{current}}$, the current displacement along the joint axis (updated by the backend each time step)
