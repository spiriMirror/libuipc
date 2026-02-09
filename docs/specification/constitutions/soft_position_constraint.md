# Soft Position Constraint

**Soft Position Constraint** is a type of constraint to control the motion of vertices in finite element simulations, which can be applied to all the vertex-based soft bodies.

## #14 SoftPositionConstraint

For a vertex in a finite element simulation at position $\mathbf{x}$ with vertex effective mass $m$, the soft position constraint energy is defined as:

$$
E = \frac{1}{2} \eta m \|\mathbf{x} - \hat{\mathbf{x}}\|^2
$$

where:

- $\hat{\mathbf{x}}$ is the target (aim) position for the vertex
- $\eta \in (0, +\infty)$ is the strength ratio parameter
- $m$ is the vertex effective mass

The soft position constraint energy has the form of "Kinetic" energy, similar to [SoftTransformConstraint](./soft_transform_constraint.md) but for finite element vertices instead of affine bodies.

The reason we use strength ratio is that the mass $m$ already contains the mass information, so that users only need to care about how strong the constraint is compared to the mass of the vertex, which is more intuitive.

Because the constraint is "SOFT", we don't allow a too-strong strength which will lead to numerical instability. Normally, $\eta$ is better in the range of $[0, 100]$.

