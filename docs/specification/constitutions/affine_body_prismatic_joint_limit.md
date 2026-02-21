# Affine Body Prismatic Joint Limit

## #669 AffineBodyPrismaticJointLimit

**Affine Body Prismatic Joint Limit** restricts how far two bodies can slide relative to each other along a [Prismatic Joint](./affine_body_prismatic_joint.md) axis. When the joint translation goes outside a specified lower or upper bound, a cubic penalty is applied. It is an InterAffineBody extra constitution defined on prismatic-joint geometry.

## Energy

The limit adds an energy $E(x)$ where $x$ is the scalar joint coordinate (its exact meaning is given in [Meaning of $x$](#meaning-of-x)). Let $l,u$ be lower/upper limits and $s$ be `limit/strength`.

For normal range width ($u>l$):

$$
E(x)=
\begin{cases}
s\left(\frac{x-u}{u-l}\right)^3, & x>u \\
0, & l \le x \le u \\
s\left(\frac{l-x}{u-l}\right)^3, & x<l
\end{cases}
$$

The normalized gaps are dimensionless, so changing $(l,u)$ does not require
retuning $s$ just because the interval width changed.

For degenerate limit ($u=l$), use fallback cubic:

$$
E(x)=s|x-l|^3=
\begin{cases}
s(x-l)^3, & x>l \\
0, & x=l \\
s(l-x)^3, & x<l
\end{cases}
$$

Boundary points are treated as in-range.

## Meaning of $x$

The prismatic coordinate is evaluated in incremental form:

$$
x=\theta^t+\delta,
$$

$$
\theta^t=\Delta\Theta(\mathbf{q}^{t},\mathbf{q}_{ref}), \quad
\delta=\Delta\Theta(\mathbf{q},\mathbf{q}^{t}).
$$

Here:

- $\mathbf{q}$ is the current affine-body DOF.
- $\mathbf{q}^{t}$ is the previous-time-step DOF.
- $\mathbf{q}_{ref}$ is the reference DOF captured at initialization.
- $\theta^t$ is the accumulated prismatic coordinate from the previous step.

Note that $\mathbf{q}$ is the concatenation of the DOF of the two affine bodies connected by the prismatic joint.

$$
\mathbf{q} =
\begin{bmatrix}
\mathbf{q}_i \\
\mathbf{q}_j
\end{bmatrix},
$$

The [Prismatic Joint](./affine_body_prismatic_joint.md) geometry is represented as an edge with two endpoints. The joint axis direction is defined by the order from $\mathbf{c}_i$ to $\mathbf{c}_j$, denoted $\mathbf{c}_i \to \mathbf{c}_j$, giving $+\hat{\mathbf{t}}$.

![Prismatic joint axis $\hat{\mathbf{t}}$](./media/external_articulation_prismatic_constraint_fig1.drawio.svg)

- $x=0$: current relative prismatic coordinate equals the reference coordinate.
- $x>0$: link $j$ translates relative to link $i$ along $+\hat{\mathbf{t}}$.
- $x<0$: link $j$ translates relative to link $i$ along $-\hat{\mathbf{t}}$.

Equivalent symmetric coordinate (same convention as [External Articulation Constraint](./external_articulation_constraint.md)):

$$
\theta=
\frac{
(\mathbf{c}_j-\mathbf{c}_i)\cdot\hat{\mathbf{t}}_i
-
(\mathbf{c}_i-\mathbf{c}_j)\cdot\hat{\mathbf{t}}_j
}{2}.
$$

## Requirement

This limit term is meaningful only on a geometry that already represents a
[Prismatic Joint](./affine_body_prismatic_joint.md) (UID=20). The limit augments that base relation as an extra constitution term.

## Attributes

On `edges`:

- `limit/lower`: $l$ in the energy above
- `limit/upper`: $u$ in the energy above
- `limit/strength`: $s$ in the energy above
