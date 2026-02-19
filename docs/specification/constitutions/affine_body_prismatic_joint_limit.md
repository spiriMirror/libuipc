# Affine Body Prismatic Joint Limit

## #669 AffineBodyPrismaticJointLimit

This constitution is an **InterAffineBody extra constitution** defined on a
prismatic-joint geometry. It contributes a unilateral cubic barrier-like penalty
to the joint scalar coordinate.

The coordinate is denoted by $x$ (unit: length), with lower/upper bounds
$l, u$ and penalty coefficient $s$:

$$
E(x)=
\begin{cases}
0, & l \le x \le u \\
s(x-u)^3, & x>u \\
s(l-x)^3, & x<l
\end{cases}
$$

Interpretation of each term:
- $x$: current prismatic relative coordinate of the joint.
- $l$: admissible lower bound.
- $u$: admissible upper bound.
- $s$: penalty strength (energy scale).

The scalar coordinate is evaluated in incremental form:

$$
x = \theta_0 + \delta,
$$
$$
\theta_0 = \Delta \Theta(\mathbf{q}^{t-1}, \mathbf{q}_{ref}), \quad
\delta = \Delta \Theta(\mathbf{q}, \mathbf{q}^{t-1}),
$$

where:
- $\mathbf{q}$ is the current affine-body DOF,
- $\mathbf{q}^{t-1}$ is the previous-step DOF,
- $\mathbf{q}_{ref}$ is the reference DOF captured at initialization.

So $\theta_0$ measures accumulated offset from the reference state, and
$\delta$ is the optimization-time increment in the current step.

First and second derivatives with respect to $x$:

$$
\frac{dE}{dx}=
\begin{cases}
0, & l \le x \le u \\
3s(x-u)^2, & x>u \\
-3s(l-x)^2, & x<l
\end{cases}
$$

$$
\frac{d^2E}{dx^2}=
\begin{cases}
0, & l \le x \le u \\
6s(x-u), & x>u \\
6s(l-x), & x<l
\end{cases}
$$

Boundary points ($x=l$ or $x=u$) are treated as in-range; energy and derivatives
are zero there.

## Conceptual Requirement

This limit term is meaningful only on a geometry that already represents a
prismatic joint relation (base joint UID = 20). The limit does not replace that
base relation; it augments it as an extra constitution term.

## Stored Attributes

Per joint edge:
- `limit/lower`
- `limit/upper`
- `limit/strength`

Geometry metadata:
- extra constitution UID `669` in `meta.extra_constitution_uids`.
