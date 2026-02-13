# Affine Body Revolute Joint Limit

## #670 AffineBodyRevoluteJointLimit

This constitution is an **InterAffineBody extra constitution** defined on a
revolute-joint geometry. It contributes a unilateral cubic penalty to the
relative angular coordinate.

Let $x$ be the revolute angle (unit: rad), with lower/upper bounds $l, u$ and
penalty coefficient $s$:

$$
E(x)=
\begin{cases}
0, & l \le x \le u \\
s(x-u)^3, & x>u \\
s(l-x)^3, & x<l
\end{cases}
$$

Interpretation of each term:
- $x$: current relative revolute angle around the joint axis.
- $l$: admissible lower angle bound.
- $u$: admissible upper angle bound.
- $s$: penalty strength (energy scale).

The angle is represented in incremental form:

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

This avoids direct optimization over inverse-trigonometric reconstruction of the
absolute angle and follows the same delta-theta idea used by external articulation.

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
revolute joint relation (base joint UID = 18). The limit augments that base
relation as an extra constitution term.

## Stored Attributes

Per joint edge:
- `limit/lower`
- `limit/upper`
- `limit/strength`

Geometry metadata:
- extra constitution UID `670` in `meta.extra_constitution_uids`.
