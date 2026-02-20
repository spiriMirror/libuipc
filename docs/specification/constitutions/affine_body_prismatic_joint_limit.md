# Affine Body Prismatic Joint Limit

## #669 AffineBodyPrismaticJointLimit

This constitution is an **InterAffineBody extra constitution** defined on a
prismatic-joint geometry. It adds a unilateral cubic penalty to the scalar
joint coordinate.

## Meaning of `x` 

The prismatic coordinate is evaluated in incremental form:

$$
x=\theta^t+\delta,
$$

$$
\theta^t=\Delta\Theta(\mathbf{q}^{t-1},\mathbf{q}_{ref}), \quad
\delta=\Delta\Theta(\mathbf{q},\mathbf{q}^{t-1}).
$$

Here:
- $\mathbf{q}$ is the current affine-body DOF.
- $\mathbf{q}^{t-1}$ is the previous-step DOF.
- $\mathbf{q}_{ref}$ is the reference DOF captured at initialization.
- $\theta^t$ is the accumulated prismatic coordinate from the previous step.

The joint axis direction is defined by edge order `p0 -> p1`, i.e. $+\hat{\mathbf{t}}$.

- $x=0$: current relative prismatic coordinate equals the reference coordinate.
- $x>0$: link $j$ translates relative to link $i$ along $+\hat{\mathbf{t}}$.
- $x<0$: link $j$ translates relative to link $i$ along $-\hat{\mathbf{t}}$.

Equivalent symmetric coordinate (same convention as external articulation):

$$
\theta=
\frac{
(\mathbf{c}_j-\mathbf{c}_i)\cdot\hat{\mathbf{t}}_i
-
(\mathbf{c}_i-\mathbf{c}_j)\cdot\hat{\mathbf{t}}_j
}{2}.
$$

## Energy

Let $l,u$ be lower/upper limits and $s$ be `limit/strength`.

For normal range width ($u>l$):

$$
E(x)=
\begin{cases}
0, & l \le x \le u \\
s\left(\frac{x-u}{u-l}\right)^3, & x>u \\
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

## Conceptual Requirement

This limit term is meaningful only on a geometry that already represents a
prismatic joint relation (base joint UID = 20). The limit augments that base
relation as an extra constitution term.

## Stored Attributes

Per joint edge:
- `limit/lower`
- `limit/upper`
- `limit/strength`

Geometry metadata:
- extra constitution UID `669` in `meta.extra_constitution_uids`.
