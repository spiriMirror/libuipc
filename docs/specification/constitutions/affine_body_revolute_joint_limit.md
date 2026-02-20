# Affine Body Revolute Joint Limit

## #670 AffineBodyRevoluteJointLimit

This constitution is an **InterAffineBody extra constitution** defined on a
revolute-joint geometry. It adds a unilateral cubic penalty to the relative
joint angle.

## Meaning of `x` 

The revolute coordinate is evaluated in incremental form:

$$
x=\theta_0+\delta,
$$

$$
\theta_0=\Delta\Theta(\mathbf{q}^{t-1},\mathbf{q}_{ref}), \quad
\delta=\Delta\Theta(\mathbf{q},\mathbf{q}^{t-1}).
$$

Here:
- $\mathbf{q}$ is the current affine-body DOF.
- $\mathbf{q}^{t-1}$ is the previous-step DOF.
- $\mathbf{q}_{ref}$ is the reference DOF captured at initialization.

The joint axis direction is defined by edge order `p0 -> p1`, i.e. $+\hat{\mathbf{t}}$.

- $x=0$: current relative revolute angle equals the reference angle.
- $x>0$: positive rotation around $+\hat{\mathbf{t}}$ (right-hand rule; counterclockwise when viewed along $+\hat{\mathbf{t}}$).
- $x<0$: negative rotation around $+\hat{\mathbf{t}}$ (clockwise when viewed along $+\hat{\mathbf{t}}$).

For revolute joints:

$$
\Delta\Theta(\mathbf{q}_a,\mathbf{q}_b)=
\operatorname{atan2}
\left(
\sin\theta_a\cos\theta_b-\cos\theta_a\sin\theta_b,\;
\cos\theta_a\cos\theta_b+\sin\theta_a\sin\theta_b
\right),
$$

$$
\cos\theta=\frac{\hat{\mathbf{b}}_i\cdot\hat{\mathbf{b}}_j+\hat{\mathbf{n}}_i\cdot\hat{\mathbf{n}}_j}{2},
\quad
\sin\theta=\frac{\hat{\mathbf{n}}_i\cdot\hat{\mathbf{b}}_j-\hat{\mathbf{b}}_i\cdot\hat{\mathbf{n}}_j}{2}.
$$

The sign of $x$ follows the sign of $\sin\theta$ under this convention.
The angle branch is $(-\pi,\pi]$.

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
revolute joint relation (base joint UID = 18). The limit augments that base
relation as an extra constitution term.

## Stored Attributes

Per joint edge:
- `limit/lower`
- `limit/upper`
- `limit/strength`

Geometry metadata:
- extra constitution UID `670` in `meta.extra_constitution_uids`.
