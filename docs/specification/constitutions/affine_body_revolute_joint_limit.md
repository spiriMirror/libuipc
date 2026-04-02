# Affine Body Revolute Joint Limit

## #670 AffineBodyRevoluteJointLimit

**Affine Body Revolute Joint Limit** restricts the rotation angle of a [Revolute Joint](./affine_body_revolute_joint.md) to a specified range. When the joint angle goes beyond the lower or upper bound, a cubic penalty is applied. It is an InterAffineBody extra constitution defined on revolute-joint geometry.

## Energy

The limit adds an energy $E(x)$ where $x$ is the scalar joint angle (its exact meaning is given in [Meaning of $x$](#meaning-of-x)). The effective bounds are computed from the user-specified limits and the initial angle:

$$
l = l_{\text{user}} + \alpha_0, \quad u = u_{\text{user}} + \alpha_0,
$$

where $l_{\text{user}},u_{\text{user}}$ are `limit/lower`, `limit/upper` and $\alpha_0$ is `init_angle`. Let $s$ be `limit/strength`.

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

The revolute coordinate is evaluated in incremental form:

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
- $\theta^t$ is the accumulated revolute angle from the previous step.

Note that $\mathbf{q}$ is the concatenation of the DOF of the two affine bodies connected by the revolute joint.

$$
\mathbf{q} =
\begin{bmatrix}
\mathbf{q}_i \\
\mathbf{q}_j
\end{bmatrix},
$$

The [Revolute Joint](./affine_body_revolute_joint.md) geometry is represented as an edge with two endpoints. The joint axis direction is defined by the order of the two endpoints giving $+\hat{\mathbf{t}}$.

![Revolute joint axis $\hat{\mathbf{t}}$](./media/external_articulation_revolute_constraint_fig1.drawio.svg)

- $x=0$: current relative revolute angle equals the reference angle.
- $x>0$: positive rotation around $+\hat{\mathbf{t}}$ (right-hand rule; counterclockwise when viewed along $+\hat{\mathbf{t}}$).
- $x<0$: negative rotation around $+\hat{\mathbf{t}}$ (clockwise when viewed along $+\hat{\mathbf{t}}$).

For [Revolute Joints](./affine_body_revolute_joint.md):

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

## Requirement

This limit term is meaningful only on a geometry that already represents a
[Revolute Joint](./affine_body_revolute_joint.md) (UID=18). The limit augments that base relation as an extra constitution term.

## Attributes

The host geometry is **edge-based** (one edge per joint), with the same linking fields as [Affine Body Revolute Joint](./affine_body_revolute_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

On **edges** (in addition to the base joint attributes above):

- `limit/lower`: $l_{\text{user}}$ — relative lower bound (default `0.0`)
- `limit/upper`: $u_{\text{user}}$ — relative upper bound (default `0.0`)
- `limit/strength`: $s$ — penalty strength (default `1.0`)
- `init_angle`: $\alpha_0$ — initial revolute angle offset (default `0.0`). Added to `limit/lower` and `limit/upper` to obtain the effective bounds $l$ and $u$ used in the energy
