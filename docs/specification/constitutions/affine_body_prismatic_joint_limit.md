# Affine Body Prismatic Joint Limit

## #669 AffineBodyPrismaticJointLimit

**Affine Body Prismatic Joint Limit** restricts how far two bodies can slide relative to each other along a [Prismatic Joint](./affine_body_prismatic_joint.md) axis. When the joint translation goes outside a specified lower or upper bound, a cubic penalty is applied. It is an InterAffineBody extra constitution defined on prismatic-joint geometry.

## Energy

`limit/lower` and `limit/upper` are **absolute bounds on the reported `distance` attribute** (i.e. $d_{\text{current}}$ as defined by the base [Prismatic Joint](./affine_body_prismatic_joint.md#distance-state)). The user can set them directly against the value they read back from `distance`.

Internally the penalty is evaluated on the scalar coordinate $x$, whose
meaning is the raw joint coordinate $d(\mathbf{q})$ defined by the base
[Prismatic Joint](./affine_body_prismatic_joint.md#distance-state). The
reported distance is

$$
d_{\text{current}} = x + d_0,
$$

so the effective bounds on $x$ are shifted by $-d_0$:

$$
l = l_{\text{user}} - d_0, \quad u = u_{\text{user}} - d_0,
$$

where $l_{\text{user}}, u_{\text{user}}$ are `limit/lower`, `limit/upper` and $d_0$ is `init_distance` from the base joint. Algebraically this is equivalent to enforcing $l_{\text{user}} \le d_{\text{current}} \le u_{\text{user}}$. Let $s$ be `limit/strength`.

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

The coordinate $x$ used by the limit is the same raw joint coordinate
$d(\mathbf{q})$ as the base
[Prismatic Joint](./affine_body_prismatic_joint.md#distance-state):

$$
x = d(\mathbf{q}) =
\frac{
(\mathbf{c}_j(\mathbf{q})-\mathbf{c}_i(\mathbf{q}))
\cdot
(\hat{\mathbf{t}}_i(\mathbf{q})+\hat{\mathbf{t}}_j(\mathbf{q}))
}{2}.
$$

Here $\mathbf{c}_i(\mathbf{q})$ and $\mathbf{c}_j(\mathbf{q})$ are the current
world-space joint anchor positions on the two affine bodies, and
$\hat{\mathbf{t}}_i(\mathbf{q})$, $\hat{\mathbf{t}}_j(\mathbf{q})$ are the
current world-space prismatic axes. Note that $\mathbf{q}$ is the
concatenation of the DOF of the two affine bodies connected by the prismatic
joint.

$$
\mathbf{q} =
\begin{bmatrix}
\mathbf{q}_i \\
\mathbf{q}_j
\end{bmatrix},
$$

The [Prismatic Joint](./affine_body_prismatic_joint.md) geometry is represented as an edge with two endpoints. The joint axis direction is defined by the order from $\mathbf{c}_i$ to $\mathbf{c}_j$, denoted $\mathbf{c}_i \to \mathbf{c}_j$, giving $+\hat{\mathbf{t}}$.

![Prismatic joint axis $\hat{\mathbf{t}}$](./media/external_articulation_prismatic_constraint_fig1.drawio.svg)

- $x=0$: the two joint anchors have zero signed separation along the averaged
  prismatic axis.
- $x>0$: link $j$ is ahead of link $i$ along $+\hat{\mathbf{t}}$.
- $x<0$: link $j$ is behind link $i$ along $+\hat{\mathbf{t}}$.

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

The host geometry is **edge-based** (one edge per joint), with the same linking fields as [Affine Body Prismatic Joint](./affine_body_prismatic_joint.md): `l_geo_id`, `r_geo_id`, `l_inst_id`, `r_inst_id`, `strength_ratio`, and optional `l_position0`, `l_position1`, `r_position0`, `r_position1` when created via Local `create_geometry`.

On **edges** (in addition to the base joint attributes above):

- `limit/lower`: $l_{\text{user}}$ — absolute lower bound on the reported `distance` (default `0.0`)
- `limit/upper`: $u_{\text{user}}$ — absolute upper bound on the reported `distance` (default `0.0`)
- `limit/strength`: $s$ — penalty strength (default `1.0`)

The limit also consumes the base-joint attribute `init_distance` ($d_0$, default `0.0`); it is **subtracted** from `limit/lower`/`limit/upper` to form the effective bounds $l$/$u$ used on $x=d(\mathbf{q})$. Equivalently, the penalty enforces the user-facing reported distance $d_{\text{current}} = x + d_0$.
