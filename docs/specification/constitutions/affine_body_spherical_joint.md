# Affine Body Spherical Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

Related: [Affine Body](./affine_body.md) (state and Jacobian $\mathbf{J}$), [Affine Body Fixed Joint](./affine_body_fixed_joint.md) (full rigid weld; spherical joint keeps only the translation part of that formulation).

## #26 AffineBodySphericalJoint

The **Affine Body Spherical Joint** (ball-and-socket joint) constrains two affine bodies so that a single anchor point has the same world position on both bodies, while **relative rotation is unconstrained**. Only three translational degrees of freedom at the anchor are penalized.

### State variables

Each body $k \in \{i,j\}$ carries the affine state $\mathbf{q}_k \in \mathbb{R}^{12}$ as in [Affine Body](./affine_body.md). World position of a material point with rest local coordinates $\bar{\mathbf{x}}$ is $\mathbf{x}_k = \mathbf{J}(\bar{\mathbf{x}})\mathbf{q}_k$.

### Rest anchor and local coordinates

At setup, a single world anchor $\mathbf{c}$ is shared by both bodies. When the joint is created via Local `create_geometry`, optional **vertex** attributes `l_position` (`Vector3`, left body local) and `r_position` (`Vector3`, right body local) define the attachment; with rest transforms $\mathbf{T}_i$, $\mathbf{T}_j$,

$$
\mathbf{c} = \mathbf{T}_i \bar{\mathbf{c}}_i = \mathbf{T}_j \bar{\mathbf{c}}_j,
$$

where $\bar{\mathbf{c}}_i$ and $\bar{\mathbf{c}}_j$ are taken from `l_position` and `r_position` respectively when those attributes are present. If they are omitted, the implementation derives $\bar{\mathbf{c}}_i$, $\bar{\mathbf{c}}_j$ from the joint-creation path (e.g. a shared world anchor and the bodies' rest transforms). The backend stores these fixed local anchors per joint and uses them in the constraint below.

### Translation constraint

The constraint requires both bodies to map their local anchor to the same world position at all times:

$$
\mathbf{F}_t = \mathbf{c}_i - \mathbf{c}_j = \mathbf{0}
$$

where

$$
\mathbf{c}_k = \mathbf{J}(\bar{\mathbf{c}}_k)\mathbf{q}_k, \quad k \in \{i,j\}
$$

and $\mathbf{J}(\bar{\mathbf{c}}_k)$ is the $3\times12$ Jacobian in [Affine Body](./affine_body.md). Equivalently,

$$
\mathbf{F}_t = \mathbf{J}_t \begin{bmatrix} \mathbf{q}_i \\ \mathbf{q}_j \end{bmatrix}
$$

with

$$
\mathbf{J}_t = \begin{bmatrix} \mathbf{J}(\bar{\mathbf{c}}_i) & -\mathbf{J}(\bar{\mathbf{c}}_j) \end{bmatrix} \in \mathbb{R}^{3\times24}
$$

### Energy

The joint potential is a quadratic penalty on $\mathbf{F}_t$:

$$
E = \frac{K}{2} \| \mathbf{F}_t \|_2^2 = \frac{K}{2} \sum_{d=1}^{3} F_{t,d}^2
$$

where $\|\cdot\|_2$ is the Euclidean vector norm. The stiffness scale is

$$
K = \gamma (m_i + m_j)
$$

where $\gamma$ is the user-defined `strength_ratio` and $m_i$, $m_j$ are the affine bodies' masses.

Unlike the [Affine Body Fixed Joint](./affine_body_fixed_joint.md), there is **no** additional energy on relative orientation, so the two bodies may rotate freely about the anchor.

### Parameter range

`strength_ratio` $\gamma$ is dimensionless; it scales the penalty with combined mass. A proper initial value is $\gamma = 100$.

## Attributes and geometry

The joint is stored as a **0D simplicial complex**: **one vertex per joint** (no edges).

On each **vertex**:

- `l_geo_id` (`IndexT`): scene geometry slot id for the left body $i$
- `r_geo_id` (`IndexT`): scene geometry slot id for the right body $j$
- `l_inst_id` (`IndexT`): instance index within the left geometry
- `r_inst_id` (`IndexT`): instance index within the right geometry
- `strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$

When the joint is created via Local `create_geometry`, optional **vertex** attributes (each `Vector3`): `l_position` (left body's local frame) and `r_position` (right body's local frame).

Constitution UID: $26$ (see [Constitutions index](./index.md)).
