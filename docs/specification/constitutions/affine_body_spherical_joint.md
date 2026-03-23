# Affine Body Spherical Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

Related: [Affine Body](./affine_body.md) (state and Jacobian $\mathbf{J}$), [Affine Body Fixed Joint](./affine_body_fixed_joint.md) (full rigid weld; spherical joint keeps only the translation part of that formulation).

## #26 AffineBodySphericalJoint

The **Affine Body Spherical Joint** (ball-and-socket joint) constrains two affine bodies so that a single anchor point has the same world position on both bodies, while **relative rotation is unconstrained**. Only three translational degrees of freedom at the anchor are penalized.

### State variables

Each body $k \in \{i,j\}$ carries the affine state $\mathbf{q}_k \in \mathbb{R}^{12}$ as in [Affine Body](./affine_body.md). World position of a material point with rest local coordinates $\bar{\mathbf{x}}$ is $\mathbf{x}_k = \mathbf{J}(\bar{\mathbf{x}})\mathbf{q}_k$.

### Rest anchor and local coordinates

At setup, the user specifies an attachment $\bar{\mathbf{p}}_j$ in body $j$'s (the **right** body's) rest local frame. The corresponding world anchor at rest is:

$$
\mathbf{c} = \mathbf{T}_j \bar{\mathbf{p}}_j
$$

where $\mathbf{T}_j$ is that instance's rest transform. The same world point is expressed in each body's local frame:

$$
\bar{\mathbf{c}}_i = \mathbf{T}_i^{-1} \mathbf{c}, \qquad \bar{\mathbf{c}}_j = \mathbf{T}_j^{-1} \mathbf{c} = \bar{\mathbf{p}}_j
$$

These fixed $\bar{\mathbf{c}}_i$, $\bar{\mathbf{c}}_j$ are stored per joint and used in the constraint below.

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

`strength_ratio` $\gamma$ is dimensionless; it scales the penalty with combined mass. Typical values are problem-dependent; the regression test uses $\gamma = 100$ for a small two-cube scene.

## Attributes and geometry

The joint is stored as a 1D simplicial complex (one edge per joint).

On each **edge**:

- `geo_ids`: `Vector2i` — scene geometry slot ids $(\text{left},\text{right})$ for bodies $i$ and $j$
- `inst_ids`: `Vector2i` — instance indices within each geometry
- `strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$

On **vertices** (two per joint, for visualization and initialization):

- `position` / builtin positions: vertex $2k$ is body $i$'s rest translation (edge endpoint for drawing); vertex $2k+1$ is the world-space anchor $\mathbf{c} = \mathbf{T}_j \bar{\mathbf{p}}_j$ at setup. The backend evaluates $\bar{\mathbf{c}}_i$, $\bar{\mathbf{c}}_j$ from the anchor vertex and the two rest transforms.

Constitution UID: $26$ (see [Constitutions index](./index.md)).
