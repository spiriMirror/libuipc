# Affine Body Free Joint

## #33 AffineBodyFreeJoint

The **Affine Body Free Joint** constitution gives a single affine body $A$ all 6 degrees of freedom (3 translational + 3 rotational) by coupling it to an **implicit world frame** $W$ with identity transform $T_W = I$. Unlike the revolute or prismatic joints which connect two scene bodies, the free joint connects one body to the world origin and therefore requires only a single body reference.

> **Note:** The free joint is only meaningful within the [External Articulation Constraint](./external_articulation_constraint.md) system. It has no effect outside of that context.

We assume affine body $A$ with state vector $\mathbf{q}_A$ as defined in the [Affine Body](./affine_body.md) constitution. The world body $W$ is not a scene geometry; its state is the identity:

$$
\mathbf{q}_W = (0,0,0,\; 1,0,0,\; 0,1,0,\; 0,0,1)^T.
$$

### DOF Decomposition

The free joint decomposes into 6 independent 1-DOF sub-problems along the world axes $\mathbf{e}_0, \mathbf{e}_1, \mathbf{e}_2$. Each DOF index $d \in \{0,\dots,5\}$ measures a single generalized coordinate of body $A$ relative to the world:

$$
\delta\theta_d = 
\begin{cases}
\delta\theta_{\text{prismatic}}\!\left(\mathbf{q}_W, \mathbf{q}_A;\, \mathbf{e}_d\right), & d \in \{0,1,2\}, \\[4pt]
\delta\theta_{\text{revolute}}\!\left(\mathbf{q}_W, \mathbf{q}_A;\, \mathbf{e}_{d-3}\right), & d \in \{3,4,5\},
\end{cases}
$$

where $\delta\theta_{\text{prismatic}}$ and $\delta\theta_{\text{revolute}}$ are the variational DOF definitions from the [External Articulation Constraint](./external_articulation_constraint.md), and $\mathbf{e}_k$ is the $k$-th standard basis vector.

### Basis Construction

The world body $W$ is the L-side (body $i$) and the free body $A$ is the R-side (body $j$). For each DOF index $d$, let the axis direction be $\mathbf{a} = \mathbf{e}_{d \bmod 3}$ and let $T_A$ denote the initial transform of body $A$ with rotation $R_A$ and center $\mathbf{c}_A$.

**Translational DOFs** ($d < 3$). The joint frame $\mathbf{F}$ follows the [Prismatic Joint](./affine_body_prismatic_joint.md) definition:

$$
\mathbf{F}_d = \begin{pmatrix} \mathbf{c}_W \\ \hat{\mathbf{t}}_W \\ \mathbf{c}_A \\ \hat{\mathbf{t}}_A \end{pmatrix}, \quad
\bar{\mathbf{c}}_W = T_W^{-1}(\mathbf{c}_A) = \mathbf{c}_A, \;\; \bar{\mathbf{t}}_W = \mathbf{a}, \;\;
\bar{\mathbf{c}}_A = T_A^{-1}(\mathbf{c}_A), \;\; \bar{\mathbf{t}}_A = R_A^{-1}\mathbf{a}.
$$

**Rotational DOFs** ($d \geq 3$). Let $(\mathbf{b}, \mathbf{n})$ be an orthonormal pair spanning the plane perpendicular to $\mathbf{a}$. The joint frame follows the [Revolute Joint](./affine_body_revolute_joint.md) definition:

$$
\mathbf{F}_d = \begin{pmatrix} \hat{\mathbf{b}}_W \\ \hat{\mathbf{n}}_W \\ \hat{\mathbf{b}}_A \\ \hat{\mathbf{n}}_A \end{pmatrix}, \quad
\bar{\mathbf{b}}_W = \mathbf{b}, \;\; \bar{\mathbf{n}}_W = \mathbf{n}, \;\;
\bar{\mathbf{b}}_A = R_A^{-1}\mathbf{b}, \;\; \bar{\mathbf{n}}_A = R_A^{-1}\mathbf{n}.
$$

Since $T_W = I$, the world-side rest-shape vectors are simply the world-frame axes themselves.

### Gradient and Hessian Reduction

Each sub-problem produces a 24-dimensional gradient $\hat{G}^q = (G^q_W,\; G^q_A)^T$ and a $24\times 24$ Hessian $\hat{H}^q$. Because $W$ is fixed, the world-side contributions are discarded:

$$
G^q = \begin{pmatrix} \mathbf{0}_{12} \\ G^q_A \end{pmatrix}, \qquad
H^q = \begin{pmatrix} \mathbf{0} & \mathbf{0} \\ \mathbf{0} & H^q_{AA} \end{pmatrix},
$$

where $H^q_{AA} \in \mathbb{R}^{12\times 12}$ is the body-$A$ diagonal block. The assembler maps both the L-slot and R-slot to body $A$, so only $G^q_A$ and $H^q_{AA}$ contribute to the global system.

## Attributes

The joint geometry is a **0D simplicial complex**: **6 vertices per body** (no edges), one vertex per DOF.

On **vertices**:

- `l_geo_id` (`IndexT`): scene geometry slot id for body $A$
- `l_inst_id` (`IndexT`): instance index within the geometry
- `dof_type` (`IndexT`): DOF index $d \in \{0,\dots,5\}$, where 0--2 are translational (x, y, z) and 3--5 are rotational (x, y, z)

All 6 vertices of a single body share the same position $\bar{\mathbf{c}}_A$. No `r_geo_id` or `r_inst_id` attributes are needed because the world body $W$ is implicit.

> **Note:** Although the mathematical convention designates $W$ as the L-side and $A$ as the R-side, the attributes use the `l_` prefix because body $A$ is the only scene body stored. The `l_` naming follows the convention that the single stored body reference occupies the left slot.

When the joint is created via `create_geometry`, the vertex positions are automatically set to the body center extracted from the instance transform.
