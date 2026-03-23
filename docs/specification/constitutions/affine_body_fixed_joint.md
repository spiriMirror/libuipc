# Affine Body Fixed Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #25 AffineBodyFixedJoint

The **Affine Body Fixed Joint** constitution constrains two affine bodies to be rigidly connected, preventing all relative translation and rotation. This is the most restrictive joint type, effectively welding two bodies together.

We assume 2 affine body indices $i$ and $j$, each with their own state vector (to be concrete, transform) $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

At the beginning of the simulation, a shared frame $(\mathbf{c}, \hat{\mathbf{t}}, \hat{\mathbf{n}}, \hat{\mathbf{b}})$ is established such that it is well-aligned with the corresponding frame of affine body $i$ and $j$:

$$
\begin{aligned}
\mathbf{c} &= \mathbf{J}^{c}_i \mathbf{q}_i &= \mathbf{J}^{c}_j \mathbf{q}_j \\
\hat{\mathbf{t}} &= \mathring{\mathbf{J}}^{\hat{t}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{t}}_j \mathbf{q}_j \\
\hat{\mathbf{n}} &= \mathring{\mathbf{J}}^{\hat{n}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{n}}_j \mathbf{q}_j \\
\hat{\mathbf{b}} &= \mathring{\mathbf{J}}^{\hat{b}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{b}}_j \mathbf{q}_j \\
\end{aligned}
$$

where $\mathbf{c}$ is the midpoint between the two body centers, and $\hat{\mathbf{t}}$, $\hat{\mathbf{n}}$, $\hat{\mathbf{b}}$ are the world-frame coordinate axes expressed in each body's local space. $\mathbf{J}^{x}_k$ is the local coordinate of point $\mathbf{x}$ in affine body $k$'s local space, and $\mathring{\mathbf{J}}^{\hat{v}}_k$ is the local coordinate of direction vector $\hat{\mathbf{v}}$ in affine body $k$'s local space. Typically, $\mathbf{J}^{x}$ obeys the definition of $\mathbf{J}$ in the [Affine Body](./affine_body.md), while $\mathring{\mathbf{J}}^{\hat{v}}$ is defined as:

$$
\mathring{\mathbf{J}}(\hat{\mathbf{v}})
= 
\left[\begin{array}{ccc|ccc:ccc:ccc}
0 &   &   & \hat{v}_1 & \hat{v}_2 & \hat{v}_3 &  &  &  &  &  & \\
& 0 &   &  &  &  & \hat{v}_1 & \hat{v}_2 & \hat{v}_3 &  &  &  \\
&   & 0 &  &  &  &  &  &  &  \hat{v}_1 & \hat{v}_2 & \hat{v}_3\\
\end{array}\right],
$$

which omits the translational part.

To be concise, we define:

$$
\begin{aligned}
\mathbf{c}_k &= \mathbf{J}^{c}_k \mathbf{q}_k, &k \in \{i,j\} \\
\hat{\mathbf{t}}_k &= \mathring{\mathbf{J}}^{\hat{t}}_k \mathbf{q}_k, &k \in \{i,j\} \\
\hat{\mathbf{n}}_k &= \mathring{\mathbf{J}}^{\hat{n}}_k \mathbf{q}_k, &k \in \{i,j\} \\
\hat{\mathbf{b}}_k &= \mathring{\mathbf{J}}^{\hat{b}}_k \mathbf{q}_k, &k \in \{i,j\} \\
\end{aligned}
$$

### Rotation Constraints

To prevent relative rotation, the frame vectors must remain aligned:

$$
\begin{aligned}
C_0 &= \hat{\mathbf{t}}_i - \hat{\mathbf{t}}_j &= \mathbf{0} \\
C_1 &= \hat{\mathbf{n}}_i - \hat{\mathbf{n}}_j &= \mathbf{0} \\
C_2 &= \hat{\mathbf{b}}_i - \hat{\mathbf{b}}_j &= \mathbf{0}
\end{aligned}
$$

The rotation constraint vector is assembled as:

$$
\mathbf{F}_{r} = \begin{bmatrix}
\hat{\mathbf{t}}_i - \hat{\mathbf{t}}_j\\
\hat{\mathbf{n}}_i - \hat{\mathbf{n}}_j\\
\hat{\mathbf{b}}_i - \hat{\mathbf{b}}_j\\
\end{bmatrix}_{9 \times 1}
$$

with the affine body mapping:

$$
\mathbf{J}_r = \begin{bmatrix}
\mathring{\mathbf{J}}(\bar{\mathbf{t}}_i) & -\mathring{\mathbf{J}}(\bar{\mathbf{t}}_j) \\
\mathring{\mathbf{J}}(\bar{\mathbf{n}}_i) & -\mathring{\mathbf{J}}(\bar{\mathbf{n}}_j) \\
\mathring{\mathbf{J}}(\bar{\mathbf{b}}_i) & -\mathring{\mathbf{J}}(\bar{\mathbf{b}}_j) \\
\end{bmatrix}_{9 \times 24}
$$

such that $\mathbf{F}_{r} = \mathbf{J}_{r} \begin{bmatrix} \mathbf{q}_i \\ \mathbf{q}_j \end{bmatrix}$.

The rotation energy is:

$$
E_r = \frac{K}{2} \| \mathbf{F}_r \|^2_2 = \frac{K}{2} \left( \| C_0 \|^2 + \| C_1 \|^2 + \| C_2 \|^2 \right)
$$

### Translation Constraint

To prevent relative translation, the center points must coincide:

$$
C_3 = \| \mathbf{c}_i - \mathbf{c}_j \|_2 = 0
$$

The translation constraint vector is:

$$
\mathbf{F}_{t} = \begin{bmatrix}
\mathbf{c}_i - \mathbf{c}_j
\end{bmatrix}_{3 \times 1}
$$

with the affine body mapping:

$$
\mathbf{J}_{t} = \begin{bmatrix}
\mathbf{J}(\bar{\mathbf{c}}_i) & -\mathbf{J}(\bar{\mathbf{c}}_j)
\end{bmatrix}_{3 \times 24}
$$

such that $\mathbf{F}_{t} = \mathbf{J}_{t} \begin{bmatrix} \mathbf{q}_i \\ \mathbf{q}_j \end{bmatrix}$.

The translation energy is:

$$
E_t = \frac{K}{2} \| \mathbf{F}_t \|^2_2
$$

### Total Energy

The total energy for the fixed joint is the sum of the rotation and translation energies:

$$
E = E_r + E_t
$$

where $K$ is the stiffness constant of the joint, we choose $K=\gamma (m_i + m_j)$, where $\gamma$ is a **user defined** `strength_ratio` parameter, and $m_i$ and $m_j$ are the masses of the two affine bodies.

## Attributes

On the joint geometry (1D simplicial complex), on `edges`:

- `strength_ratio`: $\gamma$ in $K = \gamma(m_i + m_j)$ above
