# Affine Body Prismatic Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #20 AffineBodyPrismaticJoint

The **Affine Body Prismatic Joint** constitution constrains two affine bodies to translate relative to each other along a given direction vector $\hat{\mathbf{t}}$ while restricting other translational and rotational motion. 

We assume 2 affine body indices $i$ and $j$, each with their own state vector (to be concrete, transform) $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

![Affine Body Prismatic Joint](./media/affine_body_prismatic_joint_fig1.drawio.svg)

At the beginning of the simulation, the relationships are established such that the frame $(\mathbf{c}, \hat{\mathbf{t}},  \hat{\mathbf{n}},  \hat{\mathbf{b}})$ is well-aligned with the corresponding frame of affine body $i$ and $j$:

$$
\begin{aligned}
\mathbf{c} &= \mathbf{J}^{c}_i \mathbf{q}_i &= \mathbf{J}^{c}_j \mathbf{q}_j \\
\hat{\mathbf{t}} &= \mathring{\mathbf{J}}^{\hat{t}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{t}}_j \mathbf{q}_j \\
\hat{\mathbf{n}} &= \mathring{\mathbf{J}}^{\hat{n}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{n}}_j \mathbf{q}_j \\
\hat{\mathbf{b}} &= \mathring{\mathbf{J}}^{\hat{b}}_i \mathbf{q}_i &= \mathring{\mathbf{J}}^{\hat{b}}_j \mathbf{q}_j \\
\end{aligned}
$$

where $\mathbf{c}$ is the common point on the joint axis before simulation starts, and $\hat{\mathbf{t}}$, $\hat{\mathbf{n}}$, and $\hat{\mathbf{b}}$ are the direction, normal, and binormal vectors defining the joint frame. $\mathbf{J}^{x}_k$ is the local coordinate of point $\mathbf{x}$ in affine body $k$'s local space, and $\mathring{\mathbf{J}}^{\hat{v}}_k$ is the local coordinate of direction vector $\hat{\mathbf{v}}$ in affine body $k$'s local space. Typically, $\mathbf{J}^{x}$ obeys the definition of $\mathbf{J}$ in the [Affine Body](./affine_body.md), while $\mathring{\mathbf{J}}^{\hat{v}}$ is defined as:

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
\mathbf{c}_k &= \mathbf{J}^{c}_k \mathbf{q}_k, k \in \{i,j\} \\
\hat{\mathbf{t}}_k &= \mathring{\mathbf{J}}^{\hat{t}}_k \mathbf{q}_k, k \in \{i,j\} \\
\hat{\mathbf{n}}_k &= \mathring{\mathbf{J}}^{\hat{n}}_k \mathbf{q}_k, k \in \{i,j\} \\
\hat{\mathbf{b}}_k &= \mathring{\mathbf{J}}^{\hat{b}}_k \mathbf{q}_k, k \in \{i,j\} \\
\end{aligned}
$$

To form the prismatic joint, we define the following constraint functions:

$$
\begin{aligned}
C_0 &= (\mathbf{c}_j - \mathbf{c}_i) \times \hat{\mathbf{t}}_i &= \mathbf{0} \\
C_1 &= (\mathbf{c}_i - \mathbf{c}_j) \times \hat{\mathbf{t}}_j &= \mathbf{0} \\
C_2 &= \hat{\mathbf{n}}_i - \hat{\mathbf{n}}_j &= 0 \\
C_3 &= \hat{\mathbf{b}}_i - \hat{\mathbf{b}}_j &= 0
\end{aligned}
$$

First two symmetric constraints ensure that the two bodies can only translate along the direction of $\hat{\mathbf{t}}$. The last two constraints ensure that the two bodies do not rotate relative to each other around any axis.


The energy function for the **Affine Body Prismatic Joint** is defined as some quatratic penalty on the constraint violations:

$$
E = \sum_{k=0}^{3} \frac{K}{2} \| C_k \|^2_2,
$$

where $K$ is the stiffness constant of the joint, we choose $K=\gamma (m_i + m_j)$, where $\gamma$ is a **user defined** `strength_ratio` parameter, and $m_i$ and $m_j$ are the masses of the two affine bodies.

