# Affine Body Revolute Joint

References:

[A unified newton barrier method for multibody dynamics](https://dl.acm.org/doi/pdf/10.1145/3528223.3530076)

## #18 AffineBodyRevoluteJoint

The **Affine Body Revolute Joint** constitution constrains two affine bodies to rotate relative to each other about a shared axis. This joint allows for rotational motion while restricting translational movement between the two bodies.

![Affine Body Revolute Joint](./media/affine_body_revolute_joint_fig1.drawio.svg)

We assume 2 affine body indices $i$ and $j$, each with their own state vector (to be concerete, transform) $\mathbf{q}_i$ and $\mathbf{q}_j$ as defined in the [Affine Body](./affine_body.md) constitution.

The joint axis in world space is defined by 2 points $\mathbf{x}^0$ and $\mathbf{x}^1$. At the beginning of the simulation, the relationships are kept:

$$
\mathbf{x}^0 = \mathbf{J}^0_i \mathbf{q}_i = \mathbf{J}^0_j \mathbf{q}_j,
$$

and,

$$
\mathbf{x}^1 = \mathbf{J}^1_i \mathbf{q}_i = \mathbf{J}^1_j \mathbf{q}_j,
$$

intuitively $\mathbf{J}^0$ and $\mathbf{J}^1$ tell the local coordinates of the two points on affine bodies $i$ and $j$. 

The energy function for the **Affine Body Revolute Joint** is defined as:

$$
E = \frac{K}{2} \cdot \left( 
    \| \mathbf{J}^0_i \mathbf{q}_i - \mathbf{J}^0_j \mathbf{q}_j \|^2
    +
    \| \mathbf{J}^1_i \mathbf{q}_i - \mathbf{J}^1_j \mathbf{q}_j \|^2
\right),
$$

where $K$ is the stiffness constant of the joint, we choose $K=\gamma (m_i + m_j)$, where $\gamma$ is a **user defined** `strength_ratio` parameter, and $m_i$ and $m_j$ are the masses of the two affine bodies.
