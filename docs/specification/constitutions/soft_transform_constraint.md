# Soft Transform Constraint

Soft Transform Constraint is a type of constraint to control the motion of an affine body. 

The state of an affine body is represented as:

$$
\mathbf{q} = \begin{bmatrix}
p_x \\
p_y \\
p_z \\
\hline
a_{11}\\
a_{12}\\
a_{13}\\
\hdashline
a_{21}\\
a_{22}\\
a_{23}\\
\hdashline
a_{31}\\
a_{32}\\
a_{33}\\
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{p}   \\
\mathbf{a}_1 \\
\mathbf{a}_2 \\
\mathbf{a}_3 \\
\end{bmatrix},
$$

$\mathbf{p}$ is the position of the affine body and the $\mathbf{a}_i$ are the rows of the affine transformation matrix.

## #16 SoftTransformConstraint

The soft transform constraint energy has the form of "Kinetic" defined as

$$
\Psi = \frac{1}{2} \left( \mathbf{q} - \hat{\mathbf{q}} \right)^T \tilde{\mathbf{M}} \left( \mathbf{q} - \hat{\mathbf{q}} \right),
$$

where $\hat{\mathbf{q}}$ is the target state, and $\tilde{\mathbf{M}}$ is the modified mass matrix obtained by scaling the blocks of the original mass matrix $\mathbf{M}$ according to the strength ratios:

$$
\tilde{\mathbf{M}} = \begin{bmatrix}
\eta_p \mathbf{M}^{pp} & \sqrt{\eta_p \eta_a} \mathbf{M}^{pa} \\
\sqrt{\eta_p \eta_a} \mathbf{M}^{ap} & \eta_a \mathbf{M}^{aa}
\end{bmatrix},
$$

where:

- $\mathbf{M}^{pp}_{3 \times 3}$ is the position block of $\mathbf{M}$
- $\mathbf{M}^{pa}_{3 \times 9}$ is the cross-term block of $\mathbf{M}$
- $\mathbf{M}^{ap}_{9 \times 3}$ is the cross-term block of $\mathbf{M}$
- $\mathbf{M}^{aa}_{9 \times 9}$ is the affine block of $\mathbf{M}$
- $\eta_p \in (0,+\infty)$ is the position constraint strength ratio
- $\eta_a \in (0,+\infty)$ is the affine (or rotation if not so rigorous) constraint strength ratio

The cross-terms use the geometric mean $\sqrt{\eta_p \eta_a}$ to maintain consistency between the position and affine constraints.

The reason we use strength ratio is that the mass matrix $\mathbf{M}$ already contains the mass and inertia information, so that users only need to care about how strong the constraint is compared to the mass and inertia of the body, which is more intuitive.

Because the constraint is "SOFT", we don't allow a too-strong strength which will lead to numerical instability. Normally, $\eta_p$ and $\eta_a$ are better in the range of $[0, 100]$.