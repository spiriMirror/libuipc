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
\mathbf{\eta} = \begin{bmatrix}
\eta_p \otimes \mathbf{I}_{3\times3} & \mathbf{0}_{3\times 9} \\
\mathbf{0}_{9\times 3} & \eta_a \otimes \mathbf{I}_{9\times9}
\end{bmatrix}  
$$

$$
\Psi = \frac{1}{2} \left( \mathbf{q} - \hat{\mathbf{q}} \right)^T \mathbf{\eta} \mathbf{M} \left( \mathbf{q} - \hat{\mathbf{q}} \right),
$$

where $\hat{\mathbf{q}}$ is the target state, $\mathbf{M}$ is the mass matrix of the affine body, $\eta_p \in (0,+\infty)$ is the position constraint strength ratio, and $\eta_a \in (0,+\infty)$ is the affine (or rotation if not so rigorous) constraint strength ratio.

The reason we use strength ratio is that the mass matrix $\mathbf{M}$ already contains the mass and inertia information, so that users only need to care about how strong the constraint is compared to the mass and inertia of the body, which is more intuitive.

Because the constraint is "SOFT", we don't allow a too-strong strength which will lead to numerical instability. Normally, $\eta_p$ and $\eta_a$ are better in the range of $[0, 100]$.