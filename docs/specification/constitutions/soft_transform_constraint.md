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

The constraint energy is

$$
\Psi = \frac{1}{2} \delta\mathbf{q}^T \tilde{\mathbf{M}}\, \delta\mathbf{q}, \quad \delta\mathbf{q} = \mathbf{q} - \hat{\mathbf{q}},
$$

where $\hat{\mathbf{q}}$ is the target state. The constraint mass matrix $\tilde{\mathbf{M}}$ is constructed from the **parallel-axis decomposition** of the physical mass matrix $\mathbf{M}$:

$$
\tilde{\mathbf{M}} = \eta_p\,\mathbf{M}_{cm} + \eta_a\,\mathbf{M}_{rot}, \quad \mathbf{M} = \mathbf{M}_{cm} + \mathbf{M}_{rot},
$$

where:

- $\mathbf{c}$ is the center of mass in the reference frame ($m\mathbf{c} = m\bar{\mathbf{x}}$)
- $\mathbf{M}_{cm} = m\,\mathbf{J}(\mathbf{c})^T\mathbf{J}(\mathbf{c})$ is the ABD mass matrix of a point mass $m$ at $\mathbf{c}$, capturing the kinetic energy of center-of-mass translation
- $\mathbf{M}_{rot} = \mathbf{M} - \mathbf{M}_{cm}$ captures rotation/deformation *about* the center of mass, with $\mathbf{S}_{cm} = \mathbf{S} - m\mathbf{c}\mathbf{c}^T$ in its affine block, where $\mathbf{S} = \int\rho\,\bar{\mathbf{x}}\bar{\mathbf{x}}^T\,dV$
- $\eta_p \in [0,+\infty)$: translation (center-of-mass) constraint strength
- $\eta_a \in [0,+\infty)$: rotation/deformation constraint strength

Equivalently: $\tilde{\mathbf{M}} = \eta_a\,\mathbf{M} + (\eta_p - \eta_a)\,\mathbf{M}_{cm}$.

Because the constraint is "SOFT", overly large values will cause numerical instability. Normally $\eta_p$ and $\eta_a$ are better kept in the range $[0, 100]$.

## Attributes

On `instances`:

- `strength_ratio`: $(\eta_p, \eta_a)$ in the kinetic term
- `aim_transform`: target $\tilde{\mathbf{q}}$ in the kinetic term