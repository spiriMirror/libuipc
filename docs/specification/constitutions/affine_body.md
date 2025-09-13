# Affine Body

[Affine body dynamics: fast, stable and intersection-free simulation of stiff materials](https://dl.acm.org/doi/10.1145/3528223.3530064)

## State Variable

The state variable for an **Affine Body**:

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

where $\mathbf{p}$ is the position of the affine body and the $\mathbf{a}_i$ are the rows of the affine transformation matrix.

$$
\mathbf{A} = [\mathbf{a}_1,\mathbf{a}_2,\mathbf{a}_3]^T
= \begin{bmatrix}
\mathbf{a}_1^T \\
\mathbf{a}_2^T \\
\mathbf{a}_3^T \\
\end{bmatrix}
$$

P.S. The state variable for a **Soft Body** Vertex:
$$
\mathbf{x} = 
\begin{bmatrix}
x\\
y\\
z\\
\end{bmatrix}
$$

## Jacobi Matrix

The Jacobi Matrix for **Affine Body**:

$$
\mathbf{J}(\bar{\mathbf{x}})
= 
\left[\begin{array}{ccc|ccc:ccc:ccc}
1 &   &   & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  &  &  & \\
& 1 &   &  &  &  & \bar{x}_1 & \bar{x}_2 & \bar{x}_3 &  &  &  \\
&   & 1 &  &  &  &  &  &  &  \bar{x}_1 & \bar{x}_2 & \bar{x}_3\\
\end{array}\right]
$$

where, $\bar{\mathbf{x}} = \begin{bmatrix}\bar{x}^1 & \bar{x}^2 & \bar{x}^2\end{bmatrix}^T$, $\bar{\mathbf{x}}$ is the material point position in the local(model) space. Every material point$\mathbf{x}$ has a Jacobi Matrix $\mathbf{J}(\bar{\mathbf{x}})$

P.S. The Jacobi Matrix for a **Soft Body** Vertex is an identity matrix:

$$
\mathbf{J} = \mathbf{I}
$$

## Calculate $\mathbf{x}(t)$

For a vertex in an **Affine Body**:

$$
\mathbf{x}(t) = \mathbf{J}(\bar{\mathbf{x}}) \mathbf{q}(t)
$$

To efficiently calculate $\mathbf{x}(t)$, we just use:

$$
\mathbf{x}(t) = 
\begin{bmatrix}
\mathbf{a}_1(t)\cdot \bar{\mathbf{x}} \\
\mathbf{a}_2(t)\cdot \bar{\mathbf{x}} \\
\mathbf{a}_3(t)\cdot \bar{\mathbf{x}} \\
\end{bmatrix}
+\mathbf{p}
$$

## #1 OrthoPotential

Formulation: [Affine body dynamics: fast, stable and intersection-free simulation of stiff materials](https://dl.acm.org/doi/10.1145/3528223.3530064) (eq. 7)

$$
V_{\perp}(\mathbf{q}) = \kappa v \|\mathbf{A}\mathbf{A}^T - \mathbf{I}_3\|_F^2,
$$

where $\kappa$ is the stiffness parameter, $v$ is the volume of the affine body, $\mathbf{I}_3$ is the $3\times3$ identity matrix, and $\|\cdot\|_F$ is the [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm).

We'd like to take $100MPa \le \kappa \le 100GPa$.

## #2 ARAP

[Dynamic Deformables:
Implementation and Production
Practicalities (Now With Code!)
](http://www.tkim.graphics/DYNAMIC_DEFORMABLES/)

As Rigid As Possible (ARAP) deformation energy:

$$
V = \kappa v \|\mathbf{A}-\mathbf{R}\|_F^2 = \kappa v \|\mathbf{S}-\mathbf{I}\|_F^2
$$

where $\mathbf{R}\mathbf{S}$ is the polar decomposition of $\mathbf{A}$. 
