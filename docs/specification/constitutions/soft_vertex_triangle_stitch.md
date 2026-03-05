# Soft Vertex Triangle Stitch

**Soft Vertex Triangle Stitch** is an inter-primitive constitutive model where each (vertex, triangle) pair from two separate meshes forms a tetrahedron element. The tetrahedron's shape-keeping energy uses the [Stable Neo-Hookean](./stable_neo_hookean.md) model.

The first mesh provides a set of vertices, and the second mesh provides a set of triangles. For each paired vertex $\mathbf{x}_0$ and triangle $(\mathbf{x}_1, \mathbf{x}_2, \mathbf{x}_3)$, a tetrahedron is formed with vertices $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, $\mathbf{x}_3$.

## #30 Soft Vertex Triangle Stitch

For a tetrahedron element with vertex $\mathbf{x}_0$ (from the vertex mesh) and triangle vertices $\mathbf{x}_1$, $\mathbf{x}_2$, $\mathbf{x}_3$ (from the triangle mesh), we define the deformed shape matrix:

$$
\mathbf{D}_s = \begin{bmatrix}
\mathbf{x}_1 - \mathbf{x}_0 & \mathbf{x}_2 - \mathbf{x}_0 & \mathbf{x}_3 - \mathbf{x}_0
\end{bmatrix}
$$

Similarly, the rest shape matrix from the reference configuration:

$$
\mathbf{D}_m = \begin{bmatrix}
\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0 & \bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_0
\end{bmatrix}
$$

The deformation gradient is:

$$
\mathbf{F} = \mathbf{D}_s \, \mathbf{D}_m^{-1}
$$

### Deformation Energy Density

The energy density follows [#10 Stable Neo-Hookean](./stable_neo_hookean.md):

$$
\Psi = \frac{\mu}{2}(I_c - 3) - \mu(J - 1) + \frac{\lambda}{2}(J - 1)^2 + \frac{\mu^2}{\lambda^2}
$$

where:

- $I_c = \|\mathbf{F}\|_F^2$ is the first invariant of the right Cauchy–Green deformation tensor

- $J = \det(\mathbf{F})$ is the determinant of the deformation gradient

- $\mu$ is the shear modulus

- $\lambda$ is the Lamé parameter

- $\|\cdot\|_F$ is the [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm)

### Total Energy

The total energy for each element is:

$$
E = \Psi \cdot V_0
$$

where $V_0 = \frac{1}{6} |\det(\mathbf{D}_m)|$ is the rest volume of the tetrahedron.

### Degenerate Handling

When the vertex $\bar{\mathbf{x}}_0$ is too close to the triangle plane $(\bar{\mathbf{x}}_1, \bar{\mathbf{x}}_2, \bar{\mathbf{x}}_3)$ in the rest configuration (signed distance $< d$, where $d$ is `min_separate_distance`):

The rest vertex is placed at the triangle centroid plus $d$ along the outward triangle normal:

$$
\bar{\mathbf{x}}_0' = \frac{\bar{\mathbf{x}}_1 + \bar{\mathbf{x}}_2 + \bar{\mathbf{x}}_3}{3} + d \, \hat{\mathbf{n}}
$$

where $\hat{\mathbf{n}}$ is the unit outward normal of the triangle, computed as:

$$
\hat{\mathbf{n}} = \frac{(\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1) \times (\bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_1)}{\|(\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1) \times (\bar{\mathbf{x}}_3 - \bar{\mathbf{x}}_1)\|}
$$

## Attributes

On `instances`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above
- `min_separate_distance`: $d$, the minimum rest separation distance for degenerate handling
