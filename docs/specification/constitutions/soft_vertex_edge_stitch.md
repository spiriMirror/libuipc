# Soft Vertex Edge Stitch

**Soft Vertex Edge Stitch** is an inter-primitive constitutive model where each (vertex, edge) pair from two separate meshes forms a triangle element. The triangle's shape-keeping energy is based on the **Saint-Venant–Kirchhoff (StVK) membrane** model, which is polynomial and free of logarithmic singularities.

The first mesh provides a set of vertices, and the second mesh provides a set of edges. For each paired vertex $\mathbf{x}_0$ and edge $(\mathbf{x}_1, \mathbf{x}_2)$, a triangle is formed with vertices $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$.

## #29 Soft Vertex Edge Stitch

For a triangle element with vertices at positions $\mathbf{x}_0$ (from the vertex mesh), $\mathbf{x}_1$ and $\mathbf{x}_2$ (from the edge mesh), we define the edge vectors:

$$
\begin{aligned}
\mathbf{e}_{01} = \mathbf{x}_1 - \mathbf{x}_0\\
\mathbf{e}_{02} = \mathbf{x}_2 - \mathbf{x}_0
\end{aligned}
$$

The current metric tensor is:

$$
\mathbf{A} = \begin{bmatrix}
\mathbf{e}_{01} \cdot \mathbf{e}_{01} & \mathbf{e}_{01} \cdot \mathbf{e}_{02} \\
\mathbf{e}_{02} \cdot \mathbf{e}_{01} & \mathbf{e}_{02} \cdot \mathbf{e}_{02}
\end{bmatrix}
$$

Similarly, for the reference (rest) configuration with positions $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, $\bar{\mathbf{x}}_2$:

$$
\bar{\mathbf{B}} = \begin{bmatrix}
\bar{\mathbf{e}}_{01} \cdot \bar{\mathbf{e}}_{01} & \bar{\mathbf{e}}_{01} \cdot \bar{\mathbf{e}}_{02} \\
\bar{\mathbf{e}}_{02} \cdot \bar{\mathbf{e}}_{01} & \bar{\mathbf{e}}_{02} \cdot \bar{\mathbf{e}}_{02}
\end{bmatrix}
$$

The right Cauchy–Green deformation tensor in material coordinates is:

$$
\mathbf{C} = \bar{\mathbf{B}}^{-1} \mathbf{A}
$$

The Green–Lagrange strain tensor is:

$$
\mathbf{E}_G = \frac{1}{2}(\mathbf{C} - \mathbf{I})
= \frac{1}{2}(\bar{\mathbf{B}}^{-1} \mathbf{A} - \mathbf{I})
$$

### Deformation Energy Density

The StVK membrane energy density is:

$$
\Psi = \mu \, \text{tr}(\mathbf{E}_G^2) + \frac{\lambda}{2} \left(\text{tr}(\mathbf{E}_G)\right)^2
$$

where:

- $\mu$ is the shear modulus

- $\lambda$ is the Lamé parameter

This energy is **polynomial** in the vertex positions (degree 4), with no logarithmic or determinant-based barriers. It remains well-defined even when the triangle degenerates or inverts, making it suitable for stitching configurations where the vertex may initially lie on or near the edge.

### Total Energy

The total energy for each element is:

$$
E = \Psi \cdot 2 \, t \, A_0
$$

where:

- $t$ is the thickness parameter

- $A_0 = \frac{1}{2} \|\bar{\mathbf{e}}_{01} \times \bar{\mathbf{e}}_{02}\|$ is the rest area of the triangle

The factor of 2 accounts for the one-sided thickness convention.

### Degenerate Handling

When the vertex $\bar{\mathbf{x}}_0$ is too close to the edge $(\bar{\mathbf{x}}_1, \bar{\mathbf{x}}_2)$ in the rest configuration (distance $< d$, where $d$ is `min_separate_distance`):

- If the vertex is **not collinear** with the edge: the rest vertex is moved along the existing vertex-to-edge direction until the distance equals $d$.

- If the vertex is **collinear** with the edge (distance $\approx 0$): the rest vertex is offset by $d$ in an arbitrary direction perpendicular to the edge.

## Attributes

On `instances`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above
- `thickness`: $t$, the shell thickness parameter
- `min_separate_distance`: $d$, the minimum rest separation distance for degenerate handling
