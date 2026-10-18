# Soft Vertex Stitch

**Soft Vertex Stitch** is an inter-primitive constitutive model that connects pairs of vertices from two separate meshes using spring-like energy. Each (vertex, vertex) pair forms a 2-point stencil with either a harmonic potential or a distance-based spring potential.

The first mesh provides a set of vertices and the second mesh provides a corresponding set of vertices. For each paired vertex $\mathbf{x}_0$ (from the first mesh) and $\mathbf{x}_1$ (from the second mesh), a soft constraint is created.

## #22 SoftVertexStitch

### Harmonic Energy (rest length $L_0 = 0$)

When the rest length is zero, a simple harmonic energy is used:

$$
E = \frac{\kappa}{2} \|\mathbf{x}_0 - \mathbf{x}_1\|^2
$$

This form has no singularity at zero distance, making it ideal for stitching vertices that should coincide.

### Distance-Based Energy (rest length $L_0 > 0$)

When a nonzero rest length is specified, the energy penalizes deviations from the rest length:

$$
E = \frac{\kappa}{2} \left(\|\mathbf{x}_0 - \mathbf{x}_1\| - L_0\right)^2
$$

where:

- $\kappa$ is the stiffness parameter
- $L_0$ is the rest length of the spring
- $\mathbf{x}_0$, $\mathbf{x}_1$ are the vertex positions from the two meshes

### Gradient and Hessian

For the harmonic case ($L_0 = 0$), the gradient and Hessian have simple closed forms:

$$
\mathbf{G} = \kappa
\begin{bmatrix}
\mathbf{x}_0 - \mathbf{x}_1 \\
\mathbf{x}_1 - \mathbf{x}_0
\end{bmatrix}, \quad
\mathbf{H} = \kappa
\begin{bmatrix}
\mathbf{I} & -\mathbf{I} \\
-\mathbf{I} & \mathbf{I}
\end{bmatrix}
$$

For the distance-based case ($L_0 > 0$), the gradient and Hessian are computed symbolically (generated via SymPy) and projected to positive semi-definite form.

## Attributes

On `instances`:

- `topo`: vertex index pairs $(\mathbf{x}_0, \mathbf{x}_1)$ identifying stitched vertices
- `kappa`: $\kappa$, the stiffness of the stitch constraint
- `rest_length`: $L_0$, the rest length of the spring (0 for harmonic energy)

## Python Input

Both `SoftVertexStitch.create_geometry` overloads accept `stitched_vert_ids`
as an integer array of shape `(N, 2)` or `(N, 2, 1)`. Each row identifies one
vertex in each geometry; IDs must fit in `int32` and be valid vertex indices
in the corresponding geometry.

The binding converts `int64` arrays and non-contiguous views to contiguous
`int32` storage as needed, retaining that storage throughout the call.
The returned geometry owns a copy of the index pairs in `instances.topo`;
changing or releasing the input afterward does not change the stitches.
