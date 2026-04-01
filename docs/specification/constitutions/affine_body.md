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
\mathbf{x} 
= \begin{bmatrix}
x \\
y \\
z \\
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

where, $\bar{\mathbf{x}} = \begin{bmatrix}\bar{x}^1 & \bar{x}^2 & \bar{x}^2\end{bmatrix}^T$, $\bar{\mathbf{x}}$ is the material point position in the local(model) space. Every material point $\mathbf{x}$ has a Jacobi Matrix $\mathbf{J}(\bar{\mathbf{x}})$

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

## Shell (Codim 2D)

Codim 2D variant of `AffineBodyConstitution` for triangle meshes (`dim == 2`).

The shell is modelled as a uniform slab of full `thickness` $2r$, giving an effective volume per triangle $i$:

$$\bar{v}_i = A_i \cdot 2r$$

where $A_i$ is the triangle area and $r$ is the thickness radius.

## Rod (Codim 1D)

Codim 1D variant of `AffineBodyConstitution` for edge meshes (`dim == 1`).

The rod is modelled as a cylinder of circular cross-section with `thickness` $r$, giving an effective volume per segment $i$:

$$\bar{v}_i = L_i \cdot \pi r^2$$

where $L_i$ is the segment length.

---

## Attributes

`AffineBodyConstitution::apply_to` sets the following attributes on the `SimplicialComplex`.

### Meta Attributes (`sc.meta()`)

Per-geometry, shared by all instances.

| Attribute | Type | Description |
|-----------|------|-------------|
| `constitution_uid` | `U64` | Constitution UID identifying the ABD formulation (e.g. 1 = OrthoPotential, 2 = ARAP). |
| `volume` | `Float` | Rest volume $\bar{v}$ of the geometry. For full 3D bodies this is the mesh volume; for shells $\bar{v} = \sum A_i \cdot 2r$; for rods $\bar{v} = \sum L_i \cdot \pi r^2$. |
| `mass_density` | `Float` | Volume mass density $\rho$ (kg/m³). |
| `abd_mass` | `Float` | Dyadic mass: total mass $m = \rho \bar{v}$. |
| `abd_mass_x_bar` | `Vector3` | Dyadic mass: first moment $m\bar{\mathbf{x}} = \int \rho\, \bar{\mathbf{x}}\, dV$. |
| `abd_mass_x_bar_x_bar` | `Matrix3x3` | Dyadic mass: second moment tensor $\mathbf{S} = \int \rho\, \bar{\mathbf{x}}\bar{\mathbf{x}}^T\, dV$. |
| `mass` | `Float` | Total mass (same value as `abd_mass`, for user convenience). |
| `mass_center` | `Vector3` | Center of mass $\mathbf{c} = m\bar{\mathbf{x}} / m$ in the rest configuration. |
| `inertia` | `Matrix3x3` | Inertia tensor about the center of mass $\mathbf{I}_{cm}$. Derived from the dyadic mass via the parallel axis theorem. |
| `self_collision` | `IndexT` | Whether self-collision detection is enabled. Default: `0` (off). |
| `dof_offset` | `IndexT` | Degree-of-freedom offset in the global system (filled by the backend). |
| `dof_count` | `IndexT` | Degree-of-freedom count in the global system (filled by the backend). |

### Instance Attributes (`sc.instances()`)

Per-instance (each instance is one affine body sharing the same rest geometry).

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `kappa` | `Float` | *user-specified* | Stiffness parameter $\kappa$ for the shape preservation energy. |
| `is_fixed` | `IndexT` | `0` | `1` = body is fixed (position unchanged, not influenced by constitution/kinetics). |
| `is_dynamic` | `IndexT` | `1` | `1` = kinetics (inertia, velocity) are considered; `0` = quasi-static. |
| `velocity` | `Matrix4x4` | `Zero` | Initial velocity as the time derivative of the 4x4 transform matrix. |
| `external_kinetic` | `IndexT` | `0` | `1` = skip default kinetic computation (used with external constraints). |

### Additional Attributes for Shell / Rod

`AffineBodyShell` and `AffineBodyRod` set extra attributes on top of the above:

| Location | Attribute | Type | Description |
|----------|-----------|------|-------------|
| `sc.meta()` | `is_codim` | `IndexT` | Always `1` for shell/rod, indicating a codimensional body. |
| `sc.vertices()` | `thickness` | `Float` | Thickness radius $r$. For shells, the effective slab thickness is $2r$; for rods, the cross-section radius. |

### Dyadic Mass and the 12x12 Mass Matrix

The three dyadic mass attributes (`abd_mass`, `abd_mass_x_bar`, `abd_mass_x_bar_x_bar`) define the 12x12 ABD mass matrix:

$$
\mathbf{M} = \begin{bmatrix}
m\mathbf{I}_3 & \mathbf{I}_3 \otimes (m\bar{\mathbf{x}})^T \\
\mathbf{I}_3 \otimes (m\bar{\mathbf{x}}) & \mathbf{I}_3 \otimes \mathbf{S}
\end{bmatrix}
$$

The user-facing attributes (`mass`, `mass_center`, `inertia`) are the equivalent rigid body quantities recovered via `geometry::affine_body::to_rigid_body`.

---

## #1 OrthoPotential

Formulation: [Affine body dynamics: fast, stable and intersection-free simulation of stiff materials](https://dl.acm.org/doi/10.1145/3528223.3530064) (eq. 7)

Shape preservation energy per body:

$$
V_{\perp}(\mathbf{q}) = \kappa \bar{v} \|\mathbf{A}\mathbf{A}^T - \mathbf{I}_3\|_F^2,
$$

where $\kappa$ is the stiffness parameter, $\bar{v}$ is the rest volume of the affine body, $\mathbf{I}_3$ is the $3\times3$ identity matrix, and $\|\cdot\|_F$ is the [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm).

We'd like to take $100MPa \le \kappa \le 100GPa$.

## #2 ARAP

[Dynamic Deformables:
Implementation and Production
Practicalities (Now With Code!)
](http://www.tkim.graphics/DYNAMIC_DEFORMABLES/)

As Rigid As Possible (ARAP) deformation energy per body:

$$
V = \kappa \bar{v} \|\mathbf{A}-\mathbf{R}\|_F^2 = \kappa \bar{v} \|\mathbf{S}-\mathbf{I}\|_F^2
$$

where $\mathbf{R}\mathbf{S}$ is the polar decomposition of $\mathbf{A}$. Each concrete Affine Body constitution documents only the attributes it introduces and how they map to its formulas.