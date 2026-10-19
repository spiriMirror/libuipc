# Discrete Shell Bending

**Discrete Shell Bending** is a constitutive model for simulating the bending behavior of thin shell structures. This model captures the dihedral angle-based bending energy between adjacent triangular faces in a shell mesh.

Reference:

- [Discrete Shell](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf)
- [Discrete bending forces and their Jacobians](https://media.disneyanimation.com/uploads/production/publication_asset/74/asset/main_preprint.pdf), Section 5, Eq. (11)

## #17 Discrete Shell Bending

For a shell bending element defined by four vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and $\mathbf{x}_3$, where $(\mathbf{x}_1, \mathbf{x}_2)$ forms the shared edge between two adjacent triangular faces, we define:

**Dihedral Angle:**

The dihedral angle $\theta$ is a signed angle in radians between the faces
sharing $(\mathbf{x}_1,\mathbf{x}_2)$. The implementation uses oriented normals

$$
\mathbf{n}_1=(\mathbf{x}_1-\mathbf{x}_0)\times(\mathbf{x}_2-\mathbf{x}_0),
\qquad
\mathbf{n}_2=(\mathbf{x}_2-\mathbf{x}_3)\times(\mathbf{x}_1-\mathbf{x}_3).
$$

It computes the magnitude with
$\arccos(\operatorname{clamp}(\mathbf{n}_1\cdot\mathbf{n}_2/
(\|\mathbf{n}_1\|\|\mathbf{n}_2\|),-1,1))$, then negates it when
$(\mathbf{n}_2\times\mathbf{n}_1)\cdot(\mathbf{x}_1-\mathbf{x}_2)<0$.
The same convention is used for the reference angle $\bar\theta$.

**Rest Configuration Parameters:**

From the reference configuration with rest positions $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, $\bar{\mathbf{x}}_2$, and $\bar{\mathbf{x}}_3$:

- **Rest Length:** $L_0 = \|\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1\|_2$ is the length of the shared edge in the rest configuration

- **Reference Height Parameter:** $\bar{h}$ is one third of the mean of the
  two triangle altitudes to the shared edge, computed as:
  $$
  \bar{h} = \frac{A}{3 L_0}
  $$
  where $A=A_1+A_2$ is the combined rest area of the two triangles:
  $$
  A = \frac{1}{2}\left(\|(\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0) \times (\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0)\|_2 + \|(\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_3) \times (\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_3)\|_2\right)
  $$

- **Rest Dihedral Angle:** $\bar{\theta}$ is the dihedral angle in the rest configuration

### Bending Energy

The per-edge bending energy is:

$$
E = \kappa \frac{(\theta - \bar{\theta})^2 L_0}{\bar{h}}
$$

where:

- $\kappa$ is the bending stiffness parameter
- $\theta$ is the current dihedral angle
- $\bar{\theta}$ is the rest dihedral angle
- $L_0$ is the rest length of the shared edge
- $\bar{h}$ is the reference height parameter, not the material thickness

Because $\bar h=A/(3L_0)$, the reference metric is
$L_0/\bar h=3L_0^2/A$. This already contains the complete Discrete Shells
geometric weight: the backend does **not** multiply by $A$ again. Consequently,
the metric is invariant under uniform scaling of the rest hinge.

For the formula overload, the mesh stores a one-sided thickness radius $r$.
The full material thickness is $h=2r$, and

$$
\kappa=\frac{D}{2}=\frac{E h^3}{24(1-\nu^2)}
        =\frac{E(2r)^3}{24(1-\nu^2)}.
$$

Here $E$ is Young's modulus (Pa), $\nu$ is Poisson's ratio, $h$ is in meters,
and $D=Eh^3/[12(1-\nu^2)]$ is the classical flexural rigidity (N m).
The hinge coefficient is half of $D$ because the edge energy above has no
additional factor of $1/2$.

The 2026-10-19 correction halves the formula-derived coefficient and bending
forces for the same material inputs. To reproduce earlier formula-based scenes,
double their bending Young's modulus. Raw `apply_to(mesh, bending_stiffness)`
values retain their existing meaning and behavior; no migration is needed for
raw coefficients. Existing baked animation must be rebaked to use the correction.

### Complete mesh energy and solver contribution

For a manifold triangle mesh, let $\mathcal E_{\mathrm{int}}$ be its interior
edges. For edge $e=(i,j)$ define

$$
r_e=\frac{r_i+r_j}{2},\qquad h_e=2r_e=r_i+r_j,\qquad
w_e=\frac{3L_{0,e}^2}{A_{1,e}+A_{2,e}},\qquad
\delta\theta_e=\theta_e(\mathbf{x})-\bar\theta_e.
$$

Here $r_i,r_j$ are the stored one-sided radii at the shared edge endpoints;
the opposite vertices' radii are not part of this material average. The full
physical elastic bending energy is

$$
U_b(\mathbf{x})
=\sum_{e\in\mathcal E_{\mathrm{int}}}\kappa_e w_e\delta\theta_e^2
=\sum_{e\in\mathcal E_{\mathrm{int}}}
\frac{E_b h_e^3 L_{0,e}^2}
     {8(1-\nu_b^2)(A_{1,e}+A_{2,e})}\delta\theta_e^2.
$$

The second expression applies to the formula-based material overload;
the raw-coefficient overload uses the first expression. $E_b$ and $\nu_b$
are the bending channel's Young's modulus and Poisson ratio. $L_{0,e}$,
$A_{1,e}$, $A_{2,e}$ and $\bar\theta_e$ are fixed reference-configuration
values; only $\theta_e(\mathbf{x})$ changes during elastic deformation.
For initially curved cloth, $\bar\theta_e$ need not be zero.

The CUDA backend adds $\Delta t^2 U_b$ to its incremental objective; the
physical energy $U_b$ itself contains no time step. Its generic outer weight
is exactly 1. There is no second area, volume, thickness or 1/2 multiplier.
With SI inputs, $\kappa_e$ is in N m, $w_e$ and angles are dimensionless, and
$U_b$ is in joules. Boundary edges have no bending stencil. The current
implementation includes edges with exactly two distinct opposite vertices;
nonmanifold edges with more neighbors are not included by this constitution.

### Force and Hessian

For one hinge, let $\mathbf{g}_e=\nabla\theta_e$ and
$H_{\theta,e}=\nabla^2\theta_e$ with respect to its 12 position coordinates.
Since the material and reference weights are fixed,

$$
\mathbf{f}_e=-2\kappa_e w_e\delta\theta_e\mathbf{g}_e,
\qquad
\nabla^2U_e=2\kappa_e w_e
\left(\mathbf{g}_e\mathbf{g}_e^T+\delta\theta_e H_{\theta,e}\right).
$$

The backend scales the gradient and Hessian by $\Delta t^2$.
For the Newton solve, `make_spd` clamps negative Hessian eigenvalues to zero;
this positive-semidefinite approximation does not change the energy or gradient.
The angular restoring moment is $-\partial U_e/\partial\theta_e
=-2\kappa_e w_e\delta\theta_e$.

These formulas assume nondegenerate triangles and a consistent signed-angle
branch. The elastic model directly subtracts the two angles: it does not wrap
their difference across the $\pm\pi$ branch or add a bending-angle barrier.
Collision response is handled by the separate contact model. There is no
velocity-dependent damping term in this elastic bending constitution.

Regression coverage checks the calibrated scalar energy/moment, flat and
curved rest hinges, both fold signs, rigid-motion invariance, and finite
differences of the spatial gradient and unprojected Hessian.

## Attributes

On `edges`:

- `bending_stiffness`: $\kappa$ in the energy above; it is not a per-area value
