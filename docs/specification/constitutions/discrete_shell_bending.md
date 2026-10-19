# Discrete Shell Bending

**Discrete Shell Bending** is a constitutive model for simulating the bending behavior of thin shell structures. This model captures the dihedral angle-based bending energy between adjacent triangular faces in a shell mesh.

Reference:

- [Discrete Shell](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf)
- [Discrete bending forces and their Jacobians](https://media.disneyanimation.com/uploads/production/publication_asset/74/asset/main_preprint.pdf), Section 5, Eq. (11)

## #17 Discrete Shell Bending

For a shell bending element defined by four vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and $\mathbf{x}_3$, where $(\mathbf{x}_1, \mathbf{x}_2)$ forms the shared edge between two adjacent triangular faces, we define:

**Dihedral Angle:**

The dihedral angle $\theta$ is the angle between the two triangular faces sharing the edge $(\mathbf{x}_1, \mathbf{x}_2)$. The first triangle is formed by vertices $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and the second triangle is formed by vertices $\mathbf{x}_1$, $\mathbf{x}_2$, $\mathbf{x}_3$.

**Rest Configuration Parameters:**

From the reference configuration with rest positions $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, $\bar{\mathbf{x}}_2$, and $\bar{\mathbf{x}}_3$:

- **Rest Length:** $L_0 = \|\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1\|_2$ is the length of the shared edge in the rest configuration

- **Average Height:** $\bar{h}$ is computed as:
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
- $\bar{h}$ is the average height parameter from the rest configuration

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

## Attributes

On `edges`:

- `bending_stiffness`: $\kappa$ in the energy above; it is not a per-area value
