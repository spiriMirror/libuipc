# Strain Plastic Discrete Shell Bending

**StrainPlasticDiscreteShellBending** is a constitutive model for simulating the bending behavior of thin shell structures with residual creases. This model yields on the wrapped dihedral-angle increment, i.e. the generalized bending strain, and evolves the bending rest angle after yielding.

Reference:

- [Discrete Shell](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf)

## #31 StrainPlasticDiscreteShellBending

For a shell bending element defined by four vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and $\mathbf{x}_3$, where $(\mathbf{x}_1, \mathbf{x}_2)$ forms the shared edge between two adjacent triangular faces, we define:

**Dihedral Angle:**

The dihedral angle $\theta$ is the angle between the two triangular faces sharing the edge $(\mathbf{x}_1, \mathbf{x}_2)$. The first triangle is formed by vertices $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and the second triangle is formed by vertices $\mathbf{x}_1$, $\mathbf{x}_2$, $\mathbf{x}_3$.

**Rest Configuration Parameters:**

From the reference configuration with rest positions $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, $\bar{\mathbf{x}}_2$, and $\bar{\mathbf{x}}_3$:

- **Rest Length:** $L_0 = \|\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1\|_2$

- **Average Height:** $\bar{h}$ is computed as:
  $$
  \bar{h} = \frac{A}{3L_0}
  $$
  where
  $$
  A = \frac{1}{2}\left(\|(\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0) \times (\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0)\|_2 + \|(\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_3) \times (\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_3)\|_2\right)
  $$

- **Rest Angle State:** $\bar{\theta}$ is initialized from the rest dihedral angle and then treated as an internal variable.

Define the wrapped angle difference
$$
\Delta\theta = \operatorname{wrap}(\theta - \bar{\theta}),
$$
where $\operatorname{wrap}(\cdot)$ maps an angle to the principal branch $[-\pi,\pi]$.

### Bending Energy Density

The bending energy density is
$$
E = \kappa \frac{\Delta\theta^2 L_0}{\bar{h}},
$$
where:

- $\kappa$ is the bending stiffness parameter
- $\theta$ is the current dihedral angle
- $\bar{\theta}$ is the current bending rest angle state
- $L_0$ is the rest length of the shared edge
- $\bar{h}$ is the average height parameter from the rest configuration

### Plastic Evolution

Let $\theta_y$ denote the yield threshold and let $H$ denote the hardening modulus. The yielding condition is written in the generalized strain space as
$$
|\Delta\theta| > \theta_y.
$$

The plastic increment is
$$
\Delta\theta_p = \max(|\Delta\theta| - \theta_y, 0).
$$

When $\Delta\theta_p > 0$, the internal variables evolve as
$$
\bar{\theta} \leftarrow \bar{\theta} + \operatorname{sign}(\Delta\theta)\Delta\theta_p,
$$
$$
\theta_y \leftarrow \theta_y + H \Delta\theta_p.
$$

The evolution of $\bar{\theta}$ stores the residual crease in the bending rest configuration. The case $H=0$ corresponds to perfect plasticity, while $H>0$ increases the admissible elastic bending range as plastic bending accumulates.

## Attributes

On `edges`:

- `bending_stiffness`: $\kappa$ in the energy above
- `bending_yield_threshold`: $\theta_y$ in the yielding condition
- `bending_hardening_modulus`: $H$ in the hardening rule
