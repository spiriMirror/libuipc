# Stress Plastic Discrete Shell Bending

**StressPlasticDiscreteShellBending** is a constitutive model for simulating the elastoplastic bending behavior of thin shell structures with residual creases. To ensure stable implicit time integration and physically accurate spring-back effects, this model adopts the **Energy-based Consistent Integration (ECI)** framework. It defines the yield condition on the generalized bending stress (bending moment) rather than the bending strain, variationally limiting the stress to the yield surface via an augmented energy density.

Reference:
- [Discrete Shell](https://www.cs.columbia.edu/cg/pdfs/10_ds.pdf)
- [Energy-based Consistent Integration for Elastoplasticity](https://dl.acm.org/doi/epdf/10.1145/3528223.3530072) (Section 4.1)

## #32 StressPlasticDiscreteShellBending (Stress-Based ECI)

For a shell bending element defined by four vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, $\mathbf{x}_2$, and $\mathbf{x}_3$, where $(\mathbf{x}_1, \mathbf{x}_2)$ forms the shared edge between two adjacent triangular faces.

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

- **Rest Angle State:** $\bar{\theta}^n$ is the historical rest angle state from the previous time step $n$.

Define the wrapped trial angle difference:
$$
\Delta\theta^{tr} = \operatorname{wrap}(\theta - \bar{\theta}^n)
$$
where $\operatorname{wrap}(\cdot)$ maps an angle to the principal branch $[-\pi,\pi]$.

### Kinematics and Yield Condition

The purely elastic bending energy density is defined as:
$$
E^E(\Delta\theta) = \kappa \frac{\Delta\theta^2 L_0}{\bar{h}}
$$
where $\kappa$ is the bending stiffness parameter.

The generalized trial bending stress (trial moment) is the derivative of the elastic energy with respect to the deformation angle:
$$
\tau^{tr} := \frac{\partial E^E(\Delta\theta^{tr})}{\partial \Delta\theta^{tr}} = 2\kappa \frac{L_0}{\bar{h}} \Delta\theta^{tr}
$$

Let $\tau_Y$ denote the constant yield bending stress. The yielding condition is defined in the stress space as:
$$
|\tau^{tr}| > \tau_Y
$$

This physically implies a geometry-dependent critical yield angle $\theta_Y$:
$$
\theta_Y = \frac{\tau_Y \bar{h}}{2\kappa L_0}
$$

### Augmented Bending Energy Density (ECI)

To formulate a force-based implicit return mapping that perfectly preserves the elastic spring-back force, we construct an augmented energy density. The plastic slip is measured by:
$$
\delta \gamma = \max(|\Delta\theta^{tr}| - \theta_Y, 0)
$$

The augmented energy density replaces the standard elastic energy in the variational solver:
$$
E_{aug}(\theta) = \begin{cases} \kappa \frac{L_0}{\bar{h}} (\Delta\theta^{tr})^2 & |\Delta\theta^{tr}| \le \theta_Y \\ \kappa \frac{L_0}{\bar{h}} \theta_Y^2 + \tau_Y \delta \gamma & \text{otherwise} \end{cases}
$$

*Note: This augmented energy function is $C^1$ continuous. Its analytical derivative smoothly transitions from the linear elastic force $2\kappa \frac{L_0}{\bar{h}}\Delta\theta^{tr}$ to the constant yield force $\operatorname{sign}(\Delta\theta^{tr})\tau_Y$ in the plastic region, naturally avoiding the zero-force issue.*

### Plastic Evolution

At the end of a converged time step (after the implicit solver finishes), if yielding occurred ($\delta \gamma > 0$), the internal variables evolve to reflect the permanent crease. 

The bending rest angle is updated as:
$$
\bar{\theta}^{n+1} = \bar{\theta}^n + \operatorname{sign}(\Delta\theta^{tr}) \delta \gamma
$$

If an isotropic hardening modulus $H$ is considered, the yield stress evolves as:
$$
\tau_Y \leftarrow \tau_Y + H \delta \gamma
$$

## Attributes

On `edges`:

- `bending_stiffness`: $\kappa$ in the energy formulation
- `bending_yield_stress`: $\tau_Y$ in the stress-based yielding condition
- `bending_hardening_modulus`: $H$ in the hardening rule
