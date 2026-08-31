# Strain Limiting Baraff-Witkin Shell

References:

[StiffGIPC: Advancing GPU IPC for Stiff Affine-Deformable Simulation](https://dl.acm.org/doi/10.1145/3735126)

## #819 StrainLimitingBaraffWitkinShell

**Strain Limiting Baraff-Witkin Shell** is a constitutive model for thin shell simulation based on triangle elements, extending the [Finite Element](./finite_element.md) base. It uses an anisotropic strain measure that separates stretch and shear contributions and applies asymmetric strain limiting: compression is penalized quadratically, while extension beyond the rest length incurs an additional cubic penalty controlled by a strain rate parameter.

For the sake of simplicity, we denote the three vertices of the triangle as $\mathbf{x}_0$, $\mathbf{x}_1$, and $\mathbf{x}_2$. The rest shape positions are denoted as $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, and $\bar{\mathbf{x}}_2$.

## Deformation Gradient

The spatial edge matrix is:

$$
\mathbf{D}_s = \begin{bmatrix} \mathbf{x}_1 - \mathbf{x}_0 & \mathbf{x}_2 - \mathbf{x}_0 \end{bmatrix} \in \mathbb{R}^{3 \times 2}
$$

The rest shape matrix $\mathbf{D}_m \in \mathbb{R}^{2 \times 2}$ is obtained by rotating the rest triangle so that its normal aligns with a canonical axis, then extracting the 2D in-plane edge vectors. The deformation gradient is:

$$
\mathbf{F} = \mathbf{D}_s \, \mathbf{D}_m^{-1} \in \mathbb{R}^{3 \times 2}
$$

## Strain Measures

Given two orthogonal material directions $\hat{\mathbf{a}} = (1, 0)^T$ and $\hat{\mathbf{b}} = (0, 1)^T$, the following invariants are defined:

**Stretch invariants:**

$$
I_{5u} = \| \mathbf{F} \hat{\mathbf{a}} \|, \quad
I_{5v} = \| \mathbf{F} \hat{\mathbf{b}} \|
$$

These measure the stretch along each material direction. When $I_{5u} = 1$ (or $I_{5v} = 1$), the material is at rest length in that direction.

**Shear invariant:**

$$
I_6 = \hat{\mathbf{a}}^T \mathbf{F}^T \mathbf{F} \hat{\mathbf{b}}
$$

This measures the shear between the two material directions.

## Deformation Energy Density

The energy density separates stretch and shear contributions:

$$
\Psi = \lambda \, \Psi_{\text{stretch}} + \mu \, \Psi_{\text{shear}}
$$

where:

- $\lambda$ is the stretch stiffness (Lamé parameter)
- $\mu$ is the shear stiffness

### Stretch Energy

$$
\Psi_{\text{stretch}} = (I_{5u} - 1)^2 + \alpha_u \, r \, (I_{5u} - 1)^3
  + (I_{5v} - 1)^2 + \alpha_v \, r \, (I_{5v} - 1)^3
$$

where $r$ is the strain rate parameter and:

$$
\alpha_u = \begin{cases} 1, & I_{5u} > 1 \\ 0, & I_{5u} \le 1 \end{cases}, \quad
\alpha_v = \begin{cases} 1, & I_{5v} > 1 \\ 0, & I_{5v} \le 1 \end{cases}
$$

The quadratic term penalizes both compression and extension. The cubic term is only active when the material is stretched beyond its rest length ($I_5 > 1$), providing asymmetric strain limiting that progressively stiffens under large extension.

### Shear Energy

$$
\Psi_{\text{shear}} = I_6^2
$$

## Total Energy

Let the stored vertex `thickness` be the one-sided collision offset $r_t$, so
the full material thickness is $h=2r_t$. The effective coefficients are

$$
k_{\mathrm{stretch}}
= (\lambda_s+2\mu_s)h
= \frac{E_s(2r_t)}{1-\nu_s^2},
\qquad
k_{\mathrm{shear}}
= \mu_{sh}
= \frac{E_{sh}}{2(1+\nu_{sh})}.
$$

The total energy for each triangle element is:

$$
E = \left(k_{\mathrm{stretch}}\Psi_{\mathrm{stretch}}
          + k_{\mathrm{shear}}\Psi_{\mathrm{shear}}\right) A_0
$$

Here $A_0$ is the rest area. Stretch uses the full shell thickness $2r_t$.
Shear remains an independently calibrated two-dimensional effective
coefficient and is intentionally thickness-independent.

The directional stretch metric is floored at $10^{-12}$ to guard the $1/\sqrt{I_5}$ singularity at fully collapsed triangles (finite huge response instead of NaN).

## Parameters

The strain rate parameter $r$ is set via the `strain_rate` argument of `apply_to` (default 100) and controls the strength of the cubic extension penalty.

The one-sided thickness radius $r_t$ and mass density are inherited from the
[Finite Element](./finite_element.md) base and set via `apply_to`.

## Attributes

On `triangles`:

- `mu`: $k_{\mathrm{shear}}$, the effective shear stiffness
- `lambda`: $k_{\mathrm{stretch}}$, the full-thickness stretch stiffness
