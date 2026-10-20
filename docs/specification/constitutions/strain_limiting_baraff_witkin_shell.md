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

The rest shape matrix $\mathbf{D}_m \in \mathbb{R}^{2 \times 2}$ is obtained by rotating the rest triangle normal toward $+Z$, then extracting the XY in-plane edge vectors. The deformation gradient is:

$$
\mathbf{F} = \mathbf{D}_s \, \mathbf{D}_m^{-1} \in \mathbb{R}^{3 \times 2}
$$

## Strain Measures

Given two orthogonal material directions $\hat{\mathbf{a}} = (1, 0)^T$ and $\hat{\mathbf{b}} = (0, 1)^T$, the following invariants are defined:

These directions are fixed in the existing per-triangle rest frame. On a
rest mesh in XY with $+Z$ normals they coincide with world X and Y. The API
does not currently accept UV coordinates or an arbitrary fiber orientation.

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
\Psi = K_a \phi(I_{5u}) + K_b \phi(I_{5v}) + K_g I_6^2
$$

where:

- $K_a$ and $K_b$ are independent directional stretch coefficients;
- $K_g$ is the independent shear coefficient;
- the old API sets $K_a=K_b=K_s$.

The attribute name `lambda` is historical: it stores $K_a$ (or the shared
$K_s$), not a physical Lamé parameter.

### Stretch Energy

$$
\phi(s) = (s-1)^2 + \eta \max(s-1,0)^3
$$

where $\eta$ is the dimensionless `strain_rate` parameter shared by both
directions. With equal coefficients, the legacy stretch term is
$\Psi_{\text{stretch}}=\phi(I_{5u})+\phi(I_{5v})$.

The quadratic term penalizes both compression and extension. The cubic term is only active when the material is stretched beyond its rest length ($I_5 > 1$), providing asymmetric strain limiting that progressively stiffens under large extension.

### Shear Energy

$$
\Psi_{\text{shear}} = I_6^2
$$

## Total Energy

Let the stored vertex `thickness` be the one-sided collision offset $r_t$, so
the full material thickness is $h=2r_t$. The effective coefficients are

$$
K_a = (\lambda_a+2\mu_a)h
= \frac{E_a(2r_t)}{1-\nu_a^2},
\qquad
K_b = (\lambda_b+2\mu_b)h
= \frac{E_b(2r_t)}{1-\nu_b^2},
\qquad
K_g
= \mu_{sh}
= \frac{E_{sh}}{2(1+\nu_{sh})}.
$$

The total energy for each triangle element is:

$$
E = A_0\left(K_a\phi(I_{5u}) + K_b\phi(I_{5v}) + K_g I_6^2\right).
$$

Here $A_0$ is the rest area. Stretch uses the full shell thickness $2r_t$.
Shear remains an independently calibrated two-dimensional effective
coefficient and is intentionally thickness-independent.

The solver multiplies this physical energy and its spatial derivatives by
$\Delta t^2$. The gradient uses the matching directional weights; the solver
Hessian weights the separately projected stretch blocks by $K_a,K_b$ and the
projected shear contribution by $K_g$. It is a positive-semidefinite
approximation rather than the unprojected second derivative in compression.

The squared stretch metric in derivative evaluation is floored at $10^{-12}$
to guard reciprocal stretch. The energy itself uses the unmodified norm.
This is not a guarantee of a smooth or well-conditioned model at degenerate
triangles.

## Parameters

The strain rate parameter $\eta$ is set via the `strain_rate` argument of `apply_to` (default 100) and controls the strength of the cubic extension penalty.

The one-sided thickness radius $r_t$ and mass density are inherited from the
[Finite Element](./finite_element.md) base and set via `apply_to`.

### Directional parameter interpretation

Each $(E,\nu)$ pair retains the existing `ElasticModuli2D` coefficient
conversion. Changing $\nu_a$ changes only $K_a$, and changing $\nu_b$ changes
only $K_b$. These inputs do not calibrate physical Young's moduli and
transverse Poisson ratios: there is no cross term between the normal strains.
For example, $\nu_a=0.3$ does not prescribe 0.3% transverse contraction for
1% axial extension. Shear, geometry and boundary conditions can still couple
the resulting motion.

A physical orthotropic plane-stress law would require normal-strain coupling
and reciprocal Poisson parameters, $\nu_{ab}/E_a=\nu_{ba}/E_b$. That is a
different material calibration and is not introduced by this extension.

Use finite positive directional E values, $-1<\nu<1$, positive thickness
and mass density, and $\eta\ge0$. General material-domain validation remains
incomplete; these are intended input domains, not a new validation guarantee.

### API compatibility

- `apply_to(sc, moduli, ...)` keeps all three channels derived from one pair.
- `apply_to(sc, stretch_moduli, shear_moduli, ...)` keeps shared directional
  stretch and independent shear.
- The new overload takes `stretch_a_moduli`, `stretch_b_moduli` and
  `shear_moduli`, all required. In Python these three arguments and the trailing
  settings are keyword-only; old positional calls and keyword names retain
  their meanings.
- Defaults remain density 200 kg/m³, one-sided radius 0.001 m and strain rate
  100. The convenience modulus remains E=1 MPa, nu=0.49.
- Reapplying either old overload removes the independent b override and
  restores shared stretch. Reapplying the new overload replaces all triangle
  coefficients.

See the [cloth tutorial](../../tutorial/cloth.md) and sample
[`95_anisotropic_cloth`](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/95_anisotropic_cloth)
for API usage and a biaxial comparison.

## Attributes

On `triangles`:

- `mu`: $K_g$, the effective shear stiffness;
- `lambda`: $K_a$, or the shared $K_s$ when no b override is present;
- optional `stretch_stiffness_b`: $K_b$. Absence means use that triangle's
  `lambda` for both directions. The backend does not materialize this fallback,
  preserving legacy per-triangle `lambda` edits;
- `strain_rate`: $\eta$. Legacy scenes without it use 100.

Generic scene serialization preserves the optional attribute. Old scenes load
with their original shared behavior in the new runtime; older runtimes do not
understand the independent b coefficient in newly saved anisotropic scenes.
