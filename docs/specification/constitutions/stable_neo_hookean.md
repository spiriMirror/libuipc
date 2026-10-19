# Stable Neo Hookean

Stable Neo Hookean constitutions implement a stable version of the classic Neo Hookean material model. This model is particularly well-suited for simulating soft, rubber-like materials due to its ability to handle large deformations while maintaining numerical stability.


There are so many constitutive models called "Neo Hookean", we will distinguish them with UID numbers.

> [Dynamic Deformables:
Implementation and Production
Practicalities (Now With Code!)
](http://www.tkim.graphics/DYNAMIC_DEFORMABLES/)

## #10 Stable Neo Hookean

Deformation energy **density**:

$$
E = \frac{1}{2} \mu (I_c - 3) + \frac{1}{2} \lambda (J - 1 - \mu / \lambda)^2,
$$

where $J = \det(F)$ and $I_c = \|F\|_F^2$.

First Piola-Kirchhoff stress (gradient):

$$
P = \frac{\partial E}{\partial F} = \mu F + (\lambda (J - 1) - \mu)\, \mathrm{cof}(F).
$$

The 9×9 energy Hessian is projected to SPD analytically — twist/flip eigensystem from the QR-SVD of $F$ plus a 3×3 direct eigensolve — instead of a generic eigendecomposition with clamping.

Here $F$ is the deformation gradient, $C=F^T F$ is the right Cauchy-Green
deformation tensor, and $I_c=\operatorname{tr}(C)=\|F\|_F^2$.
The coefficients $\lambda,\mu$ in this SNK1 energy are stored on tetrahedra;
they must be distinguished from the physical Lamé parameters below.

### Material inputs and SNK1 calibration

`ElasticModuli.youngs_poisson(E, nu)` first computes the physical 3D parameters

$$
\mu_L=\frac{E}{2(1+\nu)},\qquad
\lambda_L=\frac{E\nu}{(1+\nu)(1-2\nu)}.
$$

$E$, $\lambda_L$ and $\mu_L$ have units Pa; $\nu$ is dimensionless.
The native C++ and Python `StableNeoHookean.apply_to` defaults are
$E=120000$ Pa, $\nu=0.49$, and mass density $\rho=1000$ kg/m$^3$.
Blender defaults to $E=100000$ Pa, $\nu=0.49$, and $\rho=200$ kg/m$^3$,
and passes these values to the same native API. Tetrahedral energy is weighted
by rest volume, and vertex mass by $\rho$ times lumped rest volume. The collision
radius does not multiply the solid's material stiffness or mass.

The material entry point stores the SNK1 coefficients

$$
\mu=\mu_L,\qquad \lambda=\lambda_L+\mu_L.
$$

The active SNK1 backend uses the energy above without a logarithmic term.
Linearizing its stress at $F=I$, for infinitesimal strain $\epsilon$, gives

$$
P=2\mu\epsilon+(\lambda-\mu)\operatorname{tr}(\epsilon)I
  +O(\|F-I\|^2).
$$

Substituting the stored coefficients recovers Hooke's law with the supplied
physical $\lambda_L,\mu_L$, and therefore the requested small-strain $E,\nu$.
At identity the stress is zero; the energy has an irrelevant constant offset
$\mu^2/(2\lambda)$, which disappears from forces and tangents.

**Migration, 2026-10-19:** earlier versions stored
$\mu=4\mu_L/3$, $\lambda=\lambda_L+5\mu_L/6$. That mapping belongs to the
version with the $-\mu\log(I_c+1)/2$ term
([Smith et al., Section 3.4, Eq. (14)](https://www.tkim.graphics/NEO/StableNeoHookean2018.pdf)),
not the active SNK1 model. The former mapping gave 4/3 of the requested shear
modulus and shifted E/nu (for example, 100 kPa/0.3 behaved as 124.54 kPa/0.214286).
New applications of the constitution now match the physical inputs; existing
scenes may deform differently and should be rebaked or recalibrated as needed.
Serialized tetrahedral `mu`/`lambda` attributes remain raw energy coefficients;
reapply the constitution from physical moduli to update them. Reapplying replaces
the coefficients on every tetrahedron, including previously created attributes.

The usual positive-definite isotropic linear-elastic input domain is finite
$E>0$ and $-1<\nu<0.5$. Native validation is incomplete: it admits nonpositive
or nonfinite $E$ and invalid $\nu$, and incorrectly rejects $\nu=-0.5$.
Blender restricts solid inputs to finite positive $E$ and $0\le\nu<0.5$.
These validation findings remain open and are separate from calibration; see the
[material audit](../../../agent_docs/14-project-audit.md).

## Attributes

On `tetrahedra`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above
