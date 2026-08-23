# Stable Neo Hookean

Stable Neo Hookean constitutions implement a stable version of the classic Neo Hookean material model. This model is particularly well-suited for simulating soft, rubber-like materials due to its ability to handle large deformations while maintaining numerical stability.


There are so many constitutive models called "Neo Hookean", we will distinguish them with UID numbers.

> [Dynamic Deformables:
Implementation and Production
Practicalities (Now With Code!)
](http://www.tkim.graphics/DYNAMIC_DEFORMABLES/)

## #10 Stable Neo Hookean

Since v0.0.26, this constitution is [Stiff-GIPC](https://github.com/KemengHuang/Stiff-GIPC)'s **SNK1** (energy, gradient, and analytically SPD-projected Hessian ported verbatim from `femEnergy.cu`).

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

In continuum mechanics, $F$ is called the deformation gradient, $\lambda$ and $\mu$ are the Lamé parameters. $I_c$ is the first invariant of the right Cauchy-Green deformation tensor $C = \|F\|_F^2$, $\|\cdot\|_F$ is the [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm).

## Attributes

On `tetrahedra`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above