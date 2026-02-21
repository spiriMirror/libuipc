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
E = \frac{1}{2} \lambda (J - \alpha)^2 + \frac{1}{2} \mu (I_c - 3) - \frac{1}{2} \mu \ln(I_c + 1),
$$

where $J = \det(F)$, $I_c = \|F\|_F^2$, $\alpha = 1 + \frac{3\mu}{4\lambda}$.


In continuum mechanics, $F$ is called the deformation gradient, $\lambda$ and $\mu$ are the Lamé parameters. $I_c$ is the first invariant of the right Cauchy-Green deformation tensor $C = \|F\|_F^2$, $\|\cdot\|_F$ is the [Frobenius norm](https://en.wikipedia.org/wiki/Matrix_norm).

## Attributes

On `tetrahedra`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above