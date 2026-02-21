# ARAP

As Rigid As Possible (ARAP) constitution

## #9 ARAP

[Dynamic Deformables:
Implementation and Production
Practicalities (Now With Code!)
](http://www.tkim.graphics/DYNAMIC_DEFORMABLES/)

As Rigid As Possible (ARAP) deformation energy density per tetrahedron is defined as:

$$
V = \kappa \|\mathbf{F}-\mathbf{R}\|_F^2 = \kappa \|\mathbf{S}-\mathbf{I}\|_F^2
$$

where $\kappa$ is the stiffness parameter, $\mathbf{R}\mathbf{S}$ is the polar decomposition of deformation gradient $\mathbf{F}$. $\kappa$ could be $1 kPa$ ~ $10 MPa$ depending on the material. If you want a material with $\kappa$ larger than $10MPa$, consider directly using Affine Body for better performance.

## Attributes

On `tetrahedra`:

- `kappa`: $\kappa$ in the energy above
