# Neo Hookean Shell

**Neo Hookean Shell** is a constitutive model designed for simulating thin shell structures, based on a triangle element.

For the sake of simplicity, we denote the three vertices of the triangle as $\mathbf{x}_0$, $\mathbf{x}_1$, and $\mathbf{x}_2$. The rest shape positions are denoted as $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, and $\bar{\mathbf{x}}_2$.

## #11 Neo Hookean Shell

For a triangular shell element with vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, and $\mathbf{x}_2$, we define the edge vectors:

$$
\begin{aligned}
\mathbf{e}_{01} = \mathbf{x}_1 - \mathbf{x}_0\\
\mathbf{e}_{02} = \mathbf{x}_2 - \mathbf{x}_0
\end{aligned}
$$

The current metric tensor is constructed as:

$$
\mathbf{A} = \begin{bmatrix}
\mathbf{e}_{01} \cdot \mathbf{e}_{01} & \mathbf{e}_{01} \cdot \mathbf{e}_{02} \\
\mathbf{e}_{02} \cdot \mathbf{e}_{01} & \mathbf{e}_{02} \cdot \mathbf{e}_{02}
\end{bmatrix}
$$

Similarly, for the reference configuration with positions $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, and $\bar{\mathbf{x}}_2$:

$$
\begin{aligned}
\bar{\mathbf{e}}_{01} = \bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0\\
\bar{\mathbf{e}}_{02} = \bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_0
\end{aligned}
$$

$$
\bar{\mathbf{B}} = \begin{bmatrix}
\bar{\mathbf{e}}_{01} \cdot \bar{\mathbf{e}}_{01} & \bar{\mathbf{e}}_{01} \cdot \bar{\mathbf{e}}_{02} \\
\bar{\mathbf{e}}_{02} \cdot \bar{\mathbf{e}}_{01} & \bar{\mathbf{e}}_{02} \cdot \bar{\mathbf{e}}_{02}
\end{bmatrix}
$$

Here we use $\bar{\mathbf{B}}$ to denote the reference metric tensor.

### Deformation Energy Density

The Neo-Hookean shell deformation energy density is given by:

$$
\Psi = \frac{\mu}{2} \left( \text{tr}(\bar{\mathbf{B}}^{-1} \mathbf{A}) - 2 - 2\ln J \right) + \frac{\lambda}{2} (\ln J)^2
$$

where:

- $\mu$ is the shear modulus

- $\lambda$ is the Lamé parameter  

- $J = \sqrt{\det(\mathbf{A}) \det(\bar{\mathbf{B}}^{-1})}$ is the determinant of the deformation

- $\ln J = \frac{1}{2}\ln(\det(\mathbf{A}) \det(\bar{\mathbf{B}}^{-1}))$ is the logarithmic strain

## Attributes

On `triangles`:

- `mu`: $\mu$ in the energy above
- `lambda`: $\lambda$ in the energy above
