# Kirchhoff Rod Bending

**Kirchhoff Rod Bending** is a constitutive model for simulating the bending behavior of thin rod structures. This model captures the curvature-based bending energy of rod elements defined by three consecutive vertices.

Reference:

- [Codimensional Incremental Potential Contact (C-IPC) Section 8](https://ipc-sim.github.io/C-IPC/)

## #15 Kirchhoff Rod Bending

For a rod bending element defined by three consecutive vertices at positions $\mathbf{x}_0$, $\mathbf{x}_1$, and $\mathbf{x}_2$, we define:

**Edge Vectors:**

$$
\begin{aligned}
\mathbf{e}_0 &= \mathbf{x}_1 - \mathbf{x}_0\\
\mathbf{e}_1 &= \mathbf{x}_2 - \mathbf{x}_1
\end{aligned}
$$

**Curvature Vector:**

The curvature vector $\boldsymbol{\kappa}$ is computed as:

$$
\boldsymbol{\kappa} = \frac{2 \mathbf{e}_0 \times \mathbf{e}_1}{\sqrt{\mathbf{e}_0 \cdot \mathbf{e}_0 \cdot \mathbf{e}_1 \cdot \mathbf{e}_1} + \mathbf{e}_0 \cdot \mathbf{e}_1}
$$

**Rest Length:**

The rest length $L_0$ is the sum of the lengths of the two edges in the reference configuration:

$$
L_0 = \|\bar{\mathbf{x}}_1 - \bar{\mathbf{x}}_0\|_2 + \|\bar{\mathbf{x}}_2 - \bar{\mathbf{x}}_1\|_2
$$

where $\bar{\mathbf{x}}_0$, $\bar{\mathbf{x}}_1$, and $\bar{\mathbf{x}}_2$ are the rest positions of the three vertices.

### Bending Energy

The bending energy of the Kirchhoff rod element is given by:

$$
E = \frac{\alpha \|\boldsymbol{\kappa}\|^2}{L_0}
$$

where:

$$
\alpha = \frac{k r^4 \pi}{4}
$$

and:

- $k$ is the bending stiffness parameter
- $r$ is the radius of the rod cross-section
- $\pi$ is the mathematical constant
- $L_0$ is the rest length of the two edges
- $\boldsymbol{\kappa}$ is the curvature vector

The energy penalizes deviations from the straight configuration, with the stiffness scaling with the fourth power of the rod radius, consistent with classical beam theory.

