# External Force for Affine Body

## #666 AffineBodyExternalForce

The **Affine Body External Force** applies external 12D generalized forces to affine body instances through a constraint mechanism, supporting both translational forces and affine (rotational) forces.

The external force is applied as acceleration:

$$
\mathbf{a}_{ext} = \mathbf{M}^{-1} \mathbf{F}_{ext}
$$

where:
- $\mathbf{F}_{ext} \in \mathbb{R}^{12}$ is the generalized force vector
- $\mathbf{M} \in \mathbb{R}^{12 \times 12}$ is the affine body mass matrix
- $\mathbf{a}_{ext} \in \mathbb{R}^{12}$ is the resulting acceleration

The force vector is structured as:

$$
\mathbf{F}_{ext} = \begin{bmatrix} \mathbf{f} \\ \mathbf{f}_a \end{bmatrix} \in \mathbb{R}^{12}
$$

where:
- $\mathbf{f} \in \mathbb{R}^3$ is the translational force
- $\mathbf{f}_a \in \mathbb{R}^{9}$ is the affine force (flattened 3x3 matrix)

The energy function for the **Affine Body External Force** will be incorporated into the Affine Body Kinetic Term as:

$$
E = \frac{1}{2} \left(\mathbf{q} - \tilde{\mathbf{q}}\right)^T \mathbf{M} \left(\mathbf{q} - \tilde{\mathbf{q}}\right) 
$$

where $\tilde{\mathbf{q}}$ is updated each time step by the velocity and acceleration, the formula depends on the time integration scheme used.

Typically, when `bdf1` (implicit euler) is used:

$$
\tilde{\mathbf{q}} = \mathbf{q}^t + \Delta t \cdot \mathbf{v}^t + \Delta t^2 \cdot (\mathbf{a}_{ext} + \mathbf{g}),
$$

where $\mathbf{q}^t$, is the previous state vector at time $t$, $\mathbf{v}^t$ is the previous velocity vector at time $t$, $\Delta t$ is the time step size, and $\mathbf{g}$ is the gravitational acceleration.

Users are allowed to modify the external force $\mathbf{F}_{ext}$ using the `uipc` Animator System interface during the simulation.

