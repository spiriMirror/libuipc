# Hookean Spring

**Hookean Spring** is a constitutive model for simulating linear elastic springs connecting two particles in 3D space.

## #12 Hookean Spring

For a spring element connecting two particles at positions $\mathbf{x}_0$ and $\mathbf{x}_1$, we define:

**Distance Vector:**

$$
\mathbf{d} = \mathbf{x}_1 - \mathbf{x}_0
$$

**Current Length:**

$$
L = \|\mathbf{d}\|_2
$$

**Strain:**

$$
\epsilon = \frac{L - L_0}{L_0}
$$

where $L_0$ is the rest length of the spring.

### Strain Energy Density

The strain energy of the Hookean spring is given by:

$$
E = \frac{\kappa}{2} \epsilon^2 = \frac{\kappa}{2} \left(\frac{L - L_0}{L_0}\right)^2
$$

Substituting the expressions for $L$:

$$
E = \frac{\kappa}{2} \left(\frac{\|\mathbf{d}\|_2 - L_0}{L_0}\right)^2
$$

where:

- $\kappa$ is the spring constant (stiffness parameter)

- $L_0$ is the rest length of the spring

- $\mathbf{d}$ is the current displacement vector between the two particles