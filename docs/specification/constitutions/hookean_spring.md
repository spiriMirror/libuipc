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

The strain energy **density** of the Hookean spring is given by:

$$
\psi = \frac{\kappa}{2} \epsilon^2 = \frac{\kappa}{2} \left(\frac{L - L_0}{L_0}\right)^2
$$

Substituting the expressions for $L$:

$$
\psi = \frac{\kappa}{2} \left(\frac{\|\mathbf{d}\|_2 - L_0}{L_0}\right)^2
$$

where:

- $\kappa$ is the axial material modulus (Pa for a physical circular-section rod), not a length-independent spring constant in N/m

- $L_0$ is the rest length of the spring

- $\mathbf{d}$ is the current displacement vector between the two particles

For a uniform positive cross-section radius $r$, the CUDA constitution multiplies
this density by rest volume $V_0=\pi r^2L_0$. The physical edge energy is therefore

$$
E_{\mathrm{edge}}=V_0\psi
=\frac{\kappa\pi r^2}{2L_0}(L-L_0)^2.
$$

The corresponding linear spring constant is $k_{\mathrm{edge}}=\kappa\pi r^2/L_0$
(N/m). The incremental-potential contribution additionally carries $dt^2$; that
solver weight is not part of the physical energy. Use a positive radius for this
rod model. These weights are consumed in `hookean_spring_1d.cu`; omitting them
would give incorrect units and length/radius scaling.

## Attributes

On `edges`:

- `kappa`: $\kappa$ in the density above
