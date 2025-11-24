# External Wrench for Affine Body

**AffineBodyExternalWrench** is a user-defined constitution that applies external 12D generalized forces (wrenches) to affine body instances.

## UID

- **UID**: `666`

## Description

This constitution adds external wrenches to affine bodies, supporting both translational forces and rotational effects through shape velocity derivatives. It is useful for simulating:
- External forces (gravity, wind, thrust)
- Torques and rotational motion
- Combined orbital and spinning dynamics

## Energy Formulation

The potential energy from external wrench in implicit time integration is:

$$
E = -\mathbf{W}^T \mathbf{q} \cdot \Delta t^2
$$

where:
- $\mathbf{W} \in \mathbb{R}^{12}$ is the generalized wrench vector
- $\mathbf{q} \in \mathbb{R}^{12}$ is the affine body state: $\mathbf{q} = [\mathbf{p}, \text{vec}(\mathbf{S})]^T$
  - $\mathbf{p} \in \mathbb{R}^3$ is the position
  - $\mathbf{S} \in \mathbb{R}^{3 \times 3}$ is the shape matrix (flattened to 9D)
- $\Delta t$ is the time step

The wrench vector is structured as:

$$
\mathbf{W} = \begin{bmatrix} \mathbf{f} \\ \text{vec}(\dot{\mathbf{S}}) \end{bmatrix} \in \mathbb{R}^{12}
$$

where:
- $\mathbf{f} \in \mathbb{R}^3$ is the translational force
- $\dot{\mathbf{S}} \in \mathbb{R}^{3 \times 3}$ is the shape velocity derivative (flattened to 9D), encoding rotational effects

### Gradient

The gradient of the energy with respect to the state is:

$$
\frac{\partial E}{\partial \mathbf{q}} = -\mathbf{W} \cdot \Delta t^2
$$

This gradient is accumulated atomically to support thread-safe computation on GPU:
```cpp
muda::eigen::atomic_add(gradients(body_id), -W * dt * dt);
```

### Hessian

Since the wrench is constant (time-independent), the Hessian is zero:

$$
\frac{\partial^2 E}{\partial \mathbf{q}^2} = \mathbf{0}_{12 \times 12}
$$

## Shape Velocity Derivative

For rotation around axis $\hat{\mathbf{n}}$ with angular velocity $\omega$, the shape velocity derivative is:

$$
\dot{\mathbf{S}} = [\hat{\mathbf{n}}]_\times \mathbf{S}
$$

where $[\hat{\mathbf{n}}]_\times$ is the skew-symmetric matrix. For example, rotation around Y-axis:

$$
\dot{\mathbf{S}} = \begin{bmatrix}
0 & 0 & \omega_y \\
0 & 0 & 0 \\
-\omega_y & 0 & 0
\end{bmatrix}
$$

Flattened: $\text{vec}(\dot{\mathbf{S}}) = [0, 0, \omega_y, 0, 0, 0, -\omega_y, 0, 0]^T$

## Usage

### C++

```cpp
#include <uipc/constitution/affine_body_external_wrench.h>

// Create constitution
auto ext_wrench = uipc::constitution::AffineBodyExternalWrench();
scene.constitution_tabular().insert(ext_wrench);

// Apply translational force only
Vector3 force(0, -9.8, 0);
ext_wrench.apply_to(cube, force);

// Apply full 12D wrench (force + shape velocity derivative)
Vector12 wrench;
wrench.segment<3>(0) = Vector3(0, -9.8, 0);  // translational force
wrench.segment<9>(3).setZero();
wrench(5) = 0.1;    // dS13 for rotation
wrench(9) = -0.1;   // dS31 for rotation
ext_wrench.apply_to(cube, wrench);
```

### Python

```python
from uipc.constitution import AffineBodyExternalWrench
import numpy as np

# Create constitution
ext_wrench = AffineBodyExternalWrench()
scene.constitution_tabular().insert(ext_wrench)

# Apply translational force only
force = np.array([0.0, -9.8, 0.0])
ext_wrench.apply_to(cube, force)

# Apply full 12D wrench
wrench = np.zeros(12)
wrench[0:3] = [0.0, -9.8, 0.0]  # translational force
wrench[5] = 0.1                  # dS13 for rotation
wrench[9] = -0.1                 # dS31 for rotation
ext_wrench.apply_to(cube, wrench)
```

## Dynamic Wrench Updates

The ExtraConstitution framework supports dynamic wrench updates through animators:

```python
def animate_wrench(info: Animation.UpdateInfo):
    time = info.dt() * info.frame()

    # Compute time-varying wrench
    angle = 0.2 * time
    force = np.array([np.cos(angle), 0, np.sin(angle)]) * 0.1

    wrench = np.zeros(12)
    wrench[0:3] = force
    wrench[5] = 0.01   # spinning
    wrench[9] = -0.01

    # Update wrench attribute
    for geo_slot in info.geo_slots():
        geo = geo_slot.geometry()
        wrench_attr = geo.instances().find("external_wrench")
        if wrench_attr:
            view(wrench_attr)[:] = wrench.reshape(-1, 1)

scene.animator().insert(object, animate_wrench)
```

## Notes

- The energy is multiplied by $\Delta t^2$ for proper implicit time integration
- The wrench is applied to each affine body instance uniformly
- Shape velocity derivatives enable complex rotational motion without explicit torque formulation
- Gradients are accumulated atomically for thread-safe GPU computation
- For mass-dependent forces (like gravity), multiply the force by the body's mass before applying

## Example

See the test cases:
- C++: [apps/tests/sim_case/affine_body_external_wrench_test.cpp](../../../apps/tests/sim_case/affine_body_external_wrench_test.cpp)
- Python: [python/tests/test_affine_body_external_wrench.py](../../../python/tests/test_affine_body_external_wrench.py)

Both tests demonstrate a cube with combined orbital motion (translational force) and spinning motion (shape velocity derivative).
