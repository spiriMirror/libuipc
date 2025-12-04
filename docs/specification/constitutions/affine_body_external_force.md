# External Force for Affine Body

**AffineBodyExternalForce** is a constraint-based constitution that applies external 12D generalized forces to affine body instances.

## UID

- **UID**: `666`

## Description

This constitution adds external forces to affine bodies through a constraint mechanism, supporting both translational forces and affine forces. It is useful for simulating:
- External forces (gravity, wind, thrust)
- Torques and rotational effects
- Combined translational and affine dynamics

## Architecture

Unlike traditional energy-based constitutions, `AffineBodyExternalForce` uses the **Constraint** architecture:

1. **ExternalForceConstraint**: Reads `external_force` attribute from geometry and stores raw forces
2. **GlobalExternalForceManager**: New lifecycle phase after animator update
3. **ABDExternalForceReporter**: Computes acceleration from forces (`M^{-1} * F`)
4. **ABDTimeIntegrator**: Uses the acceleration in `do_predict_dof`

### Lifecycle

```
Each frame:
1. GlobalAnimator::step()
   └─ ExternalForceConstraint::do_step()
      - Reads "external_force" attribute
      - Stores to body_id_to_external_force_raw

2. GlobalExternalForceManager::step()
   └─ ABDExternalForceReporter::do_step()
      - Computes M^{-1} * F (acceleration)
      - Updates body_id_to_external_force

3. ABDTimeIntegrator::do_predict_dof()
   - Uses: q_tilde = q_prev + q_v*dt + (gravity + external_force)*dt^2
```

## Force Formulation

The external force is applied directly in the time integrator as acceleration:

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

## Usage

### C++

```cpp
#include <uipc/constitution/affine_body_external_force.h>

// Create constitution
auto ext_force = uipc::constitution::AffineBodyExternalForce();
scene.constitution_tabular().insert(ext_force);

// Apply translational force only
Vector3 force(0, -9.8, 0);
ext_force.apply_to(cube, force);

// Apply full 12D force (translational + affine)
Vector12 force12;
force12.segment<3>(0) = Vector3(0, -9.8, 0);  // translational force
force12.segment<9>(3).setZero();              // affine force
force12(5) = 0.1;    // f_a13
force12(9) = -0.1;   // f_a31
ext_force.apply_to(cube, force12);
```

### Python

```python
from uipc.constitution import AffineBodyExternalForce
import numpy as np

# Create constitution
ext_force = AffineBodyExternalForce()
scene.constitution_tabular().insert(ext_force)

# Apply translational force only
force = np.array([0.0, -9.8, 0.0])
ext_force.apply_to(cube, force)

# Apply full 12D force
force12 = np.zeros(12)
force12[0:3] = [0.0, -9.8, 0.0]  # translational force
force12[5] = 0.1                  # f_a13
force12[9] = -0.1                 # f_a31
ext_force.apply_to(cube, force12)
```

## Dynamic Force Updates

The constraint framework supports dynamic force updates through animators:

```python
def animate_force(info: Animation.UpdateInfo):
    time = info.dt() * info.frame()

    # Compute time-varying force
    angle = 0.2 * time
    force = np.array([np.cos(angle), 0, np.sin(angle)]) * 0.1

    force12 = np.zeros(12)
    force12[0:3] = force
    force12[5] = 0.01   # f_a13
    force12[9] = -0.01

    # Update force attribute
    for geo_slot in info.geo_slots():
        geo = geo_slot.geometry()
        force_attr = geo.instances().find("external_force")
        if force_attr:
            view(force_attr)[:] = force12.reshape(-1, 1)

scene.animator().insert(object, animate_force)
```

## Notes

- Uses constraint architecture instead of energy-based approach
- Supports multiple constraints on the same geometry
- Forces are converted to accelerations before integration
- No energy/gradient/hessian computation needed
- Thread-safe parallel computation on GPU
- Attribute name: `external_force` (changed from `external_wrench`)
- Meta attribute: `constraint_uids` (changed from `extra_constitution_uids`)

## Example

See the test cases:
- C++: [apps/tests/sim_case/affine_body_external_force_test.cpp](../../../apps/tests/sim_case/affine_body_external_force_test.cpp)
- Python: [python/tests/test_affine_body_external_force.py](../../../python/tests/test_affine_body_external_force.py)

Both tests demonstrate a cube with combined orbital motion (translational force) and affine dynamics.
