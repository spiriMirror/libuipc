# Tutorials

This section explains how to assemble libuipc scenes from the public C++ and
Python APIs. It is a technical recipe set, not a showcase gallery. Focused
screenshots or short clips are retained when they explain an API outcome; the
project-wide showcase belongs on the
[libuipc project homepage](https://github.com/spiriMirror/libuipc). Larger
runnable scenes live in
[`libuipc-samples`](https://github.com/spiriMirror/libuipc-samples).

## Recommended learning path

1. [How to Assemble a Scene](scene_setup.md) explains ownership, lifecycle,
   topology, constitutions, and state attributes.
2. Choose the physical recipe closest to your application:
   [Rigid Bodies](rigid_bodies.md), [Volumetric FEM](fem.md),
   [Cloth](cloth.md), or [Rigid-Soft Coupling and Contact](coupled_contact.md).
3. Read [Geometry](geometry.md) for mesh construction and attributes, then
   [Animation](animation.md) for time-dependent inputs.
4. Use the [full scene configuration reference](../specification/scene_config.md)
   when selecting solver, collision, and performance controls.
5. Use [Profiling](profiling.md) only after a representative scene is valid
   and physically scaled.

## Choose a recipe

| Goal | Start here | Key decisions |
| --- | --- | --- |
| Only rigid/stiff objects | [Pure Rigid-Body Scenes](rigid_bodies.md) | ABD stiffness, per-instance transforms, contact materials. |
| Only volumetric deformables | [Pure Volumetric FEM Scenes](fem.md) | Tet quality, elastic moduli, vertex constraints, optional contact. |
| Cloth or thin sheets | [Cloth and Thin-Shell Scenes](cloth.md) | Membrane vs bending, thickness, pins, self-contact. |
| Rigid and soft bodies together | [Rigid-Soft Coupling and Contact](coupled_contact.md) | Contact elements, pair table, mixed DOFs, solver scaling. |
| Time-dependent boundaries | [Animation](animation.md) | Scene updates, constraints, and frame lifecycle. |

Every physical recipe contains complete, asset-free C++ and Python programs.
The code is intentionally small enough to expose the assembly order; follow
the linked sample and regression cases for larger systems.

## Version and units

These pages track the `main` branch public API and the CUDA backend behavior
that consumes it. libuipc does not impose a global unit system. The examples
use SI-like values (metres, kilograms, seconds, pascals), and all related
geometry, density, gravity, modulus, contact distance, and stiffness values
must use one consistent system.

Configuration keys are strict: an unknown key is rejected during scene
configuration. Numeric values are not all centrally range-checked, so the
reference distinguishes enforced constraints from backend-operational or
physically meaningful domains.
