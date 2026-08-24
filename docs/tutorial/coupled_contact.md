# Rigid-Soft Coupling and Contact

Rigid-soft coupling does not require a special coupling object. Put ABD and
FEM geometries in the same `Scene`, label their collision surfaces, and enable
their material pair in `ContactTabular`. The global nonlinear solve advances
both DOF systems and the IPC barrier transfers forces between them.

## Complete mixed scene

This asset-free program places one deformable tetrahedron, one affine body,
and an implicit plane in a single scene. The soft body has its own contact
element so the rigid-soft pair can use different friction and resistance from
the default rigid-rigid/rigid-ground model.

=== "C++"

    ```cpp
    --8<-- "docs/tutorial/code/rigid_soft.cpp"
    ```

=== "Python"

    ```python
    --8<-- "docs/tutorial/code/rigid_soft.py"
    ```

The two bodies keep different state models:

| Geometry | Constitution | Dynamic state | Fixed state |
| --- | --- | --- | --- |
| Soft tetrahedron | `StableNeoHookean` | Three position DOFs per vertex | `vertices()/is_fixed` |
| Affine body | `AffineBodyConstitution` | Twelve DOFs per instance | `instances()/is_fixed` |
| Ground | implicit geometry | none | static by construction |

Grouping geometries under one object would not couple them mechanically.
Contact couples them because their surfaces share an enabled table entry.

## How the contact table is resolved

`ContactTabular` is symmetric: inserting `(rigid, soft)` defines both lookup
directions. Every geometry has one contact-element ID. An untagged geometry
uses the default element, while `create(name)` allocates a new element.

The example resolves these pairs:

| Pair | Model used |
| --- | --- |
| rigid-ground | default: friction `0.3`, resistance `1 GPa` |
| rigid-soft | explicit: friction `0.5`, resistance `300 MPa` |
| soft-ground | table fallback/default behavior unless explicitly inserted |

If a pair needs precise behavior, insert it explicitly rather than relying on
fallback. For example:

=== "C++"

    ```cpp
    table.insert(soft_contact, rigid_contact,
                 0.0, 300.0_MPa, false); // disable this pair
    ```

=== "Python"

    ```python
    table.insert(
        soft_contact,
        rigid_contact,
        friction_rate=0.0,
        resistance=300.0 * MPa,
        enable=False,
    )
    ```

Apply each element to its geometry before the geometry is copied into a scene
object. Changing the local mesh afterward does not retroactively retag the
scene copy.

## Contact setup checklist

For every expected pair, verify this sequence:

1. `config["contact"]["enable"]` is `true` (the default).
2. Explicit meshes have a collision surface from `label_surface`.
3. Tetrahedral surfaces are oriented with `label_triangle_orient`.
4. Each geometry has the intended `ContactElement`.
5. The pair entry is enabled and has non-negative friction and a sensible
   resistance in scene units.
6. Initial geometry is not intersecting; keep the sanity check enabled while
   building a scene.
7. `d_hat`, thickness, mesh scale, and initial separation are mutually
   consistent.

Friction additionally requires `contact/friction/enable = true`, which is the
default. A pairwise friction coefficient of zero remains frictionless.

## Resistance, ABD stiffness, and FEM modulus are different

- ABD `kappa` penalizes non-rigid affine deformation of one body.
- FEM Young's modulus controls deformation inside a soft body.
- Contact resistance controls the pairwise normal barrier.

They use stiffness-like units but act on different energies. Increasing all
three to solve an overlap usually hides a scale or initialization error and
makes Newton/PCG convergence harder.

The backend computes a scene-adaptive contact-stiffness corridor when possible
and clamps non-negative user resistance into it. Negative contact stiffness is
the opt-in marker for adaptive kappa. See the full
[Contact and Collision reference](../specification/scene_configs/contact.md)
for the exact defaults and precedence rules.

## Solver behavior in mixed scenes

ABD and FEM Hessian blocks enter one global linear system. The FEM
preconditioner selector affects FEM blocks only; ABD keeps its own block
preconditioner. With `linear_system/fem_preconditioner = "mas"`, all non-empty
FEM meshes are partitioned automatically using backend-compatible clusters.
Use it as a measured performance option, especially for stiff deformables,
not as a requirement for physical coupling.

## Deeper examples and implementation evidence

- ABD/FEM stack:
  [`90_abd_fem_cube_stack`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/90_abd_fem_cube_stack/main.py)
- rigid wall with cloth/FEM:
  [`93_cube_wall_cloth`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/93_cube_wall_cloth/main.py)
- material-dependent rigid contact:
  [`10_ramp_sliding`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/10_ramp_sliding/main.py)
- C++ mixed-contact regressions:
  [`18_abd_fem_contact.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/18_abd_fem_contact.cpp)
  and
  [`41_abd_fem_mesh_d_hat.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/41_abd_fem_mesh_d_hat.cpp)
- contact masks and multiple models:
  [`31_contact_mask.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/31_contact_mask.cpp)
  and
  [`8_abd_multi_contact_model.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/8_abd_multi_contact_model.cpp)
- public table API:
  [`contact_tabular.h`](https://github.com/spiriMirror/libuipc/blob/main/include/uipc/core/contact_tabular.h)
