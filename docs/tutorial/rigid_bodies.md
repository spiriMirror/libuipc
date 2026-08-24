# Pure Rigid-Body Scenes

libuipc represents a rigid/stiff object with Affine Body Dynamics (ABD). An
affine body has 12 degrees of freedom: translation plus a full 3x3 affine
transform. `AffineBodyConstitution` adds a shape-preservation energy that keeps
the affine transform close to rigid motion.

This is not an exact six-DOF rigid-body formulation. The stiffness `kappa`
controls how strongly non-rigid affine deformation is suppressed. For
rigid-looking behavior, use a physically scaled positive stiffness and verify
that visible shape change is negligible; the constitution reference suggests
roughly 100 MPa to 100 GPa as the normal working range.

## Anatomy of one ABD body

| Part | API | Meaning |
| --- | --- | --- |
| Rest geometry | `tetmesh(...)` or a closed surface mesh | Local material points used for mass properties and collision surface. |
| Collision labels | `label_surface`, `label_triangle_orient` | Extract the boundary used by contact and give tetrahedral boundary faces an orientation. |
| Physics | `AffineBodyConstitution.apply_to(mesh, kappa, mass_density)` | Creates ABD mass, stiffness, DOF, transform, velocity, and fixed-state attributes. |
| Initial pose | `mesh.transforms()` | Per-instance 4x4 transform. Do not translate local rest vertices to animate body state. |
| Boundary condition | `mesh.instances()/is_fixed` | `1` fixes that entire affine-body instance. |
| Contact material | `ContactElement.apply_to(mesh)` | Selects a row/column in the scene contact table. Untagged geometry uses the default element. |

## Complete minimal scene

This asset-free program drops one affine tetrahedron onto an implicit plane.
There are no FEM constitutions in the scene, so every dynamic explicit body is
ABD.

=== "C++"

    ```cpp
    --8<-- "docs/tutorial/code/rigid_body.cpp"
    ```

=== "Python"

    ```python
    --8<-- "docs/tutorial/code/rigid_body.py"
    ```

The order in the example is intentional:

1. create and label topology;
2. apply ABD so instance attributes exist;
3. apply the contact element;
4. write the per-instance transform; and
5. add the finished geometry to the scene.

## Fixing a body

ABD fixed state is per instance, not per vertex. Set it after applying the
constitution:

=== "C++"

    ```cpp
    auto is_fixed = body.instances().find<IndexT>(builtin::is_fixed);
    view(*is_fixed)[0] = 1;
    ```

=== "Python"

    ```python
    import uipc.builtin as builtin
    from uipc import view

    is_fixed = body.instances().find(builtin.is_fixed)
    view(is_fixed)[0] = 1
    ```

Setting selected vertex flags on a normal ABD mesh does not create a partially
fixed rigid body; vertex-level boundary conditions belong to FEM/cloth.

## Sharing one rest mesh across instances

ABD instances share topology and rest-space vertices but own independent
transforms, velocities, stiffness values, and fixed flags. Resize the instance
collection **before** applying ABD so all generated instance attributes have
the right size:

=== "C++"

    ```cpp
    cube.instances().resize(20);
    abd.apply_to(cube, 100.0_MPa, 1000.0);

    auto transforms = view(cube.transforms());
    auto fixed = view(*cube.instances().find<IndexT>(builtin::is_fixed));
    for(SizeT i = 0; i < transforms.size(); ++i)
    {
        Transform t = Transform::Identity();
        t.translation() = Vector3{0.0, 0.4 * i, 0.0};
        transforms[i] = t.matrix();
        fixed[i] = 0;
    }
    ```

=== "Python"

    ```python
    cube.instances().resize(20)
    abd.apply_to(cube, 100.0 * MPa, 1000.0)

    transforms = view(cube.transforms())
    fixed = view(cube.instances().find(builtin.is_fixed))
    for i in range(len(transforms)):
        t = Transform.Identity()
        t.translate(Vector3.Values([0.0, 0.4 * i, 0.0]))
        transforms[i] = t.matrix()
        fixed[i] = 0
    ```

Instancing is preferable to duplicating identical meshes when only pose and
per-body state differ.

## Mass and stiffness

The standard overload computes mass properties from mesh geometry and
`mass_density` (default 1000 kg/m^3). A closed, consistently oriented shape is
therefore important. The public API also provides overloads for an explicit
12x12 ABD mass matrix and `create_proxy(...)` helpers for bodies whose mass
properties come from another source; proxy bodies have no collision geometry
until a collision shape is supplied separately.

`kappa` is shape-preservation stiffness, not contact resistance. Contact
resistance is the second argument of `ContactTabular.default_model(...)` or
`insert(...)`. Changing one does not change the other.

## Contact and self-collision

`AffineBodyConstitution` creates `self_collision = 0` by default. That is
usually correct for a single closed rigid body: triangles from the same body
should not collide with one another. Contact between different ABD instances
or geometries still follows the contact table.

For material-dependent friction, assign contact elements before adding the
geometry. See [Rigid-Soft Coupling and Contact](coupled_contact.md) for a full
pairwise table.

## Deeper examples and implementation evidence

- Python starting point:
  [`1_hello_libuipc`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/1_hello_libuipc/main.py)
- larger ABD systems:
  [`6_wrecking_balls`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/6_wrecking_balls/main.py)
- material-dependent sliding:
  [`10_ramp_sliding`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/10_ramp_sliding/main.py)
- C++ regression baselines:
  [`0_abd_gravity.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/0_abd_gravity.cpp)
  and
  [`8_abd_multi_contact_model.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/8_abd_multi_contact_model.cpp)
- public constitution API:
  [`affine_body_constitution.h`](https://github.com/spiriMirror/libuipc/blob/main/include/uipc/constitution/affine_body_constitution.h)
