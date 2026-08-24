# Pure Volumetric FEM Scenes

A volumetric FEM body is a tetrahedral `SimplicialComplex` whose vertices own
the dynamic degrees of freedom. This page uses `StableNeoHookean`, the standard
large-deformation solid model, without contact so that the constitutive setup
is isolated from collision behavior.

## What defines a deformable solid

| Part | API | Meaning |
| --- | --- | --- |
| Rest mesh | `tetmesh(vertices, tetrahedra)` | Tetrahedral material domain and rest positions. |
| Boundary surface | `label_surface`, `label_triangle_orient` | Extracts and orients boundary triangles for contact and export. |
| Elastic material | `ElasticModuli.youngs_poisson(E, nu)` | Converts Young's modulus and Poisson ratio to Lame parameters. |
| Primary constitution | `StableNeoHookean.apply_to(mesh, moduli, mass_density)` | Creates vertex state, mass, and per-tetrahedron material attributes. |
| Boundary condition | `mesh.vertices()/is_fixed` | `1` fixes one vertex; `0` leaves it dynamic. |
| Contact material | `ContactElement.apply_to(mesh)` | Optional pairwise material identity when contact is enabled. |

The initial vertex positions are the material rest state. Unlike ABD, there is
no per-body transform that moves a normal FEM body without changing that rest
state. Place or transform the mesh before applying the constitution.

## Complete contact-free scene

The following asset-free program drops one tetrahedron under gravity. Contact
is explicitly disabled, so no collision surface, ground, or contact table is
needed for the physics. The surface labels are retained because they are the
normal preparation for later contact or surface export.

=== "C++"

    ```cpp
    --8<-- "docs/tutorial/code/fem.cpp"
    ```

=== "Python"

    ```python
    --8<-- "docs/tutorial/code/fem.py"
    ```

The material in this example uses `E = 50 kPa`, `nu = 0.499`, and density
`1000 kg/m^3`. The public Stable Neo-Hookean overload defaults to `E = 120
kPa`, `nu = 0.49`, and density `1000 kg/m^3`; supplying values explicitly is
recommended because stiffness and scale are scene dependent.

For an ordinary isotropic 3D elastic material, use `E > 0` and
`-1 < nu < 0.5`. Values near `0.5` model near-incompressibility and usually
make the linear system harder. The API rejects the singular endpoints but does
not otherwise enforce a physically meaningful material range.

## Fixing vertices

Apply the FEM constitution first because it creates the `is_fixed` attribute,
then mark selected vertices:

=== "C++"

    ```cpp
    auto is_fixed = solid.vertices().find<IndexT>(builtin::is_fixed);
    auto fixed = view(*is_fixed);
    fixed[0] = 1;
    fixed[1] = 1;
    ```

=== "Python"

    ```python
    import uipc.builtin as builtin
    from uipc import view

    is_fixed = solid.vertices().find(builtin.is_fixed)
    fixed = view(is_fixed)
    fixed[0] = 1
    fixed[1] = 1
    ```

Fixed vertices remain part of the deformable mesh and still participate in
surface contact. To prescribe time-dependent motion rather than a permanent
pin, use a soft position constraint and update its animation state; see the
animation tutorial and the `92_twisting_bar` sample.

## Adding ground contact

Starting from the complete program, remove the line that sets
`config["contact"]["enable"] = false`, then configure and apply a material:

=== "C++"

    ```cpp
    scene.contact_tabular().default_model(0.4, 100.0_MPa);
    auto contact = scene.contact_tabular().default_element();
    contact.apply_to(solid);

    auto floor = ground(0.0);
    contact.apply_to(floor);
    scene.objects().create("floor")->geometries().create(floor);
    ```

=== "Python"

    ```python
    from uipc.geometry import ground
    from uipc.unit import MPa

    scene.contact_tabular().default_model(0.4, 100.0 * MPa)
    contact = scene.contact_tabular().default_element()
    contact.apply_to(solid)

    floor = ground(0.0)
    contact.apply_to(floor)
    scene.objects().create("floor").geometries().create(floor)
    ```

Do this before adding `solid` to the scene. The tetrahedral mesh already has a
labeled surface; omitting `label_surface(solid)` is the common reason a solid
does not collide.

## Mesh and solver considerations

- Tetrahedra must be non-degenerate and consistently oriented. Inspect input
  mesh quality before compensating with solver settings.
- The FEM state is per vertex, so duplicating boundary vertices creates
  disconnected material unless a constraint is added intentionally.
- Material modulus, density, time step, and mesh scale interact. If Newton or
  PCG convergence degrades, first verify units and element quality, then tune
  the documented solver tolerances.
- `linear_system/fem_preconditioner = "mas"` enables the MAS preconditioner
  for every non-empty FEM mesh in the scene. It is a scene-wide performance
  choice, not a material model, and defaults to `"diag"`.
- FEM self-collision is enabled by default by the FEM base constitution. Pair
  masks and the global contact switch still determine whether candidates are
  active.

See [Scene Configuration](../specification/scene_config.md) for exact defaults
and operational domains, and
[Contact and Collision](../specification/scene_configs/contact.md) for
activation distances and material stiffness.

## Deeper examples and implementation evidence

- animated FEM boundary conditions:
  [`3_periodically_pressed_tetrahedron`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/3_periodically_pressed_tetrahedron/main.py)
- larger FEM and MAS setup:
  [`89_mas_bunny`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/89_mas_bunny/main.py)
- C++ gravity, contact, and pin regressions:
  [`13_fem_3d_gravity.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/13_fem_3d_gravity.cpp),
  [`14_fem_3d_ground_contact.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/14_fem_3d_ground_contact.cpp),
  and
  [`15_fem_3d_fixed_point.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/15_fem_3d_fixed_point.cpp)
- public material API:
  [`stable_neo_hookean.h`](https://github.com/spiriMirror/libuipc/blob/main/include/uipc/constitution/stable_neo_hookean.h)
- base FEM attribute initialization:
  [`finite_element_constitution.cpp`](https://github.com/spiriMirror/libuipc/blob/main/src/constitution/finite_element_constitution.cpp)
