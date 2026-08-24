# How to Assemble a Scene

Every libuipc simulation uses the same assembly order. The physical model
changes from rigid bodies to FEM or cloth, but the ownership and lifecycle do
not.

## The assembly pipeline

1. Create an `Engine` and keep it alive for the lifetime of its `World`.
2. Copy `Scene.default_config()`, change scene-wide controls, and create the
   `Scene`.
3. Configure pairwise contact models in `scene.contact_tabular()`.
4. Create or load geometry.
5. Label the contact surface when the geometry is a `SimplicialComplex`.
6. Apply one primary constitution and any compatible extra constitutions.
7. Set boundary conditions and initial state attributes.
8. Apply a contact element to the geometry when it does not use the default
   material.
9. Create a scene object and add the finished geometry to it.
10. Call `world.init(scene)` once, then alternate `world.advance()` and
    `world.retrieve()`.

The important boundary is step 10. Backend systems discover constitutions,
copy topology, compute rest-state data, and cache configuration during
`world.init(scene)`. Finish initial scene construction before that call.

=== "C++"

    ```cpp
    Engine engine{"cuda", "output/my_scene"};
    World world{engine};

    auto config = Scene::default_config();
    config["dt"] = 0.01;
    Scene scene{config};

    // Build contact table, geometry, constitutions, and objects here.

    world.init(scene);
    if(!world.is_valid())
        throw std::runtime_error("scene initialization failed");

    while(world.frame() < 100)
    {
        world.advance();
        world.retrieve();
    }
    ```

=== "Python"

    ```python
    engine = Engine("cuda", "output/my_scene")
    world = World(engine)

    config = Scene.default_config()
    config["dt"] = 0.01
    scene = Scene(config)

    # Build contact table, geometry, constitutions, and objects here.

    world.init(scene)
    if not world.is_valid():
        raise RuntimeError("scene initialization failed")

    while world.frame() < 100:
        world.advance()
        world.retrieve()
    ```

`advance()` runs the next time step on the backend. `retrieve()` copies the
updated scene state back to the host-side geometry slots. Call `retrieve()`
before reading positions/transforms or exporting that frame.

<figure style="text-align: center">
  <video style="width: 75%" muted controls playsinline preload="metadata"
         poster="./media/falling_tet.png"
         aria-label="Successive retrieved frames of a falling tetrahedron">
    <source src="./media/falling_tet.mp4" type="video/mp4">
  </video>
  <figcaption>The montage shows successive retrieved frames of one tetrahedron, not multiple bodies in one scene.</figcaption>
</figure>

## Choose topology, constitution, and constraints together

| Intended behavior | Geometry | Primary constitution | Fixed-state attribute |
| --- | --- | --- | --- |
| Rigid/stiff body | Closed tetrahedral or closed surface mesh | `AffineBodyConstitution` | `instances()/is_fixed` |
| Volumetric deformable | Tetrahedral mesh | `StableNeoHookean` or another 3D FEM constitution | `vertices()/is_fixed` |
| Cloth/thin sheet | Triangle mesh | a shell membrane constitution; optionally add `DiscreteShellBending` | `vertices()/is_fixed` |
| Mixed rigid-soft | ABD and FEM geometries in the same scene | Each geometry keeps its own constitution | Per-geometry rules above |

A constitution writes the attributes consumed by its backend system. Do not
manually create a `constitution_uid` or copy attributes from a different
material. Construct the topology, call the constitution's `apply_to(...)`,
then edit the documented state attributes it creates.

## Geometry and object ownership

An `Object` is a named group. A `Geometry` carries topology, material
attributes, state, and contact-element identity. One object may own multiple
geometries, but grouping them does not merge meshes or invent physical
constraints between them.

`object.geometries().create(mesh)` places a copy in the scene. Finish changes
to the local mesh first, or retain the returned geometry slot and use the
scene's update/commit workflow for later edits. Changing the old local value
does not implicitly mutate the copy already stored by the scene.

## Contact has three prerequisites

Contact between two explicit meshes exists only when all three layers agree:

1. `config["contact"]["enable"]` is on;
2. their collision surfaces are labeled with `label_surface(...)`; and
3. the `ContactTabular` row for their contact-element IDs is enabled.

An implicit `ground(...)` supplies its own surface. Friction additionally
requires the global friction flag and a non-zero pairwise friction coefficient.
See [Contact and Collision](../specification/scene_configs/contact.md) before
designing material masks.

## Initial-state rules by model

- ABD vertices are local/rest geometry. Initial body pose and velocity are
  per-instance attributes; use `mesh.transforms()` for the pose.
- FEM and cloth positions are the rest positions used to construct their
  deformation energy. Their dynamic state is per vertex.
- Apply a membrane constitution before the formula overload of
  `DiscreteShellBending`, because bending reads the membrane-created thickness
  attribute.
- For a tetrahedral mesh, call both `label_surface` and
  `label_triangle_orient`. The first enables surface contact; the second gives
  boundary triangles a reliable orientation for export and related geometry
  operations.

## Source-backed recipe map

The tutorial programs are deliberately small and asset-free. Larger examples
remain in
[`libuipc-samples`](https://github.com/spiriMirror/libuipc-samples), while the
in-tree C++ simulation cases provide regression coverage:

| Topic | Samples | C++ regression cases |
| --- | --- | --- |
| ABD | `1_hello_libuipc`, `6_wrecking_balls` | `0_abd_gravity`, `6_abd_cube_pile` |
| FEM | `3_periodically_pressed_tetrahedron`, `89_mas_bunny` | `13_fem_3d_gravity`, `14_fem_3d_ground_contact`, `15_fem_3d_fixed_point` |
| Cloth | `11_bunny_cloth`, `34_cloth_stack`, `91_pinned_cloth` | `19_shell_fixed_point`, `33_discrete_shell_bending`, `60_fem_mas_cloth` |
| Rigid-soft | `90_abd_fem_cube_stack`, `93_cube_wall_cloth` | `18_abd_fem_contact`, `41_abd_fem_mesh_d_hat` |
| Contact materials | `10_ramp_sliding` | `8_abd_multi_contact_model`, `31_contact_mask` |

The examples in the following pages were checked against those sources, the
public C++ headers, Python bindings, and the CUDA systems that consume the
attributes.
