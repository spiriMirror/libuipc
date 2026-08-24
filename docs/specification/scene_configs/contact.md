# Contact and Collision

Contact setup has two separate layers:

1. scene configuration controls the global algorithm (`contact/enable`,
   activation distance, friction regularization, IPC pipeline, and broad
   phase);
2. `ContactTabular` assigns pairwise material behavior (friction coefficient,
   contact resistance, and whether that pair may contact).

Keeping these layers separate is essential: setting `contact/enable = 1` does
not choose material coefficients, and creating a contact element does not
label a mesh surface.

## Make geometry contact-capable

For a tetrahedral, triangle, edge, or point `SimplicialComplex`, call
`label_surface(mesh)`. It creates the `is_surf` attributes used by the global
surface and collision systems. For tetrahedral meshes,
`label_triangle_orient(mesh)` additionally records boundary orientation; this
is required for reliable surface export and downstream orientation-sensitive
operations.

`ground(height, normal)` creates an implicit half-plane and participates in
contact without surface labeling.

Geometry without an explicit contact element uses element `0`, the default
element. Applying the default element explicitly is still recommended in
tutorial code because it makes the intended material assignment visible.

=== "C++"

    ```cpp
    label_surface(mesh);
    label_triangle_orient(mesh);

    auto default_contact = scene.contact_tabular().default_element();
    default_contact.apply_to(mesh);

    auto floor = ground(0.0);
    default_contact.apply_to(floor);
    ```

=== "Python"

    ```python
    label_surface(mesh)
    label_triangle_orient(mesh)

    default_contact = scene.contact_tabular().default_element()
    default_contact.apply_to(mesh)

    floor = ground(0.0)
    default_contact.apply_to(floor)
    ```

## Default model

`default_model(friction_rate, resistance, enable=True)` defines the fallback
model for every contact-element pair without a specific row.

- `friction_rate` is the Coulomb-style friction coefficient. Use `>= 0`; set
  `contact/friction/enable = 0` to remove friction terms globally.
- `resistance` is contact stiffness in Pa. Larger values enforce a stronger
  barrier but can worsen conditioning.
- `enable=False` disables the default pair response.

=== "C++"

    ```cpp
    scene.contact_tabular().default_model(0.4, 1.0_GPa);
    ```

=== "Python"

    ```python
    from uipc.unit import GPa

    scene.contact_tabular().default_model(0.4, 1.0 * GPa)
    ```

If the application never calls `default_model(...)`, the effective default
resistance is `contact/adaptive/min_kappa` (100 MPa by default). If it does set
a non-negative value, the backend clamps it into
`[contact/adaptive/min_kappa, contact/adaptive/max_kappa]` and warns when a
clamp occurs. A negative resistance is the explicit adaptive-kappa marker and
is intentionally not clamped.

## Pairwise material table

Create named contact elements, apply them to geometries, then insert symmetric
pair rows. Pair lookup is order-independent; `(rubber, steel)` and
`(steel, rubber)` are the same row. A missing row falls back to the default
model.

=== "C++"

    ```cpp
    auto& table = scene.contact_tabular();
    auto steel = table.default_element();
    auto rubber = table.create("rubber");

    table.default_model(0.3, 1.0_GPa);
    table.insert(steel, rubber, 0.8, 300.0_MPa);
    table.insert(rubber, rubber, 1.0, 100.0_MPa);

    steel.apply_to(steel_mesh);
    rubber.apply_to(rubber_mesh);
    ```

=== "Python"

    ```python
    from uipc.unit import GPa, MPa

    table = scene.contact_tabular()
    steel = table.default_element()
    rubber = table.create("rubber")

    table.default_model(0.3, 1.0 * GPa)
    table.insert(steel, rubber, 0.8, 300.0 * MPa)
    table.insert(rubber, rubber, 1.0, 100.0 * MPa)

    steel.apply_to(steel_mesh)
    rubber.apply_to(rubber_mesh)
    ```

To create a collision mask, insert the pair with `enable=False`. This is more
precise than disabling contact globally:

```python
table.insert(rubber, steel, friction_rate=0.0,
             resistance=100.0 * MPa, enable=False)
```

## Activation distance and thickness

The activation distance `d_hat` is not a penetration allowance. IPC begins
building barrier response before primitives reach their geometric thickness.
For a rest-scene diagonal $L$:

$$
d_{\mathrm{hat,eff}} =
\begin{cases}
d_{\mathrm{hat,relative}}L, & d_{\mathrm{hat,relative}} > 0,\\
d_{\mathrm{hat}}, & \text{otherwise}.
\end{cases}
$$

Shell/rod thickness is a geometry/material attribute and is combined with the
activation distance by collision primitives. Do not use a large `d_hat` as a
substitute for setting physical cloth or rod thickness.

Friction uses the analogous effective transition velocity:

$$
\epsilon_{v,\mathrm{eff}} =
\begin{cases}
\epsilon_{v,\mathrm{relative}}L, & \epsilon_{v,\mathrm{relative}} > 0,\\
\epsilon_v, & \text{otherwise}.
\end{cases}
$$

## IPC versus AL-IPC

`contact/constitution = "ipc"` is the default, broadly exercised pipeline.
`"al-ipc"` replaces it with the augmented-Lagrangian active-set pipeline and
activates the `contact/al-ipc/*` parameters. It is a pipeline choice, not a
per-material contact model, so one scene cannot mix IPC and AL-IPC pairs.

Fused-PCG CUDA graph replay is currently disabled automatically under AL-IPC.
Start with IPC unless a method comparison or an AL-specific workflow requires
the alternative.

## Broad phase and sanity checks

The supported broad phases are `info_stackless_bvh` (default),
`stackless_bvh`, and `linear_bvh`; `info_stackless_bvh_v0` remains as a legacy
comparison path. This selector changes candidate generation, not contact
material behavior.

Leave `sanity_check/enable = 1` while authoring scenes. Before engine
initialization it detects initial intersections and dangerously close
surfaces, respects the contact table's enabled masks, and prevents an invalid
world from starting. `sanity_check/mode = "normal"` also writes diagnostic
geometry for a failed check.

## Common failure checklist

| Symptom | Check |
| --- | --- |
| Bodies pass through each other | `contact/enable`, `label_surface`, pair `enable`, initial intersection, and mesh units. |
| No friction | `contact/friction/enable`, pair `friction_rate`, and `eps_velocity`. |
| Unexpected material pair | Whether the intended `ContactElement` was applied before adding the geometry to the scene. Untagged geometry is default element 0. |
| World invalid at `init` | Keep sanity checks on and inspect the emitted diagnostic geometry/log. |
| Solver becomes very stiff | Check resistance units, `d_hat`, initial gaps, and whether an explicit resistance was clamped. |
| Scale-dependent behavior | Use the relative `d_hat`, velocity tolerance, and friction velocity controls consistently. |

The tutorial section builds on these rules with a complete rigid-soft and
material-table walkthrough.
