# Cloth and Thin-Shell Scenes

Cloth is a triangle `SimplicialComplex` with vertex degrees of freedom. A
membrane constitution supplies in-plane stretch/shear behavior and mass;
`DiscreteShellBending` is an optional extra constitution for resistance to
folding. Applying bending alone does not create a complete cloth model.

## Constitution stack

| Layer | Example API | Purpose |
| --- | --- | --- |
| Triangle topology | `trimesh(vertices, triangles)` | Rest surface and vertex state. |
| Collision surface | `label_surface(mesh)` | Marks the explicit surface used by contact. |
| Membrane | `StrainLimitingBaraffWitkinShell.apply_to(...)` | Stretch, shear, density, thickness, and strain-rate control. |
| Bending, optional | `DiscreteShellBending.apply_to(mesh, E, nu)` | Interior-edge bending stiffness computed from membrane thickness. |
| Pins | `mesh.vertices()/is_fixed` | Per-vertex fixed state. |
| Contact material | `ContactElement.apply_to(mesh)` | Friction/resistance pair identity. |

The membrane must be applied before the formula-based bending overload. The
bending implementation reads the vertex `thickness` attribute written by the
membrane and evaluates

$$
D = \frac{E t^3}{12(1-\nu^2)}.
$$

The raw overload `apply_to(mesh, bending_stiffness)` defaults to `100 kPa`,
but then the supplied value is already the effective edge stiffness and is not
derived from thickness.

## Complete pinned-cloth scene

This minimal mesh has two triangles, one shared bending edge, two pinned top
vertices, and ground contact. It is intentionally educational rather than a
visual demo; production cloth needs enough resolution for the desired folds.

=== "C++"

    ```cpp
    --8<-- "docs/tutorial/code/cloth.cpp"
    ```

=== "Python"

    ```python
    --8<-- "docs/tutorial/code/cloth.py"
    ```

The two-modulus overload deliberately separates stretch and shear. Here the
stretch material uses `E = 50 kPa`, while the shear material uses `E = 0.5
kPa`; their Poisson ratios are both `0.40`. This is useful when a woven sheet
should resist extension much more strongly than in-plane shear.

## Parameter meanings and defaults

| Argument | Default in the convenience overload | Practical domain |
| --- | --- | --- |
| `moduli` | `E = 1 MPa`, `nu = 0.49` for both stretch and shear | Use `E > 0`; for ordinary isotropic 2D materials use `-1 < nu < 1`. |
| `mass_density` | `200 kg/m^3` | Positive; surface mass scales as density times thickness. |
| `thickness` | `0.001 m` | Positive and expressed in scene length units. Contact also consumes it. |
| `strain_rate` | `100` | Non-negative amplification coefficient; `0` removes the extra cubic extension penalty, while larger values progressively stiffen over-stretch. |

The C++ and Python bindings expose the same convenience and separated-moduli
overloads. The API excludes singular Poisson endpoints but does not centrally
validate every physical parameter, so invalid negative values can survive
scene assembly and fail later in a less obvious place.

## Pinning and animated boundaries

The example marks vertices fixed after the membrane is applied. Fixed state is
per vertex, not per triangle. Pinning both endpoints of a top edge is enough
for this four-vertex sheet; a real mesh should select a geometrically stable
set of support vertices.

For moving grippers or prescribed motion, do not overwrite positions after
`world.init()`. Add a soft position constraint and update its animation
attributes through the scene's animation/update path. The
[`91_pinned_cloth`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/91_pinned_cloth/main.py)
sample is the static-pin baseline.

## Contact and self-collision

The FEM base constitution sets `self_collision = 1` for cloth as well as
volumetric FEM. It prevents different parts of the same sheet from passing
through each other when global contact is enabled. The contact table still
controls material pairs, and the cloth still needs `label_surface`.

For cloth, inspect these values together:

- physical `thickness` on vertices;
- `contact/d_hat` or `contact/d_hat_relative`;
- triangle size and aspect ratio;
- `contact/eps_velocity` when friction is enabled; and
- contact resistance, which is independent of membrane and bending stiffness.

`d_hat` is a barrier activation distance, not a replacement for thickness.
Excessive activation distance on a finely discretized sheet can create many
more candidates and make the solve unnecessarily stiff.

## Deeper examples and implementation evidence

- cloth material and bending:
  [`11_bunny_cloth`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/11_bunny_cloth/main.py)
- multiple self-contacting sheets:
  [`34_cloth_stack`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/34_cloth_stack/main.py)
- pinned source-backed scene:
  [`91_pinned_cloth`](https://github.com/spiriMirror/libuipc-samples/blob/main/examples/91_pinned_cloth/main.py)
- C++ pin, bending, and MAS regressions:
  [`19_shell_fixed_point.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/19_shell_fixed_point.cpp),
  [`33_discrete_shell_bending.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/33_discrete_shell_bending.cpp),
  and
  [`60_fem_mas_cloth.cpp`](https://github.com/spiriMirror/libuipc/blob/main/apps/tests/sim_case/60_fem_mas_cloth.cpp)
- public membrane and bending APIs:
  [`strain_limiting_baraff_witkin.h`](https://github.com/spiriMirror/libuipc/blob/main/include/uipc/constitution/strain_limiting_baraff_witkin.h)
  and
  [`discrete_shell_bending.h`](https://github.com/spiriMirror/libuipc/blob/main/include/uipc/constitution/discrete_shell_bending.h)
