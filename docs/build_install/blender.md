# Blender Extension

The **libuipc Physics** extension connects Blender to the public `pyuipc` API.
Blender exports meshes and physics settings, an external Python process runs
the CUDA solver, and native Mesh Cache modifiers play the resulting MDD files.
All enabled objects share one libuipc World for coupled contact and friction.

[Project repository](https://github.com/spiriMirror/libuipc) ·
[Extension source](https://github.com/spiriMirror/libuipc/tree/refactor-main/integrations/blender)

## Prerequisites

| Component | Requirement |
|---|---|
| Blender | 4.2+ API target; tested in 4.5.3 LTS on Windows |
| Solver platform | Windows x64 or Linux x64 with a compatible NVIDIA GPU/driver |
| External Python | CPython 3.10–3.14; independent of Blender's bundled Python |
| pyuipc | Current source build for native tetrahedralization; 0.0.28+ for baking/import APIs |
| Build helper | Python 3.11+ to build the extension ZIP from source |

Blender 4.5's Python 3.11 can use a Python 3.14 solver because the bridge exchanges
files, not Python objects or native modules. The extension does not run pip or
modify Blender's Python installation. A compatible driver is sufficient for the
official 0.0.28 wheel; a system CUDA Toolkit is not required.

## Install

Licensing is scoped per component: libuipc/pyuipc and the independent worker and
cache protocol remain Apache-2.0. Only Blender-specific adapter files use
GPL-3.0-or-later, following Blender's
[published add-on guidance](https://www.blender.org/about/license/).
Copyright and maintainer: **spiriMirror**. The ZIP includes full license texts
and a per-file `LICENSE` explanation; the libuipc root license is unchanged.

1. Create a separate solver environment. On Windows, for example:

    ```powershell
    py -3.14 -m venv .venv-uipc-blender
    .\.venv-uipc-blender\Scripts\python.exe -m pip install pyuipc==0.0.28
    .\.venv-uipc-blender\Scripts\python.exe -m uipc doctor --probe-cuda
    ```

    On Linux:

    ```bash
    python3 -m venv .venv-uipc-blender
    .venv-uipc-blender/bin/python -m pip install pyuipc==0.0.28
    .venv-uipc-blender/bin/python -m uipc doctor --probe-cuda
    ```

2. Obtain `libuipc_blender-0.8.0.zip`, or build it from the repository root:

    ```shell
    python scripts/build_blender_addon.py
    ```

    Output: `output/blender-dist/libuipc_blender-0.8.0.zip`.

3. In Blender, open **Edit > Preferences > Add-ons**, open the menu, and choose
   **Install from Disk**. Select the ZIP and enable **libuipc Physics**.
4. In the add-on preferences, set **Default External Python** to the executable
   created in step 1. In the 3D Viewport, press **N**, select **libuipc**, and
   click **Check Python / CUDA**. This actually initializes the CUDA backend.
5. **External Python** in the scene panel optionally overrides the preference.
   If both fields are blank the extension searches `PATH`.

## First bake

1. Click **Create Example Scene**. This creates a new scene containing a pinned
   cloth, a falling ABD cube, a fixed platform, a floor, camera, and lights.
2. Save the `.blend`, or set an absolute **Cache Directory**. By default, caches
   live in `<blend name>_uipc_cache` beside the saved file.
3. Set the scene's start/end frames and frame rate, then click **Bake Simulation**.
   The interactive UI polls a separate worker; **Cancel Bake** stops it.
4. On success, drag the time slider or render. **Validate Cache** checks that the
   current scene still matches the bake. **Detach Cache** restores the source
   meshes by removing only libuipc cache modifiers; it retains cache files.

Each bake has a new directory. Results are attached only when every file has
completed, its header/length is valid, and the scene still matches the export.
Cancelling or failing a bake never replaces the previous completed bake.

## Rod centerlines (0.7)

The extension now exposes libuipc's existing `HookeanSpring` plus
`KirchhoffRodBending` on 1D edge meshes. This is a stretch-and-bend rod model,
without a material-frame twisting degree of freedom or torsional energy.

1. Use an edge-only mesh (no faces), or select a 3D Blender Curve and click
   **Create Rod from Curve** in the scene panel. Curve conversion creates a new
   sampled centerline and keeps the source unchanged; hide the original display
   if it would overlap the rod. Poly, Bezier and NURBS centerlines use Blender's
   sampling/resolution on a private copy with bevel/extrusion and object modifiers
   removed. **Object modifiers that deform the centerline are currently ignored**;
   apply desired deformations before conversion. Hair Curves are not this legacy
   Curve type. A base/evaluated-source choice is a future improvement, not an
   implemented behavior.
2. Set **Simulation Role > Rod (Stretch + Bending)**. Open/closed chains and
   disconnected chains are supported; branching, isolated vertices, duplicate/
   zero-length edges and exact backtracking are rejected before native calls.
3. Set the cross-section radius, density and independent stretch/bending moduli.
   Use the ordinary **Pin Vertex Group** for selected nodes, or **Fixed Entire
   Object** for the whole rod. Pinning two adjacent nodes also fixes the root
   tangent; pinning one node alone leaves a pivot. No animated rod pins are exposed.
4. **Create / Refresh Rod Surface** adds native Geometry Nodes after the simulation
   cache: circular tube display with round end caps, including nonuniform/negative
   object scale. The surface is only rendering geometry, never the FEM collision
   input. It uses the active material initially; its Material modifier input can
   be changed independently. Rebaking refreshes its radius and scale mapping.
5. Bake normally. Rods share the World, contact-pair table and friction with cloth,
   ABD and FEM. MDD centerline playback and generated surface rendering work after
   the extension is disabled. All-pinned rods use the constant-cache optimization.

| Parameter | Default | Valid domain / interpretation |
|---|---:|---|
| `rod_stretch` | 40,000 Pa | Finite, >= 1e-6 in UI; Hookean axial modulus |
| `rod_bending` | 100,000 Pa | Finite, >= 0; zero omits the bending constitution |
| `thickness` | 0.001 m | Radius r, not diameter; finite, >= 1e-7 in UI |
| `density` | 200 kg/m^3 | Finite, >= 1e-6 in UI |
| `self_collision` | True | Uses the native rod self-contact path |
| `fixed` | False | Fix every centerline node |
| `pin_group` / `pin_threshold` | Empty / 0.5 | Weights >= threshold are fixed; threshold in [0.0001, 1] |

For a circular section, $A=\pi r^2$, $I=\pi r^4/4$, axial rigidity is $E_s A$
(N), bending rigidity is $E_b I$ (N m$^2$), and linear density is $\rho A$
(kg/m). The axial edge energy is
$E_s A (L-L_0)^2/(2L_0)$. The existing bending energy is
$E_b I\|\boldsymbol{\kappa}\|^2/(L_{0a}+L_{0b})$, with curvature defined in
[Kirchhoff Rod Bending](../specification/constitutions/kirchhoff_rod_bending.md).
Here $L$ is current edge length and $L_0$ is its rest length. Bending's stress-free
state is straight: an initially curved centerline is **not** assigned rest curvature.
There is no rod Poisson-ratio or twisting parameter in this implementation.

Physical presets, quality reports and physics previews support rods. The preview
shows centerline edges, fixed nodes and diameter guides; it does not invent a
material-frame orientation. Schema 7 fingerprints include edge connectivity;
older non-rod cache fingerprints remain unchanged.

## Volumetric FEM and fixed objects

For an existing closed Blender surface:

1. Set **Simulation Role > Volumetric FEM**.
2. Enable **Preserve Original Surface** when vertex positions, vertex indices,
   and triangle connectivity must stay unchanged. This is the default.
3. Optionally set **Target Tet Edge (m)** and **Quality Passes**, then click
   **Generate Tetrahedra**. Generation runs outside Blender using libuipc's own
   C++ implementation in `src/geometry/tetrahedralization/`.
4. The resulting Blender object contains all FEM nodes, including internal nodes.
   Only the true tetrahedral boundary is represented by visible faces. In strict
   mode the original surface polygons, UVs, coordinates and vertex groups are
   copied unchanged; additional internal nodes follow the original vertex IDs.
5. Set **Solid Young's Modulus (Pa)**, Poisson ratio, density and collision radius,
   then bake. The material is libuipc's 3D **Stable Neo-Hookean** model.

The 0.0.28 wheel predates the native mesher API. **Generate Tetrahedra** requires a
current source build installed into the external Python selected by Blender.
After building this checkout with Python bindings, install its generated package
into that interpreter, for example on Windows:

```powershell
.\.venv-uipc-blender\Scripts\python.exe -m pip install --no-deps --force-reinstall .\build\python
.\.venv-uipc-blender\Scripts\python.exe -c "from uipc.geometry import tetrahedralize; print('Native mesher ready')"
```

No fTetWild, TetGen, PyVista or other tetrahedralizer is called or required.
Alternatively use **Import FEM Mesh (.msh)** to read an existing tetrahedral mesh
through `SimplicialComplexIO`; this path also retains every internal node.

When surface preservation is disabled, conforming surface refinement is allowed
before volume construction. The updated boundary is both rendered and used for
contact. **Restore Original Surface** recovers the saved mesh and vertex groups;
preparation keeps a private backup so linked source objects cannot alter it.

**Fixed Entire Object** applies to ABD, cloth and volumetric FEM. For ABD it sets
the instance `is_fixed` flag; for FEM/cloth it sets every vertex flag, including
internal nodes. The object's material/role remains intact. It overrides the pin
group, so a whole object can be fixed without creating any vertex group.

To fix only selected FEM nodes, generate/import the volume first, enter **Edit
Mode**, enable vertex selection and **X-Ray** (Alt-Z), select the surface and/or
internal nodes, and assign them to a vertex group. Select that group in **Pin
Vertex Group** after leaving Edit Mode. Weights >= **Pin Weight Threshold** mean
fixed; weights are not soft-constraint strengths. Original vertex selections
survive preparation, while newly added nodes are initially unpinned.

| Meshing control | Default | Meaning |
|---|---:|---|
| Preserve Original Surface | True | Lock vertices, coordinates and all surface triangles; add only interior nodes |
| Target Tet Edge | 0 m | Zero uses median input edge length; positive values request a mesh resolution |
| Quality Passes | 4 | 0–100; zero still builds and returns a conservative valid volume |
| Solid Young's Modulus | 1e5 Pa | >= 1e-6 Pa; 3D elastic stiffness |
| Fixed Entire Object | False | Fix the whole instance or all nodes, including internal ones |

Strict mode constructs a conservative boundary-conforming mesh before optimizing.
Quality limits and rejected quality steps retain that valid mesh, not an altered
surface. Difficult non-star-shaped inputs can require a larger interior-point
search; **Cancel Operation** remains available. The mesher requires a valid
closed, embedded surface. See [native tetrahedralization](../specification/tetrahedralization.md)
for its construction, quality metrics, and C++/Python APIs.

## URDF robots and animated rigid targets

Version 0.3 adds **Import URDF Robot**. Choose a mesh-based URDF with fixed or
revolute joints. The native libuipc `UrdfIO` loads the collision meshes; Blender
creates `Robot root - ...`, `Joint angle ...` controllers and driven link meshes.
The importer checks Blender's forward kinematics against a native controller pose.

1. Select the robot root to position the hand and keyframe its Location/Rotation.
2. Select a `Joint angle ...` object and keyframe the angle component of its
   Axis-Angle rotation. The axis and origin come from the URDF; angles are radians.
   Authored trajectories must stay inside the imported joint limits.
3. Use **Preview Robot Initial Pose** to align the visible links to the first-frame
   controls. Baking also performs this alignment automatically.
4. Click **Bake Simulation**. Controllers are sampled at every solver substep.
   The resulting link motion and coupled object motion play through native MDD.

For any other ABD body, enable **Drive from Target** and choose a separate object
as its **Motion Target**. Its motion relative to the target's first-frame pose
drives the body's initial pose. The target may have a keyframed object-parent
hierarchy. The physical mesh itself remains unparented and unanimated; the cache
contains its solved motion. Shape-changing target motion, constraints, NLA,
drivers, F-curve modifiers and bone parenting are currently rejected.

| Drive setting | Default | Meaning |
|---|---:|---|
| Drive from Target | False | Requires an unfixed ABD body |
| Motion Target | None | Separate transform controller |
| Translation Strength Ratio | 10000 | Nonnegative, mass-weighted center-of-mass servo strength |
| Rotation Strength Ratio | 10000 | Nonnegative, mass-weighted rotation/deformation servo strength |
| Robot Collision Group | Empty | A nonempty shared group disables only contact internal to that assembly |
| Robot Contact Friction | 0.8 | Nonnegative Coulomb coefficient against ordinary scene objects |

Drives use `SoftTransformConstraint` and quasi-static links (`is_dynamic=0`),
matching sample 87. Collision response may produce a small difference between
commanded and solved poses. The URDF joint tree supplies target kinematics;
joint torques and actuator dynamics are not modeled by this interface.
Internal robot collision can be disabled for assembled links, while collisions
with the table, cloth, bowl and grasped object remain enabled.

For the same four-finger grasp as a focused standalone pyuipc program without
Blender or baked caches, see
[`94_robot_hand_grasp_apple`](https://github.com/spiriMirror/libuipc-samples/tree/main/examples/94_robot_hand_grasp_apple).
It directly constructs the World, drives the sample-87 URDF links at every
solver substep, and numerically verifies the free apple's carry and release.

The imported collision meshes retain their closed solid components. Isolated
zero-volume export fragments are removed, and open boundary loops are capped
with Blender's mesh operations before rigid validation. Repair counts are stored
on imported link objects and in the import report. The native 3D ABD reporter
uses zero contact radius on its boundary surface; the cloth/FEM thickness setting
does not add a separate collision skin to ABD links. Contact activation distance
still applies.

## Scene parameters

| Setting | Default | Valid range and meaning |
|---|---:|---|
| Frame range | Blender scene start/end | End >= start; start is the unadvanced rest frame |
| Frame rate | Blender FPS / FPS Base | Positive frames/second |
| Substeps | 2 | Integer 1–1000; `dt = FPS Base / (FPS * substeps)` seconds |
| Solver Accuracy | Library Default | `DEFAULT`: inherit native settings; `CONVERGED`: tighter linear/Newton solves; `CUSTOM`: editable numerical settings |
| Gravity | (0, 0, -9.81) | Finite acceleration components, m/s², Blender world axes |
| Contact Distance | 0.001 m | >= 1e-7 m; activation distance beyond thickness offsets |
| Friction | 0.5 | >= 0; global Coulomb coefficient |
| Contact Resistance | 1e9 Pa | >= 1 Pa; global contact model resistance |
| Compact ABD Cache | True | Boolean; affects the next bake's encoding only, not physics or existing cache validity |

Geometry is multiplied by Blender's **Unit Scale** to obtain meters. Material
parameters, thickness radius, activation distance, and gravity are already SI
values; they are not multiplied a second time. Object translation, rotation,
nonuniform scale, and negative scale are included using Blender's evaluated
world matrix. Output is transformed back to the original object-local frame.

The worker uses standard IPC. **Library Default** inherits libuipc's
semi-implicit termination and `K_min` defaults. **Converged** is an explicit
per-scene accuracy option for difficult mixed systems such as strong robot
position drives coupled to light cloth. It sets `newton/semi_implicit/enable=0`,
`newton/velocity_tol=0.001` m/s, `newton/velocity_tol_relative=0`,
`linear_system/tol_rate=1e-6`, and `line_search/max_iter=32`. It does not alter
time steps, materials, contact, pins or damping, and can be substantially slower.
Changing the profile invalidates the bake; old default-profile caches remain valid.

Sanity and strict solver checks are enabled. Strict mode reports iteration-limit
failures, but is not itself a tighter convergence tolerance and does not disable
semi-implicit early termination. The bake result records effective solver
settings. With runtimes exposing `Engine.frame_stats()`, `solver_steps.jsonl`
also records each substep's iteration counts, convergence and line-search limits.
No new collision or buffer-allocation code is introduced.

### Custom solver accuracy

In the 3D Viewport sidebar, open **libuipc → libuipc Physics → Solver Accuracy →
Custom**. Numerical tolerances are text fields accepting scientific notation
(for example, `1e-8`); they are parsed as finite numbers before export. This avoids
rounding tiny displayed tolerances to zero. They are not Python expressions.
The initial custom values below provide a high-accuracy starting point; they
are independent of **Library Default** and remain saved in the `.blend`.

| Custom control | Initial value | Range / native setting |
|---|---:|---|
| PCG Relative Tolerance | `1e-6` | `(0, 1)`; `linear_system/tol_rate`, global `abs(rᵀz) / abs(r₀ᵀz₀)` threshold, not per-object position error |
| Newton Velocity Tol | `1e-3` m/s | Positive; `newton/velocity_tol`; maximum per-axis Newton displacement threshold is this value times `dt` |
| Relative Velocity Tol | `0` 1/s | Nonnegative; positive overrides absolute tolerance with this value times the rest-scene diagonal in meters |
| ABD Transform-Rate Tol | `0.1` | Positive; `newton/transrate_tol` |
| Semi-Implicit Early Exit | Off | `newton/semi_implicit/enable`; additional approximate termination criterion |
| K_min | `6` | Integer 0–100000; beta accumulation starting iteration, **not** a hard Newton iteration floor |
| Semi-Implicit Beta Tol | `1e-3` | `[0, 1]`; `newton/semi_implicit/beta_tol`; `1` can terminate immediately when semi-implicit is enabled |
| Maximum Newton Iterations | `1024` | Integer 1–100000; `newton/max_iter` |
| Minimum Newton Iterations | `0` | Integer 0–maximum; `newton/min_iter`, zero disables the hard floor |
| Maximum Line Search Trials | `32` | Integer 1–128; `line_search/max_iter` |

K_min and beta controls are grayed out when semi-implicit is off. Inconsistent
iteration limits, NaN/Infinity and invalid text are rejected before baking.
Normalized numerical values participate in the cache fingerprint: changing a
tolerance requires a new bake, while equivalent forms such as `1e-6` and
`0.000001` describe the same physics input. Preset modes ignore custom values.
After reverting edited settings to the baked values, **Validate Cache** checks
the fingerprint/files and restores the verified cache's viewport/render visibility.
Smaller tolerances and larger iteration limits can substantially increase bake
time; check the effective settings and substep statistics in the bake directory.

## Object parameters

Select a mesh and use **Object Physics** in the same sidebar.

| Setting | Default | Valid range and behavior |
|---|---:|---|
| Simulation Role | Disabled | Disabled / Cloth / Rigid Body (ABD) / Volumetric FEM / Rod / Fixed Collider |
| Fixed Entire Object | False | ABD: instance flag; FEM/cloth/rod: every node, including internal FEM nodes |
| Solid Young's Modulus | 1e5 Pa | >= 1e-6; volumetric FEM material |
| Density | 200 kg/m³ | >= 1e-6; used for cloth, rod, solid FEM and rigid mass |
| Thickness Radius `r` | 0.001 m | >= 1e-7; one-sided collision offset; full cloth thickness `h = 2r` |
| Stretch E | 5e4 Pa | >= 1e-6; membrane stretch parameter |
| Shear E Parameter | 10 | >= 1e-6; independent effective 2D shear parameter |
| Bending E | 3e4 Pa | >= 0; zero disables bending |
| Stretch / Shear / Bending Poisson Ratio | 0.49 each | Independent cloth values; UI range 0–0.499, domain 0 <= nu < 0.5 |
| Solid Poisson Ratio | 0.49 | Volume FEM only; UI range 0–0.499 |
| Strain Amplification | 100 | >= 1e-6; over-stretch amplification rate |
| ABD Rigidity | 1e8 Pa | >= 1; OrthoPotential ABD stiffness |
| Self Collision | Enabled | Cloth, FEM and rods |
| Pin Vertex Group | Empty | Optional cloth/FEM/rod node group; fixed in world space; overridden by Fixed Entire Object |
| Pin Weight Threshold | 0.5 | 0.0001–1; vertices with weight >= threshold are fixed |

The cloth coefficients preserve the library's existing conventions:

$$
k_s = \frac{E_s(2r)}{1-\nu_s^2},\qquad
k_{\mathrm{shear}} = \frac{E_{\mathrm{shear}}}{2(1+\nu_{\mathrm{shear}})},\qquad
k_b = \frac{E_b(2r)^3}{12(1-\nu_b^2)}.
$$

Each channel has its own Young's modulus and Poisson ratio; `r` is the one-sided
offset. Version 0.4 inherits the saved shared Poisson ratio from older scenes
until a channel is explicitly edited. Existing equal-ratio caches remain valid;
an independent change requires a new bake. The UI and bake result report these
coefficients before native triangle/edge geometric weights. The shear convention is an
independently calibrated effective coefficient, not a second multiplication by
thickness. Refer to [cloth modeling](../tutorial/cloth.md) for the full model.

Ordinary ABD bodies require one closed, connected surface with consistent face winding.
Driven assemblies may contain several closed components in one rigid link.
Inward global winding is normalized on export without changing vertex order.
Fixed colliders may be open triangle surfaces and use fixed FEM vertices.
Duplicate/degenerate triangles, non-manifold edges, loose vertices, invalid
coordinates, singular transforms, and invalid pin groups are rejected.

## Contact material pairs and physical presets

1. Assign **Contact Material** names to participating objects, including fixed
   colliders. Blank and `Default` use the default material. Names are case-sensitive
   and trimmed; this label is independent of Blender's visual shader material.
2. Add rows under **Contact Material Pairs**. A/B order is interchangeable.
   Friction is finite and >= 0 (new rows copy global friction, initially 0.5),
   resistance is finite and >= 1 Pa (initially 1e9 Pa), and Contact Enabled
   defaults to true. Unknown/unassigned names and duplicate unordered pairs fail
   validation instead of being silently ignored. Disable Contact to deliberately
   permit passage between that material pair. Global activation distance remains
   shared; this UI does not implement per-pair `d_hat`.
3. Named pair overrides take precedence over global/robot external-contact
   friction. A nonempty robot assembly group still disables internal assembly
   contact, even if a material pair requests contact. With no overrides, ordinary
   and robot/environment behavior remains unchanged. `result.json` records the
   complete effective contact plan for inspection.
4. Enter a **Physical Preset** name and choose **Save / Replace**. Presets are
   stored in the .blend. **Apply to Selected** requires all selected objects to
   share the preset's Cloth/ABD/FEM role and copies only constitutive parameters.
   It never changes pins, whole-object fixing, contact labels or robot targets.
   Cloth presets include all three independent E/Poisson pairs. Presets are
   explicit value copies, not live links; modifying a preset does not alter objects
   until Apply is used. No shader settings are modified.

The default empty contact table and unused presets do not invalidate legacy
caches. Editing effective contact/physical parameters requires a new bake.

## Quality report and viewport diagnostics

### Performance and storage (0.6)

Physics previews reuse unchanged topology, decoded frames and GPU batches when
orbiting the viewport. Thickness normals are computed only when guides are shown.
Edits, changed cache files and file/history changes invalidate the relevant cache.
This improves viewport overhead without changing the solver or physical materials.

New bakes include `result.json.performance`. **Load Quality Report** shows solve
plus retrieval time, diagnostic time, cache I/O time, bytes/storage saved, and the
last frontend export/attach/validation timings. Values are host wall seconds with
no added GPU synchronization, not isolated CUDA kernel times. Historical caches
without timings still load. Reproducible benchmark instructions and measured
limits are in the repository's `agent_docs/13-blender-integration.md`.

Schema-6 bakes store one MDD sample for wholly fixed ABD/cloth/FEM, including
cloth/FEM whose every vertex is pinned. They remain in coupled collision/solving;
only repeated output conversion, diagnostics and storage are removed. The native
modifier holds the sample throughout the timeline, including fractional frames,
saved files and background renders without the extension. Partially pinned,
moving and driven bodies still store every output frame. Starting with 0.8,
moving/driven ABD can use the compact full-affine encoding below; cloth/FEM/rod
outputs remain per-vertex MDD.

No physical parameter, default or solver setting changes. Updating the extension
does not require rebuilding/reinstalling pyuipc. Old schema-1 through schema-7
caches remain readable; rebake to benefit from fixed-output storage reduction.

### Compact moving ABD playback (0.8)

**Compact ABD Cache** is on by default for new bakes. Each moving/driven ABD writes
four vectors to its MDD file per frame: translation $\mathbf{t}$ and the three
columns $\mathbf{a}_0,\mathbf{a}_1,\mathbf{a}_2$ of its complete local affine map.
For a source vertex $\mathbf{x}=(x,y,z)$, native Geometry Nodes evaluates
$\mathbf{x}'=\mathbf{t}+x\mathbf{a}_0+y\mathbf{a}_1+z\mathbf{a}_2$.
This retains all 12 coefficients, including shear and nonuniform scale; no
translation/rotation/scale decomposition or Python playback callback is involved.

A generated hidden four-point helper owns the ordinary MDD modifier. The visible
body's first **libuipc Cache** modifier is now Geometry Nodes; normal display
modifiers can follow it. Keep the generated helpers and cache files with the
`.blend`. They remain functional with the addon disabled and in independent render
workers. Do not edit/delete their node graphs, transforms or MDD settings; explicit
validation and guarded rendering detect those changes. Detach/rebake removes owned
unshared helpers, while explicitly reused/fake-user node groups are preserved.

An F-frame compact ABD file uses `8 + 52*F` bytes, regardless of source vertex
count, instead of `8 + 4*F + 12*F*N` for N source vertices. A 500-frame file is
26,008 bytes. Wholly fixed ABD continues to use one per-vertex MDD sample.
Interpolation remains linear in the output samples, as with the old MDD path.
MDD and Blender geometry use float32, so reconstruction is checked at that
precision, not for bitwise equality with double-precision solver state.

For a conventional per-vertex MDD file suitable for direct import into another
tool, disable **Compact ABD Cache** before baking. Do not attach a compact
four-vector file directly to the full source mesh. Switching the checkbox does
not invalidate existing physical results; it only changes subsequent output.
Attachment is transactional across both encodings, and the render queue captures
the native helper MDD dependencies in its existing snapshot manifest.

### Robot joint controls (0.5)

Use **Robot Joint Controls** to choose an imported root. Newly imported robots
populate the joint list automatically; use **Refresh Joint List** for older
scenes or renamed controls. Pointers remain valid across object renames. Fixed
or unknown legacy joint types are read-only. Missing legacy type information is
read from the original URDF when available; otherwise re-import to recover it.

Joint sliders display **degrees**, while native controller values and saved pose
payloads use **radians**. Sliders clamp to the URDF position limits. **Preview
While Editing** defaults on and **Preview Current Pose** updates only link
transforms, not mesh vertices or solved contact. This is a kinematic authoring
preview that invalidates the old bake. Bake realigns links at the first simulated
frame and runs the existing coupled soft-transform/IPC solver; there is no new
torque controller or automatic grasp attachment.

**Save / Replace Pose** stores all revolute angles in the robot root's .blend
data. **Apply Pose** validates joint names/types/axes/limits before writing; it
does not change the root transform. Create named Open/Grasp poses for your robot.
Use **Key** or **Key All Joints** to commit edits to actual controller
`rotation_axis_angle[0]` curves; proxy UI properties themselves are not animated.
New/touched keys use Bezier Auto Clamped handles. Auto Key Joint Edits follows
Blender's auto-key toggle. Without auto-key, edits to already animated controls
must be keyed before changing frames/baking. **Key Root + Joints** additionally
keys root translation/rotation, never scale.

**Review Joint Trajectory** samples authored joint curves at the solver's substep
times without moving the timeline. It reports position-limit violations and peak
angular speed/acceleration. URDF velocity limits (rad/s) take precedence over the
fallback review threshold (default 180 deg/s; zero disables the fallback). The
acceleration review threshold defaults to 720 deg/s²; zero disables it. Both are
diagnostic only. Reviews are snapshots: rerun after graph-editor edits or timing
changes. They do not bound root/end-effector motion or all between-sample extrema.
The review accepts at most 200000 sampled times. Current supported imported joint
types remain fixed/revolute; prismatic, mimic and torque actuation are future work.

Expand **Bake Quality and Physics Preview**. New bakes produce
`quality_report.json` plus streaming `quality_frames.jsonl`, with SI world-space
speed/acceleration observations and native iteration/line-search statistics.
The result manifest authenticates the summary with SHA-256. Reports load after
completion; **Load Quality Report** reloads one after reopening a .blend. Older
bakes remain playable but need rebaking to produce a report.

| Setting/action | Default | Behavior |
|---|---:|---|
| Speed Review Threshold | 1 m/s | Finite, >= 0; zero disables this warning |
| Acceleration Review Threshold | 10 m/s² | Finite, >= 0; zero disables this warning |
| Inspect Peak Frame | — | Jump to the reported output frame and select its object; show the peak vertex in magenta when Simulation Mesh is enabled |
| Mark Review Frames | Explicit action | Add warning/solver-limit timeline markers; replace only markers prefixed `libuipc quality: ` |
| Simulation Mesh | Off | Cyan edges of the selected object's base simulation topology and current cache positions, before display subdivision/solidify |
| Fixed Nodes | Off | Red fixed nodes, including internal FEM nodes, visible through the surface |
| Thickness Guides | Off | Up to 512 sampled vertex-normal segments from -r to +r, converted through Unit Scale; no ABD radius overlay |

These controls are diagnostic/display-only: they do not change materials, solver
settings or cache fingerprints. Speed is a finite difference between successive
**output** frames; acceleration is the difference between those velocities, so
it is unavailable until three position samples exist. Fast substep events may
be missed. A threshold crossing is a request for review, not proof of a physical
error; reported native convergence is not an independent accuracy certificate.
Thickness lines illustrate material radius, not the exact rounded contact-offset
surface. The preview never applies modifiers or edits vertices and is Object
Mode/selected-object only. Native frame statistics may be unavailable on older
supported runtimes; position diagnostics remain available.

## Cache validation and guarded rendering

### Independent PNG render queue (0.5)

The timeline **Bake Start/End** still defines the simulated range. **PNG Render
Queue > Render Start/End** is a separate inclusive range, initially following the
timeline until explicitly edited. Rendering frames 200-300 of a 1-500 bake does
not modify its fingerprint, timeline or MDD files. An empty camera list uses the
active camera; otherwise every enabled, distinct camera renders the selected
range. Explicit shots override timeline camera-switch markers.

**Validate & Queue Snapshot** verifies the cache and saves a copy of the current
(including unsaved) scene into a fresh `render_<uuid>` directory. The source
.blend, camera, frame and render filepath are not changed. Background Blender
renders the snapshot without importing the native solver. The global Cancel
Operation button cancels rendering as well as simulation; only one owned job
runs at a time. **Resume Saved Snapshot** uses the previous snapshot, not current
scene edits. Choose a job directory in **Resume Job** to resume an older job.

Each PNG has a receipt binding it to the manifest, camera, frame and checksum.
Resume validates snapshot/dependency hashes and each PNG's dimensions, CRCs and
checksum; missing, incomplete or changed images are rendered again. Complete
images remain available after cancellation. Settings/camera changes require a
new queue. The default output root is `<blend name>_renders` beside the project.
Input file stamps are checked around every newly rendered frame before its
receipt is published, so mid-render dependency edits cannot bless mixed-input
frames as reusable. Receipts predating this guard are rendered again once.

This first queue supports single-view full-frame PNGs. Render borders, external
image sequences/movies, dirty unpacked images and compositor File Output nodes
are rejected rather than incompletely tracked. Secondary live Blender physics
and stateful Geometry Nodes must first become file-backed mesh caches. Assets are referenced and hashed,
not copied/packed; keep them unchanged and available. Jobs currently resume from
their original directory with the same Blender version and configured Cycles
GPU devices. Python auto-execution is disabled in the background renderer.
Use normal Blender rendering for workflows requiring unsupported scripted drivers
or additional output files. Scripts may use
`bpy.ops.uipc.render_queue(blocking=True, resume=False)`.

New 0.5/schema-5 bakes use persistent object/controller IDs rather than display
names. Renaming an object or reordering names therefore does not invalidate a
new cache or change which mesh receives it. Older caches keep their original
name-based contract until rebaked. IDs are assigned only on explicit export;
validation never modifies them. Blender duplicates copy custom properties, so
duplicate IDs are rejected: select the duplicate and use **Assign New Simulation
ID**, then rebake. This prevents a duplicate from stealing the original cache.

Automatic checks ignore unrelated camera/light updates and compare cheap raw
input tokens before repeating topology validation. Cached playback with unchanged
base geometry/controller curves avoids that repeated work. Explicit Validate and
guarded renders still perform full validation; changed cache-file timestamps
trigger byte-level checks when automatic validation runs.

Version 0.4 checks the complete result object list/provenance and every managed
Mesh Cache playback setting (time mapping, factor, axes, vertex group and stack
position). Explicit **Validate Cache** also verifies SHA-256 checksums recorded
by new bakes. Legacy caches without checksums retain structural/input validation;
rebake to add byte-level integrity. Automatic/debounced checks avoid rereading
all cache bytes. A failed attachment restores all previous modifier settings,
ordering and scene cache references, including failures on later objects.

Use **Validate & Render** or **Render Animation** in the libuipc panel to run
full validation before dispatching Blender's renderer. Invalid/stale/disabled
caches or out-of-range still frames block that operator before any rendering.
Scripts can call `bpy.ops.uipc.render_validated(animation=True)`.
Ordinary F12 and direct `bpy.ops.render.render` remain Blender's unguarded entry
points: Python handler exceptions do not reliably cancel rendering, so the
extension does not claim to intercept them. Playback without the addon is unchanged.

## Modifiers and remaining limitations

- Simulation uses the **base mesh**. The generated MDD modifier is first in the
  stack. For cloth/rigid bodies, Subdivision, Solidify, Bevel, and Weighted Normal
  modifiers may follow as visual effects; their additional geometry does not
  participate in contact. Apply other modifiers before baking. Fixed collider
  modifiers must be applied or disabled to keep collision and visible shape aligned.
- Mesh data and object transforms remain unchanged. Rigid motion is stored as
  vertex cache motion, including the full ABD affine transform; object transform
  channels do not receive keyframes. The bake has fixed topology.
- Physics edits disable stale caches immediately. Mesh/transform edits are
  checked by a debounced dependency-graph callback. **Validate Cache** also checks
  frame range/FPS/units, topology, pins, geometry, transforms, material settings,
  and cache files. Validate explicitly before final rendering, especially after
  script-driven scene edits. A mismatch during a bake prevents attachment.
- Keep cache files with the `.blend`. Saved paths are relative when possible.
  Blender's native MDD modifier supports fractional-frame interpolation, seeking,
  saving/reopening, and rendering without extension callbacks or a running solver.
  Do not change vertex count/order after baking.
- Version 0.3 rejects shape keys, animation/drivers/constraints on physical participants
  or their parents, and Blender Bullet rigid bodies on the same hierarchy.
  It supports separate animated ABD targets as described above. Animated cloth/FEM
  pins, rod twisting, torque-driven articulations and topology-changing simulation remain unsupported.
- One bake runs at a time. Loading another file or disabling the extension stops
  its worker. The worker notices a closed parent Blender process between substeps.
- `worker.log`, `request.json`, input NPZ files, `status.json`, and `result.json`
  stay in each bake directory for diagnosis. MDD data is written one frame at a
  time, with no frames-times-vertices allocation in RAM.
- Version 0.1 cloth/ABD caches remain valid when the new fixed/FEM features are
  unused. Version 0.2 caches also remain readable. New bakes use protocol v3,
  fingerprinting authored control curves and parents, with separately hashed
  per-substep target matrices in the exported NPZ files.

## Developer validation

See the [integration README](https://github.com/spiriMirror/libuipc/tree/refactor-main/integrations/blender)
for the portable tests and real Blender test commands. The latter execute the
actual operators, CUDA solve, Mesh Cache evaluation, `.blend` reopening, and EEVEE
rendering. Blender-specific behavior has been tested on Windows 4.5.3 LTS;
other advertised platform/API combinations still require equivalent runtime tests.
