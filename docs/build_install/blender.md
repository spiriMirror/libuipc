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

2. Obtain `libuipc_blender-0.2.0.zip`, or build it from the repository root:

    ```shell
    python scripts/build_blender_addon.py
    ```

    Output: `output/blender-dist/libuipc_blender-0.2.0.zip`.

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

## Scene parameters

| Setting | Default | Valid range and meaning |
|---|---:|---|
| Frame range | Blender scene start/end | End >= start; start is the unadvanced rest frame |
| Frame rate | Blender FPS / FPS Base | Positive frames/second |
| Substeps | 2 | Integer 1–1000; `dt = FPS Base / (FPS * substeps)` seconds |
| Gravity | (0, 0, -9.81) | Finite acceleration components, m/s², Blender world axes |
| Contact Distance | 0.001 m | >= 1e-7 m; activation distance beyond thickness offsets |
| Friction | 0.5 | >= 0; global Coulomb coefficient |
| Contact Resistance | 1e9 Pa | >= 1 Pa; global contact model resistance |

Geometry is multiplied by Blender's **Unit Scale** to obtain meters. Material
parameters, thickness radius, activation distance, and gravity are already SI
values; they are not multiplied a second time. Object translation, rotation,
nonuniform scale, and negative scale are included using Blender's evaluated
world matrix. Output is transformed back to the original object-local frame.

The worker uses standard IPC and inherits libuipc's semi-implicit termination
and `K_min` defaults. Sanity checks are enabled. Strict solver mode is enabled so
nonlinear/line-search limit failures are reported instead of silently baking
unconverged frames. No new collision or buffer-allocation code is introduced.

## Object parameters

Select a mesh and use **Object Physics** in the same sidebar.

| Setting | Default | Valid range and behavior |
|---|---:|---|
| Simulation Role | Disabled | Disabled / Cloth / Rigid Body (ABD) / Volumetric FEM / Fixed Collider |
| Fixed Entire Object | False | ABD: instance flag; FEM/cloth: every surface and internal node |
| Solid Young's Modulus | 1e5 Pa | >= 1e-6; volumetric FEM material |
| Density | 200 kg/m³ | >= 1e-6; used for cloth and rigid mass |
| Thickness Radius `r` | 0.001 m | >= 1e-7; one-sided collision offset; full cloth thickness `h = 2r` |
| Stretch E | 5e4 Pa | >= 1e-6; membrane stretch parameter |
| Shear E Parameter | 10 | >= 1e-6; independent effective 2D shear parameter |
| Bending E | 3e4 Pa | >= 0; zero disables bending |
| Poisson Ratio | 0.49 | UI range 0–0.499; constitutive domain 0 <= nu < 0.5 |
| Strain Amplification | 100 | >= 1e-6; over-stretch amplification rate |
| ABD Rigidity | 1e8 Pa | >= 1; OrthoPotential ABD stiffness |
| Self Collision | Enabled | Cloth only |
| Pin Vertex Group | Empty | Optional cloth/FEM node group; fixed in world space; overridden by Fixed Entire Object |
| Pin Weight Threshold | 0.5 | 0.0001–1; vertices with weight >= threshold are fixed |

The cloth coefficients preserve the library's existing conventions:

$$
k_s = \frac{E_s(2r)}{1-\nu^2},\qquad
k_{\mathrm{shear}} = \frac{E_{\mathrm{shear}}}{2(1+\nu)},\qquad
k_b = \frac{E_b(2r)^3}{12(1-\nu^2)}.
$$

Here `E_s`, `E_shear`, and `E_b` are the three interface parameters, `nu` is
Poisson's ratio, and `r` is the one-sided offset. The shear convention is an
independently calibrated effective coefficient, not a second multiplication by
thickness. Refer to [cloth modeling](../tutorial/cloth.md) for the full model.

ABD bodies require one closed, connected surface with consistent face winding.
Inward global winding is normalized on export without changing vertex order.
Fixed colliders may be open triangle surfaces and use fixed FEM vertices.
Duplicate/degenerate triangles, non-manifold edges, loose vertices, invalid
coordinates, singular transforms, and invalid pin groups are rejected.

## Modifiers, caches, and limitations

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
- Version 0.2 rejects shape keys, animation/drivers/constraints on participants
  or their parents, and Blender Bullet rigid bodies on the same hierarchy.
  Animated colliders/pins, rods, joints, and topology-changing
  simulation are future extensions, not supported controls hidden in this version.
- One bake runs at a time. Loading another file or disabling the extension stops
  its worker. The worker notices a closed parent Blender process between substeps.
- `worker.log`, `request.json`, input NPZ files, `status.json`, and `result.json`
  stay in each bake directory for diagnosis. MDD data is written one frame at a
  time, with no frames-times-vertices allocation in RAM.
- Version 0.1 cloth/ABD caches remain valid when the new fixed/FEM features are
  unused. New bakes use protocol v2, which also fingerprints tetrahedral topology.

## Developer validation

See the [integration README](https://github.com/spiriMirror/libuipc/tree/refactor-main/integrations/blender)
for the portable tests and real Blender test commands. The latter execute the
actual operators, CUDA solve, Mesh Cache evaluation, `.blend` reopening, and EEVEE
rendering. Blender-specific behavior has been tested on Windows 4.5.3 LTS;
other advertised platform/API combinations still require equivalent runtime tests.
