# Procedural white table setting

`white_table_setting.py` creates a dining still life from native Blender geometry
and shaders, then uses the existing libuipc extension worker to simulate it. It
does not download meshes or textures, use Blender/Bullet physics, sculpt the final
cloth pose, or restart a deformed cloth with an altered rest shape.

## Reproduce

Run from the repository root, substituting your executable locations:

```powershell
& 'D:\Blender\4_5\blender.exe' --background --factory-startup --python-exit-code 1 `
  --python integrations/blender/examples/white_table_setting.py -- `
  --python '<Python containing CUDA-enabled pyuipc>' `
  --output output/dining-scene-verified --mode all
```

The script imports the extension directly from this checkout. An installed
extension is not required for background generation. The external Python must
contain the project's current CUDA build; Blender's bundled Python is not used
to import pyuipc. CUDA simulation requires a supported NVIDIA GPU. Rendering
uses Cycles/OptiX when available, with a CPU fallback.

`--preview` renders half resolution with 32 samples. The full render is
2000 x 1450 with 192 samples and denoising. Other modes are `build` (initial
separated state), `bake` (simulate/validate), `finish` (validate/decorate an already
completed bake), `validate` (reopen and check), `render` (reopen/render), and
`package` (bundle the completed render, scene, reports and caches in a ZIP).
Every mode requires the same output directory. `render` does not run physics.
Use a new directory for independent trials; existing completed bake directories
are retained, but the scene and rendered image in the selected directory are
replaced by a new build/render.

Outputs include the `.blend`, PNG, `physics_validation.json`, per-object MDD
trajectories, exact exported meshes/materials/settings, runtime build information,
and the solver log under `cache/bake_.../`. Keep the scene and its cache directory
together. Frame 1 is the separated initial state; frame 961 is the final 32-second
state. Native Mesh Cache playback works without the extension or CUDA runtime.
Changing physics inputs requires a new bake, not reusing the old cache.

## Physical setup and assumptions

All dimensions and properties are explicit modeling assumptions, not measurements
recovered from the reference photo. The result is a model-based physically
simulated reconstruction, not a guarantee of identical real-world behavior.

| Item | Model / dimensions / parameters |
| --- | --- |
| Ground | Exactly z = 0 m; fixed 6 x 6 m collision plane |
| Table | Fixed table, 1.20 x 0.84 m, top at z = 0.76 m, rounded 6 mm edges; legs and aprons also participate in contact |
| Tablecloth | 1.78 x 1.48 m; 109 x 91 vertices; no pins; starts near z = 0.798 m |
| Cloth thickness/density | Radius r = 0.0004 m; material thickness h = 2r = 0.0008 m; density 250 kg/m^3; areal density 0.20 kg/m^2 |
| Cloth membrane | Baraff-Witkin, E_stretch = 2e6 Pa; independent E_shear parameter = 100; nu = 0.3; strain amplification = 100 |
| Cloth bending | Discrete Shell Bending with E_b = 3e4 Pa, nu = 0.3; D = 1.40659e-6 N m |
| Placemats | Two deformable 0.325 x 0.32 m meshes; r = 0.001 m; density 400 kg/m^3; stretch 5e6, shear 3000, bending 3e6, nu = 0.3 |
| Ceramics | Two plates, two shallow bowls and one hollow fruit bowl; closed material shells, not solid convex hulls; ABD rigidity 1e9 Pa, density 2400 kg/m^3, collision radius 0.00015 m |
| Apples | Five closed lobed meshes with radii 0.037-0.040 m; ABD rigidity 1e8 Pa, density 820 kg/m^3, collision radius 0.00015 m |
| Cutlery | Two knives and two three-tine forks; closed extruded meshes; ABD rigidity 1e9 Pa, density 7800 kg/m^3, collision radius 0.00010 m |
| Contact | Standard IPC normal contact and friction; global mu = 0.45; d_hat = 0.0008 m; input resistance 1e9 Pa, subject to the backend's scene-adaptive kappa corridor |
| Integration | One continuous World, gravity (0, 0, -9.81) m/s^2; 30 output FPS, 4 substeps, dt = 1/120 s; 961 frames = 32 s |

The actual cloth stretch coefficient is `E_stretch * (2r) / (1 - nu^2)`;
the shear coefficient is `E_shear / (2 * (1 + nu))`, without another thickness
factor. Bending is `E_b * (2r)^3 / (12 * (1 - nu^2))`. These follow
`src/constitution/strain_limiting_baraff_witkin.cpp` and
`src/constitution/discrete_shell_bending.cpp`. Triangle mass uses area times
`2r` times density, as implemented in `src/geometry/compute_vertex_volume.cpp`.
Shear and bending are independently calibrated fabric parameters, not an
assertion that woven linen is an isotropic bulk solid with a single modulus.

All tabletop bodies start separated, including collision thickness. The bowls
and plates fall onto the cloth/placemats, and the apples fall into the hollow
fruit bowl. The chair, table legs and upholstery are fixed fixtures, not
unconstrained solids being balanced by this solve. Their complete evaluated
meshes, the tabletop, and the ground are combined in **one compound fixed
collision environment**. Only its internal fixed-fixed self contact is disabled;
all moving-body contact against every furniture part remains enabled. This
permits intentional fixed assembly joints without ignoring a hanging cloth's
contact with a table leg. The visible fixtures are preserved separately for
materials; editing their geometry/pose requires rebuilding the compound collider
and re-baking, not merely changing the visible mesh. The large render-only floor continuation remains
coplanar with the physical ground without inflating the solver's scene bounds.

Cloth is rendered from the solved triangle mesh with smooth normals and a
centered `Solidify` of thickness `2r`; no post-bake subdivision changes its
mid-surface. Weave and apple skin are shader details. Apple stems are noncontact
decorations whose MDD motion is derived from the apple's solved affine motion at
every frame. They do not remain floating at the final pose during playback.

## Verification

The worker validates manifold/winding/index requirements, keeps strict solver
checks on, checks `World.is_valid()` after every substep, and rejects nonfinite
output. The scene script additionally checks:

- Initial separation including collision radii, using conservative per-triangle
  AABB bounds for the compound fixed environment. Bounding a whole chair hoop
  would incorrectly count its empty center as occupied space.
- Inter-object triangle-surface crossings at every cached frame.
- No final nonadjacent triangle self-crossings in the cloth/placemats.
- Ground height and tabletop clearance (a hem curling below the tabletop is
  not falsely classified as penetration merely from its XY projection).
- A final sampled contact path from every dynamic body to a fixed support.
- ABD principal stretches close to one, and final half-second motion residuals.
- Final rendered Solidify surfaces against other bodies and the visible furniture.
- Every simulated vertex against native Blender playback at initial, middle
  and final frames, after reopening when running `--mode validate`.

The JSON records numerical values and limitations. Contact-graph distances are
sampled vertex-to-triangle distances, not a proof of the exact global minimum.
These checks supplement IPC's continuous collision detection; discrete saved
frames alone do not certify all intermediate times. Likewise, a stable image
does not measure real fabric parameters or establish mesh/time-step convergence.
Use `--substeps 2` versus `--substeps 4` in separate output directories for a
time-step sensitivity experiment; `--cloth-x` / `--cloth-y` control resolution.
`compare_table_steps.py --coarse <dir> --fine <dir>` checks that initial meshes
and material/solver inputs agree, then compares every vertex at the same physical
time (the shorter run's final time). Cloth folds may select different local
equilibria; do not interpret a matching centroid as pointwise convergence.

The studio render uses physically traced lighting and a shadow catcher composited
over white; there is no generated or painted replacement image. The full physical
ground remains z = 0. `--restyle` explicitly reapplies the scripted camera/lighting.

Rendering explicitly revalidates the full input fingerprint and reactivates
matching native caches. This prevents an overly broad add-on property-change
notification on a decorative object from silently rendering the rest pose.
No changes to the solver or the extension's general invalidation policy are
needed by this example.

## Checked local result (2026-09-05)

Blender 4.5.3 LTS, CPython 3.14 source-built pyuipc 0.9.0 / CUDA 13.2.51, and an
RTX 5090 produced `output/dining-scene-verified/white_table_setting.png` and the
matching `.blend`/ZIP. All 961 output frames passed surface-crossing checks;
final rendered fabric thickness/furniture checks passed. The initial clearance
lower bound, after subtracting collision radii, was 5.85 mm. The final cloth
half-second RMS speed was 0.356 mm/s (maximum vertex speed 4.63 mm/s): nearly
settled, not mathematically zero motion. The model does not include air drag.

The ZIP was extracted into a different directory. Factory Blender, **without
registering the extension**, checked all vertices of 22 native caches (17 physical
bodies plus five derived stems) at frames 1, 481 and 961 with maximum error 0,
and rendered the reopened file. Reproduce with `check_table_playback.py
--directory <extracted-directory> --render` using the same Blender command prefix.

The 1/60 s vs 1/120 s experiment uses identical initial meshes/materials and
compares frame 481 (16 s) in both trajectories. Both runs passed final support
checks. Cloth vertex-centroid difference was 0.761 mm, but **pointwise RMS
difference was 27.8 mm and maximum difference was 177 mm** around different folds.
This is explicitly not a time-step convergence claim. The comparison JSON retains
every object's pointwise metrics rather than reporting the centroid alone.
