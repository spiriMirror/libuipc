# 13 — Blender Integration

`integrations/blender/libuipc_blender/` is a standalone Blender extension package.
It depends only on Blender's `bpy`/NumPy inside Blender. Its worker is launched
with an independently configured Python containing `pyuipc >= 0.0.28`; native
uipc modules never load into Blender. No native CMake/XMake changes are required.
The extension copyright and maintainer are `spiriMirror`.
The root library remains Apache-2.0. Independent `worker.py`/`protocol.py` also
use Apache-2.0; only Blender-specific adapter files use GPL-3.0-or-later.
The extension LICENSE defines the per-file boundary and ships both full texts.

## Source map

| File | Ownership |
|---|---|
| `__init__.py` | RNA properties, sidebar panels, operators, main-thread polling, stale-cache/load/unregister hooks |
| `bridge.py` | Mesh/parameter validation, base-mesh export, input signatures, transactional result attachment |
| `protocol.py` | Schema, fingerprints, mesh validity, streaming big-endian MDD writer, atomic JSON |
| `runtime.py` | Exactly one owned subprocess; cancellation, completion, fresh-directory rebakes |
| `worker.py` | Native tetrahedralization/MSH preparation, 3D FEM/ABD/cloth construction, IPC advancement and output retrieval |
| `demo.py` | Asset-free cloth/ABD/platform example scene |
| `blender_manifest.toml` | Extension identity 0.4.0; Windows/Linux; Blender >=4.2 API target |
| `materials.py` / `material_ui.py` | Portable independent cloth/pair rules and Blender contact/preset controls |
| `quality.py` / `quality_ui.py` / `preview.py` | Streaming observations, verified report navigation and non-destructive selected-object GPU preview |
| `motion.py` | Authored controller signatures, rigid substep target sampling, robot start-pose alignment |
| `robot_model.py` / `robot_ui.py` | Native URDF export, collision assembly cleanup, Blender joint hierarchy and driven links |
| `scripts/build_blender_addon.py` | Deterministic ZIP without native binaries or Python wheels |

## Invariants

- Version 0.5/schema 5 assigns persistent IDs on explicit bake export, not during
  validation. `identity.py` resolves request entries by ID and rejects copied or
  missing identities. The fingerprint sorts identified bodies and canonicalizes
  controller IDs/signatures; display names are metadata only. Legacy schemas 1-4
  remain name-based and strip new controller signature fields when hashing.
  All result/quality attachment must use request order/ID lookup, not current name
  order. `watch.py` ignores unrelated updates and gates full validation using raw
  source tokens; do not mistake an unchanged-token fast path for a deep file check.

- Quality sampling is observational and uses SI world positions before MDD's
  float32 conversion. Keep only the previous position/velocity per object, not
  all frames. Copy native views before the next advance. Acceleration needs
  three output samples; records distinguish output frame from native substep.
  Final reports/series publish only after completion and never replace a failed
  bake's previous cache. Review thresholds and display controls are not physics
  inputs. Reports do not certify convergence or detect every substep event.
- GPU preview uses explicit contiguous float32 vertex buffers. Passing a float64
  NumPy buffer into GPUVertBuf's F32 fast path draws corrupt geometry. The preview
  reads the simulation cache, not evaluated display subdivision. It draws selected
  Object Mode meshes, fixed nodes and sampled +/-r normal guides (not an exact
  offset surface). Remove draw callbacks on unregister. UI tests must close
  popup-owned RNA widgets before unregistering their property types.

- Named contact material pairs are canonicalized/validated in `materials.py`;
  UI lives in `material_ui.py`. Empty labels map to Default. An explicit unordered
  material pair wins over global/robot external friction, but same nonempty robot
  assembly exclusion wins over everything. `build_contact_plan()` assigns every
  geometry, including fixed colliders, and records the complete native pair plan.
  No new labels/rows are serialized when unused, preserving old cache signatures.
- Physical material presets are scene-local .blend data, restricted by role and
  a validated field allowlist. Batch apply prevalidates all targets and restores
  values on a failed write. Never copy pins, fixing, targets, contact labels or
  shaders through a material preset. Independent cloth E/nu values are copied.

- Cache attachment is transactional across objects: save every touched modifier
  and the scene references, restore them on failure. `validate_result()` is shared
  by attachment and later cache checks; do not trust result-provided counts.
  New MDD writers compute SHA-256 while streaming; deep explicit/render checks
  verify it. Debounced checks verify inputs, provenance, headers and all playback
  settings without hashing every frame's bytes. Hidden modifiers may be validated
  and explicitly reactivated; validated rendering rejects render-disabled caches.
- `uipc.render_validated` is the guarded render entry point. Do not pretend a
  render-handler exception cancels ordinary F12/direct Blender renders. Those
  entry points remain unguarded. Factory-Blender contract tests inject a failure
  on the second attachment and check rollback, modified playback controls,
  same-size cache corruption, invalid result lists and pre-render rejection.

- Version 0.4/schema 4 separates cloth stretch/shear/bending Poisson ratios.
  Existing E property IDs remain unchanged. Missing RNA values inherit the
  old shared `poisson` without mutating saved scenes; setters store each channel
  independently. Worker conversions and UI coefficients share `materials.py`.
  Schema 1-3 cache comparison projects the fields away only when all three
  equal the original shared value. Native 2r stretch/bending conventions stay.
  `tests/test_materials.py` checks channel isolation and compatibility;
  `tests/blender_materials.py` exercises real RNA save/reopen, and
  `tests/native_materials.py` checks native triangle lambda/mu and edge bending.

- Demo shader nodes are found by node type, not display name. Chinese Blender
  preferences translate default node names; an English-name lookup fails there.

- One Scene/World contains all participating objects, including fixed colliders.
  Dynamic bodies use ABD; cloth uses strain-limiting Baraff-Witkin plus optional
  discrete shell bending; volume FEM uses StableNeoHookean; static surfaces use
  fixed FEM vertices. Whole-object fixing keeps the role: ABD sets the instance
  flag, FEM/cloth sets every vertex, including internal nodes. The full fixed
  flag overrides pin-group validation. All-fixed scenes bypass the zero-DOF solve.
- Volume preparation is explicit, before setting added internal pins. Generate
  Tetrahedra calls the original C++ mesher under `src/geometry/tetrahedralization`
  through the external worker; Import FEM Mesh uses existing SimplicialComplexIO.
  There is no fTetWild/TetGen runtime dependency. See [ADR 0008](adr/0008-native-tetrahedralization.md).
- Strict preparation preserves original coordinate/vertex/triangle IDs and groups;
  appended vertices are internal nodes selectable in Blender Edit Mode with X-Ray.
  The Mesh ID property `uipc_tetrahedra` stores a flat, four-indices-per-cell array.
  Visible faces must equal the tetrahedral boundary. Preserve the source Mesh and
  groups in a private snapshot; linked source meshes may be edited independently.
- Native generation requires a current pyuipc source build (not PyPI 0.0.28).
  Blender float32 conversion is validated before installing the volume. Original
  local vertices are copied verbatim so transform round trips cannot move them.
- Requests use schema v4 and include tetrahedral topology, independent cloth
  channels, fixed/material fields and opt-in authored motion/contact pairs.
  Legacy v1/v2/v3 caches remain valid while new features
  are unused. Never ignore a newly enabled Fixed flag in legacy validation.
- Role enum IDs are explicit and persistent: NONE=0, CLOTH=1, RIGID=2,
  STATIC=3, FEM=4. Inserting FEM into the displayed list must not reinterpret
  an old .blend's numeric Fixed Collider value as a volume.
- Preserve Blender base-mesh vertex indexing. Only triangle winding can change.
  Closed ABD input must be connected and consistently oriented; inward global
  winding is flipped. Apply unsupported modifiers/shape keys before exporting.
- Obtain transforms from `object.evaluated_get(depsgraph).matrix_world`.
  **Original `matrix_world` can remain identity for a freshly scripted object even
  after `view_layer.update()`**; the full integration test reproduced false
  initial intersections until evaluated transforms were used.
- Convert Blender world positions to meters using Unit Scale. Keep acceleration,
  material values and thickness radius in SI. Center ABD rest positions before
  mass integration; set its instance translation once. FEM uses world positions.
- Read geometry from the **current slot** returned by `geometries().create`.
  After retrieve, ABD output requires applying its instance transform to rest
  positions. Transform every result back into the original Blender local frame.
- Thickness is the one-sided radius `r`; membrane/bending retain full thickness
  `2r`. Shear keeps the library's independently calibrated effective coefficient.
- First output frame is rest state. Subsequent frames each advance `substeps`,
  with `dt = fps_base / (fps * substeps)`. Sanity and strict solver checks remain on.
- `solver_accuracy` is opt-in: omit `DEFAULT` from serialized settings to retain
  legacy fingerprints. `CONVERGED` disables semi-implicit early exit, uses
  0.001 m/s absolute Newton tolerance, zero relative Newton tolerance, linear
  `tol_rate=1e-6` and up to 32 line-search trials. It changes no material/contact
  parameters or native defaults. `result.json` records effective settings;
  `solver_steps.jsonl` records per-substep statistics where the runtime supports
  `Engine.frame_stats()` (older supported runtimes may not expose it).
- Version 0.3.2 exposes `CUSTOM` solver accuracy in the sidebar. `SOLVER_FIELDS`
  and `validate_solver_settings()` in the portable protocol normalize scientific-
  notation text, validate finite values/ranges and min/max iteration ordering,
  and feed the same normalized dictionary to fingerprinting and the worker.
  All custom tolerances, the semi-implicit enable/K_min/beta values, Newton
  min/max and line-search limits reach native config explicitly. Custom fields
  are omitted from requests unless selected; preset modes cannot silently consume
  a custom override dictionary. The enum's persisted IDs stay DEFAULT=0,
  CONVERGED=1, CUSTOM=2. See the user guide for initial values and UI limits.
- `check_cache()` is read-only. The explicit Validate Cache operator calls
  `activate_cache()` after verification to restore only the validated objects'
  viewport/render flags. Reverting an edited tolerance must not leave a valid
  cache silently hidden. Failed validation still disables stale playback.
- MDD files stream one frame at a time, use big-endian float32, and become final
  only after all frames are written. Finite checks happen before conversion.
  Full input fingerprints and MDD header/size checks precede any attachment.
- Windows progress readers can temporarily deny atomic rename; JSON replacement
  retries sharing violations instead of truncating the live file.
- MDD modifiers are first in the stack and consume object-local coordinates.
  Display Subdivision/Solidify/Bevel/Weighted Normal may follow for dynamic bodies;
  they do not change the simulated surface. Other active modifiers are rejected.
- Fingerprints cover names, vertices/order, triangles, pins, world matrices,
  material parameters, frames/FPS/units, gravity and contacts. Physics property
  edits disable cache immediately; dependency-graph mesh/transform checks are
  debounced. Explicit Validate Cache is required after arbitrary scripted edits
  before rendering. Do not equate equal vertex counts with cache compatibility.
- File exchange uses JSON and `np.load(..., allow_pickle=False)`. Derived numeric
  filenames avoid putting object names into filesystem paths. No shell commands
  or native solver calls execute merely from loading a .blend.
- Cancellation/new file/add-on disable stops only the owned process. Parent PID
  monitoring also ends a worker between substeps after Blender exits. Incomplete
  output never replaces a previous completed bake; cache files remain recoverable.
- Native cache playback works after saving/reopening and while the extension is
  disabled. It does not use Python frame-change handlers or call the CUDA solver.

## Validation and limits

Run `python -m unittest discover -s integrations/blender/tests -p 'test_*.py'`.
Version 0.4 has 32 portable regressions and additional actual-Blender tests for
material save/reopen, transactional cache faults, named contact response, quality
reports, GPU previews and legacy caches. The 61-frame cloth/ABD and 41-frame FEM
suites retain zero checked playback error; the existing 500-frame dining cache
passes at 1/250/500. GUI validation installs the ZIP into isolated configuration,
extension and temporary directories and checks clean disable/exit. Linux/other
Blender versions still need equivalent runtime/UI validation.
`tests/blender_integration.py` runs the actual Blender operators, CUDA worker,
MDD modifier evaluation, backward/fractional frames, all-vertex comparison,
fixed pins, contact, save/reopen, EEVEE rendering, unit/negative-scale round trips,
friction contrast, cancellation, failed launch, rebake and detach checks.
`tests/install_extension.py` validates ZIP installation with the actual Blender
extension operator. `tests/blender_gui.py` exercises the interactive timer loop,
asynchronous bake/cancel, sidebar screenshot, and active-worker unregister cleanup.

Validated baseline: Windows Blender 4.5.3 LTS (bundled Python 3.11), external
CPython 3.14 with official pyuipc 0.0.28, RTX 5090. The initial 61-frame mixed
scene matched MDD playback at every tested vertex with maximum absolute error 0;
the same output interpolated correctly at a half-frame. In the friction contrast,
mu=0 slid approximately 1.296 m, while mu=1 slid approximately 0.00279 m.
These are correctness checks on a small scene, not a throughput benchmark.

Local test artifacts live under ignored `output/blender-validation*` directories;
the scripts reproduce them. See `docs/build_install/blender.md` for user-facing
installation, defaults/ranges, geometry semantics and limits. The Linux platform
and other Blender API versions still require equivalent GPU/runtime validation.
Version 0.2 adds native volume generation/import and whole-object fixing. The
`blender_fem.py` suite covers strict and refined surfaces, original groups,
surface/internal pins, fixed ABD/FEM/cloth, contact, all-node MDD checks and source
restoration. It passed in Blender 4.5.3 with a Python 3.14 source-built CUDA 13.2
runtime; cached-node playback error was zero in the checked frames.
Version 0.3 adds the URDF/controller workflow below. Rods, torque-driven
articulations, animated cloth/FEM pins, shape keys and topology-changing
simulation remain outside this plugin version.

## Robot targets (0.3)

Import URDF Robot runs native `UrdfIO` in the worker, then builds an object-parent
controller hierarchy in Blender. Link vertices already include each collision
origin. Undo the URDF-to-UIPC basis for the exported controller poses so Blender's
Z-up targets use native URDF frames; explicitly set the native root pose to zero
before export. Validate a nonzero joint pose against the native controller (the
sample 87 import measured maximum matrix error 1.64e-7).

Separate controllers can animate ABD bodies through `SoftTransformConstraint`.
For ordinary driven bodies, the first target pose defines the offset:
`T_aim(t) = T_target(t) * inverse(T_target(t0)) * T_body(t0)`, after translating
positions to meters. Imported robots automatically align their base meshes to
their first-frame target poses before export. Sample at every solver substep,
including frame zero; keep first-frame output unadvanced. Insert an animator
callback per driven object so the backend consumes the current aim transforms.
Use `is_dynamic=0` for the sample's quasi-static servos, with `is_fixed=0`.

Drive metadata is opt-in; do not add default drive fields to legacy materials.
Protocol v3 hashes controller hierarchy transforms, authored F-curves/handles,
interpolation and strength/group/friction parameters. Ignore the current value
of animated transform channels in that signature: playback must not invalidate
it. Sampled target matrices have a separate SHA256 in the request and are checked
for shape, finite values and proper rigid motion in the worker. Pose sampling
restores the caller's frame. Old v1/v2 caches remain readable when their new
features are unused; never silently ignore a newly enabled drive on a v1 cache.

Support only keyframed object transforms and object-parent hierarchies here.
Reject drivers, NLA, constraints, bone parenting, F-curve modifiers, variable
scale/shear and URDF joint-limit violations. A nonempty robot contact group
disables internal assembly contact only; contacts against the environment remain
enabled. Ordinary rigid inputs stay connected; driven assemblies may consist of
several closed components. The importer removes isolated zero-volume components
and fills open boundaries before final rigid validation. Roll back newly created
Blender data and restore existing cache visibility if attachment fails.

The native ABD vertex reporter inherits zero thickness and dimension 3 from the
global vertex manager; cloth/FEM thickness offsets do not add a separate skin to
ABD surfaces. The robot addition preserves that backend behavior.

The initial three-finger dining delivery used an unconstrained apple and 17 driven links.
All 500 cached frames passed enabled inter-object crossing checks; the hand
remained still through frame 50, lifted the apple about 22 cm, moved it about
48 cm, and released it onto the cloth. Final apple RMS speed was 3.2e-5 m/s,
hand separation 0.260 m, and native all-vertex playback error 0 in checked frames.
See `integrations/blender/examples/ROBOT_PICK_PLACE.md` and
`tests/blender_motion.py` for reproduction and validation scope.

The installed 0.3 extension also passed the motion regression, basic coupled
bake and FEM suite. A real UI window reopened the 500-frame hand file, validated
seven action frames and exposed the target controls. The portable bundle was
extracted elsewhere and factory Blender (without the addon) replayed all 39
physical/decorative caches at frames 1/250/500 with zero vertex error.

Accuracy-profile regression (0.3.1): 14 portable tests and actual Blender/CUDA
motion bakes check native-default preservation, the precise profile's effective
settings, cache invalidation/restoration on profile changes, and preservation of
the previous bake. The first 50 hold frames remain within 2.8e-7 m and the driven
body moves the requested 0.20 m.

Custom controls (0.3.2): 18 portable tests plus three real Blender/CUDA bakes
exercise Default, Converged and Custom profiles. A custom `1e-8` PCG tolerance,
0.002 m/s Newton tolerance, 64/1 Newton limits and 16 line-search trials match
the effective worker settings. Invalid input is rejected, numeric edits stale
the previous cache, and save/reopen preserves the selected profile and text.
The installed extension's real UI also validated the saved custom file at frames
1/50/81, displayed `1e-8` without numeric rounding, and replayed the correct 0.20 m
translation after explicit cache reactivation. Local GUI evidence is under
`output/blender-custom-gui-verified/`.

The original three-finger dining delivery passed crossings/playback tests but
had cloth kicks during carrying. Its third non-thumb chain was intentionally
parked at zero. A temporal audit found a remote hem vertex moving about 8 cm in
one output frame (frame 293), reaching 2.45 m/s. Crossing tests do not establish
temporal stability.

The corrected 500-frame four-finger bake selects Converged accuracy and smooth
stage-boundary target velocities, without changing cloth materials, rest shape,
pins or damping. Every fingertip moves 16-24 mm while closing and remains within
0.85-6.43 micrometers of the apple at the checked carrying frames. All 500
enabled inter-object crossing checks, additional distinct-fingertip checks,
temporal cloth bounds and native vertex playback pass. Carrying-stage cloth
maximum speed/acceleration are 0.244 m/s and 2.48 m/s²; supported tabletop maximum
speed is 0.706 mm/s. Final apple speed is 4.52e-5 m/s with about 0.242 m hand
clearance. The installed UI validated seven stages, and the relocated 39-cache
bundle replayed every vertex at frames 1/250/500 without an addon (error 0).

Controlled old-motion replays isolate a convergence-setting problem in the
strong loaded-drive/light-cloth system: tightening only PCG reduces, but does
not remove, sharp acceleration peaks; the Converged profile reduces the old
carrying peak from 2.451/70.38 to 0.224/2.44 (m/s and m/s²). Only disabling
semi-implicit mode aborts at output frame 205 on the original eight-trial line
search limit. Held/far-away robot controls are calm; the far-away test also
changes the scene-diagonal-dependent kappa clamp and is only qualitative.
See `integrations/blender/examples/ROBOT_PICK_PLACE.md` for exact controls and
`integrations/blender/examples/replay_robot_bake.py` for a fresh-directory
diagnostic runner. It records native child-process failures, not just a
potentially stale last-progress status. Native defaults remain unchanged;
these are accuracy comparisons, not isolated throughput benchmarks.

The verified joint plan was also extracted into standalone sample
`libuipc-samples/examples/94_robot_hand_grasp_apple/`. Unlike the Blender scene,
the focused Y-up sample uses only the hand, a procedural free ABD apple and a
fixed table plane. It retains all four contact checks and a 500-output-frame
headless validation without depending on Blender or MDD playback.

## Procedural dining-scene workflow

`integrations/blender/examples/white_table_setting.py` builds an asset-free white
table/chair/cloth/ceramics/apple scene, runs the existing external worker, checks
native playback/contact, and renders in Cycles. Its README records dimensions,
material formulas, limitations, and reproduction/packaging commands. This is an
example integration workflow, not a solver or extension-default change.

The cloth/placemats, hollow ceramic material shells, apples and cutlery all fall
from separated initial states in one continuous World. There are no cloth pins
and no rest-shape reset. Ground is z=0. The table, chair and upholstered cushion
are fixed fixtures. Combine their actual evaluated meshes with the ground as one
STATIC environment: internal fixed-fixed joints have no response, but every
part remains collidable by dynamic objects. A tabletop-only proxy was rejected
after the rendered-surface check caught a cloth hem crossing a table leg.
Avoid bevel widths that consume the entire thin seat thickness: Blender can
emit collapsed zero-area side triangles, rejected by the bridge validator.

Validation examines all cached frames for inter-object surface crossings,
nonadjacent cloth self-intersections at the final frame, initial
clearance including thickness (per-triangle bounds for the compound environment),
the final sampled support graph, ABD stretches,
motion residuals, and every vertex in initial/middle/final Blender playback.
The final check also includes visible Solidify thickness and fixed furniture.
`compare_table_steps.py` validates identical initial meshes/parameters and reports
time-step sensitivity at a common physical time; differing frictional folds must
not be described as pointwise convergence merely because centroids agree.

Keep caches next to the .blend or distribute the generated ZIP. Decorative apple
stems receive MDD motion derived from the same solved affine transforms at every
frame. Revalidate the full input fingerprint and explicitly reactivate matching
caches before rendering: assigning even NONE to a new decorative object's physics
role invokes the extension's broad invalidation callback. Do not silently render
the initial rest pose. Changing fixed visual fixture geometry requires rebuilding
its combined collider as well as a new bake.

Checked delivery: 961 frames / 32 s, 9,919 cloth nodes, dt=1/120 s, all cached
inter-object surface checks and final visible-furniture/thickness checks passed.
Final cloth RMS speed was 0.356 mm/s, maximum 4.63 mm/s. Relocated ZIP playback
without registering the addon checked 22 caches at frames 1/481/961 with zero
vertex error, then rendered in factory Blender. The 1/60 vs 1/120 s comparison
at t=16 s found cloth pointwise RMS/max differences 27.8/177 mm despite a 0.761 mm
centroid difference: record this sensitivity, not a false convergence claim.
