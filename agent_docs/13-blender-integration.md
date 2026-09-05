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
| `blender_manifest.toml` | Extension identity 0.3.0; Windows/Linux; Blender >=4.2 API target |
| `motion.py` | Authored controller signatures, rigid substep target sampling, robot start-pose alignment |
| `robot_model.py` / `robot_ui.py` | Native URDF export, collision assembly cleanup, Blender joint hierarchy and driven links |
| `scripts/build_blender_addon.py` | Deterministic ZIP without native binaries or Python wheels |

## Invariants

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
- Requests use schema v3 and include tetrahedral topology, fixed/material fields
  and opt-in authored motion signatures. Legacy v1/v2 caches remain valid while new features
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

Run `python -m unittest discover -s integrations/blender/tests -p test_protocol.py`.
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

The dining pick/place example uses an unconstrained apple and 17 driven links.
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
