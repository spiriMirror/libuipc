# 13 — Blender Integration

`integrations/blender/libuipc_blender/` is a standalone Blender extension package.
It depends only on Blender's `bpy`/NumPy inside Blender. Its worker is launched
with an independently configured Python containing `pyuipc >= 0.0.28`; native
uipc modules never load into Blender. No native CMake/XMake changes are required.
The extension copyright and maintainer are `spiriMirror`.
The root library remains Apache-2.0. Independent `worker.py`/`protocol.py` also
use Apache-2.0; only Blender-specific adapter files use GPL-3.0-or-later.
The extension LICENSE defines the per-file boundary and ships both full texts.

## Compact affine delivery (0.8 / schema 8)

Moving/driven ABD now uses four ordinary MDD vectors per frame: translation and
the three columns of the complete object-local affine map. `affine.py` packs/
decodes this representation; `affine_playback.py` builds an owned hidden four-point
Mesh Cache helper and a native Sample Index/Vector Math/Set Position graph on the
visible source mesh. All 12 coefficients are retained, including shear and scale.
No TRS decomposition, shape-key approximation, scripted driver or Python frame
callback is used. Source meshes and native physics are unchanged.

The worker forms `local_map * current_transform * rest_map`, where
`rest_map = S * original_matrix` with the native ABD rest center subtracted, and
`local_map = inverse(original_matrix) * inverse(S)`. S converts Blender units to
meters. This preserves the original local-coordinate playback contract under
translation, rotation, nonuniform/negative scale and non-unit scene scales.
Quality metrics still sample the real native world-space vertices. Encoding checks
compare to that SAME native run, not bitwise goldens from a second simulation.

`result.objects[].vertices` is still the source vertex count. New `encoding` is
`AFFINE` or `VERTEX` (legacy default); the former requires schema >= 8 and RIGID.
`cache_vertices()` derives an MDD header count of four for AFFINE. Wholly fixed
ABD remains single-sample VERTEX. `output_options.compact_abd` is a strict boolean,
default true for new schema-8 jobs, outside the physical fingerprint. Old schemas
retain their old encoding. The UI checkbox affects future bakes, never physics or
the validity of an existing cache. Disable it for direct per-vertex MDD exchange.

Attachment stages all new modifiers/helpers before switching, preserves the old
data until evaluation succeeds, and rolls back both encoding directions on failure.
Removal only deletes owned unshared helpers/groups; fake-user/reused data survives.
Validation checks code-owned node/link/input structure, helper topology/transform,
enabled playback, source identity/fingerprint, exact cache path/header/hash and
stack position. Watch/preview understand both encodings. Native helper file paths
are automatically included in `bpy.utils.blend_paths`, so existing snapshot/receipt
render queues remain addon-free without inventing a second renderer protocol.

| Frozen workload (31 frames) | 0.6 cache bytes | 0.8 cache bytes | Reduction |
|---|---:|---:|---:|
| Four moving + two fixed ABD spheres | 1,672,824 | 33,048 | 98.0% |
| Cloth + sphere + 17 driven hand links | 4,574,760 | 434,400 | 90.5% |
| Cloth only | 405,240 | 405,240 | None; encoding unchanged |

This is a storage/output optimization, not a claimed solver speedup. Initial
candidate process medians remain effectively unchanged (ABD 1.20 s, mixed 13.1 s).
One warmup plus three trials reuse the frozen NPZ hashes/config/native runtime;
results are under `output/blender-affine-08-benchmark` and its comparison JSON.
Single-run encoding checks under `output/blender-affine-08-native-{abd,hand}`
compare every output vertex to the same native state within float32 cache precision.

Verification includes 54 portable tests on Python 3.11/3.14; native 61-frame mixed
contact, 41-frame FEM, 81-frame motion and sample-87 controller regressions; cached
EEVEE/OptiX multi-view rendering; old schema-3 500-frame playback; and installed GUI
rod/affine overlays with repeated GPU resource reuse (exit 0). The synthetic affine
contract reopens and checks ALL 500 varying-transform frames without the addon,
including shear, mirrored/nonuniform scale and cm units. Its 500-frame file is
26,008 bytes. Evidence directories use `output/blender-affine-08-*`.
The regression also exposed/fixed a rod fingerprint assumption in volume
preparation: generic preparation materials need not contain a simulation `role`.
No native C++/CUDA changes or Python runtime reinstall are part of this delivery.

## Rod integration (0.7 / schema 7)

`rod.py` validates edge-only unbranched open/closed chains and maps radius/density/
independent axial and bending E to section properties. `worker.py` constructs
`linemesh` + `HookeanSpring` + optional `KirchhoffRodBending`, with the same node
fixing and contact table as FEM/cloth. This reuses existing native constitutions;
there is no twist term or rest-curvature extension. Rod-only fields and edge arrays
are fingerprinted without changing old non-rod hashes. Schema 2-6 workers/readers
remain supported; rods require 7. Presets, diagnostics, overlays and all-fixed
compaction include rods.

`rod_ui.py` explicitly converts a private curve copy into a new centerline object,
without editing/deleting the source, and builds a native display node group after
MDD. `node_math.py` applies full affine position fields without TRS decomposition.
The rod section is constructed in world-length coordinates and mapped back to
local coordinates, preserving negative/nonuniform scale. The display uses a
12-sided circular section and round end caps, not additional simulated vertices.
The active material initializes an exposed Material input. Bake export refreshes
existing managed displays; read-only cache checks never rebuild node groups.

Validation: `test_rod.py` plus 47 existing portable tests pass (50 total);
`blender_rods.py` checks 31 frames, exact centerline cache playback, pinned roots,
whole-fixed rods, two bending moduli, floor contact, surface rendering, save/reopen
without the addon and non-destructive beveled-curve conversion. Soft/stiff tip
displacements are -0.0926753 / -0.0056086 m. Evidence:
`output/blender-followup-rods-v2`. Compact moving ABD is delivered in 0.8 above.
The native fixed-order experiment was withdrawn; retain concurrent solver paths
per rule 16. Rounding from valid atomic accumulation orders is not a defect.

## Performance baseline (2026-09-11)

Preview preparation now reuses base topology/edges, pin selections, two decoded
MDD samples and world-space arrays. Normals/guides are lazy. Indexed GPU batches
and window-local shaders are reused on camera-only redraws. CPU caching retains
at most four objects/two frames each and clears at a conservatively counted
128 MiB; GPU batches are bounded per object/window. Mesh depsgraph updates clear
topology, evaluated geometry updates clear pins, transforms/frame/file-stat and
playback changes refresh relevant data. Load, undo/redo and unregister clear caches.
Scripts editing mesh data must tag/update the datablock, as for Blender evaluation.
The cache is only a preview: full bake integrity validation is unchanged.

Blender 4.5.3 comparison on the frozen 16,641-vertex preview grid: cold preparation
61.98 -> 54.26 ms; 20 warm calls' median 59.609 -> 0.0117 ms (CPU, not solver/GPU
draw time). `blender_preview_cache.py` verifies camera reuse, lazy normals, pin and
same-count topology edits, transform, forward/reverse/subframes, same-size atomic
file replacement and cleanup. Installed GUI `blender_quality_gui.py` verifies
successful repeated GPU draws without new CPU/GPU builds and exits with code 0.

`result.json.performance` records host wall seconds/call counts for input loading,
runtime import, scene/engine construction, initialization, advance, retrieve,
coordinate conversion, diagnostics, cache I/O and status writes. It also records
output frames/vertices, native steps and actual cache/diagnostic bytes. No CUDA
synchronization is added: report advance + retrieve together; these are not kernel
timings. `total_seconds` starts at worker input loading and excludes final result
serialization/process exit. The existing `elapsed_seconds` still starts at world
initialization. Frontend export/attach/check timings live in bounded process memory,
not Scene RNA, and are cleared on load/unregister. Quality UI displays the summary.

Reproduce the baseline with background Blender running
`integrations/blender/tests/blender_performance.py` and arguments
`-- --python <native-python> --output <inputs-dir> --urdf <sample-87 robot_hand.urdf>`.
Then run the external Python on `integrations/blender/tests/benchmark_worker.py`
with `--inputs <inputs-dir> --output <fresh-results-dir>`.
The latter uses one warmup + three measured fresh-process trials per workload and
hashes frozen NPZ inputs. Pure cloth, pure ABD (four falling/two fixed spheres),
and mixed cloth/ABD/17 driven hand links run 31 frames, two substeps, CONVERGED.
The hand is spatially separated: this is a workflow baseline, not a grasp benchmark.
Keep the inputs, native build, device and solver settings identical for comparisons.
The preview-only 129 x 129 grid measures repeated CPU preview preparation, not GPU
draw duration. Reopen its saved source with `--measure <label>` for comparisons.
Local initial evidence is in `output/blender-perf-06-*`; 43 portable tests and the
real Blender/CUDA three-frame contact regression pass with instrumentation enabled.

Final 0.6 verification: 47 portable bridge tests on Python 3.11/3.14, 49 repository
tests, 61-frame mixed contact, 41-frame FEM/fixed contact, 81-frame controller
motion, constant-cache addon-free playback, cache contracts, preview invalidation,
legacy 500-frame playback and EEVEE/OptiX snapshot queues. Checked Blender playback
error against each bake's MDD is zero for the mixed/FEM suites; that is distinct
from independent native simulations being reproducible. The installed 0.6 GUI
also draws the overlays/phase panel and verifies repeated GPU resource reuse.
The installed GUI exits with code 0; the user's profile is not changed. Re-running
the frozen schema-5 ABD job with the 0.6 worker reproduces every dense MDD byte.

Local RTX 5090 / pyuipc 0.9.0 cp314 / CUDA 13.2.51 native build, 31 output frames:

| Workload | Before / after process median (s) | Before / after MDD bytes | Interpretation |
|---|---:|---:|---|
| Four dynamic + two fixed ABD spheres | 1.201 / 1.197 | 2,469,384 / 1,672,824 | 32.3% less storage; all 617,148 compared vertex samples identical |
| Pinned 33 x 33 cloth | 9.432 / 9.365 | 405,240 / 405,240 | No fixed-output saving; timing change is not evidence of a speedup |
| Cloth + sphere + 17 driven hand links | 13.046 / 13.092 | 4,574,760 / 4,574,760 | No compactable body; runtime essentially unchanged |

These are one warmup plus three fresh-process trials. Use
`benchmark_worker.py --current-schema` only to update transport metadata of frozen
schema-5 inputs: NPZ hashes, physical fingerprints/configs and native build are
unchanged. `compare_worker_benchmarks.py --before <dir> --after <dir> --output <json>`
checks every vertex of every frame across all three trials and also reports
within-revision variation. Evidence: `output/blender-perf-06-comparison.json`.

**Benchmark variability observation (not a determinism requirement):** the cloth workload differs between
baseline trials by up to 0.021640 m per local component; optimized repeated trials
differ by 0.052424 m, and cross-revision comparisons by 0.068073 m. Mixed-workload
values are 0.006711 / 0.004028 / 0.007459 m. Scale is one in these inputs. The
initial investigation identified atomic-order rounding in linear solving and
gradient accumulation. This does not establish an algorithmic or synchronization
bug. The owner explicitly rejected restructuring parallel execution for bitwise
agreement; that experiment and its compiled backend were fully restored. Keep the
original concurrent paths. No determinism fix is pending. Frozen inputs/logs remain
available as measurements, not correctness goldens. Pure ABD comparisons in this
particular baseline happened to be exact; that is not a general solver guarantee.

## Fixed cache encoding and affine boundary (0.6 / schema 6)

`result.frames` remains the authored scene range. Each output has `stored_frames`:
one for proven wholly fixed bodies, otherwise the full range. `fully_fixed()` uses
the fixed flag or complete in-range pin coverage, never observed low velocity;
driven ABD and partially pinned FEM/cloth remain dense. Bodies still participate
in the same World/contact table. Validation derives fixedness from the current
fingerprint-checked input and rejects shortened moving/legacy caches. Schema 1-5
readers stay compatible; the worker still writes dense output for schema 2-5 jobs.

MDD bytes for N vertices/F frames are `8 + 4F + 12FN`; a constant file is `12 + 12N`.
The one-sample native modifier clamps throughout the timeline, without a Python
frame handler. Stationary diagnostics keep every output-frame row/sample count
and the same zero speed/acceleration semantics, but release per-vertex history.
`test_fixed_cache.py` checks byte-identical dense/constant diagnostic series;
`blender_fixed_cache.py` checks all-node pins, zero DOFs, cm units, mirrored scale,
frame-start 7, fractional/backward frames and reopened addon-free playback to 500.

At the 0.6 stage, dynamic ABD compact encoding was evaluated but not enabled: a 0.2 m cube with 2% shear
loses 1.514 mm after Blender TRS decomposition, while native full-affine MDD is
within 3e-8 m including the midpoint (`blender_affine_contract.py`). Version 0.8 now
meets that requirement with all 12 coefficients in a four-vector MDD and validated
native reconstruction; it does not use lossy location/rotation/scale keys.

## Source map

| File | Ownership |
|---|---|
| `__init__.py` | RNA properties, sidebar panels, operators, main-thread polling, stale-cache/load/unregister hooks |
| `bridge.py` | Mesh/parameter validation, base-mesh export, input signatures, transactional result attachment |
| `protocol.py` | Schema, fingerprints, mesh validity, streaming big-endian MDD writer, atomic JSON |
| `runtime.py` | Exactly one owned subprocess; cancellation, completion, fresh-directory rebakes |
| `worker.py` | Native tetrahedralization/MSH preparation, 3D FEM/ABD/cloth construction, IPC advancement and output retrieval |
| `demo.py` | Asset-free cloth/ABD/platform example scene |
| `blender_manifest.toml` | Extension identity 0.8.0; Windows/Linux; Blender >=4.2 API target |
| `affine.py` / `affine_playback.py` | Four-vector full-affine encoding, native helpers and structural validation |
| `rod.py` / `rod_ui.py` / `node_math.py` | Rod validation/section mapping, authoring and native surface transforms |
| `performance.py` | Host-phase and non-mutating frontend timings, without GPU synchronization |
| `materials.py` / `material_ui.py` | Portable independent cloth/pair rules and Blender contact/preset controls |
| `quality.py` / `quality_ui.py` / `preview.py` | Streaming observations, verified report navigation and non-destructive selected-object GPU preview |
| `identity.py` / `watch.py` | Persistent IDs, request-order binding and relevant/raw-input validation gates |
| `render_protocol.py` / `render_queue.py` / `render_worker.py` / `render_ui.py` | Immutable PNG render snapshots, verified receipts, owned renderer and queue controls |
| `robot_control_state.py` / `robot_controls.py` | Validated robot-local poses, joint proxies, keying and angular trajectory review |
| `motion.py` | Authored controller signatures, rigid substep target sampling, robot start-pose alignment |
| `robot_model.py` / `robot_ui.py` | Native URDF export, collision assembly cleanup, Blender joint hierarchy and driven links |
| `scripts/build_blender_addon.py` | Deterministic ZIP without native binaries or Python wheels |

## Invariants

- `robot_controls.py` provides degree-valued, non-animatable UI proxies for native
  radian joint transforms, root-local validated pose copies and explicit keying.
  Fixed/unknown rows are read-only. Preview is kinematic authoring, never solved
  grasp/contact; export still aligns every driven link to frame-start targets.
  `robot_control_state.py` validates pose layout/limits and computes sampled
  angular rates. Review evaluates allowed F-curves directly without timeline
  mutation. New joint metadata includes type/velocity; stable controller hashes
  include type/position limits but diagnostic review thresholds are not physics.

- `render_queue.py` captures an immutable saved copy and hashes external inputs;
  `render_worker.py` runs background Blender, not the native solver. Render ranges
  and camera-list properties are not simulation inputs. Render output/receipts use
  numeric paths. Resume requires the same snapshot/inputs and validates PNG CRC,
  dimensions, hash and receipt identity. Never skip a file just because it exists.
  Jobs own their process/cancel sentinel and share the one-job exclusion with the
  simulation worker. File load/unregister stops either owned job. Original-directory
  resume and documented PNG/asset restrictions are intentional initial boundaries.

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
- Requests use schema v5 and include stable identities, tetrahedral topology, independent cloth
  channels, fixed/material fields and opt-in authored motion/contact pairs.
  Legacy v1/v2/v3/v4 caches remain valid while new features
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

Version 0.5 adds identity/render/robot/manifest suites (41 portable tests total). Real
tests cover renamed body/controller IDs, copied-ID rejection, failed-token
handling, multi-view subset rendering and receipt repair, EEVEE/OptiX execution,
joint pose/key persistence and non-mutating rate review. The installed GUI runs
an asynchronous queue, draws the actual robot panel and disables cleanly.
Render receipts are published only after input-file stamps remain unchanged
around that frame. Failed watch checks never become trusted unchanged-input
tokens. The ZIP builder checks Blender's 64-character permission-message limit.
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
Version 0.3 adds the URDF/controller workflow below. Rods are exposed in 0.7; torque-driven
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
