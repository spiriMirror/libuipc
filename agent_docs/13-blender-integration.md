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
| `blender_manifest.toml` | Extension identity 0.2.0; Windows/Linux; Blender >=4.2 API target |
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
- Requests use schema v2 and include tetrahedral topology plus the fixed/material
  fields in signatures. Legacy v1 cloth/ABD caches remain valid while new features
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
Rods, joints, animated colliders/pins, shape keys, and topology-changing simulation
are still outside this plugin version.
