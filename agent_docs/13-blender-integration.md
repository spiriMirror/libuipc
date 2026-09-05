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
| `worker.py` | Public libuipc scene construction, standard IPC advancement, output retrieval and coordinate conversion |
| `demo.py` | Asset-free cloth/ABD/platform example scene |
| `blender_manifest.toml` | Extension identity 0.1.0; Windows/Linux; Blender >=4.2 API target |
| `scripts/build_blender_addon.py` | Deterministic ZIP without native binaries or Python wheels |

## Invariants

- Demo shader nodes are found by node type, not display name. Chinese Blender
  preferences translate default node names; an English-name lookup fails there.

- One Scene/World contains all participating objects, including fixed colliders.
  Dynamic bodies use ABD; cloth uses strain-limiting Baraff-Witkin plus optional
  discrete shell bending; static surfaces use fixed FEM vertices.
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
Version 0.1 exposes no volume-FEM/tetrahedralization, rods, joints, animated
colliders/pins, shape keys, or topology-changing simulation.
