# libuipc Physics for Blender

Blender extension source: [`libuipc_blender/`](libuipc_blender/).
This integration is independently packaged; it uses the public `pyuipc` API and
does not add Blender dependencies to libuipc's CMake or XMake targets.

Version 0.8 supports coupled cloth, ABD rigid bodies, tetrahedral FEM, rods, fixed
triangle colliders, whole-object fixing, pinned surface/internal FEM nodes,
Coulomb friction, and native MDD bake playback. Volume generation uses libuipc's
native C++ tetrahedralizer; existing `.msh` volumes can also be imported.
URDF import creates keyframeable root/joint controllers and soft-driven ABD links.
Animated targets are sampled at every solver substep; robot/environment contact
and friction stay coupled to the cloth and other rigid bodies.
The UI lives in **3D Viewport > Sidebar (N) > libuipc**. The CUDA solver runs in
a separate Python process. Blender itself never imports the native `uipc` module.

Cloth stretch, shear and bending each expose independent E/Poisson values, with
legacy shared-ratio inheritance. Version 0.4 also adds named contact-material
pairs, scene-local batch physical presets, complete cache/playback validation,
transactional attachment, guarded render operators, streamed quality reports
and selected-object simulation-mesh/pin/thickness previews.

Version 0.5 adds rename-safe IDs for new bakes, relevant-input cache validation,
an independent multi-camera PNG render queue with verified resume, and robot
joint/pose/keying/trajectory-review controls. Render queues never resume physics;
they use an immutable saved scene and existing MDD trajectories.

Version 0.6 adds host-phase performance reports, bounded topology/frame/GPU
preview reuse, lazy thickness normals and single-sample MDD for proven fixed
bodies. Fixed bodies still participate in the same coupled simulation. Moving
ABD retains full affine MDD; a TRS-only shortcut would lose shear. No native solver
or physical parameter changes, and no pyuipc reinstall is needed. Rebuild/install
the extension and rebake only if you want the smaller fixed-object files.

Version 0.7 connects the existing Hookean/Kirchhoff rod model (stretch and bending,
no twist), edge-only meshes, curve-to-centerline conversion, pins/contact/presets,
and addon-free circular-section display with round end caps. Rods use a straight
stress-free bending state, not the initial curve's curvature. See the parameter
guide for ranges, section formulas and supported centerline topology.

Version 0.8 stores moving ABD as four MDD vectors/frame and reconstructs the full
affine map through native Geometry Nodes. Shear/scale, fractional frames and
addon-free rendering are preserved. **Compact ABD Cache** defaults on; disable it
before baking if another tool requires per-vertex MDD. Generated helpers are
validated and transactionally replaced. The native solver is unchanged.

See the [installation and parameter guide](../../docs/build_install/blender.md).

## Package

From the repository root, with Python 3.11 or newer:

```shell
python scripts/build_blender_addon.py
blender --background --command extension validate output/blender-dist/libuipc_blender-0.8.0.zip
```

The generated ZIP can be installed using Blender's **Install from Disk**.
Alternatively, Blender's own `extension build --source-dir
integrations/blender/libuipc_blender --output-dir <existing-directory>` produces
an equivalent extension.

## Verification

Portable regressions (only NumPy is needed):

```shell
python -m unittest discover -s integrations/blender/tests -p 'test_*.py' -v
```

Real Blender + NVIDIA GPU integration, including rendering and .blend reopening:

```shell
blender --background --factory-startup --python-exit-code 1 \
  --python integrations/blender/tests/blender_integration.py -- \
  --addon-parent integrations/blender \
  --python /absolute/path/to/external/python \
  --output output/blender-validation
```

The test requires `pyuipc>=0.0.28` in that external interpreter
and emits a JSON validation report, two Blender renders, and a playable `.blend`.
It checks every output vertex against native Mesh Cache evaluation, fixed pins,
contact, friction response, fractional/backward frames, transforms/units, stale
caches, cancellation, failed launches, rebakes, and restoration of source meshes.

`tests/install_extension.py` additionally installs the ZIP through Blender's
extension operator and can invoke the same suite with `--test`. Use isolated
`BLENDER_USER_CONFIG` and `BLENDER_USER_EXTENSIONS` directories for automated runs.
Its `--persist` option intentionally installs into the chosen user profile.

Additional 0.4 regressions: `blender_materials.py` (RNA/presets/save-reopen),
`blender_cache_contract.py` (playback parameters, file corruption, rollback and
blocked rendering), `blender_contacts.py` (real pair friction/exclusion),
`blender_quality.py` (native diagnostics/navigation/preview) and
`blender_quality_gui.py` (installed panels, asynchronous bake and actual GPU
overlay drawing). `native_materials.py` checks the real triangle/edge stiffness
attributes using the external Python. `blender_legacy_cache.py -- --blend <file>`
validates existing completed caches without modifying the source .blend.

The 0.5 suites add `test_identity.py`, `test_render_protocol.py` and
`test_robot_controls.py`, plus real Blender tests `blender_identity.py`,
`blender_render_queue.py` (optional `--test-gpu-render`),
`blender_robot_controls.py` (requires a URDF) and `blender_workflow_gui.py`.

The 0.6 suites add `test_performance.py`, `test_fixed_cache.py`,
`blender_preview_cache.py`, `blender_fixed_cache.py` and `blender_affine_contract.py`.
`blender_performance.py` freezes benchmark scenes/inputs; `benchmark_worker.py`
runs warmup/measured trials, and `compare_worker_benchmarks.py` checks every output
vertex and exposes within-revision drift. See `agent_docs/13-blender-integration.md`
for reproduction and measured limitations. Independent native trajectories are not
bitwise correctness goldens; small timing changes are not claimed as solver speedups.

The 0.8 suites add `test_affine.py`, `blender_affine_cache.py` (including all 500
frames after addon-free reopening) and `native_affine_cache.py` (every output
vertex compared to the same native run). The installed GUI test covers both rod
and affine overlays. Native parallel reduction/solver code is unchanged.

`tests/blender_fem.py` additionally verifies native strict/relaxed meshing,
original vertex IDs and groups, interior and surface pins, wholly fixed
ABD/FEM/cloth, rigid-soft contact, every cached node, and source restoration.
Use the same command as above with that test script and a current source build
installed in the external Python; PyPI 0.0.28 does not contain the new mesher.

## Boundaries

- Blender 4.2+ API target; actual local validation uses Blender 4.5.3 LTS on
  Windows with a Python 3.14 worker and RTX 5090. Linux packaging is supported
  but needs equivalent Blender/GPU runtime validation.
- Native Mesh Cache playback continues after the extension is disabled and
  requires neither Python nor a GPU solver once the bake is complete.
- This is an offline bake integration, not a real-time solver guarantee.
- URDF mesh links with fixed/revolute joints and animated ABD targets are exposed
  in 0.3. The joint hierarchy specifies target poses, as in sample 87; it is not
  a torque-controlled articulation solver. Rod twisting and animated cloth/FEM/rod pins remain unsupported.
- Blender-specific adapter code is GPL-3.0-or-later; the independently usable
  `worker.py`, `protocol.py` and `robot_model.py` retain Apache-2.0. Copyright: spiriMirror.
  Separately installed libuipc remains Apache-2.0. See the extension's `LICENSE`
  for the per-file boundary. The ZIP contains no third-party native binaries.

`tests/blender_motion.py` checks a 50-frame hold, substep target motion and
keyframe-driven cache invalidation using the actual CUDA worker.
`examples/robot_pick_place.py` adds sample 87's hand to an existing dining scene,
bakes a three-finger grasp/transfer/release, checks every frame for enabled
inter-object crossings, and verifies grasp contact, final support and playback.
See [the robot example](examples/ROBOT_PICK_PLACE.md) for its controls and results.
