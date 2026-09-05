# libuipc Physics for Blender

Blender extension source: [`libuipc_blender/`](libuipc_blender/).
This integration is independently packaged; it uses the public `pyuipc` API and
does not add Blender dependencies to libuipc's CMake or XMake targets.

Version 0.3 supports coupled cloth, ABD rigid bodies, tetrahedral FEM, fixed
triangle colliders, whole-object fixing, pinned surface/internal FEM nodes,
Coulomb friction, and native MDD bake playback. Volume generation uses libuipc's
native C++ tetrahedralizer; existing `.msh` volumes can also be imported.
URDF import creates keyframeable root/joint controllers and soft-driven ABD links.
Animated targets are sampled at every solver substep; robot/environment contact
and friction stay coupled to the cloth and other rigid bodies.
The UI lives in **3D Viewport > Sidebar (N) > libuipc**. The CUDA solver runs in
a separate Python process. Blender itself never imports the native `uipc` module.

See the [installation and parameter guide](../../docs/build_install/blender.md).

## Package

From the repository root, with Python 3.11 or newer:

```shell
python scripts/build_blender_addon.py
blender --background --command extension validate output/blender-dist/libuipc_blender-0.3.0.zip
```

The generated ZIP can be installed using Blender's **Install from Disk**.
Alternatively, Blender's own `extension build --source-dir
integrations/blender/libuipc_blender --output-dir <existing-directory>` produces
an equivalent extension.

## Verification

Portable regressions (only NumPy is needed):

```shell
python -m unittest discover -s integrations/blender/tests -p test_protocol.py -v
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
  a torque-controlled articulation solver. Rods and animated cloth/FEM pins remain unsupported.
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
