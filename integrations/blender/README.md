# libuipc Physics for Blender

Blender extension source: [`libuipc_blender/`](libuipc_blender/).
This integration is independently packaged; it uses the public `pyuipc` API and
does not add Blender dependencies to libuipc's CMake or XMake targets.

Version 0.2 supports coupled cloth, ABD rigid bodies, tetrahedral FEM, fixed
triangle colliders, whole-object fixing, pinned surface/internal FEM nodes,
Coulomb friction, and native MDD bake playback. Volume generation uses libuipc's
native C++ tetrahedralizer; existing `.msh` volumes can also be imported.
The UI lives in **3D Viewport > Sidebar (N) > libuipc**. The CUDA solver runs in
a separate Python process. Blender itself never imports the native `uipc` module.

See the [installation and parameter guide](../../docs/build_install/blender.md).

## Package

From the repository root, with Python 3.11 or newer:

```shell
python scripts/build_blender_addon.py
blender --background --command extension validate output/blender-dist/libuipc_blender-0.2.0.zip
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
- Rods, animated collision targets/pins, and joints are not exposed in version 0.2.0.
- Blender-specific adapter code is GPL-3.0-or-later; the independently usable
  `worker.py` and `protocol.py` retain Apache-2.0. Copyright: spiriMirror.
  Separately installed libuipc remains Apache-2.0. See the extension's `LICENSE`
  for the per-file boundary. The ZIP contains no third-party native binaries.
