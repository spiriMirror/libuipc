# 12 — Secondary Modules, Samples, and Documentation

This guide covers public surfaces that sit outside the main
Engine/Scene/constitution/CUDA path, plus the repository's user-documentation
pipeline and the sample corpus used as executable API evidence.

## Backend Feature Collection

`Engine::features()` is a runtime registry, not a compile-time promise. The common
interfaces live under `include/uipc/core/` (with focused DiffSim interfaces under
`include/uipc/diff_sim/`); CUDA systems publish concrete
features and may override a base feature implementation. Important families include:

- contact-system inspection/control;
- ABD and FEM state accessors;
- distance diagnosis;
- differentiable-simulation features.

Query with `find<T>()` and handle absence unless the feature is a hard requirement.
Availability depends on the loaded backend and on which SimSystems survived
dependency building. Samples:

- `libuipc-samples/examples/20_contact_system_feature/`
- `libuipc-samples/examples/31_state_accessor_feature/`
- `libuipc-samples/examples/40_distance_diagnoser/`

## Sanity Checker Module

Sanity checking is a module boundary of its own. Core dynamically loads
`uipc_sanity_check`, constructs the CPU checker set, and lets the backend insert or
replace checkers by ID. The current families cover context consistency, half-plane
distance, mesh partition validity, surface distance/intersection, and volume; CUDA
overrides exist for some expensive checks.

Checks run before backend Scene initialization when `sanity_check/enable` is true.
A failure permanently invalidates the World. Use the config level to control how
much checking is requested, and treat it as input validation rather than an
in-frame collision response. Sources/tests:

- `src/sanity_check/`
- `src/backends/cuda/sanity_check/`
- `apps/tests/sanity_check/`
- sample `35_mesh_check`

## Differentiable Simulation

The public scaffolding is real but the feature is still experimental:

- `include/uipc/diff_sim/` defines parameters and feature interfaces;
- `Scene::diff_sim()` (non-const) turns `diff_sim/enable` on;
- parameter collections support scalar/vector/matrix connections, build, and
  broadcast;
- successful `World::recover` broadcasts registered parameters;
- CUDA-side management lives in `src/backends/cuda/diff_sim/`.

The user docs still label DiffSim “Coming Soon”. Do not promise a stable end-to-end
workflow or compatibility until a working public example and release test exist.

## Optional USD and VDB Modules

USD and VDB are source-present, build-optional modules under `src/usd` and
`src/vdb`; their public headers are under `include/uipc/usd` and
`include/uipc/vdb`. Both CMake options default off:

- `UIPC_WITH_USD_SUPPORT`
- `UIPC_WITH_VDB_SUPPORT`

The native Python module always creates a `usd` submodule object, but USD classes
are populated only in a USD-enabled build. Therefore “`uipc.usd` imports” is not
evidence that USD functionality was compiled.

XMake does not currently expose equivalent USD/VDB targets. Treat edits to these
modules as build-parity work, not as an isolated source change.

## Python Enhancement Layer

The native extension is only part of `python/src/uipc/`. The pure-Python layer adds:

| Area | Location | Notes |
|---|---|---|
| visualization | `gui.py` | polyscope-based; this is the supported GUI after removal of the C++ GUI |
| profiling | `profile/`, including `nsight.py` | benchmark helpers and Nsight integration |
| CLI | `cli/` | benchmark, mesh doctor, and UID info commands |
| adapters | `adapter/torch`, `adapter/warp` | optional external frameworks, not base wheel dependencies |
| assets | `assets/` | downloads/builds scenes from Hugging Face |
| statistics/development | `stats.py`, `dev/` | some reporting paths use matplotlib |

Packaging/helper invariants verified in the current tree:

- root and development metadata include `matplotlib`, pin the dev extra to
  `pytest>=9.0.3`, and use the same prebuilt-wheel CUDA 12.8 statement;
- the Warp empty-strides fallback uses the dtype element size;
- `Scene.Objects.created_count()` is bound to Python and
  `assets.strip_constitutions` uses that ID upper bound, so sparse object IDs are
  handled correctly;
- `assets.show(..., gui=False)` remains a continuous runner with no built-in
  frame cap.

Base functionality is covered by 19 `test_*.py` files containing 75 top-level
test functions.

## Sample Repository as Executable API Evidence

`libuipc-samples/` is a tracked git submodule, not an untracked sibling. At the
audited revision it contains 52 directories under `examples/`. Numbering is
historical: it is non-contiguous and `40_abd_four_box_revolute_chain` and
`40_distance_diagnoser` intentionally share the numeric prefix. Use names and
paths, not numeric uniqueness, in automation or documentation.

Representative map:

| Need to understand | Start with |
|---|---|
| installation/backend smoke | `0_check_libuipc`, `1_hello_libuipc` |
| animated FEM constraint | `3_periodically_pressed_tetrahedron`, `92_twisting_bar` |
| affine rigid bodies and contact | `6_wrecking_balls`, `10_ramp_sliding` |
| pure cloth | `11_bunny_cloth`, `34_cloth_stack`, `91_pinned_cloth` |
| volumetric FEM | `3_periodically_pressed_tetrahedron`, `89_mas_bunny` |
| rods/codimension-one bending | `23_kirchoff_rod_bending` |
| mixed ABD + FEM/cloth | `90_abd_fem_cube_stack`, `93_cube_wall_cloth` |
| scene save/load | `14_load_scene` |
| incremental scene commits | `15_scene_commit` (read doc 11 limitations first) |
| geometry operations | `16_geometry_operation`, `27_compute_mesh_d_hat` |
| joints/articulation | `17_affine_body_revolute_joint`, `18_pendulum_joint`, `33_external_articulation_constraint`, `38_*`, `39_*` |
| contact pair materials | `14_load_scene` |
| subscene collision filtering | `29_subscene` |
| backend features/interoperability | `20_contact_system_feature`, `21_interop`, `31_state_accessor_feature`, `40_distance_diagnoser` |
| URDF robot | `87_robot_hand` |
| advanced solver config/performance alignment | `28_advanced_scene_config`, `88_stiff_gipc_benchmark`, `89`–`93` |

Samples often include polyscope GUI loops. For CI or agent experiments, prefer a
headless branch with a finite frame count, `advance/retrieve` pairing, and one
small numerical assertion. Examples `88`–`93` already demonstrate this pattern.

## Documentation Source and Deployment

There are two documentation audiences:

- `docs/`: user-facing MkDocs site, tutorials, configuration/reference pages,
  media, and generated C++ API navigation;
- `agent_docs/`: implementation map, lifecycle contracts, debugging knowledge,
  and open limitations for contributors/agents.

The project repository is linked through MkDocs `repo_url`. Tutorial videos and
screenshots are normal assets under `docs/media/`; keep them when they clarify
motion or contact behavior. Demos themselves belong in the project README/sample
repository, while docs should use focused examples to teach the API.

Current user-guide coverage map:

| User question | Canonical page |
|---|---|
| Scene composition and lifecycle | `docs/tutorial/scene_setup.md`, `docs/tutorial/concepts.md` |
| Pure rigid/ABD setup | `docs/tutorial/rigid_bodies.md` |
| Volumetric FEM | `docs/tutorial/fem.md` |
| Cloth/thin shells | `docs/tutorial/cloth.md` |
| Rigid-soft coupling and contact materials | `docs/tutorial/coupled_contact.md` |
| Geometry and animation | `docs/tutorial/geometry.md`, `docs/tutorial/animation.md` |
| Every registered Scene config key/default/range | `docs/specification/scene_config.md` plus `scene_configs/newton.md` and `scene_configs/contact.md` |
| Material equations and parameters | `docs/specification/constitutions/` |
| Generated C++ API | `Libuipc/...` routes from `docs/nav.md` (full API build only) |

When a public API changes, update the canonical page rather than creating a second
competing tutorial. User-facing scenario code should keep complete C++ and Python
tabs where both bindings exist; note binding gaps explicitly where they do not.

### Local preview modes

Run from the repository root:

```bash
# Production-equivalent build, including generated C++ API pages.
python scripts/build_docs.py -o site

# Live full preview. Requires Doxygen plus the mkdoxy/MkDocs dependencies.
mkdocs serve -f mkdocs-with-api.yaml

# Faster prose-only preview; generated Libuipc API routes are absent.
mkdocs serve -f mkdocs.yaml
```

The fast `mkdocs.yaml` configuration disables MkDoxy while `docs/nav.md` still
contains `Libuipc/...` routes. Consequently those API links return 404 in the fast
preview by design; use `mkdocs-with-api.yaml` to validate API documentation.

### How API pages are produced

`scripts/build_docs.py` invokes MkDocs with `mkdocs-with-api.yaml`. MkDoxy runs
Doxygen over public headers and generates the `Libuipc/...` pages referenced by
`docs/nav.md`. The wrapper prints the requested output directory and propagates a
nonzero MkDocs/Doxygen result to its caller, so a failed API build also fails CI.
If every API link is 404, check in this order:

1. the build used `mkdocs-with-api.yaml`, not the prose-only config;
2. Doxygen and mkdoxy completed without errors;
3. the generated `site/Libuipc/` tree exists;
4. `docs/nav.md` paths match the generated module/class names;
5. the deployed site came from the latest docs workflow.

The workflow `.github/workflows/docs.yml` builds on pull requests but deploys only
for pushes to `main`. Deployment copies the generated site into the separate
`spiriMirror/libuipc-doc` repository's `docs/` directory. It watches `docs/**` and
both `mkdocs*.yml` and `mkdocs*.yaml` patterns. A manual `workflow_dispatch`
builds but does not deploy because the copy/push steps are push-only. The workflow
uses Node 24-based `actions/checkout@v7` and `actions/setup-python@v7`.

The old browser login popup was caused by `polyfill.io` returning an authentication
response. Both active configurations now use the local MathJax setup plus jsDelivr
and must not reintroduce that dependency.

The XMake and deterministic-mode pages are part of the navigation, and the RMR
development link uses MkDocs' lowercase heading anchor. Remaining prose-only
warnings are generated-API routes that do not exist until MkDoxy runs.

## Release Workflow Boundary

`.github/workflows/python-wheels.yml` builds Windows/Linux wheels for CPython
3.10–3.14 on CUDA 12.8 (0.0.26 remains a 3.10–3.13 release). The normal flow is:

```text
tag/build -> installed-wheel smoke tests -> wheel artifacts -> TestPyPI
TestPyPI matrix + exact-version install gate -> official PyPI publication
official PyPI matrix + exact-version install verification
```

The manual `hotfix_publish.yml` path downloads wheel artifacts from a chosen run
and publishes them to PyPI, so it depends on those artifacts still being retained.
`scripts/smoke_test_wheel.py` is deliberately safe on hosted runners without a GPU:
it validates the imported version, packaged native directory, and
`Engine("none", workspace)`. This catches broken installation/layout and basic
ABI-load failures, but not the lazy CUDA backend. `import uipc` and the current
hosted smoke test cannot detect a missing CUDA runtime DLL; a CUDA-capable release
runner must additionally construct `Engine("cuda", workspace)` (or run the
equivalent doctor probe) before CUDA compatibility is considered verified.

## Current Build-Metadata Differences to Keep Visible

- CMake remains the primary and wheel-tested build. XMake now mirrors the
  active USD/VDB options, omits removed GUI/torch/RPC surfaces, and explicitly
  disables ccache.
- CMake/vcpkg pins fmt 10.2.1 while XMake pins fmt 12.1.0 for an NVCC compatibility
  workaround; keep ABI/version effects in mind.
- XMake's pybind target is an importable shared module (`.pyd` on Windows),
  enables USD consistently, and performs packaging copies synchronously.

When changing a component or option, update and validate both build descriptions;
package-manager version differences still require deliberate review.
