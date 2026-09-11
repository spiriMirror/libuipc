# 14 — Project Audit: API, Implementation, Frontends and Documentation

Audit date: **2026-09-11**. Implementation baseline:
`4757859ce69c039a4146fcd574e9c59484a31a87` (`refactor-main`).

This is an **inspection and documentation update**, not an implementation change.
No solver, kernel, binding, build script, workflow, addon implementation or installed
runtime was changed. Findings below are proposals for separately authorized work.
Normal rounding variation from valid floating-point atomic accumulation is **not**
a defect and is not a reason to serialize or restructure GPU execution (rule 16).

## Scope, method and limits

The review combines a repository-wide inventory, targeted source tracing across
public API/binding/backend boundaries, existing tests, small independent probes,
documentation generation/link checks and read-only CI/deployment inspection.
It is not a proof that every possible bug has been found. Generated derivatives
were not all independently re-derived; untrusted file formats were not exhaustively
fuzzed; Linux/GPU-driver matrices, optional USD/VDB and multi-GPU behavior were not
re-executed locally.

| Area | Tracked files in baseline | Review emphasis |
|---|---:|---|
| `include/uipc` | 239 | Public contracts, lifetimes, parameter semantics |
| `src/core` | 98 | Scene/World/Engine, configuration, storage and serialization |
| `src/constitution` | 42 | Parameter conversion, attributes, documentation formulas |
| `src/geometry` | 76 | Factories, measures, METIS boundary, tetrahedralization |
| `src/backends/cuda` | 512 | Integration, PCG/graphs, buffers, contact/assembly boundaries |
| `src/backends/common` | 26 | Backend lifecycle, ABI and registration |
| `src/pybind` | 228 | Ownership policies, actual exports and docstrings |
| `src/io` / `src/sanity_check` | 9 / 21 | Validation and input/trust boundaries |
| `src/usd` / `src/vdb` | 10 / 3 | Optional-module/build boundaries; runtime coverage limited |
| `python/tests` / `integrations/blender` | 24 / 83 | Python and Blender behavior, helpers and UX |
| `docs` | 105 | Public reference, tutorials, API generation and navigation |

Local untracked `GPU_IPC/`, `Stiff-GIPC/` and `references/` were not audited as
separate products. The dirty samples checkout was not changed: its current commit
was `ecad2ca149303c852d9075d16e90443a8fea8be7`, different from the root gitlink.
Sample observations must therefore not be mistaken for a clean checkout of the
root repository's pinned sample revision.

Evidence levels: **reproduced** = a focused run demonstrates the behavior;
**source-confirmed** = the relevant control/data path is explicit in source;
**risk** = a plausible failure needs a dedicated reproduction; **opportunity** =
an improvement needs measurements/design before implementation. P1 means prioritize
before relying on the affected path, not that every ordinary simulation is broken.

## Executed checks

| Check | Result |
|---|---|
| Repository Python unit tests | 49 passed |
| Blender portable protocol/helper tests | 54 passed |
| Native Python tests, `-m 'not example'` | 128 passed, 6 example tests excluded; Warp adapter skipped because `warp` is not installed |
| Existing C++ common/core/geometry binaries | 3 / 36 / 46 cases passed; 11 / 1040 / 2730 assertions |
| Existing CUDA backend test binary | 22 cases, 337 assertions passed |
| Isolated sim-case runner | **95/95 passed**, approximately 2.3 minutes |
| Targeted `linear_pcg` particle probe under Compute Sanitizer synccheck | 0 reported synchronization errors; not an exhaustive sanitizer audit |
| Constitution API, workflow-pin, release-policy, generated UID and empty-file checks | Passed |
| Scene-config Markdown defaults vs native schema and `default_config()` | **48/48 keys present and equal**; domains/selectors also reviewed against schema |
| Full MkDoxy/Doxygen site build | Passed, 800 generated HTML pages |
| Generated HTML link scans | Initial href/src scan: 3 missing video targets + 11 invalid API anchors; extending to poster attributes found 3 more missing image targets |
| Final generated href/src/poster and anchor scan | 0 missing local file targets; **11 API anchors remain unfixed** |
| Current `refactor-main` Repository Contracts run | Passed: [run 34558951608](https://github.com/spiriMirror/libuipc/actions/runs/34558951608) |

Tests used the existing Windows binaries and Python 3.14 native build (`0.9.0`,
CUDA 13.2.51, RTX 5090); no rebuild or installation was performed. CMake/XMake
cross-platform jobs for the baseline were still in progress at the CI snapshot;
do not report them as passed based on the local results.

Ignored reproducibility artifacts: `output/project-audit-2026-09-11/`, especially
`native_api_v3/api_probe.json`, `lifetimes.json`, `blender_frontend.json`,
`mesh-doctor/result.json`, `sim-isolated.log`, `synccheck-linear.log`,
`docs-build.log`, `docs-build-after.log`, `doc_links-before.json`, `doc_links.json`,
`doc_links_local.json` and `config_docs.json`. Failed drafts of diagnostic probes
are not product findings. Existing tests also write to their normal ignored output paths.

## Implementation and API findings — not fixed

### A01 · P1 · Python borrowed facade objects do not retain their owners

**Reproduced ownership gap; dangling-reference consequence is a risk, not an
intentionally triggered use-after-free.** `Scene.config()`, `Scene.objects()`,
`Scene.geometries()` and `SimplicialComplex.vertices()` return value-type facades
that hold C++ references. Their Python bindings do not keep the Scene/mesh alive.
Holding only the facade, then deleting the owner, allowed a weak reference to the
owner to expire. The control `Scene.contact_tabular()` retained its owner.

Evidence: [scene bindings](../src/pybind/pyuipc/core/scene.cpp),
[mesh bindings](../src/pybind/pyuipc/geometry/simplicial_complex.cpp),
[reference-backed mesh facade](../include/uipc/geometry/simplicial_complex_attributes.h),
[Object geometry facade](../include/uipc/core/object.h).
Related facade families should be audited together; not every family was probed.

Impact: storing a facade returned from a temporary or expired parent can leave a
Python object referring to freed C++ state. Recommendation: define and test parent
ownership for all facade returns, using suitable owner retention or owning handles.
Until then, retain explicit Scene/Geometry/Object variables. Do not dereference a
facade after deleting its owner.

### A02 · P1 · BDF2 startup does not preserve a prescribed constant velocity

**Reproduced algorithmic issue, unrelated to atomic rounding.** A single particle
with velocity `(1,0,0)`, zero gravity and disabled contact should move 0.01 m per
0.01 s step. BDF1 gave `[0.01, 0.02, 0.03, 0.04]`; BDF2 gave
`[0.0066666667, 0.0155555556, 0.0251851852, 0.0350617284]`, while `World.is_valid()`
remained true. The first-step discrepancy is one third of the intended displacement.

The [FEM BDF2 bootstrap](../src/backends/cuda/finite_element/bdf/fem_bdf2_time_integrator.cu)
copies current/previous position storage into history rather than constructing a
consistent preceding state or taking a startup step. The
[BDF2 formula](../src/backends/cuda/finite_element/bdf/sym/bdf2.inl) then applies
fixed `4/3`, `1/3`, `2/9` coefficients. The ABD implementation has the same history
construction pattern, but the focused runtime probe was FEM/particle only.

The formula also has no previous-step-size input. A variable-dt probe differs
from the constant-velocity trajectory, so do not imply variable-step BDF2 support
from the fact that dt is read live. Current docs already discourage general config
hot reload; this limitation should be explicit for BDF2. Recommendation: separately
design startup/history behavior and variable-step support or a clear restriction.
Default BDF1 was correct in both constant- and varying-dt control probes.

### A03 · P1 · Pybind post-build helper has destructive side effects

**Source-confirmed; destructive cases were not executed.**
[after_build_pyuipc.py](../scripts/after_build_pyuipc.py) uninstalls the existing
`pyuipc` before validating the build configuration, generating stubs or installing
the replacement. Even wheel-build mode reaches this uninstall; that mode only skips
the later installation. A failure can leave the chosen interpreter without its
previous working package.

The same helper deletes `binary_dir/python`. The
[CMake hook](../src/pybind/pyuipc/CMakeLists.txt) passes `CMAKE_BINARY_DIR`, and the
root CMake entry has no in-source-build rejection. In an in-source pybind build,
that path aliases the tracked source `python/` directory. This is a separate
source-loss hazard, not just a failed install. Always use an out-of-source build
and an isolated interpreter with the current implementation.

Recommendation: separate staging, verification and explicit installation; validate
resolved source/build paths before deletion; do not remove a working package before
a replacement is ready. This was previously deferred, not fixed by this audit.

### A04 · P2 · ElasticModuli's Poisson guard rejects a valid value and admits invalid ones

**Reproduced.** `ElasticModuli.youngs_poisson(1000, -0.5)` raises because the
[guard](../src/constitution/elastic_moduli.cpp) uses `abs(nu) != 0.5`, although
the 3D Lamé conversion is nonsingular and physically admissible at -0.5.
The nearby -0.49 is accepted. Conversely, 0.6 and NaN are accepted and produce
invalid bulk stiffness or NaN moduli. E/rho/radius validation across material entry
points is also inconsistent; this review did not fuzz every overload.

Recommendation: define model-specific domains and singularity checks, explicitly
including admissible auxetic values where supported, and distinguish recommended
material ranges from enforced validity constraints. Preserve valid negative lambda
values; a blanket `lambda >= 0` rule would be wrong for auxetic materials.

### A05 · P2 · Manual hotfix publication does not select the requested source run

**Source-confirmed against the pinned action.**
[hotfix_publish.yml](../.github/workflows/hotfix_publish.yml) supplies `run-id` but
not `github-token`. The pinned download action uses the requested workflow run
only when that token input is supplied; otherwise it searches the current run.
See its [input contract](https://github.com/actions/download-artifact/blob/d3f86a106a0bac45b974a628896c90dbdf5c8093/action.yml)
and [implementation](https://github.com/actions/download-artifact/blob/d3f86a106a0bac45b974a628896c90dbdf5c8093/src/download-artifact.ts).

This hotfix job has no earlier artifact-producing step, so it does not implement
its advertised cross-run recovery. Once repaired, source workflow/repository/ref,
conclusion, version, wheel matrix and artifact integrity also need validation before
publishing. Its environment/OIDC configuration should be aligned with the normal
release path. No publication or workflow changes were performed.

### A06 · P2 · MeshDoctor reports unsupported FEM checks as success

**Reproduced.** [MeshDoctor.check_mesh](../python/src/uipc/dev/mesh_doctor.py)
returns `True` for the FiniteElement branch containing a TODO. A 0D point cloud
checked with `StableNeoHookean()` therefore returns success without checking the
required tetrahedral geometry. This is the Python developer helper; the current
CLI exposes only its ABD branch.

Separately, the [CLI](../python/src/uipc/cli/mesh_doctor.py) defines `--gui` as
`store_true` with default true and no disable option. Headless CLI callers cannot
turn the viewer off. Recommendation: distinguish checked/unsupported/failed and
make visualization explicitly controllable. Native World sanity checks are separate
and should not be replaced with this helper's result.

### A07 · P2 · Curve-to-rod import silently discards centerline modifiers

**Reproduced frontend contract mismatch.**
[rod_ui.py](../integrations/blender/libuipc_blender/rod_ui.py) copies the raw Curve
datablock and clears all object modifiers. A Geometry Nodes offset made every visible
input vertex have y=1; the converted rod had y=0 and the operator returned FINISHED.
The original curve was preserved. The UI description's evaluated-centerline wording
does not explain this exclusion.

Recommendation: explicitly decide whether to apply supported centerline modifiers,
reject modified curves or expose a base/evaluated source choice. Until then, apply
desired centerline deformations before conversion; source bevel/extrusion removal
is intentional. Only the documentation limitation was clarified here.

### A08 · P2 · CUDA graph launch error propagation is incomplete

**Source-confirmed unchecked return paths; no launch failure injected.**
[GraphCapture::launch_sync](../src/backends/cuda/cuda_tool/graph.h) ignores stream
creation, graph launch and synchronization return codes. GraphWhile returns the
synchronization status but ignores the earlier creation/launch results. A launch
failure need not be reliably attributed at the point it occurs, and later scalar
reads may make diagnosis misleading.

Recommendation: preserve concurrent execution but propagate launch/capture errors
at their source. Validate failures in an isolated test; this is not a proposal to
add synchronization or alter floating-point reduction order.

## Documentation and public-facing contract findings

| ID | Priority | Evidence / disposition |
|---|---|---|
| D01 | P2 | `Scene.config()` binding docstring says dict; runtime returns `ConfigAttributes`. Public guide clarified; binding docstring remains an implementation follow-up. |
| D02 | P2 | Full generated site has 11 invalid anchors in `Libuipc/namespaceuipc`, including unit literals and a `readable_type_name` overload. Build exits successfully. Generator/slug handling and an HTML link gate remain follow-up work. |
| D03 | P2 | Three raw HTML video sources and their three poster images resolve relative to directory URLs incorrectly. Corrected all six documentation paths while retaining the videos/images. |
| D04 | P2 | [Docs workflow](../.github/workflows/docs.yml) builds API from `include/` but its path filter does not include public-header changes. Fork PRs skip the entire docs job rather than only deployment. Recommend separate unprivileged build checks from publication. No workflow edit made. |
| D05 | P2 | README's no-CPU-round-trip claim conflicts with default PCG block replay and host convergence checks; fully-differentiable wording overstates the current partial DiffSim surface. README and docs overview wording corrected. |
| D06 | P3 | Stale counts, empty-placeholder/unbound-motor claims and a manual UID-edit recipe were corrected. The geometry tutorial confused dimension with codimension and package with module name; the Blender parameter table omitted current rod/FEM support. These descriptions are corrected without changing APIs. |
| D07 | P2 | CUDA source comments claim blocking streams do not wait for the legacy default stream. NVIDIA documents the opposite for streams in the same context; non-blocking/per-thread modes are distinct. Record the correction in docs; do not remove existing synchronization based on this comment alone. |
| D08 | P2 | Hookean spring spec blurred energy density, Young-like kappa and total edge energy. Clarified the section-volume weight using the consuming kernel; no model formula changed. |
| D09 | P2 | Scene-config tables called enforced ranges "normally" valid or positive "when used". Hard numeric constraints apply even to disabled/overridden settings; corrected those descriptions and selector rejection behavior. All 48 documented defaults match the native schema. |
| D10 | P2 | The deterministic-mode design still called itself an agreed proposal. Marked historical/withdrawn, explicitly not a supported config key or an authorized implementation task, consistent with rule 16. |

For D07 see NVIDIA's [stream synchronization contract](https://docs.nvidia.com/cuda/cuda-runtime-api/stream-sync-behavior.html).
The finding concerns explanatory text, not a newly demonstrated race. The targeted
synccheck run reported no errors, and no fixed-order reduction change is warranted.

### Frontend design and cross-layer contracts

The inspection covers both the C++/Python scene-building frontend and Blender,
not only the visual layout. Preserve the existing external-worker boundary: it
avoids forcing Blender's bundled Python to match the solver's interpreter ABI.
The current split between portable protocol/material rules, Blender UI and native
worker is useful; there is no evidence here for replacing it wholesale.

| Surface | Source trace / conclusion | Remaining improvement |
|---|---|---|
| Scene config | Typed declarations generate defaults/schema; 48/48 documented defaults match; strict validation is already present | Clarify facade lifetime, init-only expectations and BDF2 limitations (A01/A02/D01/D09) |
| Constitution API | Parity check covers 43 public classes and 40 binding initializer registrations; this checks exposure, not every overload's behavior | Uniform per-model units/defaults/domains and lifetime/shape examples; targeted material validity tests (A04) |
| Independent cloth channels | [`materials.py`](../integrations/blender/libuipc_blender/materials.py) resolves three E/nu pairs; [`worker.py`](../integrations/blender/libuipc_blender/worker.py) passes separate stretch/shear moduli and bending E/nu | Keep old-scene fallback; do not merge the three controls or add another thickness factor |
| Solver controls | Default/Converged/Custom route through `apply_solver_accuracy`; docs correctly distinguish native defaults, plugin overrides and strict-mode limit reporting | Expose effective settings clearly; convergence/finite-difference diagnostics are not a physical-accuracy certificate |
| Rod preparation | Hookean stretch + Kirchhoff bending, no twist, are exposed; fixed/pin/self-contact fields reach the worker | Make base-vs-evaluated Curve conversion explicit (A07); rest-curvature/twist/animated-pin extensions need separate scope |
| Playback/rendering | Stable identity, relevant-input fingerprints, transactional attachment and full-affine compact ABD are covered by the existing 54 portable tests | Retain full affine rather than TRS; queue resume is image rendering, not solver checkpoint resume; normal F12 is not the plugin's guarded render entry |
| Bulk material UX | [`material_ui.py`](../integrations/blender/libuipc_blender/material_ui.py) assigns multiple fields; each update can invoke `changed()` and scan scenes/objects in `__init__.py` | Batch invalidation once, after measurements, rather than dropping cache correctness checks |

The checked cloth conversions retain the full material thickness `2r` for stretch
and bending, the owner's thickness-independent shear convention, and the complete
DSB hinge metric `3 L0^2 / A` without a second area multiplier. These are not newly
found bugs. Persistent CUB scratch and growth policies already exist; do not propose
introducing them as if the project currently allocates every operation from scratch.

### Deployment and advisory observations

- Public homepage, API classes/links and scene-config reference returned HTTP 200
  without a `WWW-Authenticate` challenge. This does not prove that every external
  subresource or browser interaction is healthy; no automated browser was available.
- `/build_install/blender/` returned 404. `main` was
  `9c5710ee4b716502e3757e7312360e5ee68cb79f` and did not contain that file; deployment
  runs from main while the addon/docs are on refactor-main. This is a branch/release
  gap, not evidence that the local API build is broken. No merge/deployment was done.
- GitHub's open Dependabot alert query returned an empty list. That is not a full
  dependency-security audit or evidence that optional local libraries are current.
- The pytest floor is supported by [CVE-2025-71176 / GHSA-6w46-j5rx-g56g](https://github.com/advisories/GHSA-6w46-j5rx-g56g);
  the repository's >=9.0.3 requirement and tested 9.1.1 are beyond that advisory's
  affected range. The optional OpenUSD note previously used an unqualified critical
  label: the verified [GHSA-q75h-g2h7-fgxg](https://github.com/PixarAnimationStudios/OpenUSD/security/advisories/GHSA-q75h-g2h7-fgxg)
  is Moderate, with a fix available in 25.08. Advisory identity and installed version
  must accompany any vulnerability claim; no general all-advisories clearance is implied.

## Risks requiring dedicated validation

1. **GPU graph cache invalidation across same-DOF topology/preconditioner changes.**
   The PCG cache keys include solution/work-vector and matrix pointers, but not all
   pointers embedded by a preconditioner. Inspect same-size geometry replacement,
   MAS rebuilds and allocator growth before asserting safety. No failure reproduced.
2. **Native tetrahedralization latency/cancellation.** The conservative advancing
   front retries without a finite construction budget. This preserves the owner's
   boundary contract, but direct callers have no cooperative cancellation/progress
   API. Blender can kill its worker. Improve observability/cancellation without
   changing the boundary or treating a quality budget as construction failure.
3. **Raw NumPy/string views and reallocation.** Owner retention for an attribute
   slot does not by itself stabilize its vector storage across resize/COW/rebinding.
   Reacquire views and copy persistent snapshots. Audit string-span ownership and
   storage-retaining designs separately; no freed-buffer dereference was attempted.
4. **Executable remote assets.** `uipc.assets.load()` imports downloaded `scene.py`,
   even with `geometry_only=True`, and defaults to revision main. It is code loading,
   not a sandboxed mesh parser. Use trusted/pinned revisions; strengthen user-facing
   trust warnings and consider an explicit data-only route. No malicious asset was run.
5. **Optional/cross-platform coverage.** Hosted CI compiles CUDA but does not run GPU
   simulation; local coverage is one Windows GPU/runtime. Warp was unavailable;
   USD/VDB, different drivers/architectures, multi-GPU contexts, malformed-input
   fuzzing and all generated derivatives remain unverified by this audit.

## Improvement opportunities — design or measure before implementing

| Area | Candidate improvement | Required evidence / boundary |
|---|---|---|
| API ownership | Consistent facade lifetime contracts, return types and invalidation rules | Parent-GC, resize, COW and temporary-expression tests; retain the data-oriented API |
| Material/config UX | Machine-readable constitution parameter units, defaults and valid domains | Trace every field to its consumer; distinguish recommendations, constraints and init-only/live settings |
| Build/install | Stage and verify packages before any explicit install | Test failures, in-source/path-alias rejection and isolated interpreters; no editable-install requirement inferred |
| Documentation | Generated HTML file/anchor checks, executable C++/Python snippets, versioned API coverage | Test the generated site, not just Markdown links; keep demos on the homepage and educational videos in tutorials |
| Blender large scenes | Batch preset invalidation once per scene; keep existing preview/frame/GPU caches | Current per-property callbacks can repeatedly scan all objects; measure 1k+ object scenes before changing callbacks |
| Solver throughput | Profile existing atomic SpMV, contact assembly and preconditioning end to end | Use current four-scene baselines; preserve CUB/persistent scratch, geometric growth and kernel fusion rules |
| Memory observability | Optional capacity/utilization, contact-count and peak-memory diagnostics | No new unconditional GPU synchronization, per-frame reallocations or oversized initial buffers |
| Meshing startup | Measure METIS/mesher startup and safe reusable topology products | Cache keys must include topology/partition policy; preserve exact boundary/partition contracts |
| Release confidence | GPU-backed engine/simulation smoke and explicit artifact provenance | Cover ordinary release and manual recovery paths; separate builds from secret-bearing publication |
| Architecture hygiene | Keep ownership boundaries and retire only proven unused paths | Do not replace RMR/DOP or split/fuse kernels merely for stylistic consistency |

Suggested order for separately approved work: A01/A03 lifetime and data-safety
boundaries; A02 integrator correctness; A04/A05/A06/A07/A08 focused fixes; then
documentation automation and measured performance work. Do not turn this list into
authorization to implement or into an obligation to achieve bitwise determinism.
