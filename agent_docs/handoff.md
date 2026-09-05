# Handoff — Current State of the Repo

> **Native tetrahedralization and Blender FEM/Fixed (2026-09-05)**: implemented
> original Apache-2.0 geometry code under `src/geometry/tetrahedralization`, with
> exact protected boundary or optional surface refinement. Conservative volume
> construction precedes quality work and survives quality rollback. C++/Python
> APIs expose mesh/report/source IDs; both build systems preserve FP predicates.
> Blender 0.2 prepares native volumes or imports MSH, retains all internal nodes
> for pin selection, and supports Fixed Entire Object for ABD/FEM/cloth. See
> [ADR 0008](adr/0008-native-tetrahedralization.md), the native geometry tests,
> and `integrations/blender/tests/blender_fem.py` for the checked contracts.

> **Blender integration (2026-09-05, `refactor-main`)**: added an independently
> packaged Blender extension for cloth/ABD/fixed contact, friction and pins,
> with a separate Python CUDA worker and native MDD Mesh Cache playback.
> Real Blender 4.5.3 + official pyuipc 0.0.28 + RTX 5090 tests cover every cached
> vertex, backward/fractional playback, .blend reopening/rendering, scale/units,
> friction, cancellation, invalidation, rebake and detach. Export uses evaluated
> matrices after reproducing stale original transforms; Windows atomic status
> replacement retries reader-sharing conflicts. Source, packaging commands,
> integration tests and remaining scope are in
> [`13-blender-integration.md`](13-blender-integration.md).

> **Routing note (2026-08-30)**: this file is the chronological audit trail and
> may retain detailed commands/incidents. New durable architecture rationale
> belongs in `agent_docs/adr/`; reusable performance conclusions and rejected
> experiments belong in `agent_docs/performance/`. Add a short handoff pointer
> instead of growing this file as the only source of truth.

> **Embedded C++ METIS migration (2026-09-03, `refactor-main`)**: geometry now
> links the private `uipc_metis` target from `src/geometry/metis/`; the separate
> `external/METIS` and `external/GKlib` source trees and build targets were
> removed. CMake and XMake keep METIS independently compilable and exclude its
> sources from direct `uipc_geometry` compilation. The C++ port was made
> const-correct for diagnostic strings and self-contained for modern MSVC.
> Unused glibc getopt/regex/qsort and optional MT19937-64 sources were removed;
> required sorting now uses a clean in-tree deterministic C++
> partition/insertion implementation that preserves the former equal-key
> ordering, so only the METIS/GKlib Apache notices remain. A deterministic
> public `mesh_partition` regression covers the linked
> API; zero-sized partitions, 32-bit capacity overflow, and invalid returned
> partition IDs are rejected before division or indexing. Portable CPU/wall
> timers and out-of-core temporary-file removal replace incomplete port stubs.
> Against preserved pre-removal C binaries, three synthetic graph families,
> four full `fluffy_ball.msh` configurations, and two `animal_well.msh`
> configurations on both Windows/MSVC and Linux/GCC produced identical return
> codes, edge cuts, and every per-vertex partition ID on the same platform. See
> [`ADR 0007`](adr/0007-embedded-cpp-metis.md) for exact hashes and boundaries.
> PR #492's first Linux XMake run caught a compiler-filtered `-fPIC` flag being
> silently omitted from `uipc_metis`; the target now applies the C++ flag
> without a tool-name filter while retaining XMake's support probe, matching
> CMake's PIC property and allowing its
> thread-local GKlib state to link into `libuipc_geometry.so`. The adjacent
> GNU/POSIX `strerror_r` portability warning was fixed at the same time so Linux
> error paths return the correct diagnostic string.

> **AL-IPC sample 88 trajectory correction (2026-09-03, `refactor-main`)**:
> the severe early trajectory split was traced to AL ignoring the configured
> `K_min=6`, an EE friction derivative assembled with the PT Jacobian, and the
> uniform `diag_norm` penalty being unsuitable as the default for mixed
> cloth/volumetric masses. AL now delays cumulative safe-path attenuation until
> `K_min`, uses the correct EE Jacobian, and defaults to mass-based `per_vertex`
> scaling while keeping `diag_norm` experimental. Exhausted line search restores its recorded start
> point rather than accepting the last energy-increasing trial. PT/EE/plane
> finite-difference tests pass, as does the AL `K_min` simulation assertion.
> In matched 60-frame sample 88 runs, upper/lower centroid error versus IPC fell
> from 0.2963/0.0807 to 0.0115/0.0045. A full corrected 250-frame run completed
> with all 250 frames converged and no line-search limit, Newton limit, or
> runtime error. See the durable evidence and
> regression boundary in
> [`2026-09-03-al-ipc-case88-correction.md`](performance/2026-09-03-al-ipc-case88-correction.md).
> **Parallel-EE policy (2026-09-03, `refactor-main`)**: standard IPC retains
> normal `need_mollify()` detection with coefficient `1e-3` and its complete
> mollified normal energy/gradient/Hessian; standard IPC friction skips the
> detected parallel pairs. AL-IPC uses the shared negative disabled threshold
> for both normal and frictional contact, so all AL pairs follow ordinary EE
> paths. The CUDA regression checks both threshold behaviors with exactly
> parallel edges.
> Final validation passed CMake/Core 36 cases / 1040 assertions, CUDA backend
> 22 / 335, the single-process simulation suite 95 / 14213, Python portable
> tests 80 passed / 1 skipped, repository contracts 43/43, fast CTest 3/3,
> the complete Doxygen/MkDoxy/MkDocs build, and the XMake production CUDA
> target. XMake's aggregate CUDA test target still encounters the already
> documented CUDA 13.2/fmt 12 character-literal incompatibility; the CMake
> build compiled and executed every new CUDA test.

> **AL-IPC `AL-release` integration audit (2026-09-02, `refactor-main`)**:
> the six fork-only commits were reviewed against current libuipc and the
> AL-IPC paper rather than cherry-picked across 188 intervening local commits.
> The useful core is reimplemented in current `cuda_tool`: per-vertex
> earliest-TOI candidate filtering (excluding existing pairs), decay-derived
> active-pair lifetime, stable preservation of old pair state, configurable
> Hessian-diagonal penalty initialization, AL-specific CCD safety margin, and
> the full-step boundary between inner Newton work and outer multiplier/CCD
> updates. Scratch is persistent and amortized. Important fork defects were
> not copied: its 32-bit `__float_as_int` atomic corrupts this project's
> double-precision `Float`; its fixed 25-update lifetime disagrees with the
> documented `gamma < 0.01` rule; it includes existing pairs in the candidate
> minima; and its raw displacement convergence test discards the current
> `NewtonToleranceManager`. The fork's global CCD-margin edit was scoped to
> AL-IPC so normal IPC retains its established margin. Large benchmark assets,
> screenshots, and unrelated example/README edits were intentionally omitted.
> At this audit revision AL kept its pre-existing `K_min = 1`
> cumulative-safe-path termination; the 2026-09-03 correction above supersedes
> that limitation with the configured multi-state `K_min > 1` rule. On
> CUDA 13.2 / RTX 5090, CMake and XMake production builds passed, as did the
> focused AL math test (17 assertions after final review), Core (36 cases /
> 1040 assertions), CUDA backend (17 / 291), all 19 AL sections, and the full
> single-process simulation suite (95 / 14212). Repository contracts passed
> 43/43, the current native Python schema reports 48 keys, and the full
> Doxygen/MkDoxy/MkDocs site built. In mixed ABD/FEM case 18, maximum active
> pairs fell from 81 to 42 and total PCG iterations from 1125 to 700; outer
> solves rose from 114 to 135, so future performance claims must still use the
> canonical large scenes.

> **cuBLAS-free CUDA runtime boundary (2026-09-02)**: published wheels through
> 0.0.27 directly import CUDA 12 cuBLAS, but current source no longer uses or
> links cuBLAS. `LinearSystemContext` dot/norm now use named block-partial
> kernels plus CUB final reduction; partial/result storage is persistent and
> geometrically grown, CUB scratch remains per-stream and persistent, and norm
> uses a scaled-square state to avoid overflow/underflow. The heavy reduction
> header is included only by the three calling TUs and focused CUDA test, so the
> lightweight `linear_system.h`/`cuda_tool.h` path does not transitively pull in
> all CUB algorithms. CMake and XMake links are synchronized. A wheel CI audit
> rejects dynamic Toolkit libraries via dumpbin/readelf; compatibility policy
> requires driver >=525.60.13 (Linux) or >=528.33 (Windows) when a packaged
> SASS image applies, and >=570.124.06 / >=572.61 when the CUDA 12.8 Update 1
> PTX image must be JIT compiled. Final
> CUDA 13.2/RTX 5090 validation passed the CMake and XMake backend builds, the
> 12-assertion focused reduction test, all 274 CUDA-backend assertions, all
> 14,212 assertions in the 95-case simulation suite, four representative
> benchmark scenes, 43 repository-script tests, 79 portable Python tests, the
> real CUDA doctor probe, and a full Doxygen/MkDoxy/MkDocs build. Both local
> build systems produced a backend DLL with no CUDA Toolkit imports.

> **Windows wheel RDC fix (2026-09-01)**: `main@b6f2b006` had five real
> Windows wheel failures; CMake, XMake, repository contracts, and all five Linux
> wheels passed. scikit-build-core's Visual Studio generator compiled all 198
> domain OBJECT sources with RDC but omitted `nvcc -dlink`, ending with 198
> unresolved `__cudaRegisterLinkedBinary_*` symbols. A minimal cross-TU CUDA
> probe reproduced the failure for both OBJECT attachment idioms. Giving the
> final shared target one directly owned, generated comment-only `.cu` source
> made Visual Studio emit the correct device-link and the probe passed. CMake
> now carries that language anchor; domain ownership and XMake are unchanged,
> and a repository contract prevents its removal. Validation then configured
> the real project with the Visual Studio 2022 generator, compiled all 198 CUDA
> sources, executed `nvcc -dlink` over every domain object, produced and loaded
> the 17,622,528-byte `uipc_backend_cuda.dll`, and passed `0_abd_gravity`
> (IPC + AL-IPC, 178 assertions). The existing Ninja build also regenerated,
> device-linked, and rebuilt `sim_case` successfully; repository contracts pass
> 36/36. GitHub then passed CMake run `33490698492`, XMake run `33490698388`,
> and Repository Contracts run `33490698389`. A manual non-publishing wheel run
> `33490746393` passed all ten CPython 3.10–3.14 Windows/Linux jobs; the five
> Windows jobs that had failed on `main` now build, install-test, and upload
> successfully.

> **Canonical benchmark suite expansion (2026-09-01)**: root benchmark
> ownership now covers sample 6 (`rigid-wrecking-balls`), 88
> (`stiff-gipc-case2`), 89 (`mas-bunny`), and 93 (`cube-wall-cloth`). The sample
> implementations share machine-readable headless reporting without changing
> scene parameters. Normal runs keep synchronized Timer scopes off and archive
> full-precision frame times, backend Newton/line-search/linear-solver counts,
> final-state observables, raw logs, revisions/runtime facts, and an approximate
> WDDM total-memory peak; separate Timer-enabled runs provide stage diagnostics.
> All four entries completed real three-frame smoke runs and then three full
> interleaved throughput runs. Median run means on RTX 5090/CUDA 13.2 are
> 129.5 ms/frame (rigid, 120 frames), 201.1 (case2, 250), 60.2 (MAS bunny,
> 100), and 125.6 (wall/cloth, 100); all frames completed/converged with no
> iteration-limit hits. Collision-rich trajectories and WDDM memory/timing
> have measured envelopes rather than exact goldens. Durable method, raw run
> IDs, stage diagnostics, and interpretation are in
> `agent_docs/performance/2026-09-01-cross-domain-baseline.md`; historical
> 73/156/301 ms notes below are not the current baseline.

> **Post-merge clang-format race fix (2026-09-01)**: PR #486 merged while its
> format job was running. The job then fetched moving `origin/main` with
> `--depth=1`, replacing the original PR base and producing `no merge base`;
> no source formatting violation occurred. The workflow now diffs the immutable
> pull-request `base.sha...head.sha`, and a repository contract prevents the
> moving shallow-fetch pattern from returning.

> **QR-SVD float sign hardening (2026-09-01)**: the Wilkinson shift in
> libuipc, GPU_IPC, and Stiff-GIPC now computes its magnitude in the template
> scalar type and applies the sign with `d < 0 ? -shift : shift`, explicitly
> taking `sign(0)=+1`. This removes the unsafe standard-library sign-copy call
> from CUDA float paths. The sibling commits are GPU_IPC `4fb6019` and
> Stiff-GIPC `bb2849a`. Libuipc has a named-kernel regression covering float
> positive, negative, and negative-zero inputs on the GPU; the repository
> contract also forbids reintroducing the call. A standalone CUDA 13.2 / sm_120
> execution returned `[-1, -0.414213538, 2.41421366]`; libuipc's complete CUDA
> backend built with device-link in 541 seconds and sim_case passed 95 cases /
> 14212 assertions. GPU_IPC and Stiff-GIPC both completed full Release builds;
> GPU_IPC also completed one frame. Stiff cases 1 and 2 remain blocked before
> QR-SVD by their pre-existing `DeviceBuffer allocation size overflow` during
> initial triplet allocation. The local aggregate backend test target remains
> blocked by the unrelated CUDA 13.2/fmt 12 character-literal issue, although
> the new test translation unit itself compiles.

> **Samples cloth calibration (2026-09-01)**: at that checkpoint,
> `libuipc-samples/main` ended at `4e83b83` and the parent gitlink followed it.
> The later benchmark contract advances the gitlink to `4fb26b7` on the
> samples `benchmark-baseline` branch; measurements used its scene-identical
> instrumentation parent `8701983`, and the child only corrects benchmark
> prose. All
> eight simulated-cloth
> examples use Baraff-Witkin membrane plus formula-based Discrete Shell
> bending with one-sided thickness `r=1e-3` and density 200. Example 88 remains
> the common baseline (`stretch E=5e4`, `shear E=1e1`, `nu=0.49`,
> `strain_rate=100`, `bending E=3e4`); subsequent owner tuning sets example 11
> to stretch/bending `E=1e4`, example 34 to `E=1e4`, `nu=0.40`, and example 93
> to `strain_rate=10000`. Python syntax checks passed for every changed scene.

> **Semi-implicit Newton default (2026-09-01, `refactor-main`)**:
> `newton/semi_implicit/enable` now defaults to `1` and
> `newton/semi_implicit/K_min` defaults to `6`. The beta tolerance remains
> `1e-3`, and `K_min` remains an accumulation start rather than a hard Newton
> iteration floor; `newton/min_iter` is still the separate floor and defaults
> to zero. The scene-config schema test locks both new defaults. Validation
> passed Core 36 cases / 1005 assertions and the complete CUDA sim suite 95
> cases / 14212 assertions.

> **Thin-shell reference-weight correction (2026-09-01, `refactor-main`)**:
> the elastic, strain-plastic, and stress-plastic Discrete Shells paths had
> multiplied their already integrated `L0/h_bar = 3L0^2/A` metric by `A` a
> second time. Their generic outer weight is now neutral, preserving the
> paper's inverse-area normalization and uniform-scale invariance. The stored
> vertex `thickness=r` is explicitly one-sided: formula-based bending uses
> `E*(2r)^3/(12*(1-nu^2))`, and Baraff-Witkin stretch uses
> `(lambda+2mu)*(2r)`. Its separately calibrated shear coefficient remains
> thickness-independent. Focused regression coverage checks all three bending
> variants at two uniformly scaled rest hinges. Validation passed repository
> contracts 30/30, Core 36 cases / 1001 assertions, the full CUDA backend build
> including device-link, sim_case 95 cases / 14212 assertions, and the focused
> MAS stitch regression 1 case / 4 assertions. Rebuilding the complete local
> CUDA test executable remains blocked by the existing CUDA 13.2/fmt 12
> character-literal incompatibility in unrelated `.cu` tests; the new host-side
> reference-weight test itself compiles.

> **GitHub check portability fixes (2026-08-31, `refactor-main`)**: successive
> full XMake runs exposed that CUDA OBJECT dependencies first lacked the project
> `src/` include, then backend definitions, then Linux PIC, and finally did not
> enter XMake's CUDA device-link at all. The last issue produced 198 unresolved
> `__cudaRegisterLinkedBinary_*` symbols on Windows; Linux had allowed the same
> unresolved references in a shared object. The final design keeps CMake OBJECT
> targets, but XMake attaches its matching logical component manifest directly
> to the one shared target. That gives both platforms one complete RDC
> device-link and inherits the final target's includes, definitions, and PIC.
> A repository-contract test prevents reintroducing XMake CUDA OBJECT
> dependencies. Repository Contracts deliberately checks out no submodules, so
> its benchmark-manifest test now
> validates the declaration without requiring
> samples assets; runtime asset validation has an isolated partial-checkout unit
> test. Rapid-push cancellations were concurrency behavior, not additional
> source failures. Local repository contracts passed 29/29. A clean Windows
> `xmake build --jobs=8 cuda` emitted the expected
> `devlinking.release uipc_backend_cuda_gpucode.cu.obj`, linked the DLL, and
> completed in 574.109 seconds; a legacy-off configuration omitted exactly the
> three legacy filter sources. The local XMake configuration was then restored
> to legacy-on and native architecture selection.

> **Aggregate architecture validation (2026-08-30, through `67ff50c3`)**:
> the default Release build completed for Core, none/CUDA backends, and all
> native test targets. CTest passed 7/7 aggregates with CPU work concurrent and
> all four GPU entries serialized by `uipc_gpu`; this includes sim_case 95 /
> 14212. Python passed 79 portable tests (1 skipped, 54 deselected) and 48 CUDA
> non-example tests (1 skipped, 85 deselected). Repository/script contracts
> passed 27/27; default, legacy-off, and CUDA-off XMake configurations parsed;
> full MkDocs+Doxygen output built; clang-format-18 passed for every C++ change
> versus `origin/main`. Both tracked submodules were clean and `refactor-main`
> matched `origin/refactor-main` before this documentation checkpoint.

> **ADR and performance evidence archive (2026-08-30, `refactor-main`)**:
> five accepted ADRs now cover the backend handshake, CUDA component boundary,
> scene-config contract, deterministic SimSystem topology, and test/benchmark
> entry points. `agent_docs/performance/` defines evidence/interpretation rules,
> provides a reusable template, and consolidates the four 2026-08-30 case2
> assembly stages plus the rejected stencil split. Existing handoff history is
> preserved rather than rewritten. Archive numbering/sections/local links are
> enforced by the scripts contract suite (27/27 passed).

> **Test sharding and benchmark registry (2026-08-30, `refactor-main`)**:
> the 95-case single-process `uipc.sim_case` remains the authoritative
> global-state-pollution regression. `run_sim_case_isolated.py` now also emits
> JSON manifests and stable sorted round-robin shards for parallel diagnosis;
> a real 4-way discovery produced 24 cases in shard 0. CTest serializes its four
> aggregate GPU executables through the `uipc_gpu` resource lock while allowing
> CPU concurrency. Root `benchmarks/manifest.json` promotes samples example 88
> as `stiff-gipc-case2`; `run_benchmark.py` validates assets/canonical env and
> records both git revisions plus run status. Validation: runner unit tests 8
> passed (complete scripts contract suite 25/25), dry-run/list, and an actual
> one-frame case2 run (return 0, metadata written with hardware/runtime facts
> and parsed frame timing).

> **Optional legacy collision component (2026-08-30, `refactor-main`)**:
> the V0, stackless, and linear-BVH simplex trajectory filters now belong to
> the logical `collision_legacy` component (CMake target
> `cuda_collision_legacy_objects`), separate from the default collision path.
> Compatibility builds retain all 198 CUDA sources; setting
> `UIPC_WITH_CUDA_LEGACY_COLLISION=OFF` or
> `cuda_legacy_collision=false` omits those three registrations and links 195
> sources. The scene schema carries the same build capability and exposes only
> selectors present in the DLL. A CUDA integration test constructs an engine
> for every advertised selector. Validation covered CMake/XMake ON and OFF,
> Python schema tests 5 passed in both configurations, default IPC simulation
> in both configurations, Core 36 cases / 1001 assertions, CUDA backend 13 cases
> / 250 assertions, and the full simulation suite 95 cases / 14212 assertions
> after restoring the default compatibility build. CUDA-only test targets are
> now conditionally included as well, so no-CUDA CMake/XMake configuration no
> longer retains dangling simulation/regression/example dependencies; an
> isolated no-CUDA build passed the none-only sanity suite (3 cases / 50
> assertions).

> **Deterministic SimSystem topology (2026-08-30, `refactor-main`)**:
> backend creators are sorted by complete demangled type name before system
> construction; exact lookup uses `std::type_index` rather than a potentially
> colliding raw hash; compatible derived lookup follows the same order and
> skips invalid variants. Build, invalidation, formatting, and `systems.json`
> now share that order. Active strong-dependency cycles abort initialization
> with the complete cycle path. Validation: new dependency-graph unit tests
> (including disabled nodes and self-cycles), Core/Common CTest, CUDA backend
> 12 cases / 238 assertions, ordinal-sorted IPC and AL-IPC system manifests,
> and full simulation suite 95 cases / 14212 assertions.

> **Single-source scene configuration contract (2026-08-30,
> `refactor-main`)**: `scene_default_config.cpp` now declares each key once,
> including its typed default and schema metadata. `Scene::default_config()`
> and `Scene::config_schema()` are derived from that same contract, eliminating
> the previous parallel default/metadata lists. The normalized public schema is
> byte-for-byte equivalent to the pre-refactor schema. Validation: focused C++
> schema case 668 assertions, Core 36 cases / 988 assertions, and Python schema
> tests 5 passed.

> **CUDA internal component build (2026-08-30, `refactor-main`)**: the CUDA
> backend's 198 compiled sources are partitioned into seven primary domain
> components plus optional legacy collision. CMake uses internal OBJECT targets;
> XMake's matching manifest attaches sources directly to `uipc_backend_cuda` so
> its built-in device-link sees every RDC object. Runtime registration and ABI
> behavior remain one DLL. Configuration rejects missing or duplicate source
> ownership. Validation: clean Release build, CUDA backend 12 cases / 238
> assertions, and full simulation suite 95 cases / 14212 assertions. The XMake
> implementation detail was corrected after the later CI audit above.

> **Backend ABI handshake and artifact parity (2026-08-30,
> `refactor-main`)**: every backend now exports `uipc_query_module` in
> addition to init/create/destroy. Before initialization, Core validates the
> size-versioned ABI record, exact backend identity, and libuipc major/minor
> version, turning stale/mixed DLLs into an immediate diagnostic. CMake no
> longer changes a backend from MODULE to SHARED when tests are enabled;
> CMake and XMake both always produce the same runtime-loadable shared-library
> form. Both none/CUDA DLL export tables contain all four symbols. Validation:
> Core 36 cases / 988 assertions through the new loader path, plus CUDA backend
> 12 cases / 238 assertions.

> **Profile-guided contact/FEM assembly (2026-08-30, `refactor-main`)**:
> Nsight Systems identified the two fused simplex-contact assembly kernels,
> StableNeoHookean3D gradient/Hessian, and shell bending as the dominant raw
> assembly kernels. SNH no longer materializes dense `9x12 dF/dx` and `12x12`
> Hessian matrices: reusable FEM helpers project the energy gradient/Hessian
> directly through tetrahedron shape gradients into the four gradient vectors
> and ten upper-triangular `3x3` blocks. The SNH kernel's per-thread stack fell
> from 6440 to 1320 bytes and its profiled average from 1.795 to 1.047 ms
> (-41.7%). Case-88 `Assemble Subsystems` fell from 3.60 to 2.97 ms/Newton
> (-17.6%) and `Build Linear System` from 7.93 to 7.32 ms/Newton (-7.8%). Two
> clean 60-frame runs measured 156.0-157.0 ms mean and 173.1-173.9 ms median,
> versus 158.1/178.4 ms before this stage; iteration-count variation limits
> the wall-time gain, so the scoped/kernel measurements are primary.
>
> The simplex contact kernels retain one PT/EE/PE/PP launch but compile
> separate gradient-only and Hessian variants. Gradient-only resources are
> normal 106 registers/272-byte stack and friction 112/144, while full
> Hessian performance stays flat (combined profiler average about 4.12 to
> 4.08 ms). A tested per-contact-type split reduced static stack usage but
> serialized rare, individually expensive PT/EE Hessian threads; it regressed
> `Assemble Dytopo Effect` from 4.52 to 7.67 ms/Newton and was rejected.
> Validation: Release CUDA build; CUDA backend 12 cases / 238 assertions;
> focused contact/FEM/MAS/bending 10 cases / 1426 assertions; full simulation
> suite 95 cases / 14212 assertions; targeted compute-sanitizer memcheck 0
> errors and 0 leaked bytes (2 cases / 504 assertions).

> **Device-side line-search energy aggregation (2026-08-30,
> `refactor-main`)**: top-level ABD, FEM, and DyTopo energy reporters now write
> their totals into contiguous device slots. `LineSearcher` performs one final
> CUB reduction into a separate output slot, then downloads all reporter totals
> and the aggregate with one contiguous D2H copy/synchronization. Per-reporter
> finite-value diagnostics and detailed reporting remain intact. ABD and FEM
> retain their existing component reductions but combine those device results
> with named one-thread kernels; DyTopo reduces directly into its assigned
> slot. On case 88 over 60 frames, initial-energy evaluation fell from 0.624 to
> 0.529 ms/call (-15.2%), trial-energy evaluation from 0.542 to 0.448 ms/call
> (-17.3%), and aggregate line search from 7.38 to 7.01 ms/Newton (-5.0%). Wall
> mean/median moved from 162.7/182.3 to 158.1/178.4 ms/frame. Validation: CUDA
> backend build, 11 CUDA test cases / 217 assertions, and the full simulation
> suite (95 cases / 14212 assertions).

> **Batched collision-count readback (2026-08-30, `refactor-main`)**:
> the default `InfoStacklessBVH` exposes launch-only detect/query operations
> plus an explicit result-finalization step. The simplex trajectory filter now
> launches all active PP/PE/PT/EE broad-phase queries, gathers their four device
> counters with one tiny kernel, and performs one contiguous D2H copy/sync.
> Overflow queues retain required-based growth and are the only queries rerun.
> The four CUB selection counts are likewise stored contiguously and downloaded
> once. Thus a fully populated detect/filter cycle uses two count readbacks
> instead of eight; synchronous BVH callers keep their original API. On case 88
> after the discard-growth change, clean-run trajectory detection moved from
> 5.07 to 5.01 ms/Newton and aggregate DCD from 4.67 to 4.61 ms/detect; wall
> mean/median moved 163.7/183.2 to 162.7/182.3 ms but remains within normal
> contact-stage variance. Validation: CUDA backend build, 11 CUDA test cases /
> 213 assertions, and the full simulation suite (95 cases / 14212 assertions).

> **Required-based CUDA output growth (2026-08-30, `refactor-main`)**:
> `cuda_tool::DeviceVector` now distinguishes value-initialized `resize()`
> from `resize_discard()` / `resize_preserve()` and exact or amortized reserve
> operations. Discard growth allocates 150% of the latest requirement, does
> not copy stale contents, and does not initialize ranges that a following
> kernel or CUB primitive completely regenerates. Existing subsystem-specific
> 1.1x/1.5x policies remain in force through exact `reserve_discard()` calls.
> The migration covers matrix-converter scratch, global/DyTopo triplets,
> line-search energy arrays, active-set scratch, and collision candidate/TOI
> buffers; state vectors and buffers with an initialization contract retain
> normal `resize()`. Case 88, 60 frames on RTX 5090, reduced `Scan and
> Allocate` from 80.8 ms total to 38.0-65.5 ms and the two `Compute Energy`
> scopes from 667.1 ms to 390.2-415.8 ms across clean runs. Wall time remains
> contact-sensitive (163.7-171.0 ms mean versus a 169.8 ms baseline), so the
> scoped timers are the reliable result. Unchanged runs diverge at atomic
> roundoff scale and reach about 0.59 mm by frame 60, matching the observed
> baseline-to-change envelope. Validation: CUDA backend build, 11 CUDA test
> cases / 213 assertions, full simulation suite (95 cases / 14212 assertions),
> and compute-sanitizer memcheck (0 errors, 0 leaked bytes).

> **CUB completion and active sparse-format clarification (2026-08-25,
> `refactor-main`)**: the legacy `stackless_bvh` and
> `info_stackless_bvh_v0` builders now use CUB radix sort and exclusive scan,
> with named initialization kernels and separate persistent sort outputs. MAS
> hierarchy prefix scans also moved from Thrust to `cuda_tool::DeviceScan`.
> All of these calls reuse the existing persistent per-stream CUB scratch cache;
> no Newton-iteration path allocates temporary CUB storage per call. Unused
> Thrust compatibility iterators/includes were removed from the CUDA backend.
> The active linear solve remains symmetric 3x3 **BCOO**, not BSR: triplets are
> canonicalized/reduced into `bcoo_A`, and both PCG SpMV paths consume it. BSR
> currently exists only as a container/converter option. Validation on RTX 5090:
> CUDA backend build/link; legacy/default/V0 BVH tests (38 assertions); all ten MAS
> simulation cases (275 assertions); full simulation suite (95 cases / 14212
> assertions); and MAS soft-stitch regression (4 assertions). The monolithic
> build reached the already-linked Python extension, then its local post-build
> package-uninstall step was denied access to the user-level `uipc.exe`; this is
> outside the compiled targets and is not a source/link failure.

> **Wheel CI path filtering (2026-08-25, `refactor-main`)**: prose/docs-only
> pull requests, including `agent_docs/`-only maintenance, no longer start the
> native builds or ten-wheel Python matrix. Release, code-bearing push/PR, and
> manual publication triggers remain unchanged.

> **Single-receiver DyTopo assembly fast path (2026-08-25,
> `refactor-main`)**: pure FEM or pure ABD scenes whose only diagonal
> `DyTopoEffectReceiver` owns the dynamic vertex prefix now forward the raw
> contact/inter-primitive doublets and triplets directly to that receiver.
> The final `GlobalLinearSystem` matrix conversion already canonicalizes and
> reduces those entries, so the former intermediate sort/reduce plus
> classify/copy pass was redundant. Trailing non-DOF global vertices such as
> half planes are allowed; multi-receiver and ABD/FEM coupling scenes retain
> the original conversion/distribution path. On RTX 5090, the clean case-88
> 60-frame run reduced `Compute DyTopo Effect` from 2014.6 ms / 330 Newton
> iterations (6.11 ms/iteration) to 1576.6 ms / 331 (4.76 ms/iteration,
> -22.0%). `Convert To BCOO` stayed effectively flat at 1.85 versus 1.86
> ms/iteration, while wall mean/median moved 175.6/196.4 to 165.5/184.2
> ms/frame. Maximum trajectory displacement versus the CUB baseline was
> 0.55 mm, below the 1.35-1.84 mm unchanged-run variability already measured.
> Pure ABD, pure FEM, and mixed ABD/FEM focused tests pass (736 assertions),
> as do the full simulation suite (95 cases / 14212 assertions) and the MAS
> soft-stitch regression (4 assertions).

> **CUB BVH build hot-path optimization (2026-08-25, `refactor-main`)**:
> the default `info_stackless_bvh` broad phase no longer uses Thrust. Morton
> pair sorting uses `cuda_tool::DeviceRadixSort`, internal-node offsets use
> `DeviceScan`, and identity generation plus all required build-state resets
> are fused into one named initialization kernel. The CUB wrappers reuse their
> persistent per-stream scratch workspace and grow it only when capacity is
> insufficient. Scene-AABB, leaf-LCA, and depth initialization now precede
> their parallel reductions/builds, removing the former cross-block reset
> races. On RTX 5090, case 88 over the same 60-frame phase reduced trajectory
> detection from 7.98 to 4.98-5.00 ms/Newton (-37.5%) and aggregate DCD from
> 7.35 to 4.59-4.60 ms/detect (-37.5%); two runs measured 175.6-177.1
> ms/frame versus 208.1 ms before (-14.9% to -15.6%). The full simulation
> suite passes (95 cases / 14212 assertions), as does the dedicated MAS
> soft-stitch regression (4 assertions).

> **MAS assembly hot-path optimization (2026-08-25, `refactor-main`)**:
> the mesh partition and multi-level MAS hierarchy are now built once during
> engine initialization instead of being restored and rebuilt on every Newton
> iteration. Hessian scatter traverses the BCOO arrays directly, removing the
> per-iteration identity-index allocation/fill/read, and the 48x48 Gauss-Jordan
> kernel no longer executes a redundant block-wide barrier between independent
> row updates. On RTX 5090, sample 88 over the same 60-frame phase and 333
> Newton iterations reduced `Assemble Preconditioner` from 1149.9 ms to
> 834.4 ms (-27.4%, 3.45 to 2.51 ms/Newton); wall mean moved from 213.3 to
> 208.1 ms/frame, with the remaining wall variance dominated by contact stages.
> The full simulation suite passes (95 cases / 14212 assertions); this includes
> all ten MAS sim cases (275 assertions). The dedicated MAS soft-stitch
> regression also passes (4 assertions), as do the runtime-check CUDA build and
> clang-format-18 gate.

> **Repository contracts and source hygiene (2026-08-25,
> `refactor-main`)**: all external GitHub Actions now use reviewed full commit
> SHAs, and the vcpkg action/container revision matches the project's registry
> baseline. A dedicated repository-contracts workflow rejects mutable action
> refs, zero-byte source files, and drift between exported constitution classes,
> pybind classes, and binding initializer registration. Thirteen empty CUDA/C++
> scaffold translation units were removed; the two public zero-byte headers are
> now documented compatibility includes. The full docs helper finds a standard
> Windows Doxygen install even when it is absent from `PATH`, and the local
> preview guide distinguishes deployable API builds from prose-only previews.
> Local validation: full C++/CUDA pybind build; 36 core cases / 988 assertions;
> 79 portable Python tests; 48 non-interactive CUDA tests; 5 repository-contract
> tests; clang-format-18; release-policy/parity/pin/zero-byte checks; and a full
> Doxygen + MkDoxy site containing the `Engine::frame_stats()` API page.

> **Structured solver observability (2026-08-25, `refactor-main`)**:
> the backend-neutral `Engine::frame_stats()` API (also Python) returns `{}` by
> default; CUDA schema v1 reports latest-frame pipeline, completion/convergence,
> Newton and cumulative line-search/linear-solver counts, iteration-limit hits,
> and final line-search/CCD/CFL factors. Python also now exposes the existing
> C++ `Engine::to_json()`, `Engine::status()`, and status `clear()`. Profile runs
> persist one `frame_stats.json` entry per measured frame, and performance
> baselines consume the structured counters as diagnostics instead of scraping
> log messages. Focused none/IPC/AL tests cover optional-backend behavior,
> status access, schema values, and profile persistence.

> **Reproducible performance gates (2026-08-25, `refactor-main`)**:
> `uipc.profile` now excludes warmup/recovery from reported wall time and drains
> warmup Timer data before the first measured frame. Saved artifacts use a
> versioned schema and include the exact phase plan plus runner/build
> compatibility facts. `python -m uipc benchmark baseline/check` (also Python
> APIs) creates deterministic JSON baselines and returns a structured,
> CI-friendly nonzero regression result for wall average and Timer median/p95;
> Newton counts are diagnostic. Checks reject missing scenes, frame/phase drift,
> and environment mismatch unless explicitly relaxed. Focused validation covers
> pass/fail thresholds, overrides, deterministic output, schema rejection,
> environment matching, CLI output, and CUDA/none session integration.

> **IPC/AL frame-lifecycle parity (2026-08-25, `refactor-main`)**: both CUDA
> advance paths now share the ordered external-force lifecycle (clear old
> device buffers, run animation, consume current forces) before DOF prediction.
> AL-IPC no longer enables the process-global Timer or prints a merged report on
> every frame, and its adaptive-mu/CFL stages plus Newton/line-search indices are
> timed/tracked consistently. A parameterized FEM regression exercises the same
> two-frame apply/clear sequence under `ipc` and `al-ipc` and rejects implicit
> Timer output. Local validation: 2 focused cases, 48 non-interactive CUDA
> tests, and 64 portable Python tests pass.

> **Portable local docs builder (2026-08-25, `refactor-main`)**:
> `scripts/build_docs.py` now runs MkDocs through the active Python interpreter
> (`sys.executable -m mkdocs`) instead of assuming a `mkdocs` launcher is on
> `PATH`. This fixes full local builds from ordinary Windows virtual
> environments while preserving the production `mkdocs-with-api.yaml` path.

> **Validated scene-configuration contract (2026-08-25,
> `refactor-main`)**: `Scene::config_schema()` / `Scene.config_schema()` exposes
> all 46 registered keys with defaults, types, units, hard constraints,
> lifecycle/status, descriptions, and source consumers; `python -m uipc
> config-schema [key]` makes it available to tools and agents. Construction and
> `World::init(scene)` validate the contract, including mutable edits and
> cross-field kappa/Newton ordering. The unimplemented
> `newton/use_adaptive_tol` is constrained to `0`, `sanity_check/mode=quiet` is
> now an explicit choice, and non-empty Contact/Subscene extension configs are
> rejected instead of silently ignored. `SceneGUI` consumes the same schema and
> renders reserved entries read-only.

> **Python/CUDA compatibility policy and doctor (2026-08-25,
> `refactor-main`)**: the next wheel matrix covers CPython 3.10–3.14; the
> immutable 0.0.26 release still stops at 3.13. Published builds now target
> 75/80/86/89 SASS plus compute-89 PTX rather than only architecture 89, and
> embed ABI/toolkit/architecture metadata in `build_info()`. The packaged
> `compatibility.json` is checked against both pyprojects and CI. `python -m
> uipc doctor` diagnoses Python ABI, the self-contained CUDA runtime boundary,
> backend dynamic loading, NVIDIA driver/GPU architecture, and optionally a real
> CUDA engine construction via `--probe-cuda`.
> Pytest now defaults to the portable `not example and not cuda` suite; GPU and
> interactive cases have explicit markers, module-stubbing tests restore global
> import state, and every cibuildwheel job executes the portable suite against
> the installed artifact. Local validation covered 59 portable tests and 47
> non-interactive CUDA tests on CPython 3.14.

> **Executable CI and release gates (2026-08-25, `refactor-main`)**: CMake now
> registers all C++ tests with CTest and labels the `common`/`core`/`geometry`
> CPU suite for fast CI execution; the CMake and XMake workflows execute those
> binaries on pushes, pull requests, and manual runs instead of stopping after a
> successful compile. Every built wheel is installed and smoke-tested, TestPyPI
> must expose the complete interpreter/platform matrix and pass an exact-version
> install before formal publication, and PyPI is verified the same way afterward.
> The hosted smoke test uses the no-GPU backend; CUDA-runtime validation remains a
> distinct compatibility gate rather than an inferred result of `import uipc`.

> **Documentation navigation hygiene (2026-08-25, `refactor-main`)**: XMake and
> deterministic-mode guides are now reachable from the site navigation, directory
> links point at explicit index pages, and the RMR/SpreadSheetIO links use valid
> MkDocs source paths. Prose-only builds now leave only expected generated-API
> warnings when Doxygen output is absent. Workflow-dispatch run `32758550867`
> completed the UID, Doxygen, MkDoxy, and MkDocs stages successfully without
> deploying; the docs workflow then moved checkout/setup-python to their current
> Node 24-based v7 majors.

> **XMake parity and deterministic packaging (2026-08-25, `refactor-main`)**:
> stale GUI/torch/RPC configuration was removed, ccache is explicitly disabled,
> and optional OpenUSD/OpenVDB targets now mirror CMake. The pybind target enables
> USD consistently and performs one synchronous source copy plus explicit
> extension/runtime-library copies, eliminating duplicate detached copy races.
> The XMake user guide and build-agent notes describe the current switches.

> **Python packaging and helper parity (2026-08-25, `refactor-main`)**: release
> and development metadata now both include matplotlib, require pytest 9.0.3+ for
> development, and state the prebuilt-wheel CUDA 12.8 runtime requirement. The
> Warp empty-strides fallback calls the real element-size helper. Python now
> exposes the C++ `Scene.Objects.created_count()` ID upper bound, and
> `assets.strip_constitutions` uses it to process sparse-ID scenes. Focused Python
> tests cover the Warp fallback and sparse create/delete lifecycle.

> **Complete UID documentation generation (2026-08-25, `refactor-main`)**:
> `scripts/gen_uid_doc.py` now parses designated initializers and statement-based
> `UIDInfo` assignments, restoring constitution UIDs 15, 17, 31, and 32 to the
> generated specification. A dependency-free unit test covers both forms and the
> formerly missing real registrations. Documentation CI now runs the test plus
> the generator's `--check`, and UID source changes trigger that workflow. The
> docs build wrapper also reports the requested output path and propagates
> MkDocs/Doxygen failures instead of returning success after a failed build.

> **Runtime lifecycle parity (2026-08-25, `refactor-main`)**: an implicit
> synchronization performed by `World::retrieve()` now marks the engine
> synchronized, so repeated retrieves do not issue redundant backend syncs until
> another `advance()`. The `none` backend now enters the Scene pending phase at
> initialization and settles pending geometry creation/destruction on each
> advance, matching the frontend lifecycle contract without pretending to run a
> physical simulation. Core tests cover both behaviors.

> **Evolving-only atlas projection (2026-08-25, `refactor-main`)**:
> `GeometryAtlas::create(..., true)` now filters named collections and every
> collection inside a geometry to slots marked `is_evolving`, preserving
> collection topology and row counts for baseline-dependent streaming. Evolving
> markers now survive attribute clones and atlas JSON round trips (legacy JSON
> defaults to `false`), and Python exposes the same optional argument. Core tests
> cover strict filtering, clone behavior, dimension preservation, and serialized
> round trips; the modified pybind translation unit also compiles independently.

> **Scene lifecycle hardening (2026-08-25, `refactor-main`)**: snapshot commits
> now replicate current/rest geometry independently, explicit slot removals,
> sparse IDs and next-ID state, contact/subscene topology, and the contact default
> model's user-set state. Attribute commits and full attribute serialization also
> preserve row counts when a collection has no columns. Decoders validate
> duplicate IDs, current/rest topology, commit/removal overlap, and invalid atlas
> entries while retaining legacy-field fallbacks. Focused core tests cover
> full-snapshot and commit-JSON round trips, different current/rest mutations,
> deletion plus exact-ID insertion, allocation gaps, table state, and empty
> nonzero attribute collections.

> **Post-merge update (2026-08-23)**: everything below (the whole
> refactor-main line: muda→cuda_tool, raw kernels, Stiff-GIPC alignment,
> cloth stiffness model, kappa policy, hygiene batch) was merged to `main`
> via **PR #468** (merge commit `9bf45950`, 2026-08-22). CI on the PR was
> green at merge time. `refactor-main` is done; new work branches off
> `main`. Sections below are pre-merge history — still accurate as records
> of *why* things are the way they are. For open issues and plans see
> **`09-known-issues-and-roadmap.md`**; for the collected pitfalls see
> **`08-pitfalls-and-debugging.md`**.
>
> Post-merge events:
> - CI repair round on the PR (all pushed, all green): mass clang-format
>   (327 files, `72cf876b`); tinygltf v2.9.6 stale hash → overlay port
>   `ports/tinygltf` wired via `overlay-ports` in the generated
>   `vcpkg-configuration.json` (`bcd04cc2`; env-var wiring does NOT reach
>   cibuildwheel's inner vcpkg install — see doc 07/08); xmake pins
>   `octree v2.5` (`f4230c31`) and `tinygltf <3` (`557f7017`) against
>   upstream v3 layout drift.
> - PR #469 merged: pytest >=9.0.3 (dependabot tmpdir CVE), `uv.lock`
>   regenerated with uv 0.12.5 (large diff — it backfilled missing entries).
> - Samples repo (separate, `spiriMirror/libuipc-samples`): added case
>   **87_robot_hand** (URDF hand + ABD cube, manual GUI posing; ported from
>   references/Robotics-Libuipc and rewritten; scripted auto-grasp removed
>   at user request) and case **88_stiff_gipc_benchmark** (the old
>   `Stiff-GIPC-benchmark.py` moved there with a GUI; `--headless [N]`
>   keeps the benchmark loop; parameters untouched).
> - External PR #461 (EmbeddedCollisionMesh) reviewed — verdict and the
>   three must-fix bugs recorded in doc 09.
> - **FusedPCG CUDA-graph block replay (2026-08-23)**: `check_interval`-sized
>   iteration blocks are captured once per (buffer-set, N, triplet-count) and
>   replayed as single graph launches; case2 250-frame benchmark 301 →
>   233 ms/frame (~1.29×). Config `linear_system/use_cuda_graph` (default 1);
>   `linear_system/check_interval` is now a registered key (default 5 — it was
>   unregistered and silently dropped before). Details and traps in doc 05/08.
>   Notable bugs found during the work: rz_tol async-upload vs graph-launch
>   stream race (sync upload now), triplet_count missing from the graph key,
>   and Timer objects created during stream capture deterministically crashing
>   the single-process suite binary (0xC0000409) — capture path creates no
>   Timer objects anymore (plain path keeps them for case 59's SpMV counts).
>   The al-ipc pipeline is gated off graph replay for now (crash observed only
>   in the C++ suite binary's al-ipc section; python repro passes — root cause
>   open, see doc 09).
> - **MAS preconditioner line (2026-08-23, PRs #473/#474)**: ported from
>   Stiff-GIPC and then hardened. The apply path is stream-plumbed so MAS
>   scenes join the PCG CUDA graph. Activation is now all-or-nothing via
>   scene config `linear_system/fem_preconditioner = "mas"` (default
>   "diag"): every non-Empty FEM geometry is auto-partitioned internally
>   (fixed cluster size 16 = BANKSIZE) on a private clone; the python
>   `mesh_partition` export was REMOVED (partial coverage measured
>   net-negative — the coverage-rule table is in doc 09). Sim cases 53-61/81
>   migrated to the switch. Escape hatch if MAS+graph ever misbehaves:
>   `linear_system/use_cuda_graph = 0`.
> - **Newton exit semantics split (2026-08-24)**: `newton/min_iter` is a pure
>   hard floor (default 0 = off); the semi-implicit beta start moved to
>   `newton/semi_implicit/K_min` (default 1 at that revision; superseded by
>   the 2026-09-01 default of 6 with semi-implicit termination enabled). Found
>   via the case-89 parity run: Stiff averages 2.55 Newton/frame while we forced
>   >=6. Case 88
>   429 -> 320 ms/frame from this alone.
> - **Perf rounds on case 88 (2026-08-24, now ~266 ms mean / ~299 ms
>   median)**: two-level warp->block reduction in `Spmv_rbk_sym_spmv_dot` /
>   `fused_dot` (same-address atomic storms eliminated); exact-distance DCD
>   leaf predicates (kills the 450k-candidate retry double-traversal).
>   Negative result: make_spd Cholesky early-out (register blow-up) — see
>   doc 08. Frame budget and next levers in doc 09.
> - **Samples repo**: 87 robot_hand, 88 case2 benchmark, 89 MAS parity
>   bunny (NO_MAS/NO_GRAPH env A/B), 90-93 = the remaining Stiff set_cases
>   1/4/5/6 (see doc 09 samples section for the mapping + asset notes).
> - **Documentation authentication-popup fix (2026-08-24)**: replacing
>   Bilibili iframes with links removed one popup source, but the deployed
>   site still loaded `polyfill.io` globally from both MkDocs configs. The
>   service now returns HTTP 401, causing a browser authentication dialog on
>   every page. The obsolete ES6 polyfill was removed; MathJax 3 remains
>   loaded from jsDelivr. The docs workflow's path filters were also fixed
>   to watch the repository's actual `mkdocs*.yaml` files (it previously
>   watched only the unused `.yml` suffix, so config-only fixes did not
>   deploy).
> - **Documentation demo separation (2026-08-24)**: the standalone docs home
>   Demos section, Gallery navigation entry, and `docs/gallery.md` were
>   removed so the project-wide showcase lives on the homepage. Focused clips
>   and well-framed poster screenshots remain allowed when they explain an API
>   result. Both MkDocs configurations expose `spiriMirror/libuipc` as the
>   global project repository link in the site header.
> - **Configuration documentation audit (2026-08-24)**: the scene-config
>   reference now enumerates all 46 unique keys registered by
>   `scene_default_config.cpp`, including defaults, SI units, selector values,
>   operational domains, relative-value precedence, lifecycle rules, and
>   the reserved `newton/use_adaptive_tol` key, whose nonzero values are now
>   rejected instead of silently ignored. Dedicated
>   Newton/linear-solver and contact/collision pages trace behavior through the
>   CUDA consumers and explain MAS, graph modes, adaptive kappa, pairwise
>   contact elements, and sanity checks. Navigation exposes all three pages.
> - **Source-backed scenario tutorials (2026-08-24)**: the tutorial landing
>   page now routes to scene assembly, pure ABD, pure volumetric FEM, cloth,
>   and rigid-soft/contact guides. Each physical guide contains complete
>   asset-free C++ and Python programs, parameter/state explanations, failure
>   checks, and links to the matching `libuipc-samples`, C++ regression cases,
>   public headers, and backend attribute consumers. The four Python programs
>   run against the current source build; sample `91_pinned_cloth --headless
>   1` also passes. MkDocs builds successfully and expands every code snippet;
>   strict mode is still blocked only by the pre-existing generated-API nav
>   entries when Doxygen output is absent.
> - **PyPI 0.0.26 CUDA-major runtime finding (2026-08-24)**: installation and
>   `import uipc` succeed, but the Windows wheel's CUDA backend directly
>   imports `cublas64_12.dll`. A machine with only CUDA 13.2 therefore fails at
>   `Engine("cuda", ...)` even though the driver is new enough. The source
>   build on that machine imports `cublas64_13.dll` and works. User-facing
>   install docs and package metadata now say the prebuilt wheel requires the
>   CUDA 12.8 runtime; CUDA 13 users must install 12.8 side-by-side or build
>   from source. A plain import is not an adequate release smoke test.
>
> Older header note (2026-08-20, pre-merge): the muda→cuda_tool migration
> is complete AND fully verified: all apps/tests pass, including the
> 95-case sim suite (2/2 runs, 14214 assertions — same count as the
> pre-migration baseline).
> Verify against the working tree before assuming anything beyond this file.

## TL;DR

- **All of this is merged to `main`** (PR #468, `9bf45950`). The CUDA
  backend no longer depends on muda in any form (no submodule, no vendored
  copy, no xmake package).
- All 273 lambda kernel launch sites were rewritten as named `__global__`
  functions with raw `<<<>>>` launches.
- **All tests green**: 6 fast binaries (common/core/geometry/sanity_check/
  backend_cuda/regression) + `uipc_test_sim_case.exe` full suite 95/95 cases,
  14214 assertions, run twice deterministically.
- (The "uncommitted fix batch" mentioned in older revisions was committed
  and merged as part of PR #468.)

## Commits (oldest → newest, on top of `74a5df62`)

```
ef87325c docs(agent_docs): record muda vendoring completion and fix stale references
b2aec545 feat(cuda_tool): complete primitives for muda replacement
8e3299af refactor(cuda): migrate backend from muda to cuda_tool
423be546 refactor(cuda): rewrite lambda kernels as named __global__ functions
cb9341c1 build: drop the vendored muda from cuda_tool and sync xmake
f6fd6bb3 refactor(cuda_tool): trim unused primitives and refresh agent docs
ee4bea1e refactor(cuda): convert the last lambda kernel and remove ParallelFor
2a8f78d7 refactor(cuda_tool): second trim of zero-reference helpers
```

## Fix commits on top of `2a8f78d7` (root-cause fix for the suite failure)

1. `fix(cuda_tool)` — `launch.h`: `best_block_dim` occupancy cache keyed by
   kernel function address (`std::unordered_map<const void*, int>`) instead
   of a `static thread_local int` per template instantiation (**ROOT-CAUSE
   FIX**, see below); `buffer.h`: `DeviceVector::resize(n)` value-initializes
   the grown tail (memset 0 for trivial types, `T{}` fill otherwise),
   matching thrust/muda resize semantics.
2. `test/build sync` — `apps/tests/backends/cuda/CMakeLists.txt`:
   `/Zc:preprocessor` (CUDA>=13 CCCL requires it),
   `--extended-lambda --expt-relaxed-constexpr` (test .cu use cuda_tool
   launch/dense math), nvcc diag-suppress list; 5 test .cu files gain
   global-scope `namespace cuda_tool = uipc::backend::cuda_tool;` alias
   fixes (`lbvh.cu` uses `copy_from` instead of rvalue copy-init); xmake
   parity (static check only, no local xmake):
   `apps/tests/backends/cuda/xmake.lua` gains the same three flags,
   `src/backends/cuda/xmake.lua` gains `-Xcompiler=/Zc:preprocessor`
   (public, windows block); agent_docs refreshed.

## The full-suite failure and its root cause (RESOLVED)

Symptom: after the migration, `uipc_test_sim_case.exe` (95 cases, one
process) failed deterministically 3/3 at case `36_no_surf_but_contact_on`
frame 17: `Line Search Exits with Max Iteration: 8`. Case 36 run in an
isolated process passed 6/6. Pre-migration baseline (`ef87325c`) full suite
passed 3/3 (14214 assertions).

Root cause: `best_block_dim(Kernel kernel)` cached the occupancy result in a
`static thread_local int` **per template instantiation**, i.e. per function
*pointer type*. Distinct kernels with identical signatures share one
pointer type, so whichever same-signature kernel was queried first set the
block size for all of them. muda's equivalent cache was keyed per unique
lambda type (= per call site), so no cross-kernel pollution existed. In the
full-suite process, 35+ engines ran before case 36 and poisoned shared cache
entries; wrong block sizes on atomic-accumulation assembly kernels perturbed
float atomic-reduction order, shifting the FP trajectory enough to push the
frame-17 line search over the iteration limit. In isolation fewer collisions
occurred, so the perturbation stayed below the threshold.

Fix: cache keyed by kernel address. Verified: full suite 2/2 green with the
same assertion count as baseline; case-36 isolation residual series matches
baseline except residual ULP-level noise (expected: two different binaries
have different address layouts → different atomic arrival order).

Ruled out during the hunt (do not re-open): compile-flag drift (none —
`git diff` of CMakeLists), wrapper-vs-raw occupancy difference (probe with
the verbatim `abd_linear_subsystem_assemble_reporters_k2` body: 256 == 256,
see `output/probe_occupancy2.cu`), eigen port drift (normalized diff vs muda
ext/eigen: macro/namespace renames only, math bodies identical), thrust
calls in bvh (verbatim from baseline), buffer fill/copy block sizes
(per-element ops, no FP effect), stream defaults, cub/cublas call shapes.

## What was done (migration recap)

1. **cuda_tool primitive completion** (`b2aec545`) — stream/view/view_nd/
   launch/buffer/cub/linear_system(+views)/debug/logger/atomic + the eigen
   subdirectory (ported verbatim from muda ext/eigen, numerically
   bit-identical). The `UIPC_KERNEL_*` macro family is enabled together
   with `uipc::RUNTIME_CHECK`.
2. **Mechanical migration** (`8e3299af`) — 280 files `muda::`→`cuda_tool::`,
   umbrella header changed to `cuda_tool/cuda_tool.h`, macros renamed,
   `.name("...")` labels deleted.
3. **Kernel rewrite** (`423be546` + `ee4bea1e`) — 273 lambda kernels →
   named `__global__` (anonymous namespace, bodies verbatim, captures →
   parameters). Launches use `cuda_tool::best_grid_dim/best_block_dim` to
   keep the same occupancy choice; the `ParallelFor` mechanism was
   subsequently removed from cuda_tool (business lambda kernels = 0).
4. **Delete vendored muda + build-system sync** (`cb9341c1`) — deleted
   `cuda_tool/muda/` (288 files) and `muda_compat.h`; CMake dropped the
   MUDA_* macros; xmake dropped `add_requires/add_packages("muda")`.
5. **Two rounds of cuda_tool trimming** (`f6fd6bb3`, `2a8f78d7`) — deleted
   zero-reference primitives and helpers.

- **Performance-investigation lessons (6_wrecking_balls, 619a5412 baseline
  A/B)**:
  1. The real regression root cause: the first-version cuda_tool cub
     wrappers did a per-call `cudaMalloc/cudaFree` of temporary storage
     (~10-100µs each plus an implicit device sync; dozens of cub calls per
     frame) → fixed as a stream-level workspace cache (`cub.h` details);
     frame time 73.9→66.7ms, and the median of the first 8 frames
     (pre-contact) is level with the baseline.
  2. Investigation pitfall a: **build contention thoroughly pollutes
     timing** (sanity_check was once misjudged as the ~112ms/frame culprit;
     on an idle machine it is only ~2-5ms/frame) — timing experiments must
     run on an idle machine.
  3. Investigation pitfall b: after contact activates, frame-to-frame
     phase comparison is meaningless — the two binaries have different
     block sizes → ULP-level differences → trajectory divergence, so later
     frames are no longer in the same physical state.
  4. The ~65ms/frame floor of this scene consists of: dump ~5-10ms +
     per-frame Timer.report/log printing ~10-30ms + sanity_check ~2-5ms +
     the real pipeline 20-45ms (the baseline is the same) — the "feels
     slow" is mostly inherent structure, not a regression.

## Scene-diagonal adaptive parameters (Stiff-GIPC alignment, after `7cf19f21`)

- At `GlobalVertexManager` init the rest bounding-box diagonal
  `scene_diagonal()` is computed (printed to the log; measured 28.93 in the
  wrecking-ball scene, exactly matching Stiff-GIPC's
  √bboxDiagSize2=√834.9).
- New config (default 0 = off, backward compatible):
  - `contact/d_hat_relative`: when >0, d_hat = relative value × diagonal
    (Stiff-GIPC's relative_dhat convention; its dHat stores the square, so
    the original text is rel²·diag²).
  - `newton/velocity_tol_relative`: when >0, the Newton exit threshold =
    relative value × diagonal × dt (MaxTranslationChecker, Stiff-GIPC's
    threshold×diag×dt convention).
- 6_wrecking_balls is now fully parameter-aligned with set_case3 (mu 0.2,
  per-object densities 1000/7680, tol_rate 1e-4, relative d_hat/tol).
  **Note: the previously recorded "advance 26-28ms/frame ≈ Stiff 1.1×" was
  a mismeasurement taken pre-contact / with a polluted dll; the true
  contact-phase behavior once collapsed to 3-18s/frame — root cause and fix
  in the next section.**
- Performance-investigation record: the cub workspace fix + the two
  measurement pitfalls (build contention, trajectory divergence) are in doc
  05 and the performance section above.

## Stiff-GIPC barrier alignment: the log² hard barrier (after `4294c1d5`)

- **Root-cause chain (wrecking ball once at 3-18s/frame, Newton hitting the
  1024 cap)**:
  1. In Stiff-GIPC's incremental potential the barrier term is **not
     multiplied by dt²** (GIPC.cu `computeEnergy`); libuipc's barrier
     coefficient is `kt2 = κ·dt²` → at the same nominal κ the barrier
     strength differs by 1/dt² (10⁴× at dt=0.01). The "set κ=1e4 on both
     sides" alignment was therefore a fake alignment.
  2. Stiff-GIPC's barrier is a RANK=2 log² hard barrier
     `κ(D-d̂²)²ln²(D/d̂²)` (GIPC.cu:28 `#define RANK 2`; `_d_EE` returns
     squared distance); in the deep-penetration regime its force is
     ~2|ln(D/d̂²)|× stronger than the classic log barrier, so under the
     same load the equilibrium gap is ~10× wider, the equilibrium curvature
     ~10× lower, and Newton converges in 3-5 iterations/frame. libuipc's
     classic log barrier (with κ_eff 10⁴× weaker) sinks deep into the
     ill-conditioned D→0 region, CCD crushes α to ~1e-3, and Newton crawls
     linearly for hundreds of iterations.
- **Changes**:
  - `sym/codim_ipc_contact.inl` regenerated (generator notebook
    `scripts/symbol_calculation/codim_ipc_contact.ipynb` updated in sync):
    ξ==0 (volume/ground contact) takes the newly generated log² barrier
    `KappaBarrierLog2` family; ξ>0 (codim shells) keeps the classic
    thickness barrier. The public entry points `KappaBarrier`/
    `dKappaBarrierdD`/`ddKappaBarrierddD` became runtime dispatchers —
    PT/EE/PE/PP, the ground half-plane, and friction normal_force all
    follow automatically, with no call-site changes.
  - samples 6_wrecking_balls: κ=1e8 (equivalent conversion: libuipc κ =
    Stiff Kappa / dt² = 1e4/1e-4).
- **Verification**: the probe converged in all 120 frames; Newton mean
  4.54/max 22 (Stiff 3.44/7); min_alpha 0.15-0.72 (previously crawling at
  ~1e-3); frame average 132ms (was 3-18s); the free-swing segment (f0-80)
  trajectory, after translation-aligning with Stiff, differs by <3mm; the
  full suite is green: 95 cases / 14214 assertions.
- **This section's "remaining gap" was superseded by a later fix**: the PCG
  spikes / pressing-frame crawl recorded at the time were rooted in
  d_hat_relative not being propagated (see "biggest hidden bug" in the next
  section) and disappeared after the fix. For the final comparison basis
  see the **second correction** in the "⚠ comparison-basis correction"
  section (clean-run data).
- **Measurement-pitfall memo**:
  1. post-build syncs the dlls only when pyuipc is relinked — if you only
     change backend .cu files, the `uipc_backend_cuda.dll` in site-packages
     is not updated; you must sync manually:
     `cp build/Release/bin/uipc_*.dll build/python/src/uipc/_native/` and
     `.../site-packages/uipc/_native/`.
  2. pyuipc's `Scene(config)` copies the config dict by value; modifying
     the python-side dict after construction silently has no effect — all
     config keys must be set before `Scene(config)`.
  3. The Stiff-GIPC reference copy block-buffers printf when stdout is
     redirected, so a timeout kill loses logs — an instrumented printf must
     be followed by `fflush(stdout)`.

## Second alignment round (friction smoothing + CFL semantics, after `81e52fce`)

- **`contact/eps_velocity_relative`** (new config, default 0=off): when >0,
  the friction C1 smoothing threshold eps_velocity = relative value ×
  scene_diagonal (Stiff-GIPC convention: its per-step slip threshold is
  sqrt(fDhat)·dt = 1e-2·diag·dt). libuipc's original default was an
  absolute 0.01 m/s, which in this scene makes the friction Hessian
  curvature ~840× stiffer than Stiff's. Implemented in
  `global_contact_manager.cu` Impl::init (same pattern as d_hat_relative),
  default key registered in `scene_default_config.cpp`. 6_wrecking_balls
  and the probe are set to 1e-2. **Real effect (measured after the
  registration fix): Newton mean 4.46→3.70 (Stiff 3.44 — essentially
  aligned), sum_pcg 349→250, frame average 129.9→114.5ms.**
- **Config-key lesson**: scene config only honors keys registered in
  `scene_default_config.cpp`; an unregistered key pushed from python is
  **silently dropped** (`find` returns nullptr and the default branch
  runs). The first version of eps_velocity_relative was never registered,
  wasting a whole alignment-experiment run — a new key must have its
  default registered at the same time, and its taking effect must be
  confirmed via the log line ("Contact eps_velocity (relative): ...").
  **→ Now fixed at the root (see below)**: `from_config_json` now
  recursively checks the user json first and directly throws a guided error
  for unregistered keys ("typo, or a missing default registration"). That
  check immediately unearthed two historical typos/dead keys: ①
  `sanity_check/method` in `apps/tests/core/engine.cpp` (the schema has
  `mode`; the real intent was `enable=0`); ② `contact/al-ipc/mu_scale` in
  `apps/tests/sim_case/11_abd_ramp_sliding.cpp` (that key was split into
  `mu_scale_fem`/`mu_scale_abd`; the old key had been silently dropped all
  along). Note: python already raises KeyError for unknown top-level keys;
  the new check covers nested typos under a valid prefix (such as
  `contact/dhat_typo`).
- **d_hat_relative propagation fix (biggest hidden bug)**: the first
  version only changed `GlobalContactManager`'s scalar d_hat (used by
  CFL/logging); **the per-vertex `d_hats` buffer (what the filter/contact
  kernels actually read) was still filled with the absolute default
  `contact/d_hat`=0.01**. wrecking ball therefore ran at d_hat=0.01 (it
  should be 0.0289) — a too-small d_hat leaves contact inactive at shallow
  gaps, vertices plunge into deep gaps before the barrier stops them, and
  the system becomes ill-conditioned (this was the true source of the
  earlier PCG spikes of 150-565). Fix: at the end of
  `GlobalVertexManager::Impl::init`, propagate `d_hat_relative ×
  scene_diagonal` into the per-vertex buffer (compare-and-set only
  overwrites entries holding the absolute default; per-geometry meta d_hat
  is preserved; `global_vertex_manager.{h,cu}`). Post-fix wrecking ball:
  **Newton mean 1.88/max 6 (Stiff 3.44/7), sum_pcg 56 (Stiff 53), min_alpha
  constant 1.0, frame average 73.0ms (Stiff clean run 42.8ms/frame
  GPU-timed, ~1.7×)**; the set_case2 ported scene's self-contact pair count
  collapsed from 450k (bogus) to ~3.7k (same magnitude as Stiff's 3.5k).
- **CFL semantics correction**: the original implementation only counted
  displacements of "activated contact vertices" (the
  `vert_is_active_contact` mask) — measured zero triggers over 120
  wrecking-ball frames, because vertices hurtling toward contact at high
  speed happen to be outside the mask and can plunge into deep gaps in one
  step. Changed to the Stiff-GIPC design: max|dx| covers **all surface
  vertices** (`GlobalSimplicialSurfaceManager::surf_vertices`, falling back
  to all vertices when there is no surface manager); and in the
  `advance_ipc.cu` line search it is applied only when CCD hits
  (ccd_alpha<1) — avoiding needlessly capping free-flight frames. In this
  scene it still never triggers in practice (vertex steps within contact
  iterations are mostly under 5cm) — this is a semantic alignment whose
  value lies in high-speed-impact scenes.
- **Current state of the remaining frame-time gap**: after the d_hat fix,
  wrecking ball averages 73.0ms/frame, about **1.7×** Stiff's clean-run
  42.8ms/frame (GPU event timing, frames 2-120). The early "PCG spikes"
  (150-565 iterations/solve) were in fact caused by small-d_hat deep gaps
  and vanished with the d_hat fix (sum_pcg now 56 ≈ Stiff 53).
  Verification means on record: linear-system dump (config
  `extras/debug/dump_linear_system=1`) + scipy recomputation.

## Stiff-GIPC set_case2 ported benchmark (now samples case `88_stiff_gipc_benchmark`; formerly `examples/Stiff-GIPC-benchmark.py`)

- Scene: ABD bunny (scale 0.2, y+0.5, ρ1000, ABD κ=1e8) + FEM bunny (same
  mesh, y-0.65, SNH E=1e4/ν0.49/ρ1000) + cloth (cloth_high.obj 4225
  vertices, E=5e4/ν0.49/ρ200/t=1e-3/strain_rate=100, bending value-matched
  to E=5e7→κ_b=5.48e-3) + ground y=-1; μ=0.2, κ=1e8 (=Stiff 1e4/dt²),
  dt=0.01, g=-9.8, d_hat_relative=1e-3, velocity_tol_relative=1e-2,
  eps_velocity_relative=1e-2, tol_rate=1e-4, **semi_implicit enabled
  (Kmin=6, beta_tol=1e-2 — Stiff's beta early-exit design: hard stacking
  frames exit capped at ~6 Newton iterations by design)**.
- Mesh assets: bunny2.msh was converted to standard Gmsh 2.2 (the original
  file is Stiff MeshProcess's nonstandard 7-field variant, unreadable by
  libigl; the tetrahedron content is bit-identical) and copied into samples
  assets (tetmesh/bunny2.msh + trimesh/cloth_high.obj).
- **Results (250 frames)**: libuipc averages 301ms/frame (stacking phase
  steady at ~310ms, no spikes), Newton capped at ~6 (=Kmin); Stiff clean
  run on the same scene: **142.8ms/frame** (GPU event timing, 447-frame
  average; frames 2-250 is 171.8ms), Newton mean ~2.4 (under that counting
  basis). So the case2 gap is about **1.8-2.1×** (not the previously
  miscomputed 5× — that came from misaligned frame grouping when
  recomputing marginal quantities on an instrumented run). The remaining
  gap is per-iteration throughput: this scene's dense bunny surface (6mm
  spacing after 0.2 scaling) makes each detect's AABB candidates ~450k
  (only ~3-8k active after distance filtering) — the separation of
  candidate generation from activation filtering is structural overhead,
  and fused/distance-aware queries are the follow-up optimization lead;
  there is also a uniform gap in PCG iteration count and kernel throughput
  (dissected in the next section).
- Note: cloth contact in libuipc takes the thickness-offset barrier (the
  ξ=1e-3>0 branch); since 238df28e the barrier shape is unified to log²
  (the thickness branch uses the log² form of the shifted distance (D-ξ²));
  Stiff has no thickness concept and is uniformly log² — this is the
  closest semantic equivalent.

## Line-search pre-cap alignment (feasible-step pre-cap, after `3982c6bb`)

- **Stiff-GIPC design** (GIPC.cu:10941-10973): the line search first
  computes `alpha = min(1, ground_feasible(0.8), self_feasible(0.8, MCP))`
  — an exact CCD cap over the currently active contact set (each pair keeps
  ≥20% of its current gap) — and **then** generates the full CCD trajectory
  candidates on the capped step (smaller swept boxes → fewer candidates);
  CFL intervenes only when CCD pairs exist (`h_ccd_cpNum>0`), with the
  floor semantics `alpha = max(alpha, alpha_CFL)` (prevents
  over-crushing).
- **libuipc implementation** (`global_contact_manager.{h,cu}` +
  `engine/advance_ipc.cu`): `GlobalContactManager::compute_feasible_step()`
  reuses the project's own
  `distance::{point_triangle,edge_edge,point_edge,point_point}_ccd`
  (`utils/distance/ccd.h`) to compute the CCD-TOI pair by pair over the
  active PT/EE/PE/PP pairs exposed by `SimplexTrajectoryFilter`'s public
  accessors (eta=1-slackness=0.2), with a single DeviceReduce().Min
  reduction; alpha is capped after the line search's record_start_point and
  before `detect_trajectory_candidates(alpha)` — trajectory candidates are
  generated on the capped step and shrink accordingly. The compound
  semantics of the `filter_toi` lambda was also fixed: the filter returns a
  fraction of the swept step; the absolute step = alpha·toi.
- **Verification**: in a weak-κ synthetic test (where the barrier cannot
  hold), the pre-cap triggers correctly (alpha=0.047/0.011/0.012); the
  wrecking-ball probe shows zero regression (71.7ms/frame, Newton 1.84,
  min_alpha constant 1); Stiff-side reference (instrumented): its feasible
  step triggers 273 times / 40s on case2 (alpha 0.2-0.4). Rare triggering
  in a well-conditioned aligned scene is correct behavior (under a strong
  barrier the Newton direction does not overshoot the active pairs); its
  value lies in under-converged / high-speed-impact scenes.
- **CFL floor not aligned (deliberately deferred)**: Stiff's
  `alpha = max(alpha, alpha_CFL)` floor semantics can push the step past
  the CCD hit point and requires an isIntersected-style crossing test as
  backstop (ground signed distance + edge-face crossing, D=0 contact
  legal). libuipc's current filter D>0 assertion would kill the process
  outright, so before adding the floor, penetration detection must first be
  changed to crossing semantics — follow-up standalone work.

## Kernel-level dissection of case2's remaining ~2× gap (nsys evidence)

Frame time ~245ms/frame (stacking phase), of which GPU kernels are busy
~115ms — **host-side API overhead is about half**: per frame 7429 kernel
launches + 522 memcpys + 1787 memsets + 563 stream syncs. Breakdown
(/frame):
- FusedPCG 68.6ms: ~83 iterations/solve × 118µs/iteration; of that, the
  kernels actually compute only ~36µs — the rest is launch gaps (7
  kernels/iteration in serial dependency + a convergence D2H sync every 5
  iterations draining the pipeline). → Lever: cooperative-groups
  persistent-kernel fusion (1 launch/iteration) or CUDA graph capture of
  the PCG inner loop; estimated 2-3× reduction.
- BVH self-queries (stacklessSelf 1.34ms + stacklessOther 0.89ms ×
  ~14/frame ≈ 31.5ms): the dense bunny surface yields ~450k candidates per
  detect. → Lever: fuse exact distance tests into the query predicate and
  materialize only active pairs.
- Contact assembly do_assemble ×2 ≈ 26ms; SNH G/H 2.12ms×7 ≈ 14.9ms
  (Stiff's equivalent FEM assembly kernel is 0.77ms/call — a 2.75× gap; the
  make_spd 9×9 EVD is a suspect, but removing it makes convergence worse —
  do not simply delete it); cloth DSB 8.8ms.
- Stiff same-scene nsys: ~900 launches/frame (libuipc is 8× that), FEM
  assembly 0.77ms/call.
- Negative result: `linear_system/check_interval` 5→25 has no effect on
  frame time (206 vs 205.7ms) — PCG's pipeline-drain stalls are not the
  main cost, don't spend time here; PCG's cost is mainly the iteration
  count itself (83/solve vs Stiff 27/solve, determined by the system
  condition number).
Conclusion: case2's ~2× is the product of structural host overhead +
multi-kernel throughput, and needs a dedicated round of kernel
fusion/graph-capture engineering (every change must pass the full-suite
regression).
- **⚠ Comparison-basis correction (second, final)**: the Stiff log's
  "average time cost" = totalTime/totalNT (GIPC.cu:11262) — it is the GPU
  time **per Newton iteration** (totalNT/totalTime/total_Frames are
  file-level globals accumulated across frames), NOT per-frame time! Also,
  the PAIRCOUNT printf/fflush I added to the reference copy inflates its
  GPU event timing. **Clean reference numbers (uninstrumented runs):
  wrecking ball 42.8ms/frame (frames 2-120, GPU event timing; full-run
  1654-frame average 50.4ms), case2 142.8ms/frame (447-frame average).**
  Corresponding final comparisons: wrecking ball libuipc 73.0ms vs
  42.8-50.4ms ≈ **1.5-1.7×**; case2 libuipc 301ms vs 142.8ms ≈ **2.1×**
  (same 250-frame window: 171.8ms → 1.75×).

## Cloth stiffness model update + strain_rate exposure (after `b7056879`)

> **Partly superseded on 2026-09-01:** the historical area multiplier and
> one-sided stretch/bending formulas below were corrected as recorded at the
> top of this handoff. They remain here only as the chronological explanation
> of the regression.

- Cloth stiffness formulas aligned with mas-pncg; membrane-element weights
  use the triangle **area** (not volume, avoiding incorrect volume-measure
  weighting of the thickness-independent shear):
  - stretch: `StrainLimitingBaraffWitkinShell`'s triangle attribute
    `"lambda"` is written as `(λ+2μ)·t` (identically `E·t/(1-ν²)`); in the
    backend `strain_limiting_baraff_witkin_shell_2d.cu`, the measure in
    both energy kernels was changed from `area·2t` to pure `rest_area`.
  - shear: attribute `"mu"` = `E/(2(1+ν))`, thickness-independent (no t
    under an area measure).
  - bend: the `DiscreteShellBending` family (including strain/stress
    plastic variants) unified its measure to area `V_bar = A` (3 function
    headers); the κ semantics of raw `apply_to(sc, κ)` became "stiffness
    per unit area" — **all raw call sites were migrated to κ×t to preserve
    physical consistency** (libuipc tests 33/82-87 and regression all
    t=0.001; samples 11/24/26/33ext/34 at their respective thicknesses).
    The formula overload `apply_to(sc, E, ν)` and the static helper
    `bending_stiffness(E,ν,t)` write the literal value `E·t³/(12(1-ν²))`.
  - strainRate: no longer hardcoded to 100 — `apply_to(...,
    strain_rate=100)` writes the triangle attribute `"strain_rate"`, and
    the backend reads the attribute (old scenes missing it get it
    auto-created and backfilled with 100); pybind exposure synced.
  - **stretch/shear material-parameter separation**:
    `StrainLimitingBaraffWitkinShell::apply_to` dual-modulus overload
    `apply_to(sc, stretch_moduli, shear_moduli, ρ, t, strain_rate)`
    (stretch uses `(λ_s+2μ_s)·t`, shear uses `μ_sh`, each with independent
    (E,ν), aligned with mas-pncg ClothMaterialConfig); the old
    single-modulus overload is kept for compatibility. samples
    11_bunny_cloth/34_cloth_stack now use the separated parameters (shear
    softened 100:1), and 11's bend additionally demonstrates the formula
    overload; the new
    `apps/tests/core/strain_limiting_baraff_witkin_constitution.cpp` covers
    the dual-modulus attribute layout.
  - **Single source of truth for thickness**: the DSB formula overload is
    `apply_to(sc, E, ν)` — bending is optional, stretch is required, so
    thickness is set only by the membrane constitution (vertex
    `"thickness"` attribute); DSB reads it averaged over edge endpoints
    (naturally supporting non-uniform-thickness shells); a clear error is
    raised when it is missing.
- Verification: Python smoke numerics all correct; DSB-related sim_case 33,
  82-87 all pass; the 6 fast binaries all pass; the full suite passes;
  `11_bunny_cloth` headless OK.
- Note: `ElasticModuli2D`/`EP_to_lame_2D` and other constitutions such as
  NeoHookeanShell are untouched (scope is only the cloth BW strain-limiting
  shell + DSB); the BW shell is unused by any suite/sample, so the measure
  change has zero regression surface.

## Hygiene & test-robustness batch (after `d2f48087`)

- **Duplicate-include sweep**: 17 files had identical `#include` lines
  (mostly migration-cruft `cuda_tool/cuda_tool.h`); deduped. The
  `geometry_export_types.inl` double-include in `geometry_factory.cpp` is an
  intentional X-macro pattern — do NOT dedupe it.
- **Catch2 v3.8 filter syntax (measured)**: multiple specs as separate argv
  are AND-intersected ("No tests ran"); OR requires comma-separated specs in
  ONE argv: `uipc_test_sim_case "0_abd_gravity,13_fem_3d_gravity"`.
  `--list-tests --verbosity quiet` prints one case name per line.
- **`file(GLOB ... CONFIGURE_DEPENDS)`** added to all 44 project CMake files
  (external/ untouched) — adding/removing sources no longer needs a manual
  re-configure.
- **Line-search diagnostics**: the "Line Search Exits with Max Iteration"
  warning and the strict-mode exception now carry
  `alpha_last / E0 / E_last / rel_E_increase / ccd_alpha / cfl_alpha`
  (`engine/advance_ipc.cu`, `engine/advance_al.cu`) so a threshold-crossing
  can be judged as real regression vs ULP jitter from the log alone.
- **Isolated suite runner**: `scripts/run_sim_case_isolated.py` runs each
  sim case in its own process (`--filter/--start-from/--timeout`) —
  complements the single-process suite to separate cross-case global-state
  pollution from case-local failures.
- The CMake-side ccache integration from this session was implemented and then
  reverted at user request. The later XMake audit also removed its stale
  `dev=true` ccache policy; both build paths now keep compiler caches disabled.

## Build-time optimization (after `88965feb`)

- **Umbrella split**: `cuda_tool/cuda_tool.h` no longer includes `cub.h`
  (CCCL device-algorithm headers add ~165K preprocessed lines per TU); the
  ~23 files using `Device*` wrappers include `<cuda_tool/cub.h>` explicitly
  (found via API grep over `DeviceReduce(/DeviceScan(/...` + `cub::`).
  `linear_system.h` also dropped its (unused) cub.h include.
- **RDC correction roller-coaster**: d2f48087 once turned RDC off claiming
  "no cross-TU device symbols" — **that conclusion was wrong**. The
  `UIPC_GENERIC` free functions in `affine_body/utils.cu` (`q_to_transform`
  etc.) are called by kernels in other TUs, so disabling RDC guarantees
  `ptxas fatal: Unresolved extern`. It went undetected then because
  CMake+ninja does not track flag changes, so stale RDC-on objects slipped
  through the link; the pyuipc build triggered a broad recompile and
  exposed it. `CUDA_SEPARABLE_COMPILATION/RESOLVE_DEVICE_SYMBOLS ON` has
  been restored and xmake gained `-rdc=true`. The "full rebuild ~4.7 min"
  figure in the next entry is also affected (some TUs were not recompiled)
  — for reference only.
- ~~**RDC off**~~ (corrected, see above)
- Measured (32-core, CUDA 13.2): full rebuild wall **~4.7 min** (was ~10 min
  perceived), CUDA TU CPU 8.3K→6.5K s, per-TU avg 38→35 s. Line-count
  attribution of a non-cub TU (~1.63M lines after -E): CUDA toolkit headers
  ~860K, WinSDK ~310K, MSVC STL ~150K, Eigen ~150K, project <20K — the
  remaining cost is toolchain headers, not project includes. This session once
  listed ccache (`CMAKE_CUDA_COMPILER_LAUNCHER`) as a possible next lever; the
  owner subsequently rejected compiler caches, so rule 8 supersedes that idea.
- Verified after both changes: 6 fast binaries pass; full sim suite passes
  (14214 assertions / 95 cases).

## Environment notes (unchanged)

- Build: `output/build.bat` (vcvars64 + `cmake --build build --config
  Release --target sim_case -j8`); for full error collection use
  `output/build_keepgoing.bat` (`ninja -k 0`); all test targets via
  `output/build_all_tests.bat`.
- nvcc needs the MSVC environment; a bare shell reports "Cannot find
  compiler 'cl.exe'".
- Configure: `cmake -S . -B build --preset ci-release
  -DUIPC_BUILD_BENCHMARKS=OFF -DUIPC_BUILD_EXAMPLES=OFF` (you must pass `-B
  build`; after removing/adding source files you must re-configure, because
  file(GLOB) is expanded at configure time).
- Filtering Catch2 by multiple case names does not work on this machine —
  run them one by one; single-name filtering works (e.g.
  `./uipc_test_sim_case.exe "36_no_surf_but_contact_on"`).
- `output/test_compile.cu` + `output/compile_smoke.bat` are the standalone
  smoke compile/run entry points for cuda_tool
  (`src/.../test_compile.cu.txt` is the in-repo archive).
- Occupancy probe: `output/probe_occupancy2.cu` + `.bat` (a
  `cudaOccupancyMaxPotentialBlockSize` comparison template of bare kernel
  vs muda-wrapped kernel).
- Full-suite logs: `output/test_sim_case_fix1.log` / `_run2.log`
  (post-migration, all passing); baseline reference
  `output/test_sim_case_baseline.log`.

## Default kappa policy (after `3982c6bb`)

- Rule (user requirement): if the user never calls `default_model(...)`, the
  effective default contact stiffness is `contact/adaptive/min_kappa`
  (default 1e8); if the user set it, the value is clamped into
  [min_kappa, max_kappa] (defaults [1e8, 1e11]) with a warning that prints the
  valid range; negative kappa (adaptive-kappa opt-in) is never clamped and
  takes precedence (the GIPCAdaptiveParameterStrategy path).
- Implementation: `ContactTabular` tracks `default_model_is_user_set()` (new
  public getter, additive); the policy is applied at
  `GlobalContactManager::Impl::_build_contact_tabular` when the host-side
  coeff table is filled, so the device table always carries the resolved
  values while the adaptive strategy keeps reading the core attribute for its
  negative-marker detection (unclamped by design).
- Verified: smoke test of all five cases (unset / below-min / above-max /
  in-range / negative marker) behaves exactly per spec; full sim suite
  95/14214 + 6 fast binaries green. Note the built-in default stiffness
  effectively changes 1e9 -> 1e8 for scenes that never set the default model.

## Wheel CUDA architecture list never reached nvcc (2026-09-03)

- Symptom: `pyuipc` 0.0.27 wheels (cp312 and cp313 manylinux both checked)
  contain `sm_75` SASS only and zero PTX, although `pyproject.toml` and
  `compatibility.json` declared `75/80/86/89-real` plus `89-virtual`. On an
  RTX 5090 (`sm_120`) `world.init(scene)` throws CUDA error 500
  `named symbol not found` from `cuda_tool/launch.h`.
- Root cause: `set_target_properties(... PROPERTIES ... CUDA_ARCHITECTURES
  ${UIPC_CUDA_ARCHITECTURES} ...)` did not quote the variable. An unquoted
  multi-element list expands into separate arguments and destroys the
  key/value pairing, so the property kept `75-real` and the remaining entries
  became bogus property names (`80-real` ended up as a property whose value
  was `86-real`). The `89-virtual` PTX entry was lost the same way.
- Fixed by quoting three sites: `src/backends/cuda/CMakeLists.txt`,
  `src/backends/cuda/components.cmake`,
  `apps/tests/backends/cuda/CMakeLists.txt`.
- Release matrix now also carries `120-real` for consumer Blackwell, mirrored
  in `python/src/uipc/compatibility.json` so
  `scripts/check_release_policy.py` stays green. Note this grows wheel size
  and CI compile time by one full architecture.
- Verification: configured with the full list and confirmed all 199 CUDA
  translation units in `compile_commands.json` now carry six
  `arch=compute_*,code=*` entries including `code=[compute_89]`; before the
  fix only `sm_75` was emitted. Reference build with `native` on the 5090
  (`sm_120`) runs `hello_affine_body` and the Python `0_check_libuipc` sample
  to completion.
- XMake was deliberately left alone (rule 7 reviewed): its CUDA arch surface
  is `add_cugencodes("sm_89")` under `github_actions` and
  `add_cugencodes("native")` otherwise, passed as single values with no
  multi-arch list, so it has neither the quoting defect nor a release matrix
  to mirror. XMake does not build the published wheel; the PyPI path is
  scikit-build-core plus CMake.
- Follow-up in the same area: the option's documented comma form
  (`-DUIPC_CUDA_ARCHITECTURES=75,89`) was normalized into
  `CMAKE_CUDA_ARCHITECTURES` only, while the backend targets read the option
  itself, so the comma reached nvcc and the first `.cu` failed with
  `'89' is not in 'keyword=value' format`. The root `CMakeLists.txt` now
  normalizes the cache entry in place. A `get_target_property` fast fail on the
  `cuda` target guards the truncation case; configure was checked with the full
  six-entry list, the comma form, and `native`.
- Post-merge hardening on `refactor-main` replaces all three raw property sites
  with `uipc_set_target_cuda_architectures`, which sets and reads back the
  property for the final library, every component OBJECT target, and the CUDA
  test target. A repository contract prevents any target kind from bypassing
  the helper. Wheel compatibility now distinguishes the CUDA 12.x SASS driver
  floor from the CUDA 12.8 PTX-JIT floor; `uipc doctor` reports the selected
  code path and no longer labels an old-driver PTX-only GPU compatible.
  Validation configured the comma-form release matrix and found all 214 CUDA
  translation units in the test-enabled build carrying all six codegen flags,
  then restored `native`. CMake and XMake production builds, fast CTest 3/3,
  repository contracts 48/48, portable Python tests 80 passed / 1 skipped, the
  real Python 3.14 CUDA doctor probe, and the complete documentation build all
  passed.
