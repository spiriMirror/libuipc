# 09 — Known Issues, Tech Debt, and Roadmap

Status as of 2026-09-03. Completed performance work is
recorded in `handoff.md`; this file tracks what is **open** — analyze here first
before planning new work.

Blender integration v0.1 was added on 2026-09-05. Cloth/ABD/fixed contact and
native cache playback have actual Windows Blender 4.5.3/RTX 5090 verification;
see [the integration guide](13-blender-integration.md). Linux/other Blender
versions, rods, joints and animated collision targets remain follow-up work.
Version 0.2 now adds native geometry/tetrahedralization, volume FEM, surface and
internal pins, and whole-object Fixed. See ADR 0008 for the strict boundary and
conservative construction contract. This does not change the simulation solver.

AL-IPC now has the fork-derived earliest-TOI active-set filter, decay-derived
pair lifetime, conditioning-aware penalty initialization, and a full-step inner
solve boundary. Its multi-state `K_min > 1` cumulative termination rule is now
implemented and validated on sample 88. Remaining paper-parity work includes
penalty-free moving boundaries and the specialized conflict-free analytic PSD
Hessian assembly. Do not claim complete parity with the reference simulator
until those paths and its large stress scenes are independently ported and
validated.

The uniform AL `diag_norm` penalty mode remains experimental. It caused
repeated non-descent line-search failures in mixed-resolution cloth/FEM sample
88 even after friction derivatives were corrected; `per_vertex` is therefore
the default. A future conditioning design should retain local mass/material
heterogeneity rather than broadcasting one global scalar.

## Cross-cutting source-audit findings

These previously verified gaps are closed on `refactor-main` as of 2026-08-25:

- The historical `baraff_witkin_shell.h` is now a compatibility include for the
  implemented `StrainLimitingBaraffWitkinShell`; its empty `.cpp` and other
  unreferenced zero-byte scaffolds were removed. A repository gate rejects new
  zero-byte files under `include/` and `src/`.
- `scripts/check_constitution_api.py` compares every exported constitution class
  with its pybind class name and verifies every binding initializer is registered.
  `RotatingMotor` and `LinearMotor` are both covered. Internal UID 27/28 remain
  intentionally internal and therefore outside the public-header contract.
- Every external GitHub Action is pinned to a reviewed full commit SHA. Both
  `johnwason/vcpkg-action` inputs and the Linux wheel container use the same
  immutable vcpkg commit as the generated registry baseline. The repository
  contracts workflow rejects floating action and revision refs.

## Current performance baseline

The current machine-specific reference is the
[2026-09-01 cross-domain baseline](performance/2026-09-01-cross-domain-baseline.md):
RTX 5090, CUDA 13.2, Windows Release, three fresh processes per scene, canonical
Timer-free throughput paths.

| Benchmark | Frames | Reference mean | Three-run range | Newton/frame | PCG/frame |
|---|---:|---:|---:|---:|---:|
| pure ABD wrecking balls | 120 | 129.5 ms | 126.4–131.3 ms | 3.95 | 107.3 |
| case2 FEM + cloth | 250 | 201.1 ms | 199.0–230.6 ms | 6.64 | 246.8 |
| MAS bunny | 100 | 60.2 ms | 60.1–62.0 ms | 4.67 | 358.8 |
| ABD wall + cloth | 100 | 125.6 ms | 121.8–165.0 ms | 5.13 | 200.1 |

All 12 throughput runs completed and converged without hitting iteration
limits. Collision-rich scenes have trajectory and WDDM/dynamic-memory
variability, so future work must compare three-run envelopes plus structured
iteration counts. The older Stiff-GIPC ratios below have not been refreshed
under this contract and must not be presented as current cross-project results.

### Superseded Stiff-GIPC comparison context

Measured on aligned scenes (same machine, clean runs; see handoff for the
full evidence chain):

- 6_wrecking_balls: libuipc 73.0 ms/frame vs Stiff 42.8-50.4 → **~1.5-1.7×**
- case2 (samples 88_stiff_gipc_benchmark): 301 ms/frame vs 142.8 → **~2.1×**

Root cause is NOT numerical parameters anymore (kappa, d_hat, eps_velocity,
semi-implicit exit are all aligned; Newton iteration counts match). The gap
is structural host/device overhead (nsys evidence, case2 stacking phase):

- ~7429 kernel launches + 522 memcpys + 1787 memsets + 563 stream syncs per
  frame (Stiff: ~900 launches) — **host-side API overhead ≈ half the frame**
- FusedPCG: 68.6 ms/frame = ~83 iters × 118 µs/iter, of which only ~36 µs
  is kernel compute — the rest is launch gaps (7 serially-dependent kernels
  per iteration) + a convergence D2H sync every 5 iterations
- BVH self-queries ~31.5 ms/frame: dense bunny surface → ~450k AABB
  candidates per detect, only ~3-8k survive distance filtering
- Contact assembly ≈ 26 ms; SNH G/H 2.12 ms/call ×7 (Stiff's equivalent:
  0.77 ms — 2.75×; `make_spd` 9×9 EVD is a suspect but removing it hurts
  convergence — see doc 08)
- Newton iterations per frame (found 2026-08-23 evening, case 89 parity
  run): Stiff averages **2.55 Newton/frame** while libuipc ran **7.04** —
  the old `newton/min_iter` doubled as a hard floor (≥6 with the benchmark
  configs), cancelling the semi-implicit early exit. Since then
  `min_iter` is a pure floor with default 0 and the beta-accumulation
  start moved to `newton/semi_implicit/K_min`. Semi-implicit termination is
  now enabled by default with `K_min=6`. Per-Newton
  solver time already beats Stiff on the same MAS bunny (≈39 ms wall vs
  48.8 ms GPU), so the frame-time gap on MAS scenes is dominated by the
  Newton count, not solver efficiency.

**Historical optimization ledger (completed or explicitly deferred):**
1. ~~Cooperative-groups persistent-kernel fusion or CUDA-graph capture of the
   PCG inner loop~~ **DONE (2026-08-23)**: FusedPCG now replays
   `check_interval`-sized iteration blocks as CUDA graphs
   (`linear_system/use_cuda_graph`, default on; details in doc 05).
   Remaining within-PCG cost is the ~83-iteration count itself.
2. ~~Fuse exact distance tests into BVH query predicates~~ **PARTIALLY DONE
   (2026-08-23 evening)**: the four DCD leaf predicates in
   `info_stackless_bvh_simplex_trajectory_filter.cu` now run the exact
   `distance::*_distance2` test when `alpha == 0` (DCD detect pass),
   keeping only `D2 < (d_hat+thickness)^2` pairs; the `alpha > 0`
   trajectory pass keeps the conservative `ccd_broadphase` (CCD superset
   requirement). Kills the 450k-candidate materialization + the
   overflow-retry double traversal on the DCD path; Detect DCD Candidates
   dropped 39 → 29 ms/frame on case 88. The trajectory/CCD pass
   (`Detect Trajectory Candidates`, ~38 ms/frame on 88) still uses the
   conservative swept test — tightening it needs a provably conservative
   exact swept-distance predicate, more delicate.
3. ~~SNH/FEM assembly kernel throughput~~ **DONE (2026-08-24)**: the SNH
   constitution was replaced wholesale by Stiff-GIPC's SNK1 (their energy
   `0.5μ(Ic-3) + 0.5λ(J-1-μ/λ)²`, their gradient, and their analytic
   twist/flip + 3x3-direct SPD projection — the generic 9x9 `make_spd`
   EVD is gone from this kernel; `make_spd` itself remains for the other
   constitutions). Case 88 median 297 -> 266 ms/frame; case 89 PCG
   213 -> 77 per solve (the analytic projection is also a much
   better-conditioned system). NOTE: the FEM energy changed numerically —
   trajectories shift slightly vs older baselines (89 resting centroid
   -0.775 -> -0.786) but stay physically equivalent. Same round:
   `cuda_tool::eigen::svd/pd` re-implemented on our own
   `algorithm/qr_svd.hpp` (GEIGEN port) — no more Eigen JacobiSVD on the
   host path, and the double overload no longer downcasts to float.
4. Same-address atomic storms (DONE 2026-08-23 evening): `Spmv_rbk_sym_
   spmv_dot_kernel` and `fused_dot_kernel` now do two-level (warp → block)
   reduction with one atomicAdd per block — ~9k/~4k same-address atomic
   doubles per call serialized ~20-30 us before. SpMV 114 → 93 us/call;
   case 88 median 312 → 297 ms/frame. NEXT within-PCG target: the
   symmetric-storage transpose scatter (~243k atomic vec3 per SpMV);
   a full-storage row-based SpMV would eliminate atomics entirely but
   doubles matrix traffic and needs a converter variant — deferred.
5. MAS assembly overhead (DONE 2026-08-25): the static mesh-partition
   hierarchy is cached after `init_matrix()` instead of being rebuilt for
   every Newton iteration; Hessian scatter now walks BCOO directly instead of
   filling an identity-index buffer; and the 48x48 Gauss-Jordan kernel drops
   the barrier between independent row updates. On RTX 5090, case 88 over the
   same 60 frames / 333 Newton iterations reduced `Assemble Preconditioner`
   1149.9 -> 834.4 ms (-27.4%, 3.45 -> 2.51 ms/Newton). The single-run wall
   mean moved 213.3 -> 208.1 ms/frame; contact-stage variance is larger than
   the remaining MAS contribution, so stage timing is the reliable metric.
6. Default BVH rebuild overhead (DONE 2026-08-25):
   `info_stackless_bvh` now uses CUB radix sort and scan with the existing
   persistent per-stream workspace. Identity generation and required state
   resets are fused into one kernel; redundant/dead fills are gone. Moving
   scene-AABB, leaf-LCA, and depth initialization before the multi-block
   consumers also removes cross-block reset races. Case 88, same 60-frame
   phase on RTX 5090: trajectory detection 7.98 -> 4.98-5.00 ms/Newton and
   aggregate DCD 7.35 -> 4.59-4.60 ms/detect (both about -37.5%); two wall
   runs measured 175.6-177.1 ms/frame versus 208.1 ms before.
7. DyTopo intermediate conversion (DONE 2026-08-25): a scene with one diagonal
   receiver owning the dynamic vertex prefix now forwards its raw assembled
   doublets/triplets directly to that receiver. The final global matrix
   converter performs the required sort/reduce already. Multiple receivers,
   off-diagonal ranges, and ABD/FEM coupling retain the classified path. On
   case 88, `Compute DyTopo Effect` fell from 6.11 to 4.76 ms/Newton (-22.0%);
   final `Convert To BCOO` stayed flat (1.85 -> 1.86 ms/Newton), and the clean
   wall mean/median improved 175.6/196.4 -> 165.5/184.2 ms/frame.
8. Historical case-88 frame budget at that revision/window (60 frames,
   165.5 ms mean representative):
   Build Linear System 43.4 ms + DyTopo 26.3 ms + trajectory detect 26.9 ms
   + aggregate DCD 24.5 ms + FusedPCG 24.8 ms + misc. The next evidence-led
   targets are raw contact/FEM subsystem assembly and the remaining
   line-search launch traffic, not another broad-phase or DyTopo sort rewrite.
9. Dynamic output initialization/growth (DONE 2026-08-30):
   `DeviceVector` now has explicit discard/preserve and amortized reserve
   policies. Fully regenerated collision, line-search, matrix-conversion,
   active-set, and triplet outputs no longer copy or value-initialize dead
   ranges when their logical size fluctuates. Existing `resize()` semantics
   are unchanged for state and sentinel buffers. On case 88, `Scan and
   Allocate` fell from 80.8 ms total over 60 frames to 38.0-65.5 ms, while
   the initial/trial `Compute Energy` scopes together fell from 667.1 ms to
   390.2-415.8 ms. End-to-end wall variance remains larger than this isolated
   gain. The next step is batching collision counter readbacks, not broadening
   discard semantics to buffers whose initialization contract is uncertain.
10. Collision count readbacks (DONE 2026-08-30): the default trajectory
    filter batches four broad-phase query counters into one D2H synchronization
    and batches the four PP/PE/PT/EE CUB selection counters into another. Queue
    overflow handling remains exact and reruns only affected queries. A fully
    active detect/filter cycle therefore uses two count synchronizations rather
    than eight. On case 88, trajectory detection changed from 5.07 to 5.01
    ms/Newton and aggregate DCD from 4.67 to 4.61 ms/detect; the structural
    synchronization reduction is larger than the noisy wall-time movement.
    The next host/device target is line-search energy aggregation.
11. Line-search energy aggregation (DONE 2026-08-30): ABD, FEM, and DyTopo
    reporters now publish totals to contiguous device slots. One final CUB
    reduction writes to a separate slot, followed by one contiguous D2H copy
    for reporter diagnostics and the aggregate. On case 88, initial/trial
    energy evaluations improved by 15.2%/17.3%, aggregate line search improved
    by 5.0% per Newton iteration, and wall mean/median moved from 162.7/182.3
    to 158.1/178.4 ms/frame. The next evidence-led target remains raw
    contact/FEM gradient-Hessian assembly.
12. Raw contact/FEM assembly (DONE 2026-08-30): SNH now projects its `9x9`
    material Hessian directly into ten `3x3` vertex blocks instead of forming
    dense `9x12` and `12x12` intermediates. Stack use fell 6440 -> 1320
    bytes/thread, the SNH kernel fell 1.795 -> 1.047 ms/call (-41.7%), and
    case-88 `Assemble Subsystems` fell 3.60 -> 2.97 ms/Newton (-17.6%). IPC
    contact keeps a single heterogeneous launch but compile-time specializes
    gradient-only versus Hessian work. A per-stencil split was measured and
    rejected: serial PT/EE kernels raised DyTopo assembly 4.52 -> 7.67
    ms/Newton despite smaller static stack frames. Final DyTopo assembly is
    4.47 ms/Newton, and two clean wall runs measured 156.0-157.0 ms mean /
    173.1-173.9 ms median.
13. Backend module boundary (DONE 2026-08-30): test and packaged builds now
    use the same shared-library artifact semantics. A required
    `uipc_query_module` handshake validates ABI version, libuipc major/minor,
    and backend identity before PMR synchronization or engine construction.
    This prevents stale/mixed backend DLLs from reaching the C++ virtual ABI.
14. CUDA build ownership (DONE 2026-08-30, corrected 2026-08-31 and
    2026-09-01): 198 compiled
    backend sources belong to seven primary domains and one optional
    legacy-collision component, followed by one final RDC device-link into the
    existing backend DLL. Matching CMake/XMake manifests reject unowned and
    multiply-owned sources. CMake uses internal OBJECT targets; XMake keeps the
    same logical partition but attaches sources to the final target because its
    device-link omits CUDA OBJECT dependencies.
    CMake's final target additionally owns one generated comment-only CUDA
    language anchor because Visual Studio otherwise omits the RDC device-link
    when all real CUDA sources arrive through OBJECT expressions. Functional
    source ownership remains unchanged.
15. Scene configuration ownership (DONE 2026-08-30): typed runtime defaults and
    machine-readable schema metadata now come from one declaration. The public
    normalized schema remains exactly equivalent, while future key additions can
    no longer silently update only one of the two former parallel lists.
16. SimSystem topology (DONE 2026-08-30): creator instantiation and every
    collection traversal now use one deterministic complete-type-name order;
    exact lookup uses `std::type_index`, compatible lookup skips invalid
    variants, and active strong-dependency cycles fail with an explicit path.
17. Legacy collision isolation (DONE 2026-08-30): the three alternate simplex
    trajectory filters have a dedicated optional component. Lean builds omit
    the sources and registrations, while the build-specific scene schema drops
    the unavailable selectors instead of accepting a configuration that can
    only fail later during backend system construction.
18. Test/performance entry points (DONE 2026-08-30): isolated sim cases support
    stable manifests and round-robin shards without replacing the aggregate
    pollution test; CTest GPU aggregates share a resource lock; the Stiff-GIPC
    case2 sample has a root-owned benchmark contract and revision-recording
    runner instead of another copied scene.
19. Decision/evidence retention (DONE 2026-08-30): accepted architecture now
    has numbered ADRs, performance work has an evidence policy/template and a
    case2 roll-up, and `handoff.md` is explicitly the chronological trail rather
    than the sole permanent home for rationale and measurements.
20. CI portability follow-up (DONE 2026-08-31): XMake CUDA sources now stay on
    the final shared target, which supplies the source-root include,
    backend-directory definitions, shared-library PIC behavior, and the one
    complete RDC device-link on both platforms. This replaces an intermediate
    OBJECT-target implementation that failed successively on missing includes,
    definitions, Linux PIC, and finally Windows device-link. The repository
    contract guards the final-target ownership requirement. In addition,
    repository-contract tests no longer confuse an intentionally unmaterialized
    samples submodule with an invalid benchmark declaration.
21. Thin-shell reference measure (DONE 2026-09-01): elastic and both plastic
    Discrete Shells paths retain the paper's complete `L0/h_bar = 3L0^2/A`
    metric and no longer multiply the adjacent area a second time. The stored
    thickness is consistently the one-sided collision radius `r`: formula-based
    bending uses full thickness `2r`, and Baraff-Witkin stretch uses `2r` while
    its separately calibrated shear coefficient remains thickness-independent.
22. QR-SVD float sign transfer (DONE 2026-09-01): the Wilkinson shift in
    libuipc, GPU_IPC, and Stiff-GIPC no longer calls the standard-library
    sign-copy function from host/device templates. It applies the sign with a
    branch in the original scalar type and defines `sign(0)=+1`; a libuipc CUDA
    regression instantiates and executes the float path on the GPU.
Every such change must re-pass the full sim suite (currently 95 cases / 14212
assertions).

## Deliberately deferred

- **CFL floor semantics** (`alpha = max(alpha, alpha_CFL)` in Stiff-GIPC):
  can push the step past the CCD hit point and needs crossing-based
  penetration detection (signed-distance + edge-face crossing, D=0 legal)
  as a backstop. libuipc's current filter asserts D>0 and would abort.
  Do that semantics change first, then the floor. (handoff: "Line-search
  pre-cap alignment")

## Dependency pins to unwind (standing debt)

| Pin | Where | Unwind when |
|---|---|---|
| `ports/tinygltf` overlay (SHA512 of regenerated v2.9.6 tarball) | `ports/`, `scripts/gen_vcpkg_json.py` | microsoft/vcpkg fixes the tinygltf port hash |
| `octree v2.5` | `src/geometry/xmake.lua` | xmake-repo layout stabilizes |
| `tinygltf <3` | `src/core/xmake.lua` | xmake-repo v3 include layout decided / our includes updated |

## Open issues

- **Published wheels through 0.0.27 need the CUDA 12 cuBLAS runtime; current
  source removes it**: package
  installation and `import uipc` succeed, but `Engine("cuda", ...)` fails on
  a CUDA 13.2-only machine because `uipc_backend_cuda.dll` directly imports
  `cublas64_12.dll`; the machine provides only `cublas64_13.dll`. This is not
  a missing bundled vcpkg DLL and not a driver-compatibility problem. Current
  workaround for 0.0.27: install CUDA 12.8 side-by-side and expose its `bin`
  directory, or build from current source. The source tree now replaces the
  remaining cuBLAS dot/norm calls with persistent raw-CUDA/CUB reductions and
  audits every wheel for dynamic Toolkit dependencies. Future wheels require a
  compatible NVIDIA driver rather than a local Toolkit: the base CUDA 12.x
  floor applies to packaged SASS, while PTX-only GPUs require the recorded CUDA
  12.8 JIT-driver floor. Remaining before
  calling the release-level issue closed: publish those wheels and add a
  GPU-capable CI job that constructs the CUDA engine; hosted binary inspection
  plus the no-GPU smoke test cannot prove actual driver/GPU execution.
- **CUDA-graph capture crash in the C++ suite binary (worked around)**: with
  Timer objects created inside the captured call chain, the single-process
  suite deterministically fail-fasted (0xC0000409) at the second engine's
  capture (never in isolation, never in python multi-engine repro). The
  capture path now creates no Timers. A second unresolved thread: al-ipc +
  graph capture crashed in the suite binary even without Timers (python
  al-ipc captures fine) — graph replay is gated to `contact/constitution ==
  "ipc"` until root-caused. Both symptoms point at some lurking global-state
  interaction with stream capture in the test binary; if someone revisits,
  start from `scripts/run_sim_case_isolated.py` + a binary-search over
  engine-count.
- **Performance work must start from the current four-scene baseline**: graph
  replay, DCD distance fusion, SNK1, discard-aware growth, batched readbacks,
  and direct FEM/contact assembly are already included. The synchronized
  diagnostic has no single universal hotspot: rigid is global-solve/assembly
  heavy, MAS bunny is linear-solve heavy, and case2/wall-cloth distribute cost
  across solve, line search, trajectory detection, DyTopo, and DCD. Profile the
  target scene before selecting another lever.

## External PRs under review

- **libuipc PR #461** (tcordeboeuf, EmbeddedCollisionMesh — barycentric
  coupling of a dense passive surface onto a coarse FEM tet mesh, SOFA-style).
  Direction is valuable, but as submitted it: (1) never writes
  `ecm_tet_geo_id` in `apply_to`, so the CUDA side silently no-ops —
  feature dead as written; (2) writes reporter gradient doublets
  conditionally → uninitialized slots get atomic-added into the linear
  system (corruption); (3) forward pass hooks outside the Newton loop →
  stale surface positions from iteration 1 on; (4) predates the muda→
  cuda_tool migration and the CBCOO gradient layout — needs a port. Not
  merged; a fix round was scoped and then declined for now. If revived:
  port to cuda_tool raw kernels, write `ecm_tet_geo_id`, fill all reported
  doublet slots, move the forward hook into the Newton loop, populate the
  frame-0 positions, add pybind + tests.
- **libuipc-samples PR #5** (hugooole, keyboard→imgui in case 4): reviewed,
  safe to merge; would only want `keyboard` dropped from requirements.txt.

## Security advisories

- pytest `< 9.0.3` tmpdir CVE → both metadata files require
  `pytest>=9.0.3`; `python/uv.lock` resolves 9.1.1. Dev-only dependency.
- usd-core `< 25.8` (critical) is marked fixed in the repo, but USD is a
  local/optional dep — upgrade any local install to `usd-core >= 25.8`.

## Samples submodule state (spiriMirror/libuipc-samples)

The root repository tracks this repository as the `libuipc-samples/` submodule.
It currently has 52 example directories; numbering is non-contiguous and two
directories use the `40_` prefix, so paths/names—not integer IDs—are the stable
reference.

- `87_robot_hand` — URDF robot hand (ABD links + soft transform
  constraints) + ABD cube on the ground, manual GUI posing (sliders +
  pose save/load). Ported from `references/Robotics-Libuipc` and rewritten
  in a different structure; assets renamed `leap_hand → robot_hand`
  (urdf `filename=` refs rewritten; link/joint names unchanged because the
  pose jsons key on them). The scripted auto-grasp was removed at the
  user's request (manual posing instead).
- `88_stiff_gipc_benchmark` — the Stiff-GIPC set_case2 benchmark with a GUI
  (default) and the original headless loop (`--headless [N]`); both modes
  write `traj.csv` + timing summary.
- `89_mas_bunny` — Stiff set_case7 parity (single MAS bunny, E=1e7);
  `NO_MAS=1` / `NO_GRAPH=1` env A/B switches.
- `90_abd_fem_cube_stack` / `91_pinned_cloth` / `92_twisting_bar` /
  `93_cube_wall_cloth` — the remaining Stiff-GIPC set_cases 1/4/5/6
  (2026-08-24). 92 uses animated SoftPositionConstraint ends (twist);
  93 has MAS on (case6 P_type=1) plus a 1920-body ABD wall. Assets added:
  `stiff_cube.msh` (Stiff's own 0.4-size cube — the samples' cube.msh is
  dimensionally different and overlaps at case1/6 spacings) and
  `high_mat.msh`. Both Stiff meshes were missing `\$MeshFormat` /
  `\$EndNodes` / `\$EndElements` tags and were completed for igl::readMSH.

## MAS preconditioner parity check (2026-08-23, case 89)

Cross-project comparison on the same scene (SNK bunny2, E=1e7, 100 frames;
libuipc samples `89_mas_bunny` vs Stiff-GIPC `set_case7`, both with MAS):

| run | total PCG | avg per solve | Newton/frame |
|---|---|---|---|
| Stiff MAS (P_type=1) | 60293 | 236 | 2.55 |
| libuipc MAS (mesh_partition) | 3500 | **5** | 6.0 |
| Stiff diag (P_type=0) | 376877 | 1513 | 2.49 |
| libuipc diag (NO_MAS=1) | 922185 | 1310 | 6.0 |

Verdicts:
- diag-vs-diag is same order (1310 vs 1513) — solvers/systems are comparable.
- ITERATION-COUNT CAVEAT (corrected 2026-08-23 evening): the 5/solve figure
  was an artifact of a graph-capture bug — the non-blocking capture stream
  let the un-plumbed MAS engine execute during capture instead of joining
  the graph, so replays ran with a frozen preconditioner (false r^Tz
  collapse). Fixed by using a blocking capture stream (un-plumbed callees
  now invalidate capture -> automatic fallback). After the fix, MAS averages
  ~41/solve vs diag ~86/solve on the E=1e4 bunny, with trajectories matching
  to 0.3mm at frame 100. The MAS preconditioner itself was never wrong.
- MAS + CUDA graph (closed 2026-08-23): the engine's `apply` path is now
  stream-plumbed (`MASPreconditionerEngine::apply(..., stream)` +
  `FEMMASPreconditioner::do_apply` forwards `info.stream()`), so MAS scenes
  join the captured graph instead of falling back to plain launches
  (graph_mode=1 confirmed active in MAS scenes). Validation at
  E=1e7/100 frames with contact: MAS+graph trajectory identical to the
  diag+graph reference at 1e-4 print precision; resting centroid -0.7753 vs
  the rigid-body geometric prediction -0.7731 (2mm compression); iteration
  counts real (frame-20 solves: MAS 5/20/155/435/495/410/505 vs diag
  2380/1625/880/1585/1650/1625/785 — MAS 3-4x fewer, no false-convergence
  signature). Free-fall no-contact: graph on/off bit-identical. One red
  herring cost a debug round: a "frozen" E=1e4 trajectory turned out to be
  a stale site-packages dll, not a solver bug (see doc 08 stale-dll trap).
  Measured benefit (89_mas_bunny, E=1e7, 100 frames, 704 solves/~198k PCG
  iters, identical iteration counts and f100 centroid on both paths):
  graph on 29.8s vs graph off 36.0s wall = ~17% end-to-end (~32 us/PCG
  iteration of launch-gap savings). For reference the same scene with the
  diagonal preconditioner (graph on) takes ~74s — MAS itself is the bigger
  win (~2.5x), the graph plumbing adds its share on top.
- COVERAGE RULE (measured on 88_stiff_gipc_benchmark, E=1e7, two 19k-vert
  bunnies + 4.2k-vert cloth, 60 frames, 2026-08-23): MAS only pays off when
  it covers nearly all of the stiff DoFs. Partial coverage is net NEGATIVE:  | config | total PCG | avg/solve | wall |
  |---|---|---|---|
  | all diagonal | 366875 | 853 | 60s |
  | MAS on upper bunny only (~45% of FEM verts) | 397730 | 923 | 73s |
  | MAS on both bunnies (~90%) | 79225 | 184 | 29s |
  f59 centroids identical in all three — physics unaffected; only the
  preconditioner spectrum changes. PCG converges at the rate of the
  worst-preconditioned block, so a diag-covered stiff block (the lower
  bunny in ground contact + the cloth) dominates and the MAS per-iteration
  overhead (cluster-inverse assembly + 5-launch apply) is pure cost.
  This table was measured with per-mesh tagging; it is the direct
  motivation for the all-or-nothing redesign below.
- ALL-OR-NOTHING REDESIGN (2026-08-23): MAS activation is now a scene
  config switch — `linear_system/fem_preconditioner = "mas"` (default
  `"diag"`). When on, `FEMMASPreconditioner::do_init` auto-partitions
  EVERY non-Empty FEM SimplicialComplex internally on a private clone
  (fixed cluster size = `BANKSIZE` 16, the only size the engine's shared-
  memory kernels accept); a pre-existing `mesh_part` attribute (custom C++
  partitioning) is still respected as-is, but manual tagging alone no
  longer activates MAS. The python `uipc.geometry.mesh_partition` export
  was removed (the exposed `part_max_size` was a footgun — any value > 16
  trips the BANKSIZE assert). Defensive fix included: switch on but
  nothing partitionable (e.g. all-Empty FEM) -> do_apply falls back to
  z=r instead of leaving z stale. Migrated: sim_case 53-61/81 + the
  stitch regression (config switch instead of mesh_partition calls;
  58_hybrid_mix now verifies custom + auto partitions coexist), samples
  88/89 (NO_MAS=1 env flips the switch to "diag").
- SCENE-ADAPTIVE KAPPA CORRIDOR (2026-08-24): Stiff-GIPC's
  `suggestKappa`/`upperBoundKappa` (`GIPC.cu:9850-9888`) is ported into
  `GlobalContactManager::init` with the dt^2 conversion (their barrier
  applies kappa raw, ours scales by dt^2 → computed values divided by
  dt^2). The corridor [suggest, 100×suggest] now rules all kappa clamping
  (default-model resolution, per-model clamps, the adaptive strategy's
  projection); `contact/adaptive/{min,max}_kappa` are fallback-only when
  the corridor is not computable. The evaluation scale is user-configurable
  via `contact/adaptive/kappa_eval_scale` (default 1e-16 — Stiff's value;
  the /dt^2 conversion keeps it valid for any dt). Measured corridors:
  case 89 → [4.4e7, 4.4e9], case 88 → [4.7e6, 4.7e8]; the established
  κ=1e8 sits inside both, so aligned scenes are behaviorally unchanged.
- Structural reason libuipc's port is stronger: FEMMASPreconditioner
  rebuilds the cluster inverses from the current full diagonal Hessian
  (kinetic + material) every Newton iteration
  (`fem_mas_preconditioner.cu` `set_preconditioner(A.values(), ...)`).
- Not ported by design: Stiff's collision-aware clustering
  (`_buildCollisionConnection`, deliberately skipped per owner decision)
  and the size-specialized Schwarz kernels (`_schwarzLocalXSym3/6/9`,
  `__inverse6_P96x96`) — the latter is a throughput optimization lead.
- Stiff-GIPC local benchmark edits (uncommitted, local only): `set_case7`
  in `gl_main.cu` (single bunny, E=1e7, P_type=1), a 100-frame cap writing
  `timeCost.txt` (totalNT/totalCgCount), and a `GIPC_PTYPE` env var to flip
  the preconditioner type.
