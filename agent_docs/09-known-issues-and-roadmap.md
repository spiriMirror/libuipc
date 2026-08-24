# 09 — Known Issues, Tech Debt, and Roadmap

Status as of 2026-08-25, source audit at `947bb921`. Completed performance work is
recorded in `handoff.md`; this file tracks what is **open** — analyze here first
before planning new work.

## Cross-cutting source-audit findings

These are verified code/documentation gaps, independent of the performance plan
below:

- **XMake violates current owner/build-parity rules.** Default `dev=true` enables
  `build.ccache`; stale `gui`/`torch` options remain; `src/xmake.lua` checks an
  undeclared `grpc` option and a missing `rpc` directory; optional USD/VDB have no
  equivalent target; the pybind post-build path starts duplicate asynchronous
  package-copy actions.
- **The UID documentation generator is incomplete while still passing `--check`.**
  Its parser misses statement-assigned `UIDInfo` registrations, currently omitting
  UIDs 15, 17, 31, and 32. The generated constitution UID page must not be treated
  as exhaustive until parser regression tests cover both registration styles.
- **One constitution header is only a placeholder.**
  `baraff_witkin_shell.h` and its `.cpp` are zero bytes. The implemented cloth
  model is `StrainLimitingBaraffWitkinShell` in
  `strain_limiting_baraff_witkin.h`.
- **Public C++ and Python surfaces differ.** `RotatingMotor` and `LinearMotor` are
  implemented C++ classes sharing UID 16 with `SoftTransformConstraint`, but are
  not bound to Python. Internal UID 27/28 registrations have no public classes.
- **Python metadata/helper drift remains.** Root package metadata omits matplotlib
  used by reporting paths and leaves the dev pytest dependency unpinned; the
  development description still says CUDA 12.6+; the Warp empty-strides path
  references undefined `BufferUtils.element_size`; `strip_constitutions` assumes
  dense object IDs.
- **Schema does not guarantee behavior.** `newton/use_adaptive_tol` is registered
  but has no consumer. Contact/Subscene JSON config arguments are accepted but
  ignored. Adding a default must be paired with a runtime reader and behavior test.
- **IPC and AL-IPC orchestration are not fully symmetric.** The AL path currently
  differs in external-force sequencing and timer behavior. Changes that claim
  support for both constitutions need two focused tests.
- **User-doc navigation has small standing drift.** The prose-only MkDocs build
  succeeds but reports `docs/build_install/xmake.md` and
  `docs/development/deterministic_mode.md` as orphan pages, plus a missing
  `#Reporter-Manager-Receiver-Model` anchor from `development/index.md`. The
  geometry tutorial also has a generated-class link that can only be confirmed in
  a successful Doxygen/MkDoxy build.

## Performance: remaining gap vs Stiff-GIPC

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
  start moved to `newton/semi_implicit/K_min` (default 1). Per-Newton
  solver time already beats Stiff on the same MAS bunny (≈39 ms wall vs
  48.8 ms GPU), so the frame-time gap on MAS scenes is dominated by the
  Newton count, not solver efficiency.

**Planned levers (in priority order)**:
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
5. Case-88 frame budget after these rounds (60 frames, mean ~263 ms):
   FusedPCG 92 ms + Build Linear System 54 ms + trajectory detect 38 ms +
   DCD 29 ms + DyTopo 27 ms + misc. No single 2x item remains; it's a
   grind of 10-30 ms items.
Every such change must re-pass the full sim suite (95 cases / 14214
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
| `johnwason/vcpkg-action revision: master` | `.github/workflows/*.yml` | pin to a known-good commit proactively (drift risk, same class as the three above) |

## Open issues

- **PyPI 0.0.26 Windows wheel needs the CUDA 12 cuBLAS runtime**: package
  installation and `import uipc` succeed, but `Engine("cuda", ...)` fails on
  a CUDA 13.2-only machine because `uipc_backend_cuda.dll` directly imports
  `cublas64_12.dll`; the machine provides only `cublas64_13.dll`. This is not
  a missing bundled vcpkg DLL and not a driver-compatibility problem. Current
  workaround: install CUDA 12.8 side-by-side and expose its `bin` directory,
  or build from source against CUDA 13. The immutable 0.0.26 wheel cannot be
  corrected in place. Before the next release, decide whether to keep the
  explicit CUDA 12.8 runtime requirement, bundle/link CUDA runtime components
  where licensing and wheel size permit, or publish a deliberate multi-CUDA
  packaging strategy. In every case, CI/release verification must instantiate
  the CUDA engine from the produced wheel; a plain Python import is insufficient.
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
- **Remaining case2 gap after the PCG graph work**: measure again with the
  graph on; the next levers are BVH distance-fusion and FEM assembly
  throughput (doc above).

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

- pytest `< 9.0.3` tmpdir CVE → development metadata/lock are fixed
  (`pytest>=9.0.3`, lock at 9.1.1), but root `pyproject.toml` still declares the
  dev extra as unpinned `pytest`; synchronize it before calling the metadata fix
  complete. Dev-only dependency.
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
