# 05 — CUDA Backend

Directory: `src/backends/cuda/` (compiled as the runtime-loadable shared library `uipc_backend_cuda`). Entry point `entrance.cpp` → `cuda::SimEngine` (`sim_engine.h` + `engine/*.cu`). The GPU id comes from the engine config `gpu/device`.

## Internal Build Components

The backend remains one DLL and performs one final CUDA device-link. Its 198
compiled `.cu/.cpp` files are owned by seven primary logical components plus
one optional legacy-collision component:

| Logical component | CMake OBJECT target | Owned domains |
|---|---|---|
| `runtime` | `cuda_runtime_objects` | Root/engine, global geometry, animation, time integration, pipeline, diagnostics, and orchestration |
| `affine_body` | `cuda_affine_body_objects` | ABD state, constitutions, constraints, joints, and subsystem assembly |
| `collision` | `cuda_collision_objects` | Default broad phase and current trajectory filters |
| `collision_legacy` | `cuda_collision_legacy_objects` | Optional V0, stackless, and linear-BVH simplex trajectory filters |
| `contact` | `cuda_contact_objects` | Active set, contact, DyTopo, distance, and inter-primitive effects |
| `fem` | `cuda_fem_objects` | FEM data, constitutions, constraints, MAS, and subsystem assembly |
| `linear_system` | `cuda_linear_system_objects` | Global sparse assembly, PCG, SpMV, and preconditioners |
| `coupling` | `cuda_coupling_objects` | ABD/FEM coupling receivers and subsystems |

`components.cmake` and `components.lua` are matching ownership manifests. Both
reject a compiled source that is unowned or assigned twice. CMake realizes the
partition as internal OBJECT targets. XMake attaches the same logical groups
directly to `uipc_backend_cuda`: its built-in CUDA device-link scans source
batches owned by the final target and does not include CUDA OBJECT dependencies.
Both paths use RDC and resolve all device symbols once in the final module. This
preserves static `REGISTER_SIM_SYSTEM` objects without turning domains into
runtime DLL boundaries.

CMake's Visual Studio generator has a language-detection limitation: when every
`.cu` reaches the final DLL only through OBJECT expressions, it skips
`nvcc -dlink` even with `CUDA_RESOLVE_DEVICE_SYMBOLS=ON`. CMake therefore
generates one comment-only `uipc_cuda_device_link_anchor.cu` in the binary
directory and attaches it directly to the final target. It contains no runtime
code and is not a ninth domain; it only makes multi-config generators schedule
the same single device-link that Ninja and XMake already perform.

`UIPC_WITH_CUDA_LEGACY_COLLISION` (CMake) /
`cuda_legacy_collision` (XMake) defaults to ON for compatibility. Turning it
off omits the three legacy filter translation units and their static
registrations, leaving 195 compiled backend sources. Core receives the same
build capability and removes those selectors from `Scene::config_schema()`, so
configuration cannot advertise a filter missing from the backend DLL. The
`wrecking_ball` example now follows the default filter and is included only in
CUDA-enabled builds; examples no longer create a hidden dependency on the
legacy component.

## Subsystem Directories

| Directory | Responsibility |
|---|---|
| `engine/` | Pipeline orchestration: `sim_engine_do_init/do_advance/do_retrieve/do_sync`, etc. |
| `pipeline/` | Pipeline action (SimAction) definitions |
| `global_geometry/` | Global vertex management (GlobalVertexManager), bounding boxes |
| `affine_body/` | ABD dynamics (97 files): Jacobians, mass/energy assembly, joints, state accessors |
| `finite_element/` | FEM (124 files at the audited revision): gradient/Hessian assembly kernels for each constitution (Fem3D/Codim2D/Codim1D families) |
| `collision_detection/` | StacklessBVH, simplex distance, global trajectory filter, DCD/CCD candidate detection |
| `contact_system/` | IPC / AL-IPC contact models (symplectic/implicit), normal/tangential forces, CFL condition |
| `distance_system/` | Distance computation |
| `active_set_system/` | Contact active set |
| `dytopo_effect_system/` | Dynamic-topology effects (friction and other coupling terms) |
| `inter_primitive_effect_system/` | Inter-primitive effects (soft stitching, etc.) |
| `joint_dof_system/` | Joint DOF system |
| `coupling_system/` | Multi-body coupling |
| `linear_system/` | Linear solves: GlobalLinearSystem, fused PCG, SpMV, preconditioners |
| `line_search/` | Line-search energy evaluation |
| `newton_tolerance/` | Newton convergence criteria |
| `time_integrator/` | Time integration and velocity update (BDF1, etc.) |
| `animator/` | Animation stepping |
| `external_force/` | External force computation |
| `diff_sim/` | Differentiable simulation |
| `implicit_geometry/` | Implicit geometry (HalfPlane, etc.) |
| `sanity_check/` | GPU-side checkers |
| `cuda_device/` | Device management |
| `algorithm/`, `utils/` | Shared algorithms and utilities |
| Root-level files | `sim_engine.h/.cpp`, `sim_system.h`, `sim_action*.h`, `sim_engine_state.h`, `energy_component_flags.*`, `kernel_cout.*`, `type_define.h` |

## advance Pipeline (`engine/sim_engine_do_advance.cu`)

```
Pipeline
├── Rebuild Scene                          (includes conditional branch Update Diff Parm)
└── Simulation
    ├── Clear External Forces              (conditional)
    ├── Step Animation                     (conditional)
    ├── Compute External Force Accel.      (conditional)
    ├── Detect DCD Candidates              (conditional)
    ├── Newton Iteration                   (LOOP)
    │   ├── Compute DyTopo Effect          (conditional: Assemble → Convert → Distribute)
    │   ├── Solve Global Linear System     (Build → PCG: Apply Preconditioner + SpMV)
    │   └── Line Search
    │       ├── Detect Trajectory Cand.    (skipped while k > 0 reuses candidates)
    │       ├── Compute Energy             (initial E0)
    │       ├── Filter CCD TOI
    │       ├── Compute CFL Condition
    │       └── Line Search Iteration      (LOOP: Filter Contact Cand. → Compute Energy)
    └── Update Velocity
```

Only the pre-Newton pass runs DCD detection. Iteration `k > 0` starts from the
active set that the last line-search trial of iteration `k - 1` filtered from
that iteration's trajectory candidates: the trajectory pass covers the swept
segment `x_k -> x_k + alpha*dx_k`, which contains the accepted position, so a
DCD broad phase there would only repeat it. `step_forward` and the EE/PT
trajectory leaf predicates form `alpha*dx` with the same non-FMA rounding
(`utils/step_displacement.h`), so the accepted position lies inside the tested
swept boxes. Within a frame the trajectory BVH keeps the topology it has at
the frame start (each frame's first detect requests a rebuild, which the
filter honours every eighth time, see below) and is only refitted: a refit
recomputes every node box from the leaves (and falls
back to a build when it cannot refit), and the candidate set does not depend
on the topology, so an in-loop rebuild would change only the pair order. The
trajectory filter also refits when it refreshes its stored pairs (see below).

Iteration `k > 0` also skips the trajectory pass while the candidates of an
earlier pass of the same frame still cover its step. Each Newton-loop
trajectory pass widens the leaf tests of every family (the FP32 swept bound and
the FP64 swept-box test) by a pair-distance skin `s = 0.25 * min d_hat`
(`CandidateSkinRatio`, `global_trajectory_filter.cu`); the boxes need no share,
because each grows by its primitive's full `d_hat + thickness` and so keeps a
pair `d_hat` of slack over the leaf tests. `SimEngine` keeps a per-axis bound
of the distance from the iteration's start point to the detected swept
segment: zero after a detecting iteration (the accepted position lies on that
segment), plus `alpha * max|dx|` for every reusing one. While
`2 sqrt(3) (drift + max|dx_k|) <= s`, `GlobalTrajectoryFilter::reuse_candidates`
keeps the candidates: a pair they lack stays farther than `d_hat + thickness`
apart over the step, so no trial activates it, and ACCD would report no hit for
it, since `s <= d_hat / 2` keeps its first advancement step above the initial
`toc` of 1.1; a held pair that a fresh pass would drop is inert for the same
reasons. Filter TOI therefore returns the same minimum bit for bit and the
active sets are the same multisets, possibly in another order. The `max |dx|`
readback is shared with the velocity tolerance check
(`GlobalVertexManager` caches it until the displacements are collected again).
Iterations
with small steps reuse, typically the last ones of a frame (the converged
step is below `velocity_tol * dt`). The skin trades skipped passes against
longer candidate lists: a larger skin lets more iterations reuse, but every
trajectory pass then stores and tests more pairs.
`frame_stats()["trajectory_reuses"]` counts the reusing iterations of a frame. Every filter must honor
`DetectInfo::skin()` (`TrajectoryFilter::do_supports_detect_skin`): the info
stackless BVH filter widens its tests, the half-plane filter keeps no
candidates, and with a legacy simplex filter every iteration detects.

The same argument crosses the frame boundary. After a completed Newton loop
`GlobalTrajectoryFilter::record_active_set()` copies the vertex positions, at
which the last line-search trial ran `filter_active`. At the next frame start
`reuse_active_set()` skips the pre-Newton DCD pass when those positions are
unchanged (one compare kernel and one flag readback) and keeps that active
set; iteration 0's trajectory detect then makes the topology-rebuild request
that the DCD pass would have made. The compare is by value: a NaN counts as a
change and signed zeros compare equal, because the half-plane vertex reporter
rewrites its fixed plane points (read by no filter) every frame, restoring a
`-0.0` coordinate that `step_forward` had turned into `+0.0`. So every frame
requests a topology rebuild. The stackless BVH filter honours every eighth
request (`TopologyRebuildPeriod`)
and otherwise refits the kept trees and keeps the Morton orders of its
queries: the candidate multisets do not depend on the topology, which only
fits the moved boxes looser until the next rebuild; the pair order does.
Prediction,
animation and external forces write `x_tilde`, aim targets and forces, not the
vertex positions, so on a plain frame sequence every frame after the first
reuses. The record serves one frame only and is dropped when a frame throws,
by `require_discard_friction()` (state-accessor position or transform writes
through `update_attributes`) and by `apply_recover`; any other position change
fails the compare. The other inputs of the active set (thickness, `d_hat`,
contact and subscene ids and tabulars, surface topology) are uploaded only at
init. The reused active set holds the same PT/EE/PE/PP/PH multisets as the DCD
pass, possibly in another order, so contact sums at iteration 0 can differ only
in summation order. `frame_stats()["active_set_reused"]` reports the decision.
For validation, `UIPC_CHECK_ACTIVE_SET_REUSE=1` (read when the systems are
built) still runs the DCD pass on every reusing frame, continues from its
result and throws a `SimEngineException` unless the sorted multisets match;
the `frame-start active-set reuse` backend test runs in this mode and also
checks that state-accessor writes and a recover force the DCD pass.

The default simplex filter and
the easy vertex half-plane filter take their TOI minimum inside their TOI
kernels (`collision_detection/filters/toi_min.h`) instead of a
`DeviceReduce::Min` after them (two kernels and two host dispatches): each
warp takes `atomicMin` of an order-preserving 64-bit key into the filter's
slot, and the last block to finish writes the minimum and resets the slot.
The half-plane filter has one TOI launch, whose last block finishes. The
simplex filter's four launches (`filter_toi_k1-k4`: PP, PE, PT, EE) all count
their blocks into the slot (`TOIMinSlot::blocks`, the sum of their grids), so
the last block of all of them finishes. The minimum of
values without NaN is unique up to the sign of zero, so the result has
`DeviceReduce::Min`'s bytes in any arrival order (`TOI minimum` backend test,
with the launches in stream order).

Note: a parent timer's duration **includes** its child timers (e.g. when Newton Iteration takes 80%, PCG/Line Search are already counted in it). The actual hierarchy is defined by the runtime output `timer_frames.json` (it changes dynamically with the enabled features).

### Line-search energy aggregation

Top-level line-search reporters do not download independent scalar totals.
`LineSearcher` assigns one contiguous device slot to every ABD, FEM, or DyTopo
reporter. Reporters launch their component reductions asynchronously and write
their final value into that slot. A final CUB sum reads the reporter slots and
writes to a distinct aggregate slot; one contiguous D2H transfer then returns
the individual values plus the total. This preserves reporter-specific finite
checks and diagnostic output while reducing the normal energy pass to one host
synchronization.

ABD and FEM still require separate reductions for their physical energy
components, but named one-thread combine kernels add those device scalars
without an intervening host read. DyTopo's CUB reduction writes directly into
its assigned slot. Keep every CUB input and output range disjoint, including
the final reporter-total reduction.

`Engine::frame_stats()` / Python `Engine.frame_stats()` is the stable
machine-readable complement to the Timer tree. CUDA returns schema v1 with the
pipeline, completion/convergence flags, Newton count, cumulative line-search
trials, cumulative iterative-linear-solver iterations, limit-hit flags, the
last line-search/CCD/CFL step factors, and whether the frame started from the
previous frame's active set (`active_set_reused`, IPC pipeline only) and how
many Newton iterations reused trajectory candidates (`trajectory_reuses`).
`reset_frame_stats()` runs immediately
after incrementing the frame; `completed` is set only after the end-of-frame
iteration checks. `GlobalLinearSystem::last_solve_iterations()` transfers each
solver's `SolvingInfo::iter_count` into the per-frame accumulator. Keep this
contract host-only and cheap: profilers call it once per measured frame.

During `do_init/do_advance/do_retrieve/do_sync`, the callbacks each sim system registered via `SimActionCollection` (`on_init_scene/on_rebuild_scene/on_write_scene`) are invoked in sequence.

### DyTopo single-receiver fast path

`GlobalDyTopoEffectManager` normally assembles raw doublets/triplets, converts
them to sorted unique BCOO storage, and then classifies/copies a subrange for
each `DyTopoEffectReceiver`. There is a strict shortcut when all of these hold:

- exactly one receiver is registered;
- it reports a diagonal gradient/Hessian range beginning at global vertex 0;
- its gradient, Hessian-row, and Hessian-column ranges are identical and lie
  within the global vertex array.

That range is the complete dynamic vertex prefix. Global vertices after it can
be non-DOF implicit geometry (for example, half planes); their contact kernels
differentiate only the dynamic vertex. The manager therefore passes the raw
assembled views directly to the receiver. FEM and ABD subsystem assembly is
linear, and the final `GlobalLinearSystem` `ge2sym` + `convert` pass already
sorts and reduces the global triplets, so the intermediate conversion is
mathematically redundant. Do not broaden this path to multiple receivers or
non-diagonal ranges: mixed ABD/FEM coupling relies on the existing classifier
to route diagonal and off-diagonal blocks to different subsystems.

The diagram above describes the IPC path. `advance_al.cu` implements a separate
AL-IPC active-set pipeline, but both paths call
`SimEngine::step_animation_and_external_forces()` immediately before
`predict_dof()`. The shared frame hook deliberately orders the operations as
**clear prior device force buffers -> run user animation -> consume the freshly
animated forces**. Keep orchestration that is common to both contact
constitutions in this hook and retain focused IPC/AL tests; the Newton, contact,
CCD, and termination algorithms remain intentionally distinct. Timer enablement
and reporting are caller-controlled--neither advance path changes the
process-global Timer state or prints a report implicitly.

FEM vertex forces and ABD body forces consume the dedicated
`external_force/is_constrained` attribute. Only legacy geometries without it
fall back to the shared `is_constrained` mask. Both constraint reporters resize
their force and index buffers to the active count on every step, including zero;
otherwise disabling the last force would leave stale contributions on the GPU.
`python/tests/test_external_force.py` covers default activation, soft-constraint
independence, legacy fallback, and disable/re-enable through both IPC and AL-IPC.

AL-IPC follows a nested solve boundary that must not be collapsed into the IPC
line search. The unconstrained/penalty iterate may penetrate, so its energy line
search starts at one without the IPC CFL clamp. A backtracked step continues the
same fixed-multiplier inner solve. Only a full step updates the multipliers,
runs CCD from the last collision-free state to the iterate, and advances that
safe state by a finite step clamped to `[0, 1]`.

AL's cumulative safe-path check consumes
`newton/semi_implicit/{enable,K_min}`. The initial-state weight remains one
through the first `K_min - 1` completed outer updates, is attenuated by each
later `(1 - CCD alpha)`, and is compared with
`contact/al-ipc/toi_threshold`. `beta_tol` remains IPC-only;
`newton/min_iter` is the separate hard floor on both paths. A failed AL line
search restores both vertex and reporter state to the recorded start point;
never allow the final rejected trial to become the next frame's state.

`GlobalActiveSetManager` implements the per-vertex earliest-TOI filter from
AL-IPC Algorithm 3. Existing active pairs are excluded when computing each
vertex minimum. PT and EE candidates are mapped through the global surface
topology, then retained when their TOI matches at least one incident minimum.
`Float` is double in this project; never port the fork's `int*`/
`__float_as_int` atomic minimum. The implementation uses a 64-bit CAS minimum
and persistent 1.5x-grown scratch buffers. CUB radix sort is stable, so old
pairs precede duplicate new pairs and keep their lambda/decay state; do not
restore the old cross-thread `sort_idx(i-1)` rewrite.

Inactive constraints increment a non-negative decay count, active constraints
reset it to zero, and a pair is removed at the first count for which
`pow(decay_factor, count) < 0.01`. The removal limit is derived from the scene
factor instead of the historical hard-coded 25. The default
`mu_scale_mode="per_vertex"` uses the FEM/ABD mass-based estimators. The
experimental `"diag_norm"` mode assembles the non-contact system and fills all
vertex penalties with `mu_scale_diag_norm * max_i(abs(H_E(i,i)))`; this
uniform scale produced repeated non-descent steps in the mixed cloth/FEM case
88 and must remain opt-in until a heterogeneous conditioning strategy is
validated.

AL simplex friction uses PT and EE-specific tangent Jacobians. Keep the EE
energy, gradient, and Hessian on `edge_edge_jacobi`; substituting
`point_triangle_jacobi` gives a finite but physically wrong direction.
Standard IPC evaluates parallel EE normally with the positive `1e-3`
mollifier coefficient; its normal path implements the mollified energy,
gradient, and Hessian, while its friction path skips detected parallel pairs.
AL-IPC passes the shared negative disabled threshold in both normal and
frictional contact. Its `need_mollify()` result is therefore always false, and
every AL pair follows the corresponding ordinary EE path.

## Cross-domain performance measurement

Use the [portable workload guide](../docs/development/testing_and_benchmarks.md)
to select small, standard or stress scenes and collect fresh before/after runs
on the same machine and configuration. Inspect structured iteration counts and
final observables alongside wall-time distributions; collect synchronized
Timer diagnostics separately. Bottlenecks depend on the scene and environment.

The [2026-09-01 measurements](performance/2026-09-01-cross-domain-baseline.md)
and older numbers below are historical evidence for their recorded machines,
revisions and frame windows. They prescribe neither a GPU model nor a community
speed target. Capture a runner's own repeated measurements before setting any
local performance gate.

## Sparse linear-system storage: BCOO is the active solve path

The production CUDA solve path does **not** currently solve a BSR matrix.
`GlobalLinearSystem` assembles 3x3 block triplets, canonicalizes them to the
symmetric upper triangle with `MatrixConverter::ge2sym`, and reduces duplicate
entries into `DeviceBCOOMatrix<Scalar, 3> bcoo_A`. Both the standalone SpMV and
the fused PCG SpMV/dot path call `rbk_sym_spmv*` with `bcoo_A.cview()`.

`DeviceBSRMatrix` and BCOO-to-BSR conversion remain available in `cuda_tool`
for converter coverage and future experiments, but no registered linear solver
consumes a BSR instance today. Do not describe the default solver as BSR. A
future BSR switch must be justified with end-to-end measurements that include
conversion, SpMV, preconditioner assembly/application, memory footprint, and
representative FEM/ABD/contact scenes; an isolated sparse-kernel benchmark is
not sufficient.

## Broad-phase BVH builds

`collision_detection/method = "info_stackless_bvh"` selects the optimized
default broad phase. Its rebuild path is deliberately CUB-only:

- `DeviceRadixSort::SortPairs` sorts Morton codes and primitive IDs into
  separate persistent output buffers; a named initialization kernel produces
  the identity IDs, so there is no sequence primitive or staging allocation.
- `DeviceScan::ExclusiveSum` builds the internal-node offset table.
- The CUB wrappers use `details::cub_temp_storage`, a persistent per-stream
  workspace that doubles capacity only when it must grow. Never replace this
  with per-call `cudaMalloc`/`cudaFree`; those calls serialize the device.
- The same initialization kernel resets the atomic flags, depth counts, leaf
  LCAs, and scene AABB before any parallel builder/reduction can write them.
  Keep those resets outside the consuming multi-block kernels: resetting an
  output from `idx == 0` inside such a kernel is not a grid-wide barrier and
  races blocks that have already published results.

All three LBVH builders order equal Morton codes by their sorted rank when
they compute the split metric, as if the key were `code << 32 | rank`.
Otherwise every tie has metric 0, runs of duplicate codes on dense meshes
degenerate into deep chains, and build and refit (both walk leaf to root) slow
down with the depth. Only the tree shape depends on this, not the set of
overlapping pairs.

The `info_stackless_bvh_v0` selector is a legacy comparison path, while
`stackless_bvh` and `linear_bvh` are alternate broad phases. All three
registered simplex filters live in the `collision_legacy` component (the CMake
target is `cuda_collision_legacy_objects`) and are
available only when the legacy-collision build option is enabled. The V0 and
`stackless_bvh` builders now use the same separate-input/output CUB radix-sort
pattern, named initialization kernels, and persistent scan workspace as the
default implementation. This removes the final production Thrust dependency
from the CUDA backend. The performance numbers above still apply only to the
default selector. `stackless_bvh` and V0 both have direct brute-force
build/detect/query parity coverage; the V0 test rebuilds at two sizes to cover
persistent CUB workspace reuse and growth. The alternate selectors still need
representative benchmarks before making speed claims.

The default simplex trajectory filter batches host-visible counts. It launches
all active broad-phase queries first, gathers the queue counters (four, plus
the cover-tree PT queue below) into a single device array, and downloads them
with one synchronization. If a count exceeds its queue capacity, only that
queue grows and reruns. The PP/PE/PT/EE selection counts use the same
contiguous one-copy pattern; the four lists are compacted in one launch
(`compact_active_lists`, `collision_detection/filters/compact_active_lists.h`:
one block per 512-element tile, each counting its list's earlier tiles itself,
so lists over 16384 elements take four `DeviceSelect::If` calls instead) with
`DeviceSelect::If`'s outputs and counts (`compact_active_lists` test). The
PT and EE lists' filter kernels before it run as one launch (one-warp blocks,
the PT pairs' blocks first), each thread doing what its own kernel did.
`filter_active` is two-phase across filters
(`TrajectoryFilter::do_launch_filter_active` / `do_finish_filter_active`):
`GlobalTrajectoryFilter::filter_active()` lets every filter that supports it
(the default simplex filter and the easy vertex half-plane filter) launch its
work and read back its counts, synchronizes once, and then lets them size
their active lists; the half-plane filter still grows and reruns its query
when the count exceeds the buffer.
Filters without the two phases run
`do_filter_active` whole afterwards. Every line-search trial thus waits once
for its active set instead of once per filter. The public `InfoStacklessBVH::detect/query`
methods remain synchronous for independent callers;
`launch_detect/launch_query` plus `prepare_query_result` are the internal
batching interface. Do not consume a launch-only query result before the count
batch has been synchronized and finalized.

The three tree chains of one detect are independent and run concurrently:
`lbvh_E` (refit or build, then the CodimP-E and EE queries) stays on the
default stream, while `lbvh_T` with the PT query and `lbvh_T_cover` with the
cover PT query run on the filter's two `cuda_tool::SideStream`s. They fork
after the AABB, cover-split and gather kernels and join the default stream
before the count kernel, so overflow reruns, the cover remap and every later
consumer stay on the default stream. `InfoStacklessBVH::build/refit/launch_query`
take the stream that receives all of their work, including the query sort,
value-initialized growth and CUB calls (CUB temporary storage is per stream).
The candidate multisets do not depend on the streams; only the pair order can
change, which already depends on atomic arrival order. The
`info_stackless_bvh side streams` backend test checks that parity for two
forked build/refit-query chains.
The refit launches 128-thread blocks: the occupancy-maximizing block (768
threads) held its SMs through the whole leaf-to-root chain and kept the
concurrently refitted trees off them. Every internal box is the union of its
children's boxes and every bid/cid the merge of theirs, whichever thread
computes it, so the refitted nodes do not depend on the block size.

`InfoStacklessBVH` Other queries (`launch_query`/`query`) run a
warp-cooperative traversal. The per-lane kernel gave each query one lane, so a
launch with few queries (such as the cover queries when the cover is small)
filled a small fraction of the GPU, and the heaviest warps, whose lanes
diverged between node visits and the expensive leaf predicate, set the kernel
time. Now a warp owns `Config::queries_per_warp` queries (0, the default,
picks the largest power of two up to 32 that still leaves 32 warps per SM) and
shares their traversal among its lanes:

- A lane's task is the part of one query's stackless traversal from node `st`
  until the escape chain reaches `end`. When a lane descends into the left
  child of an internal node it donates the remainder (the right child, from
  `refit_right_child`, up to `end`) to a 64-entry per-warp pool while the pool
  has room, and ends its own task at that right child, which is exactly where
  the left subtree's escape chain leads. Idle lanes take pooled tasks. The
  tasks partition every query's traversal, so each (query, node) pair is still
  visited once.
- Leaves that pass the box and node tests go to a 64-entry per-warp queue; as
  soon as 32 are queued, all lanes evaluate the leaf predicate together.
  Accepted pairs are written straight to the output with one warp-aggregated
  atomic per drain, and the counter keeps counting past the output size, so
  the usual count download and retry handle overflow.
- The kernel is compiled with `__launch_bounds__(128, 4)`. Uncapped, the EE
  trajectory predicate takes 182 registers and leaves 8 warps per SM; capped
  at 128 registers (some state spills to the stack frame) 16 warps fit per
  SM; the EE query gains more from the extra warps than it loses to the
  spills.

The candidate multiset is unchanged; only the pair order differs.
`Config::warp_cooperative_query = false` selects the per-lane kernel, which the
`info_stackless_bvh` backend tests use as the reference for the cooperative
one. Self queries (`detect`) keep the per-lane kernel.

The EE and PT queries start only from a vertex cover of the contact tabular
(`update_cover` in the filter). Every enabled contact pair has an element in a
cover `S` of the graph whose edges are the enabled tabular entries, so a pair
of primitives that both lie outside `S` can never form a candidate (subscene
masks only remove further pairs). `S` is a weighted minimum cover per family,
weighted by edge counts for EE and point counts for PT: exhaustive over up to
20 elements, greedy above. Elements without query primitives and elements
with self-contact always join `S`, and primitives whose contact id lies outside
the tabular always query. The tabular is downloaded and `S` chosen again only
when the tabular buffer pointer, its size or a primitive count changes. That
suffices because the backend fixes the mask tabular and the contact ids at
`World::init`: `GlobalContactManager::Impl::_build_contact_tabular` (called
only from `Impl::init`) is the one writer of the mask, adaptive kappa swaps
only the coefficient tabular, and geometries created after init stay pending
(`GlobalVertexManager::Impl::rebuild` is not implemented), so a contact
tabular edited in the scene after init is ignored by the whole backend. The
split of the primitives by `S` (a flag kernel, a `DevicePartition` and a
blocking count download per family) is made once per `S`: it depends only on
`S` and the primitives' contact ids, which the vertex reporters write at init
(the half-plane reporter rewrites the same ids every frame). A feature that
changes contact ids during a simulation must clear `cover_split_ready`.
- EE: the edges of `S_E` query the full edge LBVH (`launch_query`) instead of
  the edge self-join. A leaf edge outside `S_E` is reached only from the query
  side; a pair inside `S_E` is kept from its smaller index. Pairs go through
  the unchanged EE predicate and are stored as `(min, max)`, as the self-join
  stores them.
- PT: the points of `S_P` query the full triangle LBVH, the other points query
  a second LBVH that holds only the triangles of `S_P`. Its pairs are appended
  behind the first ones.

Compacted query and leaf ids are mapped back inside the leaf predicates and by
a remap pass, so the candidate lists keep their formats and hold the same
multiset of pairs as the original queries; only their order changes. A family
keeps its original query when its `S` holds at least 60% of its query
primitives, and both keep it when the tabular is missing, not square or not
symmetric.

Most detects judge stored pairs instead of traversing the trees. A refresh
grows every box (the four primitive families and the gathered cover boxes) by
a skin, refits or rebuilds the trees on the grown boxes and runs every query
with the leaf predicates reduced to their position-free checks (contact and
subscene masks, shared vertices, body self-collision, the cover side rule).
The resulting (query, leaf) pairs are kept per family. A later detect checks
that every current box lies inside its grown copy and judges each stored pair
as the traversal judges a leaf: the current boxes overlap, then the leaf
predicate's geometric tests. The node predicate (an internal node's ids are
the common ids of its leaves, so its culls are implied) and the position-free
checks passed when the pair was stored and depend only on ids and the tabular,
whose change refreshes the stored pairs, so the judge skips them
(`skip_position_free` of the EE and PT predicates). Each list's judge launch
runs one 128-pair tile per block and writes the tile's kept count. The cover EE
predicate calls the self-join predicate (a called function, not inlined) from
one of two call sites by the edge order, so a warp whose lanes order their
edges both ways runs it twice; the cover EE list's judge therefore queues each
tile's box-overlapping pairs by call site and calls the same compiled predicate
from the same two sites in whole warps, which gives the flags and counts of the
one-pass judge. In a trajectory detect with FP32 inputs that list takes three
launches: a screen (one tile per block, no call) runs each pair's box test and,
on the predicate's arguments, the certain form of the predicate's first FP32
rejection (`swept_lower_bound_f32_certain.h`), writes flag 0 for the pairs
either rejects, zeroes the tile counts and queues the others; a second pass
runs the certain form of the second rejection on those in whole warps, writes
flag 0 for the pairs it rejects and queues the rest by call site; a finish
calls the predicate, FP32 rejection
included, on them from the same two call sites in whole warps and adds their
keeps to the counts. A certain
test holds only where the predicate's test rejects under every FP32 evaluation
of its source (nvcc may fuse any multiply into the add that consumes it, and a
separately compiled copy of the test does disagree with the predicate's on a
few pairs per billion). Both bound the threshold above (any evaluation is
within 10.5u of the real value, u = 2^-24, as all its terms are nonnegative).
The first requires the conditioning tests to pass by a margin and bounds the
predicate's bound below by forward error bounds of the projections, gaps and
normal (sums of products within gamma_k of their magnitudes, plus one rounding
per operation), taking h as 0 unless its own test passes in every evaluation.
The second bounds the gap along every direction that `segment_closest_dir` may
return: each of its three branches (t kept, t clamped to 0 or 1) puts the
direction near a segment of directions affine in s, the branches that no
evaluation can take are left out by a range of t, and the two interval gaps
times |w| are concave in w, so their least values over a segment lie at its
ends. So every flag the screen does not set comes from the predicate's compiled
code and the flags and counts are again those of the one-pass judge. Then one scan launch (one block, 16 consecutive tiles of the
five lists' tile array per thread) turns the counts of all five lists into
tile offsets and writes the counts and the containment flags into the query
count array, and one scatter launch (each warp looks at about four tiles spread
over the array at once) writes the kept pairs of each tile that keeps any in
order, as `DeviceSelect::Flagged` would. The judged lists keep the query format, so the
cover remap rewrites them as before. Two boxes that overlap lie in overlapping grown
copies, so every pair the queries would judge is stored and the candidate
multisets equal those of the queries; only their order changes. The judged
lists are written before the containment flags arrive (they ride in the count
download, so a detect keeps one synchronization); a box outside its grown copy
discards them and refreshes. So do every topology-rebuild request
(`reuse_bvh_topology() == false`, which every frame's first detect makes),
honoured or not,
a refreshed cover split and a changed primitive count. Stored pairs need a
symmetric contact tabular: the edge self-join reaches a pair from its lower
Morton rank, and only a symmetric node predicate makes that end irrelevant;
without one the filter queries every detect.

Each box is checked against its own grown copy, so the skin may differ per
primitive. Without a contact cover every primitive takes `d_hat`
(`StoredSkinOverDHat`). With one, the cover's primitives (the ones that query)
take `d_hat` or, for a frame after one in which some detect found one of their
boxes outside its `d_hat` copy, `2 d_hat` (`StoredThickSkinOverDHat`); the
others take `0.01 d_hat` (`StoredThinSkinOverDHat`) or, after a frame in which
one of them left that, `d_hat`. The containment kernel flags both cases in the
same download as the escape (the box against its copy shrunk to its group's
lower level), and a refresh caused by an escape raises the level of the group
that left it for the rest of the frame. A thicker skin stores more pairs and
refreshes less often. When the cover's primitives are the moving bodies and
the others mostly kinematic or static geometry, the static boxes keep a thin
skin and the moving ones a thick one, so fewer boxes leave their grown copies
than with `d_hat` for every box. A frame refreshes at its first detect and
otherwise only when a later Newton step leaves the stored boxes behind (the
Newton loop refits for the whole frame, so no mid-frame rebuild forces a
refresh); the other detects skip the traversal.

While the cover split stays (`cover_unchanged()`: the same tabular, primitive
counts and a ready split), a detect writes its boxes, cover gathers, FP32
inputs and containment flags in two launches instead of up to eleven
(`launch_trajectory_prefix`, `collision_detection/filters/trajectory_prefix.h`).
A per-vertex kernel rounds each position (the boxes' start point and the FP32
input), the step displacement and `pos + dx * alpha` (the boxes' end point,
with the same contraction as `detect_k1-k4`) to FP32 once per vertex and
clears the escape flags. One kernel over all four primitive families (a block
range each) then builds each box from those FP32 points with the FP64 `d_hat`
and thickness expansion of `detect_k1-k4`, extending in their order, writes it
with its bid and cid, copies them to the box's slot of its family's gathered
cover list (each split keeps `slots`, the inverse of `ids` over the gathered
range) and, when the detect judges stored pairs, checks the box against its
grown copy as the containment kernel does. The outputs equal the separate
kernels' byte for byte (`trajectory prefix` backend test). A detect that
refreshes the split (the first one, or after a tabular or count change) needs
this detect's contact ids for the split, so it keeps the separate kernels.

The PT and EE leaf predicates of this filter reject far candidates with
conservative FP32 lower bounds before their FP64 tests. In the DCD pass
(`alpha == 0`), `utils/distance/float_lower_bound.h` bounds the distance from
below on coordinates made relative in FP64 (plane plus outermost edge
half-plane for PT, common normal plus edge-axis projections for EE) and reports
"farther" only when the bound exceeds `d_hat + thickness` by
`0.1% + 1e-4 * local coordinate scale`; slivers, degenerate edges and
near-parallel common normals skip the bound. Such a pair fails the FP64 test as
well, so the DCD candidates do not change.

In the trajectory pass (`alpha > 0`), `utils/distance/swept_lower_bound.h`
rejects a PT or EE pair before the FP64 swept box test when it stays apart for
the whole step. A point with fixed barycentric weights on either primitive
moves by a convex combination of its vertex displacements, so the distance
shrinks during the step by at most `M = max |dx_a - dx_b|`. The pair is rejected
when a start-distance bound clears both `d_hat + thickness + M` and
`thickness + kAccd * maxDispMag`, where `maxDispMag` bounds ACCD's displacement
magnitude from above and `kAccd = 1.1 / (1 - 0.1) * 1.005`. The second term
makes ACCD's first step exceed the initial TOI of 1.1 (`large_enough_toi`) for
every `eta <= 0.1` (`toi_safety_margin`: 0.1 for IPC, 0.001 for AL-IPC), so
`filter_toi` would return no hit for the pair. The start-distance bound is the
DCD bound above or, as a fallback for slivers and near-parallel edges, the gap
of both primitives projected onto an approximate FP32 closest direction (any
unit axis gives a lower bound). The active set and the TOI therefore do not
change; only the candidate list shrinks. `kAccd` is tied to `large_enough_toi`
and to `eta <= 0.1` by this argument only; changing either requires revisiting
it. The `float_contact_lower_bound` backend test checks the DCD and swept
bounds in device code.

Before those, every trajectory detect runs the two swept rejections on FP32
inputs (`utils/distance/swept_lower_bound_f32.h`), so that a far pair costs no
FP64 work: each vertex's position (with its `d_hat` in w) and step
displacement (with its thickness in w) is rounded to FP32 once per detect
(`f32_positions`, `f32_steps`). The displacements equal the FP32 casts above,
so `M` and `maxDispMag` are unchanged. The positions are absolute, which moves
each relative coordinate by at most 2 ulp of the pair's largest coordinate
magnitude `X` and the distance between the two primitives by less than
`2^-20 X`; the rejection margin adds that term. A pair it rejects therefore
satisfies both conditions above. It may still be one the FP64-input bound
keeps (the two bounds round differently): such a pair is inert by those
conditions, so it leaves the candidate lists without changing the active set
or the TOI. Pairs it keeps take the FP64-input path unchanged, so the
candidate lists equal the former ones minus such pairs, in the same order.

The MAS preconditioner also performs its hierarchy prefix scans through
`cuda_tool::DeviceScan`. These wrappers share `details::cub_temp_storage`, so
their temporary device memory is retained per CUDA stream and only grows when
required. New CUB calls must use these wrappers (or an equally persistent owned
workspace), never allocate/free temporary storage inside a Newton iteration.

## Kernels and Kernel Naming

All kernels are **named `__global__` functions** (defined in anonymous namespaces), launched via bare `<<<grid, block, 0, stream>>>`. Kernel names follow the `<owning class>_<original function>[_kN]_kernel` convention, so they are directly readable in ncu reports:

```
InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel
FEMLineSearchReporter_step_forward_kernel
```

- Launch block size is auto-selected by occupancy via `cuda_tool::best_block_dim(kernel)` (using the existing occupancy-based launch policy; atomic arrival order can still vary); the grid is computed by `cuda_tool::best_grid_dim(n, kernel)`. **The block-size cache is keyed by kernel function address** (`unordered_map<const void*, int>`); it must not be a `static` cache keyed by function-pointer type — different kernels with the same signature would share a cache entry, and cross-kernel block-size interference perturbs the atomic reduction order, which once caused a case 36 frame 17 line-search failure in the full test suite (not reproducible when run in isolation).
- Exception: the CCD and active-set filter kernels (`InfoStacklessBVHSimplexTrajectoryFilter` `filter_active_k1`, `k2` and `k34` (PT and EE in one launch) and `filter_toi_k1-k4`, `GlobalContactManager`'s `compute_feasible_step_{PT,EE,PE,PP}`) launch one-warp blocks (`cuda_tool::spread_block_dim` = 32, `spread_grid_dim(n)`). Each thread runs a long FP64 computation (distance, CCD) on lists of a few hundred to a few thousand pairs, and GB20x issues FP64 at 1/64 of the FP32 rate per SM, so the occupancy block size (384-768 threads) left them on a handful of SMs. The per-thread code is unchanged and the outputs are bitwise identical (`ccd_launch_shape` test); the TOI minimum is exact (`toi_min.h`).
- Device fills and copies use the `cuda_tool` buffer/stream APIs; fill
  kernels appear as `details::buffer_fill_kernel<T>` in profiler output.
- `_shorten_kernel_name()` in `python/src/uipc/profile/nsight.py` was originally used to extract the outer function name from `parallel_for_kernel<Lambda>`; with named kernels this proxy is no longer needed (the symbols are readable as-is).

### Resource-aware contact and FEM assembly

`StableNeoHookean3D_do_compute_gradient_hessian_kernel` projects derivatives
to vertex space without constructing dense `dF/dx` or a full `12x12` element
Hessian. `fem_utils.h` provides the tetrahedron shape gradients plus direct
`B^T g` and `B_l^T H B_r` helpers. The kernel writes four gradient doublets and
the ten upper-triangular `3x3` Hessian triplets directly, preserving the global
index ordering used by `TripletMatrixAssembler::half_block`. The dense and
block formulas have backend regression coverage. On RTX 5090 this reduced the
kernel stack from 6440 to 1320 bytes/thread and Nsight Systems time from 1.795
to 1.047 ms/call.

The IPC simplex normal/friction assembly kernels deliberately keep PT, EE, PE,
and PP in one launch. Although splitting by stencil lowers each specialized
kernel's static stack frame, PT/EE Hessian evaluation is expensive even for a
few contacts. The fused launch overlaps those threads with the much larger PE
population; four sequential launches regressed case-88 DyTopo assembly by
about 70%. The uniform `gradient_only` decision is compile-time specialized
instead, giving lightweight gradient variants without changing the full
Hessian launch topology. Resource counts alone are therefore insufficient:
validate launch restructuring with both kernel profiles and the enclosing
stage timer.

## FusedPCG CUDA graph modes (linear_system/use_cuda_graph, default 1)

The per-iteration kernel chain of `LinearFusedPCG` (spmv_dot → update_xr →
preconditioner → dot → converged → update_p → swap_rz) is ~10 tiny launches
whose launch gaps dominated the wall time (~80 µs of ~118 µs per iteration on
case2-scale scenes). Two graph modes exist; `graph_mode` is auto-selected in
`do_build` (logged at info level):

- **mode 2 — full-GPU while-loop** (opt-in, toolkit AND driver ≥ CUDA 12.4):
  the whole solve is ONE graph launch — setup chain (reset → r=b → precond →
  p=z → rz=rᵀz → rz_tol=tol·|rz0| on device) → WHILE conditional node whose
  body is one iteration + a control kernel that publishes convergence and
  calls `cudaGraphSetConditional`. Exit happens at the exact convergence
  iteration; zero D2H/H2D inside the loop; a single D2H at the end reads
  back the iteration count. **Measured SLOWER than mode 1 on case2-scale
  scenes (median ~226 vs ~203 ms/frame): the WHILE node's per-iteration
  evaluation costs more than the amortized host check every 5 iterations —
  so it is opt-in (`use_cuda_graph=2`), useful when freeing the CPU matters
  more than raw frame time.**
- **mode 1 — block replay (default)**: `check_interval` iterations captured
  per block via `cuda_tool::GraphCapture`, host convergence check between
  blocks (same cadence as the plain loop).
- **mode 0** — plain launches (config off, or `contact/constitution != ipc`).

Shared infrastructure: `cuda_tool/graph.h` holds `GraphCapture` (flat block
capture) and `GraphWhile` (WHILE-loop assembly via conditional nodes);
instantiation API is dispatched by `CUDART_VERSION` (WithFlags ≥11.4 through
13.x, legacy 5-arg below), and `GraphWhile` additionally runtime-checks the
driver (≥12.4 for conditional nodes).

Graph-stability design (no rebuilds from contact-pair fluctuation):
- The SpMV triplet count lives on device (`triplet_count_dev`, uploaded once
  per assembly) and the launch grid is sized by the reserved triplet
  CAPACITY; the kernel guards with the device count and reads the triplet
  arrays through raw pointers (`A.row_indices().data()` etc.) — the view's
  `A(i)` accessor asserts against the capture-time count and must NOT be
  used for this.
- rz_tol is device-side; the validity key is buffer pointers + N + max_iter.
  A matrix realloc (new pointers) still forces a rebuild.
- Stream plumbing: `Spmv::rbk_sym_spmv_dot`, `GlobalLinearSystem::Impl`
  `spmv_dot/apply_preconditioner`, `IterativeSolver` pass-throughs,
  `ApplyPreconditionerInfo::stream()`, and the ABD/FEM diag preconditioners
  all accept an optional launch stream (default = legacy default stream =
  unchanged behavior). The MAS preconditioner is plumbed too:
  `MASPreconditionerEngine::apply` and its three phases (restrict /
  Schwarz local solve / prolongate) plus the unpartitioned-vertex diagonal
  fallback in `FEMMASPreconditioner::do_apply` all launch on
  `ApplyPreconditionerInfo::stream()`, so MAS scenes join the captured
  graph instead of invalidating it. The engine's workspace buffers are
  sized once at init from the mesh partition (contact does not recluster —
  collision-aware clustering is deliberately not ported), so the captured
  pointers stay valid as contact nnz fluctuates. The partition hierarchy is
  likewise constructed once in `init_matrix()` and reused by every Newton
  assembly; only the current BCOO Hessian scatter and cluster inversion are
  repeated. Scatter reads BCOO entries directly (there is no identity-index
  staging buffer). The 48x48 Gauss-Jordan inversion synchronizes after the
  pivot row is ready, but not between independent row updates.
- MAS activation (all-or-nothing, since 2026-08-23): scene config
  `linear_system/fem_preconditioner = "mas"` (default `"diag"`) selects the
  FEM local preconditioner. When on, `FEMMASPreconditioner::do_init`
  auto-partitions EVERY non-Empty FEM SimplicialComplex internally (fixed
  cluster size = `MASPreconditionerEngine::BANKSIZE` 16 — the only size the
  Schwarz kernels' shared-memory layout accepts) on a private clone, so
  scene geometries are never mutated; a pre-existing `mesh_part` vertex
  attribute (custom C++ partitioning) is respected as-is. Empty-constitution
  geometries are skipped and stay on the internal diagonal fallback. If the
  switch is on but nothing is partitionable, `do_apply` falls back to z=r.
  The python `mesh_partition` export is gone — per-mesh manual tagging no
  longer activates MAS (partial coverage measured net-negative, doc 09).
- `linear_system/check_interval` is now a registered config key (default 5) —
  it was previously unregistered and silently dropped.
- Per-iteration "SpMV"/"Apply Preconditioner" Timer entries only exist on
  the plain path (capture paths create no Timer objects — Timer creation
  during stream capture deterministically fail-fasted the single-process
  suite binary).

## GPU Coding Guidelines (excerpted from review-pr / simulation-dev)

- Buffer parameters use view types such as `cuda_tool::BufferView` / `cuda_tool::TripletMatrixView`; **raw pointers are forbidden**.
- Put index guards at the top of the kernel body; return immediately on out-of-bounds.
- For debugging, use `cuda_tool::debug_sync_all()` as a fail-fast barrier; kernel assertions use `UIPC_KERNEL_ASSERT` (gated by `uipc::RUNTIME_CHECK`).
- Check `isfinite`; add conservative guards for division by zero / square roots of negatives.
- When modifying solvers/kernels, follow the debug loop in `.cursor/skills/simulation-dev/SKILL.md`.

## Performance Analysis Toolchain

- `python -m uipc.cli.benchmark run/profile/analyze/compare/baseline/check`
  (CLI) and `uipc.profile` / `uipc.profile.nsight` (Python API). `run --warmup`
  excludes warmup/recovery from both wall time and the first Timer frame;
  versioned baselines gate wall average plus Timer median/p95 and reject runner,
  build, frame-count, or phase-plan drift by default.
- Workflow: first `run` to get per-stage wall-clock (`report/report.md` + `timer_frames.json`), then `profile` to get per-kernel metrics (ncu), and cross-reference to locate "hot stages + inefficient kernels"; stages taking <5% of frame time are not optimized.
- See `.cursor/skills/gpu-optimization/SKILL.md` for details.

## cuda_tool (in-house raw-CUDA utility library)

`src/backends/cuda/cuda_tool/`, namespace `uipc::backend::cuda_tool`. **The backend's only device utility library** (all muda dependencies have been removed: no vendored copy, no external submodule; Eigen is kept).

| File | Contents |
|---|---|
| `stream.h` | `CUDA_TOOL_CHECK` error checking, `default_stream`, `Stream` (`Stream::Default()`), `SideStream` (owning non-blocking stream ordered against its parent stream only by `fork()`/`join()` events) |
| `view.h` / `view_nd.h` | `CBufferView/BufferView/VarView/CVarView`, `Dense/CDense` (scalar viewers), `ViewerBase`, `Extent2D`, `Buffer2DView`, `Dense1D/Dense2D` (with `make_dense_1d/2d`) |
| `launch.h` | `best_block_dim/best_grid_dim` (occupancy-based block size / grid computation); `spread_block_dim/spread_grid_dim` (one-warp blocks for FP64-heavy per-item kernels over short lists) |
| `buffer.h` | `DeviceVector/DeviceBuffer/DeviceVar/DeviceBuffer2D`, `BufferLaunch`. `resize()` value-initializes growth; `resize_discard()` is for fully regenerated output and grows to 150% of the latest requirement; `resize_preserve()` retains the old logical range without initializing the new range. Exact and amortized preserve/discard reserve variants make ownership and growth policy explicit. `PinnedHostBuffer` is a page-locked host array whose `copy_from` enqueues a device-to-host copy without blocking, so several small read-backs share one stream synchronization (a copy into pageable memory waits for the stream on its own). |
| `cub.h` | Thin wrappers (pointer-style) for `DeviceReduce/Scan/Select/Partition/RadixSort/MergeSort/RunLengthEncode` + warp-level cub headers; **temporary storage uses a stream-level workspace cache** (`details::cub_temp_storage`, grows 2× on demand, reused across calls — per-call cudaMalloc/cudaFree costs ~10-100µs each and implicitly synchronizes with the device, which once caused a ~15%/frame performance regression in the 6_wrecking_balls scene). Each wrapper calls CUB once, on the stream's whole workspace (`details::run_with_temp_storage`): CUB returns `cudaErrorInvalidValue` before it launches anything when the storage is too small, and only then is the size queried, the workspace grown and the call repeated (a separate size query would repeat the dispatch's host work, about 2.5 µs per call). Every call's error is checked, so a wrapper's callable returns CUB's `cudaError_t` |
| `linear_system.h` + `linear_reduction.h` + `linear_system/views.h` | `DeviceTripletMatrix/DoubletVector/DenseVector/BCOOMatrix/BSRMatrix/DenseMatrix` + full set of views (Triplet/Doublet/DenseVector/BCOO) + `LinearSystemContext`; dot/norm use named partial-reduction kernels, persistent geometrically grown partial/result buffers, the per-stream CUB workspace, and scaled-square norm accumulation without cuBLAS |
| `eigen.h` + `eigen/` | Device-side small-matrix math (`eigen::evd/svd/pd/inverse/atomic_add`). The family originated in the MUDA migration; current `svd/pd` use `algorithm/qr_svd.hpp` with signed-SVD fixup and native float/double precision. |
| `debug.h` | `debug_sync_all/check_finite` + the `UIPC_KERNEL_ASSERT/ERROR/WARN` macro family (gated by `uipc::RUNTIME_CHECK`) |
| `graph.h` | `GraphCapture`: self-contained CUDA-graph block capture/replay (capture body on a capture stream, replay via `launch_sync()`; permanent fallback on capture failure, also when the body throws on an invalidated capture; a body that throws on a valid capture rethrows and leaves no instantiated graph). `GraphWhile`: setup chain plus WHILE conditional node, replayed by `launch_sync()`. Instantiation API is selected by `CUDART_VERSION` (`cudaGraphInstantiateWithFlags` ≥11.4 through 13.x, legacy 5-arg form below), so the same source builds across CUDA 11/12/13 |
| `logger.h` | `LoggerViewer` (in-kernel `cout <<`, device printf) + `KernelCout` |
| `atomic.h` | Scalar `atomic_add/atomic_exch` |

- Build notes: requires `--extended-lambda --expt-relaxed-constexpr`; MSVC + CUDA≥13 requires `/Zc:preprocessor`; fmt has a UTF-8 conflict in the nvcc device pass, so cuda_tool uses `std::runtime_error`; Eigen `::arg` needs a global shim (already built into `type_define.h`).
- Use `resize_discard()` only when every live output element is subsequently
  written by a kernel, copy, or CUB primitive. State/history buffers and
  sentinel ranges that rely on zero/default construction must use `resize()`
  or an explicit fill. Subsystems with a tuned reserve ratio call
  `reserve_discard(required * ratio)` before `resize_discard(required)`, so
  capacity is based on the latest requirement without silently changing the
  subsystem's memory budget.
- **The umbrella header `cuda_tool.h` does not include `cub.h`**: the CCCL device-algorithm headers are extremely heavy (~165k extra expanded lines per TU), so files using the `Device*` wrappers explicitly `#include <cuda_tool/cub.h>`; `linear_system.h` does not transitively include it. The separate `linear_reduction.h` is included only by the three dot/norm callers and focused test.
- **RDC must be enabled** (`CUDA_SEPARABLE_COMPILATION ON` + `CUDA_RESOLVE_DEVICE_SYMBOLS ON`): `affine_body/utils.cu` defines `UIPC_GENERIC` free functions (e.g. `q_to_transform`) that are called across translation units by device code in other TUs; disabling RDC gives `ptxas fatal: Unresolved extern`. The xmake-side equivalent is `add_cuflags("-rdc=true")`.
- **Verify flag changes in the generated build.** Ninja tracks command-line
  changes, including compile flags ([manual](https://ninja-build.org/manual.html#_comparison_to_make)).
  Reconfigure and inspect `compile_commands.json` plus verbose build/device-link
  output to confirm the intended flags and targets. Use a fresh build directory
  when investigating suspected stale artifacts; a successful partial build is
  not evidence that RDC can be disabled. Historical handoff claims that Ninja
  ignores flag changes are incorrect; the cross-TU RDC requirement above remains.
- Historical compile diagnostics (32 cores, CUDA 13.2, RTX 5090): the old ~4.7-minute wall figure involved incomplete recompilation and is not a clean-build baseline; a single TU's preprocessed expansion is ~1.63M lines, of which CUDA toolchain headers ~860k, WinSDK ~310k, MSVC STL ~150k, Eigen ~150k, the project itself <20k — the bulk is toolchain headers; the project headers have already been trimmed as far as possible.
- The smoke test is kept as `test_compile.cu.txt`.

## none Backend

`src/backends/none/`: an empty implementation: it increments the frame and logs,
but performs no physics. It is useful as a backend template and for limited core
interface/geometry checks. Initialization enters the Scene pending phase,
and each advance settles queued geometry creation/destruction. Environments
without a GPU have no physical simulation backend.
