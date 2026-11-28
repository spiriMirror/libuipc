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
    │   ├── Compute DyTopo Effect          (conditional: Assemble → Distribute)
    │   ├── Solve Global Linear System     (Build → PCG: Apply Preconditioner + SpMV)
    │   └── Line Search
    │       ├── Compute Feasible Step      (side stream, beside the detection)
    │       ├── Detect Trajectory Cand.    (skipped while k > 0 reuses candidates)
    │       ├── Compute Energy             (initial E0; reused for k > 0)
    │       ├── Filter CCD TOI             (side stream, beside E0 and the first trial step)
    │       ├── Compute CFL Condition
    │       └── Line Search Iteration      (LOOP: Filter Contact Cand. → Compute Energy)
    └── Update Velocity
```

The line search halves `alpha` until the trial energy does not exceed `E0`.
With `line_search/cubic_refinement = 1` (off by default; standard IPC only,
`LineSearcher` logs a warning under AL-IPC) it refines a backtracked step once: the minimizer of the cubic through `E0`, the slope
`g.dx` (`GlobalLinearSystem::newton_step_slope()`, `-b.x` of the solved
system) and the rejected and accepted trials
(`line_search/cubic_line_minimizer.h`) is evaluated when it lies inside the
bracket and differs from the accepted step by more than 25%, and the lower of
the two energies wins (after a worse refinement, `step_to_trial` returns to
the accepted step to restore its state and active set; its energy is known).
It helps when halving alone accepts steps far from the line minimum, for
example when the lagged friction model underestimates the curvature along
`dx` as sliding contacts stick again. It changes the accepted step, so it is
not trajectory-neutral.

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
readback is shared with the serial velocity tolerance check
(`GlobalVertexManager` caches it until the displacements are collected again);
with the prologue overlap the check reduces on its own stream. Iterations
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
requests a topology rebuild (iteration 0 another one when it re-detects after
a truncated feasible step). The stackless BVH filter honours every eighth
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
The same reasoning lets iteration `k > 0` skip its initial energy
evaluation: nothing between the last line-search trial of iteration `k - 1`
and `E_0` of iteration `k` moves a vertex, runs `filter_active`, or changes
kappa or the friction candidates, so `E_0` is that trial's energy. It is
reused while the animation substep ratio is unchanged (with
`scene.animator().substep(n)`, n > 1, the ratio grows during the first
iterations) and recomputed when
`line_search/report_energy` asks for one log entry per evaluation. A converged
trial that terminates the Newton loop still steps and filters the active set
(the next frame records friction candidates from it), but its energy is never
read and is not computed. A reporter that keeps per-Newton state entering
its energy would invalidate this reuse.

### Line-search prologue overlap

The copies that open a line search run as one `cuda_tool::CopyBatch` kernel
on the legacy stream: the displacement copies of the vertex reporters
(`GlobalVertexManager::collect_vertex_displacements(copies)`: half planes
zero theirs through `VertexDisplacementInfo::zero_displacements()`; ABD and
FEM compute theirs with their own kernels before them, see below) and the
start point (`LineSearcher::record_start_point(copies)`: ABD `q_temp = q`,
FEM `x_temps = xs` through `RecordInfo::copies()`;
`GlobalVertexManager::record_start_point(copies)`: `safe_positions =
positions`). None of the ranges overlaps and none is a source of another, so
the batch writes the bytes of the five separate copies, fills and kernels it
replaces. The overloads without a batch (AL-IPC) launch their own.

In the IPC Newton loop the solve defers the ABD and FEM retrievals
(`GlobalLinearSystem::solve(true)`, `SolutionInfo::deferred()`: the subsystem
keeps the solution view and launches nothing): the vertex reporters then
compute the step and the displacements in one kernel each
(`affine_body_vertex_reporter_retrieve_displacements_kernel`: thread i stores
body i's `dq = -x` and vertex i's `J * dq` from the same values;
`FiniteElementVertexReporter_retrieve_displacements_kernel`: `dxs = -x` and
the displacement), the same arithmetic as the two retrieval kernels and the
displacement kernel or copy they replace. Both fold their displacements into
the per-axis maximum |d| (`VertexDisplacementInfo::axis_max()`,
`global_geometry/axis_max.h`: warp maximum, then `atomicMax` on the bits of
the non-negative double, exact in any order, into one of two slots used in
turn; the first taker clears the other for the next collection). When every
reporter took the slot or zeroed its range, `compute_axis_max_displacement()`
and `launch_axis_max_displacement()` publish the slot with one one-thread
kernel instead of the CUB reduction (two kernels); otherwise they reduce as
before. Nothing reads `dq`/`dxs` between the solve and the collection; a
deferred retrieval that was never collected trips an assertion at the next
retrieval. AL-IPC keeps the separate retrievals.

Every trial steps forward the same way: the IPC Newton loop collects the
vertex positions (`GlobalVertexManager::step_forward(StepBatch&)`) and the
reporters' DOFs (`LineSearcher::step_forward(StepBatch&)`, reached through
`StepInfo::batch()`: ABD `q = q_temp + alpha * dq` except fixed bodies, FEM
`xs = x_temps + alpha * dxs`) into one `StepBatch` (`line_search/step_batch.h`)
and launches one kernel instead of three. Each element is computed by the
same expression as in the separate kernels (the positions through
`step_position()`, which never fuses `alpha * dx` into an FMA; the DOFs as
`x0 + alpha * dx`, contracted to DFMA there and here alike), so the bytes are
those of the separate kernels. A reporter that steps without a batch (AL-IPC)
launches its own kernel. A new reporter with DOFs must add them to the batch
when `StepInfo::batch()` is set.

Before its first trial the line search needs the feasible step over the
active set, the trajectory candidates, the CCD TOI of the capped step and the
Newton tolerance checks; each of the short ones ends in a host read.
`SimEngine::advance()` runs the short ones on the line searcher's prologue
stream (`LineSearcher::m_prologue_stream`, a `cuda_tool::SideStream`) beside
the long ones on the legacy stream:

- Right after the start point is recorded,
  `GlobalContactManager::launch_feasible_step(stream)` and
  `NewtonToleranceManager::launch_check(stream)` enqueue the feasible-step CCD
  and the checkers' device work, whose kernels store their results into
  read-back buffers (`cuda_tool::ReadbackBuffer`, mapped host memory: the
  feasible step's minimum TOI, the max-translation residual, and the ABD
  flag through a copy kernel), so no device-to-host copy holds the prologue
  stream between its kernels, while the legacy stream decides whether the
  held trajectory candidates can be kept. `finish_feasible_step()` then only
  waits for the side stream, and the trajectory detection runs over the
  feasible step, as in the serial order (a detection over the full step
  beside the feasible step would change the filter's state, its BVH topology
  requests and stored pairs, whenever the step truncates). A detection calls
  `GlobalTrajectoryFilter::DetectInfo::before_read_back()` right before it
  waits for the read-back of its candidate counts (again when a refresh
  repeats the read-back): the Newton loop forks the TOI's stream there, after
  the detection's work, so the TOI is launched without waiting for that
  read-back. An iteration that keeps the held trajectory candidates (they
  cover any step `alpha <= 1`) detects nothing and only waits for the
  feasible step.
- `GlobalTrajectoryFilter::launch_filter_toi(alpha, stream)` computes the TOI
  from the recorded start positions (`safe_positions`, bitwise the current
  positions until a trial steps) while the legacy stream evaluates `E_0` when
  it is not reused and steps the first trial forward (and its energy
  producers that read only the stepped DOFs are launched, see the energy
  aggregation below). `finish_filter_toi()` waits for the side stream. The
  first trial's `filter_active` runs only after that, once the TOI and the
  CFL condition kept the step: a step beyond the TOI may penetrate, and
  `filter_active` asserts a penetration-free state. When the TOI or the CFL
  condition shortens the step, the launched producers are joined and the
  loop steps to the first trial again; every trial is recomputed from the
  recorded start point and no energy was reduced for the discarded one, so
  it leaves no trace.
- The next `NewtonToleranceManager::check()` waits for the side stream once;
  checkers that implement `do_launch_check()` then only read their copied
  results (`CheckResultInfo::launched()`), the others do all of their work in
  `do_check()` as before.

Every `TrajectoryFilter::do_filter_toi` puts all of its launches, CUB calls
and fills on `FilterTOIInfo::stream()` (or on a stream that it orders after
that stream and joins back into it before it returns, as the simplex
filter's EE launch below) and reads `FilterTOIInfo::positions()`
(the swept step's start positions; the serial `filter_toi(alpha)` used by
AL-IPC passes the current positions on the default stream). Host values reach
device memory on the side stream through fill kernels, because a pageable
host-to-device copy may wait for the stream; the TOI slot itself
(`FilterTOIInfo::toi()`) is a read-back (`GlobalTrajectoryFilter::Impl::tois`,
a `cuda_tool::ReadbackBuffer`), so a host value goes there only through
`cuda_tool::store_value()` (the simplex filter without candidates). The default simplex filter and
the easy vertex half-plane filter take their TOI minimum inside their TOI
kernels (`collision_detection/filters/toi_min.h`) instead of a
`DeviceReduce::Min` after them (two kernels and two host dispatches): each
warp takes `atomicMin` of an order-preserving 64-bit key into the filter's
slot, and the last block to finish writes the minimum and resets the slot.
The half-plane filter has one TOI launch, whose last block finishes. The
simplex filter's four launches (`filter_toi_k1-k4`: PP, PE, PT, EE) all count
their blocks into the slot (`TOIMinSlot::blocks`, the sum of their grids), so
the last block of all of them finishes, and the EE launch runs beside the
other three on a stream of the filter (`toi_side`, ordered after
`FilterTOIInfo::stream()` by an event and joined back into it) instead of
after them (the PT and EE kernels are latency-bound with few blocks and had
run one after the other). The minimum of
values without NaN is unique up to the sign of zero, so the result has
`DeviceReduce::Min`'s bytes in any arrival order (`TOI minimum` backend test,
with the launches in stream order). The overlap is off with
`extras/debug/dump_surface` (a repeated trial would dump twice) and without
line-search trials.

Preconditioner reuse across Newton iterations is **opt-in and off by
default** (`linear_system/keep_preconditioner`, default 0). With the key set
to 1 on the standard IPC pipeline, `GlobalLinearSystem::keeps_preconditioner(k)`
holds for every odd Newton iteration `k >= 3`: that solve keeps the local and
global preconditioners assembled for iteration `k - 1` (same DoF layout; a
layout change always reassembles), so a kept preconditioner is one Newton step
old. Iterations 0 and 1 (the large first steps of a frame) and every even
iteration assemble fresh preconditioners. AL-IPC assembles every iteration
whatever the key, because its multipliers and active set change between
Newton iterations. What it saves: the local preconditioner assembly (ABD
12x12 block inverses, MAS cluster scatter and inversion) on about half of
the iterations from the third on. With the key at 0, `build_linear_system`
assembles the preconditioners in every iteration, on the same path as
without the feature.

Why it is off by default: PCG stops on `|r^T z| <= tol_rate * |r0^T z0|`, in
the norm of its preconditioner. A kept preconditioner can inflate that norm,
so the relative tolerance is met while the free bodies' part of the step is
still about zero; Newton's displacement test (`newton/velocity_tol`) then
passes on that step and the frame ends with momentum lost: falling bodies fall
too slowly, and stacked or swinging bodies lose kinetic energy, with every
frame converged and no iteration limit hit. Enable it only for a scene that
has been checked to move the same with both values.

Note: a parent timer's duration **includes** its child timers (e.g. when Newton Iteration takes 80%, PCG/Line Search are already counted in it). The actual hierarchy is defined by the runtime output `timer_frames.json` (it changes dynamically with the enabled features).

### Line-search energy aggregation

Top-level line-search reporters do not download independent scalar totals.
`LineSearcher` assigns one contiguous device slot to every ABD, FEM, or DyTopo
reporter. Reporters launch their component reductions asynchronously and write
their final value into that slot. A final CUB sum reads the reporter slots and
writes the total: for a trial (`compute_energy()`) into a read-back buffer
(`m_total_energy`, see "Read-backs"), for `E_0` into a distinct aggregate slot
copied with the reporters' values. A finite total implies finite reporter
energies (a NaN or an infinity among the addends makes the sum NaN or
infinite), so a trial copies and checks the individual values only when its
total is not finite or `line_search/report_energy` is on: the same checks fail
with the same messages, and the normal energy pass is one host
synchronization with no copy after the sum.

ABD and FEM still require separate reductions for their physical energy
components, but named one-thread combine kernels add those device scalars
without an intervening host read. DyTopo's CUB reduction writes directly into
its assigned slot. Keep every CUB input and output range disjoint, including
the final reporter-total reduction.

The energy producers of one evaluation run concurrently, in two parts.
The fixed part is every reporter whose energy does not read the trial's
active contact set (`LineSearchReporter::do_reads_active_set()` returns
false: ABD and FEM; the default is true): `LineSearcher` forks its own four
`cuda_tool::SideStreams` (`m_fixed_streams`) from the legacy stream and hands
them to those reporters (`ComputeEnergyInfo::side_streams()`). Each fixed
reporter then reduces on a reduce stream of its own (`m_fixed_reduce_streams`,
one per reporter, cycled if there are more than four), which joins only the
side streams that reporter's producers took (`SideStreams::join_next()`
right after its `do_compute_energy`): `do_reduce_energy` runs with
`ComputeEnergyInfo::stream()` set to that stream, so the fixed-body kinetic
fix, the CUB reductions and the combine kernels of one reporter start as soon
as its own producers finish, not after the other reporter's. The reduce
streams and the side streams are then joined (into the origin stream when
capturing, else into `m_fixed_reduce_streams[0]`, where an event marks the
end of the fixed part). The other reporters (DyTopo) take four more side streams
(`m_energy_streams`), forked when their producers launch; `LineSearcher`
joins only the streams handed out (`SideStreams::join_next()`) and reduces
them on the legacy stream (`stream()` is the legacy stream for them); the
legacy stream then waits for the fixed part's event and runs the final sum.
In `do_compute_energy` ABD, FEM and DyTopo give every producer (kinetic
term, constitution, joint or constraint, contact model) its own
`cuda_tool::LazySideStream` through the producer's `ComputeEnergyInfo` or
`EnergyInfo::stream()`. Most energy kernels are small FP64 kernels
(ortho, joints, contact, SNH) that keep only a few SMs busy, so concurrent
producers spread over the SMs instead of running
back to back; the ABD kinetic, ortho and joint ones launch one-warp blocks
(see the launch-shape exceptions), so none of them waits for a whole free SM.
The per-element energies and every reduction input, size and order are
unchanged, so each evaluation is bitwise the serial one; only independent
reductions run on different streams. A producer must launch everything on the
stream it was handed; one that ignores it (the AL contact models still do)
runs on the legacy stream: the fixed part's reductions then wait for the
legacy stream as well (such a launch invalidates a capture of the fixed
part, and the graph stays off for the engine), and the DyTopo reductions
follow the join on it.

The contact energies of an evaluated trial can be prelaunched.
`SimEngine::advance()` passes `filter_active` a `before_sync` callback. Once
the legacy-stream filters' two-phase work is enqueued,
`GlobalTrajectoryFilter::filter_active()` records
an event (`Impl::filtered`) behind it and calls
`LineSearcher::prelaunch_energy()` with it; only then does it launch the
filters that run beside (the half-plane filter), whose work the legacy stream
waits for (`Impl::side_filtered`) behind the prelaunched energies, and the
host waits for those two events instead of the legacy stream (no prelaunched
energy reads the half-plane pairs; the half-plane energies are launched after
the read-back). Until its finish, the simplex filter publishes the
pending active set (`SimplexTrajectoryFilter::pending_active_set()`: the four
lists at their capacity and the device addresses of their counts, which the
compaction also stores into `selected_counts`). `DyTopoEffectLineSearchReporter`
launches the leading DyTopo reporters that report a prelaunch extent
(`DyTopoEffectReporter::do_report_prelaunch_energy_extent`), up to the first
whose count is pending on the device (the offsets after it are known only
after the read-back): the simplex frictional contact (the frame start's
pairs, its usual launch) on a side stream that waits for the event, and the
IPC simplex normal contact by the capacity of the pending lists on the legacy
stream (the fused kernel takes the device counts and stops at them). After
the read-back, `compute_energy(false)` joins those side streams, sizes and
offsets every reporter on the host as before and keeps the prelaunched
ranges when the offsets and counts confirm them and the energy buffer did
not grow (`EnergyInfo::prelaunched()`: the reporter only takes its ranges);
otherwise it launches every reporter again behind the prelaunch. The later
reporters (the half-plane contacts) launch on the legacy stream, and the
reductions run on the host-known sizes as before, so every energy, its
position and every sum are those of the serial evaluation. The final
converged trial also completes this evaluation, so contact exports have
matching pairs and energy views.

The fixed part is launched right after a trial steps forward
(`LineSearcher::launch_energy()`), before the trial's `filter_active`: for
the first trial of an overlapped line search (see the prologue overlap
above) its host launches run while the GPU computes the CCD TOI, and for
every evaluated trial its GPU work runs beside `filter_active`. A restored
trial skips that launch. `compute_energy()` and
`launch_initial_energy()` launch it (through `enqueue_energy()`) when nothing
was launched (an evaluated `E_0`, the AL pipeline). When TOI or CFL truncates
the step, `discard_energy()` makes the legacy stream wait for the launched
part before anything it reads is overwritten.

An evaluated `E_0` of the IPC Newton loop is only launched
(`LineSearcher::launch_initial_energy()`: the evaluation, then a copy of every
reporter energy and the total into page-locked memory and an event on the
legacy stream). The host steps the first trial and launches its energy
meanwhile; `E_0` is read (`initial_energy()`: the event, then the same finite
checks as `compute_energy()`) where it is first used, the first trial's
`E <= E_0` test, whose own read-back has already waited for the copy in
stream order, or after a line search that evaluated no trial. Every write to
the energy slots and every stepped DOF comes after the copy on the legacy
stream (the trial's producers fork from it after the step), so `E_0` is
bitwise the synchronous evaluation. With `line_search/report_energy` the
evaluation stays synchronous.

The fixed part runs as a cached CUDA graph, kept across frames. Its launches
take by value only `dt`, the animation substep ratio (it enters the soft
position/transform constraint energies), constants and extents (element
counts, energy ranges); everything else they read is device memory. The only
extents that change between frames are the animator constraints' energy
counts (the constraints rebuild their lists in `step()`), and a buffer keeps
its device memory until it is reallocated, freed or moved. So the graph is
keyed (`LineSearcher::FixedKey`) on `dt`, the substep ratio,
`GlobalAnimator::energy_extent_generation()` (bumped by `step()` when an
animator's per-constraint energy counts, `Animator::do_energy_counts()`,
change) and `cuda_tool::device_allocation_epoch()` (bumped by every
allocation, free and move of a `DeviceVar`/`DeviceVector`, so the key changes
only when a buffer is reallocated). Per key the first evaluation launches
directly (it
sizes every buffer and grows the CUB workspaces of the reduce streams, which
the graph keeps and nothing else uses; the key is taken again after it, so
its own allocations are in the capture), the second captures the same
launches (`cuda_tool::GraphCapture::recapture()`: `cudaGraphExecUpdate` when
the topology is unchanged) and replays the graph, and later ones, in any
frame, replay it (one `cudaGraphLaunch` instead of the producers' launches
and the stream event calls; E_0 no longer launches directly and the first
trial no longer captures every frame). The graph holds the direct launches'
kernels,
parameters and dependencies, so its results are bitwise those of the direct
launches. A fixed energy producer must keep that contract: a by-value
argument that changes between frames beyond `dt`, the substep ratio and the
reported energy counts needs its own key entry. A fixed reporter must not
read anything back to the host. A producer that launches on the legacy stream
invalidates the capture (the capture stream is blocking): the graph is then
disabled for the engine's lifetime and every evaluation launches directly.

`Engine::frame_stats()` / Python `Engine.frame_stats()` is the stable
machine-readable complement to the Timer tree. CUDA returns schema v1 with the
pipeline, completion/convergence flags, Newton count, cumulative line-search
trials, cumulative iterative-linear-solver iterations, limit-hit flags, the
last line-search/CCD/CFL step factors, and whether the frame started from the
previous frame's active set (`active_set_reused`, IPC pipeline only) and how
many Newton iterations reused trajectory candidates (`trajectory_reuses`).
`reset_frame_stats()` runs immediately after incrementing the frame and
after a successful recovery. The engine sets `completed` after the end-of-frame
iteration checks. `GlobalLinearSystem::last_solve_iterations()` transfers each
solver's `SolvingInfo::iter_count` into the per-frame accumulator. A solver may
instead report the count pending (`SolvingInfo::iter_count_pending()`, the
mode-2 FusedPCG below): the solve is only enqueued, and
`last_solve_iterations()` asks the solver's `do_pending_iter_count()`, which
waits for that solve. The IPC Newton loop therefore collects the count after
the line search, whose read-backs have already waited for the solve. Keep this
contract host-only and cheap: profilers call it once per measured frame.

`ContactSystemFeature` exports zero instances until a completed advance after
initialization or recovery. On a completed IPC frame, per-pair energies come
from the accepted trial, and gradients and Hessians come from the last system
assembly.

During `do_init/do_advance/do_retrieve/do_sync`, the callbacks each sim system registered via `SimActionCollection` (`on_init_scene/on_rebuild_scene/on_write_scene`) are invoked in sequence.

### DyTopo distribution of raw entries

The DyTopo reporters (normal and frictional contact of simplices and half
planes, and any other DyTopo reporter) write disjoint ranges of the
collected doublets/triplets, so `_assemble` forks up to four
`cuda_tool::SideStreams` and gives each reporter its own through
`GradientHessianInfo::stream()`; the contact reporters pass it to their
assembly launches (`IPCSimplex*ContactAssemblyLaunchInfo::stream`). The
reporters with latency-bound assembly kernels lead the critical path, so
those that report one-warp assembly blocks
(`DyTopoEffectReporter::do_report_assembly_blocks()`, nonzero for the IPC
simplex normal contact's full-Hessian launch) are launched before the others,
each group keeping its order, and the
`ComputeDyTopoEffectInfo::after_leading_launch()` hook runs between the two
groups (see the FEM in-place prelaunch). The legacy stream joins the side
streams before the distribution reads the collected entries. The frictional
assembly thus runs beside the latency-bound normal assembly, launched after
it.

`GlobalDyTopoEffectManager` assembles raw doublets/triplets and hands each
`DyTopoEffectReceiver` the entries in its reported range without sorting or
reducing them first. The receiver contract (`ClassifiedDyTopoEffectInfo`)
allows duplicates in any order: FEM and ABD add gradients atomically, FEM
copies its Hessian triplets into the global triplets, ABD aggregates its
triplets per body pair (`abd_linear_subsystem_aggregate_dytopo_body_pairs_kernel`),
and ABD-FEM writes four global blocks per triplet. ABD's `report_extent`
groups the pairs. Up to `ABDBodyPairOneBlockMax` (3,072) receiver entries go
through one 1024-thread block (`abd_dytopo_body_pairs_one_block_kernel`: a
stable `cub::BlockRadixSort` of the same key bits, the runs from the sorted
keys and two block scans);
more go through a sort, a run-length encoding and two scans (about ten
launches and four CUB calls) with the run count kept on the device: the scans
cover all receiver slots, the slots past the run count holding zeros. Both
paths give the same integers, and the run count and the aggregated block
count come back in one read-back (a `ReadbackBuffer` and an event, no stream
synchronization). The aggregated blocks are late blocks
(`DiagExtentInfo::late_extent()`): the global system places and assembles
every other range first and only then asks for their count
(`do_report_late_extent()`, which waits for the event) and puts them behind
all the other blocks (`do_assemble_late()`, on the gathers' side stream, after
the reporters' diagonal additions), so the clears and the other subsystems'
assembly are issued while the grouping runs. The final `GlobalLinearSystem`
`convert_upper` pass sorts and reduces every global triplet, so an intermediate
sort/reduce would only change the summation order.

The distribution selects every receiver's entries in one packed pass per
entry kind (`_distribute_packed()`): a kernel per kind gives each collected
entry a 64-bit word with a field of `64 / n` bits for each of the kind's `n`
receivers, whose low bit flags the entry for that receiver, plus one trailing
zero word, and one `DeviceScan::ExclusiveSum` over the words yields in every
field that receiver's exclusive prefix sum, with its count in the last
element: the same integers as a scan of its own flags, since no field
overflows into the next. When both kinds are selected, the gradient kernel and
its scan run on DyTopo side stream 0, forked from the legacy stream after the
reporters' join, beside the Hessian ones, and the legacy stream waits for
them (an event) before `GlobalDyTopoEffectManager_gather_packed_counts_kernel`
stores every receiver's counts into a `ReadbackBuffer`, so the host issues no
copy per count before its one synchronization (it leaves the synchronization
earlier, with more lead at the BCOO conversion behind the FEM assembly).
After the read-back one fill kernel per kind writes each selected entry to
every receiver whose field is set, at the offset in that field: the same
copies to the same positions as a compaction per receiver. This replaces the
per-receiver selections (a flag kernel and a scan each) and the per-receiver
fills issued one by one after the read-back.

For each receiver the manager selects the gradient doublets with `i` in the
gradient range and the Hessian triplets with `(i, j)` in the Hessian ranges.
Classification by range, not by sortedness, is what
routes mixed ABD/FEM coupling: diagonal and off-diagonal blocks go to different
subsystems, and entries outside every range (for example of non-DOF half-plane
vertices) are dropped. The packed pass takes up to four receivers per kind and
eight in all, while each kind's entry count fits a field; otherwise every
receiver has its own selection buffers: the manager first launches all
selections (flag kernel, exclusive scan with one trailing zero for the
total), stores all counts with one
`GlobalDyTopoEffectManager_gather_counts_kernel` (up to 16; more fall back to
one `copy_from()` each) and synchronizes once, then compacts each receiver's
entries. Either way every receiver's entries keep their collected order and
the receivers get them in the same order as before (one host round trip for
all counts instead of one per receiver).

A strict shortcut skips the selection when all of these hold:

- exactly one receiver is registered;
- it reports a diagonal gradient/Hessian range beginning at global vertex 0;
- its gradient, Hessian-row, and Hessian-column ranges are identical and lie
  within the global vertex array.

That range is the complete dynamic vertex prefix. Global vertices after it can
be non-DOF implicit geometry (for example, half planes); their contact kernels
differentiate only the dynamic vertex. Every raw entry therefore belongs to the
receiver, and the manager passes the assembled views directly.

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

AL-IPC checkpoints save active PH/PT/EE index pairs, multipliers and retention
counters, plus the non-penetrating positions used by lagged friction.
`World.recover()` validates the contact history before applying a frame.
Older AL checkpoints predating this history cannot reproduce continued
contact and are rejected. The backend publishes a frame manifest after all
subsystem payloads have been written successfully.

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

### Read-backs

The Newton loop's small read-backs (the TOI per filter, the simplex filter's
compacted active-set counts, the trial energy total, the judge counts, the
BCOO converter's counts, the axis maximum displacement, the DyTopo
distribution's selected counts) are stored by their
producer into a `cuda_tool::ReadbackBuffer`: the producing kernel or CUB call
writes `device_data()`, `enqueue_copy(s)` follows it on its stream `s`, and
the host reads `data()` only after synchronizing `s` (never polling). With
mapped read-backs (`cuda_tool::mapped_readbacks()`: the device maps
page-locked host memory into the unified address space) `device_data()` is the
device address of host memory allocated with
`cudaHostAlloc(cudaHostAllocMapped)` (cached, not write-combined) and `enqueue_copy()` enqueues nothing: a device-to-host
copy after a kernel waits for the copy engine to take over from the kernel,
2-8 us each time the host leads, and the producer's own stores avoid it.
Otherwise `device_data()` is device memory and `enqueue_copy()` is that copy.
Both paths read back the producer's bytes (`readback_buffer` test). Rules:
the producer and every later use of the value on the device keep a device
copy where the device needs it (the BCOO k3 kernel stores its counts both into
the converter's `DeviceVar` and the read-back); a host value goes into a
read-back slot only through `store_value()` (a host-to-device copy into
mapped host memory is undefined behaviour); a device array that another path
produced (the `DeviceSelect::If` fallback of the compaction) reaches the host
elements through `copy_from()`; a buffer is not reallocated between its
producer and the read (it only grows, and the sites size it before
launching).

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
`GlobalLinearSystem` assembles 3x3 block triplets and
`MatrixConverter::convert_upper` canonicalizes them to the symmetric upper
triangle (`row <= col` on the signed indices, so `(-1, j)` slots stay and
`(i >= 0, -1)` go) and reduces duplicate entries into
`DeviceBCOOMatrix<Scalar, 3> bcoo_A`. Both the standalone SpMV and the fused
PCG SpMV/dot path call `rbk_sym_spmv*` with `bcoo_A.cview()`.

`convert_upper` produces bitwise what `ge2sym(triplet_A)` followed by
`convert(triplet_A, bcoo_A)` produced (the segmented reduction is the same, so
only a segment spanning three or more warps keeps its usual atomic-order
rounding), without compacting or copying the input blocks: lower triplets get a tail key one row code past the `-1` code, the
stable radix sort keeps the upper triplets in ge2sym's order and moves the
lower ones behind them, segment heads and one inclusive scan give each sorted
triplet its BCOO index, and `FastSegmentalReduce` reads every block through
the sort permutation. The upper-triplet and unique counts are computed on the
device (`MatrixConverterUpperCounts`) and read back once per conversion, which
is the only host synchronization left in `Convert To BCOO`. `ge2sym(triplet)`
remains as the reference path of the regression test. The segmented
reduction does not clear its output: the head of a segment within one
32-triplet warp stores its sum, and the unpack kernel clears exactly the
blocks the reduction adds to atomically (segments that span two warps, and
the last one when the last warp is partial;
`FastSegmentalReduce::reduce_uncleared`), which removes a fill of the whole
output per conversion; the regression test stays bitwise.
`FastSegmentalReduce`'s kernels reduce like the fused SpMV: one segment
analysis per warp (`cuda_tool::warp_head_segments`) and, for all values of
a lane in one loop (the nine entries of a 3x3 block), the steps of CUB's
`WarpReduce::HeadSegmentedReduce` below the warp's longest segment (larger
offsets change nothing in any lane), instead of one CUB reduction per
entry and another for the packed flags; whether a segment continues across
warps comes from its first and last lane. The sums and the stored or
atomically added outputs are bitwise those of the former kernels
(`fast_segmental_reduce_lean` test against copies of them: segments
within a warp and over two and over three or more warps, partial last
warps, a product-valued scalar op, int values, 64- and 128-thread blocks).

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
when the count exceeds the buffer. A filter whose `do_runs_beside()` returns
true (the easy vertex half-plane filter) launches its two-phase
`filter_active` and its `filter_toi` on a side stream
(`GlobalTrajectoryFilter::Impl::filter_active_side` / `filter_toi_side`,
handed over as `FilterActiveInfo::stream()` / `FilterTOIInfo::stream()`)
beside the other filters, which are launched first on the legacy (TOI)
stream: the side stream waits for the parent stream's work so far
(`SideStream::mark()` / `fork_marked()`) and is joined back before the one
synchronization, so the half-plane TOI kernel no longer leads the TOI chain.
On that side stream the half-plane filter clears its count with a memset
(not a pageable host-to-device copy) and publishes it into its
`ReadbackBuffer` with a copy kernel (no device-to-host copy behind the
query). Such a filter must put all of that work on the info's stream and share no
buffer with another filter. Filters without the two phases run
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
cover remap rewrites them as before, but right after the scatter and before the
counts' read-back: `cover_remap_judged_kernel` takes each segment's count (and
the cover-T pairs' offset) from the scan's per-list counts on the device and
covers them in one grid-stride launch, writing the same pairs to the same
places as `cover_remap_pairs_kernel`, and the host waits for an event recorded
before it. Two boxes that overlap lie in overlapping grown
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
- Exception: the ABD kinetic, joint and constraint producers of the assembly (BDF1 kinetic gradient/Hessian, the fixed joint, the revolute, prismatic and driving joints' k1/k2, the joint limits' k1, external articulation k1) and the ABD energy kernels of the line search's fixed part (BDF1 kinetic, ortho potential, the fixed, revolute and prismatic joints with their limits and driving joints, external articulation) also launch one-warp blocks; the spherical joint, ARAP, BDF2 kinetic and the soft transform constraint keep the occupancy block. They work on a few to a few dozen joints or bodies, so the occupancy block was a single block of 384-1024 threads with 52-128 registers per thread, which needs most of one SM's register file (the BDF1 kinetic gradient/Hessian: 896 threads x 68 registers, a whole SM) however few threads work. On the greatest-priority side streams such a block waits until an SM is free of the contact assembly's warps, and the block scheduler dispatches no later grid of the same priority meanwhile, so every producer behind it waits too. Each thread computes from its own index and writes its own slots; device code and outputs are unchanged (`abd_launch_shape` test).
- Device fills and copies use the `cuda_tool` buffer/stream APIs; fill
  kernels appear as `details::buffer_fill_kernel<T>` in profiler output.
- `_shorten_kernel_name()` in `python/src/uipc/profile/nsight.py` was originally used to extract the outer function name from `parallel_for_kernel<Lambda>`; with named kernels this proxy is no longer needed (the symbols are readable as-is).

### Resource-aware contact and FEM assembly

`StableNeoHookean3D_do_compute_gradient_hessian_kernel` projects derivatives
to vertex space without constructing dense `dF/dx` or a full `12x12` element
Hessian. `fem_utils.h` provides the tetrahedron shape gradients plus direct
`B^T g` and `B_l^T H B_r` helpers. The kernel writes four gradient doublets and
the ten upper-triangular `3x3` Hessian triplets directly, preserving the global
index ordering used by `TripletMatrixAssembler::half_block`. It visits the
blocks by the global-index rank of their left vertex (the one with the smaller
global index) and forms the products `shape_gradient(k) * ddEddF(3k+a, 3l+b)`
once per left vertex for all of its blocks: every block adds the same terms in
the same order as `fem::project_F_hessian_block` and is stored at the same
offset with the same indices, so the Hessian is bitwise unchanged with 486
fewer DMULs per tetrahedron (4771 -> 4285 FP64 instructions). The
direct-gradient kernels of the in-place prelaunch (see the assembly side
streams) share this device code and add the gradient into b instead of writing
doublets. The dense and
block formulas have backend regression coverage. On RTX 5090 this reduced the
kernel stack from 6440 to 1320 bytes/thread and Nsight Systems time from 1.795
to 1.047 ms/call.

The IPC simplex normal/friction assembly kernels deliberately keep PT, EE, PE,
and PP in one launch. Although splitting by stencil lowers each specialized
kernel's static stack frame, PT/EE Hessian evaluation is expensive even for a
few contacts. The fused launch overlaps those threads with the much larger PE
population; four sequential launches regressed case-46 DyTopo assembly by
about 70%. The uniform `gradient_only` decision is compile-time specialized
instead, giving lightweight gradient variants without changing the full
Hessian launch topology. Resource counts alone are therefore insufficient:
validate launch restructuring with both kernel profiles and the enclosing
stage timer. The normal contact's full-Hessian launch runs one-warp blocks,
each contact type's range padded to whole blocks so that no block mixes types
(`IPCSimplexNormalContactFullHessianBlockSize`): GB20x issues an FP64 warp
instruction at the same cost however many lanes are active, so whole warps
give this latency-bound kernel the latency of the former eight-lane blocks on
a quarter of the SMs, with a quarter of the FP64 issue.
`IPCSimplexNormalContact::do_report_assembly_blocks()` reports those blocks
for the DyTopo launch order and the elastic producer's grid cap (see the
assembly side streams); the gradient-only variant keeps contiguous ranges and
the occupancy block.

IPC simplex friction is lagged: the closest-point weights, the 3x2 tangent
basis, the normal force (a barrier derivative) and, for EE, the mollifier
decision depend only on the frame-start positions, kappa, `d_hat` and the
thickness. `IPCSimplexFrictionalContact` keeps them per friction pair
(`FrictionLagged`, contiguous PT | EE | PE | PP) and recomputes them in the
first energy or assembly launch after `SimplexTrajectoryFilter` recorded new
friction candidates (`friction_candidates_version()`, also bumped when the
candidates are cleared) or the contact tabular changed
(`GlobalContactManager::contact_tabular_version()`: the adaptive kappa
evaluates the contacts with a test tabular, then writes the new kappa).
Every later energy and assembly evaluation of the frame only forms the
tangential relative displacement from the current positions. The
`*_friction_lagged` and `*_friction_tan_rel_dx` functions split the old
`*_friction_basis` in two, and both the recomputing and the cached overloads
call them, so the cached path performs the same operations on the same
inputs: the `ipc_simplex_frictional_contact_energy_fusion` test compares the
production launch (fill, then reuse) with per-pair recomputation by `memcmp`.

### Assembly side streams

Joint and constraint producers are short dependency chains of one or a few
blocks each, so `GlobalLinearSystem` owns six non-blocking, highest-priority
`cuda_tool::SideStreams` that let them overlap instead of running back to
back on the legacy stream. `DiagInfo::side_streams()` exposes them during
`_assemble_linear_system` (it is null in the gradient-only path). The ABD
subsystem forks them after its reporter buffers are sized; every ABD
reporter producer (inter-body constitutions, inter-body and body
constraints) takes one stream on its first launch through
`cuda_tool::LazySideStream` and must put all of its launches there. The ABD
gathers (reporter and DyTopo atomics onto the kinetic/shape stores) run on
side stream 0 after `fork_one(0)` from the legacy stream and a `join` of the
other side streams (after a prelaunch, that join is issued at the end of the
prelaunch, beside the DyTopo work), while the coupling assembly and the non-prelaunched FEM
assembly continue on the legacy stream (the prelaunched FEM: see below);
`_assemble_linear_system` joins all side streams before the BCOO
conversion. The two 12x12-block gathers (kinetic/shape and reporters) run one
thread per 3x3 block (`affine_body/abd_hessian_blocks.h`, 16 threads per
12x12 block in 64-thread blocks): with one thread per 12x12 block a few
dozen bodies or reporters sat in one 256-thread block on one SM. Every
coefficient sees the former arithmetic, so
the triplets and stored diagonal blocks are bitwise unchanged
(`abd_hessian_blocks` test against copies of the former kernels); the
reporters' diagonal contributions stay atomic. The ABD-FEM coupling kernel
(FP64-heavy threads) launches 64-thread blocks for the same reason: the
occupancy block size of 512 put a few thousand of them on a few SMs. `_assemble_preconditioner` forks them again and hands each
local preconditioner `LocalPreconditionerAssemblyInfo::stream()`; the ABD
diagonal inverse uses it, and so does the FEM MAS preconditioner's diagonal
fallback for unpartitioned (e.g. Empty-constitution) vertices (identity fill
plus a pass over the BCOO), which thereby runs beside the
MAS cluster scatter and inversion that stay on the legacy stream; the join
comes before the solve. In a Newton iteration that assembles the
preconditioners, each local preconditioner's `do_prepare_assembly()` is
called once before `Convert To BCOO` (it must not read the global matrix):
from `GlobalLinearSystem::prelaunch_assembly()` when the iteration does not
keep the preconditioner (beside the DyTopo work, off the host's issue path
of the assembly), otherwise from `build_linear_system` once it decides to
assemble; the FEM MAS preconditioner clears its cluster
Hessians there (`MASPreconditionerEngine::prepare_preconditioner()`, two
memsets on a side stream of the engine forked from the legacy stream), so
the clear runs beside the conversion, and
`set_preconditioner()` joins that stream before the scatter instead of
clearing (it clears inline when nothing was prepared, e.g. at the first
assembly, which sizes the buffers). Side streams have no implicit synchronization with the
legacy stream: an ABD producer that ignores its stream stays correct only
because it then runs entirely on the legacy stream before the gathers' fork.
A FEM producer must launch on its stream (see the FEM prelaunch below: on
the legacy stream it would race with the FEM-local clear).

The IPC Newton loop calls `GlobalLinearSystem::prepare_prelaunch()` before
`Compute DyTopo Effect`, with the one-warp blocks of the latency-bound DyTopo
assembly kernels to come (`GlobalDyTopoEffectManager::assembly_blocks()`),
and `prelaunch_assembly(true)` from the DyTopo
manager's `ComputeDyTopoEffectInfo::after_assembly_launch()` hook, once the
DyTopo reporters' assembly kernels are launched and before their
distribution synchronizes with them (without DyTopo it calls
`prelaunch_assembly(false)` instead), so contact-independent producers
overlap the DyTopo computation. In between, the manager's
`after_leading_launch()` hook calls `prelaunch_ahead()` (the FEM in-place
prelaunch issued ahead, below). A diag subsystem may implement
`do_prepare_prelaunch()` and `do_prelaunch_assembly()`
(`PrelaunchInfo::contact_launched()` tells the DyTopo call apart,
`ahead_of_contact()` the call from `prelaunch_ahead()`, which only the
in-place lead receives); the
following assembly then sees `DiagInfo::prelaunched()`. ABD reports and
sizes its reporter ranges in the preparation (they do not depend on contact)
and forks the side streams there, before the DyTopo assembly kernels are
enqueued on the legacy stream; the prelaunch then launches kinetic and shape
terms on side stream 0
(`ABDLinearSubsystem::ComputeGradientHessianInfo::stream()`) and the
reporters on the others. The leading producers go first: a constitution
or constraint whose `do_leads_assembly()` returns true (the revolute joint
limit and the external articulation, whose 12x12 warp eigensolves make them
the longest chains) is launched by a first pass of the reporters
(`AssembleInfo` with `ABDLinearSubsystem::ProducerSet::Leading`; reporters
launch only the producers `AssembleInfo::launches()` accepts), each on a
side stream of its own; `SideStreams::keep_handed_out()` then keeps those
streams and side stream 0 to their chains, the kinetic/shape terms follow
on side stream 0, and a second pass (`ProducerSet::Others`) spreads the
short producers over the remaining streams, so none of them queues behind
a long chain. Launched last, a long chain can still run when the elastic FEM
kernel starts, which then shares SMs with it. Paths that assemble without a prelaunch use `ProducerSet::All`. The legacy-stream DyTopo chain is the longer one,
so its kernels are issued first and the producers' launch time is spent while
the contact assembly runs instead of in front of it; the producers wait for the same legacy-stream work as before.
Its assembly then only adds the kinetic/shape stores and the gathers on side
stream 0 after `fork_one(0)` from the legacy stream, which has cleared A
and b by then. DyTopo synchronizations stay on the legacy stream and never
wait for the linear system's assembly side streams. Paths without the call
(AL advance, gradient-only assembly) launch the producers inside the
assembly as before.

FEM acts only in the calls from DyTopo (in the earlier `prelaunch_ahead()`
one only in place, see below): it launches its kinetic and reporter
producers (elastics, animator constraints) there on its own
`cuda_tool::SideStream`. The stream's start point is marked
(`SideStream::mark()`) in `do_prepare_prelaunch()`, before the DyTopo work is
enqueued: from there (`fork_marked()`) run the FEM-local clear, the kinetic
term and the reporters whose `do_launches_beside_contact()` returns true (the
animator's per-vertex constraints), beside the contact assembly kernels,
whose launches precede theirs on the host, so the contact blocks are placed
first. The stream then forks again from the legacy stream, after the contact
assembly kernels, and the other reporters (the elastics) follow. This fork
lets the elastic producer start as soon as the contact kernels end instead of
behind the light producers (the contact kernels run slightly longer beside
them); where the in-place prelaunch applies, it issues the elastic producer
ahead instead (below).
The producers read only the Newton
iterate and the frame's animation targets and write FEM-private buffers: the
gradients as before and the Hessians into a FEM-local triplet matrix whose
slots are first marked unwritten by their row index (-1) only: the gather
turns a slot that stays unwritten into the global clear's `(-1, -1, 0)` and
reads columns and values of written slots only, so values and columns need
no clear (filling the values, the largest array, would sit at the head of
this stream in front of the elastic producer). The fork point matters:
an elastic producer such as `StableNeoHookean3D` fills each SM with one
256-thread, 255-register block, so dispatched beside the contact assembly
it holds back the latency-bound contact kernels; started after them it runs
while the legacy stream distributes the contact entries; held to the SMs the
contact blocks leave, it can run beside them (the in-place prelaunch issued
ahead, below). (The FEM-local
clear does not move ahead of the contact launch either: a fill still
running when the contact kernel starts skews its blocks over the SMs.) FEM
reports its triplets as fully written (`PrelaunchInfo::fully_written_triplets`),
so the global clear skips that prefix of the FEM range.

From the second assembly on, the FEM prelaunch writes the global system in
place (`InitDofExtentInfo::prelaunch_in_place()`, then
`PrelaunchInfo::in_place_available()`, `in_place_hessians()`,
`in_place_gradients()`, `in_place_done()`). The global system puts the
in-place subsystem's triplet range first (the other ranges follow in
subsystem order, then the late blocks of each subsystem in subsystem order;
DoF order and subsystem indices are unchanged, no `(row, col)` key is shared
across subsystems and a subsystem's late blocks stay behind its others, so
the conversion's stable sort puts the same entries in the same order under
every key and the reduction reads the same sequence: converting copies in the
same and in the former order gives the same indices and the same rate of
last-bit value differences, which come from the reduction's atomic
cross-warp partial sums in either order), so the FEM prefix sits at triplet 0
whatever the contact-dependent counts of the other subsystems are. A late
subsystem writes every one of its late blocks (the global clear does not
cover them); when they do not fit the triplet capacity, the global system
waits for the other blocks and grows the matrix keeping them. On the FEM stream the
prelaunch marks the prefix slots unwritten (row -1), clears the FEM range of
b (an event records it; the legacy stream waits for it before it adds the
DyTopo gradients), and launches the kinetic term and the beside-contact
reporters with global Hessian views; after the contact fork the elastic
producers follow, and `StableNeoHookean3D` adds its element gradients
straight into b (`AssembleInfo::direct_gradients()`; atomics, skipping fixed
vertices; a constitution that does not do so writes doublets, which
`FiniteElementElastics` adds after it). The kinetic and beside-contact
gradients are added on a second stream forked after the contact assembly,
beside the elastic producer (added before the fork they held SMs from the
contact kernels). A fix-up kernel then applies the gather's two rules in
place (an unwritten slot becomes `(-1, -1, 0)`, a block touching a fixed
vertex 0 or I). `assemble()` writes only the DyTopo terms; the legacy stream
waits for the FEM stream (`in_place_done()`'s event) only at the end of the
global assembly, before the conversion, instead of before a gather. If the
DoF layout changed since the previous assembly, the global system waits for
the in-place writes and the FEM subsystem assembles in full; a triplet
capacity growth after the prelaunch copies the prefix (it waits first). The
Hessian is bitwise the gathered one; the FEM gradient is a sum of the same
terms in another atomic order (atomic-order class, `rule.md` rule 16).

The in-place prelaunch can also be issued ahead. When `prepare_prelaunch()`
was given contact blocks, `prelaunch_ahead()` (from the
`after_leading_launch()` hook, once the reporters with one-warp assembly
blocks, the IPC simplex normal contact, are launched and before the other
DyTopo reporters, behind whose kernels the elastic producer would find no SM
free) calls the in-place lead with `PrelaunchInfo::ahead_of_contact()` and a
grid cap for its elastic producers, `elastic_block_cap()`, that is
`max(SMs / 2, SMs - contact blocks)`. FEM goes ahead only in place and only
when one round of 256-thread blocks covers the largest 3D constitution
(`ceil(elements / 256)` at most the SM count; every 3D constitution launches
its own producer); otherwise it prelaunches in the later call as above.
Going ahead, the FEM stream starts from the marked point, marks the prefix
slots unwritten, clears the FEM range of b and runs the elastic producers
beside the contact kernels: `StableNeoHookean3D` keeps its grid within the cap
(`FEMLinearSubsystem::AssembleInfo::block_cap()`), so it takes the SMs the
contact blocks leave, and the other elastic producers launch their usual
grids. The later call (`prelaunch_assembly(true)`) issues the rest: the
kinetic term, the beside-contact reporters and their gradient additions on a
second stream that starts from the clears, then on the FEM stream the
remaining doublet additions, the join of the second stream, the fix-up and
`in_place_done()`. When its 256-thread grid exceeds the cap,
`StableNeoHookean3D` launches the narrowest of 384, 416, 448 and 480 threads
per block that covers its elements in one round within the cap
(`StableNeoHookean3D_compute_direct_gradient_hessian_wide_kernel<Wide>`,
`__launch_bounds__(Wide, 1)`, so that one block still fills an SM: a round on
a full SM is bound by its FP64 pipe and lasts in proportion to the block
width); a cap that would need wider blocks keeps the 256-thread kernel, which
then loops over the elements with `block_cap()` blocks. Every element runs
the same code in every variant. The `in_place_prelaunch` backend test drives
this call too
(its test lead writes its whole prefix there) and requires the bytes of
`assemble()`.

**FEM stream contract (required).** FEM producers take their stream from
`FEMLinearSubsystem::ComputeGradientHessianInfo::stream()`,
`FEMLinearSubsystem::AssembleInfo::stream()`,
`FiniteElementElastics::ComputeGradientHessianInfo::stream()` (also through
the 1D/2D/3D constitution infos) and
`FiniteElementAnimator::ComputeGradientHessianInfo::stream()` (the 2D
shells and bending use `BendingGradientHessianLaunchInfo::stream` and
`NeoHookeanShellGradientHessianLaunchInfo::stream`). Every launch of a FEM
producer must go to that stream: the slot marking runs before them and the
fix-up after them on it, so a producer launched on the legacy stream would
race with both and silently lose or mis-fix Hessian blocks (with
legacy-stream shell producers `fem_mas_soft_vertex_stitch_regression` saw
triplets with a row index and a cleared column index). The in-place
prelaunch checks this whenever the producers' extents change: it launches
them while a blocking stream captures (`LegacyStreamCheck`), so a launch
into the legacy stream fails and the assertion names the contract.

### Warp-cooperative ABD diagonal inversion

`ABDDiagPreconditioner::do_assemble` inverts each body's 12x12 diagonal
Hessian block with one warp (`launch_abd_diag_inverse`,
`affine_body/abd_diag_inverse.h`) instead of one thread running
`cuda_tool::eigen::inverse`, whose serial elimination over a local-memory
12x24 array made the launch latency-bound for a few dozen bodies. Lane `c`
keeps column `c` of the augmented `[H | I]` in registers and performs exactly
the serial routine's operations on it, in the same order (pivot repair, row
division, `row += e * pivot_row` contractions); lane `i` broadcasts the pivot
column. Because `m[i][i] = ep / ep` is exactly 1 after the row division of a
finite pivot, the multiplier division is skipped for such pivots. The result
is bitwise identical to `GaussEliminationInverse`. For a block without a
usable pivot that routine returned an uninitialized matrix; the warp kernel
stores the identity, as the subsystem does for fixed bodies. The
`abd_diag_inverse` backend test compares both with `memcmp` on SPD, ABD-like,
general, sparse, nearly singular and pivot-repair blocks. Keep the two
routines in lockstep: any change to `GaussEliminationInverse` must be mirrored
here.
The heavy joint Hessian kernels (external articulation
`external_articulation_constraint_compute_gradient_hessian_k2_kernel`,
revolute joint limit, prismatic joint and prismatic joint limit `..._k2_kernel`)
run one joint (pair) per 32-thread block and let the warp's lanes share the
joint's work. A scene has only a few joints, so the thread-per-joint versions
ran one joint's serial FP64 chain per warp, and on GB20x FP64
issues per warp instruction whatever the active lanes (a dependent DFMA chain
runs at about 19 ns per instruction): a 12x12 Eigen `make_spd` alone took about
350 us, a 24x24 `JT_H_J` about 55 us. In the warp kernels the generated
derivative code (`dDeltaTheta_dQ`, `ddDeltaTheta_ddF`, `ddE01ddF01`, ...) still
runs redundantly on every lane (no extra cost), while
- the SPD projection is `warp_make_spd<N>` (`utils/warp_spd.h`): the
  classical cyclic Jacobi eigensolver in its parallel (round-robin) ordering,
  all in FP64: the rotation tangent is solved to double precision by two
  Newton steps from an FP32 estimate (no double division or square root on
  the critical path), the annihilated entry is set to zero and the diagonal
  updated as `app - t apq`, `aqq + t apq`; the matrix is scaled exactly by a
  power of two; it stops when the largest off-diagonal entry is below 2e-15
  of the Frobenius norm or after 24 sweeps, with no "slow sweep" stop (nearly
  degenerate clusters, e.g. the ortho-potential Hessian near a rotation,
  converge linearly for a few sweeps first; a first version that stopped there
  was 1e-7 off); from the fifth sweep on, a rotation whose entry is negligible
  by Demmel and Veselic's relative criterion `|a_pq| <= 2^-53 sqrt(|a_pp a_qq|)`
  is skipped and the entry set to zero (Rutishauser's skip with that
  threshold; it moves `A` by at most one rounding of a diagonal entry). The
  skip matters for exactly degenerate clusters: the revolute joint angle
  Hessian (external articulation, revolute joint limit) has eigenvalues
  `+-1/(2 rho)` about six times each; once their diagonal entries agree to the
  last bit, rounding-sized entries between them drove 45-degree rotations that
  kept stirring the cross-cluster couplings, which then only halved per
  sweep, so such an eigensolve could run to the 24-sweep cap and stop short of
  full accuracy; with the skip they converge in a few sweeps, and the
  projection is closer to a long double reference on average in every test
  class. `warp_make_spd` returns the sweeps it ran. Two details only make
  the rounds shorter and keep the iteration bitwise (a backend test compares
  every projection and sweep count with a copy of the former kernel): the
  rotation scales `d`, `w` by one multiplication with the exact power of two
  when the library `scalbn` would do the same (normal `m`, `|e| <= 216`; it
  splits larger shifts), and the update of `A` evaluates the rotated entry
  and Rutishauser's diagonal update for every lane and selects, instead of
  branching per entry (the lanes of a warp take different branches). The
  projection is
  assembled from the side with the smaller eigenvalue mass,
  `P = V max(L, 0) V^T` or `P = A - V min(L, 0) V^T`, so a
  nearly PSD matrix keeps its own rounding; the input is read from its lower
  triangle (as Eigen) and `P` is written exactly symmetric;
- the `24x24` outer products, `J^T H J` maps (`utils/warp_joint_hessian.h`:
  `basis_jt_h_j_entry` for the two-frames-per-body maps of the revolute limit
  and external articulation, with the first frame's translation for the
  external prismatic joint; `prismatic_jt_h_j_entry` for the prismatic joint
  `F01` maps; `prismatic_e23_hessian_entry` for the q-independent orientation
  term) and the three triplet blocks (`warp_write_half_block2`, same slots and
  upper-triangle placement as `half_block<2>().write`) are entry-parallel.
The result equals the former thread-serial kernels up to rounding (different
eigensolver and summation order; the projection is 1-Lipschitz in the input):
the `[warp_spd]` backend tests compare `warp_make_spd` with Eigen on random,
rank-deficient, clustered, diagonal and nearly degenerate two-cluster matrices (N = 9, 12) and the
revolute joint angle Hessians of the generated code (every projection must
converge before the sweep cap, the clustered ones to 2e-15 of `|M|_F`), and the
entry-parallel maps with the generated `JT_H_J` / `J01T_H01_J01` / `ddE23ddQ`.
The same test measures each projection's error against a long double
eigendecomposition on the host (`long_double_reference.cpp`) and requires the
warp projection to be on average as accurate as Eigen's thread-serial
`make_spd` and the fixed-bank variant it replaces (mean within 1.5x of the
better of the two, worst case within 3x).

The fixed-joint and revolute-joint Hessians do not change between Newton
iterations: their constraint measures (`Fr`/`Ft`, `Faxis`) are linear in `q`
and the energies quadratic in them, so the Hessian depends only on
`kappa = strength_ratio * (m_i + m_j)` and the rest frames. Both constitutions
keep a per-joint cache of the `24x24` Hessian together with the `kappa` it was
computed with. The Hessian kernel recomputes a joint only when its current
`kappa` differs bitwise from the cached one (the cache starts as NaN), so a
change of strength ratio or body mass invalidates it; `do_init`, the only
writer of the rest frames, resets the cache. The cached matrix goes through the
same `half_block` write as a fresh one, so the assembled triplets are
unchanged.

## FusedPCG CUDA graph modes (linear_system/use_cuda_graph, default 2)

The per-iteration kernel chain of `LinearFusedPCG` (spmv_dot → update_xr →
preconditioner → dot → converged → update_p → swap_rz) is ~10 tiny launches
whose launch gaps dominated the wall time (~80 µs of ~118 µs per iteration on
case2-scale scenes). Two graph modes exist; `graph_mode` is auto-selected in
`do_build` (logged at info level):

- **mode 2 — full-GPU while-loop** (default, toolkit and driver ≥ CUDA 12.4):
  the whole solve is ONE graph launch — setup chain (init: reset incl.
  rz=0, x=0, r=b and the slot-0 clears → precond → p=z with rz=rᵀz in one
  pass → rz_tol=tol·|rz0| on device; `do_solve` no longer clears x before
  the launch, only the block-replay and plain paths do)
  → WHILE conditional node whose body is two block-replay graph iterations,
  one per ping-pong slot (pipelined SpMV, fused local-preconditioner
  update/apply/dot, direction publication). The direction kernel's
  publisher (block 0) also counts the iteration and calls
  `cudaGraphSetConditional`: it publishes Converged (|rz_new| ≤ rz_tol),
  Stopped (the max_iter−1-th iteration without convergence) or Running,
  with the plain loop's decisions; any terminal state turns the rest of the
  body into no-ops. rz and the iteration count are double-buffered by the
  body slot (`m_while_rz`, `m_while_iter`: an iteration reads its slot and
  publishes to the other), so every block computes beta and the decision
  from the same scalars itself, bitwise the publisher's, and the WHILE
  direction kernel needs no claim/countdown protocol (the block-replay
  kernel keeps it, with its single rz). Its threads read z and p before
  the barrier that publishes block 0's decision (they do not depend on it),
  so those loads overlap the scalar work; the update is the same
  expression on the same values (`fused_pcg_while_publication` test
  against copies of the claim/countdown and the former slot-buffered
  kernel).
  Exit happens at the exact convergence iteration; zero D2H/H2D inside the
  loop. The host does not wait for the graph (`GraphWhile::launch()` on its
  blocking stream): legacy-stream work that consumes `x` waits for it by the
  legacy-stream rule, the iteration count and state are copied into pinned
  memory behind it (one copy: the publisher also writes the state it
  returns next to the count, `m_while_out`), and `do_pending_iter_count()`
  reads them when the frame
  statistics ask (see `last_solve_iterations()` above). Work on non-blocking
  streams must still be forked from the legacy stream to be ordered after the
  solve. The historical one-iteration body took about 226 ms/frame on
  case2-scale scenes, compared with 203 ms/frame for block replay. The current
  body runs 7 kernels per iteration and evaluates the WHILE condition once per
  two iterations. The current [graph mode benchmark](performance/2026-11-25-pcg-graph-modes.md)
  supports selecting mode 2 by default on runtimes with conditional nodes.
- **mode 1 — block replay**: `check_interval` iterations captured
  per block via `cuda_tool::GraphCapture`, host convergence check between
  blocks (same cadence as the plain loop). A mode 2 request selects this mode
  when conditional nodes are unavailable.
- **mode 0** — plain launches (config off, or `contact/constitution != ipc`).

Shared infrastructure: `cuda_tool/graph.h` holds `GraphCapture` (flat block
capture) and `GraphWhile` (WHILE-loop assembly via conditional nodes);
instantiation API is dispatched by `CUDART_VERSION` (WithFlags ≥11.4 through
13.x, legacy 5-arg below), and `GraphWhile` additionally runtime-checks the
driver (≥12.4 for conditional nodes).

Graph-stability design (no rebuilds from contact-pair fluctuation):
- The SpMV triplet count lives on device (`triplet_count_dev`, uploaded once
  per assembly) and the launch grid is sized by the reserved triplet
  CAPACITY (128-thread blocks, each warp reducing its own 32 triplets, so
  a call with nothing to do, the pipelined no-op slot, schedules a quarter
  of the blocks it did with one warp per block; the y and pᵀAp atomics keep
  their addends and only arrive in another order); the kernel guards with
  the device count (read together with the converged flag) and reads the triplet
  arrays through raw pointers (`A.row_indices().data()` etc.) — the view's
  `A(i)` accessor asserts against the capture-time count and must NOT be
  used for this. The fused PCG always passes a = 1; `rbk_sym_spmv_dot*`
  then launch the `UnitScale` instantiation, which skips every
  multiplication by a (14 of the kernel's 77 FP64 instructions, 13 of about
  55 executed per off-diagonal triplet: the 9 of `a * M^T`, the 3 of the
  row-head `a * vec`, `2a`). GB20x runs FP64 at 1:64 and the kernel is
  FP64-bound; x * 1 is exact, so y and pᵀAp are bitwise those of the
  general kernel (its SASS is unchanged, and the remaining DFMA/DADD
  sequence of the unit kernel is identical in order). Diagonal and
  off-diagonal blocks share one pᵀAp path, `(i == j ? 1 : 2) * a *
  x_i.dot(M x_j)` (x_i is x_j on the diagonal; the factor is exact), so a
  warp holding both no longer issues two dot products. The three row
  sums are one segment analysis per warp (CUB's ballot, tail flags and
  segment ends; `cuda_tool::warp_head_segments`, shared with
  `FastSegmentalReduce`) and the shuffle-down additions of CUB's
  `HeadSegmentedReduce` at offsets 1, 2, 4, ... below the warp's longest
  segment: a larger offset adds nothing in any lane, and the upper-BCOO
  rows are a few blocks long, so the last one or two of the five steps
  are skipped. One loop serves the three components so that their
  shuffle-add chains interleave as in CUB's unrolled steps (three separate
  loops measured slower). y and pᵀAp keep their bits
  (`fused_pcg_spmv_pipeline` test against a copy of the former kernel).
- rz_tol is device-side; the validity key is buffer pointers + N + max_iter.
  A matrix realloc (new pointers) still forces a rebuild.
- Stream plumbing: `Spmv::rbk_sym_spmv_dot`, `GlobalLinearSystem::Impl`
  `spmv_dot/apply_preconditioner`, `IterativeSolver` pass-throughs,
  `ApplyPreconditionerInfo::stream()`, and the ABD/FEM diag preconditioners
  all accept an optional launch stream (default = legacy default stream =
  unchanged behavior). The MAS preconditioner is plumbed too:
  `MASPreconditionerEngine::apply` and its three phases (restrict /
  Schwarz local solve / prolongate) all launch on
  `ApplyPreconditionerInfo::stream()`, so MAS scenes join the captured
  graph instead of invalidating it. `FEMMASPreconditioner::do_apply` hands
  `apply()` the diagonal-fallback inverses, and the prolongation pass writes
  z = D⁻¹r of the unpartitioned vertices with the expression of the
  separate pass it replaced (bitwise, `mas_preconditioner_direct_store`
  test). The graph iterations' fused
  update/apply/dot step also covers unpartitioned (e.g. Empty-constitution)
  vertices: `MASPreconditionerEngine::fused_pcg_update_apply_dot`
  takes the diagonal-fallback inverses, and its final pass updates their
  x/r, applies z = D⁻¹r and adds their rᵀz, so mixed MAS/diagonal coverage
  takes three launches per iteration (restrict with the x/r update, local
  solve, prolongate with rᵀz) instead of the seven of the reference update
  → MAS apply → diagonal apply → dot sequence. The restriction accumulates
  into the coarse residual levels; the Schwarz local solve clears them right
  after reading them (`m_coarse_R_zero` tracks that invariant on the host;
  each of its threads reads its 3x3 inverse block before the barrier that
  shares the cluster's residuals, bitwise the former kernel by its device
  test),
  so no clear node precedes the restriction (`apply()` still clears once).
  The restriction and the prolongation with rᵀz load their flag, partition
  index and x/p/r/Ap operands before any store (the compiler otherwise
  serializes load, update and store per component, as the vectors may
  alias), and the restriction launches 64-thread blocks (four times as many
  blocks, spread over more SMs); every value keeps its expression, and only the
  arrival order of the coarse levels' float atomics and of the rᵀz block
  sums changes (a device test on exactly representable data checks every
  output bitwise against copies of the former kernels; the accuracy rule
  covers the order).
  With several local preconditioners (unique owners, e.g. ABD + FEM MAS) the
  fused step forks every segment but the last onto
  `GlobalLinearSystem::Impl::pcg_segment_streams`, so a captured iteration
  runs them as parallel graph branches (disjoint DoF ranges; only the order
  of the atomic rᵀz additions changes). `apply_preconditioner` forks the
  same way when it runs on a launch stream of its own, i.e. in the captured
  WHILE setup chain, which then applies the ABD preconditioner beside the
  MAS passes (the ABD apply and the diagonal pass leave the setup chain's
  serial path); the plain path keeps the serial sequence. The engine's workspace buffers are
  sized once at init from the mesh partition (contact does not recluster —
  collision-aware clustering is deliberately not ported), so the captured
  pointers stay valid as contact nnz fluctuates. The partition hierarchy is
  likewise constructed once in `init_matrix()` and reused by every Newton
  assembly; only the current BCOO Hessian scatter and cluster inversion are
  repeated. Scatter reads BCOO entries directly (there is no identity-index
  staging buffer). An entry whose two nodes share a level-0 cluster is the
  only one for its cluster slot (the upper BCOO has one entry per node pair
  and `real_to_part` is a bijection), so it is stored as `0.0 + h`, bitwise
  the atomic addition onto the cleared slot; coarser levels still add
  atomically. On a coarse diagonal block (both nodes merged into one coarse
  node) the scatter adds only the upper triangle, all the inversion reads:
  an entry adds `H(i,j) + H(j,i)` (6 atomics instead of 18) and the
  propagation of level-0 blocks skips the lower triangle (a fully connected
  bank's warp sums 6 entries instead of 9); the debug dump mirrors the
  upper triangle. The `mas_preconditioner_direct_store` test
  checks the scatter against a Galerkin reference with exactly
  representable blocks. Each 48x48 cluster matrix is inverted by sweeping its
  pivots (Goodnight's sweep operator, no pivoting, zero diagonal entries of
  padding nodes repaired to 1 first): sweeping pivot k maps the matrix to
  `a_ij - a_ik a_kj / a_kk`, `a_ik / a_kk` and `-1 / a_kk`, and all pivots
  leave `-A^{-1}`. The sweeps keep the matrix symmetric, so only its upper
  triangle is kept, in registers. That is half the double-precision work of
  the former Gauss-Jordan elimination of the full matrix, which bounds the
  kernel on GPUs with low FP64 throughput. It reads only the upper triangle
  of each diagonal 3x3 block (the elimination read the whole block, whose
  lower half can differ from the transposed upper half by rounding) and
  writes the lower half of each diagonal block of the float inverse from
  the upper one, as the elimination did. The result is the same inverse to
  rounding, not bitwise (`mas_cluster_inverse` test against a copy of the
  former shared-memory elimination; inverse residual and a double
  reference). One 96-thread block sweeps one cluster: the columns are
  paired (c, 47 - c), each pair's 49 upper-triangle entries split over four
  threads (13/12/12/12 registers); per pivot the holders publish column k,
  the holder of `a_kk` alone computes `1 / a_kk`, threads 0..47 publish the
  quotients, and every entry is updated with exactly the operations of the
  former 24-thread layout (one thread per column pair, four clusters per
  block), so the inverse is bitwise that layout's (test against a copy of
  it on 10,002 clusters). With four times the warps per cluster the FP64
  pipe stays busy.
  The kernel is launch-bounded to 6 blocks per SM: an SM's registers are
  split among its 4 warp schedulers, so at the 117 registers the compiler
  picks otherwise (and still at 112) only 4 warps fit per scheduler, 5
  blocks of 3 warps per SM, so a cluster count just above a multiple of that
  leaves a nearly empty second wave. The bound keeps it at 96 registers (20
  bytes per thread spill, two reloads per pivot): 5 warps per scheduler, 6
  blocks per SM. Likewise the scatter's k2 pass is launch-bounded to 5 blocks
  of 256 threads per SM (48 registers, one value spilled around a loop; 4
  blocks per SM at the 52 registers the compiler picks otherwise).
  Only the register allocation changes, so both stay bitwise (the tests
  above).
  The banks between level 0 and level 1 (present when unpartitioned
  vertices make the FEM vertex count exceed the partitioned node count)
  hold no Hessian entry and nothing reads what is
  computed from their inverses, so `invert_cluster_matrices()` skips them;
  their inverses keep the zeros of the allocation. Nothing writes their
  residuals or reads their solutions either, so the Hessian clear, the
  coarse-residual clears (which start at level 1) and the Schwarz local
  solve (one 256-thread block per real cluster, none for an empty bank) skip
  them as well; a
  device test poisons the empty banks of every buffer with NaN and requires
  z equal to a clean twin's and the NaN still in place afterwards.
  `MASPreconditionerEngine::invert_cluster_matrices` also has a static
  overload on explicit buffers (with an optional skipped range) for the
  tests.
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
| `stream.h` | `CUDA_TOOL_CHECK` error checking, `default_stream`, `Stream` (`Stream::Default()`), `SideStreams` (non-blocking fork/join streams with events; `join_next()` joins only the streams handed out by `next()` since the last fork or `join_next()`; `keep_handed_out()` leaves the streams handed out so far to their producers, and `next()` then cycles over the others until the next `fork()`) and `LazySideStream` (one producer's stream, taken on first use), `SideStream` (owning non-blocking stream ordered against its parent stream only by `fork()`/`join()` events; `mark()` and `fork_marked()` split `fork()` into recording the parent's point and waiting for it) |
| `view.h` / `view_nd.h` | `CBufferView/BufferView/VarView/CVarView`, `Dense/CDense` (scalar viewers), `ViewerBase`, `Extent2D`, `Buffer2DView`, `Dense1D/Dense2D` (with `make_dense_1d/2d`) |
| `launch.h` | `best_block_dim/best_grid_dim` (occupancy-based block size / grid computation); `spread_block_dim/spread_grid_dim` (one-warp blocks for FP64-heavy per-item kernels over short lists) |
| `buffer.h` | `DeviceVector/DeviceBuffer/DeviceVar/DeviceBuffer2D`, `BufferLaunch`. `resize()` value-initializes growth; `resize_discard()` is for fully regenerated output and grows to 150% of the latest requirement; `resize_preserve()` retains the old logical range without initializing the new range. Exact and amortized preserve/discard reserve variants make ownership and growth policy explicit. `PinnedHostBuffer` is a page-locked host array whose `copy_from` enqueues a device-to-host copy without blocking, so several small read-backs share one stream synchronization (a copy into pageable memory waits for the stream on its own). `ReadbackBuffer` holds read-backs whose producer stores them directly (see "Read-backs" below): `device_data()` is where the producer stores, `enqueue_copy(s)` follows it, `data()` is read after synchronizing `s`; `mapped_readbacks()` decides the path, and `store_value()` stores a host value into device memory or a read-back slot. `device_allocation_epoch()` counts every allocation, free and move of a `DeviceVar`/`DeviceVector` (work recorded with buffer pointers, such as a cached graph, stays valid while it is unchanged). |
| `copy_batch.h` | `CopyBatch` collects device-to-device copies and zero fills of non-overlapping ranges (`copy()`, `zero()`) and enqueues them with `launch()` as one kernel over 8-byte words (a range that is not made of whole words gets its own `cudaMemcpyAsync`/`cudaMemsetAsync`): the same bytes, one launch. Used for the copies that open a line search |
| `cub.h` | Thin wrappers (pointer-style) for `DeviceReduce/Scan/Select/Partition/RadixSort/MergeSort/RunLengthEncode` + warp-level cub headers; **temporary storage uses a stream-level workspace cache** (`details::cub_temp_storage`, grows 2× on demand, reused across calls — per-call cudaMalloc/cudaFree costs ~10-100µs each and implicitly synchronizes with the device, which once caused a ~15%/frame performance regression in the 6_wrecking_balls scene). Each wrapper calls CUB once, on the stream's whole workspace (`details::run_with_temp_storage`): CUB returns `cudaErrorInvalidValue` before it launches anything when the storage is too small, and only then is the size queried, the workspace grown and the call repeated (a separate size query would repeat the dispatch's host work, about 2.5 µs per call). Every call's error is checked, so a wrapper's callable returns CUB's `cudaError_t` |
| `linear_system.h` + `linear_reduction.h` + `linear_system/views.h` | `DeviceTripletMatrix/DoubletVector/DenseVector/BCOOMatrix/BSRMatrix/DenseMatrix` + full set of views (Triplet/Doublet/DenseVector/BCOO) + `LinearSystemContext`; dot/norm use named partial-reduction kernels, persistent geometrically grown partial/result buffers, the per-stream CUB workspace, and scaled-square norm accumulation without cuBLAS |
| `eigen.h` + `eigen/` | Device-side small-matrix math (`eigen::evd/svd/pd/inverse/atomic_add`). The family originated in the MUDA migration; current `svd/pd` use `algorithm/qr_svd.hpp` with signed-SVD fixup and native float/double precision. |
| `debug.h` | `debug_sync_all/check_finite` + the `UIPC_KERNEL_ASSERT/ERROR/WARN` macro family (gated by `uipc::RUNTIME_CHECK`) |
| `graph.h` | `GraphCapture`: self-contained CUDA-graph block capture/replay (capture body on a capture stream, replay via `launch_sync()` or, without waiting, `launch(stream)`; `recapture()` updates an instantiated graph in place with `cudaGraphExecUpdate` when the new capture keeps its topology; permanent fallback on capture failure, also when the body throws on an invalidated capture; a body that throws on a valid capture rethrows and leaves no instantiated graph, in `capture()` and `recapture()` alike). `GraphWhile`: setup chain plus WHILE conditional node, replayed by `launch_sync()` or, without waiting, by `launch()` on `launch_stream()`. Instantiation API is selected by `CUDART_VERSION` (`cudaGraphInstantiateWithFlags` ≥11.4 through 13.x, legacy 5-arg form below), so the same source builds across CUDA 11/12/13 |
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
