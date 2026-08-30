# 05 — CUDA Backend

Directory: `src/backends/cuda/` (compiled as the runtime-loadable shared library `uipc_backend_cuda`). Entry point `entrance.cpp` → `cuda::SimEngine` (`sim_engine.h` + `engine/*.cu`). The GPU id comes from the engine config `gpu/device`.

## Internal Build Components

The backend remains one DLL and performs one final CUDA device-link. Its 198
compiled `.cu/.cpp` files are owned by seven primary OBJECT components plus one
optional legacy-collision component:

| Component | Owned domains |
|---|---|
| `cuda_runtime_objects` | Root/engine, global geometry, animation, time integration, pipeline, diagnostics, and orchestration |
| `cuda_affine_body_objects` | ABD state, constitutions, constraints, joints, and subsystem assembly |
| `cuda_collision_objects` | Default broad phase and current trajectory filters |
| `cuda_collision_legacy_objects` | Optional V0, stackless, and linear-BVH simplex trajectory filters |
| `cuda_contact_objects` | Active set, contact, DyTopo, distance, and inter-primitive effects |
| `cuda_fem_objects` | FEM data, constitutions, constraints, MAS, and subsystem assembly |
| `cuda_linear_system_objects` | Global sparse assembly, PCG, SpMV, and preconditioners |
| `cuda_coupling_objects` | ABD/FEM coupling receivers and subsystems |

`components.cmake` and `components.lua` are the ownership manifests. Both
reject a compiled source that is unowned or assigned twice. OBJECT targets use
RDC, while `uipc_backend_cuda` alone resolves device symbols and exports the
module ABI. This preserves static `REGISTER_SIM_SYSTEM` objects without
turning domains into runtime DLL boundaries.

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
    │   ├── Detect DCD Candidates          (iter > 0)
    │   ├── Compute DyTopo Effect          (conditional: Assemble → Convert → Distribute)
    │   ├── Solve Global Linear System     (Build → PCG: Apply Preconditioner + SpMV)
    │   └── Line Search
    │       ├── Detect Trajectory Cand.
    │       ├── Compute Energy             (initial E0)
    │       ├── Filter CCD TOI
    │       ├── Compute CFL Condition
    │       └── Line Search Iteration      (LOOP: Filter Contact Cand. → Compute Energy)
    └── Update Velocity
```

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
trials, cumulative iterative-linear-solver iterations, limit-hit flags, and the
last line-search/CCD/CFL step factors. `reset_frame_stats()` runs immediately
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

The `info_stackless_bvh_v0` selector is a legacy comparison path, while
`stackless_bvh` and `linear_bvh` are alternate broad phases. All three
registered simplex filters live in `cuda_collision_legacy_objects` and are
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
all active broad-phase queries first, gathers the four queue counters into a
single device array, and downloads them with one synchronization. If a count
exceeds its queue capacity, only that queue grows and reruns. PP/PE/PT/EE CUB
selection counts use the same contiguous one-copy pattern. The public
`InfoStacklessBVH::detect/query` methods remain synchronous for independent
callers; `launch_detect/launch_query` plus `prepare_query_result` are the
internal batching interface. Do not consume a launch-only query result before
the count batch has been synchronized and finalized.

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

- Launch block size is auto-selected by occupancy via `cuda_tool::best_block_dim(kernel)` (consistent with muda's occupancy-based selection, guaranteeing unchanged FP behavior); the grid is computed by `cuda_tool::best_grid_dim(n, kernel)`. **The block-size cache is keyed by kernel function address** (`unordered_map<const void*, int>`); it must not be a `static` cache keyed by function-pointer type — different kernels with the same signature would share a cache entry, and cross-kernel block-size interference perturbs the atomic reduction order, which once caused a case 36 frame 17 line-search failure in the full test suite (not reproducible when run in isolation).
- Memory operations such as `buffer::kernel_fill<int>` uniformly go through `cuda_tool::BufferLaunch` (internally named template kernels).
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
| `stream.h` | `CUDA_TOOL_CHECK` error checking, `default_stream`, `Stream` (`Stream::Default()`) |
| `view.h` / `view_nd.h` | `CBufferView/BufferView/VarView/CVarView`, `Dense/CDense` (scalar viewers), `ViewerBase`, `Extent2D`, `Buffer2DView`, `Dense1D/Dense2D` (with `make_dense_1d/2d`) |
| `launch.h` | `best_block_dim/best_grid_dim` (occupancy-based block size / grid computation) |
| `buffer.h` | `DeviceVector/DeviceBuffer/DeviceVar/DeviceBuffer2D`, `BufferLaunch`. `resize()` value-initializes growth; `resize_discard()` is for fully regenerated output and grows to 150% of the latest requirement; `resize_preserve()` retains the old logical range without initializing the new range. Exact and amortized preserve/discard reserve variants make ownership and growth policy explicit. |
| `cub.h` | Thin wrappers (pointer-style) for `DeviceReduce/Scan/Select/Partition/RadixSort/MergeSort/RunLengthEncode` + warp-level cub headers; **temporary storage uses a stream-level workspace cache** (`details::cub_temp_storage`, grows 2× on demand, reused across calls — per-call cudaMalloc/cudaFree costs ~10-100µs each and implicitly synchronizes with the device, which once caused a ~15%/frame performance regression in the 6_wrecking_balls scene) |
| `linear_system.h` + `linear_system/views.h` | `DeviceTripletMatrix/DoubletVector/DenseVector/BCOOMatrix/BSRMatrix/DenseMatrix` + full set of views (Triplet/Doublet/DenseVector/BCOO) + `LinearSystemContext` (cublas dot/norm) |
| `eigen.h` + `eigen/` | Device-side small-matrix math (`eigen::evd/svd/pd/inverse/atomic_add`; ported verbatim from muda ext/eigen, bit-for-bit identical) |
| `debug.h` | `debug_sync_all/check_finite` + the `UIPC_KERNEL_ASSERT/ERROR/WARN` macro family (gated by `uipc::RUNTIME_CHECK`) |
| `graph.h` | `GraphCapture`: self-contained CUDA-graph block capture/replay (capture body on a capture stream, replay via `launch_sync()`, permanent fallback on capture failure). Instantiation API is selected by `CUDART_VERSION` (`cudaGraphInstantiateWithFlags` ≥11.4 through 13.x, legacy 5-arg form below), so the same source builds across CUDA 11/12/13 |
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
- **The umbrella header `cuda_tool.h` does not include `cub.h`**: the CCCL device-algorithm headers are extremely heavy (~165k extra expanded lines per TU), so the ~23 files using the `Device*` wrappers explicitly `#include <cuda_tool/cub.h>`; `linear_system.h` no longer transitively includes cub.h either.
- **RDC must be enabled** (`CUDA_SEPARABLE_COMPILATION ON` + `CUDA_RESOLVE_DEVICE_SYMBOLS ON`): `affine_body/utils.cu` defines `UIPC_GENERIC` free functions (e.g. `q_to_transform`) that are called across translation units by device code in other TUs; disabling RDC gives `ptxas fatal: Unresolved extern`. The xmake-side equivalent is `add_cuflags("-rdc=true")`.
- **Lesson learned (correction dated 2026-08-20)**: CMake+ninja does not track compile-flag changes (purely mtime-driven) — after switching a global flag like RDC, a "full build" may actually only recompile TUs whose source files changed, and stale objects can slip through linking and create a false "verification passed" impression; after changing global flags you must manually clean the object directory before verifying. This once led to the wrong conclusion that RDC could be disabled (the RDC part of commit d2f48087 has since been corrected).
- Compile-time profile (32 cores, CUDA 13.2, RTX 5090): full build ~4.7 min / ~8800 CPU·s, 219 .cu files averaging ~35s; a single TU's preprocessed expansion is ~1.63M lines, of which CUDA toolchain headers ~860k, WinSDK ~310k, MSVC STL ~150k, Eigen ~150k, the project itself <20k — the bulk is toolchain headers; the project headers have already been trimmed as far as possible.
- The smoke test is kept as `test_compile.cu.txt`.

## none Backend

`src/backends/none/`: an empty implementation: it increments the frame and logs,
but performs no physics. It is useful as a backend template and for limited core
interface/geometry checks. It does not enter/solve the Scene pending-geometry
cycle, so post-init object additions remain pending. **Environments without a GPU
have no usable compute backend**.
