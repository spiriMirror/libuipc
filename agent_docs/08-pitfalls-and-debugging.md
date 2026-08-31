# 08 — Pitfalls and Debugging Playbook

Every entry here cost real debugging time. Read the relevant section **before**
touching that area; several of these have bitten us more than once.

## Build & CI

- **Build the `pyuipc` target after backend `.cu` changes.** Its link now
  explicitly depends on every backend target file, so a backend rebuild relinks
  pyuipc and the POST_BUILD step refreshes `build/python`, stubs, runtime
  libraries, and site-packages. Building only `cuda` or `none` still cannot run
  pyuipc's packaging hook. In that case, either build `pyuipc` next or invoke
  `scripts/after_build_pyuipc.py` with the arguments emitted by CMake (target
  pyd, project/binary dirs, config/build type, and `--build_wheel OFF`). A stale
  dll once made a whole performance-alignment round compare against an old
  binary.
- **Changing `IEngine` virtuals requires every backend's most-derived vtable to
  be rebuilt.** Do not trust a core-only relink: explicitly rebuild `none` and
  `cuda`, then run the Python post-build sync above. A mixed old/new vtable can
  dispatch `status()` or `frame_stats()` into a neighboring virtual slot and
  appear as recursion, an unrelated assertion, or an invalid world.
- **CMake+ninja does not track compile-flag changes** (purely mtime-driven).
  After flipping a global flag (e.g. RDC), stale objects slip through the
  link and fake a "verified" result — clean the object dir before believing
  it. This produced the wrong "RDC can be disabled" conclusion (RDC must
  stay ON: `affine_body/utils.cu` `UIPC_GENERIC` free functions are called
  cross-TU; off → `ptxas fatal: Unresolved extern`).
- **`file(GLOB ... CONFIGURE_DEPENDS)`** is now on all project CMake files:
  adding/removing sources does NOT need a manual re-configure (the older
  note in handoff's environment section predates this).
- **ccache is forbidden by the owner.** Root `xmake.lua` now explicitly sets
  `build.ccache=false` for every configuration, including `dev=true`. Do not
  re-enable compiler-cache wrappers in either CMake or XMake.
- **GitHub regenerates release tarballs** → pinned source hashes go stale.
  Our fix pattern: overlay port in `ports/` + `overlay-ports` in the
  *generated* `vcpkg-configuration.json` (env vars don't reach
  cibuildwheel's inner `vcpkg install`). See doc 07 "CI dependency-drift
  incidents".
- **`.github/workflows/*.yml` are CRLF files** — edit them with a
  CRLF-aware tool (python binary read/write), not a naive LF editor.
- **Do not load scripts from `polyfill.io` in the documentation.** The
  service returns HTTP 401 and causes browsers to show an authentication
  popup on every page load. MathJax 3 does not need that ES6 polyfill; both
  MkDocs configurations intentionally load only the local MathJax setup and
  the MathJax bundle from jsDelivr.
- The documentation workflow must watch both `mkdocs*.yml` and
  `mkdocs*.yaml`. The repository's real configurations use the `.yaml`
  suffix; watching only `.yml` silently leaves configuration-only fixes
  undeployed.
- In-progress GitHub Actions jobs expose logs only for **completed steps**;
  grep on a running step returns nothing. Use `gh run view --job <id>` for
  step status instead.
- clang-format matching CI (v18): `output/venv_clangfmt/Scripts/clang-format.exe`.
- `gh` CLI on this machine: `"C:/Program Files/GitHub CLI/gh.exe"` (not on
  PATH), logged in as KemengHuang.

## Test suite (apps/tests)

- **Catch2 v3.8 multi-spec filtering**: separate argv entries are
  AND-intersected ("No tests ran"). OR = comma-separated in ONE argv:
  `./uipc_test_sim_case.exe "0_abd_gravity,13_fem_3d_gravity"`.
  `--list-tests --verbosity quiet` lists one case per line.
- **Cross-case pollution is real in the single-process suite.** The
  occupancy-cache incident: `best_block_dim` cached per function-pointer
  *type*, so same-signature kernels shared block sizes; atomic float
  reduction order shifted and case 36 frame 17 failed only in the full
  suite, never in isolation. Fixed by keying the cache on kernel address —
  keep it that way. To separate pollution from case-local bugs, use
  `scripts/run_sim_case_isolated.py` (one process per case).
- Full suite baseline on record: **95 cases / 14214 assertions** green
  (multiple runs). `uipc_test_sim_case.exe` ≈ 15 min.
- The line-search max-iteration warning/exception now prints
  `alpha_last/E0/E_last/rel_E_increase/ccd_alpha/cfl_alpha` — judge real
  regression vs ULP jitter from those numbers before hunting.

## Performance measurement

- **Build contention pollutes timing completely.** A hot build once made
  sanity_check look like a 112 ms/frame culprit (idle machine: 2-5 ms).
  Benchmark only on an idle machine.
- **After contact activates, frame-to-frame A/B comparison across binaries
  is meaningless**: different block sizes → different atomic arrival order →
  ULP differences → trajectory divergence. Compare only pre-contact
  segments, or recompute marginal costs (instrumented runs) instead of
  naive frame diffs.
- **Stiff-GIPC's logged "average time cost" is per-Newton-iteration GPU
  time** (`totalTime/totalNT`), not per-frame. Its clean reference numbers:
  wrecking ball 42.8 ms/frame (frames 2-120), case2 142.8 ms/frame.
- cub wrappers keep a **stream-level workspace cache**
  (`cuda_tool/details::cub_temp_storage`); per-call cudaMalloc/cudaFree
  (~10-100 µs + implicit sync each) once cost ~15%/frame. Never go back to
  per-call temp allocation.
- `linear_system/check_interval` tuning is a **negative result** (5→25 did
  nothing): PCG's cost is iteration count, not convergence-check stalls.
- **`make_spd` Cholesky early-out is a NEGATIVE RESULT (2026-08-23)**:
  gating the 9x9/12x12 EVD behind an in-register Cholesky PD test made
  every constitution/contact kernel 1.7-2.7x SLOWER (SNH 3.9->6.7 ms,
  contact assemble 1.9->5.3 ms, bending 1.3->2.9 ms per call) — the local
  L matrix (81/144 doubles) blows the register budget of already-fat
  kernels and occupancy collapses. Do not retry unless the PD test is
  register-free. RESOLVED differently (2026-08-24): we adopted Stiff's
  SNK1 wholesale (their energy convention + their analytic eigensystem),
  so the generic 9x9 EVD is simply gone from the SNH kernel — see doc 04.
- **Same-address atomic storms** in reduction kernels: one atomicAdd per
  warp on a single counter costs 20-30 µs/call at ~300k threads — always
  do warp→block two-level reduction (done for `Spmv_rbk_sym_spmv_dot` and
  `fused_dot`).
- **Cloth membrane `1/sqrt(I5)` singularity** (`strain_limiting_baraff_
  witkin_shell_2d.h`): a fully collapsed triangle (directional stretch
  λ→0) produced inf/NaN forces — and even the NEAR-singular regime was
  silently poisoning PCG conditioning (case 88: 145 PCG/solve with
  pathological 14-Newton frames). Fixed with a 1e-12 floor on I5; case 88
  dropped 266 -> 203 ms/frame as a side effect (35/solve now). Stiff-GIPC
  has the same unguarded hole.
- **Cloth vs Stiff-GIPC alignment audit (2026-08-24)**: the SLBWS membrane
  (energy/gradient/Hessian incl. the one-sided cubic limiter and both
  analytic SPD projections) matches Stiff's Baraff-Witkin term for term.
  Convention differences are stiffness mappings (ours `E·(2r)/(1-ν²)` per
  area; Stiff `E/(2(1+ν))` per area×thickness) and the bending model
  (ours: dihedral hinge with rest angle + per-Newton 12x12 EVD; Stiff's
  default build: flat-rest quadratic cotangent-Q with a constant PSD
  Hessian — their perf edge, our feature edge).

## Python / runtime API

- **Named C++ backends need explicit process initialization.** Call
  `uipc::init` with an existing `module_dir` before `Engine{"cuda", ...}`. The
  Python package does this automatically by pointing at its `_native` directory.
- **`World` holds only a weak reference to `Engine`.** If `Engine` goes out
  of scope (e.g. created inside a builder function that returns the world),
  the next `advance` dies with "Engine is expired". Keep the engine object
  alive alongside the world.
- **`Scene(config)` copies the config dict by value.** Mutating the dict
  after construction silently does nothing — set every key before
  `Scene(config)`.
- **Unregistered config keys are now a hard error** (guided exception).
  Register defaults in `scene_default_config.cpp` when adding keys, and
  confirm via the log line that the value took effect.
- **`Logger.set_level(Logger.Level.Off)`** silences text output (the samples
  use it); `Warn` keeps kappa-clamp warnings visible.
- stdout redirection block-buffers printf in some native code — a timeout
  kill loses logs; instrumented prints need `fflush(stdout)`.
- **Stale-dll trap when benchmarking solver changes through the python
  samples:** the samples load the dlls from
  `<python>\\Lib\\site-packages\\uipc\\_native\\`, NOT from `build/`. After
  rebuilding backend `.cu` files, re-copy `build/Release/bin/uipc_*.dll`
  into both `build/python/src/uipc/_native/` and the site-packages
  `_native/` dir before running — otherwise python silently runs the OLD
  solver and trajectory/timing comparisons are garbage (this once cost a
  full debug round chasing a MAS "freeze" that was just a stale dll).
- **Do not use incremental Scene commits as a general replication protocol.**
  The current update path omits subscene changes, does not faithfully propagate
  geometry removal, and assumes append-aligned geometry IDs. Use full SceneIO for
  topology-changing transfers and read doc 11 before extending commit/update.
- **The `none` backend is not a CPU simulator.** It advances the frame without
  physics and does not settle post-init pending geometry mutations.

## Simulation semantics (contact/constraints/ABD)

- **ABD bodies: `geometry().positions()` are REST positions — forever
  constant.** The motion lives in `instances().find(builtin.transform)`
  (4×4). Any centroid/world-position readback must go through the instance
  transform.
- **`aim_transform` vs `transform`**: SoftTransformConstraint pulls the body
  toward `aim_transform` during `advance`. Writing `builtin.transform`
  teleports the body visually but the next advance yanks it back toward the
  stale aim. Teleport recipes must update the aim too
  (`controller.apply_to(attr=builtin.aim_transform)`), and rate-limited
  drivers converge over frames — a single "snap" that only writes
  `transform` is a classic broken probe.
- **Dragging GUI sliders while paused writes `builtin.transform` directly
  (pure kinematics, no contact solve)** — visible interpenetration in pause
  mode is expected, not a contact bug. Judge contact only while running.
- **Sanity check failure permanently invalidates the world**
  ("World is not valid, frame set to 0"): intersection reports set
  `m_valid=false` and `advance` becomes a no-op. Use sanity_check=enable as
  a *probe* (e.g. verifying an initial pose is penetration-free), not in
  production scenes where mid-run contact is expected.
- **Implicit ground half-plane CCD asserts `toi > 0`**: an object starting
  *exactly* on the plane (gap == 0, moving into it) reports `toi == -0` and
  aborts. Start objects ~1 mm above the ground and let contact engage
  through the normal path.
- **URDF `move_root(xyz, rpy)` axis mapping** (calibrated on the robot-hand
  URDF): `xyz[0] → world -z`, `xyz[1] → world -x`, `xyz[2] → world +y`.
  World-up adjustments go on index 2.
- `SceneIO.write_surface` merges ALL objects' surfaces into one .obj,
  vertices ordered by geometry creation (the last N verts belong to the
  last-created geometry) — usable for cheap per-object tracking.
- polyscope automation: `ps.screenshot(path)` inside the user callback gives
  ground-truth renders; `ps.unshow()` exits the loop from code.
- **The kappa-clamp warnings at startup** ("Contact kappa 0 ... clamped to
  100000000") are the min/max policy working as designed on
  disabled/zero-stiffness pairs — not an error (handoff: "Default kappa
  policy").

## GPU / cuda_tool

- Kernel launch block sizes come from `cuda_tool::best_block_dim(kernel)`
  with a **per-kernel-address** occupancy cache (see the suite-pollution
  entry above — never key it by type).
- **CUDA-graph capture traps** (learned the hard way in the FusedPCG graph
  work):
  - A captured graph bakes every kernel's **by-value arguments** — including
    counts embedded in views. Either keep counts out of the launch entirely
    (device-side count + capacity-sized grid) or include them in the
    validity key.
  - View accessors may bounds-check against the capture-time count
    (`TripletMatrixViewT::operator()` does): a graph-stable kernel must read
    through the view's raw pointers (`row_indices().data()`, …) instead.
  - **`cudaGraphNodeParams.conditional.phGraph_out` is an OUTPUT**: after
    `cudaGraphAddNode` returns, the driver has written a pointer to its own
    body-graph array into the params struct — read
    `params.conditional.phGraph_out[0]`; do NOT point it at your own
    variable (the node is created but the body graph handle is never
    populated and the error code stays "success").
  - **Blocking streams do not wait for legacy-default-stream work.** A
    `cudaMemcpyAsync` H2D on the default stream followed by
    `cudaGraphLaunch` on a blocking stream races (we read an uninitialized
    device tolerance → `dx=0` → flat line-search energy). Upload
    graph-consumed host data with a *synchronous* `cudaMemcpy`.
  - A launch into the legacy default stream during capture invalidates the
    capture (detected at `cudaStreamEndCapture`) — use that as the automatic
    fallback signal for un-plumbed callees.
  - Graph replay vs plain launches can schedule blocks differently →
    different atomic-add arrival order → ULP divergence → chaotic scenes
    diverge after contact onset. Expected; do not chase bit-equality there.
  - **Use a BLOCKING stream as the capture stream** (cudaStreamDefault).
    With a non-blocking capture stream, a callee launching into the legacy
    default stream during capture neither invalidates the capture nor joins
    the graph — the kernel just EXECUTES, uncaptured: the graph then replays
    with that work frozen at its capture-time output. (In FusedPCG this made
    MAS scenes replay with a frozen preconditioner output — false rᵀz
    collapse, "converged" in 5 iterations with residual 3.3×‖b‖, wrong
    physics at E=1e4. Caught by a dump-based residual check + a trajectory
    A/B against the diagonal preconditioner.)
  - **A WHILE conditional node is not automatically faster than host-checked
    block replay**: per-iteration condition evaluation + body dispatch cost
    real microseconds; measured on ~85-iteration solves the block path (host
    check every 5) wins by ~10%. Graphs kill *launch* gaps, not *loop-control*
    gaps — amortize the control decision over several iterations.
- `DeviceVector::resize(n)` value-initializes the grown tail (thrust
  parity); `loose_resize_entries` deliberately does NOT initialize —
  see below.
- **Doublet/triplet reporter contract**: if you report N slots at
  `report_extent`, you MUST write all N slots at assemble time (zeros
  included). Buffers are `loose_resize`d (uninitialized) and the merge
  kernels `atomic_add` every slot — holes mean garbage indices → OOB
  atomics/NaN. (Found reviewing PR #461, which skipped zero gradients.)
- Event-hook placement matters: `on_init_scene` fires once, but
  `detect_dcd_candidates` also runs inside the Newton loop
  (`advance_ipc.cu`, iter>0). Anything that must track the current iterate
  (e.g. a driven mesh's forward pass) has to hook inside the loop, not
  once per frame.
- `make_spd`'s 9×9 EVD is a throughput suspect in FEM assembly, but
  removing it worsens Newton convergence — measured; don't simply delete.
- nvcc needs the MSVC environment (vcvars64); a bare shell reports
  "Cannot find compiler 'cl.exe'". MSVC + CUDA≥13 needs `/Zc:preprocessor`;
  cuda_tool TUs need `--extended-lambda --expt-relaxed-constexpr`.
