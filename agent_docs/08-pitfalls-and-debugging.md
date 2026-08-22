# 08 — Pitfalls and Debugging Playbook

Every entry here cost real debugging time. Read the relevant section **before**
touching that area; several of these have bitten us more than once.

## Build & CI

- **Backend `.cu` changes do not reach the Python package by themselves.**
  The post-build step syncs `uipc_*.dll` into `build/python/src/uipc/_native/`
  and site-packages only when pyuipc is *relinked*. If you only changed CUDA
  backend sources, sync manually:
  `cp build/Release/bin/uipc_*.dll build/python/src/uipc/_native/` and the
  same into `D:/Python/312-6/Lib/site-packages/uipc/_native/`. A stale dll
  once made a whole performance-alignment round compare against an old
  binary.
- **CMake+ninja does not track compile-flag changes** (purely mtime-driven).
  After flipping a global flag (e.g. RDC), stale objects slip through the
  link and fake a "verified" result — clean the object dir before believing
  it. This produced the wrong "RDC can be disabled" conclusion (RDC must
  stay ON: `affine_body/utils.cu` `UIPC_GENERIC` free functions are called
  cross-TU; off → `ptxas fatal: Unresolved extern`).
- **`file(GLOB ... CONFIGURE_DEPENDS)`** is now on all project CMake files:
  adding/removing sources does NOT need a manual re-configure (the older
  note in handoff's environment section predates this).
- **ccache was integrated and then reverted at the user's explicit
  request.** Nothing ccache-related remains; do not re-add without asking.
- **GitHub regenerates release tarballs** → pinned source hashes go stale.
  Our fix pattern: overlay port in `ports/` + `overlay-ports` in the
  *generated* `vcpkg-configuration.json` (env vars don't reach
  cibuildwheel's inner `vcpkg install`). See doc 07 "CI dependency-drift
  incidents".
- **`.github/workflows/*.yml` are CRLF files** — edit them with a
  CRLF-aware tool (python binary read/write), not a naive LF editor.
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

## Python / runtime API

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
