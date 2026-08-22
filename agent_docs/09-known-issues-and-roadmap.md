# 09 — Known Issues, Tech Debt, and Roadmap

Status as of 2026-08-23 (post PR #468 merge). Completed work is recorded in
`handoff.md`; this file tracks what is **open** — analyze here first before
planning new work.

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

**Planned levers (in priority order)**:
1. ~~Cooperative-groups persistent-kernel fusion or CUDA-graph capture of the
   PCG inner loop~~ **DONE (2026-08-23)**: FusedPCG now replays
   `check_interval`-sized iteration blocks as CUDA graphs
   (`linear_system/use_cuda_graph`, default on; details in doc 05).
   Remaining within-PCG cost is the ~83-iteration count itself.
2. Fuse exact distance tests into BVH query predicates; materialize only
   active pairs (kills the 450k-candidate overhead).
3. SNH/FEM assembly kernel throughput (profile `make_spd` EVD alternatives
   that keep the convergence behavior).
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

- pytest `< 9.0.3` tmpdir CVE → fixed on main (PR #469, pytest pinned
  `>=9.0.3`, lock at 9.1.1). Dev-only dependency.
- usd-core `< 25.8` (critical) is marked fixed in the repo, but USD is a
  local/optional dep — upgrade any local install to `usd-core >= 25.8`.

## Samples repo state (spiriMirror/libuipc-samples)

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
