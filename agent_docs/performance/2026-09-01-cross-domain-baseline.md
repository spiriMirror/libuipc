# 2026-09-01 — Cross-Domain Main Baseline

- Status: Accepted as the current machine-specific reference
- Comparison: measurement-only; no simulation algorithm changed
- libuipc commit: `3e08e005fbcacbf42799b6b8555521edc9b82462`
- libuipc-samples commit: `8701983696dbf03855471646c478580f3b370279`

## Question

What is the current end-to-end performance envelope on `refactor-main` after
the completed CUDA, solver, collision, shell, and default-configuration work?
The previous documents mixed different revisions, frame windows, and scene
parameters, so their 73/156/301 ms values are historical experiment evidence,
not a current baseline.

## Environment

| Field | Value |
|---|---|
| GPU / driver | NVIDIA GeForce RTX 5090, driver 595.79, WDDM display device |
| CUDA toolkit | 13.2 (`nvcc` 13.2.51) |
| OS / Python | Windows 11 10.0.26200, CPython 3.14.2 |
| Build | Release, native `sm_120`, locally built CUDA backend |
| Backend binary | `uipc_backend_cuda.dll`, SHA-256 `87E647D634BA259D6EA35BA72B79E3C5A7B70B0DCD2864B4A2563B07E7CC662C` |
| Installed package | development package reports `0.9.0`; installed DLL hash matches `build/Release/bin` exactly |
| Worktree | tracked files clean; these archived runs were made before the runner separated untracked paths, so their `dirty=true` reflects only the known sibling/reference directories `GPU_IPC/`, `Stiff-GIPC/`, and `references/` |

The GPU also drove the desktop and other GUI processes. Three interleaved fresh
processes were used to expose this variability rather than hide it. Timings are
valid as a reference for this machine and revision, not as a universal CI
threshold.

## Workload and method

| Manifest name | Sample | Domain | Frames per run |
|---|---:|---|---:|
| `rigid-wrecking-balls` | 6 | pure affine-body contact | 120 |
| `stiff-gipc-case2` | 88 | two FEM bunnies, cloth, MAS, contact | 250 |
| `mas-bunny` | 89 | stiff FEM with MAS and ground contact | 100 |
| `cube-wall-cloth` | 93 | 1920 affine bodies coupled to pinned cloth | 100 |

Each workload ran three times in interleaved/reversed order. There was no
warmup skip: frame 1 through the declared final frame is the workload contract,
including graph construction, free fall, impact, and settling. The timed region
is `world.advance(); world.retrieve();`; structured `engine.frame_stats()` is
read immediately afterward, outside the interval. Throughput runs set
`UIPC_BENCHMARK_TIMERS=0`.

Total device memory was sampled through `nvidia-smi` every 0.2 seconds. The
reported delta is peak total device usage minus the pre-launch total. On WDDM
it includes concurrent display/application movement and is an approximate
upper envelope, not process-private allocation.

## Throughput results

The reference is the median of the three run means. `Frame median` and `P95`
are the medians of those per-run statistics. PCG is the total linear-solver
iteration count per simulation frame, across all Newton solves.

| Benchmark | Three run means (ms/frame) | Reference | Frame median | P95 | Newton/frame | PCG/frame | Peak-memory delta range |
|---|---|---:|---:|---:|---:|---:|---:|
| rigid wrecking balls | 129.5, 131.3, 126.4 | **129.5** | 126.4 | 211.6 | 3.95 | 107.3 | 5450–5930 MiB |
| stiff-GIPC case2 | 230.6, 199.0, 201.1 | **201.1** | 205.6 | 255.6 | 6.64 | 246.8 | 4181–7516 MiB |
| MAS bunny | 60.2, 60.1, 62.0 | **60.2** | 64.4 | 96.5 | 4.67 | 358.8 | 3243–3245 MiB |
| cube wall + cloth | 125.6, 165.0, 121.8 | **125.6** | 118.0 | 272.9 | 5.13 | 200.1 | 3909–7282 MiB |

The high case2/93 runs are retained. Their iteration counts stayed close to the
other repeats, while both wall time and sampled memory increased; this is the
observed WDDM/dynamic-contact envelope, not evidence for a deterministic
throughput regression.

## Synchronized stage diagnostics

One separate diagnostic run enabled Timer scopes over a contact-bearing window.
Timer scopes synchronize around nested regions, so the following values locate
cost but are not substituted for the throughput table. Values are milliseconds
per Newton call; parent and child rows are inclusive and must not be added.

| Benchmark (frames / Newton calls) | Newton | Global solve | FusedPCG | Subsystem assembly | Line search | Trajectory detect | DyTopo | inner DCD detect |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| rigid (100 / 355) | 33.64 | 19.76 | 8.78 | 6.66 | 6.62 | 4.01 | 3.98 | 4.53 |
| case2 (60 / 326) | 27.88 | 11.64 | 4.64 | 2.80 | 7.88 | 4.89 | 4.66 | 4.49 |
| MAS bunny (60 / 288) | 12.72 | 8.53 | 5.32 | 0.68 | 2.58 | 1.66 | 0.55 | 1.30 |
| wall + cloth (70 / 388) | 21.84 | 11.17 | 6.23 | 2.92 | 4.70 | 2.19 | 3.85 | 2.55 |

This baseline does not support one universal hotspot claim. The rigid scene is
global-solve/assembly heavy; MAS bunny is dominated by the linear solve; case2
spreads time across global solve, line search, trajectory detection, DyTopo,
and DCD; the wall/cloth scene is split between global solve, line search, and
DyTopo.

## Correctness and numerical variability

- All 12 throughput runs and all four Timer diagnostics returned zero.
- Every recorded frame reported `completed=true` and `converged=true`.
- No frame hit the Newton or line-search limit.
- MAS bunny is the stable numerical sentinel: the maximum distance between its
  three final centroids was `1.72e-6` m.
- Collision-rich scenes are not bitwise deterministic. At frame 120 the rigid
  ball centers spanned x `8.009–8.213`, y `2.459–2.811`, and z
  `-0.083–0.052` m. The case2 final centroids also diverged after repeated
  contact, and the wall/cloth final minimum y spanned `0.2776–0.2926` m.
  Their physical observables are recorded as envelopes, not exact golden
  values.

## Interpretation

The old Stiff-GIPC ratios are not refreshed here. The historical reference used
different source revisions, some different frame windows, and in the wrecking
ball case a different aligned scene configuration. A new cross-project ratio
requires rerunning Stiff-GIPC under a separately documented equivalent contract.

The old 156–157 ms case2 number covered a 60-frame optimization window. The
current canonical contract covers all 250 frames, including later repeated
impact/stacking phases, and therefore must not be compared directly. This run's
own 60-frame Timer-enabled window averaged 156.3 ms/frame, confirming that the
window length—not a single unexplained regression—accounts for much of the
difference.

## Decision

Use this record as the current human-reviewed reference for `refactor-main`.
Future performance work must repeat the same manifest entry and frame count,
compare structured iteration counts, and report three runs. A stable dedicated
GPU runner may promote these values into an enforced `uipc.profile` baseline;
the shared WDDM machine should retain ranges rather than a hard wall-time gate.

No simulation algorithm or scene parameter was changed by this baseline work.

## Reproduction and artifacts

Throughput commands:

```shell
python scripts/run_benchmark.py run rigid-wrecking-balls
python scripts/run_benchmark.py run stiff-gipc-case2
python scripts/run_benchmark.py run mas-bunny
python scripts/run_benchmark.py run cube-wall-cloth
```

Diagnostic commands add `--env UIPC_BENCHMARK_TIMERS=1` and used 100, 60, 60,
and 70 frames respectively. The ignored raw JSON/log archives are under
`output/benchmark-runs/`; throughput run IDs are `20260831T212459Z`,
`212848Z`, `212920Z` (rigid), `212522Z`, `212748Z`, `213018Z` (case2),
`212630Z`, `212735Z`, `212943Z` (MAS), and `212645Z`, `212709Z`, `212957Z`
(wall/cloth). Diagnostic IDs are `213257Z`, `213318Z`, `213243Z`, and
`213338Z`.
