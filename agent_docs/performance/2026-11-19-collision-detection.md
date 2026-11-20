# 2026-11-19 — Collision detection: BVH refit, detection reuse, FP32 rejection (PR 2/6)

- Status: Proposed
- Before commit: `03652516d70fd39adeefe809fb308626ba7c425c` (`perf/1-cuda-tool`, PR 1/6)
- After commit: `3704967ce297009930c565b89ce864f92d335ad3` (`perf/2-collision`, PR 2/6) (built from `dcdf53ba`, which has the same code but for the register cap of the warp-cooperative query, which the branch drops; PR 2's evidence record has an A/B of that change)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Does PR 2 (the LBVH scene-bounds fix; BVH build, refit and queries; reuse of detection results across Newton iterations and frames; FP32 rejection and stored pairs; leaner CCD launches) reduce frame time while the active sets and the TOI stay the same (the trajectory candidate lists may differ only by pairs that can reach neither)? The scope expected to move is collision detection in every scene with contact; the iteration counts should not move.

## Environment

| Field | Value |
|---|---|
| GPU / driver | NVIDIA RTX PRO 5000 72GB Blackwell / 580.178.04 (timing card held by the benchmark job alone; the sim cases ran on a second card of the same model) |
| CUDA toolkit | 13.4.92 |
| OS / compiler | Linux 7.0.0 x86_64, glibc 2.39 / GCC 13.3.0 |
| Build type and important flags | Release; CMake with Ninja, `CMAKE_CUDA_ARCHITECTURES=120`, `UIPC_WITH_CUDA_BACKEND=ON`, `UIPC_BUILD_PYBIND=ON`, `UIPC_BUILD_TESTS=ON`, `UIPC_BUILD_EXAMPLES=ON`, `UIPC_BUILD_BENCHMARKS=ON`, `UIPC_WITH_CUDA_LEGACY_COLLISION=ON`, no other options; Python 3.12.3 |
| Worktree state | clean: every run record has `libuipc` and `libuipc-samples` without local changes, the runtime package and the scene inputs had the same fingerprints before and after every run |

## Workload and method

The five scenes of `benchmarks/manifest.json` ran at their default frame counts (`anisotropic-cloth-small` 30, `rigid-wrecking-balls` 120, `stiff-gipc-case2` 250, `mas-bunny` 100, `cube-wall-cloth` 100) and canonical environment (`WB_LOG=Warn`, `UIPC_BENCHMARK_TIMERS=0`), each with `python scripts/run_benchmark.py run <scene>` in a fresh process. The timer is the samples' own per-frame wall time of `world.advance()` plus `world.retrieve()` (`world.init()` is outside it; no frame is excluded). A run's value is the mean over its frames; a build's value is the median of its five run means. No profiler and no stage timer ran (`UIPC_BENCHMARK_TIMERS=0`), so the only timer is the per-frame wall time, which includes everything a frame does.

Seven builds with the same flags (main and the six PR tips) were measured in one job of five rounds. In each round every scene ran on all seven builds back to back, in an order rotated by three places per round, so that each build took five different positions. The job held the timing card alone; before every run it checked the card for other processes and for utilisation above 10% in two samples a second apart, and found no foreign load before any of the 175 runs. The second card ran correctness jobs and was busy during 21 of the 175 runs; a run's mean relative to the median of its build and scene was 1.0000 with that card busy and 1.0004 with it idle.

Newton iterations per frame and PCG iterations per Newton iteration are medians over the runs' frame statistics. The p-values are exact two-sided Mann-Whitney tests of one build's five run means against another's; with five against five, 0.008 is the smallest possible value (every run of one build faster than every run of the other).

## Correctness and safety

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 52 test cases (main: 28; 20 added by this PR), all passing (`lbvh` fails on main; this PR fixes its CPU oracle). `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 254 passed, 0 failed (main: 254 passed, the same tests).
- Focused tests of this PR: `bvh_scene_bounds` (three builders against one CPU oracle), `info_stackless_bvh` (refit against rebuild, cooperative against per-lane traversal, forked chains), `active_set_reuse` (validation mode: reused and DCD multisets equal), `float_contact_lower_bound` (FP32 bounds against the FP64 tests and ACCD; coincident points get no bound), `swept_lower_bound_f32_certain` (against explicitly rounded FP32 evaluations, every FMA contraction), `ccd_launch_shape` (same bits with both launch shapes), `compact_active_lists`, `toi_min` (also a launch without values), `trajectory_prefix`, `cuda_tool_side_stream`, `cuda_tool_buffer`; all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison.

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| PR 1 | 88 | 7 | 19 | 0 | 0 | 1 |
| main (`158d39fd`) | 87 | 8 | 19 | 0 | 0 | 1 |

Sections above 1e-10 against PR 1 (26 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `5_abd_contact_ee_parallel.al-ipc` | tie | 4.8e-07 | 4.1e-01 | 1.2e-11 |
| `5_abd_contact_ee_parallel.ipc` | tie | 2.1e-08 | 3.2e-02 | 1.3e-09 |
| `9_abd_multi_constitution.ipc` | tie | 5.7e-03 | 7.4e-02 | 1.7e-11 |
| `12_abd_self_ground_contact.ipc` | tie | 3.2e-02 | 3.2e-02 | 2.3e-13 |
| `29_abd_fem_recover.default` | tie | 3.8e-03 | 3.9e-03 | 1.7e-07 |
| `43_abd_fem_state_access.default` | tie | 4.4e-02 | 4.3e-02 | 1.0e-12 |
| `80_abd_revolute_joint_driving_and_external_torque.default` | tie | 5.5e-09 | 3.7e-05 | 4.9e-14 |
| `8_abd_multi_contact_model.al-ipc` | chaotic | 8.8e-10 | 4.0e-01 | 8.5e-09 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 5.0e-06 | 3.4e-01 | 2.9e-02 |
| `11_abd_ramp_sliding.al-ipc` | chaotic | 1.1e-05 | 1.3e-02 | 6.4e-05 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 3.6e-02 | 1.3e-01 | 7.0e-02 |
| `18_abd_fem_contact.al-ipc` | chaotic | 3.4e-02 | 3.9e-02 | 8.2e-03 |
| `18_abd_fem_contact.ipc` | chaotic | 3.4e-03 | 2.0e-02 | 5.2e-03 |
| `23_kirchhoff_rod_bending.default` | chaotic | 2.6e-01 | 3.1e-01 | 1.8e-01 |
| `28_fem_periodically_pressed_tet.default` | chaotic | 4.2e-10 | 1.3e-09 | 1.0e-09 |
| `36_no_surf_but_contact_on.default` | chaotic | 1.5e-04 | 2.0e-04 | 3.9e-04 |
| `47_external_articulation_multijoints.default` | chaotic | 2.2e-05 | 2.1e-05 | 2.5e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 3.8e-03 | 6.8e-02 | 2.9e-03 |
| `56_fem_mas_bunny_ground.default` | chaotic | 1.1e-07 | 2.4e-07 | 2.9e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 2.3e-09 | 1.8e-07 | 6.2e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 2.0e-06 | 2.3e-06 | 2.2e-06 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 1.0e-09 | 1.0e-09 | 1.0e-09 |
| `60_fem_mas_cloth.default` | chaotic | 2.8e-09 | 2.2e-09 | 2.6e-09 |
| `61_fem_mas_rod.default` | chaotic | 4.7e-06 | 4.3e-04 | 2.6e-04 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 1.4e-05 | 1.2e-05 | 1.3e-05 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 1.4e-05 | 1.0e-05 | 1.0e-05 |

- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 9.28 | 9.27 | -0.2% (1.00x) | 5 runs each; run means 9.25–9.29 before, 9.26–9.30 after; Mann-Whitney p = 0.841; 1.00x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 143.08 | 119.58 | -16.4% (1.20x) | 5 runs each; run means 140.56–144.37 before, 117.10–121.73 after; Mann-Whitney p = 0.008; 1.21x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 3.925 | 3.967 | +1.1% | median of 5 runs; range 3.925–4.017 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.10 | 27.27 | +0.6% | median of 5 runs; range 27.16–27.37 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 226.03 | 169.53 | -25.0% (1.33x) | 5 runs each; run means 225.22–235.20 before, 168.25–170.03 after; Mann-Whitney p = 0.008; 1.33x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.680 | 6.680 | +0.0% | median of 5 runs; range 6.640–6.688 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 38.76 | 38.98 | +0.6% | median of 5 runs; range 38.64–39.31 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 67.95 | 57.83 | -14.9% (1.17x) | 5 runs each; run means 67.78–68.06 before, 57.80–57.91 after; Mann-Whitney p = 0.008; 1.17x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.14 | 78.13 | -0.0% | median of 5 runs; range 78.07–78.14 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 123.92 | 98.20 | -20.8% (1.26x) | 5 runs each; run means 122.12–126.17 before, 97.23–99.09 after; Mann-Whitney p = 0.008; 1.29x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.040 | 5.110 | +1.4% | median of 5 runs; range 5.040–5.120 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 38.67 | 38.76 | +0.2% | median of 5 runs; range 38.64–38.83 after |

## Interpretation

Measured: Faster than PR 1 (Mann-Whitney p < 0.05 on the five run means): `rigid-wrecking-balls` 1.20x, `stiff-gipc-case2` 1.33x, `mas-bunny` 1.17x, `cube-wall-cloth` 1.26x. Not distinguishable from PR 1 (p ≥ 0.05): `anisotropic-cloth-small` 1.00x (p = 0.841). Iteration counts: in 10 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs.

Inference: with the active-set multisets and the TOI unchanged by design (the frame-start reuse is asserted by `UIPC_CHECK_ACTIVE_SET_REUSE=1`), the gain comes from fewer and cheaper detection passes and fewer host round trips. This record has no stage timing, so it does not split the gain among the five changes of this PR. The primary metric is end-to-end ms/frame. The iteration counts are the check on the solver's path; with ten comparisons per PR, a single p-value below 0.05 is not unusual by chance.

Register cap: the measured build capped the warp-cooperative query at 128 registers per thread (`__launch_bounds__(K_OTHER_THREADS, 4)`). The branch drops the cap: the compiler may keep a pair predicate as a function of its own, and ptxas rejects a capped kernel's call to a function that needs more registers, which Windows builds for upstream's wheel architectures did. Measured: in an A/B of the capped and uncapped builds of PR 2 and PR 6 on the five scenes (five rounds, run order rotated, one GPU of the same model), the uncapped run means are 0.995 to 1.009 times the capped ones; only mas-bunny's differences reach p < 0.05 (exact Mann-Whitney), with the uncapped builds 0.2% and 0.5% faster, and the Newton and PCG counts are unchanged.

## Decision

Proposed for merge as PR 2/6, after PR 1/6. No config change; `frame_stats()` gains `active_set_reused` and `trajectory_reuses`.

## Reproduction and artifacts

For each build (main and the six PR tips, built with the flags above), five rounds in rotated order:

```shell
python scripts/run_benchmark.py run anisotropic-cloth-small
python scripts/run_benchmark.py run rigid-wrecking-balls
python scripts/run_benchmark.py run stiff-gipc-case2
python scripts/run_benchmark.py run mas-bunny
python scripts/run_benchmark.py run cube-wall-cloth
```

Unit tests of each tip:

```shell
for t in geometry core common sanity_check backend_cuda sim_case regression; do
    <build>/Release/bin/uipc_test_$t --reporter JUnit --out $t.xml
done
python -m pytest -m "not example" python/tests
```

The runner wrote one record per run in the format of `output/benchmark-runs/` (ignored; not committed). The sim-case harness is not part of this repository. No profiler output was taken.
