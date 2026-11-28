# 2026-11-19 — Line search and host synchronization (PR 6/6)

- Status: Proposed
- Before commit: `f95baff8a542e5062d2494dd832c4989138b9249` (`perf/5-linear`, PR 5/6)
- After commit: `2cefe209e57e1ac2b7ec5845ba2b3171157b0200` (`perf/6-line-search-host`, PR 6/6) (built from `e29ef77a`, which has the same code but for the register cap of PR 2's warp-cooperative query, which the branch drops; PR 2's evidence record has an A/B of that change)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Does PR 6 (the feasible step, the tolerance checks and the TOI beside the line search, the reused start energy, the cached graph of the fixed energy part, read-backs through mapped memory, batched copies and steps, one `uipc.view()` overload) reduce frame time with bitwise results? The scope expected to move is host-side waiting in the Newton loop, which matters most in scenes with short frames. The opt-in key `line_search/cubic_refinement` is off in every run of this record.

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

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 152 test cases (main: 28; 17 added by this PR), all passing (`lbvh` fails on main; PR 2 fixes its CPU oracle). `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 255 passed, 0 failed (main: 254 passed).
- Focused tests of this PR: `graph_capture` (`recapture()`), `readback_buffer` (the TOI, count, energy-total and axis-maximum producers on the mapped and the copy path), `step_batch`, `cuda_tool_copy_batch`, `cuda_tool_buffer`, `cuda_tool_readback_buffer`, `cuda_tool_side_streams`, `ipc_simplex_normal_contact_energy_fusion` (launch by capacity), `cubic_line_minimizer`, `python/tests/test_attribute.py` (`view()` of every creatable slot class); all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison.

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| PR 5 | 86 | 8 | 20 | 0 | 0 | 1 |
| main (`158d39fd`) | 79 | 11 | 24 | 0 | 0 | 1 |

Sections above 1e-10 against PR 5 (28 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `5_abd_contact_ee_parallel.al-ipc` | tie | 2.5e-07 | 4.1e-01 | 3.7e-11 |
| `5_abd_contact_ee_parallel.ipc` | tie | 5.2e-08 | 3.2e-02 | 3.0e-09 |
| `9_abd_multi_constitution.ipc` | tie | 9.6e-02 | 9.1e-02 | 1.2e-11 |
| `12_abd_self_ground_contact.ipc` | tie | 1.8e-02 | 2.9e-02 | 1.1e-12 |
| `18_abd_fem_contact.al-ipc` | tie | 1.1e-02 | 5.4e-02 | 5.1e-05 |
| `29_abd_fem_recover.default` | tie | 3.5e-03 | 3.9e-03 | 4.8e-08 |
| `43_abd_fem_state_access.default` | tie | 1.0e-02 | 4.3e-02 | 1.4e-13 |
| `80_abd_revolute_joint_driving_and_external_torque.default` | tie | 6.6e-08 | 3.7e-05 | 1.1e-14 |
| `6_abd_cube_pile.al-ipc` | chaotic | 1.8e-10 | 4.6e-01 | 2.4e-10 |
| `8_abd_multi_contact_model.al-ipc` | chaotic | 2.9e-10 | 4.0e-01 | 4.7e-10 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 1.9e-07 | 3.4e-01 | 1.3e-05 |
| `11_abd_ramp_sliding.al-ipc` | chaotic | 4.1e-06 | 1.3e-02 | 1.0e-03 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 6.1e-02 | 2.8e-01 | 9.9e-02 |
| `18_abd_fem_contact.ipc` | chaotic | 1.7e-02 | 3.0e-02 | 4.1e-03 |
| `23_kirchhoff_rod_bending.default` | chaotic | 2.0e-01 | 4.1e-01 | 9.3e-02 |
| `28_fem_periodically_pressed_tet.default` | chaotic | 9.4e-10 | 1.1e-09 | 1.1e-09 |
| `30_fem_animiator_substep.default` | chaotic | 1.1e-09 | 1.1e-09 | 7.0e-10 |
| `36_no_surf_but_contact_on.default` | chaotic | 2.6e-07 | 4.0e-04 | 3.5e-04 |
| `47_external_articulation_multijoints.default` | chaotic | 2.4e-05 | 2.9e-05 | 2.6e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 1.7e-03 | 1.6e-02 | 7.8e-02 |
| `56_fem_mas_bunny_ground.default` | chaotic | 1.9e-08 | 2.4e-07 | 4.0e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 5.8e-09 | 1.8e-07 | 3.4e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 1.7e-06 | 1.0e-06 | 7.4e-07 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 6.3e-10 | 1.1e-09 | 1.7e-09 |
| `60_fem_mas_cloth.default` | chaotic | 3.3e-09 | 2.2e-09 | 2.5e-09 |
| `61_fem_mas_rod.default` | chaotic | 5.6e-05 | 4.7e-04 | 3.2e-04 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 8.2e-06 | 1.6e-05 | 1.5e-05 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 9.8e-06 | 7.1e-06 | 1.3e-05 |

- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 4.33 | 3.72 | -14.0% (1.16x) | 5 runs each; run means 4.32–4.35 before, 3.71–3.75 after; Mann-Whitney p = 0.008; 2.50x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 15.27 | 14.10 | -7.7% (1.08x) | 5 runs each; run means 14.83–15.36 before, 13.88–14.72 after; Mann-Whitney p = 0.008; 10.30x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 4.000 | 3.983 | -0.4% | median of 5 runs; range 3.908–4.058 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.25 | 27.31 | +0.2% | median of 5 runs; range 26.67–27.49 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 80.06 | 74.73 | -6.7% (1.07x) | 5 runs each; run means 79.68–80.65 before, 74.16–74.89 after; Mann-Whitney p = 0.008; 3.03x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.672 | 6.680 | +0.1% | median of 5 runs; range 6.656–6.692 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 39.10 | 38.92 | -0.5% | median of 5 runs; range 38.34–39.84 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 27.03 | 25.92 | -4.1% (1.04x) | 5 runs each; run means 26.98–27.05 before, 25.83–26.14 after; Mann-Whitney p = 0.008; 2.62x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.17 | 78.16 | -0.0% | median of 5 runs; range 78.11–78.20 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 36.19 | 34.45 | -4.8% (1.05x) | 5 runs each; run means 35.41–36.51 before, 33.11–35.62 after; Mann-Whitney p = 0.016; 3.67x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.030 | 5.100 | +1.4% | median of 5 runs; range 5.010–5.200 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 38.57 | 38.85 | +0.7% | median of 5 runs; range 37.99–39.40 after |

## Interpretation

Measured: Faster than PR 5 (Mann-Whitney p < 0.05 on the five run means): `anisotropic-cloth-small` 1.16x, `rigid-wrecking-balls` 1.08x, `stiff-gipc-case2` 1.07x, `mas-bunny` 1.04x, `cube-wall-cloth` 1.05x. Iteration counts: in 10 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs.

Inference: PR 6 changes only streams, launch order and host synchronization points, and skips the energy evaluation of a converged trial that ends the Newton loop (main accepts that trial without a decrease test), so its gain is the host and launch latency it removes and that one evaluation per Newton loop; it is a larger fraction of short frames. This record has no stage timing, so it does not split the gain among the changes of this PR. The `uipc.view()` change does not enter these scenes' timed frames.

## Decision

Proposed for merge as PR 6/6, after PR 5/6. `line_search/cubic_refinement` is added and stays off by default: it changes the accepted step and therefore the trajectory, and it was not measured here.

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
