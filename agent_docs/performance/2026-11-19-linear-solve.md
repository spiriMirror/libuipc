# 2026-11-19 — Linear solve: BCOO conversion, MAS, fused PCG iteration (PR 5/6)

- Status: Proposed
- Before commit: `a4fbaf5f62d8bd2c05aacd34ec560435585d7d3b` (`perf/4-fem`, PR 4/6)
- After commit: `f95baff8a542e5062d2494dd832c4989138b9249` (`perf/5-linear`, PR 5/6) (built from `21568eb8`, which has the same code but for the register cap of PR 2's warp-cooperative query, which the branch drops; PR 2's evidence record has an A/B of that change)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Does PR 5 (upper BCOO conversion without compaction, a leaner SpMV, faster MAS assembly and inversion, fused and parallel preconditioner steps in the PCG iteration) reduce frame time with the same PCG iteration counts? Results may change only in the order of atomic sums and in the rounding of the MAS cluster inverse. The scope expected to move is the linear solve, above all in the MAS scenes (`stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth`). The opt-in key `linear_system/keep_preconditioner` is off in every run of this record.

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

The benchmark scenes use the default fused-PCG mode (`linear_system/use_cuda_graph = 1`, block replay); the mode-2 changes of this PR are covered by the unit tests only.

## Correctness and safety

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 135 test cases (main: 28; 49 added by this PR), all passing (`lbvh` fails on main; PR 2 fixes its CPU oracle). `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 254 passed, 0 failed (main: 254 passed, the same tests).
- Focused tests of this PR: `matrix_converter_convert_upper`, `matrix_converter_compact_radix_key` (also -1 padding entries), `fast_segmental_reduce_lean` (also both forms of the warp maximum), `fused_pcg_spmv_pipeline`, `mas_cluster_inverse` (against Gauss-Jordan), `mas_preconditioner_direct_store` (also without a partitioned vertex), `abd_diag_inverse`, `fused_pcg_*_update_apply_dot`, `fused_pcg_direction_publication`, `fused_pcg_duplicate_preconditioner`, `fused_pcg_while_publication`, `preconditioner_reuse`, `cuda_tool_graph`; all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison.

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| PR 4 | 79 | 8 | 27 | 0 | 0 | 1 |
| main (`158d39fd`) | 79 | 9 | 26 | 0 | 0 | 1 |

Sections above 1e-10 against PR 4 (35 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `5_abd_contact_ee_parallel.al-ipc` | tie | 2.7e-07 | 4.1e-01 | 2.5e-11 |
| `5_abd_contact_ee_parallel.ipc` | tie | 1.2e-08 | 3.2e-02 | 8.6e-10 |
| `9_abd_multi_constitution.ipc` | tie | 7.2e-02 | 9.1e-02 | 2.2e-11 |
| `12_abd_self_ground_contact.ipc` | tie | 1.9e-02 | 3.5e-02 | 9.3e-13 |
| `35_abd_surf_cube_pile.default` | tie | 2.5e-02 | 8.6e-02 | 2.1e-13 |
| `41_abd_fem_mesh_d_hat.default` | tie | 1.3e-02 | 6.6e-03 | 1.4e-11 |
| `43_abd_fem_state_access.default` | tie | 4.5e-02 | 4.4e-02 | 9.8e-14 |
| `80_abd_revolute_joint_driving_and_external_torque.default` | tie | 6.6e-08 | 3.7e-05 | 7.7e-14 |
| `6_abd_cube_pile.al-ipc` | chaotic | 1.4e-10 | 4.6e-01 | 2.8e-09 |
| `8_abd_multi_contact_model.al-ipc` | chaotic | 4.1e-10 | 4.0e-01 | 8.2e-10 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 6.6e-07 | 3.4e-01 | 1.4e-02 |
| `11_abd_ramp_sliding.al-ipc` | chaotic | 2.0e-05 | 1.3e-02 | 7.8e-06 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 8.8e-02 | 2.7e-01 | 1.1e-01 |
| `13_fem_3d_gravity.ipc` | chaotic | 2.0e-08 | 1.3e-08 | 2.8e-08 |
| `16_fem_3d_scaling.ipc` | chaotic | 1.9e-05 | 3.1e-03 | 4.6e-06 |
| `17_fem_multi_constituion.ipc` | chaotic | 1.5e-09 | 1.6e-06 | 9.2e-08 |
| `18_abd_fem_contact.al-ipc` | chaotic | 1.2e-02 | 5.5e-02 | 2.0e-02 |
| `18_abd_fem_contact.ipc` | chaotic | 2.0e-02 | 2.0e-02 | 5.3e-03 |
| `23_kirchhoff_rod_bending.default` | chaotic | 2.9e-01 | 3.9e-01 | 4.3e-01 |
| `28_fem_periodically_pressed_tet.default` | chaotic | 1.3e-09 | 1.4e-09 | 2.0e-09 |
| `29_abd_fem_recover.default` | chaotic | 1.8e-07 | 3.9e-03 | 6.6e-08 |
| `30_fem_animiator_substep.default` | chaotic | 1.1e-09 | 6.6e-10 | 7.3e-10 |
| `32_different_gravity.default` | chaotic | 2.2e-10 | 1.6e-10 | 1.1e-10 |
| `36_no_surf_but_contact_on.default` | chaotic | 3.5e-04 | 1.9e-04 | 3.7e-04 |
| `47_external_articulation_multijoints.default` | chaotic | 1.4e-05 | 2.0e-05 | 3.2e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 3.2e-03 | 6.8e-02 | 8.0e-02 |
| `56_fem_mas_bunny_ground.default` | chaotic | 6.1e-08 | 2.4e-07 | 1.3e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 3.9e-09 | 1.8e-07 | 5.8e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 2.1e-06 | 4.2e-06 | 5.7e-06 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 1.2e-09 | 1.4e-09 | 1.4e-09 |
| `60_fem_mas_cloth.default` | chaotic | 1.3e-09 | 3.8e-09 | 2.7e-09 |
| `61_fem_mas_rod.default` | chaotic | 5.5e-05 | 4.3e-04 | 4.7e-04 |
| `82_discrete_shell_bending_strain_plastic_open_strip.default` | chaotic | 1.3e-10 | 1.3e-10 | 1.3e-10 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 1.5e-05 | 1.5e-05 | 1.1e-05 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 8.7e-06 | 1.4e-05 | 8.7e-06 |

- `41_abd_fem_mesh_d_hat.default`: the harness's expectations list it as stable (within 1e-10 in their reference runs); here D = 1.3e-02, S = 6.6e-03, P = 1.4e-11: tie.
- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 5.61 | 4.33 | -22.9% (1.30x) | 5 runs each; run means 5.59–5.62 before, 4.32–4.35 after; Mann-Whitney p = 0.008; 2.15x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 22.64 | 15.27 | -32.6% (1.48x) | 5 runs each; run means 22.31–23.84 before, 14.83–15.36 after; Mann-Whitney p = 0.008; 9.51x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 3.958 | 4.000 | +1.1% | median of 5 runs; range 3.942–4.050 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.13 | 27.25 | +0.5% | median of 5 runs; range 26.69–27.36 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 130.52 | 80.06 | -38.7% (1.63x) | 5 runs each; run means 130.32–132.75 before, 79.68–80.65 after; Mann-Whitney p = 0.008; 2.82x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.664 | 6.672 | +0.1% | median of 5 runs; range 6.640–6.688 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 38.57 | 39.10 | +1.4% | median of 5 runs; range 38.81–39.37 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 55.25 | 27.03 | -51.1% (2.04x) | 5 runs each; run means 55.21–55.29 before, 26.98–27.05 after; Mann-Whitney p = 0.008; 2.51x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.15 | 78.17 | +0.0% | median of 5 runs; range 78.13–78.18 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 63.91 | 36.19 | -43.4% (1.77x) | 5 runs each; run means 63.45–66.72 before, 35.41–36.51 after; Mann-Whitney p = 0.008; 3.50x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.040 | 5.030 | -0.2% | median of 5 runs; range 5.010–5.130 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 39.00 | 38.57 | -1.1% | median of 5 runs; range 37.92–38.94 after |

## Interpretation

Measured: Faster than PR 4 (Mann-Whitney p < 0.05 on the five run means): `anisotropic-cloth-small` 1.30x, `rigid-wrecking-balls` 1.48x, `stiff-gipc-case2` 1.63x, `mas-bunny` 2.04x, `cube-wall-cloth` 1.77x. Iteration counts: in 8 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs; outside it: `rigid-wrecking-balls` Newton/frame 4.000, +1.69% from main's median (main's runs 3.908–3.992; Mann-Whitney p = 0.087); `cube-wall-cloth` Newton/frame 5.030, -1.76% from main's median (main's runs 5.100–5.200; Mann-Whitney p = 0.048).

Inference: where the PCG iterations per Newton iteration of the MAS scenes stay within run-to-run variation (see the table), the gain is in the cost per iteration and in the preconditioner assembly, not in fewer iterations. The MAS synchronization fix is a correctness change; this record does not show whether the race occurred in these runs. This record has no stage timing, so it does not split the gain among the changes of this PR.

## Decision

Proposed for merge as PR 5/6, after PR 4/6. `linear_system/keep_preconditioner` is added and stays off by default: it is not accuracy-neutral (it changes PCG's stopping threshold) and was not measured here.

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
