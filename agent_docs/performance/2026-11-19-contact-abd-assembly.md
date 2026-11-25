# 2026-11-19 — Contact and affine-body assembly (PR 3/6)

- Status: Proposed
- Before commit: `3704967ce297009930c565b89ce864f92d335ad3` (`perf/2-collision`, PR 2/6)
- After commit: `12766c14f33ab9b2822406c608bafd2d01da36dc` (`perf/3-contact-abd`, PR 3/6) (built from `9d3b91a3`, which has the same code but for the register cap of PR 2's warp-cooperative query, which the branch drops; PR 2's evidence record has an A/B of that change)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Does PR 3 (fused contact kernels with PSD projection in shared memory, DyTopo distribution of raw entries in one packed pass, ABD producers on side streams ahead of DyTopo, per-warp joint Hessians) reduce frame time while every result stays bitwise or changes only in the rounding of the PSD projections and the joint maps and the summation order of the DyTopo entries? The scope expected to move is contact assembly in every contact scene and the ABD assembly, which dominates `rigid-wrecking-balls`.

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

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 77 test cases (main: 28; 25 added by this PR), all passing (`lbvh` fails on main; PR 2 fixes its CPU oracle). `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 254 passed, 0 failed (main: 254 passed, the same tests).
- Focused tests of this PR: `fixed_size_evd`, `four_vertex_translation_free_spd`, `ipc_simplex_*_contact_assembly*`, `ipc_simplex_*_contact_energy_fusion` (fused energies bitwise), `simplex_friction_hessian_lift`, `abd_jacobi_hessian_blocks`, `abd_hessian_blocks` (gathers bitwise against the former kernels), `warp_joint_hessian` (projections against long double, lane-parallel maps against the generated ones), `abd_launch_shape` (same bits with both launch shapes), `cuda_tool_readback_buffer`, `cuda_tool_side_streams`, `cuda_tool_cub`; all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison.

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| PR 2 | 84 | 10 | 20 | 0 | 0 | 1 |
| main (`158d39fd`) | 82 | 10 | 22 | 0 | 0 | 1 |

Sections above 1e-10 against PR 2 (30 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `3_abd_contact_pe.ipc` | tie | 2.9e-06 | 1.2e-01 | 1.5e-08 |
| `5_abd_contact_ee_parallel.al-ipc` | tie | 4.0e-07 | 4.1e-01 | 1.6e-11 |
| `5_abd_contact_ee_parallel.ipc` | tie | 6.1e-03 | 3.2e-02 | 2.2e-09 |
| `8_abd_multi_contact_model.ipc` | tie | 5.0e-10 | 7.1e-02 | 3.5e-13 |
| `9_abd_multi_constitution.ipc` | tie | 4.2e-02 | 7.5e-02 | 8.9e-12 |
| `11_abd_ramp_sliding.al-ipc` | tie | 5.3e-04 | 1.3e-02 | 3.7e-05 |
| `12_abd_self_ground_contact.ipc` | tie | 3.3e-02 | 4.7e-02 | 1.2e-12 |
| `18_abd_fem_contact.ipc` | tie | 3.4e-03 | 2.0e-02 | 6.0e-07 |
| `40_abd_ignore_self_intersection.default` | tie | 3.5e-07 | 1.0e-06 | 7.7e-16 |
| `80_abd_revolute_joint_driving_and_external_torque.default` | tie | 1.5e-06 | 3.7e-05 | 9.1e-14 |
| `6_abd_cube_pile.al-ipc` | chaotic | 1.2e-09 | 4.6e-01 | 5.4e-10 |
| `8_abd_multi_contact_model.al-ipc` | chaotic | 8.1e-09 | 4.0e-01 | 6.2e-09 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 2.1e-06 | 3.4e-01 | 3.2e-02 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 1.8e-01 | 1.4e-01 | 7.7e-02 |
| `17_fem_multi_constituion.ipc` | chaotic | 3.5e-10 | 1.6e-06 | 9.9e-10 |
| `18_abd_fem_contact.al-ipc` | chaotic | 3.3e-02 | 5.1e-02 | 1.1e-02 |
| `23_kirchhoff_rod_bending.default` | chaotic | 5.6e-02 | 1.4e-01 | 8.5e-02 |
| `29_abd_fem_recover.default` | chaotic | 1.9e-09 | 3.9e-03 | 1.2e-08 |
| `36_no_surf_but_contact_on.default` | chaotic | 3.8e-07 | 3.8e-04 | 3.4e-04 |
| `47_external_articulation_multijoints.default` | chaotic | 2.6e-05 | 4.2e-05 | 4.7e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 3.4e-03 | 6.7e-02 | 2.8e-02 |
| `56_fem_mas_bunny_ground.default` | chaotic | 2.3e-07 | 2.4e-07 | 2.5e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 4.9e-09 | 1.8e-07 | 6.2e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 6.2e-07 | 1.7e-06 | 2.1e-06 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 8.5e-10 | 1.1e-09 | 1.7e-09 |
| `60_fem_mas_cloth.default` | chaotic | 2.6e-09 | 3.0e-09 | 2.9e-09 |
| `61_fem_mas_rod.default` | chaotic | 6.3e-06 | 4.2e-04 | 4.2e-04 |
| `84_strain_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 1.2e-10 | 1.2e-10 | 1.0e-10 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 8.5e-06 | 1.5e-05 | 1.5e-05 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 1.3e-05 | 1.4e-05 | 1.2e-05 |

- `8_abd_multi_contact_model.ipc`: the harness's expectations list it as stable (within 1e-10 in their reference runs); here D = 5.0e-10, S = 7.1e-02, P = 3.5e-13: tie.
- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 9.27 | 9.27 | -0.0% (1.00x) | 5 runs each; run means 9.26–9.30 before, 9.25–9.28 after; Mann-Whitney p = 0.548; 1.00x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 119.58 | 27.62 | -76.9% (4.33x) | 5 runs each; run means 117.10–121.73 before, 26.06–28.01 after; Mann-Whitney p = 0.008; 5.26x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 3.967 | 3.975 | +0.2% | median of 5 runs; range 3.925–4.000 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.27 | 27.34 | +0.3% | median of 5 runs; range 26.82–27.54 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 169.53 | 141.07 | -16.8% (1.20x) | 5 runs each; run means 168.25–170.03 before, 139.46–141.36 after; Mann-Whitney p = 0.008; 1.60x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.680 | 6.668 | -0.2% | median of 5 runs; range 6.628–6.688 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 38.98 | 39.06 | +0.2% | median of 5 runs; range 38.90–39.28 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 57.83 | 56.33 | -2.6% (1.03x) | 5 runs each; run means 57.80–57.91 before, 56.31–56.66 after; Mann-Whitney p = 0.008; 1.21x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.13 | 78.13 | +0.0% | median of 5 runs; range 78.11–78.19 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 98.20 | 74.54 | -24.1% (1.32x) | 5 runs each; run means 97.23–99.09 before, 72.37–75.33 after; Mann-Whitney p = 0.008; 1.70x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.110 | 5.060 | -1.0% | median of 5 runs; range 4.950–5.070 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 38.76 | 38.88 | +0.3% | median of 5 runs; range 38.46–39.08 after |

## Interpretation

Measured: Faster than PR 2 (Mann-Whitney p < 0.05 on the five run means): `rigid-wrecking-balls` 4.33x, `stiff-gipc-case2` 1.20x, `mas-bunny` 1.03x, `cube-wall-cloth` 1.32x. Not distinguishable from PR 2 (p ≥ 0.05): `anisotropic-cloth-small` 1.00x (p = 0.548). Iteration counts: in 9 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs; outside it: `cube-wall-cloth` Newton/frame 5.060, -1.17% from main's median (main's runs 5.100–5.200; Mann-Whitney p = 0.008).

Inference: the PSD projections now come from other eigensolvers (fixed-bank, warp Jacobi) that match the previous ones to rounding (the tests compare them with Eigen and a long double reference), and the DyTopo entries are summed in another order; the sim-case classification shows no systematic difference from either. This record has no stage timing, so it does not split the gain among the four changes of this PR.

## Decision

Proposed for merge as PR 3/6, after PR 2/6. No config change. `NOTICE` gains the MPL-2.0 entry for `utils/fixed_bank_soa_evd.h`.

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
