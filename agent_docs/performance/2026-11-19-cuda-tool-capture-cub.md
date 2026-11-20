# 2026-11-19 — cuda_tool graph capture, CUB and launch-helper fixes (PR 1/6)

- Status: Proposed
- Before commit: `158d39fdf69c0e6f00465a85aa1d80d17a61a579` (main)
- After commit: `03652516d70fd39adeefe809fb308626ba7c425c` (`perf/1-cuda-tool`, PR 1/6)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Do the three `cuda_tool` fixes of PR 1 (graph captures whose body throws, CUB wrappers that dispatch once per call, hidden launch helpers) leave every result and the frame time unchanged? The only performance change is on the host: one CUB dispatch per wrapper call instead of two. No scene is expected to move beyond run-to-run variation.

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

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 32 test cases (main: 28; 4 added by this PR); `lbvh` fails, as on main (its CPU oracle reads a destroyed temporary; PR 2 fixes the test), every other case passes. `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 254 passed, 0 failed (main: 254 passed, the same tests).
- Focused tests of this PR: `cuda_tool_graph` (bodies that throw on a valid and on an invalidated capture, failing WHILE-body captures), `cuda_tool_cub` (on-demand workspace growth, results as CUB's), `cuda_tool_buffer` (`Sum` over an emptied, reused range); all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison. For scale: this PR leaves the results unchanged by construction, and yet 25 sections deviate from main by more than 1e-10; all of them are ties (6) or chaotic (19).

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| main (`158d39fd`) | 89 | 6 | 19 | 0 | 0 | 1 |

Sections above 1e-10 against main (25 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `5_abd_contact_ee_parallel.al-ipc` | tie | 1.2e-07 | 4.1e-01 | 3.7e-11 |
| `9_abd_multi_constitution.ipc` | tie | 3.2e-02 | 9.0e-02 | 1.6e-11 |
| `12_abd_self_ground_contact.ipc` | tie | 2.3e-02 | 3.4e-02 | 1.2e-12 |
| `29_abd_fem_recover.default` | tie | 3.8e-03 | 3.9e-03 | 1.4e-08 |
| `43_abd_fem_state_access.default` | tie | 3.6e-02 | 3.9e-02 | 1.8e-14 |
| `80_abd_revolute_joint_driving_and_external_torque.default` | tie | 3.6e-07 | 3.7e-05 | 2.0e-14 |
| `8_abd_multi_contact_model.al-ipc` | chaotic | 6.0e-10 | 4.0e-01 | 4.0e-09 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 1.3e-07 | 3.4e-01 | 2.9e-02 |
| `11_abd_ramp_sliding.al-ipc` | chaotic | 2.3e-06 | 1.3e-02 | 2.7e-05 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 2.9e-02 | 1.8e-01 | 1.1e-01 |
| `18_abd_fem_contact.al-ipc` | chaotic | 1.5e-02 | 4.0e-02 | 5.7e-02 |
| `18_abd_fem_contact.ipc` | chaotic | 3.1e-05 | 2.0e-02 | 2.6e-04 |
| `23_kirchhoff_rod_bending.default` | chaotic | 2.7e-01 | 2.1e-01 | 1.4e-01 |
| `36_no_surf_but_contact_on.default` | chaotic | 1.2e-04 | 8.2e-05 | 1.7e-05 |
| `47_external_articulation_multijoints.default` | chaotic | 2.8e-05 | 8.2e-05 | 4.8e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 1.2e-03 | 6.9e-02 | 2.1e-03 |
| `56_fem_mas_bunny_ground.default` | chaotic | 1.9e-07 | 3.7e-07 | 3.5e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 2.0e-09 | 1.8e-07 | 5.1e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 8.1e-07 | 1.7e-06 | 2.1e-06 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 9.0e-10 | 1.4e-09 | 1.2e-09 |
| `60_fem_mas_cloth.default` | chaotic | 2.3e-09 | 3.5e-09 | 2.2e-09 |
| `61_fem_mas_rod.default` | chaotic | 3.3e-09 | 5.0e-04 | 6.3e-04 |
| `82_discrete_shell_bending_strain_plastic_open_strip.default` | chaotic | 1.0e-10 | 9.5e-11 | 9.7e-11 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 9.7e-06 | 1.2e-05 | 9.7e-06 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 1.1e-05 | 1.4e-05 | 1.6e-05 |

- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 9.30 | 9.28 | -0.2% (1.00x) | 5 runs each; run means 9.26–9.31 before, 9.25–9.29 after; Mann-Whitney p = 0.310; 1.00x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 145.22 | 143.08 | -1.5% (1.01x) | 5 runs each; run means 141.91–147.35 before, 140.56–144.37 after; Mann-Whitney p = 0.151; 1.01x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 3.933 | 3.925 | -0.2% | median of 5 runs; range 3.850–3.967 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.32 | 27.10 | -0.8% | median of 5 runs; range 26.74–27.60 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 226.16 | 226.03 | -0.1% (1.00x) | 5 runs each; run means 225.11–226.99 before, 225.22–235.20 after; Mann-Whitney p = 0.841; 1.00x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.676 | 6.680 | +0.1% | median of 5 runs; range 6.652–6.704 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 39.01 | 38.76 | -0.6% | median of 5 runs; range 38.72–38.86 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 67.90 | 67.95 | +0.1% (1.00x) | 5 runs each; run means 67.77–68.37 before, 67.78–68.06 after; Mann-Whitney p = 1.000; 1.00x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.06 | 78.14 | +0.1% | median of 5 runs; range 78.11–78.24 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 126.54 | 123.92 | -2.1% (1.02x) | 5 runs each; run means 124.93–128.07 before, 122.12–126.17 after; Mann-Whitney p = 0.056; 1.02x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.120 | 5.040 | -1.6% | median of 5 runs; range 5.010–5.140 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 38.52 | 38.67 | +0.4% | median of 5 runs; range 38.00–39.10 after |

## Interpretation

Measured: Not distinguishable from main (p ≥ 0.05): `anisotropic-cloth-small` 1.00x (p = 0.310), `rigid-wrecking-balls` 1.01x (p = 0.151), `stiff-gipc-case2` 1.00x (p = 0.841), `mas-bunny` 1.00x (p = 1.000), `cube-wall-cloth` 1.02x (p = 0.056). Iteration counts: in 8 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs; outside it: `rigid-wrecking-balls` PCG/Newton 27.10, -0.81% from main's median (main's runs 27.20–27.55; Mann-Whitney p = 0.421); `cube-wall-cloth` Newton/frame 5.040, -1.56% from main's median (main's runs 5.100–5.200; Mann-Whitney p = 0.111).

Inference: PR 1 changes no kernel, and its CUB wrappers call the same CUB algorithms on the same inputs, so its results are unchanged by construction (apart from a sum over an empty range, now 0). The sim-case deviations above are therefore differences that rounding alone produces in sections that are ties or chaotic; they give the scale against which the later PRs' comparisons can be read. The primary evidence for this PR is the unit tests of the fixed paths, not the frame time.

## Decision

Proposed for merge as PR 1/6. No config change.

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
