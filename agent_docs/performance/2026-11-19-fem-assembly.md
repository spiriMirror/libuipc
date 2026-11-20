# 2026-11-19 — FEM assembly beside the contact work (PR 4/6)

- Status: Proposed
- Before commit: `12766c14f33ab9b2822406c608bafd2d01da36dc` (`perf/3-contact-abd`, PR 3/6)
- After commit: `a4fbaf5f62d8bd2c05aacd34ec560435585d7d3b` (`perf/4-fem`, PR 4/6) (built from `10bc72bc`, which has the same code but for the register cap of PR 2's warp-cooperative query, which the branch drops; PR 2's evidence record has an A/B of that change)
- Benchmark manifest name and samples commit: `anisotropic-cloth-small`, `rigid-wrecking-balls`, `stiff-gipc-case2`, `mas-bunny`, `cube-wall-cloth` (`benchmarks/manifest.json`, unchanged by the series); libuipc-samples `77f43414b649ad6c94387f367f2d0edabffa8a9f`

## Question

Does PR 4 (shell and hinge Hessians projected in shared memory; FEM producers launched beside the contact assembly and writing the global system in place; the assembly head reordered) reduce frame time with bitwise results, apart from the rounding of the shell PSD projections and the atomic order of the in-place FEM gradient? The scope expected to move is the FEM and shell assembly: cloth and FEM scenes, and the assembly head of every scene.

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

- Unit tests at this PR's tip: Catch2 (the seven `uipc_test_*` executables): `geometry`, `core`, `common`, `sanity_check`, `regression` have the same test cases and results as on main. `backend_cuda` runs 86 test cases (main: 28; 9 added by this PR), all passing (`lbvh` fails on main; PR 2 fixes its CPU oracle). `sim_case` fails 2 of 14213 assertions, both `stress_plastic_angle > 5.0e-2` in `86_discrete_shell_bending_stress_plastic_residual`, exactly as on main. `python/tests` (`pytest -m "not example"`): 254 passed, 0 failed (main: 254 passed, the same tests).
- Focused tests of this PR: `discrete_shell_bending_derivatives` and `stress_plastic_discrete_shell_bending` (`d2Edx2` bitwise against `dEdx` and `ddEddx`), `material_gradient_only_launch` (gradient-only variants against the full kernels and a host oracle), `neo_hookean_shell_energy_launch` (the energy launcher against per-element `NH::E`), `in_place_prelaunch` (in-place and ahead prelaunch byte for byte against `assemble()`), `cuda_tool_side_stream`, `cuda_tool_side_streams`; all pass.
- Sim cases: libuipc's 95 sim cases (`apps/tests/sim_case`, 115 sections) ran through the Python bindings in a harness that records every vertex position of every frame (a Python port of the Catch2 cases; not in this repository), on the second card; main's run is an earlier run of the same main build on that card. 114 sections pass and 1 fails (`86_discrete_shell_bending_stress_plastic_residual.default`), the same section as on main. Each section's trajectory is compared with the previous step's run and with main's run. D is the largest vertex distance over the trajectory relative to the scene's bounding-box diagonal; D ≤ 1e-10 is *stable*. Every other section is rerun on both builds from three starts with each initial vertex coordinate moved by 1e-12 × N(0, 1). S is the largest deviation of the reference build's perturbed reruns from its own run, P the largest deviation between the two builds from the same perturbed start. *tie*: P < D/10; *chaotic*: P ≥ D/10 and S ≥ D/10; *systematic*: P ≥ D/10 and S < D/10; *unknown*: a rerun produced no frames. Result: no section is systematic or unknown in either comparison.

| compared with | stable | tie | chaotic | systematic | unknown | no trajectory |
|---|---:|---:|---:|---:|---:|---:|
| PR 3 | 86 | 7 | 21 | 0 | 0 | 1 |
| main (`158d39fd`) | 82 | 9 | 23 | 0 | 0 | 1 |

Sections above 1e-10 against PR 3 (28 of 115):

| section | class | D | S | P |
|---|---|---:|---:|---:|
| `5_abd_contact_ee_parallel.al-ipc` | tie | 9.6e-07 | 4.1e-01 | 1.0e-11 |
| `5_abd_contact_ee_parallel.ipc` | tie | 5.5e-07 | 3.2e-02 | 2.0e-09 |
| `8_abd_multi_contact_model.ipc` | tie | 5.0e-10 | 7.1e-02 | 2.5e-13 |
| `9_abd_multi_constitution.ipc` | tie | 9.5e-02 | 9.7e-02 | 3.1e-11 |
| `12_abd_self_ground_contact.ipc` | tie | 2.3e-02 | 3.3e-02 | 3.6e-13 |
| `35_abd_surf_cube_pile.default` | tie | 2.5e-02 | 6.6e-02 | 2.1e-13 |
| `43_abd_fem_state_access.default` | tie | 4.6e-02 | 4.3e-02 | 6.2e-14 |
| `9_abd_multi_constitution.al-ipc` | chaotic | 2.0e-06 | 3.4e-01 | 3.8e-02 |
| `11_abd_ramp_sliding.al-ipc` | chaotic | 5.1e-04 | 1.3e-02 | 7.6e-04 |
| `12_abd_self_ground_contact.al-ipc` | chaotic | 6.0e-02 | 2.7e-01 | 8.3e-02 |
| `17_fem_multi_constituion.ipc` | chaotic | 1.6e-09 | 1.6e-06 | 1.5e-09 |
| `18_abd_fem_contact.al-ipc` | chaotic | 1.2e-02 | 6.4e-02 | 6.6e-02 |
| `18_abd_fem_contact.ipc` | chaotic | 1.8e-03 | 2.0e-02 | 2.6e-04 |
| `23_kirchhoff_rod_bending.default` | chaotic | 1.3e-01 | 4.3e-01 | 4.5e-01 |
| `28_fem_periodically_pressed_tet.default` | chaotic | 4.3e-10 | 1.1e-09 | 2.1e-09 |
| `29_abd_fem_recover.default` | chaotic | 7.2e-10 | 3.9e-03 | 1.8e-08 |
| `30_fem_animiator_substep.default` | chaotic | 2.9e-10 | 7.9e-10 | 5.1e-10 |
| `36_no_surf_but_contact_on.default` | chaotic | 3.7e-06 | 5.4e-05 | 1.2e-06 |
| `47_external_articulation_multijoints.default` | chaotic | 3.3e-05 | 3.8e-05 | 2.6e-05 |
| `55_fem_mas_stiff_stack.default` | chaotic | 2.6e-04 | 6.8e-02 | 2.9e-03 |
| `56_fem_mas_bunny_ground.default` | chaotic | 1.0e-07 | 2.4e-07 | 2.2e-07 |
| `57_fem_mas_two_cubes.default` | chaotic | 2.0e-09 | 1.8e-07 | 5.3e-09 |
| `59_fem_mas_vs_diag_benchmark.diag` | chaotic | 4.3e-07 | 2.5e-06 | 9.3e-07 |
| `59_fem_mas_vs_diag_benchmark.mas` | chaotic | 1.4e-09 | 1.3e-09 | 1.5e-09 |
| `60_fem_mas_cloth.default` | chaotic | 2.2e-09 | 3.8e-09 | 3.7e-09 |
| `61_fem_mas_rod.default` | chaotic | 7.1e-05 | 4.3e-04 | 5.9e-04 |
| `85_stress_plastic_discrete_shell_bending_open_strip.default` | chaotic | 1.4e-05 | 1.3e-05 | 1.3e-05 |
| `87_stress_plastic_discrete_shell_bending_recover.recover_continues_from_dumped_state` | chaotic | 1.3e-05 | 1.3e-05 | 9.8e-06 |

- `8_abd_multi_contact_model.ipc`: the harness's expectations list it as stable (within 1e-10 in their reference runs); here D = 5.0e-10, S = 7.1e-02, P = 2.5e-13: tie.
- Compute Sanitizer was not run for this record.
- clang-format 18.1.8 (`--dry-run --Werror`, as CI) on the changed C++ files of every commit: clean.

## Results

| Metric | Before | After | Change | Samples / notes |
|---|---:|---:|---:|---|
| Focused scope | not measured | not measured | | no kernel or stage timings were taken for this record; the unit tests listed above check the changed kernels' results |
| Enclosing subsystem | not measured | not measured | | as above (`UIPC_BENCHMARK_TIMERS=0` in every run) |
| End-to-end median, `anisotropic-cloth-small` (ms/frame, 30 frames) | 9.27 | 5.61 | -39.4% (1.65x) | 5 runs each; run means 9.25–9.28 before, 5.59–5.62 after; Mann-Whitney p = 0.008; 1.66x against main |
| Newton iterations per frame, `anisotropic-cloth-small` | 4.600 | 4.600 | +0.0% | median of 5 runs; range 4.600–4.600 after |
| PCG iterations per Newton iteration, `anisotropic-cloth-small` | 15.36 | 15.36 | +0.0% | median of 5 runs; range 15.36–15.36 after |
| End-to-end median, `rigid-wrecking-balls` (ms/frame, 120 frames) | 27.62 | 22.64 | -18.0% (1.22x) | 5 runs each; run means 26.06–28.01 before, 22.31–23.84 after; Mann-Whitney p = 0.008; 6.41x against main |
| Newton iterations per frame, `rigid-wrecking-balls` | 3.975 | 3.958 | -0.4% | median of 5 runs; range 3.908–4.008 after |
| PCG iterations per Newton iteration, `rigid-wrecking-balls` | 27.34 | 27.13 | -0.8% | median of 5 runs; range 26.71–27.68 after |
| End-to-end median, `stiff-gipc-case2` (ms/frame, 250 frames) | 141.07 | 130.52 | -7.5% (1.08x) | 5 runs each; run means 139.46–141.36 before, 130.32–132.75 after; Mann-Whitney p = 0.008; 1.73x against main |
| Newton iterations per frame, `stiff-gipc-case2` | 6.668 | 6.664 | -0.1% | median of 5 runs; range 6.640–6.692 after |
| PCG iterations per Newton iteration, `stiff-gipc-case2` | 39.06 | 38.57 | -1.2% | median of 5 runs; range 38.41–39.68 after |
| End-to-end median, `mas-bunny` (ms/frame, 100 frames) | 56.33 | 55.25 | -1.9% (1.02x) | 5 runs each; run means 56.31–56.66 before, 55.21–55.29 after; Mann-Whitney p = 0.008; 1.23x against main |
| Newton iterations per frame, `mas-bunny` | 4.860 | 4.860 | +0.0% | median of 5 runs; range 4.860–4.860 after |
| PCG iterations per Newton iteration, `mas-bunny` | 78.13 | 78.15 | +0.0% | median of 5 runs; range 78.12–78.19 after |
| End-to-end median, `cube-wall-cloth` (ms/frame, 100 frames) | 74.54 | 63.91 | -14.3% (1.17x) | 5 runs each; run means 72.37–75.33 before, 63.45–66.72 after; Mann-Whitney p = 0.008; 1.98x against main |
| Newton iterations per frame, `cube-wall-cloth` | 5.060 | 5.040 | -0.4% | median of 5 runs; range 5.020–5.220 after |
| PCG iterations per Newton iteration, `cube-wall-cloth` | 38.88 | 39.00 | +0.3% | median of 5 runs; range 38.47–39.28 after |

## Interpretation

Measured: Faster than PR 3 (Mann-Whitney p < 0.05 on the five run means): `anisotropic-cloth-small` 1.65x, `rigid-wrecking-balls` 1.22x, `stiff-gipc-case2` 1.08x, `mas-bunny` 1.02x, `cube-wall-cloth` 1.17x. Iteration counts: in 7 of 10 cases the median Newton/frame or PCG/Newton of this PR's five runs lies within the range of main's five runs; outside it: `rigid-wrecking-balls` PCG/Newton 27.13, -0.71% from main's median (main's runs 27.20–27.55; Mann-Whitney p = 0.548); `stiff-gipc-case2` PCG/Newton 38.57, -1.13% from main's median (main's runs 38.74–39.46; Mann-Whitney p = 0.222); `cube-wall-cloth` Newton/frame 5.040, -1.56% from main's median (main's runs 5.100–5.200; Mann-Whitney p = 0.397).

Inference: launch shapes such as `StableNeoHookean3D`'s cap of max(SMs / 2, SMs - contact blocks) blocks and its choice among 256 to 480 threads per block are computed from the GPU's SM count and the scene's element count, not fixed for one GPU; this record measures them on one GPU model only. This record has no stage timing, so it does not split the gain between the two changes of this PR.

## Decision

Proposed for merge as PR 4/6, after PR 3/6. No config change.

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
