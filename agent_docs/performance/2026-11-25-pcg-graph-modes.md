# 2026-11-25 Fused PCG graph modes

The same Release build of `2344c1c9` ran both modes on Windows 11 with an AMD
Ryzen 9 9950X3D, an RTX 5090, driver 610.88, CUDA 13.2.51, native GPU
architecture, and Python 3.14. The tracked worktree was clean, and the sample
submodule was pinned to `77f43414`.

Each mode ran the five workloads in `benchmarks/manifest.json` at their normal
frame counts. Each of four rounds used fresh processes, rotated the scene
order, and reversed the mode order for each scene relative to the previous
round. The scene configuration specified `linear_system/use_cuda_graph`
explicitly for each run. Timer scopes were disabled. The sample timer measures
`world.advance()` plus `world.retrieve()` for every frame after world
initialization. Each timed run included its first frame. Each table entry is
the median of the four runs' mean frame times in milliseconds.

| Workload | Frames per run | Mode 1 | Mode 2 | Rounds with faster Mode 2 |
|---|---:|---:|---:|---:|
| Anisotropic cloth small | 30 | 6.43 | 6.39 | 2/4 |
| Rigid wrecking balls | 120 | 14.99 | 14.94 | 2/4 |
| Stiff-GIPC case2 | 250 | 62.14 | 60.22 | 4/4 |
| MAS Bunny | 100 | 27.27 | 24.00 | 4/4 |
| Cube wall cloth | 100 | 33.20 | 31.50 | 4/4 |

The geometric mean of the five median speedup ratios was 1.045×. The mode 2
gain was repeatable for case2, MAS Bunny, and cube wall cloth. The small cloth
and rigid wrecking ball results overlapped across repeats. The 4,800 timed
frames completed and converged, and the measured runs emitted no graph
fallback warnings. Separate short case2 runs confirmed each selected mode in
the solver log. Case2's median Newton iteration counts were 1667 and 1667.5
over 250 frames for modes 1 and 2, respectively. MAS Bunny took 486 Newton
iterations per run in both modes. These are end-to-end timings on the recorded
system.

Select mode 2 as the default for standard IPC. Runtimes without conditional
CUDA Graph nodes select mode 1.

To reproduce, set `config["linear_system"]["use_cuda_graph"]` to the requested
mode immediately before constructing the scene. Run each manifest entry with
its default frame count and environment. Use the `BENCHMARK_RESULT` frame
times and convergence flags. Local trial logs and results are under
`output/perf-series-recheck/runs/graph-ab-2344c1c9-r*-m*/`.
