# Project Benchmarks

`manifest.json` is the versioned registry for end-to-end performance scenes.
The scene implementations and large assets remain in the `libuipc-samples`
submodule; this directory owns their stable names, canonical arguments,
environment, required assets, and reproducibility metadata.

List and run benchmarks from the repository root:

```shell
python scripts/run_benchmark.py list
python scripts/run_benchmark.py run rigid-wrecking-balls --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 250
python scripts/run_benchmark.py run mas-bunny --frames 100
python scripts/run_benchmark.py run cube-wall-cloth --frames 100
```

The canonical suite covers pure affine-body contact (sample 6), two-bunny FEM
plus cloth contact (88), a stiff MAS FEM body (89), and a large affine-body wall
coupled to pinned cloth (93). The sample implementations emit full-precision
per-frame wall timings, backend frame statistics, and a small final-state
observable. Keep `UIPC_BENCHMARK_TIMERS=0` for throughput runs. Set it to `1`
only for a separate synchronized stage-timing diagnostic; Timer instrumentation
changes the measured wall time.

Use `--dry-run` to validate and print the resolved command without starting a
simulation. Intentional A/B variants can override canonical environment values,
for example `--env NO_MAS=1`, but must not be compared as the same benchmark.
Every completed invocation writes commit IDs, command, environment overrides,
GPU/driver/CUDA/Python facts, the scene-reported frame distribution and
Newton/line-search/linear-solver counts, final-state observables, approximate
total-device peak memory, duration, return code, and the raw log under
`output/benchmark-runs/`. On WDDM/display GPUs the memory delta also includes
concurrent desktop activity and must be interpreted as an approximate upper
bound rather than process-private allocation.
The timestamped record is retained and the un-suffixed JSON is refreshed as a
convenient latest-run pointer.

The normal regression suite remains correctness-focused. Benchmark results are
hardware-specific evidence and should be compared with the Python
`uipc.profile` baseline/check tools, not asserted as universal CI wall times.
Native microbenchmarks under `apps/benchmarks/` are a separate, lower-level
facility.
