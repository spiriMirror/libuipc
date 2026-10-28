# Project Benchmarks

`manifest.json` is the versioned registry for end-to-end performance scenes.
The scene implementations and large assets remain in the `libuipc-samples`
submodule; this directory owns their stable names, canonical arguments,
environment, required assets, and reproducibility metadata.

List and run benchmarks from the repository root:

```shell
python scripts/run_benchmark.py list
python scripts/run_benchmark.py validate
python scripts/run_benchmark.py run anisotropic-cloth-small
python scripts/run_benchmark.py run rigid-wrecking-balls --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 250
python scripts/run_benchmark.py run mas-bunny --frames 100
python scripts/run_benchmark.py run cube-wall-cloth --frames 100
```

The suite offers small (procedural cloth, sample 95), standard (affine-body
contact and MAS FEM, samples 6/89) and stress (FEM/cloth contact and a large
affine-body wall on cloth, samples 88/93) workloads. Tiers describe problem
size, with no prescribed GPU model or universal speed threshold. `--quick`
shortens only the frame count, not mesh size or memory demand. `validate` checks
required inputs and syntax at the pinned samples revision without a GPU.
The sample implementations emit full-precision
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
concurrent desktop activity; it is not a measurement of process-private allocation.
Tracked modifications and untracked paths are recorded separately, so local
reference checkouts do not falsely mark an otherwise reproducible source tree
dirty.
The timestamped record is retained and the un-suffixed JSON is refreshed as a
convenient latest-run pointer.

The normal regression suite remains correctness-focused. Benchmark results are
environment-specific evidence. Follow the
[same-environment comparison protocol](../docs/development/testing_and_benchmarks.md#same-environment-comparisons)
and retain repeated before/after runs. The separate `uipc.profile` baseline/check
tools consume their own scene-runner format, not this registry's JSON files.
Older device measurements remain in the
[historical evidence archive](../agent_docs/performance/README.md);
they do not define community requirements or thresholds for another machine.
Native microbenchmarks under `apps/benchmarks/` are a separate, lower-level
facility.
