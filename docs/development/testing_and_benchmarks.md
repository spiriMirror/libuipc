# Testing and Benchmarks

libuipc deliberately keeps correctness regression tests and performance
measurements separate. Test outcomes should be portable; benchmark timings are
meaningful only with recorded software and hardware context.

## Native test layers

CMake registers each Catch2 executable as one aggregate CTest test. The
`uipc.sim_case` aggregate runs all 95 simulation cases in a single process and
must remain part of final validation: it exposes accidental process-global
state that fresh-process tests cannot detect.

```shell
ctest --test-dir build -C Release -R "^uipc.sim_case$" --output-on-failure
```

GPU-labelled CTest tests share the `uipc_gpu` resource lock. Running CTest with
`-j` can still overlap CPU work, but it will not launch multiple libuipc GPU
test executables against the same device concurrently.

For diagnosis or parallel CI, the isolated runner discovers names from the
built Catch2 executable, sorts them, and assigns them round-robin to stable
zero-based shards:

```shell
python scripts/run_sim_case_isolated.py --shard-count 4 --shard-index 0
python scripts/run_sim_case_isolated.py --filter "3*" --timeout 900
python scripts/run_sim_case_isolated.py --list-only \
  --manifest output/sim-case-manifest.json
```

Each selected case runs in a fresh process. A case that passes in isolation but
fails in the aggregate is evidence of cross-case state pollution; the isolated
runner is complementary coverage, not a replacement for the aggregate.

## End-to-end performance registry

[`benchmarks/manifest.json`](https://github.com/spiriMirror/libuipc/blob/main/benchmarks/manifest.json)
is the repository-owned registry for large performance scenes. Implementations
and assets remain in the tracked `libuipc-samples` submodule, avoiding a second
copy in the core repository.

```shell
python scripts/run_benchmark.py list
python scripts/run_benchmark.py run rigid-wrecking-balls --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 250
python scripts/run_benchmark.py run mas-bunny --frames 100
python scripts/run_benchmark.py run cube-wall-cloth --frames 100
```

The four canonical workloads cover pure affine-body contact (sample 6), mixed
FEM/cloth contact (88), stiff FEM with MAS (89), and a large affine-body wall
coupled to cloth (93).

The canonical runner validates required assets, removes noncanonical tuning
environment variables, launches the headless sample from its declared working
directory, preserves the child return code, and writes repo/submodule commits,
command, frame count, environment overrides, duration, and status to
`output/benchmark-runs/`.
The metadata also captures GPU/driver, CUDA compiler, Python/libuipc versions,
the full per-frame wall distribution, Newton/line-search/linear-solver counts,
a final physical observable, approximate peak total-device memory, and the raw
log.
Timestamped records are retained; the un-suffixed JSON points to the latest
run.

Canonical throughput runs keep `UIPC_BENCHMARK_TIMERS=0`. For a separate stage
diagnostic use, for example:

```shell
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 60 \
  --env UIPC_BENCHMARK_TIMERS=1
```

Timer scopes synchronize nested GPU regions and therefore change wall time.
Use them to attribute cost, not as a substitute for the Timer-free throughput
result.

## Current reference measurement

The 2026-09-01 reference used an RTX 5090, driver 595.79, CUDA 13.2, Windows
Release build, and three fresh processes per workload. The value below is the
median of the three run means; ranges retain desktop/WDDM and dynamic-contact
variability.

| Benchmark | Frames | Reference | Three-run range | Newton/frame | Linear iterations/frame |
|---|---:|---:|---:|---:|---:|
| `rigid-wrecking-balls` | 120 | 129.5 ms/frame | 126.4–131.3 | 3.95 | 107.3 |
| `stiff-gipc-case2` | 250 | 201.1 ms/frame | 199.0–230.6 | 6.64 | 246.8 |
| `mas-bunny` | 100 | 60.2 ms/frame | 60.1–62.0 | 4.67 | 358.8 |
| `cube-wall-cloth` | 100 | 125.6 ms/frame | 121.8–165.0 | 5.13 | 200.1 |

All 12 runs completed and converged without an iteration-limit hit. These are
machine-specific references, not portable pass/fail limits. Collision-rich
final positions are envelopes rather than bitwise goldens. The complete method,
memory ranges, numerical variability, run IDs, and stage diagnostics are in the
[cross-domain baseline record](https://github.com/spiriMirror/libuipc/blob/main/agent_docs/performance/2026-09-01-cross-domain-baseline.md).

Use `--dry-run` to inspect a resolved invocation. Overrides such as
`--env NO_MAS=1` are useful for A/B experiments, but they define a different
variant and must not be compared as if they were the canonical configuration.
Use `uipc.profile` baseline/check reports for repeated measurements, and never
turn one machine's wall time into a universal correctness assertion.
