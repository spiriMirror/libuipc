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
python scripts/run_benchmark.py run stiff-gipc-case2 --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 250
```

The canonical runner validates required assets, removes noncanonical tuning
environment variables, launches the headless sample from its declared working
directory, preserves the child return code, and writes repo/submodule commits,
command, frame count, environment overrides, duration, and status to
`output/benchmark-runs/`.
The metadata also captures GPU/driver, CUDA compiler, Python/libuipc versions,
and the scene's reported mean/median frame time.
Timestamped records are retained; the un-suffixed JSON points to the latest
run.

Use `--dry-run` to inspect a resolved invocation. Overrides such as
`--env NO_MAS=1` are useful for A/B experiments, but they define a different
variant and must not be compared as if they were the canonical configuration.
Use `uipc.profile` baseline/check reports for repeated measurements, and never
turn one machine's wall time into a universal correctness assertion.
