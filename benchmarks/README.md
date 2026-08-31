# Project Benchmarks

`manifest.json` is the versioned registry for end-to-end performance scenes.
The scene implementations and large assets remain in the `libuipc-samples`
submodule; this directory owns their stable names, canonical arguments,
environment, required assets, and reproducibility metadata.

List and run benchmarks from the repository root:

```shell
python scripts/run_benchmark.py list
python scripts/run_benchmark.py run stiff-gipc-case2 --quick
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 250
```

Use `--dry-run` to validate and print the resolved command without starting a
simulation. Intentional A/B variants can override canonical environment values,
for example `--env NO_MAS=1`, but must not be compared as the same benchmark.
Every completed invocation writes commit IDs, command, environment overrides,
GPU/driver/CUDA/Python facts, the scene-reported mean/median frame time,
duration, and return code under `output/benchmark-runs/`.
The timestamped record is retained and the un-suffixed JSON is refreshed as a
convenient latest-run pointer.

The normal regression suite remains correctness-focused. Benchmark results are
hardware-specific evidence and should be compared with the Python
`uipc.profile` baseline/check tools, not asserted as universal CI wall times.
Native microbenchmarks under `apps/benchmarks/` are a separate, lower-level
facility.
