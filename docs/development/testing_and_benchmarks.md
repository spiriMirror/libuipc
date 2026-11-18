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
is the repository-owned registry for portable performance workloads. Implementations
and assets remain in the tracked `libuipc-samples` submodule, avoiding a second
copy in the core repository.

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

Choose a workload by problem size and subsystem. No GPU model is prescribed;
simulation requires a CUDA device supported by the installed build.

| Tier | Workload | Scope | Default frames |
|---|---|---|---:|
| Small | `anisotropic-cloth-small` (95) | Three procedural cloth sheets, 243 vertices / 384 triangles, no contact | 30 |
| Standard | `rigid-wrecking-balls` (6) | Affine-body contact | 120 |
| Standard | `mas-bunny` (89) | One stiff FEM bunny with MAS and ground contact | 100 |
| Stress | `stiff-gipc-case2` (88) | Two FEM bunnies and high-resolution cloth | 250 |
| Stress | `cube-wall-cloth` (93) | 1920 affine bodies and high-resolution cloth | 100 |

Tiers describe workloads, not hardware eligibility or speed targets. Start with
the small workload; run larger scenes when relevant and within local resources.
`--quick` reduces only the frame count to three: it is an execution smoke test,
not a smaller mesh or a representative performance measurement. Init and GPU
runtime overhead can dominate small scenes; report that limitation.

Initialize the pinned samples with `git submodule update --init libuipc-samples`.
`validate` checks the actual required files and Python syntax without importing
uipc or requiring a GPU. Repository Contracts CI runs it against the pinned
samples revision. `--dry-run` inspects an invocation without executing it.

The canonical runner validates required assets, removes noncanonical tuning
environment variables, launches the headless sample from its declared working
directory, preserves child failures, and writes repo/submodule commits,
command, frame count, environment overrides, duration, and status to
`output/benchmark-runs/`.
The metadata also captures GPU/driver, CUDA compiler, Python/libuipc versions,
the full per-frame wall distribution, Newton/line-search/linear-solver counts,
a final physical observable, approximate peak total-device memory, and the raw
log.
Timestamped records are retained; the un-suffixed JSON points to the latest
run. A zero child exit without a `BENCHMARK_RESULT` containing the complete
frame sequence and finite non-negative frame times is a runner failure; both
the child status and validation error are archived. Completion
does not imply convergence: inspect convergence and iteration-limit flags too.

Each run fingerprints the selected interpreter's package before and after
execution: `build_info()`, package path, Python source hashes, and SHA-256 hashes
of colocated native binaries. Declared scene/helper/asset files are also hashed
before and after the run. All this work happens outside the frame timing interval
and does not launch CUDA work. `provenance.runtimeUnchanged` and
`provenance.inputsUnchanged` must both be true for a controlled comparison.
An unavailable runtime probe is not counted as an unchanged runtime. These are
package-file fingerprints, not a claim that every bundled DLL was loaded, nor a
proof that the installed package was built from the recorded worktree commit.
Check the backend hash against the intended build output as well.

To avoid changing local sample parameters, create an isolated samples worktree
under the root repository and select it explicitly:

```shell
git -C libuipc-samples worktree add --detach ../output/perf-samples <samples-commit>
python scripts/run_benchmark.py run stiff-gipc-case2 \
  --samples-directory output/perf-samples --frames 250
```

The selected worktree's revision and input hashes are recorded; the original
submodule and its local edits are untouched. This option rebases sample inputs
only, not the benchmark's result destination. Keep concurrent GPU workloads off
the device during measurements and alternate repeated baseline/candidate runs.

Canonical throughput runs keep `UIPC_BENCHMARK_TIMERS=0`. For a separate stage
diagnostic use, for example:

```shell
python scripts/run_benchmark.py run stiff-gipc-case2 --frames 60 \
  --env UIPC_BENCHMARK_TIMERS=1
```

Timer scopes synchronize nested GPU regions and therefore change wall time.
Use them to attribute cost, not as a substitute for the Timer-free throughput
result.

## Same-environment comparisons

The project standardizes workloads and reporting, not an absolute ms/frame
target. GPU model, CPU/OS, driver, toolkit and build flags describe a result;
they do not define the community's required computer. Historical measurements
remain in the [performance archive](https://github.com/spiriMirror/libuipc/blob/main/agent_docs/performance/README.md)
with their original environment and revisions. They are not current defaults
or community regression thresholds.

For a library change:

1. Select a workload and freeze its sample revision, asset hashes, arguments,
   frame count and environment. Record both library revisions and verify each
   installed package against its intended build. The library revisions and
   binary hashes may differ between the before/after runs, but must not change
   during either run.
2. Use the same machine/device, driver, toolkit, Python, Release build flags,
   power settings and background-load conditions. Keep timer mode unchanged.
   Changing `--env NO_MAS=1`, scene parameters or any of these conditions creates
   a separate experiment; do not silently pool it with the original workload.
3. Alternate at least three fresh-process runs per version. The registry times
   every requested `advance()` + `retrieve()` frame (including any frame-validity
   check) from frame 1, with no discarded
   warmup frames; initialization, reporting and file output are outside that
   interval. Use the same full frame window on both versions because contact
   workloads evolve. Retain the timestamped JSON/log files for every run.
4. Compare the distribution of run means and per-frame median/p95, reporting
   relative change `(candidate / before - 1) * 100%` and repeat variation.
   Check completion/convergence, Newton/line-search/linear counts and final
   physical observables before attributing a change to throughput. Final
   positions in contact scenes are not bitwise goldens. Memory samples are total
   device usage and include other processes; they are not exact scene allocations.

Portable CI checks the runner/submodule contract, not wall time. A dedicated
GPU runner may maintain its own baseline and an explicitly chosen allowance
based on repeated local measurements. Cross-machine results are useful reports,
but do not constitute a regression gate for each other.

For the separate `python -m uipc benchmark` scene runner, reuse the existing
[baseline/check workflow](../tutorial/profiling.md#performance-regression-gates)
with environment matching enabled. It consumes that runner's result directories;
the registry JSON files described here use a different format and should be
compared using the protocol above, not passed directly to `baseline/check`.
