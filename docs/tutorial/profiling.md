# Profiling

## Performance Report

After running a simulation, use `SimulationStats` to generate a full performance report — including per-frame timer charts, a profiler heatmap, and system dependency graphs — in one call:

```python
from uipc import Timer
from uipc.stats import SimulationStats

stats = SimulationStats()
Timer.enable_all()  # synchronized diagnostic scopes; not throughput timing

for i in range(num_frames):
    world.advance()
    world.retrieve()
    stats.collect()

stats.summary_report(output_dir='perf_report', workspace='workspace')
Timer.disable_all()
```

This creates a `perf_report/` folder containing `report.md` and a set of SVG figures. Open `report.md` in any Markdown viewer to inspect the results.

Enabling Timer makes CUDA stages synchronize at scope boundaries. This is
appropriate for attribution and charts, but it perturbs end-to-end throughput.
Collect normal wall time with Timer disabled and use a separate Timer-enabled
run for stage diagnosis.

## Benchmarking (`uipc.profile`)

Use `profile.session` to mix **warmup** (no data collected) and **profiling** (data collected) phases. Calls to `advance` / `profile` are deferred — the simulation runs when the `with` block exits.

```python
from uipc import profile

with profile.session(world, name='phased') as s:
    s.advance(10)     # warmup phase 1
    s.profile(5)      # measure phase 1
    s.advance(20)     # warmup phase 2
    s.profile(5)      # measure phase 2
```

The result is available as `s.result` after the block exits, which includes a `SimulationStats` instance and a human-readable summary:

```python
print(s.result['summary'])
# Scene: phased  |  Frames: 10  |  Wall time: 2.345s  |  Avg: 234.5ms/frame
```

Warmup/recovery time is excluded from `wall_time`, and timers accumulated by a
warmup phase are drained before the first measured frame. The saved
`benchmark.json` records the exact warmup/profile plan plus backend, Python,
platform, UIPC, CUDA-toolkit, and architecture compatibility facts.

The command-line runner exposes the same boundary:

```bash
python -m uipc benchmark run --scene cube_ground --warmup 20 --frames 50 --output bench
```

### Performance regression gates

Create a baseline on a stable runner, then check later results on that same
runner:

```bash
python -m uipc benchmark baseline bench/cube_ground -o baselines/cube_ground.json
python -m uipc benchmark check bench-candidate/cube_ground \
    --baseline baselines/cube_ground.json \
    --json-output bench-candidate/gate.json
```

The check exits nonzero when an enforced metric exceeds its saved allowance
(10% by default), a benchmark/frame plan is missing, or compatibility facts do
not match. It gates profile-only wall time and Timer median/p95 frame time;
Newton iterations are retained as a diagnostic so a faster algorithm is not
rejected solely because its iteration count changed. Use
`--max-regression-percent` to override the saved allowance. Only use
`--allow-environment-mismatch` for exploratory comparisons: timings from
different GPUs, builds, CUDA toolkits, or platforms are not a valid CI gate.

The equivalent Python API is
`uipc.profile.create_baseline(...)` / `uipc.profile.check_baseline(...)`.

For the repository's fixed cross-domain workloads, revision archival, and
current machine-specific reference numbers, see
[Testing and Benchmarks](../development/testing_and_benchmarks.md). That runner
keeps throughput and synchronized diagnostics separate and records structured
solver counters after each frame.

### Structured solver statistics

The CUDA backend exposes the latest frame without requiring log parsing:

```python
world.advance()
world.retrieve()

frame = engine.frame_stats()
print(frame['frame'], frame['newton_iterations'])
if not frame['completed'] or not frame['converged']:
    print(frame)
```

`frame_stats()` returns an empty dictionary for a backend that does not publish
frame statistics (including the `none` backend). The CUDA schema is:

| Key | Meaning |
|---|---|
| `schema_version` | Version of this small JSON contract; currently `1`. |
| `frame`, `pipeline` | Latest frame number and `ipc` / `al-ipc` path. |
| `completed` | The frame pipeline reached its end without throwing. |
| `converged` | The nonlinear solver met that pipeline's termination rule. |
| `newton_iterations` | Number of nonlinear iteration bodies executed. |
| `line_search_trials` | Total candidate steps evaluated across the frame. |
| `linear_solver_iterations` | Total iterative-linear-solver iterations across all Newton solves. |
| `hit_newton_limit`, `hit_line_search_limit` | Whether any configured iteration ceiling was reached. |
| `last_line_search_alpha`, `last_ccd_toi`, `last_cfl_alpha` | Last step-size diagnostics (`1` means no reduction). |

`completed` and `converged` are deliberately separate: a non-strict scene may
finish a frame after reaching an iteration limit. Conversely, if an exception
interrupts the frame, `completed` remains false and `engine.status().to_json()`
contains backend messages. `engine.to_json()` continues to provide the heavier
backend topology/feature inventory.

Profiling sessions collect this snapshot after every measured frame. In-memory
results expose it as `result['frame_stats']`; persisted runs write
`frame_stats.json`. Baseline reports use these structured counters for Newton,
line-search, and linear-solver diagnostics while Timer data remains the source
of duration gates.

## GPU Profiling (`uipc.profile.nsight`)

`nsight.session` has the same `advance` / `profile` interface, but runs the simulation under [Nsight Compute](https://developer.nvidia.com/nsight-compute) (`ncu`) to collect kernel-level GPU metrics.

```python
from uipc.profile import nsight

with nsight.session(world, name='test') as s:
    s.advance(50)
    s.profile(10)
```

Reports are written to the output directory as Markdown, JSON, and `.ncu-rep` (for the Nsight Compute GUI).
