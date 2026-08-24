# Profiling

## Performance Report

After running a simulation, use `SimulationStats` to generate a full performance report — including per-frame timer charts, a profiler heatmap, and system dependency graphs — in one call:

```python
from uipc.stats import SimulationStats

stats = SimulationStats()

for i in range(num_frames):
    world.advance()
    world.retrieve()
    stats.collect()

stats.summary_report(output_dir='perf_report', workspace='workspace')
```

This creates a `perf_report/` folder containing `report.md` and a set of SVG figures. Open `report.md` in any Markdown viewer to inspect the results.

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

## GPU Profiling (`uipc.profile.nsight`)

`nsight.session` has the same `advance` / `profile` interface, but runs the simulation under [Nsight Compute](https://developer.nvidia.com/nsight-compute) (`ncu`) to collect kernel-level GPU metrics.

```python
from uipc.profile import nsight

with nsight.session(world, name='test') as s:
    s.advance(50)
    s.profile(10)
```

Reports are written to the output directory as Markdown, JSON, and `.ncu-rep` (for the Nsight Compute GUI).
