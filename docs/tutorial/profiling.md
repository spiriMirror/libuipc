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

## GPU Profiling (`uipc.profile.nsight`)

`nsight.session` has the same `advance` / `profile` interface, but runs the simulation under [Nsight Compute](https://developer.nvidia.com/nsight-compute) (`ncu`) to collect kernel-level GPU metrics.

```python
from uipc.profile import nsight

with nsight.session(world, name='test') as s:
    s.advance(50)
    s.profile(10)
```

Reports are written to the output directory as Markdown, JSON, and `.ncu-rep` (for the Nsight Compute GUI).
