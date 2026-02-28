# Profiling

The `uipc.profile` package provides tools to benchmark and profile UIPC simulations. It has two modules:

| Module | Purpose | How it works |
|--------|---------|--------------|
| `uipc.profile` | **Benchmark** — wall-clock timer statistics | Runs in-process, collects `SimulationStats` |
| `uipc.profile.nsight` | **GPU profile** — kernel-level metrics | Launches subprocess under Nsight Compute (`ncu`) |

Both accept a `World` as the primary input. Build your scene, create an engine and world, then pass the world to the profiler.

## Setup

```python
from uipc import Scene, Engine, World
from uipc.geometry import tetmesh, ground, label_surface
from uipc.constitution import AffineBodyConstitution

# Build scene
scene = Scene(Scene.default_config())
abd = AffineBodyConstitution()
scene.contact_tabular().default_model(0.5, 1e9)
de = scene.contact_tabular().default_element()

# ... add geometries, constitutions, etc. ...

# Create engine and world
engine = Engine('cuda', 'my_workspace')
world = World(engine)
world.init(scene)
```

## Benchmarking (`uipc.profile`)

### Simple: `profile.run()`

```python
from uipc import profile

result = profile.run(world, num_frames=10, name='baseline', output_dir='bench')
print(result['summary'])
# Scene: baseline  |  Frames: 10  |  Wall time: 2.345s  |  Avg: 234.5ms/frame
```

The result dict contains:

| Key | Description |
|-----|-------------|
| `name` | Label |
| `num_frames` | Frames benchmarked |
| `wall_time` | Total wall-clock seconds |
| `stats` | `SimulationStats` instance (for plotting, reports) |
| `timer_frames` | Raw per-frame timer tree (list of dicts) |
| `summary` | Human-readable summary string |
| `workspace` | Path to the engine workspace |

### Flexible: `profile.session()`

Use the context manager to mix warmup and profiling:

```python
from uipc import profile

with profile.session(world, name='test', output_dir='bench') as s:
    s.advance(50)    # warmup 50 frames (no stats collected)
    s.profile(10)    # benchmark 10 frames (stats collected)

print(s.result['summary'])
```

The `advance()` and `profile()` calls are **deferred** — they describe what to do, but no work happens until the `with` block exits. You can chain any sequence:

```python
with profile.session(world, name='phased') as s:
    s.advance(10)     # warmup phase 1
    s.profile(5)      # measure phase 1
    s.advance(20)     # warmup phase 2
    s.profile(5)      # measure phase 2
```

### Comparing Results

```python
from uipc import profile

md = profile.compare('bench/baseline', 'bench/optimized', output_dir='comparison')
print(md)
```

This produces a Markdown report with wall-clock deltas and per-timer comparisons.

### Generating Reports

The benchmark result includes a `SimulationStats` object with visualization tools. See [Performance Statistics](./performance_stats.md) for details on:

- `stats.summary_report()` — comprehensive Markdown + SVG report
- `stats.profiler_heatmap()` — sunburst chart
- `stats.plot()` — per-frame charts
- `stats.to_markdown()` — Markdown tables

## GPU Profiling (`uipc.profile.nsight`)

This module wraps [Nsight Compute CLI](https://developer.nvidia.com/nsight-compute) (`ncu`) to collect kernel-level GPU metrics: duration, SM utilization, occupancy, register usage, etc.

!!! note "Prerequisites"
    - Nsight Compute must be installed. Set `NCU_PATH` or ensure `ncu` is on your `PATH`.
    - Use `--ncu-set full` for duration metrics (requires admin/root on some systems).

### Simple: `nsight.run()`

```python
from uipc.profile import nsight

result = nsight.run(world, num_frames=3, name='my_scene',
                    output_dir='ncu_results', ncu_set='default')
```

### Flexible: `nsight.session()`

```python
from uipc.profile import nsight

with nsight.session(world, name='my_scene') as s:
    s.profile(10)    # profiles 10 frames from world.frame()

print(s.result['report_md'])   # path to the Markdown report
print(s.result['report_json']) # path to the JSON report
```

### How It Works

Since `ncu` instruments an entire process, the profiler:

1. Saves the scene to a temporary JSON file via `SceneIO`.
2. If a `World` is passed, calls `world.dump()` to checkpoint the simulation state.
3. Generates a Python subprocess script that loads the scene, recovers the world state (if applicable), and runs the simulation.
4. Executes the script under `ncu`.
5. Parses the resulting `.ncu-rep` → CSV → Markdown/JSON reports.

When a `World` at frame N is passed, the subprocess uses `world.recover(N)` to skip directly to frame N — no replay cost.

### Output Files

```
<output_dir>/
  <name>_report.md      # kernel hotspot table (Markdown)
  <name>_report.json    # structured metrics with source_hints
  <name>.ncu-rep        # binary (for Nsight Compute GUI)
  <name>.csv            # raw CSV (already parsed into reports)
```

### Reading the Reports

The JSON report contains per-kernel entries like:

```json
{
  "name": "StacklessBVH::calcExtNodeSplitMetrics",
  "launches": 3,
  "total_duration_ns": 123456.0,
  "registers_per_thread": 40,
  "avg_sm_pct": 100.0,
  "avg_occupancy_pct": 48.0,
  "source_hint": "src/backends/cuda/collision_detection/",
  "optimization_hints": ["Low occupancy - consider reducing registers"]
}
```

Key fields:

| Field | Meaning |
|-------|---------|
| `name` | Shortened kernel name (extracted from muda lambda) |
| `total_duration_ns` | Total GPU time across all launches (nanoseconds) |
| `registers_per_thread` | Register pressure |
| `avg_occupancy_pct` | Achieved occupancy (higher is better) |
| `source_hint` | Directory where the CUDA source likely lives |
| `optimization_hints` | Auto-generated suggestions |

## Scene Shortcut

Both `profile.run()`/`profile.session()` and `nsight.run()`/`nsight.session()` also accept a `Scene` as a convenience shortcut. A temporary Engine + World is created internally:

```python
from uipc import profile
from uipc.profile import nsight

# These work but World is preferred:
result = profile.run(scene, num_frames=10, name='test', backend='none')

with nsight.session(scene, name='test') as s:
    s.advance(50)
    s.profile(10)
```

## CLI

The CLI provides quick access for benchmarking assets from [HuggingFace: MuGdxy/uipc-assets](https://huggingface.co/datasets/MuGdxy/uipc-assets):

```bash
# List available scenes
python -m uipc.cli.benchmark list

# Run a benchmark
python -m uipc.cli.benchmark run --scene cube_ground --frames 10 --output bench

# Profile with Nsight Compute
python -m uipc.cli.benchmark profile --scene cube_ground --frames 3 --output ncu_results

# Analyze a benchmark result
python -m uipc.cli.benchmark analyze bench/cube_ground --ncu-csv ncu_results/cube_ground.csv

# Compare before/after
python -m uipc.cli.benchmark compare bench/baseline bench/optimized --output comparison
```

## Full Example

```python
from uipc import Scene, Engine, World, profile
from uipc.assets import load
from uipc.profile import nsight

# 1. Build scene and world
scene = Scene(Scene.default_config())
load('cube_ground', scene)
engine = Engine('cuda', 'workspace')
world = World(engine)
world.init(scene)

# 2. Benchmark baseline
result = profile.run(world, num_frames=20, name='baseline', output_dir='bench')
print(result['summary'])

# Generate full stats report
result['stats'].summary_report(
    output_dir='bench/baseline/report',
    workspace=engine.workspace(),
)

# 3. Profile with Nsight Compute
with nsight.session(world, name='cube_ground', output_dir='ncu') as s:
    s.profile(5)

# 4. Read reports, identify bottlenecks, optimize code, rebuild

# 5. Re-benchmark after optimization
engine2 = Engine('cuda', 'workspace2')
world2 = World(engine2)
world2.init(scene)
after = profile.run(world2, num_frames=20, name='optimized', output_dir='bench')

# 6. Compare
md = profile.compare('bench/baseline', 'bench/optimized', output_dir='comparison')
print(md)
```
