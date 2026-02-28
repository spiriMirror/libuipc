# Performance Statistics

The `SimulationStats` class provides tools to collect, visualise, and report per-frame performance data from your simulation.  It works with the built-in timer system to produce publication-ready charts and a comprehensive Markdown report.

## Quick Start

```python
from uipc.stats import SimulationStats

# Create stats collector (automatically enables timers)
stats = SimulationStats()

for i in range(num_frames):
    world.advance()
    world.retrieve()
    stats.collect()

# Generate a full report folder
stats.summary_report(output_dir='perf_report', workspace=workspace)
```

The call to `summary_report` creates a folder with:

| File | Description |
|------|-------------|
| `report.md` | Markdown file referencing all figures |
| `profiler_heatmap.svg` | Hierarchical sunburst of the last frame's timer breakdown |
| `<timer>.svg` | Dual-axis chart (duration + count) per timer |
| `system_deps.svg` | Directed graph of backend system dependencies |

## Collecting Data

`SimulationStats` automatically calls `uipc.Timer.enable_all()` when created.  After each simulation step, call `collect()` to record the current frame's timer data:

```python
stats = SimulationStats()

for i in range(100):
    world.advance()
    world.retrieve()
    stats.collect()

print(f"Collected {stats.num_frames} frames")
```

## Plotting Individual Timers

### Line Plot

```python
stats.plot('Newton Iteration', metric='duration')
```

### Bar Chart

```python
stats.plot('Newton Iteration', metric='count', kind='bar',
           title='Newton Iteration Count per Frame')
```

### Multiple Timers

```python
stats.plot(['Newton Iteration', 'Line Search'],
           metric='duration', kind='line')
```

Save to a file instead of showing interactively:

```python
stats.plot('Newton Iteration', metric='duration',
           output_path='newton_duration.svg')
```

!!! note "Automatic Time Unit"
    The y-axis label is chosen automatically based on the data range: **µs**, **ms**, or **s**.  Frame numbers on the x-axis are always integers.

## Profiler Heatmap

The profiler heatmap is a sunburst chart showing the hierarchical time breakdown of a single frame:

```python
stats.profiler_heatmap(output_path='heatmap.svg')
```

By default it uses the last collected frame.  Pass `frame_index=0` to visualise the first frame instead.

## Markdown Table

Export per-frame data as a Markdown table:

```python
print(stats.to_markdown(['Newton Iteration', 'Line Search'],
                         metric='duration'))
```

Output:

```
| Frame | Newton Iteration | Line Search |
|-------|------------------|-------------|
| 0     | 0.0532s          | 0.0032s     |
| 1     | 0.0047s          | 0.0021s     |
| ...   | ...              | ...         |
```

## Summary Report

`summary_report()` generates a complete performance report as a folder of Markdown + SVG files:

```python
stats.summary_report(
    output_dir='perf_report',
    workspace=workspace,   # optional: path to workspace for system deps
)
```

The generated `report.md` includes:

1. **System Dependencies** (collapsible) — directed graph of backend systems
2. **Profiler Heatmap** — sunburst chart of the last frame
3. **Per-Frame Statistics** — one dual-axis chart per timer key
4. **Duration Table** (collapsible) — raw duration data
5. **Count Table** (collapsible) — raw count data

### Default Timer Keys

By default, `summary_report` looks for these timers using regex matching:

| Alias | Regex Pattern | Display Name |
|-------|--------------|--------------|
| `newton_iteration` | `(?i)newton.?iteration` | Newton Iteration |
| `global_linear_system` | `(?i)(solve.?)?global.?linear.?system` | Solve Global Linear System |
| `line_search` | `(?i)line.?search$` | Line Search |
| `dcd` | `(?i)detect.?dcd.?candidates` | Detect DCD Candidates |
| `spmv` | `(?i)spmv` | SPMV |

Timers not found in the data are automatically skipped.

### Custom Timer Keys

Pass exact timer names or alias keys:

```python
stats.summary_report(
    keys=['Newton Iteration', 'Build Linear System'],
    output_dir='perf_report',
)
```

### Custom Aliases

Add your own regex patterns:

```python
# Per-call
stats.summary_report(
    keys=['newton_iteration', 'my_timer'],
    aliases={'my_timer': r'(?i)my.*custom.*timer'},
    output_dir='perf_report',
)

# Or globally
SimulationStats.DEFAULT_ALIASES['my_timer'] = r'(?i)my.*custom.*timer'
SimulationStats.ALIAS_DISPLAY['my_timer'] = 'My Custom Timer'
```

## System Dependency Graph

When `workspace` is provided to `summary_report`, the backend system dependency graph is included automatically.  You can also generate it standalone:

```python
stats.system_dependency_graph(
    workspace,                          # or path to systems.json
    output_path='system_deps.svg',
)
```

The graph shows:

- **Green boxes** — engine-aware systems
- **Blue boxes** — internal systems
- **Solid blue arrows** — strong dependencies
- **Dashed grey arrows** — weak dependencies

## Sample Report

Below is an example of what the generated `report.md` looks like.  You can use this as a reference to understand the output structure.

---

<details>
<summary><strong>System Dependencies</strong></summary>

Directed graph of backend system dependencies.  Green nodes are **engine-aware**, blue nodes are internal.  Solid blue arrows are strong dependencies, dashed grey arrows are weak dependencies.

*(system_deps.svg would be displayed here)*

</details>

### Profiler Heatmap

Hierarchical time breakdown of the last collected frame (sunburst chart).

*(profiler_heatmap.svg would be displayed here)*

### Per-Frame Statistics

Each chart shows **duration** (left y-axis, line) and **count** (right y-axis, bars) per simulation frame.

#### Newton Iteration

*(newton_iteration.svg would be displayed here)*

#### Solve Global Linear System

*(solve_global_linear_system.svg would be displayed here)*

#### Line Search

*(line_search.svg would be displayed here)*

#### Detect DCD Candidates

*(detect_dcd_candidates.svg would be displayed here)*

<details>
<summary><strong>Duration Table</strong></summary>

| Frame | Newton Iteration | Solve Global Linear System | Line Search | Detect DCD Candidates |
|-------|------------------|----------------------------|-------------|----------------------|
| 0     | 53.2ms           | 48.2ms                     | 3.2ms       | 1.1ms                |
| 1     | 4.7ms            | 1.6ms                      | 2.1ms       | 0.7ms                |
| ...   | ...              | ...                        | ...         | ...                  |
| 15    | 34.4ms           | 25.7ms                     | 4.6ms       | 2.1ms                |
| 17    | 109.5ms          | 5.0ms                      | 9.0ms       | 3.1ms                |

</details>

<details>
<summary><strong>Count Table</strong></summary>

| Frame | Newton Iteration | Solve Global Linear System | Line Search | Detect DCD Candidates |
|-------|------------------|----------------------------|-------------|----------------------|
| 0     | 2                | 2                          | 2           | 1                    |
| ...   | ...              | ...                        | ...         | ...                  |
| 15    | 4                | 4                          | 4           | 3                    |
| 17    | 4                | 4                          | 4           | 3                    |

</details>

---

!!! tip "Viewing the Report"
    Open `report.md` in any Markdown viewer (VS Code, GitHub, GitLab) to see the SVG charts rendered inline.  The collapsible sections (`<details>`) work in GitHub and most modern Markdown renderers.

## Full Example

```python
import uipc
from uipc import Engine, World, Scene, SceneIO, Matrix4x4, view
from uipc.geometry import *
from uipc.constitution import AffineBodyConstitution
from uipc.stats import SimulationStats
import numpy as np

# Setup simulation
engine = Engine("cuda", "workspace")
world = World(engine)
scene = Scene(Scene.default_config())

abd = AffineBodyConstitution()
scene.contact_tabular().default_model(0.5, 1e9)
de = scene.contact_tabular().default_element()

io = SimplicialComplexIO()
cube = io.read("cube.msh")
label_surface(cube)
label_triangle_orient(cube)
cube = flip_inward_triangles(cube)
abd.apply_to(cube, 1e8)
de.apply_to(cube)

obj = scene.objects().create("cubes")
obj.geometries().create(cube)
obj.geometries().create(ground(0.0))

world.init(scene)

# Collect performance data
stats = SimulationStats()
for frame in range(20):
    world.advance()
    world.retrieve()
    stats.collect()

# Generate report
stats.summary_report(
    output_dir='perf_report',
    workspace='workspace',
)
```
