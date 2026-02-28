"""Headless benchmark runner for UIPC simulation scenes.

Runs scenes headlessly, collects per-frame timer statistics via
:class:`~uipc.stats.SimulationStats`, and persists the results as JSON
for later comparison.

The primary input is a :class:`~uipc.World` — build your scene, create
an engine and world, then pass the world to :func:`session` or
:func:`run`.

Usage::

    from uipc import Scene, Engine, World
    from uipc.profile import run, session

    # Build scene + world
    scene = Scene(Scene.default_config())
    # ... populate the scene ...
    engine = Engine('cuda', 'my_workspace')
    world = World(engine)
    world.init(scene)

    # Simple
    result = run(world, num_frames=10, name='my_scene')
    print(result['summary'])

    # Flexible (context manager)
    with session(world, name='test', output_dir='bench') as s:
        s.advance(50)    # warmup 50 frames
        s.profile(10)    # benchmark 10 frames
    print(s.result)
"""

from __future__ import annotations

import json
import pathlib
import time
import traceback
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from uipc import Engine, Scene, World

__all__ = [
    'session',
    'run',
    'load_result',
    'compare',
]


# ---------------------------------------------------------------------------
# Session (context manager)
# ---------------------------------------------------------------------------

class Session:
    """Deferred-execution benchmarking session.

    Build a plan inside a ``with`` block, then the plan is
    executed when the block exits.

    The primary input is a :class:`~uipc.World` which is used
    directly in-process.  A :class:`~uipc.Scene` is also accepted
    as a shortcut (a temporary Engine + World is created internally).

    Usage::

        with session(world, name='test', output_dir='bench') as s:
            s.advance(50)    # warmup 50 frames (no stats)
            s.profile(10)    # benchmark 10 frames (collect stats)

        print(s.result)      # result dict available after the block

    The ``advance`` and ``profile`` calls are *deferred* — they
    describe what to do, but no work happens until ``__exit__``.
    """

    def __init__(
        self,
        world_or_scene: World | Scene,
        *,
        name: str = 'scene',
        output_dir: str | None = None,
        backend: str = 'cuda',
    ):
        from uipc import World
        if isinstance(world_or_scene, World):
            self._world = world_or_scene
            self._scene = None
        else:
            # Scene shortcut — will create Engine+World in _execute
            self._world = None
            self._scene = world_or_scene
        self._name = name
        self._output_dir = output_dir
        self._backend = backend
        self._steps: list[tuple[str, int]] = []
        self.result: dict | None = None

    def advance(self, num_frames: int) -> 'Session':
        """Queue *num_frames* warmup frames (no stats collected)."""
        self._steps.append(('advance', num_frames))
        return self

    def profile(self, num_frames: int) -> 'Session':
        """Queue *num_frames* frames to benchmark (stats collected)."""
        self._steps.append(('profile', num_frames))
        return self

    def __enter__(self) -> 'Session':
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if exc_type is not None:
            return
        self.result = _execute_session(self)


def session(
    world_or_scene: World | Scene,
    *,
    name: str = 'scene',
    output_dir: str | None = None,
    backend: str = 'cuda',
) -> Session:
    """Create a benchmarking session for use as a context manager.

    The primary input is a :class:`~uipc.World`.  The session runs
    advance/profile steps directly on the world in-process.

    A :class:`~uipc.Scene` is also accepted as a convenience shortcut
    (a temporary Engine + World is created internally).

    Usage::

        # Primary — pass a World
        with session(world, name='test', output_dir='bench') as s:
            s.profile(10)

        # Shortcut — pass a Scene
        with session(scene, name='test', output_dir='bench') as s:
            s.advance(50)
            s.profile(10)

    Args:
        world_or_scene: A :class:`~uipc.World` (preferred) or
               :class:`~uipc.Scene` (convenience shortcut).
        name: Human-readable label for this benchmark.
        output_dir: If given, results are written to this directory.
        backend: Engine backend name (default ``'cuda'``).
               Only used when a Scene is passed.

    Returns:
        A :class:`Session` to use as a context manager.
    """
    return Session(
        world_or_scene,
        name=name,
        output_dir=output_dir,
        backend=backend,
    )


def _execute_session(s: Session) -> dict:
    """Run the deferred benchmarking plan (internal)."""
    from uipc import Engine, World, Logger
    from uipc.stats import SimulationStats

    if not s._steps:
        raise ValueError('No steps queued. Call s.advance() or s.profile().')

    profile_frames = sum(n for op, n in s._steps if op == 'profile')
    if profile_frames == 0:
        raise ValueError('No profile() steps queued — nothing to benchmark.')

    Logger.set_level(Logger.Level.Warn)

    name = s._name
    output_dir = s._output_dir

    if s._world is not None:
        # Use the existing World directly — no replay needed
        world = s._world
        workspace = None
    else:
        # Create a fresh Engine + World from the Scene
        if output_dir is not None:
            workspace = str(pathlib.Path(output_dir) / f'workspace_{name}')
        else:
            import tempfile
            workspace = str(pathlib.Path(tempfile.mkdtemp()) / name)
        pathlib.Path(workspace).mkdir(parents=True, exist_ok=True)

        engine = Engine(s._backend, workspace)
        world = World(engine)
        world.init(s._scene)

    # Execute steps
    stats = SimulationStats()
    t0 = time.perf_counter()

    for op, n in s._steps:
        if op == 'advance':
            for _ in range(n):
                world.advance()
                world.retrieve()
        elif op == 'profile':
            for _ in range(n):
                world.advance()
                world.retrieve()
                stats.collect()

    wall_time = time.perf_counter() - t0

    summary = (
        f'Scene: {name}  |  Frames: {profile_frames}  |  '
        f'Wall time: {wall_time:.3f}s  |  '
        f'Avg: {wall_time / max(profile_frames, 1) * 1000:.1f}ms/frame'
    )

    result = {
        'name': name,
        'num_frames': profile_frames,
        'wall_time': wall_time,
        'stats': stats,
        'timer_frames': list(stats._frames),
        'summary': summary,
        'workspace': workspace,
        'steps': s._steps,
    }

    # Persist
    if output_dir is not None:
        scene_out = str(pathlib.Path(output_dir) / name)
        _save_result(result, scene_out)

    return result


# ---------------------------------------------------------------------------
# Simple API
# ---------------------------------------------------------------------------

def run(
    world_or_scene: World | Scene,
    num_frames: int = 10,
    *,
    name: str = 'scene',
    output_dir: str | None = None,
    backend: str = 'cuda',
) -> dict:
    """Run a simulation benchmark.

    The primary input is a :class:`~uipc.World` — benchmarks from its
    current frame.  A :class:`~uipc.Scene` is also accepted as a
    convenience shortcut.

    For full control, use :func:`session` as a context manager instead.

    Args:
        world_or_scene: A :class:`~uipc.World` (preferred) or
               :class:`~uipc.Scene` (convenience shortcut).
        num_frames: Number of simulation frames to benchmark.
        name: Human-readable label for this benchmark.
        output_dir: If given, results are written to this directory.
        backend: Engine backend name (default ``'cuda'``).
               Only used when a Scene is passed.

    Returns:
        Result dict with keys:

        - ``name`` -- label
        - ``num_frames`` -- frames benchmarked
        - ``wall_time`` -- total wall-clock seconds
        - ``stats`` -- the :class:`~uipc.stats.SimulationStats` instance
        - ``timer_frames`` -- raw list of per-frame timer dicts
        - ``summary`` -- short human-readable summary string
        - ``workspace`` -- path to the engine workspace
    """
    with session(
        world_or_scene,
        name=name,
        output_dir=output_dir,
        backend=backend,
    ) as s:
        s.profile(num_frames)

    return s.result


def load_result(result_dir: str) -> dict:
    """Load a previously saved benchmark result from disk.

    Args:
        result_dir: Path to the directory written by :func:`run`
                    (contains ``benchmark.json``).

    Returns:
        The stored dict with ``timer_frames`` populated from disk.
    """
    p = pathlib.Path(result_dir)
    meta_path = p / 'benchmark.json'
    if not meta_path.exists():
        raise FileNotFoundError(f'No benchmark.json in {result_dir}')

    data = json.loads(meta_path.read_text(encoding='utf-8'))

    frames_path = p / 'timer_frames.json'
    if frames_path.exists():
        data['timer_frames'] = json.loads(
            frames_path.read_text(encoding='utf-8')
        )

    # Reconstruct SimulationStats if possible
    if 'timer_frames' in data:
        try:
            from uipc.stats import SimulationStats
            stats = SimulationStats.__new__(SimulationStats)
            stats._frames = data['timer_frames']
            data['stats'] = stats
        except Exception:
            pass

    return data


def compare(
    before_dir: str,
    after_dir: str,
    output_dir: str | None = None,
) -> str:
    """Compare two benchmark result folders and produce a delta report.

    Each directory should contain ``benchmark.json`` and
    ``timer_frames.json`` as written by :func:`run`.

    Args:
        before_dir: Path to the *baseline* benchmark results.
        after_dir: Path to the *optimized* benchmark results.
        output_dir: If given, write the comparison report and any
                    generated charts to this directory.

    Returns:
        Markdown-formatted comparison report.
    """
    before = load_result(before_dir)
    after = load_result(after_dir)

    lines: list[str] = ['# Benchmark Comparison', '']

    # --- Wall-clock summary ---
    bw = before.get('wall_time', 0.0)
    aw = after.get('wall_time', 0.0)
    delta_pct = ((aw - bw) / bw * 100) if bw > 0 else 0.0
    sign = '+' if delta_pct >= 0 else ''
    speedup = bw / aw if aw > 0 else float('inf')

    lines.append('## Wall-Clock Summary')
    lines.append('')
    lines.append('| Metric | Before | After | Delta |')
    lines.append('|--------|--------|-------|-------|')
    lines.append(
        f'| Wall time | {bw:.3f}s | {aw:.3f}s '
        f'| {sign}{delta_pct:.1f}% |'
    )
    bf = before.get('num_frames', 0)
    af = after.get('num_frames', 0)
    b_avg = (bw / bf * 1000) if bf > 0 else 0
    a_avg = (aw / af * 1000) if af > 0 else 0
    avg_delta = ((a_avg - b_avg) / b_avg * 100) if b_avg > 0 else 0
    sign2 = '+' if avg_delta >= 0 else ''
    lines.append(
        f'| Avg frame | {b_avg:.1f}ms | {a_avg:.1f}ms '
        f'| {sign2}{avg_delta:.1f}% |'
    )
    lines.append(
        f'| Speedup | | | {speedup:.2f}x |'
    )
    lines.append('')

    # --- Per-timer comparison ---
    b_stats = before.get('stats')
    a_stats = after.get('stats')

    if b_stats is not None and a_stats is not None:
        lines.append('## Per-Timer Comparison')
        lines.append('')

        # Collect all timer names from both
        from uipc.stats import SimulationStats
        all_names: set[str] = set()
        for frame in b_stats._frames:
            SimulationStats._collect_names(frame, all_names)
        for frame in a_stats._frames:
            SimulationStats._collect_names(frame, all_names)

        # Aggregate mean durations
        timer_deltas: list[tuple[str, float, float, float]] = []
        for name in all_names:
            _, b_dur = b_stats.get_values(name, 'duration')
            _, a_dur = a_stats.get_values(name, 'duration')
            b_mean = float(b_dur.mean()) if len(b_dur) > 0 else 0.0
            a_mean = float(a_dur.mean()) if len(a_dur) > 0 else 0.0
            if b_mean > 0:
                delta = (a_mean - b_mean) / b_mean * 100
            else:
                delta = 0.0
            timer_deltas.append((name, b_mean, a_mean, delta))

        # Sort by absolute improvement (most improved first)
        timer_deltas.sort(key=lambda t: t[3])

        lines.append(
            '| Timer | Before (ms) | After (ms) | Delta |'
        )
        lines.append(
            '|-------|-------------|------------|-------|'
        )
        for name, b_mean, a_mean, delta in timer_deltas:
            if b_mean == 0 and a_mean == 0:
                continue
            sign3 = '+' if delta >= 0 else ''
            lines.append(
                f'| {name} | {b_mean*1000:.2f} | {a_mean*1000:.2f} '
                f'| {sign3}{delta:.1f}% |'
            )
        lines.append('')

    md_text = '\n'.join(lines)

    if output_dir is not None:
        out = pathlib.Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'comparison.md').write_text(md_text, encoding='utf-8')

        # Save structured data too
        comparison_data = {
            'before': {
                'name': before.get('name'),
                'wall_time': before.get('wall_time'),
                'num_frames': before.get('num_frames'),
            },
            'after': {
                'name': after.get('name'),
                'wall_time': after.get('wall_time'),
                'num_frames': after.get('num_frames'),
            },
            'speedup': speedup,
            'wall_time_delta_pct': delta_pct,
        }
        (out / 'comparison.json').write_text(
            json.dumps(comparison_data, indent=2), encoding='utf-8'
        )

    return md_text


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _deep_update(base: dict, overrides: dict) -> dict:
    """Recursively merge *overrides* into *base* in-place."""
    for k, v in overrides.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = v
    return base


def _save_result(result: dict, output_dir: str) -> None:
    """Write benchmark result to *output_dir*."""
    out = pathlib.Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # Metadata (without bulky frames or non-serialisable objects)
    meta = {
        'name': result['name'],
        'num_frames': result['num_frames'],
        'wall_time': result['wall_time'],
        'summary': result['summary'],
        'workspace': result.get('workspace'),
    }
    (out / 'benchmark.json').write_text(
        json.dumps(meta, indent=2), encoding='utf-8'
    )

    # Timer frames (full tree per frame)
    if 'timer_frames' in result:
        (out / 'timer_frames.json').write_text(
            json.dumps(result['timer_frames'], indent=2), encoding='utf-8'
        )

    # Generate stats report if we have a SimulationStats object
    stats = result.get('stats')
    if stats is not None and stats.num_frames > 0:
        report_dir = str(out / 'report')
        stats.summary_report(
            output_dir=report_dir,
            workspace=result.get('workspace'),
        )
