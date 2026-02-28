"""Nsight Compute CLI wrapper for GPU kernel profiling.

Provides helpers to invoke ``ncu`` on UIPC simulation scenes, parse the
resulting reports, and rank kernel-level bottlenecks.

The primary input is a :class:`~uipc.World`.

Usage::

    from uipc.profile import nsight

    # Simple one-liner
    result = nsight.run(world, num_frames=3, name='test')

    # Flexible: mix warmup and profiling
    with nsight.session(world, name='test') as s:
        s.profile(10)    # profiles from world.frame()

    # Parse an existing .ncu-rep file to CSV, then to Python dicts
    rows = parse_ncu_csv('ncu_results/cube_ground.csv')

    # Combine with uipc.stats for a full bottleneck report
    md = analyze_bottlenecks(stats, rows)
    print(md)
"""

from __future__ import annotations

import csv
import io
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from uipc import Scene, World

__all__ = [
    'find_ncu',
    'session',
    'run',
    'export_csv',
    'parse_ncu_csv',
    'analyze_bottlenecks',
]


# ---------------------------------------------------------------------------
# ncu discovery
# ---------------------------------------------------------------------------

def find_ncu() -> str | None:
    """Auto-detect the ``ncu`` executable.

    Search order:

    1. ``NCU_PATH`` environment variable.
    2. ``ncu`` / ``ncu.bat`` on ``PATH``.
    3. Default NVIDIA install locations on Windows and Linux.

    Returns:
        Absolute path to ``ncu`` executable, or *None* if not found.
    """
    # 1. env var
    env = os.environ.get('NCU_PATH')
    if env and os.path.isfile(env):
        return env

    # 2. PATH
    which = shutil.which('ncu') or shutil.which('ncu.bat')
    if which:
        return which

    # 3. well-known install dirs
    candidates: list[str] = []
    if platform.system() == 'Windows':
        pf = os.environ.get('ProgramFiles', r'C:\Program Files')
        nv = os.path.join(pf, 'NVIDIA Corporation')
        if os.path.isdir(nv):
            for d in sorted(os.listdir(nv), reverse=True):
                if d.lower().startswith('nsight compute'):
                    candidates.append(os.path.join(nv, d, 'ncu.bat'))
                    candidates.append(os.path.join(nv, d, 'ncu.exe'))
    else:
        for base in ('/usr/local/cuda', '/opt/nvidia/nsight-compute'):
            if os.path.isdir(base):
                candidates.append(os.path.join(base, 'bin', 'ncu'))
        # versioned dirs
        nsight_base = '/opt/nvidia/nsight-compute'
        if os.path.isdir(nsight_base):
            for d in sorted(os.listdir(nsight_base), reverse=True):
                candidates.append(
                    os.path.join(nsight_base, d, 'ncu')
                )

    for c in candidates:
        if os.path.isfile(c):
            return c

    return None


# ---------------------------------------------------------------------------
# Profiling — Session (context manager)
# ---------------------------------------------------------------------------

class Session:
    """Deferred-execution profiling session.

    Build a profiling plan inside a ``with`` block, then the plan is
    executed under ``ncu`` when the block exits.

    Usage::

        with nsight.session(scene, name='test') as s:
            s.advance(50)    # warmup 50 frames (not profiled)
            s.profile(10)    # profile 10 frames

        # With a World — uses dump/recover, no replay cost:
        with nsight.session(world, engine, name='test') as s:
            s.profile(10)    # profiles from world.frame()

        print(s.result)      # results available after the with block

    The ``advance`` and ``profile`` calls are *deferred* — they
    describe what the subprocess should do, but no work happens
    until the ``with`` block exits.
    """

    def __init__(
        self,
        scene: Scene,
        *,
        name: str = 'scene',
        output_dir: str = 'ncu_results',
        ncu_path: str | None = None,
        kernel_filter: str | None = None,
        ncu_set: str = 'full',
        skip_kernels: int = 0,
        max_kernels: int | None = None,
        extra_ncu_args: list[str] | None = None,
        _recover_frame: int | None = None,
        _recover_workspace: str | None = None,
    ):
        self._scene = scene
        self._name = name
        self._output_dir = output_dir
        self._ncu_path = ncu_path
        self._kernel_filter = kernel_filter
        self._ncu_set = ncu_set
        self._skip_kernels = skip_kernels
        self._max_kernels = max_kernels
        self._extra_ncu_args = extra_ncu_args
        self._recover_frame = _recover_frame
        self._recover_workspace = _recover_workspace
        self._steps: list[tuple[str, int]] = []
        self.result: dict | None = None

    def advance(self, num_frames: int) -> 'Session':
        """Queue *num_frames* warmup frames (not profiled)."""
        self._steps.append(('advance', num_frames))
        return self

    def profile(self, num_frames: int) -> 'Session':
        """Queue *num_frames* frames to profile under ncu."""
        self._steps.append(('profile', num_frames))
        return self

    def __enter__(self) -> 'Session':
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if exc_type is not None:
            return  # don't run if an exception occurred in the block
        self.result = _execute_session(self)


def session(
    world_or_scene: World | Scene,
    *,
    name: str = 'scene',
    output_dir: str = 'ncu_results',
    ncu_path: str | None = None,
    kernel_filter: str | None = None,
    ncu_set: str = 'full',
    skip_kernels: int = 0,
    max_kernels: int | None = None,
    extra_ncu_args: list[str] | None = None,
) -> Session:
    """Create a profiling session for use as a context manager.

    The primary input is a :class:`~uipc.World`.  When a World is
    given, the world is dumped and the subprocess uses ``recover()``
    to skip to the right frame instantly — no replay cost.

    A :class:`~uipc.Scene` is also accepted as a convenience shortcut
    (a temporary Engine + World is created in the subprocess).

    Usage::

        # Primary — pass a World
        with nsight.session(world, name='test') as s:
            s.profile(10)    # profiles from world.frame()

        # Shortcut — pass a Scene
        with nsight.session(scene, name='test') as s:
            s.advance(50)
            s.profile(10)

    Args:
        world_or_scene: A :class:`~uipc.World` (preferred) or
               :class:`~uipc.Scene` (convenience shortcut).
        name: Label for output file naming.
        output_dir: Where to write output files.
        ncu_path: Explicit path to ``ncu``.
        kernel_filter: Regex for ``ncu -k``.
        ncu_set: Section set (``'full'``, ``'default'``, ``'detailed'``).
        skip_kernels: Kernel launches to skip (``ncu -s``).
        max_kernels: Max kernel launches to profile (``ncu -c``).
        extra_ncu_args: Extra CLI flags for ``ncu``.

    Returns:
        A :class:`Session` to use as a context manager.
    """
    from uipc import World

    recover_frame = None
    recover_workspace = None
    scene = world_or_scene

    if isinstance(world_or_scene, World):
        from uipc.backend import WorldVisitor
        wv = WorldVisitor(world_or_scene)
        scene = wv.scene().get()
        start_frame = world_or_scene.frame()

        if start_frame > 0:
            # Dump the world so the subprocess can recover instantly
            world_or_scene.dump()
            recover_frame = start_frame
            recover_workspace = str(wv.engine().workspace())

    s = Session(
        scene,
        name=name,
        output_dir=output_dir,
        ncu_path=ncu_path,
        kernel_filter=kernel_filter,
        ncu_set=ncu_set,
        skip_kernels=skip_kernels,
        max_kernels=max_kernels,
        extra_ncu_args=extra_ncu_args,
        _recover_frame=recover_frame,
        _recover_workspace=recover_workspace,
    )

    return s


def run(
    world_or_scene: World | Scene,
    num_frames: int = 3,
    output_dir: str = 'ncu_results',
    *,
    name: str = 'scene',
    ncu_path: str | None = None,
    kernel_filter: str | None = None,
    ncu_set: str = 'full',
    skip_kernels: int = 0,
    max_kernels: int | None = None,
    extra_ncu_args: list[str] | None = None,
) -> dict:
    """Profile a World or Scene with Nsight Compute CLI.

    The primary input is a :class:`~uipc.World`.  When a World is
    passed, ``dump()``/``recover()`` is used automatically.

    A :class:`~uipc.Scene` is also accepted as a convenience shortcut.

    For full control, use :func:`session` as a context manager instead.

    Args:
        world_or_scene: A :class:`~uipc.World` (preferred) or
               :class:`~uipc.Scene` (convenience shortcut).
        num_frames: Number of simulation frames to profile.
        output_dir: Where to write output files.
        name: Label for output file naming.
        ncu_path: Explicit path to ``ncu``.
        kernel_filter: Regex for ``ncu -k``.
        ncu_set: Section set (``'full'``, ``'default'``, ``'detailed'``).
        skip_kernels: Kernel launches to skip (``ncu -s``).
        max_kernels: Max kernel launches to profile (``ncu -c``).
        extra_ncu_args: Extra CLI flags for ``ncu``.

    Returns:
        Result dict (see :class:`Session`).
    """
    with session(
        world_or_scene,
        name=name,
        output_dir=output_dir,
        ncu_path=ncu_path,
        kernel_filter=kernel_filter,
        ncu_set=ncu_set,
        skip_kernels=skip_kernels,
        max_kernels=max_kernels,
        extra_ncu_args=extra_ncu_args,
    ) as s:
        s.profile(num_frames)

    return s.result


def _execute_session(s: Session) -> dict:
    """Run the deferred profiling plan under ncu (internal)."""
    from uipc import SceneIO

    if not s._steps:
        raise ValueError('No steps queued. Call s.advance() or s.profile().')

    profile_frames = sum(n for op, n in s._steps if op == 'profile')
    if profile_frames == 0:
        raise ValueError('No profile() steps queued — nothing to profile.')

    ncu = s._ncu_path or find_ncu()
    if ncu is None:
        raise FileNotFoundError(
            'Cannot find ncu (Nsight Compute CLI).  '
            'Install it or set the NCU_PATH environment variable.'
        )

    name = s._name
    out = pathlib.Path(s._output_dir)
    out.mkdir(parents=True, exist_ok=True)

    rep_path = str(out / f'{name}')
    csv_path = str(out / f'{name}.csv')

    # Save the scene so the subprocess can reload it
    scene_file = out / f'_scene_{name}.json'
    sio = SceneIO(s._scene)
    sio.save(str(scene_file))
    scene_file_posix = scene_file.resolve().as_posix()

    # Build the ncu command
    cmd: list[str] = [ncu]
    cmd += ['--set', s._ncu_set]
    cmd += ['-o', rep_path]
    if s._kernel_filter:
        cmd += ['-k', s._kernel_filter]
    if s._skip_kernels > 0:
        cmd += ['-s', str(s._skip_kernels)]
    if s._max_kernels is not None:
        cmd += ['-c', str(s._max_kernels)]
    if s._extra_ncu_args:
        cmd += s._extra_ncu_args

    # Generate the subprocess script from queued steps
    if s._recover_frame is not None and s._recover_workspace is not None:
        # Use dump/recover — subprocess restores to the dumped frame
        ws_posix = pathlib.Path(s._recover_workspace).resolve().as_posix()
        bench_lines = [
            'from uipc import Engine, World, SceneIO, Logger',
            'from uipc.stats import SimulationStats',
            '',
            'Logger.set_level(Logger.Level.Warn)',
            f'scene = SceneIO.load("{scene_file_posix}")',
            f'engine = Engine("cuda", "{ws_posix}")',
            'world = World(engine)',
            'world.init(scene)',
            f'world.recover({s._recover_frame})',
            'stats = SimulationStats()',
        ]
    else:
        bench_lines = [
            'from uipc import Engine, World, SceneIO, Logger',
            'from uipc.stats import SimulationStats',
            '',
            'Logger.set_level(Logger.Level.Warn)',
            f'scene = SceneIO.load("{scene_file_posix}")',
            'engine = Engine("cuda")',
            'world = World(engine)',
            'world.init(scene)',
            'stats = SimulationStats()',
        ]

    for op, n in s._steps:
        if op == 'advance':
            bench_lines += [
                f'# Advance {n} frames (warmup)',
                f'for _ in range({n}):',
                '    world.advance()',
                '    world.retrieve()',
            ]
        elif op == 'profile':
            bench_lines += [
                f'# Profile {n} frames',
                f'for _ in range({n}):',
                '    world.advance()',
                '    world.retrieve()',
                '    stats.collect()',
            ]

    bench_script_path = out / f'_bench_{name}.py'
    bench_script_path.write_text(
        '\n'.join(bench_lines) + '\n',
        encoding='utf-8',
    )
    cmd += [sys.executable, str(bench_script_path)]

    # Summarize the plan
    plan_parts = []
    if s._recover_frame is not None:
        frame = s._recover_frame
        plan_parts.append(f'recover to frame {frame}')
    else:
        frame = 0
    for op, n in s._steps:
        if op == 'advance':
            plan_parts.append(f'advance {n} frames ({frame}..{frame + n})')
        else:
            plan_parts.append(f'profile {n} frames ({frame}..{frame + n})')
        frame += n
    print(f'[ncu] Plan: {" -> ".join(plan_parts)}')
    print(f'[ncu] Running: {" ".join(cmd)}')
    proc = subprocess.run(cmd, capture_output=True, text=True)

    # Clean up temp files
    bench_script_path.unlink(missing_ok=True)
    scene_file.unlink(missing_ok=True)

    if proc.returncode != 0:
        error_msg = (
            f'--- stdout ---\n{proc.stdout}\n'
            f'--- stderr ---\n{proc.stderr}'
        )
        raise RuntimeError(
            f'ncu exited with code {proc.returncode}:\n{error_msg}'
        )

    # The actual .ncu-rep file may have a numeric suffix appended by ncu
    rep_file = _find_rep_file(out, name)

    # Export CSV from the .ncu-rep
    if rep_file:
        export_csv(ncu, rep_file, csv_path)
        rows = parse_ncu_csv(csv_path)
    else:
        rows = []

    # Generate agent-readable reports
    report_md_path = str(out / f'{name}_report.md')
    report_json_path = str(out / f'{name}_report.json')
    if rows:
        md = analyze_bottlenecks(ncu_rows=rows, top_n=20)
        pathlib.Path(report_md_path).write_text(md, encoding='utf-8')

        summary = _build_kernel_summary(rows)
        pathlib.Path(report_json_path).write_text(
            json.dumps(summary, indent=2, default=str), encoding='utf-8'
        )

    return {
        'ncu_rep': str(rep_file) if rep_file else None,
        'csv_path': csv_path if rep_file else None,
        'report_md': report_md_path if rows else None,
        'report_json': report_json_path if rows else None,
        'rows': rows,
        'steps': s._steps,
        'command': ' '.join(cmd),
        'stdout': proc.stdout,
        'stderr': proc.stderr,
    }


def export_csv(
    ncu_path: str,
    rep_file: str | pathlib.Path,
    csv_path: str | pathlib.Path,
) -> None:
    """Export an ``.ncu-rep`` file to CSV using ``ncu --import``.

    Uses ``--page raw`` which outputs one row per kernel launch with
    all collected metrics as columns.

    Args:
        ncu_path: Path to ``ncu`` executable.
        rep_file: Path to the ``.ncu-rep`` report file.
        csv_path: Destination CSV file path.
    """
    cmd = [
        ncu_path,
        '--import', str(rep_file),
        '--csv',
        '--page', 'raw',
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f'ncu CSV export failed ({proc.returncode}):\n{proc.stderr}'
        )
    pathlib.Path(csv_path).write_text(proc.stdout, encoding='utf-8')


def parse_ncu_csv(csv_path: str | pathlib.Path) -> list[dict[str, Any]]:
    """Parse an ``ncu``-generated CSV file into a list of row dicts.

    Each row corresponds to one kernel launch and contains metric
    name-value pairs.  The ``--page raw`` format has a units row
    immediately after the header which is automatically skipped.

    Key columns (from ``--page raw``):

    - ``Kernel Name`` -- short demangled name
    - ``launch__kernel_name`` -- full demangled name
    - ``launch__registers_per_thread``
    - ``launch__block_size``, ``launch__grid_size``
    - ``gpu__time_duration.sum`` -- kernel duration (normalized to nanoseconds)
    - ``sm__maximum_warps_per_active_cycle_pct`` -- occupancy %

    The function automatically reads the units row (if present) and converts
    all duration values to nanoseconds for internal consistency.

    Args:
        csv_path: Path to the CSV file produced by :func:`export_csv`.

    Returns:
        List of dicts, one per profiled kernel launch.
    """
    text = pathlib.Path(csv_path).read_text(encoding='utf-8')

    # ncu CSV may have leading comment lines starting with "=="
    lines = [
        line for line in text.splitlines()
        if not line.startswith('==')
    ]
    if not lines:
        return []

    reader = csv.DictReader(io.StringIO('\n'.join(lines)))
    rows: list[dict[str, Any]] = []
    units_row_skipped = False
    duration_unit = None  # Will be 'ns', 'us', 'ms', or 's'
    
    for row in reader:
        # The --page raw format has a units row right after the header.
        # Detect it: the "ID" column is empty and "Kernel Name" is empty.
        if not units_row_skipped:
            id_val = (row.get('ID') or '').strip().strip('"')
            if id_val == '':
                # This is the units row - read the duration unit
                # Try to find the duration column
                duration_key = None
                for candidate in ['gpu__time_duration.sum', 'gpu__time_duration']:
                    if candidate in row:
                        duration_key = candidate
                        break
                if not duration_key:
                    # Try prefix match
                    for k in row.keys():
                        if k and 'gpu__time_duration' in k.lower():
                            duration_key = k
                            break
                if duration_key:
                    unit_str = (row.get(duration_key) or '').strip().strip('"').lower()
                    # Map common unit strings to our internal unit
                    if unit_str in ('ns', 'nsec', 'nanosecond', 'nanoseconds'):
                        duration_unit = 'ns'
                    elif unit_str in ('us', 'usec', 'microsecond', 'microseconds', 'µs'):
                        duration_unit = 'us'
                    elif unit_str in ('ms', 'msec', 'millisecond', 'milliseconds'):
                        duration_unit = 'ms'
                    elif unit_str in ('s', 'sec', 'second', 'seconds'):
                        duration_unit = 's'
                    else:
                        # Default to nanoseconds if unknown
                        duration_unit = 'ns'
                units_row_skipped = True
                continue
            units_row_skipped = True  # no units row present
            # Default to nanoseconds if no units row found
            if duration_unit is None:
                duration_unit = 'ns'

        # Try to convert numeric strings
        parsed: dict[str, Any] = {}
        for k, v in row.items():
            if k is None:
                continue
            k = k.strip().strip('"')
            if v is None:
                parsed[k] = None
                continue
            v = v.strip().strip('"')
            # Remove thousands separators (ncu uses "1,024" format)
            v_clean = v.replace(',', '')
            try:
                parsed[k] = int(v_clean)
            except ValueError:
                try:
                    parsed[k] = float(v_clean)
                except ValueError:
                    parsed[k] = v
            # Handle percentage strings like "45.2%"
            if isinstance(parsed[k], str) and parsed[k].endswith('%'):
                try:
                    parsed[k] = float(parsed[k][:-1])
                except ValueError:
                    pass
        
        # Convert duration to nanoseconds for internal consistency
        duration_key = None
        for candidate in ['gpu__time_duration.sum', 'gpu__time_duration']:
            if candidate in parsed:
                duration_key = candidate
                break
        if not duration_key:
            # Try prefix match
            for k in parsed.keys():
                if k and 'gpu__time_duration' in k.lower():
                    duration_key = k
                    break
        if duration_key and duration_key in parsed:
            val = parsed[duration_key]
            if isinstance(val, (int, float)):
                # Convert to nanoseconds
                if duration_unit == 'us':
                    parsed[duration_key] = val * 1000.0
                elif duration_unit == 'ms':
                    parsed[duration_key] = val * 1_000_000.0
                elif duration_unit == 's':
                    parsed[duration_key] = val * 1_000_000_000.0
                # 'ns' or None: already in nanoseconds, no conversion needed
        
        rows.append(parsed)
    return rows


# ---------------------------------------------------------------------------
# Bottleneck analysis
# ---------------------------------------------------------------------------

def analyze_bottlenecks(
    stats=None,
    ncu_rows: list[dict] | None = None,
    *,
    top_n: int = 10,
) -> str:
    """Produce a Markdown bottleneck report combining stats + ncu data.

    Args:
        stats: A :class:`~uipc.stats.SimulationStats` instance (optional).
        ncu_rows: Parsed ncu CSV rows from :func:`parse_ncu_csv` (optional).
        top_n: Number of top entries to show in each section.

    Returns:
        Markdown-formatted report string.
    """
    sections: list[str] = ['# GPU Bottleneck Analysis', '']

    # --- Section 1: uipc.stats timer breakdown ---
    if stats is not None and stats.num_frames > 0:
        sections.append('## Timer Breakdown (uipc.stats)')
        sections.append('')
        sections.append(
            'Aggregated across all collected frames.  '
            'Sorted by total duration (descending).'
        )
        sections.append('')

        # Collect all timer names
        all_names: set[str] = set()
        for frame in stats._frames:
            stats._collect_names(frame, all_names)

        # Aggregate
        timer_totals: dict[str, dict] = {}
        for name in all_names:
            frames_arr, durations = stats.get_values(name, 'duration')
            _, counts = stats.get_values(name, 'count')
            if len(durations) == 0:
                continue
            timer_totals[name] = {
                'total_duration': float(durations.sum()),
                'mean_duration': float(durations.mean()),
                'max_duration': float(durations.max()),
                'total_count': int(counts.sum()),
                'mean_count': float(counts.mean()),
                'appearances': len(frames_arr),
            }

        ranked = sorted(
            timer_totals.items(),
            key=lambda kv: kv[1]['total_duration'],
            reverse=True,
        )[:top_n]

        sections.append(
            '| Rank | Timer | Total (s) | Mean (ms) | Max (ms) '
            '| Total Count |'
        )
        sections.append(
            '|------|-------|-----------|-----------|----------'
            '|-------------|'
        )
        for i, (name, d) in enumerate(ranked, 1):
            sections.append(
                f'| {i} | {name} | {d["total_duration"]:.4f} '
                f'| {d["mean_duration"]*1000:.2f} '
                f'| {d["max_duration"]*1000:.2f} '
                f'| {d["total_count"]} |'
            )
        sections.append('')

    # --- Section 2: ncu kernel hotspots ---
    if ncu_rows:
        sections.append('## Kernel Hotspots (Nsight Compute)')
        sections.append('')

        # Detect key column names (ncu CSV column names vary by version)
        duration_key = _find_key_prefix(
            ncu_rows[0],
            ['Duration', 'gpu__time_duration.sum', 'Kernel Duration',
             'gpu__time_duration'],
        )
        sm_key = _find_key_prefix(
            ncu_rows[0],
            ['SM [%]', 'SM Coverage',
             'sm__throughput.avg.pct_of_peak_sustained_elapsed',
             'sm__maximum_warps_per_active_cycle_pct'],
        )
        mem_key = _find_key_prefix(
            ncu_rows[0],
            ['Memory [%]', 'Mem Busy',
             'dram__throughput.avg.pct_of_peak_sustained_elapsed'],
        )
        occ_key = _find_key_prefix(
            ncu_rows[0],
            ['Achieved Occupancy', 'Occupancy',
             'sm__warps_active.avg.pct_of_peak_sustained_active',
             'sm__maximum_warps_avg_per_active_cycle'],
        )
        name_key = _find_key_prefix(
            ncu_rows[0],
            ['Kernel Name', 'kernel_name', 'launch__kernel_name'],
        )
        regs_key = _find_key_prefix(
            ncu_rows[0],
            ['Registers Per Thread', 'launch__registers_per_thread'],
        )
        block_key = _find_key_prefix(
            ncu_rows[0],
            ['Block Size', 'launch__block_size'],
        )
        grid_key = _find_key_prefix(
            ncu_rows[0],
            ['Grid Size', 'launch__grid_size'],
        )

        if name_key is None:
            sections.append('*Could not identify kernel name column.*')
            sections.append('')
        else:
            # Aggregate by kernel name
            kernel_agg: dict[str, dict] = {}
            for row in ncu_rows:
                kname = str(row.get(name_key, 'unknown'))
                # Shorten demangled names
                short = _shorten_kernel_name(kname)
                if short not in kernel_agg:
                    kernel_agg[short] = {
                        'full_name': kname,
                        'launches': 0,
                        'total_duration': 0.0,
                        'regs': '-',
                        'block_size': '-',
                        'grid_size': '-',
                        'sm_pcts': [],
                        'mem_pcts': [],
                        'occ_pcts': [],
                    }
                agg = kernel_agg[short]
                agg['launches'] += 1
                if duration_key:
                    val = row.get(duration_key)
                    if isinstance(val, (int, float)):
                        agg['total_duration'] += val
                if regs_key:
                    val = row.get(regs_key)
                    if isinstance(val, (int, float)):
                        agg['regs'] = int(val)
                if block_key:
                    val = row.get(block_key)
                    if val is not None:
                        agg['block_size'] = val
                if grid_key:
                    val = row.get(grid_key)
                    if val is not None:
                        agg['grid_size'] = val
                for col, lst in [
                    (sm_key, 'sm_pcts'),
                    (mem_key, 'mem_pcts'),
                    (occ_key, 'occ_pcts'),
                ]:
                    if col:
                        val = row.get(col)
                        if isinstance(val, (int, float)):
                            agg[lst].append(val)

            ranked_kernels = sorted(
                kernel_agg.items(),
                key=lambda kv: kv[1]['total_duration'],
                reverse=True,
            )[:top_n]

            # Determine best duration unit (ns from gpu__time_duration.sum)
            max_dur = max(
                (a['total_duration'] for _, a in ranked_kernels),
                default=0,
            )
            if max_dur > 1e9:
                dur_scale, dur_unit = 1e-9, 's'
            elif max_dur > 1e6:
                dur_scale, dur_unit = 1e-6, 'ms'
            else:
                dur_scale, dur_unit = 1e-3, 'us'

            header = f'| Rank | Kernel | Launches | Total ({dur_unit})'
            sep = '|------|--------|----------|----------'
            header += ' | Regs'
            sep += '|------'
            if sm_key:
                header += ' | Avg SM%'
                sep += '|--------'
            if mem_key:
                header += ' | Avg Mem%'
                sep += '|---------'
            if occ_key:
                header += ' | Avg Occ%'
                sep += '|---------'
            header += ' |'
            sep += '|'

            sections.append(header)
            sections.append(sep)

            for i, (short, agg) in enumerate(ranked_kernels, 1):
                dur_display = agg['total_duration'] * dur_scale
                regs = agg.get('regs', '-')
                line = (
                    f'| {i} | `{short}` | {agg["launches"]} '
                    f'| {dur_display:.2f}'
                    f' | {regs}'
                )
                if sm_key and agg['sm_pcts']:
                    avg_sm = sum(agg['sm_pcts']) / len(agg['sm_pcts'])
                    line += f' | {avg_sm:.1f}'
                elif sm_key:
                    line += ' | -'
                if mem_key and agg['mem_pcts']:
                    avg_mem = sum(agg['mem_pcts']) / len(agg['mem_pcts'])
                    line += f' | {avg_mem:.1f}'
                elif mem_key:
                    line += ' | -'
                if occ_key and agg['occ_pcts']:
                    avg_occ = sum(agg['occ_pcts']) / len(agg['occ_pcts'])
                    line += f' | {avg_occ:.1f}'
                elif occ_key:
                    line += ' | -'
                line += ' |'
                sections.append(line)
            sections.append('')

            # Provide improvement hints
            sections.append('### Potential Improvement Areas')
            sections.append('')
            for i, (short, agg) in enumerate(ranked_kernels[:5], 1):
                hints: list[str] = []
                if sm_key and agg['sm_pcts']:
                    avg_sm = sum(agg['sm_pcts']) / len(agg['sm_pcts'])
                    if avg_sm < 30:
                        hints.append(
                            'Low SM utilization -- consider increasing '
                            'parallelism or reducing register pressure'
                        )
                if mem_key and agg['mem_pcts']:
                    avg_mem = sum(agg['mem_pcts']) / len(agg['mem_pcts'])
                    if avg_mem > 70:
                        hints.append(
                            'Memory-bound -- consider coalescing accesses, '
                            'using shared memory, or reducing data movement'
                        )
                if occ_key and agg['occ_pcts']:
                    avg_occ = sum(agg['occ_pcts']) / len(agg['occ_pcts'])
                    if avg_occ < 50:
                        hints.append(
                            'Low occupancy -- reduce registers/shared mem '
                            'per thread or adjust block size'
                        )
                if not hints:
                    hints.append('No obvious bottleneck from high-level metrics')

                sections.append(f'{i}. **`{short}`**')
                for h in hints:
                    sections.append(f'   - {h}')
            sections.append('')

    if len(sections) <= 2:
        sections.append(
            '*No data provided. Pass stats and/or ncu_rows.*'
        )

    return '\n'.join(sections)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _find_rep_file(
    directory: pathlib.Path, base_name: str
) -> pathlib.Path | None:
    """Find the .ncu-rep file in *directory* matching *base_name*."""
    # ncu may append a number, e.g. cube_ground.ncu-rep or cube_ground.1.ncu-rep
    candidates = sorted(
        directory.glob(f'{base_name}*.ncu-rep'), reverse=True
    )
    return candidates[0] if candidates else None


def _find_key_prefix(
    row: dict, candidates: list[str]
) -> str | None:
    """Find a column key by exact match first, then prefix match."""
    # Exact match
    for c in candidates:
        if c in row:
            return c
    # Prefix / substring match (ncu columns sometimes have extra suffixes)
    row_keys = list(row.keys())
    for c in candidates:
        for rk in row_keys:
            if rk and c.lower() in rk.lower():
                return rk
    return None


def _shorten_kernel_name(name: str) -> str:
    """Shorten a demangled CUDA kernel name for display.

    For muda kernels like ``parallel_for_kernel<Foo::bar()::lambda>``,
    extracts the enclosing function ``Foo::bar`` as the meaningful
    identifier rather than stripping all template args.
    """
    # --- muda pattern: extract the enclosing function from the lambda ---
    # Match: parallel_for_kernel<ClassName::method(...)::lambda>
    #   or:  generic_kernel<ClassName::method(...)::lambda>
    m = re.search(
        r'(?:parallel_for_kernel|generic_kernel)<'
        r'([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)'  # Class::method
        r'\(',                                  # opening paren of args
        name,
    )
    if m:
        caller = m.group(1)
        # When a function has multiple lambdas, NVCC appends
        # "(instance N)" to distinguish them. Preserve this.
        inst = re.search(r'\(instance\s+(\d+)\)', name)
        suffix = f'#{inst.group(1)}' if inst else ''
        if 'generic_kernel' in name:
            return f'[generic] {caller}{suffix}'
        return f'{caller}{suffix}'

    # --- buffer operations: kernel_fill, kernel_assign, kernel_construct ---
    m = re.search(r'buffer::(kernel_\w+)<([^>]+)>', name)
    if m:
        op = m.group(1)
        # Simplify the type
        type_name = m.group(2)
        # Remove Eigen:: template noise
        type_short = re.sub(r'Eigen::Matrix<[^>]+>', 'Matrix', type_name)
        type_short = re.sub(r'uipc::', '', type_short)
        return f'buffer::{op}<{type_short}>'

    # --- CUB / thrust kernels ---
    m = re.search(r'cub::(\w+)', name)
    if m:
        return f'cub::{m.group(1)}'

    m = re.search(r'for_each::static_kernel', name)
    if m:
        # Try to identify what it's filling
        fill_m = re.search(r'(fill|tabulate|uninitialized_fill)', name)
        if fill_m:
            return f'thrust::{fill_m.group(1)}'
        return 'thrust::for_each'

    # --- Fallback: strip templates but keep outer name ---
    result = []
    depth = 0
    for ch in name:
        if ch == '<':
            depth += 1
        elif ch == '>':
            depth -= 1
        elif depth == 0:
            result.append(ch)
    short = ''.join(result).strip()
    # Take last qualified segment
    if '::' in short:
        parts = short.split('::')
        short = '::'.join(parts[-2:]) if len(parts) >= 2 else parts[-1]
    # Truncate if still too long
    if len(short) > 60:
        short = short[:57] + '...'
    return short or name[:60]


# Map from class/function name patterns to source file paths
# (relative to src/backends/cuda/)
_KERNEL_SOURCE_MAP = {
    'AffineBody':          'affine_body/',
    'ABD':                 'affine_body/',
    'FiniteElement':       'finite_element/',
    'FEM':                 'finite_element/',
    'NeoHookean':          'finite_element/constitutions/',
    'LinearPCG':           'linear_system/linear_pcg.cu',
    'SPMV':                'linear_system/spmv.cu',
    'GlobalLinearSystem':  'linear_system/global_linear_system.cu',
    'Preconditioner':      'linear_system/',
    'LineSearch':          'line_search/',
    'StacklessBVH':        'collision_detection/',
    'AtomicCountingLBVH':  'collision_detection/',
    'TrajectoryFilter':    'collision_detection/',
    'SimplexTrajectory':   'collision_detection/',
    'HalfPlane':           'collision_detection/',
    'GlobalContact':       'contact_system/',
    'ContactCoeff':        'contact_system/',
    'IPC':                 'contact_system/',
    'DytopoEffect':       'dytopo_effect_system/',
    'GlobalVertex':        'global_geometry/',
    'GlobalSimplicial':    'global_geometry/',
    'Animator':            'animator/',
    'ExternalForce':       'external_force/',
    'DiffSim':             'diff_sim/',
    'TimeIntegrator':      'time_integrator/',
    'NewtonTolerance':     'newton_tolerance/',
    'Particle':            'finite_element/',
}


def _guess_source_file(kernel_short_name: str) -> str | None:
    """Guess the CUDA source directory for a kernel based on its name."""
    for pattern, path in _KERNEL_SOURCE_MAP.items():
        if pattern.lower() in kernel_short_name.lower():
            return f'src/backends/cuda/{path}'
    return None


def _build_kernel_summary(ncu_rows: list[dict]) -> dict:
    """Build a compact, agent-readable JSON summary from ncu rows.

    The summary contains per-kernel aggregated metrics and source file
    hints that an AI agent can use to locate and optimize code.
    """
    name_key = _find_key_prefix(
        ncu_rows[0],
        ['Kernel Name', 'kernel_name', 'launch__kernel_name'],
    )
    duration_key = _find_key_prefix(
        ncu_rows[0],
        ['Duration', 'gpu__time_duration.sum'],
    )
    regs_key = _find_key_prefix(
        ncu_rows[0],
        ['Registers Per Thread', 'launch__registers_per_thread'],
    )
    sm_key = _find_key_prefix(
        ncu_rows[0],
        ['SM [%]', 'sm__maximum_warps_per_active_cycle_pct'],
    )
    occ_key = _find_key_prefix(
        ncu_rows[0],
        ['Achieved Occupancy', 'sm__maximum_warps_avg_per_active_cycle'],
    )
    block_key = _find_key_prefix(
        ncu_rows[0],
        ['Block Size', 'launch__block_size'],
    )
    grid_key = _find_key_prefix(
        ncu_rows[0],
        ['Grid Size', 'launch__grid_size'],
    )

    # Aggregate by shortened kernel name
    kernel_agg: dict[str, dict] = {}
    for row in ncu_rows:
        if name_key is None:
            continue
        raw_name = str(row.get(name_key, 'unknown'))
        short = _shorten_kernel_name(raw_name)

        if short not in kernel_agg:
            kernel_agg[short] = {
                'short_name': short,
                'full_names': [],
                'launches': 0,
                'total_duration_ns': 0.0,
                'registers_per_thread': None,
                'block_sizes': [],
                'grid_sizes': [],
                'sm_pcts': [],
                'occ_pcts': [],
                'source_hint': _guess_source_file(short),
            }
        agg = kernel_agg[short]
        agg['launches'] += 1
        if raw_name not in agg['full_names']:
            agg['full_names'].append(raw_name)

        def _get_num(key):
            if key is None:
                return None
            v = row.get(key)
            return v if isinstance(v, (int, float)) else None

        dur = _get_num(duration_key)
        if dur is not None:
            agg['total_duration_ns'] += dur

        regs = _get_num(regs_key)
        if regs is not None:
            agg['registers_per_thread'] = int(regs)

        sm = _get_num(sm_key)
        if sm is not None:
            agg['sm_pcts'].append(sm)

        occ = _get_num(occ_key)
        if occ is not None:
            agg['occ_pcts'].append(occ)

        bs = _get_num(block_key)
        if bs is not None and bs not in agg['block_sizes']:
            agg['block_sizes'].append(int(bs))

        gs = _get_num(grid_key)
        if gs is not None and gs not in agg['grid_sizes']:
            agg['grid_sizes'].append(int(gs))

    # Compute averages and build final list
    kernels = []
    for short, agg in sorted(
        kernel_agg.items(),
        key=lambda kv: kv[1]['total_duration_ns'],
        reverse=True,
    ):
        entry = {
            'name': agg['short_name'],
            'launches': agg['launches'],
            'total_duration_ns': agg['total_duration_ns'],
            'registers_per_thread': agg['registers_per_thread'],
            'block_sizes': agg['block_sizes'],
            'grid_sizes': agg['grid_sizes'],
            'source_hint': agg['source_hint'],
        }
        if agg['sm_pcts']:
            entry['avg_sm_pct'] = round(
                sum(agg['sm_pcts']) / len(agg['sm_pcts']), 1
            )
        if agg['occ_pcts']:
            entry['avg_occupancy_pct'] = round(
                sum(agg['occ_pcts']) / len(agg['occ_pcts']), 1
            )
        # Add optimization hints
        hints = []
        if entry.get('avg_sm_pct', 100) < 30:
            hints.append('Low SM utilization - increase parallelism')
        if entry.get('avg_occupancy_pct', 100) < 25:
            hints.append('Very low occupancy - reduce register pressure')
        elif entry.get('avg_occupancy_pct', 100) < 50:
            hints.append('Low occupancy - consider reducing registers')
        regs = entry.get('registers_per_thread')
        if regs and regs > 64:
            hints.append(
                f'High register usage ({regs}/thread) - '
                f'consider __launch_bounds__ or code simplification'
            )
        if hints:
            entry['optimization_hints'] = hints

        # Add full demangled names for reference (truncated)
        entry['full_names'] = [
            n[:200] for n in agg['full_names'][:3]
        ]

        kernels.append(entry)

    return {
        'total_kernels_profiled': len(ncu_rows),
        'unique_kernel_types': len(kernel_agg),
        'kernels': kernels,
    }
