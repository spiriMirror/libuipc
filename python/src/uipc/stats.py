from __future__ import annotations

from uipc import Logger
import copy
import os
import re
import json
import pathlib
from os import PathLike
from typing import Any, Iterable, Literal, Mapping
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import MaxNLocator

__all__ = ['SimulationStats']


class SimulationStats:
    """Collect and analyse per-frame simulation performance statistics.

    Typical usage::

        import uipc
        from uipc.stats import SimulationStats

        stats = SimulationStats()

        for i in range(num_frames):
            world.advance()
            world.retrieve()
            stats.collect()

        # Curve plot of Newton iteration count per frame
        stats.plot('Newton', metric='count')

        # Bar chart comparing multiple timers
        stats.plot(['Newton', 'PCG'], metric='duration', kind='bar')

        # Markdown table for documentation or logging
        print(stats.to_markdown(['Newton', 'PCG'], metric='count'))

        # Comprehensive summary report (Markdown + SVGs in a folder)
        stats.summary_report(output_dir='perf_report')

        # Standalone profiler heatmap (sunburst chart)
        stats.profiler_heatmap()
    """

    #: Default name aliases.  Each key is a short label used in reports;
    #: each value is a regex pattern matched against actual timer names.
    #: The first match wins.  Override or extend via
    #: ``SimulationStats.DEFAULT_ALIASES['my_key'] = r'...'``.
    DEFAULT_ALIASES = {
        'newton_iteration':     r'(?i)newton.?iteration',
        'global_linear_system': r'(?i)(solve.?)?global.?linear.?system',
        'line_search':          r'(?i)line.?search$',
        'dcd':                  r'(?i)detect.?dcd.?candidates',
        'spmv':                 r'(?i)spmv',
    }

    #: Pretty display names for the short alias keys.
    ALIAS_DISPLAY = {
        'newton_iteration':     'Newton Iteration',
        'global_linear_system': 'Solve Global Linear System',
        'line_search':          'Line Search',
        'dcd':                  'Detect DCD Candidates',
        'spmv':                 'SPMV',
    }

    def __init__(self) -> None:
        # Each entry is the dict returned by uipc.Timer.report_as_json()
        self._frames: list[dict[str, Any]] = []
        # Ensure timers are enabled so collect() captures data
        try:
            import uipc
            uipc.Timer.enable_all()
        except (AttributeError, ImportError):
            pass  # native module not available (e.g. in tests)

    @classmethod
    def load_timer_frames_json(
        cls, path: str | PathLike[str] | pathlib.Path
    ) -> 'SimulationStats':
        """
        Load a ``SimulationStats`` instance from a ``timer_frames.json`` file.

        Parameters
        ----------
        path : str | os.PathLike[str] | pathlib.Path
            Path to a JSON file containing serialized timer frames.

        Returns
        -------
        SimulationStats
            A ``SimulationStats`` instance loaded from the JSON file.
        """
        p = pathlib.Path(path)
        frames = json.loads(p.read_text(encoding='utf-8'))
        obj = cls()
        obj._frames = list(frames) if frames is not None else []
        return obj

    def save_timer_frames_json(
        self, path: str | PathLike[str] | pathlib.Path
    ) -> pathlib.Path:
        """
        Save collected timer frames to a JSON file.

        Parameters
        ----------
        path : str | os.PathLike[str] | pathlib.Path
            Output file path.

        Returns
        -------
        pathlib.Path
            Normalized output path where the JSON file is written.
        """
        p = pathlib.Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self._frames, indent=2), encoding='utf-8')
        return p

    @staticmethod
    def create_comparison(
        comparison_map: Mapping[str, SimulationStats | str | PathLike[str] | pathlib.Path],
        output_dir: str | PathLike[str] | pathlib.Path,
        keys: list[str] | str | None = None,
        metric: Literal['duration', 'count'] = 'duration',
        align: Literal['union', 'intersection'] = 'union',
    ) -> str:
        """
        Create an N-way comparison report from timer-frame JSON files.

        Parameters
        ----------
        comparison_map : Mapping[str, SimulationStats | str | os.PathLike[str] | pathlib.Path]
            Mapping from config names to either ``SimulationStats`` instances or
            ``timer_frames.json`` file paths.
        output_dir : str | os.PathLike[str] | pathlib.Path
            Directory used to store generated report artifacts.
        keys : list[str] | str | None, optional
            Timer keys to compare. Keys may be exact timer names or alias keys.
            If ``None``, default key timers are used.
        metric : str, optional
            Comparison metric. Typically ``'duration'`` or ``'count'``.
        align : str, optional
            Frame alignment mode, usually ``'union'`` or ``'intersection'``.

        Returns
        -------
        str
            Markdown text of the generated N-way report.
        """
        if not comparison_map:
            raise ValueError('comparison_map must not be empty')

        stats_map: dict[str, SimulationStats] = {}
        for name, value in comparison_map.items():
            if isinstance(value, SimulationStats):
                stats_map[name] = value
            elif isinstance(value, (str, pathlib.Path, PathLike)):
                stats_map[name] = SimulationStats.load_timer_frames_json(value)
            else:
                raise TypeError(
                    f"Unsupported comparison input type for '{name}': "
                    f'{type(value).__name__}'
                )

        anchor = next(iter(stats_map.values()))
        return anchor._compare_many_report(
            stats_map=stats_map,
            output_dir=output_dir,
            keys=keys,
            metric=metric,
            align=align,
        )

    def collect(self) -> SimulationStats:
        """
        Record timer statistics for the current frame.

        Returns
        -------
        SimulationStats
            ``self`` for optional method chaining.
        """
        import uipc
        timer_data = uipc.Timer.report_as_json()
        self._frames.append(timer_data)
        return self

    @property
    def num_frames(self) -> int:
        """Number of frames collected so far."""
        return len(self._frames)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _find_node(node: dict[str, Any], name: str) -> dict[str, Any] | None:
        """Depth-first search for a node with the given ``name``."""
        if node.get('name') == name:
            return node
        for child in node.get('children', []):
            result = SimulationStats._find_node(child, name)
            if result is not None:
                return result
        return None

    @staticmethod
    def _aggregate_metric_for_name(
        node: dict[str, Any], name: str, metric: Literal['duration', 'count']
    ) -> tuple[bool, float]:
        """Aggregate one metric across all nodes with the same name."""
        found = False
        total = 0.0
        if node.get('name') == name:
            found = True
            total += float(node.get(metric, 0))
        for child in node.get('children', []):
            child_found, child_total = SimulationStats._aggregate_metric_for_name(
                child, name, metric
            )
            found = found or child_found
            total += child_total
        return found, total

    @staticmethod
    def _collect_names(node: dict[str, Any], names: set[str] | None = None) -> set[str]:
        """Recursively collect every timer name present in *node*."""
        if names is None:
            names = set()
        n = node.get('name')
        if n:
            names.add(n)
        for child in node.get('children', []):
            SimulationStats._collect_names(child, names)
        return names

    @property
    def all_timer_names(self) -> set[str]:
        """Return all timer names found across all collected frames."""
        names: set[str] = set()
        for frame_data in self._frames:
            self._collect_names(frame_data, names)
        return names

    def _resolve_keys(
        self, keys: list[str], aliases: Mapping[str, str] | None = None
    ) -> tuple[list[str], dict[str, str]]:
        """Resolve short alias keys to actual timer names found in data.

        Each key is checked in order:

        1. If *key* matches an actual timer name exactly, use it as-is.
        2. If *key* is in *aliases* (or :attr:`DEFAULT_ALIASES`), compile
           the regex pattern and search all known timer names for a match.
        3. Otherwise the key is kept as-is (will be filtered later).

        Parameters
        ----------
        keys : list[str]
            Key strings (short aliases or exact timer names).
        aliases : dict[str, str] | None, optional
            Extra alias-to-regex mapping merged into ``DEFAULT_ALIASES``.

        Returns
        -------
        tuple[list[str], dict[str, str]]
            Resolved timer names and their display-name mapping.
        """
        merged = dict(self.DEFAULT_ALIASES)
        if aliases:
            merged.update(aliases)

        # Collect all timer names from data
        available = set()
        for frame_data in self._frames:
            self._collect_names(frame_data, available)

        resolved = []
        display_map = {}
        for key in keys:
            # 1. Exact match
            if key in available:
                resolved.append(key)
                display_map[key] = key
                continue
            # 2. Alias lookup
            pattern = merged.get(key)
            if pattern:
                regex = re.compile(pattern)
                match = next((n for n in sorted(available) if regex.search(n)), None)
                if match:
                    display = self.ALIAS_DISPLAY.get(key, key)
                    resolved.append(match)
                    display_map[match] = display
                    continue
            # 3. Keep as-is (may be filtered out later)
            resolved.append(key)
            display_map[key] = key

        return resolved, display_map

    @staticmethod
    def _auto_time_unit(values: np.ndarray) -> tuple[np.ndarray, str]:
        """
        Choose the most readable time unit for plotting.

        Parameters
        ----------
        values : numpy.ndarray
            Duration values in seconds.

        Returns
        -------
        tuple[numpy.ndarray, str]
            Scaled values and axis label, for example ``('Time (ms)')``.
        """
        if len(values) == 0:
            return values, 'Time (s)'
        max_val = np.max(np.abs(values))
        if max_val == 0:
            return values, 'Time (s)'
        if max_val < 1e-3:
            return values * 1e6, 'Time (\u00b5s)'
        if max_val < 1.0:
            return values * 1e3, 'Time (ms)'
        return values, 'Time (s)'

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_values(
        self, key: str, metric: Literal['duration', 'count'] = 'duration'
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Return per-frame values for a named timer.

        Parameters
        ----------
        key : str
            Timer name to search for.
        metric : str, optional
            ``'duration'`` (seconds) or ``'count'``.

        Returns
        -------
        tuple[numpy.ndarray, numpy.ndarray]
            Frame indices and values. Values are aggregated across all nodes
            with the same timer name in each frame. Missing frames are omitted.

        Raises
        ------
        ValueError
            If ``metric`` is neither ``'duration'`` nor ``'count'``.
        """
        if metric not in ('duration', 'count'):
            raise ValueError(f"metric must be 'duration' or 'count', got '{metric}'")
        frames = []
        values = []
        for i, timer_data in enumerate(self._frames):
            found, total = self._aggregate_metric_for_name(timer_data, key, metric)
            if found:
                frames.append(i)
                values.append(total)
        return np.array(frames, dtype=int), np.array(values, dtype=float)

    @staticmethod
    def _safe_name(name: Any) -> str:
        """Convert arbitrary timer/config names to filesystem-safe tokens."""
        return re.sub(r'[^0-9A-Za-z._-]+', '_', str(name)).strip('_') or 'unnamed'

    def _compare_many(
        self,
        stats_map: Mapping[str, SimulationStats],
        keys: list[str] | str | None = None,
        metric: Literal['duration', 'count'] = 'duration',
        align: Literal['union', 'intersection'] = 'union',
    ) -> dict[str, Any]:
        """
        Align and compare multiple configurations frame by frame.

        Parameters
        ----------
        stats_map : Mapping[str, SimulationStats]
            Config name to stats object mapping.
        keys : list[str] | str | None, optional
            Timer names to compare. If ``None``, all discovered timers are used.
        metric : str, optional
            ``'duration'`` or ``'count'``.
        align : str, optional
            ``'union'`` fills missing frame values with 0, ``'intersection'``
            keeps only common frames.

        Returns
        -------
        dict[str, object]
            Aligned per-timer arrays and summary means for all configs.
        """
        if metric not in ('duration', 'count'):
            raise ValueError(f"metric must be 'duration' or 'count', got '{metric}'")
        if align not in ('union', 'intersection'):
            raise ValueError(f"align must be 'union' or 'intersection', got '{align}'")
        if not hasattr(stats_map, 'items'):
            raise TypeError('stats_map must be a mapping {config_name: SimulationStats}')
        if len(stats_map) == 0:
            raise ValueError('stats_map must not be empty')

        configs = list(stats_map.keys())
        for cfg, stats in stats_map.items():
            if not hasattr(stats, 'get_values'):
                raise TypeError(f"stats_map['{cfg}'] must provide get_values(key, metric)")

        if keys is None:
            all_names = set()
            for stats in stats_map.values():
                all_names |= stats.all_timer_names
            keys = sorted(all_names)
        elif isinstance(keys, str):
            keys = [keys]

        compared = {
            'metric': metric,
            'align': align,
            'configs': configs,
            'timers': {},
        }

        for key in keys:
            per_cfg_frames = {}
            per_cfg_values = {}
            frame_sets = []
            for cfg, stats in stats_map.items():
                f, v = stats.get_values(key, metric=metric)
                f = np.array(f, dtype=int)
                v = np.array(v, dtype=float)
                per_cfg_frames[cfg] = f
                per_cfg_values[cfg] = v
                frame_sets.append(set(int(x) for x in f.tolist()))

            if len(frame_sets) == 0:
                aligned_frames = []
            elif align == 'union':
                aligned_frames = sorted(set().union(*frame_sets))
            else:
                aligned_frames = sorted(set.intersection(*frame_sets)) if frame_sets else []

            frames = np.array(aligned_frames, dtype=int)
            values = {}
            means = {}
            for cfg in configs:
                fmap = {
                    int(ff): float(vv)
                    for ff, vv in zip(per_cfg_frames[cfg].tolist(), per_cfg_values[cfg].tolist())
                }
                arr = np.array([fmap.get(int(ff), 0.0) for ff in frames], dtype=float)
                values[cfg] = arr
                means[cfg] = float(arr.mean()) if len(arr) > 0 else 0.0

            compared['timers'][key] = {
                'frames': frames,
                'values': values,
                'means': means,
            }

        return compared

    def _compare_many_report(
        self,
        stats_map: Mapping[str, SimulationStats],
        output_dir: str | PathLike[str] | pathlib.Path,
        keys: list[str] | str | None = None,
        metric: Literal['duration', 'count'] = 'duration',
        align: Literal['union', 'intersection'] = 'union',
    ) -> str:
        """Generate N-way comparison report with charts.

        Charts include:
        - one heatmap per config
        - grouped bar chart (timer means, config colors)
        - one per-timer frame chart (duration + count in one panel)
        """
        compared = self._compare_many(
            stats_map, keys=keys, metric=metric, align=align
        )

        # Match summary_report defaults: only key panels users usually care about.
        display_map = {}
        if keys is None:
            keys = ['newton_iteration', 'global_linear_system',
                    'line_search', 'dcd', 'spmv']
            if self._frames:
                keys, display_map = self._resolve_keys(keys)
            compared = self._compare_many(
                stats_map, keys=keys, metric=metric, align=align
            )
        elif self._frames:
            keys, display_map = self._resolve_keys(keys)
            compared = self._compare_many(
                stats_map, keys=keys, metric=metric, align=align
            )

        out = pathlib.Path(output_dir)
        plots_dir = out / 'plots'
        out.mkdir(parents=True, exist_ok=True)
        plots_dir.mkdir(parents=True, exist_ok=True)

        configs = compared['configs']
        timers = sorted(compared['timers'].keys())
        cmap = plt.get_cmap('tab10')
        color_map = {cfg: cmap(i % 10) for i, cfg in enumerate(configs)}

        # 1) Heatmaps: one per config
        heatmap_paths = {}
        for cfg, stats in stats_map.items():
            heatmap_name = f'heatmap_{self._safe_name(cfg)}.svg'
            heatmap_path = plots_dir / heatmap_name
            if getattr(stats, 'num_frames', 0) > 0:
                stats.profiler_heatmap(output_path=str(heatmap_path))
                plt.close('all')
                heatmap_paths[cfg] = f'plots/{heatmap_name}'

        # 2) Grouped bar chart for per-timer means
        grouped_bar_rel = None
        if len(timers) > 0:
            means_matrix = []
            for cfg in configs:
                means_matrix.append([
                    compared['timers'][timer]['means'][cfg] for timer in timers
                ])
            means_matrix = np.array(means_matrix, dtype=float)  # [n_cfg, n_timer]

            if metric == 'duration':
                scaled, ylabel = self._auto_time_unit(means_matrix.flatten())
                if len(scaled) > 0 and np.max(np.abs(means_matrix)) > 0:
                    scale = float(np.max(np.abs(scaled)) / np.max(np.abs(means_matrix)))
                else:
                    scale = 1.0
            else:
                scale = 1.0
                ylabel = 'Count'

            fig, ax = plt.subplots(figsize=(max(10, len(timers) * 0.45), 5))
            x = np.arange(len(timers), dtype=float)
            total_width = 0.82
            bar_w = total_width / max(len(configs), 1)
            for i, cfg in enumerate(configs):
                offset = (i - (len(configs) - 1) / 2.0) * bar_w
                y = means_matrix[i] * scale
                ax.bar(x + offset, y, width=bar_w, label=cfg, color=color_map[cfg], alpha=0.9)
            ax.set_xticks(x)
            ax.set_xticklabels(timers, rotation=35, ha='right')
            ax.set_ylabel(ylabel)
            ax.set_title(f'Per-Timer Mean ({metric}) by Config')
            ax.grid(True, axis='y', alpha=0.3)
            ax.legend()
            plt.tight_layout()
            grouped_bar_name = 'grouped_bar.svg'
            plt.savefig(plots_dir / grouped_bar_name, dpi=150, bbox_inches='tight')
            plt.close(fig)
            grouped_bar_rel = f'plots/{grouped_bar_name}'

        # 2.5) Total time per frame chart (same spirit as summary_report).
        total_time_rel = None
        if metric == 'duration':
            fig, ax = plt.subplots(figsize=(9, 4))
            has_total = False
            ylabel = 'Time (s)'
            for cfg in configs:
                stats = stats_map[cfg]

                # Match summary_report logic: GlobalTimer duration may be zero,
                # so use the first non-zero child (typically "Pipeline").
                root_name = ''
                for fd in stats._frames:
                    children = fd.get('children', [])
                    if children and children[0].get('duration', 0) > 0:
                        root_name = children[0].get('name', '')
                        break
                    if fd.get('duration', 0) > 0:
                        root_name = fd.get('name', '')
                        break
                frames, total_values = (
                    stats.get_values(root_name, 'duration') if root_name
                    else (np.array([], dtype=int), np.array([], dtype=float))
                )
                if len(frames) == 0 or len(total_values) == 0:
                    continue
                has_total = True
                v_plot, ylabel = self._auto_time_unit(total_values)
                ax.plot(
                    frames, v_plot, label=cfg, color=color_map[cfg],
                    marker='o', markersize=2.5, linewidth=1.2
                )
            if has_total:
                ax.xaxis.set_major_locator(MaxNLocator(integer=True))
                ax.set_xlabel('Frame')
                ax.set_ylabel(ylabel)
                ax.set_title('Total Time Per Frame')
                ax.grid(True, alpha=0.3)
                ax.legend()
                plt.tight_layout()
                total_name = 'total_time_per_frame.svg'
                plt.savefig(plots_dir / total_name, dpi=150, bbox_inches='tight')
                total_time_rel = f'plots/{total_name}'
            plt.close(fig)

        # 3) One per-timer panel (duration + count in one chart when metric=duration)
        compared_count = None
        if metric == 'duration':
            compared_count = self._compare_many(
                stats_map, keys=keys, metric='count', align=align
            )

        line_paths = {}
        count_means = {}
        for timer in timers:
            item = compared['timers'][timer]
            frames = item['frames']
            if len(frames) == 0:
                continue

            fig, ax_dur = plt.subplots(figsize=(9, 4))
            if metric == 'duration':
                all_series = np.concatenate([item['values'][cfg] for cfg in configs]) \
                    if len(configs) > 0 else np.array([], dtype=float)
                _, ylabel = self._auto_time_unit(all_series)
                max_val = np.max(np.abs(all_series)) if len(all_series) > 0 else 0.0
                if max_val < 1e-3 and max_val > 0:
                    scale = 1e6
                elif max_val < 1.0 and max_val > 0:
                    scale = 1e3
                else:
                    scale = 1.0

                for cfg in configs:
                    ax_dur.plot(
                        frames,
                        item['values'][cfg] * scale,
                        label=f'{cfg} (time)',
                        color=color_map[cfg],
                        marker='o',
                        markersize=2.5,
                        linewidth=1.2,
                    )

                ax_cnt = ax_dur.twinx()
                count_item = compared_count['timers'].get(timer) if compared_count else None
                if count_item is not None and len(count_item['frames']) > 0:
                    bar_total_w = 0.72
                    bar_w = bar_total_w / max(len(configs), 1)
                    for cfg in configs:
                        cfg_i = configs.index(cfg)
                        offset = (cfg_i - (len(configs) - 1) / 2.0) * bar_w
                        ax_cnt.plot(
                            [], [],
                            label=f'{cfg} (count)',
                            color=color_map[cfg]
                        )
                        ax_cnt.bar(
                            count_item['frames'] + offset,
                            count_item['values'][cfg],
                            width=bar_w,
                            color=color_map[cfg],
                            alpha=0.28,
                        )
                    count_means[timer] = count_item['means']
                ax_cnt.set_ylabel('Count')
                ax_cnt.tick_params(axis='y')

                lines_d, labels_d = ax_dur.get_legend_handles_labels()
                lines_c, labels_c = ax_cnt.get_legend_handles_labels()
                if lines_d or lines_c:
                    ax_dur.legend(lines_d + lines_c, labels_d + labels_c,
                                  loc='upper left', fontsize=8, ncol=2)
                ax_dur.set_ylabel(ylabel)
                ax_dur.set_title(f'{display_map.get(timer, timer)} (time + count)')
            else:
                for cfg in configs:
                    ax_dur.plot(
                        frames,
                        item['values'][cfg],
                        label=cfg,
                        color=color_map[cfg],
                        marker='o',
                        markersize=2.5,
                        linewidth=1.2,
                    )
                ax_dur.set_ylabel('Count')
                ax_dur.set_title(f'{display_map.get(timer, timer)} (count)')
                ax_dur.legend(loc='upper left')

            ax_dur.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax_dur.set_xlabel('Frame')
            ax_dur.grid(True, alpha=0.3)
            plt.tight_layout()
            line_name = f'line_{self._safe_name(timer)}.svg'
            plt.savefig(plots_dir / line_name, dpi=150, bbox_inches='tight')
            plt.close(fig)
            line_paths[timer] = f'plots/{line_name}'

        # Markdown report
        lines = ['# N-Config Comparison Report', '']
        lines.append(f'- Metric: `{metric}`')
        lines.append(f'- Alignment: `{align}`')
        lines.append('')
        lines.append('## Heatmaps')
        lines.append('')
        for cfg in configs:
            if cfg in heatmap_paths:
                lines.append(f'### {cfg}')
                lines.append('')
                lines.append(f'![]({heatmap_paths[cfg]})')
                lines.append('')

        if grouped_bar_rel is not None:
            lines.append('## Grouped Bar Chart')
            lines.append('')
            lines.append(f'![]({grouped_bar_rel})')
            lines.append('')

        if total_time_rel is not None:
            lines.append('## Total Time Per Frame')
            lines.append('')
            lines.append(f'![]({total_time_rel})')
            lines.append('')

        lines.append('## Framewise Time + Count Charts')
        lines.append('')
        for timer in timers:
            if timer in line_paths:
                lines.append(f'### {display_map.get(timer, timer)}')
                lines.append('')
                lines.append(f'![]({line_paths[timer]})')
                lines.append('')

        lines.append('## Per-Timer Means')
        lines.append('')
        header = '| Timer | ' + ' | '.join(configs) + ' |'
        sep = '|---|' + '|'.join(['---:'] * len(configs)) + '|'
        lines.append(header)
        lines.append(sep)
        for timer in timers:
            means = compared['timers'][timer]['means']
            row = '| ' + timer + ' | ' + ' | '.join(
                f"{means[cfg]:.6f}" for cfg in configs
            ) + ' |'
            lines.append(row)
        lines.append('')

        if count_means:
            lines.append('## Per-Timer Mean Counts')
            lines.append('')
            lines.append(header)
            lines.append(sep)
            for timer in timers:
                if timer not in count_means:
                    continue
                means = count_means[timer]
                row = '| ' + display_map.get(timer, timer) + ' | ' + ' | '.join(
                    f"{means[cfg]:.6f}" for cfg in configs
                ) + ' |'
                lines.append(row)
            lines.append('')

        md_text = '\n'.join(lines)
        (out / 'comparison.md').write_text(md_text, encoding='utf-8')

        json_data = {
            'metric': metric,
            'align': align,
            'configs': configs,
            'timers': {},
        }
        for timer in timers:
            item = compared['timers'][timer]
            json_data['timers'][timer] = {
                'frames': item['frames'].tolist(),
                'values': {
                    cfg: item['values'][cfg].tolist()
                    for cfg in configs
                },
                'means': item['means'],
            }
        (out / 'comparison.json').write_text(
            json.dumps(json_data, indent=2), encoding='utf-8'
        )

        return md_text

    def plot(
        self,
        keys: str | list[str],
        metric: Literal['duration', 'count'] = 'duration',
        kind: Literal['line', 'bar'] = 'line',
        title: str | None = None,
        output_path: str | PathLike[str] | pathlib.Path | None = None,
    ) -> Any:
        """
        Plot per-frame timer values as a line or bar chart.

        Parameters
        ----------
        keys : str | list[str]
            Timer name(s) to plot.
        metric : str, optional
            ``'duration'`` (seconds) or ``'count'``.
        kind : str, optional
            ``'line'`` for curves or ``'bar'`` for grouped bars.
        title : str | None, optional
            Chart title. Auto-generated when omitted.
        output_path : str | os.PathLike[str] | None, optional
            Save path for the figure. When omitted, the plot is shown.

        Returns
        -------
        matplotlib.figure.Figure
            The created Matplotlib figure.

        Raises
        ------
        ValueError
            If ``metric`` or ``kind`` is invalid.
        """
        if metric not in ('duration', 'count'):
            raise ValueError(f"metric must be 'duration' or 'count', got '{metric}'")
        if kind not in ('line', 'bar'):
            raise ValueError(f"kind must be 'line' or 'bar', got '{kind}'")
        if isinstance(keys, str):
            keys = [keys]

        num_keys = len(keys)

        fig, ax = plt.subplots(figsize=(10, 5))
        has_data = False
        ylabel = 'Count'
        scale = 1.0

        # For duration: determine a single consistent unit across all keys
        if metric == 'duration':
            all_vals = []
            for k in keys:
                _, v = self.get_values(k, 'duration')
                if len(v) > 0:
                    all_vals.append(v)
            if all_vals:
                combined = np.concatenate(all_vals)
                _, ylabel = self._auto_time_unit(combined)
                max_val = np.max(np.abs(combined))
                if max_val > 0:
                    if max_val < 1e-3:
                        scale = 1e6
                    elif max_val < 1.0:
                        scale = 1e3

        for idx, key in enumerate(keys):
            frames, values = self.get_values(key, metric)
            if len(frames) == 0:
                Logger.warn(f"No data found for timer '{key}'")
                continue
            has_data = True
            if metric == 'duration':
                values = values * scale
            if kind == 'bar':
                if num_keys > 1:
                    total_width = 0.8
                    bar_width = total_width / num_keys
                    offset = (idx - (num_keys - 1) / 2.0) * bar_width
                    positions = frames + offset
                else:
                    bar_width = 0.8
                    positions = frames
                ax.bar(positions, values, label=key, alpha=0.7, width=bar_width)
            else:
                ax.plot(frames, values, label=key, marker='o', markersize=3)

        ax.set_xlabel('Frame')
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_ylabel(ylabel)
        ax.set_title(title or f'{", ".join(keys)} \u2014 {metric} per frame')
        if has_data:
            ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            Logger.info(f'Plot saved to {output_path}')
        else:
            plt.show()
        return fig

    def to_markdown(
        self,
        keys: list[str] | str | None = None,
        metric: Literal['duration', 'count'] = 'duration',
        display_map: Mapping[str, str] | None = None,
    ) -> str:
        """
        Export per-frame statistics as a Markdown table.

        Parameters
        ----------
        keys : list[str] | str | None, optional
            Timer name(s) used as table columns. If ``None``, all timers found
            in data are included.
        metric : str, optional
            ``'duration'`` (seconds) or ``'count'``.
        display_map : dict[str, str] | None, optional
            Optional timer-name to display-label mapping.

        Returns
        -------
        str
            Markdown-formatted table text.
        """
        if keys is None:
            found = set()
            root_names = {td.get('name', '') for td in self._frames}
            for timer_data in self._frames:
                SimulationStats._collect_names(timer_data, found)
            # Remove the root node names (usually 'GlobalTimer')
            keys = sorted(found - root_names)
        elif isinstance(keys, str):
            keys = [keys]
        if display_map is None:
            display_map = {}

        labels = [display_map.get(k, k) for k in keys]
        col_w = max((len(l) for l in labels), default=7)
        col_w = max(col_w, 7)

        header = '| Frame | ' + ' | '.join(l.ljust(col_w) for l in labels) + ' |'
        sep = '|-------|' + '|'.join('-' * (col_w + 2) for _ in keys) + '|'
        rows = [header, sep]

        for i, timer_data in enumerate(self._frames):
            cells = []
            for key in keys:
                found, val = self._aggregate_metric_for_name(timer_data, key, metric)
                if found:
                    text = f'{val:.4f}s' if metric == 'duration' else str(int(val))
                else:
                    text = 'N/A'
                cells.append(text.ljust(col_w))
            rows.append('| ' + str(i).ljust(5) + ' | ' + ' | '.join(cells) + ' |')

        return '\n'.join(rows)

    # ------------------------------------------------------------------
    # Profiler heatmap (sunburst)
    # ------------------------------------------------------------------

    @staticmethod
    def _draw_profiler_heatmap(
        ax: Any,
        timer_data: dict[str, Any],
        max_depth: int = 999,
        include_other: bool = True,
    ) -> bool:
        """Draw a hierarchical sunburst chart onto *ax*."""
        if not timer_data.get('children'):
            return False
        level_data = {}
        node_angles = {}
        legend_items = []
        children = timer_data['children']
        if not children or not children[0]:
            return False
        total_duration = children[0].get('duration', 0)
        frame_count = children[0].get('count', 0)
        if total_duration == 0:
            return False

        def collect_level(node: dict[str, Any], depth: int = 0, parent_name: str = 'root') -> None:
            if depth > max_depth:
                return
            name = node.get('name', 'Unknown')
            level_data.setdefault(depth, {})[name] = {
                'name': name,
                'full_name': f'{parent_name} -> {name}' if depth > 0 else name,
                'duration': node.get('duration', 0), 'percentage': 0,
                'parent': parent_name if depth > 0 else None,
                'count': node.get('count', 0),
            }
            for child in node.get('children', []):
                collect_level(child, depth + 1, name)

        collect_level(timer_data)
        if not level_data:
            return False
        ring_width = 0.3
        max_depth_seen = max(level_data.keys())

        if include_other:
            for depth in sorted(level_data.keys()):
                if depth == max_depth_seen:
                    continue
                for parent_name, parent_data in level_data[depth].items():
                    if 'other' in parent_name:
                        continue
                    child_dur = sum(
                        cd['duration'] for cd in level_data.get(depth + 1, {}).values()
                        if cd['parent'] == parent_name and 'other' not in parent_data['name'])
                    child_count = sum(
                        1 for cd in level_data.get(depth + 1, {}).values()
                        if cd['parent'] == parent_name and 'other' not in parent_data['name'])
                    if child_count == 0:
                        continue
                    remaining = parent_data['duration'] - child_dur
                    if remaining > 1e-6:
                        level_data.setdefault(depth + 1, {})[f'{parent_name}_other'] = {
                            'name': 'Other',
                            'full_name': f'{parent_name} -> Other',
                            'duration': remaining,
                            'percentage': (remaining / total_duration) * 100,
                            'parent': parent_name, 'count': 1, 'is_other': True,
                        }

        for depth, nodes in level_data.items():
            if depth == 0:
                continue
            inner_radius = 0.25 + (max_depth_seen - depth) * ring_width
            if depth == 1:
                sorted_items = list(nodes.values())
                sizes = [item['duration'] for item in sorted_items]
                for item in sorted_items:
                    item['percentage'] = (item['duration'] / total_duration) * 100
                wedges, _ = ax.pie(
                    sizes, labels=nodes, radius=inner_radius + ring_width,
                    startangle=90, counterclock=True, autopct=None,
                    wedgeprops={'width': ring_width, 'edgecolor': 'white',
                                'linewidth': 0.5})
                node_angles[1] = {}
                for idx, (wedge, item) in enumerate(zip(wedges, sorted_items)):
                    node_angles[1][item['name']] = (wedge.theta1, wedge.theta2)
                    if not item.get('is_other', False):
                        ang = np.deg2rad((wedge.theta2 + wedge.theta1) / 2)
                        radius = inner_radius + ring_width / 2
                        x, y = radius * np.cos(ang), radius * np.sin(ang)
                    ax.text(x, y, f'{item["percentage"]:.2f}%', ha='center',
                            va='center', fontsize=8, color='white', fontweight='bold')
                    legend_items.append((item['full_name'], item['percentage'],
                                        wedge.get_facecolor()))
            else:
                parent_order = list(level_data.get(depth - 1, {}).keys())
                groups_ordered = {p: [] for p in parent_order}
                for item in nodes.values():
                    if item['parent'] in groups_ordered:
                        groups_ordered[item['parent']].append(item)
                sorted_items = []
                for p in parent_order:
                    sorted_items.extend(groups_ordered.get(p, []))
                parent_groups = {}
                for item in sorted_items:
                    parent_groups.setdefault(item['parent'], []).append(item)
                node_angles[depth] = {}
                for parent, group_children in parent_groups.items():
                    if parent not in level_data.get(depth - 1, {}):
                        continue
                    for item in group_children:
                        item['percentage'] = (item['duration'] / total_duration) * 100
                    child_sizes = [item['duration'] for item in group_children]
                    if parent in node_angles.get(depth - 1, {}):
                        p_start, p_end = node_angles[depth - 1][parent]
                        p_angle = p_end - p_start
                        wedges, _ = ax.pie(
                            child_sizes, labels=None,
                            radius=inner_radius + ring_width, startangle=90,
                            counterclock=True, autopct=None,
                            wedgeprops={'width': ring_width,
                                        'edgecolor': 'white', 'linewidth': 0.5})
                        total_child = sum(child_sizes)
                        cur_angle = p_start
                        for idx, (wedge, cs) in enumerate(zip(wedges, child_sizes)):
                            t1 = cur_angle
                            t2 = cur_angle + p_angle * (cs / total_child)
                            cur_angle = t2
                            wedge.set_theta1(t1)
                            wedge.set_theta2(t2)
                            if idx < len(group_children):
                                group_children[idx]['mapped_angles'] = (t1, t2)
                    else:
                        wedges, _ = ax.pie(
                            child_sizes, labels=None,
                            radius=inner_radius + ring_width, startangle=0,
                            counterclock=True, autopct=None,
                            wedgeprops={'width': ring_width,
                                        'edgecolor': 'white', 'linewidth': 0.5})
                    for idx, (wedge, item) in enumerate(zip(wedges, group_children)):
                        node_angles[depth][item['name']] = (wedge.theta1, wedge.theta2)
                        angle_size = wedge.theta2 - wedge.theta1
                        if not item.get('is_other', False):
                            ang = np.deg2rad((wedge.theta2 + wedge.theta1) / 2)
                            radius = inner_radius + ring_width / 2
                            x, y = radius * np.cos(ang), radius * np.sin(ang)
                            if angle_size > 5:
                                ax.text(x, y, f'{item["percentage"]:.2f}%',
                                        ha='center', va='center', fontsize=8,
                                        color='white', fontweight='bold')
                            legend_items.append((item['full_name'],
                                                 item['percentage'],
                                                 wedge.get_facecolor()))

        ax.text(0, 0, f'{total_duration:.3f}s', ha='center', va='center',
                fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray',
                          alpha=0.95))
        ax.set_aspect('equal')
        legend_items.sort(key=lambda x: x[1], reverse=True)
        patches = [mpatches.Patch(label=f'{n} ({p:.2f}%)', color=c)
                   for n, p, c in legend_items]
        ax.legend(handles=patches, loc='center left',
                  bbox_to_anchor=(1.05, 0.5), fontsize=7, frameon=True,
                  fancybox=True, shadow=True,
                  title=f'Total: {total_duration:.3f}s / {frame_count} frames')
        return True

    @staticmethod
    def _merge_timer_trees(frames: list[dict[str, Any]]) -> dict[str, Any]:
        """
        Merge multiple per-frame timer trees by summing durations and counts.

        Parameters
        ----------
        frames : list[dict[str, Any]]
            Timer trees, one entry per frame.

        Returns
        -------
        dict[str, Any]
            One merged timer tree.
        """
        if not frames:
            return {}
        if len(frames) == 1:
            return frames[0]

        def _merge_node(target: dict[str, Any], source: dict[str, Any]) -> None:
            target['duration'] = target.get('duration', 0) + source.get('duration', 0)
            target['count'] = target.get('count', 0) + source.get('count', 0)
            target_children = {c['name']: c for c in target.get('children', [])}
            for src_child in source.get('children', []):
                name = src_child['name']
                if name in target_children:
                    _merge_node(target_children[name], src_child)
                else:
                    target_children[name] = copy.deepcopy(src_child)
            target['children'] = list(target_children.values())

        merged = copy.deepcopy(frames[0])
        for frame in frames[1:]:
            _merge_node(merged, frame)
        return merged

    def profiler_heatmap(
        self,
        frame_index: int | None = None,
        output_path: str | PathLike[str] | pathlib.Path | None = None,
        max_depth: int = 999,
        include_other: bool = True,
    ) -> Any | None:
        """Create a hierarchical sunburst chart from collected frames.

        When multiple frames have been collected and *frame_index* is not
        specified, all frames are merged (durations and counts summed)
        into a single aggregate heatmap.  When only one frame exists, or
        a specific *frame_index* is given, that single frame is used.

        Parameters
        ----------
        frame_index : int | None, optional
            Frame index to visualize. If omitted, all frames are merged when
            more than one frame exists.
        output_path : str | os.PathLike[str] | None, optional
            Save path for the figure. When omitted, the figure is shown.
        max_depth : int, optional
            Maximum hierarchy depth to include.
        include_other : bool, optional
            Whether to include ``Other`` slices for unaccounted time.

        Returns
        -------
        matplotlib.figure.Figure | None
            The rendered figure, or ``None`` when data is unavailable.
        """
        if not self._frames:
            Logger.warn('No frames collected; cannot create profiler heatmap')
            return None

        if frame_index is not None:
            timer_data = self._frames[frame_index]
        elif len(self._frames) > 1:
            timer_data = self._merge_timer_trees(self._frames)
        else:
            timer_data = self._frames[0]

        fig, ax = plt.subplots(figsize=(15, 10))
        drawn = self._draw_profiler_heatmap(ax, timer_data, max_depth,
                                            include_other)
        if not drawn:
            ax.text(0.5, 0.5, 'No profiling data', ha='center', va='center',
                    transform=ax.transAxes, fontsize=12, color='gray')

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight',
                        pad_inches=0.5)
            Logger.info(f'Profiler heatmap saved to {output_path}')
        else:
            plt.show()
        return fig

    # ------------------------------------------------------------------
    # System dependency graph
    # ------------------------------------------------------------------

    _SYSTEM_NAME_PREFIXES = [
        'class uipc::backend::cuda::',
        'class uipc::backend::none::',
        'class uipc::backend::',
    ]

    @staticmethod
    def _strip_system_name(name: str) -> str:
        """Remove common C++ namespace prefixes from a system name."""
        for prefix in SimulationStats._SYSTEM_NAME_PREFIXES:
            if name.startswith(prefix):
                return name[len(prefix):]
        return name

    @staticmethod
    def _remove_overlaps(
        pos: dict[Any, tuple[float, float]],
        sizes: dict[Any, tuple[float, float]],
        iterations: int = 80,
        pad: float = 1.15,
    ) -> None:
        """
        Push nodes apart until no bounding boxes overlap.

        Parameters
        ----------
        pos : dict[Any, tuple[float, float]]
            Mutable node position map.
        sizes : dict[Any, tuple[float, float]]
            Node half-extents.
        iterations : int, optional
            Maximum adjustment iterations.
        pad : float, optional
            Multiplicative padding factor for extents.
        """
        node_list = list(pos.keys())
        for _ in range(iterations):
            moved = False
            for i, node1 in enumerate(node_list):
                x1, y1 = pos[node1]
                hw1 = sizes[node1][0] * pad
                hh1 = sizes[node1][1] * pad
                for node2 in node_list[i + 1:]:
                    x2, y2 = pos[node2]
                    hw2 = sizes[node2][0] * pad
                    hh2 = sizes[node2][1] * pad
                    overlap_x = (hw1 + hw2) - abs(x1 - x2)
                    overlap_y = (hh1 + hh2) - abs(y1 - y2)
                    if overlap_x > 0 and overlap_y > 0:
                        # Push apart along the axis of least overlap
                        if overlap_x < overlap_y:
                            shift = overlap_x / 2 + 0.001
                            if x1 < x2:
                                pos[node1] = (x1 - shift, y1)
                                pos[node2] = (x2 + shift, y2)
                            else:
                                pos[node1] = (x1 + shift, y1)
                                pos[node2] = (x2 - shift, y2)
                        else:
                            shift = overlap_y / 2 + 0.001
                            if y1 < y2:
                                pos[node1] = (x1, y1 - shift)
                                pos[node2] = (x2, y2 + shift)
                            else:
                                pos[node1] = (x1, y1 + shift)
                                pos[node2] = (x2, y2 - shift)
                        moved = True
            if not moved:
                break

    @staticmethod
    def _draw_system_dependency_graph(
        systems_json_path: str | PathLike[str] | pathlib.Path,
        output_path: str | PathLike[str] | pathlib.Path | None = None,
    ) -> Any | None:
        """Render a directed dependency graph of backend systems.

        Nodes are drawn as rounded-rectangle label boxes that are
        guaranteed not to overlap.  Edges curve freely to avoid clutter.

        Parameters
        ----------
        systems_json_path : str | os.PathLike[str]
            Path to ``systems.json``.
        output_path : str | os.PathLike[str] | None, optional
            Save path for the figure. When omitted, the figure is shown.

        Returns
        -------
        matplotlib.figure.Figure | None
            The rendered figure, or ``None`` when the graph cannot be built.
        """
        try:
            import networkx as nx
        except ImportError:
            Logger.warn('networkx is required for system dependency graphs; '
                        'install it with: pip install networkx')
            return None

        from matplotlib.patches import FancyBboxPatch
        from matplotlib.lines import Line2D

        p = pathlib.Path(systems_json_path)
        if not p.exists():
            return None

        with open(p, 'r', encoding='utf-8') as f:
            data = json.load(f)

        systems = data.get('sim_systems', [])
        if not systems:
            return None

        strip = SimulationStats._strip_system_name

        G = nx.DiGraph()
        engine_aware_nodes = set()
        for sys_info in systems:
            name = strip(sys_info.get('name', ''))
            if not name:
                continue
            G.add_node(name)
            if sys_info.get('engine_aware', False):
                engine_aware_nodes.add(name)
            for dep in sys_info.get('strong_deps', []):
                G.add_edge(name, strip(dep), style='strong')
            for dep in sys_info.get('weak_deps', []):
                G.add_edge(name, strip(dep), style='weak')

        if len(G.nodes) == 0:
            return None

        # -- Initial layout ---------------------------------------------------
        try:
            pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
        except (ImportError, ValueError):
            pos = nx.kamada_kawai_layout(G, scale=2.0)

        # -- Measure label sizes & remove overlaps ----------------------------
        n_nodes = len(G.nodes)
        font_size = max(6, 9 - n_nodes / 25)

        # Create a temporary figure to measure text extents in points
        tmp_fig, tmp_ax = plt.subplots(figsize=(30, 20))
        tmp_ax.set_xlim(-2, 2)
        tmp_ax.set_ylim(-2, 2)
        renderer = tmp_fig.canvas.get_renderer()

        sizes = {}  # {node: (half_w, half_h)} in data coords
        for node in G.nodes:
            txt = tmp_ax.text(0, 0, node, fontsize=font_size,
                              fontfamily='sans-serif')
            tmp_fig.canvas.draw()
            bb = txt.get_window_extent(renderer=renderer)
            bb_data = bb.transformed(tmp_ax.transData.inverted())
            hw = bb_data.width * 0.7 + 0.015
            hh = bb_data.height * 1.0 + 0.008
            sizes[node] = (hw, hh)
            txt.remove()
        plt.close(tmp_fig)

        # Scale positions so they are spaced proportionally to label sizes
        xs = [pos[n][0] for n in pos]
        ys = [pos[n][1] for n in pos]
        x_range = max(xs) - min(xs) if max(xs) != min(xs) else 1
        y_range = max(ys) - min(ys) if max(ys) != min(ys) else 1
        max_hw = max(s[0] for s in sizes.values())
        max_hh = max(s[1] for s in sizes.values())
        # Scale so nodes have room; x needs more than y (labels are wide)
        scale_x = max_hw * n_nodes * 0.55 / x_range
        scale_y = max_hh * n_nodes * 0.7 / y_range
        cx = (max(xs) + min(xs)) / 2
        cy = (max(ys) + min(ys)) / 2
        pos = {n: ((pos[n][0] - cx) * scale_x,
                    (pos[n][1] - cy) * scale_y) for n in pos}

        # Aggressively push apart any remaining overlaps
        SimulationStats._remove_overlaps(pos, sizes, iterations=300,
                                         pad=1.4)

        # -- Figure -----------------------------------------------------------
        xs = [pos[n][0] for n in pos]
        ys = [pos[n][1] for n in pos]
        margin_x = max_hw * 3
        margin_y = max_hh * 3
        data_w = max(xs) - min(xs) + 2 * margin_x
        data_h = max(ys) - min(ys) + 2 * margin_y
        aspect = data_w / data_h if data_h > 0 else 1.5
        fig_h = max(18, min(36, n_nodes * 0.6))
        fig_w = max(fig_h * aspect, 22)
        fig_w = min(fig_w, 48)
        fig, ax = plt.subplots(figsize=(fig_w, fig_h))

        # -- Edges (freely curved) --------------------------------------------
        strong_edges = [e for e in G.edges
                        if G.edges[e].get('style') == 'strong']
        weak_edges = [e for e in G.edges
                      if G.edges[e].get('style') != 'strong']
        if strong_edges:
            nx.draw_networkx_edges(
                G, pos, ax=ax, edgelist=strong_edges,
                edge_color='#6C8EBF', style='solid', width=1.0,
                arrows=True, arrowsize=12, arrowstyle='-|>',
                connectionstyle='arc3,rad=0.25', alpha=0.55,
                min_source_margin=22, min_target_margin=22)
        if weak_edges:
            nx.draw_networkx_edges(
                G, pos, ax=ax, edgelist=weak_edges,
                edge_color='#AAAAAA', style=(0, (4, 3)), width=0.7,
                arrows=True, arrowsize=9, arrowstyle='-|>',
                connectionstyle='arc3,rad=0.35', alpha=0.4,
                min_source_margin=22, min_target_margin=22)

        # -- Nodes as rounded boxes -------------------------------------------
        renderer = fig.canvas.get_renderer()
        for node in G.nodes:
            x, y = pos[node]
            is_ea = node in engine_aware_nodes
            fc = '#D5E8D4' if is_ea else '#DAE8FC'
            ec = '#82B366' if is_ea else '#6C8EBF'
            lw = 1.8 if is_ea else 1.0

            txt = ax.text(x, y, node, ha='center', va='center',
                          fontsize=font_size, fontfamily='sans-serif',
                          fontweight='bold' if is_ea else 'normal',
                          zorder=4)
            fig.canvas.draw()
            bb = txt.get_window_extent(renderer=renderer)
            bb_data = bb.transformed(ax.transData.inverted())
            pad_x = bb_data.width * 0.18 + 0.002
            pad_y = bb_data.height * 0.3 + 0.002
            box = FancyBboxPatch(
                (bb_data.x0 - pad_x, bb_data.y0 - pad_y),
                bb_data.width + 2 * pad_x,
                bb_data.height + 2 * pad_y,
                boxstyle='round,pad=0.008',
                facecolor=fc, edgecolor=ec, linewidth=lw,
                zorder=3, alpha=0.92)
            ax.add_patch(box)

        # -- Legend -----------------------------------------------------------
        legend_elements = [
            mpatches.Patch(facecolor='#D5E8D4', edgecolor='#82B366',
                           linewidth=1.5, label='Engine-aware system'),
            mpatches.Patch(facecolor='#DAE8FC', edgecolor='#6C8EBF',
                           linewidth=1.0, label='Internal system'),
            Line2D([0], [0], color='#6C8EBF', lw=1.5,
                   label='Strong dependency'),
            Line2D([0], [0], color='#AAAAAA', lw=1.0, linestyle='dashed',
                   label='Weak dependency'),
        ]
        ax.legend(handles=legend_elements, loc='upper left', fontsize=8,
                  frameon=True, fancybox=True, framealpha=0.9,
                  edgecolor='#CCCCCC')
        ax.set_title('Backend System Dependencies', fontsize=13,
                     fontweight='bold', pad=12)
        ax.axis('off')
        ax.margins(0.03)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, bbox_inches='tight', pad_inches=0.3)
            Logger.info(f'System dependency graph saved to {output_path}')
            plt.close(fig)
        else:
            plt.show()
        return fig

    def system_dependency_graph(
        self,
        systems_json: str | PathLike[str] | pathlib.Path,
        output_path: str | PathLike[str] | pathlib.Path | None = None,
    ) -> Any | None:
        """
        Create a standalone system dependency graph figure.

        Parameters
        ----------
        systems_json : str | os.PathLike[str]
            Path to ``systems.json`` or a workspace directory containing it.
        output_path : str | os.PathLike[str] | None, optional
            Save path for the figure.

        Returns
        -------
        matplotlib.figure.Figure | None
            The rendered figure, or ``None`` if ``systems.json`` is missing.
        """
        p = pathlib.Path(systems_json)
        if p.is_dir():
            p = p / 'systems.json'
        if not p.exists():
            Logger.warn(f'systems.json not found at {p}')
            return None

        return self._draw_system_dependency_graph(str(p), output_path)

    # ------------------------------------------------------------------
    # Summary report
    # ------------------------------------------------------------------

    def _save_dual_axis_panel(
        self,
        key: str,
        color_index: int,
        output_path: str | PathLike[str] | pathlib.Path,
        title: str | None = None,
    ) -> Any:
        """
        Save a single dual-axis (duration + count) panel for one timer.

        Parameters
        ----------
        key : str
            Timer name in the timer tree.
        color_index : int
            Color cycle index for the duration line.
        output_path : str | os.PathLike[str]
            Figure output path.
        title : str | None, optional
            Panel title.

        Returns
        -------
        matplotlib.figure.Figure
            The generated figure.
        """
        fig, ax_dur = plt.subplots(figsize=(8, 4))
        dur_color = f'C{color_index}'
        count_color = 'C7' if color_index != 7 else 'C5'

        # Duration (left y-axis)
        frames_d, values_d = self.get_values(key, 'duration')
        has_dur = len(frames_d) > 0
        dur_label = 'Time (s)'
        if has_dur:
            values_d, dur_label = self._auto_time_unit(values_d)
            ax_dur.plot(frames_d, values_d, marker='o', markersize=3,
                        color=dur_color, label='Duration')
            ax_dur.fill_between(frames_d, values_d, alpha=0.15,
                                color=dur_color)
        ax_dur.set_xlabel('Frame')
        ax_dur.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax_dur.set_ylabel(dur_label, color=dur_color)
        ax_dur.tick_params(axis='y', labelcolor=dur_color)

        # Count (right y-axis)
        ax_cnt = ax_dur.twinx()
        frames_c, values_c = self.get_values(key, 'count')
        has_cnt = len(frames_c) > 0
        if has_cnt:
            ax_cnt.bar(frames_c, values_c, alpha=0.30, color=count_color,
                       label='Count', width=0.6)
        ax_cnt.set_ylabel('Count', color=count_color)
        ax_cnt.tick_params(axis='y', labelcolor=count_color)
        if has_cnt:
            max_cnt = int(np.max(values_c))
            ax_cnt.set_yticks(np.arange(0, max_cnt + 2,
                                        max(1, (max_cnt + 1) // 6)))

        ax_dur.set_title(title or key)
        ax_dur.grid(True, alpha=0.3)

        if not has_dur and not has_cnt:
            ax_dur.text(0.5, 0.5, 'No data', ha='center', va='center',
                        transform=ax_dur.transAxes, fontsize=12, color='gray')

        lines_d, labels_d = ax_dur.get_legend_handles_labels()
        lines_c, labels_c = ax_cnt.get_legend_handles_labels()
        if lines_d or lines_c:
            ax_dur.legend(lines_d + lines_c, labels_d + labels_c,
                          loc='upper left', fontsize=8)

        plt.tight_layout()
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        Logger.info(f'Panel saved to {output_path}')
        plt.close(fig)
        return fig

    @staticmethod
    def _key_to_filename(key: str) -> str:
        """Convert a timer key to a safe filename stem."""
        return key.lower().replace(' ', '_')

    def summary_report(
        self,
        keys: list[str] | None = None,
        output_dir: str | PathLike[str] | pathlib.Path | None = None,
        workspace: str | PathLike[str] | pathlib.Path | None = None,
        aliases: Mapping[str, str] | None = None,
    ) -> pathlib.Path | None:
        """Generate a comprehensive performance summary as a folder.

        The output folder will contain:

        * ``report.md`` — a Markdown file that references all figures.
        * ``profiler_heatmap.svg`` — hierarchical sunburst of the last
          frame's timer breakdown.
        * One ``<key>.svg`` per timer key — dual-axis chart with
          *duration* on the left y-axis and *count* on the right y-axis.
        * ``system_deps.svg`` — directed graph of backend system
          dependencies (when *workspace* is provided).

        Parameters
        ----------
        keys : list[str] | None, optional
            Timer keys (or alias keys) for per-frame panels. Defaults to
            ``['newton_iteration', 'global_linear_system', 'line_search', 'dcd', 'spmv']``.
        output_dir : str | os.PathLike[str] | None, optional
            Report output directory. When omitted, plots are shown
            interactively and files are not written.
        workspace : str | os.PathLike[str] | None, optional
            Workspace path containing ``systems.json`` for dependency graph
            rendering.
        aliases : dict[str, str] | None, optional
            Extra alias-to-regex mapping merged into ``DEFAULT_ALIASES``.

        Returns
        -------
        pathlib.Path | None
            Path to ``report.md`` when saved, otherwise ``None``.
        """
        if keys is None:
            keys = ['newton_iteration', 'global_linear_system',
                    'line_search', 'dcd', 'spmv']

        # Resolve aliases → actual timer names via regex matching
        if self._frames:
            keys, display_map = self._resolve_keys(keys, aliases)
            # Filter out keys still not found in data
            available = set()
            for frame_data in self._frames:
                self._collect_names(frame_data, available)
            original = keys[:]
            keys = [k for k in keys if k in available]
            skipped = [k for k in original if k not in available]
            if skipped:
                Logger.info(f'Skipping timers not found in data: {skipped}')
        else:
            display_map = {k: k for k in keys}

        save = output_dir is not None
        if save:
            out = pathlib.Path(output_dir)
            out.mkdir(parents=True, exist_ok=True)

        md_lines = ['# Simulation Performance Summary', '']

        # -- System dependency graph ------------------------------------------
        systems_json = None
        if workspace is not None:
            candidate = pathlib.Path(workspace) / 'systems.json'
            if candidate.exists():
                systems_json = candidate

        if systems_json is not None:
            dep_file = 'system_deps.svg'
            md_lines.append('<details>')
            md_lines.append('<summary><strong>System Dependencies</strong>'
                            '</summary>')
            md_lines.append('')
            md_lines.append('Directed graph of backend system dependencies.  '
                            'Green nodes are **engine-aware**, blue nodes are '
                            'internal.  Solid blue arrows are strong '
                            'dependencies, dashed grey arrows are weak '
                            'dependencies.')
            md_lines.append('')
            md_lines.append(f'![System Dependencies]({dep_file})')
            md_lines.append('')
            md_lines.append('</details>')
            md_lines.append('')

            if save:
                self.system_dependency_graph(str(systems_json),
                                            output_path=str(out / dep_file))
                plt.close('all')
            else:
                self.system_dependency_graph(str(systems_json))

        # -- Profiler heatmap -------------------------------------------------
        has_heatmap = (self._frames
                       and self._frames[-1].get('children'))
        heatmap_file = 'profiler_heatmap.svg'

        if has_heatmap:
            md_lines.append('## Profiler Heatmap')
            md_lines.append('')
            if len(self._frames) > 1:
                md_lines.append(f'Aggregated time breakdown across all '
                                f'{len(self._frames)} collected frames '
                                f'(sunburst chart).')
            else:
                md_lines.append('Hierarchical time breakdown of the '
                                'collected frame (sunburst chart).')
            md_lines.append('')
            md_lines.append(f'![Profiler Heatmap]({heatmap_file})')
            md_lines.append('')

            if save:
                self.profiler_heatmap(output_path=str(out / heatmap_file))
            else:
                self.profiler_heatmap()

        # -- Total time per frame (first plot after heatmap) ------------------
        total_file = 'total_time_per_frame.svg'
        # The timer tree root is 'GlobalTimer' which has duration=0.
        # The actual total frame time is in its first child (typically
        # 'Pipeline').  Walk down to find the first child with a non-zero
        # duration to use as the "total time" source.
        root_name = ''
        for fd in self._frames:
            # Try the first child of the root (e.g. 'Pipeline')
            children = fd.get('children', [])
            if children and children[0].get('duration', 0) > 0:
                root_name = children[0].get('name', '')
                break
            # Fallback: use root if it has duration
            if fd.get('duration', 0) > 0:
                root_name = fd.get('name', '')
                break
        frames_t, values_t = (
            self.get_values(root_name, 'duration') if root_name
            else (np.array([]), np.array([]))
        )
        has_total = len(frames_t) > 0

        if has_total:
            md_lines.append('## Total Time Per Frame')
            md_lines.append('')
            md_lines.append('Total simulation time per frame.')
            md_lines.append('')
            md_lines.append(f'![Total Time Per Frame]({total_file})')
            md_lines.append('')

            if save:
                # color_index=0 → matplotlib C0 (first colour in cycle)
                self._save_dual_axis_panel(root_name, 0,
                                           str(out / total_file),
                                           title='Total Time Per Frame')
                plt.close('all')
            else:
                fig, ax_dur = plt.subplots(figsize=(8, 4))
                values_scaled, dur_label = self._auto_time_unit(values_t)
                ax_dur.plot(frames_t, values_scaled, marker='o', markersize=3,
                            color='C0', label='Duration')
                ax_dur.fill_between(frames_t, values_scaled, alpha=0.15,
                                    color='C0')
                ax_dur.set_xlabel('Frame')
                ax_dur.xaxis.set_major_locator(MaxNLocator(integer=True))
                ax_dur.set_ylabel(dur_label, color='C0')
                ax_dur.tick_params(axis='y', labelcolor='C0')
                ax_dur.set_title('Total Time Per Frame')
                ax_dur.grid(True, alpha=0.3)
                plt.tight_layout()
                plt.show()

        # -- Per-key panels ---------------------------------------------------
        if keys:
            md_lines.append('## Per-Frame Statistics')
            md_lines.append('')
            md_lines.append('Each chart shows **duration** (left y-axis, '
                            'line) and **count** (right y-axis, bars) '
                            'per simulation frame.')
            md_lines.append('')

            for col, key in enumerate(keys):
                label = display_map.get(key, key)
                fname = f'{self._key_to_filename(label)}.svg'
                md_lines.append(f'### {label}')
                md_lines.append('')
                md_lines.append(f'![{label}]({fname})')
                md_lines.append('')

                if save:
                    self._save_dual_axis_panel(key, col, str(out / fname),
                                              title=label)
                else:
                    # Show interactively
                    fig, ax_dur = plt.subplots(figsize=(8, 4))
                    dur_color = f'C{col}'
                    count_color = 'C7' if col != 7 else 'C5'
                    frames_d, values_d = self.get_values(key, 'duration')
                    dur_label = 'Time (s)'
                    if len(frames_d) > 0:
                        values_d, dur_label = self._auto_time_unit(values_d)
                        ax_dur.plot(frames_d, values_d, marker='o',
                                    markersize=3, color=dur_color,
                                    label='Duration')
                        ax_dur.fill_between(frames_d, values_d, alpha=0.15,
                                            color=dur_color)
                    ax_dur.set_xlabel('Frame')
                    ax_dur.xaxis.set_major_locator(MaxNLocator(integer=True))
                    ax_dur.set_ylabel(dur_label, color=dur_color)
                    ax_dur.tick_params(axis='y', labelcolor=dur_color)
                    ax_cnt = ax_dur.twinx()
                    frames_c, values_c = self.get_values(key, 'count')
                    if len(frames_c) > 0:
                        ax_cnt.bar(frames_c, values_c, alpha=0.30,
                                   color=count_color, label='Count',
                                   width=0.6)
                    ax_cnt.set_ylabel('Count', color=count_color)
                    ax_cnt.tick_params(axis='y', labelcolor=count_color)
                    ax_dur.set_title(label)
                    ax_dur.grid(True, alpha=0.3)
                    plt.tight_layout()
                    plt.show()

        # -- Markdown table (collapsible) -------------------------------------
        if self._frames and keys:
            md_lines.append('<details>')
            md_lines.append('<summary><strong>Duration Table</strong></summary>')
            md_lines.append('')
            md_lines.append(self.to_markdown(keys, metric='duration',
                                            display_map=display_map))
            md_lines.append('')
            md_lines.append('</details>')
            md_lines.append('')
            md_lines.append('<details>')
            md_lines.append('<summary><strong>Count Table</strong></summary>')
            md_lines.append('')
            md_lines.append(self.to_markdown(keys, metric='count',
                                            display_map=display_map))
            md_lines.append('')
            md_lines.append('</details>')
            md_lines.append('')

        # -- Write Markdown ---------------------------------------------------
        md_text = '\n'.join(md_lines)
        if save:
            report_path = out / 'report.md'
            report_path.write_text(md_text, encoding='utf-8')
            Logger.info(f'Summary report written to {report_path.resolve()}')
            return report_path
        else:
            print(md_text)
            return None
