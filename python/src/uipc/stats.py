from uipc import Logger
import os
import re
import json
import pathlib
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

    def __init__(self):
        # Each entry is the dict returned by uipc.Timer.report_as_json()
        self._frames = []
        # Ensure timers are enabled so collect() captures data
        try:
            import uipc
            uipc.Timer.enable_all()
        except (AttributeError, ImportError):
            pass  # native module not available (e.g. in tests)

    def collect(self):
        """Record timer statistics for the current frame.

        Call this once per simulation step, after ``world.advance()`` and
        ``world.retrieve()``.  Each call also resets the internal timer
        counters so that successive calls capture individual-frame data.

        :return: ``self`` (for optional method chaining).
        """
        import uipc
        timer_data = uipc.Timer.report_as_json()
        self._frames.append(timer_data)
        return self

    @property
    def num_frames(self):
        """Number of frames collected so far."""
        return len(self._frames)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _find_node(node, name):
        """Depth-first search for a node with the given ``name``."""
        if node.get('name') == name:
            return node
        for child in node.get('children', []):
            result = SimulationStats._find_node(child, name)
            if result is not None:
                return result
        return None

    @staticmethod
    def _collect_names(node, names=None):
        """Recursively collect every timer name present in *node*."""
        if names is None:
            names = set()
        n = node.get('name')
        if n:
            names.add(n)
        for child in node.get('children', []):
            SimulationStats._collect_names(child, names)
        return names

    def _resolve_keys(self, keys, aliases=None):
        """Resolve short alias keys to actual timer names found in data.

        Each key is checked in order:

        1. If *key* matches an actual timer name exactly, use it as-is.
        2. If *key* is in *aliases* (or :attr:`DEFAULT_ALIASES`), compile
           the regex pattern and search all known timer names for a match.
        3. Otherwise the key is kept as-is (will be filtered later).

        :param keys: List of key strings (short aliases or exact names).
        :param aliases: Optional ``{alias: regex_pattern}`` dict.
            Merged with :attr:`DEFAULT_ALIASES` (user values take priority).
        :return: ``(resolved_keys, display_map)`` where *resolved_keys*
            is a list of actual timer names and *display_map* maps each
            resolved name back to a pretty display label.
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
                match = next((n for n in available if regex.search(n)), None)
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
    def _auto_time_unit(values):
        """Choose the best time unit for display.

        :param values: Array of durations in seconds.
        :return: ``(scaled_values, label)`` where *label* is e.g.
            ``'Time (ms)'`` and *scaled_values* is the array converted
            to the chosen unit.
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

    def get_values(self, key, metric='duration'):
        """Return per-frame values for a named timer.

        :param key: Timer name to search for (e.g. ``'Newton'``, ``'PCG'``).
        :param metric: ``'duration'`` (seconds) or ``'count'``.
        :return: ``(frames, values)`` — two 1-D NumPy arrays of the same
            length.  Frames where the timer was absent are omitted.
        :raises ValueError: If *metric* is not ``'duration'`` or ``'count'``.
        """
        if metric not in ('duration', 'count'):
            raise ValueError(f"metric must be 'duration' or 'count', got '{metric}'")
        frames = []
        values = []
        for i, timer_data in enumerate(self._frames):
            node = self._find_node(timer_data, key)
            if node is not None:
                frames.append(i)
                values.append(node.get(metric, 0))
        return np.array(frames, dtype=int), np.array(values, dtype=float)

    def plot(self, keys, metric='duration', kind='line',
             title=None, output_path=None):
        """Plot per-frame timer values as a line or bar chart.

        :param keys: A timer name (str) or list of timer names to plot.
        :param metric: ``'duration'`` (seconds) or ``'count'``.
        :param kind: ``'line'`` for a curve plot or ``'bar'`` for a grouped
            bar chart.
        :param title: Chart title.  Auto-generated when *None*.
        :param output_path: File path to save the figure (passed to
            :func:`matplotlib.pyplot.savefig`).  When *None* the figure is
            shown interactively but not saved.
        :return: The :class:`matplotlib.figure.Figure` object.
        :raises ValueError: If *metric* or *kind* is invalid.
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
        for idx, key in enumerate(keys):
            frames, values = self.get_values(key, metric)
            if len(frames) == 0:
                Logger.warn(f"No data found for timer '{key}'")
                continue
            has_data = True
            if metric == 'duration':
                values, ylabel = self._auto_time_unit(values)
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

    def to_markdown(self, keys=None, metric='duration', display_map=None):
        """Export per-frame statistics as a Markdown table.

        :param keys: Timer name(s) to include as columns.  When *None* all
            timer names found in the collected data are used.
        :param metric: ``'duration'`` (seconds) or ``'count'``.
        :param display_map: Optional ``{timer_name: display_label}`` dict
            for column headers.
        :return: A Markdown-formatted table as a plain string.
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
                node = self._find_node(timer_data, key)
                if node is not None:
                    val = node.get(metric, 0)
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
    def _draw_profiler_heatmap(ax, timer_data, max_depth=999, include_other=True):
        """Draw a hierarchical sunburst chart onto *ax*."""
        if not timer_data.get('children'):
            return False
        ld = {}; na = {}; li = []
        ch = timer_data['children']
        if not ch or not ch[0]:
            return False
        td = ch[0].get('duration', 0); fc = ch[0].get('count', 0)
        if td == 0:
            return False
        def _cl(node, d=0, pn='root'):
            if d > max_depth: return
            nm = node.get('name', 'Unknown')
            ld.setdefault(d, {})[nm] = {'name': nm, 'full_name': f'{pn} -> {nm}' if d > 0 else nm,
                'duration': node.get('duration', 0), 'percentage': 0,
                'parent': pn if d > 0 else None, 'count': node.get('count', 0)}
            for c in node.get('children', []): _cl(c, d + 1, nm)
        _cl(timer_data)
        if not ld: return False
        rw = 0.3; mod = max(ld.keys())
        if include_other:
            for d in sorted(ld.keys()):
                if d == mod: continue
                for pn, pd in ld[d].items():
                    if 'other' in pn: continue
                    cdur = sum(cd['duration'] for cd in ld.get(d+1, {}).values()
                               if cd['parent'] == pn and 'other' not in pd['name'])
                    cc = sum(1 for cd in ld.get(d+1, {}).values()
                             if cd['parent'] == pn and 'other' not in pd['name'])
                    if cc == 0: continue
                    rem = pd['duration'] - cdur
                    if rem > 1e-6:
                        ld.setdefault(d+1, {})[f'{pn}_other'] = {
                            'name': 'Other', 'full_name': f'{pn} -> Other',
                            'duration': rem, 'percentage': (rem/td)*100,
                            'parent': pn, 'count': 1, 'is_other': True}
        for d, nodes in ld.items():
            if d == 0: continue
            ir = 0.25 + (mod - d) * rw
            if d == 1:
                si = list(nodes.values()); sz = [i['duration'] for i in si]
                for i in si: i['percentage'] = (i['duration']/td)*100
                w, _ = ax.pie(sz, labels=nodes, radius=ir+rw, startangle=90,
                    counterclock=True, autopct=None,
                    wedgeprops={'width': rw, 'edgecolor': 'white', 'linewidth': 0.5})
                na[1] = {}
                for i, (wg, it) in enumerate(zip(w, si)):
                    na[1][it['name']] = (wg.theta1, wg.theta2)
                    if not it.get('is_other', False):
                        a = np.deg2rad((wg.theta2+wg.theta1)/2); r = ir+rw/2
                        x, y = r*np.cos(a), r*np.sin(a)
                    ax.text(x, y, f'{it["percentage"]:.2f}%', ha='center', va='center',
                            fontsize=8, color='white', fontweight='bold')
                    li.append((it['full_name'], it['percentage'], wg.get_facecolor()))
            else:
                po = list(ld.get(d-1, {}).keys())
                pgo = {p: [] for p in po}
                for it in nodes.values():
                    if it['parent'] in pgo: pgo[it['parent']].append(it)
                si = []; 
                for p in po: si.extend(pgo.get(p, []))
                pg = {}
                for it in si: pg.setdefault(it['parent'], []).append(it)
                na[d] = {}
                for par, gc in pg.items():
                    if par not in ld.get(d-1, {}): continue
                    for it in gc: it['percentage'] = (it['duration']/td)*100
                    cs = [i['duration'] for i in gc]
                    if par in na.get(d-1, {}):
                        ps, pe = na[d-1][par]; pas = pe - ps
                        w, _ = ax.pie(cs, labels=None, radius=ir+rw, startangle=90,
                            counterclock=True, autopct=None,
                            wedgeprops={'width': rw, 'edgecolor': 'white', 'linewidth': 0.5})
                        tcd = sum(cs); ca = ps
                        for i, (wg, c) in enumerate(zip(w, cs)):
                            t1 = ca; t2 = ca + pas*(c/tcd); ca = t2
                            wg.set_theta1(t1); wg.set_theta2(t2)
                            if i < len(gc): gc[i]['mapped_angles'] = (t1, t2)
                    else:
                        w, _ = ax.pie(cs, labels=None, radius=ir+rw, startangle=0,
                            counterclock=True, autopct=None,
                            wedgeprops={'width': rw, 'edgecolor': 'white', 'linewidth': 0.5})
                    for i, (wg, it) in enumerate(zip(w, gc)):
                        na[d][it['name']] = (wg.theta1, wg.theta2)
                        asz = wg.theta2 - wg.theta1
                        if not it.get('is_other', False):
                            a = np.deg2rad((wg.theta2+wg.theta1)/2); r = ir+rw/2
                            x, y = r*np.cos(a), r*np.sin(a)
                            if asz > 5:
                                ax.text(x, y, f'{it["percentage"]:.2f}%', ha='center',
                                        va='center', fontsize=8, color='white', fontweight='bold')
                            li.append((it['full_name'], it['percentage'], wg.get_facecolor()))
        ax.text(0, 0, f'{td:.3f}s', ha='center', va='center', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.95))
        ax.set_aspect('equal')
        li.sort(key=lambda x: x[1], reverse=True)
        patches = [mpatches.Patch(label=f'{n} ({p:.2f}%)', color=c) for n, p, c in li]
        ax.legend(handles=patches, loc='center left', bbox_to_anchor=(1.05, 0.5),
                  fontsize=7, frameon=True, fancybox=True, shadow=True,
                  title=f'Total: {td:.3f}s / {fc} frames')
        return True

    def profiler_heatmap(self, frame_index=-1, output_path=None,
                         max_depth=999, include_other=True):
        """Create a hierarchical sunburst chart from a collected frame.

        :param frame_index: Which frame's timer data to visualise.
            Defaults to ``-1`` (the last collected frame).
        :param output_path: File path to save the figure.  When *None*
            the figure is shown interactively.
        :param max_depth: Maximum depth of the hierarchy to include.
        :param include_other: Whether to include "Other" slices for
            unaccounted time.
        :return: The :class:`matplotlib.figure.Figure` object, or *None*
            if no data is available.
        """
        if not self._frames:
            Logger.warn('No frames collected; cannot create profiler heatmap')
            return None

        timer_data = self._frames[frame_index]
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
    def _strip_system_name(name):
        """Remove common C++ namespace prefixes from a system name."""
        for prefix in SimulationStats._SYSTEM_NAME_PREFIXES:
            if name.startswith(prefix):
                return name[len(prefix):]
        return name

    @staticmethod
    def _remove_overlaps(pos, sizes, iterations=80, pad=1.15):
        """Push nodes apart until no bounding boxes overlap.

        :param pos: ``{node: (x, y)}`` mutable dict of positions.
        :param sizes: ``{node: (half_w, half_h)}`` half-extents.
        :param iterations: Maximum adjustment passes.
        :param pad: Multiplicative padding on extents.
        """
        nodes = list(pos.keys())
        for _ in range(iterations):
            moved = False
            for i, a in enumerate(nodes):
                ax, ay = pos[a]
                aw, ah = sizes[a][0] * pad, sizes[a][1] * pad
                for b in nodes[i + 1:]:
                    bx, by = pos[b]
                    bw, bh = sizes[b][0] * pad, sizes[b][1] * pad
                    ox = (aw + bw) - abs(ax - bx)
                    oy = (ah + bh) - abs(ay - by)
                    if ox > 0 and oy > 0:
                        # Push apart along the axis of least overlap
                        if ox < oy:
                            shift = ox / 2 + 0.001
                            if ax < bx:
                                pos[a] = (ax - shift, ay)
                                pos[b] = (bx + shift, by)
                            else:
                                pos[a] = (ax + shift, ay)
                                pos[b] = (bx - shift, by)
                        else:
                            shift = oy / 2 + 0.001
                            if ay < by:
                                pos[a] = (ax, ay - shift)
                                pos[b] = (bx, by + shift)
                            else:
                                pos[a] = (ax, ay + shift)
                                pos[b] = (bx, by - shift)
                        moved = True
            if not moved:
                break

    @staticmethod
    def _draw_system_dependency_graph(systems_json_path, output_path=None):
        """Render a directed dependency graph of backend systems.

        Nodes are drawn as rounded-rectangle label boxes that are
        guaranteed not to overlap.  Edges curve freely to avoid clutter.

        :param systems_json_path: Path to ``systems.json``.
        :param output_path: File path to save the figure (SVG recommended).
            When *None* the figure is shown interactively.
        :return: The :class:`matplotlib.figure.Figure`, or *None* if the
            data could not be loaded.
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
        except Exception:
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

    def system_dependency_graph(self, systems_json, output_path=None):
        """Create a standalone system dependency graph figure.

        :param systems_json: Path to ``systems.json`` or to the workspace
            directory that contains it.
        :param output_path: File path to save the figure (SVG recommended).
        :return: The :class:`matplotlib.figure.Figure`, or *None* if the
            file was not found.
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

    def _save_dual_axis_panel(self, key, color_index, output_path, title=None):
        """Save a single dual-axis (duration + count) figure for *key*.

        :param key: Timer name (actual name in the timer tree).
        :param color_index: Colour cycle index for the duration line.
        :param output_path: File path to save the PNG.
        :return: The :class:`matplotlib.figure.Figure` object.
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
    def _key_to_filename(key):
        """Convert a timer key to a safe filename stem."""
        return key.lower().replace(' ', '_')

    def summary_report(self, keys=None, output_dir=None, workspace=None,
                       aliases=None):
        """Generate a comprehensive performance summary as a folder.

        The output folder will contain:

        * ``report.md`` — a Markdown file that references all figures.
        * ``profiler_heatmap.svg`` — hierarchical sunburst of the last
          frame's timer breakdown.
        * One ``<key>.svg`` per timer key — dual-axis chart with
          *duration* on the left y-axis and *count* on the right y-axis.
        * ``system_deps.svg`` — directed graph of backend system
          dependencies (when *workspace* is provided).

        :param keys: Timer names (or short alias keys) for per-frame
            panels.  Defaults to ``['newton_iteration',
            'global_linear_system', 'line_search', 'dcd', 'spmv']``.
            Each key is resolved via regex patterns in
            :attr:`DEFAULT_ALIASES` (or *aliases*).  Exact timer names
            are also accepted.
        :param output_dir: Directory to write the report into.  Created
            if it does not exist.  When *None* the figures are shown
            interactively and no files are written.
        :param workspace: Path to the simulation workspace directory
            that contains ``systems.json``.  When provided the backend
            system dependency graph is included in the report.
        :param aliases: Optional ``{alias: regex_pattern}`` dict merged
            with :attr:`DEFAULT_ALIASES`.  User values take priority.
        :return: The path to ``report.md`` as a :class:`pathlib.Path`,
            or *None* when *output_dir* is not given.
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
                            'internal.  Solid red arrows are strong '
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
            md_lines.append('Hierarchical time breakdown of the last '
                            'collected frame (sunburst chart).')
            md_lines.append('')
            md_lines.append(f'![Profiler Heatmap]({heatmap_file})')
            md_lines.append('')

            if save:
                self.profiler_heatmap(output_path=str(out / heatmap_file))
            else:
                self.profiler_heatmap()

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
