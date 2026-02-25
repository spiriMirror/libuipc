from uipc import Logger
import os
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

__all__ = ['SimulationStats']


class SimulationStats:
    """Collect and analyse per-frame simulation performance statistics.

    Typical usage::

        import uipc
        from uipc.stats import SimulationStats

        uipc.Timer.enable_all()
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

        # Comprehensive summary report (Markdown + PNGs in a folder)
        stats.summary_report(output_dir='perf_report')

        # Standalone profiler heatmap (sunburst chart)
        stats.profiler_heatmap()
    """

    def __init__(self):
        # Each entry is the dict returned by uipc.Timer.report_as_json()
        self._frames = []

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

        ylabel = 'Time (s)' if metric == 'duration' else 'Count'
        num_keys = len(keys)

        fig, ax = plt.subplots(figsize=(10, 5))
        has_data = False
        for idx, key in enumerate(keys):
            frames, values = self.get_values(key, metric)
            if len(frames) == 0:
                Logger.warn(f"No data found for timer '{key}'")
                continue
            has_data = True
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
        ax.set_ylabel(ylabel)
        ax.set_title(title or f'{", ".join(keys)} \u2014 {metric} per frame')
        if has_data:
            ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
        else:
            plt.show()
        return fig

    def to_markdown(self, keys=None, metric='duration'):
        """Export per-frame statistics as a Markdown table.

        :param keys: Timer name(s) to include as columns.  When *None* all
            timer names found in the collected data are used.
        :param metric: ``'duration'`` (seconds) or ``'count'``.
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

        col_w = max((len(k) for k in keys), default=7)
        col_w = max(col_w, 7)

        header = '| Frame | ' + ' | '.join(k.ljust(col_w) for k in keys) + ' |'
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
    def _draw_profiler_heatmap(ax, timer_data, max_depth=999,
                               include_other=True):
        """Draw a hierarchical sunburst chart onto *ax*.

        This is the core rendering routine used by both
        :meth:`profiler_heatmap` and :meth:`summary_report`.

        :param ax: A :class:`matplotlib.axes.Axes` to draw on.
        :param timer_data: A single timer tree dict (one frame).
        :param max_depth: Maximum hierarchy depth to include.
        :param include_other: Whether to synthesise "Other" slices for
            unaccounted time.
        :return: ``True`` if something was drawn, ``False`` otherwise.
        """
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

        # -- Collect hierarchy ------------------------------------------------
        def collect_level_data(node, depth=0, parent_name='root'):
            if depth > max_depth:
                return
            name = node.get('name', 'Unknown')
            duration = node.get('duration', 0)
            count = node.get('count', 0)
            level_data.setdefault(depth, {})[name] = {
                'name': name,
                'full_name': f'{parent_name} -> {name}' if depth > 0 else name,
                'duration': duration,
                'percentage': 0,
                'parent': parent_name if depth > 0 else None,
                'count': count,
            }
            for child in node.get('children', []):
                collect_level_data(child, depth + 1, name)

        collect_level_data(timer_data)
        if not level_data:
            return False

        # -- Synthesise "Other" slices ----------------------------------------
        ring_width = 0.3
        max_observed_depth = max(level_data.keys())

        if include_other:
            for depth in sorted(level_data.keys()):
                if depth == max_observed_depth:
                    continue
                for parent_name, parent_data in level_data[depth].items():
                    if 'other' in parent_name:
                        continue
                    parent_duration = parent_data['duration']
                    child_total_duration = 0
                    child_count = 0
                    if depth + 1 in level_data:
                        for child_data in level_data[depth + 1].values():
                            if (child_data['parent'] == parent_name
                                    and 'other' not in parent_data['name']):
                                child_total_duration += child_data['duration']
                                child_count += 1
                    if child_count == 0:
                        continue
                    remaining_time = parent_duration - child_total_duration
                    if remaining_time > 1e-6:
                        other_key = f'{parent_name}_other'
                        level_data.setdefault(depth + 1, {})[other_key] = {
                            'name': 'Other',
                            'full_name': f'{parent_name} -> Other',
                            'duration': remaining_time,
                            'percentage': (remaining_time / total_duration) * 100,
                            'parent': parent_name,
                            'count': 1,
                            'is_other': True,
                        }

        # -- Draw rings -------------------------------------------------------
        for depth, nodes in level_data.items():
            if depth == 0:
                continue
            inner_radius = 0.25 + (max_observed_depth - depth) * ring_width

            if depth == 1:
                sorted_items = list(nodes.values())
                sizes = [it['duration'] for it in sorted_items]
                for it in sorted_items:
                    it['percentage'] = (it['duration'] / total_duration) * 100

                wedges, _ = ax.pie(
                    sizes, labels=nodes, radius=inner_radius + ring_width,
                    startangle=90, counterclock=True, autopct=None,
                    wedgeprops={'width': ring_width, 'edgecolor': 'white',
                                'linewidth': 0.5})

                node_angles[1] = {}
                for i, (wedge, item) in enumerate(zip(wedges, sorted_items)):
                    node_angles[1][item['name']] = (wedge.theta1, wedge.theta2)
                    if not item.get('is_other', False):
                        ang = np.deg2rad((wedge.theta2 + wedge.theta1) / 2)
                        r = inner_radius + ring_width / 2
                        x, y = r * np.cos(ang), r * np.sin(ang)
                    ax.text(x, y, f'{item["percentage"]:.2f}%',
                            ha='center', va='center', fontsize=8,
                            color='white', fontweight='bold')
                    legend_items.append((item['full_name'], item['percentage'],
                                        wedge.get_facecolor()))
            else:
                prev_level = level_data.get(depth - 1, {})
                parent_order = list(prev_level.keys())
                parent_groups_ordered = {p: [] for p in parent_order}
                for item in nodes.values():
                    p = item['parent']
                    if p in parent_groups_ordered:
                        parent_groups_ordered[p].append(item)
                sorted_items = []
                for p in parent_order:
                    sorted_items.extend(parent_groups_ordered.get(p, []))

                parent_groups = {}
                for item in sorted_items:
                    parent_groups.setdefault(item['parent'], []).append(item)

                node_angles[depth] = {}
                for parent, group_children in parent_groups.items():
                    if parent not in level_data.get(depth - 1, {}):
                        continue
                    for item in group_children:
                        item['percentage'] = (item['duration'] / total_duration) * 100
                    child_sizes = [it['duration'] for it in group_children]

                    if parent in node_angles.get(depth - 1, {}):
                        parent_start, parent_end = node_angles[depth - 1][parent]
                        parent_angle_size = parent_end - parent_start

                        wedges, _ = ax.pie(
                            child_sizes, labels=None,
                            radius=inner_radius + ring_width,
                            startangle=90, counterclock=True, autopct=None,
                            wedgeprops={'width': ring_width,
                                        'edgecolor': 'white', 'linewidth': 0.5})

                        total_child_dur = sum(child_sizes)
                        cur_angle = parent_start
                        for i, (wedge, cs) in enumerate(zip(wedges, child_sizes)):
                            prop = cs / total_child_dur
                            new_t1 = cur_angle
                            new_t2 = cur_angle + parent_angle_size * prop
                            cur_angle = new_t2
                            wedge.set_theta1(new_t1)
                            wedge.set_theta2(new_t2)
                            if i < len(group_children):
                                group_children[i]['mapped_angles'] = (new_t1, new_t2)
                    else:
                        wedges, _ = ax.pie(
                            child_sizes, labels=None,
                            radius=inner_radius + ring_width,
                            startangle=0, counterclock=True, autopct=None,
                            wedgeprops={'width': ring_width,
                                        'edgecolor': 'white', 'linewidth': 0.5})

                    for i, (wedge, item) in enumerate(zip(wedges, group_children)):
                        node_angles[depth][item['name']] = (wedge.theta1, wedge.theta2)
                        angle_size = wedge.theta2 - wedge.theta1
                        if not item.get('is_other', False):
                            ang = np.deg2rad((wedge.theta2 + wedge.theta1) / 2)
                            r = inner_radius + ring_width / 2
                            x, y = r * np.cos(ang), r * np.sin(ang)
                            if angle_size > 5:
                                ax.text(x, y, f'{item["percentage"]:.2f}%',
                                        ha='center', va='center', fontsize=8,
                                        color='white', fontweight='bold')
                            legend_items.append((item['full_name'],
                                                 item['percentage'],
                                                 wedge.get_facecolor()))

        # -- Centre label -----------------------------------------------------
        ax.text(0, 0, f'{total_duration:.3f}s', ha='center', va='center',
                fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray',
                          alpha=0.95))
        ax.set_aspect('equal')

        # -- Legend -----------------------------------------------------------
        legend_items.sort(key=lambda x: x[1], reverse=True)
        patches = [mpatches.Patch(label=f'{n} ({p:.2f}%)', color=c)
                   for n, p, c in legend_items]
        ax.legend(handles=patches, loc='center left',
                  bbox_to_anchor=(1.05, 0.5), fontsize=7, frameon=True,
                  fancybox=True, shadow=True,
                  title=f'Total: {total_duration:.3f}s / {frame_count} frames')
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
        else:
            plt.show()
        return fig

    # ------------------------------------------------------------------
    # Summary report
    # ------------------------------------------------------------------

    def _save_dual_axis_panel(self, key, color_index, output_path):
        """Save a single dual-axis (duration + count) figure for *key*.

        :param key: Timer name.
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
        if has_dur:
            ax_dur.plot(frames_d, values_d, marker='o', markersize=3,
                        color=dur_color, label='Duration')
            ax_dur.fill_between(frames_d, values_d, alpha=0.15,
                                color=dur_color)
        ax_dur.set_xlabel('Frame')
        ax_dur.set_ylabel('Time (s)', color=dur_color)
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

        ax_dur.set_title(key)
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
        plt.close(fig)
        return fig

    @staticmethod
    def _key_to_filename(key):
        """Convert a timer key to a safe filename stem."""
        return key.lower().replace(' ', '_')

    def summary_report(self, keys=None, output_dir=None):
        """Generate a comprehensive performance summary as a folder.

        The output folder will contain:

        * ``report.md`` — a Markdown file that references all figures.
        * ``profiler_heatmap.png`` — hierarchical sunburst of the last
          frame's timer breakdown.
        * One ``<key>.png`` per timer key — dual-axis chart with
          *duration* on the left y-axis and *count* on the right y-axis.

        :param keys: Timer names for per-frame panels.  Defaults to
            ``['Newton', 'PCG', 'Line Search', 'Collision Detection']``.
        :param output_dir: Directory to write the report into.  Created
            if it does not exist.  When *None* the figures are shown
            interactively and no files are written.
        :return: The path to ``report.md`` as a :class:`pathlib.Path`,
            or *None* when *output_dir* is not given.
        """
        if keys is None:
            keys = ['Newton', 'PCG', 'Line Search', 'Collision Detection']

        save = output_dir is not None
        if save:
            out = pathlib.Path(output_dir)
            out.mkdir(parents=True, exist_ok=True)

        md_lines = ['# Simulation Performance Summary', '']

        # -- Profiler heatmap -------------------------------------------------
        has_heatmap = (self._frames
                       and self._frames[-1].get('children'))
        heatmap_file = 'profiler_heatmap.png'

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
                fname = f'{self._key_to_filename(key)}.png'
                md_lines.append(f'### {key}')
                md_lines.append('')
                md_lines.append(f'![{key}]({fname})')
                md_lines.append('')

                if save:
                    self._save_dual_axis_panel(key, col, str(out / fname))
                else:
                    # Show interactively (reuse the helper but no save)
                    fig, ax_dur = plt.subplots(figsize=(8, 4))
                    dur_color = f'C{col}'
                    count_color = 'C7' if col != 7 else 'C5'
                    frames_d, values_d = self.get_values(key, 'duration')
                    if len(frames_d) > 0:
                        ax_dur.plot(frames_d, values_d, marker='o',
                                    markersize=3, color=dur_color,
                                    label='Duration')
                        ax_dur.fill_between(frames_d, values_d, alpha=0.15,
                                            color=dur_color)
                    ax_dur.set_xlabel('Frame')
                    ax_dur.set_ylabel('Time (s)', color=dur_color)
                    ax_dur.tick_params(axis='y', labelcolor=dur_color)
                    ax_cnt = ax_dur.twinx()
                    frames_c, values_c = self.get_values(key, 'count')
                    if len(frames_c) > 0:
                        ax_cnt.bar(frames_c, values_c, alpha=0.30,
                                   color=count_color, label='Count',
                                   width=0.6)
                    ax_cnt.set_ylabel('Count', color=count_color)
                    ax_cnt.tick_params(axis='y', labelcolor=count_color)
                    ax_dur.set_title(key)
                    ax_dur.grid(True, alpha=0.3)
                    plt.tight_layout()
                    plt.show()

        # -- Markdown table (collapsible) -------------------------------------
        if self._frames and keys:
            md_lines.append('<details>')
            md_lines.append('<summary><strong>Duration Table</strong></summary>')
            md_lines.append('')
            md_lines.append(self.to_markdown(keys, metric='duration'))
            md_lines.append('')
            md_lines.append('</details>')
            md_lines.append('')
            md_lines.append('<details>')
            md_lines.append('<summary><strong>Count Table</strong></summary>')
            md_lines.append('')
            md_lines.append(self.to_markdown(keys, metric='count'))
            md_lines.append('')
            md_lines.append('</details>')
            md_lines.append('')

        # -- Write Markdown ---------------------------------------------------
        md_text = '\n'.join(md_lines)
        if save:
            report_path = out / 'report.md'
            report_path.write_text(md_text, encoding='utf-8')
            return report_path
        else:
            print(md_text)
            return None
