from uipc import Logger
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

__all__ = ['create_profiler_heatmap', 'SimulationStats']


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

        # Comprehensive multi-panel summary report
        stats.summary_report()
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
        """
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
        :param kind: ``'line'`` for a curve plot or ``'bar'`` for a bar chart.
        :param title: Chart title.  Auto-generated when *None*.
        :param output_path: File path to save the figure (passed to
            :func:`matplotlib.pyplot.savefig`).  When *None* the figure is
            shown interactively but not saved.
        :return: The :class:`matplotlib.figure.Figure` object.
        """
        if isinstance(keys, str):
            keys = [keys]

        ylabel = 'Time (s)' if metric == 'duration' else 'Count'

        fig, ax = plt.subplots(figsize=(10, 5))
        has_data = False
        for key in keys:
            frames, values = self.get_values(key, metric)
            if len(frames) == 0:
                Logger.warn(f"No data found for timer '{key}'")
                continue
            has_data = True
            if kind == 'bar':
                ax.bar(frames, values, label=key, alpha=0.7)
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
            for timer_data in self._frames:
                SimulationStats._collect_names(timer_data, found)
            # Remove the root node name (usually 'GlobalTimer')
            keys = sorted(found - {timer_data.get('name', '')
                                   for timer_data in self._frames})
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

    def summary_report(self, keys=None, output_path=None):
        """Generate a comprehensive multi-panel performance summary.

        Creates a grid of subplots with *duration* (top row) and *count*
        (bottom row) for each requested timer key.

        :param keys: Timer names to include.  Defaults to
            ``['Newton', 'PCG', 'Line Search', 'Collision Detection']``.
        :param output_path: File path to save the figure.
        :return: The :class:`matplotlib.figure.Figure` object.
        """
        if keys is None:
            keys = ['Newton', 'PCG', 'Line Search', 'Collision Detection']

        n = len(keys)
        fig, axes = plt.subplots(2, n, figsize=(5 * n, 8), squeeze=False)

        metrics = [('duration', 'Time (s)'), ('count', 'Count')]
        for col, key in enumerate(keys):
            for row, (metric, ylabel) in enumerate(metrics):
                ax = axes[row][col]
                frames, values = self.get_values(key, metric)
                if len(frames) == 0:
                    ax.text(0.5, 0.5, 'No data', ha='center', va='center',
                            transform=ax.transAxes, fontsize=12, color='gray')
                else:
                    color = f'C{col}'
                    ax.plot(frames, values, marker='o', markersize=3, color=color)
                    ax.fill_between(frames, values, alpha=0.15, color=color)
                ax.set_xlabel('Frame')
                ax.set_ylabel(ylabel)
                ax.set_title(f'{key} \u2014 {metric}')
                ax.grid(True, alpha=0.3)

        fig.suptitle('Simulation Performance Summary', fontsize=14, fontweight='bold')
        plt.tight_layout()

        if output_path:
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.show()
        return fig


def create_profiler_heatmap(
    timer_data, output_path=None, max_depth=999, include_other=True
):
    '''
    Create a hierarchical heatmap chart from profiling timer data.
    
    Usage:
        create_profiler_heatmap(uipc.Timer.report_as_json())
    
    :param timer_data: uipc timer data in JSON format, as returned by uipc.Timer.report_as_json()
    :param output_path: Path to save the output chart image. If None, the chart will be displayed but not saved.
    :param max_depth: Maximum depth of the hierarchy to include in the heatmap.
    :param include_other: Whether to include "other" categories in the heatmap.
    '''

    level_data = {}  # Store all level data
    node_angles = {}  # {level: {node_name: (start_angle, end_angle)}}

    legend_items = []  # Used to store legend items
    total_duration = (
        timer_data['children'][0].get('duration', 0) if timer_data['children'][0] else 0
    )
    frame_count = (
        timer_data['children'][0].get('count', 0) if timer_data['children'][0] else 0
    )

    if total_duration == 0:
        Logger.warn('Total duration is zero, cannot create heatmap')
        return

    def collect_level_data(node, depth=0, parent_name='root'):
        '''Recursively collect hierarchical data'''
        if depth > max_depth:
            return

        name = node.get('name', 'Unknown')
        duration = node.get('duration', 0)
        count = node.get('count', 0)

        # Store current node data
        level_data.setdefault(depth, {})[name] = {
            'name': name,
            'full_name': f'{parent_name} -> {name}' if depth > 0 else name,
            'duration': duration,
            'percentage': 0,
            'parent': parent_name if depth > 0 else None,
            'count': count,
        }

        # Recursively process child nodes
        if 'children' in node:
            for child in node['children']:
                collect_level_data(child, depth + 1, name)

    # Collect all hierarchical data
    collect_level_data(timer_data)

    if not level_data:
        Logger.warn('No valid hierarchical data collected')
        return

    # Create chart
    plt.figure(figsize=(15, 10))  # Enlarge chart size to accommodate more labels
    ring_width = 0.3
    # Find the maximum observed depth for reverse order calculation
    max_observed_depth = max(level_data.keys()) if level_data else 0
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
                        if (
                            child_data['parent'] == parent_name
                            and 'other' not in parent_data['name']
                        ):
                            child_total_duration += child_data['duration']
                            child_count += 1
                if child_count == 0:
                    # Skip parent node if it has no children
                    continue
                # Calculate remaining time
                remaining_time = parent_duration - child_total_duration
                # If there is remaining time, create an 'Other' node
                if remaining_time > 0.000001:  # Use a small threshold to avoid floating point errors
                    remaining_percentage = (remaining_time / total_duration) * 100
                    # Create remaining time node
                    other_node = {
                        'name': 'Other',
                        'full_name': f'{parent_name} -> Other',
                        'duration': remaining_time,
                        'percentage': remaining_percentage,
                        'parent': parent_name,
                        'count': 1,
                        'is_other': True,  # Mark as 'Other' node
                    }
                    # Add to next level data
                    other_key = f'{parent_name}_other'
                    level_data.setdefault(depth + 1, {})[other_key] = other_node

    for depth, nodes in level_data.items():
        if depth == 0:  # Skip root node
            continue
        # Inner radius calculation: deeper levels are more inward
        inner_radius = 0.25 + (max_observed_depth - depth) * ring_width
        if depth == 1:
            sorted_items = nodes.values()
            sizes = [item['duration'] for item in sorted_items]
            for item in sorted_items:
                item['percentage'] = (item['duration'] / total_duration) * 100
            # Draw the first ring
            wedges, _ = plt.pie(
                sizes,
                labels=nodes,
                radius=inner_radius + ring_width,
                startangle=90,
                counterclock=True,
                autopct=None,
                wedgeprops={
                    'width': ring_width,
                    'edgecolor': 'white',
                    'linewidth': 0.5,
                },
            )

            node_angles[1] = {}
            for i, (wedge, item) in enumerate(zip(wedges, sorted_items)):
                node_name = item['name']
                node_angles[1][node_name] = (wedge.theta1, wedge.theta2)
                # Only add internal labels for larger sectors and important nodes (not Other nodes)
                angle_size = wedge.theta2 - wedge.theta1
                if not item.get('is_other', False):
                    ang = (wedge.theta2 + wedge.theta1) / 2
                    ang_rad = np.deg2rad(ang)
                    # Label position inside the ring
                    r = inner_radius + ring_width / 2
                    x = r * np.cos(ang_rad)
                    y = r * np.sin(ang_rad)
                plt.text(
                    x,
                    y,
                    f'{item["percentage"]:.2f}%',  # Only show percentage
                    ha='center',
                    va='center',
                    fontsize=10,  # Increase font size
                    color='white',
                    fontweight='bold',
                )
                legend_items.append(
                    (
                        item['full_name'],
                        item['percentage'],
                        wedge.get_facecolor(),
                    )
                )
        else:
            # Get parent node order from previous level
            prev_level = level_data.get(depth - 1, {})
            parent_order = list(prev_level.keys())
            # Group by parent node
            parent_groups = {p: [] for p in parent_order}
            for item in nodes.values():
                p = item['parent']
                if p in parent_groups:
                    parent_groups[p].append(item)
            sorted_items = []
            for p in parent_order:
                group = parent_groups.get(p, [])
                group_sorted = group
                sorted_items.extend(group_sorted)
            # For deeper levels, group by parent node
            parent_groups = {}
            for item in sorted_items:
                parent = item['parent']
                if parent not in parent_groups:
                    parent_groups[parent] = []
                parent_groups[parent].append(item)

            # Create angle record dict for current depth
            node_angles[depth] = {}
            # Process children for each parent node
            for parent, children in parent_groups.items():
                if parent not in level_data.get(depth - 1, {}):
                    continue
                for item in children:
                    item['percentage'] = (item['duration'] / total_duration) * 100
                child_sizes = [item['duration'] for item in children]
                # Get parent's angle range (if exists)
                if parent in node_angles.get(depth - 1, {}):
                    parent_start, parent_end = node_angles[depth - 1][parent]
                    # Calculate parent node angle range size
                    parent_angle_size = parent_end - parent_start
                    # Draw pie chart first
                    wedges, _ = plt.pie(
                        child_sizes,
                        labels=None,
                        radius=inner_radius + ring_width,
                        startangle=90,  # Keep standard start angle
                        counterclock=True,
                        autopct=None,
                        wedgeprops={
                            'width': ring_width,
                            'edgecolor': 'white',
                            'linewidth': 0.5,
                        },
                    )
                    # Then adjust each sector's angle, limit within parent node range
                    # Save original sector angle range for record
                    orig_angles = []
                    for wedge in wedges:
                        orig_angles.append((wedge.theta1, wedge.theta2))

                    # Calculate total duration of all children under this parent
                    total_child_duration = sum(child_sizes)

                    # Initialize current angle position
                    current_angle = parent_start

                    # Assign angle for each child node
                    for i, (wedge, child_size) in enumerate(zip(wedges, child_sizes)):
                        # Calculate angle range based on child's duration proportion
                        angle_proportion = child_size / total_child_duration
                        angle_range = parent_angle_size * angle_proportion

                        # Set new angle range
                        new_theta1 = current_angle
                        new_theta2 = current_angle + angle_range

                        # Update current angle position
                        current_angle = new_theta2

                        # Update sector angle
                        wedge.set_theta1(new_theta1)
                        wedge.set_theta2(new_theta2)

                        # Update record with angle
                        if i < len(children):
                            children[i]['mapped_angles'] = (new_theta1, new_theta2)
                else:
                    # If no parent node angle info, draw normally
                    wedges, _ = plt.pie(
                        child_sizes,
                        labels=None,
                        radius=inner_radius + ring_width,
                        startangle=0,
                        counterclock=True,
                        autopct=None,
                        wedgeprops={
                            'width': ring_width,
                            'edgecolor': 'white',
                            'linewidth': 0.5,
                        },
                    )
                # After recording child node angle positions, add internal label drawing
                for i, (wedge, item) in enumerate(zip(wedges, children)):
                    node_angles[depth][item['name']] = (wedge.theta1, wedge.theta2)
                    # Only add internal labels for larger sectors and important nodes (not Other nodes)
                    angle_size = wedge.theta2 - wedge.theta1
                    if not item.get('is_other', False):
                        ang = (wedge.theta2 + wedge.theta1) / 2
                        ang_rad = np.deg2rad(ang)
                        # Label position inside the ring
                        r = inner_radius + ring_width / 2
                        x = r * np.cos(ang_rad)
                        y = r * np.sin(ang_rad)
                        if angle_size > 5:
                            plt.text(
                                x,
                                y,
                                f'{item["percentage"]:.2f}%',  # Only show percentage
                                ha='center',
                                va='center',
                                fontsize=10,  # Increase font size
                                color='white',
                                fontweight='bold',
                            )
                        legend_items.append(
                            (
                                item['full_name'],
                                item['percentage'],
                                wedge.get_facecolor(),
                            )
                        )

    # Add center text to show total time
    plt.text(
        0,
        0,
        f'{total_duration:.3f}s',
        ha='center',
        va='center',
        fontsize=12,
        fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.95),
    )

    plt.axis('equal')  # Ensure the circle is displayed correctly
    # Sort legend items by percentage
    legend_items.sort(key=lambda x: x[1], reverse=True)
    # Create legend patches
    legend_patches = []
    for name, percentage, color in legend_items:
        label = f'{name} ({percentage:.2f}%)'
        patch = mpatches.Patch(label=label, color=color)
        legend_patches.append(patch)

    # Add legend, place it on the right side of the chart
    plt.legend(
        handles=legend_patches,
        loc='center left',
        bbox_to_anchor=(1.05, 0.5),
        fontsize=9,
        frameon=True,
        fancybox=True,
        shadow=True,
        title=f'Total time:{total_duration:.3f}s/{frame_count} Frames',
    )
    # Save or show chart
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight', pad_inches=0.5)
    plt.tight_layout()
    plt.show()
