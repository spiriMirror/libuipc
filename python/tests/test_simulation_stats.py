"""Tests for uipc.stats.SimulationStats.

The tests exercise the pure-Python data processing logic (get_values,
to_markdown) without requiring the native uipc extension to be built or a display.
They work by injecting pre-built timer-tree dictionaries directly into
SimulationStats._frames, bypassing collect() which requires a live Timer.
"""
import os
import sys
import types
import pathlib
import importlib.util

import pytest
import numpy as np

# Use a non-interactive matplotlib backend so tests run in headless CI
os.environ.setdefault('MPLBACKEND', 'Agg')

# ---------------------------------------------------------------------------
# Bootstrap: load uipc.stats without the native uipc extension
# ---------------------------------------------------------------------------

_STATS_PATH = pathlib.Path(__file__).parent.parent / 'src' / 'uipc' / 'stats.py'


def _load_stats_module():
    """Load uipc/stats.py with a minimal stub for the uipc package."""
    # Build a minimal uipc stub (needs __path__ to be treated as a package)
    if 'uipc' not in sys.modules:
        stub = types.ModuleType('uipc')
        stub.__path__ = []

        class _Logger:
            @staticmethod
            def warn(msg):
                pass

            @staticmethod
            def info(msg):
                pass

        stub.Logger = _Logger
        sys.modules['uipc'] = stub

    # Load the stats module directly from the source tree
    spec = importlib.util.spec_from_file_location('uipc.stats', str(_STATS_PATH))
    mod = importlib.util.module_from_spec(spec)
    sys.modules['uipc.stats'] = mod
    spec.loader.exec_module(mod)
    return mod


_stats = _load_stats_module()
SimulationStats = _stats.SimulationStats


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Timer data constants used in _make_timer_data
_NEWTON_BASE_DURATION = 0.5
_NEWTON_DURATION_INCREMENT = 0.01
_NEWTON_BASE_COUNT = 3
_PCG_BASE_DURATION = 0.2
_PCG_DURATION_INCREMENT = 0.005
_PCG_BASE_COUNT = 10
_PCG_COUNT_INCREMENT = 2
_CD_BASE_DURATION = 0.3
_CD_DURATION_INCREMENT = 0.002


def _make_timer_data(frame_index):
    """Return a minimal timer-tree dict that mimics Timer.report_as_json()."""
    return {
        'name': 'GlobalTimer',
        'duration': 1.0,
        'count': 1,
        'parent': '',
        'children': [
            {
                'name': 'Newton',
                'duration': _NEWTON_BASE_DURATION + frame_index * _NEWTON_DURATION_INCREMENT,
                'count': _NEWTON_BASE_COUNT + frame_index,
                'parent': 'GlobalTimer',
                'children': [
                    {
                        'name': 'PCG',
                        'duration': _PCG_BASE_DURATION + frame_index * _PCG_DURATION_INCREMENT,
                        'count': _PCG_BASE_COUNT + frame_index * _PCG_COUNT_INCREMENT,
                        'parent': '/GlobalTimer/Newton',
                        'children': [],
                    }
                ],
            },
            {
                'name': 'Collision Detection',
                'duration': _CD_BASE_DURATION + frame_index * _CD_DURATION_INCREMENT,
                'count': 1,
                'parent': 'GlobalTimer',
                'children': [],
            },
        ],
    }


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_num_frames_empty():
    """A new SimulationStats starts with zero frames."""
    s = SimulationStats()
    assert s.num_frames == 0


@pytest.mark.basic
def test_num_frames_after_inject():
    """num_frames matches the number of manually injected entries."""
    s = SimulationStats()
    n = 5
    for i in range(n):
        s._frames.append(_make_timer_data(i))
    assert s.num_frames == n


@pytest.mark.basic
def test_get_values_duration():
    """get_values() returns correct duration array for a top-level timer."""
    n = 4
    s = SimulationStats()
    for i in range(n):
        s._frames.append(_make_timer_data(i))

    frames, durations = s.get_values('Newton', metric='duration')

    assert len(frames) == n
    np.testing.assert_array_equal(frames, np.arange(n))
    expected = np.array([_NEWTON_BASE_DURATION + i * _NEWTON_DURATION_INCREMENT for i in range(n)])
    np.testing.assert_allclose(durations, expected, rtol=1e-6)


@pytest.mark.basic
def test_get_values_count():
    """get_values() returns correct count array."""
    n = 3
    s = SimulationStats()
    for i in range(n):
        s._frames.append(_make_timer_data(i))

    _, counts = s.get_values('Newton', metric='count')
    expected = np.array([_NEWTON_BASE_COUNT + i for i in range(n)], dtype=float)
    np.testing.assert_allclose(counts, expected)


@pytest.mark.basic
def test_get_values_nested():
    """get_values() finds nested timers (PCG inside Newton)."""
    n = 3
    s = SimulationStats()
    for i in range(n):
        s._frames.append(_make_timer_data(i))

    _, counts = s.get_values('PCG', metric='count')
    expected = np.array([_PCG_BASE_COUNT + i * _PCG_COUNT_INCREMENT for i in range(n)], dtype=float)
    np.testing.assert_allclose(counts, expected)


@pytest.mark.basic
def test_get_values_missing_key():
    """get_values() returns empty arrays for an unknown timer name."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))

    frames, values = s.get_values('NonExistentTimer')
    assert len(frames) == 0
    assert len(values) == 0


@pytest.mark.basic
def test_to_markdown_row_count():
    """to_markdown() produces header + separator + one row per frame."""
    n = 3
    s = SimulationStats()
    for i in range(n):
        s._frames.append(_make_timer_data(i))

    md = s.to_markdown(['Newton', 'PCG'], metric='count')
    lines = md.strip().split('\n')
    # header + separator + n data rows
    assert len(lines) == n + 2


@pytest.mark.basic
def test_to_markdown_column_headers():
    """to_markdown() includes the requested column names in the header."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))

    md = s.to_markdown(['Newton', 'Collision Detection'], metric='count')
    header = md.strip().split('\n')[0]
    assert 'Newton' in header
    assert 'Collision Detection' in header


@pytest.mark.basic
def test_to_markdown_pipe_structure():
    """Every line in the markdown table starts with '|'."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))

    md = s.to_markdown(['Newton'], metric='duration')
    for line in md.strip().split('\n'):
        assert line.startswith('|'), f"Line does not start with '|': {line!r}"


@pytest.mark.basic
def test_to_markdown_duration_suffix():
    """Duration values in the markdown table end with 's'."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))

    md = s.to_markdown(['Newton'], metric='duration')
    data_row = md.strip().split('\n')[2]
    assert 's' in data_row


@pytest.mark.basic
def test_to_markdown_na_for_missing():
    """to_markdown() puts 'N/A' for timers absent in a frame."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))

    md = s.to_markdown(['Newton', 'NonExistent'], metric='count')
    data_row = md.strip().split('\n')[2]
    assert 'N/A' in data_row


@pytest.mark.basic
def test_get_values_invalid_metric():
    """get_values() raises ValueError for an invalid metric."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))
    with pytest.raises(ValueError, match="metric must be"):
        s.get_values('Newton', metric='invalid')


@pytest.mark.basic
def test_plot_invalid_metric():
    """plot() raises ValueError for an invalid metric."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))
    with pytest.raises(ValueError, match="metric must be"):
        s.plot('Newton', metric='invalid')


@pytest.mark.basic
def test_plot_invalid_kind():
    """plot() raises ValueError for an invalid kind."""
    s = SimulationStats()
    s._frames.append(_make_timer_data(0))
    with pytest.raises(ValueError, match="kind must be"):
        s.plot('Newton', kind='invalid')


@pytest.mark.basic
def test_to_markdown_auto_keys_excludes_root():
    """to_markdown(keys=None) auto-discovers keys and excludes the root node."""
    s = SimulationStats()
    for i in range(2):
        s._frames.append(_make_timer_data(i))

    md = s.to_markdown(keys=None, metric='count')
    header = md.strip().split('\n')[0]
    # Root timer name must not appear as a column
    assert 'GlobalTimer' not in header
    # Child timer names should be columns
    assert 'Newton' in header


@pytest.mark.basic
def test_summary_report_total_time_first_plot():
    """summary_report() includes 'Total Time Per Frame' as the first plot
    after the heatmap section."""
    import tempfile, os
    s = SimulationStats()
    for i in range(3):
        s._frames.append(_make_timer_data(i))

    with tempfile.TemporaryDirectory() as tmpdir:
        report_path = s.summary_report(keys=['Newton'], output_dir=tmpdir)
        md_text = report_path.read_text(encoding='utf-8')

    # 'Total Time Per Frame' must appear before 'Per-Frame Statistics'
    idx_total = md_text.find('Total Time Per Frame')
    idx_per_frame = md_text.find('Per-Frame Statistics')
    assert idx_total != -1, "'Total Time Per Frame' section missing from report"
    assert idx_per_frame != -1, "'Per-Frame Statistics' section missing from report"
    assert idx_total < idx_per_frame, (
        "'Total Time Per Frame' must appear before 'Per-Frame Statistics'"
    )
    # The SVG file must be referenced in the markdown
    assert 'total_time_per_frame.svg' in md_text
