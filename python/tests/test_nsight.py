"""Tests for uipc.profile.nsight module.

Tests the pure-Python logic (CSV parsing, kernel name shortening,
ncu discovery, bottleneck analysis) without requiring ncu to be installed.
"""
import os
import pathlib
import textwrap

import pytest


# ---------------------------------------------------------------------------
# Bootstrap: load nsight module directly (avoids needing native uipc)
# ---------------------------------------------------------------------------

import importlib.util
import sys
import types

_NSIGHT_PATH = (
    pathlib.Path(__file__).parent.parent / 'src' / 'uipc' / 'profile' / 'nsight.py'
)


def _load_nsight():
    """Load uipc/profile/nsight.py with minimal stubs."""
    # Ensure uipc stub exists
    if 'uipc' not in sys.modules:
        stub = types.ModuleType('uipc')
        stub.__path__ = []
        sys.modules['uipc'] = stub

    if 'uipc.profile' not in sys.modules:
        profile_stub = types.ModuleType('uipc.profile')
        profile_stub.__path__ = []
        sys.modules['uipc.profile'] = profile_stub

    spec = importlib.util.spec_from_file_location(
        'uipc.profile.nsight', str(_NSIGHT_PATH)
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules['uipc.profile.nsight'] = mod
    spec.loader.exec_module(mod)
    return mod


_nsight = _load_nsight()
find_ncu = _nsight.find_ncu
parse_ncu_csv = _nsight.parse_ncu_csv
analyze_bottlenecks = _nsight.analyze_bottlenecks
_shorten_kernel_name = _nsight._shorten_kernel_name
_build_kernel_summary = _nsight._build_kernel_summary


# ---------------------------------------------------------------------------
# find_ncu
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_find_ncu_from_env(monkeypatch, tmp_path):
    """find_ncu() finds ncu via NCU_PATH env var."""
    fake_ncu = tmp_path / 'ncu'
    fake_ncu.write_text('fake')
    monkeypatch.setenv('NCU_PATH', str(fake_ncu))
    assert find_ncu() == str(fake_ncu)


@pytest.mark.basic
def test_find_ncu_env_missing_file(monkeypatch):
    """find_ncu() skips NCU_PATH if the file doesn't exist."""
    monkeypatch.setenv('NCU_PATH', '/nonexistent/ncu')
    monkeypatch.setattr('shutil.which', lambda _: None)
    # Should not raise, just return None (or find via PATH/well-known)
    result = find_ncu()
    # We can't assert None because ncu might actually be installed;
    # we just verify it doesn't return the nonexistent path
    assert result != '/nonexistent/ncu'


@pytest.mark.basic
def test_find_ncu_from_path(monkeypatch, tmp_path):
    """find_ncu() finds ncu on PATH via shutil.which."""
    monkeypatch.delenv('NCU_PATH', raising=False)
    fake_ncu = str(tmp_path / 'ncu')
    monkeypatch.setattr('shutil.which', lambda name: fake_ncu if name == 'ncu' else None)
    assert find_ncu() == fake_ncu


# ---------------------------------------------------------------------------
# _shorten_kernel_name
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_shorten_muda_parallel_for():
    """Extracts Class::method from parallel_for_kernel<...> names."""
    name = 'void muda::parallel_for_kernel<StacklessBVH::calcExtNodeSplitMetrics()::$_0>(int, StacklessBVH::calcExtNodeSplitMetrics()::$_0)'
    assert _shorten_kernel_name(name) == 'StacklessBVH::calcExtNodeSplitMetrics'


@pytest.mark.basic
def test_shorten_muda_instance():
    """Preserves instance numbers for multiple lambdas."""
    name = 'void muda::parallel_for_kernel<Filter::detect()::$_0>(int, Filter::detect()::$_0) (instance 2)'
    result = _shorten_kernel_name(name)
    assert result == 'Filter::detect#2'


@pytest.mark.basic
def test_shorten_generic_kernel():
    """Tags generic_kernel names with [generic]."""
    name = 'void muda::generic_kernel<Foo::bar()::$_0>(int, Foo::bar()::$_0)'
    result = _shorten_kernel_name(name)
    assert result == '[generic] Foo::bar'


@pytest.mark.basic
def test_shorten_buffer_kernel():
    """Extracts buffer operation names."""
    name = 'void muda::buffer::kernel_fill<int>(int*, int, unsigned long)'
    result = _shorten_kernel_name(name)
    assert 'buffer::kernel_fill' in result


@pytest.mark.basic
def test_shorten_cub_kernel():
    """Extracts CUB kernel names."""
    name = 'void cub::DeviceReduceKernel<...>()'
    result = _shorten_kernel_name(name)
    assert result.startswith('cub::')


@pytest.mark.basic
def test_shorten_fallback():
    """Falls back gracefully for unknown kernel names."""
    name = 'some_random_kernel_name'
    result = _shorten_kernel_name(name)
    assert len(result) > 0
    assert len(result) <= 60


# ---------------------------------------------------------------------------
# parse_ncu_csv
# ---------------------------------------------------------------------------

_SAMPLE_CSV = textwrap.dedent("""\
    "ID","Kernel Name","launch__registers_per_thread","gpu__time_duration.sum","sm__maximum_warps_per_active_cycle_pct","launch__block_size","launch__grid_size"
    "","","","nsec","%","","",
    "1","void muda::parallel_for_kernel<Foo::bar()::$_0>(int)","32","1500.0","45.2","256","128"
    "2","void muda::parallel_for_kernel<Baz::qux()::$_0>(int)","64","3200.0","78.1","128","256"
    "3","void muda::parallel_for_kernel<Foo::bar()::$_0>(int)","32","1800.0","50.0","256","128"
""")


@pytest.mark.basic
def test_parse_ncu_csv_basic(tmp_path):
    """parse_ncu_csv parses a sample CSV correctly."""
    csv_file = tmp_path / 'test.csv'
    csv_file.write_text(_SAMPLE_CSV, encoding='utf-8')

    rows = parse_ncu_csv(str(csv_file))
    assert len(rows) == 3

    # First row
    assert rows[0]['launch__registers_per_thread'] == 32
    assert rows[0]['gpu__time_duration.sum'] == 1500.0  # already ns
    assert rows[0]['launch__block_size'] == 256


@pytest.mark.basic
def test_parse_ncu_csv_units_conversion(tmp_path):
    """parse_ncu_csv converts microsecond durations to nanoseconds."""
    csv = textwrap.dedent("""\
        "ID","Kernel Name","gpu__time_duration.sum"
        "","","usec"
        "1","kernel_a","1.5"
        "2","kernel_b","3.2"
    """)
    csv_file = tmp_path / 'test_us.csv'
    csv_file.write_text(csv, encoding='utf-8')

    rows = parse_ncu_csv(str(csv_file))
    assert len(rows) == 2
    assert rows[0]['gpu__time_duration.sum'] == pytest.approx(1500.0)
    assert rows[1]['gpu__time_duration.sum'] == pytest.approx(3200.0)


@pytest.mark.basic
def test_parse_ncu_csv_skips_comment_lines(tmp_path):
    """parse_ncu_csv skips lines starting with ==."""
    csv = '==PROF== some ncu header\n' + _SAMPLE_CSV
    csv_file = tmp_path / 'test_comments.csv'
    csv_file.write_text(csv, encoding='utf-8')

    rows = parse_ncu_csv(str(csv_file))
    assert len(rows) == 3


@pytest.mark.basic
def test_parse_ncu_csv_empty(tmp_path):
    """parse_ncu_csv returns empty list for empty file."""
    csv_file = tmp_path / 'empty.csv'
    csv_file.write_text('', encoding='utf-8')
    assert parse_ncu_csv(str(csv_file)) == []


@pytest.mark.basic
def test_parse_ncu_csv_thousands_separator(tmp_path):
    """parse_ncu_csv handles thousand separators (1,024)."""
    csv = textwrap.dedent("""\
        "ID","Kernel Name","gpu__time_duration.sum"
        "","","nsec"
        "1","kernel_a","1,500"
    """)
    csv_file = tmp_path / 'test_comma.csv'
    csv_file.write_text(csv, encoding='utf-8')

    rows = parse_ncu_csv(str(csv_file))
    assert rows[0]['gpu__time_duration.sum'] == 1500


# ---------------------------------------------------------------------------
# _build_kernel_summary
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_build_kernel_summary(tmp_path):
    """_build_kernel_summary aggregates kernels by shortened name."""
    csv_file = tmp_path / 'test.csv'
    csv_file.write_text(_SAMPLE_CSV, encoding='utf-8')
    rows = parse_ncu_csv(str(csv_file))

    summary = _build_kernel_summary(rows)
    assert summary['total_kernels_profiled'] == 3
    assert summary['unique_kernel_types'] == 2  # Foo::bar and Baz::qux

    kernels = summary['kernels']
    assert len(kernels) == 2

    # Should be sorted by total duration descending
    # Baz::qux: 3200ns, Foo::bar: 1500+1800=3300ns
    assert 'Foo::bar' in kernels[0]['name']
    assert kernels[0]['launches'] == 2
    assert kernels[0]['total_duration_ns'] == pytest.approx(3300.0)


# ---------------------------------------------------------------------------
# analyze_bottlenecks
# ---------------------------------------------------------------------------

@pytest.mark.basic
def test_analyze_bottlenecks_ncu_only(tmp_path):
    """analyze_bottlenecks produces Markdown from ncu rows."""
    csv_file = tmp_path / 'test.csv'
    csv_file.write_text(_SAMPLE_CSV, encoding='utf-8')
    rows = parse_ncu_csv(str(csv_file))

    md = analyze_bottlenecks(ncu_rows=rows, top_n=5)
    assert '# GPU Bottleneck Analysis' in md
    assert 'Kernel Hotspots' in md
    assert 'Foo::bar' in md


@pytest.mark.basic
def test_analyze_bottlenecks_empty():
    """analyze_bottlenecks handles no data gracefully."""
    md = analyze_bottlenecks()
    assert 'No data provided' in md
