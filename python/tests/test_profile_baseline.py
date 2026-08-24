"""Tests for deterministic benchmark baselines and regression gates."""

import json

import pytest

from uipc.profile import check_baseline, create_baseline
from uipc.profile.baseline import format_check_report, result_metrics


_ENVIRONMENT = {
    'backend': 'cuda',
    'uipc_version': '0.9.0',
    'python_abi': 'cp314',
    'build_type': 'Release',
    'python_version': '3.14',
    'python_implementation': 'CPython',
    'platform_system': 'Windows',
    'platform_machine': 'AMD64',
    'cuda_toolkit_version': '13.2',
    'cuda_architectures': '89',
    'nvidia_gpus': [
        {
            'name': 'NVIDIA GeForce RTX 5090',
            'compute_capability': '12.0',
            'driver_version': '999.0',
        }
    ],
}


def _write_result(
    root,
    *,
    name='scene',
    frame_ms=(100.0, 110.0, 90.0),
    wall_ms=None,
    newton_counts=(3, 4, 3),
    environment=None,
    steps=None,
):
    root.mkdir(parents=True)
    if wall_ms is None:
        wall_ms = sum(frame_ms)
    if steps is None:
        steps = [['advance', 2], ['profile', len(frame_ms)]]
    metadata = {
        'schema_version': 1,
        'name': name,
        'num_frames': len(frame_ms),
        'wall_time': wall_ms / 1000.0,
        'summary': name,
        'workspace': None,
        'steps': steps,
        'environment': environment or dict(_ENVIRONMENT),
    }
    frames = []
    for duration_ms, newton_count in zip(frame_ms, newton_counts):
        frames.append(
            {
                'name': 'GlobalTimer',
                'duration': duration_ms / 1000.0,
                'count': 1,
                'children': [
                    {
                        'name': 'Newton Iteration',
                        'duration': duration_ms / 2000.0,
                        'count': newton_count,
                        'children': [],
                    }
                ],
            }
        )
    (root / 'benchmark.json').write_text(
        json.dumps(metadata, indent=2), encoding='utf-8'
    )
    (root / 'timer_frames.json').write_text(
        json.dumps(frames, indent=2), encoding='utf-8'
    )
    return root


@pytest.mark.basic
def test_result_metrics_extracts_time_and_solver_diagnostics(tmp_path):
    result_dir = _write_result(tmp_path / 'result')
    metadata = json.loads((result_dir / 'benchmark.json').read_text())
    metadata['timer_frames'] = json.loads(
        (result_dir / 'timer_frames.json').read_text()
    )

    metrics = result_metrics(metadata)
    assert metrics['wall_time_ms_per_frame'] == pytest.approx(100.0)
    assert metrics['timer_median_ms'] == pytest.approx(100.0)
    assert metrics['timer_p95_ms'] == pytest.approx(109.0)
    assert metrics['newton_iterations_median'] == pytest.approx(3.0)


@pytest.mark.basic
def test_baseline_round_trip_passes_within_tolerance(tmp_path):
    baseline_result = _write_result(tmp_path / 'baseline_result')
    candidate = _write_result(
        tmp_path / 'candidate',
        frame_ms=(105.0, 105.0, 105.0),
        wall_ms=315.0,
        newton_counts=(20, 20, 20),
    )
    baseline_path = tmp_path / 'baseline.json'
    payload = create_baseline(
        [baseline_result], baseline_path, max_regression_percent=10.0
    )

    report = check_baseline([candidate], baseline_path)
    assert payload['schema_version'] == 1
    assert report['passed']
    assert report['benchmarks']['scene']['metrics'][
        'newton_iterations_median'
    ]['passed']
    assert '(diagnostic)' in format_check_report(report)


@pytest.mark.basic
def test_baseline_fails_enforced_time_regression(tmp_path):
    baseline_result = _write_result(tmp_path / 'baseline_result')
    candidate = _write_result(
        tmp_path / 'candidate',
        frame_ms=(125.0, 125.0, 125.0),
        wall_ms=375.0,
    )
    baseline_path = tmp_path / 'baseline.json'
    create_baseline([baseline_result], baseline_path, max_regression_percent=10)

    report = check_baseline([candidate], baseline_path)
    assert not report['passed']
    assert not report['benchmarks']['scene']['metrics'][
        'wall_time_ms_per_frame'
    ]['passed']
    assert 'Performance baseline check: FAIL' in format_check_report(report)


@pytest.mark.basic
def test_check_override_tolerance(tmp_path):
    baseline_result = _write_result(tmp_path / 'baseline_result')
    candidate = _write_result(
        tmp_path / 'candidate',
        frame_ms=(115.0, 115.0, 115.0),
        wall_ms=345.0,
    )
    baseline_path = tmp_path / 'baseline.json'
    create_baseline([baseline_result], baseline_path, max_regression_percent=10)

    assert not check_baseline([candidate], baseline_path)['passed']
    assert check_baseline(
        [candidate], baseline_path, max_regression_percent=20
    )['passed']


@pytest.mark.basic
def test_environment_mismatch_is_explicit_and_overridable(tmp_path):
    baseline_result = _write_result(tmp_path / 'baseline_result')
    changed_environment = dict(_ENVIRONMENT)
    changed_environment['cuda_toolkit_version'] = '12.8'
    candidate = _write_result(
        tmp_path / 'candidate', environment=changed_environment
    )
    baseline_path = tmp_path / 'baseline.json'
    create_baseline([baseline_result], baseline_path)

    strict = check_baseline([candidate], baseline_path)
    relaxed = check_baseline(
        [candidate], baseline_path, require_environment_match=False
    )
    assert not strict['passed']
    assert relaxed['passed']
    mismatch = relaxed['benchmarks']['scene']['environment_mismatches'][0]
    assert mismatch['key'] == 'cuda_toolkit_version'


@pytest.mark.basic
def test_baseline_requires_matching_frame_plan(tmp_path):
    baseline_result = _write_result(tmp_path / 'baseline_result')
    candidate = _write_result(
        tmp_path / 'candidate',
        frame_ms=(100.0, 110.0),
        newton_counts=(3, 4),
        steps=[['profile', 2]],
    )
    baseline_path = tmp_path / 'baseline.json'
    create_baseline([baseline_result], baseline_path)

    report = check_baseline([candidate], baseline_path)
    assert not report['passed']
    errors = report['benchmarks']['scene']['errors']
    assert any('num_frames differs' in error for error in errors)
    assert any('steps differ' in error for error in errors)


@pytest.mark.basic
def test_baseline_output_is_deterministic(tmp_path):
    result_dir = _write_result(tmp_path / 'result')
    first = tmp_path / 'first.json'
    second = tmp_path / 'second.json'
    create_baseline([result_dir], first)
    create_baseline([result_dir], second)
    assert first.read_bytes() == second.read_bytes()


@pytest.mark.basic
def test_baseline_rejects_unknown_schema(tmp_path):
    result_dir = _write_result(tmp_path / 'result')
    baseline = tmp_path / 'baseline.json'
    baseline.write_text(
        json.dumps({'schema_version': 99, 'benchmarks': {'scene': {}}}),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='schema_version'):
        check_baseline([result_dir], baseline)
