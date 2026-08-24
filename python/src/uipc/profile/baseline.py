"""Versioned performance baselines for :mod:`uipc.profile` artifacts.

The baseline format deliberately stores measurements and compatibility facts,
not machine-specific result paths or timestamps. This keeps a baseline diff
reviewable and makes ``create -> check`` suitable for a dedicated, stable GPU
runner without pretending that timings from unrelated machines are comparable.
"""

from __future__ import annotations

import json
import math
import pathlib
from collections.abc import Iterable
from os import PathLike
from typing import Any


SCHEMA_VERSION = 1

_METRIC_SPECS = {
    'wall_time_ms_per_frame': {'unit': 'ms/frame', 'enforce': True},
    'timer_median_ms': {'unit': 'ms/frame', 'enforce': True},
    'timer_p95_ms': {'unit': 'ms/frame', 'enforce': True},
    'newton_iterations_median': {'unit': 'iterations/frame', 'enforce': False},
}

_ENVIRONMENT_KEYS = (
    'backend',
    'uipc_version',
    'python_abi',
    'build_type',
    'python_version',
    'python_implementation',
    'platform_system',
    'platform_machine',
    'cuda_toolkit_version',
    'cuda_architectures',
    'nvidia_gpus',
)


def _normalise_result_dirs(
    result_dirs: str | PathLike[str] | Iterable[str | PathLike[str]],
) -> list[pathlib.Path]:
    if isinstance(result_dirs, (str, PathLike)):
        paths = [pathlib.Path(result_dirs)]
    else:
        paths = [pathlib.Path(path) for path in result_dirs]
    if not paths:
        raise ValueError('At least one benchmark result directory is required')
    return paths


def _percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError('Cannot compute a percentile of an empty sequence')
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _timer_metric(node: dict[str, Any], name: str, metric: str) -> float:
    value = float(node.get(metric, 0.0)) if node.get('name') == name else 0.0
    for child in node.get('children', []):
        if isinstance(child, dict):
            value += _timer_metric(child, name, metric)
    return value


def result_metrics(result: dict[str, Any]) -> dict[str, float]:
    """Extract stable gate and diagnostic metrics from one loaded result."""
    frame_count = int(result.get('num_frames', 0))
    wall_time = float(result.get('wall_time', 0.0))
    if frame_count <= 0:
        raise ValueError('Benchmark num_frames must be greater than zero')
    if not math.isfinite(wall_time) or wall_time < 0.0:
        raise ValueError('Benchmark wall_time must be finite and non-negative')

    metrics = {'wall_time_ms_per_frame': wall_time * 1000.0 / frame_count}
    timer_durations: list[float] = []
    newton_iterations: list[float] = []
    timer_frames = result.get('timer_frames', [])
    if timer_frames and len(timer_frames) != frame_count:
        raise ValueError(
            'timer_frames length must match benchmark num_frames '
            f'({len(timer_frames)} != {frame_count})'
        )
    for frame in timer_frames:
        if not isinstance(frame, dict):
            raise ValueError('Every timer frame must be a JSON object')
        duration = frame.get('duration')
        if not isinstance(duration, (int, float)):
            raise ValueError('Every timer frame must have a numeric root duration')
        duration = float(duration)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError('Timer root durations must be finite and non-negative')
        timer_durations.append(duration * 1000.0)
        newton_iterations.append(_timer_metric(frame, 'Newton Iteration', 'count'))

    if timer_durations:
        metrics['timer_median_ms'] = _percentile(timer_durations, 0.5)
        metrics['timer_p95_ms'] = _percentile(timer_durations, 0.95)
    if newton_iterations and any(value > 0.0 for value in newton_iterations):
        metrics['newton_iterations_median'] = _percentile(newton_iterations, 0.5)

    return metrics


def _load_results(result_dirs) -> dict[str, dict[str, Any]]:
    results: dict[str, dict[str, Any]] = {}
    for result_dir in _normalise_result_dirs(result_dirs):
        metadata_path = result_dir / 'benchmark.json'
        if not metadata_path.exists():
            raise FileNotFoundError(f'No benchmark.json in {result_dir}')
        result = json.loads(metadata_path.read_text(encoding='utf-8'))
        frames_path = result_dir / 'timer_frames.json'
        if frames_path.exists():
            result['timer_frames'] = json.loads(
                frames_path.read_text(encoding='utf-8')
            )
        name = str(result.get('name', '')).strip()
        if not name:
            raise ValueError(f'Benchmark result has no name: {result_dir}')
        if name in results:
            raise ValueError(f'Duplicate benchmark name: {name}')
        results[name] = result
    return results


def _validate_regression_percent(value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError('max_regression_percent must be finite and non-negative')
    return value


def create_baseline(
    result_dirs,
    output_path: str | PathLike[str],
    *,
    max_regression_percent: float = 10.0,
) -> dict[str, Any]:
    """Create and save a deterministic baseline from benchmark result folders."""
    tolerance = _validate_regression_percent(max_regression_percent)
    results = _load_results(result_dirs)
    benchmarks: dict[str, Any] = {}

    for name in sorted(results):
        result = results[name]
        measurements = result_metrics(result)
        metrics = {}
        for metric_name, value in measurements.items():
            spec = _METRIC_SPECS[metric_name]
            metrics[metric_name] = {
                'value': value,
                'unit': spec['unit'],
                'lower_is_better': True,
                'enforce': spec['enforce'],
                'max_regression_percent': tolerance,
            }

        benchmarks[name] = {
            'num_frames': int(result['num_frames']),
            'steps': result.get('steps', []),
            'environment': {
                key: result.get('environment', {}).get(key)
                for key in _ENVIRONMENT_KEYS
            },
            'metrics': metrics,
        }

    payload = {
        'schema_version': SCHEMA_VERSION,
        'benchmarks': benchmarks,
    }
    path = pathlib.Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    return payload


def _environment_mismatches(
    expected: dict[str, Any], actual: dict[str, Any]
) -> list[dict[str, Any]]:
    mismatches = []
    for key in _ENVIRONMENT_KEYS:
        expected_value = expected.get(key)
        if expected_value is None:
            continue
        actual_value = actual.get(key)
        if actual_value != expected_value:
            mismatches.append(
                {'key': key, 'expected': expected_value, 'actual': actual_value}
            )
    return mismatches


def check_baseline(
    result_dirs,
    baseline_path: str | PathLike[str],
    *,
    max_regression_percent: float | None = None,
    require_environment_match: bool = True,
) -> dict[str, Any]:
    """Compare result folders with a baseline and return a structured report."""
    override_tolerance = (
        None
        if max_regression_percent is None
        else _validate_regression_percent(max_regression_percent)
    )
    path = pathlib.Path(baseline_path)
    baseline = json.loads(path.read_text(encoding='utf-8'))
    if baseline.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported baseline schema_version: {baseline.get('schema_version')!r}"
        )
    expected_benchmarks = baseline.get('benchmarks')
    if not isinstance(expected_benchmarks, dict) or not expected_benchmarks:
        raise ValueError('Baseline must contain a non-empty benchmarks object')

    results = _load_results(result_dirs)
    errors: list[str] = []
    reports: dict[str, Any] = {}

    missing = sorted(set(expected_benchmarks) - set(results))
    extra = sorted(set(results) - set(expected_benchmarks))
    errors.extend(f'Missing benchmark result: {name}' for name in missing)
    errors.extend(f'Benchmark is absent from baseline: {name}' for name in extra)

    for name in sorted(set(expected_benchmarks) & set(results)):
        expected = expected_benchmarks[name]
        if not isinstance(expected, dict):
            raise ValueError(f'Baseline benchmark {name} must be a JSON object')
        expected_metrics = expected.get('metrics')
        if not isinstance(expected_metrics, dict) or not expected_metrics:
            raise ValueError(
                f'Baseline benchmark {name} must contain non-empty metrics'
            )
        result = results[name]
        benchmark_errors: list[str] = []

        expected_frames = int(expected.get('num_frames', 0))
        actual_frames = int(result.get('num_frames', 0))
        if actual_frames != expected_frames:
            benchmark_errors.append(
                f'num_frames differs: expected {expected_frames}, got {actual_frames}'
            )

        expected_steps = expected.get('steps', [])
        actual_steps = result.get('steps', [])
        if actual_steps != expected_steps:
            benchmark_errors.append(
                f'steps differ: expected {expected_steps!r}, got {actual_steps!r}'
            )

        mismatches = _environment_mismatches(
            expected.get('environment', {}), result.get('environment', {})
        )
        if require_environment_match:
            benchmark_errors.extend(
                f"environment {item['key']} differs: expected "
                f"{item['expected']!r}, got {item['actual']!r}"
                for item in mismatches
            )

        actual_metrics = result_metrics(result)
        metric_reports: dict[str, Any] = {}
        for metric_name, metric_baseline in expected_metrics.items():
            if metric_name not in _METRIC_SPECS:
                raise ValueError(f'Unsupported baseline metric: {metric_name}')
            baseline_value = float(metric_baseline['value'])
            if not math.isfinite(baseline_value) or baseline_value < 0.0:
                raise ValueError(
                    f'Baseline metric {metric_name} must be finite and non-negative'
                )
            if metric_baseline.get('lower_is_better', True) is not True:
                raise ValueError(
                    f'Baseline metric {metric_name} has an unsupported direction'
                )
            actual_value = actual_metrics.get(metric_name)
            enforce = bool(metric_baseline.get('enforce', True))
            tolerance = (
                override_tolerance
                if override_tolerance is not None
                else _validate_regression_percent(
                    metric_baseline.get('max_regression_percent', 0.0)
                )
            )
            limit = baseline_value * (1.0 + tolerance / 100.0)
            metric_passed = (
                not enforce
                if actual_value is None
                else (not enforce or actual_value <= limit)
            )
            if actual_value is None:
                regression_percent = None
                if enforce:
                    benchmark_errors.append(
                        f'Missing enforced metric: {metric_name}'
                    )
            elif baseline_value == 0.0:
                regression_percent = 0.0 if actual_value == 0.0 else None
            else:
                regression_percent = (
                    (actual_value - baseline_value) / baseline_value * 100.0
                )

            metric_reports[metric_name] = {
                'baseline': baseline_value,
                'actual': actual_value,
                'limit': limit,
                'unit': metric_baseline.get('unit', ''),
                'regression_percent': regression_percent,
                'enforce': enforce,
                'passed': metric_passed,
            }

        reports[name] = {
            'passed': not benchmark_errors
            and all(item['passed'] for item in metric_reports.values()),
            'errors': benchmark_errors,
            'environment_mismatches': mismatches,
            'metrics': metric_reports,
        }

    passed = not errors and all(item['passed'] for item in reports.values())
    return {
        'schema_version': SCHEMA_VERSION,
        'passed': passed,
        'baseline': str(path),
        'errors': errors,
        'benchmarks': reports,
    }


def format_check_report(report: dict[str, Any]) -> str:
    """Format :func:`check_baseline` output for terminals and CI logs."""
    lines = [
        'Performance baseline check: '
        + ('PASS' if report.get('passed') else 'FAIL')
    ]
    for error in report.get('errors', []):
        lines.append(f'  ERROR {error}')
    for name, benchmark in report.get('benchmarks', {}).items():
        lines.append(f"  {'PASS' if benchmark['passed'] else 'FAIL'} {name}")
        for error in benchmark.get('errors', []):
            lines.append(f'    ERROR {error}')
        for metric_name, metric in benchmark.get('metrics', {}).items():
            status = 'PASS' if metric['passed'] else 'FAIL'
            marker = '' if metric['enforce'] else ' (diagnostic)'
            actual = metric['actual']
            actual_text = 'missing' if actual is None else f'{actual:.6g}'
            lines.append(
                f"    {status} {metric_name}: {actual_text} <= "
                f"{metric['limit']:.6g} {metric['unit']}{marker}"
            )
    return '\n'.join(lines)


__all__ = [
    'SCHEMA_VERSION',
    'result_metrics',
    'create_baseline',
    'check_baseline',
    'format_check_report',
]
