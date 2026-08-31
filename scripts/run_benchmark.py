#!/usr/bin/env python3
"""Run versioned end-to-end benchmarks from benchmarks/manifest.json."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Mapping

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MANIFEST = REPO_ROOT / "benchmarks" / "manifest.json"


def repo_path(value: str) -> Path:
    path = (REPO_ROOT / value).resolve()
    if path != REPO_ROOT and REPO_ROOT not in path.parents:
        raise ValueError(f"path escapes repository: {value}")
    return path


def load_manifest(path: Path = DEFAULT_MANIFEST) -> dict[str, dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schemaVersion") != 1:
        raise ValueError("benchmark manifest schemaVersion must be 1")
    entries = payload.get("benchmarks")
    if not isinstance(entries, list) or not entries:
        raise ValueError("benchmark manifest must contain a non-empty list")

    registry: dict[str, dict[str, Any]] = {}
    required = {
        "name",
        "description",
        "entrypoint",
        "workingDirectory",
        "arguments",
        "defaultFrames",
        "quickFrames",
        "requiredPaths",
        "metadataOutput",
    }
    for entry in entries:
        if not isinstance(entry, dict) or not required <= entry.keys():
            raise ValueError("benchmark manifest entry is incomplete")
        name = entry["name"]
        if not isinstance(name, str) or not name:
            raise ValueError("benchmark name must be a non-empty string")
        if name in registry:
            raise ValueError(f"duplicate benchmark name: {name}")
        for field in ("defaultFrames", "quickFrames"):
            if not isinstance(entry[field], int) or entry[field] <= 0:
                raise ValueError(f"{name}.{field} must be a positive integer")
        if not isinstance(entry["arguments"], list) or not all(
            isinstance(arg, str) for arg in entry["arguments"]
        ):
            raise ValueError(f"{name}.arguments must be a string list")
        if not isinstance(entry["requiredPaths"], list) or not all(
            isinstance(value, str) for value in entry["requiredPaths"]
        ):
            raise ValueError(f"{name}.requiredPaths must be a string list")
        for field in ("entrypoint", "workingDirectory", "metadataOutput"):
            repo_path(entry[field])
        for value in entry["requiredPaths"]:
            repo_path(value)
        try:
            [argument.format(frames=1) for argument in entry["arguments"]]
        except (KeyError, ValueError) as error:
            raise ValueError(f"{name}.arguments has an invalid placeholder") from error
        registry[name] = entry
    return registry


def missing_required_paths(entry: Mapping[str, Any]) -> list[Path]:
    return [
        path
        for value in entry["requiredPaths"]
        if not (path := repo_path(value)).exists()
    ]


def parse_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        key, separator, setting = value.partition("=")
        if not separator or not key:
            raise ValueError(f"environment override must be KEY=VALUE: {value}")
        overrides[key] = setting
    return overrides


def build_environment(
    entry: Mapping[str, Any],
    overrides: Mapping[str, str],
    base_environment: Mapping[str, str] | None = None,
) -> dict[str, str]:
    environment = dict(os.environ if base_environment is None else base_environment)
    for key in entry.get("unsetEnvironment", []):
        environment.pop(key, None)
    environment.update(entry.get("environment", {}))
    environment.update(overrides)
    return environment


def resolve_python(value: str) -> str:
    path = Path(value)
    if path.exists():
        return str(path.resolve())
    resolved = shutil.which(value)
    if resolved is None:
        raise ValueError(f"Python executable not found: {value}")
    return resolved


def build_command(
    entry: Mapping[str, Any], python: str, frames: int
) -> tuple[list[str], Path]:
    if frames <= 0:
        raise ValueError("--frames must be greater than zero")
    entrypoint = repo_path(entry["entrypoint"])
    working_directory = repo_path(entry["workingDirectory"])
    arguments = [argument.format(frames=frames) for argument in entry["arguments"]]
    return [python, str(entrypoint), *arguments], working_directory


def git_state(path: Path) -> dict[str, Any]:
    def run(*arguments: str) -> str:
        result = subprocess.run(
            ["git", "-C", str(path), *arguments],
            capture_output=True,
            text=True,
            check=False,
        )
        return result.stdout.strip() if result.returncode == 0 else "unknown"

    status = run("status", "--porcelain")
    return {
        "commit": run("rev-parse", "HEAD"),
        "dirty": status not in ("", "unknown"),
    }


def display_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def command_output(
    command: list[str], environment: Mapping[str, str] | None = None
) -> str:
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            env=None if environment is None else dict(environment),
        )
    except OSError:
        return "unavailable"
    return result.stdout.strip() if result.returncode == 0 else "unavailable"


def runtime_facts(python: str, environment: Mapping[str, str]) -> dict[str, Any]:
    inherited_keys = ("CUDA_VISIBLE_DEVICES", "UIPC_ENGINE_DEFAULT_CONFIG")
    return {
        "platform": platform.platform(),
        "pythonVersion": command_output([python, "--version"], environment),
        "uipcVersion": command_output(
            [python, "-c", "import uipc; print(uipc.__version__)"], environment
        ),
        "gpu": command_output(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version",
                "--format=csv,noheader",
            ],
            environment,
        ),
        "cudaCompiler": command_output(["nvcc", "--version"], environment),
        "inheritedEnvironment": {
            key: environment[key] for key in inherited_keys if key in environment
        },
    }


def parse_reported_summary(output: str) -> dict[str, Any] | None:
    match = re.search(
        r"TOTAL frames=(\d+) mean=([0-9.]+)ms median=([0-9.]+)ms", output
    )
    if match is None:
        return None
    return {
        "frames": int(match.group(1)),
        "meanFrameMs": float(match.group(2)),
        "medianFrameMs": float(match.group(3)),
    }


def parse_reported_benchmark(output: str) -> dict[str, Any] | None:
    """Parse the last compact result emitted by a canonical sample."""
    prefix = "BENCHMARK_RESULT "
    lines = [line for line in output.splitlines() if line.startswith(prefix)]
    if not lines:
        return None
    payload = json.loads(lines[-1][len(prefix):])
    if not isinstance(payload, dict):
        raise ValueError("BENCHMARK_RESULT must be a JSON object")
    frame_ms = payload.get("frame_ms")
    frame_stats = payload.get("frame_stats")
    if not isinstance(frame_ms, list) or not frame_ms:
        raise ValueError("BENCHMARK_RESULT.frame_ms must be a non-empty list")
    if not isinstance(frame_stats, list) or len(frame_stats) != len(frame_ms):
        raise ValueError(
            "BENCHMARK_RESULT.frame_stats must match the frame_ms length"
        )
    if not all(
        isinstance(value, (int, float)) and not isinstance(value, bool)
        for value in frame_ms
    ):
        raise ValueError("BENCHMARK_RESULT.frame_ms must contain only numbers")
    return payload


def parse_gpu_memory_mib(output: str) -> list[int] | None:
    """Parse one total-memory row per GPU from nvidia-smi CSV output."""
    values: list[int] = []
    for line in output.splitlines():
        match = re.search(r"(\d+)", line)
        if match is None:
            return None
        values.append(int(match.group(1)))
    return values or None


def query_gpu_memory_mib(environment: Mapping[str, str]) -> list[int] | None:
    output = command_output(
        [
            "nvidia-smi",
            "--query-gpu=memory.used",
            "--format=csv,noheader,nounits",
        ],
        environment,
    )
    return None if output == "unavailable" else parse_gpu_memory_mib(output)


class GpuMemoryMonitor:
    """Best-effort total GPU-memory sampler for an isolated benchmark run."""

    def __init__(self, environment: Mapping[str, str], interval: float = 0.2):
        self._environment = environment
        self._interval = interval
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._baseline = query_gpu_memory_mib(environment)
        self._peak = None if self._baseline is None else list(self._baseline)
        self._samples = 1 if self._baseline is not None else 0

    def start(self) -> None:
        if self._baseline is None:
            return
        self._thread = threading.Thread(target=self._sample, daemon=True)
        self._thread.start()

    def _sample(self) -> None:
        while not self._stop.wait(self._interval):
            values = query_gpu_memory_mib(self._environment)
            if values is None or self._peak is None or len(values) != len(self._peak):
                continue
            self._peak = [max(old, new) for old, new in zip(self._peak, values)]
            self._samples += 1

    def stop(self) -> dict[str, Any] | None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._interval * 2.0))
        if self._baseline is None or self._peak is None:
            return None
        return {
            "semantics": "total device memory; peak delta includes concurrent WDDM/display activity",
            "baselineTotalMiB": self._baseline,
            "peakTotalMiB": self._peak,
            "peakDeltaMiB": [
                peak - baseline
                for baseline, peak in zip(self._baseline, self._peak)
            ],
            "samples": self._samples,
            "sampleIntervalSeconds": self._interval,
        }


def write_metadata(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def run_benchmark(args: argparse.Namespace, registry: Mapping[str, dict[str, Any]]) -> int:
    if args.name not in registry:
        raise ValueError(f"unknown benchmark: {args.name}")
    entry = registry[args.name]
    missing = missing_required_paths(entry)
    if missing:
        paths = "\n  ".join(str(path) for path in missing)
        raise ValueError(
            "benchmark assets are missing; run git submodule update --init --recursive:\n  "
            + paths
        )

    frames = entry["quickFrames"] if args.quick else (
        args.frames if args.frames is not None else entry["defaultFrames"]
    )
    python = resolve_python(args.python)
    overrides = parse_overrides(args.env)
    command, working_directory = build_command(entry, python, frames)
    environment = build_environment(entry, overrides)

    print(f"benchmark: {args.name}")
    print(f"frames:    {frames}")
    print(f"cwd:       {working_directory}")
    print(f"command:   {display_command(command)}")
    if overrides:
        print(f"overrides: {overrides}")
    if args.dry_run:
        return 0
    sys.stdout.flush()

    started = dt.datetime.now(dt.timezone.utc)
    start_time = time.perf_counter()
    output_lines: list[str] = []
    memory_monitor = GpuMemoryMonitor(environment)
    memory_monitor.start()
    try:
        process = subprocess.Popen(
            command,
            cwd=working_directory,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            output_lines.append(line)
        return_code = process.wait()
    except KeyboardInterrupt:
        if "process" in locals() and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        return_code = 130
    gpu_memory = memory_monitor.stop()
    duration = time.perf_counter() - start_time

    output = "".join(output_lines)
    reported_benchmark = parse_reported_benchmark(output)
    reported_timing = parse_reported_summary(output)
    if reported_benchmark is not None:
        frame_ms = [float(value) for value in reported_benchmark["frame_ms"]]
        reported_timing = {
            "frames": len(frame_ms),
            "meanFrameMs": statistics.mean(frame_ms),
            "medianFrameMs": statistics.median(frame_ms),
            "p95FrameMs": sorted(frame_ms)[round(0.95 * (len(frame_ms) - 1))],
        }

    metadata = {
        "schemaVersion": 1,
        "benchmark": args.name,
        "description": entry["description"],
        "frames": frames,
        "startedAt": started.isoformat(),
        "runId": started.strftime("%Y%m%dT%H%M%SZ"),
        "durationSeconds": duration,
        "returnCode": return_code,
        "command": command,
        "workingDirectory": str(working_directory),
        "environment": {
            "canonical": entry.get("environment", {}),
            "unset": entry.get("unsetEnvironment", []),
            "overrides": overrides,
        },
        "revisions": {
            "libuipc": git_state(REPO_ROOT),
            "libuipc-samples": git_state(REPO_ROOT / "libuipc-samples"),
        },
        "python": python,
        "runtime": runtime_facts(python, environment),
        "reportedFrameTiming": reported_timing,
        "reportedBenchmark": reported_benchmark,
        "gpuMemory": gpu_memory,
    }
    metadata_path = repo_path(entry["metadataOutput"])
    archive_path = metadata_path.with_name(
        f"{metadata_path.stem}-{metadata['runId']}{metadata_path.suffix}"
    )
    log_path = archive_path.with_suffix(".log")
    log_path.write_text(output, encoding="utf-8")
    write_metadata(archive_path, metadata)
    write_metadata(metadata_path, metadata)
    print(f"metadata:  {archive_path}")
    print(f"log:       {log_path}")
    print(f"latest:    {metadata_path}")
    return return_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="list registered project benchmarks")
    run_parser = subparsers.add_parser("run", help="run one project benchmark")
    run_parser.add_argument("name")
    frames = run_parser.add_mutually_exclusive_group()
    frames.add_argument("--frames", type=int)
    frames.add_argument("--quick", action="store_true")
    run_parser.add_argument("--python", default=sys.executable)
    run_parser.add_argument("--env", action="append", default=[], metavar="KEY=VALUE")
    run_parser.add_argument("--dry-run", action="store_true")

    args = parser.parse_args(argv)
    try:
        registry = load_manifest(args.manifest)
        if args.command == "list":
            for name, entry in registry.items():
                status = "ready" if not missing_required_paths(entry) else "missing assets"
                print(
                    f"{name:24} {status:14} frames={entry['defaultFrames']:4d}  "
                    f"{entry['description']}"
                )
            return 0
        return run_benchmark(args, registry)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
