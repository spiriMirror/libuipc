# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Fresh-process worker benchmark: one warmup, then repeated frozen-input trials."""
import argparse
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--inputs", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--repeats", type=int, default=3)
parser.add_argument("--current-schema", action="store_true", help="Upgrade only transport metadata, not physics")
args = parser.parse_args()
if args.repeats < 1:
    parser.error("--repeats must be positive")
worker = Path(__file__).resolve().parents[1] / "libuipc_blender" / "worker.py"
sys.path.insert(0, str(worker.parent))
from protocol import SCHEMA_VERSION, file_sha256

args.output.mkdir(parents=True, exist_ok=False)
report = {"python": sys.executable, "workloads": {}}
for source in sorted(args.inputs.iterdir()):
    if not (source / "request.json").is_file():
        continue
    trials = []
    for trial in range(args.repeats + 1):
        directory = args.output / source.name / str(trial)
        directory.mkdir(parents=True)
        for path in source.iterdir():
            if path.name == "request.json" or path.suffix == ".npz":
                shutil.copy2(path, directory / path.name)
        request = json.loads((directory / "request.json").read_text())
        if args.current_schema:
            request["schema_version"] = SCHEMA_VERSION
            (directory / "request.json").write_text(json.dumps(request))
        started = time.perf_counter()
        with (directory / "worker.log").open("w") as log:
            subprocess.run([sys.executable, str(worker), "--job", str(directory.resolve())],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        elapsed = time.perf_counter() - started
        result = json.loads((directory / "result.json").read_text())
        trials.append({"process_seconds": elapsed, "performance": result["performance"],
                       "build_info": result["build_info"], "fingerprint": result["fingerprint"]})
        print(source.name, trial, f"{elapsed:.3f} s", flush=True)
    report["workloads"][source.name] = {"warmup": trials[0], "trials": trials[1:],
        "input_hashes": {p.name: file_sha256(p) for p in source.glob("input_*.npz")},
        "median_process_seconds": statistics.median(t["process_seconds"] for t in trials[1:])}
(args.output / "summary.json").write_text(json.dumps(report, indent=2))
print("WORKER_BENCHMARK_OK", str(args.output / "summary.json"))
