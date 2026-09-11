# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Compare frozen inputs, native configuration and every vertex of every frame."""
import argparse
import json
from pathlib import Path
import statistics
import struct

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--before", type=Path, required=True)
parser.add_argument("--after", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()


def read(path):
    return json.loads(path.read_text())


def sample(stream, frame, output, vertices):
    stream.seek(0)
    frames, count = struct.unpack(">ii", stream.read(8))
    stream.seek(8 + 4*frames + min(frame, frames-1)*count*12)
    points = np.frombuffer(stream.read(count*12), dtype=">f4").reshape(count, 3)
    return vertices @ points[1:] + points[0] if output.get("encoding") == "AFFINE" else points


old, new = read(args.before / "summary.json"), read(args.after / "summary.json")
report = {}
for name, first in old["workloads"].items():
    second = new["workloads"][name]
    assert first["input_hashes"] == second["input_hashes"]
    assert len(first["trials"]) == len(second["trials"])
    maximum_error, changed, vertex_samples = 0., 0, 0
    for trial in range(1, len(first["trials"]) + 1):
        directories = [base / name / str(trial) for base in (args.before, args.after)]
        results = [read(d / "result.json") for d in directories]
        for key in ("fingerprint", "build_info", "effective_newton", "effective_linear_system",
                    "effective_line_search", "effective_contacts", "cloth_stiffness", "frames"):
            assert results[0][key] == results[1][key], (name, key)
        assert [(o["index"],o["vertices"]) for o in results[0]["objects"]] == [
            (o["index"],o["vertices"]) for o in results[1]["objects"]]
        for output, encoded in zip(results[0]["objects"], results[1]["objects"]):
            paths = [d / f"object_{output['index']:04d}.mdd" for d in directories]
            with np.load(directories[0] / f"input_{output['index']:04d}.npz", allow_pickle=False) as data:
                vertices = data["vertices"]
            with paths[0].open("rb") as a, paths[1].open("rb") as b:
                for frame in range(results[0]["frames"]):
                    before, after = sample(a, frame, output, vertices), sample(b, frame, encoded, vertices)
                    maximum_error = max(maximum_error, float(np.abs(before-after).max()))
                    changed += int(np.count_nonzero(before != after))
                    vertex_samples += len(before)
    values = {}
    groups = {"solve_retrieve": ("advance", "retrieve"),
              "diagnostics": ("motion_diagnostics", "solver_diagnostics", "diagnostics_finalize"),
              "cache_io": ("cache_write", "cache_finalize"), "output_transform": ("output_transform",)}
    for label, item in (("before",first), ("after",second)):
        phases = {}
        for group, keys in groups.items():
            phases[group] = statistics.median(sum(t["performance"]["phases"].get(k,{}).get("seconds",0)
                for k in keys) for t in item["trials"])
        values[label] = {"phase_median_seconds": phases,
                         "process_median_seconds": item["median_process_seconds"],
                         "cache_bytes": item["trials"][0]["performance"]["cache_bytes"]}
    report[name] = {**values, "maximum_local_vertex_difference": maximum_error,
                    "differing_components": changed, "compared_vertex_samples": vertex_samples}
    # A frozen input is not a promise that native parallel solving is bitwise
    # reproducible. Expose within-revision drift before attributing cross-run drift.
    for label, root in (("before", args.before), ("after", args.after)):
        maximum = 0.
        reference = root / name / "1"
        result = read(reference / "result.json")
        for trial in range(2, len(first["trials"]) + 1):
            for output in result["objects"]:
                filename = f"object_{output['index']:04d}.mdd"
                with np.load(reference / f"input_{output['index']:04d}.npz", allow_pickle=False) as data:
                    vertices = data["vertices"]
                with (reference / filename).open("rb") as a, (root / name / str(trial) / filename).open("rb") as b:
                    for frame in range(result["frames"]):
                        maximum = max(maximum, float(np.abs(sample(a, frame, output, vertices) - sample(b, frame, output, vertices)).max()))
        report[name][label]["within_revision_maximum_local_vertex_difference"] = maximum
args.output.write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
