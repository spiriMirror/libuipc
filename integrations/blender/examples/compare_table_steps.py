# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Compare identical table scenes with different substep counts; needs NumPy."""

import argparse
import json
from pathlib import Path
import struct

import numpy as np


def load(directory):
    report = json.loads((directory / "physics_validation.json").read_text())
    signature = report["solver"]["fingerprint"]
    jobs = []
    for candidate in (directory / "cache").glob("bake_*"):
        if (candidate / "result.json").is_file():
            result = json.loads((candidate / "result.json").read_text())
            if result["fingerprint"] == signature:
                jobs.append(candidate)
    if len(jobs) != 1:
        raise ValueError(f"Expected one matching completed bake in {directory}")
    request = json.loads((jobs[0] / "request.json").read_text())
    return report, request, jobs[0]


def final_positions(job, index, matrix, sample_frame):
    with (job / f"object_{index:04d}.mdd").open("rb") as stream:
        frames, vertices = struct.unpack(">ii", stream.read(8))
        assert 1 <= sample_frame <= frames
        stream.seek(8 + frames * 4 + (sample_frame - 1) * vertices * 12)
        points = np.frombuffer(stream.read(vertices * 12), dtype=">f4").reshape(-1, 3)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--coarse", required=True, type=Path)
    parser.add_argument("--fine", required=True, type=Path)
    args = parser.parse_args()
    coarse, cr, cj = load(args.coarse)
    fine, fr, fj = load(args.fine)
    assert cr["objects"] == fr["objects"], "Objects/material parameters differ"
    for key, value in cr["settings"].items():
        if key not in ("substeps", "frame_end"):
            assert value == fr["settings"][key], f"Different simulation setting: {key}"
    assert cr["settings"]["substeps"] < fr["settings"]["substeps"]
    sample_frame = min(coarse["solver"]["frames"], fine["solver"]["frames"])
    records = {}
    for index, entry in enumerate(cr["objects"]):
        with np.load(cj / f"input_{index:04d}.npz", allow_pickle=False) as a:
            with np.load(fj / f"input_{index:04d}.npz", allow_pickle=False) as b:
                for key in a.files:
                    assert np.array_equal(
                        a[key], b[key]
                    ), f"Different initial input: {entry['name']} / {key}"
                matrix = a["matrix"].copy()
        if entry["material"]["role"] == "STATIC":
            continue
        a, b = final_positions(cj, index, matrix, sample_frame), final_positions(
            fj, index, matrix, sample_frame
        )
        delta = np.linalg.norm(a - b, axis=1)
        records[entry["name"]] = {
            "final_vertex_rms_difference_m": float(np.sqrt(np.mean(delta**2))),
            "final_vertex_max_difference_m": float(delta.max()),
            "final_vertex_centroid_difference_m": float(
                np.linalg.norm(a.mean(axis=0) - b.mean(axis=0))
            ),
            "final_min_height_difference_m": float(abs(a[:, 2].min() - b[:, 2].min())),
        }
    report = {
        "identical_initial_geometry_and_materials": True,
        "compared_frame": sample_frame,
        "compared_time_s": (sample_frame - 1) / cr["settings"]["fps"],
        "coarse_dt_s": 1 / (cr["settings"]["fps"] * cr["settings"]["substeps"]),
        "fine_dt_s": 1 / (fr["settings"]["fps"] * fr["settings"]["substeps"]),
        "coarse_fingerprint": coarse["solver"]["fingerprint"],
        "fine_fingerprint": fine["solver"]["fingerprint"],
        "both_run_final_contact_support_checks_passed": coarse[
            "all_objects_have_contact_path_to_fixed_support"
        ]
        and fine["all_objects_have_contact_path_to_fixed_support"],
        "objects": records,
        "interpretation": "A time-step sensitivity experiment, not a convergence proof. "
        "Frictional settling can select different local cloth folds. The adaptive "
        "contact stiffness corridor itself depends on dt. Geometry, materials, "
        "all other explicit solver inputs are unchanged. The longer run is "
        "sampled at the shorter run's final time; support checks refer to each "
        "run's own final state. The JSON field prefix 'final' means this common "
        "comparison endpoint, not necessarily the end of the longer cache.",
    }
    destination = args.fine / "time_step_comparison.json"
    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
