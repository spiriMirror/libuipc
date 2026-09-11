# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Compare encoded playback to the SAME native run, not independent trajectories."""
import argparse
import json
from pathlib import Path
import shutil
import struct
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
import worker
from protocol import SCHEMA_VERSION, atomic_json, read_json
from affine import affine_positions

parser = argparse.ArgumentParser()
parser.add_argument("--input", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
directory = args.output.resolve()
directory.mkdir(parents=True)
for path in args.input.iterdir():
    if path.name == "request.json" or path.suffix == ".npz":
        shutil.copy2(path, directory / path.name)
request = read_json(directory / "request.json")
request["schema_version"] = SCHEMA_VERSION
request["output_options"] = {"compact_abd": True}
atomic_json(directory / "request.json", request)
request,bodies = worker.load_request(directory)
observed = {}
class Capture(worker.QualityRecorder):
    def record_object(self,index,frame,points):
        body = bodies[index]
        inverse = np.linalg.inv(body["matrix"])
        local = (np.asarray(points) / request["settings"]["unit_scale"]) @ inverse[:3,:3].T + inverse[:3,3]
        observed.setdefault(index, []).append(local.copy())
        super().record_object(index,frame,points)
worker.QualityRecorder = Capture
worker.simulate(directory, worker.ParentProcess(None))
result = read_json(directory / "result.json")
report = {"objects": [], "note": "Single-run cache encoding check; no bitwise solver reproducibility requirement"}
for output in result["objects"]:
    index = output["index"]
    with (directory / f"object_{index:04d}.mdd").open("rb") as stream:
        frames,count = struct.unpack(">ii",stream.read(8))
        stream.read(4*frames)
        data = np.frombuffer(stream.read(),dtype=">f4").reshape(frames,count,3)
    decoded = np.stack([affine_positions(p,bodies[index]["vertices"]) for p in data]) if output.get("encoding") == "AFFINE" else data
    reference = np.array(observed[index])
    error = float(np.max(np.abs(decoded-reference)))
    tolerance = 2e-6 * max(1, float(np.max(np.abs(reference))))
    assert error <= tolerance, (index,error,tolerance)
    report["objects"].append({"index":index,"name":bodies[index]["name"],"encoding":output.get("encoding"),
                              "maximum_local_error":error,"vertex_samples":int(reference.shape[0]*reference.shape[1])})
report["cache_bytes"] = result["performance"]["cache_bytes"]
report["dense_cache_bytes"] = result["performance"]["dense_cache_bytes"]
atomic_json(directory / "encoding_validation.json",report)
print("SINGLE_RUN_AFFINE_CACHE_OK",json.dumps(report))
