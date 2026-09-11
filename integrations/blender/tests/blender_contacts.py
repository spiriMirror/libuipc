# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Real Blender/CUDA: pair overrides change friction and collision response."""

import argparse
import importlib
import json
from pathlib import Path
import struct
import sys

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
results = {}
for label, friction, enabled in (("sliding", 0., True), ("sticking", 1., True), ("disabled", 1., False)):
    scene = bpy.data.scenes.new(label)
    bpy.context.window.scene = scene
    scene.frame_start, scene.frame_end, scene.render.fps = 1, 41, 50
    settings = scene.uipc_settings
    settings.python_executable = args.python
    settings.cache_directory = str(args.output.resolve() / label)
    settings.gravity = (4, 0, -9.81)
    settings.friction = .5  # Unchanged: only the pair controls differ.
    floor = demo.mesh_object(scene, "Floor", [(-4, -4, 0), (4, -4, 0), (4, 4, 0), (-4, 4, 0)],
                             [(0, 1, 2, 3)], "STATIC")
    cube = demo.box(scene, "Apple", (0, 0, .103), (.2, .2, .2), "RIGID")
    floor.uipc_body.contact_material = "Ceramic"
    cube.uipc_body.contact_material = "Apple skin"
    pair = settings.contact_pairs.add()
    pair.material_a, pair.material_b = "Ceramic", "Apple skin"
    pair.friction, pair.enabled = friction, enabled
    scene.frame_set(1)
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    addon.bridge.check_cache(scene, verify_data=True)
    directory = Path(bpy.path.abspath(settings.last_bake))
    result = json.loads((directory / "result.json").read_text())
    index = result["objects"][0]["index"]
    with (directory / f"object_{index:04d}.mdd").open("rb") as stream:
        frames, vertices = struct.unpack(">ii", stream.read(8))
        stream.read(4 * frames)
        points = np.frombuffer(stream.read(), dtype=">f4").reshape(frames, vertices, 3)
    if result["objects"][0].get("encoding") == "AFFINE":
        decode = importlib.import_module("libuipc_blender.affine").affine_positions
        source = np.array([v.co[:] for v in cube.data.vertices])
        points = np.stack([decode(frame, source) for frame in points])
    results[label] = {"x": float(points[-1, :, 0].mean()),
                      "z": float(points[-1, :, 2].mean() + cube.location.z)}
assert results["sliding"]["x"] > .2, results
assert abs(results["sticking"]["x"]) < results["sliding"]["x"] * .1, results
assert results["disabled"]["z"] < -1, results
(args.output / "contact_validation.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
addon.unregister()
print("PASS: pair overrides reach native contact response", results)
