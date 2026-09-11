# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Fixed encoding, all-node pins, transforms, fractional and addon-free playback."""
import argparse
import importlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_integration import evaluated, read_mdd

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--")+1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
scene = bpy.context.scene
scene.frame_start, scene.frame_end, scene.render.fps = 7, 17, 30
scene.unit_settings.scale_length = .01
scene.uipc_settings.python_executable = args.python
scene.uipc_settings.cache_directory = str(args.output / "cache")
rigid = demo.box(scene, "Fixed ABD", (100,0,100), (10,10,10), "RIGID")
rigid.scale = (-2,.5,1.5)
rigid.rotation_euler = (.2,.3,.4)
rigid.uipc_body.fixed = True
cloth = demo.mesh_object(scene, "All pinned cloth", [(0,0,100),(10,0,100),(0,10,100)], [(0,1,2)], "CLOTH")
volume = demo.mesh_object(scene, "All pinned FEM", [(0,0,0),(10,0,0),(0,10,0),(0,0,10)],
                          [(0,2,1),(0,1,3),(0,3,2),(1,2,3)], "FEM")
volume.location = (-100,0,100)
volume.data["uipc_tetrahedra"] = [0,1,2,3]
for obj in (cloth, volume):
    group = obj.vertex_groups.new(name="All nodes")
    group.add(list(range(len(obj.data.vertices))), 1, "REPLACE")
    obj.uipc_body.pin_group = group.name
# No dynamic DOFs: output still spans the full authored timeline.
scene.frame_set(7)
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
directory = Path(scene.uipc_settings.last_bake)
request = addon.protocol.read_json(directory / "request.json")
result = addon.protocol.read_json(directory / "result.json")
assert result["frames"] == 11 and result["performance"]["native_steps"] == 0
assert result["performance"]["cache_bytes"] == sum(12 + 12*o["vertices"] for o in result["objects"])
assert result["performance"]["cache_bytes"] < result["performance"]["dense_cache_bytes"] / 9
expected = {}
for output in result["objects"]:
    assert output["stored_frames"] == 1
    name = request["objects"][output["index"]]["name"]
    _, points = read_mdd(directory / f"object_{output['index']:04d}.mdd")
    assert len(points) == 1
    expected[name] = points[0]
    for frame, fraction in ((7,0), (17,.5), (8,.5), (1,0), (500,0), (7,0)):
        np.testing.assert_array_equal(evaluated(scene.objects[name], scene, frame, fraction), points[0])
addon.bridge.check_cache(scene, verify_data=True)
# A forged compact output for a merely partially fixed body is rejected upstream.
cloth.vertex_groups["All nodes"].remove([2])
bpy.context.view_layer.update()
try:
    addon.bridge.check_cache(scene)
    raise AssertionError("Pin edit did not invalidate the constant cache")
except ValueError:
    pass
cloth.vertex_groups["All nodes"].add([2], 1, "REPLACE")
addon.bridge.activate_cache(scene)
scene.frame_set(7)
path = args.output / "fixed.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(path))
addon.unregister()
bpy.ops.wm.open_mainfile(filepath=str(path))
scene = bpy.context.scene
for name, points in expected.items():
    for frame in (7,17,8,500):
        np.testing.assert_array_equal(evaluated(scene.objects[name], scene, frame, .5), points)
(args.output / "validation.json").write_text(json.dumps({"maximum_playback_error": 0,
    "cache_bytes": result["performance"]["cache_bytes"],
    "dense_cache_bytes": result["performance"]["dense_cache_bytes"], "without_addon": True}, indent=2))
print("PASS: fixed ABD/all-pinned cloth/FEM, zero-DOF, mirrored cm transform, native fractional addon-free playback")
