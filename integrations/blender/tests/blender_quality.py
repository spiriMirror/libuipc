# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Real Blender/CUDA diagnostics, frame navigation and non-destructive previews."""

import argparse
import importlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
quality_ui = importlib.import_module("libuipc_blender.quality_ui")
scene = bpy.data.scenes.new("Quality validation")
bpy.context.window.scene = scene
scene.frame_start, scene.frame_end, scene.render.fps = 7, 17, 50
scene.uipc_settings.python_executable = args.python
scene.uipc_settings.cache_directory = str(args.output / "cache")
body = demo.box(scene, "Falling body", (0, 0, 1), (.2, .2, .2), "RIGID")
initial = np.array([v.co[:] for v in body.data.vertices])
body.modifiers.new("Display subdivision", "SUBSURF")
scene.frame_set(7)
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
report = quality_ui.load_report(scene)
assert report["objects"][0]["max_speed"] > 1
assert report["objects"][0]["max_acceleration"] > 5
assert report["solver"]["steps"] == 20
assert report["frame_start"] == 7
assert bpy.ops.uipc.quality_jump(index=0, metric="speed") == {"FINISHED"}
assert bpy.context.object == body
assert scene.frame_current == report["objects"][0]["speed_frame"]
preview = addon.preview.simulation_preview(body, scene)
assert len(preview["points"]) == len(body.data.vertices) == 8
assert preview["points"][:, 2].mean() < 1
np.testing.assert_array_equal(initial, np.array([v.co[:] for v in body.data.vertices]))
scene.uipc_settings.quality_speed_limit = .1
assert bpy.ops.uipc.quality_mark() == {"FINISHED"}
assert len(scene.timeline_markers) >= 1
before = addon.bridge.collect_scene(scene)
scene.uipc_settings.quality_acceleration_limit = 123
after = addon.bridge.collect_scene(scene)
assert addon.protocol.fingerprint(*before) == addon.protocol.fingerprint(*after)
bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "quality.blend"))
bpy.ops.wm.open_mainfile(filepath=str(args.output / "quality.blend"))
assert bpy.ops.uipc.quality_load() == {"FINISHED"}

preview_scene = bpy.data.scenes.new("Pin preview")
bpy.context.window.scene = preview_scene
preview_scene.unit_settings.scale_length = .01
cloth = demo.mesh_object(preview_scene, "Cloth", [(0, 0, 0), (1, 0, 0), (0, 1, 0)], [(0, 1, 2)], "CLOTH")
cloth.uipc_body.thickness = .001
pins = cloth.vertex_groups.new(name="Pins")
pins.add([1], 1, "REPLACE")
cloth.uipc_body.pin_group = "Pins"
bpy.context.view_layer.update()
data = addon.preview.simulation_preview(cloth, preview_scene)
np.testing.assert_array_equal(data["pins"], [1])
guides = data["guides"].reshape(-1, 2, 3)
np.testing.assert_allclose(np.linalg.norm(guides[:, 1] - guides[:, 0], axis=1), .2, rtol=1e-6)
addon.unregister()
(args.output / "quality_validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print("PASS: native quality report, selected peak navigation, timeline marks, cache preview and SI thickness")
