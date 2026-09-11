# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Real cache rename/reorder, controller identity and cheap validation gates."""

import argparse
import importlib
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import bpy

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
scene = demo.create_demo()
bpy.context.window.scene = scene
scene.frame_end = 3
scene.uipc_settings.python_executable = args.python
scene.uipc_settings.cache_directory = str(args.output.resolve())
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
old = scene.uipc_settings.baked_fingerprint
cloth = scene.objects["Pinned Cloth"]
body = scene.objects["Falling ABD"]
cloth.name, body.name = "A renamed cloth", "Z renamed body"
assert addon.bridge.check_cache(scene, verify_data=True)["fingerprint"] == old
assert bpy.ops.uipc.quality_load() == {"FINISHED"}
assert bpy.ops.uipc.quality_jump(index=3, metric="speed") == {"FINISHED"}
assert bpy.context.object == cloth
addon.watch.clear()
assert addon.watch.relevant_update(scene, [])
assert not addon.watch.relevant_update(scene, [SimpleNamespace(id=scene.camera)])
assert addon.watch.check(scene)
with patch.object(addon.bridge, "validate_mesh", side_effect=AssertionError("unnecessary full validation")):
    assert not addon.watch.check(scene)
directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
cache = directory / "object_0000.mdd"
original_bytes = cache.read_bytes()
cache.write_bytes(original_bytes[:-1] + bytes([original_bytes[-1] ^ 1]))
try:
    for attempt in range(2):
        try:
            addon.watch.check(scene)
        except ValueError as error:
            assert "checksum" in str(error)
        else:
            raise AssertionError("Failed validation became a trusted fast-path token")
finally:
    cache.write_bytes(original_bytes)
assert addon.watch.check(scene)
copy = body.copy()
copy.data = body.data.copy()
scene.collection.objects.link(copy)
try:
    addon.bridge.check_cache(scene)
except ValueError:
    pass
else:
    raise AssertionError("Copied object ID did not invalidate the cache")
for obj in scene.objects:
    obj.select_set(obj == copy)
assert bpy.ops.uipc.new_identity() == {"FINISHED"}
assert copy["uipc_object_id"] != body["uipc_object_id"]
bpy.data.objects.remove(copy, do_unlink=True)
addon.bridge.activate_cache(scene)
path = args.output.resolve() / "renamed.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(path))
bpy.ops.wm.open_mainfile(filepath=str(path))
assert addon.bridge.check_cache(bpy.context.scene, verify_data=True)["fingerprint"] == old
driven_scene = bpy.data.scenes.new("Controller identities")
bpy.context.window.scene = driven_scene
driven_scene.frame_start, driven_scene.frame_end = 1, 3
driven_scene.uipc_settings.python_executable = args.python
driven_scene.uipc_settings.cache_directory = str(args.output.resolve() / "controller")
servo = demo.box(driven_scene, "Servo", (0, 0, 1), (.1, .1, .1), "RIGID")
parent = bpy.data.objects.new("Controller root", None)
target = bpy.data.objects.new("Controller target", None)
driven_scene.collection.objects.link(parent)
driven_scene.collection.objects.link(target)
target.parent = parent
servo.uipc_body.driven = True
servo.uipc_body.drive_target = target
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
signature = driven_scene.uipc_settings.baked_fingerprint
parent.name, target.name = "Renamed root", "Renamed target"
assert addon.bridge.check_cache(driven_scene)["fingerprint"] == signature
addon.unregister()
print("PASS: body/controller rename, saved UUIDs, copied-ID safety and unchanged-input fast path")
