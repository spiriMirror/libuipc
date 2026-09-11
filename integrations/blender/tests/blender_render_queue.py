# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Real snapshot queue: subset/multi-view output, checked resume and cancellation."""

import argparse
import importlib
from pathlib import Path
import sys
import time

import bpy

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--test-gpu-render", action="store_true")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
queue = importlib.import_module("libuipc_blender.render_queue")
demo = importlib.import_module("libuipc_blender.demo")
scene = demo.create_demo()
bpy.context.window.scene = scene
fixed = demo.box(scene, "Compact fixed ABD", (1.5,1.5,.25), (.2,.2,.2), "RIGID")
fixed.uipc_body.fixed = True
scene.frame_end = 3
settings = scene.uipc_settings
settings.python_executable = args.python
settings.cache_directory = str(args.output / "cache")
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
result = addon.protocol.read_json(Path(settings.last_bake) / "result.json")
assert any(o.get("stored_frames") == 1 for o in result["objects"])
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = scene.render.resolution_y = 64
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
source = args.output / "source.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(source))
source_hash = addon.protocol.file_sha256(source)
camera = scene.camera
other = camera.copy()
other.data = camera.data.copy()
scene.collection.objects.link(other)
other.location.x += .2
for cam in (camera, other):
    settings.render_cameras.add().camera = cam
settings.render_first, settings.render_last = 2, 3
settings.render_directory = str(args.output / "renders")
original = (scene.frame_start, scene.frame_end, scene.frame_current, scene.camera, scene.render.filepath)
fingerprint = settings.baked_fingerprint
result = queue.run_blocking(scene, timeout=120)
assert result["rendered"] == 4 and result["skipped"] == 0, result
assert original == (scene.frame_start, scene.frame_end, scene.frame_current, scene.camera, scene.render.filepath)
assert addon.protocol.file_sha256(source) == source_hash
assert addon.bridge.check_cache(scene)["fingerprint"] == fingerprint
directory = Path(settings.last_render_job)
manifest = addon.protocol.read_json(directory / "render_manifest.json")
assert any(record["path"].endswith(".mdd") for record in manifest["dependencies"])
camera.location.x += .1  # Resume must use the old snapshot, not this live edit.
settings.render_first = settings.render_last = 1
result = queue.run_blocking(scene, resume=True, timeout=120)
assert result["rendered"] == 0 and result["skipped"] == 4, result
image = directory / "camera_000" / "frame_000002.png"
image.write_bytes(image.read_bytes()[:-4])
result = queue.run_blocking(scene, resume=True, timeout=120)
assert result["rendered"] == 1 and result["skipped"] == 3, result
queue.start(scene, resume=True)
queue.request_cancel()
started = time.monotonic()
while queue.is_running():
    queue.poll()
    assert time.monotonic() - started < 20
    time.sleep(.1)
assert len(list(directory.glob("camera_*/*.png"))) == 4
dependency = next(Path(record["path"]) for record in manifest["dependencies"] if record["path"].endswith(".mdd"))
data = dependency.read_bytes()
try:
    dependency.write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
    try:
        queue.start(scene, resume=True)
    except ValueError as error:
        assert "dependency" in str(error)
    else:
        raise AssertionError("Changed cache dependency was accepted")
finally:
    dependency.write_bytes(data)
if args.test_gpu_render:
    preferences = bpy.context.preferences.addons["cycles"].preferences
    preferences.compute_device_type = "OPTIX"
    preferences.get_devices()
    for device in preferences.devices:
        device.use = device.type == "OPTIX"
    scene.render.engine = "CYCLES"
    scene.cycles.device = "GPU"
    scene.cycles.samples = 1
    settings.render_cameras.clear()
    settings.render_first = settings.render_last = 2
    result = queue.run_blocking(scene, timeout=120)
    assert result["rendered"] == 1
addon.unregister()
print("PASS: multi-view subset, frozen snapshot, source preservation, receipts, repair and cancellation")
