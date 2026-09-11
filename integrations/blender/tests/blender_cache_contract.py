# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Actual Blender: cache provenance, playback controls, rollback and render guard."""

import importlib
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
protocol = importlib.import_module("libuipc_blender.protocol")
scene = bpy.data.scenes.new("Cache contract")
bpy.context.window.scene = scene
scene.frame_start, scene.frame_end = 1, 3
for index in range(2):
    demo.mesh_object(scene, f"Cloth {index}", [(index * 2, 0, 1), (index * 2 + 1, 0, 1),
                                              (index * 2, 1, 1)], [(0, 1, 2)], "CLOTH")


def prepare():
    directory, request = addon.bridge.export_job(scene)
    _, bodies = addon.bridge.collect_scene(scene)
    outputs = []
    for index, body in enumerate(bodies):
        writer = protocol.MDDWriter(directory / f"object_{index:04d}.mdd", 3, 3, 24)
        for frame in range(3):
            writer.append(body["vertices"] + [0, 0, frame * 0.01])
        writer.close(commit=True)
        outputs.append({"index": index, "vertices": 3, "sha256": writer.digest.hexdigest()})
    result = {"schema_version": request["schema_version"], "fingerprint": request["fingerprint"], "frames": 3,
              "cache_integrity": 1, "objects": outputs}
    protocol.atomic_json(directory / "result.json", result)
    return directory, request


def rejected(callback, match):
    try:
        callback()
    except (ValueError, RuntimeError) as error:
        assert match in str(error), (match, error)
    else:
        raise AssertionError("Invalid state was accepted: " + match)


with tempfile.TemporaryDirectory(prefix="uipc-cache-contract-") as scratch:
    scene.uipc_settings.cache_directory = scratch
    directory, request = prepare()
    addon.bridge.attach_cache(scene, directory, request)
    addon.bridge.check_cache(scene, verify_data=True)
    obj = scene.objects["Cloth 0"]
    obj.vertex_groups.new(name="unexpected")
    modifier = obj.modifiers[protocol.MODIFIER_NAME]
    for field, value in (("factor", .5), ("frame_scale", 2), ("frame_start", 2),
                         ("flip_axis", (True, False, False)), ("vertex_group", "unexpected")):
        old = tuple(modifier.flip_axis) if field == "flip_axis" else getattr(modifier, field)
        setattr(modifier, field, value)
        rejected(lambda: addon.bridge.check_cache(scene), field)
        setattr(modifier, field, old)
    modifier.show_render = False
    rejected(lambda: addon.bridge.validate_for_render(scene), "disabled")
    addon.bridge.activate_cache(scene)
    addon.bridge.validate_for_render(scene)
    modifier.factor = .5
    scene.render.filepath = str(Path(scratch) / "must_not_render.png")
    rejected(lambda: bpy.ops.uipc.render_validated(), "factor")
    assert not Path(scene.render.filepath).exists()
    modifier.factor = 1

    original = {o.name: o.modifiers[protocol.MODIFIER_NAME].filepath for o in scene.objects}
    old_bake = scene.uipc_settings.last_bake
    new_directory, new_request = prepare()
    configure = addon.bridge.configure_cache_modifier
    calls = [0]

    def fail_second(*args):
        calls[0] += 1
        configure(*args)
        if calls[0] == 2:
            raise RuntimeError("injected attachment failure")

    with patch.object(addon.bridge, "configure_cache_modifier", side_effect=fail_second):
        rejected(lambda: addon.bridge.attach_cache(scene, new_directory, new_request), "injected")
    assert scene.uipc_settings.last_bake == old_bake
    assert original == {o.name: o.modifiers[protocol.MODIFIER_NAME].filepath for o in scene.objects}
    addon.bridge.check_cache(scene, verify_data=True)
    cache = directory / "object_0000.mdd"
    data = cache.read_bytes()
    cache.write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
    rejected(lambda: addon.bridge.check_cache(scene, verify_data=True), "checksum")
    cache.write_bytes(data)
    result = protocol.read_json(directory / "result.json")
    result["objects"] = []
    protocol.atomic_json(directory / "result.json", result)
    rejected(lambda: addon.bridge.check_cache(scene), "missing")
addon.unregister()
print("PASS: playback settings, checksums, complete result, transactional rollback and guarded render")
