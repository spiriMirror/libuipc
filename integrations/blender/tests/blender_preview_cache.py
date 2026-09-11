# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Real depsgraph invalidation, lazy normals and bounded preview frame reads."""
import importlib
from pathlib import Path
import sys
import tempfile

import bpy
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
preview, protocol = addon.preview, addon.protocol
scene = bpy.context.scene
obj = demo.mesh_object(scene, "Preview", [(0,0,0), (1,0,0), (1,1,0), (0,1,0)],
                       [(0,1,2), (0,2,3)], "CLOTH")
group = obj.vertex_groups.new(name="Pins")
group.add([0], 1, "REPLACE")
obj.uipc_body.pin_group = "Pins"
bpy.context.view_layer.update()
preview.clear()
preview.statistics(reset=True)
data = preview.simulation_preview(obj, scene, include_guides=False)
for _ in range(10):
    assert preview.simulation_preview(obj, scene, include_guides=False) is data
counts = preview.statistics()
assert counts["topology_builds"] == counts["pin_builds"] == counts["point_builds"] == 1, counts
assert not counts.get("normal_builds"), counts
preview.simulation_preview(obj, scene)
assert preview.statistics()["normal_builds"] == 1
camera = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
scene.collection.objects.link(camera)
camera.location.z = 3
bpy.context.view_layer.update()
assert preview.simulation_preview(obj, scene) is data
assert preview.statistics()["topology_builds"] == 1

group.add([1], 1, "REPLACE")
bpy.context.view_layer.update()
np.testing.assert_array_equal(preview.simulation_preview(obj, scene)["pins"], [0, 1])
obj.data.vertices[2].co.z = .2
obj.data.update()
bpy.context.view_layer.update()
assert abs(preview.simulation_preview(obj, scene)["points"][2, 2] - .2) < 1e-7
# Same vertex/edge/triangle counts, different triangulation.
obj.data.clear_geometry()
obj.data.from_pydata([(0,0,0), (1,0,0), (1,1,0), (0,1,0)], [], [(0,1,3), (1,2,3)])
obj.data.update()
bpy.context.view_layer.update()
edges = preview.simulation_preview(obj, scene)["edges"].tolist()
assert [1, 3] in edges and [0, 2] not in edges
obj.location.x = 2
bpy.context.view_layer.update()
assert preview.simulation_preview(obj, scene)["points"][0, 0] == 2

with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    request = {"settings": {"frame_start": 7}}
    protocol.atomic_json(directory / "request.json", request)
    path = directory / "object_0000.mdd"
    points = np.array([v.co[:] for v in obj.data.vertices])
    def write(offset):
        writer = protocol.MDDWriter(path, 3, len(points), 30)
        for frame in range(3):
            writer.append(points + [0, 0, frame + offset])
        writer.close(commit=True)
    write(0)
    scene.uipc_settings.last_bake = str(directory)
    modifier = obj.modifiers.new(protocol.MODIFIER_NAME, "MESH_CACHE")
    addon.bridge.configure_cache_modifier(modifier, path, request)
    scene.frame_set(7, subframe=.5)
    preview.clear()
    preview.statistics(reset=True)
    expected = points + [2, 0, .5]
    np.testing.assert_allclose(preview.simulation_preview(obj, scene)["points"], expected)
    for _ in range(10):
        preview.simulation_preview(obj, scene)
    counts = preview.statistics()
    assert counts["frame_reads"] == 2 and counts["request_reads"] == 1, counts
    topology = counts["topology_builds"]
    for frame, fraction in ((9,0), (7,.5), (8,0)):
        scene.frame_set(frame, subframe=fraction)
        data = preview.simulation_preview(obj, scene)
        np.testing.assert_allclose(data["points"], points + [2,0,frame-7+fraction])
    assert preview.statistics()["topology_builds"] == topology, preview.statistics()
    assert all(len(e["samples"]) <= 2 for e in preview._entries.values())
    write(10)  # Atomic same-length replacement must invalidate decoded samples.
    np.testing.assert_allclose(preview.simulation_preview(obj, scene)["points"], points + [2,0,11])
    modifier.factor = .5
    try:
        preview.simulation_preview(obj, scene)
        raise AssertionError("Invalid playback controls accepted")
    except ValueError:
        pass
    modifier.factor = 1
    addon._history_post(None)
    assert not preview._entries
    preview.simulation_preview(obj, scene)
    addon._load_pre(None)
    assert not preview._entries and not preview._shaders
addon.unregister()
assert not preview._entries
print("PASS: camera reuse, lazy normals, mesh/pin edits, transform, frame/subframe/reverse, file replacement, cleanup")
