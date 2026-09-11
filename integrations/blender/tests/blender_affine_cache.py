# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Synthetic full-affine playback: native helpers, rollback, corruption and reopening."""
import argparse
import importlib
from pathlib import Path
import sys
from unittest.mock import patch

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--")+1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_integration import evaluated
addon = importlib.import_module("libuipc_blender")
addon.register()
affine = importlib.import_module("libuipc_blender.affine")
playback = importlib.import_module("libuipc_blender.affine_playback")
demo = importlib.import_module("libuipc_blender.demo")
scene = bpy.context.scene
scene.frame_start, scene.frame_end = 7,9
scene.uipc_settings.cache_directory = str(args.output / "cache")
obj = demo.box(scene,"Affine body",(3,-2,1),(.3,.4,.5),"RIGID")
obj.scale, obj.rotation_euler = (-1.2,.5,1.4), (.2,.3,.4)
other = demo.box(scene,"Second body",(-3,1,2),(.2,.2,.2),"RIGID")
bpy.context.view_layer.update()
base = np.array([v.co[:] for v in obj.data.vertices])
matrices = np.repeat(np.eye(4)[None],3,axis=0)
matrices[1,:3,:3] = [[1.1,.2,-.04],[.01,.9,.15],[.03,-.02,1.2]]
matrices[1,:3,3] = [.1,-.2,.3]
matrices[2,:3,:3] = [[-.9,.3,.1],[.2,.8,.04],[-.1,.2,1.1]]
matrices[2,:3,3] = [-.2,.4,-.3]


def prepare(compact):
    directory, request = addon.bridge.export_job(scene)
    settings,bodies = addon.bridge.collect_scene(scene)
    outputs = []
    for index,body in enumerate(bodies):
        encoding = "AFFINE" if compact else "VERTEX"
        writer = addon.protocol.MDDWriter(directory/f"object_{index:04d}.mdd",len(matrices),
                                          4 if compact else len(body["vertices"]),scene.render.fps)
        for matrix in matrices:
            samples = affine.pack_affine(matrix)
            writer.append(samples if compact else affine.affine_positions(samples,body["vertices"]))
        writer.close(commit=True)
        outputs.append({"index":index,"vertices":len(body["vertices"]),"encoding":encoding,
                        "sha256":writer.digest.hexdigest()})
    addon.protocol.atomic_json(directory/"result.json", {"schema_version":request["schema_version"],
        "fingerprint":request["fingerprint"],"frames":len(matrices),"cache_integrity":1,"objects":outputs})
    return directory,request


directory,request = prepare(True)
addon.bridge.attach_cache(scene,directory,request)
addon.bridge.check_cache(scene,verify_data=True)
for frame,fraction in ((7,0),(8,0),(9,0),(7,.5),(8,.25),(1,0),(500,0),(7,0)):
    position = min(max(frame-7+fraction,0),2)
    i,alpha = int(position), position-int(position)
    matrix = matrices[i]*(1-alpha)+matrices[min(i+1,2)]*alpha
    expected = base@matrix[:3,:3].T+matrix[:3,3]
    np.testing.assert_allclose(evaluated(obj,scene,frame,fraction),expected,atol=2e-6)
    data = addon.preview.simulation_preview(obj,scene)
    world = np.asarray(obj.matrix_world)
    np.testing.assert_allclose(data["points"],expected@world[:3,:3].T+world[:3,3],atol=2e-6)
np.testing.assert_array_equal(np.array([v.co[:] for v in obj.data.vertices]),base)
modifier = obj.modifiers[addon.protocol.MODIFIER_NAME]
group = modifier.node_group
proxy = playback.proxy_object(modifier)
proxy.name = "Renamed helper"
addon.bridge.check_cache(scene)

# Missing/disabled helpers are rejected without changing the cache or source.
cached = playback.playback_modifier(modifier)
cached.show_render = False
try:
    addon.bridge.check_cache(scene)
    raise AssertionError("Disabled helper was accepted")
except ValueError:
    pass
cached.show_render = True
group.nodes["Samples"].inputs["Object"].default_value = None
try:
    addon.bridge.check_cache(scene)
    raise AssertionError("Missing helper was accepted")
except ValueError:
    pass
group.nodes["Samples"].inputs["Object"].default_value = proxy
group.nodes["Sample 1"].inputs["Index"].default_value = 2
try:
    addon.bridge.check_cache(scene)
    raise AssertionError("Modified affine reconstruction was accepted")
except ValueError:
    pass
group.nodes["Sample 1"].inputs["Index"].default_value = 1
proxy.location.x = .1
bpy.context.view_layer.update()
try:
    addon.bridge.check_cache(scene)
    raise AssertionError("Moved helper was accepted")
except ValueError:
    pass
proxy.location.x = 0
bpy.context.view_layer.update()
addon.bridge.check_cache(scene)

# Cross-encoding attachment failure preserves old modifiers, helpers and reference.
old_group, old_bake = group, scene.uipc_settings.last_bake
helper_count = len([o for o in scene.objects if o.get(playback.MARKER)])
new_directory,new_request = prepare(False)
configure = addon.bridge.configure_cache_modifier
calls = [0]
def fail_second(*args):
    calls[0] += 1
    configure(*args)
    if calls[0] == 2:
        raise RuntimeError("injected failure")
with patch.object(addon.bridge,"configure_cache_modifier",side_effect=fail_second):
    try:
        addon.bridge.attach_cache(scene,new_directory,new_request)
        raise AssertionError("Injection did not fail")
    except RuntimeError as error:
        assert "injected failure" in str(error)
assert obj.modifiers[addon.protocol.MODIFIER_NAME].node_group == old_group
assert scene.uipc_settings.last_bake == old_bake
assert len([o for o in scene.objects if o.get(playback.MARKER)]) == helper_count
addon.bridge.check_cache(scene,verify_data=True)
addon.bridge.attach_cache(scene,new_directory,new_request)
assert not any(o.get(playback.MARKER) for o in scene.objects)
calls[0] = 0
with patch.object(addon.bridge,"configure_cache_modifier",side_effect=fail_second):
    try:
        addon.bridge.attach_cache(scene,directory,request)
        raise AssertionError("Affine attachment injection did not fail")
    except RuntimeError as error:
        assert "injected failure" in str(error)
assert not any(o.get(playback.MARKER) for o in scene.objects)
assert obj.modifiers[addon.protocol.MODIFIER_NAME].type == "MESH_CACHE"
addon.bridge.attach_cache(scene,directory,request)
assert len([o for o in scene.objects if o.get(playback.MARKER)]) == 2

path = args.output/"affine.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(path))
addon.unregister()
bpy.ops.wm.open_mainfile(filepath=str(path))
scene = bpy.context.scene
obj = scene.objects["Affine body"]
for frame in (7,9,8,7):
    expected = base@matrices[frame-7,:3,:3].T+matrices[frame-7,:3,3]
    np.testing.assert_allclose(evaluated(obj,scene,frame),expected,atol=2e-6)
addon.register()
addon.bridge.check_cache(scene,verify_data=True)
addon.bridge.detach_cache(scene)
assert not any(o.get(playback.MARKER) for o in scene.objects)
np.testing.assert_array_equal(evaluated(obj,scene,7),base)

# Explicitly retained node data is not deleted by normal cache detachment.
addon.bridge.attach_cache(scene,directory,request)
retained_group = obj.modifiers[addon.protocol.MODIFIER_NAME].node_group
retained_proxy = playback.proxy_object(obj.modifiers[addon.protocol.MODIFIER_NAME])
retained_group.use_fake_user = True
addon.bridge.detach_cache(scene)
assert retained_group.name in bpy.data.node_groups and retained_proxy.name in scene.objects
assert retained_group.nodes["Samples"].inputs["Object"].default_value == retained_proxy
# Only remove this test-created retained data after verifying the ownership contract.
retained_mesh = retained_proxy.data
bpy.data.node_groups.remove(retained_group)
bpy.data.objects.remove(retained_proxy,do_unlink=True)
if retained_mesh.users == 0:
    bpy.data.meshes.remove(retained_mesh)

# A full 500-frame file is replayed, not merely clamping a short cache to frame 500.
scene = bpy.data.scenes.new("Long affine cache")
bpy.context.window.scene = scene
scene.frame_start,scene.frame_end = 1,500
scene.unit_settings.scale_length = .01
scene.uipc_settings.cache_directory = str(args.output/"long_cache")
obj = demo.box(scene,"Long body",(40,-20,10),(10,10,10),"RIGID")
obj.scale,obj.rotation_euler = (-1.2,.5,1.4),(.2,.3,.4)
base = np.array([v.co[:] for v in obj.data.vertices])
angles = np.linspace(0,2*np.pi,500)
matrices = np.repeat(np.eye(4)[None],500,axis=0)
matrices[:,0,0] = np.cos(angles)
matrices[:,0,1] = -np.sin(angles)+.07*np.sin(2*angles)
matrices[:,1,0] = np.sin(angles)
matrices[:,1,1] = .8*np.cos(angles)
matrices[:,2,2] = 1.2+.1*np.sin(angles)
matrices[:,:3,3] = np.array([.3*np.cos(angles),.2*np.sin(angles),.05*np.cos(2*angles)]).T
directory,request = prepare(True)
addon.bridge.attach_cache(scene,directory,request)
addon.bridge.check_cache(scene,verify_data=True)
long_path = args.output/"affine_500.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(long_path))
addon.unregister()
bpy.ops.wm.open_mainfile(filepath=str(long_path))
scene,obj = bpy.context.scene,bpy.context.scene.objects["Long body"]
maximum = 0.
for frame in range(1,501):
    expected = base@matrices[frame-1,:3,:3].T+matrices[frame-1,:3,3]
    actual = evaluated(obj,scene,frame)
    maximum = max(maximum,float(np.max(np.abs(actual-expected))))
    np.testing.assert_allclose(actual,expected,atol=3e-6)
addon.protocol.atomic_json(args.output/"validation.json",{"frames_without_addon":500,
    "maximum_local_error":maximum,"cache_bytes":(directory/"object_0000.mdd").stat().st_size})
print("PASS: full affine native playback, hidden helper, subframes, mirror, preview, corruption, bidirectional rollback and 500-frame addon-free reopen")
