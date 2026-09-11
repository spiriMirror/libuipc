# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Native rods: pins, bending, contact, curve import, surface display and replay."""
import argparse
import importlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np
from mathutils import Vector

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--")+1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_integration import evaluated, result_arrays
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
rod_ui = importlib.import_module("libuipc_blender.rod_ui")
scene = bpy.data.scenes.new("Rod validation")
bpy.context.window.scene = scene
scene.frame_start, scene.frame_end, scene.render.fps = 7, 37, 50
scene.uipc_settings.python_executable = args.python
scene.uipc_settings.cache_directory = str(args.output / "cache")
scene.uipc_settings.solver_accuracy = "CONVERGED"
demo.mesh_object(scene, "Floor", [(-2,-2,0),(2,-2,0),(2,2,0),(-2,2,0)], [(0,1,2,3)], "STATIC")

def rod(name, y, stiffness, fixed=False):
    vertices = np.array([[i*.03,0,0] for i in range(8)])
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [(i,i+1) for i in range(7)], [])
    obj = bpy.data.objects.new(name, mesh)
    scene.collection.objects.link(obj)
    obj.location = (-.3,y,.4)
    obj.uipc_body.role = "ROD"
    obj.uipc_body.thickness = .005
    obj.uipc_body.rod_bending = stiffness
    group = obj.vertex_groups.new(name="Pins")
    group.add([0,1] if not fixed else list(range(8)), 1, "REPLACE")
    obj.uipc_body.pin_group = group.name
    return obj

soft, stiff, fixed = rod("Soft",0,1e5), rod("Stiff",.12,1e7), rod("All pinned",.24,1e5,True)
fixed.scale = (-1.1,.7,1.2)
free = rod("Falling",-.3,1e5)
free.uipc_body.pin_group = ""
free.location.z = .15
abd = demo.box(scene, "ABD", (.4,.4,.1), (.1,.1,.1), "RIGID")
abd.uipc_body.fixed = True
scene.frame_set(7)
bpy.context.view_layer.update()
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
directory, request, result, arrays = result_arrays(scene)
assert request["schema_version"] >= 7
assert result["rod_stiffness"]["Soft"]["bending_rigidity"] > 0
for obj in (soft,stiff,fixed,free):
    points = arrays[obj.name]
    for frame, fraction in ((7,0),(37,0),(10,.5),(8,0)):
        first = frame-7
        target = points[first]*(1-fraction) + points[min(first+1,len(points)-1)]*fraction
        np.testing.assert_array_equal(evaluated(obj, scene, frame, fraction), target)
    np.testing.assert_array_equal(np.array([v.co[:] for v in obj.data.vertices]), points[0])
for obj in (soft,stiff):
    np.testing.assert_allclose(arrays[obj.name][:,:2], np.broadcast_to(arrays[obj.name][0,:2], (31,2,3)), atol=2e-7)
assert arrays["Soft"][-1,-1,2] < arrays["Stiff"][-1,-1,2] - .003
assert float(arrays["Falling"][:,:,2].min()) + free.location.z >= .0048
assert float(arrays["Falling"][-1,:,2].mean()) + free.location.z < .02
assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
preview = addon.preview.simulation_preview(soft,scene)
assert len(preview["edges"]) == 7 and preview["pins"].tolist() == [0,1]
guides = preview["guides"].reshape(-1,2,3)
np.testing.assert_allclose(np.linalg.norm(guides[:,1]-guides[:,0],axis=1), .01, rtol=1e-6)

# Renderable native display follows the simulated centerline without addon callbacks.
for obj, color in ((soft,(.1,.4,.9)),(stiff,(.9,.3,.1)),(fixed,(.1,.7,.3)),(free,(.7,.2,.7))):
    demo.material(obj,obj.name,color)
    rod_ui.build_display(obj,scene)
camera = bpy.data.objects.new("Camera", bpy.data.cameras.new("Camera"))
scene.collection.objects.link(camera)
camera.location = (.9,-1.3,1.0)
camera.rotation_euler = (Vector((-.15,0,.25))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.camera = camera
light = bpy.data.objects.new("Light", bpy.data.lights.new("Light","AREA"))
scene.collection.objects.link(light)
light.location = (0,0,2)
light.data.energy,light.data.size = 150,2
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = 640,480,100
scene.frame_set(37)
addon.bridge.check_cache(scene,verify_data=True)
scene.render.filepath = str(args.output / "rods.png")
bpy.ops.render.render(write_still=True)
blend = args.output / "rods.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(blend))
addon.unregister()
bpy.ops.wm.open_mainfile(filepath=str(blend))
scene = bpy.context.scene
for name in ("Soft","Stiff","All pinned","Falling"):
    obj = scene.objects[name]
    obj.modifiers[rod_ui.DISPLAY_NAME].show_viewport = False
    np.testing.assert_array_equal(evaluated(obj,scene,37), arrays[name][-1])
    obj.modifiers[rod_ui.DISPLAY_NAME].show_viewport = True
    assert len(evaluated(obj,scene,37)) > len(arrays[name][-1])

# A beveled curve is sampled from a private copy; original properties are preserved.
addon.register()
curve = bpy.data.curves.new("Curve source", "CURVE")
curve.dimensions, curve.bevel_depth = "3D", .012
spline = curve.splines.new("POLY")
spline.points.add(3)
for i,p in enumerate(spline.points):
    p.co = (i*.03,0,0,1)
source = bpy.data.objects.new("Curve source", curve)
scene.collection.objects.link(source)
source.location = (0,.8,.3)
bpy.context.view_layer.objects.active = source
assert bpy.ops.uipc.curve_to_rod() == {"FINISHED"}
converted = bpy.context.object
assert converted.uipc_body.role == "ROD" and len(converted.data.edges) == 3 and len(converted.data.polygons) == 0
assert abs(curve.bevel_depth-.012) < 1e-8
addon.unregister()
(args.output / "validation.json").write_text(json.dumps({"cache_vertices_exact": True,
    "soft_tip_z": float(arrays["Soft"][-1,-1,2]), "stiff_tip_z": float(arrays["Stiff"][-1,-1,2]),
    "addon_free": True, "curve_conversion": True}, indent=2))
print("PASS: native rod stretch/bending, whole/node pins, floor contact, native tube render, addon-free replay and curve import")
