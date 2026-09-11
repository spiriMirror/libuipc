# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Export immutable benchmark inputs once; --measure reuses the saved source.

Run in background Blender with -- --python <native-python> --output <fresh-dir>
and optionally --urdf <sample-87 robot_hand.urdf>. See the integration guide.
"""
import argparse
import importlib
import json
from pathlib import Path
import shutil
import sys
import time

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--urdf", type=Path)
parser.add_argument("--measure", help="Measure previews on previously exported benchmarks.blend")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
demo = importlib.import_module("libuipc_blender.demo")
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)


def cloth(scene, n, center):
    points = [(center[0] - .5 + x / (n-1), center[1] - .5 + y / (n-1), center[2])
              for y in range(n) for x in range(n)]
    faces = [(a, a+1, a+n+1) for a in range(n*n-n) if a % n != n-1]
    faces += [(a, a+n+1, a+n) for a in range(n*n-n) if a % n != n-1]
    obj = demo.mesh_object(scene, "Cloth", points, faces, "CLOTH")
    pins = obj.vertex_groups.new(name="Pins")
    pins.add([n*(n-1), n*n-1], 1, "REPLACE")
    obj.uipc_body.pin_group = pins.name
    return obj


if not args.measure:
    for name in ("cloth", "abd", "mixed"):
        scene = bpy.data.scenes.new(name)
        bpy.context.window.scene = scene
        scene.frame_start, scene.frame_end, scene.render.fps = 7, 37, 30
        settings = scene.uipc_settings
        settings.python_executable = args.python
        settings.cache_directory = str(args.output / "export")
        settings.solver_accuracy = "CONVERGED"
        demo.mesh_object(scene, "Floor", [(-5,-5,0), (5,-5,0), (5,5,0), (-5,5,0)], [(0,1,2,3)], "STATIC")
        if name in ("cloth", "mixed"):
            cloth(scene, 33, (0, 0, .8))
        if name in ("abd", "mixed"):
            for index in range(6 if name == "abd" else 1):
                bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=.15,
                    location=(-1.2 + .45*index, 1, .45))
                obj = bpy.context.object
                obj.name = f"Sphere {index}"
                obj.uipc_body.role = "RIGID"
                obj.uipc_body.fixed = index >= 4
        if name == "mixed" and args.urdf:
            assert bpy.ops.uipc.import_robot(filepath=str(args.urdf.resolve()), blocking=True) == {"FINISHED"}
            root = settings.active_robot
            root.location = (2, 0, 1)
            root.keyframe_insert("location", frame=7)
            root.location.z = .9
            root.keyframe_insert("location", frame=37)
        scene.frame_set(7)
        bpy.context.view_layer.update()
        directory, request = addon.bridge.export_job(scene)
        frozen = args.output / name
        frozen.mkdir()  # Never overwrite a baseline's physical inputs.
        for path in directory.iterdir():
            if path.name == "request.json" or path.suffix == ".npz":
                shutil.copy2(path, frozen / path.name)
    # A dense preview-only mesh stresses topology/normal work, not the solver.
    scene = bpy.data.scenes.new("preview")
    bpy.context.window.scene = scene
    cloth(scene, 129, (0, 0, 1))
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "benchmarks.blend"))
else:
    bpy.ops.wm.open_mainfile(filepath=str(args.output / "benchmarks.blend"))

scene = bpy.data.scenes["preview"]
bpy.context.window.scene = scene
obj = scene.objects["Cloth"] if "Cloth" in scene.objects else next(iter(scene.objects))
# Default public API includes thickness guides, as in extension 0.5.
durations = []
for _ in range(21):
    start = time.perf_counter()
    data = addon.preview.simulation_preview(obj, scene)
    durations.append(time.perf_counter() - start)
assert len(data["points"]) == 129**2
summary = {"blender": bpy.app.version_string, "vertices": len(data["points"]),
           "cold_seconds": durations[0], "warm_median_seconds": float(np.median(durations[1:])),
           "warm_min_seconds": min(durations[1:]), "warm_max_seconds": max(durations[1:]),
           "frontend": {s.name: addon.performance.frontend_report(s) for s in bpy.data.scenes}}
(args.output / f"preview_{args.measure or 'before'}.json").write_text(json.dumps(summary, indent=2))
addon.unregister()
print("PERFORMANCE_INPUTS_OK", json.dumps(summary))
