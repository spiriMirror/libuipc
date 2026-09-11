# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Actual sample-87 import: joint sliders, pose/keying, review and native bake."""

import argparse
import importlib
import json
import math
from pathlib import Path
import sys

import bpy
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--python", required=True)
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
addon = importlib.import_module("libuipc_blender")
addon.register()
controls = importlib.import_module("libuipc_blender.robot_controls")
scene = bpy.context.scene
scene.frame_start, scene.frame_end, scene.render.fps = 1, 31, 30
scene.uipc_settings.python_executable = args.python
scene.uipc_settings.cache_directory = str(args.output / "cache")
assert bpy.ops.uipc.import_robot(filepath=str(args.urdf.resolve()), blocking=True) == {"FINISHED"}
root = scene.uipc_settings.active_robot
assert root and len(root.uipc_robot.joints) >= 16
fixed = bpy.data.objects.new("Synthetic fixed control", None)
scene.collection.objects.link(fixed)
fixed.parent, fixed.rotation_mode = root, "AXIS_ANGLE"
fixed["uipc_joint_name"], fixed["uipc_joint_type"] = "synthetic_fixed", "fixed"
assert bpy.ops.uipc.robot_refresh() == {"FINISHED"}
fixed_entry = root.uipc_robot.joints["synthetic_fixed"]
fixed_entry.angle = 45
assert fixed.rotation_axis_angle[0] == 0
movable = [e for e in root.uipc_robot.joints if e.control.get("uipc_joint_type") == "revolute"]
assert len(movable) == 16
links = [o for o in scene.objects if o.get("uipc_robot_link")]
initial = [np.array(o.matrix_world) for o in links]
entry = movable[0]
upper = entry.control["uipc_joint_limits"][1]
entry.angle = math.degrees(upper + 1)
assert abs(entry.control.rotation_axis_angle[0] - upper) < 1e-6
assert any(np.max(np.abs(np.array(o.matrix_world) - old)) > 1e-4 for o, old in zip(links, initial))
for entry in movable:
    entry.angle = 0
root.uipc_robot.pose_name = "Open"
assert bpy.ops.uipc.robot_pose_save() == {"FINISHED"}
assert bpy.ops.uipc.robot_key() == {"FINISHED"}
scene.frame_set(31)
for entry in movable:
    low, high = entry.control["uipc_joint_limits"]
    entry.angle = math.degrees(low + .6 * (high - low))
root.uipc_robot.pose_name = "Grasp"
assert bpy.ops.uipc.robot_pose_save() == {"FINISHED"}
assert bpy.ops.uipc.robot_key(include_root=True) == {"FINISHED"}
assert all(e.control.animation_data.action for e in movable)
before = (scene.frame_current, scene.frame_subframe, [float(e.control.rotation_axis_angle[0]) for e in movable])
assert bpy.ops.uipc.robot_review() == {"FINISHED"}
assert before == (scene.frame_current, scene.frame_subframe, [float(e.control.rotation_axis_angle[0]) for e in movable])
review = json.loads(root.uipc_robot.review)
assert len(review["rows"]) == 16
assert all(row["limit_frame"] is None for row in review["rows"])
root.name = "Renamed hand"
movable[0].control.name = "Renamed first joint"
assert bpy.ops.uipc.robot_refresh() == {"FINISHED"}
root.uipc_robot.pose_name = "Open"
assert bpy.ops.uipc.robot_pose_apply() == {"FINISHED"}
root.uipc_robot.pose_name = "Grasp"
assert bpy.ops.uipc.robot_pose_apply() == {"FINISHED"}
scene.frame_end = 3
scene.frame_set(1)
assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
addon.bridge.check_cache(scene, verify_data=True)
assert len(json.loads(root["uipc_robot_links"])) == 17
path = args.output / "robot_controls.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(path))
bpy.ops.wm.open_mainfile(filepath=str(path))
root = bpy.context.scene.uipc_settings.active_robot
assert len(root.uipc_robot.poses) == 2
assert len(root.uipc_robot.joints) == 17
addon.bridge.check_cache(bpy.context.scene, verify_data=True)
addon.unregister()
print("PASS: 16 joint sliders, fixed-joint lock, pose/key persistence, trajectory review and native hand bake")
