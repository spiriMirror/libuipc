# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Installed GUI: async PNG queue and joint panel, without touching user profiles."""

import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

import bpy
import numpy as np
from mathutils import Euler

parser = argparse.ArgumentParser()
parser.add_argument("--module", required=True)
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output.mkdir(parents=True, exist_ok=True)
(args.output / "temp").mkdir(exist_ok=True)
bpy.context.preferences.filepaths.temporary_directory = str(args.output / "temp")
addon = importlib.import_module(args.module)
state = {"stage": "begin", "ticks": 0, "success": False}
started = time.monotonic()


def popup(name):
    area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
    region = next(r for r in area.regions if r.type == "WINDOW")
    bpy.context.window.event_simulate(type="MOUSEMOVE", value="NOTHING", x=area.x + 550, y=area.y + 450)
    with bpy.context.temp_override(area=area, region=region):
        bpy.ops.wm.call_panel("INVOKE_DEFAULT", name=name, keep_open=True)


def tick():
    try:
        state["ticks"] += 1
        if time.monotonic() - started > 120:
            raise TimeoutError("Installed workflow UI timed out")
        if state["stage"] == "begin":
            assert bpy.ops.uipc.create_demo() == {"FINISHED"}
            scene = bpy.context.scene
            scene.frame_end = 3
            scene.uipc_settings.python_executable = args.python
            scene.uipc_settings.cache_directory = str(args.output / "cache")
            assert bpy.ops.uipc.bake() == {"FINISHED"}
            state["stage"] = "baking"
            return .2
        if state["stage"] == "baking":
            if addon.runtime.is_running():
                return .2
            scene = bpy.context.scene
            addon.bridge.check_cache(scene)
            scene.render.engine = "BLENDER_EEVEE_NEXT"
            scene.render.resolution_x = scene.render.resolution_y = 64
            scene.render.resolution_percentage = 100
            scene.render.image_settings.file_format = "PNG"
            scene.uipc_settings.render_first, scene.uipc_settings.render_last = 2, 3
            scene.uipc_settings.render_directory = str(args.output / "renders")
            bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "queue_source.blend"))
            assert bpy.ops.uipc.render_queue() == {"FINISHED"}
            assert addon.runtime.is_running()
            state["stage"] = "rendering"
            return .2
        if state["stage"] == "rendering":
            if addon.runtime.is_running():
                return .2
            scene = bpy.context.scene
            status = addon.protocol.read_json(Path(scene.uipc_settings.last_render_job) / "render_status.json")
            assert status["state"] == "complete" and status["done"] == 2, scene.uipc_settings.status
            state["rendered_pngs"] = status["done"]
            popup("UIPC_PT_render_queue")
            state["stage"] = "queue_capture"
            return 1.0
        if state["stage"] == "queue_capture":
            bpy.ops.screen.screenshot(filepath=str(args.output / "render_queue_ui.png"))
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "open_robot"
            return .5
        if state["stage"] == "open_robot":
            bpy.ops.wm.open_mainfile(filepath=os.environ["UIPC_WORKFLOW_ROBOT_SCENE"])
            state["stage"] = "robot_panel"
            return .5
        if state["stage"] == "robot_panel":
            scene = bpy.context.scene
            root = scene.uipc_settings.active_robot
            assert root and len(root.uipc_robot.joints) >= 16
            addon.bridge.activate_cache(scene)
            for obj in scene.objects:
                obj.select_set(obj == root)
                if obj.type == "MESH" and not obj.get("uipc_robot_link"):
                    obj.hide_viewport = True
            bpy.context.view_layer.objects.active = root
            assert bpy.ops.uipc.robot_refresh() == {"FINISHED"}
            assert bpy.ops.uipc.robot_review() == {"FINISHED"}
            graph = bpy.context.evaluated_depsgraph_get()
            points = []
            for obj in scene.objects:
                if obj.get("uipc_robot_link"):
                    evaluated = obj.evaluated_get(graph)
                    corners = np.asarray(evaluated.bound_box)
                    matrix = np.asarray(evaluated.matrix_world)
                    points.extend(corners @ matrix[:3, :3].T + matrix[:3, 3])
            points = np.asarray(points)
            for area in bpy.context.screen.areas:
                if area.type == "VIEW_3D":
                    space = area.spaces.active
                    space.region_3d.view_perspective = "PERSP"
                    space.region_3d.view_location = points.mean(axis=0)
                    space.region_3d.view_distance = float(np.ptp(points, axis=0).max() * 3)
                    space.region_3d.view_rotation = Euler((1.0, 0, .5)).to_quaternion()
                    space.shading.type = "SOLID"
                    space.overlay.show_extras = False
                    area.tag_redraw()
            popup("UIPC_PT_robot_controls")
            state["joint_rows"] = len(root.uipc_robot.joints)
            state["stage"] = "robot_capture"
            return 1.0
        if state["stage"] == "robot_capture":
            bpy.ops.screen.screenshot(filepath=str(args.output / "robot_controls_ui.png"))
            assert "uipc" not in sys.modules
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "disable"
            return .5
        if state["stage"] == "disable":
            bpy.ops.preferences.addon_disable(module=args.module)
            assert not hasattr(bpy.types.Object, "uipc_body")
            assert not hasattr(bpy.types.Object, "uipc_robot")
            state["stage"] = "complete"
            return .5
        if state["stage"] == "complete":
            state.update(success=True, blender=bpy.app.version_string)
            (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
            bpy.ops.wm.quit_blender()
            return None
    except Exception:
        state.update(success=False, error=traceback.format_exc())
        (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
        traceback.print_exc()
        addon.runtime.stop()
        bpy.ops.wm.quit_blender()
        return None
    return .2


bpy.app.timers.register(tick, first_interval=1, persistent=True)
