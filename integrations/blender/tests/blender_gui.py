# SPDX-License-Identifier: Apache-2.0
"""Run in a separate Blender window to exercise timers, UI, and cancellation."""

import argparse
import importlib
import json
from pathlib import Path
import sys
import time
import traceback

import bpy


parser = argparse.ArgumentParser()
parser.add_argument("--module", required=True)
parser.add_argument("--python", required=True)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output.mkdir(parents=True, exist_ok=True)
(args.output / "temp").mkdir(exist_ok=True)
bpy.context.preferences.filepaths.temporary_directory = str(args.output / "temp")
addon = importlib.import_module(args.module)
started = time.monotonic()
state = {"stage": "begin", "heartbeats": 0, "success": False}


def tick():
    try:
        if time.monotonic() - started > 150:
            raise TimeoutError("Interactive Blender validation timed out")
        state["heartbeats"] += 1
        if state["stage"] == "begin":
            assert not bpy.app.background
            assert bpy.ops.uipc.create_demo() == {"FINISHED"}
            scene = bpy.context.scene
            scene.uipc_settings.python_executable = args.python
            scene.uipc_settings.cache_directory = str(args.output / "cache")
            assert bpy.ops.uipc.bake() == {"FINISHED"}
            assert addon.runtime.is_running()
            state["stage"] = "baking"
            return 0.1
        scene = bpy.context.scene
        if state["stage"] == "baking":
            if addon.runtime.is_running():
                return 0.1
            assert scene.uipc_settings.last_bake, scene.uipc_settings.status
            assert state["heartbeats"] > 3
            assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
            state["bake_heartbeats"] = state["heartbeats"]
            state["completed_cache"] = scene.uipc_settings.last_bake
            assert bpy.ops.uipc.bake() == {"FINISHED"}
            assert bpy.ops.uipc.cancel() == {"FINISHED"}
            state["stage"] = "cancelling"
            return 0.1
        if state["stage"] == "cancelling":
            if addon.runtime.is_running():
                return 0.1
            assert scene.uipc_settings.last_bake == state["completed_cache"]
            assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
            scene.frame_set(61)
            for obj in bpy.context.view_layer.objects:
                obj.select_set(False)
            cloth = next(o for o in scene.objects if o.uipc_body.role == "CLOTH")
            cloth.select_set(True)
            bpy.context.view_layer.objects.active = cloth
            for area in bpy.context.screen.areas:
                if area.type == "VIEW_3D":
                    space = area.spaces.active
                    space.region_3d.view_perspective = "CAMERA"
                    space.show_region_ui = True
                    space.shading.type = "MATERIAL"
                    area.tag_redraw()
            bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "libuipc_gui_demo.blend"))
            state["stage"] = "screenshot"
            return 3.0
        if state["stage"] == "screenshot":
            # In this fixed-size 4.5 test window, activate the fourth sidebar tab
            # with Blender's own event queue, without global OS mouse input.
            area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
            state["tab_attempt"] = 0
            state["stage"] = "activate_tab"
            return 0.2
        if state["stage"] == "activate_tab":
            area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
            region = next(r for r in area.regions if r.type == "UI")
            if region.active_panel_category == "libuipc":
                state["stage"] = "capture"
                return 1.0
            offsets = [205, 225, 245, 265, 185, 285, 305, 165]
            attempt = state["tab_attempt"]
            if attempt >= len(offsets):
                raise AssertionError(f"Sidebar did not activate: {region.active_panel_category}; {state}")
            x, y = region.x + region.width - 12, area.y + area.height - offsets[attempt]
            state["area_bounds"] = [area.x, area.y, area.width, area.height]
            state["region_bounds"] = [region.x, region.y, region.width, region.height]
            state["click"] = [x, y]
            window = bpy.context.window
            window.event_simulate(type="MOUSEMOVE", value="NOTHING", x=x, y=y)
            window.event_simulate(type="LEFTMOUSE", value="PRESS", x=x, y=y)
            window.event_simulate(type="LEFTMOUSE", value="RELEASE", x=x, y=y)
            state["tab_attempt"] += 1
            return 1.0
        if state["stage"] == "capture":
            area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
            region = next(r for r in area.regions if r.type == "UI")
            assert region.active_panel_category == "libuipc", region.active_panel_category
            state["sidebar_category"] = region.active_panel_category
            assert bpy.ops.screen.screenshot(filepath=str(args.output / "blender_plugin.png")) == {"FINISHED"}
            # Disabling the extension must kill only its own active worker.
            assert bpy.ops.uipc.bake() == {"FINISHED"}
            process = addon.runtime._job["process"]
            addon.unregister()
            assert process.poll() is not None
            addon.register()
            state.update(success=True, stage="complete", blender=bpy.app.version_string)
            (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
            bpy.ops.wm.quit_blender()
            return None
    except Exception:
        state["error"] = traceback.format_exc()
        (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
        traceback.print_exc()
        bpy.ops.screen.screenshot(filepath=str(args.output / "blender_error.png"))
        addon.runtime.stop()
        bpy.ops.wm.quit_blender()
        return None
    return 0.1


bpy.app.timers.register(tick, first_interval=1.0)
