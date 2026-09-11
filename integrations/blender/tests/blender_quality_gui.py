# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Isolated real window: new panels, asynchronous bake and GPU preview drawing."""

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
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
args.output.mkdir(parents=True, exist_ok=True)
(args.output / "temp").mkdir(exist_ok=True)
bpy.context.preferences.filepaths.temporary_directory = str(args.output / "temp")
addon = importlib.import_module(args.module)
quality_ui = importlib.import_module(args.module + ".quality_ui")
started = time.monotonic()
state = {"stage": "begin", "draws": 0, "ticks": 0, "success": False}
observer = None


def draw_observer():
    if addon.preview.draw():
        state["draws"] += 1
        before = addon.preview.statistics()
        repeated = addon.preview.draw()
        state["gpu_reuse_verified"] = bool(repeated and before == addon.preview.statistics())


def popup(name):
    area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
    region = next(r for r in area.regions if r.type == "WINDOW")
    window = bpy.context.window
    window.event_simulate(type="MOUSEMOVE", value="NOTHING", x=area.x + 450, y=area.y + 450)
    with bpy.context.temp_override(area=area, region=region):
        bpy.ops.wm.call_panel("INVOKE_DEFAULT", name=name, keep_open=True)


def tick():
    global observer
    try:
        state["ticks"] += 1
        if time.monotonic() - started > 100:
            raise TimeoutError("Quality UI test timed out")
        if state["stage"] == "begin":
            assert not bpy.app.background
            assert bpy.ops.uipc.create_demo() == {"FINISHED"}
            scene = bpy.context.scene
            scene.frame_end = 9
            scene.uipc_settings.python_executable = args.python
            scene.uipc_settings.cache_directory = str(args.output / "cache")
            cloth = scene.objects["Pinned Cloth"]
            cloth.uipc_body.stretch_poisson = .4
            cloth.uipc_body.shear_poisson = .45
            cloth.uipc_body.bending_poisson = .2
            for obj in scene.objects:
                obj.select_set(obj == cloth)
            bpy.context.view_layer.objects.active = cloth
            assert bpy.ops.uipc.bake() == {"FINISHED"}
            state["stage"] = "baking"
            return .2
        scene = bpy.context.scene
        if state["stage"] == "baking":
            if addon.runtime.is_running():
                return .2
            assert scene.uipc_settings.last_bake, scene.uipc_settings.status
            report = quality_ui.load_report(scene)
            entry = next(o for o in report["objects"] if o["name"] == "Pinned Cloth")
            assert bpy.ops.uipc.quality_jump(index=entry["index"], metric="speed") == {"FINISHED"}
            scene.uipc_settings.show_physics_overlay = True
            scene.uipc_settings.show_pin_overlay = True
            scene.uipc_settings.show_thickness_overlay = True
            area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
            area.spaces.active.region_3d.view_perspective = "CAMERA"
            area.spaces.active.shading.type = "SOLID"
            area.spaces.active.overlay.show_extras = False
            area.tag_redraw()
            observer = bpy.types.SpaceView3D.draw_handler_add(draw_observer, (), "WINDOW", "POST_VIEW")
            popup("UIPC_PT_quality")
            state["stage"] = "quality_capture"
            return 2.0
        if state["stage"] == "quality_capture":
            assert bpy.ops.screen.screenshot(filepath=str(args.output / "quality_ui.png")) == {"FINISHED"}
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "material_popup"
            return .5
        if state["stage"] == "material_popup":
            popup("UIPC_PT_body")
            state["stage"] = "material_capture"
            return 1.0
        if state["stage"] == "material_capture":
            assert bpy.ops.screen.screenshot(filepath=str(args.output / "material_ui.png")) == {"FINISHED"}
            assert state["draws"] > 0, "GPU overlay never completed a draw"
            assert state.get("gpu_reuse_verified"), "Repeated draw rebuilt CPU/GPU preview data"
            assert "uipc" not in sys.modules
            # Close popup-owned RNA widgets before unregistering their types.
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "affine_popup"
            return .5
        if state["stage"] == "affine_popup":
            body = scene.objects["Falling ABD"]
            assert addon.bridge.affine_playback.is_affine(body.modifiers[addon.protocol.MODIFIER_NAME])
            for obj in scene.objects:
                obj.select_set(obj == body)
            bpy.context.view_layer.objects.active = body
            state["affine_draws_before"] = state["draws"]
            popup("UIPC_PT_scene")
            state["stage"] = "affine_capture"
            return 1.0
        if state["stage"] == "affine_capture":
            assert state["draws"] > state["affine_draws_before"] and state["gpu_reuse_verified"]
            state["affine_preview_verified"] = True
            assert bpy.ops.screen.screenshot(filepath=str(args.output / "affine_ui.png")) == {"FINISHED"}
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "rod_popup"
            return .5
        if state["stage"] == "rod_popup":
            rod_scene = bpy.data.scenes.new("Rod UI controls")
            bpy.context.window.scene = rod_scene
            mesh = bpy.data.meshes.new("Rod UI centerline")
            mesh.from_pydata([(i*.04,0,.5) for i in range(8)],[(i,i+1) for i in range(7)],[])
            body = bpy.data.objects.new("Rod UI",mesh)
            rod_scene.collection.objects.link(body)
            body.uipc_body.role,body.uipc_body.thickness = "ROD",.005
            pins = body.vertex_groups.new(name="Pins")
            pins.add([0,1],1,"REPLACE")
            body.uipc_body.pin_group = pins.name
            body.select_set(True)
            bpy.context.view_layer.objects.active = body
            rod_scene.uipc_settings.show_physics_overlay = True
            rod_scene.uipc_settings.show_pin_overlay = True
            rod_scene.uipc_settings.show_thickness_overlay = True
            area = next(a for a in bpy.context.screen.areas if a.type == "VIEW_3D")
            area.spaces.active.region_3d.view_perspective = "ORTHO"
            area.spaces.active.region_3d.view_location = (.14,0,.5)
            area.spaces.active.region_3d.view_distance = .7
            state["rod_draws_before"] = state["draws"]
            popup("UIPC_PT_body")
            state["stage"] = "rod_capture"
            return 1.0
        if state["stage"] == "rod_capture":
            assert state["draws"] > state["rod_draws_before"] and state["gpu_reuse_verified"]
            state["rod_preview_verified"] = True
            assert bpy.ops.screen.screenshot(filepath=str(args.output / "rod_ui.png")) == {"FINISHED"}
            bpy.context.window.event_simulate(type="ESC", value="PRESS")
            state["stage"] = "closing_popups"
            return 1.0
        if state["stage"] == "closing_popups":
            bpy.types.SpaceView3D.draw_handler_remove(observer, "WINDOW")
            if args.module in bpy.context.preferences.addons:
                assert bpy.ops.preferences.addon_disable(module=args.module) == {"FINISHED"}
            else:
                addon.unregister()
            assert not hasattr(bpy.types.Object, "uipc_body")
            state["stage"] = "unregistered"
            return 1.0
        if state["stage"] == "unregistered":
            state.update(success=True, stage="complete", blender=bpy.app.version_string)
            (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
            bpy.ops.wm.quit_blender()
            return None
    except Exception:
        state.update(error=traceback.format_exc(), success=False)
        (args.output / "gui_validation.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
        traceback.print_exc()
        addon.runtime.stop()
        bpy.ops.wm.quit_blender()
        return None
    return .2


bpy.app.timers.register(tick, first_interval=1.0)
