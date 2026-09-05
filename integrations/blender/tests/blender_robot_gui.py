# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Inspect the completed robot bake in an installed extension's real UI."""

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


parser = argparse.ArgumentParser()
parser.add_argument("--module", required=True)
parser.add_argument("--python", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
args.output.mkdir(parents=True, exist_ok=True)
(args.output / "temp").mkdir(exist_ok=True)
bpy.context.preferences.filepaths.temporary_directory = str(args.output / "temp")
addon = importlib.import_module(args.module)
state = {"stage": "open", "frames": {}, "ticks": 0}
started = time.monotonic()


def center(obj, scene):
    deps = scene.view_layers[0].depsgraph
    deps.update()
    evaluated = obj.evaluated_get(deps)
    mesh = evaluated.to_mesh()
    try:
        return np.mean([evaluated.matrix_world @ v.co for v in mesh.vertices], axis=0)
    finally:
        evaluated.to_mesh_clear()


def tick():
    try:
        state["ticks"] += 1
        addon.protocol.atomic_json(args.output / "gui_progress.json", state)
        if time.monotonic() - started > 120:
            raise TimeoutError("Robot GUI validation timed out")
        if state["stage"] == "open":
            assert not bpy.app.background
            bpy.ops.wm.open_mainfile(filepath=os.environ["UIPC_ROBOT_GUI_SCENE"])
            state["stage"] = "frames"
            state["pending"] = [1, 50, 195, 250, 325, 420, 500]
            return 0.5
        scene = bpy.context.scene
        if state["stage"] == "frames":
            frame = state["pending"].pop(0)
            scene.frame_set(frame)
            assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
            state["frames"][str(frame)] = center(
                scene.objects["06 Apple 5"], scene
            ).tolist()
            if state["pending"]:
                return 0.2
            assert state["frames"]["250"][2] > 1.0
            assert abs(state["frames"]["500"][1]) < 0.04
            assert 0.78 < state["frames"]["500"][2] < 0.82
            scene.frame_set(250)
            selected = scene.objects["Robot link fingertip"]
            for obj in scene.objects:
                obj.select_set(obj == selected)
            scene.view_layers[0].objects.active = selected
            assert selected.uipc_body.driven and selected.uipc_body.drive_target
            for area in bpy.context.screen.areas:
                if area.type == "VIEW_3D":
                    area.spaces.active.region_3d.view_perspective = "CAMERA"
                    area.spaces.active.show_region_ui = True
                    area.spaces.active.overlay.show_extras = False
                    area.spaces.active.shading.type = "MATERIAL"
                    area.tag_redraw()
            state["stage"] = "tab"
            state["attempt"] = 0
            return 3.0
        if state["stage"] == "tab":
            area = next(
                area for area in bpy.context.screen.areas if area.type == "VIEW_3D"
            )
            region = next(region for region in area.regions if region.type == "UI")
            if region.active_panel_category == "libuipc":
                state["stage"] = "capture"
                return 1.0
            offsets = [205, 225, 245, 265, 185, 285, 305, 165]
            if state["attempt"] >= len(offsets):
                raise AssertionError(
                    f"Cannot activate the libuipc panel: {region.active_panel_category}"
                )
            x = region.x + region.width - 12
            y = area.y + area.height - offsets[state["attempt"]]
            state["attempt"] += 1
            window = bpy.context.window
            window.event_simulate(type="MOUSEMOVE", value="NOTHING", x=x, y=y)
            window.event_simulate(type="LEFTMOUSE", value="PRESS", x=x, y=y)
            window.event_simulate(type="LEFTMOUSE", value="RELEASE", x=x, y=y)
            return 1.0
        if state["stage"] == "capture":
            assert bpy.ops.screen.screenshot(
                filepath=str(args.output / "robot_blender_ui.png")
            ) == {"FINISHED"}
            state.update(success=True, blender=bpy.app.version_string, stage="complete")
            (args.output / "gui_validation.json").write_text(
                json.dumps(state, indent=2), encoding="utf-8"
            )
            bpy.ops.wm.quit_blender()
            return None
    except Exception:
        state.update(success=False, error=traceback.format_exc())
        (args.output / "gui_validation.json").write_text(
            json.dumps(state, indent=2), encoding="utf-8"
        )
        traceback.print_exc()
        bpy.ops.wm.quit_blender()
        return None
    return 0.2


bpy.app.timers.register(tick, first_interval=1.0, persistent=True)
