# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Separate render ranges and multi-camera snapshot queue controls."""

import bpy
from bpy.props import BoolProperty, IntProperty, PointerProperty

from . import render_queue


class UIPCRenderCamera(bpy.types.PropertyGroup):
    camera: PointerProperty(type=bpy.types.Object, name="Camera", poll=lambda self, obj: obj.type == "CAMERA")
    enabled: BoolProperty(name="Enabled", default=True)


class UIPC_OT_render_camera_add(bpy.types.Operator):
    bl_idname = "uipc.render_camera_add"
    bl_label = "Add Camera"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        from . import runtime
        if runtime.is_running():
            return {"CANCELLED"}
        row = context.scene.uipc_settings.render_cameras.add()
        row.camera = context.object if context.object and context.object.type == "CAMERA" else context.scene.camera
        return {"FINISHED"}


class UIPC_OT_render_camera_remove(bpy.types.Operator):
    bl_idname = "uipc.render_camera_remove"
    bl_label = "Remove Camera"
    bl_options = {"REGISTER", "UNDO"}
    index: IntProperty()

    def execute(self, context):
        from . import runtime
        rows = context.scene.uipc_settings.render_cameras
        if runtime.is_running() or not 0 <= self.index < len(rows):
            return {"CANCELLED"}
        rows.remove(self.index)
        return {"FINISHED"}


class UIPC_OT_render_queue(bpy.types.Operator):
    bl_idname = "uipc.render_queue"
    bl_label = "Start PNG Render Queue"
    bl_description = "Render an immutable scene snapshot; resume ignores current scene edits and checks existing PNG receipts"
    resume: BoolProperty(default=False)
    blocking: BoolProperty(default=False, options={"HIDDEN", "SKIP_SAVE"})

    @classmethod
    def poll(cls, context):
        from . import runtime
        return not runtime.is_running()

    def execute(self, context):
        from . import _poll_timer
        try:
            if self.blocking or bpy.app.background:
                render_queue.run_blocking(context.scene, resume=self.resume)
            else:
                render_queue.start(context.scene, resume=self.resume)
                if not bpy.app.timers.is_registered(_poll_timer):
                    bpy.app.timers.register(_poll_timer, first_interval=.2)
        except Exception as error:
            context.scene.uipc_settings.status = str(error)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_PT_render_queue(bpy.types.Panel):
    bl_label = "PNG Render Queue"
    bl_idname = "UIPC_PT_render_queue"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "libuipc"

    def draw(self, context):
        from . import runtime
        layout, settings = self.layout, context.scene.uipc_settings
        layout.ui_units_x = 26
        layout.enabled = not runtime.is_running()
        layout.label(text="Render range does not change the bake range")
        row = layout.row(align=True)
        row.prop(settings, "render_first")
        row.prop(settings, "render_last")
        layout.prop(settings, "render_directory")
        layout.label(text="Empty camera list uses the active scene camera")
        layout.operator("uipc.render_camera_add")
        for index, shot in enumerate(settings.render_cameras):
            row = layout.row(align=True)
            row.prop(shot, "enabled", text="")
            row.prop(shot, "camera", text="")
            row.operator("uipc.render_camera_remove", text="", icon="X").index = index
        layout.operator("uipc.render_queue", text="Validate & Queue Snapshot").resume = False
        layout.prop(settings, "last_render_job", text="Resume Job")
        layout.operator("uipc.render_queue", text="Resume Saved Snapshot").resume = True
        layout.label(text="Resume uses the saved snapshot, not current edits")


RENDER_CLASSES = (UIPCRenderCamera, UIPC_OT_render_camera_add, UIPC_OT_render_camera_remove,
                  UIPC_OT_render_queue, UIPC_PT_render_queue)
