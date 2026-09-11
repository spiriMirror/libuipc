# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Contact-pair editing and scene-local reusable physical material presets."""

import json

import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty, StringProperty

from .materials import PRESET_FIELDS, validate_preset


def physics_changed(self, context):
    from . import changed
    changed(self, context)


class UIPCContactPair(bpy.types.PropertyGroup):
    material_a: StringProperty(name="Material A", default="Default", update=physics_changed)
    material_b: StringProperty(name="Material B", default="Default", update=physics_changed)
    friction: FloatProperty(name="Friction", default=0.5, min=0, soft_max=2, update=physics_changed)
    resistance: FloatProperty(name="Resistance (Pa)", default=1e9, min=1, soft_max=1e10, update=physics_changed)
    enabled: BoolProperty(name="Contact Enabled", default=True, update=physics_changed)


class UIPCPhysicalPreset(bpy.types.PropertyGroup):
    payload: StringProperty(options={"HIDDEN"})


class UIPC_OT_contact_pair_add(bpy.types.Operator):
    bl_idname = "uipc.contact_pair_add"
    bl_label = "Add Contact Pair"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        from . import runtime, changed
        if runtime.is_running():
            return {"CANCELLED"}
        settings = context.scene.uipc_settings
        pair = settings.contact_pairs.add()
        pair.friction, pair.resistance = settings.friction, settings.resistance
        changed(settings, context)
        return {"FINISHED"}


class UIPC_OT_contact_pair_remove(bpy.types.Operator):
    bl_idname = "uipc.contact_pair_remove"
    bl_label = "Remove Contact Pair"
    bl_options = {"REGISTER", "UNDO"}
    index: IntProperty()

    def execute(self, context):
        from . import runtime, changed
        settings = context.scene.uipc_settings
        if runtime.is_running() or not 0 <= self.index < len(settings.contact_pairs):
            return {"CANCELLED"}
        settings.contact_pairs.remove(self.index)
        changed(settings, context)
        return {"FINISHED"}


class UIPC_OT_preset_save(bpy.types.Operator):
    bl_idname = "uipc.material_preset_save"
    bl_label = "Save Physical Preset"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        from . import bridge, runtime
        obj = context.object
        if runtime.is_running() or obj is None or obj.type != "MESH" or obj.uipc_body.role not in PRESET_FIELDS:
            return {"CANCELLED"}
        name = obj.uipc_body.preset_name.strip()
        if not name:
            self.report({"ERROR"}, "Enter a physical preset name")
            return {"CANCELLED"}
        material = bridge.object_material(obj)
        payload = validate_preset({"schema_version": 1, "role": material["role"],
                                  "values": {k: material[k] for k in PRESET_FIELDS[material["role"]]}})
        presets = context.scene.uipc_settings.material_presets
        preset = presets.get(name)
        if preset is None:
            preset = presets.add()
            preset.name = name
        preset.payload = json.dumps(payload, allow_nan=False)
        self.report({"INFO"}, f"Saved physical preset: {name}")
        return {"FINISHED"}


class UIPC_OT_preset_apply(bpy.types.Operator):
    bl_idname = "uipc.material_preset_apply"
    bl_label = "Apply Preset to Selected"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        from . import runtime
        if runtime.is_running() or context.object is None:
            return {"CANCELLED"}
        preset = context.scene.uipc_settings.material_presets.get(context.object.uipc_body.preset_name.strip())
        try:
            if preset is None:
                raise ValueError("Choose a saved physical preset")
            payload = validate_preset(json.loads(preset.payload))
            targets = list(context.selected_objects) or [context.object]
            if any(o.type != "MESH" or o.uipc_body.role != payload["role"] for o in targets):
                raise ValueError("All selected objects must have the preset's simulation role")
            snapshots = [(o, {k: getattr(o.uipc_body, k) for k in payload["values"]}) for o in targets]
            try:
                for obj in targets:
                    for key, value in payload["values"].items():
                        setattr(obj.uipc_body, key, value)
            except Exception:
                for obj, values in snapshots:
                    for key, value in values.items():
                        setattr(obj.uipc_body, key, value)
                raise
        except (ValueError, TypeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Applied preset to {len(targets)} objects; pins and drive targets unchanged")
        return {"FINISHED"}


def draw_contact_pairs(layout, settings):
    box = layout.box()
    box.label(text="Contact Material Pairs (blank = Default)")
    box.operator("uipc.contact_pair_add")
    for index, pair in enumerate(settings.contact_pairs):
        row = box.row(align=True)
        row.prop(pair, "material_a", text="")
        row.prop(pair, "material_b", text="")
        row.operator("uipc.contact_pair_remove", text="", icon="X").index = index
        row = box.row(align=True)
        row.prop(pair, "friction")
        row.prop(pair, "resistance")
        row.prop(pair, "enabled", text="Contact")


def draw_material_preset(layout, scene, body):
    box = layout.box()
    box.label(text="Physical Material Presets (saved in this .blend)")
    box.prop_search(body, "preset_name", scene.uipc_settings, "material_presets", text="Preset")
    row = box.row(align=True)
    row.operator("uipc.material_preset_save", text="Save / Replace")
    row.operator("uipc.material_preset_apply", text="Apply to Selected")


MATERIAL_CLASSES = (UIPCContactPair, UIPCPhysicalPreset, UIPC_OT_contact_pair_add,
                    UIPC_OT_contact_pair_remove, UIPC_OT_preset_save, UIPC_OT_preset_apply)
