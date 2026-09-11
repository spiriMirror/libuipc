# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Read verified quality reports and navigate output-frame observations."""

import json
from pathlib import Path

import bpy
from bpy.props import EnumProperty, IntProperty

from .protocol import read_json, file_sha256


def load_report(scene, validate=True):
    from . import bridge
    if validate:
        bridge.check_cache(scene)
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = read_json(directory / "request.json")
    result = read_json(directory / "result.json")
    path = directory / "quality_report.json"
    expected = result.get("quality_report_sha256")
    if not expected or not path.is_file():
        raise ValueError("This bake has no quality report; rebake with extension 0.4 or newer")
    if file_sha256(path) != expected:
        raise ValueError("Quality report checksum mismatch")
    report = read_json(path)
    if report.get("schema_version") != 1 or report.get("fingerprint") != request["fingerprint"]:
        raise ValueError("Quality report provenance mismatch")
    if [o["index"] for o in report["objects"]] != [o["index"] for o in result["objects"]]:
        raise ValueError("Quality report object list does not match the bake")
    from .identity import resolve_object
    for entry in report["objects"]:
        entry["name"] = resolve_object(scene, request["objects"][entry["index"]], request["schema_version"]).name
    report["performance"] = result.get("performance", {})
    scene.uipc_settings.quality_summary = json.dumps(report, allow_nan=False)
    scene.uipc_settings.quality_bake = scene.uipc_settings.last_bake
    return report


class UIPC_OT_quality_load(bpy.types.Operator):
    bl_idname = "uipc.quality_load"
    bl_label = "Load Quality Report"

    def execute(self, context):
        try:
            load_report(context.scene)
        except (OSError, ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_quality_jump(bpy.types.Operator):
    bl_idname = "uipc.quality_jump"
    bl_label = "Inspect Peak Frame"
    index: IntProperty()
    metric: EnumProperty(items=[("speed", "Speed", "Maximum sampled speed"),
                               ("acceleration", "Acceleration", "Maximum sampled acceleration")])

    def execute(self, context):
        try:
            report = load_report(context.scene)
            entry = next(o for o in report["objects"] if o["index"] == self.index)
            obj = context.scene.objects.get(entry["name"])
            if obj is None or obj.name not in context.view_layer.objects:
                raise ValueError("The reported object is not in the active view layer")
            context.scene.frame_set(entry[self.metric + "_frame"])
            for selected in context.selected_objects:
                selected.select_set(False)
            obj.select_set(True)
            context.view_layer.objects.active = obj
            context.scene.uipc_settings.quality_vertex = entry[self.metric + "_vertex"]
            context.scene.uipc_settings.quality_object = obj
            self.report({"INFO"}, f"{obj.name}: {self.metric} peak at vertex {entry[self.metric + '_vertex']}")
        except (OSError, ValueError, KeyError, StopIteration) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_quality_mark(bpy.types.Operator):
    bl_idname = "uipc.quality_mark"
    bl_label = "Mark Review Frames"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        try:
            report = load_report(context.scene)
        except (OSError, ValueError, KeyError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene, settings = context.scene, context.scene.uipc_settings
        prefix = "libuipc quality: "
        for marker in list(scene.timeline_markers):
            if marker.name.startswith(prefix):
                scene.timeline_markers.remove(marker)
        for entry in report["objects"]:
            for metric, threshold in (("speed", settings.quality_speed_limit),
                                      ("acceleration", settings.quality_acceleration_limit)):
                if threshold > 0 and entry["max_" + metric] > threshold:
                    scene.timeline_markers.new(prefix + entry["name"] + " " + metric,
                                               frame=entry[metric + "_frame"])
        for frame in report["solver"]["limit_frames"]:
            scene.timeline_markers.new(prefix + "solver limit", frame=frame)
        return {"FINISHED"}


class UIPC_PT_quality(bpy.types.Panel):
    bl_label = "Bake Quality and Physics Preview"
    bl_idname = "UIPC_PT_quality"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "libuipc"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout, settings = self.layout, context.scene.uipc_settings
        layout.ui_units_x = 26
        row = layout.row(align=True)
        row.prop(settings, "show_physics_overlay")
        row.prop(settings, "show_pin_overlay")
        layout.prop(settings, "show_thickness_overlay")
        layout.label(text="Selected object; thickness lines are normal guides")
        layout.operator("uipc.quality_load")
        if settings.quality_bake != settings.last_bake or not settings.quality_summary:
            layout.label(text="Load a completed bake's report")
            return
        try:
            report = json.loads(settings.quality_summary)
        except ValueError:
            return
        solver = report["solver"]
        performance = report.get("performance", {})
        if performance:
            phases = performance["phases"]
            seconds = lambda names: sum(phases.get(n, {}).get("seconds", 0) for n in names)
            box = layout.box()
            box.label(text="Host wall time (no extra GPU synchronization)")
            box.label(text=f"Solve + retrieve: {seconds(('advance', 'retrieve')):.3f} s")
            box.label(text=f"Diagnostics: {seconds(('motion_diagnostics', 'solver_diagnostics', 'diagnostics_finalize')):.3f} s")
            box.label(text=f"Cache I/O: {seconds(('cache_write', 'cache_finalize')):.3f} s")
            box.label(text=f"Cache: {performance['cache_bytes'] / 1048576:.2f} MiB")
            dense_bytes = performance.get("dense_cache_bytes", performance["cache_bytes"])
            if dense_bytes > performance["cache_bytes"]:
                saved = 1 - performance["cache_bytes"] / dense_bytes
                box.label(text=f"Cache storage saved: {saved:.1%}")
            from .performance import frontend_report
            for name, entry in frontend_report(context.scene).items():
                box.label(text=f"Last {name}: {entry['last_seconds']:.3f} s")
        if solver["steps"]:
            layout.label(text=f"Peak iterations: Newton {solver['max_newton_iterations']}, PCG {solver['max_linear_solver_iterations']}")
            layout.label(text=f"Peak line-search trials: {solver['max_line_search_trials']}")
            layout.label(text=f"Native nonconverged steps: {solver['nonconverged_steps']}")
            if solver["limit_frames"]:
                layout.label(text=f"Iteration-limit frames: {len(solver['limit_frames'])}", icon="ERROR")
        else:
            layout.label(text="No native substep statistics (runtime unavailable or all fixed)")
        layout.label(text="Review thresholds, not physical validity limits")
        layout.prop(settings, "quality_speed_limit")
        layout.prop(settings, "quality_acceleration_limit")
        layout.operator("uipc.quality_mark")
        layout.label(text="Output-sampled motion (not instantaneous native velocity)")
        for entry in sorted(report["objects"], key=lambda o: o["max_speed"], reverse=True):
            box = layout.box()
            box.label(text=entry["name"])
            for metric, unit, threshold in (("speed", "m/s", settings.quality_speed_limit),
                                            ("acceleration", "m/s^2", settings.quality_acceleration_limit)):
                row = box.row(align=True)
                peak = entry["max_" + metric]
                row.label(text=f"{metric.title()}: {peak:.4g} {unit}",
                          icon="ERROR" if threshold > 0 and peak > threshold else "INFO")
                operator = row.operator("uipc.quality_jump", text=f"Frame {entry[metric + '_frame']}")
                operator.index, operator.metric = entry["index"], metric
        layout.label(text="Finite differences at output FPS; subframe spikes may be missed")


QUALITY_CLASSES = (UIPC_OT_quality_load, UIPC_OT_quality_jump, UIPC_OT_quality_mark, UIPC_PT_quality)
