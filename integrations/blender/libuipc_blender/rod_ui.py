# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Explicit curve-to-centerline conversion and addon-free rod surface display."""
import bpy
import numpy as np

from .node_math import geometry_group, transform_positions
from .rod import validate_linemesh

DISPLAY_NAME = "libuipc Rod Surface"


def is_display(obj, modifier):
    return (obj.uipc_body.role == "ROD" and modifier.name == DISPLAY_NAME
            and modifier.type == "NODES" and modifier.node_group
            and modifier.node_group.get("uipc_rod_display") == 1)


def build_display(obj, scene):
    modifier = obj.modifiers.get(DISPLAY_NAME)
    if modifier is not None and not is_display(obj, modifier):
        raise ValueError(f"Rename the existing {DISPLAY_NAME} modifier first")
    matrix = np.eye(4)
    matrix[:3,:3] = np.asarray(obj.matrix_world)[:3,:3]
    if not np.isfinite(matrix).all() or np.linalg.cond(matrix[:3,:3]) > 1e12:
        raise ValueError("Rod display requires an invertible object transform")
    group, source, sink = geometry_group(DISPLAY_NAME)
    try:
        group["uipc_rod_display"] = 1
        # Construct the circular section in world-length coordinates, then map
        # back to local coordinates. Nonuniform/negative object scale is preserved.
        world, _ = transform_positions(group, source, matrix, "World")
        curve = group.nodes.new("GeometryNodeMeshToCurve")
        profile = group.nodes.new("GeometryNodeCurvePrimitiveCircle")
        profile.inputs["Resolution"].default_value = 12
        profile.inputs["Radius"].default_value = obj.uipc_body.thickness / scene.unit_settings.scale_length
        tube = group.nodes.new("GeometryNodeCurveToMesh")
        tube.inputs["Fill Caps"].default_value = True
        group.links.new(world, curve.inputs["Mesh"])
        group.links.new(curve.outputs["Curve"], tube.inputs["Curve"])
        group.links.new(profile.outputs["Curve"], tube.inputs["Profile Curve"])
        sphere = group.nodes.new("GeometryNodeMeshUVSphere")
        sphere.inputs["Segments"].default_value = 12
        sphere.inputs["Rings"].default_value = 8
        sphere.inputs["Radius"].default_value = obj.uipc_body.thickness / scene.unit_settings.scale_length
        endpoints = group.nodes.new("GeometryNodeCurveEndpointSelection")
        instances = group.nodes.new("GeometryNodeInstanceOnPoints")
        group.links.new(curve.outputs["Curve"], instances.inputs["Points"])
        group.links.new(endpoints.outputs["Selection"], instances.inputs["Selection"])
        group.links.new(sphere.outputs["Mesh"], instances.inputs["Instance"])
        realized = group.nodes.new("GeometryNodeRealizeInstances")
        group.links.new(instances.outputs["Instances"], realized.inputs["Geometry"])
        joined = group.nodes.new("GeometryNodeJoinGeometry")
        group.links.new(tube.outputs["Mesh"], joined.inputs["Geometry"])
        group.links.new(realized.outputs["Geometry"], joined.inputs["Geometry"])
        local, _ = transform_positions(group, joined.outputs["Geometry"], np.linalg.inv(matrix), "Local")
        material = group.interface.new_socket(name="Material", in_out="INPUT", socket_type="NodeSocketMaterial")
        material.default_value = obj.active_material
        surface = group.nodes.new("GeometryNodeSetMaterial")
        group.links.new(local, surface.inputs["Geometry"])
        group.links.new(group.nodes["Input"].outputs["Material"], surface.inputs["Material"])
        group.links.new(surface.outputs["Geometry"], sink)
        if modifier is None:
            modifier = obj.modifiers.new(DISPLAY_NAME, "NODES")
        old = modifier.node_group
        modifier.node_group = group
        obj.modifiers.move(list(obj.modifiers).index(modifier), len(obj.modifiers)-1)
        if old and old.users == 0:
            bpy.data.node_groups.remove(old)
    except Exception:
        if group.users == 0:
            bpy.data.node_groups.remove(group)
        raise


def refresh_displays(scene):
    for obj in scene.objects:
        modifier = obj.modifiers.get(DISPLAY_NAME)
        if modifier and is_display(obj, modifier):
            build_display(obj, scene)


class UIPC_OT_rod_surface(bpy.types.Operator):
    bl_idname = "uipc.rod_surface"
    bl_label = "Create / Refresh Rod Surface"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        from . import runtime
        return (not runtime.is_running() and context.object is not None
                and context.object.type == "MESH" and context.object.uipc_body.role == "ROD")

    def execute(self, context):
        try:
            context.view_layer.update()
            build_display(context.object, context.scene)
        except (ValueError, RuntimeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class UIPC_OT_curve_to_rod(bpy.types.Operator):
    bl_idname = "uipc.curve_to_rod"
    bl_label = "Create Rod from Curve"
    bl_description = "Sample a curve's evaluated centerline into a new rod mesh; keep the source curve unchanged"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        from . import runtime
        return not runtime.is_running() and context.object is not None and context.object.type == "CURVE"

    def execute(self, context):
        original, temporary, curve, mesh, obj = context.object, None, None, None, None
        try:
            context.view_layer.update()
            graph = context.evaluated_depsgraph_get()
            matrix = original.evaluated_get(graph).matrix_world.copy()
            # Clear bevel only on a private evaluated copy, never on the source.
            temporary = original.copy()
            curve = original.data.copy()
            temporary.data = curve
            context.scene.collection.objects.link(temporary)
            curve.bevel_depth, curve.extrude = 0, 0
            curve.bevel_object, curve.taper_object = None, None
            temporary.modifiers.clear()
            context.view_layer.update()
            mesh = bpy.data.meshes.new_from_object(temporary.evaluated_get(context.evaluated_depsgraph_get()))
            if len(mesh.polygons):
                raise ValueError("Curve conversion produced faces; use a 3D centerline curve")
            points = np.asarray([v.co[:] for v in mesh.vertices])
            edges = np.asarray([e.vertices[:] for e in mesh.edges], dtype=np.int32)
            validate_linemesh(points, edges, original.name)
            obj = bpy.data.objects.new(original.name + " Rod", mesh)
            context.scene.collection.objects.link(obj)
            obj.matrix_world = matrix
            obj.uipc_body.role = "ROD"
            for selected in context.selected_objects:
                selected.select_set(False)
            obj.select_set(True)
            context.view_layer.objects.active = obj
            context.view_layer.update()
            build_display(obj, context.scene)
            self.report({"INFO"}, "Created rod centerline; original curve retained (hide its display if needed)")
        except (ValueError, RuntimeError) as error:
            if obj:
                bpy.data.objects.remove(obj, do_unlink=True)
            if mesh and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
            context.view_layer.objects.active = original
            original.select_set(True)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        finally:
            if temporary:
                bpy.data.objects.remove(temporary, do_unlink=True)
            if curve and curve.users == 0:
                bpy.data.curves.remove(curve)
        return {"FINISHED"}


ROD_CLASSES = (UIPC_OT_curve_to_rod, UIPC_OT_rod_surface)
