# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Non-destructive selected-object simulation mesh/pin/thickness preview."""

from pathlib import Path
import struct

import bpy
import numpy as np

from .protocol import MODIFIER_NAME, read_json

_handle = None


def simulation_preview(obj, scene):
    if obj is None or obj.type != "MESH" or obj.mode != "OBJECT" or obj.uipc_body.role == "NONE":
        return None
    mesh = obj.data
    points = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
    mesh.vertices.foreach_get("co", points)
    points = points.reshape(-1, 3)
    modifier = obj.modifiers.get(MODIFIER_NAME)
    if modifier and modifier.type == "MESH_CACHE" and modifier.show_viewport:
        from .bridge import cache_settings
        request = read_json(Path(bpy.path.abspath(scene.uipc_settings.last_bake)) / "request.json")
        for key, expected in cache_settings(request).items():
            actual = tuple(getattr(modifier, key)) if key == "flip_axis" else getattr(modifier, key)
            if actual != expected:
                raise ValueError("Preview requires validated cache playback settings")
        with Path(bpy.path.abspath(modifier.filepath)).open("rb") as stream:
            frames, vertices = struct.unpack(">ii", stream.read(8))
            if frames < 1 or vertices != len(points):
                raise ValueError("Preview cache topology mismatch")
            position = np.clip(scene.frame_current + scene.frame_subframe - modifier.frame_start, 0, frames - 1)
            first, second = int(position), min(int(position) + 1, frames - 1)
            alpha = position - first
            def read_frame(frame):
                stream.seek(8 + 4 * frames + frame * vertices * 12)
                return np.frombuffer(stream.read(vertices * 12), dtype=">f4").reshape(vertices, 3).astype(np.float64)
            points = read_frame(first) * (1 - alpha) + read_frame(second) * alpha
    graph = scene.view_layers[0].depsgraph
    matrix = np.asarray(obj.evaluated_get(graph).matrix_world)
    points = points @ matrix[:3, :3].T + matrix[:3, 3]
    mesh.calc_loop_triangles()
    triangles = np.asarray([t.vertices[:] for t in mesh.loop_triangles], dtype=np.int32).reshape(-1, 3)
    edges = np.sort(np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]])), axis=1)
    edges = np.unique(edges, axis=0)
    pins = []
    if obj.uipc_body.fixed or obj.uipc_body.role == "STATIC":
        pins = list(range(len(points)))
    elif obj.uipc_body.role in ("CLOTH", "FEM") and obj.uipc_body.pin_group:
        group = obj.vertex_groups.get(obj.uipc_body.pin_group)
        if group:
            pins = [v.index for v in mesh.vertices if any(g.group == group.index and g.weight >= obj.uipc_body.pin_threshold for g in v.groups)]
    normals = np.zeros_like(points)
    if len(triangles):
        face_normals = np.cross(points[triangles[:, 1]] - points[triangles[:, 0]],
                                points[triangles[:, 2]] - points[triangles[:, 0]])
        for corner in range(3):
            np.add.at(normals, triangles[:, corner], face_normals)
        lengths = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(lengths[:, None], np.finfo(float).tiny)
    radius = 0.0 if obj.uipc_body.role == "RIGID" else obj.uipc_body.thickness / scene.unit_settings.scale_length
    samples = np.linspace(0, max(0, len(points) - 1), min(512, len(points)), dtype=int)
    guides = np.stack((points[samples] - normals[samples] * radius,
                       points[samples] + normals[samples] * radius), axis=1).reshape(-1, 3)
    return {"points": points, "edges": edges, "pins": np.asarray(pins, dtype=int), "guides": guides}


def draw():
    scene = bpy.context.scene
    if not hasattr(scene, "uipc_settings"):
        return
    settings = scene.uipc_settings
    if not (settings.show_physics_overlay or settings.show_pin_overlay or settings.show_thickness_overlay):
        return
    try:
        data = simulation_preview(bpy.context.object, scene)
        if data is None:
            return
        import gpu
        from gpu_extras.batch import batch_for_shader
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("LESS_EQUAL")
        def batch(kind, points, color):
            if len(points):
                shader.bind()
                shader.uniform_float("color", color)
                # UNIFORM_COLOR's vertex input is F32; never expose an F64
                # NumPy buffer to GPUVertBuf.attr_fill's buffer fast path.
                batch_for_shader(shader, kind, {"pos": np.ascontiguousarray(points, dtype=np.float32)}).draw(shader)
        try:
            if settings.show_physics_overlay:
                batch("LINES", data["points"][data["edges"]].reshape(-1, 3), (0.1, 0.8, 0.95, 0.9))
            if settings.show_thickness_overlay and bpy.context.object.uipc_body.role != "RIGID":
                batch("LINES", data["guides"], (1., .7, .1, .8))
            if settings.show_pin_overlay:
                gpu.state.depth_test_set("NONE")
                gpu.state.point_size_set(5)
                batch("POINTS", data["points"][data["pins"]], (1., .2, .1, 1.))
            if settings.show_physics_overlay and settings.quality_object == bpy.context.object:
                vertex = settings.quality_vertex
                if 0 <= vertex < len(data["points"]):
                    gpu.state.depth_test_set("NONE")
                    gpu.state.point_size_set(7)
                    batch("POINTS", data["points"][[vertex]], (1., .05, .8, 1.))
        finally:
            gpu.state.point_size_set(1)
            gpu.state.depth_test_set("NONE")
            gpu.state.blend_set("NONE")
        return True
    except (OSError, ValueError, RuntimeError, ReferenceError):
        # Stale data is handled by explicit validation, not mutated in a draw callback.
        return


def register():
    global _handle
    if _handle is None and not bpy.app.background:
        _handle = bpy.types.SpaceView3D.draw_handler_add(draw, (), "WINDOW", "POST_VIEW")


def unregister():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, "WINDOW")
        _handle = None
