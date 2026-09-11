# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Non-destructive preview with bounded, dependency-invalidated CPU/GPU reuse."""

from collections import Counter, OrderedDict
from pathlib import Path
import struct

import bpy
import numpy as np

from .protocol import MODIFIER_NAME, read_json

_handle = None
_entries = OrderedDict()
_shaders = OrderedDict()
_counts = Counter()
MAX_OBJECTS = 4
MAX_CPU_BYTES = 128 * 1024 * 1024


def clear():
    _entries.clear()
    _shaders.clear()


def statistics(reset=False):
    result = dict(_counts)
    if reset:
        _counts.clear()
    return result


def dependency_update(scene, updates):
    # Mesh datablock edits change rest positions/topology; evaluated Object updates
    # (MDD frames, transforms, vertex-group edits) do not change base connectivity.
    for update in updates:
        item = getattr(update.id, "original", update.id)
        for key, entry in list(_entries.items()):
            if key[0] != scene.as_pointer():
                continue
            if isinstance(item, bpy.types.Mesh) and item.as_pointer() == key[2]:
                _entries.pop(key, None)
            elif (isinstance(item, bpy.types.Object) and item.as_pointer() == key[1]
                  and update.is_updated_geometry):
                entry.pop("pins_key", None)


def _file_token(path):
    stat = path.stat()
    return str(path), stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def _entry(obj, scene):
    mesh = obj.data
    key = (scene.as_pointer(), obj.as_pointer(), mesh.as_pointer())
    entry = _entries.get(key)
    counts = (len(mesh.vertices), len(mesh.edges), len(mesh.polygons), len(mesh.loops))
    if entry is None or entry["counts"] != counts or entry["role"] != obj.uipc_body.role:
        rest = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", rest)
        mesh.calc_loop_triangles()
        triangles = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
        mesh.loop_triangles.foreach_get("vertices", triangles)
        triangles = triangles.reshape(-1, 3)
        edges = np.sort(np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]])), axis=1)
        if obj.uipc_body.role == "ROD":
            edges = np.empty(len(mesh.edges) * 2, dtype=np.int32)
            mesh.edges.foreach_get("vertices", edges)
            edges = edges.reshape(-1, 2)
        entry = {"counts": counts, "role": obj.uipc_body.role, "rest": rest.reshape(-1, 3), "triangles": triangles,
                 "edges": np.unique(edges, axis=0), "samples": OrderedDict()}
        _entries[key] = entry
        _counts["topology_builds"] += 1
    _entries.move_to_end(key)
    while len(_entries) > MAX_OBJECTS:
        _entries.popitem(last=False)
    return entry


def _local_points(entry, obj, scene):
    modifier = obj.modifiers.get(MODIFIER_NAME)
    if not (modifier and modifier.show_viewport):
        return ("rest",), entry["rest"]
    from .bridge import cache_settings
    from .affine_playback import is_affine, playback_modifier
    from .affine import affine_positions
    compact = is_affine(modifier)
    if modifier.type != "MESH_CACHE" and not compact:
        raise ValueError("Preview cache modifier is invalid")
    modifier = playback_modifier(modifier)
    vertex_count = 4 if compact else len(entry["rest"])
    request_path = Path(bpy.path.abspath(scene.uipc_settings.last_bake)) / "request.json"
    request_token = _file_token(request_path)
    if entry.get("request_token") != request_token:
        entry["settings"] = cache_settings(read_json(request_path))
        entry["request_token"] = request_token
        _counts["request_reads"] += 1
    for key, expected in entry["settings"].items():
        actual = tuple(getattr(modifier, key)) if key == "flip_axis" else getattr(modifier, key)
        if actual != expected:
            raise ValueError("Preview requires validated cache playback settings")
    path = Path(bpy.path.abspath(modifier.filepath))
    token = (*_file_token(path), compact)
    if entry.get("file_token") != token:
        with path.open("rb") as stream:
            header = stream.read(8)
        if len(header) != 8:
            raise ValueError("Truncated preview cache")
        frames, vertices = struct.unpack(">ii", header)
        if (frames < 1 or vertices != vertex_count
                or token[1] != 8 + 4*frames + 12*frames*vertices):
            raise ValueError("Preview cache topology/size mismatch")
        entry.update(file_token=token, frames=frames)
        entry["samples"].clear()
    frames, vertices = entry["frames"], vertex_count
    position = min(max(scene.frame_current + scene.frame_subframe - modifier.frame_start, 0), frames - 1)
    source_key = (token, position)
    if entry.get("source_key") != source_key:
        first, second = int(position), min(int(position) + 1, frames - 1)
        alpha = position - first
        needed = [first] if not alpha else [first, second]
        missing = [f for f in needed if f not in entry["samples"]]
        if missing:
            with path.open("rb") as stream:
                for frame in missing:
                    stream.seek(8 + 4*frames + frame*vertices*12)
                    raw = stream.read(vertices*12)
                    if len(raw) != vertices*12:
                        raise ValueError("Truncated preview frame")
                    points = np.frombuffer(raw, dtype=">f4").reshape(vertices, 3).astype(np.float64)
                    if not np.isfinite(points).all():
                        raise ValueError("Non-finite preview frame")
                    entry["samples"][frame] = points
                    _counts["frame_reads"] += 1
        for frame in needed:
            entry["samples"].move_to_end(frame)
        points = entry["samples"][first]
        entry["local"] = points if not alpha else points*(1-alpha) + entry["samples"][second]*alpha
        if compact:
            entry["local"] = affine_positions(entry["local"], entry["rest"])
        entry["source_key"] = source_key
        while len(entry["samples"]) > 2:
            entry["samples"].popitem(last=False)
    return source_key, entry["local"]


def _limit_memory():
    # Conservative accounting (shared ndarray views may be counted twice).
    # At most two samples/object, never a whole animation or an open file.
    size = 0
    for entry in _entries.values():
        values = list(entry.values()) + list(entry["samples"].values())
        if "data" in entry:
            values += list(entry["data"].values())
        size += sum(v.nbytes for v in values if isinstance(v, np.ndarray))
    if size > MAX_CPU_BYTES:
        _entries.clear()


def simulation_preview(obj, scene, *, include_guides=True, include_pins=True):
    if obj is None or obj.type != "MESH" or obj.mode != "OBJECT" or obj.uipc_body.role == "NONE":
        return None
    entry = _entry(obj, scene)
    source_key, local = _local_points(entry, obj, scene)
    graph = bpy.context.view_layer.depsgraph if bpy.context.scene == scene else scene.view_layers[0].depsgraph
    matrix = np.asarray(obj.evaluated_get(graph).matrix_world)
    point_key = (source_key, matrix.tobytes())
    if entry.get("point_key") != point_key:
        entry["data"] = {"points": local @ matrix[:3, :3].T + matrix[:3, 3],
                         "edges": entry["edges"], "batches": OrderedDict()}
        entry["point_key"] = point_key
        entry.pop("guides_key", None)
        _counts["point_builds"] += 1
    data, body = entry["data"], obj.uipc_body
    points = data["points"]
    if include_pins:
        group = obj.vertex_groups.get(body.pin_group)
        pins_key = (body.fixed, body.role, body.pin_group, group.index if group else -1, body.pin_threshold)
        if entry.get("pins_key") != pins_key:
            if body.fixed or body.role == "STATIC":
                pins = np.arange(len(points), dtype=np.int32)
            elif body.role in ("CLOTH", "FEM", "ROD") and group:
                pins = np.asarray([v.index for v in obj.data.vertices if any(
                    g.group == group.index and g.weight >= body.pin_threshold for g in v.groups)], dtype=np.int32)
            else:
                pins = np.empty(0, dtype=np.int32)
            entry.update(pins_key=pins_key, pins=pins)
            data["batches"].clear()
            _counts["pin_builds"] += 1
        data["pins"] = entry["pins"]
    if include_guides:
        radius = 0.0 if body.role == "RIGID" else body.thickness / scene.unit_settings.scale_length
        if entry.get("guides_key") != radius:
            normals = np.zeros_like(points)
            triangles = entry["triangles"]
            if radius and body.role == "ROD" and len(data["edges"]):
                edges = data["edges"]
                first = np.full(len(points), len(edges), dtype=int)
                np.minimum.at(first, edges.ravel(), np.repeat(np.arange(len(edges)), 2))
                valid = first < len(edges)
                directions = np.tile([1.,0.,0.], (len(points),1))
                directions[valid] = points[edges[first[valid],1]] - points[edges[first[valid],0]]
                axes = np.eye(3)[np.argmin(np.abs(directions), axis=1)]
                normals = np.cross(directions, axes)
                normals /= np.maximum(np.linalg.norm(normals, axis=1)[:,None], np.finfo(float).tiny)
                _counts["normal_builds"] += 1
            elif radius and len(triangles):
                face_normals = np.cross(points[triangles[:, 1]] - points[triangles[:, 0]],
                                        points[triangles[:, 2]] - points[triangles[:, 0]])
                for corner in range(3):
                    np.add.at(normals, triangles[:, corner], face_normals)
                lengths = np.linalg.norm(normals, axis=1)
                normals /= np.maximum(lengths[:, None], np.finfo(float).tiny)
                _counts["normal_builds"] += 1
            samples = np.linspace(0, max(0, len(points)-1), min(512, len(points)), dtype=int)
            data["guides"] = np.stack((points[samples] - normals[samples]*radius,
                                        points[samples] + normals[samples]*radius), axis=1).reshape(-1, 3)
            entry["guides_key"] = radius
            data["batches"].clear()
    _limit_memory()
    return data


def draw():
    scene = bpy.context.scene
    if not hasattr(scene, "uipc_settings"):
        return
    settings = scene.uipc_settings
    if not (settings.show_physics_overlay or settings.show_pin_overlay or settings.show_thickness_overlay):
        return
    try:
        data = simulation_preview(bpy.context.object, scene, include_guides=settings.show_thickness_overlay,
                                  include_pins=settings.show_pin_overlay)
        if data is None:
            return
        import gpu
        from gpu_extras.batch import batch_for_shader
        window = bpy.context.window.as_pointer()
        if window not in _shaders:
            _shaders[window] = gpu.shader.from_builtin("UNIFORM_COLOR")
            while len(_shaders) > 4:
                _shaders.popitem(last=False)
        shader = _shaders[window]
        batches = data["batches"].setdefault(window, {})
        while len(data["batches"]) > 4:
            data["batches"].popitem(last=False)
        def batch(key, kind, points, color, indices=None):
            if key not in batches:
                points = points() if callable(points) else points
                if not len(points) or (indices is not None and not len(indices)):
                    return
                # Shader input is F32: never feed GPUVertBuf an F64 buffer.
                batches[key] = batch_for_shader(shader, kind,
                    {"pos": np.ascontiguousarray(points, dtype=np.float32)}, indices=indices)
                _counts["gpu_batch_builds"] += 1
            shader.bind()
            shader.uniform_float("color", color)
            batches[key].draw(shader)
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("LESS_EQUAL")
        try:
            if settings.show_physics_overlay:
                batch("mesh", "LINES", data["points"], (0.1, 0.8, 0.95, 0.9), data["edges"])
            if settings.show_thickness_overlay and bpy.context.object.uipc_body.role != "RIGID":
                batch("guides", "LINES", data["guides"], (1., .7, .1, .8))
            if settings.show_pin_overlay:
                gpu.state.depth_test_set("NONE")
                gpu.state.point_size_set(5)
                batch("pins", "POINTS", lambda: data["points"][data["pins"]], (1., .2, .1, 1.))
            if settings.show_physics_overlay and settings.quality_object == bpy.context.object:
                vertex = settings.quality_vertex
                if 0 <= vertex < len(data["points"]):
                    if batches.get("peak_vertex") != vertex:
                        batches.pop("peak", None)
                        batches["peak_vertex"] = vertex
                    gpu.state.depth_test_set("NONE")
                    gpu.state.point_size_set(7)
                    batch("peak", "POINTS", data["points"][[vertex]], (1., .05, .8, 1.))
        finally:
            gpu.state.point_size_set(1)
            gpu.state.depth_test_set("NONE")
            gpu.state.blend_set("NONE")
        return True
    except (OSError, ValueError, RuntimeError, ReferenceError):
        # Validation owns stale-bake handling; never mutate scene state in draw.
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
    clear()
