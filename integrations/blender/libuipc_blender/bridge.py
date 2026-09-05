# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Blender scene export and native, fixed-topology MDD playback."""

from pathlib import Path
import uuid

import bpy
import numpy as np

from .protocol import (SCHEMA_VERSION, MODIFIER_NAME, OBJECT_FIELDS, atomic_json,
                       fingerprint, inspect_mdd, read_json, validate_mesh)


def cache_root(scene):
    configured = scene.uipc_settings.cache_directory
    if not configured:
        if not bpy.data.filepath:
            raise ValueError("Save the .blend first or choose an absolute Cache Directory")
        return Path(bpy.data.filepath).parent / (Path(bpy.data.filepath).stem + "_uipc_cache")
    if configured.startswith("//") and not bpy.data.filepath:
        raise ValueError("Save the .blend before using a relative cache directory")
    return Path(bpy.path.abspath(configured)).resolve()


def object_material(obj):
    return {name: getattr(obj.uipc_body, name) for name in OBJECT_FIELDS}


def collect_scene(scene):
    # New objects and script-driven transforms may not have reached matrix_world.
    # Read the final world matrices only after Blender updates its dependency graph.
    layer = scene.view_layers[0]
    layer.update()
    depsgraph = layer.depsgraph
    depsgraph.update()
    settings = scene.uipc_settings
    simulation = {
        "frame_start": scene.frame_start, "frame_end": scene.frame_end,
        "fps": scene.render.fps / scene.render.fps_base,
        "unit_scale": scene.unit_settings.scale_length,
        "substeps": settings.substeps, "gravity": list(settings.gravity),
        "d_hat": settings.d_hat, "friction": settings.friction,
        "resistance": settings.resistance,
    }
    if scene.frame_end < scene.frame_start:
        raise ValueError("End frame precedes start frame")
    bodies = []
    for obj in sorted(scene.objects, key=lambda o: o.name):
        if obj.uipc_body.role == "NONE":
            continue
        if obj.type != "MESH" or obj.mode != "OBJECT":
            raise ValueError(f"{obj.name}: use a mesh in Object Mode")
        if obj.library or obj.data.library:
            raise ValueError(f"{obj.name}: make linked objects and meshes local first")
        if obj.data.shape_keys:
            raise ValueError(f"{obj.name}: apply/remove shape keys before baking")
        current = obj
        while current:
            if current.animation_data or current.constraints or current.rigid_body:
                raise ValueError(f"{obj.name}: animated/constrained/Bullet objects or parents are unsupported in v0.1")
            current = current.parent
        for modifier in obj.modifiers:
            if modifier.name == MODIFIER_NAME and modifier.type != "MESH_CACHE":
                raise ValueError(f"{obj.name}: rename the existing '{MODIFIER_NAME}' modifier")
            if modifier.name == MODIFIER_NAME and modifier.type == "MESH_CACHE":
                continue
            if not (modifier.show_viewport or modifier.show_render):
                continue
            if obj.uipc_body.role != "STATIC" and modifier.type in ("SUBSURF", "SOLIDIFY", "BEVEL", "WEIGHTED_NORMAL"):
                continue
            raise ValueError(f"{obj.name}: apply/disable {modifier.name}; simulation uses the base mesh")
        mesh = obj.data
        vertices = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", vertices)
        vertices = vertices.reshape(-1, 3)
        mesh.calc_loop_triangles()
        triangles = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
        mesh.loop_triangles.foreach_get("vertices", triangles)
        triangles = triangles.reshape(-1, 3)
        matrix = np.array(obj.evaluated_get(depsgraph).matrix_world, dtype=np.float64)
        if not np.isfinite(matrix).all() or np.linalg.cond(matrix[:3, :3]) > 1e12:
            raise ValueError(f"{obj.name}: singular/non-finite transform; check object scale")
        world_vertices = (vertices @ matrix[:3, :3].T + matrix[:3, 3]) * simulation["unit_scale"]
        # Normalize only winding; keep vertex IDs and base coordinates untouched.
        _, triangles = validate_mesh(world_vertices, triangles, obj.uipc_body.role, obj.name)
        pins = []
        if obj.uipc_body.role == "CLOTH" and obj.uipc_body.pin_group:
            group = obj.vertex_groups.get(obj.uipc_body.pin_group)
            if group is None:
                raise ValueError(f"{obj.name}: pin group '{obj.uipc_body.pin_group}' does not exist")
            for vertex in mesh.vertices:
                if any(g.group == group.index and g.weight >= obj.uipc_body.pin_threshold for g in vertex.groups):
                    pins.append(vertex.index)
            if not pins:
                raise ValueError(f"{obj.name}: no vertices meet the pin weight threshold")
        bodies.append({"name": obj.name, "vertices": vertices, "triangles": triangles,
                       "matrix": matrix, "pins": np.array(pins, dtype=np.int32),
                       "material": object_material(obj)})
    if not bodies or not any(b["material"]["role"] != "STATIC" for b in bodies):
        raise ValueError("Assign at least one object as Cloth or Rigid Body")
    return simulation, bodies


def export_job(scene):
    settings, bodies = collect_scene(scene)
    root = cache_root(scene)
    root.mkdir(parents=True, exist_ok=True)
    directory = root / ("bake_" + uuid.uuid4().hex)
    directory.mkdir()
    request = {"schema_version": SCHEMA_VERSION, "settings": settings,
               "fingerprint": fingerprint(settings, bodies),
               "objects": [{"name": b["name"], "material": b["material"]} for b in bodies]}
    for index, body in enumerate(bodies):
        np.savez(directory / f"input_{index:04d}.npz",
                 **{key: body[key] for key in ("vertices", "triangles", "matrix", "pins")})
    atomic_json(directory / "request.json", request)
    return directory, request


def attach_cache(scene, directory, request):
    result = read_json(directory / "result.json")
    current_settings, bodies = collect_scene(scene)
    expected = request["fingerprint"]
    if result["schema_version"] != SCHEMA_VERSION or result["fingerprint"] != expected:
        raise ValueError("Bake result does not match the exported scene")
    if fingerprint(current_settings, bodies) != expected:
        raise ValueError("Scene changed during baking; result was preserved on disk. Bake again")
    expected_outputs = [i for i, b in enumerate(bodies) if b["material"]["role"] != "STATIC"]
    if [o["index"] for o in result["objects"]] != expected_outputs:
        raise ValueError("Bake result has missing/duplicate objects")
    frames = request["settings"]["frame_end"] - request["settings"]["frame_start"] + 1
    if result["frames"] != frames:
        raise ValueError("Bake result has the wrong frame count")
    for output in result["objects"]:
        index = output["index"]
        vertices = len(bodies[index]["vertices"])
        if output["vertices"] != vertices:
            raise ValueError("Bake result has the wrong vertex count")
        inspect_mdd(directory / f"object_{index:04d}.mdd", frames, vertices)
    # Validate the entire result before replacing any previous cache attachment.
    for output in result["objects"]:
        index = output["index"]
        obj = scene.objects[request["objects"][index]["name"]]
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier is None:
            modifier = obj.modifiers.new(MODIFIER_NAME, "MESH_CACHE")
        modifier.cache_format = "MDD"
        modifier.filepath = bpy.path.relpath(str(directory / f"object_{index:04d}.mdd")) if bpy.data.filepath else str(directory / f"object_{index:04d}.mdd")
        modifier.time_mode = "FRAME"
        modifier.play_mode = "SCENE"
        modifier.frame_start = request["settings"]["frame_start"]
        modifier.frame_scale = 1.0
        modifier.deform_mode = "OVERWRITE"
        modifier.interpolation = "LINEAR"
        modifier.forward_axis = "POS_Y"
        modifier.up_axis = "POS_Z"
        modifier.flip_axis = (False, False, False)
        modifier.factor = 1.0
        modifier.show_viewport = True
        modifier.show_render = True
        obj.modifiers.move(list(obj.modifiers).index(modifier), 0)
    scene.uipc_settings.last_bake = bpy.path.relpath(str(directory)) if bpy.data.filepath else str(directory)
    scene.uipc_settings.baked_fingerprint = expected
    scene.frame_set(scene.frame_current)
    return result


def check_cache(scene):
    if not scene.uipc_settings.last_bake:
        raise ValueError("No completed bake")
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = read_json(directory / "request.json")
    settings, bodies = collect_scene(scene)
    if fingerprint(settings, bodies) != request["fingerprint"]:
        raise ValueError("Cache is stale: geometry, transforms, pins, materials, or scene settings changed")
    result = read_json(directory / "result.json")
    for output in result["objects"]:
        index = output["index"]
        obj = scene.objects.get(request["objects"][index]["name"])
        modifier = obj.modifiers.get(MODIFIER_NAME) if obj else None
        if not modifier or modifier.type != "MESH_CACHE":
            raise ValueError("A baked object's cache modifier is missing")
        expected_path = directory / f"object_{index:04d}.mdd"
        if Path(bpy.path.abspath(modifier.filepath)).resolve() != expected_path.resolve():
            raise ValueError("Cache modifier path has changed")
        inspect_mdd(expected_path, result["frames"], output["vertices"])
    return result


def detach_cache(scene):
    for obj in scene.objects:
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier and modifier.type == "MESH_CACHE":
            obj.modifiers.remove(modifier)
    scene.uipc_settings.last_bake = ""
    scene.uipc_settings.baked_fingerprint = ""
    scene.frame_set(scene.frame_current)
