# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Blender scene export and native, fixed-topology MDD playback."""

from pathlib import Path
import json
import shutil
import uuid

import bpy
import numpy as np

from .protocol import (SCHEMA_VERSION, MODIFIER_NAME, OBJECT_FIELDS, atomic_json,
                       fingerprint, cache_fingerprint, inspect_mdd, read_json, validate_mesh, validate_tetmesh)


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
    from .motion import drive_material
    result = {name: getattr(obj.uipc_body, name) for name in OBJECT_FIELDS}
    drive = drive_material(obj)
    if drive is not None:
        result["drive"] = drive
    return result


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
        tetrahedra = np.empty((0, 4), dtype=np.int32)
        if obj.uipc_body.role == "FEM":
            stored = mesh.get("uipc_tetrahedra")
            if stored is None or len(stored) % 4:
                raise ValueError(f"{obj.name}: generate or import a tetrahedral mesh before baking FEM")
            tetrahedra = np.asarray(stored, dtype=np.int32).reshape(-1, 4)
            _, tetrahedra, boundary, _ = validate_tetmesh(world_vertices, tetrahedra, obj.name)
            if (len(triangles) != len(boundary)
                    or {tuple(sorted(t)) for t in triangles} != {tuple(sorted(t)) for t in boundary}):
                raise ValueError(f"{obj.name}: visible faces no longer match the tetrahedral boundary; regenerate the volume")
            triangles = boundary
        else:
            _, triangles = validate_mesh(world_vertices, triangles, obj.uipc_body.role, obj.name,
                                         allow_components=obj.uipc_body.driven)
        pins = []
        if obj.uipc_body.role in ("CLOTH", "FEM") and obj.uipc_body.pin_group and not obj.uipc_body.fixed:
            group = obj.vertex_groups.get(obj.uipc_body.pin_group)
            if group is None:
                raise ValueError(f"{obj.name}: pin group '{obj.uipc_body.pin_group}' does not exist")
            for vertex in mesh.vertices:
                if any(g.group == group.index and g.weight >= obj.uipc_body.pin_threshold for g in vertex.groups):
                    pins.append(vertex.index)
            if not pins:
                raise ValueError(f"{obj.name}: no vertices meet the pin weight threshold")
        bodies.append({"name": obj.name, "vertices": vertices, "triangles": triangles,
                       "tetrahedra": tetrahedra,
                       "matrix": matrix, "pins": np.array(pins, dtype=np.int32),
                       "material": object_material(obj)})
    if not bodies or not any(b["material"]["role"] != "STATIC" for b in bodies):
        raise ValueError("Assign at least one object as Cloth, Rigid Body, or Volumetric FEM")
    return simulation, bodies


def export_job(scene):
    from .motion import sample_targets, sample_hash, align_robot_initial
    align_robot_initial(scene)
    settings, bodies = collect_scene(scene)
    targets = sample_targets(scene, bodies)
    root = cache_root(scene)
    root.mkdir(parents=True, exist_ok=True)
    directory = root / ("bake_" + uuid.uuid4().hex)
    directory.mkdir()
    request = {"schema_version": SCHEMA_VERSION, "settings": settings,
               "fingerprint": fingerprint(settings, bodies),
               "objects": [{"name": b["name"], "material": b["material"]} for b in bodies]}
    for index, body in enumerate(bodies):
        arrays = {key: body[key] for key in ("vertices", "triangles", "tetrahedra", "matrix", "pins")}
        if "drive" in body["material"]:
            arrays["drive_targets"] = targets[body["material"]["drive"]["target"]]
            request["objects"][index]["drive_targets_sha256"] = sample_hash(arrays["drive_targets"])
        np.savez(directory / f"input_{index:04d}.npz", **arrays)
    atomic_json(directory / "request.json", request)
    return directory, request


def export_robot_job(scene, filename):
    source = Path(bpy.path.abspath(filename)).resolve()
    if not source.is_file() or source.suffix.lower() != ".urdf":
        raise ValueError("Choose a URDF file")
    directory = cache_root(scene) / ("robot_" + uuid.uuid4().hex)
    directory.mkdir(parents=True)
    request = {"schema_version": SCHEMA_VERSION, "operation": "import_robot",
               "source": str(source), "name": source.stem, "fingerprint": uuid.uuid4().hex}
    atomic_json(directory / "request.json", request)
    return directory, request


def attach_cache(scene, directory, request):
    result = read_json(directory / "result.json")
    current_settings, bodies = collect_scene(scene)
    expected = request["fingerprint"]
    if result["schema_version"] != request["schema_version"] or result["fingerprint"] != expected:
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
    if cache_fingerprint(request, settings, bodies) != request["fingerprint"]:
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


def volume_input(scene, obj):
    if obj is None or obj.type != "MESH" or obj.mode != "OBJECT":
        raise ValueError("Select a surface mesh in Object Mode")
    if obj.library or obj.data.library or obj.data.shape_keys:
        raise ValueError("Make the mesh local and apply shape keys before generating a volume")
    if obj.uipc_body.source_mesh is not None or obj.data.get("uipc_tetrahedra") is not None:
        raise ValueError("Restore the source surface before generating another volume")
    if any(m.show_viewport or m.show_render for m in obj.modifiers):
        raise ValueError("Apply or disable modifiers before generating tetrahedra")
    layer = scene.view_layers[0]
    layer.update()
    depsgraph = layer.depsgraph
    depsgraph.update()
    mesh = obj.data
    vertices = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
    mesh.vertices.foreach_get("co", vertices)
    mesh.calc_loop_triangles()
    faces = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
    mesh.loop_triangles.foreach_get("vertices", faces)
    matrix = np.array(obj.evaluated_get(depsgraph).matrix_world, dtype=np.float64)
    if not np.isfinite(matrix).all() or np.linalg.cond(matrix[:3, :3]) > 1e12:
        raise ValueError("Object has a singular or non-finite transform")
    groups = [{"name": group.name, "weights": []} for group in obj.vertex_groups]
    for vertex in mesh.vertices:
        for group in vertex.groups:
            groups[group.group]["weights"].append([vertex.index, group.weight])
    body = {"name": obj.name, "vertices": vertices.reshape(-1, 3),
            "triangles": faces.reshape(-1, 3), "matrix": matrix,
            "pins": np.empty(0, dtype=np.int32), "material": {"groups": groups}}
    options = {"preserve_surface": obj.uipc_body.preserve_surface,
               "target_edge_length": obj.uipc_body.tet_edge_length,
               "quality_passes": obj.uipc_body.tet_quality_passes,
               "refinement_budget": 256}
    settings = {"unit_scale": scene.unit_settings.scale_length, "options": options}
    return settings, body, fingerprint(settings, [body])


def export_volume_job(scene, obj=None, filename=None):
    root = cache_root(scene)
    root.mkdir(parents=True, exist_ok=True)
    directory = root / ("volume_" + uuid.uuid4().hex)
    if filename:
        source = Path(bpy.path.abspath(filename)).resolve()
        if not source.is_file() or source.suffix.lower() != ".msh":
            raise ValueError("Choose an existing tetrahedral .msh file")
        directory.mkdir()
        shutil.copyfile(source, directory / "input.msh")
        request = {"schema_version": SCHEMA_VERSION, "operation": "import_volume",
                   "name": source.stem, "fingerprint": uuid.uuid4().hex}
    else:
        settings, body, signature = volume_input(scene, obj)
        directory.mkdir()
        np.savez(directory / "surface.npz", **{k: body[k] for k in ("vertices", "triangles", "matrix")})
        request = {"schema_version": SCHEMA_VERSION, "operation": "generate_volume",
                   "name": obj.name, "fingerprint": signature, **settings,
                   "groups": body["material"]["groups"]}
    atomic_json(directory / "request.json", request)
    return directory, request


def attach_volume(scene, obj, directory, request):
    result = read_json(directory / "result.json")
    if result["schema_version"] != SCHEMA_VERSION or result["fingerprint"] != request["fingerprint"]:
        raise ValueError("Prepared volume does not match the request")
    source = None
    if request["operation"] == "generate_volume":
        if obj is None or obj.name not in scene.objects:
            raise ValueError("Source object was removed while generating tetrahedra")
        _, body, current = volume_input(scene, obj)
        if current != request["fingerprint"]:
            raise ValueError("Source geometry, groups, transforms, or meshing settings changed during preparation")
        source = obj.data
    with np.load(directory / "volume.npz", allow_pickle=False) as data:
        points, cells, boundary, _ = validate_tetmesh(data["vertices"], data["tetrahedra"], request["name"])
    strict = bool(source is not None and request["options"]["preserve_surface"])
    if strict:
        source_faces = {tuple(sorted(f)) for f in body["triangles"]}
        actual_faces = {tuple(sorted(f)) for f in boundary}
        if source_faces != actual_faces or not np.array_equal(points[:len(source.vertices)], body["vertices"]):
            raise ValueError("Internal error: protected surface changed during volume generation")
        mesh = source.copy()
        mesh.name = source.name + " FEM"
        mesh.vertices.add(len(points) - len(source.vertices))
        mesh.vertices.foreach_set("co", points.ravel())
        mesh.update()
    else:
        mesh = bpy.data.meshes.new(request["name"] + " FEM Mesh")
        mesh.from_pydata(points.tolist(), [], boundary.tolist())
        mesh.update()
        if source is not None:
            for material in source.materials:
                mesh.materials.append(material)
    mesh["uipc_tetrahedra"] = cells.ravel().tolist()
    mesh["uipc_volume_schema"] = 1
    if obj is None:
        obj = bpy.data.objects.new(request["name"] + " FEM", mesh)
        scene.collection.objects.link(obj)
        obj.uipc_body.density = 1000
    else:
        # Keep a private snapshot even when the original mesh was linked to
        # another object, so later edits cannot change the restore baseline.
        obj.uipc_body.source_mesh = source.copy()
        obj.uipc_body.source_groups = json.dumps(request.get("groups", []))
        obj.uipc_body.source_role = obj.uipc_body.role
        obj.data = mesh
        if not strict:
            for group in request.get("groups", []):
                target = obj.vertex_groups.get(group["name"])
                if target is None:
                    target = obj.vertex_groups.new(name=group["name"])
                for vertex, weight in group["weights"]:
                    target.add([vertex], weight, "REPLACE")
    obj.uipc_body.role = "FEM"
    obj.uipc_body.tet_report = json.dumps(result["report"])
    scene.uipc_settings.status = f"FEM volume ready: {len(points)} nodes, {len(cells)} tetrahedra"
    for layer in scene.view_layers:
        layer.objects.active = obj
    obj.select_set(True)
    return result


def restore_surface(obj):
    source = obj.uipc_body.source_mesh
    if source is None:
        raise ValueError("This object has no saved source surface")
    modifier = obj.modifiers.get(MODIFIER_NAME)
    if modifier and modifier.type == "MESH_CACHE":
        obj.modifiers.remove(modifier)
    groups = json.loads(obj.uipc_body.source_groups or "[]")
    obj.data = source
    obj.vertex_groups.clear()
    for group in groups:
        target = obj.vertex_groups.new(name=group["name"])
        for vertex, weight in group["weights"]:
            target.add([vertex], weight, "REPLACE")
    obj.uipc_body.role = obj.uipc_body.source_role
    obj.uipc_body.source_mesh = None
    obj.uipc_body.source_groups = ""
    obj.uipc_body.tet_report = ""
