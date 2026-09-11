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
from .protocol import SOLVER_FIELDS, validate_solver_settings
from .protocol import validate_result, file_sha256
from .protocol import match_bodies, fully_fixed
from .identity import object_id, ensure_scene_ids, resolve_object
from .performance import frontend_phase
from .affine import cache_vertices
from . import affine_playback


CACHE_PROPERTIES = ("cache_format", "filepath", "time_mode", "play_mode", "frame_start",
                    "frame_scale", "deform_mode", "interpolation", "forward_axis", "up_axis",
                    "flip_axis", "factor", "vertex_group", "invert_vertex_group",
                    "show_viewport", "show_render")


def cache_settings(request):
    return {"cache_format": "MDD", "time_mode": "FRAME", "play_mode": "SCENE",
            "frame_start": request["settings"]["frame_start"], "frame_scale": 1.0,
            "deform_mode": "OVERWRITE", "interpolation": "LINEAR",
            "forward_axis": "POS_Y", "up_axis": "POS_Z", "flip_axis": (False, False, False),
            "factor": 1.0, "vertex_group": "", "invert_vertex_group": False}


def configure_cache_modifier(modifier, path, request):
    for key, value in cache_settings(request).items():
        setattr(modifier, key, value)
    modifier.filepath = bpy.path.relpath(str(path)) if bpy.data.filepath else str(path)
    modifier.show_viewport = modifier.show_render = True


def validate_output_files(directory, request, result, bodies, verify_data):
    frames = validate_result(request, result, [len(b["vertices"]) for b in bodies],
                             [fully_fixed(b) for b in bodies])
    for output in result["objects"]:
        path = directory / f"object_{output['index']:04d}.mdd"
        inspect_mdd(path, output.get("stored_frames", frames), cache_vertices(output))
        if verify_data and output.get("sha256") and file_sha256(path) != output["sha256"]:
            raise ValueError(f"Cache checksum mismatch: {path.name}; restore or rebake the cache")


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
    from .materials import contact_label
    label = contact_label(obj.uipc_body.contact_material)
    if label != "Default":
        result["contact_material"] = label
    if obj.uipc_body.role == "CLOTH":
        from .materials import CLOTH_POISSON_FIELDS
        result.update({name: getattr(obj.uipc_body, name) for name in CLOTH_POISSON_FIELDS})
    if obj.uipc_body.role == "ROD":
        from .rod import ROD_FIELDS
        result.update({name: getattr(obj.uipc_body, name) for name in ROD_FIELDS})
    drive = drive_material(obj)
    if drive is not None:
        result["drive"] = drive
    return result


def collect_scene(scene, validate_geometry=True):
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
    # Omit DEFAULT to preserve the fingerprints of existing v1-v3 bakes.
    if settings.solver_accuracy != "DEFAULT":
        simulation["solver_accuracy"] = settings.solver_accuracy
    if settings.solver_accuracy == "CUSTOM":
        simulation["solver_settings"] = validate_solver_settings({
            name: getattr(settings, "solver_" + name) for name in SOLVER_FIELDS})
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
            from .rod_ui import is_display
            if is_display(obj, modifier):
                continue
            if modifier.name == MODIFIER_NAME and modifier.type != "MESH_CACHE" and not affine_playback.is_affine(modifier):
                raise ValueError(f"{obj.name}: rename the existing '{MODIFIER_NAME}' modifier")
            if modifier.name == MODIFIER_NAME and (modifier.type == "MESH_CACHE" or affine_playback.is_affine(modifier)):
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
        edges = np.empty((0, 2), dtype=np.int32)
        if obj.uipc_body.role == "ROD":
            if len(mesh.polygons):
                raise ValueError(f"{obj.name}: rods require edges only, not a tube surface")
            edges = np.empty(len(mesh.edges) * 2, dtype=np.int32)
            mesh.edges.foreach_get("vertices", edges)
            edges = edges.reshape(-1, 2)
            if validate_geometry:
                from .rod import validate_linemesh
                _, edges = validate_linemesh(world_vertices, edges, obj.name)
        elif obj.uipc_body.role == "FEM":
            stored = mesh.get("uipc_tetrahedra")
            if stored is None or len(stored) % 4:
                raise ValueError(f"{obj.name}: generate or import a tetrahedral mesh before baking FEM")
            tetrahedra = np.asarray(stored, dtype=np.int32).reshape(-1, 4)
            if validate_geometry:
                _, tetrahedra, boundary, _ = validate_tetmesh(world_vertices, tetrahedra, obj.name)
                if (len(triangles) != len(boundary)
                        or {tuple(sorted(t)) for t in triangles} != {tuple(sorted(t)) for t in boundary}):
                    raise ValueError(f"{obj.name}: visible faces no longer match the tetrahedral boundary; regenerate the volume")
                triangles = boundary
        elif validate_geometry:
            _, triangles = validate_mesh(world_vertices, triangles, obj.uipc_body.role, obj.name,
                                         allow_components=obj.uipc_body.driven)
        pins = []
        if obj.uipc_body.role in ("CLOTH", "FEM", "ROD") and obj.uipc_body.pin_group and not obj.uipc_body.fixed:
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
        if obj.uipc_body.role == "ROD":
            bodies[-1]["edges"] = edges
        if object_id(obj):
            bodies[-1]["id"] = object_id(obj)
    if not bodies or not any(b["material"]["role"] != "STATIC" for b in bodies):
        raise ValueError("Assign at least one object as Cloth, Rigid Body, or Volumetric FEM")
    if settings.contact_pairs:
        from .materials import normalize_contact_pairs
        pairs = [{key: getattr(pair, key) for key in ("material_a", "material_b", "friction", "resistance", "enabled")}
                 for pair in settings.contact_pairs]
        labels = {b["material"].get("contact_material", "Default") for b in bodies}
        simulation["contact_pairs"] = normalize_contact_pairs(pairs, labels)
    return simulation, bodies


@frontend_phase("export")
def export_job(scene):
    from .motion import sample_targets, sample_hash, align_robot_initial
    align_robot_initial(scene)
    from .rod_ui import refresh_displays
    refresh_displays(scene)
    ensure_scene_ids(scene)
    settings, bodies = collect_scene(scene)
    targets = sample_targets(scene, bodies)
    root = cache_root(scene)
    root.mkdir(parents=True, exist_ok=True)
    directory = root / ("bake_" + uuid.uuid4().hex)
    directory.mkdir()
    request = {"schema_version": SCHEMA_VERSION, "settings": settings,
               "output_options": {"compact_abd": scene.uipc_settings.compact_abd},
               "fingerprint": fingerprint(settings, bodies),
               "objects": [{"id": b["id"], "name": b["name"], "material": b["material"]} for b in bodies]}
    for index, body in enumerate(bodies):
        arrays = {key: body[key] for key in ("vertices", "triangles", "tetrahedra", "matrix", "pins")}
        if body["material"]["role"] == "ROD":
            arrays["edges"] = body["edges"]
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


@frontend_phase("attach")
def attach_cache(scene, directory, request):
    result = read_json(directory / "result.json")
    current_settings, bodies = collect_scene(scene)
    bodies = match_bodies(request, bodies)
    expected = request["fingerprint"]
    if result["schema_version"] != request["schema_version"] or result["fingerprint"] != expected:
        raise ValueError("Bake result does not match the exported scene")
    if fingerprint(current_settings, bodies) != expected:
        raise ValueError("Scene changed during baking; result was preserved on disk. Bake again")
    validate_output_files(directory, request, result, bodies, True)
    previous = (scene.uipc_settings.last_bake, scene.uipc_settings.baked_fingerprint)
    changes = []
    try:
        # Stage every replacement before disabling or deleting an existing cache.
        # Old modifiers/groups are kept intact until evaluation succeeds.
        for output in result["objects"]:
            index = output["index"]
            obj = resolve_object(scene, request["objects"][index], request["schema_version"])
            old_modifier = obj.modifiers.get(MODIFIER_NAME)
            snapshot = None if old_modifier is None else (
                old_modifier.name, old_modifier.show_viewport, old_modifier.show_render,
                list(obj.modifiers).index(old_modifier))
            path = directory / f"object_{index:04d}.mdd"
            if output.get("encoding", "VERTEX") == "AFFINE":
                replacement = affine_playback.create(scene, obj, path, request, index, configure_cache_modifier)
            else:
                replacement = obj.modifiers.new("libuipc Pending Cache", "MESH_CACHE")
                try:
                    configure_cache_modifier(replacement, path, request)
                    replacement.show_viewport = replacement.show_render = False
                except Exception:
                    obj.modifiers.remove(replacement)
                    raise
            changes.append((obj, old_modifier, snapshot, replacement))
        for obj, old_modifier, snapshot, replacement in changes:
            if old_modifier:
                old_modifier.name = "libuipc Previous Cache"
                old_modifier.show_viewport = old_modifier.show_render = False
            replacement.name = MODIFIER_NAME
            obj.modifiers.move(list(obj.modifiers).index(replacement), 0)
            replacement.show_viewport = replacement.show_render = True
        scene.uipc_settings.last_bake = bpy.path.relpath(str(directory)) if bpy.data.filepath else str(directory)
        scene.uipc_settings.baked_fingerprint = expected
        scene.frame_set(scene.frame_current)
    except Exception:
        for obj, old_modifier, snapshot, replacement in reversed(changes):
            affine_playback.remove(obj, replacement)
            if old_modifier:
                old_modifier.name, old_modifier.show_viewport, old_modifier.show_render = snapshot[:3]
                obj.modifiers.move(list(obj.modifiers).index(old_modifier), snapshot[3])
        scene.uipc_settings.last_bake, scene.uipc_settings.baked_fingerprint = previous
        raise
    for obj, old_modifier, snapshot, replacement in changes:
        if old_modifier:
            affine_playback.remove(obj, old_modifier)
    return result


@frontend_phase("validate")
def check_cache(scene, verify_data=False):
    if not scene.uipc_settings.last_bake:
        raise ValueError("No completed bake")
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = read_json(directory / "request.json")
    settings, bodies = collect_scene(scene)
    bodies = match_bodies(request, bodies)
    if cache_fingerprint(request, settings, bodies) != request["fingerprint"]:
        raise ValueError("Cache is stale: geometry, transforms, pins, materials, or scene settings changed")
    result = read_json(directory / "result.json")
    validate_output_files(directory, request, result, bodies, verify_data)
    for output in result["objects"]:
        index = output["index"]
        obj = resolve_object(scene, request["objects"][index], request["schema_version"])
        modifier = obj.modifiers.get(MODIFIER_NAME) if obj else None
        if not modifier:
            raise ValueError("A baked object's cache modifier is missing")
        if output.get("encoding", "VERTEX") == "AFFINE":
            cached = affine_playback.validate_binding(modifier, scene, request, index)
        elif modifier.type == "MESH_CACHE":
            cached = modifier
        else:
            raise ValueError("Cache modifier type does not match the output encoding")
        expected_path = directory / f"object_{index:04d}.mdd"
        if Path(bpy.path.abspath(cached.filepath)).resolve() != expected_path.resolve():
            raise ValueError("Cache modifier path has changed")
        if list(obj.modifiers).index(modifier) != 0:
            raise ValueError(f"{obj.name}: cache modifier must be first in the stack")
        for key, expected in cache_settings(request).items():
            actual = tuple(getattr(cached, key)) if key == "flip_axis" else getattr(cached, key)
            if actual != expected:
                raise ValueError(f"{obj.name}: cache playback setting '{key}' changed; expected {expected}")
    return result


def validate_for_render(scene, animation=False):
    if not scene.uipc_settings.last_bake:
        if any(o.uipc_body.role != "NONE" for o in scene.objects):
            raise ValueError("Bake the physical scene before validated rendering")
        return None
    result = check_cache(scene, verify_data=True)
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = read_json(directory / "request.json")
    start, end = request["settings"]["frame_start"], request["settings"]["frame_end"]
    if not animation and not start <= scene.frame_current + scene.frame_subframe <= end:
        raise ValueError("Render frame is outside the simulated cache range")
    for output in result["objects"]:
        obj = resolve_object(scene, request["objects"][output["index"]], request["schema_version"])
        if not obj.modifiers[MODIFIER_NAME].show_render:
            raise ValueError(f"{obj.name}: cache is disabled for rendering; validate/reactivate it first")
    return result


def activate_cache(scene):
    """Explicit validation can restore a cache after its inputs were reverted."""
    result = check_cache(scene, verify_data=True)
    request = read_json(Path(bpy.path.abspath(scene.uipc_settings.last_bake)) / "request.json")
    # Only restore modifiers whose input signature, path and file were checked.
    for output in result["objects"]:
        obj = resolve_object(scene, request["objects"][output["index"]], request["schema_version"])
        modifier = obj.modifiers[MODIFIER_NAME]
        modifier.show_viewport = True
        modifier.show_render = True
    scene.frame_set(scene.frame_current)
    return result


def detach_cache(scene):
    targets = [obj for obj in scene.objects if obj.get(affine_playback.MARKER) != 1]
    for obj in targets:
        modifier = obj.modifiers.get(MODIFIER_NAME)
        if modifier and (modifier.type == "MESH_CACHE" or affine_playback.is_affine(modifier)):
            affine_playback.remove(obj, modifier)
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
