# Copyright (C) 2026 spiriMirror
# SPDX-License-Identifier: Apache-2.0
"""Actual Blender/CUDA tests for native meshing, solid FEM and whole-object fixing."""

import argparse
import importlib
import json
from pathlib import Path
import struct
import sys

import bpy
import numpy as np
from mathutils import Vector


def coordinates(obj, scene, frame):
    scene.frame_set(frame)
    graph = bpy.context.evaluated_depsgraph_get()
    graph.update()
    evaluated = obj.evaluated_get(graph)
    mesh = evaluated.to_mesh()
    try:
        points = np.empty(len(mesh.vertices) * 3, dtype=np.float64)
        mesh.vertices.foreach_get("co", points)
        return points.reshape(-1, 3).copy()
    finally:
        evaluated.to_mesh_clear()


def base_coordinates(obj):
    return np.array([v.co[:] for v in obj.data.vertices])


def triangles(mesh):
    mesh.calc_loop_triangles()
    return {tuple(sorted(t.vertices)) for t in mesh.loop_triangles}


def activate(obj):
    for other in bpy.context.view_layer.objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--addon-parent", type=Path)
    parser.add_argument("--module", default="libuipc_blender")
    parser.add_argument("--python", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.addon_parent:
        sys.path.insert(0, str(args.addon_parent.resolve()))
    addon = importlib.import_module(args.module)
    if not hasattr(bpy.types.Scene, "uipc_settings"):
        addon.register()
    demo = importlib.import_module(args.module + ".demo")
    scene = bpy.data.scenes.new("libuipc Volumetric FEM and Fixed")
    bpy.context.window.scene = scene
    scene.frame_start, scene.frame_end = 1, 41
    scene.render.fps = 50
    scene.uipc_settings.python_executable = args.python
    scene.uipc_settings.cache_directory = str(args.output / "cache")
    checks = []
    demo.mesh_object(scene, "FEM Floor", [(-4,-4,0),(4,-4,0),(4,4,0),(-4,4,0)],
                     [(0,1,2,3)], "STATIC")
    platform = demo.box(scene, "Fixed ABD Platform", (0,0,0.11), (2,2,0.2), "RIGID")
    platform.uipc_body.fixed = True

    def make_solid(name, position, fixed=False, preserve=True, scale=(1,1,1)):
        obj = demo.box(scene, name, position, (0.6,0.6,0.6), "FEM")
        obj.scale = scale
        obj.uipc_body.fixed = fixed
        obj.uipc_body.preserve_surface = preserve
        obj.uipc_body.young_modulus = 20000
        obj.uipc_body.poisson = 0.35
        obj.uipc_body.density = 1000
        obj.uipc_body.tet_quality_passes = 0 if preserve else 4
        if not preserve:
            obj.uipc_body.tet_edge_length = 0.15
        original = base_coordinates(obj)
        original_faces = triangles(obj.data)
        group = obj.vertex_groups.new(name="Original selection")
        group.add([0, 1], 0.75, "REPLACE")
        activate(obj)
        assert bpy.ops.uipc.generate_volume(blocking=True) == {"FINISHED"}
        assert obj.data.get("uipc_tetrahedra") is not None
        assert obj.uipc_body.source_mesh is not None
        np.testing.assert_array_equal(base_coordinates(obj)[:len(original)], original)
        assert obj.vertex_groups["Original selection"].weight(0) == 0.75
        if preserve:
            assert triangles(obj.data) == original_faces
        else:
            assert len(triangles(obj.data)) > len(original_faces)
        return obj, original, original_faces

    falling, _, _ = make_solid("Falling FEM", (0,0,0.95))
    fixed_solid, _, _ = make_solid("Fixed FEM", (2,0,0.9), fixed=True)
    fixed_solid.uipc_body.pin_group = "No group needed when wholly fixed"
    pinned, _, _ = make_solid("Partially Fixed FEM", (-2,0,0.9))
    pins = pinned.vertex_groups.new(name="Surface and Internal Pins")
    internal_id = len(pinned.uipc_body.source_mesh.vertices)
    assert internal_id < len(pinned.data.vertices)
    pins.add([0, internal_id], 1.0, "REPLACE")
    pinned.uipc_body.pin_group = pins.name
    fixed_cloth = demo.mesh_object(scene, "Fixed Cloth", [(-0.3,2,1),(0.3,2,1),(0,2.6,1)],
                                  [(0,1,2)], "CLOTH")
    fixed_cloth.uipc_body.fixed = True
    refined, refined_original, refined_faces = make_solid("Refined FEM", (2,2,0.9), fixed=True, preserve=False)
    mirrored, _, _ = make_solid("Mirrored FEM", (-2,2,0.9), fixed=True, scale=(-1,0.7,1.3))
    checks.append("native generation: original vertex positions/groups preserved, exact triangles locked, optional refinement works")
    rest = {o.name: base_coordinates(o) for o in (platform, falling, fixed_solid, pinned, fixed_cloth, refined, mirrored)}
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    for obj in (platform, fixed_solid, fixed_cloth, refined, mirrored):
        for frame in (1, 20, 41, 3):
            np.testing.assert_allclose(coordinates(obj, scene, frame), rest[obj.name], atol=3e-6)
    for frame in (1, 20, 41):
        points = coordinates(pinned, scene, frame)
        np.testing.assert_allclose(points[[0, internal_id]], rest[pinned.name][[0, internal_id]], atol=3e-6)
    assert np.max(np.abs(coordinates(pinned, scene, 41) - rest[pinned.name])) > 1e-4
    final = coordinates(falling, scene, 41)
    assert final[:, 2].mean() < rest[falling.name][:, 2].mean() - 0.2
    assert final[:, 2].min() + falling.location.z >= 0.209
    checks.append("whole-object fixed ABD/FEM/cloth stay fixed under gravity; surface AND internal FEM pin nodes stay fixed")
    checks.append("unfixed FEM deforms/falls and collides with a fixed ABD platform")
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    result = json.loads((directory / "result.json").read_text())
    max_error = 0.0
    for entry in result["objects"]:
        name = request["objects"][entry["index"]]["name"]
        with (directory / f"object_{entry['index']:04d}.mdd").open("rb") as stream:
            count, size = struct.unpack(">ii", stream.read(8))
            stream.read(count * 4)
            data = np.frombuffer(stream.read(), dtype=">f4").reshape(count, size, 3)
        for frame in (1, 41, 11, 2):
            observed = coordinates(scene.objects[name], scene, frame)
            max_error = max(max_error, float(np.max(np.abs(observed - data[frame-1]))))
            np.testing.assert_allclose(observed, data[frame-1], atol=3e-6)
    assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
    checks.append("all volume nodes, including internal nodes, match MDD playback at forward/backward frames")

    for obj, color in ((platform,(0.2,0.3,0.4)),(falling,(0.08,0.45,0.75)),
                       (fixed_solid,(0.9,0.3,0.08)),(pinned,(0.2,0.7,0.3)),
                       (refined,(0.6,0.3,0.7)),(mirrored,(0.9,0.6,0.1))):
        demo.material(obj, obj.name, color)
    camera_data = bpy.data.cameras.new("FEM Camera")
    camera = bpy.data.objects.new("FEM Camera", camera_data)
    scene.collection.objects.link(camera)
    camera.location = (5,-7,6)
    camera.rotation_euler = (Vector((0,0.7,0.6)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    camera_data.type, camera_data.ortho_scale = "ORTHO", 8
    scene.camera = camera
    light_data = bpy.data.lights.new("FEM Light", "AREA")
    light = bpy.data.objects.new("FEM Light", light_data)
    scene.collection.objects.link(light)
    light.location = (1,-2,6)
    light_data.energy, light_data.size = 1500, 7
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x, scene.render.resolution_y, scene.render.resolution_percentage = 1000, 800, 100
    scene.frame_set(41)
    blend = args.output / "volumetric_fem_fixed.blend"
    scene_name = scene.name
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    scene = bpy.data.scenes[scene_name]
    bpy.context.window.scene = scene
    pinned = scene.objects["Partially Fixed FEM"]
    assert scene.objects["Fixed FEM"].uipc_body.fixed
    assert pinned.data.get("uipc_tetrahedra") is not None
    assert pinned.vertex_groups.get("Surface and Internal Pins") is not None
    assert bpy.ops.uipc.validate_cache() == {"FINISHED"}
    scene.render.filepath = str(args.output / "volumetric_fem_fixed.png")
    bpy.ops.render.render(write_still=True)
    checks.append("tet topology, original surface backup, fixed flags, internal pins and caches survive .blend reopening/rendering")

    refined = scene.objects["Refined FEM"]
    activate(refined)
    assert bpy.ops.uipc.restore_surface() == {"FINISHED"}
    np.testing.assert_array_equal(base_coordinates(refined), refined_original)
    assert triangles(refined.data) == refined_faces
    assert refined.vertex_groups["Original selection"].weight(0) == 0.75
    checks.append("restoring a remeshed volume recovers original surface coordinates, faces and vertex groups")

    all_fixed_scene = bpy.data.scenes.new("All Fixed")
    bpy.context.window.scene = all_fixed_scene
    all_fixed_scene.frame_start, all_fixed_scene.frame_end = 1, 3
    all_fixed_scene.uipc_settings.python_executable = args.python
    all_fixed_scene.uipc_settings.cache_directory = str(args.output / "all_fixed")
    cube = demo.box(all_fixed_scene, "Only Fixed ABD", (0,0,1), (1,1,1), "RIGID")
    cube.uipc_body.fixed = True
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    np.testing.assert_array_equal(coordinates(cube, all_fixed_scene, 3), base_coordinates(cube))
    checks.append("an all-fixed scene bakes without a zero-DOF solve")
    source_msh = Path(__file__).resolve().parents[3] / "assets/sim_data/tetmesh/cube.msh"
    assert bpy.ops.uipc.import_volume(filepath=str(source_msh)) == {"FINISHED"}
    imported = bpy.context.view_layer.objects.active
    assert imported.uipc_body.role == "FEM"
    assert len(imported.data["uipc_tetrahedra"]) > 0
    imported.location = (3, 0, 1)
    imported.uipc_body.fixed = True
    imported_rest = base_coordinates(imported)
    assert bpy.ops.uipc.bake(blocking=True) == {"FINISHED"}
    np.testing.assert_array_equal(coordinates(imported, all_fixed_scene, 3), imported_rest)
    checks.append("existing tetrahedral MSH imports through native IO and bakes as a wholly fixed solid")
    bpy.ops.wm.open_mainfile(filepath=str(blend))
    report = {"blender": bpy.app.version_string, "worker": result["build_info"],
              "checks": checks, "maximum_cache_error": max_error, "blend": str(blend)}
    (args.output / "fem_validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("BLENDER_FEM_OK", json.dumps(report))


if __name__ == "__main__":
    main()
