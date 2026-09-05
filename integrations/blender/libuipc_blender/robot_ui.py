# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Blender controller hierarchy and driven link meshes from native UrdfIO data."""

import json

import bpy
import bmesh
import numpy as np
from mathutils import Euler, Matrix

from .protocol import read_json, validate_mesh


def attach_robot(scene, directory, request):
    original = {
        "objects": set(bpy.data.objects),
        "meshes": set(bpy.data.meshes),
        "materials": set(bpy.data.materials),
        "collections": set(bpy.data.collections),
    }
    active = scene.view_layers[0].objects.active
    flags = [
        (modifier, modifier.show_viewport, modifier.show_render)
        for obj in scene.objects
        for modifier in obj.modifiers
        if modifier.type == "MESH_CACHE"
    ]
    try:
        return _attach_robot(scene, directory, request)
    except Exception:
        for obj in set(bpy.data.objects) - original["objects"]:
            bpy.data.objects.remove(obj, do_unlink=True)
        for key in ("collections", "meshes", "materials"):
            owner = getattr(bpy.data, key)
            for value in set(owner) - original[key]:
                owner.remove(value)
        scene.view_layers[0].objects.active = active
        for modifier, visible, rendered in flags:
            modifier.show_viewport, modifier.show_render = visible, rendered
        raise


def _attach_robot(scene, directory, request):
    result = read_json(directory / "result.json")
    if result["fingerprint"] != request["fingerprint"]:
        raise ValueError("URDF import result does not match this request")
    model = read_json(directory / "robot.json")
    scale = scene.unit_settings.scale_length
    collection = bpy.data.collections.new("Robot - " + request["name"])
    scene.collection.children.link(collection)

    def empty(name, parent=None):
        obj = bpy.data.objects.new(name, None)
        collection.objects.link(obj)
        obj.empty_display_type = "PLAIN_AXES"
        obj.empty_display_size = 0.025 / scale
        obj.parent = parent
        return obj

    root = empty("Robot root - " + request["name"])
    root["uipc_robot_source"] = model["source"]
    targets = {model["root"]: root}
    angles = {}
    pending = list(model["joints"])
    while pending:
        previous = len(pending)
        for joint in pending[:]:
            if joint["parent"] not in targets:
                continue
            origin = empty("Joint origin " + joint["name"], targets[joint["parent"]])
            origin.location = np.array(joint["xyz"]) / scale
            origin.rotation_euler = Euler(joint["rpy"], "XYZ")
            control = empty("Joint angle " + joint["name"], origin)
            control.rotation_mode = "AXIS_ANGLE"
            axis = np.array(joint["axis"], dtype=float)
            axis /= np.linalg.norm(axis)
            control.rotation_axis_angle = (0, *axis)
            control["uipc_joint_name"] = joint["name"]
            if joint["limits"]:
                control["uipc_joint_limits"] = joint["limits"]
            targets[joint["child"]] = control
            angles[joint["name"]] = control
            pending.remove(joint)
        if previous == len(pending):
            raise ValueError("URDF joints do not form a connected tree")
    # Verify the hierarchy against an independent native UrdfController pose.
    for name, angle in model["fk_test_angles"].items():
        angles[name].rotation_axis_angle[0] = angle
    deps = scene.view_layers[0].depsgraph
    deps.update()
    error = 0.0
    for name, pose in model["fk_test_poses"].items():
        actual = np.array(targets[name].evaluated_get(deps).matrix_world)
        expected = np.array(pose)
        expected[:3, 3] /= scale
        error = max(error, float(np.abs(actual - expected).max()))
    if error > 2e-6:
        raise ValueError(f"Blender/native URDF forward kinematics mismatch: {error}")
    for control in angles.values():
        control.rotation_axis_angle[0] = 0
    deps.update()
    bodies = {}
    materials = {}
    for label, color in (
        ("Palm", (0.19, 0.045, 0.025)),
        ("Metal", (0.45, 0.48, 0.51)),
        ("Pad", (0.018, 0.025, 0.035)),
    ):
        material = bpy.data.materials.new("Robot " + label)
        material.use_nodes = True
        node = next(n for n in material.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
        node.inputs["Base Color"].default_value = (*color, 1)
        node.inputs["Metallic"].default_value = 0.65 if label == "Metal" else 0.1
        node.inputs["Roughness"].default_value = 0.32 if label == "Metal" else 0.65
        materials[label] = material
    for link in model["links"]:
        with np.load(
            directory / f"link_{link['index']:03d}.npz", allow_pickle=False
        ) as data:
            vertices, triangles = data["vertices"].copy(), data["triangles"].copy()
        mesh = bpy.data.meshes.new("Robot mesh " + link["name"])
        mesh.from_pydata((vertices / scale).tolist(), [], triangles.tolist())
        mesh.update()
        bm = bmesh.new()
        try:
            bm.from_mesh(mesh)
            boundary = [edge for edge in bm.edges if edge.is_boundary]
            repaired = len(boundary)
            if boundary:
                bmesh.ops.holes_fill(bm, edges=boundary, sides=0)
                bmesh.ops.triangulate(bm, faces=list(bm.faces))
                bm.to_mesh(mesh)
        finally:
            bm.free()
        mesh.calc_loop_triangles()
        vertices = np.array([v.co[:] for v in mesh.vertices]) * scale
        triangles = np.array([tri.vertices[:] for tri in mesh.loop_triangles])
        validate_mesh(vertices, triangles, "RIGID", link["name"], allow_components=True)
        obj = bpy.data.objects.new("Robot link " + link["name"], mesh)
        collection.objects.link(obj)
        obj.matrix_world = targets[link["name"]].evaluated_get(deps).matrix_world.copy()
        obj.uipc_body.role = "RIGID"
        obj.uipc_body.density = 5000
        obj.uipc_body.rigidity = 1e8
        obj.uipc_body.thickness = 0.0001
        obj.uipc_body.driven = True
        obj.uipc_body.drive_target = targets[link["name"]]
        obj.uipc_body.drive_group = root.name
        label = (
            "Pad"
            if "fingertip" in link["name"]
            else "Palm" if link["name"] == model["root"] else "Metal"
        )
        mesh.materials.append(materials[label])
        for face in mesh.polygons:
            face.use_smooth = True
        obj["uipc_robot_link"] = link["name"]
        obj["uipc_repaired_boundary_edges"] = repaired
        bodies[link["name"]] = obj.name
    root["uipc_robot_links"] = json.dumps(bodies)
    root["uipc_robot_targets"] = json.dumps(
        {name: obj.name for name, obj in targets.items()}
    )
    root["uipc_robot_joints"] = json.dumps(
        {name: obj.name for name, obj in angles.items()}
    )
    root["uipc_native_fk_error"] = error
    scene.view_layers[0].objects.active = root
    for obj in scene.objects:
        obj.select_set(obj == root)
    scene.uipc_settings.status = f"Imported {len(bodies)} robot links; keyframe the root and joint angles, then bake"
    result.update(root=root.name, links=len(bodies), native_fk_error=error)
    return result
