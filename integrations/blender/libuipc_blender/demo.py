# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Asset-free example scene, also used by the real Blender integration tests."""

import bpy
from mathutils import Vector


def mesh_object(scene, name, vertices, faces, role):
    mesh = bpy.data.meshes.new(name + "Mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    scene.collection.objects.link(obj)
    obj.uipc_body.role = role
    return obj


def box(scene, name, center, size, role):
    x, y, z = [s / 2 for s in size]
    vertices = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
                (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    obj = mesh_object(scene, name, vertices, faces, role)
    obj.location = center
    return obj


def material(obj, name, color, metallic=0):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    # Default node names are translated in localized Blender installations.
    bsdf = next(node for node in mat.node_tree.nodes if node.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*color, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = 0.45
    obj.data.materials.append(mat)


def create_demo():
    scene = bpy.data.scenes.new("libuipc Cloth and Rigid Contact")
    scene.frame_start, scene.frame_end = 1, 61
    scene.render.fps = 50
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    ground = mesh_object(scene, "Ground", [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
                         [(0, 1, 2, 3)], "STATIC")
    obstacle = box(scene, "Fixed Platform", (0, 0, 0.41), (0.8, 0.8, 0.78), "STATIC")
    count = 17
    vertices = [(-0.95 + 1.9 * x / (count-1), -0.95 + 1.9 * y / (count-1), 1.3)
                for y in range(count) for x in range(count)]
    faces = []
    for y in range(count - 1):
        for x in range(count - 1):
            a = y * count + x
            faces.extend(((a, a+1, a+count+1), (a, a+count+1, a+count)))
    cloth = mesh_object(scene, "Pinned Cloth", vertices, faces, "CLOTH")
    pins = cloth.vertex_groups.new(name="Pins")
    pins.add([count * (count - 1), count * count - 1], 1.0, "REPLACE")
    cloth.uipc_body.pin_group = pins.name
    cube = box(scene, "Falling ABD", (0.2, -0.1, 1.85), (0.3, 0.3, 0.3), "RIGID")
    cube.uipc_body.density = 400
    material(ground, "Floor", (0.13, 0.16, 0.20))
    material(obstacle, "Platform", (0.28, 0.34, 0.40))
    material(cloth, "Cloth", (0.06, 0.43, 0.66))
    material(cube, "Rigid", (0.95, 0.32, 0.07), 0.3)
    camera_data = bpy.data.cameras.new("libuipc Camera")
    camera = bpy.data.objects.new("libuipc Camera", camera_data)
    scene.collection.objects.link(camera)
    camera.location = (3.4, -4.2, 3.0)
    camera.rotation_euler = (Vector((0, 0, 0.8)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = 4.0
    scene.camera = camera
    for name, location, energy, size in (("Key", (1, -2, 5), 900, 5), ("Fill", (-3, -1, 3), 500, 4)):
        data = bpy.data.lights.new(name, "AREA")
        light = bpy.data.objects.new(name, data)
        scene.collection.objects.link(light)
        light.location = location
        light.rotation_euler = (Vector((0, 0, 0.7)) - light.location).to_track_quat("-Z", "Y").to_euler()
        data.energy, data.shape, data.size = energy, "DISK", size
    scene.world = bpy.data.worlds.new("libuipc World")
    scene.world.use_nodes = True
    background = next(node for node in scene.world.node_tree.nodes if node.type == "BACKGROUND")
    background.inputs["Color"].default_value = (0.15, 0.15, 0.15, 1)
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x, scene.render.resolution_y = 1000, 800
    scene.render.resolution_percentage = 100
    scene.frame_set(scene.frame_start)
    return scene
