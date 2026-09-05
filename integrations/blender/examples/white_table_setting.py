# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Build, simulate, verify and render a completely procedural dining still life.

Run in Blender, with an external Python containing the CUDA pyuipc runtime:
  blender -b --factory-startup --python-exit-code 1 --python <this file> --
    --python <external-python> --output <directory> --mode all

No downloaded models/textures, Bullet, hand-posed cloth, or simulation restarts.
The fixed furniture is an explicit fixture; every tabletop object falls under
gravity in the same libuipc World. See README.md for assumptions and limits.
"""

import argparse
import importlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import time
import zipfile

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree


TABLE_CENTER = (-0.22, 0.27)
TABLE_SIZE = (1.20, 0.84)
TABLE_TOP = 0.76
CLOTH_NAME = "01 White linen tablecloth"


def mesh_object(name, vertices, faces, role="NONE", material=None):
    mesh = bpy.data.meshes.new(name + " mesh")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    if role != "NONE":
        obj.uipc_body.role = role
    if material:
        mesh.materials.append(material)
    return obj


def smooth(obj):
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def material(name, color, roughness, fabric=False, apple=False, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = (*color, 1)
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    shader = next(n for n in nodes if n.type == "BSDF_PRINCIPLED")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    tex = nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 700 if fabric else 140
    tex.inputs["Detail"].default_value = 2
    bump = nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.22 if fabric else 0.12
    bump.inputs["Distance"].default_value = 0.00012 if fabric else 0.00004
    links.new(tex.outputs["Fac"], bump.inputs["Height"])
    links.new(bump.outputs["Normal"], shader.inputs["Normal"])
    if fabric:
        shader.inputs["Sheen Weight"].default_value = 0.22
        shader.inputs["Sheen Roughness"].default_value = 0.75
        coord = nodes.new("ShaderNodeTexCoord")
        # UV coordinates move with the MDD-deformed cloth.
        links.new(coord.outputs["UV"], tex.inputs["Vector"])
        wave = nodes.new("ShaderNodeTexWave")
        wave.wave_type, wave.bands_direction = "BANDS", "X"
        wave.inputs["Scale"].default_value = 360
        links.new(coord.outputs["UV"], wave.inputs["Vector"])
        mix = nodes.new("ShaderNodeMixRGB")
        mix.blend_type = "MULTIPLY"
        mix.inputs[0].default_value = 0.65
        links.new(tex.outputs["Fac"], mix.inputs[1])
        links.new(wave.outputs["Color"], mix.inputs[2])
        links.new(mix.outputs[0], bump.inputs["Height"])
    if apple:
        shader.inputs["Subsurface Weight"].default_value = 0.045
        shader.inputs["Coat Weight"].default_value = 0.18
        shader.inputs["Coat Roughness"].default_value = 0.28
        broad = nodes.new("ShaderNodeTexNoise")
        broad.inputs["Scale"].default_value = 5
        ramp = nodes.new("ShaderNodeValToRGB")
        ramp.color_ramp.elements[0].position = 0.16
        ramp.color_ramp.elements[0].color = (0.10, 0.20, 0.015, 1)
        ramp.color_ramp.elements[1].position = 0.84
        ramp.color_ramp.elements[1].color = (0.40, 0.52, 0.05, 1)
        links.new(broad.outputs["Fac"], ramp.inputs["Fac"])
        links.new(ramp.outputs["Color"], shader.inputs["Base Color"])
    return mat


def box(name, center, size, mat, role="NONE", bevel=0.0):
    x, y, z = np.array(size) / 2
    vertices = [
        (-x, -y, -z),
        (x, -y, -z),
        (x, y, -z),
        (-x, y, -z),
        (-x, -y, z),
        (x, -y, z),
        (x, y, z),
        (-x, y, z),
    ]
    faces = [
        (0, 3, 2, 1),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    obj = mesh_object(name, vertices, faces, role, mat)
    obj.location = center
    if bevel:
        mod = obj.modifiers.new("Rounded physical edges", "BEVEL")
        mod.width, mod.segments = bevel, 3
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.modifier_apply(modifier=mod.name)
        obj.select_set(False)
    return obj


def tube(name, path, radius, mat, sides=12):
    points = [Vector(p) for p in path]
    vertices, faces = [], []
    for i, p in enumerate(points):
        tangent = (
            points[min(i + 1, len(points) - 1)] - points[max(0, i - 1)]
        ).normalized()
        seed = Vector((0, 1, 0)) if abs(tangent.y) < 0.95 else Vector((1, 0, 0))
        a, b = tangent.cross(seed).normalized(), None
        b = tangent.cross(a).normalized()
        r = radius[i] if isinstance(radius, list) else radius
        for j in range(sides):
            angle = 2 * math.pi * j / sides
            vertices.append(p + r * (math.cos(angle) * a + math.sin(angle) * b))
        if i:
            for j in range(sides):
                k = (j + 1) % sides
                faces.append(
                    (
                        (i - 1) * sides + j,
                        (i - 1) * sides + k,
                        i * sides + k,
                        i * sides + j,
                    )
                )
    faces.extend(
        (
            tuple(reversed(range(sides))),
            tuple((len(points) - 1) * sides + j for j in range(sides)),
        )
    )
    return smooth(mesh_object(name, vertices, faces, material=mat))


def lathe(name, profile, center, mat, role="RIGID", segments=64):
    """Revolve an oriented closed material section; axis endpoints are single vertices."""
    vertices, rings, faces = [], [], []
    for radius, height in profile:
        if radius == 0:
            rings.append([len(vertices)])
            vertices.append((0, 0, height))
        else:
            rings.append(list(range(len(vertices), len(vertices) + segments)))
            vertices.extend(
                (
                    radius * math.cos(2 * math.pi * j / segments),
                    radius * math.sin(2 * math.pi * j / segments),
                    height,
                )
                for j in range(segments)
            )
    for a, b in zip(rings, rings[1:]):
        for j in range(segments):
            k = (j + 1) % segments
            if len(a) == 1:
                faces.append((a[0], b[k], b[j]))
            elif len(b) == 1:
                faces.append((a[j], a[k], b[0]))
            else:
                faces.extend(((a[j], a[k], b[k]), (a[j], b[k], b[j])))
    obj = smooth(mesh_object(name, vertices, faces, role, mat))
    obj.location = center
    obj.uipc_body.density = 2400
    obj.uipc_body.rigidity = 1e9
    obj.uipc_body.thickness = 0.00015
    return obj


def cloth_grid(name, center, size, nx, ny, mat, placemat=False):
    vertices = []
    for j in range(ny):
        for i in range(nx):
            x = size[0] * (i / (nx - 1) - 0.5)
            y = size[1] * (j / (ny - 1) - 0.5)
            # A sub-millimeter, deterministic rest perturbation breaks symmetry;
            # it is not the final drape or an imposed gravity deformation.
            dz = (
                (0.00012 if placemat else 0.0007)
                * math.sin(23 * x + 0.3)
                * math.sin(19 * y + 0.7)
            )
            vertices.append((x, y, dz))
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            if (i + j) % 2:
                faces.extend(((a, a + 1, a + nx), (a + 1, a + nx + 1, a + nx)))
            else:
                faces.extend(((a, a + 1, a + nx + 1), (a, a + nx + 1, a + nx)))
    obj = smooth(mesh_object(name, vertices, faces, "CLOTH", mat))
    obj.location = center
    uv = obj.data.uv_layers.new(name="Woven coordinates")
    for loop in obj.data.loops:
        uv.data[loop.index].uv = (
            loop.vertex_index % nx / (nx - 1),
            loop.vertex_index // nx / (ny - 1),
        )
    body = obj.uipc_body
    body.density = 400 if placemat else 250
    body.thickness = 0.0010 if placemat else 0.0004
    body.stretch = 5e6 if placemat else 2e6
    body.shear = 3000 if placemat else 100
    body.bending = 3e6 if placemat else 3e4
    body.poisson = 0.3
    body.strain_rate = 100
    # No subdivision smoothing: the rendered mid-surface is the solved mesh.
    mod = obj.modifiers.new("Physical two-sided fabric thickness", "SOLIDIFY")
    mod.thickness, mod.offset = 2 * body.thickness, 0.0
    return obj


def apple(name, center, radius, mat, phase):
    # Closed, mildly lobed apple; no disconnected collision decorations.
    rows, segments = 18, 32
    vertices = [(0, 0, -0.88 * radius)]
    for row in range(1, rows):
        theta = math.pi * row / rows
        z = -math.cos(theta)
        for col in range(segments):
            phi = 2 * math.pi * col / segments
            rr = (
                radius
                * math.sin(theta)
                * (1 + 0.055 * z)
                * (1 + 0.035 * math.cos(5 * phi + phase) * abs(z) ** 2)
            )
            zz = radius * (z - 0.12 * math.copysign(abs(z) ** 14, z))
            vertices.append((rr * math.cos(phi), rr * math.sin(phi), zz))
    top = len(vertices)
    vertices.append((0, 0, 0.88 * radius))
    faces = [(0, 1 + (j + 1) % segments, 1 + j) for j in range(segments)]
    for row in range(rows - 2):
        a, b = 1 + row * segments, 1 + (row + 1) * segments
        for j in range(segments):
            k = (j + 1) % segments
            faces.extend(((a + j, a + k, b + k), (a + j, b + k, b + j)))
    faces.extend(
        (
            top,
            1 + (rows - 2) * segments + j,
            1 + (rows - 2) * segments + (j + 1) % segments,
        )
        for j in range(segments)
    )
    obj = smooth(mesh_object(name, vertices, faces, "RIGID", mat))
    obj.location = center
    obj.uipc_body.density = 820
    obj.uipc_body.rigidity = 1e8
    obj.uipc_body.thickness = 0.00015
    obj["stem_radius"] = radius
    return obj


def flatware(name, center, mat, fork=False):
    if fork:
        contour = [
            (-0.097, -0.010),
            (-0.045, -0.010),
            (-0.018, -0.003),
            (0.082, -0.0045),
            (0.090, 0),
            (0.082, 0.0045),
            (-0.018, 0.003),
            (-0.045, 0.010),
            (-0.097, 0.010),
            (-0.097, 0.006),
            (-0.055, 0.006),
            (-0.055, 0.002),
            (-0.097, 0.002),
            (-0.097, -0.002),
            (-0.055, -0.002),
            (-0.055, -0.006),
            (-0.097, -0.006),
        ]
    else:
        contour = [
            (-0.100, -0.004),
            (-0.080, -0.006),
            (-0.012, -0.006),
            (0.000, -0.0035),
            (0.085, -0.0045),
            (0.096, 0),
            (0.085, 0.0045),
            (0.000, 0.0035),
            (-0.012, 0.004),
            (-0.091, 0.004),
        ]
    n = len(contour)
    vertices = [(x, y, z) for z in (-0.0009, 0.0009) for x, y in contour]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
    faces.extend((i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n))
    obj = mesh_object(name, vertices, faces, "RIGID", mat)
    obj.location = center
    obj.uipc_body.density = 7800
    obj.uipc_body.rigidity = 1e9
    obj.uipc_body.thickness = 0.00010
    return obj


def furniture(paint, cushion_mat):
    tx, ty = TABLE_CENTER
    tabletop = box(
        "00 Fixed tabletop",
        (tx, ty, TABLE_TOP - 0.022),
        (*TABLE_SIZE, 0.044),
        paint,
        "STATIC",
        0.006,
    )
    tabletop.uipc_body.thickness = 0.00001
    for x in (-0.52, 0.52):
        for y in (-0.34, 0.34):
            box(
                "Table leg",
                (tx + x, ty + y, 0.358),
                (0.045, 0.045, 0.716),
                paint,
                bevel=0.004,
            )
    for y in (-0.34, 0.34):
        box(
            "Table apron", (tx, ty + y, 0.677), (1.05, 0.025, 0.065), paint, bevel=0.002
        )
    ground = mesh_object(
        "00 Floor at z = 0",
        [(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)],
        [(0, 1, 2, 3)],
        "STATIC",
        material("Studio floor", (0.88, 0.88, 0.87), 0.30),
    )
    ground.uipc_body.thickness = 0.00001
    ground.hide_render = True
    ground.hide_set(True)
    # The visual continuation is exactly coplanar, but must not inflate the
    # solver's scene scale and adaptive barrier stiffness corridor.
    mesh_object(
        "Studio floor continuation",
        [(-100, -100, 0), (100, -100, 0), (100, 100, 0), (-100, 100, 0)],
        [(0, 1, 2, 3)],
        material=ground.data.materials[0],
    )
    start = set(bpy.context.scene.objects)
    seat = box(
        "Chair sculpted seat", (0, 0, 0.438), (0.47, 0.46, 0.044), paint, bevel=0.018
    )
    for x in (-1, 1):
        for y in (-1, 1):
            leg = tube(
                "Chair splayed leg",
                [(0.242 * x, 0.232 * y, 0.017), (0.181 * x, 0.166 * y, 0.427)],
                [0.014, 0.020],
                paint,
                20,
            )
            for vertex in list(leg.data.vertices)[:20]:
                vertex.co.z = 0.0
            leg.data.update()
    for y in (-1, 1):
        tube(
            "Chair cross stretcher",
            [(-0.215, 0.208 * y, 0.15), (0.215, 0.208 * y, 0.15)],
            0.012,
            paint,
        )
    for x in (-1, 1):
        tube(
            "Chair side stretcher",
            [(0.207 * x, -0.209, 0.19), (0.207 * x, 0.209, 0.19)],
            0.011,
            paint,
        )
    path = [
        (0.229 * math.cos(t), 0.175 + 0.125 * math.sin(t), 0.45 + 0.59 * math.sin(t))
        for t in np.linspace(0, math.pi, 65)
    ]
    tube("Chair steam bent back hoop", path, 0.015, paint, 16)
    for x in (-0.164, -0.082, 0, 0.082, 0.164):
        s = math.sqrt(1 - (x / 0.229) ** 2)
        tube(
            "Chair back spindle",
            [(x, 0.171, 0.46), (x, 0.175 + 0.125 * s, 0.45 + 0.59 * s)],
            [0.010, 0.008],
            paint,
            16,
        )
    vertices, faces = [], []
    n = 49
    for j in range(n):
        v = 2 * j / (n - 1) - 1
        for i in range(n):
            u = 2 * i / (n - 1) - 1
            z = 0.478 + 0.044 * max(0, (1 - u * u) * (1 - v * v)) ** 0.55
            for a, b in ((-0.40, -0.15), (0.40, -0.15)):
                z -= 0.011 * math.exp(-((u - a) ** 2 + (v - b) ** 2) / 0.021)
            vertices.append(
                (0.220 * u * (1 - 0.11 * v**8), 0.210 * v * (1 - 0.11 * u**8), z)
            )
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i
            faces.append((a, a + 1, a + n + 1, a + n))
    cushion = smooth(
        mesh_object("Chair upholstered cushion", vertices, faces, material=cushion_mat)
    )
    thick = cushion.modifiers.new("Upholstery depth", "SOLIDIFY")
    thick.thickness = 0.022
    edge = (
        list(range(n))
        + [j * n + n - 1 for j in range(1, n)]
        + list(range(n * n - 2, n * (n - 1) - 1, -1))
        + [j * n for j in range(n - 2, 0, -1)]
    )
    tube(
        "Cushion sewn piping",
        [vertices[i] for i in edge] + [vertices[edge[0]]],
        0.0017,
        cushion_mat,
        8,
    )
    angle = math.radians(15)
    for obj in set(bpy.context.scene.objects) - start:
        p = obj.location.copy()
        obj.location = (
            0.95 + math.cos(angle) * p.x - math.sin(angle) * p.y,
            -0.40 + math.sin(angle) * p.x + math.cos(angle) * p.y,
            p.z,
        )
        obj.rotation_euler.z += angle
    # Chair/furniture legs and cushion are fixed visual fixtures, outside the
    # falling objects' swept volume. The tabletop and floor are actual colliders.
    return tabletop


def camera_lights(scene):
    data = bpy.data.cameras.new("Dining still life camera")
    camera = bpy.data.objects.new(data.name, data)
    scene.collection.objects.link(camera)
    camera.location = (0.8, -3.8, 1.95)
    target = Vector((0.18, 0.02, 0.55))
    camera.rotation_euler = (
        (target - camera.location).to_track_quat("-Z", "Y").to_euler()
    )
    data.type = "PERSP"
    data.lens = 55
    scene.camera = camera
    for name, loc, power, size in (
        ("Large softbox left", (-2.7, -3.2, 4.3), 450, 4),
        ("Softbox right", (3.0, -0.4, 3.3), 180, 3),
        ("Rear softbox", (-0.2, 3.0, 4.0), 350, 3.0),
    ):
        ld = bpy.data.lights.new(name, "AREA")
        lo = bpy.data.objects.new(name, ld)
        scene.collection.objects.link(lo)
        lo.location = loc
        lo.rotation_euler = (
            (Vector((0, 0, 0.6)) - lo.location).to_track_quat("-Z", "Y").to_euler()
        )
        ld.energy, ld.shape, ld.size = power, "DISK", size
    scene.world = bpy.data.worlds.new("White photography studio")
    scene.world.use_nodes = True
    bg = next(n for n in scene.world.node_tree.nodes if n.type == "BACKGROUND")
    bg.inputs["Color"].default_value = (1, 1, 1, 1)
    bg.inputs["Strength"].default_value = 0.45
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 2000, 1450
    scene.render.resolution_percentage = 100
    scene.view_settings.view_transform = "AgX"
    scene.view_settings.look = "AgX - Medium High Contrast"
    scene.view_settings.exposure = 0.35
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    studio_finish(scene)


def studio_finish(scene):
    """Photographic presentation only; leave every simulated coordinate intact."""
    camera = scene.camera
    camera.location = (1.7, -3.5, 1.95)
    camera.rotation_euler = (
        (Vector((0.20, 0.06, 0.47)) - camera.location)
        .to_track_quat("-Z", "Y")
        .to_euler()
    )
    camera.data.lens = 55
    camera.data.shift_y = -0.075
    for obj in scene.objects:
        if obj.name.startswith(("Chair ", "Cushion ")):
            obj.rotation_euler.z = math.radians(15)
    scene.objects["Large softbox left"].data.energy = 650
    scene.objects["Large softbox left"].data.size = 2.5
    scene.objects["Softbox right"].data.energy = 100
    bg = next(n for n in scene.world.node_tree.nodes if n.type == "BACKGROUND")
    bg.inputs["Strength"].default_value = 0.25
    cushion = scene.objects["Chair upholstered cushion"]
    if not cushion.data.uv_layers:
        uv = cushion.data.uv_layers.new(name="Upholstery weave")
        for loop in cushion.data.loops:
            uv.data[loop.index].uv = (
                loop.vertex_index % 49 / 48,
                loop.vertex_index // 49 / 48,
            )
    scene.objects["Studio floor continuation"].is_shadow_catcher = True
    scene.render.film_transparent = True
    scene.use_nodes = True
    nodes, links = scene.node_tree.nodes, scene.node_tree.links
    layers = next((n for n in nodes if n.type == "R_LAYERS"), None)
    if layers is None:
        layers = nodes.new("CompositorNodeRLayers")
    composite = next((n for n in nodes if n.type == "COMPOSITE"), None)
    if composite is None:
        composite = nodes.new("CompositorNodeComposite")
    over = nodes.get("White studio background")
    if over is None:
        over = nodes.new("CompositorNodeAlphaOver")
        over.name = "White studio background"
    over.inputs[0].default_value = 1
    over.inputs[1].default_value = (4, 4, 4, 1)
    links.new(layers.outputs["Image"], over.inputs[2])
    links.new(over.outputs["Image"], composite.inputs["Image"])


def build(args):
    scene = bpy.data.scenes.new("libuipc - White table setting")
    bpy.context.window.scene = scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1
    scene.frame_start, scene.frame_end, scene.render.fps = 1, args.frames, 30
    settings = scene.uipc_settings
    settings.python_executable = args.python
    settings.cache_directory = str(args.output / "cache")
    settings.gravity = (0, 0, -9.81)
    settings.substeps = args.substeps
    settings.friction = 0.45
    settings.d_hat = 0.0008
    settings.resistance = 1e9
    paint = material("Warm white painted wood", (0.82, 0.83, 0.81), 0.33)
    linen = material("Ivory fine woven linen", (0.86, 0.85, 0.82), 0.86, fabric=True)
    cushion = material("White woven upholstery", (0.82, 0.83, 0.81), 0.90, fabric=True)
    ceramic = material("White glazed porcelain", (0.91, 0.92, 0.90), 0.16)
    straw = material("Natural flax placemats", (0.46, 0.32, 0.18), 0.85, fabric=True)
    green = material("Golden green apple skin", (0.51, 0.61, 0.13), 0.32, apple=True)
    silver = material(
        "Brushed stainless flatware", (0.68, 0.70, 0.72), 0.23, metallic=1
    )
    furniture(paint, cushion)
    tx, ty = TABLE_CENTER
    cloth_grid(
        CLOTH_NAME, (tx, ty, 0.798), (1.78, 1.48), args.cloth_x, args.cloth_y, linen
    )
    plate_profile = [
        (0, 0),
        (0.065, 0),
        (0.081, 0.003),
        (0.104, 0.010),
        (0.124, 0.017),
        (0.130, 0.020),
        (0.129, 0.024),
        (0.123, 0.024),
        (0.102, 0.017),
        (0.080, 0.009),
        (0.060, 0.006),
        (0, 0.006),
    ]
    bowl_profile = [
        (0, 0),
        (0.042, 0),
        (0.055, 0.004),
        (0.073, 0.016),
        (0.092, 0.038),
        (0.107, 0.062),
        (0.106, 0.065),
        (0.101, 0.065),
        (0.088, 0.040),
        (0.069, 0.022),
        (0.049, 0.012),
        (0, 0.010),
    ]
    fruit_profile = [
        (0, 0),
        (0.062, 0),
        (0.080, 0.007),
        (0.103, 0.023),
        (0.123, 0.052),
        (0.137, 0.084),
        (0.145, 0.113),
        (0.144, 0.117),
        (0.139, 0.117),
        (0.130, 0.087),
        (0.117, 0.059),
        (0.098, 0.035),
        (0.076, 0.020),
        (0.053, 0.012),
        (0, 0.011),
    ]
    for label, x in (("Left", -0.60), ("Right", 0.16)):
        cloth_grid(
            f"02 {label} linen placemat",
            (x, 0.21, 0.837),
            (0.325, 0.32),
            17,
            17,
            straw,
            placemat=True,
        )
        lathe(f"03 {label} dinner plate", plate_profile, (x, 0.238, 0.875), ceramic)
        lathe(f"04 {label} shallow bowl", bowl_profile, (x, 0.238, 0.930), ceramic)
        flatware(f"07 {label} fork", (x, 0.092, 0.901), silver, fork=True)
        flatware(f"07 {label} knife", (x, 0.068, 0.913), silver)
    fruit_profile = [(r, z * 0.72) for r, z in fruit_profile]
    lathe(
        "05 Center fruit bowl",
        fruit_profile,
        (-0.22, 0.47, 0.846),
        ceramic,
        segments=72,
    )
    for i, (x, y, z, r) in enumerate(
        (
            (-0.044, -0.043, 1.055, 0.038),
            (0.043, -0.043, 1.064, 0.037),
            (-0.044, 0.044, 1.062, 0.039),
            (0.044, 0.044, 1.055, 0.038),
            (0, 0, 1.151, 0.040),
        )
    ):
        apple(f"06 Apple {i+1}", (-0.22 + x, 0.47 + y, z), r, green, i * 0.8)
    camera_lights(scene)
    combine_fixed_colliders(scene)
    scene.frame_set(1)
    scene["physical_model"] = (
        "One continuous libuipc IPC gravity solve; no pinned cloth vertices. Floor z=0 m. Furniture is fixed."
    )
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output / "white_table_setting.blend"))
    return scene


def combine_fixed_colliders(scene):
    """One compound fixed environment, using the actual evaluated visible meshes.

    Fixed-fixed furniture joints require no response. Their self contact is off
    inside this one STATIC mesh, but contact against ALL moving objects stays on.
    Do not approximate a table as just its top: a curled hem can reach a leg.
    """
    sources = [
        obj
        for obj in scene.objects
        if obj.name in ("00 Fixed tabletop", "00 Floor at z = 0")
        or obj.name.startswith(("Chair ", "Cushion ", "Table leg", "Table apron"))
    ]
    scene.view_layers[0].update()
    deps = bpy.context.evaluated_depsgraph_get()
    deps.update()
    vertices, faces, components = [], [], []
    for source in sorted(sources, key=lambda obj: obj.name):
        obj = source.evaluated_get(deps)
        mesh = obj.to_mesh()
        try:
            mesh.calc_loop_triangles()
            first = len(vertices)
            vertices.extend(obj.matrix_world @ v.co for v in mesh.vertices)
            faces.extend(
                tuple(first + i for i in tri.vertices) for tri in mesh.loop_triangles
            )
            components.append(
                {"source": source.name, "first": first, "count": len(mesh.vertices)}
            )
        finally:
            obj.to_mesh_clear()
        source.uipc_body.role = "NONE"
    collider = mesh_object("00 Fixed furniture and ground", vertices, faces, "STATIC")
    collider.uipc_body.thickness = 0.00001
    collider["component_ranges"] = json.dumps(components)
    collider.hide_render = True
    collider.hide_set(True)


def mdd(path):
    with path.open("rb") as f:
        frames, vertices = struct.unpack(">ii", f.read(8))
        times = np.frombuffer(f.read(4 * frames), dtype=">f4").astype(float)
        points = (
            np.frombuffer(f.read(), dtype=">f4")
            .reshape(frames, vertices, 3)
            .astype(float)
        )
    if not np.isfinite(points).all():
        raise AssertionError(f"Non-finite trajectory in {path}")
    return times, points


def verify(scene, addon, output):
    result = addon.bridge.check_cache(scene)
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    final, initial, triangles, metrics = {}, {}, {}, {}
    trajectories, materials = {}, {}
    for index, entry in enumerate(request["objects"]):
        name = entry["name"]
        materials[name] = entry["material"]
        with np.load(directory / f"input_{index:04d}.npz", allow_pickle=False) as data:
            matrix = data["matrix"].copy()
            triangles[name] = data["triangles"].copy()
            rest = data["vertices"] @ matrix[:3, :3].T + matrix[:3, 3]
        initial[name] = rest
        if entry["material"]["role"] == "STATIC":
            final[name] = rest
            continue
        times, points = mdd(directory / f"object_{index:04d}.mdd")
        world = points @ matrix[:3, :3].T + matrix[:3, 3]
        trajectories[name] = world
        final[name] = world[-1]
        speed = np.linalg.norm(np.diff(world[-16:], axis=0), axis=2) * scene.render.fps
        metrics[name] = {
            "vertices": len(rest),
            "min_z_m": float(world[-1, :, 2].min()),
            "center_initial_m": rest.mean(axis=0).tolist(),
            "center_final_m": world[-1].mean(axis=0).tolist(),
            "last_half_second_max_vertex_speed_m_s": float(speed.max()),
            "last_half_second_rms_vertex_speed_m_s": float(np.sqrt(np.mean(speed**2))),
        }
        if entry["material"]["role"] == "RIGID":
            fit = np.linalg.lstsq(
                np.column_stack((rest, np.ones(len(rest)))), world[-1], rcond=None
            )[0]
            singular_values = np.linalg.svd(fit[:3], compute_uv=False)
            metrics[name]["affine_principal_stretches"] = singular_values.tolist()
            assert (
                np.max(abs(singular_values - 1)) < 0.002
            ), f"{name}: excessive rigid deformation"
        assert np.min(world[:, :, 2]) > -2e-5, f"{name}: crossed floor z=0"

    # BVH triangle intersections inspect surfaces, not bounding boxes. Separate
    # nesting (apples inside the bowl cavity) is allowed; wall crossings are not.
    def intersections(positions, return_trees=False):
        trees = {
            n: BVHTree.FromPolygons(
                [Vector(p) for p in v],
                triangles[n].tolist(),
                all_triangles=True,
                epsilon=0.0,
            )
            for n, v in positions.items()
        }
        crossings = []
        names = list(trees)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                pairs = trees[a].overlap(trees[b])
                if pairs:
                    crossings.append({"a": a, "b": b, "triangle_pairs": len(pairs)})
        return (crossings, trees) if return_trees else crossings

    initial_crossings, final_crossings = intersections(initial), intersections(final)
    assert not initial_crossings, f"Initial intersecting surfaces: {initial_crossings}"
    assert not final_crossings, f"Final intersecting surfaces: {final_crossings}"
    # A positive AABB separation is a conservative lower bound on surface
    # separation, including containment cases that intersection tests miss.
    initial_clearance = float("inf")
    names = list(initial)
    component_bounds = {}
    for name, points in initial.items():
        components = json.loads(scene.objects[name].get("component_ranges", "[]"))
        # A hoop's whole AABB includes its empty center. Bound the union of
        # actual fixed triangles instead, never confuse that empty space with
        # material. This remains a conservative surface-distance lower bound.
        if components:
            p = points[triangles[name]]
            component_bounds[name] = (p.min(axis=1), p.max(axis=1))
        else:
            component_bounds[name] = (
                points.min(axis=0)[None, :],
                points.max(axis=0)[None, :],
            )
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            amin, amax = component_bounds[a]
            bmin, bmax = component_bounds[b]
            gaps = np.maximum(
                0,
                np.maximum(
                    amin[:, None, :] - bmax[None, :, :],
                    bmin[None, :, :] - amax[:, None, :],
                ),
            )
            gap = (
                float(np.linalg.norm(gaps, axis=2).min())
                - materials[a]["thickness"]
                - materials[b]["thickness"]
            )
            assert (
                gap > 1e-5
            ), f"Initial thickness separation not proven for {a}/{b}: {gap}"
            initial_clearance = min(initial_clearance, gap)
    assert (
        initial_clearance > 1e-5
    ), f"Initial thickness shells not provably separate: {initial_clearance}"
    print("Checking every cached frame for inter-object surface crossings", flush=True)
    for frame in range(1, result["frames"] - 1):
        positions = {
            name: trajectories[name][frame] if name in trajectories else points
            for name, points in initial.items()
        }
        crossings = intersections(positions)
        assert not crossings, f"Surface crossing at cached frame {frame+1}: {crossings}"
    _, trees = intersections(final, return_trees=True)
    connections = {name: set() for name in final}
    contacts = []
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            limit = (
                materials[a]["thickness"]
                + materials[b]["thickness"]
                + scene.uipc_settings.d_hat
                + 0.0001
            )
            gaps = np.maximum(
                0,
                np.maximum(
                    final[a].min(axis=0) - final[b].max(axis=0),
                    final[b].min(axis=0) - final[a].max(axis=0),
                ),
            )
            if np.linalg.norm(gaps) > limit:
                continue
            distance = min(
                min(trees[b].find_nearest(Vector(p))[3] for p in final[a]),
                min(trees[a].find_nearest(Vector(p))[3] for p in final[b]),
            )
            if distance <= limit:
                connections[a].add(b)
                connections[b].add(a)
                contacts.append(
                    {"a": a, "b": b, "sampled_surface_distance_m": distance}
                )
    supported = {n for n, m in materials.items() if m["role"] == "STATIC"}
    while True:
        expanded = supported | set().union(*(connections[n] for n in supported))
        if expanded == supported:
            break
        supported = expanded
    assert (
        set(final) <= supported
    ), f"Objects without a contact path to fixed support: {set(final)-supported}"
    for name in (CLOTH_NAME, "02 Left linen placemat", "02 Right linen placemat"):
        pairs = trees[name].overlap(trees[name])
        crossing = [
            (a, b)
            for a, b in pairs
            if a < b and not set(triangles[name][a]) & set(triangles[name][b])
        ]
        assert (
            not crossing
        ), f"{name}: nonadjacent triangles self-intersect at final frame"
    cloth = final[CLOTH_NAME]
    cx, cy = TABLE_CENTER
    on_top = (
        (abs(cloth[:, 0] - cx) < TABLE_SIZE[0] / 2 - 0.02)
        & (abs(cloth[:, 1] - cy) < TABLE_SIZE[1] / 2 - 0.02)
        & (cloth[:, 2] > TABLE_TOP - 0.04)
    )
    # A hanging hem can legitimately curl inward BELOW the tabletop. Projected
    # XY containment alone must not classify that as a tabletop penetration.
    clearance = float(cloth[on_top, 2].min() - TABLE_TOP)
    assert clearance >= -2e-5, f"Cloth penetrated tabletop: {clearance}"
    assert cloth[:, 2].min() < TABLE_TOP - 0.18, "Cloth did not drape"
    # Compare EVERY base-mesh vertex to native Blender playback with display
    # thickness temporarily disabled. Restore all modifiers immediately.
    saved = []
    for obj in scene.objects:
        for mod in obj.modifiers:
            if mod.type != "MESH_CACHE":
                saved.append((mod, mod.show_viewport))
                mod.show_viewport = False
    max_error = 0.0
    try:
        for frame in (1, max(2, scene.frame_end // 2), scene.frame_end):
            scene.frame_set(frame)
            deps = bpy.context.evaluated_depsgraph_get()
            deps.update()
            for name, world in trajectories.items():
                obj = scene.objects[name].evaluated_get(deps)
                mesh = obj.to_mesh()
                try:
                    points = np.array([v.co[:] for v in mesh.vertices])
                    mat = np.array(obj.matrix_world)
                    points = points @ mat[:3, :3].T + mat[:3, 3]
                    max_error = max(
                        max_error, float(np.max(abs(points - world[frame - 1])))
                    )
                finally:
                    obj.to_mesh_clear()
    finally:
        for mod, enabled in saved:
            mod.show_viewport = enabled
    assert max_error < 2e-6, f"Blender/cache playback mismatch: {max_error}"
    # Verify the visible thickness and fixed furniture as well, not just the
    # collision mid-surfaces. Furniture-furniture joints are intentional.
    scene.frame_set(scene.frame_end)
    deps = bpy.context.evaluated_depsgraph_get()
    deps.update()
    display_trees = {}
    fixture_trees = {}
    for source in scene.objects:
        if source.name not in final and not source.name.startswith(
            ("Chair ", "Table leg", "Table apron", "Cushion ")
        ):
            continue
        obj = source.evaluated_get(deps)
        mesh = obj.to_mesh()
        try:
            mesh.calc_loop_triangles()
            points = [obj.matrix_world @ v.co for v in mesh.vertices]
            faces = [tuple(t.vertices) for t in mesh.loop_triangles]
            tree = BVHTree.FromPolygons(points, faces, all_triangles=True, epsilon=0.0)
            if source.name in final:
                display_trees[source.name] = tree
            else:
                fixture_trees[source.name] = tree
        finally:
            obj.to_mesh_clear()
    display_crossings = []
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            if display_trees[a].overlap(display_trees[b]):
                display_crossings.append([a, b])
        if a in trajectories:
            for b, tree in fixture_trees.items():
                if display_trees[a].overlap(tree):
                    display_crossings.append([a, b])
    assert (
        not display_crossings
    ), f"Rendered thickness/furniture crossings: {display_crossings}"
    log = (directory / "worker.log").read_text(encoding="utf-8", errors="replace")
    clamp = re.search(r"Contact default kappa .*? is clamped to \[([^\]]+)\]", log)
    report = {
        "solver": result,
        "settings": request["settings"],
        "objects": metrics,
        "initial_surface_crossings": initial_crossings,
        "final_surface_crossings": final_crossings,
        "cloth_tabletop_min_vertex_clearance_m": clearance,
        "blender_all_vertex_playback_max_error_m": max_error,
        "checked_surface_crossing_frames": result["frames"],
        "initial_thickness_shell_aabb_clearance_lower_bound_m": initial_clearance,
        "final_sampled_contact_graph": contacts,
        "all_objects_have_contact_path_to_fixed_support": True,
        "final_nonadjacent_cloth_triangle_self_crossings": 0,
        "final_rendered_thickness_and_furniture_crossings": display_crossings,
        "initial_effective_contact_resistance_pa": (
            float(clamp.group(1)) if clamp else request["settings"]["resistance"]
        ),
        "model_limits": [
            "Dimensions and material parameters are assumed, not measured from the photo.",
            "Ceramics and apples use affine-body elasticity, not fracture or fruit tissue FEM.",
            "Furniture and upholstered chair are fixed; their complete meshes enter contact as one compound fixed environment.",
            "One global Coulomb friction coefficient; microscopic cloth weave is shading only.",
            "Surface crossing tests supplement IPC; they are not a continuum accuracy proof.",
        ],
    }
    (output / "physics_validation.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(
        f"VALIDATED {result['frames']} frames, {len(metrics)} dynamic bodies, "
        f"playback error={max_error:.3g} m, cloth RMS speed="
        f"{metrics[CLOTH_NAME]['last_half_second_rms_vertex_speed_m_s']:.3g} m/s",
        flush=True,
    )
    return final, trajectories


def stems(scene, trajectories, addon, output):
    # Append non-colliding details and cache their affine motion at EVERY frame.
    # Never move the simulated apple surface or leave stems at the final pose
    # during earlier frames. This is derived playback, not a second simulation.
    bark = material("Apple stems", (0.12, 0.075, 0.026), 0.8)
    for name, world in trajectories.items():
        obj = scene.objects[name]
        if "stem_radius" not in obj:
            continue
        r = obj["stem_radius"]
        rest = np.array([v.co[:] for v in obj.data.vertices])
        inverse = np.linalg.pinv(np.column_stack((rest, np.ones(len(rest)))))
        path = np.array(
            [(0, 0, 0.865 * r), (0.001, 0, 1.08 * r), (0.003, 0.001, 1.16 * r)]
        )
        stem = scene.objects.get(name + " stem")
        if stem is None:
            stem = tube(name + " stem", path.tolist(), 0.0012, bark, 8)
        if stem.uipc_body.role != "NONE":
            raise ValueError(
                f"{stem.name}: derived decoration must not be a separate simulated body"
            )
        local = np.array([v.co[:] for v in stem.data.vertices])
        local = np.column_stack((local, np.ones(len(local))))
        filename = output / (name.replace(" ", "_") + "_stem.mdd")
        writer = addon.protocol.MDDWriter(
            filename, len(world), len(local), scene.render.fps
        )
        try:
            for points in world:
                writer.append(local @ (inverse @ points))
            writer.close(commit=True)
        finally:
            writer.close()
        mod = stem.modifiers.get("Derived rigid detail playback")
        if mod is None:
            mod = stem.modifiers.new("Derived rigid detail playback", "MESH_CACHE")
        mod.cache_format = "MDD"
        mod.filepath = bpy.path.relpath(str(filename))
        mod.time_mode, mod.play_mode = "FRAME", "SCENE"
        mod.frame_start = scene.frame_start
        mod.forward_axis, mod.up_axis = "POS_Y", "POS_Z"


def activate_cache(scene, addon):
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake))
    request = json.loads((directory / "request.json").read_text())
    # Reattachment is allowed only after the full input signature validates.
    # Setting a physics property on even a disabled decorative object may have
    # invalidated playback through the add-on's intentionally broad callback.
    addon.bridge.attach_cache(scene, directory, request)
    addon.bridge.check_cache(scene)
    for obj in scene.objects:
        if obj.uipc_body.role not in ("NONE", "STATIC"):
            mod = obj.modifiers[addon.protocol.MODIFIER_NAME]
            assert mod.show_render and mod.show_viewport, f"{obj.name}: disabled cache"


def render(scene, output, preview=False):
    prefs = bpy.context.preferences.addons["cycles"].preferences
    try:
        prefs.compute_device_type = "OPTIX"
        prefs.get_devices()
        devices = [d for d in prefs.devices if d.type == "OPTIX"]
        for device in prefs.devices:
            device.use = device in devices
        scene.cycles.device = "GPU" if devices else "CPU"
    except Exception as error:
        print(f"Cycles GPU fallback: {error}", flush=True)
        scene.cycles.device = "CPU"
    scene.cycles.samples = 32 if preview else 192
    scene.render.resolution_percentage = 50 if preview else 100
    scene.render.filepath = str(
        output / ("preview.png" if preview else "white_table_setting.png")
    )
    bpy.ops.wm.save_as_mainfile(filepath=str(output / "white_table_setting.blend"))
    bpy.ops.render.render(write_still=True)


def package(scene, output):
    directory = Path(bpy.path.abspath(scene.uipc_settings.last_bake)).resolve()
    directory.relative_to(output)
    # Blank selects a new local <blend-name>_uipc_cache directory when re-baking;
    # existing playback uses the independent, relative last_bake/MDD paths.
    scene.uipc_settings.cache_directory = ""
    bpy.ops.wm.save_as_mainfile(filepath=str(output / "white_table_setting.blend"))
    members = [
        output / "white_table_setting.blend",
        output / "white_table_setting.png",
        output / "physics_validation.json",
    ]
    members.extend(sorted(output.glob("*_stem.mdd")))
    members.extend(sorted(output.glob("time_step_comparison.json")))
    members.extend(sorted(output.glob("standalone_playback_validation.json")))
    for path in directory.iterdir():
        if path.is_file() and path.suffix in (".mdd", ".json", ".npz", ".log"):
            members.append(path)
    readme = (
        "WHITE TABLE SETTING - libuipc / Blender\n\n"
        "Extract this whole archive before opening white_table_setting.blend.\n"
        "Do not move the blend file away from cache/ and the stem MDD files.\n"
        "The final frame is selected. Frame 1 shows the separated initial state.\n"
        "Spacebar plays the solved motion; F12 renders the selected frame.\n"
        "Playback needs neither CUDA nor the libuipc extension. Re-baking does.\n"
        "Ground is z=0 m. Table and chair are fixed fixtures. All tabletop\n"
        "objects fall in one libuipc World. See physics_validation.json for\n"
        "the checks and model limitations; material values are assumptions.\n"
        "The source script is in integrations/blender/examples/ in libuipc.\n"
    )
    with zipfile.ZipFile(
        output / "white_table_setting_bundle.zip",
        "w",
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=6,
    ) as archive:
        for path in members:
            archive.write(path, path.relative_to(output).as_posix())
        archive.write(Path(__file__).with_name("README.md"), "SCENE_DETAILS.md")
        archive.writestr("OPEN_ME.txt", readme)
    print("PACKAGED " + str(output / "white_table_setting_bundle.zip"), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--python", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--mode",
        choices=("all", "build", "bake", "finish", "render", "validate", "package"),
        default="all",
    )
    parser.add_argument("--frames", type=int, default=961)
    parser.add_argument("--cloth-x", type=int, default=109)
    parser.add_argument("--cloth-y", type=int, default=91)
    parser.add_argument("--substeps", type=int, default=4)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument(
        "--restyle",
        action="store_true",
        help="Apply the scripted studio presentation to an existing scene",
    )
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1 :])
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    addon = importlib.import_module("libuipc_blender")
    if not hasattr(bpy.types.Scene, "uipc_settings"):
        addon.register()
    if args.mode in ("all", "build"):
        scene = build(args)
    else:
        bpy.ops.wm.open_mainfile(
            filepath=str(args.output / "white_table_setting.blend")
        )
        scene = bpy.context.scene
    if args.restyle:
        studio_finish(scene)
    if args.mode in ("all", "bake"):
        started = time.monotonic()
        directory = addon.runtime.start(scene)
        print(f"BAKE_DIRECTORY {directory}", flush=True)
        previous = -1
        while addon.runtime.is_running():
            result = addon.runtime.poll()
            if isinstance(result, dict) and result.get("cancelled"):
                print(
                    "DINING_BAKE_CANCELLED; incomplete cache was not attached",
                    flush=True,
                )
                return
            progress = int(scene.uipc_settings.progress * 100)
            if progress != previous:
                print(
                    f"BAKE {progress}% elapsed={time.monotonic()-started:.1f}s",
                    flush=True,
                )
                previous = progress
            time.sleep(0.1)
        bpy.ops.wm.save_as_mainfile(
            filepath=str(args.output / "white_table_setting.blend")
        )
    if args.mode in ("all", "bake", "finish"):
        activate_cache(scene, addon)
        final, trajectories = verify(scene, addon, args.output)
        stems(scene, trajectories, addon, args.output)
        activate_cache(scene, addon)
        scene.frame_set(scene.frame_end)
        bpy.ops.wm.save_as_mainfile(
            filepath=str(args.output / "white_table_setting.blend")
        )
    if args.mode == "validate":
        activate_cache(scene, addon)
        verify(scene, addon, args.output)
    if args.mode in ("all", "finish", "render"):
        activate_cache(scene, addon)
        render(scene, args.output, args.preview)
    if args.mode == "package":
        activate_cache(scene, addon)
        package(scene, args.output)
    print("DINING_SCENE_COMPLETE", flush=True)


if __name__ == "__main__":
    main()
