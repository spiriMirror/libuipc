# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Native four-point MDD helper + Geometry Nodes, with no Python frame handler."""
import bpy
import numpy as np

from .node_math import geometry_group
from .protocol import MODIFIER_NAME

MARKER = "uipc_affine_cache"


def graph_spec():
    nodes = {"Input": ("NodeGroupInput", {}), "Output": ("NodeGroupOutput", {"is_active_output": True}),
             "Position": ("GeometryNodeInputPosition", {}), "XYZ": ("ShaderNodeSeparateXYZ", {}),
             "Samples": ("GeometryNodeObjectInfo", {"transform_space": "ORIGINAL"}),
             "Apply": ("GeometryNodeSetPosition", {})}
    links = [("Input","Geometry","Apply","Geometry"), ("Apply","Geometry","Output","Geometry"),
             ("Position","Position","XYZ",0)]
    defaults = {("Samples","As Instance"): False, ("Apply","Selection"): True,
                ("Apply","Offset"): (0.,0.,0.)}
    for i in range(4):
        name = f"Sample {i}"
        nodes[name] = ("GeometryNodeSampleIndex", {"data_type": "FLOAT_VECTOR", "domain": "POINT", "clamp": False})
        defaults[(name,"Index")] = i
        links.extend((("Samples","Geometry",name,"Geometry"), ("Position","Position",name,"Value")))
    for axis in range(3):
        scale, add = f"Scale {axis}", f"Add {axis}"
        nodes[scale] = ("ShaderNodeVectorMath", {"operation": "SCALE"})
        nodes[add] = ("ShaderNodeVectorMath", {"operation": "ADD"})
        links.extend(((f"Sample {axis+1}","Value",scale,0), ("XYZ",axis,scale,"Scale"),
                      (scale,"Vector",add,1)))
        links.append(("Sample 0","Value",add,0) if axis == 0 else (f"Add {axis-1}","Vector",add,0))
    links.append(("Add 2","Vector","Apply","Position"))
    return nodes, links, defaults


def is_affine(modifier):
    return bool(modifier and modifier.type == "NODES" and modifier.node_group
                and modifier.node_group.get(MARKER) == 1)


def proxy_object(modifier):
    if not is_affine(modifier):
        raise ValueError("Affine cache node group is missing")
    node = modifier.node_group.nodes.get("Samples")
    if node is None or node.bl_idname != "GeometryNodeObjectInfo":
        raise ValueError("Affine cache sample source is missing")
    proxy = node.inputs["Object"].default_value
    if proxy is None or proxy.type != "MESH" or proxy.get(MARKER) != 1:
        raise ValueError("Affine cache helper is missing or replaced")
    return proxy


def playback_modifier(modifier):
    if modifier.type == "MESH_CACHE":
        return modifier
    proxy = proxy_object(modifier)
    cached = proxy.modifiers.get(MODIFIER_NAME)
    if cached is None or cached.type != "MESH_CACHE":
        raise ValueError("Affine helper's MDD modifier is missing")
    return cached


def _socket_index(sockets, socket):
    return list(sockets).index(socket)


def validate_graph(modifier):
    """Validate against code-owned structure, not a checksum stored in editable RNA."""
    proxy = proxy_object(modifier)
    group = modifier.node_group
    if group.animation_data:
        raise ValueError("Affine cache node animation/drivers are not allowed")
    interface = [(s.in_out,s.name,s.socket_type) for s in group.interface.items_tree if s.item_type == "SOCKET"]
    if sorted(interface) != [("INPUT","Geometry","NodeSocketGeometry"), ("OUTPUT","Geometry","NodeSocketGeometry")]:
        raise ValueError("Affine cache node interface changed")
    nodes, links, defaults = graph_spec()
    if set(group.nodes.keys()) != set(nodes):
        raise ValueError("Affine cache node structure changed")
    for name, (kind, properties) in nodes.items():
        node = group.nodes[name]
        if node.bl_idname != kind or node.mute or any(getattr(node,k) != v for k,v in properties.items()):
            raise ValueError(f"Affine cache node changed: {name}")
    for (name, socket), expected in defaults.items():
        actual = group.nodes[name].inputs[socket].default_value
        if isinstance(expected, tuple):
            actual = tuple(actual)
        if actual != expected:
            raise ValueError(f"Affine cache input changed: {name}/{socket}")
    wanted = {(a, _socket_index(group.nodes[a].outputs, group.nodes[a].outputs[out]),
               b, _socket_index(group.nodes[b].inputs, group.nodes[b].inputs[inp])) for a,out,b,inp in links}
    actual = {(l.from_node.name, _socket_index(l.from_node.outputs,l.from_socket),
               l.to_node.name, _socket_index(l.to_node.inputs,l.to_socket)) for l in group.links}
    if actual != wanted or len(group.links) != len(links) or any(not l.is_valid or l.is_muted for l in group.links):
        raise ValueError("Affine cache node links changed")
    if (proxy.parent or proxy.constraints or proxy.animation_data or proxy.rigid_body
            or proxy.data.shape_keys or proxy.hide_viewport
            or not np.array_equal(np.asarray(proxy.matrix_world), np.eye(4))
            or len(proxy.data.vertices) != 4 or len(proxy.data.edges) or len(proxy.data.polygons)
            or len(proxy.modifiers) != 1 or proxy.uipc_body.role != "NONE"):
        raise ValueError("Affine cache helper geometry/transform changed")
    cached = playback_modifier(modifier)
    if not cached.show_viewport or not cached.show_render:
        raise ValueError("Affine helper cache is disabled")
    return proxy


def validate_binding(modifier, scene, request, index):
    proxy = validate_graph(modifier)
    group = modifier.node_group
    source_id = request["objects"][index]["id"]
    if (proxy.name not in scene.objects
            or group.get("uipc_source_id") != source_id or proxy.get("uipc_source_id") != source_id
            or group.get("uipc_cache_fingerprint") != request["fingerprint"]
            or proxy.get("uipc_cache_fingerprint") != request["fingerprint"]):
        raise ValueError("Affine helper provenance does not match this body/cache")
    return playback_modifier(modifier)


def create(scene, obj, path, request, index, configure):
    mesh = bpy.data.meshes.new("libuipc Affine Samples")
    mesh.from_pydata([(0,0,0)]*4, [], [])
    proxy = bpy.data.objects.new("libuipc Affine Samples: " + obj.name, mesh)
    group, modifier = None, None
    try:
        scene.collection.objects.link(proxy)
        proxy[MARKER] = 1
        proxy["uipc_source_id"] = request["objects"][index]["id"]
        proxy["uipc_cache_fingerprint"] = request["fingerprint"]
        proxy.hide_render, proxy.hide_select = True, True
        proxy.hide_set(True)
        cached = proxy.modifiers.new(MODIFIER_NAME, "MESH_CACHE")
        configure(cached, path, request)
        group, _source, _sink = geometry_group("libuipc Affine Playback: " + obj.name)
        group[MARKER] = 1
        group["uipc_source_id"] = proxy["uipc_source_id"]
        group["uipc_cache_fingerprint"] = request["fingerprint"]
        nodes, links, defaults = graph_spec()
        for name, (kind, properties) in nodes.items():
            node = group.nodes.get(name) or group.nodes.new(kind)
            node.name = name
            for key,value in properties.items():
                setattr(node,key,value)
        for (name,socket),value in defaults.items():
            group.nodes[name].inputs[socket].default_value = value
        group.nodes["Samples"].inputs["Object"].default_value = proxy
        for a,out,b,inp in links:
            group.links.new(group.nodes[a].outputs[out], group.nodes[b].inputs[inp])
        modifier = obj.modifiers.new("libuipc Pending Cache", "NODES")
        modifier.node_group = group
        modifier.show_viewport = modifier.show_render = False
        return modifier
    except Exception:
        if modifier:
            obj.modifiers.remove(modifier)
        if group and group.users == 0:
            bpy.data.node_groups.remove(group)
        bpy.data.objects.remove(proxy, do_unlink=True)
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
        raise


def remove(obj, modifier):
    """Remove only generated, unshared helpers; keep anything explicitly reused."""
    group, proxy = None, None
    if is_affine(modifier):
        group = modifier.node_group
        try:
            proxy = proxy_object(modifier)
        except ValueError:
            pass
    obj.modifiers.remove(modifier)
    if group and group.users == 0:
        bpy.data.node_groups.remove(group)
    if proxy and proxy.users == 1 and len(proxy.users_collection) == 1 and proxy.get(MARKER) == 1:
        mesh = proxy.data
        bpy.data.objects.remove(proxy, do_unlink=True)
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
