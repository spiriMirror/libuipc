# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""Native Geometry Nodes position transforms, including scale and shear."""


def geometry_group(name):
    import bpy
    group = bpy.data.node_groups.new(name, "GeometryNodeTree")
    group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    source, sink = group.nodes.new("NodeGroupInput"), group.nodes.new("NodeGroupOutput")
    source.name, sink.name = "Input", "Output"
    return group, source.outputs["Geometry"], sink.inputs["Geometry"]


def transform_positions(group, geometry, matrix, prefix):
    """x' = A x + t, with twelve directly addressable scalar coefficients."""
    nodes, links = group.nodes, group.links
    position = nodes.new("GeometryNodeInputPosition")
    position.name = prefix + " Position"
    combine = nodes.new("ShaderNodeCombineXYZ")
    combine.name = prefix + " Rows"
    sockets = []
    for row in range(3):
        dot = nodes.new("ShaderNodeVectorMath")
        dot.name, dot.operation = f"{prefix} Row {row}", "DOT_PRODUCT"
        dot.inputs[1].default_value = matrix[row][:3]
        links.new(position.outputs["Position"], dot.inputs[0])
        links.new(dot.outputs["Value"], combine.inputs[row])
        sockets.append(dot.inputs[1])
    translate = nodes.new("ShaderNodeVectorMath")
    translate.name, translate.operation = prefix + " Translation", "ADD"
    translate.inputs[1].default_value = [matrix[row][3] for row in range(3)]
    links.new(combine.outputs[0], translate.inputs[0])
    sockets.append(translate.inputs[1])
    apply = nodes.new("GeometryNodeSetPosition")
    apply.name = prefix + " Apply"
    links.new(geometry, apply.inputs["Geometry"])
    links.new(translate.outputs["Vector"], apply.inputs["Position"])
    return apply.outputs["Geometry"], sockets
