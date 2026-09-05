# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Export native UrdfIO collision meshes and joint frames for Blender controls."""

import json
from collections import defaultdict
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def clean_collision_assembly(vertices, triangles):
    """Remove isolated zero-volume export artifacts, retaining solid components."""
    edges = defaultdict(list)
    for i, tri in enumerate(triangles):
        for a, b in zip(tri, np.roll(tri, -1)):
            edges[tuple(sorted((int(a), int(b))))].append(i)
    adjacent = [set() for _ in triangles]
    for incident in edges.values():
        for i in incident:
            adjacent[i].update(incident)
    pending = set(range(len(triangles)))
    keep = []
    removed = 0
    tolerance = float(np.linalg.norm(np.ptp(vertices, axis=0)) ** 3) * 1e-12
    while pending:
        queue = [pending.pop()]
        component = []
        while queue:
            i = queue.pop()
            component.append(i)
            more = adjacent[i] & pending
            pending.difference_update(more)
            queue.extend(more)
        cells = triangles[component]
        p = vertices[cells] - vertices[np.unique(cells)].mean(axis=0)
        volume = float(
            np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6
        )
        if abs(volume) <= tolerance:
            removed += 1
        else:
            keep.extend(component)
    triangles = triangles[sorted(keep)]
    if not len(triangles):
        raise ValueError("URDF collision link has no volumetric component")
    _, unique = np.unique(np.sort(triangles, axis=1), axis=0, return_index=True)
    duplicates = len(triangles) - len(unique)
    triangles = triangles[np.sort(unique)]
    used = np.unique(triangles)
    remap = np.full(len(vertices), -1, dtype=np.int32)
    remap[used] = np.arange(len(used))
    return vertices[used], remap[triangles], removed, duplicates, used


def export_robot(source, directory):
    import uipc
    from uipc.geometry import UrdfIO
    from protocol import validate_mesh

    source, directory = Path(source).resolve(), Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    document = ET.parse(source).getroot()
    if document.tag != "robot":
        raise ValueError("Expected a URDF robot document")
    joints = []
    for joint in document.findall("joint"):
        if joint.get("type") not in ("fixed", "revolute"):
            raise ValueError(f"Unsupported URDF joint type: {joint.get('type')}")
        origin, axis, limits = (
            joint.find("origin"),
            joint.find("axis"),
            joint.find("limit"),
        )
        vector = lambda element, key, default: [
            float(v)
            for v in (
                element.get(key, default) if element is not None else default
            ).split()
        ]
        joints.append(
            {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "xyz": vector(origin, "xyz", "0 0 0"),
                "rpy": vector(origin, "rpy", "0 0 0"),
                "axis": vector(axis, "xyz", "1 0 0"),
                "limits": (
                    [float(limits.get("lower")), float(limits.get("upper"))]
                    if limits is not None
                    else None
                ),
            }
        )
    names = {link.get("name") for link in document.findall("link")}
    roots = names - {joint["child"] for joint in joints}
    if len(roots) != 1:
        raise ValueError("Expected one URDF root")
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    scene = uipc.Scene()
    obj = scene.objects().create("URDF export")
    cfg = UrdfIO.default_config()
    cfg["load_visual_mesh"] = False
    controller = UrdfIO(cfg).read(obj, str(source))
    # Set the URDF link frame explicitly; avoid the loader's initial root mesh
    # origin also appearing in the root pose. Geometry already includes origin.
    controller.move_root(np.zeros(3), np.zeros(3))
    controller.apply_to(uipc.builtin.transform)
    basis = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], dtype=float)
    inverse_basis = np.eye(4)
    inverse_basis[:3, :3] = basis.T
    links = []
    for index, slot in enumerate(controller.links()):
        geometry = slot.geometry()
        name = geometry.meta().find("urdf/name").view()[0]
        points = np.array(geometry.positions().view()).reshape(-1, 3)
        triangles = np.array(geometry.triangles().topo().view()).reshape(-1, 3)
        points, triangles, removed, duplicates, source_ids = clean_collision_assembly(
            points, triangles
        )
        # Some sample 87 collision files repeat an identical facet. Keep its
        # original surface once; retain disconnected closed assembly components.
        # Blender completes any remaining boundary loops before assigning ABD.
        # Check finite values, indices, winding and manifold edges here already.
        points, triangles = validate_mesh(
            points, triangles, "CLOTH", name, allow_components=True
        )
        pose = (
            inverse_basis @ np.array(geometry.transforms().view()).reshape(-1, 4, 4)[0]
        )
        np.savez(
            directory / f"link_{index:03d}.npz",
            vertices=points,
            triangles=triangles,
            pose=pose,
            source_vertex_indices=source_ids,
        )
        links.append(
            {
                "name": name,
                "index": index,
                "vertices": len(points),
                "triangles": len(triangles),
                "removed_duplicate_triangles": duplicates,
                "removed_zero_volume_components": removed,
                "local_min": points.min(axis=0).tolist(),
                "local_max": points.max(axis=0).tolist(),
                "zero_pose": pose.tolist(),
            }
        )
    # Record an independent native FK pose so Blender's hierarchy conversion
    # can be checked against UrdfController, including axis signs and origins.
    angles = {}
    for index, joint in enumerate(joints):
        if joint["type"] != "revolute":
            continue
        low, high = joint["limits"] or [-1, 1]
        angle = low + (0.25 + 0.025 * (index % 7)) * (high - low)
        angles[joint["name"]] = angle
        controller.rotate_to(joint["name"], angle)
    controller.apply_to(uipc.builtin.transform)
    reference = {
        slot.geometry()
        .meta()
        .find("urdf/name")
        .view()[0]: (
            inverse_basis
            @ np.array(slot.geometry().transforms().view()).reshape(-1, 4, 4)[0]
        )
        .tolist()
        for slot in controller.links()
    }
    result = {
        "source": str(source),
        "root": next(iter(roots)),
        "joints": joints,
        "links": links,
        "fk_test_angles": angles,
        "fk_test_poses": reference,
        "build_info": uipc.build_info(),
    }
    (directory / "robot.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = export_robot(args.urdf, args.output)
    print(
        json.dumps(
            {
                "root": result["root"],
                "links": len(result["links"]),
                "duplicates_removed": {
                    l["name"]: l["removed_duplicate_triangles"]
                    for l in result["links"]
                    if l["removed_duplicate_triangles"]
                },
            },
            indent=2,
        )
    )
