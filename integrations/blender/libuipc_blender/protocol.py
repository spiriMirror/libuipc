# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Versioned file protocol shared by Blender and the external Python worker.

No bpy or uipc imports: this module runs in both Python interpreters.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import struct
import time

import numpy as np

SCHEMA_VERSION = 1
MODIFIER_NAME = "libuipc Cache"
OBJECT_FIELDS = (
    "role", "density", "thickness", "stretch", "shear", "bending",
    "poisson", "strain_rate", "rigidity", "self_collision", "pin_group",
    "pin_threshold",
)


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    # Windows readers and antivirus can briefly deny FILE_SHARE_DELETE. Never
    # truncate the live status file; retry atomic replacement after readers close.
    for attempt in range(100):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 99:
                raise
            time.sleep(0.01)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def fingerprint(settings, bodies):
    """Include vertex values/order, topology, pins, transforms, and all physics inputs."""
    digest = hashlib.sha256()
    digest.update(json.dumps(settings, sort_keys=True, allow_nan=False).encode())
    for body in bodies:
        digest.update(json.dumps(body["material"], sort_keys=True, allow_nan=False).encode())
        digest.update(body["name"].encode("utf-8"))
        for key, dtype in (("vertices", "<f8"), ("triangles", "<i4"),
                           ("matrix", "<f8"), ("pins", "<i4")):
            data = np.ascontiguousarray(body[key], dtype=dtype)
            digest.update(str(data.shape).encode())
            digest.update(data.tobytes())
    return digest.hexdigest()


def positive(value, name, allow_zero=False):
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError(f"{name} must be finite and {'non-negative' if allow_zero else 'positive'}")


def validate_mesh(vertices, triangles, role, name):
    """Validate input before any native call; preserve the original vertex indexing."""
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int32)
    prefix = f"{name}: "
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3:
        raise ValueError(prefix + "expected at least three vertices")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles):
        raise ValueError(prefix + "expected triangulated faces")
    if not np.isfinite(vertices).all():
        raise ValueError(prefix + "non-finite vertex coordinates")
    if triangles.min() < 0 or triangles.max() >= len(vertices):
        raise ValueError(prefix + "triangle index out of range")
    if len(np.unique(triangles)) != len(vertices):
        raise ValueError(prefix + "loose vertices/edges are unsupported; remove them first")
    canonical = np.sort(triangles, axis=1)
    if len(np.unique(canonical, axis=0)) != len(triangles):
        raise ValueError(prefix + "duplicate triangles")
    p = vertices[triangles]
    cross = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    extent = float(np.max(np.ptp(vertices, axis=0)))
    if extent <= 0 or np.any(np.linalg.norm(cross, axis=1) <= extent**2 * 1e-12):
        raise ValueError(prefix + "degenerate triangles; clean the mesh first")
    edges = {}
    adjacency = [[] for _ in triangles]
    for face, tri in enumerate(triangles):
        for a, b in zip(tri, np.roll(tri, -1)):
            edge = tuple(sorted((int(a), int(b))))
            edges.setdefault(edge, []).append((face, int(a) < int(b)))
    for incident in edges.values():
        if len(incident) > 2:
            raise ValueError(prefix + "non-manifold edge")
        if len(incident) == 2:
            (a, direction_a), (b, direction_b) = incident
            if direction_a == direction_b:
                raise ValueError(prefix + "inconsistent face winding; recalculate normals")
            adjacency[a].append(b)
            adjacency[b].append(a)
        elif role == "RIGID":
            raise ValueError(prefix + "ABD rigid bodies require a closed surface")
    if role == "RIGID":
        seen, pending = set(), [0]
        while pending:
            face = pending.pop()
            if face not in seen:
                seen.add(face)
                pending.extend(adjacency[face])
        if len(seen) != len(triangles):
            raise ValueError(prefix + "use one connected closed surface per ABD body")
        # Center before integrating to avoid cancellation for translated meshes.
        centered = vertices - vertices.mean(axis=0)
        p = centered[triangles]
        volume = float(np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum() / 6)
        if abs(volume) <= extent**3 * 1e-12:
            raise ValueError(prefix + "ABD surface encloses no usable volume")
        if volume < 0:
            triangles = triangles[:, [0, 2, 1]].copy()
    return np.ascontiguousarray(vertices), np.ascontiguousarray(triangles)


class MDDWriter:
    """Stream one frame at a time; never allocate frames x vertices in memory."""

    def __init__(self, path, frame_count, vertex_count, fps):
        self.path = Path(path)
        self.temporary = self.path.with_suffix(".mdd.partial")
        self.frame_count, self.vertex_count = frame_count, vertex_count
        self.written = 0
        self.file = self.temporary.open("xb")
        self.file.write(struct.pack(">ii", frame_count, vertex_count))
        self.file.write((np.arange(frame_count, dtype=np.float64) / fps).astype(">f4").tobytes())

    def append(self, positions):
        positions = np.asarray(positions)
        if positions.shape != (self.vertex_count, 3) or not np.isfinite(positions).all():
            raise ValueError("Invalid or non-finite simulation output")
        with np.errstate(over="raise", invalid="raise"):
            data = positions.astype(">f4")
        self.file.write(data.tobytes())
        self.written += 1

    def close(self, commit=False):
        if not self.file.closed:
            self.file.close()
        if commit:
            if self.written != self.frame_count:
                raise ValueError("Incomplete MDD cache")
            os.replace(self.temporary, self.path)


def inspect_mdd(path, frames, vertices):
    path = Path(path)
    with path.open("rb") as stream:
        header = stream.read(8)
    if len(header) != 8 or struct.unpack(">ii", header) != (frames, vertices):
        raise ValueError(f"Invalid cache header: {path.name}")
    if path.stat().st_size != 8 + 4 * frames + 12 * frames * vertices:
        raise ValueError(f"Truncated cache: {path.name}")
