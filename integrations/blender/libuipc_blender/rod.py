# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Portable centerline validation and existing native rod coefficient mapping."""
import math

import numpy as np

ROD_FIELDS = ("rod_stretch", "rod_bending")


def rod_stiffness(material):
    values = {name: float(material[name]) for name in (*ROD_FIELDS, "thickness", "density")}
    for name, value in values.items():
        if not math.isfinite(value) or value < 0 or (name != "rod_bending" and value == 0):
            raise ValueError(f"Rod {name} must be positive and finite (bending may be zero)")
    radius = values["thickness"]
    area, moment = math.pi * radius**2, math.pi * radius**4 / 4
    return {"area": area, "second_moment": moment,
            "axial_rigidity": values["rod_stretch"] * area,
            "bending_rigidity": values["rod_bending"] * moment,
            "linear_density": values["density"] * area}


def validate_linemesh(vertices, edges, name="Rod"):
    vertices = np.asarray(vertices, dtype=np.float64)
    source_edges = np.asarray(edges)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 2
            or not np.isfinite(vertices).all()):
        raise ValueError(f"{name}: expected at least two finite 3D vertices")
    if (source_edges.ndim != 2 or source_edges.shape[1] != 2 or not len(source_edges)
            or not np.issubdtype(source_edges.dtype, np.integer)):
        raise ValueError(f"{name}: expected integer edge pairs")
    if source_edges.min() < 0 or source_edges.max() >= len(vertices):
        raise ValueError(f"{name}: edge index out of range")
    edges = np.ascontiguousarray(source_edges, dtype=np.int32)
    canonical = np.sort(edges, axis=1)
    if np.any(canonical[:,0] == canonical[:,1]) or len(np.unique(canonical, axis=0)) != len(edges):
        raise ValueError(f"{name}: repeated vertex or duplicate edge")
    degree = np.bincount(edges.ravel(), minlength=len(vertices))
    if np.any(degree == 0) or np.any(degree > 2):
        raise ValueError(f"{name}: use unbranched open/closed centerlines without isolated vertices")
    lengths = np.linalg.norm(vertices[edges[:,1]] - vertices[edges[:,0]], axis=1)
    scale = float(np.linalg.norm(np.ptp(vertices, axis=0)))
    if np.any(lengths <= max(scale * 1e-12, np.finfo(float).tiny)):
        raise ValueError(f"{name}: zero or near-zero edge length")
    neighbors = [[] for _ in vertices]
    for a,b in edges:
        neighbors[a].append(b)
        neighbors[b].append(a)
    for center, adjacent in enumerate(neighbors):
        if len(adjacent) == 2:
            a,b = vertices[adjacent] - vertices[center]
            if np.dot(a,b) >= (1 - 1e-12) * np.linalg.norm(a) * np.linalg.norm(b):
                raise ValueError(f"{name}: overlapping/backtracking adjacent edges make rod bending singular")
    return np.ascontiguousarray(vertices), edges
