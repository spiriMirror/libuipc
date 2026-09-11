# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Four MDD vectors encode all twelve affine coefficients, without TRS loss."""
import numpy as np


def pack_affine(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if (matrix.shape != (4,4) or not np.isfinite(matrix).all()
            or max(abs(matrix[3,0]), abs(matrix[3,1]), abs(matrix[3,2]), abs(matrix[3,3]-1)) > 1e-12):
        raise ValueError("Expected a finite affine 4x4 matrix")
    samples = np.empty((4,3), dtype=np.float64)
    samples[0], samples[1:] = matrix[:3,3], matrix[:3,:3].T
    return samples


def affine_positions(samples, vertices):
    samples, vertices = np.asarray(samples), np.asarray(vertices)
    if (samples.shape != (4,3) or not np.isfinite(samples).all()
            or vertices.ndim != 2 or vertices.shape[1] != 3):
        raise ValueError("Invalid affine samples or source vertices")
    return vertices @ samples[1:] + samples[0]


def cache_vertices(output):
    return 4 if output.get("encoding", "VERTEX") == "AFFINE" else output["vertices"]
