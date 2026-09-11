# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from affine import pack_affine, affine_positions, cache_vertices
from protocol import validate_result


class AffineTests(unittest.TestCase):
    def test_full_affine_and_linear_interpolation(self):
        random = np.random.default_rng(482)
        points = random.normal(size=(113,3))
        first, second = np.eye(4), np.eye(4)
        first[:3,:] = random.normal(size=(3,4))
        second[:3,:] = random.normal(size=(3,4))
        for alpha in (0, .15, .5, 1):
            samples = pack_affine(first)*(1-alpha) + pack_affine(second)*alpha
            expected = points @ ((1-alpha)*first[:3,:3]+alpha*second[:3,:3]).T
            expected += (1-alpha)*first[:3,3] + alpha*second[:3,3]
            np.testing.assert_allclose(affine_positions(samples, points), expected, atol=1e-14)

    def test_encoding_requires_rigid_body_and_new_schema(self):
        request = {"schema_version": 8, "fingerprint": "test", "settings": {"frame_start": 7,"frame_end": 9},
                   "objects": [{"material": {"role": "RIGID"}}]}
        result = {"schema_version": 8,"fingerprint": "test","frames": 3,
                  "objects": [{"index": 0,"vertices": 1089,"encoding": "AFFINE"}]}
        self.assertEqual(validate_result(request,result,[1089]), 3)
        self.assertEqual(cache_vertices(result["objects"][0]), 4)
        for role in ("CLOTH","FEM","ROD"):
            invalid = copy.deepcopy(request)
            invalid["objects"][0]["material"]["role"] = role
            with self.assertRaises(ValueError):
                validate_result(invalid,result,[1089])
        request["schema_version"] = result["schema_version"] = 7
        with self.assertRaises(ValueError):
            validate_result(request,result,[1089])
        result["objects"][0]["encoding"] = "VERTEX"
        self.assertEqual(validate_result(request,result,[1089]), 3)

    def test_invalid_affine_payload(self):
        for matrix in (np.eye(3), np.full((4,4),np.nan), np.zeros((4,4))):
            with self.assertRaises(ValueError):
                pack_affine(matrix)
        with self.assertRaises(ValueError):
            affine_positions(np.zeros((3,3)), np.eye(3))
