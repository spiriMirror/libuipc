# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
import copy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from rod import rod_stiffness, validate_linemesh
from protocol import fingerprint, fully_fixed
from materials import validate_preset


class RodTests(unittest.TestCase):
    def test_volume_preparation_signatures_do_not_require_a_simulation_role(self):
        body = {"name": "Surface", "material": {"preserve_surface": True},
                "vertices": np.eye(3), "triangles": [[0,1,2]], "matrix": np.eye(4), "pins": []}
        self.assertEqual(fingerprint({}, [body], 6), fingerprint({}, [body], 8))

    def test_native_section_coefficients_and_preset(self):
        material = {"rod_stretch": 4e4, "rod_bending": 1e5, "thickness": .005, "density": 200, "self_collision": True}
        values = rod_stiffness(material)
        self.assertAlmostEqual(values["axial_rigidity"], np.pi)
        self.assertAlmostEqual(values["bending_rigidity"], 1e5*np.pi*.005**4/4)
        self.assertAlmostEqual(values["linear_density"], 200*np.pi*.005**2)
        material["rod_bending"] = 0
        validate_preset({"schema_version": 1, "role": "ROD", "values": material})
        self.assertEqual(rod_stiffness(material)["bending_rigidity"], 0)

    def test_edge_domain_and_backtracking(self):
        points = [[0,0,0],[1,0,0],[2,0,0]]
        validate_linemesh(points, [[0,1],[1,2]])
        for edges in ([[0,0]], [[0,1],[1,0]], [[0,1]], [[0,1],[1,3]], [[0.,1.],[1.,2.]]):
            with self.assertRaises(ValueError):
                validate_linemesh(points, edges)
        with self.assertRaises(ValueError):
            validate_linemesh([[0,0,0],[1,0,0],[.5,0,0]], [[0,1],[1,2]])
        with self.assertRaises(ValueError):
            validate_linemesh([[0,0,0],[1,0,0],[0,1,0],[0,0,1]], [[0,1],[0,2],[0,3]])
        validate_linemesh([[0,0,0],[1,0,0],[1,1,0],[0,1,0]], [[0,1],[1,2],[2,3],[3,0]])

    def test_edges_and_parameters_are_in_physical_signature(self):
        body = {"id": "rod", "name": "Rod", "material": {"role": "ROD", "rod_bending": 1e5},
                "vertices": np.eye(4,3), "edges": [[0,1],[1,2],[2,3]], "triangles": [],
                "tetrahedra": [], "pins": [0,1,2,3], "matrix": np.eye(4)}
        self.assertTrue(fully_fixed(body))
        original = fingerprint({}, [body])
        other = copy.deepcopy(body)
        other["edges"] = [[0,2],[2,1],[1,3]]
        self.assertNotEqual(original, fingerprint({}, [other]))
        other = copy.deepcopy(body)
        other["material"]["rod_bending"] *= 2
        self.assertNotEqual(original, fingerprint({}, [other]))
