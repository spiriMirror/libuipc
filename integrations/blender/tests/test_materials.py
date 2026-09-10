# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Independent cloth parameters and legacy-cache compatibility."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from materials import cloth_moduli, cloth_stiffness
import protocol
import worker


class MaterialTests(unittest.TestCase):
    def material(self):
        return dict(role="CLOTH", stretch=1e4, shear=10, bending=1e4,
                    poisson=0.49, stretch_poisson=0.4, shear_poisson=0.45,
                    bending_poisson=0.4, thickness=0.0005, density=200, strain_rate=100)

    def test_independent_values_reach_each_constitution(self):
        shell, bending, moduli = Mock(), Mock(), Mock()
        moduli.youngs_poisson.side_effect = lambda e, nu: (e, nu)
        mesh = object()
        worker.apply_cloth_material(mesh, self.material(), shell, bending, moduli)
        self.assertEqual(shell.apply_to.call_args.kwargs["stretch_moduli"], (1e4, 0.4))
        self.assertEqual(shell.apply_to.call_args.kwargs["shear_moduli"], (10, 0.45))
        bending.apply_to.assert_called_once_with(mesh, 1e4, 0.4)

    def test_coefficients_and_independent_changes(self):
        material = self.material()
        values = cloth_stiffness(material)
        self.assertAlmostEqual(values["stretch"], 1e4 * 0.001 / 0.84)
        self.assertAlmostEqual(values["shear"], 10 / 2.9)
        self.assertAlmostEqual(values["bending"], 1e4 * 0.001**3 / (12 * 0.84))
        for channel in values:
            changed = cloth_stiffness({**material, channel + "_poisson": 0.2})
            self.assertNotEqual(changed[channel], values[channel])
            for other in values.keys() - {channel}:
                self.assertEqual(changed[other], values[other])

    def test_legacy_defaults_and_invalid_parameters(self):
        old = {k: v for k, v in self.material().items() if not k.endswith("_poisson")}
        self.assertTrue(all(nu == old["poisson"] for e, nu in cloth_moduli(old).values()))
        for channel in ("stretch", "shear", "bending"):
            for invalid in (-0.01, 0.5, float("nan")):
                with self.assertRaises(ValueError):
                    cloth_moduli({**old, channel + "_poisson": invalid})

    def test_old_cache_accepts_equivalent_but_not_independent_changes(self):
        material = {k: v for k, v in self.material().items() if not k.endswith("_poisson")}
        body = {"name": "cloth", "material": material, "vertices": np.eye(3),
                "triangles": np.array([[0, 1, 2]]), "tetrahedra": np.empty((0, 4)),
                "pins": np.array([]), "matrix": np.eye(4)}
        for schema in (2, 3):
            expected = protocol.fingerprint({}, [body], schema)
            current = {**body, "material": {**material, **{k + "_poisson": 0.49 for k in ("stretch", "shear", "bending")}}}
            request = {"schema_version": schema, "objects": [{"material": material}]}
            self.assertEqual(protocol.cache_fingerprint(request, {}, [current]), expected)
            current["material"]["shear_poisson"] = 0.45
            self.assertIsNone(protocol.cache_fingerprint(request, {}, [current]))


if __name__ == "__main__":
    unittest.main()
