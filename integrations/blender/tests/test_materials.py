# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Independent cloth parameters and legacy-cache compatibility."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from materials import (cloth_moduli, cloth_stiffness, build_contact_plan,
                       normalize_contact_pairs, validate_preset, PRESET_FIELDS)
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

    def test_explicit_channels_do_not_require_a_shared_ratio(self):
        material = self.material()
        expected = cloth_stiffness(material)
        material.pop("poisson")
        self.assertEqual(cloth_stiffness(material), expected)

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

    def pair(self, a="Apple", b="Fabric", friction=.7, enabled=True):
        return dict(material_a=a, material_b=b, friction=friction, resistance=1e8, enabled=enabled)

    def body(self, name, label="", drive=None):
        material = {"role": "RIGID", "contact_material": label}
        if drive:
            material["drive"] = drive
        return {"name": name, "material": material}

    def test_contact_pairs_are_symmetric_and_reject_typos(self):
        self.assertEqual(normalize_contact_pairs([self.pair()]),
                         normalize_contact_pairs([self.pair("Fabric", "Apple")]))
        for pairs in ([self.pair(), self.pair("Fabric", "Apple")],
                      [self.pair(friction=-1)], [self.pair(enabled=1)],
                      [{**self.pair(), "resistance": float("nan")}], [self.pair("Typo")]):
            with self.assertRaises(ValueError):
                normalize_contact_pairs(pairs, {"Apple", "Fabric"})

    def test_contact_override_and_assembly_exclusion_precedence(self):
        drive = {"group": "hand", "friction": .8}
        bodies = [self.body("apple", "Apple"), self.body("cloth", "Fabric"),
                  self.body("finger1", "Rubber", drive), self.body("finger2", "Metal", drive)]
        settings = {"friction": .5, "resistance": 1e9, "contact_pairs": [
            self.pair(), self.pair("Rubber", "Apple", .9), self.pair("Rubber", "Metal", 2)]}
        plan = build_contact_plan(settings, bodies)
        models = {(m["a"], m["b"]): m for m in plan["models"]}
        pair = lambda a, b: models[tuple(sorted((plan["assignments"][a], plan["assignments"][b])))]
        self.assertEqual(pair(0, 1)["friction"], .7)
        self.assertEqual(pair(0, 2)["friction"], .9)
        self.assertEqual(pair(1, 2)["friction"], .8)
        self.assertFalse(pair(2, 3)["enabled"])

    def test_no_overrides_preserves_global_and_robot_defaults(self):
        bodies = [self.body("ground"), self.body("apple"),
                  self.body("robot", drive={"group": "hand", "friction": .8})]
        plan = build_contact_plan({"friction": .5, "resistance": 1e9}, bodies)
        self.assertEqual(plan["assignments"], [0, 0, 1])
        self.assertEqual(plan["models"][0]["friction"], .5)
        self.assertEqual(plan["models"][1]["friction"], .8)
        self.assertFalse(plan["models"][2]["enabled"])

    def test_preset_rejects_nonphysical_fields_and_invalid_values(self):
        material = {**self.material(), "self_collision": True}
        payload = {"schema_version": 1, "role": "CLOTH",
                   "values": {k: material[k] for k in PRESET_FIELDS["CLOTH"]}}
        self.assertEqual(validate_preset(payload), payload)
        for key, value in (("fixed", True), ("shear_poisson", .5), ("density", -1), ("stretch", float("nan"))):
            with self.assertRaises(ValueError):
                validate_preset({**payload, "values": {**payload["values"], key: value}})


if __name__ == "__main__":
    unittest.main()
