# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Solver profile regression without Blender or a native CUDA runtime."""

import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
import worker
from protocol import validate_solver_settings


class SolverSettingsTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "newton": {
                "semi_implicit": {"enable": 1, "K_min": 6},
                "velocity_tol": 0.05,
                "velocity_tol_relative": 0,
            },
            "linear_system": {"tol_rate": 1e-3, "solver": "fused_pcg"},
            "line_search": {"max_iter": 8},
            "dt": 1 / 120,
            "contact": {"constitution": "ipc", "d_hat": 0.0008},
        }

    def test_default_keeps_every_native_setting(self):
        expected = copy.deepcopy(self.config)
        worker.apply_solver_accuracy(self.config, "DEFAULT")
        self.assertEqual(self.config, expected)

    def test_converged_changes_accuracy_not_physics(self):
        original = copy.deepcopy(self.config)
        worker.apply_solver_accuracy(self.config, "CONVERGED")
        self.assertEqual(self.config["newton"]["semi_implicit"]["enable"], 0)
        self.assertEqual(self.config["newton"]["semi_implicit"]["K_min"], 6)
        self.assertEqual(self.config["newton"]["velocity_tol"], 0.001)
        self.assertEqual(self.config["newton"]["velocity_tol_relative"], 0)
        self.assertEqual(self.config["linear_system"]["tol_rate"], 1e-6)
        self.assertEqual(self.config["line_search"]["max_iter"], 32)
        self.assertEqual(self.config["dt"], original["dt"])
        self.assertEqual(self.config["contact"], original["contact"])
        self.assertEqual(self.config["linear_system"]["solver"], "fused_pcg")

    def test_unknown_profile_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unsupported solver accuracy"):
            worker.apply_solver_accuracy(self.config, "TYPO")

    def custom(self):
        return {
            "linear_tolerance": "1e-8",
            "velocity_tolerance": "0.002",
            "relative_velocity_tolerance": "2e-4",
            "transrate_tolerance": "0.2",
            "semi_implicit": True,
            "k_min": 12,
            "beta_tolerance": "5e-4",
            "newton_max_iter": 99,
            "newton_min_iter": 4,
            "line_search_max_iter": 16,
        }

    def test_custom_values_reach_all_native_keys(self):
        worker.apply_solver_accuracy(self.config, "CUSTOM", self.custom())
        self.assertEqual(self.config["linear_system"]["tol_rate"], 1e-8)
        newton = self.config["newton"]
        self.assertEqual(newton["velocity_tol"], 0.002)
        self.assertEqual(newton["velocity_tol_relative"], 2e-4)
        self.assertEqual(newton["transrate_tol"], 0.2)
        self.assertEqual(
            newton["semi_implicit"], {"enable": 1, "K_min": 12, "beta_tol": 5e-4}
        )
        self.assertEqual((newton["min_iter"], newton["max_iter"]), (4, 99))
        self.assertEqual(self.config["line_search"]["max_iter"], 16)

    def test_text_notation_does_not_change_physics_fingerprint_inputs(self):
        a = self.custom()
        b = {**a, "linear_tolerance": "0.00000001"}
        self.assertEqual(validate_solver_settings(a), validate_solver_settings(b))

    def test_custom_invalid_values_and_inconsistent_limits_rejected(self):
        for key, value in (
            ("linear_tolerance", "nan"),
            ("linear_tolerance", "1"),
            ("linear_tolerance", "0"),
            ("velocity_tolerance", "bad"),
            ("relative_velocity_tolerance", "-1"),
            ("beta_tolerance", "1.1"),
            ("transrate_tolerance", "inf"),
            ("semi_implicit", 1),
            ("k_min", -1),
            ("newton_max_iter", 2.5),
            ("newton_min_iter", 100),
            ("line_search_max_iter", 129),
        ):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_solver_settings({**self.custom(), key: value})

    def test_custom_cannot_be_missing_or_silently_ignored(self):
        with self.assertRaises(ValueError):
            worker.apply_solver_accuracy(self.config, "CUSTOM")
        with self.assertRaises(ValueError):
            worker.apply_solver_accuracy(self.config, "DEFAULT", self.custom())
        with self.assertRaises(ValueError):
            validate_solver_settings({**self.custom(), "typo": 1})


if __name__ == "__main__":
    unittest.main()
