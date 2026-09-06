# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Solver profile regression without Blender or a native CUDA runtime."""

import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
import worker


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


if __name__ == "__main__":
    unittest.main()
