# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Pose layouts, joint limits and output-independent trajectory review math."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from robot_control_state import layout_signature, validate_pose, trajectory_metrics


class RobotControlTests(unittest.TestCase):
    def joints(self):
        return [{"name": "hinge", "kind": "revolute", "limits": [-.5, .5], "axis": [0, 0, 1]},
                {"name": "fixed", "kind": "fixed", "limits": None, "axis": [1, 0, 0]}]

    def test_pose_covers_only_movable_joints_and_checks_layout(self):
        joints = self.joints()
        pose = {"schema_version": 1, "layout": layout_signature(joints), "angles": {"hinge": .2}}
        self.assertEqual(validate_pose(pose, joints), {"hinge": .2})
        for angles in ({}, {"hinge": 1}, {"hinge": float("nan")}, {"hinge": .1, "fixed": 1}):
            with self.assertRaises(ValueError):
                validate_pose({**pose, "angles": angles}, joints)
        joints[0]["axis"] = [1, 0, 0]
        with self.assertRaises(ValueError):
            validate_pose(pose, joints)

    def test_joint_rates_use_fps_and_substeps_and_keep_fractional_frames(self):
        metrics = trajectory_metrics([0, math.radians(1), math.radians(3)], 10, 2, 7, [-1, 1])
        self.assertAlmostEqual(metrics["max_speed_deg_s"], 40)
        self.assertAlmostEqual(metrics["max_acceleration_deg_s2"], 400)
        self.assertEqual(metrics["speed_frame"], 8)
        self.assertIsNone(metrics["limit_frame"])
        metrics = trajectory_metrics([0, 1, 2], 10, 2, 7, [-.5, .5])
        self.assertEqual(metrics["limit_frame"], 7.5)

    def test_constant_joint_has_zero_motion_and_invalid_values_fail(self):
        metrics = trajectory_metrics([.1], 30, 4, 1)
        self.assertEqual(metrics["max_speed_deg_s"], 0)
        for values in ([], [float("nan")]):
            with self.assertRaises(ValueError):
                trajectory_metrics(values, 30, 4, 1)


if __name__ == "__main__":
    unittest.main()
