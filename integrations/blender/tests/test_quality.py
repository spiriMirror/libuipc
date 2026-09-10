# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Streaming diagnostic math and provenance without a native solver."""

import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from quality import QualityRecorder
from protocol import file_sha256


class QualityTests(unittest.TestCase):
    def request(self):
        return {"fingerprint": "input", "settings": {"fps": 10, "frame_start": 7, "frame_end": 9},
                "objects": [{"name": "body", "material": {"role": "RIGID"}}]}

    def test_world_space_differences_and_owned_history(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = QualityRecorder(directory, self.request())
            points = np.eye(3)
            recorder.record_object(0, 7, points)
            points += [0.1, 0, 0]  # Native views can be overwritten in place.
            recorder.record_object(0, 8, points)
            points += [0.2, 0, 0]
            recorder.record_object(0, 9, points)
            report = recorder.finish()
            obj = report["objects"][0]
            self.assertAlmostEqual(obj["max_speed"], 2)
            self.assertAlmostEqual(obj["max_acceleration"], 10)
            self.assertAlmostEqual(obj["final_rms_speed"], 2)
            self.assertEqual(obj["speed_frame"], 9)
            self.assertEqual(report["series_sha256"], file_sha256(Path(directory) / "quality_frames.jsonl"))

    def test_solver_uses_output_frame_not_native_substep_number(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = QualityRecorder(directory, self.request())
            recorder.record_solver(8, 2, {"frame": 42, "newton_iterations": 5,
                                         "converged": False, "hit_line_search_limit": True})
            report = recorder.finish()
            self.assertEqual(report["solver"]["limit_frames"], [8])
            self.assertEqual(report["solver"]["nonconverged_steps"], 1)
            row = json.loads((Path(directory) / "quality_frames.jsonl").read_text())
            self.assertEqual(row["frame"], 8)
            self.assertEqual(row["native_frame"], 42)

    def test_cancelled_or_invalid_sampling_does_not_publish_report(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = QualityRecorder(directory, self.request())
            with self.assertRaises(ValueError):
                recorder.record_object(0, 7, np.full((3, 3), np.nan))
            recorder.close()
            self.assertFalse((Path(directory) / "quality_report.json").exists())


if __name__ == "__main__":
    unittest.main()
