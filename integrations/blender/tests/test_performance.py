# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from performance import Timings, clear, frontend_phase, frontend_report


class PerformanceTests(unittest.TestCase):
    def test_accumulation_including_failure_and_report_snapshot(self):
        with patch("performance.time.perf_counter", side_effect=[1, 2, 5, 7, 9, 10]):
            timings = Timings()
            with timings.measure("phase"):
                pass
            with self.assertRaises(ValueError), timings.measure("phase"):
                raise ValueError("test")
            report = timings.report()
        self.assertEqual(report["phases"]["phase"], {"seconds": 5, "calls": 2})
        self.assertEqual(report["total_seconds"], 9)
        timings.add("phase", 1)
        self.assertEqual(report["phases"]["phase"]["calls"], 2)

    def test_frontend_does_not_mutate_scene_or_mask_failure(self):
        class Scene:
            __slots__ = ()
            def as_pointer(self):
                return 1
        @frontend_phase("read")
        def read(scene, fail=False):
            if fail:
                raise ValueError("test")
            return 42
        clear()
        scene = Scene()
        self.assertEqual(read(scene), 42)
        with self.assertRaises(ValueError):
            read(scene, True)
        self.assertEqual(frontend_report(scene)["read"]["calls"], 2)
        self.assertFalse(frontend_report(scene)["read"]["succeeded"])
        clear()
        self.assertEqual(frontend_report(scene), {})
