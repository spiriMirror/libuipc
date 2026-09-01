from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.run_benchmark import (
    DEFAULT_MANIFEST,
    build_command,
    build_environment,
    load_manifest,
    missing_required_paths,
    parse_gpu_memory_mib,
    parse_reported_benchmark,
    parse_reported_summary,
    parse_overrides,
    percentile,
    resolve_python,
)


class BenchmarkManifestTests(unittest.TestCase):
    def test_project_manifest_declares_canonical_benchmark_suite(self) -> None:
        registry = load_manifest(DEFAULT_MANIFEST)
        self.assertEqual(
            set(registry),
            {
                "rigid-wrecking-balls",
                "stiff-gipc-case2",
                "mas-bunny",
                "cube-wall-cloth",
            },
        )
        entry = registry["stiff-gipc-case2"]
        self.assertIn(entry["entrypoint"], entry["requiredPaths"])
        self.assertTrue(
            all(
                value.startswith("libuipc-samples/")
                for value in entry["requiredPaths"]
            )
        )

        command, working_directory = build_command(
            entry, resolve_python(sys.executable), entry["quickFrames"]
        )
        self.assertEqual(command[-2:], ["--headless", "3"])
        self.assertEqual(working_directory.name, "88_stiff_gipc_benchmark")

        for benchmark in registry.values():
            self.assertIn(
                "libuipc-samples/examples/benchmark_utils.py",
                benchmark["requiredPaths"],
            )

    def test_runtime_asset_check_reports_partial_checkout(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "present.txt").touch()
            entry = {"requiredPaths": ["present.txt", "missing.txt"]}
            with patch("scripts.run_benchmark.REPO_ROOT", root):
                self.assertEqual(
                    missing_required_paths(entry), [root / "missing.txt"]
                )

    def test_canonical_environment_removes_noncanonical_variants(self) -> None:
        entry = {
            "environment": {"WB_LOG": "Warn"},
            "unsetEnvironment": ["NO_MAS", "NO_GRAPH"],
        }
        environment = build_environment(
            entry,
            {"NO_MAS": "1"},
            {"NO_MAS": "old", "NO_GRAPH": "1", "KEEP": "yes"},
        )
        self.assertEqual(environment["WB_LOG"], "Warn")
        self.assertEqual(environment["NO_MAS"], "1")
        self.assertNotIn("NO_GRAPH", environment)
        self.assertEqual(environment["KEEP"], "yes")

    def test_environment_override_requires_assignment(self) -> None:
        self.assertEqual(parse_overrides(["A=1", "B="]), {"A": "1", "B": ""})
        with self.assertRaises(ValueError):
            parse_overrides(["BROKEN"])

    def test_reported_frame_summary_is_machine_readable(self) -> None:
        self.assertEqual(
            parse_reported_summary("TOTAL frames=3 mean=12.5ms median=11.0ms"),
            {"frames": 3, "meanFrameMs": 12.5, "medianFrameMs": 11.0},
        )
        self.assertIsNone(parse_reported_summary("no summary"))

    def test_structured_benchmark_result_is_validated(self) -> None:
        payload = {
            "frame_ms": [12.0, 10.0],
            "frame_stats": [{"frame": 1}, {"frame": 2}],
            "observables": {"height": 0.5},
        }
        output = "BENCHMARK_RESULT " + __import__("json").dumps(payload)
        self.assertEqual(parse_reported_benchmark(output), payload)
        with self.assertRaises(ValueError):
            parse_reported_benchmark(
                'BENCHMARK_RESULT {"frame_ms":[1],"frame_stats":[]}'
            )

    def test_gpu_memory_csv_parser_handles_multiple_devices(self) -> None:
        self.assertEqual(parse_gpu_memory_mib("1024\n2048 MiB"), [1024, 2048])
        self.assertIsNone(parse_gpu_memory_mib("N/A"))

    def test_percentile_interpolates_short_samples(self) -> None:
        self.assertEqual(percentile([0.0, 10.0, 20.0], 0.5), 10.0)
        self.assertEqual(percentile([0.0, 10.0], 0.95), 9.5)


if __name__ == "__main__":
    unittest.main()
