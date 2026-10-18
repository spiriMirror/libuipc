from __future__ import annotations

import sys
import json
import hashlib
import io
import tempfile
from types import SimpleNamespace
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.run_benchmark import (
    DEFAULT_MANIFEST,
    build_command,
    build_environment,
    load_manifest,
    input_fingerprints,
    missing_required_paths,
    parse_gpu_memory_mib,
    parse_reported_benchmark,
    parse_reported_summary,
    parse_overrides,
    percentile,
    resolve_python,
    runtime_fingerprint,
    run_benchmark,
    use_samples_directory,
)
from scripts.benchmark_runtime import collect_runtime


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

    def test_isolated_samples_rebase_does_not_change_registry(self) -> None:
        entry = load_manifest(DEFAULT_MANIFEST)["mas-bunny"]
        original = json.loads(json.dumps(entry))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            with patch("scripts.run_benchmark.REPO_ROOT", root):
                rebased = use_samples_directory(entry, root / "output" / "samples")
                self.assertEqual(entry, original)
                self.assertTrue(rebased["entrypoint"].startswith("output/samples/"))
                self.assertTrue(all(p.startswith("output/samples/")
                                    for p in rebased["requiredPaths"]))
                self.assertEqual(rebased["metadataOutput"], entry["metadataOutput"])
                with self.assertRaises(ValueError):
                    use_samples_directory(entry, root.parent / "outside-samples")

    def test_input_fingerprints_detect_uncommitted_parameter_changes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            source = root / "main.py"
            source.write_text("E = 1e4\n", encoding="utf-8")
            entry = {"requiredPaths": ["main.py"]}
            with patch("scripts.run_benchmark.REPO_ROOT", root):
                before = input_fingerprints(entry)
                self.assertEqual(before, input_fingerprints(entry))
                source.write_text("E = 1e7\n", encoding="utf-8")
                self.assertNotEqual(before, input_fingerprints(entry))

    def test_runtime_identity_hashes_native_binaries_and_python_sources(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "uipc"
            native = package / "_native"
            native.mkdir(parents=True)
            (package / "__init__.py").write_text("# test package\n", encoding="utf-8")
            for name in ("cuda.dll", "pyuipc.pyd", "libcore.so", "libcore.so.1", "core.dylib"):
                (native / name).write_bytes(b"baseline")
            (native / "ignore.lib").write_bytes(b"not a runtime library")
            module = SimpleNamespace(__file__=str(package / "__init__.py"),
                                     build_info=lambda: {"cuda_backend": True})
            before = collect_runtime(module)
            self.assertEqual(len(before["nativeFiles"]), 5)
            self.assertTrue(all(f["sha256"] == hashlib.sha256(b"baseline").hexdigest()
                                for f in before["nativeFiles"]))
            self.assertEqual(before["pythonSources"][0]["path"], "__init__.py")
            (native / "cuda.dll").write_bytes(b"candidate")
            self.assertNotEqual(before, collect_runtime(module))

    def test_failed_runtime_probe_is_not_a_valid_fingerprint(self) -> None:
        for output in ("unavailable", "{}", "[]"):
            with patch("scripts.run_benchmark.command_output", return_value=output):
                self.assertFalse(runtime_fingerprint(sys.executable, {}, Path.cwd())["available"])

    def test_run_records_before_after_identity_and_selected_worktree(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            samples = root / "output" / "samples"
            samples.mkdir(parents=True)
            (samples / "main.py").write_text("# fake sample\n", encoding="utf-8")
            entry = {"name": "fake", "description": "CPU-only harness test",
                     "entrypoint": "libuipc-samples/main.py",
                     "workingDirectory": "libuipc-samples/",
                     "requiredPaths": ["libuipc-samples/main.py"], "arguments": [],
                     "defaultFrames": 1, "quickFrames": 1,
                     "metadataOutput": "output/results/fake.json"}
            args = SimpleNamespace(name="fake", samples_directory=samples, quick=True,
                                   frames=None, python=sys.executable, env=[], dry_run=False)
            child = SimpleNamespace(stdout=io.StringIO('BENCHMARK_RESULT '
                                    '{"frame_ms":[1.0],"frame_stats":[{"frame":1}]}\n'),
                                    wait=lambda: 0)
            fingerprints = [{"available": True, "nativeFiles": ["before"]},
                            {"available": True, "nativeFiles": ["after"]}]
            with patch("scripts.run_benchmark.REPO_ROOT", root), \
                 patch("scripts.run_benchmark.git_state", return_value={"commit": "test"}) as state, \
                 patch("scripts.run_benchmark.runtime_facts", return_value={}), \
                 patch("scripts.run_benchmark.runtime_fingerprint", side_effect=fingerprints), \
                 patch("scripts.run_benchmark.GpuMemoryMonitor") as monitor, \
                 patch("scripts.run_benchmark.subprocess.Popen", return_value=child), \
                 patch("sys.stdout", new_callable=io.StringIO):
                monitor.return_value.stop.return_value = None
                self.assertEqual(run_benchmark(args, {"fake": entry}), 0)
                state.assert_any_call(samples)
            metadata = json.loads((root / "output/results/fake.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["samplesDirectory"], str(samples))
            self.assertFalse(metadata["provenance"]["runtimeUnchanged"])
            self.assertTrue(metadata["provenance"]["inputsUnchanged"])
            self.assertEqual(metadata["reportedFrameTiming"]["meanFrameMs"], 1.0)


if __name__ == "__main__":
    unittest.main()
