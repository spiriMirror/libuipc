"""Shared stub-generation protocol, without importing a native extension."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]


class PyuipcStubgenTests(unittest.TestCase):
    def load_generator(self, backend):
        spec = importlib.util.spec_from_file_location(
            "uipc_stubgen_under_test", ROOT / "scripts/pyuipc_stubgen.py"
        )
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"pybind11_stubgen": backend}):
            spec.loader.exec_module(module)
        return module

    def test_full_package_output_and_scoped_stale_stub_cleanup(self):
        seen = []
        backend = types.SimpleNamespace(main=lambda args: seen.append((args, sys.path[0])))
        generator = self.load_generator(backend)
        with tempfile.TemporaryDirectory(prefix="uipc-stubs-") as directory:
            root = Path(directory)
            source = root / "source with spaces"
            output = root / "staged output"
            nested = output / "uipc" / "_native"
            nested.mkdir(parents=True)
            stale = nested / "old.pyi"
            stale.write_text("old stub", encoding="utf-8")
            implementation = output / "uipc" / "__init__.py"
            implementation.write_text("# retain implementation", encoding="utf-8")
            unrelated = output / "another_package.pyi"
            unrelated.write_text("# retain unrelated stub", encoding="utf-8")
            original = source / "uipc" / "__init__.pyi"
            original.parent.mkdir(parents=True)
            original.write_text("# retain source stub", encoding="utf-8")

            with patch.object(sys, "path", sys.path.copy()):
                generator.generate_uipc_stubs(source, output)

            self.assertEqual(seen, [([
                "-o", str(output), "uipc", "--ignore-unresolved-names", "json"
            ], str(source))])
            self.assertFalse(stale.exists())
            for path in (implementation, unrelated, original):
                self.assertTrue(path.exists(), path)

    def test_cli_keeps_build_type_compatibility_and_propagates_failures(self):
        backend = types.SimpleNamespace(main=Mock(side_effect=RuntimeError("stub failure")))
        generator = self.load_generator(backend)
        with tempfile.TemporaryDirectory(prefix="uipc-stub-cli-") as directory:
            arguments = [
                "pyuipc_stubgen.py", "--source_dir", directory,
                "--output_dir", directory, "--build_type", "RelWithDebInfo",
            ]
            with patch.object(sys, "argv", arguments), patch.object(sys, "path", sys.path.copy()):
                with self.assertRaisesRegex(RuntimeError, "stub failure"):
                    generator.main()

    def test_cmake_and_xmake_use_shared_package_root_generator(self):
        post_build = (ROOT / "scripts/after_build_pyuipc.py").read_text(encoding="utf-8")
        xmake = (ROOT / "xmake/pack.lua").read_text(encoding="utf-8")
        self.assertIn("from pyuipc_stubgen import generate_uipc_stubs", post_build)
        self.assertIn("generate_uipc_stubs(typings_dir, typings_dir)", post_build)
        self.assertIn('scripts/pyuipc_stubgen.py', xmake)
        self.assertIn('"--output_dir=" .. path.join(build_dir, "src")', xmake)
        self.assertIn('"pip", "install", "--python", python', xmake)


if __name__ == "__main__":
    unittest.main()
