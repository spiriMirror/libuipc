"""CMake dependency checks must not bootstrap pip or install packages."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import venv


ROOT = Path(__file__).resolve().parents[2]
CMAKE = shutil.which("cmake")


@unittest.skipUnless(CMAKE, "CMake is required")
class CMakePythonRequirementTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="uipc-python-check-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.prefix = self.root / "python environment"
        venv.EnvBuilder(with_pip=False).create(self.prefix)
        self.python = self.prefix / (
            "Scripts/python.exe" if os.name == "nt" else "bin/python"
        )
        self.env = os.environ.copy()
        self.env.pop("PYTHONPATH", None)
        self.env.pop("PYTHONHOME", None)
        self.env["PYTHONNOUSERSITE"] = "1"
        self.env["PYTHONDONTWRITEBYTECODE"] = "1"
        self.env["PIP_NO_INDEX"] = "1"
        self.assert_pip_absent()

    def assert_pip_absent(self):
        result = subprocess.run(
            [str(self.python), "-c",
             "import importlib.util; assert importlib.util.find_spec('pip') is None"],
            env=self.env, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def check_module(self, module):
        script = self.root / "check.cmake"
        script.write_text(
            "cmake_minimum_required(VERSION 3.26)\n"
            f"include([[{(ROOT / 'cmake/uipc_utils.cmake').as_posix()}]])\n"
            f"uipc_require_python_module([[{self.python.as_posix()}]] [[{module}]])\n",
            encoding="utf-8",
        )
        return subprocess.run(
            [CMAKE, "-P", str(script)], env=self.env,
            capture_output=True, text=True, timeout=60,
        )

    def test_existing_module_does_not_require_or_bootstrap_pip(self):
        result = self.check_module("json")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assert_pip_absent()

    def test_missing_module_fails_without_bootstrapping_pip(self):
        result = self.check_module("uipc_missing_build_dependency")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("uipc_missing_build_dependency", result.stderr)
        self.assertIn(
            "CMake does not install Python packages", " ".join(result.stderr.split())
        )
        self.assert_pip_absent()

    def test_cmake_editable_metadata_uses_development_version(self):
        import sys

        sys.path.insert(0, str(ROOT / "packaging"))
        import uipc_build

        class FakeCMakeBackend:
            @staticmethod
            def prepare_metadata_for_build_editable(directory, config_settings):
                return os.environ["SETUPTOOLS_SCM_PRETEND_VERSION"]

        original_backend = uipc_build._cmake_backend
        previous = os.environ.get("SETUPTOOLS_SCM_PRETEND_VERSION")
        uipc_build._cmake_backend = lambda: FakeCMakeBackend
        os.environ["SETUPTOOLS_SCM_PRETEND_VERSION"] = "8.8.8"
        try:
            with tempfile.TemporaryDirectory(prefix="uipc-metadata-") as directory:
                name = uipc_build.prepare_metadata_for_build_editable(
                    directory, {"builder": "cmake"}
                )
            self.assertEqual(name, "0.9.0")
        finally:
            uipc_build._cmake_backend = original_backend
            if previous is None:
                os.environ.pop("SETUPTOOLS_SCM_PRETEND_VERSION", None)
            else:
                os.environ["SETUPTOOLS_SCM_PRETEND_VERSION"] = previous


if __name__ == "__main__":
    unittest.main()
