"""Preserve direct-CMake auto-install without mutating frontend-managed builds."""

import ast
import json
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

    def check_module(self, module, *, managed=False):
        script = self.root / "check.cmake"
        managed_config = "set(SKBUILD 2)\n" if managed else ""
        script.write_text(
            "cmake_minimum_required(VERSION 3.26)\n"
            f"include([[{(ROOT / 'cmake/uipc_utils.cmake').as_posix()}]])\n"
            f"{managed_config}"
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

    def test_missing_managed_module_fails_without_bootstrapping_pip(self):
        result = self.check_module("uipc_missing_build_dependency", managed=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("uipc_missing_build_dependency", result.stderr)
        self.assertIn(
            "CMake does not install Python packages", " ".join(result.stderr.split())
        )
        self.assert_pip_absent()

    def fake_pip(self, *, install_module=True, exit_code=0):
        """Observe installer commands without using the network or real pip."""
        modules = self.root / "fake modules"
        modules.mkdir()
        self.env["PYTHONPATH"] = str(modules)
        (modules / "pip.py").write_text(
            "import json, pathlib, sys\n"
            "if __name__ == '__main__':\n"
            "    root = pathlib.Path(__file__).parent\n"
            "    (root / 'pip-call.json').write_text(json.dumps({"
            "'python': sys.executable, 'args': sys.argv[1:]}))\n"
            + (
                "    (root / 'uipc_missing_build_dependency.py').write_text('READY = True\\n')\n"
                if install_module else ""
            )
            + f"    raise SystemExit({exit_code})\n",
            encoding="utf-8",
        )
        return modules / "pip-call.json"

    def test_direct_build_installs_once_into_selected_interpreter(self):
        marker = self.fake_pip()
        result = self.check_module("uipc_missing_build_dependency")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call = json.loads(marker.read_text(encoding="utf-8"))
        self.assertEqual(Path(call["python"]), self.python)
        self.assertEqual(call["args"], ["install", "uipc_missing_build_dependency"])
        marker.unlink()
        result = self.check_module("uipc_missing_build_dependency")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(marker.exists(), "An available module must not be reinstalled")

    def test_managed_build_does_not_call_available_pip(self):
        marker = self.fake_pip()
        result = self.check_module("uipc_missing_build_dependency", managed=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(marker.exists())

    def test_direct_build_bootstraps_missing_pip_with_selected_interpreter(self):
        marker = self.fake_pip()
        modules = marker.parent
        (modules / "pip.py").rename(modules / "pip_template.py")
        (modules / "ensurepip.py").write_text(
            "import json, pathlib, shutil, sys\n"
            "if __name__ == '__main__':\n"
            "    root = pathlib.Path(__file__).parent\n"
            "    (root / 'bootstrap.json').write_text(json.dumps({"
            "'python': sys.executable, 'args': sys.argv[1:]}))\n"
            "    shutil.copyfile(root / 'pip_template.py', root / 'pip.py')\n",
            encoding="utf-8",
        )
        self.assert_pip_absent()
        result = self.check_module("uipc_missing_build_dependency")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        call = json.loads((modules / "bootstrap.json").read_text(encoding="utf-8"))
        self.assertEqual(Path(call["python"]), self.python)
        self.assertEqual(call["args"], ["--upgrade"])
        self.assertTrue(marker.exists())

    def test_direct_build_reports_installer_failure(self):
        marker = self.fake_pip(install_module=False, exit_code=1)
        result = self.check_module("uipc_missing_build_dependency")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("failed to install", " ".join(result.stderr.split()))
        self.assertTrue(marker.exists())

    def test_direct_build_verifies_import_after_installation(self):
        marker = self.fake_pip(install_module=False)
        result = self.check_module("uipc_missing_build_dependency")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("after pip installation", " ".join(result.stderr.split()))
        self.assertTrue(marker.exists())


class PythonBuildMetadataTests(unittest.TestCase):
    def test_existing_uninstall_first_policy_is_preserved(self):
        tree = ast.parse((ROOT / "scripts/after_build_pyuipc.py").read_text(encoding="utf-8"))
        entry = next(
            node for node in tree.body
            if isinstance(node, ast.If) and "__main__" in ast.unparse(node.test)
        )
        calls = sorted(
            (node.lineno, node.func.id)
            for node in ast.walk(entry)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        )
        order = {name: line for line, name in calls}
        self.assertLess(order["uninstall_package"], order["clear_binary_python_dir"])
        self.assertLess(order["clear_binary_python_dir"], order["generate_build_stubs"])
        self.assertLess(order["generate_build_stubs"], order["install_package"])

    def test_standard_backend_and_configure_time_requirements(self):
        try:
            import tomllib
        except ImportError:
            self.skipTest("TOML inspection requires Python 3.11+")
        metadata = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
        build = metadata["build-system"]
        self.assertEqual(build["build-backend"], "scikit_build_core.build")
        self.assertNotIn("backend-path", build)
        for package in ("pybind11", "pybind11-stubgen", "numpy", "typing_extensions"):
            self.assertTrue(
                any(item.startswith(package + ">=") for item in build["requires"])
            )
        self.assertEqual(
            metadata["tool"]["scikit-build"]["metadata"]["version"]["provider"],
            "scikit_build_core.metadata.setuptools_scm",
        )


if __name__ == "__main__":
    unittest.main()
