from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scripts.check_release_policy import ROOT, check_policy


class ReleasePolicyTests(unittest.TestCase):
    def test_repository_mirrors_are_synchronized(self) -> None:
        self.assertEqual(check_policy(), [])

    def test_detects_python_matrix_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for relative_path in (
                "pyproject.toml",
                "python/pyproject.toml",
                "python/src/uipc/compatibility.json",
                ".github/workflows/python-wheels.yml",
            ):
                source = ROOT / relative_path
                destination = root / relative_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(source.read_bytes())

            policy_path = root / "python/src/uipc/compatibility.json"
            policy = json.loads(policy_path.read_text(encoding="utf-8"))
            policy["python"]["abi_tags"].append("cp315")
            policy_path.write_text(json.dumps(policy), encoding="utf-8")

            errors = check_policy(root)
            self.assertTrue(any("Python matrix" in error for error in errors))

    def test_detects_cuda_matrix_drift(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for relative_path in (
                "pyproject.toml",
                "python/pyproject.toml",
                "python/src/uipc/compatibility.json",
                ".github/workflows/python-wheels.yml",
            ):
                source = ROOT / relative_path
                destination = root / relative_path
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(source.read_bytes())

            workflow_path = root / ".github/workflows/python-wheels.yml"
            workflow = workflow_path.read_text(encoding="utf-8")
            workflow_path.write_text(
                workflow.replace("- 12.8.1", "- 13.0.1"), encoding="utf-8"
            )

            errors = check_policy(root)
            self.assertTrue(any("CUDA matrix" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
