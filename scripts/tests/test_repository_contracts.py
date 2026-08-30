from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.check_constitution_api import ROOT, check_constitution_api
from scripts.check_workflow_pins import check_workflow_pins
from scripts.detect_0kb_files import detect_0kb_files


class RepositoryContractTests(unittest.TestCase):
    def test_current_repository_contracts(self) -> None:
        self.assertEqual(detect_0kb_files(ROOT), [])
        self.assertEqual(check_constitution_api(ROOT), [])
        self.assertEqual(check_workflow_pins(ROOT), [])

    def test_zero_byte_detector_is_scoped_and_deterministic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "include").mkdir()
            (root / "src").mkdir()
            (root / "outside").mkdir()
            (root / "include" / "b.h").touch()
            (root / "src" / "a.cpp").touch()
            (root / "src" / "nonempty.cpp").write_text(
                "// source\n", encoding="utf-8"
            )
            (root / "outside" / "ignored.cpp").touch()

            self.assertEqual(
                [path.relative_to(root).as_posix() for path in detect_0kb_files(root)],
                ["include/b.h", "src/a.cpp"],
            )

    def test_constitution_checker_reports_missing_binding(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            headers = root / "include/uipc/constitution"
            bindings = root / "src/pybind/pyuipc/constitution"
            headers.mkdir(parents=True)
            bindings.mkdir(parents=True)
            (headers / "material.h").write_text(
                "class UIPC_CONSTITUTION_API Material {};\n", encoding="utf-8"
            )

            errors = check_constitution_api(root)
            self.assertIn(
                "public constitution class Material has no Python binding", errors
            )

    def test_constitution_checker_reports_unregistered_initializer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            headers = root / "include/uipc/constitution"
            bindings = root / "src/pybind/pyuipc/constitution"
            headers.mkdir(parents=True)
            bindings.mkdir(parents=True)
            (headers / "material.h").write_text(
                "class UIPC_CONSTITUTION_API Material {};\n", encoding="utf-8"
            )
            (bindings / "material.cpp").write_text(
                """
                PyMaterial::PyMaterial(py::module& m)
                {
                    py::class_<Material>(m, "Material");
                }
                """,
                encoding="utf-8",
            )

            errors = check_constitution_api(root)
            self.assertIn(
                "binding initializer PyMaterial is never registered in a pybind module",
                errors,
            )

    def test_workflow_checker_rejects_mutable_refs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            workflows = root / ".github/workflows"
            workflows.mkdir(parents=True)
            (workflows / "ci.yml").write_text(
                """
                steps:
                  - uses: actions/checkout@v7
                  - uses: owner/action@0123456789abcdef0123456789abcdef01234567
                    with:
                      revision: master
                """,
                encoding="utf-8",
            )

            errors = check_workflow_pins(root)
            self.assertTrue(any("mutable ref" in error for error in errors))
            self.assertTrue(any("reviewed tag comment" in error for error in errors))
            self.assertTrue(any("revision must be" in error for error in errors))

    def test_cuda_xmake_objects_are_position_independent_on_linux(self) -> None:
        components = (
            ROOT / "src/backends/cuda/components.lua"
        ).read_text(encoding="utf-8")

        self.assertIn('target:is_plat("linux")', components)
        self.assertIn('target:add("cxflags", "-fPIC")', components)
        self.assertIn('target:add("cuflags", "-Xcompiler=-fPIC")', components)


if __name__ == "__main__":
    unittest.main()
