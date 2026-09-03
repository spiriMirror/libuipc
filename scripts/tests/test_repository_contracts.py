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

    def test_cuda_xmake_sources_stay_on_shared_target_for_device_link(self) -> None:
        components = (
            ROOT / "src/backends/cuda/components.lua"
        ).read_text(encoding="utf-8")
        backend = (ROOT / "src/backends/cuda/xmake.lua").read_text(
            encoding="utf-8"
        )

        self.assertIn("function uipc_add_cuda_component_sources()", components)
        self.assertIn("add_files(path.join(backend_dir, pattern))", components)
        self.assertNotIn('target:set("kind", "object")', components)
        self.assertIn("uipc_add_cuda_component_sources()", backend)
        self.assertIn('add_cuflags("-rdc=true")', backend)

    def test_cmake_final_cuda_target_has_multiconfig_device_link_anchor(self) -> None:
        backend = (ROOT / "src/backends/cuda/CMakeLists.txt").read_text(
            encoding="utf-8"
        )

        self.assertIn("UIPC_CUDA_DEVICE_LINK_ANCHOR", backend)
        self.assertIn("uipc_cuda_device_link_anchor.cu", backend)
        self.assertIn(
            'target_sources(cuda PRIVATE "${UIPC_CUDA_DEVICE_LINK_ANCHOR}")',
            backend,
        )
        self.assertIn("CUDA_RESOLVE_DEVICE_SYMBOLS ON", backend)

    def test_cmake_cuda_architecture_guard_covers_every_target_kind(self) -> None:
        utilities = (ROOT / "cmake/uipc_utils.cmake").read_text(encoding="utf-8")
        backend = (ROOT / "src/backends/cuda/CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        components = (ROOT / "src/backends/cuda/components.cmake").read_text(
            encoding="utf-8"
        )
        tests = (ROOT / "apps/tests/backends/cuda/CMakeLists.txt").read_text(
            encoding="utf-8"
        )

        self.assertIn("function(uipc_set_target_cuda_architectures", utilities)
        self.assertIn("get_target_property(", utilities)
        self.assertIn(
            'uipc_set_target_cuda_architectures(cuda "${UIPC_CUDA_ARCHITECTURES}")',
            backend,
        )
        self.assertIn(
            'uipc_set_target_cuda_architectures(${name} "${UIPC_CUDA_ARCHITECTURES}")',
            components,
        )
        self.assertIn(
            'uipc_set_target_cuda_architectures(backend_cuda "${CMAKE_CUDA_ARCHITECTURES}")',
            tests,
        )

    def test_cuda_backend_has_no_toolkit_library_link_or_transitive_cub_umbrella(
        self,
    ) -> None:
        cmake_backend = (ROOT / "src/backends/cuda/CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        cmake_components = (ROOT / "src/backends/cuda/components.cmake").read_text(
            encoding="utf-8"
        )
        xmake_backend = (ROOT / "src/backends/cuda/xmake.lua").read_text(
            encoding="utf-8"
        )
        linear_system = (
            ROOT / "src/backends/cuda/cuda_tool/linear_system.h"
        ).read_text(encoding="utf-8")
        reduction = (
            ROOT / "src/backends/cuda/cuda_tool/linear_reduction.h"
        ).read_text(encoding="utf-8")
        wheel_workflow = (
            ROOT / ".github/workflows/python-wheels.yml"
        ).read_text(encoding="utf-8")

        for text in (cmake_backend, cmake_components, xmake_backend):
            for library in ("cublas", "cusparse", "cusolver"):
                self.assertNotIn(library, text.lower())
        self.assertNotIn("cublas", linear_system.lower())
        self.assertNotIn("#include <cuda_tool/cub.h>", linear_system)
        self.assertIn("DeviceReduce(m_stream)", reduction)
        self.assertIn("audit_wheel_cuda_dependencies.py", wheel_workflow)

    def test_thin_shell_reference_weight_and_thickness_contract(self) -> None:
        cuda_constitutions = ROOT / "src/backends/cuda/finite_element/constitutions"
        reference = (cuda_constitutions / "discrete_shell_bending_reference.h").read_text(
            encoding="utf-8"
        )
        wrappers = [
            "discrete_shell_bending_function.h",
            "strain_plastic_discrete_shell_bending_function.h",
            "stress_plastic_discrete_shell_bending_function.h",
        ]

        self.assertIn("h_bar      = A / 3.0 / L0;", reference)
        self.assertIn("outer_weight = 1.0;", reference)
        for wrapper in wrappers:
            text = (cuda_constitutions / wrapper).read_text(encoding="utf-8")
            self.assertIn("compute_discrete_shell_bending_reference", text)

        stretch = (ROOT / "src/constitution/strain_limiting_baraff_witkin.cpp").read_text(
            encoding="utf-8"
        )
        bending = (ROOT / "src/constitution/discrete_shell_bending.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("* (2 * thickness)", stretch)
        self.assertIn("Float full_thickness = 2 * thickness;", bending)

    def test_metis_is_embedded_without_legacy_external_or_license_fragments(
        self,
    ) -> None:
        self.assertFalse((ROOT / "external").exists())

        root_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        root_xmake = (ROOT / "xmake.lua").read_text(encoding="utf-8")
        geometry_cmake = (ROOT / "src/geometry/CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        geometry_xmake = (ROOT / "src/geometry/xmake.lua").read_text(
            encoding="utf-8"
        )
        embedded = ROOT / "src/geometry/metis"

        self.assertNotIn("add_subdirectory(external)", root_cmake)
        self.assertNotIn("external/GKlib", root_xmake)
        self.assertNotIn("external/METIS", root_xmake)
        self.assertIn("add_subdirectory(metis)", geometry_cmake)
        self.assertIn("uipc_metis", geometry_cmake)
        self.assertIn('includes("metis")', geometry_xmake)
        self.assertIn('add_deps("uipc_core", "uipc_metis")', geometry_xmake)
        self.assertTrue((embedded / "LICENSE-METIS").is_file())
        self.assertTrue((embedded / "LICENSE-GKlib").is_file())

        source_files = [
            path
            for path in sorted(embedded.rglob("*"))
            if path.suffix in {".h", ".cpp"}
        ]
        for path in source_files:
            notice = path.read_text(encoding="utf-8")[:512].lower()
            self.assertIn("adapted", notice, path.relative_to(ROOT).as_posix())

        source = "\n".join(
            path.read_text(encoding="utf-8") for path in source_files
        )
        for forbidden in (
            "GK_MKQSORT",
            "USE_GKRAND",
            "gk_getopt",
            "GNU C Library",
            "LESSER GENERAL PUBLIC LICENSE",
            "Mersenne Twister",
            "std::qsort",
            "std::sort",
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn("deterministic_partition_sort", source)

    def test_qr_svd_uses_explicit_t_precision_sign(self) -> None:
        qr_svd = (
            ROOT / "src/backends/cuda/algorithm/qr_svd.hpp"
        ).read_text(encoding="utf-8")

        self.assertNotIn("copysign", qr_svd)
        self.assertIn("if(d < (T)0)", qr_svd)
        self.assertIn("shift = -shift;", qr_svd)

    def test_clang_format_uses_immutable_pull_request_shas(self) -> None:
        workflow = (
            ROOT / ".github/workflows/clang-format.yml"
        ).read_text(encoding="utf-8")

        self.assertIn("github.event.pull_request.base.sha", workflow)
        self.assertIn("github.event.pull_request.head.sha", workflow)
        self.assertNotIn('git fetch origin "${{ github.base_ref }}" --depth=1', workflow)


if __name__ == "__main__":
    unittest.main()
