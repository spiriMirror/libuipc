from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import ModuleType
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
DOCTOR_PATH = ROOT / "python" / "src" / "uipc" / "cli" / "doctor.py"


def load_doctor() -> ModuleType:
    spec = importlib.util.spec_from_file_location("pyuipc_doctor_under_test", DOCTOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {DOCTOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


doctor = load_doctor()


class PyUIPCDoctorTests(unittest.TestCase):
    def test_release_architectures_cover_turing_ampere_ada_and_future(self) -> None:
        architectures = doctor.load_policy()["wheel"]["cuda_architectures"]
        for capability in ("7.5", "8.0", "8.6", "8.9", "9.0", "12.0"):
            with self.subTest(capability=capability):
                self.assertTrue(
                    doctor.architecture_supports(architectures, capability)
                )

    def test_real_code_does_not_claim_forward_compatibility(self) -> None:
        self.assertFalse(doctor.architecture_supports(["89-real"], "9.0"))
        self.assertTrue(doctor.architecture_supports(["89-virtual"], "9.0"))

    def test_architecture_code_path_prefers_compatible_sass(self) -> None:
        architectures = ["80-real", "89-virtual"]
        self.assertEqual(
            doctor.architecture_code_path(architectures, "8.6"), "sass"
        )
        self.assertEqual(
            doctor.architecture_code_path(architectures, "9.0"), "ptx"
        )
        self.assertEqual(
            doctor.architecture_code_path(["89-real"], "9.0"), "unsupported"
        )
        self.assertIsNone(doctor.architecture_code_path("native", "8.9"))

    def test_unknown_architecture_is_reported_as_indeterminate(self) -> None:
        self.assertIsNone(doctor.architecture_supports("native", "8.9"))
        self.assertIsNone(doctor.architecture_supports("unknown", "8.9"))

    def test_parses_nvidia_smi_rows(self) -> None:
        rows = doctor.parse_nvidia_smi_rows(
            "NVIDIA GeForce RTX 3060, 8.6, 591.74\n"
            "NVIDIA GeForce RTX 5090, 12.0, 591.74\n"
        )
        self.assertEqual(rows[0]["compute_capability"], "8.6")
        self.assertEqual(rows[1]["name"], "NVIDIA GeForce RTX 5090")

    def test_driver_version_comparison_handles_different_widths(self) -> None:
        self.assertTrue(doctor.version_at_least("528.33", "528.33"))
        self.assertTrue(doctor.version_at_least("595.79", "528.33"))
        self.assertTrue(doctor.version_at_least("525.60.13", "525.60.13"))
        self.assertFalse(doctor.version_at_least("525.59", "525.60.13"))

    def test_release_wheel_does_not_require_a_system_cuda_toolkit(self) -> None:
        wheel = doctor.load_policy()["wheel"]
        self.assertFalse(wheel["requires_system_cuda_toolkit"])
        self.assertEqual(wheel["minimum_driver"]["windows"], "528.33")
        self.assertEqual(wheel["minimum_driver"]["linux"], "525.60.13")
        self.assertEqual(wheel["minimum_ptx_jit_driver"]["windows"], "572.61")
        self.assertEqual(wheel["minimum_ptx_jit_driver"]["linux"], "570.124.06")

    def test_ptx_fallback_requires_the_toolkit_generation_driver(self) -> None:
        wheel = doctor.load_policy()["wheel"]
        architectures = wheel["cuda_architectures"]

        sass_driver, sass_path = doctor.required_driver_for_architecture(
            wheel, "windows", architectures, "12.0"
        )
        self.assertEqual((sass_driver, sass_path), ("528.33", "sass"))

        ptx_driver, ptx_path = doctor.required_driver_for_architecture(
            wheel, "windows", architectures, "9.0"
        )
        self.assertEqual((ptx_driver, ptx_path), ("572.61", "ptx"))

    def test_diagnostics_reject_old_driver_for_ptx_only_gpu(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            package_dir = Path(directory) / "uipc"
            (package_dir / "_native").mkdir(parents=True)
            fake_uipc = ModuleType("uipc")
            fake_uipc.__file__ = str(package_dir / "__init__.py")
            fake_uipc.__version__ = "test"
            fake_uipc.build_info = lambda: {
                "python_abi": f"cp{doctor.sys.version_info.major}{doctor.sys.version_info.minor}",
                "cuda_backend": True,
                "cuda_toolkit_version": "12.8.1",
                "cuda_architectures": "75-real,80-real,86-real,89-real,120-real,89-virtual",
            }
            gpu = {
                "name": "NVIDIA H100",
                "compute_capability": "9.0",
                "driver_version": "528.33",
            }

            with mock.patch.object(
                doctor.importlib, "import_module", return_value=fake_uipc
            ), mock.patch.object(doctor, "query_gpus", return_value=([gpu], None)):
                report = doctor.collect_diagnostics()

        driver = next(
            check for check in report["checks"] if check["name"] == "NVIDIA driver"
        )
        architecture = next(
            check
            for check in report["checks"]
            if check["name"] == "GPU architecture #0"
        )
        self.assertEqual(driver["status"], "fail")
        driver_platform = "windows" if doctor.os.name == "nt" else "linux"
        expected_driver = doctor.load_policy()["wheel"]["minimum_ptx_jit_driver"][
            driver_platform
        ]
        self.assertIn(expected_driver, driver["hint"])
        self.assertEqual(driver["data"][0]["code_path"], "ptx")
        self.assertEqual(driver["data"][0]["minimum_driver"], expected_driver)
        self.assertEqual(architecture["status"], "ok")
        self.assertIn("PTX", architecture["detail"])
        self.assertEqual(architecture["data"]["code_path"], "ptx")


if __name__ == "__main__":
    unittest.main()
