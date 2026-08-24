from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path
from types import ModuleType


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


if __name__ == "__main__":
    unittest.main()
