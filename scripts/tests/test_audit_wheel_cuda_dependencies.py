from __future__ import annotations

import unittest

from scripts.audit_wheel_cuda_dependencies import (
    forbidden_dependencies,
    parse_dependency_names,
)


class WheelCudaDependencyAuditTests(unittest.TestCase):
    def test_parses_dumpbin_and_rejects_cuda_toolkit_dlls(self) -> None:
        output = """
          Image has the following dependencies:
            cublas64_12.dll
            nvcuda.dll
            KERNEL32.dll
        """
        names = parse_dependency_names(output)
        self.assertEqual(names, ["cublas64_12.dll", "nvcuda.dll", "KERNEL32.dll"])
        self.assertEqual(forbidden_dependencies(names), ["cublas64_12.dll"])

    def test_parses_readelf_and_allows_driver_library(self) -> None:
        output = """
         0x0000000000000001 (NEEDED) Shared library: [libcuda.so.1]
         0x0000000000000001 (NEEDED) Shared library: [libc.so.6]
        """
        names = parse_dependency_names(output)
        self.assertEqual(names, ["libcuda.so.1", "libc.so.6"])
        self.assertEqual(forbidden_dependencies(names), [])

    def test_rejects_all_cuda_toolkit_runtime_families(self) -> None:
        names = [
            "libcudart.so.12",
            "libcusparse.so.12",
            "cusolver64_12.dll",
            "nvJitLink_120_0.dll",
        ]
        self.assertEqual(forbidden_dependencies(names), sorted(names))

    def test_parses_objdump_needed_entries(self) -> None:
        output = "  NEEDED               libcudart.so.12\n"
        names = parse_dependency_names(output)
        self.assertEqual(names, ["libcudart.so.12"])
        self.assertEqual(forbidden_dependencies(names), names)


if __name__ == "__main__":
    unittest.main()
