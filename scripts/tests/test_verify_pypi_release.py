from __future__ import annotations

import unittest

from scripts.verify_pypi_release import validate_release


def release_payload(version: str = "0.0.26") -> dict[str, object]:
    urls = []
    for python_tag in ("cp310", "cp311", "cp312", "cp313", "cp314"):
        urls.extend(
            [
                {
                    "filename": (
                        f"pyuipc-{version}-{python_tag}-{python_tag}-"
                        "manylinux_2_34_x86_64.whl"
                    )
                },
                {
                    "filename": (
                        f"pyuipc-{version}-{python_tag}-{python_tag}-win_amd64.whl"
                    )
                },
            ]
        )
    return {"info": {"version": version}, "urls": urls}


class VerifyPyPIReleaseTests(unittest.TestCase):
    def test_accepts_complete_release_matrix(self) -> None:
        wheels = validate_release(release_payload(), "pyuipc", "0.0.26")
        self.assertEqual(len(wheels), 10)

    def test_rejects_missing_platform_variant(self) -> None:
        payload = release_payload()
        payload["urls"] = payload["urls"][:-1]  # type: ignore[index]
        with self.assertRaisesRegex(ValueError, "cp314/win_amd64"):
            validate_release(payload, "pyuipc", "0.0.26")

    def test_rejects_unexpected_release_version(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected '0.0.27'"):
            validate_release(release_payload(), "pyuipc", "0.0.27")


if __name__ == "__main__":
    unittest.main()
