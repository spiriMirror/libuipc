# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Catch extension manifest limits before Blender's install-time validation."""

from pathlib import Path
import unittest
try:
    import tomllib
except ModuleNotFoundError:
    tomllib = None


@unittest.skipIf(tomllib is None, "Manifest packaging uses Python 3.11+ stdlib tomllib")
class ManifestTests(unittest.TestCase):
    def test_permission_descriptions_fit_blender_limit(self):
        source = Path(__file__).resolve().parents[1] / "libuipc_blender" / "blender_manifest.toml"
        manifest = tomllib.loads(source.read_text(encoding="utf-8"))
        for name, description in manifest.get("permissions", {}).items():
            with self.subTest(permission=name):
                self.assertIsInstance(description, str)
                self.assertTrue(1 <= len(description) <= 64)


if __name__ == "__main__":
    unittest.main()
