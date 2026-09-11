# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 spiriMirror
"""PNG receipts and immutable snapshot/dependency checks."""

from pathlib import Path
import struct
import os
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from protocol import atomic_json, file_sha256
from render_protocol import png_info, dependency_record, validate_job, completed_frame, frame_paths, dependency_stamps


def tiny_png():
    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(payload, zlib.crc32(kind)))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(b"\0\xff\0\0")) + chunk(b"IEND", b""))


class RenderProtocolTests(unittest.TestCase):
    def test_receipt_requires_same_job_frame_hash_and_complete_png(self):
        with tempfile.TemporaryDirectory() as directory:
            image, receipt = frame_paths(directory, 0, 3)
            image.parent.mkdir()
            image.write_bytes(tiny_png())
            self.assertEqual(png_info(image), (1, 1))
            record = {"job_signature": "job", "camera_index": 0, "frame": 3, "sha256": file_sha256(image), "input_guard": True}
            atomic_json(receipt, record)
            self.assertTrue(completed_frame(directory, 0, 3, "job", [1, 1]))
            self.assertFalse(completed_frame(directory, 0, 3, "other job", [1, 1]))
            image.write_bytes(tiny_png()[:-4])
            self.assertFalse(completed_frame(directory, 0, 3, "job", [1, 1]))
            with self.assertRaises(ValueError):
                png_info(image)

    def test_snapshot_and_dependencies_cannot_change_on_resume(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve()
            (directory / "scene.blend").write_bytes(b"snapshot")
            asset = directory / "texture.dat"
            asset.write_bytes(b"original")
            manifest = {"schema_version": 1, "directory": str(directory),
                        "snapshot_sha256": file_sha256(directory / "scene.blend"),
                        "resolution": [1, 1], "shots": [{"camera": "Camera", "first": 2, "last": 3}],
                        "dependencies": [dependency_record(asset)]}
            atomic_json(directory / "render_manifest.json", manifest)
            validate_job(directory)
            original = dependency_stamps(directory, manifest)
            info = asset.stat()
            os.utime(asset, ns=(info.st_atime_ns, info.st_mtime_ns + 1000000))
            self.assertNotEqual(original, dependency_stamps(directory, manifest))
            validate_job(directory)  # Same bytes, but an in-flight run detects the changed stamp.
            asset.write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "dependency"):
                validate_job(directory)
            asset.write_bytes(b"original")
            (directory / "scene.blend").write_bytes(b"edited snapshot")
            with self.assertRaisesRegex(ValueError, "snapshot"):
                validate_job(directory)


if __name__ == "__main__":
    unittest.main()
