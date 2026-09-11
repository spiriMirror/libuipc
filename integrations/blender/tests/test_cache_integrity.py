# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 spiriMirror
"""Provenance and streaming checksum checks without Blender/CUDA."""

import copy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libuipc_blender"))
from protocol import MDDWriter, file_sha256, validate_result


class CacheIntegrityTests(unittest.TestCase):
    def test_result_validates_provenance_and_complete_object_list(self):
        request = {"schema_version": 4, "fingerprint": "expected",
                   "settings": {"frame_start": 1, "frame_end": 2},
                   "objects": [{"material": {"role": "CLOTH"}}, {"material": {"role": "STATIC"}}]}
        result = {"schema_version": 4, "fingerprint": "expected", "frames": 2,
                  "cache_integrity": 1, "objects": [{"index": 0, "vertices": 3, "sha256": "a" * 64}]}
        self.assertEqual(validate_result(request, result, [3, 4]), 2)
        for key, value in (("fingerprint", "wrong"), ("schema_version", 3), ("frames", 3),
                           ("objects", []), ("objects", result["objects"] * 2)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_result(request, {**result, key: value}, [3, 4])
        for key, value in (("index", False), ("vertices", 4), ("sha256", "")):
            invalid = copy.deepcopy(result)
            invalid["objects"][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_result(request, invalid, [3, 4])

    def test_writer_hash_covers_header_times_and_every_vertex(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cache.mdd"
            writer = MDDWriter(path, 2, 3, 24)
            writer.append(np.eye(3))
            writer.append(np.eye(3) + 1)
            writer.close(commit=True)
            self.assertEqual(writer.digest.hexdigest(), file_sha256(path))
            with path.open("r+b") as stream:
                stream.seek(-1, 2)
                byte = stream.read(1)
                stream.seek(-1, 2)
                stream.write(bytes([byte[0] ^ 1]))
            self.assertNotEqual(writer.digest.hexdigest(), file_sha256(path))


if __name__ == "__main__":
    unittest.main()
